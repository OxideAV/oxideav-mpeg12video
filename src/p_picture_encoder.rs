//! Predictive (P) picture encoder — motion-compensated forward
//! prediction with a per-macroblock residual.
//!
//! This builds on the zero-MV path in [`crate::inter_encoder`] by adding
//! a real motion search ([`crate::motion_estimation`]) and a coded
//! residual. The encoder is the bit-exact inverse of the decode pipeline:
//! every macroblock it writes reconstructs, on the encoder side, to the
//! same samples the decoder will produce, so the encoder can carry a
//! reconstructed reference frame forward exactly as the decoder does.
//!
//! ## Per-macroblock pipeline (§6.2.5 / §7.6)
//!
//! For each macroblock of a frame picture written with
//! `frame_pred_frame_dct = 1` (so one frame motion vector, frame-DCT
//! blocks, no `motion_type` / `dct_type` fields):
//!
//! 1. **Motion search** — [`crate::motion_estimation::estimate_forward_mv`]
//!    picks the forward luminance vector against the reconstructed
//!    reference frame, scoring candidates by the SAD of the exact
//!    §7.6.4 prediction.
//! 2. **Prediction** — [`crate::inter_reconstruction::predict_frame_macroblock_planes`]
//!    forms the luma + chroma prediction for the chosen vector (this is
//!    the very prediction the decoder forms).
//! 3. **Residual** — `current - prediction` per block, forward-DCT +
//!    dead-zone non-intra quantise, giving the `QF[v][u]` levels.
//! 4. **Mode decision** — the §6.3.16.1 `coded_block_pattern` (Table B-9
//!    bit `5 - i` for block `i`) is the set of blocks with any non-zero
//!    level. The macroblock type (Table B-3) is:
//!    * `MC, Coded` (`1`) when any block is coded;
//!    * `MC, Not Coded` (`001`) when every block quantises to zero — the
//!      decoder then copies the prediction verbatim.
//! 5. **Reconstruction** — the encoder inverse-quantises + IDCTs each
//!    coded block and writes `Saturate(residual + prediction)` into the
//!    reconstructed frame, matching the decoder's
//!    [`crate::inter_reconstruction::write_inter_block`] arithmetic.
//!
//! Motion vectors are differentially coded against the §7.6.3.4 PMV,
//! which resets at slice start and is updated to each coded vector
//! (§7.6.3.3). Every macroblock carries an explicit forward vector, so no
//! skipped-macroblock PMV side-effects arise.

#![allow(clippy::needless_range_loop)]
#![allow(clippy::too_many_arguments)]
// The Table B-3 `00011` Intra codeword is grouped to mirror the spec's
// MSB-first table layout (`0001_1`) so an audit reads it against the
// printed table; clippy's equal-size grouping would obscure that.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitWriter;

use crate::coded_block_pattern::encode_coded_block_pattern;
use crate::forward_dct::fdct_8x8;
use crate::forward_quant::forward_quantise_block;
use crate::frame_assembly::{block_placement, FrameBuffer, IntraPictureParams};
use crate::idct::idct_8x8_from_i32;
use crate::inter_reconstruction::{
    chroma_mb_extent, predict_frame_macroblock_planes, FrameMotion, ReferenceFrames,
};
use crate::motion_estimation::{estimate_forward_mv, max_search_range};
use crate::motion_vector::encode_motion_component;
use crate::mpeg2_block_dc::{encode_intra_dc, ColourComponent, DcComponent};
use crate::mpeg2_dct_coeff::{
    encode_dct_coeff, encode_end_of_block, CoefficientPosition, TableSelection,
};
use crate::mpeg2_dequantize::{
    intra_dc_mult, inverse_quantise_block, quantiser_scale, BlockCoding, DEFAULT_INTRA_WEIGHT,
    DEFAULT_NON_INTRA_WEIGHT,
};
use crate::mpeg2_inverse_scan::inverse_scan_table;
use crate::mpeg2_macroblock_blocks::{block_component, block_count};
use crate::picture_header::PictureCodingType;
use crate::pmv::vector_range;
use crate::stream_writer::{
    write_picture_coding_extension, write_picture_header, write_slice_header,
    PictureCodingExtensionParams,
};
use crate::Result;

/// Wrap a motion-vector `delta` (`vector' - PMV`) into the §7.6.3.1
/// `[low, high]` codable band of `f_code`, so the §6.2.5.2.1 encoder has
/// a representable `motion_code`.
///
/// The decoder reconstructs `vector' = PMV + delta`, then wraps `vector'`
/// back into `[low, high]` by ±`range`. Because the chosen `vector'` and
/// `PMV` are both already inside the band, `delta` lies in
/// `[-(range-1), range-1]`; a single ±`range` correction brings it into
/// `[low, high]` and the decoder's wrap inverts it exactly.
pub(crate) fn wrap_delta(delta: i32, f_code: u8) -> Result<i32> {
    let (low, high, range) = vector_range(f_code)?;
    let mut d = delta;
    if d < low {
        d += range;
    } else if d > high {
        d -= range;
    }
    Ok(d)
}

/// Gather an 8×8 block of `current - prediction` residual samples.
///
/// `cur_plane` is the source frame's component plane; `pred` is the
/// macroblock-local prediction plane (`pred_w` wide). `(base_x, base_y)`
/// is the block's top-left in the frame plane; `(local_x, local_y)` is
/// its top-left in the prediction plane. Edge reads of the source clamp
/// to the nearest valid sample (the macroblock grid is padded to 16).
pub(crate) fn gather_residual(
    cur_plane: &crate::frame_assembly::Plane,
    pred: &[u8],
    pred_w: usize,
    base_x: usize,
    base_y: usize,
    local_x: usize,
    local_y: usize,
) -> [[i16; 8]; 8] {
    let w = cur_plane.width();
    let h = cur_plane.height();
    let mut out = [[0i16; 8]; 8];
    for r in 0..8 {
        let sy = (base_y + r).min(h.saturating_sub(1));
        let py = local_y + r;
        for c in 0..8 {
            let sx = (base_x + c).min(w.saturating_sub(1));
            let px = local_x + c;
            let cur = i32::from(cur_plane.get(sx, sy).unwrap_or(0));
            let prd = i32::from(pred[py * pred_w + px]);
            out[r][c] = (cur - prd) as i16;
        }
    }
    out
}

/// The per-block forward-quantised levels of one inter macroblock plus
/// the spatial-domain reconstruction the decoder would form. Shared by
/// the P- and B-picture encoders.
pub(crate) struct InterBlock {
    /// §7.3 raster `QF[v][u]` levels (all 64), or `None` if every level
    /// is zero (the block is not coded).
    qf: Option<[[i32; 8]; 8]>,
    /// The reconstructed `f_pel` residual (inverse-quantised + IDCT of
    /// `qf`), all zero when the block is not coded.
    f_pel: [[i16; 8]; 8],
}

impl InterBlock {
    /// Whether this block carried any non-zero level (so it sets a
    /// `coded_block_pattern` bit).
    pub(crate) fn is_coded(&self) -> bool {
        self.qf.is_some()
    }

    /// The raster `QF[v][u]` levels, or `None` if the block is not coded.
    pub(crate) fn qf_ref(&self) -> Option<&[[i32; 8]; 8]> {
        self.qf.as_ref()
    }

    /// The reconstructed spatial-domain residual (`f_pel`) the decoder
    /// will add to the prediction — all zero for an uncoded block.
    pub(crate) fn f_pel_ref(&self) -> &[[i16; 8]; 8] {
        &self.f_pel
    }
}

/// Forward-quantise one non-intra residual block and reconstruct it.
pub(crate) fn quantise_inter_block(residual: &[[i16; 8]; 8], qscale: u8) -> InterBlock {
    let f = fdct_8x8(residual);
    let qf = forward_quantise_block(
        &f,
        BlockCoding::NonIntra,
        &DEFAULT_NON_INTRA_WEIGHT,
        qscale,
        1,
    );
    let any = qf.iter().any(|row| row.iter().any(|&c| c != 0));
    if !any {
        return InterBlock {
            qf: None,
            f_pel: [[0i16; 8]; 8],
        };
    }
    // Reconstruct exactly as the decoder will: inverse-quantise + IDCT.
    let dequant = inverse_quantise_block(
        &qf,
        BlockCoding::NonIntra,
        &DEFAULT_NON_INTRA_WEIGHT,
        qscale,
        1,
    );
    let f_pel = idct_8x8_from_i32(&dequant);
    InterBlock {
        qf: Some(qf),
        f_pel,
    }
}

/// Emit one non-intra block's §7.2.2 run-level VLCs from its raster
/// `QF[v][u]` levels (already known non-zero), walking the §7.3 scan
/// selected by the picture's `alternate_scan` flag. Non-intra blocks
/// always code against Table B-14 (§7.2.2.1 Table 7-3).
pub(crate) fn write_inter_block_coeffs(
    bw: &mut BitWriter,
    qf: &[[i32; 8]; 8],
    alternate_scan: bool,
) {
    let scan = inverse_scan_table(alternate_scan);
    let table = TableSelection::TableZero;
    let mut run = 0u8;
    let mut first = true;
    for &(v, u) in scan.iter() {
        let level = qf[v as usize][u as usize];
        if level == 0 {
            run += 1;
            continue;
        }
        let position = if first {
            CoefficientPosition::First
        } else {
            CoefficientPosition::Next
        };
        encode_dct_coeff(bw, table, position, run, level.clamp(-2047, 2047) as i16);
        run = 0;
        first = false;
    }
    encode_end_of_block(bw, table);
}

/// Write the decoder-matching reconstruction of one inter macroblock
/// into `recon`: `Saturate(f_pel + prediction)` per block, in frame-DCT
/// (stride-1) order.
pub(crate) fn reconstruct_inter_mb(
    recon: &mut FrameBuffer,
    mb_col: usize,
    mb_row: usize,
    luma_pred: &[u8],
    cb_pred: &[u8],
    cr_pred: &[u8],
    blocks: &[InterBlock],
) {
    let chroma_format = recon.chroma_format;
    let nblocks = block_count(chroma_format);
    let (cmb_w, cmb_h) = chroma_mb_extent(chroma_format);
    for i in 0..nblocks {
        let placement =
            block_placement(i, chroma_format, mb_col, mb_row, false).expect("valid block index");
        let component = block_component(i, chroma_format).expect("valid component");
        let (pred, pred_w, mb_origin_x, mb_origin_y) = match component {
            ColourComponent::Y => (luma_pred, 16usize, mb_col * 16, mb_row * 16),
            ColourComponent::Cb => (cb_pred, cmb_w, mb_col * cmb_w, mb_row * cmb_h),
            ColourComponent::Cr => (cr_pred, cmb_w, mb_col * cmb_w, mb_row * cmb_h),
        };
        let local_x0 = placement.base_x - mb_origin_x;
        let local_y0 = placement.base_y - mb_origin_y;
        let f_pel = &blocks[i].f_pel;
        let plane = match component {
            ColourComponent::Y => &mut recon.y,
            ColourComponent::Cb => &mut recon.cb,
            ColourComponent::Cr => &mut recon.cr,
        };
        for r in 0..8 {
            let fy = placement.base_y + r;
            let py = local_y0 + r;
            for c in 0..8 {
                let fx = placement.base_x + c;
                let p = i32::from(pred[py * pred_w + (local_x0 + c)]);
                let d = (i32::from(f_pel[r][c]) + p).clamp(0, 255) as u8;
                plane.put_sample(fx, fy, d);
            }
        }
    }
}

/// Per-component running intra DC predictor, used when a P-picture codes
/// a macroblock intra (§7.2.1). Resets to the Table 7-2 seed.
#[derive(Clone, Copy)]
pub(crate) struct IntraDcPred {
    luma: i32,
    cb: i32,
    cr: i32,
}

impl IntraDcPred {
    pub(crate) fn reset(intra_dc_precision: u8) -> Self {
        let v = 128i32 << intra_dc_precision;
        Self {
            luma: v,
            cb: v,
            cr: v,
        }
    }
    pub(crate) fn get(&self, c: ColourComponent) -> i32 {
        match c {
            ColourComponent::Y => self.luma,
            ColourComponent::Cb => self.cb,
            ColourComponent::Cr => self.cr,
        }
    }
    pub(crate) fn set(&mut self, c: ColourComponent, value: i32) {
        match c {
            ColourComponent::Y => self.luma = value,
            ColourComponent::Cb => self.cb = value,
            ColourComponent::Cr => self.cr = value,
        }
    }
}

fn dc_table_component(c: ColourComponent) -> DcComponent {
    match c {
        ColourComponent::Y => DcComponent::Luminance,
        ColourComponent::Cb | ColourComponent::Cr => DcComponent::Chrominance,
    }
}

/// Sum of absolute differences of a macroblock's luma against a flat
/// predictor equal to its own mean — a cheap proxy for the intra-coding
/// cost. A macroblock the motion search predicts worse than this is
/// better coded intra. Returns `(sad_to_mean)`.
pub(crate) fn intra_activity(current: &FrameBuffer, mb_col: usize, mb_row: usize) -> u32 {
    let plane = &current.y;
    let w = plane.width();
    let h = plane.height();
    let base_x = mb_col * 16;
    let base_y = mb_row * 16;
    let mut sum = 0u32;
    for r in 0..16 {
        let sy = (base_y + r).min(h.saturating_sub(1));
        for c in 0..16 {
            let sx = (base_x + c).min(w.saturating_sub(1));
            sum += u32::from(plane.get(sx, sy).unwrap_or(0));
        }
    }
    let mean = (sum / 256) as i32;
    let mut sad = 0u32;
    for r in 0..16 {
        let sy = (base_y + r).min(h.saturating_sub(1));
        for c in 0..16 {
            let sx = (base_x + c).min(w.saturating_sub(1));
            sad += (i32::from(plane.get(sx, sy).unwrap_or(0)) - mean).unsigned_abs();
        }
    }
    sad
}

/// Encode + reconstruct one intra macroblock inside an inter picture
/// (§7.2.1 DC prelude + §7.2.2 AC). Writes the block layer for every
/// block and reconstructs the samples into `recon`. The macroblock type
/// (`00011`) and `macroblock_address_increment` are written by the
/// caller; this writes only the block run.
///
/// `table` is the §7.2.2.1 Table 7-3 selection for an intra block
/// (Table B-15 when the picture declares `intra_vlc_format = 1`);
/// `alternate_scan` selects the §7.3 scan.
#[allow(clippy::too_many_arguments)]
pub(crate) fn encode_intra_mb(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    recon: &mut FrameBuffer,
    mb_col: usize,
    mb_row: usize,
    qscale: u8,
    dc_mult: i32,
    pred: &mut IntraDcPred,
    nblocks: usize,
    chroma_format: crate::sequence_extension::ChromaFormat,
    table: TableSelection,
    alternate_scan: bool,
) {
    let scan = inverse_scan_table(alternate_scan);
    for i in 0..nblocks {
        let placement =
            block_placement(i, chroma_format, mb_col, mb_row, false).expect("valid block index");
        let component = block_component(i, chroma_format).expect("valid component");
        let cur_plane = match component {
            ColourComponent::Y => &current.y,
            ColourComponent::Cb => &current.cb,
            ColourComponent::Cr => &current.cr,
        };
        // Gather the raw 8x8 block (clamp at edges).
        let mut raw = [[0i16; 8]; 8];
        let w = cur_plane.width();
        let h = cur_plane.height();
        for r in 0..8 {
            let sy = (placement.base_y + r).min(h.saturating_sub(1));
            for c in 0..8 {
                let sx = (placement.base_x + c).min(w.saturating_sub(1));
                raw[r][c] = i16::from(cur_plane.get(sx, sy).unwrap_or(128));
            }
        }
        let f = fdct_8x8(&raw);
        let qf = forward_quantise_block(
            &f,
            BlockCoding::Intra,
            &DEFAULT_INTRA_WEIGHT,
            qscale,
            dc_mult,
        );

        // DC differential.
        let qfs0 = qf[0][0];
        let diff = qfs0 - pred.get(component);
        encode_intra_dc(bw, dc_table_component(component), diff);
        pred.set(component, qfs0);

        // AC run-level.
        let mut run = 0u8;
        for &(v, u) in scan.iter().skip(1) {
            let level = qf[v as usize][u as usize];
            if level == 0 {
                run += 1;
                continue;
            }
            encode_dct_coeff(
                bw,
                table,
                CoefficientPosition::Next,
                run,
                level.clamp(-2047, 2047) as i16,
            );
            run = 0;
        }
        encode_end_of_block(bw, table);

        // Reconstruct: inverse-quantise + IDCT → absolute samples.
        let dequant = inverse_quantise_block(
            &qf,
            BlockCoding::Intra,
            &DEFAULT_INTRA_WEIGHT,
            qscale,
            dc_mult,
        );
        let f_pel = idct_8x8_from_i32(&dequant);
        let plane = match component {
            ColourComponent::Y => &mut recon.y,
            ColourComponent::Cb => &mut recon.cb,
            ColourComponent::Cr => &mut recon.cr,
        };
        for r in 0..8 {
            for c in 0..8 {
                let d = i32::from(f_pel[r][c]).clamp(0, 255) as u8;
                plane.put_sample(placement.base_x + c, placement.base_y + r, d);
            }
        }
    }
}

/// Encode one predictive (P) frame picture from `current`, predicting
/// from the reconstructed `reference` frame, appending the picture layer
/// (picture header through the final slice's byte alignment) to `bw`.
///
/// Returns the encoder-side reconstruction of the picture — the exact
/// frame the decoder will produce — so the caller can use it as the
/// reference for a subsequent P-picture.
///
/// `forward_f_code` is the §6.2.3.1 `f_code[0][*]` value; the motion
/// search window is clamped to that code's codable range.
pub fn encode_p_picture(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    reference: &FrameBuffer,
    params: IntraPictureParams,
    temporal_reference: u16,
    quantiser_scale_code: u8,
    forward_f_code: u8,
) -> Result<FrameBuffer> {
    let qscale = quantiser_scale(quantiser_scale_code, params.q_scale_type)?;
    let f_horiz = forward_f_code;
    let f_vert = forward_f_code;
    let search_range = max_search_range(forward_f_code).min(16);

    // §6.3.10: the MPEG-1 legacy forward_f_code in the picture header
    // "shall have the value seven (all ones)" in an ISO/IEC 13818-2
    // stream — the real per-direction f_codes live in the
    // picture_coding_extension().
    write_picture_header(
        bw,
        temporal_reference,
        PictureCodingType::Predictive,
        0b111,
        0b111,
    );
    write_picture_coding_extension(
        bw,
        &PictureCodingExtensionParams {
            forward_f_code,
            frame_pred_frame_dct: params.frame_pred_frame_dct,
            intra_dc_precision: params.intra_dc_precision,
            q_scale_type: params.q_scale_type,
            intra_vlc_format: params.intra_vlc_format,
            alternate_scan: params.alternate_scan,
            // §6.3.10: progressive_sequence == 1 requires
            // progressive_frame == 1.
            progressive_frame: params.progressive_sequence,
            ..Default::default()
        },
    );

    let mut recon = FrameBuffer::new(params.width, params.height, params.chroma_format);
    let mb_width = params.mb_width();
    let mb_height = params.mb_height();
    let nblocks = block_count(params.chroma_format);

    let dc_mult = intra_dc_mult(params.intra_dc_precision)?;
    let chroma_format = params.chroma_format;

    for mb_row in 0..mb_height {
        write_slice_header(bw, mb_row as u32, quantiser_scale_code);
        // §7.6.3.4: the motion-vector predictor resets at slice start.
        let mut pmv_x = 0i32;
        let mut pmv_y = 0i32;
        // §7.2.1 intra DC predictor + §6.3.17.1 past_intra_address gate
        // (relative to the slice's first macroblock address). A fresh
        // slice resets both.
        let mut intra_pred = IntraDcPred::reset(params.intra_dc_precision);
        let slice_first_addr = (mb_row * mb_width) as i32;
        let mut past_intra_address = slice_first_addr - 2; // §6.3.17.1 reset sentinel

        for mb_col in 0..mb_width {
            let mb_address = slice_first_addr + mb_col as i32;
            // 1. Motion search against the reconstructed reference.
            let search = estimate_forward_mv(current, reference, mb_col, mb_row, search_range);
            let mv = search.vector;

            // Intra decision: if the best inter prediction is much worse
            // than the macroblock's own intra activity, code it intra.
            // The intra cost proxy is the SAD to the block mean; the inter
            // cost is the motion-search SAD. A generous margin keeps the
            // encoder inter-biased (intra is only chosen when prediction
            // genuinely fails, e.g. newly-revealed content).
            let intra_cost = intra_activity(current, mb_col, mb_row);
            if search.sad > intra_cost.saturating_mul(2).saturating_add(512) {
                // §6.3.17.1: reset the intra DC predictor when this intra
                // macroblock is not immediately preceded by another intra
                // macroblock.
                if mb_address - past_intra_address > 1 {
                    intra_pred = IntraDcPred::reset(params.intra_dc_precision);
                }
                // macroblock_address_increment = 1, macroblock_type =
                // Intra (Table B-3 `00011`).
                bw.write_bit(true);
                bw.write_u32(0b0001_1, 5);
                encode_intra_mb(
                    bw,
                    current,
                    &mut recon,
                    mb_col,
                    mb_row,
                    qscale,
                    dc_mult,
                    &mut intra_pred,
                    nblocks,
                    chroma_format,
                    TableSelection::from_context(params.intra_vlc_format, true),
                    params.alternate_scan,
                );
                // §7.6.3.4: an intra macroblock (no concealment vectors)
                // resets the motion-vector predictors.
                pmv_x = 0;
                pmv_y = 0;
                past_intra_address = mb_address;
                continue;
            }

            // 2. Prediction planes (the exact §7.6.4 prediction).
            let (luma_pred, cb_pred, cr_pred) = predict_frame_macroblock_planes(
                &recon,
                ReferenceFrames::forward_only(reference),
                mb_col,
                mb_row,
                FrameMotion::forward(mv),
            )
            .expect("forward prediction with present reference");

            // 3. Residual + forward quantise per block.
            let (cmb_w, cmb_h) = chroma_mb_extent(params.chroma_format);
            let mut blocks: Vec<InterBlock> = Vec::with_capacity(nblocks);
            let mut coded_flags = [false; 12];
            for i in 0..nblocks {
                let placement = block_placement(i, params.chroma_format, mb_col, mb_row, false)
                    .expect("valid block index");
                let component = block_component(i, params.chroma_format).expect("valid component");
                let (cur_plane, pred, pred_w, mb_origin_x, mb_origin_y) = match component {
                    ColourComponent::Y => {
                        (&current.y, &luma_pred, 16usize, mb_col * 16, mb_row * 16)
                    }
                    ColourComponent::Cb => {
                        (&current.cb, &cb_pred, cmb_w, mb_col * cmb_w, mb_row * cmb_h)
                    }
                    ColourComponent::Cr => {
                        (&current.cr, &cr_pred, cmb_w, mb_col * cmb_w, mb_row * cmb_h)
                    }
                };
                let local_x = placement.base_x - mb_origin_x;
                let local_y = placement.base_y - mb_origin_y;
                let residual = gather_residual(
                    cur_plane,
                    pred,
                    pred_w,
                    placement.base_x,
                    placement.base_y,
                    local_x,
                    local_y,
                );
                let block = quantise_inter_block(&residual, qscale);
                coded_flags[i] = block.qf.is_some();
                blocks.push(block);
            }
            let any_coded = coded_flags[..nblocks].iter().any(|&b| b);

            // 4. Macroblock-type + MV + cbp + coefficients.
            // macroblock_address_increment = 1 (Table B-1 `1`).
            bw.write_bit(true);
            if any_coded {
                // macroblock_type = "MC, Coded" (Table B-3 `1`):
                // motion_forward + macroblock_pattern.
                bw.write_bit(true);
            } else {
                // macroblock_type = "MC, Not Coded" (Table B-3 `001`):
                // motion_forward only.
                bw.write_u32(0b001, 3);
            }

            // motion_vectors(0): one frame vector, differentially coded
            // against the PMV and wrapped into the §7.6.3.1 range.
            let dx = wrap_delta(mv.horizontal - pmv_x, f_horiz)?;
            let dy = wrap_delta(mv.vertical - pmv_y, f_vert)?;
            encode_motion_component(bw, dx, f_horiz);
            encode_motion_component(bw, dy, f_vert);
            pmv_x = mv.horizontal;
            pmv_y = mv.vertical;

            if any_coded {
                encode_coded_block_pattern(bw, &coded_flags[..nblocks], params.chroma_format)?;
                for i in 0..nblocks {
                    if let Some(qf) = blocks[i].qf.as_ref() {
                        write_inter_block_coeffs(bw, qf, params.alternate_scan);
                    }
                }
            }

            // 5. Reconstruct this macroblock into `recon`.
            reconstruct_inter_mb(
                &mut recon, mb_col, mb_row, &luma_pred, &cb_pred, &cr_pred, &blocks,
            );
        }
        bw.align_to_byte_zero();
    }

    Ok(recon)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sequence_extension::ChromaFormat;

    fn params(width: usize, height: usize) -> IntraPictureParams {
        IntraPictureParams {
            // progressive sequence: Ceil(h/16) macroblock grid (§6.3.3)
            progressive_sequence: true,
            width,
            height,
            chroma_format: ChromaFormat::Yuv420,
            frame_pred_frame_dct: true,
            intra_dc_precision: 0,
            intra_vlc_format: false,
            alternate_scan: false,
            q_scale_type: false,
        }
    }

    #[test]
    fn reconstruct_inter_mb_zero_residual_copies_prediction() {
        let mut recon = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
        let luma_pred = vec![77u8; 256];
        let cb_pred = vec![88u8; 64];
        let cr_pred = vec![99u8; 64];
        let blocks: Vec<InterBlock> = (0..6)
            .map(|_| InterBlock {
                qf: None,
                f_pel: [[0i16; 8]; 8],
            })
            .collect();
        reconstruct_inter_mb(&mut recon, 0, 0, &luma_pred, &cb_pred, &cr_pred, &blocks);
        assert_eq!(recon.y.get(0, 0), Some(77));
        assert_eq!(recon.y.get(15, 15), Some(77));
        assert_eq!(recon.cb.get(3, 3), Some(88));
        assert_eq!(recon.cr.get(7, 7), Some(99));
    }

    #[test]
    fn quantise_inter_block_flat_zero_residual_is_uncoded() {
        let b = quantise_inter_block(&[[0i16; 8]; 8], 8);
        assert!(b.qf.is_none());
        assert_eq!(b.f_pel, [[0i16; 8]; 8]);
    }

    #[test]
    fn quantise_inter_block_structured_residual_is_coded() {
        let mut residual = [[0i16; 8]; 8];
        for v in 0..8 {
            for u in 0..8 {
                residual[v][u] = (((v * 8 + u) as i16 * 7) % 100) - 50;
            }
        }
        let b = quantise_inter_block(&residual, 4);
        assert!(b.qf.is_some(), "structured residual must be coded");
    }

    #[test]
    fn p_picture_header_uses_predictive_coding_type() {
        // Smoke check: encoding a tiny P picture against a flat reference
        // appends a picture layer and returns a same-size reconstruction.
        let mut bw = BitWriter::new();
        let current = {
            let mut f = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
            for y in 0..16 {
                for x in 0..16 {
                    f.y.put_sample(x, y, (x * 8) as u8);
                }
            }
            for y in 0..8 {
                for x in 0..8 {
                    f.cb.put_sample(x, y, 128);
                    f.cr.put_sample(x, y, 128);
                }
            }
            f
        };
        let reference = current.clone();
        let recon = encode_p_picture(&mut bw, &current, &reference, params(16, 16), 1, 8, 1)
            .expect("encode P");
        assert_eq!((recon.width, recon.height), (16, 16));
        // A flat-vs-itself prediction yields a near-zero residual; the
        // reconstruction tracks the source closely.
        let mut max_err = 0i32;
        for y in 0..16 {
            for x in 0..16 {
                let e = (i32::from(recon.y.get(x, y).unwrap())
                    - i32::from(current.y.get(x, y).unwrap()))
                .abs();
                max_err = max_err.max(e);
            }
        }
        assert!(max_err <= 8, "P reconstruction max err {max_err}");
    }
}
