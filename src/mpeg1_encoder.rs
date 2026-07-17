//! ISO/IEC 11172-2 (MPEG-1 Video) picture and sequence **encoders** —
//! the 11172-2 counterpart of [`crate::intra_encoder`] /
//! [`crate::p_picture_encoder`] / [`crate::b_picture_encoder`] /
//! [`crate::inter_encoder`], built as the bit-exact inverse of the
//! crate's own §2.4 MPEG-1 decode pipeline so that everything emitted
//! round-trips through [`crate::decode_video_sequence`].
//!
//! ## What differs from the MPEG-2 encoders
//!
//! * **Sequence layer** — a bare §2.4.2.3 `sequence_header()`
//!   ([`crate::mpeg1_stream_writer`]) with **no** `sequence_extension()`
//!   (its absence classifies the stream as 11172-2), followed by the
//!   **mandatory** §2.4.2.4 group-of-pictures layer: §2.4.2.2 loops
//!   `group_of_pictures()` directly under each `sequence_header()`, so
//!   every emitted stream carries at least one GOP header
//!   ([`crate::gop_header::write_gop_header`]) with a §2.4.3.3
//!   time code derived from the GOP's first display frame.
//! * **Picture layer** — no `picture_coding_extension()`; the motion
//!   context is the picture header's own `full_pel_*_vector` (`'0'`
//!   here) and 3-bit `forward_f_code` / `backward_f_code` (`1..=7`,
//!   §2.4.3.4).
//! * **Block layer** — the §2.4.2.8 DC prelude (Tables B.5a / B.5b via
//!   [`crate::block_dc::encode_dc_coefficient`], differential against
//!   the §2.4.4.1 `dct_dc_*_past` chain in **reconstruction units /
//!   8**), the §2.4.3.7 run-level VLCs with the Table B.5f escape
//!   ([`crate::dct_coeff::encode_dct_coeff`]), and the §2.4.4.1 /
//!   §2.4.4.2 forward quantisers
//!   ([`crate::forward_quant::mpeg1_forward_quantise_intra_ac`] /
//!   [`crate::forward_quant::mpeg1_forward_quantise_non_intra`]) whose
//!   `quantizer_scale` is the §2.4.3.6 value used directly.
//! * **Motion vectors** — the §2.4.4.2 / §2.4.4.3 predictor lifecycle
//!   (reset at slice start; P additionally resets after an intra
//!   macroblock; a B direction's predictor updates only when that
//!   direction transmits a vector), with the wire form shared with
//!   MPEG-2 ([`crate::motion_vector::encode_motion_component`]: the
//!   Table B.4 `motion_code` VLC is Table B-10 row-for-row and the
//!   residual gate/width arithmetic coincides).
//!
//! Every reconstruction the encoder carries forward is produced by the
//! decoder's own arithmetic ([`crate::dequantize`] + [`crate::idct`] +
//! the shared §7.6-equivalent prediction core), so the encoder's
//! reference frames are the decoder's exact reconstructions and
//! prediction drift cannot accumulate.

#![allow(clippy::too_many_arguments)]
#![allow(clippy::needless_range_loop)]
// Macroblock-type codewords are grouped to mirror the printed Annex B
// tables (`0001_1` etc.) for audit against the spec pages.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitWriter;

use crate::block_dc::{encode_dc_coefficient, DcComponent};
use crate::coded_block_pattern::encode_cbp420;
use crate::dct_coeff::{encode_dct_coeff, encode_end_of_block, CoefficientPosition};
use crate::dequantize::{
    dequantize_intra_block, dequantize_non_intra_block, finalise_intra_macroblock, IntraBlockKind,
    IntraDcPredictors, DC_PREDICTOR_RESET,
};
use crate::forward_dct::fdct_8x8;
use crate::forward_quant::{mpeg1_forward_quantise_intra_ac, mpeg1_forward_quantise_non_intra};
use crate::frame_assembly::{block_placement, place_intra_block, FrameBuffer};
use crate::gop_header::{write_gop_header, Mpeg2Gop, TimeCode};
use crate::idct::idct_8x8_from_i32;
use crate::inter_reconstruction::{predict_frame_macroblock_planes, FrameMotion, ReferenceFrames};
use crate::motion_estimation::{estimate_forward_mv, max_search_range};
use crate::motion_vector::encode_motion_component;
use crate::mpeg1_block_decoder::intra_block_kind;
use crate::mpeg1_picture::Mpeg1PictureParams;
use crate::mpeg1_stream_writer::{
    constrained_parameters_admissible, write_mpeg1_sequence_header, Mpeg1SequenceParams,
};
use crate::mpeg2_block_dc::ColourComponent;
use crate::mpeg2_macroblock_blocks::block_component;
use crate::p_picture_encoder::wrap_delta;
use crate::picture_header::PictureCodingType;
use crate::sequence_extension::ChromaFormat;
use crate::stream_writer::{write_picture_header, write_slice_header, SEQUENCE_END_CODE};
use crate::{Error, Result};

/// A macroblock's chroma extent in samples for the 4:2:0 layout MPEG-1
/// mandates (§2.4.1): 8×8 per chroma block, one Cb + one Cr block.
const CHROMA_MB: usize = 8;

/// MPEG-1 4:2:0 blocks per macroblock (§2.4.3.7: four Y, Cb, Cr).
const NBLOCKS: usize = 6;

// =============================================================
// Shared helpers
// =============================================================

/// Validate the §2.4.3.6 `quantizer_scale` range (`1..=31`; MPEG-1
/// uses the value directly — there is no §7.4.2.2 mapping).
fn check_quantizer_scale(quantizer_scale_code: u8) -> Result<()> {
    if (1..=31).contains(&quantizer_scale_code) {
        Ok(())
    } else {
        Err(Error::InvalidBitstream(
            "mpeg1 encoder: quantizer_scale outside 1..=31 (§2.4.3.6)",
        ))
    }
}

/// Validate a §2.4.3.4 `forward_f_code` / `backward_f_code` (`1..=7`).
fn check_f_code(f_code: u8) -> Result<()> {
    if (1..=7).contains(&f_code) {
        Ok(())
    } else {
        Err(Error::InvalidBitstream(
            "mpeg1 encoder: f_code outside 1..=7 (§2.4.3.4)",
        ))
    }
}

/// Which Table B.5a / B.5b DC VLC a 4:2:0 block index uses.
fn dc_component_for(block_index: usize) -> DcComponent {
    if block_index < 4 {
        DcComponent::Luminance
    } else {
        DcComponent::Chrominance
    }
}

/// Gather an 8×8 block of raw samples from a component plane,
/// clamping reads past the visible edge to the nearest valid sample
/// (edge macroblocks pad to the 16-grid).
fn gather_block(
    frame: &FrameBuffer,
    c: ColourComponent,
    base_x: usize,
    base_y: usize,
) -> [[i16; 8]; 8] {
    let plane = match c {
        ColourComponent::Y => &frame.y,
        ColourComponent::Cb => &frame.cb,
        ColourComponent::Cr => &frame.cr,
    };
    let w = plane.width();
    let h = plane.height();
    let mut out = [[0i16; 8]; 8];
    for (r, row) in out.iter_mut().enumerate() {
        let sy = (base_y + r).min(h.saturating_sub(1));
        for (cx, cell) in row.iter_mut().enumerate() {
            let sx = (base_x + cx).min(w.saturating_sub(1));
            *cell = i16::from(plane.get(sx, sy).unwrap_or(128));
        }
    }
    out
}

/// Gather an 8×8 `current - prediction` residual block. `pred` is the
/// macroblock-local prediction plane (`pred_w` wide); `(base_x,
/// base_y)` the block's frame-plane origin, `(local_x, local_y)` its
/// origin inside the prediction plane.
fn gather_residual(
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
        for c in 0..8 {
            let sx = (base_x + c).min(w.saturating_sub(1));
            let cur = i32::from(cur_plane.get(sx, sy).unwrap_or(0));
            let prd = i32::from(pred[(local_y + r) * pred_w + (local_x + c)]);
            out[r][c] = (cur - prd) as i16;
        }
    }
    out
}

/// Emit the §2.4.3.7 coefficient walk of one **non-intra** block from
/// its zig-zag `dct_zz[]`: `dct_coeff_first` (`i = run`) for the first
/// non-zero entry, `dct_coeff_next` (`i += run + 1`) for the rest,
/// `end_of_block` terminator.
fn write_non_intra_coefficients(bw: &mut BitWriter, dct_zz: &[i32; 64]) {
    let mut prev: Option<usize> = None;
    for (j, &level) in dct_zz.iter().enumerate() {
        if level == 0 {
            continue;
        }
        match prev {
            None => {
                // dct_coeff_first: i = run.
                encode_dct_coeff(bw, CoefficientPosition::First, j as u8, level as i16);
            }
            Some(p) => {
                encode_dct_coeff(
                    bw,
                    CoefficientPosition::Next,
                    (j - p - 1) as u8,
                    level as i16,
                );
            }
        }
        prev = Some(j);
    }
    encode_end_of_block(bw);
}

/// Emit the §2.4.3.7 AC walk of one **intra** block (the DC field
/// stood in for index 0): every symbol is `dct_coeff_next` with the
/// running index starting at 0.
fn write_intra_ac_coefficients(bw: &mut BitWriter, dct_zz: &[i32; 64]) {
    let mut prev = 0usize;
    for (j, &level) in dct_zz.iter().enumerate().skip(1) {
        if level == 0 {
            continue;
        }
        encode_dct_coeff(
            bw,
            CoefficientPosition::Next,
            (j - prev - 1) as u8,
            level as i16,
        );
        prev = j;
    }
    encode_end_of_block(bw);
}

/// The §2.4.4.1 DC base the decoder will add `dct_zz[0] * 8` to for
/// this block — the per-component `dct_dc_*_past` predictor, or the
/// `128 * 8` reset when the
/// `(macroblock_address - past_intra_address) > 1` branch fires (every
/// block kind except `LuminanceSubsequent` evaluates the reset test).
fn dc_base(kind: IntraBlockKind, predictors: &IntraDcPredictors, address: i32) -> i32 {
    let reset = (address - predictors.past_intra_address) > 1;
    match kind {
        IntraBlockKind::LuminanceFirst => {
            if reset {
                DC_PREDICTOR_RESET
            } else {
                predictors.y_past
            }
        }
        IntraBlockKind::LuminanceSubsequent => predictors.y_past,
        IntraBlockKind::ChrominanceCb => {
            if reset {
                DC_PREDICTOR_RESET
            } else {
                predictors.cb_past
            }
        }
        IntraBlockKind::ChrominanceCr => {
            if reset {
                DC_PREDICTOR_RESET
            } else {
                predictors.cr_past
            }
        }
    }
}

/// Encode + reconstruct one **intra** macroblock (six 4:2:0 blocks).
/// The caller has already written `macroblock_address_increment` and
/// `macroblock_type`; this writes the six block layers and places the
/// §2.4.4.1 reconstruction into `recon`, updating the shared
/// `dct_dc_*_past` chain exactly as the decoder will.
fn encode_intra_macroblock(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    recon: &mut FrameBuffer,
    mb_col: usize,
    mb_row: usize,
    quantizer_scale: u8,
    params: &Mpeg1PictureParams,
    predictors: &mut IntraDcPredictors,
    address: i32,
) -> Result<()> {
    for i in 0..NBLOCKS {
        let placement = block_placement(i, ChromaFormat::Yuv420, mb_col, mb_row, false).ok_or(
            Error::InvalidBitstream("mpeg1 encoder: invalid block index"),
        )?;
        let component = block_component(i, ChromaFormat::Yuv420).ok_or(Error::InvalidBitstream(
            "mpeg1 encoder: invalid block component",
        ))?;
        let raw = gather_block(current, component, placement.base_x, placement.base_y);
        let f = fdct_8x8(&raw);

        // AC levels (§2.4.4.1 inverse), then the DC differential
        // against the exact §2.4.4.1 base the decoder will use. The
        // predictor chain lives on the reconstruction lattice
        // (multiples of 8: the slice-start reset is 128*8 and every
        // update adds dct_zz[0]*8), so base/8 is exact.
        let mut dct_zz = mpeg1_forward_quantise_intra_ac(&f, quantizer_scale, &params.intra_quant);
        let kind = intra_block_kind(i as u8)?;
        let base = dc_base(kind, predictors, address);
        debug_assert_eq!(base % 8, 0);
        // F[0][0] of 8-bit samples is non-negative (8 × block mean).
        let dc_target = (f[0][0] + 4) / 8;
        let diff = (dc_target - base / 8).clamp(-255, 255);
        dct_zz[0] = diff;

        encode_dc_coefficient(bw, dc_component_for(i), diff);
        write_intra_ac_coefficients(bw, &dct_zz);

        // Reconstruct with the decoder's own §2.4.4.1 arithmetic (this
        // also advances the dct_dc_*_past chain).
        let dct_recon = dequantize_intra_block(
            &dct_zz,
            quantizer_scale,
            &params.intra_quant,
            kind,
            predictors,
            address,
        )?;
        let f_pel = idct_8x8_from_i32(&dct_recon);
        place_intra_block(recon, placement, &f_pel);
    }
    finalise_intra_macroblock(predictors, address);
    Ok(())
}

/// One forward-quantised non-intra block: the zig-zag levels (when any
/// are non-zero) and the decoder-exact residual reconstruction.
struct Mpeg1InterBlock {
    /// §2.4.3.7 zig-zag levels, `None` when every level quantised to
    /// zero (the block is not coded).
    dct_zz: Option<[i32; 64]>,
    /// The §2.4.4.2 + Annex A reconstruction (all zero when uncoded).
    f_pel: [[i16; 8]; 8],
}

/// Forward-quantise one residual block with the §2.4.4.2 dead-zone
/// quantiser and reconstruct it exactly as the decoder will.
fn quantise_inter_block(
    residual: &[[i16; 8]; 8],
    quantizer_scale: u8,
    non_intra_quant: &[[u8; 8]; 8],
) -> Result<Mpeg1InterBlock> {
    let f = fdct_8x8(residual);
    let dct_zz = mpeg1_forward_quantise_non_intra(&f, quantizer_scale, non_intra_quant);
    if dct_zz.iter().all(|&z| z == 0) {
        return Ok(Mpeg1InterBlock {
            dct_zz: None,
            f_pel: [[0i16; 8]; 8],
        });
    }
    let dct_recon = dequantize_non_intra_block(&dct_zz, quantizer_scale, non_intra_quant)?;
    let f_pel = idct_8x8_from_i32(&dct_recon);
    Ok(Mpeg1InterBlock {
        dct_zz: Some(dct_zz),
        f_pel,
    })
}

/// Quantise all six residual blocks of a macroblock against its
/// prediction planes, returning the blocks and the §2.4.3.6 cbp.
fn quantise_macroblock_residuals(
    current: &FrameBuffer,
    luma_pred: &[u8],
    cb_pred: &[u8],
    cr_pred: &[u8],
    mb_col: usize,
    mb_row: usize,
    quantizer_scale: u8,
    non_intra_quant: &[[u8; 8]; 8],
) -> Result<(Vec<Mpeg1InterBlock>, u8)> {
    let mut blocks = Vec::with_capacity(NBLOCKS);
    let mut cbp = 0u8;
    for i in 0..NBLOCKS {
        let placement = block_placement(i, ChromaFormat::Yuv420, mb_col, mb_row, false).ok_or(
            Error::InvalidBitstream("mpeg1 encoder: invalid block index"),
        )?;
        let component = block_component(i, ChromaFormat::Yuv420).ok_or(Error::InvalidBitstream(
            "mpeg1 encoder: invalid block component",
        ))?;
        let (cur_plane, pred, pred_w, origin_x, origin_y) = match component {
            ColourComponent::Y => (&current.y, luma_pred, 16usize, mb_col * 16, mb_row * 16),
            ColourComponent::Cb => (
                &current.cb,
                cb_pred,
                CHROMA_MB,
                mb_col * CHROMA_MB,
                mb_row * CHROMA_MB,
            ),
            ColourComponent::Cr => (
                &current.cr,
                cr_pred,
                CHROMA_MB,
                mb_col * CHROMA_MB,
                mb_row * CHROMA_MB,
            ),
        };
        let residual = gather_residual(
            cur_plane,
            pred,
            pred_w,
            placement.base_x,
            placement.base_y,
            placement.base_x - origin_x,
            placement.base_y - origin_y,
        );
        let block = quantise_inter_block(&residual, quantizer_scale, non_intra_quant)?;
        if block.dct_zz.is_some() {
            cbp |= 1 << (5 - i);
        }
        blocks.push(block);
    }
    Ok((blocks, cbp))
}

/// Write the decoder-matching reconstruction of one inter macroblock
/// into `recon`: `Saturate(f_pel + prediction)` per block.
fn reconstruct_inter_mb(
    recon: &mut FrameBuffer,
    mb_col: usize,
    mb_row: usize,
    luma_pred: &[u8],
    cb_pred: &[u8],
    cr_pred: &[u8],
    blocks: &[Mpeg1InterBlock],
) {
    for i in 0..NBLOCKS {
        let placement =
            block_placement(i, ChromaFormat::Yuv420, mb_col, mb_row, false).expect("valid index");
        let component = block_component(i, ChromaFormat::Yuv420).expect("valid component");
        let (pred, pred_w, origin_x, origin_y) = match component {
            ColourComponent::Y => (luma_pred, 16usize, mb_col * 16, mb_row * 16),
            ColourComponent::Cb => (cb_pred, CHROMA_MB, mb_col * CHROMA_MB, mb_row * CHROMA_MB),
            ColourComponent::Cr => (cr_pred, CHROMA_MB, mb_col * CHROMA_MB, mb_row * CHROMA_MB),
        };
        let local_x0 = placement.base_x - origin_x;
        let local_y0 = placement.base_y - origin_y;
        let f_pel = &blocks[i].f_pel;
        let plane = match component {
            ColourComponent::Y => &mut recon.y,
            ColourComponent::Cb => &mut recon.cb,
            ColourComponent::Cr => &mut recon.cr,
        };
        for r in 0..8 {
            for c in 0..8 {
                let p = i32::from(pred[(local_y0 + r) * pred_w + (local_x0 + c)]);
                let d = (i32::from(f_pel[r][c]) + p).clamp(0, 255) as u8;
                plane.put_sample(placement.base_x + c, placement.base_y + r, d);
            }
        }
    }
}

/// Luma SAD of the current macroblock against its own mean — the
/// cheap intra-cost proxy shared with the MPEG-2 P encoder's
/// intra-fallback decision.
fn intra_activity(current: &FrameBuffer, mb_col: usize, mb_row: usize) -> u32 {
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

/// Luma SAD between the current macroblock and a 16×16 prediction.
fn luma_sad(current: &FrameBuffer, mb_col: usize, mb_row: usize, pred: &[u8]) -> u32 {
    let plane = &current.y;
    let w = plane.width();
    let h = plane.height();
    let base_x = mb_col * 16;
    let base_y = mb_row * 16;
    let mut sad = 0u32;
    for r in 0..16 {
        let sy = (base_y + r).min(h.saturating_sub(1));
        for c in 0..16 {
            let sx = (base_x + c).min(w.saturating_sub(1));
            let cur = i32::from(plane.get(sx, sy).unwrap_or(0));
            let prd = i32::from(pred[r * 16 + c]);
            sad += (cur - prd).unsigned_abs();
        }
    }
    sad
}

/// Frame rows of macroblocks (`Ceil(height / 16)` — MPEG-1 pictures
/// are always frame pictures).
fn mb_height(params: &Mpeg1PictureParams) -> usize {
    params.height.div_ceil(16)
}

// =============================================================
// Picture encoders
// =============================================================

/// Encode one MPEG-1 **I** picture from `frame`, appending the
/// picture layer (picture header through the final slice's byte
/// alignment) to `bw`. One slice per macroblock row; every macroblock
/// coded (§2.4.4.4 requires it for all-intra pictures).
///
/// Returns the §2.4.4.1 reconstruction — the exact frame the decoder
/// will produce — for use as the forward reference of a following
/// P/B picture.
///
/// # Errors
/// [`Error::InvalidBitstream`] on geometry mismatch or an
/// out-of-range `quantizer_scale_code`.
pub fn encode_mpeg1_intra_picture(
    bw: &mut BitWriter,
    frame: &FrameBuffer,
    params: &Mpeg1PictureParams,
    temporal_reference: u16,
    quantizer_scale_code: u8,
) -> Result<FrameBuffer> {
    if frame.width != params.width || frame.height != params.height {
        return Err(Error::InvalidBitstream(
            "encode_mpeg1_intra_picture: frame dimensions do not match params",
        ));
    }
    check_quantizer_scale(quantizer_scale_code)?;

    // I-pictures carry no f_code fields; the writer's P/B-gated
    // arguments are unused.
    write_picture_header(bw, temporal_reference, PictureCodingType::Intra, 7, 7);

    let mut recon = FrameBuffer::new(params.width, params.height, ChromaFormat::Yuv420);
    let mb_w = params.mb_width();
    for mb_row in 0..mb_height(params) {
        write_slice_header(bw, mb_row as u32, quantizer_scale_code);
        // §2.4.4.1: the DC chain resets at slice start.
        let mut predictors = IntraDcPredictors::at_slice_start();
        for mb_col in 0..mb_w {
            let address = (mb_row * mb_w + mb_col) as i32;
            // macroblock_address_increment = 1 (Table B.1 `1`).
            bw.write_bit(true);
            // macroblock_type = intra (Table B.2a `1`).
            bw.write_bit(true);
            encode_intra_macroblock(
                bw,
                frame,
                &mut recon,
                mb_col,
                mb_row,
                quantizer_scale_code,
                params,
                &mut predictors,
                address,
            )?;
        }
        bw.align_to_byte_zero();
    }
    Ok(recon)
}

/// Encode one MPEG-1 **P** picture from `current`, predicting from
/// the reconstructed `reference`, appending the picture layer to
/// `bw`. Per macroblock: a full-search + half-pel motion estimate,
/// the §2.4.4.2 residual, Table B.2b `MC, Coded` / `MC, Not Coded`
/// modes, and an intra fallback (`0001 1`) for content the prediction
/// cannot capture. Motion vectors are coded differentially against
/// the §2.4.4.2 `recon_*_prev` pair (reset at slice start and after
/// every intra macroblock).
///
/// Returns the encoder-side reconstruction (the decoder's exact
/// frame) to chain as the next reference.
pub fn encode_mpeg1_p_picture(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    reference: &FrameBuffer,
    params: &Mpeg1PictureParams,
    temporal_reference: u16,
    quantizer_scale_code: u8,
    forward_f_code: u8,
) -> Result<FrameBuffer> {
    check_quantizer_scale(quantizer_scale_code)?;
    check_f_code(forward_f_code)?;
    let search_range = max_search_range(forward_f_code).min(16);

    write_picture_header(
        bw,
        temporal_reference,
        PictureCodingType::Predictive,
        forward_f_code,
        7,
    );

    let mut recon = FrameBuffer::new(params.width, params.height, ChromaFormat::Yuv420);
    let mb_w = params.mb_width();
    for mb_row in 0..mb_height(params) {
        write_slice_header(bw, mb_row as u32, quantizer_scale_code);
        // §2.4.4.2: recon_right_for_prev / recon_down_for_prev reset
        // at slice start; §2.4.4.1: the DC chain resets too.
        let mut pmv = (0i32, 0i32);
        let mut predictors = IntraDcPredictors::at_slice_start();

        for mb_col in 0..mb_w {
            let address = (mb_row * mb_w + mb_col) as i32;
            let search = estimate_forward_mv(current, reference, mb_col, mb_row, search_range);
            let mv = search.vector;

            // Intra fallback when the best prediction is much worse
            // than the macroblock's own activity.
            let intra_cost = intra_activity(current, mb_col, mb_row);
            if search.sad > intra_cost.saturating_mul(2).saturating_add(512) {
                bw.write_bit(true); // increment 1
                bw.write_u32(0b0001_1, 5); // Table B.2b intra
                encode_intra_macroblock(
                    bw,
                    current,
                    &mut recon,
                    mb_col,
                    mb_row,
                    quantizer_scale_code,
                    params,
                    &mut predictors,
                    address,
                )?;
                // §2.4.4.2: an intra macroblock carries no forward MV
                // data — the predictors reset.
                pmv = (0, 0);
                continue;
            }

            let (luma_pred, cb_pred, cr_pred) = predict_frame_macroblock_planes(
                &recon,
                ReferenceFrames::forward_only(reference),
                mb_col,
                mb_row,
                FrameMotion::forward(mv),
            )
            .map_err(Error::from)?;

            let (blocks, cbp) = quantise_macroblock_residuals(
                current,
                &luma_pred,
                &cb_pred,
                &cr_pred,
                mb_col,
                mb_row,
                quantizer_scale_code,
                &params.non_intra_quant,
            )?;

            // macroblock_address_increment = 1.
            bw.write_bit(true);
            if cbp != 0 {
                // Table B.2b `MC, Coded` (`1`).
                bw.write_bit(true);
            } else {
                // Table B.2b `MC, Not Coded` (`001`).
                bw.write_u32(0b001, 3);
            }

            // Forward motion vector, §2.4.4.2 differential.
            let dx = wrap_delta(mv.horizontal - pmv.0, forward_f_code)?;
            let dy = wrap_delta(mv.vertical - pmv.1, forward_f_code)?;
            encode_motion_component(bw, dx, forward_f_code);
            encode_motion_component(bw, dy, forward_f_code);
            pmv = (mv.horizontal, mv.vertical);

            if cbp != 0 {
                encode_cbp420(bw, cbp);
                for block in &blocks {
                    if let Some(zz) = block.dct_zz.as_ref() {
                        write_non_intra_coefficients(bw, zz);
                    }
                }
            }

            reconstruct_inter_mb(
                &mut recon, mb_col, mb_row, &luma_pred, &cb_pred, &cr_pred, &blocks,
            );
        }
        bw.align_to_byte_zero();
    }
    Ok(recon)
}

/// The chosen §2.4.4.3 prediction direction for one B macroblock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BDirection {
    Forward,
    Backward,
    Interpolated,
}

/// Emit the Table B.2c `macroblock_type` codeword for the chosen
/// direction + coded flag (the `macroblock_quant == 0` rows).
fn write_b_macroblock_type(bw: &mut BitWriter, dir: BDirection, coded: bool) {
    let (code, bits): (u32, u32) = match (dir, coded) {
        (BDirection::Interpolated, false) => (0b10, 2),
        (BDirection::Interpolated, true) => (0b11, 2),
        (BDirection::Backward, false) => (0b010, 3),
        (BDirection::Backward, true) => (0b011, 3),
        (BDirection::Forward, false) => (0b0010, 4),
        (BDirection::Forward, true) => (0b0011, 4),
    };
    bw.write_u32(code, bits);
}

/// Encode one MPEG-1 **B** picture from `current`, predicting from
/// the reconstructed `forward` (past) and `backward` (future)
/// anchors. Per macroblock the forward / backward / interpolated
/// predictions are scored by luma SAD and the cheapest is coded
/// (Table B.2c), forward vectors before backward, each direction with
/// its own §2.4.4.3 predictor pair (updated only when that direction
/// transmits — the carry-over-on-absence rule).
///
/// B-pictures are never references, so nothing is returned.
pub fn encode_mpeg1_b_picture(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    forward: &FrameBuffer,
    backward: &FrameBuffer,
    params: &Mpeg1PictureParams,
    temporal_reference: u16,
    quantizer_scale_code: u8,
    forward_f_code: u8,
    backward_f_code: u8,
) -> Result<()> {
    check_quantizer_scale(quantizer_scale_code)?;
    check_f_code(forward_f_code)?;
    check_f_code(backward_f_code)?;
    let fwd_range = max_search_range(forward_f_code).min(16);
    let bwd_range = max_search_range(backward_f_code).min(16);

    write_picture_header(
        bw,
        temporal_reference,
        PictureCodingType::Bidirectional,
        forward_f_code,
        backward_f_code,
    );

    let mb_w = params.mb_width();
    // Geometry carrier for the prediction former (samples never read).
    let geom = FrameBuffer::new(params.width, params.height, ChromaFormat::Yuv420);

    for mb_row in 0..mb_height(params) {
        write_slice_header(bw, mb_row as u32, quantizer_scale_code);
        // §2.4.4.3: both predictor pairs reset at slice start.
        let mut pmv_fwd = (0i32, 0i32);
        let mut pmv_bwd = (0i32, 0i32);

        for mb_col in 0..mb_w {
            let fwd = estimate_forward_mv(current, forward, mb_col, mb_row, fwd_range).vector;
            let bwd = estimate_forward_mv(current, backward, mb_col, mb_row, bwd_range).vector;

            let pred_fwd = predict_frame_macroblock_planes(
                &geom,
                ReferenceFrames::forward_only(forward),
                mb_col,
                mb_row,
                FrameMotion::forward(fwd),
            )
            .map_err(Error::from)?;
            let pred_bwd = predict_frame_macroblock_planes(
                &geom,
                ReferenceFrames {
                    forward: None,
                    backward: Some(backward),
                },
                mb_col,
                mb_row,
                FrameMotion::backward(bwd),
            )
            .map_err(Error::from)?;
            let pred_int = predict_frame_macroblock_planes(
                &geom,
                ReferenceFrames::bidirectional(forward, backward),
                mb_col,
                mb_row,
                FrameMotion::bidirectional(fwd, bwd),
            )
            .map_err(Error::from)?;

            let sad_fwd = luma_sad(current, mb_col, mb_row, &pred_fwd.0);
            let sad_bwd = luma_sad(current, mb_col, mb_row, &pred_bwd.0);
            let sad_int = luma_sad(current, mb_col, mb_row, &pred_int.0);

            let (dir, chosen) = {
                let mut best_dir = BDirection::Forward;
                let mut best_sad = sad_fwd;
                let mut best = &pred_fwd;
                if sad_bwd < best_sad {
                    best_dir = BDirection::Backward;
                    best_sad = sad_bwd;
                    best = &pred_bwd;
                }
                if sad_int < best_sad {
                    best_dir = BDirection::Interpolated;
                    best = &pred_int;
                }
                (best_dir, best)
            };
            let (luma_pred, cb_pred, cr_pred) = (&chosen.0, &chosen.1, &chosen.2);

            let (blocks, cbp) = quantise_macroblock_residuals(
                current,
                luma_pred,
                cb_pred,
                cr_pred,
                mb_col,
                mb_row,
                quantizer_scale_code,
                &params.non_intra_quant,
            )?;

            bw.write_bit(true); // increment 1
            let coded = cbp != 0;
            write_b_macroblock_type(bw, dir, coded);

            // Forward vector(s) precede backward (§2.4.2.7 order).
            if matches!(dir, BDirection::Forward | BDirection::Interpolated) {
                let dx = wrap_delta(fwd.horizontal - pmv_fwd.0, forward_f_code)?;
                let dy = wrap_delta(fwd.vertical - pmv_fwd.1, forward_f_code)?;
                encode_motion_component(bw, dx, forward_f_code);
                encode_motion_component(bw, dy, forward_f_code);
                pmv_fwd = (fwd.horizontal, fwd.vertical);
            }
            if matches!(dir, BDirection::Backward | BDirection::Interpolated) {
                let dx = wrap_delta(bwd.horizontal - pmv_bwd.0, backward_f_code)?;
                let dy = wrap_delta(bwd.vertical - pmv_bwd.1, backward_f_code)?;
                encode_motion_component(bw, dx, backward_f_code);
                encode_motion_component(bw, dy, backward_f_code);
                pmv_bwd = (bwd.horizontal, bwd.vertical);
            }

            if coded {
                encode_cbp420(bw, cbp);
                for block in &blocks {
                    if let Some(zz) = block.dct_zz.as_ref() {
                        write_non_intra_coefficients(bw, zz);
                    }
                }
            }
        }
        bw.align_to_byte_zero();
    }
    Ok(())
}

// =============================================================
// Whole-sequence assembler (GOP structure)
// =============================================================

/// Derive the [`Mpeg1PictureParams`] (default §2.4.3.2 matrices) for a
/// sequence-parameter set.
fn picture_params(seq: &Mpeg1SequenceParams) -> Mpeg1PictureParams {
    Mpeg1PictureParams {
        width: usize::from(seq.horizontal_size),
        height: usize::from(seq.vertical_size),
        intra_quant: crate::dequantize::DEFAULT_INTRA_QUANT,
        non_intra_quant: crate::dequantize::DEFAULT_NON_INTRA_QUANT,
    }
}

/// Encode a whole **display-order** frame list as one conformant
/// ISO/IEC 11172-2 elementary stream with the classic
/// `I (B…) P (B…) P …` group-of-pictures structure:
///
/// * `sequence_header()` (§2.4.2.3) with the
///   `constrained_parameters_flag` set automatically when the
///   §2.4.3.2 bounds hold for this geometry / rate / f_code set;
/// * one **mandatory** `group_of_pictures()` layer per GOP
///   (§2.4.2.2): each GOP header carries the §2.4.3.3 time code of
///   its first display frame, `closed_gop = 1` (the GOP structure
///   emitted here codes no B-picture across a GOP boundary — every
///   GOP starts at an I-picture whose display predecessor is the
///   previous GOP's final anchor), and `broken_link = 0`;
/// * within each GOP, anchors every `b_between + 1` display frames
///   (`anchors_per_gop` predictive periods per GOP, clamped so the
///   final anchor of each GOP — and of the sequence — is an anchor,
///   §2.4.1: B-pictures cannot trail their backward reference), each
///   anchor coded before the B-pictures that precede it in display
///   order, and `temporal_reference` restarting from `0` at each
///   GOP's first display frame (§2.4.3.4);
/// * D-pictures are never emitted (§2.4.3.4 confines them to D-only
///   sequences).
///
/// `b_between == 0` degenerates to `I P P …` GOPs;
/// `frames.len() == 1` to a single all-intra GOP.
///
/// As with the MPEG-2 assemblers, every anchor the encoder predicts
/// from is the decoder's exact reconstruction, so the round-trip is
/// faithful end-to-end.
///
/// # Errors
/// [`Error::InvalidBitstream`] for an empty `frames`, geometry
/// mismatches, `anchors_per_gop == 0`, or out-of-range
/// `quantizer_scale_code` / f_codes; propagates encode errors.
pub fn encode_mpeg1_display_order_sequence(
    frames: &[FrameBuffer],
    b_between: usize,
    anchors_per_gop: usize,
    seq: &Mpeg1SequenceParams,
    quantizer_scale_code: u8,
    forward_f_code: u8,
    backward_f_code: u8,
) -> Result<Vec<u8>> {
    if frames.is_empty() {
        return Err(Error::InvalidBitstream(
            "encode_mpeg1_display_order_sequence: no frames to encode",
        ));
    }
    if anchors_per_gop == 0 {
        return Err(Error::InvalidBitstream(
            "encode_mpeg1_display_order_sequence: anchors_per_gop must be >= 1",
        ));
    }
    check_quantizer_scale(quantizer_scale_code)?;
    check_f_code(forward_f_code)?;
    check_f_code(backward_f_code)?;
    let params = picture_params(seq);
    for f in frames {
        if f.width != params.width || f.height != params.height {
            return Err(Error::InvalidBitstream(
                "encode_mpeg1_display_order_sequence: frame geometry does not match the sequence",
            ));
        }
    }

    // §2.4.3.2: set the constrained_parameters_flag exactly when the
    // bounds hold for the f_codes this sequence will actually code
    // (backward only participates when B-pictures exist).
    let uses_b = b_between > 0 && frames.len() > 1;
    let seq_effective = Mpeg1SequenceParams {
        constrained_parameters_flag: constrained_parameters_admissible(
            seq,
            forward_f_code,
            if uses_b { backward_f_code } else { 1 },
        ),
        ..*seq
    };

    let mut bw = BitWriter::new();
    write_mpeg1_sequence_header(&mut bw, &seq_effective);

    // GOP partition: each GOP spans display indices [gop_start,
    // gop_end] where gop_end is the GOP's final anchor (clamped to
    // the sequence tail); the next GOP starts right after.
    let step = b_between + 1;
    let mut gop_start = 0usize;
    while gop_start < frames.len() {
        let gop_end = (gop_start + anchors_per_gop * step).min(frames.len() - 1);

        // §2.4.2.4 / §2.4.3.3: the GOP header's time code refers to
        // the GOP's first display frame; no B-picture in this GOP
        // references the previous GOP, so closed_gop = 1.
        write_gop_header(
            &mut bw,
            &Mpeg2Gop {
                time_code: TimeCode::from_display_index(
                    gop_start as u64,
                    seq_effective.picture_rate_code,
                )?,
                closed_gop: true,
                broken_link: false,
            },
        );

        // The GOP's I-picture (temporal_reference = 0).
        let mut forward_ref = encode_mpeg1_intra_picture(
            &mut bw,
            &frames[gop_start],
            &params,
            0,
            quantizer_scale_code,
        )?;

        // Anchor periods: P coded before the Bs that precede it in
        // display order (§2.4.1 coded order).
        let mut prev_anchor = gop_start;
        while prev_anchor < gop_end {
            let next_anchor = (prev_anchor + step).min(gop_end);
            let backward_ref = encode_mpeg1_p_picture(
                &mut bw,
                &frames[next_anchor],
                &forward_ref,
                &params,
                (next_anchor - gop_start) as u16,
                quantizer_scale_code,
                forward_f_code,
            )?;
            for b in (prev_anchor + 1)..next_anchor {
                encode_mpeg1_b_picture(
                    &mut bw,
                    &frames[b],
                    &forward_ref,
                    &backward_ref,
                    &params,
                    (b - gop_start) as u16,
                    quantizer_scale_code,
                    forward_f_code,
                    backward_f_code,
                )?;
            }
            forward_ref = backward_ref;
            prev_anchor = next_anchor;
        }

        gop_start = gop_end + 1;
    }

    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
    Ok(stream)
}

/// Encode a single all-intra MPEG-1 stream (sequence header, one GOP,
/// one I-picture, sequence end code) — the `frames.len() == 1`
/// degenerate of [`encode_mpeg1_display_order_sequence`].
pub fn encode_mpeg1_intra_stream(
    frame: &FrameBuffer,
    seq: &Mpeg1SequenceParams,
    quantizer_scale_code: u8,
) -> Result<Vec<u8>> {
    encode_mpeg1_display_order_sequence(
        std::slice::from_ref(frame),
        0,
        1,
        seq,
        quantizer_scale_code,
        1,
        1,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat_frame(width: usize, height: usize, luma: u8) -> FrameBuffer {
        let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
        for y in 0..height {
            for x in 0..width {
                f.y.put_sample(x, y, luma);
            }
        }
        for y in 0..f.cb.height() {
            for x in 0..f.cb.width() {
                f.cb.put_sample(x, y, 128);
                f.cr.put_sample(x, y, 128);
            }
        }
        f
    }

    fn seq(width: u16, height: u16) -> Mpeg1SequenceParams {
        Mpeg1SequenceParams {
            horizontal_size: width,
            vertical_size: height,
            ..Default::default()
        }
    }

    #[test]
    fn intra_stream_layout_is_mpeg1() {
        let frame = flat_frame(16, 16, 128);
        let stream = encode_mpeg1_intra_stream(&frame, &seq(16, 16), 8).expect("encode");
        // sequence_header_code at the head…
        assert_eq!(&stream[0..4], &[0x00, 0x00, 0x01, 0xB3]);
        // …GOP start code right after the 12-byte header (no
        // extension start code anywhere before it)…
        assert_eq!(&stream[12..16], &[0x00, 0x00, 0x01, 0xB8]);
        // …and the sequence_end_code at the tail.
        assert_eq!(&stream[stream.len() - 4..], &[0x00, 0x00, 0x01, 0xB7]);
        // No 0xB5 extension start code may appear (MPEG-1-legal
        // extension absence).
        assert!(
            !stream
                .windows(4)
                .any(|w| w[0] == 0 && w[1] == 0 && w[2] == 1 && w[3] == 0xB5),
            "extension start code in an MPEG-1 stream"
        );
    }

    #[test]
    fn flat_grey_frame_roundtrips_exactly() {
        // Flat 128 everywhere: DC differential 0 against the slice
        // reset, all AC zero → decodes back to exactly 128.
        let frame = flat_frame(16, 16, 128);
        let stream = encode_mpeg1_intra_stream(&frame, &seq(16, 16), 8).expect("encode");
        let frames = crate::decode_video_sequence(&stream).expect("decode");
        assert_eq!(frames.len(), 1);
        for y in 0..16 {
            for x in 0..16 {
                assert_eq!(frames[0].frame.y.get(x, y), Some(128), "luma ({x},{y})");
            }
        }
    }

    #[test]
    fn intra_reconstruction_matches_decoder_exactly() {
        // Structured content: the returned reconstruction must equal
        // the decode of the emitted stream sample-for-sample.
        let mut frame = flat_frame(48, 32, 0);
        for y in 0..32 {
            for x in 0..48 {
                frame.y.put_sample(x, y, (16 + (x * 5 + y * 3) % 200) as u8);
            }
        }
        for y in 0..16 {
            for x in 0..24 {
                frame.cb.put_sample(x, y, (100 + (x + y) % 56) as u8);
                frame.cr.put_sample(x, y, (90 + (x * 2 + y) % 80) as u8);
            }
        }
        let mut bw = BitWriter::new();
        write_mpeg1_sequence_header(&mut bw, &seq(48, 32));
        write_gop_header(
            &mut bw,
            &Mpeg2Gop {
                time_code: TimeCode::from_display_index(0, 3).unwrap(),
                closed_gop: true,
                broken_link: false,
            },
        );
        let params = picture_params(&seq(48, 32));
        let recon = encode_mpeg1_intra_picture(&mut bw, &frame, &params, 0, 6).expect("encode");
        let mut stream = bw.finish();
        stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());

        let frames = crate::decode_video_sequence(&stream).expect("decode");
        assert_eq!(frames.len(), 1);
        let dec = &frames[0].frame;
        for y in 0..32 {
            for x in 0..48 {
                assert_eq!(dec.y.get(x, y), recon.y.get(x, y), "luma ({x},{y})");
            }
        }
        for y in 0..16 {
            for x in 0..24 {
                assert_eq!(dec.cb.get(x, y), recon.cb.get(x, y), "cb ({x},{y})");
                assert_eq!(dec.cr.get(x, y), recon.cr.get(x, y), "cr ({x},{y})");
            }
        }
    }

    #[test]
    fn empty_frame_list_is_rejected() {
        let err = encode_mpeg1_display_order_sequence(&[], 0, 1, &seq(16, 16), 8, 1, 1)
            .expect_err("empty frames");
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn zero_anchor_periods_rejected() {
        let frame = flat_frame(16, 16, 128);
        let err = encode_mpeg1_display_order_sequence(&[frame], 0, 0, &seq(16, 16), 8, 1, 1)
            .expect_err("zero anchors_per_gop");
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn out_of_range_quantizer_and_f_codes_rejected() {
        let frame = flat_frame(16, 16, 128);
        for (q, ff, bf) in [(0u8, 1u8, 1u8), (32, 1, 1), (8, 0, 1), (8, 8, 1), (8, 1, 8)] {
            let err = encode_mpeg1_display_order_sequence(
                std::slice::from_ref(&frame),
                1,
                1,
                &seq(16, 16),
                q,
                ff,
                bf,
            )
            .expect_err("range violation");
            assert!(matches!(err, Error::InvalidBitstream(_)), "{q} {ff} {bf}");
        }
    }
}
