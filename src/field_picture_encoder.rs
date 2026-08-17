//! Field-picture encoders — the MPEG-2 **inter encode path for field
//! pictures** (§7.6.1: *"within a field picture all predictions are
//! field predictions"*), plus the intra field encoder and the
//! display-order assembler that emits whole field-coded sequences.
//!
//! ## Syntax emitted (§6.2 / §6.3)
//!
//! Each coded frame is sent as a §6.1.1.4.1 **field-picture pair** (top
//! field first), both fields sharing the frame's `temporal_reference`
//! (§6.3.10). Every field picture carries a
//! `picture_coding_extension()` with `picture_structure` = Top / Bottom
//! (Table 6-14) and the §6.3.10 field-picture constants
//! ([`crate::stream_writer::write_field_picture_coding_extension`]).
//! A motion-compensated macroblock writes, after its Table B-3 / B-4
//! `macroblock_type`:
//!
//! * `field_motion_type = 01` (Table 6-18 `Field-based`:
//!   `motion_vector_count = 1`, `mv_format = field`, `dmv = 0`) —
//!   §6.2.5.1 selects `field_motion_type` because
//!   `picture_structure != "Frame picture"`; no `dct_type` follows (a
//!   field picture has no frame/field DCT distinction, Table 6-19);
//! * `motion_vertical_field_select[r][s]` (§6.2.5.2 — present because
//!   `mv_format == field && dmv == 0`): `0` reads the reference
//!   frame's **top** field, `1` its **bottom** field (§7.6.4);
//! * the `motion_vector(r, s)` differentials against the §7.6.3.4 PMV.
//!   In a field picture the vertical component is coded **directly in
//!   field-sample units** (the §7.6.3.1 halving/doubling of
//!   `PMV[r][s][1]` applies only to field predictions *in frame
//!   pictures*), and the §7.6.3.3 Table 7-9/7-10 update rule copies
//!   the coded vector into both PMV slots.
//!
//! ## Prediction / reconstruction
//!
//! The motion search scores each `(reference field parity, vector)`
//! candidate by the SAD of the **exact §7.6.4 prediction** the decoder
//! forms ([`crate::forming_predictions::predict_field_block`]), visiting
//! only vectors whose whole read span stays inside the reference field
//! (§7.6.3.8). Each macroblock is reconstructed exactly as
//! [`crate::decode_field_picture`] will reconstruct it — the shared
//! [`crate::inter_reconstruction::predict_field_picture_macroblock_planes`]
//! plus residual add — so the encoder carries decoder-exact reference
//! fields forward.
//!
//! ## Reference rules (§7.6.2.1)
//!
//! * First field of a P frame / any B field: predictions read the
//!   previous (and for B, next) reconstructed reference **frame**,
//!   either parity.
//! * **Second field of a P frame**: the two most recent reference
//!   fields are the *just-coded first field of the same frame* and the
//!   previous frame's opposite-parity field — the encoder builds the
//!   same synthetic reference frame the decode loop builds
//!   ([`crate::video_sequence`]'s §7.6.2.1 pairing), so parity
//!   selection addresses exactly the fields the standard names.

#![allow(clippy::too_many_arguments)]
// The Table B-3 `00011` Intra codeword is grouped to mirror the spec's
// MSB-first table layout (`0001_1`) so an audit reads it against the
// printed table; clippy's equal-size grouping would obscure that.
#![allow(clippy::unusual_byte_groupings)]
// The macroblock walk indexes prediction planes / block arrays by the
// same loop variables the spec's pseudocode uses.
#![allow(clippy::needless_range_loop)]

use oxideav_core::bits::BitWriter;

use crate::b_picture_encoder::{write_b_macroblock_type, BDirection};
use crate::coded_block_pattern::encode_cbp420;
use crate::dual_prime::FieldParity;
use crate::forming_predictions::{predict_field_block, BlockSize, FieldReference, ReferencePlane};
use crate::frame_assembly::{block_placement, FrameBuffer, IntraPictureParams};
use crate::frame_field_encoder::PmvMirror;
use crate::gop_header::{write_gop_header, Mpeg2Gop, TimeCode};
use crate::inter_reconstruction::{
    predict_field_picture_macroblock_planes, FieldPictureDualPrimeMotion, FieldPictureMotion,
    MotionVectorPel, ReferenceFrames,
};
use crate::motion_estimation::max_search_range;
use crate::motion_vector::encode_motion_component;
use crate::mpeg2_block_dc::ColourComponent;
use crate::mpeg2_dequantize::{intra_dc_mult, quantiser_scale};
use crate::mpeg2_macroblock_blocks::{block_component, block_count};
use crate::p_picture_encoder::{
    encode_intra_mb, gather_residual, intra_activity, quantise_inter_block, reconstruct_inter_mb,
    wrap_delta, write_inter_block_coeffs, IntraDcPred,
};
use crate::picture_header::{PictureCodingType, PictureStructure};
use crate::sequence_extension::ChromaFormat;
use crate::stream_writer::{
    write_field_picture_coding_extension, write_picture_header, write_sequence_extension,
    write_sequence_header, write_slice_header, PictureCodingExtensionParams, SequenceHeaderParams,
    SEQUENCE_END_CODE,
};
use crate::video_sequence::{extract_field, reference_frame_for_second_p_field};
use crate::{Error, Result};

/// A macroblock's chroma extent in samples for the 4:2:0 field layout
/// (§7.6.7.2: the full 8×8 chroma block per field-picture macroblock).
const CHROMA_MB: usize = 8;

/// The result of a per-direction field motion search: the winning
/// vector, its reference-field parity, and the luma SAD of the exact
/// §7.6.4 prediction it forms.
#[derive(Debug, Clone, Copy)]
pub struct FieldSearchResult {
    /// The chosen luminance vector in half-sample field units.
    pub vector: MotionVectorPel,
    /// The reference field the §6.3.17.2 `motion_vertical_field_select`
    /// flag will name (`Top` → `0`, `Bottom` → `1`).
    pub parity: FieldParity,
    /// Luma SAD of the prediction against the current field macroblock.
    pub sad: u32,
}

/// SAD between the current field's macroblock and the §7.6.4 field
/// prediction `(parity, (hx, hy))` forms from `reference`'s chosen
/// field. Edge reads of the current field clamp (the macroblock grid
/// pads to 16).
fn field_luma_sad(
    current: &FrameBuffer,
    reference: &FrameBuffer,
    parity: FieldParity,
    mb_col: usize,
    mb_row: usize,
    hx: i32,
    hy: i32,
) -> u32 {
    let plane = ReferencePlane::new(
        reference.y.samples(),
        reference.y.width(),
        reference.y.height(),
    )
    .expect("reference luma plane is width*height");
    let Some(field) = FieldReference::new(plane, parity.index()) else {
        return u32::MAX;
    };
    let size = BlockSize::new(16, 16).expect("16x16 is a valid block size");
    let pred = predict_field_block(
        field,
        (mb_col * 16) as i32,
        (mb_row * 16) as i32,
        size,
        hx,
        hy,
    );
    let w = current.y.width();
    let h = current.y.height();
    let base_x = mb_col * 16;
    let base_y = mb_row * 16;
    let mut sad = 0u32;
    for r in 0..16 {
        let sy = (base_y + r).min(h.saturating_sub(1));
        for c in 0..16 {
            let sx = (base_x + c).min(w.saturating_sub(1));
            let cur = i32::from(current.y.get(sx, sy).unwrap_or(0));
            let prd = i32::from(pred[r * 16 + c]);
            sad += (cur - prd).unsigned_abs();
        }
    }
    sad
}

/// Estimate the field motion vector for the macroblock at
/// `(mb_col, mb_row)` of the current **field** against a reconstructed
/// reference **frame**, searching **both reference field parities**
/// with an integer-pel full search plus half-pel refinement per parity,
/// and returning the overall winner.
///
/// §7.6.3.8: only vectors whose whole §7.6.4 read span stays inside
/// the chosen reference field are visited (the field view is
/// `width × frame_height/2`).
pub fn estimate_field_mv(
    current: &FrameBuffer,
    reference: &FrameBuffer,
    mb_col: usize,
    mb_row: usize,
    search_range: i32,
) -> FieldSearchResult {
    let fw = reference.y.width() as i32;
    // Field-view height of the reference luma plane (both parities;
    // plane heights are even multiples of the field macroblock grid).
    let legal_for = |fh: i32| {
        move |hx: i32, hy: i32| -> bool {
            let base_x = (mb_col * 16) as i32;
            let base_y = (mb_row * 16) as i32;
            let ix = hx.div_euclid(2);
            let iy = hy.div_euclid(2);
            let ex = i32::from(hx.rem_euclid(2) != 0);
            let ey = i32::from(hy.rem_euclid(2) != 0);
            base_x + ix >= 0
                && base_y + iy >= 0
                && base_x + ix + 15 + ex < fw
                && base_y + iy + 15 + ey < fh
        }
    };
    // Tie-break toward cheaper vectors (shorter §6.2.5.2.1 codes).
    let vec_cost = |hx: i32, hy: i32| -> u32 { hx.unsigned_abs() + hy.unsigned_abs() };

    let mut best: Option<FieldSearchResult> = None;
    let mut best_score = u32::MAX;
    for parity in [FieldParity::Top, FieldParity::Bottom] {
        let fh = ((reference.y.height() - parity.index()).div_ceil(2)) as i32;
        let legal = legal_for(fh);
        // Integer-pel full search, zero-vector incumbent.
        let mut p_best_hx = 0i32;
        let mut p_best_hy = 0i32;
        let mut p_best_sad = field_luma_sad(current, reference, parity, mb_col, mb_row, 0, 0);
        let mut p_best_score = p_best_sad.saturating_add(vec_cost(0, 0));
        for dy in -search_range..=search_range {
            for dx in -search_range..=search_range {
                if dx == 0 && dy == 0 {
                    continue;
                }
                let hx = dx * 2;
                let hy = dy * 2;
                if !legal(hx, hy) {
                    continue;
                }
                let sad = field_luma_sad(current, reference, parity, mb_col, mb_row, hx, hy);
                let score = sad.saturating_add(vec_cost(hx, hy));
                if score < p_best_score {
                    p_best_score = score;
                    p_best_sad = sad;
                    p_best_hx = hx;
                    p_best_hy = hy;
                }
            }
        }
        // Half-pel refinement around the integer winner.
        let (int_hx, int_hy) = (p_best_hx, p_best_hy);
        for &(ox, oy) in &[
            (-1i32, -1i32),
            (0, -1),
            (1, -1),
            (-1, 0),
            (1, 0),
            (-1, 1),
            (0, 1),
            (1, 1),
        ] {
            let hx = int_hx + ox;
            let hy = int_hy + oy;
            if !legal(hx, hy) {
                continue;
            }
            let sad = field_luma_sad(current, reference, parity, mb_col, mb_row, hx, hy);
            let score = sad.saturating_add(vec_cost(hx, hy));
            if score < p_best_score {
                p_best_score = score;
                p_best_sad = sad;
                p_best_hx = hx;
                p_best_hy = hy;
            }
        }
        if p_best_score < best_score {
            best_score = p_best_score;
            best = Some(FieldSearchResult {
                vector: MotionVectorPel::new(p_best_hx, p_best_hy),
                parity,
                sad: p_best_sad,
            });
        }
    }
    best.expect("both parities searched")
}

/// Validate the field geometry / format constraints shared by the
/// field-picture encoders.
fn check_field_params(params: &IntraPictureParams) -> Result<()> {
    if params.chroma_format != ChromaFormat::Yuv420 {
        return Err(Error::InvalidBitstream(
            "field picture encoder: only 4:2:0 is supported",
        ));
    }
    if params.frame_pred_frame_dct {
        return Err(Error::InvalidBitstream(
            "field picture encoder: frame_pred_frame_dct applies to frame pictures only (§6.3.10)",
        ));
    }
    if params.alternate_scan || params.intra_vlc_format {
        // The field block writers code Table B-14 / zigzag scan; the
        // flags would otherwise be declared in the picture coding
        // extension but not honoured on the wire.
        return Err(Error::InvalidBitstream(
            "field picture encoder: alternate_scan / intra_vlc_format are not supported",
        ));
    }
    Ok(())
}

/// Write the picture header + field `picture_coding_extension()` for a
/// field picture of `coding_type` at `structure`.
fn write_field_picture_headers(
    bw: &mut BitWriter,
    params: &IntraPictureParams,
    structure: PictureStructure,
    coding_type: PictureCodingType,
    temporal_reference: u16,
    forward_f_code: u8,
    backward_f_code: u8,
) {
    // §6.3.10: the MPEG-1 legacy f_code fields in the picture header
    // are unused ('111') in an ISO/IEC 13818-2 stream.
    write_picture_header(bw, temporal_reference, coding_type, 0b111, 0b111);
    write_field_picture_coding_extension(
        bw,
        &PictureCodingExtensionParams {
            forward_f_code,
            backward_f_code,
            intra_dc_precision: params.intra_dc_precision,
            q_scale_type: params.q_scale_type,
            intra_vlc_format: params.intra_vlc_format,
            alternate_scan: params.alternate_scan,
            // frame_pred_frame_dct / progressive_frame are forced by
            // the field writer regardless of these fields.
            frame_pred_frame_dct: false,
            progressive_frame: false,
        },
        structure,
    );
}

/// Encode one **intra field picture** from `field` (a field-height
/// buffer), appending the picture layer to `bw` and returning the
/// decoder-exact reconstruction of the field.
///
/// `params` carries the **field** geometry (`height` = field height;
/// `frame_pred_frame_dct = false`). Macroblocks are all intra
/// (Table B-2 `1`); per §6.2.5.1 a field picture's intra macroblock
/// carries no `dct_type` bit.
///
/// # Errors
/// [`Error::InvalidBitstream`] for geometry / format mismatches.
pub fn encode_field_intra_picture(
    bw: &mut BitWriter,
    field: &FrameBuffer,
    params: &IntraPictureParams,
    structure: PictureStructure,
    temporal_reference: u16,
    quantiser_scale_code: u8,
) -> Result<FrameBuffer> {
    check_field_params(params)?;
    if field.width != params.width || field.height != params.height {
        return Err(Error::InvalidBitstream(
            "encode_field_intra_picture: field dimensions do not match params",
        ));
    }
    let qscale = quantiser_scale(quantiser_scale_code, params.q_scale_type)?;
    let dc_mult = intra_dc_mult(params.intra_dc_precision)?;

    write_field_picture_headers(
        bw,
        params,
        structure,
        PictureCodingType::Intra,
        temporal_reference,
        15,
        15,
    );

    let mut recon = params.new_frame_buffer();
    let mb_width = params.mb_width();
    let mb_height = params.mb_height();
    let nblocks = block_count(params.chroma_format);
    for mb_row in 0..mb_height {
        write_slice_header(bw, mb_row as u32, quantiser_scale_code);
        // §7.2.1: the DC predictors reset at slice start.
        let mut pred = IntraDcPred::reset(params.intra_dc_precision);
        for mb_col in 0..mb_width {
            // macroblock_address_increment = 1 (Table B-1 `1`);
            // macroblock_type = Intra (Table B-2 `1`). No dct_type in a
            // field picture (§6.2.5.1).
            bw.write_bit(true);
            bw.write_bit(true);
            encode_intra_mb(
                bw,
                field,
                &mut recon,
                mb_col,
                mb_row,
                qscale,
                dc_mult,
                &mut pred,
                nblocks,
                params.chroma_format,
                crate::mpeg2_dct_coeff::TableSelection::TableZero,
                false,
            );
        }
        bw.align_to_byte_zero();
    }
    Ok(recon)
}

/// Encode one **P field picture** from `current` (a field-height
/// buffer), predicting from the reconstructed `reference` **frame**
/// (its two fields addressed by each macroblock's
/// `motion_vertical_field_select`), appending the picture layer to `bw`.
///
/// For the **second field of a P frame** pass the §7.6.2.1 synthetic
/// reference ([`second_p_field_reference`]) so the parity selection
/// addresses the just-coded first field and the previous frame's
/// opposite-parity field.
///
/// Per macroblock: a both-parity field motion search
/// ([`estimate_field_mv`]), Table B-3 `MC, Coded` / `MC, Not Coded`
/// modes with `field_motion_type = 01` and the
/// `motion_vertical_field_select` flag, an intra fallback (Table B-3
/// `00011`) for content the prediction cannot capture, and the
/// decoder-exact reconstruction into the returned field buffer.
///
/// # Errors
/// [`Error::InvalidBitstream`] for geometry / format mismatches;
/// propagates prediction errors.
pub fn encode_field_p_picture(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    reference: &FrameBuffer,
    params: &IntraPictureParams,
    structure: PictureStructure,
    temporal_reference: u16,
    quantiser_scale_code: u8,
    forward_f_code: u8,
) -> Result<FrameBuffer> {
    check_field_params(params)?;
    if current.width != params.width || current.height != params.height {
        return Err(Error::InvalidBitstream(
            "encode_field_p_picture: field dimensions do not match params",
        ));
    }
    if reference.height != params.height * 2 || reference.width != params.width {
        return Err(Error::InvalidBitstream(
            "encode_field_p_picture: reference frame must be twice the field height (§6.1.1.4.1)",
        ));
    }
    let qscale = quantiser_scale(quantiser_scale_code, params.q_scale_type)?;
    let dc_mult = intra_dc_mult(params.intra_dc_precision)?;
    let search_range = max_search_range(forward_f_code).min(16);

    write_field_picture_headers(
        bw,
        params,
        structure,
        PictureCodingType::Predictive,
        temporal_reference,
        forward_f_code,
        forward_f_code,
    );

    let mut recon = params.new_frame_buffer();
    let mb_width = params.mb_width();
    let mb_height = params.mb_height();
    let nblocks = block_count(params.chroma_format);

    for mb_row in 0..mb_height {
        write_slice_header(bw, mb_row as u32, quantiser_scale_code);
        // §7.6.3.4: PMV resets at slice start. In a field picture the
        // §7.6.3.3 Table 7-9 update copies the coded vector into both
        // PMV slots, so one running pair mirrors the decoder exactly.
        let mut pmv = (0i32, 0i32);
        let mut intra_pred = IntraDcPred::reset(params.intra_dc_precision);
        let slice_first_addr = (mb_row * mb_width) as i32;
        let mut past_intra_address = slice_first_addr - 2;

        for mb_col in 0..mb_width {
            let mb_address = slice_first_addr + mb_col as i32;
            // 1. Both-parity field motion search.
            let search = estimate_field_mv(current, reference, mb_col, mb_row, search_range);

            // Intra fallback for content prediction can't capture.
            let intra_cost = intra_activity(current, mb_col, mb_row);
            if search.sad > intra_cost.saturating_mul(2).saturating_add(512) {
                if mb_address - past_intra_address > 1 {
                    intra_pred = IntraDcPred::reset(params.intra_dc_precision);
                }
                // macroblock_address_increment = 1; Intra (Table B-3
                // `00011`). No dct_type in a field picture.
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
                    params.chroma_format,
                    crate::mpeg2_dct_coeff::TableSelection::TableZero,
                    false,
                );
                // §7.6.3.4: an intra macroblock resets the predictors.
                pmv = (0, 0);
                past_intra_address = mb_address;
                continue;
            }

            let motion = FieldPictureMotion::forward(search.vector, search.parity);
            // 2. The exact §7.6.4 field prediction planes.
            let (luma_pred, cb_pred, cr_pred) = predict_field_picture_macroblock_planes(
                &recon,
                ReferenceFrames::forward_only(reference),
                mb_col,
                mb_row,
                motion,
            )
            .map_err(crate::Error::from)?;

            // 3. Residual + forward quantise per block.
            let mut blocks = Vec::with_capacity(nblocks);
            let mut cbp = 0u8;
            for i in 0..nblocks {
                let placement = block_placement(i, params.chroma_format, mb_col, mb_row, false)
                    .expect("valid block index");
                let component = block_component(i, params.chroma_format).expect("valid component");
                let (cur_plane, pred, pred_w, mb_origin_x, mb_origin_y) = match component {
                    ColourComponent::Y => {
                        (&current.y, &luma_pred, 16usize, mb_col * 16, mb_row * 16)
                    }
                    ColourComponent::Cb => (
                        &current.cb,
                        &cb_pred,
                        CHROMA_MB,
                        mb_col * CHROMA_MB,
                        mb_row * CHROMA_MB,
                    ),
                    ColourComponent::Cr => (
                        &current.cr,
                        &cr_pred,
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
                    placement.base_x - mb_origin_x,
                    placement.base_y - mb_origin_y,
                );
                let block = quantise_inter_block(&residual, qscale);
                if block.is_coded() {
                    cbp |= 1 << (5 - i);
                }
                blocks.push(block);
            }

            // 4. Macroblock layer: address increment, Table B-3 mode,
            // field_motion_type, field select + MV, cbp + blocks.
            bw.write_bit(true);
            if cbp != 0 {
                bw.write_bit(true); // "MC, Coded" (Table B-3 `1`)
            } else {
                bw.write_u32(0b001, 3); // "MC, Not Coded"
            }
            // field_motion_type = 01 (Table 6-18 Field-based).
            bw.write_u32(0b01, 2);
            // motion_vertical_field_select[0][0] (§6.2.5.2): 0 = top.
            bw.write_bit(search.parity == FieldParity::Bottom);
            let dx = wrap_delta(search.vector.horizontal - pmv.0, forward_f_code)?;
            let dy = wrap_delta(search.vector.vertical - pmv.1, forward_f_code)?;
            encode_motion_component(bw, dx, forward_f_code);
            encode_motion_component(bw, dy, forward_f_code);
            pmv = (search.vector.horizontal, search.vector.vertical);

            if cbp != 0 {
                encode_cbp420(bw, cbp);
                for i in 0..nblocks {
                    if let Some(qf) = blocks[i].qf_ref() {
                        write_inter_block_coeffs(bw, qf, false);
                    }
                }
            }

            // 5. Decoder-exact reconstruction into the field buffer.
            reconstruct_inter_mb(
                &mut recon, mb_col, mb_row, &luma_pred, &cb_pred, &cr_pred, &blocks,
            );
        }
        bw.align_to_byte_zero();
    }
    Ok(recon)
}

/// Encode one **B field picture** from `current`, predicting from the
/// reconstructed `forward` (past) and `backward` (future) anchor
/// **frames**, appending the picture layer to `bw` and returning the
/// decoder-exact reconstruction of the field.
///
/// Per macroblock the encoder searches both directions over both
/// reference field parities, scores forward / backward / interpolated
/// (§7.6.7.2 `// 2` average) predictions by luma SAD, emits the Table
/// B-4 mode + `field_motion_type = 01` + per-direction
/// `motion_vertical_field_select` and motion vectors (forward before
/// backward), and reconstructs exactly as the decoder will.
///
/// # Errors
/// [`Error::InvalidBitstream`] for geometry / format mismatches;
/// propagates prediction errors.
pub fn encode_field_b_picture(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    forward: &FrameBuffer,
    backward: &FrameBuffer,
    params: &IntraPictureParams,
    structure: PictureStructure,
    temporal_reference: u16,
    quantiser_scale_code: u8,
    forward_f_code: u8,
    backward_f_code: u8,
) -> Result<FrameBuffer> {
    check_field_params(params)?;
    if current.width != params.width || current.height != params.height {
        return Err(Error::InvalidBitstream(
            "encode_field_b_picture: field dimensions do not match params",
        ));
    }
    for anchor in [forward, backward] {
        if anchor.height != params.height * 2 || anchor.width != params.width {
            return Err(Error::InvalidBitstream(
                "encode_field_b_picture: anchor frame must be twice the field height (§6.1.1.4.1)",
            ));
        }
    }
    let qscale = quantiser_scale(quantiser_scale_code, params.q_scale_type)?;
    let fwd_range = max_search_range(forward_f_code).min(16);
    let bwd_range = max_search_range(backward_f_code).min(16);

    write_field_picture_headers(
        bw,
        params,
        structure,
        PictureCodingType::Bidirectional,
        temporal_reference,
        forward_f_code,
        backward_f_code,
    );

    let mut recon = params.new_frame_buffer();
    let mb_width = params.mb_width();
    let mb_height = params.mb_height();
    let nblocks = block_count(params.chroma_format);

    for mb_row in 0..mb_height {
        write_slice_header(bw, mb_row as u32, quantiser_scale_code);
        // §7.6.3.4: per-direction PMV slots reset at slice start.
        let mut pmv_fwd = (0i32, 0i32);
        let mut pmv_bwd = (0i32, 0i32);

        for mb_col in 0..mb_width {
            // 1. Per-direction (parity, vector) searches.
            let fs = estimate_field_mv(current, forward, mb_col, mb_row, fwd_range);
            let bs = estimate_field_mv(current, backward, mb_col, mb_row, bwd_range);

            // 2. Score the three candidate predictions with the exact
            // §7.6.4 / §7.6.7.2 arithmetic.
            let pred_fwd = predict_field_picture_macroblock_planes(
                &recon,
                ReferenceFrames::forward_only(forward),
                mb_col,
                mb_row,
                FieldPictureMotion::forward(fs.vector, fs.parity),
            )
            .map_err(crate::Error::from)?;
            let pred_bwd = predict_field_picture_macroblock_planes(
                &recon,
                ReferenceFrames {
                    forward: None,
                    backward: Some(backward),
                },
                mb_col,
                mb_row,
                FieldPictureMotion::backward(bs.vector, bs.parity),
            )
            .map_err(crate::Error::from)?;
            let pred_int = predict_field_picture_macroblock_planes(
                &recon,
                ReferenceFrames::bidirectional(forward, backward),
                mb_col,
                mb_row,
                FieldPictureMotion::bidirectional(fs.vector, fs.parity, bs.vector, bs.parity),
            )
            .map_err(crate::Error::from)?;

            let sad_of = |pred: &(Vec<u8>, Vec<u8>, Vec<u8>)| -> u32 {
                let w = current.y.width();
                let h = current.y.height();
                let base_x = mb_col * 16;
                let base_y = mb_row * 16;
                let mut sad = 0u32;
                for r in 0..16 {
                    let sy = (base_y + r).min(h.saturating_sub(1));
                    for c in 0..16 {
                        let sx = (base_x + c).min(w.saturating_sub(1));
                        let cur = i32::from(current.y.get(sx, sy).unwrap_or(0));
                        let prd = i32::from(pred.0[r * 16 + c]);
                        sad += (cur - prd).unsigned_abs();
                    }
                }
                sad
            };
            let sad_fwd = sad_of(&pred_fwd);
            let sad_bwd = sad_of(&pred_bwd);
            let sad_int = sad_of(&pred_int);

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

            // 3. Residual + forward quantise per block.
            let mut blocks = Vec::with_capacity(nblocks);
            let mut cbp = 0u8;
            for i in 0..nblocks {
                let placement = block_placement(i, params.chroma_format, mb_col, mb_row, false)
                    .expect("valid block index");
                let component = block_component(i, params.chroma_format).expect("valid component");
                let (cur_plane, pred, pred_w, mb_origin_x, mb_origin_y) = match component {
                    ColourComponent::Y => {
                        (&current.y, luma_pred, 16usize, mb_col * 16, mb_row * 16)
                    }
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
                    placement.base_x - mb_origin_x,
                    placement.base_y - mb_origin_y,
                );
                let block = quantise_inter_block(&residual, qscale);
                if block.is_coded() {
                    cbp |= 1 << (5 - i);
                }
                blocks.push(block);
            }

            // 4. Macroblock layer.
            bw.write_bit(true); // macroblock_address_increment = 1
            let coded = cbp != 0;
            write_b_macroblock_type(bw, dir, coded);
            // field_motion_type = 01 (Table 6-18 Field-based).
            bw.write_u32(0b01, 2);
            // Forward motion_vectors(0) precede backward
            // motion_vectors(1); each vector carries its own §6.2.5.2
            // motion_vertical_field_select flag.
            if matches!(dir, BDirection::Forward | BDirection::Interpolated) {
                bw.write_bit(fs.parity == FieldParity::Bottom);
                let dx = wrap_delta(fs.vector.horizontal - pmv_fwd.0, forward_f_code)?;
                let dy = wrap_delta(fs.vector.vertical - pmv_fwd.1, forward_f_code)?;
                encode_motion_component(bw, dx, forward_f_code);
                encode_motion_component(bw, dy, forward_f_code);
                pmv_fwd = (fs.vector.horizontal, fs.vector.vertical);
            }
            if matches!(dir, BDirection::Backward | BDirection::Interpolated) {
                bw.write_bit(bs.parity == FieldParity::Bottom);
                let dx = wrap_delta(bs.vector.horizontal - pmv_bwd.0, backward_f_code)?;
                let dy = wrap_delta(bs.vector.vertical - pmv_bwd.1, backward_f_code)?;
                encode_motion_component(bw, dx, backward_f_code);
                encode_motion_component(bw, dy, backward_f_code);
                pmv_bwd = (bs.vector.horizontal, bs.vector.vertical);
            }

            if coded {
                encode_cbp420(bw, cbp);
                for i in 0..nblocks {
                    if let Some(qf) = blocks[i].qf_ref() {
                        write_inter_block_coeffs(bw, qf, false);
                    }
                }
            }

            // 5. Decoder-exact reconstruction.
            reconstruct_inter_mb(
                &mut recon, mb_col, mb_row, luma_pred, cb_pred, cr_pred, &blocks,
            );
        }
        bw.align_to_byte_zero();
    }
    Ok(recon)
}

/// Build the §7.6.2.1 synthetic reference frame for the **second field
/// of a P frame**: the just-coded first field in its own parity slot
/// paired with the previous reference frame's opposite-parity field —
/// exactly the frame the decode loop builds, so encoder and decoder
/// address identical reference fields.
///
/// # Errors
/// Propagates field-assembly geometry errors.
pub fn second_p_field_reference(
    first_field: &FrameBuffer,
    first_structure: PictureStructure,
    previous_frame: &FrameBuffer,
) -> Result<FrameBuffer> {
    reference_frame_for_second_p_field(first_field, first_structure, previous_frame)
}

/// Derive the **field** geometry from a frame-level `params`: half the
/// height, `frame_pred_frame_dct` off (§6.3.10), and a 16-aligned
/// field macroblock grid (`progressive_sequence = true` selects the
/// plain `Ceil(h/16)` grid arithmetic in field coordinates — the
/// sequence itself is still coded interlaced).
fn field_geometry(frame_params: &IntraPictureParams) -> IntraPictureParams {
    IntraPictureParams {
        height: frame_params.height / 2,
        frame_pred_frame_dct: false,
        progressive_sequence: true,
        ..*frame_params
    }
}

/// Encode a whole **display-order** frame sequence as an interlaced
/// MPEG-2 elementary stream coded entirely as §6.1.1.4.1 **field-picture
/// pairs** (top field first), with the same GOP structure as
/// [`crate::encode_display_order_gop_sequence`]:
///
/// * each GOP opens with an I frame (both fields intra), anchors every
///   `b_between + 1` display frames, closed GOPs, per-GOP
///   `temporal_reference` reset — both fields of a frame share its
///   `temporal_reference` (§6.3.10);
/// * a P frame's first field predicts from the previous anchor frame;
///   its **second field** predicts through the §7.6.2.1 synthetic
///   reference ([`second_p_field_reference`]) whose most recent field
///   is the frame's just-coded first field;
/// * B frames' fields predict from the two anchor frames in either
///   parity, forward / backward / interpolated per macroblock;
/// * the sequence declares `progressive_sequence = 0` and every
///   picture `progressive_frame = 0` (§6.3.5 / §6.3.10).
///
/// Every anchor the encoder predicts from is the decoder's exact
/// reconstruction (fields are reconstructed with the decode-side §7.6
/// drivers and re-interleaved with the decoder's own
/// [`crate::assemble_frame_from_fields`]), so the stream decodes
/// faithfully through [`crate::decode_video_sequence`].
///
/// `frame_params` carries the **frame** geometry;
/// `frame_params.height` must be a multiple of 32 (so both field
/// macroblock grids are exact) and `progressive_sequence` must be
/// `false`.
///
/// # Errors
/// [`Error::InvalidBitstream`] for an empty `frames`, geometry /
/// format violations, or `anchors_per_gop == 0`; propagates encode
/// errors.
pub fn encode_field_display_order_gop_sequence(
    frames: &[FrameBuffer],
    b_between: usize,
    anchors_per_gop: usize,
    frame_params: &IntraPictureParams,
    quantiser_scale_code: u8,
    forward_f_code: u8,
    backward_f_code: u8,
) -> Result<Vec<u8>> {
    if frames.is_empty() {
        return Err(Error::InvalidBitstream(
            "encode_field_display_order_gop_sequence: no frames to encode",
        ));
    }
    if anchors_per_gop == 0 {
        return Err(Error::InvalidBitstream(
            "encode_field_display_order_gop_sequence: anchors_per_gop must be >= 1",
        ));
    }
    if frame_params.progressive_sequence {
        return Err(Error::InvalidBitstream(
            "encode_field_display_order_gop_sequence: field pictures require \
             progressive_sequence = 0 (§6.3.5)",
        ));
    }
    if frame_params.height % 32 != 0 {
        return Err(Error::InvalidBitstream(
            "encode_field_display_order_gop_sequence: frame height must be a multiple of 32 \
             (each field's §6.3.3 macroblock grid must be exact)",
        ));
    }
    let field_params = field_geometry(frame_params);
    check_field_params(&field_params)?;
    for f in frames {
        if f.width != frame_params.width || f.height != frame_params.height {
            return Err(Error::InvalidBitstream(
                "encode_field_display_order_gop_sequence: frame geometry mismatch",
            ));
        }
    }

    let sequence_params = SequenceHeaderParams {
        horizontal_size: frame_params.width as u16,
        vertical_size: frame_params.height as u16,
        ..Default::default()
    };
    let mut bw = BitWriter::new();
    write_sequence_header(&mut bw, &sequence_params);
    // §6.3.5: field pictures occur only in interlaced sequences.
    write_sequence_extension(&mut bw, frame_params.chroma_format, false);

    // Encode one frame as its top+bottom field-picture pair, returning
    // the assembled decoder-exact frame reconstruction.
    let encode_frame_pair = |bw: &mut BitWriter,
                             frame: &FrameBuffer,
                             tr: u16,
                             kind: PictureCodingType,
                             fwd_anchor: Option<&FrameBuffer>,
                             bwd_anchor: Option<&FrameBuffer>|
     -> Result<FrameBuffer> {
        let top_src = extract_field(frame, PictureStructure::TopField);
        let bottom_src = extract_field(frame, PictureStructure::BottomField);

        let (top_recon, bottom_recon) = match kind {
            PictureCodingType::Intra => {
                let top = encode_field_intra_picture(
                    bw,
                    &top_src,
                    &field_params,
                    PictureStructure::TopField,
                    tr,
                    quantiser_scale_code,
                )?;
                let bottom = encode_field_intra_picture(
                    bw,
                    &bottom_src,
                    &field_params,
                    PictureStructure::BottomField,
                    tr,
                    quantiser_scale_code,
                )?;
                (top, bottom)
            }
            PictureCodingType::Predictive => {
                let reference = fwd_anchor.ok_or(Error::InvalidBitstream(
                    "field P frame needs a forward anchor",
                ))?;
                let top = encode_field_p_picture(
                    bw,
                    &top_src,
                    reference,
                    &field_params,
                    PictureStructure::TopField,
                    tr,
                    quantiser_scale_code,
                    forward_f_code,
                )?;
                // §7.6.2.1: the second field predicts through the
                // synthetic reference pairing the just-coded first
                // field with the previous frame's opposite parity.
                let second_ref =
                    second_p_field_reference(&top, PictureStructure::TopField, reference)?;
                let bottom = encode_field_p_picture(
                    bw,
                    &bottom_src,
                    &second_ref,
                    &field_params,
                    PictureStructure::BottomField,
                    tr,
                    quantiser_scale_code,
                    forward_f_code,
                )?;
                (top, bottom)
            }
            PictureCodingType::Bidirectional => {
                let fwd = fwd_anchor.ok_or(Error::InvalidBitstream(
                    "field B frame needs a forward anchor",
                ))?;
                let bwd = bwd_anchor.ok_or(Error::InvalidBitstream(
                    "field B frame needs a backward anchor",
                ))?;
                let top = encode_field_b_picture(
                    bw,
                    &top_src,
                    fwd,
                    bwd,
                    &field_params,
                    PictureStructure::TopField,
                    tr,
                    quantiser_scale_code,
                    forward_f_code,
                    backward_f_code,
                )?;
                let bottom = encode_field_b_picture(
                    bw,
                    &bottom_src,
                    fwd,
                    bwd,
                    &field_params,
                    PictureStructure::BottomField,
                    tr,
                    quantiser_scale_code,
                    forward_f_code,
                    backward_f_code,
                )?;
                (top, bottom)
            }
            PictureCodingType::DcIntra => {
                return Err(Error::InvalidBitstream(
                    "D pictures do not exist in ISO/IEC 13818-2 (Table 6-12)",
                ))
            }
        };
        crate::frame_assembly::assemble_frame_from_fields(&top_recon, &bottom_recon)
    };

    let step = b_between + 1;
    let mut gop_start = 0usize;
    while gop_start < frames.len() {
        let gop_end = (gop_start + anchors_per_gop * step).min(frames.len() - 1);

        write_gop_header(
            &mut bw,
            &Mpeg2Gop {
                time_code: TimeCode::from_display_index(
                    gop_start as u64,
                    sequence_params.frame_rate_code,
                )?,
                closed_gop: true,
                broken_link: false,
            },
        );

        let mut forward_ref = encode_frame_pair(
            &mut bw,
            &frames[gop_start],
            0,
            PictureCodingType::Intra,
            None,
            None,
        )?;

        let mut prev_anchor = gop_start;
        while prev_anchor < gop_end {
            let next_anchor = (prev_anchor + step).min(gop_end);
            let backward_ref = encode_frame_pair(
                &mut bw,
                &frames[next_anchor],
                (next_anchor - gop_start) as u16,
                PictureCodingType::Predictive,
                Some(&forward_ref),
                None,
            )?;
            for b in (prev_anchor + 1)..next_anchor {
                encode_frame_pair(
                    &mut bw,
                    &frames[b],
                    (b - gop_start) as u16,
                    PictureCodingType::Bidirectional,
                    Some(&forward_ref),
                    Some(&backward_ref),
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

// =============================================================
// Adaptive per-macroblock mode selection: simple field prediction
// vs 16×8 MC vs dual-prime (Table 6-18 / §7.6.7.3 / §7.6.3.6)
// =============================================================

/// Flat bit-cost bias (in SAD units) charged to a 16×8-MC macroblock
/// when competing with simple field prediction: a second vector and a
/// second field-select flag cost wire bits one field vector does not.
const SIXTEEN_BY_EIGHT_BIAS: u32 = 48;

/// Flat bias charged to a dual-prime macroblock: one vector plus two
/// 1–2-bit Table B-11 `dmvector` codes.
const DUAL_PRIME_BIAS: u32 = 32;

/// Per-macroblock statistics an adaptive field-picture encode reports,
/// so callers and tests can confirm the Table 6-18 mode decisions
/// actually fire.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FieldModeStats {
    /// Macroblocks coded `field_motion_type = 01` (simple field
    /// prediction, Table 7-13 `Field-based`).
    pub simple_field: usize,
    /// Macroblocks coded `field_motion_type = 10` (§7.6.7.3 16×8 MC).
    pub sixteen_by_eight: usize,
    /// Macroblocks coded `field_motion_type = 11` (§7.6.3.6
    /// dual-prime).
    pub dual_prime: usize,
    /// Intra-coded macroblocks (P intra fallbacks).
    pub intra: usize,
}

impl FieldModeStats {
    /// Accumulate another picture's counters into this one.
    pub fn add(&mut self, other: &FieldModeStats) {
        self.simple_field += other.simple_field;
        self.sixteen_by_eight += other.sixteen_by_eight;
        self.dual_prime += other.dual_prime;
        self.intra += other.intra;
    }
}

/// Luma SAD between one 16×8 **region** of the current field's
/// macroblock (`region_offset` = 0 upper / 8 lower, in field lines)
/// and the §7.6.4 field prediction `(parity, (hx, hy))` forms from
/// `reference`'s chosen field.
fn field_region_luma_sad(
    current: &FrameBuffer,
    reference: &FrameBuffer,
    parity: FieldParity,
    mb_col: usize,
    mb_row: usize,
    region_offset: usize,
    hx: i32,
    hy: i32,
) -> u32 {
    let plane = ReferencePlane::new(
        reference.y.samples(),
        reference.y.width(),
        reference.y.height(),
    )
    .expect("reference luma plane is width*height");
    let Some(field) = FieldReference::new(plane, parity.index()) else {
        return u32::MAX;
    };
    let size = BlockSize::new(16, 8).expect("16x8 is a valid block size");
    let base_y = mb_row * 16 + region_offset;
    let pred = predict_field_block(field, (mb_col * 16) as i32, base_y as i32, size, hx, hy);
    let w = current.y.width();
    let h = current.y.height();
    let base_x = mb_col * 16;
    let mut sad = 0u32;
    for r in 0..8 {
        let sy = (base_y + r).min(h.saturating_sub(1));
        for c in 0..16 {
            let sx = (base_x + c).min(w.saturating_sub(1));
            let cur = i32::from(current.y.get(sx, sy).unwrap_or(0));
            let prd = i32::from(pred[r * 16 + c]);
            sad += (cur - prd).unsigned_abs();
        }
    }
    sad
}

/// Whether a `height_rows`-tall §7.6.4 read span at field-line
/// `base_y` stays inside the `parity` field view of the reference luma
/// plane (§7.6.3.8).
fn field_read_span_legal(
    reference: &FrameBuffer,
    parity: FieldParity,
    base_x: usize,
    base_y: usize,
    height_rows: i32,
    hx: i32,
    hy: i32,
) -> bool {
    let fw = reference.y.width() as i32;
    let fh = ((reference.y.height() - parity.index()).div_ceil(2)) as i32;
    let ix = hx.div_euclid(2);
    let iy = hy.div_euclid(2);
    let ex = i32::from(hx.rem_euclid(2) != 0);
    let ey = i32::from(hy.rem_euclid(2) != 0);
    let bx = base_x as i32;
    let by = base_y as i32;
    bx + ix >= 0 && by + iy >= 0 && bx + ix + 15 + ex < fw && by + iy + height_rows - 1 + ey < fh
}

/// Estimate the motion vector for one 16×8 **region** of a
/// field-picture macroblock (`region_offset` = 0 upper / 8 lower),
/// searching both reference field parities over §7.6.3.8-legal spans.
pub fn estimate_field_region_mv(
    current: &FrameBuffer,
    reference: &FrameBuffer,
    mb_col: usize,
    mb_row: usize,
    region_offset: usize,
    search_range: i32,
) -> FieldSearchResult {
    let base_x = mb_col * 16;
    let base_y = mb_row * 16 + region_offset;
    let vec_cost = |hx: i32, hy: i32| -> u32 { hx.unsigned_abs() + hy.unsigned_abs() };

    let mut best: Option<FieldSearchResult> = None;
    let mut best_score = u32::MAX;
    for parity in [FieldParity::Top, FieldParity::Bottom] {
        let mut p_best: Option<(i32, i32, u32, u32)> = None;
        let consider = |hx: i32, hy: i32, p_best: &mut Option<(i32, i32, u32, u32)>| {
            if !field_read_span_legal(reference, parity, base_x, base_y, 8, hx, hy) {
                return;
            }
            let sad = field_region_luma_sad(
                current,
                reference,
                parity,
                mb_col,
                mb_row,
                region_offset,
                hx,
                hy,
            );
            let score = sad.saturating_add(vec_cost(hx, hy));
            if p_best.map(|b| score < b.3).unwrap_or(true) {
                *p_best = Some((hx, hy, sad, score));
            }
        };
        for dy in -search_range..=search_range {
            for dx in -search_range..=search_range {
                consider(dx * 2, dy * 2, &mut p_best);
            }
        }
        if let Some((int_hx, int_hy, _, _)) = p_best {
            for &(ox, oy) in &[
                (-1i32, -1i32),
                (0, -1),
                (1, -1),
                (-1, 0),
                (1, 0),
                (-1, 1),
                (0, 1),
                (1, 1),
            ] {
                consider(int_hx + ox, int_hy + oy, &mut p_best);
            }
        }
        if let Some((hx, hy, sad, score)) = p_best {
            if score < best_score {
                best_score = score;
                best = Some(FieldSearchResult {
                    vector: MotionVectorPel::new(hx, hy),
                    parity,
                    sad,
                });
            }
        }
    }
    best.expect("the (0,0) same-parity candidate is always legal")
}

/// One scored adaptive-B candidate: `(direction, is_16x8,
/// (luma, cb, cr) prediction planes, biased score)`.
type BCandidateChoice = (BDirection, bool, (Vec<u8>, Vec<u8>, Vec<u8>), u32);

/// The chosen prediction mode of one adaptive P-field macroblock.
enum FieldPMode {
    Simple(FieldSearchResult),
    SixteenByEight(FieldSearchResult, FieldSearchResult),
    DualPrime(MotionVectorPel, (i8, i8), FieldPictureDualPrimeMotion),
}

/// Emit one field-picture motion vector `r` for direction `s`: the
/// §6.2.5.2 field-select flag then the components, coded against
/// `PMV[r][s]` directly (a field picture has no §7.6.3.1
/// vertical-half-pred). Updates the slot; the caller applies the
/// Table 7-11 copy row where the mode prescribes one.
fn emit_field_picture_vector(
    bw: &mut BitWriter,
    pmv: &mut PmvMirror,
    r: usize,
    s: usize,
    vector: MotionVectorPel,
    parity: FieldParity,
    f_code: u8,
) -> Result<()> {
    bw.write_bit(parity == FieldParity::Bottom);
    let dx = wrap_delta(vector.horizontal - pmv.get(r, s, 0), f_code)?;
    let dy = wrap_delta(vector.vertical - pmv.get(r, s, 1), f_code)?;
    encode_motion_component(bw, dx, f_code);
    encode_motion_component(bw, dy, f_code);
    pmv.set(r, s, 0, vector.horizontal);
    pmv.set(r, s, 1, vector.vertical);
    Ok(())
}

/// Quantise the six 4:2:0 blocks of a field-picture macroblock against
/// its prediction planes, returning `(blocks, cbp)`.
fn quantise_field_mb(
    current: &FrameBuffer,
    luma_pred: &[u8],
    cb_pred: &[u8],
    cr_pred: &[u8],
    mb_col: usize,
    mb_row: usize,
    qscale: u8,
    chroma_format: ChromaFormat,
) -> (Vec<crate::p_picture_encoder::InterBlock>, u8) {
    let nblocks = block_count(chroma_format);
    let mut blocks = Vec::with_capacity(nblocks);
    let mut cbp = 0u8;
    for i in 0..nblocks {
        let placement =
            block_placement(i, chroma_format, mb_col, mb_row, false).expect("valid block index");
        let component = block_component(i, chroma_format).expect("valid component");
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
        let block = quantise_inter_block(&residual, qscale);
        if block.is_coded() {
            cbp |= 1 << (5 - i);
        }
        blocks.push(block);
    }
    (blocks, cbp)
}

/// Encode one **P field picture** with per-macroblock Table 6-18 mode
/// selection: **simple field prediction** (`01`), **16×8 MC** (`10`,
/// §7.6.7.3 — independent `(vector, field-select)` per 16×8 region)
/// and — behind the §7.6.3.6 no-B gate — **dual-prime** (`11`, one
/// vector + Table B-11 `dmvector`, §7.6.7.4 two-field average), plus
/// the intra fallback. The bit-cost-biased luma SAD winner is coded;
/// reconstruction runs through the decode-side §7.6 drivers.
///
/// The §7.6.3 predictor bank is mirrored exactly: a simple field
/// vector reconstructs against `PMV[0][s]` and the Table 7-11
/// `Field-based` row copies it into `PMV[1][s]`; the two 16×8 vectors
/// use their own `r` slots with no copy ("(none)" row); dual-prime
/// uses `PMV[0][0]` with the `Dual-prime` copy row; an intra
/// macroblock resets the bank.
///
/// For the **second field of a P frame** pass the §7.6.2.1 synthetic
/// reference ([`second_p_field_reference`]).
///
/// # Errors
/// [`Error::InvalidBitstream`] for geometry / format mismatches;
/// propagates prediction errors.
pub fn encode_field_p_picture_adaptive(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    reference: &FrameBuffer,
    params: &IntraPictureParams,
    structure: PictureStructure,
    temporal_reference: u16,
    quantiser_scale_code: u8,
    forward_f_code: u8,
    allow_dual_prime: bool,
) -> Result<(FrameBuffer, FieldModeStats)> {
    check_field_params(params)?;
    if current.width != params.width || current.height != params.height {
        return Err(Error::InvalidBitstream(
            "encode_field_p_picture_adaptive: field dimensions do not match params",
        ));
    }
    if reference.height != params.height * 2 || reference.width != params.width {
        return Err(Error::InvalidBitstream(
            "encode_field_p_picture_adaptive: reference frame must be twice the field height",
        ));
    }
    let predicted_parity = match structure {
        PictureStructure::TopField => FieldParity::Top,
        PictureStructure::BottomField => FieldParity::Bottom,
        PictureStructure::Frame => {
            return Err(Error::InvalidBitstream(
                "encode_field_p_picture_adaptive: field structures only",
            ))
        }
    };
    let qscale = quantiser_scale(quantiser_scale_code, params.q_scale_type)?;
    let dc_mult = intra_dc_mult(params.intra_dc_precision)?;
    let search_range = max_search_range(forward_f_code).min(16);

    write_field_picture_headers(
        bw,
        params,
        structure,
        PictureCodingType::Predictive,
        temporal_reference,
        forward_f_code,
        forward_f_code,
    );

    let mut recon = params.new_frame_buffer();
    let mut stats = FieldModeStats::default();
    let mb_width = params.mb_width();
    let mb_height = params.mb_height();
    let nblocks = block_count(params.chroma_format);

    for mb_row in 0..mb_height {
        write_slice_header(bw, mb_row as u32, quantiser_scale_code);
        let mut pmv = PmvMirror::new();
        let mut intra_pred = IntraDcPred::reset(params.intra_dc_precision);
        let slice_first_addr = (mb_row * mb_width) as i32;
        let mut past_intra_address = slice_first_addr - 2;

        for mb_col in 0..mb_width {
            let mb_address = slice_first_addr + mb_col as i32;

            // ---- Mode search ----
            let simple = estimate_field_mv(current, reference, mb_col, mb_row, search_range);
            let upper =
                estimate_field_region_mv(current, reference, mb_col, mb_row, 0, search_range);
            let lower =
                estimate_field_region_mv(current, reference, mb_col, mb_row, 8, search_range);
            let sad_16x8 = upper.sad.saturating_add(lower.sad);

            let mut best_mode = FieldPMode::Simple(simple);
            let mut best_sad = simple.sad;
            let mut best_score = simple.sad;
            if sad_16x8.saturating_add(SIXTEEN_BY_EIGHT_BIAS) < best_score {
                best_score = sad_16x8.saturating_add(SIXTEEN_BY_EIGHT_BIAS);
                best_sad = sad_16x8;
                best_mode = FieldPMode::SixteenByEight(upper, lower);
            }
            if allow_dual_prime {
                // Base search: same-parity field, whole 16×16 field MB.
                let mut base_best: Option<(i32, i32, u32)> = None;
                let legal = |hx: i32, hy: i32| {
                    field_read_span_legal(
                        reference,
                        predicted_parity,
                        mb_col * 16,
                        mb_row * 16,
                        16,
                        hx,
                        hy,
                    )
                };
                for dy in -search_range..=search_range {
                    for dx in -search_range..=search_range {
                        let (hx, hy) = (dx * 2, dy * 2);
                        if !legal(hx, hy) {
                            continue;
                        }
                        let sad = field_luma_sad(
                            current,
                            reference,
                            predicted_parity,
                            mb_col,
                            mb_row,
                            hx,
                            hy,
                        );
                        if base_best.map(|b| sad < b.2).unwrap_or(true) {
                            base_best = Some((hx, hy, sad));
                        }
                    }
                }
                if let Some((bhx, bhy, _)) = base_best {
                    let base_mv = MotionVectorPel::new(bhx, bhy);
                    let mut dp_best: Option<(u32, (i8, i8), FieldPictureDualPrimeMotion)> = None;
                    for dmv_v in [-1i8, 0, 1] {
                        for dmv_h in [-1i8, 0, 1] {
                            let Ok(derived) = crate::dual_prime::derive_all(
                                crate::dual_prime::DualPrimePicture::Field { predicted_parity },
                                base_mv.horizontal,
                                base_mv.vertical,
                                i32::from(dmv_h),
                                i32::from(dmv_v),
                            ) else {
                                continue;
                            };
                            let Some(opp) = derived.first() else {
                                continue;
                            };
                            let opp_mv = MotionVectorPel::new(opp.horiz, opp.vert);
                            if !field_read_span_legal(
                                reference,
                                predicted_parity.opposite(),
                                mb_col * 16,
                                mb_row * 16,
                                16,
                                opp_mv.horizontal,
                                opp_mv.vertical,
                            ) {
                                continue;
                            }
                            let motion =
                                FieldPictureDualPrimeMotion::new(predicted_parity, base_mv, opp_mv);
                            let Ok((luma, _, _)) =
                                crate::inter_reconstruction::predict_field_picture_dual_prime_macroblock_planes(
                                    &recon, reference, mb_col, mb_row, motion,
                                )
                            else {
                                continue;
                            };
                            let w = current.y.width();
                            let h = current.y.height();
                            let mut sad = 0u32;
                            for r in 0..16 {
                                let sy = (mb_row * 16 + r).min(h.saturating_sub(1));
                                for c in 0..16 {
                                    let sx = (mb_col * 16 + c).min(w.saturating_sub(1));
                                    let cur = i32::from(current.y.get(sx, sy).unwrap_or(0));
                                    sad += (cur - i32::from(luma[r * 16 + c])).unsigned_abs();
                                }
                            }
                            if dp_best.map(|b| sad < b.0).unwrap_or(true) {
                                dp_best = Some((sad, (dmv_h, dmv_v), motion));
                            }
                        }
                    }
                    if let Some((sad, dmv, motion)) = dp_best {
                        if sad.saturating_add(DUAL_PRIME_BIAS) < best_score {
                            best_sad = sad;
                            best_mode = FieldPMode::DualPrime(base_mv, dmv, motion);
                        }
                    }
                }
            }

            // ---- Intra fallback ----
            let intra_cost = intra_activity(current, mb_col, mb_row);
            if best_sad > intra_cost.saturating_mul(2).saturating_add(512) {
                if mb_address - past_intra_address > 1 {
                    intra_pred = IntraDcPred::reset(params.intra_dc_precision);
                }
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
                    params.chroma_format,
                    crate::mpeg2_dct_coeff::TableSelection::TableZero,
                    false,
                );
                pmv.reset();
                past_intra_address = mb_address;
                stats.intra += 1;
                continue;
            }

            // ---- Prediction planes ----
            let (luma_pred, cb_pred, cr_pred) = match &best_mode {
                FieldPMode::Simple(s) => predict_field_picture_macroblock_planes(
                    &recon,
                    ReferenceFrames::forward_only(reference),
                    mb_col,
                    mb_row,
                    FieldPictureMotion::forward(s.vector, s.parity),
                )
                .map_err(crate::Error::from)?,
                FieldPMode::SixteenByEight(u, l) => {
                    crate::inter_reconstruction::predict_field_picture_16x8_macroblock_planes(
                        &recon,
                        ReferenceFrames::forward_only(reference),
                        mb_col,
                        mb_row,
                        crate::inter_reconstruction::FieldPicture16x8Motion::forward(
                            (u.vector, u.parity),
                            (l.vector, l.parity),
                        ),
                    )
                    .map_err(crate::Error::from)?
                }
                FieldPMode::DualPrime(_, _, motion) => {
                    crate::inter_reconstruction::predict_field_picture_dual_prime_macroblock_planes(
                        &recon, reference, mb_col, mb_row, *motion,
                    )
                    .map_err(crate::Error::from)?
                }
            };

            // ---- Residual quantisation ----
            let (blocks, cbp) = quantise_field_mb(
                current,
                &luma_pred,
                &cb_pred,
                &cr_pred,
                mb_col,
                mb_row,
                qscale,
                params.chroma_format,
            );

            // ---- Macroblock layer ----
            bw.write_bit(true); // macroblock_address_increment = 1
            if cbp != 0 {
                bw.write_bit(true); // Table B-3 "MC, Coded"
            } else {
                bw.write_u32(0b001, 3); // "MC, Not Coded"
            }
            // field_motion_type (Table 6-18). No dct_type in a field
            // picture (§6.2.5.1).
            match &best_mode {
                FieldPMode::Simple(s) => {
                    bw.write_u32(0b01, 2);
                    emit_field_picture_vector(
                        bw,
                        &mut pmv,
                        0,
                        0,
                        s.vector,
                        s.parity,
                        forward_f_code,
                    )?;
                    // Table 7-11 Field-based fwd row: PMV[1][0] = PMV[0][0].
                    pmv.copy_r0_to_r1(0);
                    stats.simple_field += 1;
                }
                FieldPMode::SixteenByEight(u, l) => {
                    bw.write_u32(0b10, 2);
                    emit_field_picture_vector(
                        bw,
                        &mut pmv,
                        0,
                        0,
                        u.vector,
                        u.parity,
                        forward_f_code,
                    )?;
                    emit_field_picture_vector(
                        bw,
                        &mut pmv,
                        1,
                        0,
                        l.vector,
                        l.parity,
                        forward_f_code,
                    )?;
                    // Table 7-11 16x8 row: "(none)".
                    stats.sixteen_by_eight += 1;
                }
                FieldPMode::DualPrime(mv, dmv, _) => {
                    bw.write_u32(0b11, 2);
                    // §6.2.5.2: no field-select flag when dmv == 1;
                    // dmvector[t] follows each component.
                    let dx = wrap_delta(mv.horizontal - pmv.get(0, 0, 0), forward_f_code)?;
                    encode_motion_component(bw, dx, forward_f_code);
                    crate::motion_vector::encode_dmvector(bw, dmv.0);
                    let dy = wrap_delta(mv.vertical - pmv.get(0, 0, 1), forward_f_code)?;
                    encode_motion_component(bw, dy, forward_f_code);
                    crate::motion_vector::encode_dmvector(bw, dmv.1);
                    pmv.set(0, 0, 0, mv.horizontal);
                    pmv.set(0, 0, 1, mv.vertical);
                    // Table 7-11 Dual-prime row: PMV[1][0] = PMV[0][0].
                    pmv.copy_r0_to_r1(0);
                    stats.dual_prime += 1;
                }
            }
            if cbp != 0 {
                encode_cbp420(bw, cbp);
                for b in &blocks {
                    if let Some(qf) = b.qf_ref() {
                        write_inter_block_coeffs(bw, qf, false);
                    }
                }
            }

            // ---- Decoder-exact reconstruction ----
            let residuals: Vec<crate::inter_reconstruction::ResidualBlock<'_>> = blocks
                .iter()
                .enumerate()
                .filter(|(_, b)| b.is_coded())
                .map(|(i, b)| crate::inter_reconstruction::ResidualBlock {
                    block_index: i as u8,
                    f_pel: b.f_pel_ref(),
                })
                .collect();
            match &best_mode {
                FieldPMode::Simple(s) => {
                    crate::inter_reconstruction::reconstruct_field_picture_macroblock(
                        &mut recon,
                        ReferenceFrames::forward_only(reference),
                        mb_col,
                        mb_row,
                        FieldPictureMotion::forward(s.vector, s.parity),
                        &residuals,
                    )
                    .map_err(crate::Error::from)?;
                }
                FieldPMode::SixteenByEight(u, l) => {
                    crate::inter_reconstruction::reconstruct_field_picture_16x8_macroblock(
                        &mut recon,
                        ReferenceFrames::forward_only(reference),
                        mb_col,
                        mb_row,
                        crate::inter_reconstruction::FieldPicture16x8Motion::forward(
                            (u.vector, u.parity),
                            (l.vector, l.parity),
                        ),
                        &residuals,
                    )
                    .map_err(crate::Error::from)?;
                }
                FieldPMode::DualPrime(_, _, motion) => {
                    crate::inter_reconstruction::reconstruct_field_picture_dual_prime_macroblock(
                        &mut recon, reference, mb_col, mb_row, *motion, &residuals,
                    )
                    .map_err(crate::Error::from)?;
                }
            }
        }
        bw.align_to_byte_zero();
    }
    Ok((recon, stats))
}

/// Encode one **B field picture** with per-macroblock Table 6-18 mode
/// selection between **simple field prediction** and **16×8 MC**, each
/// scored forward / backward / interpolated (dual-prime is P-only,
/// §7.6.3.6). Emits the Table B-4 mode, `field_motion_type`, and the
/// per-direction vectors (forward before backward; two `r` slots per
/// direction when 16×8), mirroring the Table 7-11 predictor-update
/// rows exactly.
///
/// # Errors
/// [`Error::InvalidBitstream`] for geometry / format mismatches;
/// propagates prediction errors.
pub fn encode_field_b_picture_adaptive(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    forward: &FrameBuffer,
    backward: &FrameBuffer,
    params: &IntraPictureParams,
    structure: PictureStructure,
    temporal_reference: u16,
    quantiser_scale_code: u8,
    forward_f_code: u8,
    backward_f_code: u8,
) -> Result<(FrameBuffer, FieldModeStats)> {
    check_field_params(params)?;
    if current.width != params.width || current.height != params.height {
        return Err(Error::InvalidBitstream(
            "encode_field_b_picture_adaptive: field dimensions do not match params",
        ));
    }
    for anchor in [forward, backward] {
        if anchor.height != params.height * 2 || anchor.width != params.width {
            return Err(Error::InvalidBitstream(
                "encode_field_b_picture_adaptive: anchor frame must be twice the field height",
            ));
        }
    }
    let qscale = quantiser_scale(quantiser_scale_code, params.q_scale_type)?;
    let fwd_range = max_search_range(forward_f_code).min(16);
    let bwd_range = max_search_range(backward_f_code).min(16);

    write_field_picture_headers(
        bw,
        params,
        structure,
        PictureCodingType::Bidirectional,
        temporal_reference,
        forward_f_code,
        backward_f_code,
    );

    let mut recon = params.new_frame_buffer();
    let mut stats = FieldModeStats::default();
    let mb_width = params.mb_width();
    let mb_height = params.mb_height();

    for mb_row in 0..mb_height {
        write_slice_header(bw, mb_row as u32, quantiser_scale_code);
        let mut pmv = PmvMirror::new();

        for mb_col in 0..mb_width {
            // ---- Per-direction searches (simple + 16×8 forms) ----
            let fs = estimate_field_mv(current, forward, mb_col, mb_row, fwd_range);
            let bs = estimate_field_mv(current, backward, mb_col, mb_row, bwd_range);
            let fu = estimate_field_region_mv(current, forward, mb_col, mb_row, 0, fwd_range);
            let fl = estimate_field_region_mv(current, forward, mb_col, mb_row, 8, fwd_range);
            let bu = estimate_field_region_mv(current, backward, mb_col, mb_row, 0, bwd_range);
            let bl = estimate_field_region_mv(current, backward, mb_col, mb_row, 8, bwd_range);

            let refs_fwd = ReferenceFrames::forward_only(forward);
            let refs_bwd = ReferenceFrames {
                forward: None,
                backward: Some(backward),
            };
            let refs_both = ReferenceFrames::bidirectional(forward, backward);
            let refs_of = |dir: BDirection| match dir {
                BDirection::Forward => refs_fwd,
                BDirection::Backward => refs_bwd,
                BDirection::Interpolated => refs_both,
            };

            // Candidates: (direction, is_16x8).
            let dirs = [
                BDirection::Forward,
                BDirection::Backward,
                BDirection::Interpolated,
            ];
            let luma_sad = |pred: &[u8]| -> u32 {
                let w = current.y.width();
                let h = current.y.height();
                let mut sad = 0u32;
                for r in 0..16 {
                    let sy = (mb_row * 16 + r).min(h.saturating_sub(1));
                    for c in 0..16 {
                        let sx = (mb_col * 16 + c).min(w.saturating_sub(1));
                        let cur = i32::from(current.y.get(sx, sy).unwrap_or(0));
                        sad += (cur - i32::from(pred[r * 16 + c])).unsigned_abs();
                    }
                }
                sad
            };
            let simple_motion = |dir: BDirection| FieldPictureMotion {
                forward: matches!(dir, BDirection::Forward | BDirection::Interpolated)
                    .then_some((fs.vector, fs.parity)),
                backward: matches!(dir, BDirection::Backward | BDirection::Interpolated)
                    .then_some((bs.vector, bs.parity)),
            };
            let sixteen_motion =
                |dir: BDirection| crate::inter_reconstruction::FieldPicture16x8Motion {
                    forward: matches!(dir, BDirection::Forward | BDirection::Interpolated)
                        .then_some([(fu.vector, fu.parity), (fl.vector, fl.parity)]),
                    backward: matches!(dir, BDirection::Backward | BDirection::Interpolated)
                        .then_some([(bu.vector, bu.parity), (bl.vector, bl.parity)]),
                };

            let mut best: Option<BCandidateChoice> = None;
            for dir in dirs {
                let planes_simple = predict_field_picture_macroblock_planes(
                    &recon,
                    refs_of(dir),
                    mb_col,
                    mb_row,
                    simple_motion(dir),
                )
                .map_err(crate::Error::from)?;
                let score_simple = luma_sad(&planes_simple.0);
                if best.as_ref().map(|b| score_simple < b.3).unwrap_or(true) {
                    best = Some((dir, false, planes_simple, score_simple));
                }
                let planes_16 =
                    crate::inter_reconstruction::predict_field_picture_16x8_macroblock_planes(
                        &recon,
                        refs_of(dir),
                        mb_col,
                        mb_row,
                        sixteen_motion(dir),
                    )
                    .map_err(crate::Error::from)?;
                let score_16 = luma_sad(&planes_16.0).saturating_add(SIXTEEN_BY_EIGHT_BIAS);
                if best.as_ref().map(|b| score_16 < b.3).unwrap_or(true) {
                    best = Some((dir, true, planes_16, score_16));
                }
            }
            let (dir, is_16x8, (luma_pred, cb_pred, cr_pred), _) =
                best.expect("six candidates were scored");

            // ---- Residual quantisation ----
            let (blocks, cbp) = quantise_field_mb(
                current,
                &luma_pred,
                &cb_pred,
                &cr_pred,
                mb_col,
                mb_row,
                qscale,
                params.chroma_format,
            );

            // ---- Macroblock layer ----
            bw.write_bit(true); // macroblock_address_increment = 1
            write_b_macroblock_type(bw, dir, cbp != 0);
            bw.write_u32(if is_16x8 { 0b10 } else { 0b01 }, 2);
            // Forward motion_vectors(0) precede backward (§6.2.5.2).
            if matches!(dir, BDirection::Forward | BDirection::Interpolated) {
                if is_16x8 {
                    emit_field_picture_vector(
                        bw,
                        &mut pmv,
                        0,
                        0,
                        fu.vector,
                        fu.parity,
                        forward_f_code,
                    )?;
                    emit_field_picture_vector(
                        bw,
                        &mut pmv,
                        1,
                        0,
                        fl.vector,
                        fl.parity,
                        forward_f_code,
                    )?;
                } else {
                    emit_field_picture_vector(
                        bw,
                        &mut pmv,
                        0,
                        0,
                        fs.vector,
                        fs.parity,
                        forward_f_code,
                    )?;
                    pmv.copy_r0_to_r1(0); // Table 7-11 Field-based row
                }
            }
            if matches!(dir, BDirection::Backward | BDirection::Interpolated) {
                if is_16x8 {
                    emit_field_picture_vector(
                        bw,
                        &mut pmv,
                        0,
                        1,
                        bu.vector,
                        bu.parity,
                        backward_f_code,
                    )?;
                    emit_field_picture_vector(
                        bw,
                        &mut pmv,
                        1,
                        1,
                        bl.vector,
                        bl.parity,
                        backward_f_code,
                    )?;
                } else {
                    emit_field_picture_vector(
                        bw,
                        &mut pmv,
                        0,
                        1,
                        bs.vector,
                        bs.parity,
                        backward_f_code,
                    )?;
                    pmv.copy_r0_to_r1(1); // Table 7-11 Field-based row
                }
            }
            if cbp != 0 {
                encode_cbp420(bw, cbp);
                for b in &blocks {
                    if let Some(qf) = b.qf_ref() {
                        write_inter_block_coeffs(bw, qf, false);
                    }
                }
            }
            if is_16x8 {
                stats.sixteen_by_eight += 1;
            } else {
                stats.simple_field += 1;
            }

            // ---- Decoder-exact reconstruction ----
            let residuals: Vec<crate::inter_reconstruction::ResidualBlock<'_>> = blocks
                .iter()
                .enumerate()
                .filter(|(_, b)| b.is_coded())
                .map(|(i, b)| crate::inter_reconstruction::ResidualBlock {
                    block_index: i as u8,
                    f_pel: b.f_pel_ref(),
                })
                .collect();
            if is_16x8 {
                crate::inter_reconstruction::reconstruct_field_picture_16x8_macroblock(
                    &mut recon,
                    refs_of(dir),
                    mb_col,
                    mb_row,
                    sixteen_motion(dir),
                    &residuals,
                )
                .map_err(crate::Error::from)?;
            } else {
                crate::inter_reconstruction::reconstruct_field_picture_macroblock(
                    &mut recon,
                    refs_of(dir),
                    mb_col,
                    mb_row,
                    simple_motion(dir),
                    &residuals,
                )
                .map_err(crate::Error::from)?;
            }
        }
        bw.align_to_byte_zero();
    }
    Ok((recon, stats))
}

/// [`encode_field_display_order_gop_sequence`] with the **adaptive**
/// per-macroblock Table 6-18 mode selection (simple field / 16×8 MC /
/// dual-prime) in every P and B field picture. `allow_dual_prime`
/// requires `b_between == 0` (§7.6.3.6: no B-pictures between the
/// predicted and reference fields).
///
/// Returns `(stream, stats)` — the accumulated mode counters over
/// every coded field picture.
///
/// # Errors
/// As [`encode_field_display_order_gop_sequence`], plus
/// [`Error::InvalidBitstream`] for `allow_dual_prime` with
/// `b_between > 0`.
pub fn encode_field_adaptive_display_order_gop_sequence(
    frames: &[FrameBuffer],
    b_between: usize,
    anchors_per_gop: usize,
    frame_params: &IntraPictureParams,
    quantiser_scale_code: u8,
    forward_f_code: u8,
    backward_f_code: u8,
    allow_dual_prime: bool,
) -> Result<(Vec<u8>, FieldModeStats)> {
    if frames.is_empty() {
        return Err(Error::InvalidBitstream(
            "encode_field_adaptive_display_order_gop_sequence: no frames to encode",
        ));
    }
    if anchors_per_gop == 0 {
        return Err(Error::InvalidBitstream(
            "encode_field_adaptive_display_order_gop_sequence: anchors_per_gop must be >= 1",
        ));
    }
    if allow_dual_prime && b_between > 0 {
        return Err(Error::InvalidBitstream(
            "encode_field_adaptive_display_order_gop_sequence: dual-prime requires no \
             B-pictures between the predicted and reference fields (§7.6.3.6)",
        ));
    }
    if frame_params.progressive_sequence {
        return Err(Error::InvalidBitstream(
            "encode_field_adaptive_display_order_gop_sequence: field pictures require \
             progressive_sequence = 0 (§6.3.5)",
        ));
    }
    if frame_params.height % 32 != 0 {
        return Err(Error::InvalidBitstream(
            "encode_field_adaptive_display_order_gop_sequence: frame height must be a \
             multiple of 32 (each field's §6.3.3 macroblock grid must be exact)",
        ));
    }
    let field_params = field_geometry(frame_params);
    check_field_params(&field_params)?;
    for f in frames {
        if f.width != frame_params.width || f.height != frame_params.height {
            return Err(Error::InvalidBitstream(
                "encode_field_adaptive_display_order_gop_sequence: frame geometry mismatch",
            ));
        }
    }

    let sequence_params = SequenceHeaderParams {
        horizontal_size: frame_params.width as u16,
        vertical_size: frame_params.height as u16,
        ..Default::default()
    };
    let mut bw = BitWriter::new();
    write_sequence_header(&mut bw, &sequence_params);
    write_sequence_extension(&mut bw, frame_params.chroma_format, false);

    let mut stats = FieldModeStats::default();

    // Encode one frame as its top+bottom field-picture pair.
    let encode_frame_pair = |bw: &mut BitWriter,
                             frame: &FrameBuffer,
                             tr: u16,
                             kind: PictureCodingType,
                             fwd_anchor: Option<&FrameBuffer>,
                             bwd_anchor: Option<&FrameBuffer>,
                             stats: &mut FieldModeStats|
     -> Result<FrameBuffer> {
        let top_src = extract_field(frame, PictureStructure::TopField);
        let bottom_src = extract_field(frame, PictureStructure::BottomField);

        let (top_recon, bottom_recon) = match kind {
            PictureCodingType::Intra => {
                let top = encode_field_intra_picture(
                    bw,
                    &top_src,
                    &field_params,
                    PictureStructure::TopField,
                    tr,
                    quantiser_scale_code,
                )?;
                let bottom = encode_field_intra_picture(
                    bw,
                    &bottom_src,
                    &field_params,
                    PictureStructure::BottomField,
                    tr,
                    quantiser_scale_code,
                )?;
                (top, bottom)
            }
            PictureCodingType::Predictive => {
                let reference = fwd_anchor.ok_or(Error::InvalidBitstream(
                    "field P frame needs a forward anchor",
                ))?;
                let (top, s1) = encode_field_p_picture_adaptive(
                    bw,
                    &top_src,
                    reference,
                    &field_params,
                    PictureStructure::TopField,
                    tr,
                    quantiser_scale_code,
                    forward_f_code,
                    allow_dual_prime,
                )?;
                stats.add(&s1);
                let second_ref =
                    second_p_field_reference(&top, PictureStructure::TopField, reference)?;
                let (bottom, s2) = encode_field_p_picture_adaptive(
                    bw,
                    &bottom_src,
                    &second_ref,
                    &field_params,
                    PictureStructure::BottomField,
                    tr,
                    quantiser_scale_code,
                    forward_f_code,
                    allow_dual_prime,
                )?;
                stats.add(&s2);
                (top, bottom)
            }
            PictureCodingType::Bidirectional => {
                let fwd = fwd_anchor.ok_or(Error::InvalidBitstream(
                    "field B frame needs a forward anchor",
                ))?;
                let bwd = bwd_anchor.ok_or(Error::InvalidBitstream(
                    "field B frame needs a backward anchor",
                ))?;
                let (top, s1) = encode_field_b_picture_adaptive(
                    bw,
                    &top_src,
                    fwd,
                    bwd,
                    &field_params,
                    PictureStructure::TopField,
                    tr,
                    quantiser_scale_code,
                    forward_f_code,
                    backward_f_code,
                )?;
                stats.add(&s1);
                let (bottom, s2) = encode_field_b_picture_adaptive(
                    bw,
                    &bottom_src,
                    fwd,
                    bwd,
                    &field_params,
                    PictureStructure::BottomField,
                    tr,
                    quantiser_scale_code,
                    forward_f_code,
                    backward_f_code,
                )?;
                stats.add(&s2);
                (top, bottom)
            }
            PictureCodingType::DcIntra => {
                return Err(Error::InvalidBitstream(
                    "D pictures do not exist in ISO/IEC 13818-2 (Table 6-12)",
                ))
            }
        };
        crate::frame_assembly::assemble_frame_from_fields(&top_recon, &bottom_recon)
    };

    let step = b_between + 1;
    let mut gop_start = 0usize;
    while gop_start < frames.len() {
        let gop_end = (gop_start + anchors_per_gop * step).min(frames.len() - 1);

        write_gop_header(
            &mut bw,
            &Mpeg2Gop {
                time_code: TimeCode::from_display_index(
                    gop_start as u64,
                    sequence_params.frame_rate_code,
                )?,
                closed_gop: true,
                broken_link: false,
            },
        );

        let mut forward_ref = encode_frame_pair(
            &mut bw,
            &frames[gop_start],
            0,
            PictureCodingType::Intra,
            None,
            None,
            &mut stats,
        )?;

        let mut prev_anchor = gop_start;
        while prev_anchor < gop_end {
            let next_anchor = (prev_anchor + step).min(gop_end);
            let backward_ref = encode_frame_pair(
                &mut bw,
                &frames[next_anchor],
                (next_anchor - gop_start) as u16,
                PictureCodingType::Predictive,
                Some(&forward_ref),
                None,
                &mut stats,
            )?;
            for b in (prev_anchor + 1)..next_anchor {
                encode_frame_pair(
                    &mut bw,
                    &frames[b],
                    (b - gop_start) as u16,
                    PictureCodingType::Bidirectional,
                    Some(&forward_ref),
                    Some(&backward_ref),
                    &mut stats,
                )?;
            }
            forward_ref = backward_ref;
            prev_anchor = next_anchor;
        }

        gop_start = gop_end + 1;
    }

    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
    Ok((stream, stats))
}
