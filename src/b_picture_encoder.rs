//! Bidirectional (B) picture encoder — forward, backward, and
//! interpolated motion-compensated prediction with a per-macroblock
//! residual.
//!
//! A B-picture predicts from two anchors: a **forward** (past) reference
//! and a **backward** (future) reference, both already reconstructed when
//! the B-picture is coded. Per macroblock the encoder evaluates three
//! prediction modes — forward-only, backward-only, and interpolated
//! (the §7.6.7.1 `// 2` average of the two) — and keeps the lowest-SAD
//! one, then codes the residual exactly as the P-encoder does.
//!
//! ## Per-macroblock pipeline (§6.2.5 / §7.6, Table B-4)
//!
//! 1. **Motion search** in both directions against the two reconstructed
//!    anchors ([`crate::motion_estimation::estimate_forward_mv`], reused
//!    for the backward reference since the search is direction-agnostic).
//! 2. **Mode decision** — the three §7.6.7 prediction directions are
//!    scored by the luma SAD of the prediction they form; the cheapest
//!    wins. Table B-4 then selects `Fwd, Coded` (`0011`) / `Fwd, Not
//!    Coded` (`0010`); `Bwd, Coded` (`011`) / `Bwd, Not Coded` (`010`);
//!    or `Interp, Coded` (`11`) / `Interp, Not Coded` (`10`), with the
//!    `Coded` form chosen when any block carries a non-zero level (the
//!    `coded_block_pattern` is non-empty).
//! 3. **Residual** + forward-DCT + dead-zone non-intra quantise, as in
//!    [`crate::p_picture_encoder`].
//! 4. **MV coding** — forward vectors precede backward vectors
//!    (bitstream order). Each direction keeps its own §7.6.3.4 PMV slot,
//!    both reset at slice start; a direction's PMV updates only when that
//!    direction is present in the macroblock (§7.6.3.3 Tables 7-10/7-11
//!    Frame-based rows).
//!
//! B-pictures are **not** reference frames, so the encoder does not need
//! to return a reconstruction.

#![allow(clippy::needless_range_loop)]
#![allow(clippy::too_many_arguments)]

use oxideav_core::bits::BitWriter;

use crate::coded_block_pattern::encode_coded_block_pattern;
use crate::encode_options::{FrameEncodeOptions, FrameEncodeStats};
use crate::frame_assembly::{block_placement, FrameBuffer, IntraPictureParams};
use crate::inter_reconstruction::{
    chroma_mb_extent, predict_frame_macroblock_planes, FrameMotion, MotionVectorPel,
    ReferenceFrames,
};
use crate::mb_address_increment::encode_mb_address_increment;
use crate::motion_estimation::{estimate_forward_mv, frame_vector_legal, max_search_range};
use crate::motion_vector::encode_motion_component;
use crate::mpeg2_block_dc::ColourComponent;
use crate::mpeg2_dequantize::quantiser_scale;
use crate::mpeg2_macroblock_blocks::{block_component, block_count};
use crate::p_picture_encoder::{
    nonintra_block_has_cbp_slot, quantise_inter_block, wrap_delta, write_inter_block_coeffs,
    InterBlock,
};
use crate::picture_header::PictureCodingType;
use crate::stream_writer::{
    write_picture_coding_extension, write_picture_header, write_slice_header,
};
use crate::Result;

/// The chosen prediction direction for one B macroblock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum BDirection {
    Forward,
    Backward,
    Interpolated,
}

/// Sum of absolute differences between the current macroblock's luma and
/// a candidate prediction plane (16×16, row-major).
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

/// Encode one bidirectional (B) frame picture from `current`, predicting
/// from the `forward` (past) and `backward` (future) reconstructed
/// anchors, appending the picture layer to `bw`.
///
/// `forward_f_code` / `backward_f_code` are the §6.2.3.1 `f_code[0][*]` /
/// `f_code[1][*]` values; the motion searches clamp to each code's range.
/// B-pictures are not references, so nothing is returned.
pub fn encode_b_picture(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    forward: &FrameBuffer,
    backward: &FrameBuffer,
    params: IntraPictureParams,
    temporal_reference: u16,
    quantiser_scale_code: u8,
    forward_f_code: u8,
    backward_f_code: u8,
) -> Result<()> {
    encode_b_picture_with_matrices(
        bw,
        current,
        forward,
        backward,
        params,
        temporal_reference,
        quantiser_scale_code,
        forward_f_code,
        backward_f_code,
        &crate::quant_matrix_extension::QuantiserMatrixState::defaults(),
    )
}

/// [`encode_b_picture`] quantising against an explicit §6.3.11
/// [`crate::quant_matrix_extension::QuantiserMatrixState`] (Table 7-5:
/// `w = 1` luminance / `w = 3` chrominance non-intra at 4:2:2 /
/// 4:4:4). The caller must have emitted the matching downloads.
#[allow(clippy::too_many_arguments)]
pub fn encode_b_picture_with_matrices(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    forward: &FrameBuffer,
    backward: &FrameBuffer,
    params: IntraPictureParams,
    temporal_reference: u16,
    quantiser_scale_code: u8,
    forward_f_code: u8,
    backward_f_code: u8,
    matrix_state: &crate::quant_matrix_extension::QuantiserMatrixState,
) -> Result<()> {
    encode_b_picture_with_options(
        bw,
        current,
        forward,
        backward,
        params,
        temporal_reference,
        quantiser_scale_code,
        forward_f_code,
        backward_f_code,
        matrix_state,
        FrameEncodeOptions::default(),
    )
}

/// [`encode_b_picture_with_matrices`] with the optional
/// [`FrameEncodeOptions`] behaviours: §7.6.6.4 skipped macroblocks
/// (the previous macroblock's prediction direction with the current
/// `PMV` vectors, tried first and taken whenever the residual
/// quantises to zero — never the first / last of a slice) and the
/// §6.3.10 output-cadence flags. The B encoder codes no intra
/// macroblocks, so `concealment_motion_vectors` only sets the
/// picture-coding-extension flag.
///
/// # Errors
/// [`crate::Error::InvalidBitstream`] on a §6.3.10 flag violation or
/// an invalid quantiser / f_code.
#[allow(clippy::too_many_arguments)]
pub fn encode_b_picture_with_options(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    forward: &FrameBuffer,
    backward: &FrameBuffer,
    params: IntraPictureParams,
    temporal_reference: u16,
    quantiser_scale_code: u8,
    forward_f_code: u8,
    backward_f_code: u8,
    matrix_state: &crate::quant_matrix_extension::QuantiserMatrixState,
    options: FrameEncodeOptions,
) -> Result<()> {
    encode_b_picture_with_stats(
        bw,
        current,
        forward,
        backward,
        params,
        temporal_reference,
        quantiser_scale_code,
        forward_f_code,
        backward_f_code,
        matrix_state,
        options,
    )
    .map(|_stats| ())
}

/// [`encode_b_picture_with_options`] also returning the per-macroblock
/// decision counts ([`FrameEncodeStats`]).
///
/// # Errors
/// As [`encode_b_picture_with_options`].
#[allow(clippy::too_many_arguments)]
pub fn encode_b_picture_with_stats(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    forward: &FrameBuffer,
    backward: &FrameBuffer,
    params: IntraPictureParams,
    temporal_reference: u16,
    quantiser_scale_code: u8,
    forward_f_code: u8,
    backward_f_code: u8,
    matrix_state: &crate::quant_matrix_extension::QuantiserMatrixState,
    options: FrameEncodeOptions,
) -> Result<FrameEncodeStats> {
    let qscale = quantiser_scale(quantiser_scale_code, params.q_scale_type)?;
    let fwd_range = max_search_range(forward_f_code).min(16);
    let bwd_range = max_search_range(backward_f_code).min(16);
    let mut stats = FrameEncodeStats::default();

    // §6.3.10: the MPEG-1 legacy forward/backward_f_code fields in the
    // picture header "shall have the value seven (all ones)" in an
    // ISO/IEC 13818-2 stream — the real per-direction f_codes live in
    // the picture_coding_extension().
    write_picture_header(
        bw,
        temporal_reference,
        PictureCodingType::Bidirectional,
        0b111,
        0b111,
    );
    let pce = options.picture_coding_extension(&params, forward_f_code, backward_f_code)?;
    write_picture_coding_extension(bw, &pce);

    let mb_width = params.mb_width();
    let mb_height = params.mb_height();
    let nblocks = block_count(params.chroma_format);
    // A throw-away frame supplying the macroblock geometry to the
    // prediction former (its samples are never read — only its
    // dimensions / chroma format matter).
    let geom = FrameBuffer::new(params.width, params.height, params.chroma_format);
    let (cmb_w, cmb_h) = chroma_mb_extent(params.chroma_format);

    // Residual + forward quantise every block of one macroblock against
    // the given prediction planes.
    let quantise_mb = |mb_col: usize,
                       mb_row: usize,
                       pred: &(Vec<u8>, Vec<u8>, Vec<u8>)|
     -> (Vec<InterBlock>, [bool; 12]) {
        let (luma_pred, cb_pred, cr_pred) = (&pred.0, &pred.1, &pred.2);
        let mut blocks = Vec::with_capacity(nblocks);
        let mut coded_flags = [false; 12];
        for i in 0..nblocks {
            let placement = block_placement(i, params.chroma_format, mb_col, mb_row, false)
                .expect("valid block index");
            let component = block_component(i, params.chroma_format).expect("valid component");
            let (cur_plane, pred, pred_w, mb_origin_x, mb_origin_y) = match component {
                ColourComponent::Y => (&current.y, luma_pred, 16usize, mb_col * 16, mb_row * 16),
                ColourComponent::Cb => {
                    (&current.cb, cb_pred, cmb_w, mb_col * cmb_w, mb_row * cmb_h)
                }
                ColourComponent::Cr => {
                    (&current.cr, cr_pred, cmb_w, mb_col * cmb_w, mb_row * cmb_h)
                }
            };
            let local_x = placement.base_x - mb_origin_x;
            let local_y = placement.base_y - mb_origin_y;
            let mut residual = [[0i16; 8]; 8];
            let w = cur_plane.width();
            let h = cur_plane.height();
            for r in 0..8 {
                let sy = (placement.base_y + r).min(h.saturating_sub(1));
                for c in 0..8 {
                    let sx = (placement.base_x + c).min(w.saturating_sub(1));
                    let cur = i32::from(cur_plane.get(sx, sy).unwrap_or(0));
                    let prd = i32::from(pred[(local_y + r) * pred_w + (local_x + c)]);
                    residual[r][c] = (cur - prd) as i16;
                }
            }
            // Table 7-5: non-intra luminance → w 1, non-intra
            // chrominance → w 3 (mirrors w 1 at 4:2:0).
            let weight = match component {
                ColourComponent::Y => &matrix_state.non_intra_luma,
                ColourComponent::Cb | ColourComponent::Cr => &matrix_state.non_intra_chroma,
            };
            let block = if nonintra_block_has_cbp_slot(i, params.chroma_format) {
                quantise_inter_block(&residual, qscale, weight)
            } else {
                // Printed §6.3.17.4: no wire slot — leave the
                // block uncoded so encoder and decoder agree.
                InterBlock::uncoded()
            };
            coded_flags[i] = block.is_coded();
            blocks.push(block);
        }
        (blocks, coded_flags)
    };

    // Form the prediction for a direction + vector pair.
    let predict = |mb_col: usize,
                   mb_row: usize,
                   dir: BDirection,
                   fwd: MotionVectorPel,
                   bwd: MotionVectorPel|
     -> (Vec<u8>, Vec<u8>, Vec<u8>) {
        match dir {
            BDirection::Forward => predict_frame_macroblock_planes(
                &geom,
                ReferenceFrames::forward_only(forward),
                mb_col,
                mb_row,
                FrameMotion::forward(fwd),
            )
            .expect("forward prediction"),
            BDirection::Backward => predict_frame_macroblock_planes(
                &geom,
                ReferenceFrames {
                    forward: None,
                    backward: Some(backward),
                },
                mb_col,
                mb_row,
                FrameMotion::backward(bwd),
            )
            .expect("backward prediction"),
            BDirection::Interpolated => predict_frame_macroblock_planes(
                &geom,
                ReferenceFrames::bidirectional(forward, backward),
                mb_col,
                mb_row,
                FrameMotion::bidirectional(fwd, bwd),
            )
            .expect("interpolated prediction"),
        }
    };

    for mb_row in 0..mb_height {
        write_slice_header(bw, mb_row as u32, quantiser_scale_code);
        // §7.6.3.4: forward and backward PMV slots both reset at slice
        // start.
        let mut pmv_fwd = (0i32, 0i32);
        let mut pmv_bwd = (0i32, 0i32);
        // §7.6.6.4: a skipped macroblock inherits the previous
        // macroblock's prediction direction. `None` until the slice's
        // first macroblock is coded.
        let mut prev_dir: Option<BDirection> = None;
        let mut pending_skips = 0u32;

        for mb_col in 0..mb_width {
            // 0. Skip test (§7.6.6.4): the previous direction with the
            // PMV vectors, taken when the residual quantises to zero.
            // §6.3.17: never the first / last macroblock of a slice.
            if options.skipped_macroblocks && mb_col != 0 && mb_col + 1 != mb_width {
                if let Some(dir) = prev_dir {
                    let fwd = MotionVectorPel::new(pmv_fwd.0, pmv_fwd.1);
                    let bwd = MotionVectorPel::new(pmv_bwd.0, pmv_bwd.1);
                    // §7.6.3.8: the inherited vectors must read inside
                    // the reference; a predictor from a neighbour can
                    // overhang at the picture edge.
                    let uses_fwd = matches!(dir, BDirection::Forward | BDirection::Interpolated);
                    let uses_bwd = matches!(dir, BDirection::Backward | BDirection::Interpolated);
                    let legal = (!uses_fwd
                        || frame_vector_legal(
                            forward.y.width(),
                            forward.y.height(),
                            mb_col,
                            mb_row,
                            fwd.horizontal,
                            fwd.vertical,
                        ))
                        && (!uses_bwd
                            || frame_vector_legal(
                                backward.y.width(),
                                backward.y.height(),
                                mb_col,
                                mb_row,
                                bwd.horizontal,
                                bwd.vertical,
                            ));
                    if legal {
                        let pred = predict(mb_col, mb_row, dir, fwd, bwd);
                        let (_blocks, coded_flags) = quantise_mb(mb_col, mb_row, &pred);
                        if !coded_flags[..nblocks].iter().any(|&b| b) {
                            pending_skips += 1;
                            stats.skipped += 1;
                            // Predictors and direction are unaffected.
                            continue;
                        }
                    }
                }
            }

            // 1. Motion search in both directions.
            let fwd = estimate_forward_mv(current, forward, mb_col, mb_row, fwd_range).vector;
            let bwd = estimate_forward_mv(current, backward, mb_col, mb_row, bwd_range).vector;

            // 2. Score the three candidate predictions.
            let pred_fwd = predict(mb_col, mb_row, BDirection::Forward, fwd, bwd);
            let pred_bwd = predict(mb_col, mb_row, BDirection::Backward, fwd, bwd);
            let pred_int = predict(mb_col, mb_row, BDirection::Interpolated, fwd, bwd);

            let sad_fwd = luma_sad(current, mb_col, mb_row, &pred_fwd.0);
            let sad_bwd = luma_sad(current, mb_col, mb_row, &pred_bwd.0);
            let sad_int = luma_sad(current, mb_col, mb_row, &pred_int.0);

            // Pick the cheapest; ties prefer interpolated then forward
            // (fewer bits than two separate vectors only for fwd/bwd, but
            // interpolation usually yields the smallest residual — bias to
            // it only on a strict win).
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

            // 3. Residual + forward quantise per block.
            let (blocks, coded_flags) = quantise_mb(mb_col, mb_row, chosen);

            // 4. Macroblock layer.
            encode_mb_address_increment(bw, pending_skips + 1);
            pending_skips = 0;
            let coded = coded_flags[..nblocks].iter().any(|&b| b);
            write_b_macroblock_type(bw, dir, coded);
            if coded {
                stats.coded += 1;
            } else {
                stats.not_coded += 1;
            }

            // Forward MV(s) precede backward MV(s) (bitstream order).
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
            prev_dir = Some(dir);

            if coded {
                encode_coded_block_pattern(bw, &coded_flags[..nblocks], params.chroma_format)?;
                for i in 0..nblocks {
                    if let Some(qf) = blocks[i].qf_ref() {
                        write_inter_block_coeffs(bw, qf, params.alternate_scan);
                    }
                }
            }
        }
        debug_assert_eq!(
            pending_skips, 0,
            "the last macroblock of a slice is never skipped"
        );
        bw.align_to_byte_zero();
    }

    Ok(stats)
}

/// Emit the Table B-4 `macroblock_type` codeword for the chosen
/// direction + coded flag (the baseline `macroblock_quant == 0` rows).
pub(crate) fn write_b_macroblock_type(bw: &mut BitWriter, dir: BDirection, coded: bool) {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sequence_extension::ChromaFormat;

    #[test]
    fn b_macroblock_type_codewords_match_table_b4() {
        // (direction, coded, expected bit length)
        let cases = [
            (BDirection::Interpolated, false, 2u32),
            (BDirection::Interpolated, true, 2),
            (BDirection::Backward, false, 3),
            (BDirection::Backward, true, 3),
            (BDirection::Forward, false, 4),
            (BDirection::Forward, true, 4),
        ];
        for (dir, coded, bits) in cases {
            let mut bw = BitWriter::new();
            write_b_macroblock_type(&mut bw, dir, coded);
            // The writer emitted exactly `bits` bits of the codeword.
            assert_eq!(bw.bit_position(), u64::from(bits), "{dir:?} coded={coded}");
        }
    }

    #[test]
    fn luma_sad_zero_for_matching_prediction() {
        let mut f = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
        for y in 0..16 {
            for x in 0..16 {
                f.y.put_sample(x, y, (x * 4 + y) as u8);
            }
        }
        let mut pred = vec![0u8; 256];
        for y in 0..16 {
            for x in 0..16 {
                pred[y * 16 + x] = (x * 4 + y) as u8;
            }
        }
        assert_eq!(luma_sad(&f, 0, 0, &pred), 0);
    }
}
