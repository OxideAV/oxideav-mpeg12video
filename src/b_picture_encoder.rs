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
use crate::frame_assembly::{block_placement, FrameBuffer, IntraPictureParams};
use crate::inter_reconstruction::{
    chroma_mb_extent, predict_frame_macroblock_planes, FrameMotion, ReferenceFrames,
};
use crate::motion_estimation::{estimate_forward_mv, max_search_range};
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
    PictureCodingExtensionParams,
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
    let qscale = quantiser_scale(quantiser_scale_code, params.q_scale_type)?;
    let fwd_range = max_search_range(forward_f_code).min(16);
    let bwd_range = max_search_range(backward_f_code).min(16);

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
    write_picture_coding_extension(
        bw,
        &PictureCodingExtensionParams {
            forward_f_code,
            backward_f_code,
            frame_pred_frame_dct: params.frame_pred_frame_dct,
            intra_dc_precision: params.intra_dc_precision,
            q_scale_type: params.q_scale_type,
            intra_vlc_format: params.intra_vlc_format,
            alternate_scan: params.alternate_scan,
            // §6.3.10: progressive_sequence == 1 requires
            // progressive_frame == 1.
            progressive_frame: params.progressive_sequence,
        },
    );

    let mb_width = params.mb_width();
    let mb_height = params.mb_height();
    let nblocks = block_count(params.chroma_format);
    // A throw-away frame supplying the macroblock geometry to the
    // prediction former (its samples are never read — only its
    // dimensions / chroma format matter).
    let geom = FrameBuffer::new(params.width, params.height, params.chroma_format);

    for mb_row in 0..mb_height {
        write_slice_header(bw, mb_row as u32, quantiser_scale_code);
        // §7.6.3.4: forward and backward PMV slots both reset at slice
        // start.
        let mut pmv_fwd = (0i32, 0i32);
        let mut pmv_bwd = (0i32, 0i32);

        for mb_col in 0..mb_width {
            // 1. Motion search in both directions.
            let fwd = estimate_forward_mv(current, forward, mb_col, mb_row, fwd_range).vector;
            let bwd = estimate_forward_mv(current, backward, mb_col, mb_row, bwd_range).vector;

            // 2. Score the three candidate predictions.
            let pred_fwd = predict_frame_macroblock_planes(
                &geom,
                ReferenceFrames::forward_only(forward),
                mb_col,
                mb_row,
                FrameMotion::forward(fwd),
            )
            .expect("forward prediction");
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
            .expect("backward prediction");
            let pred_int = predict_frame_macroblock_planes(
                &geom,
                ReferenceFrames::bidirectional(forward, backward),
                mb_col,
                mb_row,
                FrameMotion::bidirectional(fwd, bwd),
            )
            .expect("interpolated prediction");

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
            let (luma_pred, cb_pred, cr_pred) = (&chosen.0, &chosen.1, &chosen.2);

            // 3. Residual + forward quantise per block.
            let (cmb_w, cmb_h) = chroma_mb_extent(params.chroma_format);
            let mut blocks = Vec::with_capacity(nblocks);
            let mut coded_flags = [false; 12];
            for i in 0..nblocks {
                let placement = block_placement(i, params.chroma_format, mb_col, mb_row, false)
                    .expect("valid block index");
                let component = block_component(i, params.chroma_format).expect("valid component");
                let (cur_plane, pred, pred_w, mb_origin_x, mb_origin_y) = match component {
                    ColourComponent::Y => {
                        (&current.y, luma_pred, 16usize, mb_col * 16, mb_row * 16)
                    }
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

            // 4. Macroblock layer.
            // macroblock_address_increment = 1 (Table B-1 `1`).
            bw.write_bit(true);
            let coded = coded_flags[..nblocks].iter().any(|&b| b);
            write_b_macroblock_type(bw, dir, coded);

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

            if coded {
                encode_coded_block_pattern(bw, &coded_flags[..nblocks], params.chroma_format)?;
                for i in 0..nblocks {
                    if let Some(qf) = blocks[i].qf_ref() {
                        write_inter_block_coeffs(bw, qf, params.alternate_scan);
                    }
                }
            }
        }
        bw.align_to_byte_zero();
    }

    Ok(())
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
