//! §7.6 Picture-level P/B reconstruction driver per ISO/IEC 13818-2
//! (ITU-T H.262) / ISO/IEC 11172-2 (MPEG-1) — the top-level loop that
//! reconstructs a whole **P** or **B** picture to real pixels by
//! threading the §6.2.4 slice walk + §7.6.3 motion-vector
//! reconstruction into the §7.6.4–§7.6.8 motion-compensated
//! reconstruction of [`crate::inter_reconstruction`].
//!
//! ## Where this sits
//!
//! [`crate::decode_intra_picture`] already reconstructs an **I**
//! picture end-to-end: it scans the slice start codes, walks each
//! slice with the §6.2.6 `block(i)` pipeline enabled, and places every
//! intra macroblock's reconstructed blocks into a [`FrameBuffer`].
//!
//! This module is the equivalent for **P / B** pictures. The per-slice
//! §7.6.3 motion-vector reconstruction
//! ([`crate::reconstruct_slice_motion_vectors`]) and the per-macroblock
//! §7.6.4–§7.6.8 motion-compensated reconstruction
//! ([`crate::reconstruct_inter_macroblock`]) already exist; this driver
//! is the loop that:
//!
//! 1. Scans each `slice_start_code` of the picture.
//! 2. Walks the slice with block decoding enabled
//!    ([`crate::walk_slice_at`]) to recover the per-macroblock records
//!    (`macroblock_type`, motion vectors, `pattern_code[]`, decoded
//!    residual blocks).
//! 3. Reconstructs the slice's motion vectors
//!    ([`crate::reconstruct_slice_motion_vectors`]) — the §7.6.3.4
//!    slice-start PMV reset, the §7.6.3.1 per-macroblock
//!    reconstruction, the §7.6.3.3 update, and the §7.6.6
//!    skipped-macroblock PMV side-effects.
//! 4. For each coded macroblock: **intra** macroblocks route to
//!    [`crate::place_intra_macroblock`] (the §7.6.8 `d = saturate(f)`
//!    intra path); **inter** macroblocks route to
//!    [`crate::reconstruct_inter_macroblock`] with the reconstructed
//!    frame-based motion vector(s).
//! 5. For each **skipped** macroblock (the address gap between coded
//!    macroblocks, §6.3.17.1): reconstruct the §7.6.6 prediction —
//!    a P-picture `(0,0)` forward prediction, or a B-picture
//!    prediction inheriting the previous macroblock's direction with
//!    the carried PMVs.
//!
//! ## Scope
//!
//! This driver handles the **frame-picture, frame-based** prediction
//! modes (Table 7-14 `Frame-based` rows) plus the §7.6.6 skipped
//! cases — the modes that cover MPEG-1 entirely and the bulk of
//! MPEG-2 frame-coded P/B pictures. Field pictures and the
//! field-based / 16×8-MC / dual-prime modes select their predictions
//! through [`crate::select_predictions`] but their per-field reference
//! assembly is a later milestone; a macroblock that requires one is
//! reported through [`crate::InterError::UnsupportedPredictionMode`].
//!
//! Spec citations refer to **ISO/IEC 13818-2 (H.262)** §6.2.4 / §7.6
//! and **ISO/IEC 11172-2** §2.4.

use crate::frame_assembly::{place_intra_macroblock, FrameBuffer, IntraPictureParams};
use crate::inter_reconstruction::{
    reconstruct_inter_macroblock, FrameMotion, InterError, MotionVectorPel, ReferenceFrames,
    ResidualBlock,
};
use crate::macroblock_modes::PredictionType;
use crate::picture_header::{PictureCodingType, PictureStructure};
use crate::slice_macroblock_walk::{
    reconstruct_slice_motion_vectors, walk_slice_at, MacroblockRecord, ReconstructedMotionVectors,
    SliceWalkContext,
};
use crate::Result;

/// The per-picture parameters the P/B reconstruction driver needs: the
/// geometry + per-block DCT context (shared with the intra driver via
/// [`IntraPictureParams`]) plus the §6.3.10 / §6.3.11 picture-coding
/// fields that gate the §7.6.3 motion-vector reconstruction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PicturePredictionParams {
    /// The geometry + per-block DCT context shared with the intra
    /// picture driver.
    pub geometry: IntraPictureParams,
    /// `picture_coding_type` (§6.3.10): `Predictive` (P) or
    /// `Bidirectional` (B). `Intra` is rejected — use
    /// [`crate::decode_intra_picture`] for I-pictures.
    pub picture_coding_type: PictureCodingType,
    /// `f_code[0][0]` — forward horizontal f_code (§6.3.11).
    pub f_code_fwd_horiz: u8,
    /// `f_code[0][1]` — forward vertical f_code.
    pub f_code_fwd_vert: u8,
    /// `f_code[1][0]` — backward horizontal f_code (B-pictures).
    pub f_code_bwd_horiz: u8,
    /// `f_code[1][1]` — backward vertical f_code.
    pub f_code_bwd_vert: u8,
    /// `concealment_motion_vectors` (§6.3.11).
    pub concealment_motion_vectors: bool,
}

/// Reconstruct a whole **P** or **B** picture into a [`FrameBuffer`],
/// using `references` for the §7.6.4 motion-compensated prediction.
///
/// `picture` is the slice of the elementary stream from the first
/// `slice_start_code` of the picture up to (but not including) the
/// next picture / GOP / sequence start code — the same input
/// [`crate::decode_intra_picture`] consumes.
///
/// `references` carries the forward (past) anchor for P-pictures and
/// both anchors for B-pictures. For a P-picture only
/// [`ReferenceFrames::forward`] is read; a B-picture reads both.
///
/// Returns the assembled [`FrameBuffer`] and the number of macroblocks
/// reconstructed (coded + skipped).
///
/// # Errors
///
/// * Propagates [`crate::Error`] from slice-header parsing, the
///   macroblock walk, or the §7.6.3 motion-vector reconstruction.
/// * Surfaces [`InterError`] (wrapped into [`crate::Error`]) for an
///   unsupported prediction mode or a missing reference frame.
pub fn decode_inter_picture(
    picture: &[u8],
    params: PicturePredictionParams,
    references: ReferenceFrames<'_>,
) -> Result<(FrameBuffer, usize)> {
    use crate::slice_header::{SliceContext, SliceHeader};

    let geom = params.geometry;
    let mut frame = FrameBuffer::new(geom.width, geom.height, geom.chroma_format);
    let mb_width = geom.mb_width() as u32;
    let slice_ctx = SliceContext::non_scalable(geom.height as u32);

    let mut placed = 0usize;
    let mut offset = 0usize;
    while let Some(rel) = find_slice_start_code(&picture[offset..]) {
        let start = offset + rel;
        let body = &picture[start..];
        let end = find_next_start_code(&body[4..])
            .map(|p| p + 4)
            .unwrap_or(body.len());
        // Include the terminating start code's bytes in the slice
        // buffer: the §6.2.4 walker peeks for the next start code's
        // 23-zero prefix to detect end-of-slice (§5.2.3), so the buffer
        // must carry that prefix or the walker stops one macroblock
        // early. `next_slice_offset` (where the *next* iteration begins)
        // still points at the start code itself.
        let next_slice_offset = start + end;
        let slice_buf = &picture[start..(start + end + 4).min(picture.len())];

        let header = SliceHeader::parse(slice_buf, slice_ctx)?;
        let mb_row = u32::from(header.slice_vertical_position) - 1;

        let ctx = SliceWalkContext::first_slice_with_block_decoding(
            mb_width,
            mb_row,
            params.picture_coding_type,
            header.quantiser_scale_code,
            PictureStructure::Frame,
            geom.frame_pred_frame_dct,
            params.f_code_fwd_horiz,
            params.f_code_fwd_vert,
            params.f_code_bwd_horiz,
            params.f_code_bwd_vert,
            params.concealment_motion_vectors,
            geom.chroma_format,
            geom.intra_vlc_format,
            geom.alternate_scan,
            geom.intra_dc_precision,
            geom.q_scale_type,
        );

        let walk = walk_slice_at(slice_buf, header.body_bit_position, ctx)?;
        let motion = reconstruct_slice_motion_vectors(&walk, &ctx)?;

        // Reconstruct each macroblock. The §7.6.3 motion walk emits one
        // SliceMotionRecord per coded macroblock, in the same order as
        // the SliceWalk records, so we zip the two.
        let mut previous_inter_direction: Option<InterDirection> = None;
        for (record, motion_record) in walk.macroblocks.iter().zip(motion.records.iter()) {
            // §7.6.6: skipped macroblocks immediately preceding this
            // coded one. Their address range is
            // `macroblock_address - skipped_before .. macroblock_address`.
            let skipped = motion_record.skipped_before;
            for k in 0..skipped {
                let skipped_address = record.macroblock_address - skipped + k;
                placed += reconstruct_skipped_macroblock(
                    &mut frame,
                    references,
                    skipped_address as usize,
                    mb_width as usize,
                    params.picture_coding_type,
                    previous_inter_direction,
                )?;
            }

            placed += reconstruct_one_macroblock(
                &mut frame,
                references,
                record,
                &motion_record.reconstructed,
                mb_width as usize,
                geom.chroma_format,
                &mut previous_inter_direction,
            )?;
        }

        offset = next_slice_offset;
    }

    Ok((frame, placed))
}

/// The §7.6.6 inherited prediction direction carried across skipped
/// B-picture macroblocks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct InterDirection {
    forward: Option<MotionVectorPel>,
    backward: Option<MotionVectorPel>,
}

/// Reconstruct one coded macroblock — intra via
/// [`place_intra_macroblock`], inter via
/// [`reconstruct_inter_macroblock`]. Updates `previous_inter_direction`
/// to this macroblock's reconstructed motion when it is a non-intra
/// frame-based macroblock (so a following §7.6.6 B-skip can inherit it).
fn reconstruct_one_macroblock(
    frame: &mut FrameBuffer,
    references: ReferenceFrames<'_>,
    record: &MacroblockRecord,
    reconstructed: &ReconstructedMotionVectors,
    mb_width: usize,
    chroma_format: crate::sequence_extension::ChromaFormat,
    previous_inter_direction: &mut Option<InterDirection>,
) -> Result<usize> {
    if record.macroblock_type.macroblock_intra {
        // §7.6.8 intra path: place the saturated IDCT output. An intra
        // MB has no motion to carry forward for a following B-skip; the
        // §7.6.6 driver rejects a B-skip after an intra MB, so leave
        // `previous_inter_direction` unchanged.
        place_intra_macroblock(frame, record, mb_width, chroma_format);
        return Ok(1); // exactly one macroblock placed
    }

    // §7.6.5 / Table 7-14: reject the field-based / 16×8-MC / dual-prime
    // frame-picture modes this driver does not yet assemble per-field
    // references for.
    if let Some(mt) = record.motion_type {
        if !matches!(mt.prediction_type, PredictionType::FrameBased) {
            return Err(crate::Error::from(InterError::UnsupportedPredictionMode));
        }
    }

    let motion = frame_motion_from_reconstructed(reconstructed);
    let mb_col = (record.macroblock_address as usize) % mb_width;
    let mb_row = (record.macroblock_address as usize) / mb_width;
    let field_dct = record.dct_type == Some(true);

    // Gather the coded residual blocks: the walker emits one
    // DecodedBlock per coded slot, each carrying its block_index and the
    // §A IDCT output `f_pel`.
    let f_pels: Vec<([[i16; 8]; 8], u8)> = match record.decoded_blocks.as_ref() {
        Some(blocks) => blocks
            .iter()
            .map(|b| (b.decoded.f_pel, b.block_index))
            .collect(),
        None => Vec::new(),
    };
    let residuals: Vec<ResidualBlock<'_>> = f_pels
        .iter()
        .map(|(f, idx)| ResidualBlock {
            block_index: *idx,
            f_pel: f,
        })
        .collect();

    reconstruct_inter_macroblock(
        frame, references, mb_col, mb_row, field_dct, motion, &residuals,
    )
    .map_err(crate::Error::from)?;

    *previous_inter_direction = Some(InterDirection {
        forward: motion.forward,
        backward: motion.backward,
    });
    Ok(1)
}

/// Reconstruct one §7.6.6 skipped macroblock at `address`.
///
/// * **P-picture (§7.6.6.2)** — prediction as Frame-based with a
///   `(0, 0)` forward motion vector (PMVs were reset by the §7.6.3.4
///   side-effect in the motion walk).
/// * **B-picture (§7.6.6.4)** — prediction inheriting the previous
///   coded macroblock's direction and motion vectors (carried in
///   `previous_inter_direction`).
fn reconstruct_skipped_macroblock(
    frame: &mut FrameBuffer,
    references: ReferenceFrames<'_>,
    address: usize,
    mb_width: usize,
    picture_coding_type: PictureCodingType,
    previous_inter_direction: Option<InterDirection>,
) -> Result<usize> {
    let mb_col = address % mb_width;
    let mb_row = address / mb_width;

    let motion = match picture_coding_type {
        PictureCodingType::Predictive => FrameMotion::forward(MotionVectorPel::new(0, 0)),
        PictureCodingType::Bidirectional => {
            // §7.6.6.4: inherit the previous MB's direction + vectors.
            let prev = previous_inter_direction.unwrap_or(InterDirection {
                forward: Some(MotionVectorPel::new(0, 0)),
                backward: None,
            });
            FrameMotion {
                forward: prev.forward,
                backward: prev.backward,
            }
        }
        PictureCodingType::Intra => {
            // §7.6.6: no skipped macroblocks in a non-scalable
            // I-picture. The motion walk already rejected this, so this
            // arm is unreachable in a well-formed stream; treat it as a
            // zero-MV forward to stay total.
            FrameMotion::forward(MotionVectorPel::new(0, 0))
        }
    };

    reconstruct_inter_macroblock(frame, references, mb_col, mb_row, false, motion, &[])
        .map_err(crate::Error::from)?;
    Ok(1)
}

/// Translate the §7.6.3 reconstructed motion vectors of a frame-based
/// macroblock into a [`FrameMotion`]. The first reconstructed forward
/// / backward vector's `vector_prime` components are the luminance
/// motion vector the §7.6.4 reader consumes.
fn frame_motion_from_reconstructed(reconstructed: &ReconstructedMotionVectors) -> FrameMotion {
    let forward = reconstructed
        .forward
        .as_ref()
        .and_then(|v| v.first())
        .map(|rv| MotionVectorPel::new(rv.horizontal.vector_prime, rv.vertical.vector_prime));
    let backward = reconstructed
        .backward
        .as_ref()
        .and_then(|v| v.first())
        .map(|rv| MotionVectorPel::new(rv.horizontal.vector_prime, rv.vertical.vector_prime));
    FrameMotion { forward, backward }
}

/// Find the byte offset of the next `slice_start_code`
/// (`0x000001` prefix + `0x01..=0xAF`, Table 6-1) in `buf`, or `None`.
fn find_slice_start_code(buf: &[u8]) -> Option<usize> {
    buf.windows(4)
        .position(|w| w[0] == 0x00 && w[1] == 0x00 && w[2] == 0x01 && (0x01..=0xAF).contains(&w[3]))
}

/// Find the byte offset of the next start code of any kind, or `None`.
fn find_next_start_code(buf: &[u8]) -> Option<usize> {
    buf.windows(3)
        .position(|w| w[0] == 0x00 && w[1] == 0x00 && w[2] == 0x01)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pmv::ReconstructedComponent;
    use crate::sequence_extension::ChromaFormat;
    use crate::slice_macroblock_walk::ReconstructedVector;

    fn rc(v: i32) -> ReconstructedComponent {
        ReconstructedComponent {
            vector_prime: v,
            new_pmv: v,
            delta: 0,
            range: 16,
        }
    }

    #[test]
    fn frame_motion_picks_first_vector_components() {
        let recon = ReconstructedMotionVectors {
            forward: Some(vec![ReconstructedVector {
                horizontal: rc(6),
                vertical: rc(-4),
            }]),
            backward: None,
        };
        let m = frame_motion_from_reconstructed(&recon);
        assert_eq!(m.forward, Some(MotionVectorPel::new(6, -4)));
        assert_eq!(m.backward, None);
    }

    #[test]
    fn frame_motion_bidirectional() {
        let recon = ReconstructedMotionVectors {
            forward: Some(vec![ReconstructedVector {
                horizontal: rc(2),
                vertical: rc(2),
            }]),
            backward: Some(vec![ReconstructedVector {
                horizontal: rc(-2),
                vertical: rc(-2),
            }]),
        };
        let m = frame_motion_from_reconstructed(&recon);
        assert_eq!(m.forward, Some(MotionVectorPel::new(2, 2)));
        assert_eq!(m.backward, Some(MotionVectorPel::new(-2, -2)));
    }

    #[test]
    fn skipped_p_picture_is_forward_zero_mv() {
        let cf = ChromaFormat::Yuv420;
        let mut reference = FrameBuffer::new(16, 16, cf);
        for y in 0..16 {
            for x in 0..16 {
                reference.y.put_sample(x, y, 99);
            }
        }
        let mut frame = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let n = reconstruct_skipped_macroblock(
            &mut frame,
            refs,
            0,
            1,
            PictureCodingType::Predictive,
            None,
        )
        .unwrap();
        assert_eq!(n, 1);
        // Skipped P macroblock copies the reference verbatim.
        assert_eq!(frame.y.get(0, 0), Some(99));
        assert_eq!(frame.y.get(15, 15), Some(99));
    }
}
