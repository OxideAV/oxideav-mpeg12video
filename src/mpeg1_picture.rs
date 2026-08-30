//! MPEG-1 (ISO/IEC 11172-2) picture-level decode drivers: the §2.4.4
//! reconstruction of a whole I / P / B picture to real pixels, the
//! 11172-2 counterpart of [`crate::decode_intra_picture`] /
//! [`crate::decode_inter_picture`].
//!
//! ## What differs from the ISO/IEC 13818-2 drivers
//!
//! * **Block layer** — the §2.4.3.7 coefficient walk + §2.4.4.1/.2
//!   dequantisation (odd-value mismatch rule, `dct_dc_*_past` +
//!   `past_intra_address` DC chain, the B.5f escape encoding) run
//!   through [`crate::mpeg1_block_decoder`], selected inside the slice
//!   walker by `SliceWalkContext::mpeg1`.
//! * **Motion vectors** — §2.4.4.2 / §2.4.4.3 reconstruction with the
//!   `recon_*_prev` predictor pair (`crate::mpeg1_reconstruct`):
//!   * P-pictures: the predictor resets at slice start and after any
//!     macroblock that carried no forward vector (skipped, intra, or
//!     `macroblock_motion_forward == 0`); a macroblock with no
//!     forward data predicts with a zero vector.
//!   * B-pictures: the predictors reset only at slice start and after
//!     an intra macroblock; a direction with no vector data inherits
//!     the predictor value (`recon_* = recon_*_prev`).
//!   * `full_pel_*_vector` doubles the reconstructed vector into
//!     integer-pel units (§2.4.4.2 final shift).
//! * **Skipped macroblocks (§2.4.4.4)** — P: zero reconstructed
//!   vector, no coefficients (and the predictor resets); B: same
//!   macroblock type (prediction directions) as the prior macroblock
//!   with zero differentials — i.e. the predictor vectors — and no
//!   coefficients.
//!
//! The §2.4.4.2 chrominance scaling (`recon_* / 2`) is reproduced by
//! the shared §7.6.3.7-equivalent chroma halving in the prediction
//! core (integer division toward zero on both paths; MPEG-1 is always
//! 4:2:0), so the reconstructed luminance vector bridges through
//! [`crate::inter_reconstruction::FrameMotion::from_mpeg1`] into the
//! same motion-compensation pipeline the MPEG-2 frame-based path uses.

use crate::frame_assembly::{place_intra_macroblock, FrameBuffer};
use crate::inter_reconstruction::{
    reconstruct_inter_macroblock, FrameMotion, ReferenceFrames, ResidualBlock,
};
use crate::mpeg1_motion_vector::{Mpeg1MotionDirection, Mpeg1MotionVector};
use crate::mpeg1_reconstruct::{
    reconstruct, reconstruct_absent, reconstruct_zero, Mpeg1FrameMvContext, Mpeg1Predictor,
    Mpeg1ReconstructedMv,
};
use crate::picture_header::{PictureCodingType, PictureStructure};
use crate::quant_matrix_extension::QuantiserMatrixState;
use crate::sequence_extension::ChromaFormat;
use crate::slice_header::{SliceContext, SliceHeader};
use crate::slice_macroblock_walk::{walk_slice_at, MacroblockRecord, SliceWalkContext};
use crate::{Error, Result};

/// The per-picture parameters shared by every MPEG-1 picture type:
/// the geometry plus the sequence-header quantiser matrices
/// (§2.4.2.3 — one intra and one non-intra matrix, shared by
/// luminance and chrominance).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mpeg1PictureParams {
    /// `horizontal_size` from the sequence header.
    pub width: usize,
    /// `vertical_size` from the sequence header.
    pub height: usize,
    /// `intra_quant[m][n]` in raster order — the loaded
    /// `intra_quantizer_matrix` or the §2.4.2.3 default.
    pub intra_quant: [[u8; 8]; 8],
    /// `non_intra_quant[m][n]` in raster order — the loaded matrix or
    /// the §2.4.2.3 all-16 default.
    pub non_intra_quant: [[u8; 8]; 8],
}

impl Mpeg1PictureParams {
    /// Macroblocks per row (`Ceil(width / 16)`).
    pub fn mb_width(&self) -> usize {
        self.width.div_ceil(16)
    }
}

/// The extra picture-header fields a P / B picture carries (§2.4.3.4).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mpeg1InterParams {
    /// Shared geometry + matrices.
    pub base: Mpeg1PictureParams,
    /// `picture_coding_type`: `Predictive` or `Bidirectional`.
    pub picture_coding_type: PictureCodingType,
    /// `forward_f_code` (`1..=7`).
    pub forward_f_code: u8,
    /// `full_pel_forward_vector`.
    pub full_pel_forward_vector: bool,
    /// `backward_f_code` (`1..=7`; only consumed for B-pictures).
    pub backward_f_code: u8,
    /// `full_pel_backward_vector`.
    pub full_pel_backward_vector: bool,
}

/// Build the MPEG-1 slice-walk context for one slice.
fn mpeg1_walk_context(
    params: &Mpeg1PictureParams,
    picture_coding_type: PictureCodingType,
    quantiser_scale_code: u8,
    f_fwd: u8,
    f_bwd: u8,
) -> SliceWalkContext {
    let mut ctx = SliceWalkContext::first_slice_with_block_decoding(
        params.mb_width() as u32,
        0, // overwritten per slice below
        picture_coding_type,
        quantiser_scale_code,
        PictureStructure::Frame,
        true, // MPEG-1 has no field DCT / field prediction
        f_fwd,
        f_fwd,
        f_bwd,
        f_bwd,
        false, // no concealment motion vectors in ISO/IEC 11172-2
        ChromaFormat::Yuv420,
        false, // no intra_vlc_format
        false, // no alternate_scan
        0,     // 8-bit DC precision equivalent
        false, // linear quantiser scale
    );
    ctx.mpeg1 = true;
    ctx.quantiser_matrices = QuantiserMatrixState {
        intra_luma: params.intra_quant,
        non_intra_luma: params.non_intra_quant,
        intra_chroma: params.intra_quant,
        non_intra_chroma: params.non_intra_quant,
    };
    ctx
}

/// Decode a whole MPEG-1 **I** picture into a [`FrameBuffer`].
///
/// `picture` runs from the first `slice_start_code` (or from the
/// picture header — the driver scans forward) up to the next
/// picture / GOP / sequence boundary.
///
/// # Errors
/// Propagates slice-header / macroblock-walk / block-decode errors.
pub fn decode_mpeg1_intra_picture(
    picture: &[u8],
    params: &Mpeg1PictureParams,
) -> Result<(FrameBuffer, usize)> {
    let mut frame = FrameBuffer::new(params.width, params.height, ChromaFormat::Yuv420);
    let mb_width = params.mb_width();
    let slice_ctx = SliceContext::non_scalable(params.height as u32);

    let mut placed = 0usize;
    let mut offset = 0usize;
    while let Some(rel) = find_slice_start_code(&picture[offset..]) {
        let start = offset + rel;
        let body = &picture[start..];
        let end = find_next_start_code(&body[4..])
            .map(|p| p + 4)
            .unwrap_or(body.len());
        let slice_buf = &body[..end];

        let header = SliceHeader::parse(slice_buf, slice_ctx)?;
        let mb_row = u32::from(header.slice_vertical_position) - 1;
        let mut ctx = mpeg1_walk_context(
            params,
            PictureCodingType::Intra,
            header.quantiser_scale_code,
            7,
            7,
        );
        ctx.mb_row = mb_row;

        let walk = walk_slice_at(slice_buf, header.body_bit_position, ctx)?;
        for record in &walk.macroblocks {
            placed += usize::from(
                place_intra_macroblock(&mut frame, record, mb_width, ChromaFormat::Yuv420) > 0,
            );
        }
        offset = start + end;
    }
    Ok((frame, placed))
}

/// The §2.4.4.4 B-picture skip state: the prior macroblock's
/// prediction directions.
#[derive(Debug, Clone, Copy)]
struct PriorDirections {
    forward: bool,
    backward: bool,
}

/// Decode a whole MPEG-1 **P** or **B** picture into a
/// [`FrameBuffer`], predicting from `references` (§2.4.4.2 /
/// §2.4.4.3).
///
/// # Errors
/// Propagates slice / walk / block errors; a P/B macroblock that
/// needs a missing reference surfaces
/// [`crate::inter_reconstruction::InterError`] wrapped in
/// [`crate::Error`].
pub fn decode_mpeg1_inter_picture(
    picture: &[u8],
    params: &Mpeg1InterParams,
    references: ReferenceFrames<'_>,
) -> Result<(FrameBuffer, usize)> {
    if params.picture_coding_type == PictureCodingType::Intra {
        return Err(Error::InvalidBitstream(
            "decode_mpeg1_inter_picture: use decode_mpeg1_intra_picture for I-pictures",
        ));
    }
    let is_b = params.picture_coding_type == PictureCodingType::Bidirectional;
    let fwd_ctx = Mpeg1FrameMvContext {
        f_code: params.forward_f_code,
        full_pel: params.full_pel_forward_vector,
    };
    let bwd_ctx = Mpeg1FrameMvContext {
        f_code: params.backward_f_code,
        full_pel: params.full_pel_backward_vector,
    };

    let mut frame = FrameBuffer::new(params.base.width, params.base.height, ChromaFormat::Yuv420);
    let mb_width = params.base.mb_width();
    let slice_ctx = SliceContext::non_scalable(params.base.height as u32);

    let mut placed = 0usize;
    let mut offset = 0usize;
    while let Some(rel) = find_slice_start_code(&picture[offset..]) {
        let start = offset + rel;
        let body = &picture[start..];
        let end = find_next_start_code(&body[4..])
            .map(|p| p + 4)
            .unwrap_or(body.len());
        let slice_buf = &body[..end];

        let header = SliceHeader::parse(slice_buf, slice_ctx)?;
        let mb_row = u32::from(header.slice_vertical_position) - 1;
        let mut ctx = mpeg1_walk_context(
            &params.base,
            params.picture_coding_type,
            header.quantiser_scale_code,
            params.forward_f_code,
            params.backward_f_code,
        );
        ctx.mb_row = mb_row;

        let walk = walk_slice_at(slice_buf, header.body_bit_position, ctx)?;

        // §2.4.4.2 / §2.4.4.3: the predictors reset at the start of
        // each slice.
        let mut fwd_pred = Mpeg1Predictor::new();
        let mut bwd_pred = Mpeg1Predictor::new();
        let mut prior: Option<PriorDirections> = None;

        for record in &walk.macroblocks {
            // §2.4.4.4: the skip run preceding this coded macroblock.
            let skipped = record.skipped_macroblock_count;
            for k in 0..skipped {
                let address = record.macroblock_address - skipped + k;
                let motion = if is_b {
                    // B: same directions as the prior macroblock,
                    // zero differentials -> the predictor vectors.
                    let dirs = prior.unwrap_or(PriorDirections {
                        forward: true,
                        backward: false,
                    });
                    let fwd = dirs.forward.then(|| reconstruct_absent(fwd_ctx, &fwd_pred));
                    let bwd = dirs
                        .backward
                        .then(|| reconstruct_absent(bwd_ctx, &bwd_pred));
                    FrameMotion::from_mpeg1(fwd.as_ref(), bwd.as_ref())
                } else {
                    // P: reconstructed vector zero (and the predictor
                    // resets — the *next* coded MB sees prev = 0).
                    let zero = reconstruct_zero(&mut fwd_pred);
                    FrameMotion::from_mpeg1(Some(&zero), None)
                };
                reconstruct_inter_macroblock(
                    &mut frame,
                    references,
                    address as usize % mb_width,
                    address as usize / mb_width,
                    false,
                    motion,
                    &[],
                )
                .map_err(Error::from)?;
                placed += 1;
            }

            placed += reconstruct_mpeg1_macroblock(
                &mut frame,
                references,
                record,
                mb_width,
                is_b,
                fwd_ctx,
                bwd_ctx,
                &mut fwd_pred,
                &mut bwd_pred,
                &mut prior,
            )?;
        }
        offset = start + end;
    }
    Ok((frame, placed))
}

/// Bridge one wire-parsed `motion_vectors(s)` entry into the
/// [`Mpeg1MotionVector`] shape §2.4.4.2 reconstructs from.
fn wire_to_mpeg1_mv(
    record: &MacroblockRecord,
    direction: Mpeg1MotionDirection,
) -> Option<Mpeg1MotionVector> {
    let wire = match direction {
        Mpeg1MotionDirection::Forward => record.motion_vectors_forward.as_ref(),
        Mpeg1MotionDirection::Backward => record.motion_vectors_backward.as_ref(),
    }?;
    let entry = wire.entries.first()?;
    let mv = &entry.motion_vector;
    Some(Mpeg1MotionVector {
        direction,
        horizontal_code: mv.motion_code_horiz,
        horizontal_r: mv.motion_residual_horiz,
        vertical_code: mv.motion_code_vert,
        vertical_r: mv.motion_residual_vert,
        bit_position_after: mv.bit_position_after,
    })
}

/// Reconstruct one coded MPEG-1 macroblock (intra or inter) and apply
/// the §2.4.4.2 / §2.4.4.3 predictor lifecycle.
#[allow(clippy::too_many_arguments)]
fn reconstruct_mpeg1_macroblock(
    frame: &mut FrameBuffer,
    references: ReferenceFrames<'_>,
    record: &MacroblockRecord,
    mb_width: usize,
    is_b: bool,
    fwd_ctx: Mpeg1FrameMvContext,
    bwd_ctx: Mpeg1FrameMvContext,
    fwd_pred: &mut Mpeg1Predictor,
    bwd_pred: &mut Mpeg1Predictor,
    prior: &mut Option<PriorDirections>,
) -> Result<usize> {
    let mt = &record.macroblock_type;

    if mt.macroblock_intra {
        // §2.4.4.1 intra path. §2.4.4.2: an intra macroblock carries
        // no motion vector information, so the P forward predictor
        // resets; §2.4.4.3: in B-pictures both predictors reset after
        // an intra macroblock.
        place_intra_macroblock(frame, record, mb_width, ChromaFormat::Yuv420);
        fwd_pred.reset();
        if is_b {
            bwd_pred.reset();
        }
        // §2.4.4.4: "a skipped macroblock shall not follow an
        // intra-coded macroblock" — clear the prior-direction state
        // so a malformed stream falls back to the forward-zero
        // default rather than replaying stale directions.
        *prior = None;
        return Ok(1);
    }

    // Forward vector (§2.4.4.2, and §2.4.4.3 first step).
    let forward: Option<Mpeg1ReconstructedMv> = if mt.macroblock_motion_forward {
        let mv = wire_to_mpeg1_mv(record, Mpeg1MotionDirection::Forward).ok_or(
            Error::InvalidBitstream(
                "mpeg1 macroblock_motion_forward set but no motion vector payload (§2.4.3.6)",
            ),
        )?;
        Some(reconstruct(
            &mv,
            fwd_ctx,
            fwd_pred,
            Mpeg1MotionDirection::Forward,
        )?)
    } else if !is_b {
        // P-picture, no forward data: zero vector, predictor reset.
        Some(reconstruct_zero(fwd_pred))
    } else {
        // B-picture: the forward *value* is inherited by the
        // predictor chain, but no forward prediction is formed for
        // this macroblock (the macroblock_type row decides).
        None
    };

    // Backward vector (§2.4.4.3 second step; B-pictures only).
    let backward: Option<Mpeg1ReconstructedMv> = if mt.macroblock_motion_backward {
        let mv = wire_to_mpeg1_mv(record, Mpeg1MotionDirection::Backward).ok_or(
            Error::InvalidBitstream(
                "mpeg1 macroblock_motion_backward set but no motion vector payload (§2.4.3.6)",
            ),
        )?;
        Some(reconstruct(
            &mv,
            bwd_ctx,
            bwd_pred,
            Mpeg1MotionDirection::Backward,
        )?)
    } else {
        None
    };

    let motion = FrameMotion::from_mpeg1(forward.as_ref(), backward.as_ref());

    // Residual blocks the walker decoded through the §2.4.3.7 /
    // §2.4.4 pipeline.
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

    let mb_col = record.macroblock_address as usize % mb_width;
    let mb_row = record.macroblock_address as usize / mb_width;
    reconstruct_inter_macroblock(frame, references, mb_col, mb_row, false, motion, &residuals)
        .map_err(Error::from)?;

    *prior = Some(PriorDirections {
        forward: mt.macroblock_motion_forward,
        backward: mt.macroblock_motion_backward,
    });
    Ok(1)
}

/// Decode a whole MPEG-1 **D** picture (dc intra-coded,
/// `picture_coding_type == 4`, §2.4.3.4) into a [`FrameBuffer`].
///
/// Per §2.4.2.7 / §2.4.2.8 a D-picture's macroblock layer is maximally
/// lean: every macroblock is the single Table B.2d `macroblock_type`
/// codeword `'1'` (intra, no quant / motion / pattern), each of its
/// six 4:2:0 blocks carries **only** the `dct_dc_size_*` +
/// `dct_dc_differential` prelude (the AC walk and `end_of_block` are
/// gated by `if (picture_coding_type != 4)`), and the macroblock is
/// terminated by the 1-bit `end_of_macroblock` marker (value `'1'`).
/// The §2.4.4.1 `dct_dc_*_past` predictor chain applies unchanged.
///
/// §2.4.4.4 requires every macroblock of an all-intra picture to be
/// coded, so a `macroblock_address_increment != 1` is rejected.
///
/// # Errors
/// Propagates slice-header / VLC / dequantiser errors;
/// [`Error::InvalidBitstream`] on a `macroblock_type` other than the
/// Table B.2d `'1'`, a zero `end_of_macroblock` bit, or a skipped
/// macroblock.
pub fn decode_mpeg1_d_picture(
    picture: &[u8],
    params: &Mpeg1PictureParams,
) -> Result<(FrameBuffer, usize)> {
    use crate::dequantize::{finalise_intra_macroblock, IntraDcPredictors};
    use crate::frame_assembly::{block_placement, place_intra_block};
    use crate::mb_address_increment::{MbAddressIncrement, MbAddressIncrementContext};
    use oxideav_core::bits::BitReader;

    let mut frame = FrameBuffer::new(params.width, params.height, ChromaFormat::Yuv420);
    let mb_width = params.mb_width();
    let slice_ctx = SliceContext::non_scalable(params.height as u32);

    let mut placed = 0usize;
    let mut offset = 0usize;
    while let Some(rel) = find_slice_start_code(&picture[offset..]) {
        let start = offset + rel;
        let body = &picture[start..];
        let end = find_next_start_code(&body[4..])
            .map(|p| p + 4)
            .unwrap_or(body.len());
        let slice_buf = &body[..end];

        let header = SliceHeader::parse(slice_buf, slice_ctx)?;
        let mb_row = u32::from(header.slice_vertical_position) - 1;

        let mut br = BitReader::new(slice_buf);
        let to_skip =
            u32::try_from(header.body_bit_position).map_err(|_| crate::Error::ShortHeader)?;
        br.skip(to_skip).map_err(|_| crate::Error::ShortHeader)?;

        // §2.4.4.1: predictors reset at slice start.
        let mut predictors = IntraDcPredictors::at_slice_start();
        // Address of the macroblock about to decode; the first
        // macroblock of the slice sits at the head of `mb_row`.
        let mut address = mb_row as usize * mb_width;
        let mut is_first = true;

        // §2.4.2.6 do-while: decode macroblocks while the next 23
        // bits (zero-extended at a truncated tail, §5.2.3) are not
        // the all-zero stop pattern.
        loop {
            let stop = match br.peek_u32(23) {
                Ok(0) => true,
                Ok(_) => false,
                Err(_) => {
                    let remaining = br.bits_remaining().min(22) as u32;
                    remaining == 0 || matches!(br.peek_u32(remaining), Ok(0))
                }
            };
            if stop {
                break;
            }

            // §2.4.2.7: stuffing/escape chains + the increment VLC.
            let increment = MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg1())?;
            if increment.value != 1 {
                return Err(Error::InvalidBitstream(
                    "D-picture macroblock_address_increment != 1: every macroblock of an all-intra picture shall be coded (§2.4.4.4)",
                ));
            }
            if !is_first {
                address += 1;
            }
            is_first = false;

            // Table B.2d: the single macroblock_type codeword '1'.
            let mb_type_bit = br.read_bit().map_err(|_| crate::Error::ShortHeader)?;
            if !mb_type_bit {
                return Err(Error::InvalidBitstream(
                    "D-picture macroblock_type: only the Table B.2d codeword '1' exists",
                ));
            }

            // §2.4.2.8: six DC-only blocks (macroblock_intra == 1 sets
            // every pattern_code[i], §2.4.3.6).
            let mb_col = address % mb_width;
            let mb_row_now = address / mb_width;
            for i in 0..6u8 {
                let block = crate::mpeg1_block_decoder::decode_d_block(
                    &mut br,
                    i,
                    header.quantiser_scale_code,
                    &params.intra_quant,
                    &mut predictors,
                    address as i32,
                )?;
                if let Some(placement) =
                    block_placement(i as usize, ChromaFormat::Yuv420, mb_col, mb_row_now, false)
                {
                    place_intra_block(&mut frame, placement, &block.f_pel);
                }
            }
            finalise_intra_macroblock(&mut predictors, address as i32);

            // §2.4.2.7: end_of_macroblock — "a bit which is set to 1"
            // (§2.4.3.6).
            let eom = br.read_bit().map_err(|_| crate::Error::ShortHeader)?;
            if !eom {
                return Err(Error::InvalidBitstream(
                    "D-picture end_of_macroblock: bit shall be '1' (§2.4.3.6)",
                ));
            }
            placed += 1;
        }
        offset = start + end;
    }
    Ok((frame, placed))
}

/// Find the byte offset of the next `slice_start_code`
/// (`0x00000101..=0x000001AF`) in `buf`.
fn find_slice_start_code(buf: &[u8]) -> Option<usize> {
    buf.windows(4)
        .position(|w| w[0] == 0x00 && w[1] == 0x00 && w[2] == 0x01 && (0x01..=0xAF).contains(&w[3]))
}

/// Find the byte offset of the next start code prefix (any
/// `0x000001??`) in `buf`.
fn find_next_start_code(buf: &[u8]) -> Option<usize> {
    buf.windows(3)
        .position(|w| w[0] == 0x00 && w[1] == 0x00 && w[2] == 0x01)
}
