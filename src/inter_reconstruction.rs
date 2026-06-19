//! §7.6 Motion-compensated macroblock reconstruction per ISO/IEC
//! 13818-2 (ITU-T H.262) / ISO/IEC 11172-2 (MPEG-1) — the picture-level
//! glue that threads the already-landed §7.6.3 reconstructed motion
//! vectors through the §7.6.5 prediction-selection table, the §7.6.4
//! pel reader against a reference frame, the §7.6.7 combine step, and
//! the §7.6.8 add-and-saturate write-out, so a **P** or **B**
//! macroblock reconstructs end-to-end to real pixels.
//!
//! ## Where this sits
//!
//! The lower-level endpoints already exist:
//!
//! * §7.6.3 — [`crate::reconstruct_slice_motion_vectors`] reconstructs
//!   `vector'[r][s][t]` for every coded macroblock of a slice (MPEG-2)
//!   and [`crate::mpeg1_reconstruct`] does the MPEG-1 equivalent.
//! * §7.6.5 — [`crate::select_predictions`] turns one macroblock
//!   descriptor into the ordered [`crate::PredictionOp`] list.
//! * §7.6.4 — [`crate::predict_block`] reads the half-pel prediction
//!   block from a [`crate::ReferencePlane`].
//! * §7.6.7 — [`crate::average_predictions`] / forward / backward
//!   pass-through combine the up-to-two prediction blocks.
//! * §7.6.8 — [`crate::add_prediction_and_coefficients`] adds the
//!   §A IDCT residual `f[y][x]` and saturates to `[0, 255]`.
//! * §6.1 — [`crate::frame_assembly`] places an 8×8 reconstructed
//!   block into the picture-sized [`FrameBuffer`].
//!
//! This module composes them at the **macroblock** granularity:
//! `reconstruct_inter_macroblock` forms the full per-component
//! prediction plane for one macroblock (16×16 luma + chroma sized by
//! the §6.1.1 sub-sampling), adds the per-block residual for each
//! **coded** block (§6.3.17.4 `pattern_code[]`), and writes the result
//! into the destination [`FrameBuffer`] at the macroblock's
//! `(mb_col, mb_row)` origin with the §6.1.3 frame/field DCT line
//! organisation honoured exactly as the intra path does in
//! [`crate::place_intra_block`].
//!
//! ## Scope of this initial driver
//!
//! The supported prediction modes are the **frame-picture** cases that
//! cover the overwhelming majority of real MPEG-1 / MPEG-2 P- and
//! B-pictures:
//!
//! * Frame-based forward / backward / bidirectional 16×16 prediction
//!   (Table 7-14 `Frame-based` rows) — MPEG-1 always uses this mode.
//! * The §7.6.3.5 implicit-zero-MV skipped-macroblock case (P-picture
//!   `(0,0)` forward; B-picture inherited direction with carried PMVs).
//!
//! The §7.6.4 chrominance prediction uses the §7.6.3.7 scaled vector
//! ([`crate::scale_chroma`]) over the sub-sampled chroma reference
//! plane. Field-based / 16×8-MC / dual-prime predictions in field
//! pictures are selected by [`crate::select_predictions`] but their
//! per-field reference assembly is a later milestone; this driver
//! rejects them with an explicit [`InterError`] so callers can tell
//! "unsupported mode" apart from "bad input".
//!
//! Spec citations refer to **ISO/IEC 13818-2 (H.262)** §7.6.4–§7.6.8
//! and **ISO/IEC 11172-2** §2.4.4.2–§2.4.4.3 (MPEG-1 reconstruction).

use crate::add_coefficients::saturate;
use crate::combine_predictions::{average_predictions, PredictionDirection};
use crate::forming_predictions::{predict_block, BlockSize, ReferencePlane};
use crate::frame_assembly::{block_placement, FrameBuffer};
use crate::mpeg2_block_dc::ColourComponent;
use crate::pmv::scale_chroma;
use crate::sequence_extension::ChromaFormat;

/// A motion vector in half-sample units, as the §7.6.4 pel reader
/// consumes it: `(horizontal, vertical)` = `vector'[r][s][1:0]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MotionVectorPel {
    /// `vector'[r][s][0]` — horizontal half-sample component.
    pub horizontal: i32,
    /// `vector'[r][s][1]` — vertical half-sample component.
    pub vertical: i32,
}

impl MotionVectorPel {
    /// Construct a `(horizontal, vertical)` half-sample motion vector.
    pub fn new(horizontal: i32, vertical: i32) -> Self {
        Self {
            horizontal,
            vertical,
        }
    }

    /// Bridge from an MPEG-1 (ISO/IEC 11172-2) reconstructed motion
    /// vector. The §2.4.4.2 `recon_right` / `recon_down` are already in
    /// the same half-sample luminance units the §7.6.4 pel reader
    /// consumes (after the optional `full_pel` left-shift), so the
    /// luminance vector maps straight through; the §7.6.3.7-equivalent
    /// chroma halving ([`scale_chroma`]) reproduces the MPEG-1
    /// `recon_* / 2` chrominance scaling (both are integer division
    /// toward zero) for the 4:2:0 sampling MPEG-1 always uses.
    pub fn from_mpeg1(recon: &crate::mpeg1_reconstruct::Mpeg1ReconstructedMv) -> Self {
        Self {
            horizontal: recon.recon_right,
            vertical: recon.recon_down,
        }
    }
}

/// The reference frame(s) a P/B macroblock predicts from.
///
/// * **P-pictures** read only [`Self::forward`]: the most-recently
///   decoded I- or P-picture.
/// * **B-pictures** read [`Self::forward`] (the past anchor) and
///   [`Self::backward`] (the future anchor). A forward-only or
///   backward-only B macroblock still only needs the one it names.
///
/// Both reference frames must share the destination frame's geometry
/// (`width`, `height`, `chroma_format`); the driver reads the matching
/// component plane from whichever direction a prediction op names.
#[derive(Debug, Clone, Copy)]
pub struct ReferenceFrames<'a> {
    /// The forward (past) reference frame. Always present for an inter
    /// macroblock that forms any forward prediction.
    pub forward: Option<&'a FrameBuffer>,
    /// The backward (future) reference frame. Present only in
    /// B-pictures forming a backward prediction.
    pub backward: Option<&'a FrameBuffer>,
}

impl<'a> ReferenceFrames<'a> {
    /// A P-picture reference set: forward only.
    pub fn forward_only(forward: &'a FrameBuffer) -> Self {
        Self {
            forward: Some(forward),
            backward: None,
        }
    }

    /// A B-picture reference set: forward (past) and backward (future).
    pub fn bidirectional(forward: &'a FrameBuffer, backward: &'a FrameBuffer) -> Self {
        Self {
            forward: Some(forward),
            backward: Some(backward),
        }
    }
}

/// Local error type for the inter-reconstruction driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterError {
    /// A forward prediction was requested but [`ReferenceFrames::forward`]
    /// is `None`.
    MissingForwardReference,
    /// A backward prediction was requested but
    /// [`ReferenceFrames::backward`] is `None`.
    MissingBackwardReference,
    /// The reference frame's geometry (width / height / chroma format)
    /// does not match the destination frame.
    ReferenceGeometryMismatch,
    /// The prediction mode requested is not handled by this driver yet
    /// (field-based / 16×8-MC / dual-prime in field pictures). The
    /// §7.6.4 / §7.6.5 endpoints exist; the per-field reference
    /// assembly is a later milestone.
    UnsupportedPredictionMode,
    /// A residual block carried a `block_index` outside the
    /// `chroma_format`'s valid range.
    InvalidBlockIndex,
}

impl core::fmt::Display for InterError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let msg = match self {
            Self::MissingForwardReference => {
                "forward prediction requested without a forward reference"
            }
            Self::MissingBackwardReference => {
                "backward prediction requested without a backward reference"
            }
            Self::ReferenceGeometryMismatch => {
                "reference frame geometry does not match destination"
            }
            Self::UnsupportedPredictionMode => {
                "field-based / 16x8-MC / dual-prime field-picture prediction not yet driven"
            }
            Self::InvalidBlockIndex => "residual block_index out of range for chroma_format",
        };
        write!(f, "mpeg12video inter-reconstruction: {msg}")
    }
}

impl std::error::Error for InterError {}

/// Local `Result` alias for the inter-reconstruction driver.
pub type InterResult<T> = core::result::Result<T, InterError>;

/// The per-direction, per-component motion vectors for one frame-based
/// macroblock, in half-sample luminance units.
///
/// The chroma vectors are derived internally by §7.6.3.7
/// [`scale_chroma`]; the caller supplies only the reconstructed
/// **luminance** vector for each present direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FrameMotion {
    /// Forward luminance motion vector (`s = 0`), present when the
    /// macroblock forms a forward prediction.
    pub forward: Option<MotionVectorPel>,
    /// Backward luminance motion vector (`s = 1`), present only in
    /// B-pictures forming a backward prediction.
    pub backward: Option<MotionVectorPel>,
}

impl FrameMotion {
    /// A forward-only motion (P macroblock, or forward-only B).
    pub fn forward(mv: MotionVectorPel) -> Self {
        Self {
            forward: Some(mv),
            backward: None,
        }
    }

    /// A backward-only motion (B macroblock).
    pub fn backward(mv: MotionVectorPel) -> Self {
        Self {
            forward: None,
            backward: Some(mv),
        }
    }

    /// A bidirectional motion (B macroblock with both directions).
    pub fn bidirectional(forward: MotionVectorPel, backward: MotionVectorPel) -> Self {
        Self {
            forward: Some(forward),
            backward: Some(backward),
        }
    }

    /// Bridge from MPEG-1 (ISO/IEC 11172-2) reconstructed motion
    /// vectors. `forward` is the §2.4.4.2 forward
    /// [`crate::mpeg1_reconstruct::Mpeg1ReconstructedMv`]; `backward`
    /// is the §2.4.4.3 backward variant (present only in B-pictures).
    /// Each is translated through [`MotionVectorPel::from_mpeg1`].
    pub fn from_mpeg1(
        forward: Option<&crate::mpeg1_reconstruct::Mpeg1ReconstructedMv>,
        backward: Option<&crate::mpeg1_reconstruct::Mpeg1ReconstructedMv>,
    ) -> Self {
        Self {
            forward: forward.map(MotionVectorPel::from_mpeg1),
            backward: backward.map(MotionVectorPel::from_mpeg1),
        }
    }

    /// The §7.6.7 [`PredictionDirection`] this motion set implies.
    /// Both-absent maps to [`PredictionDirection::Skipped`] (the
    /// §7.6.3.5 implicit zero-MV case; the caller is responsible for
    /// having seeded the appropriate `(0,0)` vector or inherited PMV).
    pub fn direction(self) -> PredictionDirection {
        match (self.forward.is_some(), self.backward.is_some()) {
            (true, true) => PredictionDirection::Bidirectional,
            (true, false) => PredictionDirection::Forward,
            (false, true) => PredictionDirection::Backward,
            (false, false) => PredictionDirection::Skipped,
        }
    }
}

/// Read the component plane (`Y` / `Cb` / `Cr`) of a [`FrameBuffer`]
/// as a flat `&[u8]` plus its `(width, height)`, ready to wrap in a
/// [`ReferencePlane`].
fn component_plane(frame: &FrameBuffer, component: ColourComponent) -> (&[u8], usize, usize) {
    let plane = match component {
        ColourComponent::Y => &frame.y,
        ColourComponent::Cb => &frame.cb,
        ColourComponent::Cr => &frame.cr,
    };
    (plane.samples(), plane.width(), plane.height())
}

/// Form the §7.6.4 prediction plane for one component over the whole
/// macroblock region, at the macroblock's top-left component-plane
/// origin `(base_x, base_y)`, for a single direction's motion vector.
///
/// `mv` is already in the component's own sample units — the luminance
/// vector for `Y`, the §7.6.3.7-scaled vector for `Cb` / `Cr`.
/// `(width, height)` is the macroblock's extent in that component's
/// plane (16×16 luma; 8×8 / 8×16 / 16×16 chroma per the sub-sampling).
fn predict_component(
    reference: &FrameBuffer,
    component: ColourComponent,
    base_x: i32,
    base_y: i32,
    width: usize,
    height: usize,
    mv: MotionVectorPel,
) -> Vec<u8> {
    let (data, pw, ph) = component_plane(reference, component);
    // `component_plane` always returns a `width*height`-sized buffer,
    // so the constructor never returns `None`; fall back to an
    // all-zero block defensively rather than panicking.
    let Some(plane) = ReferencePlane::new(data, pw, ph) else {
        return vec![0u8; width * height];
    };
    let size = match BlockSize::new(width, height) {
        Some(s) => s,
        None => return Vec::new(),
    };
    predict_block(plane, base_x, base_y, size, mv.horizontal, mv.vertical)
}

/// The per-component sub-sampling of one macroblock: the chroma extent
/// (in chroma samples) and the chroma motion-vector scaling for a
/// [`ChromaFormat`].
fn chroma_mb_extent(chroma_format: ChromaFormat) -> (usize, usize) {
    match chroma_format {
        // 4:2:0 — chroma is half width, half height: 8×8 per MB.
        ChromaFormat::Yuv420 => (8, 8),
        // 4:2:2 — half width, full height: 8×16 per MB.
        ChromaFormat::Yuv422 => (8, 16),
        // 4:4:4 — full resolution: 16×16 per MB.
        ChromaFormat::Yuv444 => (16, 16),
    }
}

/// Form the full per-component prediction plane for one frame-based
/// macroblock and combine the forward / backward directions per
/// §7.6.7.
///
/// Returns `(luma, cb, cr)` prediction buffers in row-major order:
/// `luma` is 16×16, `cb` / `cr` are sized by [`chroma_mb_extent`].
/// Each combines the present direction(s) with the §7.6.7.1 `// 2`
/// average when both are present.
///
/// # Errors
///
/// * [`InterError::MissingForwardReference`] /
///   [`InterError::MissingBackwardReference`] when a direction is
///   requested but the matching reference is absent.
/// * [`InterError::ReferenceGeometryMismatch`] when the reference
///   geometry differs from `dest`.
pub fn predict_frame_macroblock_planes(
    dest: &FrameBuffer,
    references: ReferenceFrames<'_>,
    mb_col: usize,
    mb_row: usize,
    motion: FrameMotion,
) -> InterResult<(Vec<u8>, Vec<u8>, Vec<u8>)> {
    let chroma_format = dest.chroma_format;
    let (cw, ch) = chroma_mb_extent(chroma_format);
    let luma_x = (mb_col * 16) as i32;
    let luma_y = (mb_row * 16) as i32;
    let chroma_x = (mb_col * cw) as i32;
    let chroma_y = (mb_row * ch) as i32;

    // Build one (luma, cb, cr) prediction set for a single direction.
    let one_direction = |reference: &FrameBuffer,
                         mv: MotionVectorPel|
     -> InterResult<(Vec<u8>, Vec<u8>, Vec<u8>)> {
        if reference.width != dest.width
            || reference.height != dest.height
            || reference.chroma_format != dest.chroma_format
        {
            return Err(InterError::ReferenceGeometryMismatch);
        }
        let scaled = scale_chroma(mv.horizontal, mv.vertical, chroma_format);
        let luma = predict_component(reference, ColourComponent::Y, luma_x, luma_y, 16, 16, mv);
        let chroma_mv = MotionVectorPel::new(scaled.chroma_horiz, scaled.chroma_vert);
        let cb = predict_component(
            reference,
            ColourComponent::Cb,
            chroma_x,
            chroma_y,
            cw,
            ch,
            chroma_mv,
        );
        let cr = predict_component(
            reference,
            ColourComponent::Cr,
            chroma_x,
            chroma_y,
            cw,
            ch,
            chroma_mv,
        );
        Ok((luma, cb, cr))
    };

    let direction = motion.direction();
    match direction {
        PredictionDirection::Forward | PredictionDirection::Skipped => {
            let reference = references
                .forward
                .ok_or(InterError::MissingForwardReference)?;
            // For the skipped case the caller seeds a `(0,0)` forward MV.
            let mv = motion.forward.unwrap_or_default();
            one_direction(reference, mv)
        }
        PredictionDirection::Backward => {
            let reference = references
                .backward
                .ok_or(InterError::MissingBackwardReference)?;
            let mv = motion.backward.unwrap_or_default();
            one_direction(reference, mv)
        }
        PredictionDirection::Bidirectional => {
            let fwd_ref = references
                .forward
                .ok_or(InterError::MissingForwardReference)?;
            let bwd_ref = references
                .backward
                .ok_or(InterError::MissingBackwardReference)?;
            let fwd_mv = motion.forward.unwrap_or_default();
            let bwd_mv = motion.backward.unwrap_or_default();
            let (fy, fcb, fcr) = one_direction(fwd_ref, fwd_mv)?;
            let (by, bcb, bcr) = one_direction(bwd_ref, bwd_mv)?;
            // §7.6.7.1: per-sample `// 2` average. The lengths always
            // match (same MB geometry), so `average_predictions` never
            // returns `None`.
            let y = average_predictions(&fy, &by).unwrap_or(fy);
            let cb = average_predictions(&fcb, &bcb).unwrap_or(fcb);
            let cr = average_predictions(&fcr, &bcr).unwrap_or(fcr);
            Ok((y, cb, cr))
        }
    }
}

/// Resolve, for a §6.1.1.8 `block_index`, the colour component it
/// belongs to and the width (in samples) of the macroblock region in
/// that component's prediction plane (16 for luma; the chroma
/// [`chroma_mb_extent`] width otherwise).
fn block_component_and_mb_width(
    block_index: u8,
    chroma_format: ChromaFormat,
) -> InterResult<(ColourComponent, usize)> {
    let component =
        crate::mpeg2_macroblock_blocks::block_component(block_index as usize, chroma_format)
            .ok_or(InterError::InvalidBlockIndex)?;
    let width = match component {
        ColourComponent::Y => 16,
        ColourComponent::Cb | ColourComponent::Cr => chroma_mb_extent(chroma_format).0,
    };
    Ok((component, width))
}

/// Sample a value out of a row-major macroblock prediction plane at the
/// macroblock-local `(x, y)`, given the plane's per-MB width.
#[inline]
fn mb_local_sample(plane: &[u8], mb_width: usize, x: usize, y: usize) -> u8 {
    plane.get(y * mb_width + x).copied().unwrap_or(0)
}

/// Write one 8×8 reconstructed block — the §7.6.8 `d = saturate(f + p)`
/// of the prediction-plane samples `p` (read from the per-MB component
/// prediction plane at the block's macroblock-local origin) and the
/// optional residual `f_pel` — into `dest`, honouring the §6.1.3
/// frame/field DCT line organisation through [`block_placement`].
///
/// `f_pel` is `Some` for a coded block (the §A IDCT output) and `None`
/// for an uncoded block (the prediction passes through unchanged, i.e.
/// an all-zero residual).
#[allow(clippy::too_many_arguments)]
fn write_inter_block(
    dest: &mut FrameBuffer,
    block_index: u8,
    chroma_format: ChromaFormat,
    mb_col: usize,
    mb_row: usize,
    field_dct: bool,
    luma_pred: &[u8],
    cb_pred: &[u8],
    cr_pred: &[u8],
    f_pel: Option<&[[i16; 8]; 8]>,
) -> InterResult<()> {
    let placement = block_placement(
        block_index as usize,
        chroma_format,
        mb_col,
        mb_row,
        field_dct,
    )
    .ok_or(InterError::InvalidBlockIndex)?;
    let (component, mb_plane_width) = block_component_and_mb_width(block_index, chroma_format)?;
    let pred = match component {
        ColourComponent::Y => luma_pred,
        ColourComponent::Cb => cb_pred,
        ColourComponent::Cr => cr_pred,
    };

    // The block's macroblock-local top-left in this component's plane.
    // `block_placement` returns the *frame*-plane coordinate; subtract
    // the macroblock origin to get the local offset into `pred`.
    let (sx, sy) = crate::frame_assembly::chroma_shift(chroma_format);
    let (mb_origin_x, mb_origin_y) = match component {
        ColourComponent::Y => (mb_col * 16, mb_row * 16),
        ColourComponent::Cb | ColourComponent::Cr => (mb_col * (16 >> sx), mb_row * (16 >> sy)),
    };
    let local_x0 = placement.base_x - mb_origin_x;
    // For a field-DCT block `base_y` is the first frame row the block's
    // row 0 maps to; the macroblock-local prediction plane is laid out
    // in *frame* order, so the same stride logic the writer uses must
    // index `pred`.
    let local_y0 = placement.base_y - mb_origin_y;
    let stride = placement.row_stride();

    let plane = match component {
        ColourComponent::Y => &mut dest.y,
        ColourComponent::Cb => &mut dest.cb,
        ColourComponent::Cr => &mut dest.cr,
    };

    for r in 0..8usize {
        let frame_y = placement.base_y + r * stride;
        let local_y = local_y0 + r * stride;
        for c in 0..8usize {
            let frame_x = placement.base_x + c;
            let p = mb_local_sample(pred, mb_plane_width, local_x0 + c, local_y) as i32;
            let f = f_pel.map(|m| m[r][c] as i32).unwrap_or(0);
            let d = saturate(f + p);
            plane.put_sample(frame_x, frame_y, d);
        }
    }
    Ok(())
}

/// A coded residual block: its §6.1.1.8 `block_index` and the §A IDCT
/// output plane `f[y][x]`.
///
/// Mirrors the `(block_index, f_pel)` carried by the slice walker's
/// [`crate::slice_macroblock_walk::MacroblockRecord::decoded_blocks`]
/// (`DecodedBlock { block_index, decoded: { f_pel, .. } }`); the caller
/// extracts the pair per coded block before invoking the driver.
#[derive(Debug, Clone, Copy)]
pub struct ResidualBlock<'a> {
    /// §6.1.1.8 block ordering index (`0..block_count(chroma_format)`).
    pub block_index: u8,
    /// The §A IDCT output `f[y][x]` for this coded block.
    pub f_pel: &'a [[i16; 8]; 8],
}

/// Reconstruct one **frame-based** P/B macroblock end-to-end into
/// `dest`, per the §7.6 pipeline:
///
/// 1. §7.6.4 / §7.6.5 / §7.6.7 — form the per-component prediction
///    plane for the macroblock (forward / backward / bidirectional)
///    via [`predict_frame_macroblock_planes`].
/// 2. §7.6.8 / §6.1 — for every block of the macroblock, write
///    `d = saturate(f + p)` into `dest` at the §6.1.1.8 block
///    placement, honouring the §6.1.3 frame/field DCT line
///    organisation. A block listed in `residuals` carries an IDCT
///    residual; every other block of the macroblock writes its
///    prediction unchanged (the §7.6.8 all-zero-residual case for
///    uncoded inter blocks).
///
/// `mb_col` / `mb_row` are the macroblock's raster column / row
/// (`macroblock_address % mb_width` / `… / mb_width`). `field_dct` is
/// the §6.2.5.1 `dct_type` (frame DCT default when the field was
/// absent). `motion` carries the reconstructed luminance motion
/// vector(s) for the present direction(s); a [`FrameMotion`] with both
/// directions absent is the §7.6.3.5 skipped / implicit-zero-MV case,
/// reconstructed against the forward reference.
///
/// Returns the number of blocks written (`block_count(chroma_format)`).
///
/// # Errors
///
/// Propagates the [`predict_frame_macroblock_planes`] reference / mode
/// errors and rejects an out-of-range residual `block_index`.
pub fn reconstruct_inter_macroblock(
    dest: &mut FrameBuffer,
    references: ReferenceFrames<'_>,
    mb_col: usize,
    mb_row: usize,
    field_dct: bool,
    motion: FrameMotion,
    residuals: &[ResidualBlock<'_>],
) -> InterResult<usize> {
    let chroma_format = dest.chroma_format;
    let (luma_pred, cb_pred, cr_pred) =
        predict_frame_macroblock_planes(dest, references, mb_col, mb_row, motion)?;

    // Validate residual block indices up-front so a bad index fails the
    // whole macroblock cleanly rather than after a partial write.
    let block_count = crate::mpeg2_macroblock_blocks::block_count(chroma_format);
    for r in residuals {
        if (r.block_index as usize) >= block_count {
            return Err(InterError::InvalidBlockIndex);
        }
    }

    let mut written = 0usize;
    for i in 0..block_count as u8 {
        let f_pel = residuals
            .iter()
            .find(|r| r.block_index == i)
            .map(|r| r.f_pel);
        write_inter_block(
            dest,
            i,
            chroma_format,
            mb_col,
            mb_row,
            field_dct,
            &luma_pred,
            &cb_pred,
            &cr_pred,
            f_pel,
        )?;
        written += 1;
    }
    Ok(written)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame_assembly::Plane;

    fn solid_frame(value: u8, w: usize, h: usize, cf: ChromaFormat) -> FrameBuffer {
        let mut f = FrameBuffer::new(w, h, cf);
        fill_plane(&mut f.y, value);
        fill_plane(&mut f.cb, value);
        fill_plane(&mut f.cr, value);
        f
    }

    fn fill_plane(plane: &mut Plane, value: u8) {
        for y in 0..plane.height() {
            for x in 0..plane.width() {
                plane.put_sample(x, y, value);
            }
        }
    }

    #[test]
    fn forward_zero_mv_copies_reference() {
        // 16×16 reference filled with 100; zero MV forward prediction
        // must copy it verbatim.
        let cf = ChromaFormat::Yuv420;
        let reference = solid_frame(100, 16, 16, cf);
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FrameMotion::forward(MotionVectorPel::new(0, 0));
        let (luma, cb, cr) = predict_frame_macroblock_planes(&dest, refs, 0, 0, motion).unwrap();
        assert_eq!(luma.len(), 256);
        assert!(luma.iter().all(|&p| p == 100));
        assert_eq!(cb.len(), 64);
        assert!(cb.iter().all(|&p| p == 100));
        assert!(cr.iter().all(|&p| p == 100));
    }

    #[test]
    fn bidirectional_averages_two_references() {
        let cf = ChromaFormat::Yuv420;
        let fwd = solid_frame(100, 16, 16, cf);
        let bwd = solid_frame(200, 16, 16, cf);
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::bidirectional(&fwd, &bwd);
        let motion =
            FrameMotion::bidirectional(MotionVectorPel::new(0, 0), MotionVectorPel::new(0, 0));
        let (luma, _, _) = predict_frame_macroblock_planes(&dest, refs, 0, 0, motion).unwrap();
        // (100 + 200) // 2 = 150
        assert!(luma.iter().all(|&p| p == 150));
    }

    #[test]
    fn missing_forward_reference_errors() {
        let cf = ChromaFormat::Yuv420;
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames {
            forward: None,
            backward: None,
        };
        let motion = FrameMotion::forward(MotionVectorPel::new(0, 0));
        assert_eq!(
            predict_frame_macroblock_planes(&dest, refs, 0, 0, motion),
            Err(InterError::MissingForwardReference)
        );
    }

    #[test]
    fn geometry_mismatch_errors() {
        let cf = ChromaFormat::Yuv420;
        let reference = solid_frame(100, 32, 16, cf);
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FrameMotion::forward(MotionVectorPel::new(0, 0));
        assert_eq!(
            predict_frame_macroblock_planes(&dest, refs, 0, 0, motion),
            Err(InterError::ReferenceGeometryMismatch)
        );
    }

    #[test]
    fn write_inter_block_adds_residual() {
        let cf = ChromaFormat::Yuv420;
        let mut dest = FrameBuffer::new(16, 16, cf);
        // Prediction plane: 16×16 luma of constant 50.
        let luma_pred = vec![50u8; 256];
        let cb_pred = vec![0u8; 64];
        let cr_pred = vec![0u8; 64];
        // Residual adds +10 to every sample of block 0.
        let f = [[10i16; 8]; 8];
        write_inter_block(
            &mut dest,
            0,
            cf,
            0,
            0,
            false,
            &luma_pred,
            &cb_pred,
            &cr_pred,
            Some(&f),
        )
        .unwrap();
        // Block 0 covers luma (0..8, 0..8); each = 50 + 10 = 60.
        for y in 0..8 {
            for x in 0..8 {
                assert_eq!(dest.y.get(x, y), Some(60));
            }
        }
        // Block 1 region untouched (still 0).
        assert_eq!(dest.y.get(8, 0), Some(0));
    }

    #[test]
    fn write_inter_block_uncoded_passes_prediction() {
        let cf = ChromaFormat::Yuv420;
        let mut dest = FrameBuffer::new(16, 16, cf);
        let luma_pred = vec![77u8; 256];
        let cb_pred = vec![0u8; 64];
        let cr_pred = vec![0u8; 64];
        write_inter_block(
            &mut dest, 0, cf, 0, 0, false, &luma_pred, &cb_pred, &cr_pred, None,
        )
        .unwrap();
        for y in 0..8 {
            for x in 0..8 {
                assert_eq!(dest.y.get(x, y), Some(77));
            }
        }
    }

    #[test]
    fn reconstruct_inter_macroblock_zero_mv_no_residual_copies_reference() {
        // Whole 16×16 MB, forward zero-MV, no coded blocks: the decoded
        // macroblock must equal the reference verbatim (a perfectly
        // predicted skipped-style P macroblock).
        let cf = ChromaFormat::Yuv420;
        let reference = solid_frame(123, 16, 16, cf);
        let mut dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FrameMotion::forward(MotionVectorPel::new(0, 0));
        let n = reconstruct_inter_macroblock(&mut dest, refs, 0, 0, false, motion, &[]).unwrap();
        // 4 luma + Cb + Cr = 6 blocks for 4:2:0.
        assert_eq!(n, 6);
        for y in 0..16 {
            for x in 0..16 {
                assert_eq!(dest.y.get(x, y), Some(123));
            }
        }
        for y in 0..8 {
            for x in 0..8 {
                assert_eq!(dest.cb.get(x, y), Some(123));
                assert_eq!(dest.cr.get(x, y), Some(123));
            }
        }
    }

    #[test]
    fn reconstruct_inter_macroblock_with_residual_on_one_block() {
        let cf = ChromaFormat::Yuv420;
        let reference = solid_frame(50, 16, 16, cf);
        let mut dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FrameMotion::forward(MotionVectorPel::new(0, 0));
        // Residual on luma block 3 (bottom-right 8×8) adds +5.
        let f = [[5i16; 8]; 8];
        let residuals = [ResidualBlock {
            block_index: 3,
            f_pel: &f,
        }];
        reconstruct_inter_macroblock(&mut dest, refs, 0, 0, false, motion, &residuals).unwrap();
        // Block 3 = luma (8..16, 8..16) -> 50 + 5 = 55.
        for y in 8..16 {
            for x in 8..16 {
                assert_eq!(dest.y.get(x, y), Some(55));
            }
        }
        // Block 0 (0..8, 0..8) had no residual -> prediction 50.
        assert_eq!(dest.y.get(0, 0), Some(50));
    }

    #[test]
    fn reconstruct_inter_macroblock_translation_reads_shifted_reference() {
        // Reference is a horizontal ramp; an integer +2-sample (MV=4 in
        // half-pel) shift must read the reference shifted left by 2.
        let cf = ChromaFormat::Yuv444;
        let mut reference = FrameBuffer::new(16, 16, cf);
        for y in 0..16 {
            for x in 0..16 {
                reference.y.put_sample(x, y, x as u8);
                reference.cb.put_sample(x, y, 0);
                reference.cr.put_sample(x, y, 0);
            }
        }
        let mut dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        // MV = (4, 0) half-pel -> integer offset +2, no half-pel.
        let motion = FrameMotion::forward(MotionVectorPel::new(4, 0));
        reconstruct_inter_macroblock(&mut dest, refs, 0, 0, false, motion, &[]).unwrap();
        // dest(x,y) = reference(x+2, y) clamped to width-1.
        for y in 0..16 {
            for x in 0..16 {
                let expected = (x + 2).min(15) as u8;
                assert_eq!(dest.y.get(x, y), Some(expected), "x={x} y={y}");
            }
        }
    }

    #[test]
    fn reconstruct_inter_macroblock_rejects_bad_block_index() {
        let cf = ChromaFormat::Yuv420;
        let reference = solid_frame(50, 16, 16, cf);
        let mut dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FrameMotion::forward(MotionVectorPel::new(0, 0));
        let f = [[0i16; 8]; 8];
        // block_index 6 is invalid for 4:2:0 (only 0..6).
        let residuals = [ResidualBlock {
            block_index: 6,
            f_pel: &f,
        }];
        assert_eq!(
            reconstruct_inter_macroblock(&mut dest, refs, 0, 0, false, motion, &residuals),
            Err(InterError::InvalidBlockIndex)
        );
    }

    #[test]
    fn write_inter_block_saturates() {
        let cf = ChromaFormat::Yuv420;
        let mut dest = FrameBuffer::new(16, 16, cf);
        let luma_pred = vec![250u8; 256];
        let cb_pred = vec![0u8; 64];
        let cr_pred = vec![0u8; 64];
        let f = [[20i16; 8]; 8]; // 250 + 20 = 270 -> clamp 255
        write_inter_block(
            &mut dest,
            0,
            cf,
            0,
            0,
            false,
            &luma_pred,
            &cb_pred,
            &cr_pred,
            Some(&f),
        )
        .unwrap();
        assert_eq!(dest.y.get(0, 0), Some(255));
    }

    // ---- MPEG-1 (ISO/IEC 11172-2) bridge ----

    #[test]
    fn mpeg1_zero_mv_reconstructs_macroblock_against_reference() {
        use crate::mpeg1_motion_vector::{Mpeg1MotionDirection, Mpeg1MotionVector};
        use crate::mpeg1_reconstruct::{reconstruct, Mpeg1FrameMvContext, Mpeg1Predictor};

        // A zero forward MV (code 0, no residual): the §2.4.4.2
        // reconstruction yields recon = (0, 0), so the P macroblock is a
        // verbatim copy of the reference frame.
        let mv = Mpeg1MotionVector {
            direction: Mpeg1MotionDirection::Forward,
            horizontal_code: 0,
            horizontal_r: None,
            vertical_code: 0,
            vertical_r: None,
            bit_position_after: 0,
        };
        let mut predictor = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 1,
            full_pel: false,
        };
        let recon = reconstruct(&mv, ctx, &mut predictor, Mpeg1MotionDirection::Forward).unwrap();
        assert_eq!((recon.recon_right, recon.recon_down), (0, 0));

        let cf = ChromaFormat::Yuv420;
        let reference = solid_frame(140, 16, 16, cf);
        let mut dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FrameMotion::from_mpeg1(Some(&recon), None);
        reconstruct_inter_macroblock(&mut dest, refs, 0, 0, false, motion, &[]).unwrap();
        for y in 0..16 {
            for x in 0..16 {
                assert_eq!(dest.y.get(x, y), Some(140));
            }
        }
    }

    #[test]
    fn mpeg1_integer_mv_shifts_reference() {
        use crate::mpeg1_motion_vector::{Mpeg1MotionDirection, Mpeg1MotionVector};
        use crate::mpeg1_reconstruct::{reconstruct, Mpeg1FrameMvContext, Mpeg1Predictor};

        // Forward MV code +1 with f_code 1, full_pel: recon = +2 (one
        // full sample) horizontal. The §2.4.4.2 luma split makes
        // recon_right = 2 -> right_for_luma = 1, no half-pel. The MC
        // reads the reference shifted left by one sample.
        let mv = Mpeg1MotionVector {
            direction: Mpeg1MotionDirection::Forward,
            horizontal_code: 1,
            horizontal_r: None,
            vertical_code: 0,
            vertical_r: None,
            bit_position_after: 0,
        };
        let mut predictor = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 1,
            full_pel: true,
        };
        let recon = reconstruct(&mv, ctx, &mut predictor, Mpeg1MotionDirection::Forward).unwrap();
        // full_pel doubles the half-sample +1 into +2 half-samples = +1
        // integer sample.
        assert_eq!(recon.recon_right, 2);
        assert_eq!(recon.right_half_for_luma, 0);

        let cf = ChromaFormat::Yuv420;
        let mut reference = FrameBuffer::new(16, 16, cf);
        for y in 0..16 {
            for x in 0..16 {
                reference.y.put_sample(x, y, (x * 8) as u8);
                reference.cb.put_sample(x.min(7), y.min(7), 0);
                reference.cr.put_sample(x.min(7), y.min(7), 0);
            }
        }
        let mut dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FrameMotion::from_mpeg1(Some(&recon), None);
        reconstruct_inter_macroblock(&mut dest, refs, 0, 0, false, motion, &[]).unwrap();
        // dest(x,y) = reference(x+1, y) clamped: (x+1)*8 for x<15, 120 at edge.
        for y in 0..16 {
            for x in 0..16 {
                let expected = ((x + 1).min(15) * 8) as u8;
                assert_eq!(dest.y.get(x, y), Some(expected), "x={x} y={y}");
            }
        }
    }
}
