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
//! In addition, the **frame-picture field-based** prediction (Table
//! 7-14 `Field-based` rows) is now driven by
//! `reconstruct_field_based_macroblock`: each present direction carries
//! a top-field and a bottom-field luminance vector, the top-field vector
//! predicts the macroblock's even (top-field) frame lines from the top
//! reference field and the bottom-field vector its odd (bottom-field)
//! lines from the bottom reference field (via the §7.6.4
//! [`crate::FieldReference`] field view), the two directions combine per
//! §7.6.7.2, and the result writes out through the same residual-add /
//! block placement path as the frame-based driver.
//!
//! The §7.6.4 chrominance prediction uses the §7.6.3.7 scaled vector
//! ([`crate::scale_chroma`]) over the sub-sampled chroma reference
//! plane (per-field for the field-based path). 16×8-MC and dual-prime
//! field-picture predictions are selected by
//! [`crate::select_predictions`] but their per-field reference assembly
//! is a later milestone.
//!
//! Spec citations refer to **ISO/IEC 13818-2 (H.262)** §7.6.4–§7.6.8
//! and **ISO/IEC 11172-2** §2.4.4.2–§2.4.4.3 (MPEG-1 reconstruction).

use crate::add_coefficients::saturate;
use crate::combine_predictions::{average_predictions, PredictionDirection};
use crate::dual_prime::FieldParity;
use crate::forming_predictions::{
    predict_block, predict_field_block, BlockSize, FieldReference, ReferencePlane,
};
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
    pub const fn new(horizontal: i32, vertical: i32) -> Self {
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

/// One field motion vector together with the reference field it reads.
///
/// §7.6.4: *"In the case of field-based prediction and 16x8 MC an
/// additional bit, motion_vertical_field_select, is encoded to indicate
/// which field to use. If motion_vertical_field_select is zero then the
/// prediction is taken from the top reference field. If
/// motion_vertical_field_select is one then the prediction is taken from
/// the bottom reference field."* The vector's horizontal component is in
/// half-sample luminance units; the vertical component is in
/// **field**-sample half units (the §7.6.3 reconstruction already
/// produces field vectors in those units).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FieldVector {
    /// The luminance motion vector.
    pub vector: MotionVectorPel,
    /// The reference field the §7.6.4 reader samples
    /// (`motion_vertical_field_select`: `0` → `Top`, `1` → `Bottom`).
    pub reference_field: FieldParity,
}

impl FieldVector {
    /// Bundle a vector with its `motion_vertical_field_select`d
    /// reference field.
    pub fn new(vector: MotionVectorPel, reference_field: FieldParity) -> Self {
        Self {
            vector,
            reference_field,
        }
    }
}

/// The per-field, per-direction luminance motion vectors for one
/// **frame-picture field-based** macroblock (Table 7-14 `Field-based`
/// rows).
///
/// Each present direction carries a [`FieldVector`] predicting the
/// macroblock's top-field (even) frame lines and one predicting its
/// bottom-field (odd) frame lines — the first / second vector of the
/// §7.6.5 bitstream order. Each vector reads the reference field its
/// own §6.3.17.2 `motion_vertical_field_select` flag names (§7.6.4);
/// the destination field does **not** imply the source field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FieldBasedMotion {
    /// Forward `(top_field_vector, bottom_field_vector)` — present when
    /// the macroblock forms a forward prediction.
    pub forward: Option<(FieldVector, FieldVector)>,
    /// Backward `(top_field_vector, bottom_field_vector)` — present only
    /// in B-pictures forming a backward prediction.
    pub backward: Option<(FieldVector, FieldVector)>,
}

impl FieldBasedMotion {
    /// A forward-only field-based motion.
    pub fn forward(top: FieldVector, bottom: FieldVector) -> Self {
        Self {
            forward: Some((top, bottom)),
            backward: None,
        }
    }

    /// A backward-only field-based motion (B macroblock).
    pub fn backward(top: FieldVector, bottom: FieldVector) -> Self {
        Self {
            forward: None,
            backward: Some((top, bottom)),
        }
    }

    /// A bidirectional field-based motion (B macroblock, both
    /// directions).
    pub fn bidirectional(
        forward_top: FieldVector,
        forward_bottom: FieldVector,
        backward_top: FieldVector,
        backward_bottom: FieldVector,
    ) -> Self {
        Self {
            forward: Some((forward_top, forward_bottom)),
            backward: Some((backward_top, backward_bottom)),
        }
    }

    /// The §7.6.7 [`PredictionDirection`] this field-based motion set
    /// implies (both-absent maps to [`PredictionDirection::Skipped`]).
    pub fn direction(self) -> PredictionDirection {
        match (self.forward.is_some(), self.backward.is_some()) {
            (true, true) => PredictionDirection::Bidirectional,
            (true, false) => PredictionDirection::Forward,
            (false, true) => PredictionDirection::Backward,
            (false, false) => PredictionDirection::Skipped,
        }
    }
}

/// Form one component's full macroblock prediction plane for a
/// **frame-picture field-based** prediction in a single direction.
///
/// The returned buffer is laid out in **frame** order (`height` rows,
/// `width` columns, row-major) so the same residual-add / block-write
/// plumbing the frame-based path uses applies unchanged: even rows are
/// filled from the top reference field using `top_mv`, odd rows from the
/// bottom reference field using `bottom_mv`. Each field block is
/// `width × (height / 2)` and is interleaved back at stride 2.
///
/// `(base_x, base_y)` is the macroblock's top-left **frame** coordinate
/// in this component's plane. The field motion vectors' vertical
/// components are in field-sample units; their horizontal components are
/// shared with the frame grid. The field block's top-left field line is
/// `base_y / 2` (top field) — i.e. the field line co-located with the
/// macroblock's first top-field frame row.
#[allow(clippy::too_many_arguments)]
fn predict_field_component_one_direction(
    reference: &FrameBuffer,
    component: ColourComponent,
    base_x: i32,
    base_y: i32,
    width: usize,
    height: usize,
    top: FieldVector,
    bottom: FieldVector,
) -> Vec<u8> {
    let (data, pw, ph) = component_plane(reference, component);
    let Some(plane) = ReferencePlane::new(data, pw, ph) else {
        return vec![0u8; width * height];
    };
    let half = height / 2;
    let Some(size) = BlockSize::new(width, half) else {
        return vec![0u8; width * height];
    };
    // The macroblock's first top-field frame row is base_y; its field
    // line index is base_y / 2 (base_y is even for a frame-picture
    // macroblock origin, which is a multiple of 16 / chroma height).
    // The co-located field line index is the same in either reference
    // field (the two fields share the field-line coordinate system).
    let field_top_line = base_y / 2;
    let mut out = vec![0u8; width * height];
    for (dest_parity, fv) in [(FieldParity::Top, top), (FieldParity::Bottom, bottom)] {
        // §7.6.4: the *source* field is the one this vector's
        // motion_vertical_field_select names — not the destination
        // field's own parity.
        let Some(field) = FieldReference::new(plane, fv.reference_field.index()) else {
            continue;
        };
        let block = predict_field_block(
            field,
            base_x,
            field_top_line,
            size,
            fv.vector.horizontal,
            fv.vector.vertical,
        );
        // Interleave the field block back into frame rows: field line r
        // of the destination parity maps to frame row 2*r + parity,
        // relative to the macroblock origin that even = top, odd =
        // bottom.
        let row_off = dest_parity.index();
        for r in 0..half {
            let frame_row = 2 * r + row_off;
            let src = &block[r * width..r * width + width];
            let dst = &mut out[frame_row * width..frame_row * width + width];
            dst.copy_from_slice(src);
        }
    }
    out
}

/// Form the full per-component prediction planes (luma, cb, cr) for one
/// **frame-picture field-based** macroblock and combine the
/// forward/backward directions per §7.6.7.2.
///
/// Returns `(luma, cb, cr)` in frame-order row-major layout (16×16
/// luma; chroma sized by [`chroma_mb_extent`]) ready for the same
/// residual-add / block-write path the frame-based driver uses. The
/// chroma field vectors are derived per direction/field by §7.6.3.7
/// [`scale_chroma`].
///
/// # Errors
///
/// Mirrors [`predict_frame_macroblock_planes`]: missing reference for a
/// requested direction, or a reference geometry mismatch.
pub fn predict_field_based_macroblock_planes(
    dest: &FrameBuffer,
    references: ReferenceFrames<'_>,
    mb_col: usize,
    mb_row: usize,
    motion: FieldBasedMotion,
) -> InterResult<(Vec<u8>, Vec<u8>, Vec<u8>)> {
    let chroma_format = dest.chroma_format;
    let (cw, ch) = chroma_mb_extent(chroma_format);
    let luma_x = (mb_col * 16) as i32;
    let luma_y = (mb_row * 16) as i32;
    let chroma_x = (mb_col * cw) as i32;
    let chroma_y = (mb_row * ch) as i32;

    let one_direction = |reference: &FrameBuffer,
                         top: FieldVector,
                         bottom: FieldVector|
     -> InterResult<(Vec<u8>, Vec<u8>, Vec<u8>)> {
        if reference.width != dest.width
            || reference.height != dest.height
            || reference.chroma_format != dest.chroma_format
        {
            return Err(InterError::ReferenceGeometryMismatch);
        }
        // §7.6.3.7 chroma scaling is applied to each field vector
        // independently (the horizontal halving / vertical halving of
        // the luminance field vector; the vertical component stays in
        // field-sample units of the chroma field). The chroma
        // prediction reads the same reference field the luminance
        // vector selected.
        let top_chroma = scale_chroma(top.vector.horizontal, top.vector.vertical, chroma_format);
        let bottom_chroma = scale_chroma(
            bottom.vector.horizontal,
            bottom.vector.vertical,
            chroma_format,
        );
        let top_cmv = FieldVector::new(
            MotionVectorPel::new(top_chroma.chroma_horiz, top_chroma.chroma_vert),
            top.reference_field,
        );
        let bottom_cmv = FieldVector::new(
            MotionVectorPel::new(bottom_chroma.chroma_horiz, bottom_chroma.chroma_vert),
            bottom.reference_field,
        );

        let luma = predict_field_component_one_direction(
            reference,
            ColourComponent::Y,
            luma_x,
            luma_y,
            16,
            16,
            top,
            bottom,
        );
        let cb = predict_field_component_one_direction(
            reference,
            ColourComponent::Cb,
            chroma_x,
            chroma_y,
            cw,
            ch,
            top_cmv,
            bottom_cmv,
        );
        let cr = predict_field_component_one_direction(
            reference,
            ColourComponent::Cr,
            chroma_x,
            chroma_y,
            cw,
            ch,
            top_cmv,
            bottom_cmv,
        );
        Ok((luma, cb, cr))
    };

    // Fallback pair for an absent direction: zero vectors reading each
    // destination field's own parity (same-parity `(0, 0)` prediction).
    let zero_pair = (
        FieldVector::new(MotionVectorPel::new(0, 0), FieldParity::Top),
        FieldVector::new(MotionVectorPel::new(0, 0), FieldParity::Bottom),
    );

    match motion.direction() {
        PredictionDirection::Forward | PredictionDirection::Skipped => {
            let reference = references
                .forward
                .ok_or(InterError::MissingForwardReference)?;
            let (top, bottom) = motion.forward.unwrap_or(zero_pair);
            one_direction(reference, top, bottom)
        }
        PredictionDirection::Backward => {
            let reference = references
                .backward
                .ok_or(InterError::MissingBackwardReference)?;
            let (top, bottom) = motion.backward.unwrap_or(zero_pair);
            one_direction(reference, top, bottom)
        }
        PredictionDirection::Bidirectional => {
            let fwd_ref = references
                .forward
                .ok_or(InterError::MissingForwardReference)?;
            let bwd_ref = references
                .backward
                .ok_or(InterError::MissingBackwardReference)?;
            let (ft, fb) = motion.forward.unwrap_or(zero_pair);
            let (bt, bb) = motion.backward.unwrap_or(zero_pair);
            let (fy, fcb, fcr) = one_direction(fwd_ref, ft, fb)?;
            let (by, bcb, bcr) = one_direction(bwd_ref, bt, bb)?;
            let y = average_predictions(&fy, &by).unwrap_or(fy);
            let cb = average_predictions(&fcb, &bcb).unwrap_or(fcb);
            let cr = average_predictions(&fcr, &bcr).unwrap_or(fcr);
            Ok((y, cb, cr))
        }
    }
}

/// Reconstruct one **frame-picture field-based** P/B macroblock
/// end-to-end into `dest`, per the §7.6 pipeline (Table 7-14
/// `Field-based` rows): form the per-field prediction planes
/// ([`predict_field_based_macroblock_planes`]), then add the §A IDCT
/// residual per coded block and write out with the §6.1.3 frame/field
/// DCT line organisation honoured — identical to
/// [`reconstruct_inter_macroblock`] once the prediction planes are
/// formed, because the field-based planes are returned in frame-row
/// order.
///
/// Returns the number of blocks written (`block_count(chroma_format)`).
///
/// # Errors
///
/// Propagates [`predict_field_based_macroblock_planes`] reference / mode
/// errors and rejects an out-of-range residual `block_index`.
pub fn reconstruct_field_based_macroblock(
    dest: &mut FrameBuffer,
    references: ReferenceFrames<'_>,
    mb_col: usize,
    mb_row: usize,
    field_dct: bool,
    motion: FieldBasedMotion,
    residuals: &[ResidualBlock<'_>],
) -> InterResult<usize> {
    let chroma_format = dest.chroma_format;
    let (luma_pred, cb_pred, cr_pred) =
        predict_field_based_macroblock_planes(dest, references, mb_col, mb_row, motion)?;

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

/// The per-direction luminance motion vector **and** its selected
/// reference field for one **field-picture** macroblock using simple
/// field prediction (Table 7-13 `Field-based` rows).
///
/// Within a field picture every prediction is a field prediction
/// (§7.6.1): the whole macroblock is a single 16×16 field block read
/// from **one** chosen reference field. The field is selected by the
/// §6.3.17.2 `motion_vertical_field_select` flag carried alongside the
/// vector — `Top` when the flag is `0`, `Bottom` when `1` (§7.6.4).
///
/// The vertical component of [`MotionVectorPel`] is already in
/// field-sample units (the §7.6.3 reconstruction for a field-picture
/// macroblock produces a field vector directly), so the block reads
/// contiguously from the chosen [`FieldReference`] with no interleave —
/// the destination is itself one field of a reconstructed frame
/// (§3-defn "reconstructed picture").
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FieldPictureMotion {
    /// Forward `(luminance_vector, selected_reference_field)` — present
    /// when the macroblock forms a forward prediction.
    pub forward: Option<(MotionVectorPel, FieldParity)>,
    /// Backward `(luminance_vector, selected_reference_field)` — present
    /// only in B-field-pictures forming a backward prediction.
    pub backward: Option<(MotionVectorPel, FieldParity)>,
}

impl FieldPictureMotion {
    /// A forward-only field-picture motion reading reference field
    /// `field`.
    pub fn forward(mv: MotionVectorPel, field: FieldParity) -> Self {
        Self {
            forward: Some((mv, field)),
            backward: None,
        }
    }

    /// A backward-only field-picture motion (B-field-picture).
    pub fn backward(mv: MotionVectorPel, field: FieldParity) -> Self {
        Self {
            forward: None,
            backward: Some((mv, field)),
        }
    }

    /// A bidirectional field-picture motion (B-field-picture, both
    /// directions), each direction selecting its own reference field.
    pub fn bidirectional(
        forward: MotionVectorPel,
        forward_field: FieldParity,
        backward: MotionVectorPel,
        backward_field: FieldParity,
    ) -> Self {
        Self {
            forward: Some((forward, forward_field)),
            backward: Some((backward, backward_field)),
        }
    }

    /// The §7.6.7 [`PredictionDirection`] this motion set implies
    /// (both-absent maps to [`PredictionDirection::Skipped`], the
    /// §7.6.3.5 implicit zero-MV case).
    pub fn direction(self) -> PredictionDirection {
        match (self.forward.is_some(), self.backward.is_some()) {
            (true, true) => PredictionDirection::Bidirectional,
            (true, false) => PredictionDirection::Forward,
            (false, true) => PredictionDirection::Backward,
            (false, false) => PredictionDirection::Skipped,
        }
    }
}

/// Form one component's full macroblock prediction block for a
/// **field-picture** simple field prediction in a single direction.
///
/// `reference` is the most-recently-decoded reference **frame** (top
/// and bottom fields interleaved). `parity` selects which of its two
/// fields the §7.6.4 pel reader samples. `(base_x, base_y)` is the
/// macroblock's top-left coordinate **in field-sample units** of this
/// component's field plane (the destination picture being one field).
///
/// The returned buffer is `width × height` row-major in field order —
/// exactly the macroblock's own footprint in the destination field
/// plane, so the same residual-add / block-write path the frame-based
/// path uses applies unchanged (there is no frame/field DCT interleave
/// inside a field picture, §6.1.3 Table 6-19).
#[allow(clippy::too_many_arguments)]
fn predict_field_picture_component(
    reference: &FrameBuffer,
    component: ColourComponent,
    parity: FieldParity,
    base_x: i32,
    base_y: i32,
    width: usize,
    height: usize,
    mv: MotionVectorPel,
) -> Vec<u8> {
    let (data, pw, ph) = component_plane(reference, component);
    let Some(plane) = ReferencePlane::new(data, pw, ph) else {
        return vec![0u8; width * height];
    };
    let Some(field) = FieldReference::new(plane, parity.index()) else {
        return vec![0u8; width * height];
    };
    let Some(size) = BlockSize::new(width, height) else {
        return vec![0u8; width * height];
    };
    predict_field_block(field, base_x, base_y, size, mv.horizontal, mv.vertical)
}

/// Form the full per-component prediction planes (luma, cb, cr) for one
/// **field-picture** simple-field-prediction macroblock and combine the
/// forward / backward directions per §7.6.7.2 (the `// 2` average).
///
/// `dest` is the destination **field** buffer (one field of a frame;
/// its `height` is the field height). `references` carries the
/// reference **frame(s)** whose individual fields are selected by each
/// direction's [`FieldParity`]. `(mb_col, mb_row)` index the macroblock
/// in the field picture's own macroblock grid.
///
/// Returns `(luma, cb, cr)` in field-order row-major layout (16×16
/// luma; chroma sized by [`chroma_mb_extent`] — in a field picture the
/// per-field chroma block is the full extent per §7.6.7.2) ready for
/// the residual-add / block-write path.
///
/// # Errors
///
/// Mirrors [`predict_frame_macroblock_planes`]: a missing reference for
/// a requested direction. The reference is a full frame, so its height
/// is twice the destination field height — only the chroma format and
/// the field-vs-frame height relationship are validated.
pub fn predict_field_picture_macroblock_planes(
    dest: &FrameBuffer,
    references: ReferenceFrames<'_>,
    mb_col: usize,
    mb_row: usize,
    motion: FieldPictureMotion,
) -> InterResult<(Vec<u8>, Vec<u8>, Vec<u8>)> {
    let chroma_format = dest.chroma_format;
    let (cw, ch) = chroma_mb_extent(chroma_format);
    let luma_x = (mb_col * 16) as i32;
    let luma_y = (mb_row * 16) as i32;
    let chroma_x = (mb_col * cw) as i32;
    let chroma_y = (mb_row * ch) as i32;

    let one_direction = |reference: &FrameBuffer,
                         mv: MotionVectorPel,
                         parity: FieldParity|
     -> InterResult<(Vec<u8>, Vec<u8>, Vec<u8>)> {
        // The reference is a full frame; the destination is one field
        // (half its height). Validate the format match and that the
        // reference frame is exactly twice the field height (the field
        // picture's two fields constitute the coded frame, §6.1.1).
        if reference.chroma_format != dest.chroma_format
            || reference.width != dest.width
            || reference.height != dest.height.saturating_mul(2)
        {
            return Err(InterError::ReferenceGeometryMismatch);
        }
        // §7.6.3.7 chroma scaling: the vertical component stays in
        // field-sample units of the chosen chroma field.
        let scaled = scale_chroma(mv.horizontal, mv.vertical, chroma_format);
        let chroma_mv = MotionVectorPel::new(scaled.chroma_horiz, scaled.chroma_vert);
        let luma = predict_field_picture_component(
            reference,
            ColourComponent::Y,
            parity,
            luma_x,
            luma_y,
            16,
            16,
            mv,
        );
        let cb = predict_field_picture_component(
            reference,
            ColourComponent::Cb,
            parity,
            chroma_x,
            chroma_y,
            cw,
            ch,
            chroma_mv,
        );
        let cr = predict_field_picture_component(
            reference,
            ColourComponent::Cr,
            parity,
            chroma_x,
            chroma_y,
            cw,
            ch,
            chroma_mv,
        );
        Ok((luma, cb, cr))
    };

    match motion.direction() {
        PredictionDirection::Forward | PredictionDirection::Skipped => {
            let reference = references
                .forward
                .ok_or(InterError::MissingForwardReference)?;
            let (mv, parity) = motion
                .forward
                .unwrap_or((MotionVectorPel::default(), FieldParity::Top));
            one_direction(reference, mv, parity)
        }
        PredictionDirection::Backward => {
            let reference = references
                .backward
                .ok_or(InterError::MissingBackwardReference)?;
            let (mv, parity) = motion
                .backward
                .unwrap_or((MotionVectorPel::default(), FieldParity::Top));
            one_direction(reference, mv, parity)
        }
        PredictionDirection::Bidirectional => {
            let fwd_ref = references
                .forward
                .ok_or(InterError::MissingForwardReference)?;
            let bwd_ref = references
                .backward
                .ok_or(InterError::MissingBackwardReference)?;
            let (fwd_mv, fwd_parity) = motion
                .forward
                .unwrap_or((MotionVectorPel::default(), FieldParity::Top));
            let (bwd_mv, bwd_parity) = motion
                .backward
                .unwrap_or((MotionVectorPel::default(), FieldParity::Top));
            let (fy, fcb, fcr) = one_direction(fwd_ref, fwd_mv, fwd_parity)?;
            let (by, bcb, bcr) = one_direction(bwd_ref, bwd_mv, bwd_parity)?;
            let y = average_predictions(&fy, &by).unwrap_or(fy);
            let cb = average_predictions(&fcb, &bcb).unwrap_or(fcb);
            let cr = average_predictions(&fcr, &bcr).unwrap_or(fcr);
            Ok((y, cb, cr))
        }
    }
}

/// Reconstruct one **field-picture** simple-field-prediction P/B
/// macroblock end-to-end into `dest` (one field of a frame), per the
/// §7.6 pipeline (Table 7-13 `Field-based` rows): form the per-direction
/// field prediction planes ([`predict_field_picture_macroblock_planes`])
/// then add the §A IDCT residual per coded block and write out.
///
/// There is no frame/field DCT distinction inside a field picture
/// (§6.1.3 Table 6-19), so blocks place contiguously in the field plane;
/// `field_dct` is therefore fixed `false` for the write-out.
///
/// Returns the number of blocks written (`block_count(chroma_format)`).
///
/// # Errors
///
/// Propagates [`predict_field_picture_macroblock_planes`] reference
/// errors and rejects an out-of-range residual `block_index`.
pub fn reconstruct_field_picture_macroblock(
    dest: &mut FrameBuffer,
    references: ReferenceFrames<'_>,
    mb_col: usize,
    mb_row: usize,
    motion: FieldPictureMotion,
    residuals: &[ResidualBlock<'_>],
) -> InterResult<usize> {
    let chroma_format = dest.chroma_format;
    let (luma_pred, cb_pred, cr_pred) =
        predict_field_picture_macroblock_planes(dest, references, mb_col, mb_row, motion)?;

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
            false,
            &luma_pred,
            &cb_pred,
            &cr_pred,
            f_pel,
        )?;
        written += 1;
    }
    Ok(written)
}

/// The per-region, per-direction luminance motion vectors **and** their
/// selected reference fields for one **field-picture 16×8-MC** macroblock
/// (Table 7-13 `16x8 MC` rows).
///
/// 16×8 motion compensation forms two separate predictions for a
/// macroblock (§7.6.7.3): `vector'[0]` predicts the **upper** 16×8
/// luminance region (the macroblock's top eight lines), `vector'[1]` the
/// **lower** 16×8 region. Each region carries its own §6.3.17.2
/// `motion_vertical_field_select` flag (the §7.6.4 NOTE: *"in the case of
/// field-based prediction and 16x8 MC an additional bit,
/// `motion_vertical_field_select`, is encoded to indicate which field to
/// use"*), so each region's vector reads from its own chosen reference
/// field — `Top` when the flag is `0`, `Bottom` when `1`.
///
/// As in a field picture the destination is one field of a frame, so the
/// reconstructed regions place contiguously with no frame/field
/// interleave (§6.1.3 Table 6-19): the upper region occupies the
/// macroblock's first half of lines, the lower region the second half.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FieldPicture16x8Motion {
    /// Forward `[(upper_vector, upper_field), (lower_vector,
    /// lower_field)]` — present when the macroblock forms a forward
    /// prediction (`vector'[0][0]` / `vector'[1][0]`).
    pub forward: Option<[(MotionVectorPel, FieldParity); 2]>,
    /// Backward `[(upper_vector, upper_field), (lower_vector,
    /// lower_field)]` — present only in a B-field-picture forming a
    /// backward prediction (`vector'[0][1]` / `vector'[1][1]`).
    pub backward: Option<[(MotionVectorPel, FieldParity); 2]>,
}

impl FieldPicture16x8Motion {
    /// A forward-only 16×8 motion: `upper` predicts the top 16×8 region,
    /// `lower` the bottom 16×8 region.
    pub fn forward(
        upper: (MotionVectorPel, FieldParity),
        lower: (MotionVectorPel, FieldParity),
    ) -> Self {
        Self {
            forward: Some([upper, lower]),
            backward: None,
        }
    }

    /// A backward-only 16×8 motion (B-field-picture).
    pub fn backward(
        upper: (MotionVectorPel, FieldParity),
        lower: (MotionVectorPel, FieldParity),
    ) -> Self {
        Self {
            forward: None,
            backward: Some([upper, lower]),
        }
    }

    /// The §7.6.7 [`PredictionDirection`] this motion set implies
    /// (both-absent maps to [`PredictionDirection::Skipped`]).
    pub fn direction(self) -> PredictionDirection {
        match (self.forward.is_some(), self.backward.is_some()) {
            (true, true) => PredictionDirection::Bidirectional,
            (true, false) => PredictionDirection::Forward,
            (false, true) => PredictionDirection::Backward,
            (false, false) => PredictionDirection::Skipped,
        }
    }
}

/// Form one component's full macroblock prediction block for a
/// **field-picture 16×8-MC** prediction in a single direction.
///
/// The component plane is split horizontally into an upper and a lower
/// 16×8 region (luma; the chroma region is the full component width ×
/// half its height per §7.6.7.3 — 4:2:0 → 8×4, 4:2:2 → 8×8, 4:4:4 →
/// 16×8). Each region is predicted independently: the upper region reads
/// the `upper_field` of `reference` with `upper_mv`, the lower region the
/// `lower_field` with `lower_mv`. The two regions are stacked into the
/// macroblock's `width × height` footprint with no interleave (the
/// destination is one field plane).
///
/// `(base_x, base_y)` is the macroblock's top-left coordinate in
/// field-sample units of this component's field plane; the lower region's
/// field-line origin is `base_y + height/2`.
#[allow(clippy::too_many_arguments)]
fn predict_field_picture_16x8_component(
    reference: &FrameBuffer,
    component: ColourComponent,
    base_x: i32,
    base_y: i32,
    width: usize,
    height: usize,
    upper: (MotionVectorPel, FieldParity),
    lower: (MotionVectorPel, FieldParity),
) -> Vec<u8> {
    let (data, pw, ph) = component_plane(reference, component);
    let Some(plane) = ReferencePlane::new(data, pw, ph) else {
        return vec![0u8; width * height];
    };
    let half = height / 2;
    let Some(size) = BlockSize::new(width, half) else {
        return vec![0u8; width * height];
    };
    let mut out = vec![0u8; width * height];
    // Each region is a `width × half` block read from its chosen field.
    for (region, (mv, parity)) in [upper, lower].into_iter().enumerate() {
        let Some(field) = FieldReference::new(plane, parity.index()) else {
            continue;
        };
        // Upper region occupies field lines [base_y, base_y+half); the
        // lower region [base_y+half, base_y+height).
        let region_top = base_y + (region * half) as i32;
        let block =
            predict_field_block(field, base_x, region_top, size, mv.horizontal, mv.vertical);
        let row_off = region * half;
        for r in 0..half {
            let dst_row = row_off + r;
            let src = &block[r * width..r * width + width];
            let dst = &mut out[dst_row * width..dst_row * width + width];
            dst.copy_from_slice(src);
        }
    }
    out
}

/// Form the full per-component prediction planes (luma, cb, cr) for one
/// **field-picture 16×8-MC** macroblock and combine the forward /
/// backward directions per §7.6.7.2 (the `// 2` average).
///
/// `dest` is the destination **field** buffer (one field of a frame).
/// `references` carries the reference **frame(s)**; each region's
/// [`FieldParity`] selects which of a reference frame's two fields is
/// sampled. Returns `(luma, cb, cr)` in field-order row-major layout
/// ready for the residual-add / block-write path. Chroma vectors are
/// derived per region via §7.6.3.7 [`scale_chroma`].
///
/// # Errors
///
/// Mirrors [`predict_field_picture_macroblock_planes`]: a missing
/// reference for a requested direction or a reference geometry mismatch
/// (the reference frame's height must be twice the field height).
pub fn predict_field_picture_16x8_macroblock_planes(
    dest: &FrameBuffer,
    references: ReferenceFrames<'_>,
    mb_col: usize,
    mb_row: usize,
    motion: FieldPicture16x8Motion,
) -> InterResult<(Vec<u8>, Vec<u8>, Vec<u8>)> {
    let chroma_format = dest.chroma_format;
    let (cw, ch) = chroma_mb_extent(chroma_format);
    let luma_x = (mb_col * 16) as i32;
    let luma_y = (mb_row * 16) as i32;
    let chroma_x = (mb_col * cw) as i32;
    let chroma_y = (mb_row * ch) as i32;

    // §7.6.3.5 fallback for the (unreachable) skipped 16×8 case — a
    // field-picture skip uses simple field prediction, never 16×8, so the
    // `Skipped` arm is defensive only.
    const ZERO_REGIONS: [(MotionVectorPel, FieldParity); 2] = [
        (MotionVectorPel::new(0, 0), FieldParity::Top),
        (MotionVectorPel::new(0, 0), FieldParity::Top),
    ];

    let scale = |mv: MotionVectorPel| -> MotionVectorPel {
        let s = scale_chroma(mv.horizontal, mv.vertical, chroma_format);
        MotionVectorPel::new(s.chroma_horiz, s.chroma_vert)
    };

    let one_direction = |reference: &FrameBuffer,
                         regions: [(MotionVectorPel, FieldParity); 2]|
     -> InterResult<(Vec<u8>, Vec<u8>, Vec<u8>)> {
        if reference.chroma_format != dest.chroma_format
            || reference.width != dest.width
            || reference.height != dest.height.saturating_mul(2)
        {
            return Err(InterError::ReferenceGeometryMismatch);
        }
        let [(up_mv, up_par), (lo_mv, lo_par)] = regions;
        let up_cmv = scale(up_mv);
        let lo_cmv = scale(lo_mv);
        let luma = predict_field_picture_16x8_component(
            reference,
            ColourComponent::Y,
            luma_x,
            luma_y,
            16,
            16,
            (up_mv, up_par),
            (lo_mv, lo_par),
        );
        let cb = predict_field_picture_16x8_component(
            reference,
            ColourComponent::Cb,
            chroma_x,
            chroma_y,
            cw,
            ch,
            (up_cmv, up_par),
            (lo_cmv, lo_par),
        );
        let cr = predict_field_picture_16x8_component(
            reference,
            ColourComponent::Cr,
            chroma_x,
            chroma_y,
            cw,
            ch,
            (up_cmv, up_par),
            (lo_cmv, lo_par),
        );
        Ok((luma, cb, cr))
    };

    match motion.direction() {
        PredictionDirection::Forward | PredictionDirection::Skipped => {
            let reference = references
                .forward
                .ok_or(InterError::MissingForwardReference)?;
            let regions = motion.forward.unwrap_or(ZERO_REGIONS);
            one_direction(reference, regions)
        }
        PredictionDirection::Backward => {
            let reference = references
                .backward
                .ok_or(InterError::MissingBackwardReference)?;
            let regions = motion.backward.unwrap_or(ZERO_REGIONS);
            one_direction(reference, regions)
        }
        PredictionDirection::Bidirectional => {
            let fwd_ref = references
                .forward
                .ok_or(InterError::MissingForwardReference)?;
            let bwd_ref = references
                .backward
                .ok_or(InterError::MissingBackwardReference)?;
            let (fy, fcb, fcr) = one_direction(fwd_ref, motion.forward.unwrap_or(ZERO_REGIONS))?;
            let (by, bcb, bcr) = one_direction(bwd_ref, motion.backward.unwrap_or(ZERO_REGIONS))?;
            let y = average_predictions(&fy, &by).unwrap_or(fy);
            let cb = average_predictions(&fcb, &bcb).unwrap_or(fcb);
            let cr = average_predictions(&fcr, &bcr).unwrap_or(fcr);
            Ok((y, cb, cr))
        }
    }
}

/// Reconstruct one **field-picture 16×8-MC** P/B macroblock end-to-end
/// into `dest` (one field of a frame), per the §7.6 pipeline (Table 7-13
/// `16x8 MC` rows): form the two-region prediction planes
/// ([`predict_field_picture_16x8_macroblock_planes`]), add the §A IDCT
/// residual per coded block, and write out. There is no frame/field DCT
/// distinction inside a field picture (§6.1.3 Table 6-19), so blocks
/// place contiguously (`field_dct` fixed `false`).
///
/// Returns the number of blocks written (`block_count(chroma_format)`).
///
/// # Errors
///
/// Propagates [`predict_field_picture_16x8_macroblock_planes`] reference
/// errors and rejects an out-of-range residual `block_index`.
pub fn reconstruct_field_picture_16x8_macroblock(
    dest: &mut FrameBuffer,
    references: ReferenceFrames<'_>,
    mb_col: usize,
    mb_row: usize,
    motion: FieldPicture16x8Motion,
    residuals: &[ResidualBlock<'_>],
) -> InterResult<usize> {
    let chroma_format = dest.chroma_format;
    let (luma_pred, cb_pred, cr_pred) =
        predict_field_picture_16x8_macroblock_planes(dest, references, mb_col, mb_row, motion)?;

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
            false,
            &luma_pred,
            &cb_pred,
            &cr_pred,
            f_pel,
        )?;
        written += 1;
    }
    Ok(written)
}

/// The two same-/opposite-parity luminance motion vectors **and** their
/// selected reference fields for one **field-picture dual-prime**
/// macroblock (Table 7-13 `Dual prime` row).
///
/// In dual-prime prediction only one field motion vector
/// (`vector'[0][0][1:0]`) is decoded from the bitstream; it forms the
/// same-parity prediction, reading the reference field whose parity
/// matches the field being predicted. The §7.6.3.6 arithmetic
/// ([`crate::dual_prime::derive_all`]) derives a second vector
/// (`vector'[2][0][1:0]`) that forms the opposite-parity prediction,
/// reading the reference field of the **opposite** parity. The two
/// predictions are averaged per §7.6.7.4.
///
/// Dual-prime is only ever a **forward** P-picture prediction (§7.6.3.6,
/// Table 7-13), so there is no backward direction. The destination is
/// one field of a frame, so each prediction is a single 16×16 field block
/// read contiguously from its chosen [`FieldReference`] — there is no
/// frame/field interleave (§6.1.3 Table 6-19).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FieldPictureDualPrimeMotion {
    /// Same-parity luminance vector `vector'[0][0][1:0]` (decoded from
    /// the bitstream), reading the reference field of `same_parity`.
    pub same_parity_vector: MotionVectorPel,
    /// Opposite-parity luminance vector `vector'[2][0][1:0]` (derived by
    /// §7.6.3.6), reading the reference field of the opposite parity.
    pub opposite_parity_vector: MotionVectorPel,
    /// Parity of the reference field the same-parity prediction reads —
    /// equal to the parity of the field being predicted. The opposite-
    /// parity prediction reads `same_parity.opposite()`.
    pub same_parity: FieldParity,
}

impl FieldPictureDualPrimeMotion {
    /// A dual-prime motion predicting a field of parity `same_parity`
    /// from the same-parity reference field (with `same_parity_vector`)
    /// and the opposite-parity reference field (with
    /// `opposite_parity_vector`).
    pub fn new(
        same_parity: FieldParity,
        same_parity_vector: MotionVectorPel,
        opposite_parity_vector: MotionVectorPel,
    ) -> Self {
        Self {
            same_parity_vector,
            opposite_parity_vector,
            same_parity,
        }
    }
}

/// Form the full per-component prediction planes (luma, cb, cr) for one
/// **field-picture dual-prime** macroblock and combine the same-parity
/// and opposite-parity predictions per §7.6.7.4 (the `// 2` average).
///
/// `dest` is the destination **field** buffer (one field of a frame; its
/// `height` is the field height). `reference` is the most-recently
/// decoded reference **frame**, whose two fields are read independently
/// — the same-parity field with `motion.same_parity_vector`, the
/// opposite-parity field with `motion.opposite_parity_vector`. Both
/// predictions are field blocks read contiguously (no frame/field
/// interleave inside a field picture).
///
/// Per §7.6.7.4 the field-picture chroma prediction for each region is
/// the full component extent (8×8 / 8×16 / 16×16 for 4:2:0 / 4:2:2 /
/// 4:4:4), matching [`chroma_mb_extent`].
///
/// # Errors
///
/// [`InterError::ReferenceGeometryMismatch`] when the reference frame's
/// format / width differs from `dest` or its height is not twice the
/// field height (§6.1.1).
pub fn predict_field_picture_dual_prime_macroblock_planes(
    dest: &FrameBuffer,
    reference: &FrameBuffer,
    mb_col: usize,
    mb_row: usize,
    motion: FieldPictureDualPrimeMotion,
) -> InterResult<(Vec<u8>, Vec<u8>, Vec<u8>)> {
    let chroma_format = dest.chroma_format;
    let (cw, ch) = chroma_mb_extent(chroma_format);
    let luma_x = (mb_col * 16) as i32;
    let luma_y = (mb_row * 16) as i32;
    let chroma_x = (mb_col * cw) as i32;
    let chroma_y = (mb_row * ch) as i32;

    if reference.chroma_format != dest.chroma_format
        || reference.width != dest.width
        || reference.height != dest.height.saturating_mul(2)
    {
        return Err(InterError::ReferenceGeometryMismatch);
    }

    // One same-/opposite-parity prediction: a single 16×16 (luma) field
    // block read from `parity`'s reference field with the luma vector,
    // and the §7.6.3.7-scaled chroma blocks.
    let one_field = |parity: FieldParity, mv: MotionVectorPel| -> (Vec<u8>, Vec<u8>, Vec<u8>) {
        let scaled = scale_chroma(mv.horizontal, mv.vertical, chroma_format);
        let chroma_mv = MotionVectorPel::new(scaled.chroma_horiz, scaled.chroma_vert);
        let luma = predict_field_picture_component(
            reference,
            ColourComponent::Y,
            parity,
            luma_x,
            luma_y,
            16,
            16,
            mv,
        );
        let cb = predict_field_picture_component(
            reference,
            ColourComponent::Cb,
            parity,
            chroma_x,
            chroma_y,
            cw,
            ch,
            chroma_mv,
        );
        let cr = predict_field_picture_component(
            reference,
            ColourComponent::Cr,
            parity,
            chroma_x,
            chroma_y,
            cw,
            ch,
            chroma_mv,
        );
        (luma, cb, cr)
    };

    let (sy, scb, scr) = one_field(motion.same_parity, motion.same_parity_vector);
    let (oy, ocb, ocr) = one_field(motion.same_parity.opposite(), motion.opposite_parity_vector);
    // §7.6.7.4: average the same-parity and opposite-parity predictions.
    let y = average_predictions(&sy, &oy).unwrap_or(sy);
    let cb = average_predictions(&scb, &ocb).unwrap_or(scb);
    let cr = average_predictions(&scr, &ocr).unwrap_or(scr);
    Ok((y, cb, cr))
}

/// Reconstruct one **field-picture dual-prime** P macroblock end-to-end
/// into `dest` (one field of a frame), per the §7.6 pipeline (Table 7-13
/// `Dual prime` row): form the same-/opposite-parity prediction planes
/// ([`predict_field_picture_dual_prime_macroblock_planes`]), then add the
/// §A IDCT residual per coded block and write out.
///
/// There is no frame/field DCT distinction inside a field picture
/// (§6.1.3 Table 6-19), so blocks place contiguously; `field_dct` is
/// therefore fixed `false` for the write-out.
///
/// Returns the number of blocks written (`block_count(chroma_format)`).
///
/// # Errors
///
/// Propagates [`predict_field_picture_dual_prime_macroblock_planes`]
/// reference errors and rejects an out-of-range residual `block_index`.
pub fn reconstruct_field_picture_dual_prime_macroblock(
    dest: &mut FrameBuffer,
    reference: &FrameBuffer,
    mb_col: usize,
    mb_row: usize,
    motion: FieldPictureDualPrimeMotion,
    residuals: &[ResidualBlock<'_>],
) -> InterResult<usize> {
    let chroma_format = dest.chroma_format;
    let (luma_pred, cb_pred, cr_pred) = predict_field_picture_dual_prime_macroblock_planes(
        dest, reference, mb_col, mb_row, motion,
    )?;

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
            false,
            &luma_pred,
            &cb_pred,
            &cr_pred,
            f_pel,
        )?;
        written += 1;
    }
    Ok(written)
}

/// The same-/opposite-parity luminance motion vectors for one
/// **frame-picture dual-prime** macroblock (Table 7-14 `Dual prime`
/// row).
///
/// A frame-picture dual-prime macroblock forms **four** field predictions
/// (§7.6.2, §7.6.7.4): the predicted frame's top field is predicted from
/// the top reference field (same parity, `vector'[0][0]`) and the bottom
/// reference field (opposite parity, derived `vector'[2][0]`); the
/// predicted frame's bottom field is predicted from the bottom reference
/// field (same parity, `vector'[0][0]`) and the top reference field
/// (opposite parity, derived `vector'[3][0]`). The two predictions of
/// each field are averaged per §7.6.7.4, then the two fields are
/// interleaved into the frame.
///
/// `vector'[0][0]` is the single decoded vector, shared by both fields'
/// same-parity predictions. `top_field_opposite` is `vector'[2][0]`
/// (derived for the top field's bottom-reference prediction);
/// `bottom_field_opposite` is `vector'[3][0]` (derived for the bottom
/// field's top-reference prediction). All vectors carry their vertical
/// component in field-sample units.
///
/// Dual-prime is forward-only (P-picture), so there is no backward
/// direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameDualPrimeMotion {
    /// The decoded same-parity vector `vector'[0][0][1:0]`, used for both
    /// the top field's top-reference prediction and the bottom field's
    /// bottom-reference prediction.
    pub same_parity_vector: MotionVectorPel,
    /// The §7.6.3.6-derived `vector'[2][0][1:0]` — the top field's
    /// opposite-parity (bottom-reference) prediction.
    pub top_field_opposite_vector: MotionVectorPel,
    /// The §7.6.3.6-derived `vector'[3][0][1:0]` — the bottom field's
    /// opposite-parity (top-reference) prediction.
    pub bottom_field_opposite_vector: MotionVectorPel,
}

impl FrameDualPrimeMotion {
    /// A frame-picture dual-prime motion from the decoded same-parity
    /// vector and the two §7.6.3.6-derived opposite-parity vectors.
    pub fn new(
        same_parity_vector: MotionVectorPel,
        top_field_opposite_vector: MotionVectorPel,
        bottom_field_opposite_vector: MotionVectorPel,
    ) -> Self {
        Self {
            same_parity_vector,
            top_field_opposite_vector,
            bottom_field_opposite_vector,
        }
    }
}

/// Form one component's full macroblock prediction plane for a
/// **frame-picture dual-prime** prediction, in **frame** order.
///
/// Each of the macroblock's two fields is predicted by averaging its
/// same-parity and opposite-parity field predictions (§7.6.7.4), then the
/// two fields are interleaved back into the frame at stride 2 (even rows
/// from the top field, odd rows from the bottom field) exactly as the
/// frame-picture field-based path does.
///
/// `same_mv` is the shared decoded vector; `(top_opp_mv,
/// bottom_opp_mv)` are the derived opposite-parity vectors for the top
/// and bottom predicted fields respectively. The motion vectors are in
/// the component's own sample units (luma for `Y`; §7.6.3.7-scaled for
/// chroma). `(base_x, base_y)` is the macroblock's top-left **frame**
/// coordinate in this component's plane.
#[allow(clippy::too_many_arguments)]
fn predict_frame_dual_prime_component(
    reference: &FrameBuffer,
    component: ColourComponent,
    base_x: i32,
    base_y: i32,
    width: usize,
    height: usize,
    same_mv: MotionVectorPel,
    top_opp_mv: MotionVectorPel,
    bottom_opp_mv: MotionVectorPel,
) -> Vec<u8> {
    let (data, pw, ph) = component_plane(reference, component);
    let Some(plane) = ReferencePlane::new(data, pw, ph) else {
        return vec![0u8; width * height];
    };
    let half = height / 2;
    let Some(size) = BlockSize::new(width, half) else {
        return vec![0u8; width * height];
    };
    let field_top_line = base_y / 2;
    let mut out = vec![0u8; width * height];

    // Read one parity's field block at the macroblock's field origin.
    let read_field = |parity: FieldParity, mv: MotionVectorPel| -> Option<Vec<u8>> {
        let field = FieldReference::new(plane, parity.index())?;
        Some(predict_field_block(
            field,
            base_x,
            field_top_line,
            size,
            mv.horizontal,
            mv.vertical,
        ))
    };

    // Predicted top field (even frame rows): same parity = top reference
    // with the decoded vector, opposite parity = bottom reference with
    // vector'[2]. §7.6.7.4 averages them.
    let top_same = read_field(FieldParity::Top, same_mv);
    let top_opp = read_field(FieldParity::Bottom, top_opp_mv);
    if let (Some(s), Some(o)) = (top_same.as_ref(), top_opp.as_ref()) {
        let avg = average_predictions(s, o);
        let top = avg.as_deref().unwrap_or(s.as_slice());
        for r in 0..half {
            let frame_row = 2 * r;
            out[frame_row * width..frame_row * width + width]
                .copy_from_slice(&top[r * width..r * width + width]);
        }
    }

    // Predicted bottom field (odd frame rows): same parity = bottom
    // reference with the decoded vector, opposite parity = top reference
    // with vector'[3]. §7.6.7.4 averages them.
    let bottom_same = read_field(FieldParity::Bottom, same_mv);
    let bottom_opp = read_field(FieldParity::Top, bottom_opp_mv);
    if let (Some(s), Some(o)) = (bottom_same.as_ref(), bottom_opp.as_ref()) {
        let avg = average_predictions(s, o);
        let bottom = avg.as_deref().unwrap_or(s.as_slice());
        for r in 0..half {
            let frame_row = 2 * r + 1;
            out[frame_row * width..frame_row * width + width]
                .copy_from_slice(&bottom[r * width..r * width + width]);
        }
    }
    out
}

/// Form the full per-component prediction planes (luma, cb, cr) for one
/// **frame-picture dual-prime** macroblock per §7.6.7.4, returned in
/// frame-order row-major layout (16×16 luma; chroma sized by
/// [`chroma_mb_extent`]) ready for the same residual-add / block-write
/// path the frame-based driver uses.
///
/// The §7.6.7.4 frame-picture chroma prediction for each field is the
/// full component width × half height; interleaved over the two fields
/// it fills the [`chroma_mb_extent`] block, so the returned chroma
/// buffers match the frame-based chroma extents and need no special
/// handling downstream.
///
/// # Errors
///
/// [`InterError::ReferenceGeometryMismatch`] when the reference frame's
/// geometry differs from `dest`.
pub fn predict_frame_dual_prime_macroblock_planes(
    dest: &FrameBuffer,
    reference: &FrameBuffer,
    mb_col: usize,
    mb_row: usize,
    motion: FrameDualPrimeMotion,
) -> InterResult<(Vec<u8>, Vec<u8>, Vec<u8>)> {
    let chroma_format = dest.chroma_format;
    let (cw, ch) = chroma_mb_extent(chroma_format);
    let luma_x = (mb_col * 16) as i32;
    let luma_y = (mb_row * 16) as i32;
    let chroma_x = (mb_col * cw) as i32;
    let chroma_y = (mb_row * ch) as i32;

    if reference.width != dest.width
        || reference.height != dest.height
        || reference.chroma_format != dest.chroma_format
    {
        return Err(InterError::ReferenceGeometryMismatch);
    }

    // §7.6.3.7 chroma scaling is applied to each field vector
    // independently (the vertical component stays in field-sample units
    // of the chroma field).
    let scale = |mv: MotionVectorPel| {
        let s = scale_chroma(mv.horizontal, mv.vertical, chroma_format);
        MotionVectorPel::new(s.chroma_horiz, s.chroma_vert)
    };
    let same_cmv = scale(motion.same_parity_vector);
    let top_opp_cmv = scale(motion.top_field_opposite_vector);
    let bottom_opp_cmv = scale(motion.bottom_field_opposite_vector);

    let luma = predict_frame_dual_prime_component(
        reference,
        ColourComponent::Y,
        luma_x,
        luma_y,
        16,
        16,
        motion.same_parity_vector,
        motion.top_field_opposite_vector,
        motion.bottom_field_opposite_vector,
    );
    let cb = predict_frame_dual_prime_component(
        reference,
        ColourComponent::Cb,
        chroma_x,
        chroma_y,
        cw,
        ch,
        same_cmv,
        top_opp_cmv,
        bottom_opp_cmv,
    );
    let cr = predict_frame_dual_prime_component(
        reference,
        ColourComponent::Cr,
        chroma_x,
        chroma_y,
        cw,
        ch,
        same_cmv,
        top_opp_cmv,
        bottom_opp_cmv,
    );
    Ok((luma, cb, cr))
}

/// Reconstruct one **frame-picture dual-prime** P macroblock end-to-end
/// into `dest`, per the §7.6 pipeline (Table 7-14 `Dual prime` row): form
/// the four-field-prediction planes
/// ([`predict_frame_dual_prime_macroblock_planes`]), add the §A IDCT
/// residual per coded block, and write out with the §6.1.3 frame/field
/// DCT line organisation honoured (the planes are returned in frame-row
/// order, so `field_dct` threads straight into [`write_inter_block`]).
///
/// Returns the number of blocks written (`block_count(chroma_format)`).
///
/// # Errors
///
/// Propagates [`predict_frame_dual_prime_macroblock_planes`] reference
/// errors and rejects an out-of-range residual `block_index`.
pub fn reconstruct_frame_dual_prime_macroblock(
    dest: &mut FrameBuffer,
    reference: &FrameBuffer,
    mb_col: usize,
    mb_row: usize,
    field_dct: bool,
    motion: FrameDualPrimeMotion,
    residuals: &[ResidualBlock<'_>],
) -> InterResult<usize> {
    let chroma_format = dest.chroma_format;
    let (luma_pred, cb_pred, cr_pred) =
        predict_frame_dual_prime_macroblock_planes(dest, reference, mb_col, mb_row, motion)?;

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

    // ---- frame-picture field-based prediction (Table 7-14) ----

    /// Build a 16×16 4:4:4 frame whose Y value encodes the frame row so
    /// a field prediction's parity is directly observable.
    fn row_encoded_frame() -> FrameBuffer {
        let cf = ChromaFormat::Yuv444;
        let mut f = FrameBuffer::new(16, 16, cf);
        for y in 0..16 {
            for x in 0..16 {
                f.y.put_sample(x, y, (y * 16 + x) as u8);
                f.cb.put_sample(x, y, 0);
                f.cr.put_sample(x, y, 0);
            }
        }
        f
    }

    #[test]
    fn field_based_zero_mv_copies_same_parity_lines() {
        // Forward field-based, both field vectors (0,0). The top-field
        // prediction must copy the reference's even rows into the dest's
        // even rows, and the bottom-field prediction the odd rows into
        // the odd rows — i.e. a verbatim frame copy.
        let reference = row_encoded_frame();
        let dest = FrameBuffer::new(16, 16, ChromaFormat::Yuv444);
        let refs = ReferenceFrames::forward_only(&reference);
        let zero = MotionVectorPel::new(0, 0);
        let motion = FieldBasedMotion::forward(
            FieldVector::new(zero, FieldParity::Top),
            FieldVector::new(zero, FieldParity::Bottom),
        );
        let (luma, _, _) =
            predict_field_based_macroblock_planes(&dest, refs, 0, 0, motion).unwrap();
        for y in 0..16 {
            for x in 0..16 {
                assert_eq!(luma[y * 16 + x], (y * 16 + x) as u8, "x={x} y={y}");
            }
        }
    }

    #[test]
    fn field_based_distinct_field_vectors_shift_each_parity() {
        // Top-field vector = +1 field line (vertical 2 half-samples =
        // +1 field line); bottom-field vector = 0. The top-field dest
        // rows read the *next* top-field line of the reference; bottom
        // rows are unchanged.
        let reference = row_encoded_frame();
        let dest = FrameBuffer::new(16, 16, ChromaFormat::Yuv444);
        let refs = ReferenceFrames::forward_only(&reference);
        // vertical = 2 half-samples -> int_vec 1 field line, no half-pel.
        let top_mv = MotionVectorPel::new(0, 2);
        let bottom_mv = MotionVectorPel::new(0, 0);
        let motion = FieldBasedMotion::forward(
            FieldVector::new(top_mv, FieldParity::Top),
            FieldVector::new(bottom_mv, FieldParity::Bottom),
        );
        let (luma, _, _) =
            predict_field_based_macroblock_planes(&dest, refs, 0, 0, motion).unwrap();
        // Even dest rows (top field line k) read top field line k+1 =
        // frame row 2*(k+1). Dest frame row 2k = reference row 2k+2
        // (clamped at the last top line).
        for k in 0..8 {
            let dest_row = 2 * k;
            let src_top_line = (k + 1).min(7); // clamp inside top field
            let src_frame_row = 2 * src_top_line;
            assert_eq!(
                luma[dest_row * 16],
                (src_frame_row * 16) as u8,
                "top field line {k}"
            );
        }
        // Odd dest rows unchanged: dest frame row 2k+1 = reference row 2k+1.
        for k in 0..8 {
            let dest_row = 2 * k + 1;
            assert_eq!(
                luma[dest_row * 16],
                (dest_row * 16) as u8,
                "bottom line {k}"
            );
        }
    }

    #[test]
    fn field_based_reconstruct_with_residual() {
        // Whole field-based MB, zero MVs, a +7 residual on luma block 0.
        let reference = row_encoded_frame();
        let mut dest = FrameBuffer::new(16, 16, ChromaFormat::Yuv444);
        let refs = ReferenceFrames::forward_only(&reference);
        let zero = MotionVectorPel::new(0, 0);
        let motion = FieldBasedMotion::forward(
            FieldVector::new(zero, FieldParity::Top),
            FieldVector::new(zero, FieldParity::Bottom),
        );
        let f = [[7i16; 8]; 8];
        let residuals = [ResidualBlock {
            block_index: 0,
            f_pel: &f,
        }];
        let n =
            reconstruct_field_based_macroblock(&mut dest, refs, 0, 0, false, motion, &residuals)
                .unwrap();
        // 4:4:4 -> 12 blocks.
        assert_eq!(n, 12);
        // Block 0 = luma (0..8, 0..8): reference(x,y) + 7, saturated.
        for y in 0..8 {
            for x in 0..8 {
                let expect = ((y * 16 + x) as i32 + 7).clamp(0, 255) as u8;
                assert_eq!(dest.y.get(x, y), Some(expect), "x={x} y={y}");
            }
        }
        // Block 3 = luma (8..16, 8..16): no residual -> verbatim copy.
        assert_eq!(dest.y.get(8, 8), Some((8 * 16 + 8) as u8));
    }

    #[test]
    fn field_based_bidirectional_averages_directions() {
        let cf = ChromaFormat::Yuv444;
        let fwd = {
            let mut f = FrameBuffer::new(16, 16, cf);
            for y in 0..16 {
                for x in 0..16 {
                    f.y.put_sample(x, y, 100);
                    f.cb.put_sample(x, y, 100);
                    f.cr.put_sample(x, y, 100);
                }
            }
            f
        };
        let bwd = {
            let mut f = FrameBuffer::new(16, 16, cf);
            for y in 0..16 {
                for x in 0..16 {
                    f.y.put_sample(x, y, 200);
                    f.cb.put_sample(x, y, 200);
                    f.cr.put_sample(x, y, 200);
                }
            }
            f
        };
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::bidirectional(&fwd, &bwd);
        let zero = MotionVectorPel::new(0, 0);
        let zt = FieldVector::new(zero, FieldParity::Top);
        let zb = FieldVector::new(zero, FieldParity::Bottom);
        let motion = FieldBasedMotion::bidirectional(zt, zb, zt, zb);
        let (luma, _, _) =
            predict_field_based_macroblock_planes(&dest, refs, 0, 0, motion).unwrap();
        // (100 + 200) // 2 = 150 everywhere.
        assert!(luma.iter().all(|&p| p == 150));
    }

    #[test]
    fn field_based_missing_reference_errors() {
        let dest = FrameBuffer::new(16, 16, ChromaFormat::Yuv444);
        let refs = ReferenceFrames {
            forward: None,
            backward: None,
        };
        let zero = MotionVectorPel::new(0, 0);
        let motion = FieldBasedMotion::forward(
            FieldVector::new(zero, FieldParity::Top),
            FieldVector::new(zero, FieldParity::Bottom),
        );
        assert_eq!(
            predict_field_based_macroblock_planes(&dest, refs, 0, 0, motion),
            Err(InterError::MissingForwardReference)
        );
    }

    #[test]
    fn field_based_420_chroma_field_split() {
        // 4:2:0: chroma MB is 8×8, split into 4 top + 4 bottom field
        // lines. A zero-MV field-based prediction copies the chroma
        // verbatim (each parity reads its own lines).
        let cf = ChromaFormat::Yuv420;
        let mut reference = FrameBuffer::new(16, 16, cf);
        for y in 0..16 {
            for x in 0..16 {
                reference.y.put_sample(x, y, 0);
            }
        }
        for y in 0..8 {
            for x in 0..8 {
                reference.cb.put_sample(x, y, (y * 8 + x) as u8);
                reference.cr.put_sample(x, y, 0);
            }
        }
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let zero = MotionVectorPel::new(0, 0);
        let motion = FieldBasedMotion::forward(
            FieldVector::new(zero, FieldParity::Top),
            FieldVector::new(zero, FieldParity::Bottom),
        );
        let (_, cb, _) = predict_field_based_macroblock_planes(&dest, refs, 0, 0, motion).unwrap();
        assert_eq!(cb.len(), 64);
        for y in 0..8 {
            for x in 0..8 {
                assert_eq!(cb[y * 8 + x], (y * 8 + x) as u8, "chroma x={x} y={y}");
            }
        }
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

    // ---- §7.6 field-picture simple field prediction ----

    /// A 16×32 reference frame whose luma encodes the *frame row* in the
    /// top byte (`y * 4`) so a parity selection is visible (top field =
    /// even frame rows, bottom field = odd frame rows). One field of this
    /// frame is 16×16 luma — exactly one macroblock tall. Chroma is
    /// zeroed.
    fn vertical_ramp_frame() -> FrameBuffer {
        let cf = ChromaFormat::Yuv420;
        let mut f = FrameBuffer::new(16, 32, cf);
        for y in 0..32 {
            for x in 0..16 {
                f.y.put_sample(x, y, (y * 4) as u8);
            }
        }
        for y in 0..16 {
            for x in 0..8 {
                f.cb.put_sample(x, y, 0);
                f.cr.put_sample(x, y, 0);
            }
        }
        f
    }

    #[test]
    fn field_picture_top_field_zero_mv_reads_even_frame_rows() {
        // Field picture: destination is one field, 16 luma rows (half the
        // 32-row reference frame). A top-parity, zero-MV forward
        // prediction reads field line k = frame row 2k of the reference.
        let cf = ChromaFormat::Yuv420;
        let reference = vertical_ramp_frame();
        let dest = FrameBuffer::new(16, 16, cf); // one field plane
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FieldPictureMotion::forward(MotionVectorPel::new(0, 0), FieldParity::Top);
        let (luma, _, _) =
            predict_field_picture_macroblock_planes(&dest, refs, 0, 0, motion).unwrap();
        assert_eq!(luma.len(), 256); // 16×16 field block
        for k in 0..16usize {
            // field line k → frame row 2k (top parity) → value 2k*4.
            let frame_row = 2 * k;
            let expected = (frame_row * 4) as u8;
            for x in 0..16 {
                assert_eq!(luma[k * 16 + x], expected, "field line {k} col {x}");
            }
        }
    }

    #[test]
    fn field_picture_bottom_field_zero_mv_reads_odd_frame_rows() {
        // A bottom-parity, zero-MV prediction reads field line k = frame
        // row 2k+1 of the reference.
        let cf = ChromaFormat::Yuv420;
        let reference = vertical_ramp_frame();
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FieldPictureMotion::forward(MotionVectorPel::new(0, 0), FieldParity::Bottom);
        let (luma, _, _) =
            predict_field_picture_macroblock_planes(&dest, refs, 0, 0, motion).unwrap();
        for k in 0..16usize {
            let frame_row = 2 * k + 1;
            let expected = (frame_row * 4) as u8;
            assert_eq!(luma[k * 16], expected, "field line {k}");
        }
    }

    #[test]
    fn field_picture_half_pel_vertical_averages_adjacent_field_lines() {
        // A vertical motion_code +1 (vector' vertical = +1 half-sample)
        // reads the // 2 average of adjacent FIELD lines (frame rows two
        // apart for the chosen parity), proving the field grid, not the
        // frame grid, is sampled.
        let cf = ChromaFormat::Yuv420;
        let reference = vertical_ramp_frame();
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FieldPictureMotion::forward(MotionVectorPel::new(0, 1), FieldParity::Top);
        let (luma, _, _) =
            predict_field_picture_macroblock_planes(&dest, refs, 0, 0, motion).unwrap();
        for k in 0..16usize {
            let lo_row = 2 * k;
            let hi_row = (2 * (k + 1)).min(30); // top field's last line = frame row 30
            let expected = ((lo_row * 4) as u32).midpoint((hi_row * 4) as u32) as u8;
            assert_eq!(luma[k * 16], expected, "half-pel field line {k}");
        }
    }

    #[test]
    fn field_picture_bidirectional_averages_two_reference_frames() {
        let cf = ChromaFormat::Yuv420;
        let fwd = solid_frame(100, 16, 32, cf);
        let bwd = solid_frame(200, 16, 32, cf);
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::bidirectional(&fwd, &bwd);
        let motion = FieldPictureMotion::bidirectional(
            MotionVectorPel::new(0, 0),
            FieldParity::Top,
            MotionVectorPel::new(0, 0),
            FieldParity::Bottom,
        );
        let (luma, _, _) =
            predict_field_picture_macroblock_planes(&dest, refs, 0, 0, motion).unwrap();
        // (100 + 200) // 2 = 150.
        assert!(luma.iter().all(|&p| p == 150));
    }

    #[test]
    fn field_picture_geometry_mismatch_errors() {
        // Reference height must be exactly twice the destination field
        // height; a same-height reference is a mismatch.
        let cf = ChromaFormat::Yuv420;
        let reference = solid_frame(100, 16, 16, cf);
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FieldPictureMotion::forward(MotionVectorPel::new(0, 0), FieldParity::Top);
        assert_eq!(
            predict_field_picture_macroblock_planes(&dest, refs, 0, 0, motion),
            Err(InterError::ReferenceGeometryMismatch)
        );
    }

    #[test]
    fn field_picture_reconstruct_adds_residual() {
        // reconstruct_field_picture_macroblock writes d = saturate(f + p)
        // contiguously into the field plane (no frame/field DCT interleave
        // inside a field picture). With a zero MV top-parity prediction
        // and a +5 residual on every luma block the field plane is the
        // even-frame-row ramp + 5.
        let cf = ChromaFormat::Yuv420;
        let reference = vertical_ramp_frame();
        let mut dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FieldPictureMotion::forward(MotionVectorPel::new(0, 0), FieldParity::Top);
        let residual = [[5i16; 8]; 8];
        let blocks: Vec<ResidualBlock<'_>> = (0..4)
            .map(|i| ResidualBlock {
                block_index: i,
                f_pel: &residual,
            })
            .collect();
        let written =
            reconstruct_field_picture_macroblock(&mut dest, refs, 0, 0, motion, &blocks).unwrap();
        assert_eq!(written, 6); // 4 luma + 2 chroma blocks (4:2:0)
                                // Field line k → frame row 2k value, plus the +5 residual.
        for k in 0..16usize {
            let expected = ((2 * k * 4) as i32 + 5).clamp(0, 255) as u8;
            assert_eq!(dest.y.get(0, k), Some(expected), "field line {k}");
        }
    }

    // ---- field-picture 16×8 MC (§7.6.7.3, Table 7-13 `16x8 MC`) ----

    #[test]
    fn field_picture_16x8_both_regions_same_field_zero_mv_equals_simple_field() {
        // Two 16×8 regions both selecting the top field with a zero MV is
        // identical to a simple 16×16 field prediction reading even rows.
        let cf = ChromaFormat::Yuv420;
        let reference = vertical_ramp_frame();
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FieldPicture16x8Motion::forward(
            (MotionVectorPel::new(0, 0), FieldParity::Top),
            (MotionVectorPel::new(0, 0), FieldParity::Top),
        );
        let (luma, _, _) =
            predict_field_picture_16x8_macroblock_planes(&dest, refs, 0, 0, motion).unwrap();
        assert_eq!(luma.len(), 256);
        for k in 0..16usize {
            // field line k → frame row 2k (top parity) → value 2k*4.
            let expected = (2 * k * 4) as u8;
            for x in 0..16 {
                assert_eq!(luma[k * 16 + x], expected, "line {k} col {x}");
            }
        }
    }

    #[test]
    fn field_picture_16x8_regions_select_independent_fields() {
        // Upper region reads the TOP field, lower region the BOTTOM field,
        // both zero MV. The upper eight lines must therefore match even
        // frame rows and the lower eight match odd frame rows — the field
        // origin of the lower region is field line 8, i.e. frame rows
        // 2*8+1 = 17, 19, … (bottom parity).
        let cf = ChromaFormat::Yuv420;
        let reference = vertical_ramp_frame();
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FieldPicture16x8Motion::forward(
            (MotionVectorPel::new(0, 0), FieldParity::Top),
            (MotionVectorPel::new(0, 0), FieldParity::Bottom),
        );
        let (luma, _, _) =
            predict_field_picture_16x8_macroblock_planes(&dest, refs, 0, 0, motion).unwrap();
        // Upper region: lines 0..8 → top field line k → frame row 2k.
        for k in 0..8usize {
            let expected = (2 * k * 4) as u8;
            assert_eq!(luma[k * 16], expected, "upper line {k}");
        }
        // Lower region: lines 8..16 occupy field lines 8..16 of the BOTTOM
        // field → frame row 2*line + 1.
        for k in 8..16usize {
            let frame_row = 2 * k + 1;
            let expected = (frame_row * 4) as u8;
            assert_eq!(luma[k * 16], expected, "lower line {k}");
        }
    }

    #[test]
    fn field_picture_16x8_lower_region_uses_lower_vector() {
        // Distinct vectors per region: upper region zero MV, lower region a
        // +2 half-sample vertical (one full field line) on the same top
        // field. The lower region's line 8 should read top-field line 9
        // (frame row 18) rather than line 8 (frame row 16).
        let cf = ChromaFormat::Yuv420;
        let reference = vertical_ramp_frame();
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FieldPicture16x8Motion::forward(
            (MotionVectorPel::new(0, 0), FieldParity::Top),
            (MotionVectorPel::new(0, 2), FieldParity::Top),
        );
        let (luma, _, _) =
            predict_field_picture_16x8_macroblock_planes(&dest, refs, 0, 0, motion).unwrap();
        // Lower region line 8 = top field line (8 + 1) = frame row 18.
        assert_eq!(luma[8 * 16], (18 * 4) as u8);
        // Upper region line 0 unchanged = frame row 0.
        assert_eq!(luma[0], 0);
    }

    #[test]
    fn field_picture_16x8_chroma_region_split() {
        // 4:2:0 chroma macroblock is 8×8; each 16×8 luma region maps to an
        // 8×4 chroma region (§7.6.7.3). A reference whose top-field chroma
        // ramps and bottom-field chroma is constant proves the upper four
        // chroma lines come from the top field and the lower four from the
        // bottom field when the regions select different fields.
        let cf = ChromaFormat::Yuv420;
        let mut reference = FrameBuffer::new(16, 32, cf);
        for y in 0..32 {
            for x in 0..16 {
                reference.y.put_sample(x, y, 0);
            }
        }
        for y in 0..16usize {
            for x in 0..8 {
                // top field (even frame rows) cb = field-line*2; bottom = 200.
                reference.cb.put_sample(x, 2 * y, (y * 2) as u8);
                reference.cb.put_sample(x, 2 * y + 1, 200);
                reference.cr.put_sample(x, 2 * y, (y * 2) as u8);
                reference.cr.put_sample(x, 2 * y + 1, 200);
            }
        }
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FieldPicture16x8Motion::forward(
            (MotionVectorPel::new(0, 0), FieldParity::Top),
            (MotionVectorPel::new(0, 0), FieldParity::Bottom),
        );
        let (_, cb, _) =
            predict_field_picture_16x8_macroblock_planes(&dest, refs, 0, 0, motion).unwrap();
        assert_eq!(cb.len(), 64); // 8×8
                                  // Upper four chroma lines: top field ramp = line*2.
        for k in 0..4usize {
            assert_eq!(cb[k * 8], (k * 2) as u8, "upper chroma line {k}");
        }
        // Lower four chroma lines: bottom field constant 200.
        for k in 4..8usize {
            assert_eq!(cb[k * 8], 200, "lower chroma line {k}");
        }
    }

    #[test]
    fn field_picture_16x8_bidirectional_averages() {
        let cf = ChromaFormat::Yuv420;
        let fwd = solid_frame(80, 16, 32, cf);
        let bwd = solid_frame(160, 16, 32, cf);
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::bidirectional(&fwd, &bwd);
        let motion = FieldPicture16x8Motion {
            forward: Some([
                (MotionVectorPel::new(0, 0), FieldParity::Top),
                (MotionVectorPel::new(0, 0), FieldParity::Top),
            ]),
            backward: Some([
                (MotionVectorPel::new(0, 0), FieldParity::Bottom),
                (MotionVectorPel::new(0, 0), FieldParity::Bottom),
            ]),
        };
        let (luma, _, _) =
            predict_field_picture_16x8_macroblock_planes(&dest, refs, 0, 0, motion).unwrap();
        // (80 + 160) // 2 = 120.
        assert!(luma.iter().all(|&p| p == 120));
    }

    #[test]
    fn field_picture_16x8_geometry_mismatch_errors() {
        let cf = ChromaFormat::Yuv420;
        let reference = solid_frame(100, 16, 16, cf); // not twice the field height
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FieldPicture16x8Motion::forward(
            (MotionVectorPel::new(0, 0), FieldParity::Top),
            (MotionVectorPel::new(0, 0), FieldParity::Top),
        );
        assert_eq!(
            predict_field_picture_16x8_macroblock_planes(&dest, refs, 0, 0, motion),
            Err(InterError::ReferenceGeometryMismatch)
        );
    }

    #[test]
    fn field_picture_16x8_missing_backward_reference_errors() {
        let cf = ChromaFormat::Yuv420;
        let dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames {
            forward: None,
            backward: None,
        };
        let motion = FieldPicture16x8Motion::backward(
            (MotionVectorPel::new(0, 0), FieldParity::Top),
            (MotionVectorPel::new(0, 0), FieldParity::Top),
        );
        assert_eq!(
            predict_field_picture_16x8_macroblock_planes(&dest, refs, 0, 0, motion),
            Err(InterError::MissingBackwardReference)
        );
    }

    #[test]
    fn field_picture_16x8_reconstruct_adds_residual() {
        let cf = ChromaFormat::Yuv420;
        let reference = vertical_ramp_frame();
        let mut dest = FrameBuffer::new(16, 16, cf);
        let refs = ReferenceFrames::forward_only(&reference);
        let motion = FieldPicture16x8Motion::forward(
            (MotionVectorPel::new(0, 0), FieldParity::Top),
            (MotionVectorPel::new(0, 0), FieldParity::Top),
        );
        let residual = [[3i16; 8]; 8];
        let blocks: Vec<ResidualBlock<'_>> = (0..4)
            .map(|i| ResidualBlock {
                block_index: i,
                f_pel: &residual,
            })
            .collect();
        let written =
            reconstruct_field_picture_16x8_macroblock(&mut dest, refs, 0, 0, motion, &blocks)
                .unwrap();
        assert_eq!(written, 6);
        for k in 0..16usize {
            let expected = ((2 * k * 4) as i32 + 3).clamp(0, 255) as u8;
            assert_eq!(dest.y.get(0, k), Some(expected), "field line {k}");
        }
    }

    // ---- dual-prime prediction (§7.6.3.6 / §7.6.7.4, Table 7-13 / 7-14
    //      `Dual prime`) ----

    /// A 16×32 4:2:0 reference frame whose top field (even frame rows) is
    /// a solid `top` and whose bottom field (odd frame rows) is a solid
    /// `bottom`, so a dual-prime same-/opposite-parity average is directly
    /// observable from the two field constants.
    fn parity_split_frame(top: u8, bottom: u8) -> FrameBuffer {
        let cf = ChromaFormat::Yuv420;
        let mut f = FrameBuffer::new(16, 32, cf);
        for y in 0..32usize {
            let v = if y % 2 == 0 { top } else { bottom };
            for x in 0..16 {
                f.y.put_sample(x, y, v);
            }
        }
        for y in 0..16usize {
            for x in 0..8 {
                f.cb.put_sample(x, y, 0);
                f.cr.put_sample(x, y, 0);
            }
        }
        f
    }

    #[test]
    fn field_picture_dual_prime_averages_same_and_opposite_parity_fields() {
        // Predicting the top field with a zero same-parity vector and a
        // zero opposite-parity vector reads the top field (80) and the
        // bottom field (160); §7.6.7.4 averages them: (80 + 160)//2 = 120.
        let reference = parity_split_frame(80, 160);
        let dest = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
        let motion = FieldPictureDualPrimeMotion::new(
            FieldParity::Top,
            MotionVectorPel::new(0, 0),
            MotionVectorPel::new(0, 0),
        );
        let (luma, _, _) =
            predict_field_picture_dual_prime_macroblock_planes(&dest, &reference, 0, 0, motion)
                .unwrap();
        assert_eq!(luma.len(), 256);
        assert!(luma.iter().all(|&p| p == 120), "dual-prime average");
    }

    #[test]
    fn field_picture_dual_prime_bottom_field_swaps_parities() {
        // Predicting the bottom field: same parity = bottom field (160),
        // opposite parity = top field (80). The average is identical
        // (120) but reading from the swapped parities proves the
        // same/opposite selection follows the predicted field's parity.
        let reference = parity_split_frame(80, 160);
        let dest = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
        let motion = FieldPictureDualPrimeMotion::new(
            FieldParity::Bottom,
            MotionVectorPel::new(0, 0),
            MotionVectorPel::new(0, 0),
        );
        let (luma, _, _) =
            predict_field_picture_dual_prime_macroblock_planes(&dest, &reference, 0, 0, motion)
                .unwrap();
        assert!(luma.iter().all(|&p| p == 120));
    }

    #[test]
    fn field_picture_dual_prime_geometry_mismatch_errors() {
        let reference = solid_frame(100, 16, 16, ChromaFormat::Yuv420);
        let dest = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
        let motion = FieldPictureDualPrimeMotion::new(
            FieldParity::Top,
            MotionVectorPel::new(0, 0),
            MotionVectorPel::new(0, 0),
        );
        assert_eq!(
            predict_field_picture_dual_prime_macroblock_planes(&dest, &reference, 0, 0, motion),
            Err(InterError::ReferenceGeometryMismatch)
        );
    }

    #[test]
    fn field_picture_dual_prime_reconstruct_adds_residual() {
        let reference = parity_split_frame(80, 160);
        let mut dest = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
        let motion = FieldPictureDualPrimeMotion::new(
            FieldParity::Top,
            MotionVectorPel::new(0, 0),
            MotionVectorPel::new(0, 0),
        );
        let residual = [[5i16; 8]; 8];
        let blocks: Vec<ResidualBlock<'_>> = (0..4)
            .map(|i| ResidualBlock {
                block_index: i,
                f_pel: &residual,
            })
            .collect();
        let written = reconstruct_field_picture_dual_prime_macroblock(
            &mut dest, &reference, 0, 0, motion, &blocks,
        )
        .unwrap();
        assert_eq!(written, 6);
        // Each luma sample = average(120) + 5 residual = 125.
        for k in 0..16usize {
            assert_eq!(dest.y.get(0, k), Some(125), "field line {k}");
        }
    }

    /// A 16×16 4:2:0 frame whose top field (even rows) is `top` and bottom
    /// field (odd rows) is `bottom` — the frame-picture analogue of
    /// [`parity_split_frame`] for the dual-prime four-field interleave.
    fn parity_split_frame16(top: u8, bottom: u8) -> FrameBuffer {
        let mut f = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
        for y in 0..16usize {
            let v = if y % 2 == 0 { top } else { bottom };
            for x in 0..16 {
                f.y.put_sample(x, y, v);
            }
        }
        for y in 0..8 {
            for x in 0..8 {
                f.cb.put_sample(x, y, 0);
                f.cr.put_sample(x, y, 0);
            }
        }
        f
    }

    #[test]
    fn frame_dual_prime_zero_vectors_interleave_field_averages() {
        // Frame picture: each field of the predicted frame averages its
        // same-parity and opposite-parity references. With all-zero
        // vectors the top field = avg(top=80, bottom=160) = 120 and the
        // bottom field = avg(bottom=160, top=80) = 120, so the whole
        // 16×16 frame prediction is 120 — but the values come from the
        // four-field interleave path, not a single block read.
        let reference = parity_split_frame16(80, 160);
        let dest = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
        let motion = FrameDualPrimeMotion::new(
            MotionVectorPel::new(0, 0),
            MotionVectorPel::new(0, 0),
            MotionVectorPel::new(0, 0),
        );
        let (luma, _, _) =
            predict_frame_dual_prime_macroblock_planes(&dest, &reference, 0, 0, motion).unwrap();
        assert_eq!(luma.len(), 256);
        assert!(luma.iter().all(|&p| p == 120), "frame dual-prime average");
    }

    #[test]
    fn frame_dual_prime_distinct_opposite_vectors_shift_each_field() {
        // The top predicted field uses vector'[2]; the bottom uses
        // vector'[3]. Give the bottom field's opposite vector a +2
        // half-sample (= +1 field line) vertical shift so the two fields
        // diverge, proving each field consumes its own derived vector.
        let mut reference = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
        // Vertical ramp so a field-line shift is observable: frame row y
        // -> value y*4.
        for y in 0..16usize {
            for x in 0..16 {
                reference.y.put_sample(x, y, (y * 4) as u8);
            }
        }
        for y in 0..8 {
            for x in 0..8 {
                reference.cb.put_sample(x, y, 0);
                reference.cr.put_sample(x, y, 0);
            }
        }
        let dest = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
        // same vector = 0, top opposite = 0, bottom opposite = +2 (one
        // field line down on the top reference field it reads).
        let motion = FrameDualPrimeMotion::new(
            MotionVectorPel::new(0, 0),
            MotionVectorPel::new(0, 0),
            MotionVectorPel::new(0, 2),
        );
        let (luma, _, _) =
            predict_frame_dual_prime_macroblock_planes(&dest, &reference, 0, 0, motion).unwrap();
        // Top predicted field (even frame rows 2k): same parity top field
        // line k = frame row 2k (value 8k); opposite parity bottom field
        // line k = frame row 2k+1 (value 8k+4). Average = 8k+2.
        for k in 0..8usize {
            let expected = (2 * k * 4) as u32; // top field line k value
            let opp = ((2 * k + 1) * 4) as u32; // bottom field line k value
            let avg = expected.midpoint(opp) as u8;
            assert_eq!(luma[(2 * k) * 16], avg, "top field row {k}");
        }
        // Bottom predicted field (odd frame rows 2k+1): same parity bottom
        // field line k = frame row 2k+1 (value 8k+4); opposite parity top
        // field with +1 field-line shift reads top line k+1 = frame row
        // 2(k+1) (value 8k+8), clamped at the last top line (k=7 -> 7).
        for k in 0..8usize {
            let same = ((2 * k + 1) * 4) as u32; // bottom field line k
            let opp_line = (k + 1).min(7);
            let opp = (2 * opp_line * 4) as u32; // top field line k+1
            let avg = same.midpoint(opp) as u8;
            assert_eq!(luma[(2 * k + 1) * 16], avg, "bottom field row {k}");
        }
    }

    #[test]
    fn frame_dual_prime_geometry_mismatch_errors() {
        let reference = solid_frame(100, 32, 16, ChromaFormat::Yuv420);
        let dest = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
        let motion = FrameDualPrimeMotion::new(
            MotionVectorPel::new(0, 0),
            MotionVectorPel::new(0, 0),
            MotionVectorPel::new(0, 0),
        );
        assert_eq!(
            predict_frame_dual_prime_macroblock_planes(&dest, &reference, 0, 0, motion),
            Err(InterError::ReferenceGeometryMismatch)
        );
    }

    #[test]
    fn frame_dual_prime_reconstruct_adds_residual_with_field_dct() {
        let reference = parity_split_frame16(80, 160);
        let mut dest = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
        let motion = FrameDualPrimeMotion::new(
            MotionVectorPel::new(0, 0),
            MotionVectorPel::new(0, 0),
            MotionVectorPel::new(0, 0),
        );
        let residual = [[4i16; 8]; 8];
        let blocks: Vec<ResidualBlock<'_>> = (0..4)
            .map(|i| ResidualBlock {
                block_index: i,
                f_pel: &residual,
            })
            .collect();
        // field_dct = false: frame-organised write-out of the frame-order
        // prediction. Every luma sample = 120 average + 4 = 124.
        let written = reconstruct_frame_dual_prime_macroblock(
            &mut dest, &reference, 0, 0, false, motion, &blocks,
        )
        .unwrap();
        assert_eq!(written, 6);
        for y in 0..16usize {
            assert_eq!(dest.y.get(0, y), Some(124), "row {y}");
        }
    }
}
