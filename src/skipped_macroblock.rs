//! MPEG-2 §7.6.6 skipped-macroblock specification per
//! **ISO/IEC 13818-2 (ITU-T H.262)**.
//!
//! A skipped macroblock is a macroblock for which no data is encoded in
//! the slice. They are surfaced by the §6.2.4 slice walker as the
//! address-gap between `previous_macroblock_address + 1` and
//! `macroblock_address - 1` (recorded per [`crate::MacroblockRecord`]'s
//! `skipped_macroblock_count`). This module turns each such
//! address-gap macroblock into a deterministic specification of the
//! prediction the decoder must form for it, per the four §7.6.6
//! sub-cases:
//!
//! * **§7.6.6.1 P field picture** — prediction as if
//!   `field_motion_type` were `Field-based`; from the field of the
//!   same parity as the field being predicted; PMV reset to zero; the
//!   motion vector is `(0, 0)`.
//! * **§7.6.6.2 P frame picture** — prediction as if
//!   `frame_motion_type` were `Frame-based`; PMV reset to zero; the
//!   motion vector is `(0, 0)`.
//! * **§7.6.6.3 B field picture** — prediction as if
//!   `field_motion_type` were `Field-based`; from the field of the
//!   same parity as the field being predicted; direction
//!   (forward / backward / bidirectional) is the same as the previous
//!   macroblock; PMVs are **unaffected**; motion vectors are taken
//!   from the appropriate motion-vector predictors. Chroma scaling
//!   per §7.6.3.7.
//! * **§7.6.6.4 B frame picture** — prediction as if
//!   `frame_motion_type` were `Frame-based`; direction is the same as
//!   the previous macroblock; PMVs are **unaffected**; motion vectors
//!   are taken from the appropriate motion-vector predictors. Chroma
//!   scaling per §7.6.3.7.
//!
//! The §7.6.6 preamble also says: *"There shall be no skipped
//! macroblocks in I-pictures except when
//! `picture_spatial_scalable_extension()` follows the
//! `picture_header()` of the current picture, or
//! `sequence_scalable_extension()` is present in the bitstream and
//! `scalable_mode = "SNR scalability"`."* This crate does not yet
//! parse the scalability extensions, so the I-picture path is
//! rejected with the spec-cited reason; the
//! [`SkippedMacroblockContext::scalable_i_picture`] gate exposes the
//! exemption for a future scalability round.
//!
//! ## What this module owns
//!
//! [`describe_skipped_macroblock`] turns a [`SkippedMacroblockContext`]
//! into a [`SkippedMacroblock`] description that pins:
//!
//! * the §7.6.6 prediction type (Frame-based or Field-based);
//! * the [`mv_format`](crate::MvFormat);
//! * the field parity to predict from (field pictures only — the
//!   §7.6.6.1 / §7.6.6.3 "same parity" rule);
//! * the [`PredictionDirection`](crate::PredictionDirection) (from the
//!   previous MB in B-pictures, always `Forward` in P-pictures);
//! * the motion-vector source — `(0, 0)` for P-pictures, the
//!   appropriate PMV slot(s) for B-pictures;
//! * whether the caller must `reset_pmv` (P-pictures only — §7.6.3.4
//!   bullet "In a P-picture when a macroblock is skipped" and the
//!   §7.6.6.1 / §7.6.6.2 "Motion vector predictors shall be reset to
//!   zero" lines).
//!
//! [`apply_to_pmv`] is the §7.6.3.4 hook the per-slice driver fires
//! once for each skipped macroblock: in P-pictures it zeroes every
//! PMV slot; in B-pictures it is a no-op (per the §7.6.6.3 /
//! §7.6.6.4 "Motion vector predictors are unaffected" lines).
//!
//! ## What this module **does not** own
//!
//! * Actually forming the prediction sample plane. Once the caller
//!   has the prediction type, MV, parity, and direction, it dispatches
//!   to [`crate::predict_block`] (§7.6.4) and then
//!   [`crate::combine_directional_predictions`] (§7.6.7).
//! * Tracking "previous MB direction" across macroblocks. The
//!   §6.2.4 slice walker does the per-MB state plumbing; this module
//!   just consumes whatever the slice walker provides.
//! * Walking *multiple* skipped macroblocks. The slice walker
//!   surfaces `skipped_macroblock_count` per record; the caller
//!   iterates and calls [`describe_skipped_macroblock`] once per
//!   skipped slot. Each call is independent (P-pictures: identical
//!   description per slot; B-pictures: identical description per
//!   slot because PMVs do not change across skipped MBs in B-pictures).

use crate::combine_predictions::PredictionDirection;
use crate::dual_prime::FieldParity;
use crate::macroblock_modes::{MvFormat, PredictionType};
use crate::picture_header::{PictureCodingType, PictureStructure};
use crate::pmv::{Component, Direction, Pmv, VectorIndex};
use crate::{Error, Result};

/// Inputs the §6.2.4 slice walker hands to [`describe_skipped_macroblock`]
/// for a single skipped macroblock.
///
/// `picture_coding_type` and `picture_structure` come from the
/// `picture_header()` / `picture_coding_extension()` already parsed
/// by [`crate::Mpeg2PictureHeader`] / [`crate::PictureCodingExtension`].
/// `previous_direction` is the direction of the macroblock at
/// `previous_macroblock_address` (i.e. the macroblock immediately
/// before the run of skipped MBs); it is required for B-pictures
/// per §7.6.6.3 / §7.6.6.4 and unused in P-pictures.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SkippedMacroblockContext {
    /// `picture_coding_type` from §6.3.10 / Table 6-12.
    pub picture_coding_type: PictureCodingType,
    /// `picture_structure` from §6.3.11 / Table 6-14.
    pub picture_structure: PictureStructure,
    /// The §6.3.17.1 prediction direction of the macroblock at
    /// `previous_macroblock_address` — i.e. the macroblock immediately
    /// preceding the run of skipped MBs. Required for B-pictures
    /// (§7.6.6.3 / §7.6.6.4 "same as the previous macroblock"); the
    /// value is ignored in P-pictures.
    ///
    /// For B-pictures, the §6.3.17.1 / §7.6.6 wording requires the
    /// previous MB to be a non-intra MB with at least one of
    /// `macroblock_motion_forward` / `macroblock_motion_backward`
    /// set — otherwise the bitstream is malformed (an intra MB
    /// immediately followed by a skipped MB in a B-picture has no
    /// "previous direction" to copy). The caller is responsible for
    /// enforcing that and surfacing the malformed-bitstream error
    /// before calling into this module; the validated direction
    /// passed in here is then `Forward`, `Backward`, or
    /// `Bidirectional`, never `Skipped`.
    pub previous_direction: PredictionDirection,
    /// `true` if the current picture is an I-picture *and* a
    /// `picture_spatial_scalable_extension()` or
    /// `sequence_scalable_extension()` with
    /// `scalable_mode = "SNR scalability"` is in force, in which
    /// case the §7.6.6 preamble allows skipped MBs in the I-picture.
    /// The scalability parsers are not yet in this crate, so the
    /// gate is exposed but a future round must wire it from those
    /// extensions; non-scalable callers leave it `false` and the
    /// I-picture path is rejected per the §7.6.6 preamble's main
    /// rule.
    pub scalable_i_picture: bool,
    /// The PMV state at the entry to the skipped macroblock. In
    /// P-pictures the description does not consult it (the MVs are
    /// `(0, 0)` regardless); in B-pictures it is the source of the
    /// per-direction motion-vector values (§7.6.6.3 / §7.6.6.4
    /// "The motion vectors are taken from the appropriate motion
    /// vector predictors").
    pub pmv: Pmv,
}

/// The §7.6.6 motion-vector source for a single skipped macroblock.
///
/// In P-pictures the spec is unconditional: `vector = (0, 0)`. In
/// B-pictures the §7.6.6.3 / §7.6.6.4 wording says "motion vectors are
/// taken from the appropriate motion-vector predictors"; we surface
/// the PMV `[r][s][t]` slot value(s) the description points to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkippedMotionVector {
    /// `(0, 0)` — the §7.6.6.1 / §7.6.6.2 P-picture rule.
    Zero,
    /// `PMV[0][s][t]` values — the §7.6.6.3 / §7.6.6.4 B-picture rule
    /// where the prediction takes vectors from the PMV slots.
    /// `forward` and `backward` are each `Some((horizontal,
    /// vertical))` in half-sample units when the corresponding
    /// [`SkippedMacroblock::direction`] component is present, `None`
    /// otherwise.
    FromPmv {
        /// `(PMV[0][0][0], PMV[0][0][1])` — present iff the previous
        /// MB's direction includes forward.
        forward: Option<(i32, i32)>,
        /// `(PMV[0][1][0], PMV[0][1][1])` — present iff the previous
        /// MB's direction includes backward.
        backward: Option<(i32, i32)>,
    },
}

/// The §7.6.6 description of a single skipped macroblock — the
/// deterministic prediction the decoder must form for it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SkippedMacroblock {
    /// The prediction type the §7.6.6 case prescribes (Frame-based or
    /// Field-based). Field-based for §7.6.6.1 / §7.6.6.3, Frame-based
    /// for §7.6.6.2 / §7.6.6.4.
    pub prediction_type: PredictionType,
    /// `mv_format` for the prediction — `Field` when
    /// `prediction_type` is `FieldBased`, `Frame` when it is
    /// `FrameBased` (§6.3.17.2 derivation against the §7.6.6 cases).
    pub mv_format: MvFormat,
    /// The reference field parity — `Some(parity)` for §7.6.6.1 /
    /// §7.6.6.3 (field pictures, same-parity rule), `None` for
    /// §7.6.6.2 / §7.6.6.4 (frame pictures).
    pub reference_parity: Option<FieldParity>,
    /// The §7.6.6.3 / §7.6.6.4 "same as the previous macroblock"
    /// direction (B-pictures), or the §7.6.6.1 / §7.6.6.2
    /// `Forward`-with-zero-MV implicit direction (P-pictures).
    pub direction: PredictionDirection,
    /// The motion vector(s) to use for this skipped macroblock.
    pub motion_vector: SkippedMotionVector,
    /// `true` when the §7.6.3.4 bullet "In a P-picture when a
    /// macroblock is skipped" fires (P-pictures only). When this is
    /// `true`, the caller must invoke [`apply_to_pmv`] (or zero the
    /// PMV state by hand) before processing any further macroblock
    /// in the slice. `false` for B-pictures per §7.6.6.3 / §7.6.6.4
    /// "Motion vector predictors are unaffected".
    pub reset_pmv: bool,
}

/// Per the §7.6.6 preamble: derive a [`SkippedMacroblock`] for a
/// single skipped slot at a given context.
///
/// # Errors
///
/// * Returns [`Error::InvalidBitstream`] when
///   `ctx.picture_coding_type == PictureCodingType::Intra` and
///   `ctx.scalable_i_picture` is `false`. The §7.6.6 preamble
///   forbids skipped macroblocks in non-scalable I-pictures.
/// * Returns [`Error::InvalidBitstream`] when the picture is a
///   B-picture and `ctx.previous_direction` is
///   [`PredictionDirection::Skipped`]. §7.6.6.3 / §7.6.6.4 says
///   "the same as the previous macroblock"; the previous MB must
///   itself be a coded non-intra MB with at least one direction
///   set, which the §6.3.17.1 slice walker validates before this
///   module sees it.
pub fn describe_skipped_macroblock(ctx: SkippedMacroblockContext) -> Result<SkippedMacroblock> {
    // §7.6.6 preamble: I-pictures shall have no skipped MBs unless a
    // scalability extension exempts them.
    if matches!(ctx.picture_coding_type, PictureCodingType::Intra) && !ctx.scalable_i_picture {
        return Err(Error::InvalidBitstream(
            "skipped macroblock in non-scalable I-picture (§7.6.6 preamble)",
        ));
    }

    let parity = match ctx.picture_structure {
        PictureStructure::TopField => Some(FieldParity::Top),
        PictureStructure::BottomField => Some(FieldParity::Bottom),
        PictureStructure::Frame => None,
    };

    let (prediction_type, mv_format) = match ctx.picture_structure {
        // §7.6.6.1 P field / §7.6.6.3 B field: Field-based.
        PictureStructure::TopField | PictureStructure::BottomField => {
            (PredictionType::FieldBased, MvFormat::Field)
        }
        // §7.6.6.2 P frame / §7.6.6.4 B frame: Frame-based.
        PictureStructure::Frame => (PredictionType::FrameBased, MvFormat::Frame),
    };

    match ctx.picture_coding_type {
        PictureCodingType::Predictive => {
            // §7.6.6.1 / §7.6.6.2: MV = (0, 0), PMVs reset, direction
            // is implicitly forward (the spec wording "the motion
            // vector shall be zero" with no direction qualifier
            // describes the forward zero-MV prediction the decoder
            // forms — same shape as §7.6.3.5's implicit forward
            // zero-MV reset for non-intra P MBs with no MVs encoded).
            Ok(SkippedMacroblock {
                prediction_type,
                mv_format,
                reference_parity: parity,
                direction: PredictionDirection::Forward,
                motion_vector: SkippedMotionVector::Zero,
                reset_pmv: true,
            })
        }
        PictureCodingType::Bidirectional => {
            // §7.6.6.3 / §7.6.6.4: direction is the same as the
            // previous MB; MVs come from the appropriate PMV slots;
            // PMVs themselves are unaffected.
            let previous_direction = ctx.previous_direction;
            if matches!(previous_direction, PredictionDirection::Skipped) {
                return Err(Error::InvalidBitstream(
                    "skipped macroblock in B-picture: previous macroblock has no encoded direction (§7.6.6.3 / §7.6.6.4)",
                ));
            }

            let pmv0 = |s: Direction| -> (i32, i32) {
                (
                    ctx.pmv.get(VectorIndex::First, s, Component::Horizontal),
                    ctx.pmv.get(VectorIndex::First, s, Component::Vertical),
                )
            };

            let (forward, backward) = match previous_direction {
                PredictionDirection::Forward => (Some(pmv0(Direction::Forward)), None),
                PredictionDirection::Backward => (None, Some(pmv0(Direction::Backward))),
                PredictionDirection::Bidirectional => (
                    Some(pmv0(Direction::Forward)),
                    Some(pmv0(Direction::Backward)),
                ),
                PredictionDirection::Skipped => unreachable!("rejected above"),
            };

            Ok(SkippedMacroblock {
                prediction_type,
                mv_format,
                reference_parity: parity,
                direction: previous_direction,
                motion_vector: SkippedMotionVector::FromPmv { forward, backward },
                reset_pmv: false,
            })
        }
        PictureCodingType::Intra => {
            // Reachable only when scalable_i_picture is true. The
            // §7.6.6 preamble names the two extensions; the spec
            // does not give a sub-clause for the prediction
            // formation in that case (the scalability annexes
            // own it), so this round refuses to synthesise a
            // description.
            Err(Error::InvalidBitstream(
                "skipped macroblock in scalable I-picture: prediction formation defined by the scalability extensions, not §7.6.6 (not yet supported)",
            ))
        }
    }
}

/// Apply the §7.6.6 / §7.6.3.4 PMV side-effect for a single skipped
/// macroblock. In P-pictures this zeroes every PMV slot per the
/// §7.6.6.1 / §7.6.6.2 "Motion vector predictors shall be reset to
/// zero" line. In B-pictures this is a no-op per §7.6.6.3 /
/// §7.6.6.4 "Motion vector predictors are unaffected".
///
/// Idempotent: invoking it on a description that already had
/// `reset_pmv = false` leaves the PMV state untouched.
pub fn apply_to_pmv(description: &SkippedMacroblock, pmv: &mut Pmv) {
    if description.reset_pmv {
        pmv.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pmv::{Component, Direction, VectorIndex};

    fn p_field_top_ctx() -> SkippedMacroblockContext {
        SkippedMacroblockContext {
            picture_coding_type: PictureCodingType::Predictive,
            picture_structure: PictureStructure::TopField,
            previous_direction: PredictionDirection::Forward,
            scalable_i_picture: false,
            pmv: Pmv::new(),
        }
    }

    fn p_frame_ctx() -> SkippedMacroblockContext {
        SkippedMacroblockContext {
            picture_coding_type: PictureCodingType::Predictive,
            picture_structure: PictureStructure::Frame,
            previous_direction: PredictionDirection::Forward,
            scalable_i_picture: false,
            pmv: Pmv::new(),
        }
    }

    fn b_frame_ctx_with_direction(dir: PredictionDirection) -> SkippedMacroblockContext {
        SkippedMacroblockContext {
            picture_coding_type: PictureCodingType::Bidirectional,
            picture_structure: PictureStructure::Frame,
            previous_direction: dir,
            scalable_i_picture: false,
            pmv: Pmv::new(),
        }
    }

    #[test]
    fn p_field_top_describes_field_based_same_parity_zero_mv_with_pmv_reset() {
        // §7.6.6.1: P field picture (top) → Field-based, parity Top
        // (same as field being predicted), MV = (0, 0), PMV reset.
        let desc = describe_skipped_macroblock(p_field_top_ctx()).unwrap();
        assert_eq!(desc.prediction_type, PredictionType::FieldBased);
        assert_eq!(desc.mv_format, MvFormat::Field);
        assert_eq!(desc.reference_parity, Some(FieldParity::Top));
        assert_eq!(desc.direction, PredictionDirection::Forward);
        assert_eq!(desc.motion_vector, SkippedMotionVector::Zero);
        assert!(desc.reset_pmv);
    }

    #[test]
    fn p_field_bottom_describes_field_based_with_bottom_parity() {
        let mut ctx = p_field_top_ctx();
        ctx.picture_structure = PictureStructure::BottomField;
        let desc = describe_skipped_macroblock(ctx).unwrap();
        assert_eq!(desc.prediction_type, PredictionType::FieldBased);
        assert_eq!(desc.reference_parity, Some(FieldParity::Bottom));
    }

    #[test]
    fn p_frame_describes_frame_based_no_parity_zero_mv_with_pmv_reset() {
        // §7.6.6.2: P frame picture → Frame-based, no parity,
        // MV = (0, 0), PMV reset.
        let desc = describe_skipped_macroblock(p_frame_ctx()).unwrap();
        assert_eq!(desc.prediction_type, PredictionType::FrameBased);
        assert_eq!(desc.mv_format, MvFormat::Frame);
        assert_eq!(desc.reference_parity, None);
        assert_eq!(desc.direction, PredictionDirection::Forward);
        assert_eq!(desc.motion_vector, SkippedMotionVector::Zero);
        assert!(desc.reset_pmv);
    }

    #[test]
    fn b_frame_inherits_previous_direction_forward_and_pmv_is_unaffected() {
        // §7.6.6.4: B frame picture, previous MB direction = Forward
        // → take forward MV from PMV[0][0][0..1]; PMVs unaffected.
        let mut ctx = b_frame_ctx_with_direction(PredictionDirection::Forward);
        ctx.pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Horizontal,
            7,
        );
        ctx.pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Vertical,
            -3,
        );
        let desc = describe_skipped_macroblock(ctx).unwrap();
        assert_eq!(desc.prediction_type, PredictionType::FrameBased);
        assert_eq!(desc.mv_format, MvFormat::Frame);
        assert_eq!(desc.reference_parity, None);
        assert_eq!(desc.direction, PredictionDirection::Forward);
        assert_eq!(
            desc.motion_vector,
            SkippedMotionVector::FromPmv {
                forward: Some((7, -3)),
                backward: None,
            }
        );
        assert!(!desc.reset_pmv);
    }

    #[test]
    fn b_frame_inherits_previous_direction_backward_and_reads_only_backward_pmv() {
        let mut ctx = b_frame_ctx_with_direction(PredictionDirection::Backward);
        ctx.pmv.set(
            VectorIndex::First,
            Direction::Backward,
            Component::Horizontal,
            11,
        );
        ctx.pmv.set(
            VectorIndex::First,
            Direction::Backward,
            Component::Vertical,
            22,
        );
        // Also stash a forward value to make sure it is NOT
        // surfaced when the previous direction is backward-only.
        ctx.pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Horizontal,
            999,
        );
        let desc = describe_skipped_macroblock(ctx).unwrap();
        assert_eq!(desc.direction, PredictionDirection::Backward);
        assert_eq!(
            desc.motion_vector,
            SkippedMotionVector::FromPmv {
                forward: None,
                backward: Some((11, 22)),
            }
        );
    }

    #[test]
    fn b_frame_inherits_bidirectional_direction_and_reads_both_pmv_slots() {
        let mut ctx = b_frame_ctx_with_direction(PredictionDirection::Bidirectional);
        ctx.pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Horizontal,
            1,
        );
        ctx.pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Vertical,
            2,
        );
        ctx.pmv.set(
            VectorIndex::First,
            Direction::Backward,
            Component::Horizontal,
            -4,
        );
        ctx.pmv.set(
            VectorIndex::First,
            Direction::Backward,
            Component::Vertical,
            -5,
        );
        let desc = describe_skipped_macroblock(ctx).unwrap();
        assert_eq!(desc.direction, PredictionDirection::Bidirectional);
        assert_eq!(
            desc.motion_vector,
            SkippedMotionVector::FromPmv {
                forward: Some((1, 2)),
                backward: Some((-4, -5)),
            }
        );
    }

    #[test]
    fn b_field_top_picks_top_parity_and_inherits_direction() {
        let mut ctx = b_frame_ctx_with_direction(PredictionDirection::Forward);
        ctx.picture_structure = PictureStructure::TopField;
        ctx.pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Horizontal,
            8,
        );
        let desc = describe_skipped_macroblock(ctx).unwrap();
        assert_eq!(desc.prediction_type, PredictionType::FieldBased);
        assert_eq!(desc.mv_format, MvFormat::Field);
        assert_eq!(desc.reference_parity, Some(FieldParity::Top));
        assert_eq!(desc.direction, PredictionDirection::Forward);
    }

    #[test]
    fn b_field_bottom_picks_bottom_parity() {
        let mut ctx = b_frame_ctx_with_direction(PredictionDirection::Forward);
        ctx.picture_structure = PictureStructure::BottomField;
        let desc = describe_skipped_macroblock(ctx).unwrap();
        assert_eq!(desc.reference_parity, Some(FieldParity::Bottom));
    }

    #[test]
    fn i_picture_non_scalable_rejected() {
        // §7.6.6 preamble: I-pictures shall have no skipped MBs in
        // the non-scalable case.
        let ctx = SkippedMacroblockContext {
            picture_coding_type: PictureCodingType::Intra,
            picture_structure: PictureStructure::Frame,
            previous_direction: PredictionDirection::Forward,
            scalable_i_picture: false,
            pmv: Pmv::new(),
        };
        let err = describe_skipped_macroblock(ctx).unwrap_err();
        assert!(matches!(
            err,
            Error::InvalidBitstream(msg) if msg.contains("non-scalable I-picture")
        ));
    }

    #[test]
    fn i_picture_scalable_currently_unsupported() {
        // The exemption gate is open, but the scalability annexes
        // are not yet wired — surface as unsupported, not as a
        // bitstream violation. We tag it as InvalidBitstream with
        // the explicit "not yet supported" suffix so the spec
        // citation is preserved.
        let ctx = SkippedMacroblockContext {
            picture_coding_type: PictureCodingType::Intra,
            picture_structure: PictureStructure::Frame,
            previous_direction: PredictionDirection::Forward,
            scalable_i_picture: true,
            pmv: Pmv::new(),
        };
        let err = describe_skipped_macroblock(ctx).unwrap_err();
        assert!(matches!(
            err,
            Error::InvalidBitstream(msg) if msg.contains("not yet supported")
        ));
    }

    #[test]
    fn b_picture_with_skipped_previous_direction_rejected() {
        // §7.6.6.3 / §7.6.6.4: the previous MB must have a
        // direction the skipped MB can copy. A "previous MB had
        // no encoded direction" feed is a malformed bitstream.
        let ctx = b_frame_ctx_with_direction(PredictionDirection::Skipped);
        let err = describe_skipped_macroblock(ctx).unwrap_err();
        assert!(matches!(
            err,
            Error::InvalidBitstream(msg) if msg.contains("previous macroblock has no encoded direction")
        ));
    }

    #[test]
    fn apply_to_pmv_zeroes_in_p_picture() {
        let mut pmv = Pmv::new();
        pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Horizontal,
            100,
        );
        pmv.set(
            VectorIndex::Second,
            Direction::Backward,
            Component::Vertical,
            -50,
        );
        let desc = describe_skipped_macroblock(p_frame_ctx()).unwrap();
        apply_to_pmv(&desc, &mut pmv);
        // §7.6.3.4 / §7.6.6.2: every slot reset to zero.
        assert_eq!(
            pmv.get(
                VectorIndex::First,
                Direction::Forward,
                Component::Horizontal
            ),
            0
        );
        assert_eq!(
            pmv.get(
                VectorIndex::Second,
                Direction::Backward,
                Component::Vertical
            ),
            0
        );
    }

    #[test]
    fn apply_to_pmv_is_noop_in_b_picture() {
        // §7.6.6.3 / §7.6.6.4: PMVs unaffected.
        let mut pmv = Pmv::new();
        pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Horizontal,
            42,
        );
        let mut ctx = b_frame_ctx_with_direction(PredictionDirection::Forward);
        ctx.pmv = pmv;
        let desc = describe_skipped_macroblock(ctx).unwrap();
        apply_to_pmv(&desc, &mut pmv);
        assert_eq!(
            pmv.get(
                VectorIndex::First,
                Direction::Forward,
                Component::Horizontal
            ),
            42
        );
    }

    #[test]
    fn p_picture_reset_pmv_is_idempotent() {
        // Calling apply_to_pmv twice does not corrupt the zeroed
        // state.
        let mut pmv = Pmv::new();
        pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Horizontal,
            17,
        );
        let desc = describe_skipped_macroblock(p_frame_ctx()).unwrap();
        apply_to_pmv(&desc, &mut pmv);
        apply_to_pmv(&desc, &mut pmv);
        assert_eq!(
            pmv.get(
                VectorIndex::First,
                Direction::Forward,
                Component::Horizontal
            ),
            0
        );
    }

    #[test]
    fn field_parity_index_matches_spec_numbering() {
        // §7.6.3.6 wording: "top field has parity zero, the bottom
        // field has parity one".
        assert_eq!(FieldParity::Top.index(), 0);
        assert_eq!(FieldParity::Bottom.index(), 1);
    }
}
