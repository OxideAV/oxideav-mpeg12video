//! §7.6.5 Motion vector selection per ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262), page 101–102 — Tables 7-13 (field pictures) and 7-14
//! (frame pictures).
//!
//! ## What §7.6.5 specifies
//!
//! Once the motion vectors of a macroblock have been reconstructed
//! (§7.6.3, the `vector'[r][s][1:0]` array) and before the per-sample
//! pel reader runs (§7.6.4), the decoder must decide **which** of the
//! up-to-four reconstructed vectors are used, **which reference**
//! (field or frame, and for dual-prime which parity) each prediction
//! is formed from, and **which region** of the macroblock each
//! prediction covers (the full 16×16, or the upper / lower 16×8 half
//! for `16x8 MC`). Tables 7-13 / 7-14 enumerate that mapping keyed off
//! the picture structure, the prediction type
//! (`field_motion_type` / `frame_motion_type`), and the three
//! `macroblock_type` flags (`macroblock_motion_forward`,
//! `macroblock_motion_backward`, `macroblock_intra`).
//!
//! This module is the **table driver**: it turns one parsed macroblock
//! prediction descriptor into the ordered list of [`PredictionOp`]s the
//! §7.6.4 [`crate::predict_block`] reader must perform, in the order the
//! vectors appear in the bitstream (the table NOTE: *"Motion vectors are
//! listed in the order they appear in the bitstream"*). It does **not**
//! read samples (that is §7.6.4), reconstruct vectors (§7.6.3), derive
//! the dual-prime opposite-parity vector (§7.6.3.6, see
//! [`crate::dual_prime`]), or scale for chrominance (§7.6.3.7, see
//! [`crate::pmv::scale_chroma`]). It is the glue that names what the
//! already-landed endpoints operate on.
//!
//! ## Reference targets
//!
//! [`ReferenceTarget`] captures the *"Prediction formed for"* column:
//!
//! * [`ReferenceTarget::Frame`] — the whole reference frame (the
//!   frame-picture `Frame-based` rows of Table 7-14).
//! * [`ReferenceTarget::WholeField`] — a whole reference field of the
//!   stated parity. In a **field** picture the parity is signalled by
//!   `motion_vertical_field_select` (§7.6.4: *"If
//!   motion_vertical_field_select is zero then the prediction is taken
//!   from the top reference field"*); the table rows themselves do not
//!   fix the parity, so the caller supplies it via
//!   [`MacroblockPrediction::field_select`]. The frame-picture
//!   `Field-based` rows always pair the first vector with the **top**
//!   field and the second with the **bottom** field.
//! * [`ReferenceTarget::DualPrimeSameParity`] /
//!   [`ReferenceTarget::DualPrimeOppositeParity`] — the two dual-prime
//!   predictions of §7.6.3.6. The opposite-parity prediction uses the
//!   derived `vector'[2][0]` / `vector'[3][0]` (the `*` / `†`
//!   table footnotes: *"not present in the bitstream … derived from
//!   vector'[0][0][1:0] as described in 7.6.3.6"*).
//!
//! ## Block geometry
//!
//! [`PredictionRegion`] captures *which part of the 16×16 macroblock*
//! a prediction covers:
//!
//! * [`PredictionRegion::Whole`] — the full 16×16 luma block (every
//!   Table 7-13 / 7-14 row except `16x8 MC`).
//! * [`PredictionRegion::Upper16x8`] / [`PredictionRegion::Lower16x8`]
//!   — the upper / lower 16×8 halves of the `16x8 MC` rows.
//!
//! The luma [`crate::BlockSize`] each region maps to is exposed by
//! [`PredictionRegion::luma_block_size`]; the per-region vertical
//! offset (0 for the whole / upper region, 8 for the lower region) by
//! [`PredictionRegion::luma_top_offset`].
//!
//! Spec citations: **ISO/IEC 13818-2 (H.262)** §7.6.4 (the
//! `motion_vertical_field_select` semantics), §7.6.5 (Tables 7-13 /
//! 7-14), and §7.6.3.5 / §7.6.3.6 (the table footnotes' implicit-zero
//! and dual-prime-derived vectors).

use crate::dual_prime::FieldParity;
use crate::forming_predictions::BlockSize;
use crate::macroblock_modes::PredictionType;
use crate::picture_header::PictureStructure;
use crate::pmv::{Direction, VectorIndex};
use crate::{Error, Result};

/// The reference a single §7.6.5 prediction is formed from — the
/// *"Prediction formed for"* column of Tables 7-13 / 7-14.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReferenceTarget {
    /// The whole reference frame. Frame-picture `Frame-based` rows
    /// (Table 7-14).
    Frame,
    /// A whole reference field of the stated parity. Field-picture
    /// rows (Table 7-13, parity from `motion_vertical_field_select`)
    /// and the frame-picture `Field-based` rows (Table 7-14, top
    /// field for the first vector, bottom field for the second).
    WholeField(FieldParity),
    /// The dual-prime same-parity prediction (§7.6.3.6). The vector is
    /// `vector'[0][0][1:0]`; the reference is the field of the stated
    /// parity.
    DualPrimeSameParity(FieldParity),
    /// The dual-prime opposite-parity prediction (§7.6.3.6). The
    /// vector is the **derived** `vector'[2][0][1:0]` (field picture)
    /// or `vector'[2][0]` / `vector'[3][0]` (frame picture) — *"not
    /// present in the bitstream … derived from vector'[0][0][1:0]"*.
    DualPrimeOppositeParity(FieldParity),
}

/// Which part of the 16×16 macroblock a single prediction covers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PredictionRegion {
    /// The full 16×16 luma block (every row except `16x8 MC`).
    Whole,
    /// The upper 16×8 half (the first vector of a `16x8 MC` row).
    Upper16x8,
    /// The lower 16×8 half (the second vector of a `16x8 MC` row).
    Lower16x8,
}

impl PredictionRegion {
    /// The luma [`BlockSize`] this region covers (16×16 for
    /// [`PredictionRegion::Whole`], 16×8 for the two halves).
    pub fn luma_block_size(self) -> BlockSize {
        match self {
            PredictionRegion::Whole => BlockSize::new(16, 16).expect("16x16 is valid"),
            PredictionRegion::Upper16x8 | PredictionRegion::Lower16x8 => {
                BlockSize::new(16, 8).expect("16x8 is valid")
            }
        }
    }

    /// The vertical offset (in luma samples) of this region from the
    /// top of the macroblock — `0` for [`PredictionRegion::Whole`] and
    /// [`PredictionRegion::Upper16x8`], `8` for
    /// [`PredictionRegion::Lower16x8`].
    pub fn luma_top_offset(self) -> i32 {
        match self {
            PredictionRegion::Whole | PredictionRegion::Upper16x8 => 0,
            PredictionRegion::Lower16x8 => 8,
        }
    }
}

/// One prediction operation: the §7.6.4 pel reader is invoked once per
/// [`PredictionOp`], reading the [`PredictionOp::region`] of the
/// macroblock from the [`PredictionOp::reference`] using the
/// reconstructed vector named by [`PredictionOp::vector_index`] /
/// [`PredictionOp::direction`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PredictionOp {
    /// `r` — which of the up-to-two reconstructed motion vectors in
    /// the `vector'[r][s][1:0]` array this prediction uses. For the
    /// dual-prime opposite-parity ops `r` names the **derived** slot
    /// (`Second` for `vector'[2][0]`, etc.) the §7.6.3.6 derivation
    /// writes; the caller is responsible for having run that
    /// derivation before forming the prediction.
    pub vector_index: VectorIndex,
    /// `s` — forward (`0`) or backward (`1`).
    pub direction: Direction,
    /// The reference this prediction is formed from.
    pub reference: ReferenceTarget,
    /// Which part of the macroblock this prediction covers.
    pub region: PredictionRegion,
}

/// The parsed inputs §7.6.5 needs to select the prediction set for one
/// non-intra (or intra-with-concealment) macroblock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MacroblockPrediction {
    /// The picture structure (§6.3.10) — selects Table 7-13 (field
    /// pictures) vs Table 7-14 (frame pictures).
    pub picture_structure: PictureStructure,
    /// The resolved prediction type (`field_motion_type` /
    /// `frame_motion_type`, §6.3.17.2 / Table 6-17 / Table 6-18).
    pub prediction_type: PredictionType,
    /// `macroblock_motion_forward` (§6.3.17.1).
    pub macroblock_motion_forward: bool,
    /// `macroblock_motion_backward` (§6.3.17.1).
    pub macroblock_motion_backward: bool,
    /// `macroblock_intra` (§6.3.17.1). When `true` the only operation
    /// produced is the §7.6.3.9 concealment-motion-vector prediction
    /// (a single Field-/Frame-based forward op on `vector'[0][0]`),
    /// and only if `concealment_present` is set.
    pub macroblock_intra: bool,
    /// `concealment_motion_vectors` (§6.3.11). Gates the intra-MB
    /// concealment prediction op.
    pub concealment_present: bool,
    /// `motion_vertical_field_select[0][s]` for the first vector — the
    /// field parity the §7.6.4 reader uses in a **field** picture.
    /// Ignored in frame pictures (the table fixes top/bottom there).
    pub field_select: FieldParity,
    /// `motion_vertical_field_select[1][s]` for the second vector of a
    /// `16x8 MC` field-picture row. Ignored elsewhere.
    pub field_select_lower: FieldParity,
}

/// Error-message constant: an intra macroblock without
/// `concealment_motion_vectors` set has no §7.6.5 prediction — the
/// table's intra rows only apply when a concealment motion vector is
/// present (§7.6.3.9). A normal intra MB forms no prediction (its
/// reconstruction is `d = saturate(f)` per §7.6.8).
const ERR_INTRA_WITHOUT_CONCEALMENT: &str =
    "§7.6.5: intra macroblock without concealment_motion_vectors forms no prediction";
/// Error-message constant: `Dual-Prime` is only defined for
/// forward-only macroblocks in P-pictures (Tables 7-13 / 7-14 list it
/// solely at `forward=1, backward=0`).
const ERR_DUAL_PRIME_NOT_FORWARD_ONLY: &str =
    "§7.6.5: dual-prime prediction requires forward-only motion (forward=1, backward=0)";
/// Error-message constant: a non-intra macroblock with neither motion
/// flag set is the §7.6.3.5 implicit-zero (skipped) case the table
/// marks `*§`; it is handled by [`crate::skipped_macroblock`], not
/// here.
const ERR_NO_MOTION_FLAGS: &str =
    "§7.6.5: non-intra macroblock with no motion flags is the skipped/implicit-zero case";

/// §7.6.5: select the ordered list of [`PredictionOp`]s for one
/// macroblock, per Table 7-13 (field pictures) / Table 7-14 (frame
/// pictures).
///
/// The operations are returned in bitstream order (the table NOTE).
/// Forward vectors precede backward vectors within a row, and for the
/// two-region `16x8 MC` rows the upper-region vector precedes the
/// lower-region vector.
pub fn select_predictions(mb: &MacroblockPrediction) -> Result<Vec<PredictionOp>> {
    if mb.macroblock_intra {
        // §7.6.3.9 / Tables 7-13 / 7-14 intra rows: a single
        // Field-/Frame-based forward prediction on vector'[0][0], used
        // only for concealment.
        if !mb.concealment_present {
            return Err(Error::InvalidBitstream(ERR_INTRA_WITHOUT_CONCEALMENT));
        }
        let reference = match mb.picture_structure {
            PictureStructure::Frame => ReferenceTarget::Frame,
            PictureStructure::TopField | PictureStructure::BottomField => {
                ReferenceTarget::WholeField(mb.field_select)
            }
        };
        return Ok(vec![PredictionOp {
            vector_index: VectorIndex::First,
            direction: Direction::Forward,
            reference,
            region: PredictionRegion::Whole,
        }]);
    }

    let fwd = mb.macroblock_motion_forward;
    let bwd = mb.macroblock_motion_backward;

    if mb.prediction_type == PredictionType::DualPrime {
        if !fwd || bwd {
            return Err(Error::InvalidBitstream(ERR_DUAL_PRIME_NOT_FORWARD_ONLY));
        }
        return Ok(dual_prime_ops(mb.picture_structure));
    }

    if !fwd && !bwd {
        return Err(Error::InvalidBitstream(ERR_NO_MOTION_FLAGS));
    }

    let mut ops = Vec::new();
    // Forward ops precede backward ops (bitstream order). Within a
    // direction, the upper-region vector (or whole-block vector)
    // precedes the lower-region vector.
    if fwd {
        push_directional_ops(&mut ops, mb, Direction::Forward);
    }
    if bwd {
        push_directional_ops(&mut ops, mb, Direction::Backward);
    }
    Ok(ops)
}

/// Push the §7.6.5 prediction op(s) for one direction of a
/// non-dual-prime, non-intra macroblock.
fn push_directional_ops(
    ops: &mut Vec<PredictionOp>,
    mb: &MacroblockPrediction,
    direction: Direction,
) {
    match (mb.picture_structure, mb.prediction_type) {
        // ---- Table 7-14 frame pictures ----
        (PictureStructure::Frame, PredictionType::FrameBased) => {
            ops.push(PredictionOp {
                vector_index: VectorIndex::First,
                direction,
                reference: ReferenceTarget::Frame,
                region: PredictionRegion::Whole,
            });
        }
        (PictureStructure::Frame, PredictionType::FieldBased) => {
            // First vector → top field, second vector → bottom field.
            ops.push(PredictionOp {
                vector_index: VectorIndex::First,
                direction,
                reference: ReferenceTarget::WholeField(FieldParity::Top),
                region: PredictionRegion::Whole,
            });
            ops.push(PredictionOp {
                vector_index: VectorIndex::Second,
                direction,
                reference: ReferenceTarget::WholeField(FieldParity::Bottom),
                region: PredictionRegion::Whole,
            });
        }
        // ---- Table 7-13 field pictures ----
        (
            PictureStructure::TopField | PictureStructure::BottomField,
            PredictionType::FieldBased,
        ) => {
            // Whole field; parity from motion_vertical_field_select.
            ops.push(PredictionOp {
                vector_index: VectorIndex::First,
                direction,
                reference: ReferenceTarget::WholeField(mb.field_select),
                region: PredictionRegion::Whole,
            });
        }
        (
            PictureStructure::TopField | PictureStructure::BottomField,
            PredictionType::SixteenByEight,
        ) => {
            // Two field vectors: upper 16x8 then lower 16x8, each with
            // its own motion_vertical_field_select parity.
            ops.push(PredictionOp {
                vector_index: VectorIndex::First,
                direction,
                reference: ReferenceTarget::WholeField(mb.field_select),
                region: PredictionRegion::Upper16x8,
            });
            ops.push(PredictionOp {
                vector_index: VectorIndex::Second,
                direction,
                reference: ReferenceTarget::WholeField(mb.field_select_lower),
                region: PredictionRegion::Lower16x8,
            });
        }
        // `16x8 MC` never occurs in a frame picture; `Frame-based`
        // never occurs in a field picture (Table 6-17 / 6-18). These
        // arms are unreachable for a well-formed descriptor, but we
        // fall back to a single whole-block op rather than panic so a
        // malformed descriptor degrades predictably.
        _ => {
            let reference = match mb.picture_structure {
                PictureStructure::Frame => ReferenceTarget::Frame,
                _ => ReferenceTarget::WholeField(mb.field_select),
            };
            ops.push(PredictionOp {
                vector_index: VectorIndex::First,
                direction,
                reference,
                region: PredictionRegion::Whole,
            });
        }
    }
}

/// §7.6.5 dual-prime rows (Table 7-13 field picture / Table 7-14 frame
/// picture). Always forward-only.
fn dual_prime_ops(picture_structure: PictureStructure) -> Vec<PredictionOp> {
    match picture_structure {
        // Table 7-13 field picture: two whole-field predictions.
        // vector'[0][0] → same parity; vector'[2][0] (derived) →
        // opposite parity. The actual parities are picture-relative;
        // the field a field-picture decodes is itself one parity and
        // the same/opposite labels are resolved by §7.6.3.6 against
        // the picture's own parity. We carry the labels and leave the
        // concrete parity to the dual_prime derivation; the parity
        // stored here mirrors the §7.6.3.6 numbering convention (top =
        // parity 0).
        PictureStructure::TopField => vec![
            PredictionOp {
                vector_index: VectorIndex::First,
                direction: Direction::Forward,
                reference: ReferenceTarget::DualPrimeSameParity(FieldParity::Top),
                region: PredictionRegion::Whole,
            },
            PredictionOp {
                vector_index: VectorIndex::Second,
                direction: Direction::Forward,
                reference: ReferenceTarget::DualPrimeOppositeParity(FieldParity::Bottom),
                region: PredictionRegion::Whole,
            },
        ],
        PictureStructure::BottomField => vec![
            PredictionOp {
                vector_index: VectorIndex::First,
                direction: Direction::Forward,
                reference: ReferenceTarget::DualPrimeSameParity(FieldParity::Bottom),
                region: PredictionRegion::Whole,
            },
            PredictionOp {
                vector_index: VectorIndex::Second,
                direction: Direction::Forward,
                reference: ReferenceTarget::DualPrimeOppositeParity(FieldParity::Top),
                region: PredictionRegion::Whole,
            },
        ],
        // Table 7-14 frame picture: four whole-block predictions.
        // vector'[0][0] → top field same parity; vector'[0][0] →
        // bottom field same parity; vector'[2][0] (derived) → top
        // field opposite parity; vector'[3][0] (derived) → bottom
        // field opposite parity. Both same-parity ops read the same
        // bitstream vector vector'[0][0]; the two opposite-parity ops
        // read the two derived vectors.
        PictureStructure::Frame => vec![
            PredictionOp {
                vector_index: VectorIndex::First,
                direction: Direction::Forward,
                reference: ReferenceTarget::DualPrimeSameParity(FieldParity::Top),
                region: PredictionRegion::Whole,
            },
            PredictionOp {
                vector_index: VectorIndex::First,
                direction: Direction::Forward,
                reference: ReferenceTarget::DualPrimeSameParity(FieldParity::Bottom),
                region: PredictionRegion::Whole,
            },
            PredictionOp {
                vector_index: VectorIndex::Second,
                direction: Direction::Forward,
                reference: ReferenceTarget::DualPrimeOppositeParity(FieldParity::Top),
                region: PredictionRegion::Whole,
            },
            PredictionOp {
                // vector'[3][0] — the second derived opposite-parity
                // vector; the §7.6.3.6 derivation writes it into the
                // dual-prime working set, which this driver names via
                // the `Second` slot's opposite-parity bottom-field op.
                vector_index: VectorIndex::Second,
                direction: Direction::Forward,
                reference: ReferenceTarget::DualPrimeOppositeParity(FieldParity::Bottom),
                region: PredictionRegion::Whole,
            },
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base() -> MacroblockPrediction {
        MacroblockPrediction {
            picture_structure: PictureStructure::Frame,
            prediction_type: PredictionType::FrameBased,
            macroblock_motion_forward: true,
            macroblock_motion_backward: false,
            macroblock_intra: false,
            concealment_present: false,
            field_select: FieldParity::Top,
            field_select_lower: FieldParity::Top,
        }
    }

    // ---- region geometry ----

    #[test]
    fn region_whole_is_16x16() {
        let s = PredictionRegion::Whole.luma_block_size();
        assert_eq!((s.width, s.height), (16, 16));
        assert_eq!(PredictionRegion::Whole.luma_top_offset(), 0);
    }

    #[test]
    fn region_16x8_halves() {
        let u = PredictionRegion::Upper16x8.luma_block_size();
        let l = PredictionRegion::Lower16x8.luma_block_size();
        assert_eq!((u.width, u.height), (16, 8));
        assert_eq!((l.width, l.height), (16, 8));
        assert_eq!(PredictionRegion::Upper16x8.luma_top_offset(), 0);
        assert_eq!(PredictionRegion::Lower16x8.luma_top_offset(), 8);
    }

    // ---- Table 7-14 frame pictures ----

    #[test]
    fn frame_framebased_forward_only() {
        let ops = select_predictions(&base()).unwrap();
        assert_eq!(ops.len(), 1);
        assert_eq!(ops[0].vector_index, VectorIndex::First);
        assert_eq!(ops[0].direction, Direction::Forward);
        assert_eq!(ops[0].reference, ReferenceTarget::Frame);
        assert_eq!(ops[0].region, PredictionRegion::Whole);
    }

    #[test]
    fn frame_framebased_bidirectional_order() {
        let mb = MacroblockPrediction {
            macroblock_motion_backward: true,
            ..base()
        };
        let ops = select_predictions(&mb).unwrap();
        assert_eq!(ops.len(), 2);
        // Forward precedes backward (bitstream order).
        assert_eq!(ops[0].direction, Direction::Forward);
        assert_eq!(ops[1].direction, Direction::Backward);
        assert!(ops.iter().all(|o| o.reference == ReferenceTarget::Frame));
        assert!(ops.iter().all(|o| o.region == PredictionRegion::Whole));
    }

    #[test]
    fn frame_framebased_backward_only() {
        let mb = MacroblockPrediction {
            macroblock_motion_forward: false,
            macroblock_motion_backward: true,
            ..base()
        };
        let ops = select_predictions(&mb).unwrap();
        assert_eq!(ops.len(), 1);
        assert_eq!(ops[0].direction, Direction::Backward);
        assert_eq!(ops[0].vector_index, VectorIndex::First);
    }

    #[test]
    fn frame_fieldbased_forward_top_then_bottom() {
        let mb = MacroblockPrediction {
            prediction_type: PredictionType::FieldBased,
            ..base()
        };
        let ops = select_predictions(&mb).unwrap();
        assert_eq!(ops.len(), 2);
        assert_eq!(ops[0].vector_index, VectorIndex::First);
        assert_eq!(
            ops[0].reference,
            ReferenceTarget::WholeField(FieldParity::Top)
        );
        assert_eq!(ops[1].vector_index, VectorIndex::Second);
        assert_eq!(
            ops[1].reference,
            ReferenceTarget::WholeField(FieldParity::Bottom)
        );
    }

    #[test]
    fn frame_fieldbased_bidirectional_four_ops() {
        let mb = MacroblockPrediction {
            prediction_type: PredictionType::FieldBased,
            macroblock_motion_backward: true,
            ..base()
        };
        let ops = select_predictions(&mb).unwrap();
        assert_eq!(ops.len(), 4);
        // forward top, forward bottom, backward top, backward bottom.
        assert_eq!(ops[0].direction, Direction::Forward);
        assert_eq!(ops[1].direction, Direction::Forward);
        assert_eq!(ops[2].direction, Direction::Backward);
        assert_eq!(ops[3].direction, Direction::Backward);
        assert_eq!(
            ops[2].reference,
            ReferenceTarget::WholeField(FieldParity::Top)
        );
        assert_eq!(
            ops[3].reference,
            ReferenceTarget::WholeField(FieldParity::Bottom)
        );
    }

    // ---- Table 7-13 field pictures ----

    #[test]
    fn field_fieldbased_uses_field_select() {
        let mb = MacroblockPrediction {
            picture_structure: PictureStructure::TopField,
            prediction_type: PredictionType::FieldBased,
            field_select: FieldParity::Bottom,
            ..base()
        };
        let ops = select_predictions(&mb).unwrap();
        assert_eq!(ops.len(), 1);
        assert_eq!(
            ops[0].reference,
            ReferenceTarget::WholeField(FieldParity::Bottom)
        );
        assert_eq!(ops[0].region, PredictionRegion::Whole);
    }

    #[test]
    fn field_16x8_upper_then_lower_with_independent_selects() {
        let mb = MacroblockPrediction {
            picture_structure: PictureStructure::BottomField,
            prediction_type: PredictionType::SixteenByEight,
            field_select: FieldParity::Top,
            field_select_lower: FieldParity::Bottom,
            ..base()
        };
        let ops = select_predictions(&mb).unwrap();
        assert_eq!(ops.len(), 2);
        assert_eq!(ops[0].region, PredictionRegion::Upper16x8);
        assert_eq!(
            ops[0].reference,
            ReferenceTarget::WholeField(FieldParity::Top)
        );
        assert_eq!(ops[1].region, PredictionRegion::Lower16x8);
        assert_eq!(
            ops[1].reference,
            ReferenceTarget::WholeField(FieldParity::Bottom)
        );
        assert_eq!(ops[0].vector_index, VectorIndex::First);
        assert_eq!(ops[1].vector_index, VectorIndex::Second);
    }

    #[test]
    fn field_16x8_bidirectional_four_ops_order() {
        let mb = MacroblockPrediction {
            picture_structure: PictureStructure::TopField,
            prediction_type: PredictionType::SixteenByEight,
            macroblock_motion_backward: true,
            ..base()
        };
        let ops = select_predictions(&mb).unwrap();
        assert_eq!(ops.len(), 4);
        assert_eq!(ops[0].direction, Direction::Forward);
        assert_eq!(ops[0].region, PredictionRegion::Upper16x8);
        assert_eq!(ops[1].direction, Direction::Forward);
        assert_eq!(ops[1].region, PredictionRegion::Lower16x8);
        assert_eq!(ops[2].direction, Direction::Backward);
        assert_eq!(ops[2].region, PredictionRegion::Upper16x8);
        assert_eq!(ops[3].direction, Direction::Backward);
        assert_eq!(ops[3].region, PredictionRegion::Lower16x8);
    }

    // ---- dual prime ----

    #[test]
    fn dual_prime_field_two_ops() {
        let mb = MacroblockPrediction {
            picture_structure: PictureStructure::TopField,
            prediction_type: PredictionType::DualPrime,
            ..base()
        };
        let ops = select_predictions(&mb).unwrap();
        assert_eq!(ops.len(), 2);
        assert_eq!(
            ops[0].reference,
            ReferenceTarget::DualPrimeSameParity(FieldParity::Top)
        );
        assert_eq!(
            ops[1].reference,
            ReferenceTarget::DualPrimeOppositeParity(FieldParity::Bottom)
        );
        assert!(ops.iter().all(|o| o.direction == Direction::Forward));
    }

    #[test]
    fn dual_prime_bottom_field_parities_swap() {
        let mb = MacroblockPrediction {
            picture_structure: PictureStructure::BottomField,
            prediction_type: PredictionType::DualPrime,
            ..base()
        };
        let ops = select_predictions(&mb).unwrap();
        assert_eq!(
            ops[0].reference,
            ReferenceTarget::DualPrimeSameParity(FieldParity::Bottom)
        );
        assert_eq!(
            ops[1].reference,
            ReferenceTarget::DualPrimeOppositeParity(FieldParity::Top)
        );
    }

    #[test]
    fn dual_prime_frame_four_ops() {
        let mb = MacroblockPrediction {
            picture_structure: PictureStructure::Frame,
            prediction_type: PredictionType::DualPrime,
            ..base()
        };
        let ops = select_predictions(&mb).unwrap();
        assert_eq!(ops.len(), 4);
        // same top, same bottom, opposite top, opposite bottom.
        assert_eq!(
            ops[0].reference,
            ReferenceTarget::DualPrimeSameParity(FieldParity::Top)
        );
        assert_eq!(
            ops[1].reference,
            ReferenceTarget::DualPrimeSameParity(FieldParity::Bottom)
        );
        assert_eq!(
            ops[2].reference,
            ReferenceTarget::DualPrimeOppositeParity(FieldParity::Top)
        );
        assert_eq!(
            ops[3].reference,
            ReferenceTarget::DualPrimeOppositeParity(FieldParity::Bottom)
        );
        // The two same-parity ops both read vector'[0][0].
        assert_eq!(ops[0].vector_index, VectorIndex::First);
        assert_eq!(ops[1].vector_index, VectorIndex::First);
        assert!(ops.iter().all(|o| o.direction == Direction::Forward));
    }

    #[test]
    fn dual_prime_with_backward_is_rejected() {
        let mb = MacroblockPrediction {
            picture_structure: PictureStructure::Frame,
            prediction_type: PredictionType::DualPrime,
            macroblock_motion_backward: true,
            ..base()
        };
        assert_eq!(
            select_predictions(&mb),
            Err(Error::InvalidBitstream(ERR_DUAL_PRIME_NOT_FORWARD_ONLY))
        );
    }

    // ---- intra / concealment ----

    #[test]
    fn intra_with_concealment_frame_single_forward() {
        let mb = MacroblockPrediction {
            macroblock_intra: true,
            concealment_present: true,
            macroblock_motion_forward: false,
            ..base()
        };
        let ops = select_predictions(&mb).unwrap();
        assert_eq!(ops.len(), 1);
        assert_eq!(ops[0].direction, Direction::Forward);
        assert_eq!(ops[0].vector_index, VectorIndex::First);
        assert_eq!(ops[0].reference, ReferenceTarget::Frame);
    }

    #[test]
    fn intra_with_concealment_field_uses_field_select() {
        let mb = MacroblockPrediction {
            picture_structure: PictureStructure::BottomField,
            macroblock_intra: true,
            concealment_present: true,
            macroblock_motion_forward: false,
            field_select: FieldParity::Bottom,
            ..base()
        };
        let ops = select_predictions(&mb).unwrap();
        assert_eq!(ops.len(), 1);
        assert_eq!(
            ops[0].reference,
            ReferenceTarget::WholeField(FieldParity::Bottom)
        );
    }

    #[test]
    fn intra_without_concealment_rejected() {
        let mb = MacroblockPrediction {
            macroblock_intra: true,
            concealment_present: false,
            ..base()
        };
        assert_eq!(
            select_predictions(&mb),
            Err(Error::InvalidBitstream(ERR_INTRA_WITHOUT_CONCEALMENT))
        );
    }

    // ---- skipped / no-motion ----

    #[test]
    fn no_motion_flags_rejected() {
        let mb = MacroblockPrediction {
            macroblock_motion_forward: false,
            macroblock_motion_backward: false,
            ..base()
        };
        assert_eq!(
            select_predictions(&mb),
            Err(Error::InvalidBitstream(ERR_NO_MOTION_FLAGS))
        );
    }
}
