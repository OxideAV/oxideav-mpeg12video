//! §7.6.3.6 dual-prime additional arithmetic per ISO/IEC 13818-2
//! (Recommendation ITU-T H.262) — derive the opposite-parity motion
//! vector from the encoded same-parity vector and the differential
//! `dmvector[0..1]` that the §6.2.5.2.1 syntax decoded.
//!
//! In dual-prime prediction only one field motion vector
//! `vector'[0][0][1:0]` is decoded by the §7.6.3.1 procedure that
//! [`crate::pmv::reconstruct_motion_vector`] implements. That vector
//! references the field of the same parity as the field being predicted.
//! §7.6.3.6 derives the *opposite*-parity vector (`vector'[r][0][1:0]`,
//! with `r = 2` for a field picture or `r = 2` (top) / `r = 3` (bottom)
//! for a frame picture) from the decoded vector by:
//!
//! 1. Scaling its horizontal and vertical components by the field-spacing
//!    factor `m[parity_ref][parity_pred]` defined in Table 7-12 and
//!    halving (`//2`, the spec's integer-division-with-rounding-
//!    to-the-nearest-integer operator, half-integer values rounded away
//!    from zero — see §4.1).
//! 2. Adjusting the vertical component by the integer offset
//!    `e[parity_ref][parity_pred]` from Table 7-13 to absorb the
//!    vertical shift between the lines of the top and bottom fields.
//! 3. Adding the small differential motion vector `dmvector[0..1]`
//!    decoded inline by the §6.2.5.2.1 syntax. Each `dmvector`
//!    component is one of `{-1, 0, +1}` per the Table B-11 VLC.
//!
//! The two spec formulae (page 87 of the H.262 1995 base text):
//!
//! ```text
//! vector'[r][0][0] = ((vector'[0][0][0] * m[parity_ref][parity_pred]) // 2)
//!                  + dmvector[0]
//! vector'[r][0][1] = ((vector'[0][0][1] * m[parity_ref][parity_pred]) // 2)
//!                  + e[parity_ref][parity_pred]
//!                  + dmvector[1]
//! ```
//!
//! For a field picture only `r = 2` is needed; for a frame picture
//! both `r = 2` (top reference field) and `r = 3` (bottom reference
//! field) are derived from the same `vector'[0][0][1:0]`.
//!
//! The Tables that drive the parity arithmetic:
//!
//! * **Table 7-12** — `m[parity_ref][parity_pred]`: the field distance
//!   between the predicted field and the reference field, in units of
//!   the field period. In a frame picture the two `m` entries actually
//!   used depend on `top_field_first`; in a field picture only the
//!   opposite-parity entry of the matching column is consulted.
//! * **Table 7-13** — `e[parity_ref][parity_pred]`: the vertical
//!   adjustment in line units. Positive when going `top → bottom`
//!   (`parity_ref = 0, parity_pred = 1`), negative when going `bottom
//!   → top` (`parity_ref = 1, parity_pred = 0`), zero on the diagonal.
//!
//! Spec citations refer to the 1994/1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262) §§7.6.3.6 + Tables 7-12 / 7-13 / 7-14;
//! the `//` operator definition is §4.1 page 9.

use crate::picture_header::PictureStructure;
use crate::{Error, Result};

/// One field's parity per §7.6.3.6: top fields have parity `0`, bottom
/// fields have parity `1`. The `parity_ref` of Tables 7-12 / 7-13 is
/// the parity of the reference field for which the opposite-parity
/// vector is being computed; the `parity_pred` is the parity of the
/// field actually being predicted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FieldParity {
    /// Top field (`parity = 0`).
    Top,
    /// Bottom field (`parity = 1`).
    Bottom,
}

impl FieldParity {
    /// Numeric parity index (`0` for [`FieldParity::Top`], `1` for
    /// [`FieldParity::Bottom`]).
    pub fn index(self) -> usize {
        match self {
            FieldParity::Top => 0,
            FieldParity::Bottom => 1,
        }
    }

    /// The opposite parity. Top swaps with bottom.
    pub fn opposite(self) -> FieldParity {
        match self {
            FieldParity::Top => FieldParity::Bottom,
            FieldParity::Bottom => FieldParity::Top,
        }
    }
}

/// Per the `picture_structure` column of Table 7-14, dual-prime is
/// permitted only in P-pictures (the same-parity vector is forward).
/// Each dual-prime derivation site supplies the picture-level
/// `(picture_structure, top_field_first)` pair and a field-picture
/// site additionally supplies which field is being decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DualPrimePicture {
    /// A frame picture (`picture_structure == 11`). Both reference
    /// parities derive a vector per macroblock: `r = 2` is the
    /// top-reference vector used by the top field of the predicted
    /// frame, `r = 3` is the bottom-reference vector used by the
    /// bottom field. `top_field_first` is the `picture_coding_extension`
    /// flag that selects between the Table 7-12 frame rows.
    Frame {
        /// `top_field_first` flag (§6.3.11). When `1` the top field is
        /// predicted first and consumes Table 7-12 row "Frame, tff=1";
        /// when `0` the bottom field is predicted first and consumes the
        /// "Frame, tff=0" row.
        top_field_first: bool,
    },
    /// A field picture (`picture_structure == 01` top, `10` bottom).
    /// Only `r = 2` is needed: the opposite-parity vector for the field
    /// being predicted (whose parity follows directly from the
    /// `picture_structure` field).
    Field {
        /// Parity of the field actually being decoded (a `TopField`
        /// picture has parity [`FieldParity::Top`], a `BottomField`
        /// picture has parity [`FieldParity::Bottom`]).
        predicted_parity: FieldParity,
    },
}

/// Convenience constructor that lowers the parser-level
/// [`PictureStructure`] + `top_field_first` pair into a
/// [`DualPrimePicture`]. Errors on `Frame` shaped values where the
/// predicted parity wouldn't make sense (none currently — every Frame
/// is valid, every field picture infers its parity from the structure).
pub fn dual_prime_picture(
    picture_structure: PictureStructure,
    top_field_first: bool,
) -> DualPrimePicture {
    match picture_structure {
        PictureStructure::Frame => DualPrimePicture::Frame { top_field_first },
        PictureStructure::TopField => DualPrimePicture::Field {
            predicted_parity: FieldParity::Top,
        },
        PictureStructure::BottomField => DualPrimePicture::Field {
            predicted_parity: FieldParity::Bottom,
        },
    }
}

/// One derived opposite-parity vector from §7.6.3.6, in half-sample
/// units. The `vector_index` is the spec's `r` index (`2` or `3` for
/// the two derived dual-prime slots).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DerivedDualPrimeVector {
    /// Reference-field parity (`parity_ref` of Tables 7-12 / 7-13) —
    /// the parity of the reference field this derived vector points
    /// into.
    pub parity_ref: FieldParity,
    /// Predicted-field parity (`parity_pred` of Tables 7-12 / 7-13) —
    /// the parity of the field whose samples are being predicted with
    /// this vector.
    pub parity_pred: FieldParity,
    /// Spec `r` index for the derived vector. `2` for the opposite-
    /// parity derivation in a field picture or the top-reference
    /// derivation in a frame picture; `3` for the bottom-reference
    /// derivation in a frame picture.
    pub vector_index: u8,
    /// Horizontal component `vector'[r][0][0]`.
    pub horiz: i32,
    /// Vertical component `vector'[r][0][1]`.
    pub vert: i32,
}

/// Table 7-12 — `m[parity_ref][parity_pred]` field-distance factor.
///
/// The spec lays the table out by `picture_structure` row:
///
/// | picture_structure  | top_field_first | m[1][0] | m[0][1] |
/// |--------------------|-----------------|---------|---------|
/// | 11 (Frame)         | 1               | 1       | 3       |
/// | 11 (Frame)         | 0               | 3       | 1       |
/// | 01 (Top Field)     | -               | 1       | -       |
/// | 10 (Bottom Field)  | -               | -       | 1       |
///
/// Entries marked `-` are not consumed by §7.6.3.6 for that picture
/// type (the diagonal `m[0][0]` and `m[1][1]` are never consulted by
/// the dual-prime path — the same-parity vector is the *input*, not a
/// derived output).
///
/// This function returns the table value for the requested
/// `(parity_ref, parity_pred)` pair under the supplied
/// [`DualPrimePicture`] context. It errors if the requested pair is
/// not listed by the table for the active picture type.
pub fn m_factor(
    picture: DualPrimePicture,
    parity_ref: FieldParity,
    parity_pred: FieldParity,
) -> Result<i32> {
    match (picture, parity_ref, parity_pred) {
        // Frame picture, top_field_first = 1: m[1][0] = 1, m[0][1] = 3.
        (
            DualPrimePicture::Frame {
                top_field_first: true,
            },
            FieldParity::Bottom,
            FieldParity::Top,
        ) => Ok(1),
        (
            DualPrimePicture::Frame {
                top_field_first: true,
            },
            FieldParity::Top,
            FieldParity::Bottom,
        ) => Ok(3),
        // Frame picture, top_field_first = 0: m[1][0] = 3, m[0][1] = 1.
        (
            DualPrimePicture::Frame {
                top_field_first: false,
            },
            FieldParity::Bottom,
            FieldParity::Top,
        ) => Ok(3),
        (
            DualPrimePicture::Frame {
                top_field_first: false,
            },
            FieldParity::Top,
            FieldParity::Bottom,
        ) => Ok(1),
        // Top field picture: only m[1][0] = 1 is defined (predicting
        // a top field from a bottom reference). Any other entry for a
        // top field picture is not on Table 7-12.
        (
            DualPrimePicture::Field {
                predicted_parity: FieldParity::Top,
            },
            FieldParity::Bottom,
            FieldParity::Top,
        ) => Ok(1),
        // Bottom field picture: only m[0][1] = 1 is defined.
        (
            DualPrimePicture::Field {
                predicted_parity: FieldParity::Bottom,
            },
            FieldParity::Top,
            FieldParity::Bottom,
        ) => Ok(1),
        // Everything else is not on Table 7-12 for the active picture
        // type — the dual-prime decoder must not consult it.
        _ => Err(Error::InvalidBitstream(
            "dual_prime::m_factor: (parity_ref, parity_pred) not listed by Table 7-12 for this picture_structure",
        )),
    }
}

/// Table 7-13 — `e[parity_ref][parity_pred]` vertical adjustment.
///
/// | parity_ref | parity_pred | e |
/// |------------|-------------|---|
/// | 0          | 0           | 0 |
/// | 0          | 1           | +1|
/// | 1          | 0           | -1|
/// | 1          | 1           | 0 |
///
/// This lookup is unconditional — Table 7-13 is independent of
/// picture structure / top_field_first. The diagonal entries are
/// `0` and are never consumed by §7.6.3.6 (the same-parity vector is
/// the input, not derived), but they are listed for completeness.
pub fn e_offset(parity_ref: FieldParity, parity_pred: FieldParity) -> i32 {
    match (parity_ref, parity_pred) {
        (FieldParity::Top, FieldParity::Top) => 0,
        (FieldParity::Top, FieldParity::Bottom) => 1,
        (FieldParity::Bottom, FieldParity::Top) => -1,
        (FieldParity::Bottom, FieldParity::Bottom) => 0,
    }
}

/// The `//` operator from ISO/IEC 13818-2 §4.1 — "integer division
/// with rounding to the nearest integer; half-integer values are
/// rounded away from zero unless otherwise specified".
///
/// Examples from the spec: `3//2 = 2`, `-3//2 = -2`. For an even
/// dividend the result is exact (`4//2 = 2`, `-4//2 = -2`).
///
/// This is *not* `i32::div_euclid` (which is `DIV`, the spec's
/// integer division toward minus infinity) and *not* `i32::div`
/// (which is `/`, the spec's integer division toward zero). The
/// dual-prime arithmetic uses `//` exclusively for the `m`-scaled
/// halving.
fn div_round_away_from_zero(dividend: i32, divisor: i32) -> i32 {
    // Per the spec example `-3//2 = -2`, half-integer values round
    // away from zero. For non-half-integer values the result is the
    // nearest integer (which equals the truncated quotient ± rounding
    // adjustment). Rather than encode the rule on the magnitude and
    // sign separately, compute it as `(dividend + sign(dividend) *
    // half_divisor) / divisor` using truncation toward zero (the `/`
    // operator).
    debug_assert!(
        divisor > 0,
        "div_round_away_from_zero: divisor must be positive (every spec call has divisor=2)"
    );
    let half = divisor / 2;
    if dividend >= 0 {
        (dividend + half) / divisor
    } else {
        (dividend - half) / divisor
    }
}

/// §7.6.3.6 — derive one opposite-parity motion vector from the
/// decoded same-parity vector and the inline `dmvector[0..1]`.
///
/// `(decoded_horiz, decoded_vert)` is `vector'[0][0][1:0]` — the result
/// of running [`crate::pmv::reconstruct_motion_vector`] on the
/// macroblock's forward motion vector. `(dmvector_horiz, dmvector_vert)`
/// is the inline differential motion vector decoded by the §6.2.5.2.1
/// syntax (Table B-11 VLC), each component in `{-1, 0, +1}`.
///
/// `vector_index` is the spec's `r` (set to `2` for the single-derived
/// field-picture case or to `2`/`3` for the two frame-picture cases);
/// it is propagated through to [`DerivedDualPrimeVector::vector_index`]
/// for caller-side bookkeeping but does not affect the arithmetic.
///
/// Errors:
/// * [`Error::InvalidBitstream`] if either `dmvector` component lies
///   outside `{-1, 0, +1}`.
/// * Whatever [`m_factor`] returns when `(parity_ref, parity_pred)`
///   is not on Table 7-12 for the active picture type.
// §7.6.3.6's two formulae jointly consume the picture context, both
// parity indices, the decoded-vector pair, and the dmvector pair —
// every argument is required by the spec, so bundling them into a
// container struct would just move the parameter list one level
// deeper without changing what the caller has to pass in.
#[allow(clippy::too_many_arguments)]
pub fn derive_opposite_parity_vector(
    picture: DualPrimePicture,
    parity_ref: FieldParity,
    parity_pred: FieldParity,
    vector_index: u8,
    decoded_horiz: i32,
    decoded_vert: i32,
    dmvector_horiz: i32,
    dmvector_vert: i32,
) -> Result<DerivedDualPrimeVector> {
    // §6.2.5.2.1 / Table B-11: dmvector components are constrained
    // to {-1, 0, +1}. The §7.6.3.6 description (page 86, line 10)
    // restates this constraint explicitly.
    if !(-1..=1).contains(&dmvector_horiz) || !(-1..=1).contains(&dmvector_vert) {
        return Err(Error::InvalidBitstream(
            "derive_opposite_parity_vector: dmvector component outside {-1, 0, +1}",
        ));
    }

    let m = m_factor(picture, parity_ref, parity_pred)?;
    let e = e_offset(parity_ref, parity_pred);

    let scaled_horiz = div_round_away_from_zero(decoded_horiz * m, 2);
    let scaled_vert = div_round_away_from_zero(decoded_vert * m, 2);

    Ok(DerivedDualPrimeVector {
        parity_ref,
        parity_pred,
        vector_index,
        horiz: scaled_horiz + dmvector_horiz,
        vert: scaled_vert + e + dmvector_vert,
    })
}

/// §7.6.3.6 driver — derive all opposite-parity dual-prime vectors
/// required for the macroblock's surrounding picture.
///
/// Frame picture: two vectors are derived from the single decoded
/// vector — `r = 2` references the top reference field for the
/// top-field prediction, `r = 3` references the bottom reference
/// field for the bottom-field prediction.
///
/// Field picture: one vector is derived from the single decoded
/// vector — `r = 2` references the opposite-parity reference field
/// for the field being predicted.
///
/// In every case the same `(decoded_horiz, decoded_vert,
/// dmvector_horiz, dmvector_vert)` tuple is the input; the parity
/// arguments select which table cells fire.
///
/// Returns the derived vectors in spec `r`-index order (so frame
/// pictures yield `[r=2, r=3]`, field pictures yield `[r=2]`).
pub fn derive_all(
    picture: DualPrimePicture,
    decoded_horiz: i32,
    decoded_vert: i32,
    dmvector_horiz: i32,
    dmvector_vert: i32,
) -> Result<Vec<DerivedDualPrimeVector>> {
    match picture {
        DualPrimePicture::Frame { .. } => {
            // r = 2 references the top reference field, predicting the
            // top field of the frame (parity_pred = Top, parity_ref =
            // Bottom → Top means parity_ref = Top? No: per the figure
            // caption "the existing motion vector is scaled to reflect
            // the different temporal distance between the fields". The
            // decoded vector is the same-parity vector for one field;
            // the *opposite*-parity is the cross-field one. For a
            // frame picture both pairings are needed:
            //   - predict top field from bottom reference → parity_pred
            //     = Top, parity_ref = Bottom (r = 2 per the spec note
            //     at §7.6.3.6 page 87 lines 13-14: "the top field shall
            //     use vector'[2][0][1:0] for opposite parity").
            //   - predict bottom field from top reference → parity_pred
            //     = Bottom, parity_ref = Top (r = 3 per "the bottom
            //     field shall use vector'[3][0][1:0]").
            let top_pred = derive_opposite_parity_vector(
                picture,
                FieldParity::Bottom,
                FieldParity::Top,
                2,
                decoded_horiz,
                decoded_vert,
                dmvector_horiz,
                dmvector_vert,
            )?;
            let bottom_pred = derive_opposite_parity_vector(
                picture,
                FieldParity::Top,
                FieldParity::Bottom,
                3,
                decoded_horiz,
                decoded_vert,
                dmvector_horiz,
                dmvector_vert,
            )?;
            Ok(vec![top_pred, bottom_pred])
        }
        DualPrimePicture::Field { predicted_parity } => {
            // Per the §7.6.3.6 closing paragraph (page 87 lines 9-11):
            // "In the case of field pictures only one such motion
            // vector is required and here r=2." The opposite-parity
            // reference is the opposite of the field's own parity.
            let parity_ref = predicted_parity.opposite();
            let derived = derive_opposite_parity_vector(
                picture,
                parity_ref,
                predicted_parity,
                2,
                decoded_horiz,
                decoded_vert,
                dmvector_horiz,
                dmvector_vert,
            )?;
            Ok(vec![derived])
        }
    }
}

#[cfg(test)]
mod tests {
    //! Hand-traced §7.6.3.6 round-trips. Every numeric expectation is
    //! computed by walking the spec formulae on paper and is annotated
    //! with the arithmetic that produced it.
    use super::*;

    // ---- §4.1 `//` operator ----

    #[test]
    fn div_round_away_from_zero_matches_spec_examples() {
        // Spec example (§4.1 page 9): 3//2 = 2, -3//2 = -2.
        assert_eq!(div_round_away_from_zero(3, 2), 2);
        assert_eq!(div_round_away_from_zero(-3, 2), -2);
        // Exact-divisible cases: 4//2 = 2, -4//2 = -2, 0//2 = 0.
        assert_eq!(div_round_away_from_zero(4, 2), 2);
        assert_eq!(div_round_away_from_zero(-4, 2), -2);
        assert_eq!(div_round_away_from_zero(0, 2), 0);
        // 1//2 = 1 (half rounds away from zero toward +1), -1//2 = -1.
        assert_eq!(div_round_away_from_zero(1, 2), 1);
        assert_eq!(div_round_away_from_zero(-1, 2), -1);
        // 5//2 = 3 (2.5 rounds away from zero to 3), -5//2 = -3.
        assert_eq!(div_round_away_from_zero(5, 2), 3);
        assert_eq!(div_round_away_from_zero(-5, 2), -3);
    }

    // ---- Table 7-12 m_factor ----

    #[test]
    fn m_factor_frame_tff_one() {
        let pic = DualPrimePicture::Frame {
            top_field_first: true,
        };
        // m[1][0] = 1, m[0][1] = 3.
        assert_eq!(
            m_factor(pic, FieldParity::Bottom, FieldParity::Top).unwrap(),
            1
        );
        assert_eq!(
            m_factor(pic, FieldParity::Top, FieldParity::Bottom).unwrap(),
            3
        );
    }

    #[test]
    fn m_factor_frame_tff_zero() {
        let pic = DualPrimePicture::Frame {
            top_field_first: false,
        };
        // m[1][0] = 3, m[0][1] = 1.
        assert_eq!(
            m_factor(pic, FieldParity::Bottom, FieldParity::Top).unwrap(),
            3
        );
        assert_eq!(
            m_factor(pic, FieldParity::Top, FieldParity::Bottom).unwrap(),
            1
        );
    }

    #[test]
    fn m_factor_top_field_picture_only_one_entry() {
        let pic = DualPrimePicture::Field {
            predicted_parity: FieldParity::Top,
        };
        // m[1][0] = 1 is the only listed entry for the top field row.
        assert_eq!(
            m_factor(pic, FieldParity::Bottom, FieldParity::Top).unwrap(),
            1
        );
        // m[0][1] is not on the top-field row; consulting it is an
        // error.
        let err = m_factor(pic, FieldParity::Top, FieldParity::Bottom).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn m_factor_bottom_field_picture_only_one_entry() {
        let pic = DualPrimePicture::Field {
            predicted_parity: FieldParity::Bottom,
        };
        assert_eq!(
            m_factor(pic, FieldParity::Top, FieldParity::Bottom).unwrap(),
            1
        );
        let err = m_factor(pic, FieldParity::Bottom, FieldParity::Top).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn m_factor_diagonal_not_on_table() {
        // m[0][0] and m[1][1] are never on the table for any picture
        // type (the diagonal is the same-parity case, which is the
        // *input* to §7.6.3.6, not an output).
        let frame = DualPrimePicture::Frame {
            top_field_first: true,
        };
        assert!(matches!(
            m_factor(frame, FieldParity::Top, FieldParity::Top),
            Err(Error::InvalidBitstream(_))
        ));
        assert!(matches!(
            m_factor(frame, FieldParity::Bottom, FieldParity::Bottom),
            Err(Error::InvalidBitstream(_))
        ));
    }

    // ---- Table 7-13 e_offset ----

    #[test]
    fn e_offset_table_7_13_all_four_entries() {
        assert_eq!(e_offset(FieldParity::Top, FieldParity::Top), 0);
        assert_eq!(e_offset(FieldParity::Top, FieldParity::Bottom), 1);
        assert_eq!(e_offset(FieldParity::Bottom, FieldParity::Top), -1);
        assert_eq!(e_offset(FieldParity::Bottom, FieldParity::Bottom), 0);
    }

    // ---- §7.6.3.6 derive_opposite_parity_vector ----

    #[test]
    fn derive_field_top_zero_decoded_zero_dmv() {
        // Field picture predicting the top field. parity_pred = Top,
        // parity_ref = Bottom. m = 1, e = -1.
        // decoded = (0, 0), dmv = (0, 0).
        // vector'[2][0][0] = (0 * 1) // 2 + 0 = 0
        // vector'[2][0][1] = (0 * 1) // 2 + (-1) + 0 = -1
        let pic = DualPrimePicture::Field {
            predicted_parity: FieldParity::Top,
        };
        let out = derive_opposite_parity_vector(
            pic,
            FieldParity::Bottom,
            FieldParity::Top,
            2,
            0,
            0,
            0,
            0,
        )
        .unwrap();
        assert_eq!(out.horiz, 0);
        assert_eq!(out.vert, -1);
        assert_eq!(out.parity_ref, FieldParity::Bottom);
        assert_eq!(out.parity_pred, FieldParity::Top);
        assert_eq!(out.vector_index, 2);
    }

    #[test]
    fn derive_field_bottom_unit_decoded_zero_dmv() {
        // Field picture predicting the bottom field. parity_pred =
        // Bottom, parity_ref = Top. m = 1, e = +1.
        // decoded = (2, 2), dmv = (0, 0).
        // vector'[2][0][0] = (2 * 1) // 2 + 0 = 1
        // vector'[2][0][1] = (2 * 1) // 2 + 1 + 0 = 2
        let pic = DualPrimePicture::Field {
            predicted_parity: FieldParity::Bottom,
        };
        let out = derive_opposite_parity_vector(
            pic,
            FieldParity::Top,
            FieldParity::Bottom,
            2,
            2,
            2,
            0,
            0,
        )
        .unwrap();
        assert_eq!(out.horiz, 1);
        assert_eq!(out.vert, 2);
    }

    #[test]
    fn derive_frame_tff_one_top_prediction_path() {
        // Frame picture, top_field_first = 1. r = 2 path: parity_pred
        // = Top, parity_ref = Bottom, m = 1, e = -1.
        // decoded = (4, 6), dmv = (1, -1).
        // horiz = (4 * 1) // 2 + 1 = 2 + 1 = 3
        // vert  = (6 * 1) // 2 + (-1) + (-1) = 3 - 2 = 1
        let pic = DualPrimePicture::Frame {
            top_field_first: true,
        };
        let out = derive_opposite_parity_vector(
            pic,
            FieldParity::Bottom,
            FieldParity::Top,
            2,
            4,
            6,
            1,
            -1,
        )
        .unwrap();
        assert_eq!(out.horiz, 3);
        assert_eq!(out.vert, 1);
    }

    #[test]
    fn derive_frame_tff_one_bottom_prediction_path() {
        // Frame picture, top_field_first = 1. r = 3 path: parity_pred
        // = Bottom, parity_ref = Top, m = 3, e = +1.
        // decoded = (4, 6), dmv = (0, 0).
        // horiz = (4 * 3) // 2 + 0 = 12 // 2 = 6
        // vert  = (6 * 3) // 2 + 1 + 0 = 18 // 2 + 1 = 10
        let pic = DualPrimePicture::Frame {
            top_field_first: true,
        };
        let out = derive_opposite_parity_vector(
            pic,
            FieldParity::Top,
            FieldParity::Bottom,
            3,
            4,
            6,
            0,
            0,
        )
        .unwrap();
        assert_eq!(out.horiz, 6);
        assert_eq!(out.vert, 10);
    }

    #[test]
    fn derive_frame_tff_zero_swaps_m() {
        // top_field_first = 0 swaps the two m entries: r = 2 path now
        // uses m = 3 (instead of 1), r = 3 path now uses m = 1 (instead
        // of 3).
        let pic = DualPrimePicture::Frame {
            top_field_first: false,
        };
        // r = 2: m = 3, e = -1. decoded = (2, 2), dmv = (0, 0).
        // horiz = (2 * 3) // 2 + 0 = 6 // 2 = 3
        // vert  = (2 * 3) // 2 + (-1) + 0 = 3 - 1 = 2
        let out2 = derive_opposite_parity_vector(
            pic,
            FieldParity::Bottom,
            FieldParity::Top,
            2,
            2,
            2,
            0,
            0,
        )
        .unwrap();
        assert_eq!(out2.horiz, 3);
        assert_eq!(out2.vert, 2);

        // r = 3: m = 1, e = +1. decoded = (2, 2), dmv = (0, 0).
        // horiz = (2 * 1) // 2 + 0 = 1
        // vert  = (2 * 1) // 2 + 1 + 0 = 2
        let out3 = derive_opposite_parity_vector(
            pic,
            FieldParity::Top,
            FieldParity::Bottom,
            3,
            2,
            2,
            0,
            0,
        )
        .unwrap();
        assert_eq!(out3.horiz, 1);
        assert_eq!(out3.vert, 2);
    }

    #[test]
    fn derive_half_integer_rounds_away_from_zero() {
        // decoded_horiz = 3 with m = 1: (3 * 1) // 2 = 3//2 = 2 per
        // spec (half-integer rounds away from zero, +2 from +1.5).
        // decoded_horiz = -3 with m = 1: (-3 * 1) // 2 = -3//2 = -2.
        let pic = DualPrimePicture::Field {
            predicted_parity: FieldParity::Top,
        };
        let pos = derive_opposite_parity_vector(
            pic,
            FieldParity::Bottom,
            FieldParity::Top,
            2,
            3,
            0,
            0,
            0,
        )
        .unwrap();
        assert_eq!(pos.horiz, 2);

        let neg = derive_opposite_parity_vector(
            pic,
            FieldParity::Bottom,
            FieldParity::Top,
            2,
            -3,
            0,
            0,
            0,
        )
        .unwrap();
        assert_eq!(neg.horiz, -2);
    }

    #[test]
    fn derive_rejects_out_of_range_dmvector() {
        // dmvector components are constrained to {-1, 0, +1}; any other
        // value comes from a buggy upstream parser.
        let pic = DualPrimePicture::Field {
            predicted_parity: FieldParity::Top,
        };
        for bad in [-2, 2, 3, -5] {
            let err = derive_opposite_parity_vector(
                pic,
                FieldParity::Bottom,
                FieldParity::Top,
                2,
                0,
                0,
                bad,
                0,
            )
            .unwrap_err();
            assert!(
                matches!(err, Error::InvalidBitstream(_)),
                "dmv_horiz={bad}: expected InvalidBitstream"
            );
            let err = derive_opposite_parity_vector(
                pic,
                FieldParity::Bottom,
                FieldParity::Top,
                2,
                0,
                0,
                0,
                bad,
            )
            .unwrap_err();
            assert!(
                matches!(err, Error::InvalidBitstream(_)),
                "dmv_vert={bad}: expected InvalidBitstream"
            );
        }
    }

    // ---- derive_all driver ----

    #[test]
    fn derive_all_field_picture_yields_one_vector() {
        let pic = DualPrimePicture::Field {
            predicted_parity: FieldParity::Top,
        };
        let out = derive_all(pic, 0, 0, 0, 0).unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].vector_index, 2);
        assert_eq!(out[0].parity_pred, FieldParity::Top);
        assert_eq!(out[0].parity_ref, FieldParity::Bottom);
    }

    #[test]
    fn derive_all_frame_picture_yields_two_vectors() {
        let pic = DualPrimePicture::Frame {
            top_field_first: true,
        };
        let out = derive_all(pic, 0, 0, 0, 0).unwrap();
        assert_eq!(out.len(), 2);
        // r = 2 first (top-field prediction).
        assert_eq!(out[0].vector_index, 2);
        assert_eq!(out[0].parity_pred, FieldParity::Top);
        assert_eq!(out[0].parity_ref, FieldParity::Bottom);
        // r = 3 second (bottom-field prediction).
        assert_eq!(out[1].vector_index, 3);
        assert_eq!(out[1].parity_pred, FieldParity::Bottom);
        assert_eq!(out[1].parity_ref, FieldParity::Top);
    }

    #[test]
    fn derive_all_frame_picture_carries_both_e_offsets() {
        // Frame picture, top_field_first = 1, decoded = (0, 0), dmv =
        // (0, 0). The two derivations differ only in the `e` offset:
        // r = 2 uses e[1][0] = -1, r = 3 uses e[0][1] = +1.
        let pic = DualPrimePicture::Frame {
            top_field_first: true,
        };
        let out = derive_all(pic, 0, 0, 0, 0).unwrap();
        assert_eq!(out[0].vert, -1);
        assert_eq!(out[1].vert, 1);
    }

    // ---- dual_prime_picture lowering ----

    #[test]
    fn dual_prime_picture_lowering_matches_picture_structure() {
        let frame = dual_prime_picture(PictureStructure::Frame, true);
        assert_eq!(
            frame,
            DualPrimePicture::Frame {
                top_field_first: true,
            }
        );
        let frame0 = dual_prime_picture(PictureStructure::Frame, false);
        assert_eq!(
            frame0,
            DualPrimePicture::Frame {
                top_field_first: false,
            }
        );
        let top = dual_prime_picture(PictureStructure::TopField, false);
        assert_eq!(
            top,
            DualPrimePicture::Field {
                predicted_parity: FieldParity::Top,
            }
        );
        let bottom = dual_prime_picture(PictureStructure::BottomField, true);
        assert_eq!(
            bottom,
            DualPrimePicture::Field {
                predicted_parity: FieldParity::Bottom,
            }
        );
    }

    // ---- FieldParity helpers ----

    #[test]
    fn field_parity_index_and_opposite() {
        assert_eq!(FieldParity::Top.index(), 0);
        assert_eq!(FieldParity::Bottom.index(), 1);
        assert_eq!(FieldParity::Top.opposite(), FieldParity::Bottom);
        assert_eq!(FieldParity::Bottom.opposite(), FieldParity::Top);
    }
}
