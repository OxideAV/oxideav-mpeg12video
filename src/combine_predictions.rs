//! §7.6.7 Combining predictions per ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262), pages 104–105 of the 1995 base text — the bidirectional
//! averaging step that combines the up-to-four §7.6.4 prediction blocks
//! produced for a single macroblock into the final per-component
//! prediction sample plane.
//!
//! ## What §7.6.7 specifies
//!
//! After §7.6.4 has formed each individual reference read (forward,
//! backward, top-parity, bottom-parity, dual-prime same-parity,
//! dual-prime opposite-parity) into its own `pel_pred[y][x]` block,
//! the decoder combines them. The base text enumerates four cases —
//! §7.6.7.1 (simple frame), §7.6.7.2 (simple field), §7.6.7.3 (16x8
//! MC), §7.6.7.4 (dual prime) — and in all of them the only
//! operation needed for the **bi-directional** case is the same
//! `// 2` average of two prediction samples:
//!
//! ```text
//! pel_pred[y][x] = (pel_pred_forward[y][x] + pel_pred_backward[y][x]) // 2;
//! ```
//!
//! (For dual prime the same formula is written as
//! `(pel_pred_same_parity[y][x] + pel_pred_opposite_parity[y][x]) // 2`
//! per §7.6.7.4; the arithmetic is identical.)
//!
//! The `//` operator is §4.1 integer division with rounding toward the
//! nearest integer (half-integer values rounded **away** from zero —
//! `3//2 = 2`, `-3//2 = -2`). For a sum of two unsigned-8-bit
//! prediction samples the result is in `[0, 510]` and the `// 2`
//! collapses to the canonical `(sum + 1) >> 1` rounded-up form.
//!
//! In the **forward-only** or **backward-only** B-frame cases (Tables
//! 7-13 / 7-14, second-and-third rows) only one of the two predictions
//! exists and the final block is that single prediction unchanged —
//! no averaging step is needed.
//!
//! In the **field-based-but-`(0, 0)` direction-flag** zero-MV case
//! (Tables 7-13 / 7-14 last `Field-based`/`Frame-based` row) the
//! macroblock's prediction is the implicit forward prediction at
//! `(0, 0)` per §7.6.3.5; this is again a single prediction with no
//! combination step.
//!
//! ## What this module provides
//!
//! * [`average_predictions`] — combine two equal-shape `Vec<u8>`
//!   prediction blocks into a third using the `// 2` operator. Both
//!   inputs must be the same length; the output has the same length.
//! * [`average_predictions_in_place`] — the same operation, writing
//!   the result back into the forward buffer.
//! * [`PredictionDirection`] — which subset of `(forward, backward)`
//!   is present for a given block, driven by the
//!   `macroblock_motion_forward` / `macroblock_motion_backward` flags
//!   of §6.3.17.1.
//! * [`combine_directional_predictions`] — convenience driver that
//!   takes the four §7.6.5 cases (forward-only, backward-only,
//!   bidirectional, none) and returns the combined block, mirroring
//!   the Tables 7-13 / 7-14 selection.
//!
//! `combine_directional_predictions` does **not** attempt to enforce
//! the macroblock-type semantic constraints (e.g. "both flags off
//! plus `macroblock_intra == 0` only occurs in the `Field-based` /
//! `Frame-based` implicit-zero-MV case"); the caller is expected to
//! map the parsed `macroblock_type` and `frame_motion_type` /
//! `field_motion_type` to the correct `(forward, backward)` block
//! pair before calling this layer.
//!
//! Spec citations refer to **ISO/IEC 13818-2 (H.262) §7.6.7.1**
//! through **§7.6.7.4** plus the §4.1 arithmetic operators (`//`).

/// Average two `u8` samples with the §4.1 `// 2` operator.
///
/// The sum of two unsigned-8-bit values is in `[0, 510]`, well inside
/// `u16`. The `// 2` operator rounds half-integer values away from
/// zero, which on a non-negative sum is identical to "add half the
/// divisor before truncating", i.e. `(sum + 1) >> 1`.
#[inline]
fn avg2(a: u8, b: u8) -> u8 {
    ((a as u16 + b as u16 + 1) >> 1) as u8
}

/// Combine `forward` and `backward` prediction blocks into a single
/// per-sample average per **§7.6.7.1** page 105:
///
/// ```text
/// pel_pred[y][x] = (pel_pred_forward[y][x] + pel_pred_backward[y][x]) // 2
/// ```
///
/// Both inputs must have the same length (typically `block_width *
/// block_height` from the §7.6.4 [`crate::forming_predictions::predict_block`]
/// call). The output is allocated fresh and has the same length.
///
/// Returns `None` if the two inputs are of different lengths — that
/// would imply a caller bug, since the §7.6.5 prediction-block
/// geometry is identical for both directions.
pub fn average_predictions(forward: &[u8], backward: &[u8]) -> Option<Vec<u8>> {
    if forward.len() != backward.len() {
        return None;
    }
    let mut out = Vec::with_capacity(forward.len());
    for (f, b) in forward.iter().zip(backward.iter()) {
        out.push(avg2(*f, *b));
    }
    Some(out)
}

/// In-place variant of [`average_predictions`] that overwrites the
/// `forward` buffer with the per-sample average of `forward` and
/// `backward` per §7.6.7.1. Returns `false` and leaves the buffer
/// unchanged if the inputs are of different lengths.
pub fn average_predictions_in_place(forward: &mut [u8], backward: &[u8]) -> bool {
    if forward.len() != backward.len() {
        return false;
    }
    for (f, b) in forward.iter_mut().zip(backward.iter()) {
        *f = avg2(*f, *b);
    }
    true
}

/// Which subset of `(forward, backward)` predictions is present for a
/// macroblock, per **§7.6.5 Tables 7-13 / 7-14** and the §6.3.17.1
/// `macroblock_motion_forward` / `macroblock_motion_backward` flags.
///
/// The `Skipped` variant covers the implicit-zero-MV case where
/// neither flag is set on a non-intra macroblock (Tables 7-13 / 7-14
/// last `Field-based`/`Frame-based` row, §7.6.3.5): the prediction is
/// taken as the implicit forward zero-MV block — i.e. a single
/// forward prediction was still formed by the caller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PredictionDirection {
    /// `(forward, backward) = (1, 0)` — single forward prediction.
    /// Tables 7-13 / 7-14 forward-only rows. Output is the forward
    /// block unchanged.
    Forward,
    /// `(forward, backward) = (0, 1)` — single backward prediction.
    /// Tables 7-13 / 7-14 backward-only rows. Output is the backward
    /// block unchanged.
    Backward,
    /// `(forward, backward) = (1, 1)` — both predictions present.
    /// §7.6.7.1 page 105 averaging applies.
    Bidirectional,
    /// `(forward, backward) = (0, 0)` on a non-intra macroblock —
    /// the §7.6.3.5 implicit forward zero-MV prediction. The caller
    /// passes the formed-from-`(0,0)`-vector block as the `forward`
    /// argument; the backward argument is ignored.
    Skipped,
}

/// Combine the up-to-two §7.6.4 prediction blocks of a macroblock
/// into the final per-component prediction sample plane per **§7.6.7.1**.
///
/// Behaviour by [`PredictionDirection`]:
///
/// * [`PredictionDirection::Forward`] — return `forward.to_vec()`.
///   The `backward` argument is ignored (a caller may pass any
///   buffer, e.g. `&[]`).
/// * [`PredictionDirection::Backward`] — return `backward.to_vec()`.
///   The `forward` argument is ignored.
/// * [`PredictionDirection::Bidirectional`] — return the
///   [`average_predictions`] of the two inputs (both inputs must be
///   the same length).
/// * [`PredictionDirection::Skipped`] — return `forward.to_vec()`
///   (the caller is expected to have built the §7.6.3.5 implicit
///   zero-MV block into the `forward` slot).
///
/// Returns `None` only when the bidirectional case is selected and
/// the two inputs differ in length.
pub fn combine_directional_predictions(
    direction: PredictionDirection,
    forward: &[u8],
    backward: &[u8],
) -> Option<Vec<u8>> {
    match direction {
        PredictionDirection::Forward => Some(forward.to_vec()),
        PredictionDirection::Backward => Some(backward.to_vec()),
        PredictionDirection::Bidirectional => average_predictions(forward, backward),
        PredictionDirection::Skipped => Some(forward.to_vec()),
    }
}

/// Dual-prime alias for [`average_predictions`] that matches the
/// **§7.6.7.4** spelling
/// `pel_pred[y][x] = (pel_pred_same_parity[y][x] +
/// pel_pred_opposite_parity[y][x]) // 2`.
///
/// The arithmetic is identical to the bidirectional `(forward,
/// backward)` average — the spec gives the formula twice with the
/// only change being the operand labels. This helper exists for
/// caller readability when wiring the §7.6.3.6 dual-prime
/// opposite-parity vectors from
/// [`crate::dual_prime::derive_opposite_parity_vector`] through the
/// §7.6.4 reader and into the final prediction.
pub fn average_dual_prime_predictions(
    same_parity: &[u8],
    opposite_parity: &[u8],
) -> Option<Vec<u8>> {
    average_predictions(same_parity, opposite_parity)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- avg2: §4.1 // 2 rounding ----

    #[test]
    fn avg2_no_tie() {
        // (10 + 12) // 2 = 11
        assert_eq!(avg2(10, 12), 11);
        // (0 + 0) // 2 = 0
        assert_eq!(avg2(0, 0), 0);
        // (254 + 254) // 2 = 254
        assert_eq!(avg2(254, 254), 254);
    }

    #[test]
    fn avg2_tie_rounds_up() {
        // (10 + 11) // 2 = 11 (half-integer rounded away from zero).
        assert_eq!(avg2(10, 11), 11);
        // (0 + 1) // 2 = 1
        assert_eq!(avg2(0, 1), 1);
        // (254 + 255) // 2 = 255
        assert_eq!(avg2(254, 255), 255);
    }

    #[test]
    fn avg2_at_u8_max() {
        // (255 + 255) // 2 = 255
        assert_eq!(avg2(255, 255), 255);
    }

    // ---- average_predictions ----

    #[test]
    fn average_predictions_block_2x2() {
        let f = vec![10u8, 20, 30, 40];
        let b = vec![20u8, 30, 40, 50];
        // averages: 15, 25, 35, 45
        let out = average_predictions(&f, &b).expect("equal length");
        assert_eq!(out, vec![15, 25, 35, 45]);
    }

    #[test]
    fn average_predictions_zero_forward() {
        // averaging x with 0 gives ceil(x / 2)
        let f = vec![0u8; 4];
        let b = vec![1u8, 2, 3, 4];
        let out = average_predictions(&f, &b).expect("equal length");
        assert_eq!(out, vec![1, 1, 2, 2]);
    }

    #[test]
    fn average_predictions_rejects_length_mismatch() {
        let f = vec![1u8, 2, 3];
        let b = vec![4u8, 5];
        assert!(average_predictions(&f, &b).is_none());
    }

    #[test]
    fn average_predictions_empty() {
        // Empty input is well-defined: an empty output.
        let f: Vec<u8> = Vec::new();
        let b: Vec<u8> = Vec::new();
        let out = average_predictions(&f, &b).expect("empty equal length");
        assert!(out.is_empty());
    }

    #[test]
    fn average_predictions_16x16_block_consistency() {
        // 16×16 = 256 samples; check that the in-place and the
        // allocating variants agree.
        let mut f: Vec<u8> = (0..=255).collect();
        let b: Vec<u8> = (0..=255).rev().collect();
        let allocating = average_predictions(&f, &b).expect("len 256");
        let backup = f.clone();
        assert!(average_predictions_in_place(&mut f, &b));
        assert_eq!(f, allocating);
        assert_ne!(f, backup, "in-place must have mutated something");
    }

    // ---- average_predictions_in_place ----

    #[test]
    fn average_predictions_in_place_rejects_length_mismatch() {
        let mut f = vec![10u8, 20, 30];
        let pre = f.clone();
        let b = vec![1u8, 2];
        assert!(!average_predictions_in_place(&mut f, &b));
        assert_eq!(f, pre, "buffer must be unchanged on length mismatch");
    }

    #[test]
    fn average_predictions_in_place_matches_avg2() {
        let mut f = vec![10u8, 11];
        let b = vec![12u8, 13];
        assert!(average_predictions_in_place(&mut f, &b));
        // (10+12)//2 = 11; (11+13)//2 = 12
        assert_eq!(f, vec![11, 12]);
    }

    // ---- combine_directional_predictions ----

    #[test]
    fn combine_forward_only_returns_forward() {
        let f = vec![10u8, 20, 30, 40];
        let b: Vec<u8> = Vec::new(); // ignored
        let out = combine_directional_predictions(PredictionDirection::Forward, &f, &b)
            .expect("forward branch never returns None");
        assert_eq!(out, f);
    }

    #[test]
    fn combine_backward_only_returns_backward() {
        let f: Vec<u8> = Vec::new(); // ignored
        let b = vec![100u8, 110, 120, 130];
        let out = combine_directional_predictions(PredictionDirection::Backward, &f, &b)
            .expect("backward branch never returns None");
        assert_eq!(out, b);
    }

    #[test]
    fn combine_bidirectional_averages() {
        let f = vec![10u8, 20, 30, 40];
        let b = vec![20u8, 30, 40, 50];
        let out =
            combine_directional_predictions(PredictionDirection::Bidirectional, &f, &b).unwrap();
        assert_eq!(out, vec![15, 25, 35, 45]);
    }

    #[test]
    fn combine_bidirectional_rejects_length_mismatch() {
        let f = vec![10u8, 20];
        let b = vec![1u8, 2, 3];
        assert!(
            combine_directional_predictions(PredictionDirection::Bidirectional, &f, &b).is_none()
        );
    }

    #[test]
    fn combine_skipped_returns_forward_unchanged() {
        // Per §7.6.3.5, skipped non-intra macroblocks have an
        // implicit `(0, 0)` motion vector. The caller is expected to
        // have populated the `forward` slot with the formed-from-zero
        // prediction; combine_directional_predictions returns it
        // unchanged.
        let f = vec![7u8, 8, 9, 10];
        let b: Vec<u8> = Vec::new();
        let out = combine_directional_predictions(PredictionDirection::Skipped, &f, &b).unwrap();
        assert_eq!(out, f);
    }

    // ---- dual-prime alias ----

    #[test]
    fn dual_prime_average_matches_bidirectional() {
        let same = vec![10u8, 20, 30, 40];
        let opp = vec![20u8, 30, 40, 50];
        let dp = average_dual_prime_predictions(&same, &opp).unwrap();
        let bidir = average_predictions(&same, &opp).unwrap();
        assert_eq!(dp, bidir);
        assert_eq!(dp, vec![15, 25, 35, 45]);
    }

    #[test]
    fn dual_prime_rejects_length_mismatch() {
        let same = vec![10u8];
        let opp = vec![20u8, 30];
        assert!(average_dual_prime_predictions(&same, &opp).is_none());
    }

    // ---- spot-check vs. spec ----

    #[test]
    fn spec_example_zero_plus_one_rounds_up() {
        // Direct cross-check of the §4.1 `//` rounding rule: for any
        // x, (x + (x+1)) // 2 == x + 1, not x.
        for x in 0..=254u8 {
            assert_eq!(avg2(x, x + 1), x + 1, "(x + (x+1)) // 2 with x={}", x);
        }
    }

    #[test]
    fn average_is_symmetric_in_arguments() {
        // (a + b) // 2 is commutative; the spec writes
        // "pel_pred_forward + pel_pred_backward" but the formula is
        // symmetric in the two operands.
        let f: Vec<u8> = (0..32).collect();
        let b: Vec<u8> = (32..64).collect();
        let fb = average_predictions(&f, &b).unwrap();
        let bf = average_predictions(&b, &f).unwrap();
        assert_eq!(fb, bf);
    }
}
