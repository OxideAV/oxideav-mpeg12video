//! §7.6.8 Adding prediction and coefficient data per ISO/IEC 13818-2
//! (Recommendation ITU-T H.262), page 106 of the 1995 base text — the
//! final reconstruction step that adds the IDCT output `f[y][x]` to the
//! prediction-block plane `p[y][x]` and saturates the result to
//! 8-bit unsigned range.
//!
//! ## What §7.6.8 specifies
//!
//! After §7.6.7 has produced the final per-component prediction sample
//! plane `p[y][x]` (reorganised to match the field/frame structure of
//! the transform data — see §7.6.7.1 and the `dct_type` flag), and
//! after §7.6.5 / §A.1 has produced the inverse-DCT output `f[y][x]`,
//! the decoder forms the final 8×8 decoded-sample block by the loop:
//!
//! ```text
//! for (y = 0; y < 8; y++) {
//!     for (x = 0; x < 8; x++) {
//!         d[y][x] = f[y][x] + p[y][x];
//!         if (d[y][x] < 0)   d[y][x] = 0;
//!         if (d[y][x] > 255) d[y][x] = 255;
//!     }
//! }
//! ```
//!
//! The spec writes the loop over an 8×8 block (the IDCT transform
//! size); the operation is intrinsically pointwise, so a `width ×
//! height` driver folds out to the same arithmetic without
//! restricting the geometry — every prediction block emitted by
//! §7.6.4 / §7.6.7 has a matching transform plane that pairs into it
//! sample-by-sample.
//!
//! `f[y][x]` is the §A.1 IDCT output — an `i16`-range signed
//! integer; §A.1 page 195 bounds the IDCT output to `[-256, 255]` for
//! intra blocks and `[-256, 256]` for non-intra blocks, but the
//! arithmetic here is independent of that bound. `p[y][x]` is the
//! §7.6.7 prediction sample — a `u8`. `d[y][x]` is the final
//! reconstructed sample — clamped to `[0, 255]` per the spec's two
//! `if` clauses.
//!
//! ## What this module provides
//!
//! * [`saturate`] — clamp a single `i32` sum to `[0, 255]`, returning
//!   a `u8`. The arithmetic is the same as the spec's two `if`
//!   clauses, factored out for reuse.
//! * [`add_prediction_and_coefficients`] — pointwise
//!   `d[i] = saturate(f[i] + p[i])` across two equal-length input
//!   slices. Returns the result in a fresh `Vec<u8>`.
//! * [`add_prediction_and_coefficients_in_place`] — the same
//!   operation, writing the result back into the prediction buffer.
//! * [`add_intra_block`] — the spec's intra-macroblock shortcut: when
//!   the macroblock is `macroblock_intra == 1`, no prediction step
//!   has run, so the prediction is conceptually all-zero and the
//!   final samples are the saturated IDCT output. Returns
//!   `saturate(f[i])` across the input slice.
//!
//! All helpers refuse a length mismatch between the IDCT and the
//! prediction slices — that would imply a caller bug, since the
//! §7.6.4 / §7.6.5 prediction-block geometry is identical to the
//! §A.1 transform-block geometry by construction.
//!
//! Spec citations refer to **ISO/IEC 13818-2 (H.262) §7.6.8**.

/// Saturate a single `i32` sum to the 8-bit unsigned range per the
/// two `if` clauses of **§7.6.8** page 106:
///
/// ```text
/// if (d < 0)   d = 0;
/// if (d > 255) d = 255;
/// ```
///
/// Implemented via `i32::clamp` which is bit-equivalent to the
/// spec's two-branch form for any integer input.
#[inline]
pub fn saturate(value: i32) -> u8 {
    value.clamp(0, 255) as u8
}

/// Pointwise add the IDCT output `f` and the prediction `p`, then
/// saturate each sample to `[0, 255]` per **§7.6.8**.
///
/// Returns the `width * height`-shaped final-decoded-sample buffer as
/// a fresh `Vec<u8>` of the same length as both inputs. Returns
/// `None` if the two inputs are of different lengths.
///
/// The function is geometry-agnostic — the spec writes the loop over
/// 8×8, but the operation is intrinsically pointwise and the
/// signature works for any block size the §7.6.5 / §A.1 chain
/// produces (8×8 transform blocks are the standard case; the chroma
/// scaling in §7.6.3.7 / §7.6.7 produces matching transforms).
pub fn add_prediction_and_coefficients(transform: &[i16], prediction: &[u8]) -> Option<Vec<u8>> {
    if transform.len() != prediction.len() {
        return None;
    }
    let mut out = Vec::with_capacity(transform.len());
    for (f, p) in transform.iter().zip(prediction.iter()) {
        let sum = *f as i32 + *p as i32;
        out.push(saturate(sum));
    }
    Some(out)
}

/// In-place variant of [`add_prediction_and_coefficients`]: overwrite
/// the `prediction` buffer with the per-sample saturated sum of the
/// IDCT output `transform` and the prediction. Returns `false` and
/// leaves the buffer unchanged if the inputs are of different lengths.
pub fn add_prediction_and_coefficients_in_place(prediction: &mut [u8], transform: &[i16]) -> bool {
    if prediction.len() != transform.len() {
        return false;
    }
    for (p, f) in prediction.iter_mut().zip(transform.iter()) {
        let sum = *f as i32 + *p as i32;
        *p = saturate(sum);
    }
    true
}

/// Intra-macroblock shortcut: no prediction step has run for an
/// intra macroblock (`macroblock_intra == 1`), so the final
/// reconstructed samples are simply the §A.1 IDCT output saturated
/// to `[0, 255]`.
///
/// Mathematically this is equivalent to
/// [`add_prediction_and_coefficients`] with an all-zero prediction
/// buffer; the dedicated entry point lets the caller skip allocating
/// a zero buffer in the common intra path.
///
/// Note: the §A.1 IDCT output for an intra block is conventionally in
/// `[-256, 255]`, well inside the `i16` range used here.
pub fn add_intra_block(transform: &[i16]) -> Vec<u8> {
    let mut out = Vec::with_capacity(transform.len());
    for f in transform.iter() {
        out.push(saturate(*f as i32));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- saturate ----

    #[test]
    fn saturate_in_range_passes_through() {
        assert_eq!(saturate(0), 0);
        assert_eq!(saturate(1), 1);
        assert_eq!(saturate(127), 127);
        assert_eq!(saturate(255), 255);
    }

    #[test]
    fn saturate_negative_clamps_to_zero() {
        assert_eq!(saturate(-1), 0);
        assert_eq!(saturate(-128), 0);
        assert_eq!(saturate(-256), 0);
        assert_eq!(saturate(i32::MIN), 0);
    }

    #[test]
    fn saturate_above_max_clamps_to_255() {
        assert_eq!(saturate(256), 255);
        assert_eq!(saturate(511), 255);
        assert_eq!(saturate(1000), 255);
        assert_eq!(saturate(i32::MAX), 255);
    }

    // ---- add_prediction_and_coefficients ----

    #[test]
    fn add_in_range_64_block() {
        // 8×8 = 64 samples, all in-range; verify pointwise sum.
        let prediction: Vec<u8> = (0..64).collect();
        let transform: Vec<i16> = (0..64).collect();
        let out = add_prediction_and_coefficients(&transform, &prediction).expect("equal length");
        assert_eq!(out.len(), 64);
        for (i, sample) in out.iter().enumerate() {
            assert_eq!(*sample, (i + i) as u8); // 0+0, 1+1, … 63+63
        }
    }

    #[test]
    fn add_saturates_negative_sum_to_zero() {
        let prediction = vec![10u8, 20, 30, 40];
        let transform = vec![-50i16, -100, -300, -1];
        // sums: -40, -80, -270, 39 -> 0, 0, 0, 39
        let out = add_prediction_and_coefficients(&transform, &prediction).unwrap();
        assert_eq!(out, vec![0, 0, 0, 39]);
    }

    #[test]
    fn add_saturates_overflow_sum_to_255() {
        let prediction = vec![250u8, 255, 200, 128];
        let transform = vec![20i16, 1, 100, 500];
        // sums: 270, 256, 300, 628 -> 255, 255, 255, 255
        let out = add_prediction_and_coefficients(&transform, &prediction).unwrap();
        assert_eq!(out, vec![255, 255, 255, 255]);
    }

    #[test]
    fn add_zero_transform_returns_prediction_unchanged() {
        // f[y][x] = 0 everywhere -> d[y][x] = p[y][x] (no clamping
        // since p is u8).
        let prediction: Vec<u8> = (0..32).collect();
        let transform = vec![0i16; 32];
        let out = add_prediction_and_coefficients(&transform, &prediction).unwrap();
        assert_eq!(out, prediction);
    }

    #[test]
    fn add_zero_prediction_returns_saturated_transform() {
        // p[y][x] = 0 -> d[y][x] = saturate(f[y][x])
        let prediction = vec![0u8; 5];
        let transform = vec![-10i16, 0, 50, 255, 300];
        let out = add_prediction_and_coefficients(&transform, &prediction).unwrap();
        assert_eq!(out, vec![0, 0, 50, 255, 255]);
    }

    #[test]
    fn add_rejects_length_mismatch() {
        let prediction = vec![10u8, 20, 30];
        let transform = vec![1i16, 2];
        assert!(add_prediction_and_coefficients(&transform, &prediction).is_none());
    }

    #[test]
    fn add_empty_inputs_well_defined() {
        let p: Vec<u8> = Vec::new();
        let f: Vec<i16> = Vec::new();
        let out = add_prediction_and_coefficients(&f, &p).expect("empty equal length");
        assert!(out.is_empty());
    }

    // ---- add_prediction_and_coefficients_in_place ----

    #[test]
    fn add_in_place_matches_allocating() {
        let mut prediction: Vec<u8> = (0..16).map(|i| i as u8).collect();
        let transform: Vec<i16> = (0..16).map(|i| i as i16 - 4).collect();
        let allocating = add_prediction_and_coefficients(&transform, &prediction).unwrap();
        assert!(add_prediction_and_coefficients_in_place(
            &mut prediction,
            &transform
        ));
        assert_eq!(prediction, allocating);
    }

    #[test]
    fn add_in_place_rejects_length_mismatch_unchanged() {
        let mut prediction = vec![10u8, 20, 30];
        let pre = prediction.clone();
        let transform = vec![1i16, 2];
        assert!(!add_prediction_and_coefficients_in_place(
            &mut prediction,
            &transform
        ));
        assert_eq!(prediction, pre);
    }

    // ---- add_intra_block ----

    #[test]
    fn add_intra_block_matches_zero_prediction() {
        // The intra shortcut must be bit-identical to passing an
        // all-zero prediction buffer.
        let transform: Vec<i16> = vec![-50, 0, 25, 255, 256, -1, 100, 1000];
        let intra = add_intra_block(&transform);
        let p_zero = vec![0u8; transform.len()];
        let via_zero = add_prediction_and_coefficients(&transform, &p_zero).unwrap();
        assert_eq!(intra, via_zero);
        // Independent saturation check:
        assert_eq!(intra, vec![0, 0, 25, 255, 255, 0, 100, 255]);
    }

    #[test]
    fn add_intra_block_empty() {
        let out = add_intra_block(&[]);
        assert!(out.is_empty());
    }

    // ---- spec-style 8×8 block ----

    #[test]
    fn add_8x8_block_spec_loop_shape() {
        // The spec writes the loop over an 8×8 block; verify a
        // canonical 64-sample call works and applies clamps in the
        // right places.
        let prediction = vec![128u8; 64];
        // f = +127 on first 32 samples, -200 on the last 32.
        let mut transform: Vec<i16> = Vec::with_capacity(64);
        transform.extend(std::iter::repeat_n(127i16, 32));
        transform.extend(std::iter::repeat_n(-200i16, 32));
        let out = add_prediction_and_coefficients(&transform, &prediction).unwrap();
        for sample in out.iter().take(32) {
            // 128 + 127 = 255, no clamp needed.
            assert_eq!(*sample, 255);
        }
        for sample in out.iter().skip(32) {
            // 128 + (-200) = -72, clamped to 0.
            assert_eq!(*sample, 0);
        }
    }
}
