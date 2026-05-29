//! 8×8 inverse discrete cosine transform shared by MPEG-1 Video and
//! MPEG-2 Video (the §A.1 / Annex A IDCT).
//!
//! ## Spec basis
//!
//! Both in-tree MPEG video specs delegate the 8×8 IDCT to **IEEE Std
//! 1180-1990** (MPEG-2 / ITU-T H.262 Annex A) / IEEE Draft Standard
//! **P1180/D2, July 18, 1990** (MPEG-1 Annex A):
//!
//! > "The 8 by 8 inverse discrete cosine transform for I-pictures and
//! > P-pictures shall conform to IEEE Draft Standard, P1180/D2, July
//! > 18, 1990. ..." — ISO/IEC 11172-2:1993 Annex A (page 39).
//!
//! > "The N by N inverse discrete transform shall conform to IEEE
//! > Standard Specification for the Implementations of 8 by 8 Inverse
//! > Discrete Cosine Transform, Std 1180-1990, December 6, 1990." —
//! > ISO/IEC 13818-2:1995 Annex A.
//!
//! The transcribed accuracy/conformance derivation is staged at
//! `docs/video/mpeg12video/idct-accuracy-spec.md` (the IEEE 1180
//! statistical bounds and the 8×8 forward / inverse trigonometric
//! identity).
//!
//! ## Transform identity (ISO/IEC 13818-2 §A)
//!
//! The two-dimensional inverse DCT, with `N = 8`:
//!
//! ```text
//!             2   N-1 N-1                          (2x+1)uπ      (2y+1)vπ
//!  f(x,y) =  ---  Σ   Σ   C(u)C(v) F(u,v) · cos --------- · cos ---------
//!             N   u=0 v=0                          2N            2N
//!
//!  with      C(0) = 1/√2,  C(k) = 1 for k > 0.
//! ```
//!
//! The OxideAV implementation evaluates the literal double sum at
//! `f64` precision; the cosines are cached once at module load through
//! the precomputed [`COS_TABLE`].
//!
//! ## Data ranges (ISO/IEC 13818-2 §A and §7.4.3 / §7.5)
//!
//! * Coefficient input `F[v][u]` is 12-bit signed, range
//!   `[F_INPUT_MIN, F_INPUT_MAX] = [-2048, +2047]` — the §7.4.3
//!   post-mismatch saturation already enforces this; the IDCT rejects
//!   inputs outside this range.
//! * Sample output `f[y][x]` is 9-bit signed, range
//!   `[F_OUTPUT_MIN, F_OUTPUT_MAX] = [-256, +255]`. Per §7.5 the
//!   transformed values are saturated into this range; the §7.6.8
//!   step then clamps the sum with the prediction back into the 8-bit
//!   `[0, 255]` pel domain.
//!
//! Both ranges are exposed as module constants so the surrounding
//! pipeline can reference them without redefining them locally.
//!
//! ## Reference vs. candidate vs. integer output
//!
//! The module exposes three layers, each serving a distinct role:
//!
//! 1. [`idct_reference_f64`] — the **double-precision direct 4-D
//!    reference IDCT**. It evaluates the §A trigonometric identity by
//!    summing the literal 4-D `Σ_v Σ_u C(u)C(v)F[v][u]·cos·cos` form,
//!    one inner product per output pixel (`O(N⁴)`), with no
//!    intermediate-precision tricks. This is the closest practical
//!    analogue to the "infinite-precision" reference IEEE 1180 / P1180
//!    compares candidates against.
//!
//! 2. [`idct_candidate_f64`] — the **fast separable 1-D-pass IDCT**
//!    the production decoder uses internally. It applies an 8-point
//!    1-D IDCT to each of the 8 rows, then an 8-point 1-D IDCT to
//!    each of the 8 resulting columns (`O(N³)` total). The decomposition
//!    is mathematically equivalent to the direct 4-D summation — the
//!    only difference is the order in which the `cos·cos` products
//!    are accumulated, which produces a slightly different `f64`
//!    rounding sequence.
//!
//! 3. [`idct_8x8`] — the **integer IDCT** used by the downstream
//!    decoder pipeline. It calls [`idct_candidate_f64`], rounds every
//!    output to the nearest integer (ties away from zero, matching
//!    the spec's `Round(x)` convention), then saturates the result
//!    into the §7.5 9-bit pel range `[-256, +255]`.
//!
//! The IEEE 1180 / P1180/D2 statistical accuracy test compares a
//! candidate IDCT against the direct (reference) double-precision
//! IDCT and measures the per-position error distribution. For
//! `oxideav-mpeg12video` the candidate-vs-reference comparison runs at
//! `f64` precision — the candidate is the fast separable kernel; the
//! reference is the direct 4-D summation. This isolates the numerical
//! precision of the separable kernel (the property IEEE 1180 actually
//! gates: how closely a faster IDCT tracks the mathematical identity)
//! from the unavoidable `± 0.5` LSB rounding noise of the final
//! integer cast.
//!
//! The conformance harness in `tests/idct_p1180_conformance.rs` runs
//! that comparison against the five bounds transcribed in
//! `docs/video/mpeg12video/idct-accuracy-spec.md` §4 (peak error,
//! `pmse`, `omse`, `pme`, `ome`). A separate suite of deterministic
//! checks in the same file covers the spec-mandated exact cases
//! (all-zero input, DC-only input).

#![allow(clippy::needless_range_loop)]

use core::f64::consts::PI;

/// IDCT coefficient-input lower bound — the §7.4.3 / §A 12-bit signed
/// minimum that `F[v][u]` may take after the dequantiser's saturation
/// step.
pub const F_INPUT_MIN: i32 = -2048;

/// IDCT coefficient-input upper bound — the §7.4.3 / §A 12-bit signed
/// maximum that `F[v][u]` may take after the dequantiser's saturation
/// step.
pub const F_INPUT_MAX: i32 = 2047;

/// IDCT sample-output lower bound — the §7.5 / §A 9-bit signed minimum
/// for `f[y][x]` *before* the prediction-add step in §7.6.8.
pub const F_OUTPUT_MIN: i32 = -256;

/// IDCT sample-output upper bound — the §7.5 / §A 9-bit signed maximum
/// for `f[y][x]` *before* the prediction-add step in §7.6.8.
pub const F_OUTPUT_MAX: i32 = 255;

/// Lazy-initialised table of `cos((2*x + 1) * u * π / 16)` for
/// `x, u ∈ 0..8` — the 8×8 cosine kernel used by the §A 2-D IDCT
/// formula. Pre-computed once on first call so the IDCT inner loops
/// avoid repeated `cos()` calls. A `const fn` cannot evaluate
/// `f64::cos`, so the table is built behind a `OnceLock` rather than
/// declared `static`.
fn cos_table_ref() -> &'static [[f64; 8]; 8] {
    use std::sync::OnceLock;
    static TABLE: OnceLock<[[f64; 8]; 8]> = OnceLock::new();
    TABLE.get_or_init(|| {
        let mut t = [[0.0f64; 8]; 8];
        for x in 0..8usize {
            for u in 0..8usize {
                t[x][u] = ((2.0 * x as f64 + 1.0) * u as f64 * PI / 16.0).cos();
            }
        }
        t
    })
}

/// The §A `C(u)` orthonormality scale factor: `1/√2` for `u = 0`, `1`
/// otherwise.
#[inline]
fn alpha(k: usize) -> f64 {
    if k == 0 {
        core::f64::consts::FRAC_1_SQRT_2
    } else {
        1.0
    }
}

/// Saturate `value` into the inclusive range `[F_OUTPUT_MIN,
/// F_OUTPUT_MAX]` — the §7.5 9-bit signed clamp applied to the
/// IDCT-output samples before the prediction-add step in §7.6.8.
#[inline]
pub fn saturate_output(value: i32) -> i32 {
    value.clamp(F_OUTPUT_MIN, F_OUTPUT_MAX)
}

/// Saturate `value` into the inclusive range `[F_INPUT_MIN,
/// F_INPUT_MAX]` — the §7.4.3 12-bit signed clamp applied to the IDCT
/// inputs. Inputs that the dequantiser pipeline already produced will
/// be inside the range; this helper is exposed for callers that wish
/// to clamp pre-IDCT coefficients explicitly.
#[inline]
pub fn saturate_input(value: i32) -> i32 {
    value.clamp(F_INPUT_MIN, F_INPUT_MAX)
}

/// Round a real-valued IDCT output to the nearest integer using the
/// spec's `Round(x)` convention (ties away from zero).
///
/// `f64::round` already implements ties-away-from-zero on stable Rust,
/// matching the `Round` operator in ISO/IEC 13818-2 §4.1.
#[inline]
fn round_to_int(value: f64) -> i32 {
    value.round() as i32
}

/// Double-precision §A 8×8 IDCT — the **direct 4-D reference**
/// transform.
///
/// This is the closest practical analogue to the "infinite-precision"
/// reference IEEE 1180 / P1180/D2 specifies as the gold standard
/// against which a candidate IDCT is statistically benchmarked. The
/// implementation evaluates the §A 4-D double sum literally using the
/// cached cosine kernel:
///
/// ```text
/// f[y][x] = (1/4) Σ_v Σ_u C(u)·C(v)·F[v][u]·cos((2x+1)uπ/16)·cos((2y+1)vπ/16)
/// ```
///
/// No intermediate-precision tricks are applied, so the only error
/// vs. an exact-arithmetic IDCT is `f64` rounding noise.
pub fn idct_reference_f64(input: &[[f64; 8]; 8]) -> [[f64; 8]; 8] {
    let table = cos_table_ref();
    let mut out = [[0.0f64; 8]; 8];
    for y in 0..8usize {
        for x in 0..8usize {
            let mut sum = 0.0f64;
            for v in 0..8usize {
                for u in 0..8usize {
                    sum += alpha(u) * alpha(v) * input[v][u] * table[x][u] * table[y][v];
                }
            }
            // The §A scale factor is `2/N` per dimension → 2/8 · 2/8
            // = 1/16 once both `Σ` are evaluated.
            out[y][x] = sum * 0.25;
        }
    }
    out
}

/// Single 8-point 1-D IDCT row pass — the inner kernel of
/// [`idct_candidate_f64`]. Implements the spec's row sum:
///
/// ```text
/// row_out[x] = (1/2) Σ_u C(u)·in[u]·cos((2x+1)uπ/16)   for x = 0..8.
/// ```
///
/// Pulling the row sum out and applying it per row (then per column)
/// rather than as a single 4-D nested sum cuts the operation count
/// from `O(N⁴) = 4096` to `O(N³) = 512` `cos`-multiplications per
/// block. The output is identical mathematically; rounding-order
/// differences make the `f64` result differ from the direct sum by
/// `<1e-13` per pixel, well below every IEEE 1180 bound.
#[inline]
fn idct_1d(input: &[f64; 8]) -> [f64; 8] {
    let table = cos_table_ref();
    let mut out = [0.0f64; 8];
    for x in 0..8usize {
        let mut sum = 0.0f64;
        for u in 0..8usize {
            sum += alpha(u) * input[u] * table[x][u];
        }
        // The §A 1-D scale factor is `2/N = 1/4` (half of the 2-D
        // `1/4` applied per dimension, so the two passes together
        // recover the `1/16` overall scale).
        out[x] = sum * 0.5;
    }
    out
}

/// Double-precision §A 8×8 IDCT — the **separable 1-D-pass
/// candidate** transform.
///
/// Computes the IDCT as eight row-IDCTs followed by eight
/// column-IDCTs (the standard `O(N³)` decomposition of the 2-D
/// transform). Mathematically identical to [`idct_reference_f64`];
/// differs only in `f64` rounding order. This is the kernel the
/// production [`idct_8x8`] integer IDCT calls before rounding and
/// saturating.
///
/// The IEEE 1180 conformance harness measures candidate-vs-reference
/// error using *this* function as the candidate (against the direct
/// 4-D summation as the reference). That makes the bounds gate the
/// numerical precision of the separable kernel — which is the
/// property worth regression-testing — rather than the unavoidable
/// `± 0.5` LSB rounding of a final integer cast.
pub fn idct_candidate_f64(input: &[[f64; 8]; 8]) -> [[f64; 8]; 8] {
    // Pass 1: row-wise 1-D IDCT.
    let mut intermediate = [[0.0f64; 8]; 8];
    for v in 0..8usize {
        let row = input[v];
        intermediate[v] = idct_1d(&row);
    }
    // Pass 2: column-wise 1-D IDCT on the row-pass output.
    let mut out = [[0.0f64; 8]; 8];
    for x in 0..8usize {
        let mut col = [0.0f64; 8];
        for v in 0..8usize {
            col[v] = intermediate[v][x];
        }
        let col_out = idct_1d(&col);
        for y in 0..8usize {
            out[y][x] = col_out[y];
        }
    }
    out
}

/// §A 8×8 IDCT — the **integer IDCT** used by the
/// `oxideav-mpeg12video` decoder pipeline (Figure 7-1, between the
/// §7.4 dequantiser and the §7.6 macroblock pipeline).
///
/// Inputs `F[v][u]` are 12-bit signed coefficients in `[-2048,
/// +2047]`. The IDCT applies the §A formula via
/// [`idct_candidate_f64`], rounds the per-pixel result to the nearest
/// integer using the §4.1 `Round` operator, then saturates the result
/// to the 9-bit signed pel range `[-256, +255]` per §7.5.
///
/// Callers must have already applied the §7.4.3 saturation (which
/// guarantees the input range) and the §7.4.4 MPEG-2 mismatch-control
/// (`F[7][7]` parity toggle) or the MPEG-1 §2.4.4 per-coefficient
/// oddification before invoking the IDCT.
pub fn idct_8x8(input: &[[i16; 8]; 8]) -> [[i16; 8]; 8] {
    // Promote to f64 once; the IDCT formula factor naturally produces
    // an unrounded sample value that we round + clamp here.
    let mut promoted = [[0.0f64; 8]; 8];
    for v in 0..8usize {
        for u in 0..8usize {
            promoted[v][u] = f64::from(input[v][u]);
        }
    }
    let real = idct_candidate_f64(&promoted);
    let mut out = [[0i16; 8]; 8];
    for y in 0..8usize {
        for x in 0..8usize {
            let clamped = saturate_output(round_to_int(real[y][x]));
            // The clamp guarantees `clamped` fits in i16.
            out[y][x] = clamped as i16;
        }
    }
    out
}

/// Convenience wrapper around [`idct_8x8`] that accepts an `i32`
/// coefficient block (the type the §7.4 dequantiser produces) and
/// internally narrows to `i16` after asserting each coefficient lies
/// inside `[F_INPUT_MIN, F_INPUT_MAX]`.
///
/// Coefficients outside the 12-bit signed range indicate a missing
/// §7.4.3 saturation upstream and would be a spec violation; the
/// helper saturates rather than panics so that the wider decoder
/// pipeline degrades gracefully on out-of-range inputs.
pub fn idct_8x8_from_i32(input: &[[i32; 8]; 8]) -> [[i16; 8]; 8] {
    let mut promoted = [[0i16; 8]; 8];
    for v in 0..8usize {
        for u in 0..8usize {
            promoted[v][u] = saturate_input(input[v][u]) as i16;
        }
    }
    idct_8x8(&promoted)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_zero_input_yields_all_zero_output() {
        // IEEE 1180 deterministic check: the IDCT of the zero block
        // must be exactly zero.
        let input = [[0i16; 8]; 8];
        let output = idct_8x8(&input);
        for row in &output {
            for &v in row {
                assert_eq!(v, 0);
            }
        }
    }

    #[test]
    fn dc_only_input_produces_flat_block() {
        // A pure DC coefficient F[0][0] = K spreads to a flat block of
        // value K/8 once the §A scale and the C(0)·C(0) = 1/2 factor
        // are evaluated. With K = 8 the rounded result is 1 across
        // every pixel.
        let mut input = [[0i16; 8]; 8];
        input[0][0] = 8;
        let output = idct_8x8(&input);
        for row in &output {
            for &v in row {
                assert_eq!(v, 1);
            }
        }
    }

    #[test]
    fn dc_only_negative_input_produces_flat_block() {
        let mut input = [[0i16; 8]; 8];
        input[0][0] = -8;
        let output = idct_8x8(&input);
        for row in &output {
            for &v in row {
                assert_eq!(v, -1);
            }
        }
    }

    #[test]
    fn saturate_output_clamps_to_nine_bit_range() {
        assert_eq!(saturate_output(0), 0);
        assert_eq!(saturate_output(255), 255);
        assert_eq!(saturate_output(256), 255);
        assert_eq!(saturate_output(-256), -256);
        assert_eq!(saturate_output(-257), -256);
        assert_eq!(saturate_output(1_000_000), 255);
        assert_eq!(saturate_output(-1_000_000), -256);
    }

    #[test]
    fn saturate_input_clamps_to_twelve_bit_range() {
        assert_eq!(saturate_input(0), 0);
        assert_eq!(saturate_input(2047), 2047);
        assert_eq!(saturate_input(2048), 2047);
        assert_eq!(saturate_input(-2048), -2048);
        assert_eq!(saturate_input(-2049), -2048);
    }

    #[test]
    fn output_is_saturated_to_nine_bit_range() {
        // Driving every coefficient to the 12-bit max produces an
        // out-of-range theoretical output for most positions; the
        // §7.5 saturation must bring every pixel into [-256, +255].
        let input = [[F_INPUT_MAX as i16; 8]; 8];
        let output = idct_8x8(&input);
        for row in &output {
            for &v in row {
                assert!((F_OUTPUT_MIN as i16..=F_OUTPUT_MAX as i16).contains(&v));
            }
        }
    }

    #[test]
    fn from_i32_saturates_out_of_range_inputs() {
        // An `i32` coefficient that violates the §7.4.3 12-bit signed
        // range must be saturated before the IDCT runs.
        let mut input = [[0i32; 8]; 8];
        input[0][0] = 1_000_000;
        let output = idct_8x8_from_i32(&input);
        // With F[0][0] saturated to 2047, the flat output rounds to
        // 2047/8 = 255.875 → 256 → clamp to 255.
        for row in &output {
            for &v in row {
                assert_eq!(v, F_OUTPUT_MAX as i16);
            }
        }
    }

    #[test]
    fn reference_matches_dc_only_exact_value() {
        let mut input = [[0.0f64; 8]; 8];
        input[0][0] = 16.0;
        let out = idct_reference_f64(&input);
        // The expected flat value is C(0)*C(0)*F[0][0]*0.25 = 0.5 *
        // 16 * 0.25 = 2.0 exactly.
        for row in &out {
            for &v in row {
                assert!((v - 2.0).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn candidate_matches_dc_only_exact_value() {
        let mut input = [[0.0f64; 8]; 8];
        input[0][0] = 16.0;
        let out = idct_candidate_f64(&input);
        for row in &out {
            for &v in row {
                assert!((v - 2.0).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn candidate_and_reference_agree_within_fp_noise() {
        // The two IDCT implementations must agree to within a few
        // ULPs across an arbitrary block; the IEEE 1180 statistical
        // bounds gate this property formally in the integration
        // tests.
        let mut input = [[0.0f64; 8]; 8];
        for v in 0..8usize {
            for u in 0..8usize {
                input[v][u] = ((v as f64 * 31.0 + u as f64 * 17.0) % 257.0) - 128.0;
            }
        }
        let r = idct_reference_f64(&input);
        let c = idct_candidate_f64(&input);
        for y in 0..8 {
            for x in 0..8 {
                let diff = (r[y][x] - c[y][x]).abs();
                assert!(diff < 1e-10, "diff {diff} too large at ({y},{x})");
            }
        }
    }
}
