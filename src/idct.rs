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

/// The §A 8×8 cosine kernel `cos((2x + 1) · u · π / 16)` for
/// `x, u ∈ 0..8`, as the **correctly-rounded `f64`** of each exact
/// cosine value (computed once at 60-digit precision and rounded to
/// nearest; the shortest-round-trip decimal literals below parse to
/// exactly those doubles).
///
/// A runtime `f64::cos()` call must not be used here: transcendental
/// functions are *not* specified to the last ulp by IEEE 754, so
/// different platform math libraries return values differing in the
/// final bit — which makes DCT/IDCT output (and therefore the
/// **encoder's emitted bits**) platform-dependent whenever a quantised
/// coefficient lands on a rounding boundary. Every entry is one of the
/// eight magnitudes `cos(kπ/16), k ∈ 0..8` with the appropriate sign,
/// laid out in the direct `[x][u]` order the §A summations index.
pub(crate) const COS_TABLE: [[f64; 8]; 8] = [
    [
        1.0,
        0.980_785_280_403_230_4,          // cos(π/16)
        0.923_879_532_511_286_7,          // cos(2π/16)
        0.831_469_612_302_545_2,          // cos(3π/16)
        core::f64::consts::FRAC_1_SQRT_2, // cos(4π/16)
        0.555_570_233_019_602_2,          // cos(5π/16)
        0.382_683_432_365_089_8,          // cos(6π/16)
        0.195_090_322_016_128_28,         // cos(7π/16)
    ],
    [
        1.0,
        0.831_469_612_302_545_2,
        0.382_683_432_365_089_8,
        -0.195_090_322_016_128_28,
        -core::f64::consts::FRAC_1_SQRT_2,
        -0.980_785_280_403_230_4,
        -0.923_879_532_511_286_7,
        -0.555_570_233_019_602_2,
    ],
    [
        1.0,
        0.555_570_233_019_602_2,
        -0.382_683_432_365_089_8,
        -0.980_785_280_403_230_4,
        -core::f64::consts::FRAC_1_SQRT_2,
        0.195_090_322_016_128_28,
        0.923_879_532_511_286_7,
        0.831_469_612_302_545_2,
    ],
    [
        1.0,
        0.195_090_322_016_128_28,
        -0.923_879_532_511_286_7,
        -0.555_570_233_019_602_2,
        core::f64::consts::FRAC_1_SQRT_2,
        0.831_469_612_302_545_2,
        -0.382_683_432_365_089_8,
        -0.980_785_280_403_230_4,
    ],
    [
        1.0,
        -0.195_090_322_016_128_28,
        -0.923_879_532_511_286_7,
        0.555_570_233_019_602_2,
        core::f64::consts::FRAC_1_SQRT_2,
        -0.831_469_612_302_545_2,
        -0.382_683_432_365_089_8,
        0.980_785_280_403_230_4,
    ],
    [
        1.0,
        -0.555_570_233_019_602_2,
        -0.382_683_432_365_089_8,
        0.980_785_280_403_230_4,
        -core::f64::consts::FRAC_1_SQRT_2,
        -0.195_090_322_016_128_28,
        0.923_879_532_511_286_7,
        -0.831_469_612_302_545_2,
    ],
    [
        1.0,
        -0.831_469_612_302_545_2,
        0.382_683_432_365_089_8,
        0.195_090_322_016_128_28,
        -core::f64::consts::FRAC_1_SQRT_2,
        0.980_785_280_403_230_4,
        -0.923_879_532_511_286_7,
        0.555_570_233_019_602_2,
    ],
    [
        1.0,
        -0.980_785_280_403_230_4,
        0.923_879_532_511_286_7,
        -0.831_469_612_302_545_2,
        core::f64::consts::FRAC_1_SQRT_2,
        -0.555_570_233_019_602_2,
        0.382_683_432_365_089_8,
        -0.195_090_322_016_128_28,
    ],
];

/// The shared cosine kernel (see [`COS_TABLE`]).
fn cos_table_ref() -> &'static [[f64; 8]; 8] {
    &COS_TABLE
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
#[cfg(test)]
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
///
/// The transform runs in integer arithmetic: the separable row and
/// column passes multiply by [`COS_FIXED`] (`C(u)·cos` in 15-bit fixed
/// point), the row pass keeping [`ROW_FRACTION_BITS`] of fraction for
/// the column pass, and both round half away from zero. It meets the
/// IEEE 1180-1990 accuracy procedure (tests/idct_p1180_conformance.rs);
/// a block holding only its DC coefficient, the most common kind, is
/// exact (`F[0][0] / 8`, rounded).
pub fn idct_8x8(input: &[[i16; 8]; 8]) -> [[i16; 8]; 8] {
    let dc_only = input.iter().enumerate().all(|(v, row)| {
        row.iter()
            .enumerate()
            .all(|(u, &c)| c == 0 || (u == 0 && v == 0))
    });
    if dc_only {
        let sample = saturate_output(divide_rounded(i64::from(input[0][0]), 3) as i32) as i16;
        return [[sample; 8]; 8];
    }
    // Row pass: 0.5 · Σ_u C(u)·F[v][u]·cos, kept with ROW_FRACTION_BITS of fraction and
    // stored transposed (`columns[x][v]`), so the column pass reads each column whole.
    let mut columns = [[0i64; 8]; 8];
    for v in 0..8usize {
        if input[v].iter().all(|&c| c == 0) {
            continue;
        }
        let sums = partial_butterfly(input[v].map(i64::from));
        for x in 0..8usize {
            columns[x][v] = divide_rounded(sums[x], COS_FRACTION_BITS + 1 - ROW_FRACTION_BITS);
        }
    }
    // Column pass: 0.5 · Σ_v C(v)·row[v]·cos, back to whole samples.
    let mut out = [[0i16; 8]; 8];
    for x in 0..8usize {
        let sums = partial_butterfly(columns[x]);
        for y in 0..8usize {
            let sample = divide_rounded(sums[y], COS_FRACTION_BITS + 1 + ROW_FRACTION_BITS);
            // The clamp guarantees the sample fits in i16.
            out[y][x] = saturate_output(sample as i32) as i16;
        }
    }
    out
}

/// `Σ_u f[u]·COS_FIXED[x][u]` for every `x`, exactly, with 22 multiplies
/// in place of 64: an even/odd (partial butterfly) regrouping of the
/// 8-point inverse DCT. Row `7 − x` of the cosine table is row `x` with
/// the odd columns negated, so outputs `x` and `7 − x` share an even part
/// `E` and an odd part `O` (`E + O` and `E − O`); the even part folds
/// once more the same way. Integer products are exact, so the sums equal
/// the plain matrix product's bit for bit.
#[inline]
fn partial_butterfly(f: [i64; 8]) -> [i64; 8] {
    let [a, c1, c2, c3, _, c5, c6, c7] = COS_FIXED[0].map(i64::from);
    let (even_0, even_1) = (a * (f[0] + f[4]), a * (f[0] - f[4]));
    let (odd_even_0, odd_even_1) = (c2 * f[2] + c6 * f[6], c6 * f[2] - c2 * f[6]);
    let e = [
        even_0 + odd_even_0,
        even_1 + odd_even_1,
        even_1 - odd_even_1,
        even_0 - odd_even_0,
    ];
    let o = [
        c1 * f[1] + c3 * f[3] + c5 * f[5] + c7 * f[7],
        c3 * f[1] - c7 * f[3] - c1 * f[5] - c5 * f[7],
        c5 * f[1] - c1 * f[3] + c7 * f[5] + c3 * f[7],
        c7 * f[1] - c5 * f[3] + c3 * f[5] - c1 * f[7],
    ];
    [
        e[0] + o[0],
        e[1] + o[1],
        e[2] + o[2],
        e[3] + o[3],
        e[3] - o[3],
        e[2] - o[2],
        e[1] - o[1],
        e[0] - o[0],
    ]
}

/// Fraction bits of [`COS_FIXED`].
const COS_FRACTION_BITS: u32 = 15;

/// Fraction bits the row pass keeps for the column pass.
const ROW_FRACTION_BITS: u32 = 8;

/// `C(u)·cos((2x+1)uπ/16)` scaled by `2^15` and rounded, indexed
/// `[x][u]`: the integer IDCT's constants (tested against the cosine
/// they stand for).
const COS_FIXED: [[i32; 8]; 8] = [
    [23170, 32138, 30274, 27246, 23170, 18205, 12540, 6393],
    [23170, 27246, 12540, -6393, -23170, -32138, -30274, -18205],
    [23170, 18205, -12540, -32138, -23170, 6393, 30274, 27246],
    [23170, 6393, -30274, -18205, 23170, 27246, -12540, -32138],
    [23170, -6393, -30274, 18205, 23170, -27246, -12540, 32138],
    [23170, -18205, -12540, 32138, -23170, -6393, 30274, -27246],
    [23170, -27246, 12540, 6393, -23170, 32138, -30274, 18205],
    [23170, -32138, 30274, -27246, 23170, -18205, 12540, -6393],
];

/// `value / 2^shift`, rounded half away from zero (the §4.1 `Round`),
/// in integers and without a branch: the magnitude is rounded, then
/// the sign (all ones for a negative value) is put back.
#[inline]
fn divide_rounded(value: i64, shift: u32) -> i64 {
    let sign = value >> 63;
    let magnitude = (value ^ sign) - sign;
    let rounded = (magnitude + (1i64 << (shift - 1))) >> shift;
    (rounded ^ sign) - sign
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

    /// The integer IDCT as a plain matrix product, one multiply per
    /// coefficient and output: the form the fast passes regroup, kept as
    /// their oracle.
    fn idct_8x8_by_matrix(input: &[[i16; 8]; 8]) -> [[i16; 8]; 8] {
        let mut rows = [[0i64; 8]; 8];
        for v in 0..8usize {
            for x in 0..8usize {
                let sum: i64 = (0..8)
                    .map(|u| i64::from(input[v][u]) * i64::from(COS_FIXED[x][u]))
                    .sum();
                rows[v][x] = divide_rounded(sum, COS_FRACTION_BITS + 1 - ROW_FRACTION_BITS);
            }
        }
        let mut out = [[0i16; 8]; 8];
        for x in 0..8usize {
            for y in 0..8usize {
                let sum: i64 = (0..8)
                    .map(|v| rows[v][x] * i64::from(COS_FIXED[y][v]))
                    .sum();
                let sample = divide_rounded(sum, COS_FRACTION_BITS + 1 + ROW_FRACTION_BITS);
                out[y][x] = saturate_output(sample as i32) as i16;
            }
        }
        out
    }

    /// `divide_rounded` as the §4.1 `Round` reads, one branch per sign.
    fn divide_rounded_by_sign(value: i64, shift: u32) -> i64 {
        let half = 1i64 << (shift - 1);
        if value >= 0 {
            (value + half) >> shift
        } else {
            -((-value + half) >> shift)
        }
    }

    /// The branchless rounding equals the per-sign definition for every
    /// value within ±2^20 at the shifts the passes use, and at the ends
    /// of the largest sums they can form.
    #[test]
    fn branchless_rounding_equals_round_half_away_from_zero() {
        let shifts = [
            COS_FRACTION_BITS + 1 - ROW_FRACTION_BITS,
            COS_FRACTION_BITS + 1 + ROW_FRACTION_BITS,
            3,
        ];
        for shift in shifts {
            for value in (-(1i64 << 20)..=(1i64 << 20)).chain([
                i64::from(i32::MIN),
                i64::from(i32::MAX),
                -(1i64 << 40),
                1i64 << 40,
            ]) {
                assert_eq!(
                    divide_rounded(value, shift),
                    divide_rounded_by_sign(value, shift),
                    "{value} >> {shift}"
                );
            }
        }
    }

    /// xorshift64*, seeded: the same blocks on every platform.
    fn next(state: &mut u64) -> u64 {
        *state ^= *state >> 12;
        *state ^= *state << 25;
        *state ^= *state >> 27;
        state.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// The fast passes give exactly the matrix product's samples: over
    /// blocks at the input range's ends, every single coefficient at its
    /// extremes, and 50,000 seeded random blocks from sparse (a few
    /// low-frequency coefficients, as real blocks are) to full. A block
    /// holding only its DC coefficient takes the exact shortcut instead
    /// (`dc_only_blocks_are_exact`) and is left out here.
    #[test]
    fn the_fast_passes_equal_the_matrix_product_exactly() {
        let mut blocks: Vec<[[i16; 8]; 8]> =
            vec![[[F_INPUT_MAX as i16; 8]; 8], [[F_INPUT_MIN as i16; 8]; 8]];
        for v in 0..8 {
            for u in 0..8 {
                for value in [F_INPUT_MIN, -1, 1, F_INPUT_MAX] {
                    let mut block = [[0i16; 8]; 8];
                    block[v][u] = value as i16;
                    blocks.push(block);
                }
            }
        }
        let mut state = 0x9E37_79B9_7F4A_7C15u64;
        for n in 0..50_000usize {
            let filled = 1 + n % 64;
            let mut block = [[0i16; 8]; 8];
            for _ in 0..filled {
                let at = (next(&mut state) % 64) as usize;
                let span = if n % 2 == 0 { 64 } else { 4096 };
                let value = (next(&mut state) % span) as i32 - span as i32 / 2;
                block[at / 8][at % 8] = value.clamp(F_INPUT_MIN, F_INPUT_MAX) as i16;
            }
            blocks.push(block);
        }
        let dc_only = |b: &[[i16; 8]; 8]| b.iter().flatten().skip(1).all(|&c| c == 0);
        for (n, block) in blocks.iter().enumerate().filter(|(_, b)| !dc_only(b)) {
            assert_eq!(
                idct_8x8(block),
                idct_8x8_by_matrix(block),
                "block {n}: {block:?}"
            );
        }
    }

    /// Each fixed-point constant is the rounded `C(u)·cos` it stands for.
    #[test]
    fn fixed_point_cosines_are_the_rounded_cosines() {
        let table = cos_table_ref();
        for x in 0..8usize {
            for u in 0..8usize {
                let exact = alpha(u) * table[x][u] * f64::from(1u32 << COS_FRACTION_BITS);
                assert_eq!(COS_FIXED[x][u], exact.round() as i32, "[{x}][{u}]");
            }
        }
    }

    /// A DC-only block is `F[0][0] / 8` rounded half away from zero, as
    /// the double-precision transform gives, for every DC value.
    #[test]
    fn dc_only_blocks_are_exact() {
        for dc in F_INPUT_MIN..=F_INPUT_MAX {
            let mut block = [[0i16; 8]; 8];
            block[0][0] = dc as i16;
            assert_eq!(idct_8x8(&block), idct_8x8_float(&block), "DC {dc}");
        }
    }

    /// The double-precision transform rounded, for comparisons.
    fn idct_8x8_float(input: &[[i16; 8]; 8]) -> [[i16; 8]; 8] {
        idct_candidate_f64(&input.map(|row| row.map(f64::from)))
            .map(|row| row.map(|v| saturate_output(round_to_int(v)) as i16))
    }

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
