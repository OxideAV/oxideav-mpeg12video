//! MPEG-2 §7.4 inverse quantisation per
//! **ISO/IEC 13818-2 (ITU-T H.262)**.
//!
//! Where the MPEG-1 (`crate::dequantize`) module covers ISO/IEC
//! 11172-2 §2.4.4 — the dead-zone, `Sign(...)`-even-mismatch
//! formulation that ships with MPEG-1 — this module covers the
//! corresponding step for an MPEG-2 stream: §7.4 of ITU-T H.262 /
//! ISO/IEC 13818-2. The two pipelines diverge at this stage, so
//! they live in separate modules; the bitstream syntax is shared
//! upstream (start codes, headers, macroblock VLCs) and the §A.1
//! IDCT is shared downstream.
//!
//! ## Inputs / outputs (Figure 7-3)
//!
//! The module operates on three quantised 8×8 arrays, in order:
//!
//! 1. **`QF[v][u]`** — the inverse-scanned coefficient array
//!    produced by §7.3 from the §7.2 variable-length decoder. This
//!    is the input here; the §7.3 inverse-scan is *not* repeated.
//! 2. **`F''[v][u]`** — the output of *Inverse Quantisation
//!    Arithmetic* (§7.4.1 for the intra DC and §7.4.2.3 for every
//!    other coefficient).
//! 3. **`F'[v][u]`** — the output of *Saturation* (§7.4.3), the
//!    `[-2048, 2047]` clamp that mirrors MPEG-1.
//! 4. **`F[v][u]`** — the output of *Mismatch Control* (§7.4.4),
//!    which forces the parity of `sum(F')` to be odd by optionally
//!    toggling the LSB of `F[7][7]`.
//!
//! ## §7.4.1 intra DC coefficient
//!
//! `F''[0][0]` for an intra block is taken directly from `QF[0][0]`
//! multiplied by an `intra_dc_mult` factor selected from the
//! `intra_dc_precision` field in `picture_coding_extension()`. The
//! mapping (Table 7-4) is `2 ^ (3 - intra_dc_precision)`:
//!
//! ```text
//! intra_dc_precision   bits   intra_dc_mult
//!         0               8         8
//!         1               9         4
//!         2              10         2
//!         3              11         1
//! ```
//!
//! ## §7.4.2.1 weighting matrices
//!
//! Up to four weighting matrices `W[w][v][u]` are addressable per
//! `chroma_format`:
//!
//! | `w` | role                                |
//! |-----|-------------------------------------|
//! |  0  | intra luminance (and 4:2:0 chroma)  |
//! |  1  | non-intra luminance (and 4:2:0)     |
//! |  2  | 4:2:2 / 4:4:4 intra chrominance     |
//! |  3  | 4:2:2 / 4:4:4 non-intra chrominance |
//!
//! Table 7-5 maps the macroblock-level `(macroblock_intra,
//! component, chroma_format)` triple to a `w` index; selection is
//! handled by [`select_weighting_matrix_index`].
//!
//! The two default weighting matrices defined in §6.3.7 are
//! exposed as [`DEFAULT_INTRA_WEIGHT`] (the intra-default matrix
//! shared with MPEG-1's `intra_quant`) and [`DEFAULT_NON_INTRA_WEIGHT`]
//! (all-16, distinct from MPEG-1's `non_intra_quant` which is also
//! all-16 but typed via the MPEG-1 module).
//!
//! ## §7.4.2.2 quantiser_scale
//!
//! The 5-bit `quantiser_scale_code` from the slice header (and from
//! any in-macroblock `quantizer_scale`) is mapped to the integer
//! `quantiser_scale` via Table 7-6, keyed on the picture-coding
//! extension flag `q_scale_type`:
//!
//! * `q_scale_type == 0` — `quantiser_scale = 2 * quantiser_scale_code`
//!   (range `2..=62`, matching MPEG-1).
//! * `q_scale_type == 1` — non-linear table for finer steps at low
//!   codes and larger steps at high codes (range `1..=112`).
//!
//! See [`QUANTISER_SCALE_LINEAR`] and [`QUANTISER_SCALE_NONLINEAR`]
//! for the lookup arrays and [`quantiser_scale`] for the safe
//! accessor.
//!
//! ## §7.4.2.3 reconstruction formula
//!
//! For every coefficient except the intra DC, §7.4.2.3 specifies
//!
//! ```text
//! F''[v][u] = ((2 * QF[v][u] + k) * W[w][v][u] * quantiser_scale) / 32
//! ```
//!
//! with `k = 0` for intra blocks and `k = Sign(QF[v][u])` for
//! non-intra blocks. `/` here is the §4.1 operator —
//! round-toward-zero on integer division — which matches Rust's
//! integer `/` for the signed operands this formula produces.
//!
//! ## §7.4.3 saturation
//!
//! The reconstructed `F''[v][u]` is clamped to `[-2048, 2047]` to
//! produce `F'[v][u]`. Same bounds as MPEG-1, but performed
//! separately here on the post-§7.4.2.3 array.
//!
//! ## §7.4.4 mismatch control
//!
//! After saturation, the spec sums every entry of `F'`. If the sum
//! is even, the LSB of `F'[7][7]` is toggled. Note 1 of §7.4.4
//! confirms this is equivalent to XOR-ing the LSB of `F'[7][7]`
//! with the inverse of the sum's LSB — a much faster realisation
//! when only the parity is needed.
//!
//! ## §7.4.5 summary code
//!
//! [`inverse_quantise_block`] composes §7.4.1 + §7.4.2.3 + §7.4.3 +
//! §7.4.4 into a single call, returning `F[v][u]` from the
//! `QF[v][u]` array plus the weighting matrix, `quantiser_scale`,
//! and the `macroblock_intra` / `intra_dc_mult` configuration.
//!
//! Spec citations refer to **ISO/IEC 13818-2** (ITU-T H.262).

// The §7.4 reference text uses `for (v = 0; v < 8; v++)` /
// `for (u = 0; u < 8; u++)` loops; mirroring those C-style index
// loops here keeps each line of code one-for-one with the spec.
// Switching to `.iter().enumerate()` would obscure that
// correspondence for a one-line ergonomic gain.
#![allow(clippy::needless_range_loop)]
use crate::picture_header::PictureCodingExtension;
use crate::sequence_extension::ChromaFormat;
use crate::{Error, Result};

// =============================================================
// §7.4.1 — intra_dc_mult (Table 7-4)
// =============================================================

/// Map `intra_dc_precision` (2 bits) to `intra_dc_mult` per
/// Table 7-4. Returns `Err` if `intra_dc_precision` is outside
/// `0..=3` (the syntax field is two bits, so callers should never
/// trip this — the check guards against unchecked promotions of
/// other widths).
pub fn intra_dc_mult(intra_dc_precision: u8) -> Result<i32> {
    match intra_dc_precision {
        0 => Ok(8),
        1 => Ok(4),
        2 => Ok(2),
        3 => Ok(1),
        _ => Err(Error::InvalidBitstream(
            "intra_dc_precision: only the 2-bit values 0..=3 are defined (Table 7-4)",
        )),
    }
}

/// Lift [`intra_dc_mult`] for the parsed
/// [`PictureCodingExtension`].
pub fn intra_dc_mult_from_extension(ext: &PictureCodingExtension) -> Result<i32> {
    intra_dc_mult(ext.intra_dc_precision)
}

// =============================================================
// §7.4.2.1 — weighting matrices (Table 7-5)
// =============================================================

/// Default intra weighting matrix from §6.3.7 (identical layout to
/// the MPEG-1 default intra quantiser; row-major).
pub const DEFAULT_INTRA_WEIGHT: [[u8; 8]; 8] = [
    [8, 16, 19, 22, 26, 27, 29, 34],
    [16, 16, 22, 24, 27, 29, 34, 37],
    [19, 22, 26, 27, 29, 34, 34, 38],
    [22, 22, 26, 27, 29, 34, 37, 40],
    [22, 26, 27, 29, 32, 35, 40, 48],
    [26, 27, 29, 32, 35, 40, 48, 58],
    [26, 27, 29, 34, 38, 46, 56, 69],
    [27, 29, 35, 38, 46, 56, 69, 83],
];

/// Default non-intra weighting matrix from §6.3.7 — every entry is
/// 16.
pub const DEFAULT_NON_INTRA_WEIGHT: [[u8; 8]; 8] = [[16u8; 8]; 8];

/// Block-coding kind for [`select_weighting_matrix_index`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockCoding {
    /// `macroblock_intra == 1` block.
    Intra,
    /// `macroblock_intra == 0` block.
    NonIntra,
}

/// Colour component for [`select_weighting_matrix_index`].
///
/// Mirrors the §7.2 `cc` index — `Luminance` is `cc == 0` and
/// `Chrominance` is `cc != 0`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Component {
    /// Y (cc == 0).
    Luminance,
    /// Cb / Cr (cc != 0).
    Chrominance,
}

/// Map the macroblock-level `(coding, component, chroma_format)`
/// triple to the §7.4.2.1 weighting-matrix index `w`. Encodes
/// Table 7-5.
pub fn select_weighting_matrix_index(
    coding: BlockCoding,
    component: Component,
    chroma_format: ChromaFormat,
) -> u8 {
    match (coding, component, chroma_format) {
        // 4:2:0 — chroma shares the luma matrix.
        (BlockCoding::Intra, _, ChromaFormat::Yuv420) => 0,
        (BlockCoding::NonIntra, _, ChromaFormat::Yuv420) => 1,
        // 4:2:2 / 4:4:4 — luma keeps w=0/1, chroma uses w=2/3.
        (BlockCoding::Intra, Component::Luminance, _) => 0,
        (BlockCoding::NonIntra, Component::Luminance, _) => 1,
        (BlockCoding::Intra, Component::Chrominance, _) => 2,
        (BlockCoding::NonIntra, Component::Chrominance, _) => 3,
    }
}

// =============================================================
// §7.4.2.2 — quantiser_scale (Table 7-6)
// =============================================================

/// `quantiser_scale[0][code]` for `q_scale_type == 0` (linear):
/// `2 * code`. `code == 0` is forbidden per Table 7-6 and is
/// surfaced as a slot of `0` here, which [`quantiser_scale`]
/// rejects.
pub const QUANTISER_SCALE_LINEAR: [u8; 32] = [
    0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48,
    50, 52, 54, 56, 58, 60, 62,
];

/// `quantiser_scale[1][code]` for `q_scale_type == 1` (non-linear).
/// `code == 0` is forbidden per Table 7-6 and is surfaced as a slot
/// of `0` here, which [`quantiser_scale`] rejects.
pub const QUANTISER_SCALE_NONLINEAR: [u8; 32] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 18, 20, 22, 24, 28, 32, 36, 40, 44, 48, 52, 56, 64,
    72, 80, 88, 96, 104, 112,
];

/// Look up `quantiser_scale` from a 5-bit `quantiser_scale_code`
/// and the `q_scale_type` flag (`false` for linear, `true` for the
/// non-linear table).
///
/// Returns `Err` if the code is `0` (Table 7-6 "forbidden" entry) or
/// exceeds the 5-bit range.
pub fn quantiser_scale(quantiser_scale_code: u8, q_scale_type: bool) -> Result<u8> {
    if quantiser_scale_code == 0 {
        return Err(Error::InvalidBitstream(
            "quantiser_scale_code: code 0 is forbidden (Table 7-6)",
        ));
    }
    if quantiser_scale_code > 31 {
        return Err(Error::InvalidBitstream(
            "quantiser_scale_code: code exceeds the 5-bit range (Table 7-6)",
        ));
    }
    let table = if q_scale_type {
        &QUANTISER_SCALE_NONLINEAR
    } else {
        &QUANTISER_SCALE_LINEAR
    };
    Ok(table[quantiser_scale_code as usize])
}

// =============================================================
// §7.4.2.3 / §7.4.3 / §7.4.4 — the arithmetic pipeline
// =============================================================

/// Saturation bounds per §7.4.3 (identical to MPEG-1 §2.4.4).
pub const F_SATURATION_MIN: i32 = -2048;
/// Upper saturation bound per §7.4.3.
pub const F_SATURATION_MAX: i32 = 2047;

/// `Sign(x)` per §4.1 — `-1` for negative, `0` for zero, `+1` for
/// positive. Used both by §7.4.2.3 (`k = Sign(QF[v][u])`) and by
/// callers that want to reproduce the table.
pub fn sign(x: i32) -> i32 {
    match x.cmp(&0) {
        core::cmp::Ordering::Less => -1,
        core::cmp::Ordering::Equal => 0,
        core::cmp::Ordering::Greater => 1,
    }
}

/// `Saturate(x)` per §7.4.3 — clamp to `[-2048, 2047]`.
pub fn saturate(x: i32) -> i32 {
    x.clamp(F_SATURATION_MIN, F_SATURATION_MAX)
}

/// Run the §7.4 inverse-quantisation pipeline on a single 8×8 block
/// and return the post-mismatch-control `F[v][u]`.
///
/// Inputs:
///
/// * `qf` — the `QF[v][u]` array out of §7.3 inverse-scan (intra DC
///   is at `qf[0][0]`).
/// * `coding` — `Intra` for `macroblock_intra == 1` blocks (§7.4.1
///   applies to `[0][0]`), `NonIntra` otherwise.
/// * `weight` — the 8×8 weighting matrix `W[w][v][u]` selected by
///   §7.4.2.1 ([`select_weighting_matrix_index`] returns `w`).
/// * `quantiser_scale_value` — the integer `quantiser_scale`
///   returned by [`quantiser_scale`] (post-Table-7-6 lookup).
/// * `intra_dc_mult_value` — the §7.4.1 multiplier from
///   [`intra_dc_mult`]. Only consulted for `Intra` blocks; pass any
///   value for `NonIntra`.
///
/// Returns the 8×8 `F[v][u]` after §7.4.3 saturation and §7.4.4
/// mismatch control.
pub fn inverse_quantise_block(
    qf: &[[i32; 8]; 8],
    coding: BlockCoding,
    weight: &[[u8; 8]; 8],
    quantiser_scale_value: u8,
    intra_dc_mult_value: i32,
) -> [[i32; 8]; 8] {
    saturate_and_mismatch(&inverse_quantise_arithmetic(
        qf,
        coding,
        weight,
        quantiser_scale_value,
        intra_dc_mult_value,
    ))
}

/// The §7.4.1 / §7.4.2.3 **inverse quantisation arithmetic** alone —
/// `F''[v][u]` *before* the §7.4.3 saturation and §7.4.4 mismatch
/// control. This is the point at which §7.8.3.4 SNR scalability adds
/// the two layers' coefficients (`F'' = F''lower + F''enhance`) before
/// the remaining steps run once on the sum.
pub fn inverse_quantise_arithmetic(
    qf: &[[i32; 8]; 8],
    coding: BlockCoding,
    weight: &[[u8; 8]; 8],
    quantiser_scale_value: u8,
    intra_dc_mult_value: i32,
) -> [[i32; 8]; 8] {
    // ----- §7.4.2.3 inverse quantisation arithmetic -----
    let mut f_double_prime = [[0i32; 8]; 8];
    let qs = i32::from(quantiser_scale_value);
    for v in 0..8 {
        for u in 0..8 {
            f_double_prime[v][u] = if v == 0 && u == 0 && coding == BlockCoding::Intra {
                // §7.4.1: F''[0][0] = intra_dc_mult * QF[0][0]
                intra_dc_mult_value * qf[0][0]
            } else {
                let k = match coding {
                    BlockCoding::Intra => 0,
                    BlockCoding::NonIntra => sign(qf[v][u]),
                };
                let w = i32::from(weight[v][u]);
                // (2 * QF + k) * W * quantiser_scale / 32, with `/`
                // being §4.1 round-toward-zero (Rust's `/` on i32).
                ((2 * qf[v][u] + k) * w * qs) / 32
            };
        }
    }
    f_double_prime
}

/// §7.4.3 saturation followed by §7.4.4 mismatch control: `F''` → `F`.
pub fn saturate_and_mismatch(f_double_prime: &[[i32; 8]; 8]) -> [[i32; 8]; 8] {
    // ----- §7.4.3 saturation -----
    let mut f_prime = [[0i32; 8]; 8];
    for v in 0..8 {
        for u in 0..8 {
            f_prime[v][u] = saturate(f_double_prime[v][u]);
        }
    }

    // ----- §7.4.4 mismatch control -----
    // Per §7.4.4 Note 1, the parity check is equivalent to XOR-ing
    // LSBs across the block. We use the literal `sum` form for
    // readability; the result is identical.
    let mut sum: i32 = 0;
    for v in 0..8 {
        for u in 0..8 {
            sum = sum.wrapping_add(f_prime[v][u]);
        }
    }
    let mut f = f_prime;
    if sum & 1 == 0 {
        // Sum is even — toggle LSB of F[7][7].
        if f[7][7] & 1 != 0 {
            f[7][7] -= 1;
        } else {
            f[7][7] += 1;
        }
    }
    f
}

#[cfg(test)]
mod tests {
    use super::*;

    // ------- §7.4.1 -------

    #[test]
    fn intra_dc_mult_table_7_4() {
        assert_eq!(intra_dc_mult(0).unwrap(), 8);
        assert_eq!(intra_dc_mult(1).unwrap(), 4);
        assert_eq!(intra_dc_mult(2).unwrap(), 2);
        assert_eq!(intra_dc_mult(3).unwrap(), 1);
    }

    #[test]
    fn intra_dc_mult_rejects_out_of_range() {
        // Defence in depth: the syntax field is two bits, but the
        // function still rejects any unexpected value.
        assert!(intra_dc_mult(4).is_err());
        assert!(intra_dc_mult(255).is_err());
    }

    // ------- §7.4.2.1 -------

    #[test]
    fn weighting_matrix_index_yuv420() {
        // 4:2:0 collapses chroma into the luma slot.
        assert_eq!(
            select_weighting_matrix_index(
                BlockCoding::Intra,
                Component::Luminance,
                ChromaFormat::Yuv420
            ),
            0
        );
        assert_eq!(
            select_weighting_matrix_index(
                BlockCoding::Intra,
                Component::Chrominance,
                ChromaFormat::Yuv420
            ),
            0
        );
        assert_eq!(
            select_weighting_matrix_index(
                BlockCoding::NonIntra,
                Component::Luminance,
                ChromaFormat::Yuv420
            ),
            1
        );
        assert_eq!(
            select_weighting_matrix_index(
                BlockCoding::NonIntra,
                Component::Chrominance,
                ChromaFormat::Yuv420
            ),
            1
        );
    }

    #[test]
    fn weighting_matrix_index_yuv422_yuv444() {
        for fmt in [ChromaFormat::Yuv422, ChromaFormat::Yuv444] {
            assert_eq!(
                select_weighting_matrix_index(BlockCoding::Intra, Component::Luminance, fmt),
                0
            );
            assert_eq!(
                select_weighting_matrix_index(BlockCoding::NonIntra, Component::Luminance, fmt),
                1
            );
            assert_eq!(
                select_weighting_matrix_index(BlockCoding::Intra, Component::Chrominance, fmt),
                2
            );
            assert_eq!(
                select_weighting_matrix_index(BlockCoding::NonIntra, Component::Chrominance, fmt),
                3
            );
        }
    }

    #[test]
    fn default_non_intra_weight_is_all_16() {
        for v in 0..8 {
            for u in 0..8 {
                assert_eq!(DEFAULT_NON_INTRA_WEIGHT[v][u], 16);
            }
        }
    }

    // ------- §7.4.2.2 -------

    #[test]
    fn quantiser_scale_linear_doubles_code() {
        for code in 1u8..=31 {
            assert_eq!(quantiser_scale(code, false).unwrap(), 2 * code);
        }
    }

    #[test]
    fn quantiser_scale_nonlinear_spec_table() {
        // Spot-check the Table 7-6 non-linear column at the
        // distinctive bend points: code 8 -> 8 (same as 2*4 by
        // accident), code 9 -> 10, code 16 -> 24, code 25 -> 64,
        // code 31 -> 112.
        assert_eq!(quantiser_scale(1, true).unwrap(), 1);
        assert_eq!(quantiser_scale(8, true).unwrap(), 8);
        assert_eq!(quantiser_scale(9, true).unwrap(), 10);
        assert_eq!(quantiser_scale(16, true).unwrap(), 24);
        assert_eq!(quantiser_scale(25, true).unwrap(), 64);
        assert_eq!(quantiser_scale(31, true).unwrap(), 112);
    }

    #[test]
    fn quantiser_scale_full_nonlinear_column_matches_table() {
        // Cross-check every entry against the Table 7-6 column.
        let expected_nonlinear = [
            // index 0 is forbidden — skipped.
            (1, 1),
            (2, 2),
            (3, 3),
            (4, 4),
            (5, 5),
            (6, 6),
            (7, 7),
            (8, 8),
            (9, 10),
            (10, 12),
            (11, 14),
            (12, 16),
            (13, 18),
            (14, 20),
            (15, 22),
            (16, 24),
            (17, 28),
            (18, 32),
            (19, 36),
            (20, 40),
            (21, 44),
            (22, 48),
            (23, 52),
            (24, 56),
            (25, 64),
            (26, 72),
            (27, 80),
            (28, 88),
            (29, 96),
            (30, 104),
            (31, 112),
        ];
        for (code, expected) in expected_nonlinear {
            assert_eq!(quantiser_scale(code, true).unwrap(), expected);
        }
    }

    #[test]
    fn quantiser_scale_rejects_forbidden_zero() {
        assert!(quantiser_scale(0, false).is_err());
        assert!(quantiser_scale(0, true).is_err());
    }

    #[test]
    fn quantiser_scale_rejects_out_of_range() {
        assert!(quantiser_scale(32, false).is_err());
        assert!(quantiser_scale(255, true).is_err());
    }

    // ------- §7.4.3 / §7.4.4 helpers -------

    #[test]
    fn sign_matches_section_4_1() {
        assert_eq!(sign(-7), -1);
        assert_eq!(sign(-1), -1);
        assert_eq!(sign(0), 0);
        assert_eq!(sign(1), 1);
        assert_eq!(sign(2047), 1);
    }

    #[test]
    fn saturate_clamps_to_12_bit_band() {
        assert_eq!(saturate(2048), 2047);
        assert_eq!(saturate(2047), 2047);
        assert_eq!(saturate(0), 0);
        assert_eq!(saturate(-2048), -2048);
        assert_eq!(saturate(-2049), -2048);
        assert_eq!(saturate(i32::MAX), 2047);
        assert_eq!(saturate(i32::MIN), -2048);
    }

    // ------- §7.4.5 end-to-end -------

    #[test]
    fn all_zero_qf_intra_produces_only_dc_then_mismatch_flip() {
        // QF is all zero except QF[0][0] = 1, intra block, default
        // intra weight, intra_dc_precision = 0 (intra_dc_mult = 8).
        // §7.4.1 yields F''[0][0] = 8 * 1 = 8.
        // Every other F'' is 0. Saturation: F'[0][0] = 8.
        // sum(F') = 8 — even — so the §7.4.4 mismatch toggles
        // F[7][7] from 0 to 1.
        let mut qf = [[0i32; 8]; 8];
        qf[0][0] = 1;
        let f = inverse_quantise_block(
            &qf,
            BlockCoding::Intra,
            &DEFAULT_INTRA_WEIGHT,
            // quantiser_scale is irrelevant for the DC-only intra
            // path (the rest is k=0 and QF=0) but we still pass a
            // valid one.
            2,
            8,
        );
        assert_eq!(f[0][0], 8);
        assert_eq!(f[7][7], 1);
        // Every other slot stays zero.
        for v in 0..8 {
            for u in 0..8 {
                if (v, u) != (0, 0) && (v, u) != (7, 7) {
                    assert_eq!(f[v][u], 0, "expected zero at [{}][{}]", v, u);
                }
            }
        }
    }

    #[test]
    fn all_zero_qf_intra_dc_odd_sum_skips_mismatch_flip() {
        // QF[0][0] = 7, intra_dc_mult = 1 (intra_dc_precision = 3).
        // F''[0][0] = 7. Saturation: 7. sum(F') = 7 — odd — so
        // §7.4.4 leaves F[7][7] alone (= 0).
        let mut qf = [[0i32; 8]; 8];
        qf[0][0] = 7;
        let f = inverse_quantise_block(&qf, BlockCoding::Intra, &DEFAULT_INTRA_WEIGHT, 2, 1);
        assert_eq!(f[0][0], 7);
        assert_eq!(f[7][7], 0);
    }

    #[test]
    fn intra_ac_uses_w_quantiser_scale_over_32() {
        // QF[1][1] = 4, intra, intra_dc_mult = 8 (precision 0),
        // weight W[1][1] = 16 (default), quantiser_scale = 8.
        // §7.4.2.3 (intra k=0): F'' = (2*4 + 0) * 16 * 8 / 32
        //                            = 8 * 16 * 8 / 32 = 32.
        let mut qf = [[0i32; 8]; 8];
        qf[1][1] = 4;
        let f = inverse_quantise_block(&qf, BlockCoding::Intra, &DEFAULT_INTRA_WEIGHT, 8, 8);
        assert_eq!(f[1][1], 32);
        assert_eq!(f[0][0], 0, "intra DC should remain zero for QF[0][0] = 0");
        // sum = 32 — even — so F[7][7] flips from 0 to 1.
        assert_eq!(f[7][7], 1);
    }

    #[test]
    fn non_intra_uses_signed_k_offset() {
        // QF[2][3] = 5, non-intra, W = 16 (default non-intra
        // weight), quantiser_scale = 8.
        // §7.4.2.3 (non-intra k=Sign(QF)=+1):
        // F'' = (2*5 + 1) * 16 * 8 / 32 = 11 * 128 / 32 = 44.
        let mut qf = [[0i32; 8]; 8];
        qf[2][3] = 5;
        let f = inverse_quantise_block(
            &qf,
            BlockCoding::NonIntra,
            &DEFAULT_NON_INTRA_WEIGHT,
            8,
            // intra_dc_mult is irrelevant for non-intra.
            42,
        );
        assert_eq!(f[2][3], 44);
        // sum = 44 — even — so F[7][7] flips from 0 to 1.
        assert_eq!(f[7][7], 1);
    }

    #[test]
    fn non_intra_negative_qf_uses_negative_k() {
        // QF[3][4] = -5, non-intra, W = 16, qs = 8.
        // F'' = (2*-5 + Sign(-5)) * 16 * 8 / 32 = -11 * 128 / 32
        //     = -44.
        let mut qf = [[0i32; 8]; 8];
        qf[3][4] = -5;
        let f =
            inverse_quantise_block(&qf, BlockCoding::NonIntra, &DEFAULT_NON_INTRA_WEIGHT, 8, 42);
        assert_eq!(f[3][4], -44);
        // sum = -44 — even — so F[7][7] flips from 0 to 1.
        assert_eq!(f[7][7], 1);
    }

    #[test]
    fn saturation_clamps_extreme_qf_to_2047() {
        // QF[0][1] = i16::MAX, non-intra, big quantiser_scale.
        // Pre-saturation F'' is far above 2047; §7.4.3 should
        // clamp it.
        let mut qf = [[0i32; 8]; 8];
        qf[0][1] = 32_000;
        let f = inverse_quantise_block(
            &qf,
            BlockCoding::NonIntra,
            &DEFAULT_NON_INTRA_WEIGHT,
            62,
            42,
        );
        assert_eq!(f[0][1], F_SATURATION_MAX);
    }

    #[test]
    fn saturation_clamps_extreme_negative_qf_to_minus_2048() {
        let mut qf = [[0i32; 8]; 8];
        qf[5][5] = -32_000;
        let f = inverse_quantise_block(
            &qf,
            BlockCoding::NonIntra,
            &DEFAULT_NON_INTRA_WEIGHT,
            62,
            42,
        );
        assert_eq!(f[5][5], F_SATURATION_MIN);
    }

    #[test]
    fn mismatch_control_only_touches_f_7_7() {
        // Pick a QF that yields a non-trivial sum but doesn't
        // overwrite F[7][7] via the arithmetic.
        let mut qf = [[0i32; 8]; 8];
        qf[0][0] = 1; // intra DC -> F''[0][0] = 8.
        qf[0][1] = 1; // F'' = (2 + 0) * 16 * 2 / 32 = 2 (using intra k=0, qs=2).
        let f = inverse_quantise_block(
            &qf,
            BlockCoding::Intra,
            &DEFAULT_NON_INTRA_WEIGHT, // all-16 weight makes the
            // arithmetic easy to mentally
            // verify.
            2,
            8,
        );
        // Pre-mismatch sum: 8 + 2 = 10 — even — flip F[7][7]:
        // 0 -> 1.
        assert_eq!(f[0][0], 8);
        assert_eq!(f[0][1], 2);
        assert_eq!(f[7][7], 1);
    }

    #[test]
    fn mismatch_control_no_op_when_sum_is_odd() {
        // Force an odd sum: intra DC of 9 plus nothing else.
        let mut qf = [[0i32; 8]; 8];
        qf[0][0] = 9;
        let f = inverse_quantise_block(&qf, BlockCoding::Intra, &DEFAULT_INTRA_WEIGHT, 2, 1);
        // F''[0][0] = 1 * 9 = 9; sum is 9 — odd — F[7][7] stays 0.
        assert_eq!(f[0][0], 9);
        assert_eq!(f[7][7], 0);
    }
}
