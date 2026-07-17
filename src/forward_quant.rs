//! Forward quantisation — the encoder inverse of the §7.4 MPEG-2
//! inverse-quantisation arithmetic in [`crate::mpeg2_dequantize`].
//!
//! ## Spec basis
//!
//! ISO/IEC 13818-2 §7.4 specifies the *decoder's* inverse quantiser.
//! The encoder's forward quantiser is the arithmetic that, when fed
//! back through that decoder pipeline, reconstructs coefficients as
//! close as possible to the forward-DCT output. The standard does not
//! mandate a particular forward quantiser (it is an encoder choice);
//! what it *does* fix is the inverse arithmetic this module inverts, so
//! that the quantised levels `QF[v][u]` this module emits decode back
//! through [`crate::mpeg2_dequantize::inverse_quantise_block`] to the
//! intended `F[v][u]`.
//!
//! ## The arithmetic (inverting §7.4.2.3)
//!
//! The §7.4.2.3 reconstruction (for the non-DC, non-intra and intra-AC
//! coefficients) is
//!
//! ```text
//! F''[v][u] = ((2*QF[v][u] + k) * W[v][u] * quantiser_scale) / 32
//! ```
//!
//! with `k = 0` for intra blocks and `k = Sign(QF)` for non-intra. The
//! forward quantiser solves for `QF` given a real-valued DCT
//! coefficient `F`:
//!
//! * **Intra AC** (`k = 0`): there is no dead-zone, so we round to the
//!   nearest level —
//!   `QF = Round( F * 16 / (W * quantiser_scale) )`.
//!   (The factor 16 = 32/2 folds in the `2*QF` doubling and the `/32`.)
//! * **Non-intra** (`k = Sign(QF)`): the inverse adds a half-step in
//!   the magnitude direction, so the matched forward quantiser
//!   truncates toward zero (a dead-zone of one level around zero) —
//!   `QF = Trunc( F * 16 / (W * quantiser_scale) )` toward zero.
//! * **Intra DC** (§7.4.1, `F''[0][0] = intra_dc_mult * QF[0][0]`):
//!   `QF[0][0] = Round( F[0][0] / intra_dc_mult )`.
//!
//! The quantised levels are clamped to the run-level VLC's codable
//! range so the entropy coder always has a representable symbol.

#![allow(clippy::needless_range_loop)]

use crate::mpeg2_dequantize::BlockCoding;

/// Maximum absolute quantised level the Annex B Table B-14 / B-15
/// run-level VLCs plus the Table B-16 escape can code: the escape
/// `signed_level` is a 12-bit signed value in `[-2047, +2047] \ {0}`
/// (§7.2.2.3). Forward-quantised levels are clamped to this magnitude
/// so the entropy coder always has a representable symbol.
pub const MAX_CODED_LEVEL: i32 = 2047;

/// Round a rational `num / den` to the nearest integer, ties away from
/// zero — the §4.1 `Round` convention. `den` must be positive.
#[inline]
fn round_div(num: i64, den: i64) -> i32 {
    debug_assert!(den > 0);
    let half = den / 2;
    let q = if num >= 0 {
        (num + half) / den
    } else {
        (num - half) / den
    };
    q as i32
}

/// Truncate a rational `num / den` toward zero — the dead-zone the
/// non-intra inverse quantiser's `+ Sign(QF)` term matches. `den` must
/// be positive.
#[inline]
fn trunc_div(num: i64, den: i64) -> i32 {
    debug_assert!(den > 0);
    (num / den) as i32
}

/// Forward-quantise a single intra DC coefficient (§7.4.1 inverse).
///
/// `f_dc` is the real DCT DC coefficient `F[0][0]` (already produced by
/// the forward DCT). `intra_dc_mult` is the §7.4.1 multiplier from
/// [`crate::mpeg2_dequantize::intra_dc_mult`] (8 / 4 / 2 / 1 for
/// `intra_dc_precision` 0..=3). Returns the quantised level
/// `QF[0][0]`.
pub fn forward_quantise_intra_dc(f_dc: i32, intra_dc_mult: i32) -> i32 {
    debug_assert!(intra_dc_mult > 0);
    round_div(i64::from(f_dc), i64::from(intra_dc_mult))
}

/// Forward-quantise one 8×8 DCT coefficient block into the §7.3
/// inverse-scan-ordered `QF[v][u]` levels.
///
/// `f` is the real-valued forward-DCT block `F[v][u]` (an `i32` block;
/// the encoder rounds the `f64` DCT once before this call). `coding`
/// selects the intra (round-to-nearest AC, DC via `intra_dc_mult`) or
/// non-intra (dead-zone toward zero) rule. `weight` is the §7.4.2.1
/// weighting matrix `W[v][u]`. `quantiser_scale` is the integer
/// quantiser scale from
/// [`crate::mpeg2_dequantize::quantiser_scale`]. `intra_dc_mult` is
/// only consulted for intra blocks (pass any value otherwise).
///
/// Returns the quantised level block `QF[v][u]`, each entry clamped to
/// `[-MAX_CODED_LEVEL, MAX_CODED_LEVEL]`.
pub fn forward_quantise_block(
    f: &[[i32; 8]; 8],
    coding: BlockCoding,
    weight: &[[u8; 8]; 8],
    quantiser_scale: u8,
    intra_dc_mult: i32,
) -> [[i32; 8]; 8] {
    let qs = i64::from(quantiser_scale);
    let mut qf = [[0i32; 8]; 8];
    for v in 0..8 {
        for u in 0..8 {
            let level = if v == 0 && u == 0 && coding == BlockCoding::Intra {
                forward_quantise_intra_dc(f[0][0], intra_dc_mult)
            } else {
                let w = i64::from(weight[v][u]);
                debug_assert!(w > 0 && qs > 0);
                // Inverse: F'' = (2*QF*W*qs)/32  →  QF = F*16/(W*qs).
                let num = i64::from(f[v][u]) * 16;
                let den = w * qs;
                match coding {
                    // Intra AC: no dead-zone, round to nearest.
                    BlockCoding::Intra => round_div(num, den),
                    // Non-intra: dead-zone, truncate toward zero.
                    BlockCoding::NonIntra => trunc_div(num, den),
                }
            };
            qf[v][u] = level.clamp(-MAX_CODED_LEVEL, MAX_CODED_LEVEL);
        }
    }
    qf
}

// =============================================================
// ISO/IEC 11172-2 (MPEG-1) forward quantisers — §2.4.4.1 / §2.4.4.2
// =============================================================

/// Maximum absolute quantised level the MPEG-1 run-level entropy
/// layer can code: Tables B.5c..B.5e cap at 40 and the Table B.5f
/// escape extends to 255 (`-256` is forbidden). MPEG-1 forward-
/// quantised levels are clamped to this magnitude.
pub const MPEG1_MAX_CODED_LEVEL: i32 = 255;

/// Forward-quantise one 8×8 DCT block into the §2.4.3.7 zig-zag
/// `dct_zz[]` array for an MPEG-1 **intra** block, inverting the
/// §2.4.4.1 AC reconstruction
/// `dct_recon = (2 * dct_zz * quantizer_scale * W) / 16`:
/// `dct_zz = Round(8 * F / (quantizer_scale * W))` (round to nearest,
/// no dead-zone — the decoder-side odd-value mismatch rule perturbs
/// the reconstruction by at most one).
///
/// Only the 63 AC entries are produced; `dct_zz[0]` is left `0` — the
/// §2.4.3.7 DC differential depends on the `dct_dc_*_past` predictor
/// chain, which the caller owns (see
/// [`crate::block_dc::encode_dc_coefficient`]).
///
/// `quantizer_scale` is the §2.4.3.6 value `1..=31` used **directly**
/// (MPEG-1 has no §7.4.2.2 non-linear mapping); `intra_quant` is the
/// raster-order `intra_quant[m][n]` matrix.
pub fn mpeg1_forward_quantise_intra_ac(
    f: &[[i32; 8]; 8],
    quantizer_scale: u8,
    intra_quant: &[[u8; 8]; 8],
) -> [i32; 64] {
    let qs = i64::from(quantizer_scale);
    let mut dct_zz = [0i32; 64];
    for m in 0..8 {
        for n in 0..8 {
            if (m, n) == (0, 0) {
                continue;
            }
            let w = i64::from(intra_quant[m][n]);
            debug_assert!(w > 0 && qs > 0);
            // Inverse: recon = (2*z*qs*W)/16  →  z = F*8/(qs*W).
            let level = round_div(i64::from(f[m][n]) * 8, qs * w);
            dct_zz[crate::block_dc::SCAN[m][n] as usize] =
                level.clamp(-MPEG1_MAX_CODED_LEVEL, MPEG1_MAX_CODED_LEVEL);
        }
    }
    dct_zz
}

/// Forward-quantise one 8×8 DCT residual block into the §2.4.3.7
/// zig-zag `dct_zz[]` array for an MPEG-1 **non-intra** block,
/// inverting the §2.4.4.2 reconstruction
/// `dct_recon = ((2 * dct_zz + Sign(dct_zz)) * quantizer_scale * W) / 16`:
/// the `+ Sign` term restores a half-step of magnitude, so the matched
/// forward quantiser truncates toward zero (a dead-zone around zero) —
/// `dct_zz = Trunc(8 * F / (quantizer_scale * W))`.
///
/// All 64 entries are produced (non-intra blocks have no DC prelude).
pub fn mpeg1_forward_quantise_non_intra(
    f: &[[i32; 8]; 8],
    quantizer_scale: u8,
    non_intra_quant: &[[u8; 8]; 8],
) -> [i32; 64] {
    let qs = i64::from(quantizer_scale);
    let mut dct_zz = [0i32; 64];
    for m in 0..8 {
        for n in 0..8 {
            let w = i64::from(non_intra_quant[m][n]);
            debug_assert!(w > 0 && qs > 0);
            let level = trunc_div(i64::from(f[m][n]) * 8, qs * w);
            dct_zz[crate::block_dc::SCAN[m][n] as usize] =
                level.clamp(-MPEG1_MAX_CODED_LEVEL, MPEG1_MAX_CODED_LEVEL);
        }
    }
    dct_zz
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mpeg2_dequantize::{
        inverse_quantise_block, DEFAULT_INTRA_WEIGHT, DEFAULT_NON_INTRA_WEIGHT,
    };

    #[test]
    fn round_div_ties_away_from_zero() {
        assert_eq!(round_div(5, 2), 3); // 2.5 -> 3
        assert_eq!(round_div(-5, 2), -3); // -2.5 -> -3
        assert_eq!(round_div(7, 2), 4); // 3.5 -> 4
        assert_eq!(round_div(3, 2), 2); // 1.5 -> 2
        assert_eq!(round_div(2, 2), 1);
        assert_eq!(round_div(1, 3), 0);
    }

    #[test]
    fn trunc_div_toward_zero() {
        assert_eq!(trunc_div(5, 2), 2);
        assert_eq!(trunc_div(-5, 2), -2);
        assert_eq!(trunc_div(31, 32), 0);
        assert_eq!(trunc_div(-31, 32), 0);
    }

    #[test]
    fn intra_dc_roundtrips_through_inverse() {
        // intra_dc_precision = 0 → mult = 8. F[0][0] = 1000 quantises to
        // 125; inverse 8*125 = 1000 exactly.
        let qdc = forward_quantise_intra_dc(1000, 8);
        assert_eq!(qdc, 125);
        // 1004 → 125 or 126? 1004/8 = 125.5 → 126; inverse 1008.
        assert_eq!(forward_quantise_intra_dc(1004, 8), 126);
    }

    #[test]
    fn intra_ac_quantise_then_dequantise_is_near_identity() {
        // Build an arbitrary intra DCT block, forward-quantise with the
        // default intra weights at quantiser_scale 8, then inverse-
        // quantise and confirm reconstruction tracks the input.
        let mut f = [[0i32; 8]; 8];
        for v in 0..8 {
            for u in 0..8 {
                f[v][u] = (((v * 8 + u) as i32 * 17) % 600) - 300;
            }
        }
        // intra_dc_precision 0 → mult 8, quantiser_scale 8.
        let qf = forward_quantise_block(&f, BlockCoding::Intra, &DEFAULT_INTRA_WEIGHT, 8, 8);
        let recon = inverse_quantise_block(&qf, BlockCoding::Intra, &DEFAULT_INTRA_WEIGHT, 8, 8);
        // For each coefficient, the reconstruction error must be bounded
        // by roughly one quantiser step (W*qs/16) plus mismatch-control
        // (only F[7][7]).
        for v in 0..8 {
            for u in 0..8 {
                if (u, v) == (7, 7) {
                    continue; // mismatch control may toggle the LSB
                }
                let step = i32::from(DEFAULT_INTRA_WEIGHT[v][u]) * 8 / 16;
                let err = (recon[v][u] - f[v][u]).abs();
                assert!(
                    err <= step.max(1),
                    "intra ({u},{v}) err {err} > step {step}"
                );
            }
        }
    }

    #[test]
    fn nonintra_quantise_has_deadzone_at_zero() {
        // A small non-intra coefficient inside the dead-zone quantises
        // to 0. With all-16 weights and quantiser_scale 8, the step is
        // 16*8/16 = 8; |F| < 8 -> QF 0.
        let mut f = [[0i32; 8]; 8];
        f[1][1] = 7; // inside the dead-zone
        f[2][2] = 9; // just outside
        let qf = forward_quantise_block(&f, BlockCoding::NonIntra, &DEFAULT_NON_INTRA_WEIGHT, 8, 8);
        assert_eq!(qf[1][1], 0, "small coeff inside dead-zone -> 0");
        assert_eq!(qf[2][2], 1, "coeff just outside -> level 1");
    }

    #[test]
    fn levels_are_clamped_to_codable_range() {
        // A huge DC coefficient with mult 1 would exceed the VLC range;
        // it must be clamped to MAX_CODED_LEVEL.
        let mut f = [[0i32; 8]; 8];
        f[0][0] = 100_000;
        let qf = forward_quantise_block(&f, BlockCoding::Intra, &DEFAULT_INTRA_WEIGHT, 8, 1);
        assert_eq!(qf[0][0], MAX_CODED_LEVEL);
    }

    // ---- MPEG-1 (§2.4.4.1 / §2.4.4.2) ------------------------------

    #[test]
    fn mpeg1_intra_ac_inverts_the_dequantiser_within_one_step() {
        use crate::dequantize::{
            dequantize_intra_block, IntraBlockKind, IntraDcPredictors, DEFAULT_INTRA_QUANT,
        };
        // Structured AC content; DC handled by the predictor chain so
        // dct_zz[0] stays 0 here.
        let mut f = [[0i32; 8]; 8];
        for m in 0..8 {
            for n in 0..8 {
                f[m][n] = (((m * 8 + n) as i32 * 23) % 700) - 350;
            }
        }
        f[0][0] = 0;
        for qs in [1u8, 4, 8, 31] {
            let dct_zz = mpeg1_forward_quantise_intra_ac(&f, qs, &DEFAULT_INTRA_QUANT);
            assert_eq!(dct_zz[0], 0, "AC-only quantiser must not touch DC");
            let mut pred = IntraDcPredictors::at_slice_start();
            let recon = dequantize_intra_block(
                &dct_zz,
                qs,
                &DEFAULT_INTRA_QUANT,
                IntraBlockKind::LuminanceFirst,
                &mut pred,
                0,
            )
            .expect("dequantise");
            for m in 0..8 {
                for n in 0..8 {
                    if (m, n) == (0, 0) {
                        continue; // DC overwritten by the predictor chain
                    }
                    // One §2.4.4.1 quantiser step (2*qs*W/16) plus the
                    // odd-value mismatch perturbation (±1) plus the
                    // truncation of the reconstruction itself.
                    let step = (2 * i32::from(qs) * i32::from(DEFAULT_INTRA_QUANT[m][n])) / 16 + 2;
                    let err = (recon[m][n] - f[m][n]).abs();
                    assert!(
                        err <= step.max(3),
                        "qs {qs} ({m},{n}): err {err} > step {step}"
                    );
                }
            }
        }
    }

    #[test]
    fn mpeg1_non_intra_inverts_the_dequantiser_within_one_step() {
        use crate::dequantize::dequantize_non_intra_block;
        let flat16 = [[16u8; 8]; 8];
        let mut f = [[0i32; 8]; 8];
        for m in 0..8 {
            for n in 0..8 {
                f[m][n] = (((m * 8 + n) as i32 * 31) % 500) - 250;
            }
        }
        for qs in [1u8, 8, 31] {
            let dct_zz = mpeg1_forward_quantise_non_intra(&f, qs, &flat16);
            let recon = dequantize_non_intra_block(&dct_zz, qs, &flat16).expect("dequantise");
            for m in 0..8 {
                for n in 0..8 {
                    let step = (2 * i32::from(qs) * 16) / 16 + 2;
                    let err = (recon[m][n] - f[m][n]).abs();
                    assert!(
                        err <= step.max(3),
                        "qs {qs} ({m},{n}): err {err} > step {step}"
                    );
                }
            }
        }
    }

    #[test]
    fn mpeg1_non_intra_dead_zone_zeroes_small_coefficients() {
        // With all-16 weights at quantizer_scale 8 the §2.4.4.2 step is
        // 2*8*16/16 = 16; |F| < 16 truncates to level 0.
        let flat16 = [[16u8; 8]; 8];
        let mut f = [[0i32; 8]; 8];
        f[1][1] = 15;
        f[2][2] = 16;
        let dct_zz = mpeg1_forward_quantise_non_intra(&f, 8, &flat16);
        let z11 = dct_zz[crate::block_dc::SCAN[1][1] as usize];
        let z22 = dct_zz[crate::block_dc::SCAN[2][2] as usize];
        assert_eq!(z11, 0, "inside the dead-zone");
        assert_eq!(z22, 1, "on the first step");
    }

    #[test]
    fn mpeg1_levels_clamp_to_table_b5f_range() {
        let flat16 = [[16u8; 8]; 8];
        let mut f = [[0i32; 8]; 8];
        f[3][4] = 100_000;
        f[4][3] = -100_000;
        let dct_zz = mpeg1_forward_quantise_non_intra(&f, 1, &flat16);
        assert_eq!(
            dct_zz[crate::block_dc::SCAN[3][4] as usize],
            MPEG1_MAX_CODED_LEVEL
        );
        assert_eq!(
            dct_zz[crate::block_dc::SCAN[4][3] as usize],
            -MPEG1_MAX_CODED_LEVEL
        );
    }
}
