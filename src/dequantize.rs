//! MPEG-1 §2.4.4.1 intra-block and §2.4.4.2 non-intra-block
//! dequantiser per **ISO/IEC 11172-2:1993** (MPEG-1 Video).
//!
//! Round 17 landed the `dct_coeff_first` / `dct_coeff_next` walker
//! that emits a stream of `(run, signed_level)` pairs into the
//! zig-zag-scanned coefficient array `dct_zz[]`. This module is the
//! next pure-math stage: it takes that fully-populated `dct_zz[]`
//! array together with the active `quantizer_scale`, the
//! sequence-header quantiser matrix (`intra_quant[m][n]` or
//! `non_intra_quant[m][n]`), and — for intra blocks — the
//! `dct_dc_*_past` predictor chain, and computes the reconstructed
//! `dct_recon[m][n]` matrix that feeds the §A.1 IDCT.
//!
//! The spec defines four nearly-identical loops (page 32):
//!
//! * the first luminance block of an intra macroblock, which resets
//!   the DC predictor against `past_intra_address`;
//! * subsequent luminance blocks of the same intra macroblock, which
//!   read `dct_dc_y_past` directly;
//! * the chrominance Cb block — same `past_intra_address` reset, but
//!   driven by `dct_dc_cb_past`;
//! * the chrominance Cr block — same again, driven by
//!   `dct_dc_cr_past`.
//!
//! The arithmetic body is shared:
//!
//! ```text
//! for (m = 0; m < 8; m++) for (n = 0; n < 8; n++) {
//!     i = scan[m][n];
//!     dct_recon[m][n] = (2 * dct_zz[i] * quantizer_scale
//!                            * intra_quant[m][n]) / 16;
//!     if ((dct_recon[m][n] & 1) == 0)
//!         dct_recon[m][n] -= Sign(dct_recon[m][n]);   // even -> mismatch
//!     if (dct_recon[m][n] >  2047) dct_recon[m][n] =  2047;
//!     if (dct_recon[m][n] < -2048) dct_recon[m][n] = -2048;
//! }
//! ```
//!
//! and the DC element `dct_recon[0][0]` is then overwritten per
//! block-kind:
//!
//! ```text
//! dct_recon[0][0] = dct_zz[0] * 8;                       // first luma / Cb / Cr
//! if ((macroblock_address - past_intra_address) > 1)
//!     dct_recon[0][0] += 128 * 8;                        // 1024
//! else
//!     dct_recon[0][0] += dct_dc_<comp>_past;
//! dct_dc_<comp>_past = dct_recon[0][0];
//! ```
//!
//! and, for subsequent luminance blocks of the same macroblock,
//!
//! ```text
//! dct_recon[0][0] = dct_dc_y_past + dct_zz[0] * 8;
//! dct_dc_y_past   = dct_recon[0][0];
//! ```
//!
//! At the end of an intra macroblock the spec sets
//! `past_intra_address = macroblock_address`.
//!
//! For non-intra (§2.4.4.2 page 35) the inner body changes to
//!
//! ```text
//! dct_recon[m][n] = ((2 * dct_zz[i] + Sign(dct_zz[i]))
//!                        * quantizer_scale
//!                        * non_intra_quant[m][n]) / 16;
//! ```
//!
//! followed by the same even-mismatch correction and saturation, and
//! a final `if (dct_zz[i] == 0) dct_recon[m][n] = 0;` zeroing pass.
//!
//! Spec citations refer to **ISO/IEC 11172-2:1993** (MPEG-1 Video)
//! §2.4.4.1 (page 32), §2.4.4.2 (page 35), and §2.4.3.2 for the
//! default intra / non-intra quantiser matrices (page 25). The
//! companion MPEG-2 dequantiser (ISO/IEC 13818-2 §7.4.2) uses
//! different arithmetic and is intentionally out of scope for this
//! round.

use crate::block_dc::SCAN;
use crate::{Error, Result};

// =============================================================
// §2.4.3.2 — default quantiser matrices (page 25)
// =============================================================

/// Default `intra_quant[m][n]` used when the sequence header sets
/// `load_intra_quantizer_matrix == 0` (§2.4.3.2 page 25). The
/// matrix is printed there in raster order; we store it row-major.
pub const DEFAULT_INTRA_QUANT: [[u8; 8]; 8] = [
    [8, 16, 19, 22, 26, 27, 29, 34],
    [16, 16, 22, 24, 27, 29, 34, 37],
    [19, 22, 26, 27, 29, 34, 34, 38],
    [22, 22, 26, 27, 29, 34, 37, 40],
    [22, 26, 27, 29, 32, 35, 40, 48],
    [26, 27, 29, 32, 35, 40, 48, 58],
    [26, 27, 29, 34, 38, 46, 56, 69],
    [27, 29, 35, 38, 46, 56, 69, 83],
];

/// Default `non_intra_quant[m][n]` used when the sequence header
/// sets `load_non_intra_quantizer_matrix == 0` (§2.4.3.2 page 25).
/// Every entry is 16.
pub const DEFAULT_NON_INTRA_QUANT: [[u8; 8]; 8] = [[16; 8]; 8];

/// The §2.4.4.1 / §2.4.4.2 saturation upper bound on `dct_recon`.
pub const DCT_RECON_MAX: i32 = 2047;

/// The §2.4.4.1 / §2.4.4.2 saturation lower bound on `dct_recon`.
pub const DCT_RECON_MIN: i32 = -2048;

/// The DC predictor reset value `128 * 8 = 1024` used at the start
/// of a slice and at non-intra-coded macroblocks per §2.4.4.1.
pub const DC_PREDICTOR_RESET: i32 = 128 * 8;

// =============================================================
// Per-block kind selector
// =============================================================

/// Which of the four §2.4.4.1 intra-block loops the caller is
/// invoking. The arithmetic body is identical; the only difference
/// is which DC predictor is touched and whether the
/// `past_intra_address` reset rule applies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntraBlockKind {
    /// First luminance block of the macroblock. The DC is built
    /// from `dct_dc_y_past` if
    /// `(macroblock_address - past_intra_address) == 1`; otherwise
    /// the spec's `(128 * 8)` reset is used.
    LuminanceFirst,
    /// Subsequent luminance block (pattern positions 1..=3) of the
    /// same macroblock. The DC is always `dct_dc_y_past +
    /// dct_zz[0] * 8` — no `past_intra_address` test.
    LuminanceSubsequent,
    /// Chrominance Cb block. Same `past_intra_address` reset as
    /// `LuminanceFirst`, driven by `dct_dc_cb_past`.
    ChrominanceCb,
    /// Chrominance Cr block. Same `past_intra_address` reset as
    /// `LuminanceFirst`, driven by `dct_dc_cr_past`.
    ChrominanceCr,
}

// =============================================================
// Intra-block DC predictor chain (§2.4.4.1)
// =============================================================

/// The §2.4.4.1 intra-block DC predictor state.
///
/// `dct_dc_*_past` are the `dct_recon[0][0]` values of the most
/// recently decoded intra-coded Y / Cb / Cr blocks; they shall be
/// reset to `128 * 8 = 1024` at the start of a slice and at every
/// non-intra-coded macroblock (including skipped macroblocks).
///
/// `past_intra_address` is the `macroblock_address` of the most
/// recently retrieved intra-coded macroblock within the slice and
/// shall be reset to `-2` at the beginning of each slice; this
/// crate stores the reset value as the sentinel
/// [`IntraDcPredictors::SLICE_START_PAST_INTRA_ADDRESS`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IntraDcPredictors {
    /// `dct_dc_y_past` (most recent intra Y block's `dct_recon[0][0]`).
    pub y_past: i32,
    /// `dct_dc_cb_past`.
    pub cb_past: i32,
    /// `dct_dc_cr_past`.
    pub cr_past: i32,
    /// `past_intra_address` — the most recent intra macroblock's
    /// `macroblock_address`. Held as a signed integer so the
    /// spec's `-2` slice-start sentinel fits without an extra flag.
    pub past_intra_address: i32,
}

impl IntraDcPredictors {
    /// `past_intra_address` value the spec demands at slice start
    /// (§2.4.4.1 page 32: *"It shall be reset to -2 at the
    /// beginning of each slice."*).
    pub const SLICE_START_PAST_INTRA_ADDRESS: i32 = -2;

    /// Construct the predictor chain in its slice-start state per
    /// §2.4.4.1: all three `dct_dc_*_past` equal `128 * 8`, and
    /// `past_intra_address == -2`.
    pub fn at_slice_start() -> Self {
        Self {
            y_past: DC_PREDICTOR_RESET,
            cb_past: DC_PREDICTOR_RESET,
            cr_past: DC_PREDICTOR_RESET,
            past_intra_address: Self::SLICE_START_PAST_INTRA_ADDRESS,
        }
    }

    /// Reset the three `dct_dc_*_past` fields back to `128 * 8`
    /// without touching `past_intra_address`. This is the
    /// per-non-intra-macroblock reset of §2.4.4.1: *"The predictors
    /// dct_dc_y_past, dct_dc_cb_past and dct_dc_cr_past shall all
    /// be reset … at non-intra-coded macroblocks (including
    /// skipped macroblocks) to the value 1 024 (128 * 8)."*
    pub fn reset_dc_to_default(&mut self) {
        self.y_past = DC_PREDICTOR_RESET;
        self.cb_past = DC_PREDICTOR_RESET;
        self.cr_past = DC_PREDICTOR_RESET;
    }
}

// =============================================================
// Arithmetic helpers (shared body)
// =============================================================

/// `Sign(x)` per the §2.4.4.1 footnote: returns `-1`, `0`, `+1`.
#[inline]
fn sign(x: i32) -> i32 {
    match x.cmp(&0) {
        core::cmp::Ordering::Less => -1,
        core::cmp::Ordering::Equal => 0,
        core::cmp::Ordering::Greater => 1,
    }
}

/// Saturate to `[DCT_RECON_MIN, DCT_RECON_MAX]` per the §2.4.4.1
/// / §2.4.4.2 saturating clip. Spec form is two cascaded if /
/// if statements; this is the same semantics via `i32::clamp`.
#[inline]
fn saturate_dct_recon(v: i32) -> i32 {
    v.clamp(DCT_RECON_MIN, DCT_RECON_MAX)
}

/// Apply the even-value mismatch-prevention rule.
///
/// Quoting §2.4.4.1 directly: *"if ( ( dct_recon[m][n] & 1 ) == 0 )
/// dct_recon[m][n] = dct_recon[m][n] - Sign(dct_recon[m][n]) ;"*
/// followed by *"Note that this process disallows even valued
/// numbers. This has been found to prevent accumulation of mismatch
/// errors."*
///
/// Bit semantics: `& 1` of a two's-complement signed integer is the
/// LSB; `0` for even, `1` for odd. Equivalent to `v.rem_euclid(2)
/// == 0`. The original AND form is preserved here so a reader can
/// match it line-by-line against the spec.
#[inline]
fn apply_mismatch_prevention(v: i32) -> i32 {
    if (v & 1) == 0 {
        v - sign(v)
    } else {
        v
    }
}

// =============================================================
// Intra-block dequantiser (§2.4.4.1)
// =============================================================

/// Dequantise one intra block per **ISO/IEC 11172-2:1993 §2.4.4.1**
/// (page 32).
///
/// Inputs:
/// * `dct_zz` — the 64 zig-zag-ordered quantised DCT coefficients
///   the §2.4.3.7 walker emitted into the block. Element 0 is the
///   DC differential reconstruction from §2.4.3.7
///   ([`crate::block_dc::DcCoefficient::dct_zz_0`]); elements
///   1..=63 are the run-level body. Out-of-range elements that the
///   walker never wrote remain `0`.
/// * `quantizer_scale` — the active `quantizer_scale` in `1..=31`
///   (§2.4.3.6). Zero is forbidden upstream; this routine rejects
///   it as a defensive guard.
/// * `intra_quant` — the 8x8 `intra_quant[m][n]` matrix from the
///   sequence header (either [`DEFAULT_INTRA_QUANT`] when
///   `load_intra_quantizer_matrix == 0`, or the loaded matrix).
///   Per §2.4.3.2 the value `0` is forbidden. The spec note about
///   `intra_quant[0][0]` is observed: the value used in the
///   per-coefficient loop is overwritten by the subsequent DC
///   computation, so `intra_quant[0][0]` need not be 8 in this
///   function — but the sequence-header parser enforces 8.
/// * `kind` — which of the four §2.4.4.1 block loops to run.
/// * `predictors` — the live `IntraDcPredictors` chain. Mutated:
///   the matching `dct_dc_<comp>_past` field is set to the new
///   `dct_recon[0][0]`.
/// * `macroblock_address` — the spec's `macroblock_address` for
///   the macroblock containing this block. Used by every block
///   except `LuminanceSubsequent` to evaluate the
///   `(macroblock_address - past_intra_address) > 1` reset test.
///   Caller is responsible for invoking [`finalise_intra_macroblock`]
///   once per macroblock to update `past_intra_address`.
///
/// Returns the 8x8 `dct_recon[m][n]` matrix in raster order, with
/// every element saturated to `[-2048, 2047]` and the DC
/// post-overwrite already applied.
pub fn dequantize_intra_block(
    dct_zz: &[i32; 64],
    quantizer_scale: u8,
    intra_quant: &[[u8; 8]; 8],
    kind: IntraBlockKind,
    predictors: &mut IntraDcPredictors,
    macroblock_address: i32,
) -> Result<[[i32; 8]; 8]> {
    if quantizer_scale == 0 {
        return Err(Error::InvalidBitstream(
            "dequantize_intra_block: quantizer_scale = 0 is forbidden (§2.4.3.6)",
        ));
    }
    if quantizer_scale > 31 {
        return Err(Error::InvalidBitstream(
            "dequantize_intra_block: quantizer_scale > 31 is impossible (§2.4.3.6)",
        ));
    }

    // Shared body: AC + tentative DC (the DC is overwritten right
    // after the loop per §2.4.4.1).
    let qs = i32::from(quantizer_scale);
    let mut dct_recon = [[0i32; 8]; 8];
    for (m, row) in dct_recon.iter_mut().enumerate() {
        for (n, cell) in row.iter_mut().enumerate() {
            let i = SCAN[m][n] as usize;
            let q = i32::from(intra_quant[m][n]);
            if q == 0 {
                return Err(Error::InvalidBitstream(
                    "dequantize_intra_block: intra_quant entry = 0 is forbidden (§2.4.3.2)",
                ));
            }
            let raw = (2 * dct_zz[i] * qs * q) / 16;
            let mismatch_fixed = apply_mismatch_prevention(raw);
            *cell = saturate_dct_recon(mismatch_fixed);
        }
    }

    // DC overwrite per §2.4.4.1, branching on block kind.
    let dc = dct_zz[0] * 8;
    let new_dc = match kind {
        IntraBlockKind::LuminanceFirst => {
            if (macroblock_address - predictors.past_intra_address) > 1 {
                DC_PREDICTOR_RESET + dc
            } else {
                predictors.y_past + dc
            }
        }
        IntraBlockKind::LuminanceSubsequent => predictors.y_past + dc,
        IntraBlockKind::ChrominanceCb => {
            if (macroblock_address - predictors.past_intra_address) > 1 {
                DC_PREDICTOR_RESET + dc
            } else {
                predictors.cb_past + dc
            }
        }
        IntraBlockKind::ChrominanceCr => {
            if (macroblock_address - predictors.past_intra_address) > 1 {
                DC_PREDICTOR_RESET + dc
            } else {
                predictors.cr_past + dc
            }
        }
    };
    dct_recon[0][0] = new_dc;

    // The per-component predictor advances to the new DC value
    // *after* the overwrite (§2.4.4.1: `dct_dc_<comp>_past =
    // dct_recon[0][0];` at the end of every block).
    match kind {
        IntraBlockKind::LuminanceFirst | IntraBlockKind::LuminanceSubsequent => {
            predictors.y_past = new_dc;
        }
        IntraBlockKind::ChrominanceCb => predictors.cb_past = new_dc,
        IntraBlockKind::ChrominanceCr => predictors.cr_past = new_dc,
    }

    Ok(dct_recon)
}

/// Per-macroblock close-out for the §2.4.4.1 intra path:
///
/// *"After all the blocks in the macroblock are processed:
/// `past_intra_address = macroblock_address ;`."*
///
/// Call this once per intra macroblock, after every block has gone
/// through [`dequantize_intra_block`].
pub fn finalise_intra_macroblock(predictors: &mut IntraDcPredictors, macroblock_address: i32) {
    predictors.past_intra_address = macroblock_address;
}

// =============================================================
// Non-intra-block dequantiser (§2.4.4.2 page 35)
// =============================================================

/// Dequantise one non-intra block per **ISO/IEC 11172-2:1993
/// §2.4.4.2** (page 35).
///
/// Differences from the intra path:
/// * The inner numerator is `(2 * dct_zz[i] + Sign(dct_zz[i])) *
///   quantizer_scale * non_intra_quant[m][n]`; the `+ Sign(...)`
///   term is the dead-zone restoration that compensates for the
///   non-intra encoder's signed truncation. For `dct_zz[i] == 0`
///   the term is `0` and the post-saturation zeroing pass below
///   makes the contribution `0` regardless.
/// * The DC element is *not* a separate predictor update — there
///   is no `dct_dc_*_past` chain for non-intra blocks. `dct_zz[0]`
///   goes through the same per-coefficient arithmetic as every
///   other element.
/// * A final zeroing pass: *"if ( dct_zz[i] == 0 ) dct_recon[m][n]
///   = 0 ;"* — this forces a zero coefficient through to zero even
///   if the `+ Sign(0) = 0` arithmetic and the mismatch-prevention
///   rule produced a non-zero `dct_recon[m][n]`. The spec lists
///   this *after* the saturation; it is therefore the last write.
///
/// `dct_recon[m][n] = 0` for all `m, n` for skipped macroblocks
/// and when `pattern[i] == 0` — both of those are caller
/// responsibilities; this routine only runs when the caller has
/// already determined the block is present.
pub fn dequantize_non_intra_block(
    dct_zz: &[i32; 64],
    quantizer_scale: u8,
    non_intra_quant: &[[u8; 8]; 8],
) -> Result<[[i32; 8]; 8]> {
    if quantizer_scale == 0 {
        return Err(Error::InvalidBitstream(
            "dequantize_non_intra_block: quantizer_scale = 0 is forbidden (§2.4.3.6)",
        ));
    }
    if quantizer_scale > 31 {
        return Err(Error::InvalidBitstream(
            "dequantize_non_intra_block: quantizer_scale > 31 is impossible (§2.4.3.6)",
        ));
    }

    let qs = i32::from(quantizer_scale);
    let mut dct_recon = [[0i32; 8]; 8];
    for (m, row) in dct_recon.iter_mut().enumerate() {
        for (n, cell) in row.iter_mut().enumerate() {
            let i = SCAN[m][n] as usize;
            let q = i32::from(non_intra_quant[m][n]);
            if q == 0 {
                return Err(Error::InvalidBitstream(
                    "dequantize_non_intra_block: non_intra_quant entry = 0 is forbidden (§2.4.3.2)",
                ));
            }
            let z = dct_zz[i];
            let raw = ((2 * z + sign(z)) * qs * q) / 16;
            let mismatch_fixed = apply_mismatch_prevention(raw);
            let saturated = saturate_dct_recon(mismatch_fixed);
            // §2.4.4.2 final zeroing pass.
            *cell = if z == 0 { 0 } else { saturated };
        }
    }

    Ok(dct_recon)
}

#[cfg(test)]
mod tests {
    //! Spec-pinned coverage of the §2.4.4.1 intra and §2.4.4.2
    //! non-intra dequantiser bodies, the DC predictor chain
    //! (including the `past_intra_address > 1` reset branch), the
    //! `Sign(...)` even-mismatch fix, the `[-2048, 2047]`
    //! saturation, and the non-intra `dct_zz[i] == 0 -> 0` zeroing
    //! pass.
    use super::*;

    fn zeros() -> [i32; 64] {
        [0i32; 64]
    }

    // ----- default matrices -----

    #[test]
    fn default_intra_quant_origin_is_eight_per_spec() {
        // §2.4.3.2 page 25 prints `intra_quant[0][0] = 8` for both
        // the default matrix and any loaded one (the spec demands
        // the value 8 at the origin even when loaded).
        assert_eq!(DEFAULT_INTRA_QUANT[0][0], 8);
    }

    #[test]
    fn default_non_intra_quant_is_uniform_sixteen() {
        // §2.4.3.2 page 25: every entry of the default non-intra
        // matrix is 16.
        for row in DEFAULT_NON_INTRA_QUANT.iter() {
            for &v in row.iter() {
                assert_eq!(v, 16);
            }
        }
    }

    #[test]
    fn default_intra_quant_matches_spec_corners() {
        // Sanity-check a handful of spec-page values from §2.4.3.2.
        assert_eq!(DEFAULT_INTRA_QUANT[0][0], 8);
        assert_eq!(DEFAULT_INTRA_QUANT[0][7], 34);
        assert_eq!(DEFAULT_INTRA_QUANT[7][0], 27);
        assert_eq!(DEFAULT_INTRA_QUANT[7][7], 83);
        assert_eq!(DEFAULT_INTRA_QUANT[3][3], 27);
    }

    // ----- predictor reset behaviour -----

    #[test]
    fn slice_start_predictors_match_spec_reset_values() {
        let p = IntraDcPredictors::at_slice_start();
        assert_eq!(p.y_past, 1024);
        assert_eq!(p.cb_past, 1024);
        assert_eq!(p.cr_past, 1024);
        assert_eq!(p.past_intra_address, -2);
    }

    #[test]
    fn reset_dc_leaves_past_intra_address_untouched() {
        let mut p = IntraDcPredictors {
            y_past: 1500,
            cb_past: 1500,
            cr_past: 1500,
            past_intra_address: 7,
        };
        p.reset_dc_to_default();
        assert_eq!(p.y_past, 1024);
        assert_eq!(p.cb_past, 1024);
        assert_eq!(p.cr_past, 1024);
        assert_eq!(p.past_intra_address, 7);
    }

    // ----- mismatch / saturation arithmetic primitives -----

    #[test]
    fn sign_returns_minus_one_zero_plus_one() {
        assert_eq!(sign(-5), -1);
        assert_eq!(sign(0), 0);
        assert_eq!(sign(7), 1);
    }

    #[test]
    fn mismatch_prevention_no_op_on_odd_values() {
        for v in &[-2047i32, -3, -1, 1, 3, 2047] {
            assert_eq!(apply_mismatch_prevention(*v), *v);
        }
    }

    #[test]
    fn mismatch_prevention_subtracts_sign_on_even_positive() {
        assert_eq!(apply_mismatch_prevention(8), 7);
        assert_eq!(apply_mismatch_prevention(2), 1);
        assert_eq!(apply_mismatch_prevention(2046), 2045);
    }

    #[test]
    fn mismatch_prevention_adds_one_on_even_negative() {
        // Sign(-x) = -1 so `v - Sign(v) = v - (-1) = v + 1`.
        assert_eq!(apply_mismatch_prevention(-8), -7);
        assert_eq!(apply_mismatch_prevention(-2), -1);
        assert_eq!(apply_mismatch_prevention(-2046), -2045);
    }

    #[test]
    fn mismatch_prevention_leaves_zero_alone() {
        // Sign(0) = 0, so 0 - 0 = 0. Spec is happy with a zero
        // intermediate; the dct_zz[i] == 0 -> 0 zeroing pass in
        // the non-intra path is the explicit force-to-zero.
        assert_eq!(apply_mismatch_prevention(0), 0);
    }

    #[test]
    fn saturate_clips_high_and_low_bounds() {
        assert_eq!(saturate_dct_recon(5000), 2047);
        assert_eq!(saturate_dct_recon(-9999), -2048);
        assert_eq!(saturate_dct_recon(0), 0);
        assert_eq!(saturate_dct_recon(2047), 2047);
        assert_eq!(saturate_dct_recon(-2048), -2048);
    }

    // ----- intra dequantiser: rejection sites -----

    #[test]
    fn intra_rejects_quantizer_scale_zero() {
        let dct_zz = zeros();
        let mut pred = IntraDcPredictors::at_slice_start();
        let err = dequantize_intra_block(
            &dct_zz,
            0,
            &DEFAULT_INTRA_QUANT,
            IntraBlockKind::LuminanceFirst,
            &mut pred,
            0,
        )
        .unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn intra_rejects_intra_quant_zero_entry() {
        let dct_zz = zeros();
        let mut q = DEFAULT_INTRA_QUANT;
        q[3][4] = 0;
        let mut pred = IntraDcPredictors::at_slice_start();
        let err =
            dequantize_intra_block(&dct_zz, 1, &q, IntraBlockKind::LuminanceFirst, &mut pred, 0)
                .unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn intra_rejects_quantizer_scale_over_31() {
        let dct_zz = zeros();
        let mut pred = IntraDcPredictors::at_slice_start();
        let err = dequantize_intra_block(
            &dct_zz,
            32,
            &DEFAULT_INTRA_QUANT,
            IntraBlockKind::LuminanceFirst,
            &mut pred,
            0,
        )
        .unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    // ----- intra dequantiser: arithmetic at zero coefficients -----

    #[test]
    fn all_zero_zz_produces_zero_ac_and_reset_dc_at_slice_start() {
        // dct_zz all zero; predictors all 1024; macroblock_address=0
        // means (0 - (-2)) = 2 > 1, so the reset branch fires →
        // DC = 1024 + 0 = 1024. AC: 2 * 0 * qs * q / 16 = 0,
        // mismatch fix: 0 stays 0 (even but Sign(0) = 0).
        let dct_zz = zeros();
        let mut pred = IntraDcPredictors::at_slice_start();
        let recon = dequantize_intra_block(
            &dct_zz,
            8,
            &DEFAULT_INTRA_QUANT,
            IntraBlockKind::LuminanceFirst,
            &mut pred,
            0,
        )
        .unwrap();
        for (m, row) in recon.iter().enumerate() {
            for (n, &cell) in row.iter().enumerate() {
                if (m, n) == (0, 0) {
                    assert_eq!(cell, 1024);
                } else {
                    assert_eq!(cell, 0, "AC ({m},{n}) should be 0");
                }
            }
        }
        // Predictor chain advanced for Y.
        assert_eq!(pred.y_past, 1024);
        assert_eq!(pred.cb_past, 1024);
        assert_eq!(pred.cr_past, 1024);
    }

    // ----- DC branch: distance-from-past test -----

    #[test]
    fn intra_dc_uses_past_for_adjacent_macroblock() {
        // past_intra_address = 4, current macroblock_address = 5
        // (difference = 1, *not* > 1) → use dct_dc_y_past.
        let dct_zz = zeros();
        let mut pred = IntraDcPredictors {
            y_past: 800,
            cb_past: 1024,
            cr_past: 1024,
            past_intra_address: 4,
        };
        let recon = dequantize_intra_block(
            &dct_zz,
            1,
            &DEFAULT_INTRA_QUANT,
            IntraBlockKind::LuminanceFirst,
            &mut pred,
            5,
        )
        .unwrap();
        // DC = y_past (800) + dct_zz[0] * 8 (0) = 800.
        assert_eq!(recon[0][0], 800);
        assert_eq!(pred.y_past, 800);
    }

    #[test]
    fn intra_dc_resets_to_1024_on_gap() {
        // past_intra_address = 4, current macroblock_address = 7
        // (difference = 3 > 1) → use the 128*8 reset.
        let dct_zz = zeros();
        let mut pred = IntraDcPredictors {
            y_past: 800,
            cb_past: 1024,
            cr_past: 1024,
            past_intra_address: 4,
        };
        let recon = dequantize_intra_block(
            &dct_zz,
            1,
            &DEFAULT_INTRA_QUANT,
            IntraBlockKind::LuminanceFirst,
            &mut pred,
            7,
        )
        .unwrap();
        assert_eq!(recon[0][0], 1024);
        assert_eq!(pred.y_past, 1024);
    }

    #[test]
    fn intra_subsequent_luma_always_uses_y_past_without_reset_test() {
        // LuminanceSubsequent ignores past_intra_address — DC is
        // always y_past + dct_zz[0]*8 even with a huge address
        // gap.
        let mut dct_zz = zeros();
        dct_zz[0] = 3; // contributes 3 * 8 = 24.
        let mut pred = IntraDcPredictors {
            y_past: 600,
            cb_past: 1024,
            cr_past: 1024,
            past_intra_address: 1, // would trigger reset on First
        };
        let recon = dequantize_intra_block(
            &dct_zz,
            1,
            &DEFAULT_INTRA_QUANT,
            IntraBlockKind::LuminanceSubsequent,
            &mut pred,
            99, // huge gap — irrelevant for Subsequent
        )
        .unwrap();
        assert_eq!(recon[0][0], 624);
        assert_eq!(pred.y_past, 624);
    }

    #[test]
    fn intra_cb_uses_cb_past_chain() {
        let mut dct_zz = zeros();
        dct_zz[0] = 1;
        let mut pred = IntraDcPredictors {
            y_past: 1000,
            cb_past: 500,
            cr_past: 1024,
            past_intra_address: 9,
        };
        let recon = dequantize_intra_block(
            &dct_zz,
            1,
            &DEFAULT_INTRA_QUANT,
            IntraBlockKind::ChrominanceCb,
            &mut pred,
            10, // diff = 1 -> use cb_past
        )
        .unwrap();
        // DC = cb_past (500) + 1 * 8 = 508.
        assert_eq!(recon[0][0], 508);
        assert_eq!(pred.cb_past, 508);
        // Other predictors untouched.
        assert_eq!(pred.y_past, 1000);
        assert_eq!(pred.cr_past, 1024);
    }

    #[test]
    fn intra_cr_uses_cr_past_chain() {
        let mut dct_zz = zeros();
        dct_zz[0] = -2; // contributes -16.
        let mut pred = IntraDcPredictors {
            y_past: 1000,
            cb_past: 500,
            cr_past: 700,
            past_intra_address: 9,
        };
        let recon = dequantize_intra_block(
            &dct_zz,
            1,
            &DEFAULT_INTRA_QUANT,
            IntraBlockKind::ChrominanceCr,
            &mut pred,
            10,
        )
        .unwrap();
        // DC = cr_past (700) + (-2)*8 = 684.
        assert_eq!(recon[0][0], 684);
        assert_eq!(pred.cr_past, 684);
        assert_eq!(pred.cb_past, 500);
        assert_eq!(pred.y_past, 1000);
    }

    // ----- finalise close-out -----

    #[test]
    fn finalise_updates_past_intra_address() {
        let mut pred = IntraDcPredictors::at_slice_start();
        finalise_intra_macroblock(&mut pred, 12);
        assert_eq!(pred.past_intra_address, 12);
        // Other fields untouched.
        assert_eq!(pred.y_past, 1024);
    }

    // ----- intra AC arithmetic worked example -----

    #[test]
    fn intra_ac_worked_example_uniform_quant_and_qs() {
        // intra_quant = 16 everywhere (synthetic flat matrix);
        // quantizer_scale = 4; dct_zz[scan[1][0]] = 5
        //   → raw = 2 * 5 * 4 * 16 / 16 = 40, even → 40 - 1 = 39,
        //     saturate(39) = 39.
        let mut quant = [[16u8; 8]; 8];
        quant[0][0] = 8; // spec demands intra_quant[0][0] == 8
        let mut dct_zz = zeros();
        // scan[1][0] = 2 (from SCAN matrix).
        dct_zz[2] = 5;
        let mut pred = IntraDcPredictors::at_slice_start();
        let recon = dequantize_intra_block(
            &dct_zz,
            4,
            &quant,
            IntraBlockKind::LuminanceFirst,
            &mut pred,
            0,
        )
        .unwrap();
        assert_eq!(recon[1][0], 39);
    }

    #[test]
    fn intra_ac_negative_even_subtracts_sign_correctly() {
        // dct_zz at scan[1][0] = -3, qs = 4, q[1][0] = 16:
        //   raw = 2 * -3 * 4 * 16 / 16 = -24, even, Sign(-24) = -1
        //   → -24 - (-1) = -23, saturate(-23) = -23.
        let mut quant = [[16u8; 8]; 8];
        quant[0][0] = 8;
        let mut dct_zz = zeros();
        dct_zz[2] = -3;
        let mut pred = IntraDcPredictors::at_slice_start();
        let recon = dequantize_intra_block(
            &dct_zz,
            4,
            &quant,
            IntraBlockKind::LuminanceFirst,
            &mut pred,
            0,
        )
        .unwrap();
        assert_eq!(recon[1][0], -23);
    }

    #[test]
    fn intra_ac_saturates_to_2047() {
        // qs = 31, q = 64 (synthetic), dct_zz = 40
        // raw = 2 * 40 * 31 * 64 / 16 = 9920 -> saturate -> 2047
        let mut quant = [[64u8; 8]; 8];
        quant[0][0] = 8;
        let mut dct_zz = zeros();
        dct_zz[2] = 40;
        let mut pred = IntraDcPredictors::at_slice_start();
        let recon = dequantize_intra_block(
            &dct_zz,
            31,
            &quant,
            IntraBlockKind::LuminanceFirst,
            &mut pred,
            0,
        )
        .unwrap();
        assert_eq!(recon[1][0], 2047);
    }

    #[test]
    fn intra_ac_saturates_to_negative_2048() {
        let mut quant = [[64u8; 8]; 8];
        quant[0][0] = 8;
        let mut dct_zz = zeros();
        dct_zz[2] = -40;
        let mut pred = IntraDcPredictors::at_slice_start();
        let recon = dequantize_intra_block(
            &dct_zz,
            31,
            &quant,
            IntraBlockKind::LuminanceFirst,
            &mut pred,
            0,
        )
        .unwrap();
        assert_eq!(recon[1][0], -2048);
    }

    // ----- non-intra dequantiser: rejection sites -----

    #[test]
    fn non_intra_rejects_quantizer_scale_zero() {
        let dct_zz = zeros();
        let err = dequantize_non_intra_block(&dct_zz, 0, &DEFAULT_NON_INTRA_QUANT).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn non_intra_rejects_non_intra_quant_zero_entry() {
        let dct_zz = zeros();
        let mut q = DEFAULT_NON_INTRA_QUANT;
        q[2][2] = 0;
        let err = dequantize_non_intra_block(&dct_zz, 1, &q).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    // ----- non-intra dequantiser: zeroing pass -----

    #[test]
    fn non_intra_zero_coeffs_yield_zero_recon() {
        let dct_zz = zeros();
        let recon = dequantize_non_intra_block(&dct_zz, 8, &DEFAULT_NON_INTRA_QUANT).unwrap();
        for (m, row) in recon.iter().enumerate() {
            for (n, &cell) in row.iter().enumerate() {
                assert_eq!(cell, 0, "non-intra zero zz at ({m},{n})");
            }
        }
    }

    #[test]
    fn non_intra_positive_coeff_worked_example() {
        // qs = 8, q = 16, dct_zz[scan[1][0]] = +3
        //   numerator = (2*3 + 1) * 8 * 16 = 7 * 128 = 896
        //   /16 = 56, even → 56 - 1 = 55, saturate(55) = 55.
        let mut dct_zz = zeros();
        dct_zz[2] = 3;
        let recon = dequantize_non_intra_block(&dct_zz, 8, &DEFAULT_NON_INTRA_QUANT).unwrap();
        assert_eq!(recon[1][0], 55);
    }

    #[test]
    fn non_intra_negative_coeff_worked_example() {
        // qs = 8, q = 16, dct_zz[scan[1][0]] = -3
        //   numerator = (2*-3 + -1) * 8 * 16 = -7 * 128 = -896
        //   /16 = -56, even → -56 - (-1) = -55, saturate = -55.
        let mut dct_zz = zeros();
        dct_zz[2] = -3;
        let recon = dequantize_non_intra_block(&dct_zz, 8, &DEFAULT_NON_INTRA_QUANT).unwrap();
        assert_eq!(recon[1][0], -55);
    }

    #[test]
    fn non_intra_saturates_to_2047() {
        let mut q = [[127u8; 8]; 8];
        q[0][0] = 16;
        let mut dct_zz = zeros();
        dct_zz[2] = 100;
        let recon = dequantize_non_intra_block(&dct_zz, 31, &q).unwrap();
        assert_eq!(recon[1][0], 2047);
    }

    #[test]
    fn non_intra_saturates_to_negative_2048() {
        let mut q = [[127u8; 8]; 8];
        q[0][0] = 16;
        let mut dct_zz = zeros();
        dct_zz[2] = -100;
        let recon = dequantize_non_intra_block(&dct_zz, 31, &q).unwrap();
        assert_eq!(recon[1][0], -2048);
    }

    #[test]
    fn non_intra_zeroing_pass_overrides_mismatch_fix_on_zero_coeffs() {
        // For dct_zz == 0 the inner arithmetic produces 0; the
        // mismatch-prevention rule then keeps it 0 (Sign(0) = 0
        // so the subtraction is a no-op). The zeroing pass is a
        // belt-and-braces guard against any future arithmetic
        // change. Cover it explicitly: a zero dct_zz must yield a
        // zero recon even when its neighbours are non-zero.
        let mut dct_zz = zeros();
        dct_zz[3] = 7;
        let recon = dequantize_non_intra_block(&dct_zz, 4, &DEFAULT_NON_INTRA_QUANT).unwrap();
        // scan[m][n] = 2 maps to (1, 0): dct_zz[2] = 0 → recon[1][0] = 0.
        assert_eq!(recon[1][0], 0);
        // scan[m][n] = 3 maps to (2, 0): dct_zz[3] = 7 (non-zero).
        // numerator = (2*7+1) * 4 * 16 = 960 / 16 = 60 even → 59.
        assert_eq!(recon[2][0], 59);
    }

    // ----- intra: full slice-start macroblock walk-through -----

    #[test]
    fn intra_macroblock_walkthrough_advances_all_three_predictors() {
        // Walk a full intra macroblock: 4 luma blocks (1 First, 3
        // Subsequent), then Cb, then Cr; verify the predictor
        // chain mutates only the matching component each time.
        let mut dct_zz = zeros();
        dct_zz[0] = 1; // every block contributes +8 to DC.
        let mut pred = IntraDcPredictors::at_slice_start();

        // First Y (slice start: address gap > 1 → reset).
        let r0 = dequantize_intra_block(
            &dct_zz,
            1,
            &DEFAULT_INTRA_QUANT,
            IntraBlockKind::LuminanceFirst,
            &mut pred,
            0,
        )
        .unwrap();
        assert_eq!(r0[0][0], 1024 + 8);
        assert_eq!(pred.y_past, 1032);
        assert_eq!(pred.cb_past, 1024);
        assert_eq!(pred.cr_past, 1024);

        // Subsequent Y blocks chain through y_past.
        for expected in [1040, 1048, 1056] {
            let r = dequantize_intra_block(
                &dct_zz,
                1,
                &DEFAULT_INTRA_QUANT,
                IntraBlockKind::LuminanceSubsequent,
                &mut pred,
                0,
            )
            .unwrap();
            assert_eq!(r[0][0], expected);
            assert_eq!(pred.y_past, expected);
        }

        // Cb: at slice start address gap is still > 1 → reset.
        let rc = dequantize_intra_block(
            &dct_zz,
            1,
            &DEFAULT_INTRA_QUANT,
            IntraBlockKind::ChrominanceCb,
            &mut pred,
            0,
        )
        .unwrap();
        assert_eq!(rc[0][0], 1032);
        assert_eq!(pred.cb_past, 1032);

        // Cr: same.
        let rcr = dequantize_intra_block(
            &dct_zz,
            1,
            &DEFAULT_INTRA_QUANT,
            IntraBlockKind::ChrominanceCr,
            &mut pred,
            0,
        )
        .unwrap();
        assert_eq!(rcr[0][0], 1032);
        assert_eq!(pred.cr_past, 1032);

        // Close the macroblock.
        finalise_intra_macroblock(&mut pred, 0);
        assert_eq!(pred.past_intra_address, 0);
        // Y predictor unchanged from last subsequent block.
        assert_eq!(pred.y_past, 1056);
    }

    #[test]
    fn intra_subsequent_macroblock_uses_past_chain_no_reset() {
        // After macroblock 0 (intra) finalised at address 0,
        // macroblock 1 (intra) at address 1 has gap = 1 (not > 1)
        // → use y_past chain directly.
        let mut dct_zz = zeros();
        dct_zz[0] = 1;
        let mut pred = IntraDcPredictors::at_slice_start();

        // Macroblock 0 (slice-start reset fires).
        let _ = dequantize_intra_block(
            &dct_zz,
            1,
            &DEFAULT_INTRA_QUANT,
            IntraBlockKind::LuminanceFirst,
            &mut pred,
            0,
        )
        .unwrap();
        finalise_intra_macroblock(&mut pred, 0);
        // y_past = 1024 + 8 = 1032.
        assert_eq!(pred.y_past, 1032);

        // Macroblock 1 First luma block: gap = 1 - 0 = 1, not > 1
        // → use y_past = 1032. DC = 1032 + 8 = 1040.
        let r = dequantize_intra_block(
            &dct_zz,
            1,
            &DEFAULT_INTRA_QUANT,
            IntraBlockKind::LuminanceFirst,
            &mut pred,
            1,
        )
        .unwrap();
        assert_eq!(r[0][0], 1040);
    }
}
