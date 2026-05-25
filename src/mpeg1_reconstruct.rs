//! MPEG-1 (ISO/IEC 11172-2:1993) motion-vector reconstruction per §2.4.4.2
//! (forward MV in P-pictures, also the engine for §2.4.4.3 B-pictures)
//! and the §2.4.4.3 backward-MV variant.
//!
//! This module is the bridge from the parsed
//! [`Mpeg1MotionVector`] element (the four
//! `(code, r)` pairs read from the bitstream) to the integral
//! `right_for / down_for` (resp. `right_back / down_back`) whole-pel
//! offsets and the `right_half_for / down_half_for` half-pel flags
//! that the §2.4.4.2 luminance / chrominance pel-prediction
//! equations consume.
//!
//! The reconstruction is the §2.4.4.2 four-step formula:
//!
//! 1. **`r_size`/`f` derivation.** `r_size = f_code - 1` and
//!    `f = 1 << r_size`.
//! 2. **`complement_*_r` derivation.** When the residual `motion_*_r`
//!    is absent (`f == 1` or `code == 0`), the complement is `0`;
//!    otherwise `complement = f - 1 - motion_*_r`.
//! 3. **`*_little` / `*_big` derivation.** `*_little = code * f`,
//!    adjusted by `±complement` toward zero, with `*_big = *_little ∓
//!    32*f` providing the wrap-around alternative.
//! 4. **PMV update + wrap-around.** Compute `new_vector = prev +
//!    little`; if it lies in `[min, max]` (= `[-16*f, 16*f-1]`), use
//!    it; otherwise wrap by switching to `prev + big`. The new value
//!    becomes `recon_*_for` and is written back to
//!    `recon_*_for_prev`. The `full_pel_*_vector` flag, when set,
//!    left-shifts the final value by one (after the PMV update).
//!
//! The whole-pel / half-pel split is computed separately for
//! luminance and chrominance per the §2.4.4.2 closing table:
//!
//! ```text
//! luminance:                              chrominance:
//!   right_for      = recon_right_for >> 1   right_for      = (recon_right_for / 2) >> 1
//!   down_for       = recon_down_for  >> 1   down_for       = (recon_down_for  / 2) >> 1
//!   right_half_for = recon_right_for - 2*right_for
//!                                            right_half_for = recon_right_for/2 - 2*right_for
//! ```
//!
//! Note the spec deliberately uses `>>` (arithmetic right shift, i.e.
//! floored division by two) for luminance and `/` (C-style truncated
//! integer division) for the chrominance halving step — these differ
//! when `recon_*_for` is negative. Both are preserved bit-exact.
//!
//! §2.4.4.3 reuses the same algorithm for B-pictures; the only
//! differences are:
//!
//! * Backward direction substitutes `backward` for `forward` in
//!   every variable name (and uses `backward_f_code` /
//!   `full_pel_backward_vector`).
//! * When forward (resp. backward) MV data is *absent* from the
//!   current B-picture macroblock, the recon is `recon_*_for_prev`
//!   (resp. `recon_*_back_prev`) unchanged — *not* zero. That
//!   "carry-over on absence" is encoded by [`reconstruct_absent`].
//! * The previous reconstructed MVs are reset only at the start of
//!   a slice or immediately after an intra-coded macroblock — never
//!   at a "no-MV" non-intra macroblock as P-pictures do. The reset
//!   policy is the caller's responsibility (it depends on
//!   macroblock-type bits this module deliberately stays clear of).
//!
//! The spec's two non-conformance guards are also enforced:
//!
//! * `right_little != ±forward_f * 16` (the wrap arithmetic would
//!   land exactly on the wrap seam and the spec rules this
//!   ambiguous — flagged as [`Error::InvalidBitstream`]).
//! * `down_little != ±forward_f * 16`, ditto.
//!
//! Spec citations refer to ISO/IEC 11172-2:1993 (MPEG-1 Video) §2.4.3.4,
//! §2.4.3.6, §2.4.4.2, §2.4.4.3.

use crate::mpeg1_motion_vector::{Mpeg1MotionDirection, Mpeg1MotionVector};
use crate::{Error, Result};

/// One reconstructed MV component pair for one macroblock — the
/// output of [`reconstruct`] / [`reconstruct_absent`].
///
/// Holds the §2.4.4.2 `recon_right_for` / `recon_down_for` (in
/// **half-sample units** when `full_pel == false`, in **full-sample
/// units** doubled by the left-shift when `full_pel == true`), plus
/// the derived luminance and chrominance whole/half-pel split.
///
/// The two `_back` analogues for the §2.4.4.3 backward direction use
/// the same type (the spec is symmetric).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mpeg1ReconstructedMv {
    /// `recon_right_*` (horizontal, half-sample units after the
    /// optional `full_pel` shift).
    pub recon_right: i32,
    /// `recon_down_*` (vertical, half-sample units after the
    /// optional `full_pel` shift).
    pub recon_down: i32,
    /// Luminance whole-pel horizontal offset (`recon_right >> 1`).
    pub right_for_luma: i32,
    /// Luminance whole-pel vertical offset (`recon_down >> 1`).
    pub down_for_luma: i32,
    /// Luminance horizontal half-pel flag.
    pub right_half_for_luma: i32,
    /// Luminance vertical half-pel flag.
    pub down_half_for_luma: i32,
    /// Chrominance whole-pel horizontal offset
    /// (`(recon_right / 2) >> 1`).
    pub right_for_chroma: i32,
    /// Chrominance whole-pel vertical offset
    /// (`(recon_down / 2) >> 1`).
    pub down_for_chroma: i32,
    /// Chrominance horizontal half-pel flag
    /// (`recon_right / 2 - 2 * right_for_chroma`).
    pub right_half_for_chroma: i32,
    /// Chrominance vertical half-pel flag
    /// (`recon_down / 2 - 2 * down_for_chroma`).
    pub down_half_for_chroma: i32,
}

/// Per-direction predictor state: the `recon_*_for_prev` (resp.
/// `recon_*_back_prev`) pair the §2.4.4.2 PMV update reads and
/// writes once per reconstructed macroblock.
///
/// The values are stored *before* the `full_pel` left-shift — the
/// spec stores the half-sample-unit PMV regardless of whether the
/// frame's `full_pel_*_vector` flag is set. (The `full_pel` shift in
/// §2.4.4.2 is applied to `recon_*_for` *after* the PMV write-back.)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Mpeg1Predictor {
    /// `recon_right_*_prev` in half-sample units.
    pub recon_right_prev: i32,
    /// `recon_down_*_prev` in half-sample units.
    pub recon_down_prev: i32,
}

impl Mpeg1Predictor {
    /// Construct a zeroed predictor.
    ///
    /// Per §2.4.4.2, the predictors are zeroed at the start of every
    /// slice. P-pictures additionally zero them at every macroblock
    /// that contributes no forward motion-vector data (skipped or
    /// `macroblock_motion_forward == 0`). B-pictures zero them only
    /// at slice start and immediately after an intra-coded
    /// macroblock — *not* at a non-intra "no MV" macroblock. The
    /// caller is responsible for invoking [`Mpeg1Predictor::reset`]
    /// at the right moments.
    pub fn new() -> Self {
        Self::default()
    }

    /// Zero both components.
    pub fn reset(&mut self) {
        self.recon_right_prev = 0;
        self.recon_down_prev = 0;
    }
}

/// Per-direction `<dir>_f_code` and `full_pel_<dir>_vector` —
/// the picture-header fields the §2.4.4.2 reconstruction needs in
/// addition to the parsed motion-vector element.
///
/// `f_code` is the `forward_f_code` (`Forward`) or
/// `backward_f_code` (`Backward`) from the surrounding
/// `picture_header()` (§2.4.2.3 / §2.4.3.4): `1..=7`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mpeg1FrameMvContext {
    /// `forward_f_code` or `backward_f_code`. Valid range: `1..=7`.
    pub f_code: u8,
    /// `full_pel_forward_vector` or `full_pel_backward_vector` flag
    /// from the picture header — when `true`, the spec's final
    /// `recon_*_for = recon_*_for << 1` step applies (doubling the
    /// half-pel-unit result to integer-pel-unit).
    pub full_pel: bool,
}

/// Reconstruct one direction's `(right, down)` motion vector for the
/// current macroblock per §2.4.4.2 (P-picture forward, and B-picture
/// forward or backward when MV data *is* present for that direction).
///
/// `predictor` is mutated in-place: the §2.4.4.2 PMV update writes
/// the post-wrap `recon_*` (in half-sample units, *before* the
/// `full_pel` shift) back into `recon_*_for_prev`.
///
/// Returns a [`Mpeg1ReconstructedMv`] with `recon_right`/`recon_down`
/// in the units the §2.4.4.2 pel-prediction equations consume
/// (half-sample-unit when `full_pel == false`, full-sample-unit when
/// `full_pel == true`).
///
/// Errors:
/// * [`Error::InvalidBitstream`] when the parsed
///   [`Mpeg1MotionVector::direction`] does not match `direction`
///   (the caller mis-routed the element).
/// * [`Error::InvalidBitstream`] when `f_code` is outside the
///   §2.4.3.4 `1..=7` range.
/// * [`Error::InvalidBitstream`] when the §2.4.4.2 conformance
///   guards on `*_little != ±forward_f * 16` are violated.
/// * [`Error::InvalidBitstream`] when a residual is present in the
///   parsed element while §2.4.3.6 forbids it (or absent while the
///   same clause requires it).
pub fn reconstruct(
    mv: &Mpeg1MotionVector,
    ctx: Mpeg1FrameMvContext,
    predictor: &mut Mpeg1Predictor,
    direction: Mpeg1MotionDirection,
) -> Result<Mpeg1ReconstructedMv> {
    if mv.direction != direction {
        return Err(Error::InvalidBitstream(
            "mpeg1_reconstruct: parsed Mpeg1MotionVector direction does not match the requested direction",
        ));
    }
    if !(1..=7).contains(&ctx.f_code) {
        return Err(Error::InvalidBitstream(
            "mpeg1_reconstruct: f_code outside the §2.4.3.4 1..=7 range",
        ));
    }
    let r_size = u32::from(ctx.f_code - 1);
    let f: i32 = 1i32 << r_size; // r_size ∈ 0..=6 so f ∈ 1..=64.

    // Horizontal component.
    let (recon_right, new_right_prev) = reconstruct_component(
        i32::from(mv.horizontal_code),
        mv.horizontal_r,
        f,
        predictor.recon_right_prev,
        ctx.full_pel,
    )?;
    predictor.recon_right_prev = new_right_prev;

    // Vertical component.
    let (recon_down, new_down_prev) = reconstruct_component(
        i32::from(mv.vertical_code),
        mv.vertical_r,
        f,
        predictor.recon_down_prev,
        ctx.full_pel,
    )?;
    predictor.recon_down_prev = new_down_prev;

    Ok(make_split(recon_right, recon_down))
}

/// §2.4.4.3 B-picture special case: the current macroblock carries
/// *no* motion-vector data for this direction (the
/// `macroblock_motion_<dir>` flag is `0`).
///
/// The spec text (§2.4.4.3 ¶1, mirrored in ¶2 for backward) reads:
///
/// > If no forward motion vector data exists for the current
/// > macroblock, the motion vectors shall be obtained by:
/// > `recon_right_for = recon_right_for_prev,
/// >  recon_down_for  = recon_down_for_prev.`
///
/// i.e. the recon is the predictor unchanged. The predictor itself
/// is *not* modified.
///
/// The `full_pel` flag still applies the post-PMV left-shift to the
/// recon output (matching the §2.4.4.2 sequence "PMV write-back
/// happens before `full_pel` shift"); the predictor stays in
/// half-sample units. (In practice `full_pel == true` is the
/// MPEG-1-only "predictor was always in integer-pel units when
/// stored" pre-history — but the spec's algebra is consistent if we
/// keep the predictor in the natural pre-shift unit.)
///
/// Note: this is the **B-picture** carry-over rule. P-pictures (per
/// §2.4.4.2 ¶3) reset both the recon and the predictor to zero for
/// the same "no MV" condition — the caller distinguishes the two
/// cases with [`reconstruct_zero`] (P-picture) vs
/// [`reconstruct_absent`] (B-picture).
pub fn reconstruct_absent(
    ctx: Mpeg1FrameMvContext,
    predictor: &Mpeg1Predictor,
) -> Mpeg1ReconstructedMv {
    let recon_right = if ctx.full_pel {
        predictor.recon_right_prev << 1
    } else {
        predictor.recon_right_prev
    };
    let recon_down = if ctx.full_pel {
        predictor.recon_down_prev << 1
    } else {
        predictor.recon_down_prev
    };
    make_split(recon_right, recon_down)
}

/// §2.4.4.2 P-picture special case: the current macroblock carries
/// *no* forward motion-vector data (skipped or
/// `macroblock_motion_forward == 0`).
///
/// The spec text (§2.4.4.2 ¶3): "If no forward motion vector data
/// exists for the current macroblock (either because it was skipped
/// or `macroblock_motion_forward == 0`), the motion vectors shall be
/// set to zero." The same paragraph also resets the PMV state to
/// zero ("`recon_right_for_prev` and `recon_down_for_prev` shall be
/// set to zero").
///
/// This helper zeroes both the returned recon and the in-out
/// `predictor`.
pub fn reconstruct_zero(predictor: &mut Mpeg1Predictor) -> Mpeg1ReconstructedMv {
    predictor.reset();
    make_split(0, 0)
}

/// Core §2.4.4.2 single-component reconstruction. Returns
/// `(recon_after_full_pel_shift, new_prev_before_full_pel_shift)`.
fn reconstruct_component(
    motion_code: i32,
    motion_r: Option<u8>,
    f: i32,
    prev: i32,
    full_pel: bool,
) -> Result<(i32, i32)> {
    // §2.4.4.2 complement derivation. The spec gates the
    // `complement = f - 1 - r` formula on `f != 1 && code != 0` (when
    // the residual is present). When that gate is false the
    // complement is forced to zero and any supplied `motion_r` is a
    // bitstream violation (mis-parsed upstream).
    let absent_residual = f == 1 || motion_code == 0;
    let complement = if absent_residual {
        if motion_r.is_some() {
            return Err(Error::InvalidBitstream(
                "mpeg1_reconstruct: residual present when §2.4.3.6 forbids it (f == 1 or code == 0)",
            ));
        }
        0i32
    } else {
        let r = motion_r.ok_or(Error::InvalidBitstream(
            "mpeg1_reconstruct: residual absent when §2.4.3.6 requires it (f != 1 && code != 0)",
        ))? as i32;
        // r ∈ 0..(1 << r_size) so f - 1 - r ∈ [-(f-1)..(f-1)] which
        // fits in i32 trivially.
        f - 1 - r
    };

    // §2.4.4.2 `*_little` / `*_big` formula.
    let mut little = motion_code * f;
    let big;
    if little == 0 {
        big = 0;
    } else if little > 0 {
        little -= complement;
        big = little - (32 * f);
    } else {
        little += complement;
        big = little + (32 * f);
    }

    // §2.4.4.2 conformance guard. The wrap-around arithmetic would
    // land ambiguously on the seam value `±f * 16`.
    if little == f * 16 || little == -f * 16 {
        return Err(Error::InvalidBitstream(
            "mpeg1_reconstruct: little hits the ±f*16 wrap seam (§2.4.4.2 conformance guard)",
        ));
    }

    let max = 16 * f - 1;
    let min = -16 * f;
    let candidate = prev + little;
    let recon_unshifted = if candidate <= max && candidate >= min {
        candidate
    } else {
        prev + big
    };

    // §2.4.4.2: the PMV write-back happens before the `full_pel`
    // shift; the predictor state is therefore stored in half-sample
    // units regardless of `full_pel`.
    let new_prev = recon_unshifted;

    let recon = if full_pel {
        recon_unshifted << 1
    } else {
        recon_unshifted
    };

    Ok((recon, new_prev))
}

/// Derive the §2.4.4.2 closing table — luminance and chrominance
/// whole/half-pel splits — from the post-shift `recon_right` /
/// `recon_down`.
fn make_split(recon_right: i32, recon_down: i32) -> Mpeg1ReconstructedMv {
    // Luminance: `right_for = recon_right >> 1`. Rust `>>` on `i32`
    // is arithmetic, i.e. floored division by two.
    let right_for_luma = recon_right >> 1;
    let down_for_luma = recon_down >> 1;
    let right_half_for_luma = recon_right - 2 * right_for_luma;
    let down_half_for_luma = recon_down - 2 * down_for_luma;

    // Chrominance: spec uses C-style truncated division for the
    // outer `/2` (toward zero) then arithmetic `>>1` (floored) for
    // the half-pel reduction. Rust `i32 / 2` is truncated toward
    // zero, matching C.
    let chroma_right_half = recon_right / 2;
    let chroma_down_half = recon_down / 2;
    let right_for_chroma = chroma_right_half >> 1;
    let down_for_chroma = chroma_down_half >> 1;
    let right_half_for_chroma = chroma_right_half - 2 * right_for_chroma;
    let down_half_for_chroma = chroma_down_half - 2 * down_for_chroma;

    Mpeg1ReconstructedMv {
        recon_right,
        recon_down,
        right_for_luma,
        down_for_luma,
        right_half_for_luma,
        down_half_for_luma,
        right_for_chroma,
        down_for_chroma,
        right_half_for_chroma,
        down_half_for_chroma,
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact tests pinning the §2.4.4.2 / §2.4.4.3
    //! reconstruction against worked examples.
    use super::*;
    use crate::mpeg1_motion_vector::Mpeg1MotionDirection;

    fn mv(
        direction: Mpeg1MotionDirection,
        hc: i8,
        hr: Option<u8>,
        vc: i8,
        vr: Option<u8>,
    ) -> Mpeg1MotionVector {
        Mpeg1MotionVector {
            direction,
            horizontal_code: hc,
            horizontal_r: hr,
            vertical_code: vc,
            vertical_r: vr,
            bit_position_after: 0,
        }
    }

    #[test]
    fn f_code_one_no_residual_zero_code_zero_recon() {
        // f_code = 1 → f = 1. code = 0 everywhere → no residual.
        // Zero PMV → zero recon.
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 1,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Forward, 0, None, 0, None);
        let rc = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap();
        assert_eq!(rc.recon_right, 0);
        assert_eq!(rc.recon_down, 0);
        assert_eq!(rc.right_for_luma, 0);
        assert_eq!(rc.down_for_luma, 0);
        assert_eq!(rc.right_half_for_luma, 0);
        assert_eq!(rc.down_half_for_luma, 0);
        assert_eq!(rc.right_for_chroma, 0);
        assert_eq!(rc.right_half_for_chroma, 0);
        assert_eq!(pred.recon_right_prev, 0);
        assert_eq!(pred.recon_down_prev, 0);
    }

    #[test]
    fn f_code_one_nonzero_code_uses_code_as_little() {
        // f_code = 1 → f = 1, no residual. little = code * 1 = code.
        // Starting PMV 0, code = +1 → recon = 1 (half-sample units).
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 1,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Forward, 1, None, -2, None);
        let rc = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap();
        assert_eq!(rc.recon_right, 1);
        assert_eq!(rc.recon_down, -2);
        // luma half-pel split: right >> 1 = 0, half = 1
        assert_eq!(rc.right_for_luma, 0);
        assert_eq!(rc.right_half_for_luma, 1);
        // down: -2 >> 1 = -1, half = 0
        assert_eq!(rc.down_for_luma, -1);
        assert_eq!(rc.down_half_for_luma, 0);
        // PMV updated in half-sample units, pre-full_pel-shift.
        assert_eq!(pred.recon_right_prev, 1);
        assert_eq!(pred.recon_down_prev, -2);
    }

    #[test]
    fn f_code_two_residual_present_positive_code() {
        // f_code = 2 → r_size = 1, f = 2.
        // code = +3, r = 1 → complement = f - 1 - r = 2 - 1 - 1 = 0.
        // little = 3 * 2 - 0 = 6. prev = 0 → new = 6 inside [-32, 31].
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 2,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Forward, 3, Some(1), 0, None);
        let rc = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap();
        assert_eq!(rc.recon_right, 6);
        assert_eq!(pred.recon_right_prev, 6);
    }

    #[test]
    fn f_code_two_residual_present_negative_code() {
        // code = -3, r = 1, f = 2 → complement = 0.
        // little = -3 * 2 + 0 = -6.
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 2,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Forward, -3, Some(1), 0, None);
        let rc = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap();
        assert_eq!(rc.recon_right, -6);
        assert_eq!(pred.recon_right_prev, -6);
    }

    #[test]
    fn f_code_two_complement_nonzero() {
        // code = +3, r = 0, f = 2 → complement = 2 - 1 - 0 = 1.
        // little = 3 * 2 - 1 = 5.
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 2,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Forward, 3, Some(0), 0, None);
        let rc = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap();
        assert_eq!(rc.recon_right, 5);
    }

    #[test]
    fn pmv_accumulates_across_macroblocks() {
        // Two predictive macroblocks in a row: PMV must carry over.
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 1,
            full_pel: false,
        };
        let _ = reconstruct(
            &mv(Mpeg1MotionDirection::Forward, 5, None, 0, None),
            ctx,
            &mut pred,
            Mpeg1MotionDirection::Forward,
        )
        .unwrap();
        assert_eq!(pred.recon_right_prev, 5);
        let rc = reconstruct(
            &mv(Mpeg1MotionDirection::Forward, 3, None, 0, None),
            ctx,
            &mut pred,
            Mpeg1MotionDirection::Forward,
        )
        .unwrap();
        assert_eq!(rc.recon_right, 8);
        assert_eq!(pred.recon_right_prev, 8);
    }

    #[test]
    fn wrap_around_high_to_low() {
        // f_code = 1 → f = 1. Range is [-16, 15], wrap = 32.
        // prev = 10, code = +10 → little = 10. candidate = 20 > 15 → wrap to big.
        // big = little - 32 = -22. recon = prev + big = -12.
        let mut pred = Mpeg1Predictor {
            recon_right_prev: 10,
            recon_down_prev: 0,
        };
        let ctx = Mpeg1FrameMvContext {
            f_code: 1,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Forward, 10, None, 0, None);
        let rc = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap();
        assert_eq!(rc.recon_right, -12);
        assert_eq!(pred.recon_right_prev, -12);
    }

    #[test]
    fn wrap_around_low_to_high() {
        // prev = -10, code = -10, f = 1 → little = -10. candidate = -20 < -16 → wrap.
        // big = little + 32 = 22. recon = prev + big = 12.
        let mut pred = Mpeg1Predictor {
            recon_right_prev: -10,
            recon_down_prev: 0,
        };
        let ctx = Mpeg1FrameMvContext {
            f_code: 1,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Forward, -10, None, 0, None);
        let rc = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap();
        assert_eq!(rc.recon_right, 12);
    }

    #[test]
    fn full_pel_doubles_recon_but_not_predictor() {
        // f_code = 1, full_pel = true. code = +5, prev = 0.
        // little = 5, recon_unshifted = 5, predictor stores 5,
        // recon (post-shift) = 10.
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 1,
            full_pel: true,
        };
        let element = mv(Mpeg1MotionDirection::Forward, 5, None, 0, None);
        let rc = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap();
        assert_eq!(rc.recon_right, 10);
        assert_eq!(pred.recon_right_prev, 5);
        // luma split: 10 >> 1 = 5, half = 0
        assert_eq!(rc.right_for_luma, 5);
        assert_eq!(rc.right_half_for_luma, 0);
    }

    #[test]
    fn wrap_seam_conformance_guard_positive() {
        // §2.4.4.2: "right_little shall not equal forward_f * 16."
        // To land exactly on the seam at f_code = 2 (f = 2,
        // seam = +32) we need `little = code * f - complement = 32`
        // with code > 0. Pick code = +16, r = 1 → complement = 0,
        // little = 16 * 2 - 0 = 32 — exact seam. Should reject.
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 2,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Forward, 16, Some(1), 0, None);
        let err = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn wrap_seam_conformance_guard_negative() {
        // Symmetric: code = -16, r = 1, f = 2, complement = 0.
        // little = -16 * 2 + 0 = -32 = -f * 16. Seam hit.
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 2,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Forward, -16, Some(1), 0, None);
        let err = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_direction_mismatch() {
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 1,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Backward, 0, None, 0, None);
        let err = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_f_code_zero() {
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 0,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Forward, 0, None, 0, None);
        let err = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_f_code_above_seven() {
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 8,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Forward, 0, None, 0, None);
        let err = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_residual_present_when_f_one() {
        // f_code = 1 → §2.4.3.6 says no residual. If one is supplied,
        // the upstream parser has malformed the element.
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 1,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Forward, 0, Some(0), 0, None);
        let err = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_residual_present_when_code_zero() {
        // f_code = 2, code = 0 → residual must be absent.
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 2,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Forward, 0, Some(0), 0, None);
        let err = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_residual_absent_when_required() {
        // f_code = 2, code = +3 → §2.4.3.6 demands a residual; if
        // omitted, the parser has truncated the element.
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 2,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Forward, 3, None, 0, None);
        let err = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn reconstruct_zero_zeros_predictor() {
        // §2.4.4.2 ¶3: "no MV data" path resets both recon and PMV.
        let mut pred = Mpeg1Predictor {
            recon_right_prev: 42,
            recon_down_prev: -7,
        };
        let rc = reconstruct_zero(&mut pred);
        assert_eq!(rc.recon_right, 0);
        assert_eq!(rc.recon_down, 0);
        assert_eq!(pred.recon_right_prev, 0);
        assert_eq!(pred.recon_down_prev, 0);
    }

    #[test]
    fn reconstruct_absent_b_picture_carries_pmv() {
        // §2.4.4.3 carry-over: recon = prev, predictor unchanged.
        let pred = Mpeg1Predictor {
            recon_right_prev: 5,
            recon_down_prev: -3,
        };
        let ctx = Mpeg1FrameMvContext {
            f_code: 1,
            full_pel: false,
        };
        let rc = reconstruct_absent(ctx, &pred);
        assert_eq!(rc.recon_right, 5);
        assert_eq!(rc.recon_down, -3);
        // predictor stays put (caller didn't move it).
        assert_eq!(pred.recon_right_prev, 5);
        assert_eq!(pred.recon_down_prev, -3);
    }

    #[test]
    fn reconstruct_absent_applies_full_pel_shift() {
        // Even on the absence path, full_pel scales the recon output.
        let pred = Mpeg1Predictor {
            recon_right_prev: 5,
            recon_down_prev: -3,
        };
        let ctx = Mpeg1FrameMvContext {
            f_code: 1,
            full_pel: true,
        };
        let rc = reconstruct_absent(ctx, &pred);
        assert_eq!(rc.recon_right, 10);
        assert_eq!(rc.recon_down, -6);
    }

    #[test]
    fn luminance_chrominance_split_negative_value() {
        // Spec deliberately uses `>>` for luma and `/` for chroma.
        // For negative recon they differ.
        // Take recon_right = -3.
        // Luma: -3 >> 1 = -2 (floored), half = -3 - 2*(-2) = 1.
        // Chroma: -3/2 = -1 (trunc-toward-zero), -1 >> 1 = -1
        //         (floored). half = -1 - 2*(-1) = 1.
        let split = make_split(-3, 0);
        assert_eq!(split.right_for_luma, -2);
        assert_eq!(split.right_half_for_luma, 1);
        assert_eq!(split.right_for_chroma, -1);
        assert_eq!(split.right_half_for_chroma, 1);
    }

    #[test]
    fn luminance_chrominance_split_positive_value() {
        // recon_right = +5.
        // Luma: 5 >> 1 = 2, half = 5 - 4 = 1.
        // Chroma: 5 / 2 = 2, 2 >> 1 = 1. half = 2 - 2 = 0.
        let split = make_split(5, 0);
        assert_eq!(split.right_for_luma, 2);
        assert_eq!(split.right_half_for_luma, 1);
        assert_eq!(split.right_for_chroma, 1);
        assert_eq!(split.right_half_for_chroma, 0);
    }

    #[test]
    fn backward_direction_independent_predictor() {
        // §2.4.4.3 backward uses its own predictor independently of
        // forward.
        let mut pred = Mpeg1Predictor::new();
        let ctx = Mpeg1FrameMvContext {
            f_code: 1,
            full_pel: false,
        };
        let element = mv(Mpeg1MotionDirection::Backward, 7, None, -4, None);
        let rc = reconstruct(&element, ctx, &mut pred, Mpeg1MotionDirection::Backward).unwrap();
        assert_eq!(rc.recon_right, 7);
        assert_eq!(rc.recon_down, -4);
        assert_eq!(pred.recon_right_prev, 7);
        assert_eq!(pred.recon_down_prev, -4);
    }

    #[test]
    fn predictor_reset_zeroes_state() {
        let mut pred = Mpeg1Predictor {
            recon_right_prev: 11,
            recon_down_prev: -9,
        };
        pred.reset();
        assert_eq!(pred.recon_right_prev, 0);
        assert_eq!(pred.recon_down_prev, 0);
    }
}
