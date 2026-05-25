//! Motion-vector reconstruction per ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262) §7.6.3.1, plus the §7.6.3.4 reset rules and the §7.6.3.7
//! chrominance scaling.
//!
//! Round 11 left the macroblock body at "the bits the syntax says are
//! present have been read into typed Option-tagged fields". Round 12 takes
//! the next coherent step: it ties the parsed `motion_code` /
//! `motion_residual` pairs together with the four motion-vector
//! predictors `PMV[r][s][t]` to compute the reconstructed luminance
//! motion vector `vector'[r][s][t]` (§7.6.3.1), with the spec's
//! wrap-around arithmetic and PMV-update side-effect. The §7.6.3.7
//! chrominance scaling that follows the luminance reconstruction is also
//! implemented (4:2:0 / 4:2:2 / 4:4:4).
//!
//! What this module does **not** cover:
//!
//! * The §7.6.3.3 inter-vector PMV-copy table (Tables 7-9 / 7-10).
//!   That table fires after every macroblock decode and needs the
//!   macroblock-loop driver to be in place; it's a small future round.
//! * §7.6.3.6 dual-prime additional arithmetic (deriving the
//!   opposite-parity vector from the decoded forward vector). Dual-prime
//!   `motion_code` / `motion_residual` / `dmvector` parsing is in
//!   round 11; the derived vector calculation is its own round once we
//!   have a `dmv_decision` engine.
//! * §7.6.3.9 concealment motion vectors (intra macroblocks with the
//!   `concealment_motion_vectors` flag set).
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)) §§7.6.3, 7.6.3.1, 7.6.3.4,
//! 7.6.3.7, and Table 7-7.

use crate::macroblock_modes::MvFormat;
use crate::motion_vector::MotionVector;
use crate::picture_header::PictureStructure;
use crate::sequence_extension::ChromaFormat;
use crate::{Error, Result};

/// `t` index per Table 7-7: horizontal (`0`) or vertical (`1`) component.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Component {
    /// Horizontal component (`t = 0`).
    Horizontal,
    /// Vertical component (`t = 1`).
    Vertical,
}

impl Component {
    /// `t` index value (`0` or `1`).
    pub fn index(self) -> usize {
        match self {
            Component::Horizontal => 0,
            Component::Vertical => 1,
        }
    }
}

/// `s` index per Table 7-7: forward (`0`) or backward (`1`) prediction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Forward prediction (`s = 0`).
    Forward,
    /// Backward prediction (`s = 1`). B-pictures only.
    Backward,
}

impl Direction {
    /// `s` index value (`0` or `1`).
    pub fn index(self) -> usize {
        match self {
            Direction::Forward => 0,
            Direction::Backward => 1,
        }
    }
}

/// `r` index per Table 7-7: which of the at most two stored motion
/// vectors per macroblock this is (`0` = first, `1` = second).
///
/// Per the Table 7-7 note, `r` also takes the values `2` and `3` for the
/// derived dual-prime vectors; those are computed by §7.6.3.6 and do not
/// have their own PMV slot, so this enum only carries the two
/// PMV-backed positions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VectorIndex {
    /// First motion vector in macroblock (`r = 0`).
    First,
    /// Second motion vector in macroblock (`r = 1`).
    Second,
}

impl VectorIndex {
    /// `r` index value (`0` or `1`).
    pub fn index(self) -> usize {
        match self {
            VectorIndex::First => 0,
            VectorIndex::Second => 1,
        }
    }
}

/// One reconstructed luminance motion-vector component, plus the PMV
/// slot value that was written back per §7.6.3.1.
///
/// `vector_prime` is `vector'[r][s][t]` — the spec's luminance vector in
/// half-sample units. `new_pmv` is the predictor value the spec assigns
/// to `PMV[r][s][t]` after the reconstruction; for the field-in-frame
/// vertical case (`mv_format == field && t == 1 && picture_structure ==
/// frame picture`) it is `vector_prime * 2`, otherwise it equals
/// `vector_prime`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReconstructedComponent {
    /// `vector'[r][s][t]` — half-sample luminance motion vector for the
    /// macroblock (§7.6.3.1).
    pub vector_prime: i32,
    /// Updated `PMV[r][s][t]` value the spec writes back at the end of
    /// the §7.6.3.1 procedure.
    pub new_pmv: i32,
    /// Cached `delta` (the bitstream-derived increment, post-sign,
    /// pre-prediction). Surfacing this lets callers and tests assert the
    /// half-sample increment without re-running the §7.6.3.1 arithmetic.
    pub delta: i32,
    /// Cached `range` (`32 * f`) for the wrap-around half-range
    /// `[low, high] = [-16*f, 16*f - 1]`. Useful for assertions and for
    /// chaining the §7.6.3.2 range-restriction check.
    pub range: i32,
}

/// The four `PMV[r][s][t]` motion-vector predictors per §7.6.3 (Table
/// 7-7), stored in half-sample units.
///
/// Index order is `[r][s][t]` matching the spec exactly: `r ∈ {0, 1}`,
/// `s ∈ {0, 1}` (forward / backward), `t ∈ {0, 1}` (horizontal /
/// vertical).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Pmv {
    /// Raw `[r][s][t]` PMV storage in half-sample units.
    pub values: [[[i32; 2]; 2]; 2],
}

impl Pmv {
    /// Construct a fresh PMV state with every slot zeroed — the value
    /// every PMV slot must hold at the start of a slice per §7.6.3.4 and
    /// at construction time.
    pub fn new() -> Self {
        Self::default()
    }

    /// Read `PMV[r][s][t]`.
    pub fn get(&self, r: VectorIndex, s: Direction, t: Component) -> i32 {
        self.values[r.index()][s.index()][t.index()]
    }

    /// Write `PMV[r][s][t]`.
    pub fn set(&mut self, r: VectorIndex, s: Direction, t: Component, value: i32) {
        self.values[r.index()][s.index()][t.index()] = value;
    }

    /// §7.6.3.4: zero every PMV slot. Fired at the start of each slice,
    /// at every non-concealment intra macroblock, and at the §7.6.3.4
    /// P-picture special cases (handled by the macroblock-loop driver).
    pub fn reset(&mut self) {
        self.values = [[[0i32; 2]; 2]; 2];
    }
}

/// §7.6.3.1: turn a parsed `(motion_code, motion_residual)` pair plus
/// an `f_code[s][t]` value into the per-component `delta` increment, in
/// half-sample units.
///
/// The spec's formula:
///
/// ```text
/// r_size = f_code - 1
/// f      = 1 << r_size
/// if (f == 1 || motion_code == 0)
///     delta = motion_code
/// else {
///     delta = (Abs(motion_code) - 1) * f + motion_residual + 1
///     if (motion_code < 0) delta = -delta
/// }
/// ```
///
/// Errors:
/// * [`Error::InvalidBitstream`] if `f_code` is outside the §6.3.11
///   `1..=9` range (a future scalable extension might extend the range,
///   but the value `15` is the "unused" sentinel and any other value is
///   a violation).
/// * [`Error::InvalidBitstream`] if a `motion_residual` field was
///   required by the formula but `None` was supplied (or the converse:
///   supplied when the formula does not consume it). This catches
///   upstream mis-parsing — the §6.2.5.2.1 syntax is unambiguous about
///   when the residual is present.
pub fn compute_delta(motion_code: i32, motion_residual: Option<u32>, f_code: u8) -> Result<i32> {
    if !(1..=9).contains(&f_code) {
        return Err(Error::InvalidBitstream(
            "compute_delta: f_code outside the §6.3.11 1..=9 range",
        ));
    }
    let r_size = u32::from(f_code - 1);
    let f: i32 = 1i32 << r_size; // r_size <= 8 so 1<<r_size fits i32 easily.

    if f == 1 || motion_code == 0 {
        if motion_residual.is_some() {
            return Err(Error::InvalidBitstream(
                "compute_delta: motion_residual present when §6.2.5.2.1 forbids it (f == 1 or motion_code == 0)",
            ));
        }
        Ok(motion_code)
    } else {
        let residual = motion_residual.ok_or(Error::InvalidBitstream(
            "compute_delta: motion_residual absent when §6.2.5.2.1 requires it (f != 1 && motion_code != 0)",
        ))?;
        let abs_code = motion_code.unsigned_abs() as i32;
        let mag = (abs_code - 1) * f + residual as i32 + 1;
        Ok(if motion_code < 0 { -mag } else { mag })
    }
}

/// The wrap-around half-range `[low, high]` that §7.6.3.1 enforces on
/// the reconstructed `vector'[r][s][t]`.
///
/// ```text
/// f      = 1 << (f_code - 1)
/// high   = 16 * f - 1
/// low    = -16 * f
/// range  = 32 * f
/// ```
///
/// Errors mirror [`compute_delta`].
pub fn vector_range(f_code: u8) -> Result<(i32, i32, i32)> {
    if !(1..=9).contains(&f_code) {
        return Err(Error::InvalidBitstream(
            "vector_range: f_code outside the §6.3.11 1..=9 range",
        ));
    }
    let f = 1i32 << u32::from(f_code - 1);
    let high = 16 * f - 1;
    let low = -16 * f;
    let range = 32 * f;
    Ok((low, high, range))
}

/// Whether the vertical-component half-prediction rule of §7.6.3.1
/// applies. This rule halves the PMV before adding `delta` (and doubles
/// the new vector before writing the PMV back) when the macroblock has
/// a field-format vector in a frame picture and we are reconstructing
/// the vertical component.
fn vertical_half_pred(mv_format: MvFormat, picture: PictureStructure, t: Component) -> bool {
    mv_format == MvFormat::Field && t == Component::Vertical && picture == PictureStructure::Frame
}

/// §7.6.3.1: reconstruct one luminance motion-vector component
/// `vector'[r][s][t]` from `(motion_code, motion_residual)`, the prior
/// `PMV[r][s][t]`, and the macroblock-level `mv_format` /
/// `picture_structure`.
///
/// Returns the spec's `delta`, the wrap-around `range`, the
/// reconstructed `vector'[r][s][t]`, and the value the spec writes back
/// into `PMV[r][s][t]`. The caller is responsible for actually storing
/// the new PMV value — this function is pure.
///
/// Bitstream-conformance check: the spec requires that the reconstructed
/// `delta`, `vector'[r][s][t]`, and `new_pmv` all lie inside
/// `[low, high]`. This function asserts that on the post-wrap value and
/// returns an [`Error::InvalidBitstream`] if the wrap result still lies
/// outside the range (which is impossible for a conforming stream).
pub fn reconstruct_component(
    motion_code: i32,
    motion_residual: Option<u32>,
    f_code: u8,
    prior_pmv: i32,
    mv_format: MvFormat,
    picture: PictureStructure,
    t: Component,
) -> Result<ReconstructedComponent> {
    let delta = compute_delta(motion_code, motion_residual, f_code)?;
    let (low, high, range) = vector_range(f_code)?;

    // §7.6.3.2: delta must lie in [low, high]. The bitstream is
    // non-conforming otherwise.
    if delta < low || delta > high {
        return Err(Error::InvalidBitstream(
            "reconstruct_component: delta outside [low, high] range (§7.6.3.2)",
        ));
    }

    let prediction = if vertical_half_pred(mv_format, picture, t) {
        // The spec's `DIV` is integer division toward negative infinity
        // (§4.3 of 13818-2: `a DIV b = floor(a / b)`). Rust's `i32::div_euclid`
        // matches that for positive divisors, but for `b = 2` it's identical
        // to `i32::div_euclid`.
        prior_pmv.div_euclid(2)
    } else {
        prior_pmv
    };

    let mut vector_prime = prediction + delta;
    if vector_prime < low {
        vector_prime += range;
    }
    if vector_prime > high {
        vector_prime -= range;
    }

    if vector_prime < low || vector_prime > high {
        return Err(Error::InvalidBitstream(
            "reconstruct_component: vector' still outside [low, high] after wrap (§7.6.3.1)",
        ));
    }

    let new_pmv = if vertical_half_pred(mv_format, picture, t) {
        vector_prime * 2
    } else {
        vector_prime
    };

    if new_pmv < low * 2 || new_pmv > high * 2 + 1 {
        // The vertical-half-pred path doubles the PMV, which extends the
        // permitted range; the non-vertical-half-pred path keeps `new_pmv
        // == vector_prime` which we already validated. So the only way
        // to fail this is a runaway doubling — guard defensively.
        return Err(Error::InvalidBitstream(
            "reconstruct_component: new PMV outside expected range",
        ));
    }

    Ok(ReconstructedComponent {
        vector_prime,
        new_pmv,
        delta,
        range,
    })
}

/// §7.6.3.1 driven from a parsed [`MotionVector`]: reconstruct both
/// components (`t = 0` and `t = 1`) of one of the macroblock's motion
/// vectors and update the corresponding two PMV slots.
///
/// `r` and `s` pick the PMV slots per Table 7-7. `f_code_horiz` /
/// `f_code_vert` are the relevant `f_code[s][t]` values. `mv_format` and
/// `picture_structure` come from the macroblock and picture headers
/// respectively; together they decide whether the vertical-half-pred
/// rule fires.
///
/// Returns the two reconstructed components in spec `[t = 0, t = 1]`
/// order; on the way back the new PMV values are written into `pmv`.
// §7.6.3.1's procedure is parameterised by exactly these eight pieces of
// state; bundling them into a context struct would just move the
// parameter list one level deeper without changing what the spec
// requires the caller to pass in.
#[allow(clippy::too_many_arguments)]
pub fn reconstruct_motion_vector(
    pmv: &mut Pmv,
    mv: &MotionVector,
    r: VectorIndex,
    s: Direction,
    f_code_horiz: u8,
    f_code_vert: u8,
    mv_format: MvFormat,
    picture: PictureStructure,
) -> Result<[ReconstructedComponent; 2]> {
    let horiz = reconstruct_component(
        i32::from(mv.motion_code_horiz),
        mv.motion_residual_horiz.map(u32::from),
        f_code_horiz,
        pmv.get(r, s, Component::Horizontal),
        mv_format,
        picture,
        Component::Horizontal,
    )?;
    pmv.set(r, s, Component::Horizontal, horiz.new_pmv);

    let vert = reconstruct_component(
        i32::from(mv.motion_code_vert),
        mv.motion_residual_vert.map(u32::from),
        f_code_vert,
        pmv.get(r, s, Component::Vertical),
        mv_format,
        picture,
        Component::Vertical,
    )?;
    pmv.set(r, s, Component::Vertical, vert.new_pmv);

    Ok([horiz, vert])
}

/// A reconstructed luminance motion vector, paired with the per-chroma
/// scaling that §7.6.3.7 derives for the surrounding picture's chroma
/// format. `vector_chroma` is `vector[r][s][t]` per Table 7-7.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScaledMotionVector {
    /// Luminance vector horizontal component (`vector'[r][s][0]`).
    pub luma_horiz: i32,
    /// Luminance vector vertical component (`vector'[r][s][1]`).
    pub luma_vert: i32,
    /// Chrominance vector horizontal component (`vector[r][s][0]`) after
    /// §7.6.3.7 scaling for the picture's chroma format.
    pub chroma_horiz: i32,
    /// Chrominance vector vertical component (`vector[r][s][1]`) after
    /// §7.6.3.7 scaling.
    pub chroma_vert: i32,
}

/// §7.6.3.7: scale a reconstructed luminance motion vector for the
/// chrominance components according to the picture's sub-sampling
/// structure.
///
/// 4:2:0 halves both components, 4:2:2 halves only the horizontal,
/// 4:4:4 leaves the vector unmodified. The spec writes the division
/// with the usual `/` operator; in 13818-2 §4.3, `/` is integer
/// division toward zero, which is exactly Rust's `i32::div`.
pub fn scale_chroma(luma_horiz: i32, luma_vert: i32, chroma: ChromaFormat) -> ScaledMotionVector {
    let (chroma_horiz, chroma_vert) = match chroma {
        ChromaFormat::Yuv420 => (luma_horiz / 2, luma_vert / 2),
        ChromaFormat::Yuv422 => (luma_horiz / 2, luma_vert),
        ChromaFormat::Yuv444 => (luma_horiz, luma_vert),
    };
    ScaledMotionVector {
        luma_horiz,
        luma_vert,
        chroma_horiz,
        chroma_vert,
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact §7.6.3.1 / §7.6.3.4 / §7.6.3.7 round-trips.
    use super::*;

    fn frame_picture() -> PictureStructure {
        PictureStructure::Frame
    }

    fn top_field_picture() -> PictureStructure {
        PictureStructure::TopField
    }

    // ---- §7.6.3.1 compute_delta ----

    #[test]
    fn compute_delta_zero_motion_code_returns_zero() {
        // f_code=1 (f=1): delta = motion_code, with motion_code = 0.
        let d = compute_delta(0, None, 1).unwrap();
        assert_eq!(d, 0);
    }

    #[test]
    fn compute_delta_f_code_one_passes_motion_code_through() {
        // f_code = 1 ⇒ f = 1 ⇒ delta = motion_code, no residual.
        for mc in -16..=16 {
            let d = compute_delta(mc, None, 1).expect("delta");
            assert_eq!(d, mc, "motion_code={mc}");
        }
    }

    #[test]
    fn compute_delta_motion_code_zero_with_higher_f_code_still_zero() {
        // motion_code = 0 ⇒ delta = 0 regardless of f_code, no residual.
        for f_code in 2..=9 {
            let d = compute_delta(0, None, f_code).expect("delta");
            assert_eq!(d, 0, "f_code={f_code}");
        }
    }

    #[test]
    fn compute_delta_f_code_two_positive_path() {
        // f_code=2 ⇒ r_size=1, f=2. motion_code=1, residual=0:
        //   delta = (1-1)*2 + 0 + 1 = 1
        let d = compute_delta(1, Some(0), 2).unwrap();
        assert_eq!(d, 1);
        // motion_code=2, residual=1:
        //   delta = (2-1)*2 + 1 + 1 = 4
        let d = compute_delta(2, Some(1), 2).unwrap();
        assert_eq!(d, 4);
    }

    #[test]
    fn compute_delta_f_code_two_negative_sign_flipped() {
        // f_code=2, motion_code=-2, residual=1 ⇒ magnitude = 4 ⇒ -4.
        let d = compute_delta(-2, Some(1), 2).unwrap();
        assert_eq!(d, -4);
    }

    #[test]
    fn compute_delta_max_magnitude_f_code_two() {
        // f_code=2 ⇒ r_size=1, residual range 0..=1. motion_code=16,
        // residual=1: delta = (16-1)*2 + 1 + 1 = 32. The 13818-2 §7.6.3.2
        // Table 7-8 entry for f_code=2 says vectors lie in [-16, +15,5];
        // here we just check the delta arithmetic on its own — the
        // range-clamp is exercised in reconstruct_component.
        let d = compute_delta(16, Some(1), 2).unwrap();
        assert_eq!(d, 32);
    }

    #[test]
    fn compute_delta_requires_residual_when_formula_uses_it() {
        // f_code=2 + motion_code=1 ⇒ residual is required by the spec.
        let err = compute_delta(1, None, 2).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn compute_delta_rejects_residual_when_formula_skips_it() {
        // motion_code=0 ⇒ residual must be absent.
        let err = compute_delta(0, Some(0), 5).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn compute_delta_rejects_out_of_range_f_code() {
        // f_code=0 forbidden, f_code=10..=15 reserved/unused.
        let err = compute_delta(1, Some(0), 0).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
        let err = compute_delta(1, Some(0), 10).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
        let err = compute_delta(1, Some(0), 15).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    // ---- §7.6.3.1 vector_range ----

    #[test]
    fn vector_range_f_code_one_is_pm_16() {
        let (low, high, range) = vector_range(1).unwrap();
        assert_eq!(low, -16);
        assert_eq!(high, 15);
        assert_eq!(range, 32);
    }

    #[test]
    fn vector_range_f_code_nine_is_pm_4096() {
        // f_code=9 ⇒ f = 1<<8 = 256, range = 32*256 = 8192,
        // [low, high] = [-4096, 4095].
        let (low, high, range) = vector_range(9).unwrap();
        assert_eq!(low, -4096);
        assert_eq!(high, 4095);
        assert_eq!(range, 8192);
    }

    #[test]
    fn vector_range_doubles_per_f_code_step() {
        // Each f_code increment doubles the range, per Table 7-8.
        let mut prev = vector_range(1).unwrap().2;
        for f_code in 2..=9 {
            let (_, _, range) = vector_range(f_code).unwrap();
            assert_eq!(range, prev * 2, "f_code={f_code}");
            prev = range;
        }
    }

    // ---- §7.6.3.1 reconstruct_component ----

    #[test]
    fn reconstruct_simple_no_prediction_no_wrap() {
        // f_code=1, mv_format=Frame in a Frame picture: prediction is
        // just prior PMV, no halving, no wrap when delta keeps the result
        // inside [-16, 15].
        let mut pmv = Pmv::new();
        let mv = MotionVector {
            motion_code_horiz: 3,
            motion_residual_horiz: None,
            dmvector_horiz: None,
            motion_code_vert: -2,
            motion_residual_vert: None,
            dmvector_vert: None,
            bit_position_after: 0,
        };
        let [h, v] = reconstruct_motion_vector(
            &mut pmv,
            &mv,
            VectorIndex::First,
            Direction::Forward,
            1,
            1,
            MvFormat::Frame,
            frame_picture(),
        )
        .unwrap();
        assert_eq!(h.vector_prime, 3);
        assert_eq!(h.new_pmv, 3);
        assert_eq!(v.vector_prime, -2);
        assert_eq!(v.new_pmv, -2);
        // PMV state is now (3, -2) for [0][0][:].
        assert_eq!(
            pmv.get(
                VectorIndex::First,
                Direction::Forward,
                Component::Horizontal
            ),
            3
        );
        assert_eq!(
            pmv.get(VectorIndex::First, Direction::Forward, Component::Vertical),
            -2
        );
    }

    #[test]
    fn reconstruct_chain_two_vectors_accumulates_prediction() {
        // First call lands vector_prime = 5 → PMV becomes 5.
        // Second call has motion_code = -1, delta = -1; vector_prime = 5 + (-1) = 4.
        let mut pmv = Pmv::new();
        let mv1 = MotionVector {
            motion_code_horiz: 5,
            motion_residual_horiz: None,
            dmvector_horiz: None,
            motion_code_vert: 0,
            motion_residual_vert: None,
            dmvector_vert: None,
            bit_position_after: 0,
        };
        reconstruct_motion_vector(
            &mut pmv,
            &mv1,
            VectorIndex::First,
            Direction::Forward,
            1,
            1,
            MvFormat::Frame,
            frame_picture(),
        )
        .unwrap();
        let mv2 = MotionVector {
            motion_code_horiz: -1,
            motion_residual_horiz: None,
            dmvector_horiz: None,
            motion_code_vert: 0,
            motion_residual_vert: None,
            dmvector_vert: None,
            bit_position_after: 0,
        };
        let [h, _v] = reconstruct_motion_vector(
            &mut pmv,
            &mv2,
            VectorIndex::First,
            Direction::Forward,
            1,
            1,
            MvFormat::Frame,
            frame_picture(),
        )
        .unwrap();
        assert_eq!(h.vector_prime, 4);
        assert_eq!(
            pmv.get(
                VectorIndex::First,
                Direction::Forward,
                Component::Horizontal
            ),
            4
        );
    }

    #[test]
    fn reconstruct_wrap_around_low() {
        // f_code=1 ⇒ range = 32. PMV = -15, delta = -3 ⇒ vector' = -18,
        // which is < low = -16 ⇒ wrap +32 → +14.
        let mut pmv = Pmv::new();
        pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Horizontal,
            -15,
        );
        let mv = MotionVector {
            motion_code_horiz: -3,
            motion_residual_horiz: None,
            dmvector_horiz: None,
            motion_code_vert: 0,
            motion_residual_vert: None,
            dmvector_vert: None,
            bit_position_after: 0,
        };
        let [h, _] = reconstruct_motion_vector(
            &mut pmv,
            &mv,
            VectorIndex::First,
            Direction::Forward,
            1,
            1,
            MvFormat::Frame,
            frame_picture(),
        )
        .unwrap();
        assert_eq!(h.vector_prime, 14);
        assert_eq!(h.new_pmv, 14);
    }

    #[test]
    fn reconstruct_wrap_around_high() {
        // f_code=1 ⇒ range = 32. PMV = 15, delta = 3 ⇒ vector' = 18,
        // which is > high = 15 ⇒ wrap -32 → -14.
        let mut pmv = Pmv::new();
        pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Horizontal,
            15,
        );
        let mv = MotionVector {
            motion_code_horiz: 3,
            motion_residual_horiz: None,
            dmvector_horiz: None,
            motion_code_vert: 0,
            motion_residual_vert: None,
            dmvector_vert: None,
            bit_position_after: 0,
        };
        let [h, _] = reconstruct_motion_vector(
            &mut pmv,
            &mv,
            VectorIndex::First,
            Direction::Forward,
            1,
            1,
            MvFormat::Frame,
            frame_picture(),
        )
        .unwrap();
        assert_eq!(h.vector_prime, -14);
        assert_eq!(h.new_pmv, -14);
    }

    #[test]
    fn reconstruct_vertical_half_pred_in_frame_picture_field_format() {
        // mv_format=Field + t==Vertical + picture=Frame ⇒ prediction is
        // PMV/2; PMV writeback is vector'*2.
        // PMV starts at 6 ⇒ prediction = 3. motion_code = 2, f_code=1 ⇒
        // delta = 2. vector' = 3 + 2 = 5. PMV writeback = 10.
        let mut pmv = Pmv::new();
        pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Vertical,
            6,
        );
        let mv = MotionVector {
            motion_code_horiz: 0,
            motion_residual_horiz: None,
            dmvector_horiz: None,
            motion_code_vert: 2,
            motion_residual_vert: None,
            dmvector_vert: None,
            bit_position_after: 0,
        };
        let [_, v] = reconstruct_motion_vector(
            &mut pmv,
            &mv,
            VectorIndex::First,
            Direction::Forward,
            1,
            1,
            MvFormat::Field,
            frame_picture(),
        )
        .unwrap();
        assert_eq!(v.vector_prime, 5);
        assert_eq!(v.new_pmv, 10);
        assert_eq!(
            pmv.get(VectorIndex::First, Direction::Forward, Component::Vertical),
            10
        );
    }

    #[test]
    fn reconstruct_vertical_half_pred_uses_floor_div_for_negative_pmv() {
        // §4.3 of 13818-2: a DIV b = floor(a/b). For PMV = -1, DIV 2 = -1
        // (not 0 as Rust's `/` would give); we use div_euclid which
        // matches floor-division for positive divisors.
        let mut pmv = Pmv::new();
        pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Vertical,
            -1,
        );
        let mv = MotionVector {
            motion_code_horiz: 0,
            motion_residual_horiz: None,
            dmvector_horiz: None,
            motion_code_vert: 0,
            motion_residual_vert: None,
            dmvector_vert: None,
            bit_position_after: 0,
        };
        let [_, v] = reconstruct_motion_vector(
            &mut pmv,
            &mv,
            VectorIndex::First,
            Direction::Forward,
            1,
            1,
            MvFormat::Field,
            frame_picture(),
        )
        .unwrap();
        // prediction = floor(-1/2) = -1; delta = 0; vector' = -1; PMV = -2.
        assert_eq!(v.vector_prime, -1);
        assert_eq!(v.new_pmv, -2);
    }

    #[test]
    fn reconstruct_no_half_pred_in_field_picture() {
        // picture=TopField + mv_format=Field + t==Vertical: half-pred does
        // NOT apply (only `picture == Frame` triggers it). PMV=6, delta=2,
        // vector' = 8, PMV = 8.
        let mut pmv = Pmv::new();
        pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Vertical,
            6,
        );
        let mv = MotionVector {
            motion_code_horiz: 0,
            motion_residual_horiz: None,
            dmvector_horiz: None,
            motion_code_vert: 2,
            motion_residual_vert: None,
            dmvector_vert: None,
            bit_position_after: 0,
        };
        let [_, v] = reconstruct_motion_vector(
            &mut pmv,
            &mv,
            VectorIndex::First,
            Direction::Forward,
            1,
            1,
            MvFormat::Field,
            top_field_picture(),
        )
        .unwrap();
        assert_eq!(v.vector_prime, 8);
        assert_eq!(v.new_pmv, 8);
    }

    #[test]
    fn reconstruct_no_half_pred_in_horizontal_component() {
        // t==Horizontal in frame picture with mv_format=Field: half-pred
        // does not apply (only the vertical component is halved).
        let mut pmv = Pmv::new();
        pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Horizontal,
            6,
        );
        let mv = MotionVector {
            motion_code_horiz: 2,
            motion_residual_horiz: None,
            dmvector_horiz: None,
            motion_code_vert: 0,
            motion_residual_vert: None,
            dmvector_vert: None,
            bit_position_after: 0,
        };
        let [h, _] = reconstruct_motion_vector(
            &mut pmv,
            &mv,
            VectorIndex::First,
            Direction::Forward,
            1,
            1,
            MvFormat::Field,
            frame_picture(),
        )
        .unwrap();
        assert_eq!(h.vector_prime, 8);
        assert_eq!(h.new_pmv, 8);
    }

    #[test]
    fn reconstruct_uses_distinct_pmv_slots_for_forward_backward() {
        // Forward and backward predictors are independent.
        let mut pmv = Pmv::new();
        pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Horizontal,
            4,
        );
        pmv.set(
            VectorIndex::First,
            Direction::Backward,
            Component::Horizontal,
            -4,
        );
        let fwd = MotionVector {
            motion_code_horiz: 1,
            motion_residual_horiz: None,
            dmvector_horiz: None,
            motion_code_vert: 0,
            motion_residual_vert: None,
            dmvector_vert: None,
            bit_position_after: 0,
        };
        let bwd = MotionVector {
            motion_code_horiz: 1,
            motion_residual_horiz: None,
            dmvector_horiz: None,
            motion_code_vert: 0,
            motion_residual_vert: None,
            dmvector_vert: None,
            bit_position_after: 0,
        };
        let [fh, _] = reconstruct_motion_vector(
            &mut pmv,
            &fwd,
            VectorIndex::First,
            Direction::Forward,
            1,
            1,
            MvFormat::Frame,
            frame_picture(),
        )
        .unwrap();
        let [bh, _] = reconstruct_motion_vector(
            &mut pmv,
            &bwd,
            VectorIndex::First,
            Direction::Backward,
            1,
            1,
            MvFormat::Frame,
            frame_picture(),
        )
        .unwrap();
        assert_eq!(fh.vector_prime, 5);
        assert_eq!(bh.vector_prime, -3);
        assert_eq!(
            pmv.get(
                VectorIndex::First,
                Direction::Forward,
                Component::Horizontal
            ),
            5
        );
        assert_eq!(
            pmv.get(
                VectorIndex::First,
                Direction::Backward,
                Component::Horizontal
            ),
            -3
        );
    }

    #[test]
    fn reconstruct_uses_distinct_pmv_slots_for_first_second_vector() {
        // r=0 and r=1 are independent slots (the 16x8 MC / Field-based
        // count==2 case).
        let mut pmv = Pmv::new();
        pmv.set(
            VectorIndex::First,
            Direction::Forward,
            Component::Horizontal,
            4,
        );
        pmv.set(
            VectorIndex::Second,
            Direction::Forward,
            Component::Horizontal,
            -4,
        );
        let mv = MotionVector {
            motion_code_horiz: 1,
            motion_residual_horiz: None,
            dmvector_horiz: None,
            motion_code_vert: 0,
            motion_residual_vert: None,
            dmvector_vert: None,
            bit_position_after: 0,
        };
        let [h0, _] = reconstruct_motion_vector(
            &mut pmv,
            &mv,
            VectorIndex::First,
            Direction::Forward,
            1,
            1,
            MvFormat::Frame,
            frame_picture(),
        )
        .unwrap();
        let [h1, _] = reconstruct_motion_vector(
            &mut pmv,
            &mv,
            VectorIndex::Second,
            Direction::Forward,
            1,
            1,
            MvFormat::Frame,
            frame_picture(),
        )
        .unwrap();
        assert_eq!(h0.vector_prime, 5);
        assert_eq!(h1.vector_prime, -3);
    }

    #[test]
    fn reconstruct_rejects_delta_outside_range() {
        // f_code=1 ⇒ range [-16, 15]. motion_code=16 + residual would
        // give delta = (16-1)*1 + 0 + 1 = 16, but for f=1 we take the
        // shortcut delta=motion_code=16. 16 > high=15 ⇒ delta-range error.
        let err = reconstruct_component(
            16,
            None,
            1,
            0,
            MvFormat::Frame,
            frame_picture(),
            Component::Horizontal,
        )
        .unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    // ---- §7.6.3.4 reset ----

    #[test]
    fn pmv_default_is_all_zeroes() {
        let pmv = Pmv::new();
        for r in [VectorIndex::First, VectorIndex::Second] {
            for s in [Direction::Forward, Direction::Backward] {
                for t in [Component::Horizontal, Component::Vertical] {
                    assert_eq!(pmv.get(r, s, t), 0);
                }
            }
        }
    }

    #[test]
    fn pmv_reset_clears_every_slot() {
        let mut pmv = Pmv::new();
        for r in [VectorIndex::First, VectorIndex::Second] {
            for s in [Direction::Forward, Direction::Backward] {
                for t in [Component::Horizontal, Component::Vertical] {
                    pmv.set(r, s, t, 7);
                }
            }
        }
        pmv.reset();
        for r in [VectorIndex::First, VectorIndex::Second] {
            for s in [Direction::Forward, Direction::Backward] {
                for t in [Component::Horizontal, Component::Vertical] {
                    assert_eq!(pmv.get(r, s, t), 0);
                }
            }
        }
    }

    // ---- §7.6.3.7 chroma scaling ----

    #[test]
    fn chroma_scale_420_halves_both() {
        let v = scale_chroma(10, -8, ChromaFormat::Yuv420);
        assert_eq!(v.chroma_horiz, 5);
        assert_eq!(v.chroma_vert, -4);
        assert_eq!(v.luma_horiz, 10);
        assert_eq!(v.luma_vert, -8);
    }

    #[test]
    fn chroma_scale_422_halves_only_horizontal() {
        let v = scale_chroma(10, -8, ChromaFormat::Yuv422);
        assert_eq!(v.chroma_horiz, 5);
        assert_eq!(v.chroma_vert, -8);
    }

    #[test]
    fn chroma_scale_444_is_identity() {
        let v = scale_chroma(10, -8, ChromaFormat::Yuv444);
        assert_eq!(v.chroma_horiz, 10);
        assert_eq!(v.chroma_vert, -8);
    }

    #[test]
    fn chroma_scale_uses_toward_zero_for_negative_odd() {
        // 13818-2 §4.3: `a/b` is integer division toward zero. Rust's
        // `i32::Div` matches that for non-zero divisors. Confirm with an
        // odd negative input.
        let v = scale_chroma(-3, -5, ChromaFormat::Yuv420);
        assert_eq!(v.chroma_horiz, -1); // -3/2 = -1 toward zero
        assert_eq!(v.chroma_vert, -2); // -5/2 = -2 toward zero
    }

    // ---- Index enums ----

    #[test]
    fn index_values_match_spec_table_7_7() {
        assert_eq!(VectorIndex::First.index(), 0);
        assert_eq!(VectorIndex::Second.index(), 1);
        assert_eq!(Direction::Forward.index(), 0);
        assert_eq!(Direction::Backward.index(), 1);
        assert_eq!(Component::Horizontal.index(), 0);
        assert_eq!(Component::Vertical.index(), 1);
    }

    #[test]
    fn debug_impls_smoke() {
        let pmv = Pmv::new();
        let s = format!("{pmv:?}");
        assert!(s.contains("Pmv"));
        let rc = ReconstructedComponent {
            vector_prime: 1,
            new_pmv: 1,
            delta: 1,
            range: 32,
        };
        let s = format!("{rc:?}");
        assert!(s.contains("ReconstructedComponent"));
    }
}
