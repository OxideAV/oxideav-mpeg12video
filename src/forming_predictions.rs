//! §7.6.4 Forming predictions per ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262), page 88–89 of the 1994/1995 base text — the
//! integer-and-half-pel sample reader that turns a fully-reconstructed
//! motion vector `vector'[r][s][1:0]` into a pel-prediction block.
//!
//! ## What §7.6.4 specifies
//!
//! A reconstructed motion vector has half-pel accuracy. The spec
//! splits each two-component motion vector `vector'[r][s][1:0]` into:
//!
//! ```text
//! for (t = 0; t < 2; t++) {
//!     int_vec[t] = vector[r][s][t] DIV 2;
//!     if ((vector[r][s][t] - (2 * int_vec[t])) != 0)
//!         half_flag[t] = 1;
//!     else
//!         half_flag[t] = 0;
//! }
//! ```
//!
//! `DIV` here is the §4.1 integer division with truncation of the
//! result toward minus infinity (`3 DIV 2 = 1`, `-3 DIV 2 = -2`).
//! The `half_flag[t]` is therefore the parity of the original vector
//! (`half_flag = 1` iff the vector is odd), and `int_vec[t]` is the
//! floor-half integer offset.
//!
//! For every pel `pel_pred[y][x]` in the prediction block the spec
//! reads one, two or four samples from the reference and combines
//! them by the `//` operator (§4.1 integer division with rounding to
//! the nearest integer, half-integer values rounded away from zero —
//! `3//2 = 2`, `-3//2 = -2`):
//!
//! ```text
//! if  !half_flag[0] && !half_flag[1]
//!     pel_pred[y][x] = pel_ref[y + int_vec[1]    ][x + int_vec[0]    ];
//! if  !half_flag[0] &&  half_flag[1]
//!     pel_pred[y][x] = ( pel_ref[y + int_vec[1]  ][x + int_vec[0]    ]
//!                      + pel_ref[y + int_vec[1]+1][x + int_vec[0]    ] ) // 2;
//! if   half_flag[0] && !half_flag[1]
//!     pel_pred[y][x] = ( pel_ref[y + int_vec[1]  ][x + int_vec[0]    ]
//!                      + pel_ref[y + int_vec[1]  ][x + int_vec[0]+1  ] ) // 2;
//! if   half_flag[0] &&  half_flag[1]
//!     pel_pred[y][x] = ( pel_ref[y + int_vec[1]  ][x + int_vec[0]    ]
//!                      + pel_ref[y + int_vec[1]  ][x + int_vec[0]+1  ]
//!                      + pel_ref[y + int_vec[1]+1][x + int_vec[0]    ]
//!                      + pel_ref[y + int_vec[1]+1][x + int_vec[0]+1  ] ) // 4;
//! ```
//!
//! Index convention: `t = 0` is the horizontal axis (x), `t = 1` is
//! the vertical axis (y); positive vector components mean the
//! prediction sample lies right of / below the current sample.
//!
//! Since the integer divisions of two and four positive sums never
//! generate half-integer ties (sum-of-2 is half-integer only when the
//! sum is odd; sum-of-4 similarly), the spec's `//` operator on a sum
//! of unsigned 8-bit samples is identical to the canonical
//! "add half the divisor, integer divide" rule `(sum + d/2) / d`,
//! which is the form this module uses.
//!
//! ## What this module provides
//!
//! * [`HalfPattern`] — the four enumerated outcomes of the
//!   `(half_flag[0], half_flag[1])` pair.
//! * [`SplitVector`] — the per-axis `int_vec` and `half_flag` derived
//!   by [`split_vector`], plus the [`SplitVector::pattern`]
//!   convenience accessor.
//! * [`ReferencePlane`] — a borrowed view over a single reference
//!   sample plane with explicit `(width, height)` so that
//!   [`ReferencePlane::sample`] can clip out-of-bounds reads to the
//!   nearest edge sample (the standard pad-to-edge convention every
//!   MPEG decoder relies on for motion vectors that reach past the
//!   frame boundary; the spec leaves the behaviour outside the
//!   reference picture undefined — see [`BoundaryMode`]).
//! * [`predict_block`] — the inner loop, reading a `width × height`
//!   prediction block at `(top_left_x, top_left_y)` from the
//!   reference plane.
//!
//! The block geometry is intentionally a free `(width, height)` pair
//! so that the §7.6.5 prediction-mode table (16×16 frame, 16×8 MC,
//! 16×16 field, 8×8 4:2:0 chroma, 8×16 4:2:2 chroma, …) can drive
//! this loop directly without per-mode duplication.
//!
//! Spec citations refer to **ISO/IEC 13818-2 (H.262) §7.6.4** plus
//! the §4.1 arithmetic operators (`DIV`, `//`).

use crate::pmv::{Component, ReconstructedComponent};

/// The four `(half_flag[0], half_flag[1])` outcomes that drive the
/// §7.6.4 sample-reading switch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HalfPattern {
    /// `(half_flag[0], half_flag[1]) = (0, 0)` — single-sample read,
    /// `pel_pred[y][x] = pel_ref[y + int_vec[1]][x + int_vec[0]]`.
    Integer,
    /// `(half_flag[0], half_flag[1]) = (1, 0)` — average of two
    /// horizontally adjacent samples.
    HalfHorizontal,
    /// `(half_flag[0], half_flag[1]) = (0, 1)` — average of two
    /// vertically adjacent samples.
    HalfVertical,
    /// `(half_flag[0], half_flag[1]) = (1, 1)` — bilinear average of
    /// the 2×2 neighbourhood.
    HalfBoth,
}

impl HalfPattern {
    /// Map a `(half_flag[0], half_flag[1])` pair to the matching
    /// pattern. Each flag is `true` iff the corresponding vector
    /// component was odd (see [`split_vector`]).
    pub fn from_flags(half_horiz: bool, half_vert: bool) -> Self {
        match (half_horiz, half_vert) {
            (false, false) => Self::Integer,
            (true, false) => Self::HalfHorizontal,
            (false, true) => Self::HalfVertical,
            (true, true) => Self::HalfBoth,
        }
    }

    /// `(half_flag[0], half_flag[1])` — the raw pair this pattern
    /// represents, with `horiz` first to match the §7.6.4 indexing
    /// (`t = 0` is horizontal).
    pub fn flags(self) -> (bool, bool) {
        match self {
            Self::Integer => (false, false),
            Self::HalfHorizontal => (true, false),
            Self::HalfVertical => (false, true),
            Self::HalfBoth => (true, true),
        }
    }

    /// `true` when at least one component is half-pel — i.e. the
    /// prediction is not just a direct copy.
    pub fn is_half_pel(self) -> bool {
        !matches!(self, Self::Integer)
    }
}

/// Per-axis split of a single motion-vector component into
/// `int_vec[t]` (the `DIV 2` quotient) and `half_flag[t]` (the
/// `true` iff the original component was odd) per §7.6.4 page 88.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ComponentSplit {
    /// `int_vec[t] = vector[r][s][t] DIV 2`, with `DIV` being the
    /// §4.1 floor-toward-minus-infinity operator.
    pub int_vec: i32,
    /// `half_flag[t] = (vector[r][s][t] - 2 * int_vec[t]) != 0`,
    /// equivalent to "the vector component is odd".
    pub half_flag: bool,
}

/// The two-axis split `(horizontal, vertical)` of a complete
/// `vector'[r][s][1:0]` per §7.6.4.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SplitVector {
    /// Horizontal axis (`t = 0`): the x-offset of the reference
    /// read, plus its half-flag.
    pub horizontal: ComponentSplit,
    /// Vertical axis (`t = 1`): the y-offset of the reference read,
    /// plus its half-flag.
    pub vertical: ComponentSplit,
}

impl SplitVector {
    /// Map the two `half_flag` bits to the matching [`HalfPattern`].
    pub fn pattern(self) -> HalfPattern {
        HalfPattern::from_flags(self.horizontal.half_flag, self.vertical.half_flag)
    }
}

/// Split a single motion-vector component `vector[r][s][t]` per the
/// inner loop of §7.6.4 page 88:
///
/// ```text
/// int_vec[t]  = vector[r][s][t] DIV 2
/// half_flag[t] = (vector[r][s][t] - 2 * int_vec[t]) != 0
/// ```
///
/// `DIV` is the §4.1 floor-toward-minus-infinity operator, so
/// `(-3) DIV 2 = -2` (not `-1` as truncated-toward-zero division
/// would give); the half-flag is set whenever the original component
/// is odd, including for negative odd vectors.
///
/// Examples (from §4.1 page 9):
///
/// * `split_component(0)`  → `int_vec = 0`,  `half_flag = false`
/// * `split_component(1)`  → `int_vec = 0`,  `half_flag = true`
/// * `split_component(2)`  → `int_vec = 1`,  `half_flag = false`
/// * `split_component(3)`  → `int_vec = 1`,  `half_flag = true`
/// * `split_component(-1)` → `int_vec = -1`, `half_flag = true`
/// * `split_component(-2)` → `int_vec = -1`, `half_flag = false`
/// * `split_component(-3)` → `int_vec = -2`, `half_flag = true`
pub fn split_component(vector: i32) -> ComponentSplit {
    // Rust's `/` truncates toward zero. The spec's `DIV` floors toward
    // minus infinity. For divisor 2 the two agree on non-negatives;
    // for negatives `DIV` rounds *down* one extra step whenever the
    // dividend is odd (`-3 DIV 2 = -2` vs `-3 / 2 = -1`).
    //
    // div_euclid(2) gives floor for any sign, which matches `DIV` for
    // a positive divisor.
    let int_vec = vector.div_euclid(2);
    let half_flag = vector - 2 * int_vec != 0;
    ComponentSplit { int_vec, half_flag }
}

/// Split a two-component motion vector `(horizontal, vertical)` per
/// §7.6.4 page 88 — i.e. apply [`split_component`] to each axis.
///
/// Index convention matches the §7.6.4 pseudocode: `t = 0` is the
/// horizontal axis (x) and `t = 1` is the vertical axis (y).
pub fn split_vector(horizontal: i32, vertical: i32) -> SplitVector {
    SplitVector {
        horizontal: split_component(horizontal),
        vertical: split_component(vertical),
    }
}

/// Convenience: split the two reconstructed components produced by
/// [`crate::pmv::reconstruct_motion_vector`] / [`Component`] for one
/// motion vector. Pass the horizontal and vertical
/// [`ReconstructedComponent::vector_prime`] values directly.
pub fn split_reconstructed(
    horizontal: ReconstructedComponent,
    vertical: ReconstructedComponent,
) -> SplitVector {
    let _ = Component::Horizontal; // index labels (see Component::index)
    let _ = Component::Vertical;
    split_vector(horizontal.vector_prime, vertical.vector_prime)
}

/// How [`ReferencePlane::sample`] handles a read that falls outside
/// `[0, width) × [0, height)` of the reference plane.
///
/// The H.262 base text says nothing about samples beyond the picture
/// boundary; in practice every conformant decoder clips reads to the
/// nearest edge sample (the **pad-to-edge** convention), which keeps
/// the linear-interpolation arithmetic of §7.6.4 well-defined for
/// motion vectors that reach past the frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoundaryMode {
    /// Replicate the nearest in-bounds sample — `x` is clamped to
    /// `[0, width-1]` and `y` to `[0, height-1]` before the lookup.
    ///
    /// This is the universally-implemented default.
    PadEdge,
}

/// A borrowed view over one reference sample plane.
///
/// `data` is a flat row-major buffer of length `width * height` (one
/// `u8` per sample). The §7.6.4 boundary case is resolved by the
/// [`BoundaryMode`] field — see [`ReferencePlane::sample`].
#[derive(Debug, Clone, Copy)]
pub struct ReferencePlane<'a> {
    /// Row-major sample buffer of length `width * height`.
    pub data: &'a [u8],
    /// Plane width in samples (number of columns).
    pub width: usize,
    /// Plane height in samples (number of rows).
    pub height: usize,
    /// How [`ReferencePlane::sample`] handles `(x, y)` outside the
    /// plane.
    pub boundary: BoundaryMode,
}

impl<'a> ReferencePlane<'a> {
    /// Build a `ReferencePlane` from a flat `data` buffer plus the
    /// `width × height` extent. Returns `None` if `data.len() !=
    /// width * height`. `boundary` defaults to
    /// [`BoundaryMode::PadEdge`].
    pub fn new(data: &'a [u8], width: usize, height: usize) -> Option<Self> {
        if width.checked_mul(height) != Some(data.len()) {
            return None;
        }
        Some(Self {
            data,
            width,
            height,
            boundary: BoundaryMode::PadEdge,
        })
    }

    /// Build a `ReferencePlane` with an explicit `boundary` mode.
    /// Mirrors [`ReferencePlane::new`] with the boundary override.
    pub fn with_boundary(
        data: &'a [u8],
        width: usize,
        height: usize,
        boundary: BoundaryMode,
    ) -> Option<Self> {
        let mut plane = Self::new(data, width, height)?;
        plane.boundary = boundary;
        Some(plane)
    }

    /// Read a single sample at `(x, y)`, resolving the
    /// `BoundaryMode::PadEdge` clip if `(x, y)` falls outside the
    /// plane.
    ///
    /// Returns `0` only if the plane has zero extent (empty plane);
    /// the [`ReferencePlane::new`] constructor refuses to build an
    /// empty plane unless `data` is also empty, so this is rare in
    /// practice.
    pub fn sample(self, x: i32, y: i32) -> u8 {
        if self.width == 0 || self.height == 0 {
            return 0;
        }
        let max_x = (self.width - 1) as i32;
        let max_y = (self.height - 1) as i32;
        let cx = x.clamp(0, max_x) as usize;
        let cy = y.clamp(0, max_y) as usize;
        self.data[cy * self.width + cx]
    }
}

/// Average two `u8` samples with the §4.1 `// 2` operator. Since
/// `a + b` is a sum of unsigned 8-bit values the result is at most
/// `510`, which fits in `u16`. The `// 2` operator (round to nearest,
/// half-integer away from zero) on a non-negative sum coincides with
/// `(sum + 1) / 2` integer division, i.e. `(a + b).div_ceil(2)`.
#[inline]
fn avg2(a: u8, b: u8) -> u8 {
    (a as u16 + b as u16).div_ceil(2) as u8
}

/// Average four `u8` samples with the §4.1 `// 4` operator. Sum is
/// at most `1020`, fits in `u16`. `// 4` on a non-negative sum is
/// `(sum + 2) / 4` integer division (the spec's "rounded to nearest,
/// half-integer away from zero" coincides with this since the sum
/// can only be `≡ 2 (mod 4)` at the tie boundary), and `(sum + 2) /
/// 4` matches the `u16::div_ceil(4)` for sums that are `>= 2 (mod 4)`
/// while remaining identical to truncating integer division otherwise.
/// Express the rounding explicitly to avoid the
/// `clippy::manual_div_ceil` lint for the `2`-tie case.
#[inline]
fn avg4(a: u8, b: u8, c: u8, d: u8) -> u8 {
    let sum: u16 = a as u16 + b as u16 + c as u16 + d as u16;
    // (sum + 2) >> 2 is the canonical rounded-to-nearest-up form; the
    // sum cannot overflow u16 since each operand is `<= 255`.
    ((sum + 2) >> 2) as u8
}

/// Form a single prediction sample at `(x, y)` of the prediction
/// block, given the §7.6.4 split vector. This is the inner branch
/// of [`predict_block`] exposed for callers that want to sample a
/// single pel without allocating a block buffer.
pub fn predict_sample(reference: ReferencePlane<'_>, x: i32, y: i32, split: SplitVector) -> u8 {
    let ix = split.horizontal.int_vec;
    let iy = split.vertical.int_vec;
    let rx = x + ix;
    let ry = y + iy;
    match split.pattern() {
        HalfPattern::Integer => reference.sample(rx, ry),
        HalfPattern::HalfHorizontal => avg2(reference.sample(rx, ry), reference.sample(rx + 1, ry)),
        HalfPattern::HalfVertical => avg2(reference.sample(rx, ry), reference.sample(rx, ry + 1)),
        HalfPattern::HalfBoth => avg4(
            reference.sample(rx, ry),
            reference.sample(rx + 1, ry),
            reference.sample(rx, ry + 1),
            reference.sample(rx + 1, ry + 1),
        ),
    }
}

/// The dimensions of one §7.6.5 prediction block in samples. The
/// §7.6.4 loop is geometry-agnostic, so this is a free
/// `(width, height)` pair (16×16 frame, 16×8 MC, 16×16 field,
/// 8×8 / 8×16 / 16×16 chroma per the 4:2:0 / 4:2:2 / 4:4:4
/// scaling in §7.6.3.7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockSize {
    /// Block width in samples.
    pub width: usize,
    /// Block height in samples.
    pub height: usize,
}

impl BlockSize {
    /// Construct a [`BlockSize`]; either axis must be non-zero or
    /// the result is `None` (a zero block would be a no-op).
    pub fn new(width: usize, height: usize) -> Option<Self> {
        if width == 0 || height == 0 {
            return None;
        }
        Some(Self { width, height })
    }
}

/// Form the `width × height` prediction block starting at
/// `(top_left_x, top_left_y)` per §7.6.4. Returns a `Vec<u8>` of
/// length `width * height` in row-major order.
///
/// The motion vector `(horizontal, vertical)` is the
/// fully-reconstructed `vector'[r][s][1:0]` from §7.6.3.1 (i.e.
/// after PMV reconstruction). Internally the vector is split with
/// [`split_vector`] and each sample is filled by [`predict_sample`].
pub fn predict_block(
    reference: ReferencePlane<'_>,
    top_left_x: i32,
    top_left_y: i32,
    size: BlockSize,
    horizontal: i32,
    vertical: i32,
) -> Vec<u8> {
    let split = split_vector(horizontal, vertical);
    let mut out = Vec::with_capacity(size.width * size.height);
    for row in 0..size.height as i32 {
        for col in 0..size.width as i32 {
            out.push(predict_sample(
                reference,
                top_left_x + col,
                top_left_y + row,
                split,
            ));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- split_component ----

    #[test]
    fn split_component_zero() {
        let s = split_component(0);
        assert_eq!(s.int_vec, 0);
        assert!(!s.half_flag);
    }

    #[test]
    fn split_component_positive_even() {
        let s = split_component(2);
        assert_eq!(s.int_vec, 1);
        assert!(!s.half_flag);
    }

    #[test]
    fn split_component_positive_odd() {
        let s = split_component(3);
        assert_eq!(s.int_vec, 1);
        assert!(s.half_flag);
    }

    #[test]
    fn split_component_one() {
        let s = split_component(1);
        assert_eq!(s.int_vec, 0);
        assert!(s.half_flag);
    }

    #[test]
    fn split_component_negative_even() {
        let s = split_component(-2);
        assert_eq!(s.int_vec, -1);
        assert!(!s.half_flag);
    }

    #[test]
    fn split_component_negative_odd() {
        // -3 DIV 2 = -2 (floor toward -inf, NOT truncate-toward-zero
        // which would give -1). half_flag = true since -3 is odd.
        let s = split_component(-3);
        assert_eq!(s.int_vec, -2);
        assert!(s.half_flag);
    }

    #[test]
    fn split_component_minus_one() {
        // -1 DIV 2 = -1 (floor), half_flag = true
        let s = split_component(-1);
        assert_eq!(s.int_vec, -1);
        assert!(s.half_flag);
    }

    #[test]
    fn split_component_large_negative_odd() {
        let s = split_component(-1023);
        assert_eq!(s.int_vec, -512);
        assert!(s.half_flag);
    }

    #[test]
    fn split_component_large_positive_odd() {
        let s = split_component(1023);
        assert_eq!(s.int_vec, 511);
        assert!(s.half_flag);
    }

    // ---- split_vector + pattern ----

    #[test]
    fn split_vector_integer_pattern() {
        let v = split_vector(4, -8);
        assert_eq!(v.horizontal.int_vec, 2);
        assert!(!v.horizontal.half_flag);
        assert_eq!(v.vertical.int_vec, -4);
        assert!(!v.vertical.half_flag);
        assert_eq!(v.pattern(), HalfPattern::Integer);
    }

    #[test]
    fn split_vector_half_horizontal_pattern() {
        let v = split_vector(3, 4);
        assert_eq!(v.pattern(), HalfPattern::HalfHorizontal);
        assert!(v.pattern().is_half_pel());
    }

    #[test]
    fn split_vector_half_vertical_pattern() {
        let v = split_vector(4, 3);
        assert_eq!(v.pattern(), HalfPattern::HalfVertical);
    }

    #[test]
    fn split_vector_half_both_pattern() {
        let v = split_vector(1, -1);
        assert_eq!(v.pattern(), HalfPattern::HalfBoth);
        assert_eq!(v.horizontal.int_vec, 0);
        assert!(v.horizontal.half_flag);
        assert_eq!(v.vertical.int_vec, -1);
        assert!(v.vertical.half_flag);
    }

    #[test]
    fn half_pattern_round_trip() {
        for &p in &[
            HalfPattern::Integer,
            HalfPattern::HalfHorizontal,
            HalfPattern::HalfVertical,
            HalfPattern::HalfBoth,
        ] {
            let (h, v) = p.flags();
            assert_eq!(HalfPattern::from_flags(h, v), p);
        }
    }

    // ---- ReferencePlane ----

    fn plane(data: &[u8], w: usize, h: usize) -> ReferencePlane<'_> {
        ReferencePlane::new(data, w, h).expect("dimensions match data length")
    }

    #[test]
    fn reference_plane_in_bounds_sample() {
        // 4x3 plane, distinct values everywhere
        let data: Vec<u8> = (0..12).collect();
        let r = plane(&data, 4, 3);
        assert_eq!(r.sample(0, 0), 0);
        assert_eq!(r.sample(3, 0), 3);
        assert_eq!(r.sample(0, 2), 8);
        assert_eq!(r.sample(3, 2), 11);
        assert_eq!(r.sample(1, 1), 5);
    }

    #[test]
    fn reference_plane_pad_edge_left() {
        let data: Vec<u8> = (0..12).collect();
        let r = plane(&data, 4, 3);
        // x = -5 clamps to 0
        assert_eq!(r.sample(-5, 1), r.sample(0, 1));
    }

    #[test]
    fn reference_plane_pad_edge_right() {
        let data: Vec<u8> = (0..12).collect();
        let r = plane(&data, 4, 3);
        // x = 10 clamps to width-1 = 3
        assert_eq!(r.sample(10, 1), r.sample(3, 1));
    }

    #[test]
    fn reference_plane_pad_edge_top_bottom() {
        let data: Vec<u8> = (0..12).collect();
        let r = plane(&data, 4, 3);
        assert_eq!(r.sample(2, -7), r.sample(2, 0));
        assert_eq!(r.sample(2, 100), r.sample(2, 2));
    }

    #[test]
    fn reference_plane_corners_clamp_both_axes() {
        let data: Vec<u8> = (0..12).collect();
        let r = plane(&data, 4, 3);
        assert_eq!(r.sample(-5, -5), 0);
        assert_eq!(r.sample(100, 100), 11);
    }

    #[test]
    fn reference_plane_rejects_size_mismatch() {
        assert!(ReferencePlane::new(&[0u8; 10], 4, 3).is_none());
    }

    // ---- predict_sample: integer pattern ----

    #[test]
    fn predict_sample_integer_zero_vector() {
        let data: Vec<u8> = (0..16).collect(); // 4x4
        let r = plane(&data, 4, 4);
        let split = split_vector(0, 0);
        for y in 0..4 {
            for x in 0..4 {
                assert_eq!(
                    predict_sample(r, x as i32, y as i32, split),
                    data[y * 4 + x]
                );
            }
        }
    }

    #[test]
    fn predict_sample_integer_with_offset() {
        // 4x4 plane with values = y*10 + x
        let data: Vec<u8> = (0..16).map(|i| ((i / 4) * 10 + (i % 4)) as u8).collect();
        let r = plane(&data, 4, 4);
        // Vector (2, 4) -> int_vec = (1, 2), no half-pel
        let split = split_vector(2, 4);
        assert_eq!(split.pattern(), HalfPattern::Integer);
        // Read at (0, 0) -> ref (1, 2) -> value 21
        assert_eq!(predict_sample(r, 0, 0, split), 21);
    }

    // ---- predict_sample: half-pel rounding ----

    #[test]
    fn predict_sample_half_horizontal_rounding() {
        // pel_ref row = [10, 11], avg = (10+11+1)/2 = 11 (// rounds away
        // from zero, half-integer ties to the larger magnitude).
        let data = vec![10u8, 11u8];
        let r = plane(&data, 2, 1);
        let split = split_vector(1, 0); // half-horizontal
        assert_eq!(split.pattern(), HalfPattern::HalfHorizontal);
        // x = 0, y = 0: reads (0,0) and (1,0).
        assert_eq!(predict_sample(r, 0, 0, split), 11);
    }

    #[test]
    fn predict_sample_half_horizontal_even_sum() {
        // [10, 12] -> avg = 11 (no tie)
        let data = vec![10u8, 12u8];
        let r = plane(&data, 2, 1);
        let split = split_vector(1, 0);
        assert_eq!(predict_sample(r, 0, 0, split), 11);
    }

    #[test]
    fn predict_sample_half_vertical_rounding() {
        // column = [10, 11], avg = 11 (tie rounded up).
        let data = vec![10u8, 11u8];
        let r = plane(&data, 1, 2);
        let split = split_vector(0, 1); // half-vertical
        assert_eq!(split.pattern(), HalfPattern::HalfVertical);
        assert_eq!(predict_sample(r, 0, 0, split), 11);
    }

    #[test]
    fn predict_sample_half_both_average_of_four() {
        // 2x2 plane:
        // [10, 11]
        // [12, 13]
        // sum = 46, // 4 = (46+2)/4 = 12.
        let data = vec![10u8, 11u8, 12u8, 13u8];
        let r = plane(&data, 2, 2);
        let split = split_vector(1, 1);
        assert_eq!(split.pattern(), HalfPattern::HalfBoth);
        assert_eq!(predict_sample(r, 0, 0, split), 12);
    }

    #[test]
    fn predict_sample_half_both_tie_rounds_up() {
        // sum = 2 (mod 4) tie case: pick four samples summing to 2 (mod 4)
        // e.g. 0,0,1,1 -> sum=2, // 4 = (2+2)/4 = 1. Spec: round
        // half-integer away from zero, so 1/2 -> 1 (not 0).
        let data = vec![0u8, 1u8, 0u8, 1u8];
        let r = plane(&data, 2, 2);
        let split = split_vector(1, 1);
        assert_eq!(predict_sample(r, 0, 0, split), 1);
    }

    // ---- predict_sample: negative vectors and clamping ----

    #[test]
    fn predict_sample_negative_integer_vector_clamps() {
        // 3x3 plane filled with consecutive values
        let data: Vec<u8> = (0..9).collect();
        let r = plane(&data, 3, 3);
        // vector (-4, -6) -> int_vec = (-2, -3), no half. From (0,0):
        // ref (-2, -3) -> pad-edge -> sample (0, 0) = 0.
        let split = split_vector(-4, -6);
        assert_eq!(split.pattern(), HalfPattern::Integer);
        assert_eq!(predict_sample(r, 0, 0, split), 0);
    }

    #[test]
    fn predict_sample_negative_odd_vector_uses_floor() {
        // 3x3 plane, values are the y*3 + x offset
        let data: Vec<u8> = (0..9).collect();
        let r = plane(&data, 3, 3);
        // vector (-3, 0) -> int_vec = (-2, 0), half_flag[0] = true
        // (HalfHorizontal). From (2, 1):
        // reads ref(0, 1) = 3 and ref(1, 1) = 4. avg = 4 (tie up).
        let split = split_vector(-3, 0);
        assert_eq!(split.pattern(), HalfPattern::HalfHorizontal);
        assert_eq!(split.horizontal.int_vec, -2);
        assert_eq!(predict_sample(r, 2, 1, split), 4);
    }

    // ---- predict_block: full geometry ----

    #[test]
    fn predict_block_copy_with_zero_vector() {
        // 4x4 plane, identity copy of the upper-left 2x2 block.
        let data: Vec<u8> = (0..16).collect();
        let r = plane(&data, 4, 4);
        let block = predict_block(r, 0, 0, BlockSize::new(2, 2).unwrap(), 0, 0);
        assert_eq!(block, vec![0, 1, 4, 5]);
    }

    #[test]
    fn predict_block_translation_one_pel_right() {
        let data: Vec<u8> = (0..16).collect();
        let r = plane(&data, 4, 4);
        // Vector (2, 0) -> integer offset, x' = x + 1. Reading the
        // upper-left 2x2 block from (0, 0) actually reads (1..3, 0..2).
        let block = predict_block(r, 0, 0, BlockSize::new(2, 2).unwrap(), 2, 0);
        assert_eq!(block, vec![1, 2, 5, 6]);
    }

    #[test]
    fn predict_block_size_geometry() {
        // 8x4 plane, request 4x2 block at (1, 1), zero vector.
        let data: Vec<u8> = (0..32).collect();
        let r = plane(&data, 8, 4);
        let block = predict_block(r, 1, 1, BlockSize::new(4, 2).unwrap(), 0, 0);
        // Rows 1, 2 starting at column 1:
        // row 1: [9, 10, 11, 12]
        // row 2: [17, 18, 19, 20]
        assert_eq!(block, vec![9, 10, 11, 12, 17, 18, 19, 20]);
    }

    #[test]
    fn predict_block_half_horizontal_block() {
        // 4x1 plane, half-horizontal vector, 3x1 output.
        let data = vec![10u8, 20u8, 30u8, 40u8];
        let r = plane(&data, 4, 1);
        let block = predict_block(r, 0, 0, BlockSize::new(3, 1).unwrap(), 1, 0);
        // x=0: avg(10, 20) = 15
        // x=1: avg(20, 30) = 25
        // x=2: avg(30, 40) = 35
        assert_eq!(block, vec![15, 25, 35]);
    }

    #[test]
    fn predict_block_half_vertical_block() {
        // 1x4 plane (column), half-vertical vector, 1x3 output.
        let data = vec![10u8, 20u8, 30u8, 40u8];
        let r = plane(&data, 1, 4);
        let block = predict_block(r, 0, 0, BlockSize::new(1, 3).unwrap(), 0, 1);
        assert_eq!(block, vec![15, 25, 35]);
    }

    #[test]
    fn predict_block_half_both_block() {
        // 3x3 plane, half-pel both axes, 2x2 output.
        let data: Vec<u8> = vec![1, 2, 3, 4, 5, 6, 7, 8, 9];
        let r = plane(&data, 3, 3);
        let block = predict_block(r, 0, 0, BlockSize::new(2, 2).unwrap(), 1, 1);
        // pixel (0,0): avg4(1,2,4,5) = (12+2)/4 = 3
        // pixel (1,0): avg4(2,3,5,6) = (16+2)/4 = 4
        // pixel (0,1): avg4(4,5,7,8) = (24+2)/4 = 6
        // pixel (1,1): avg4(5,6,8,9) = (28+2)/4 = 7
        assert_eq!(block, vec![3, 4, 6, 7]);
    }

    #[test]
    fn predict_block_pad_edge_at_right_boundary() {
        // 2x1 plane [5, 9], request 4 columns from (0, 0) with zero vector.
        // Columns x=0..3 read ref(0..3, 0) but width is 2 -> clamp:
        // x=0->5, x=1->9, x=2->9 (clamp), x=3->9 (clamp).
        let data = vec![5u8, 9u8];
        let r = plane(&data, 2, 1);
        let block = predict_block(r, 0, 0, BlockSize::new(4, 1).unwrap(), 0, 0);
        assert_eq!(block, vec![5, 9, 9, 9]);
    }

    #[test]
    fn block_size_rejects_zero() {
        assert!(BlockSize::new(0, 16).is_none());
        assert!(BlockSize::new(16, 0).is_none());
    }

    // ---- split_reconstructed wires ReconstructedComponent through ----

    #[test]
    fn split_reconstructed_matches_split_vector() {
        let horiz = ReconstructedComponent {
            vector_prime: 3,
            new_pmv: 3,
            delta: 3,
            range: 16,
        };
        let vert = ReconstructedComponent {
            vector_prime: -3,
            new_pmv: -3,
            delta: -3,
            range: 16,
        };
        let a = split_reconstructed(horiz, vert);
        let b = split_vector(3, -3);
        assert_eq!(a, b);
    }
}
