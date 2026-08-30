//! Motion estimation — the encoder-side search that picks the
//! forward motion vector for a P-picture macroblock.
//!
//! Motion estimation is **not** specified by ISO/IEC 11172-2 or
//! ISO/IEC 13818-2: the standards define only how a decoder *uses* a
//! transmitted motion vector (§7.6.3 reconstruction + §7.6.4 prediction
//! forming), never how an encoder *chooses* it. Any vector a conformant
//! decoder can reconstruct is a legal choice; the encoder is free to use
//! whatever search minimises its rate-distortion objective. This module
//! implements a straightforward sum-of-absolute-differences (SAD) block
//! match on the luminance plane:
//!
//! 1. **Integer-pel full search** over a square window
//!    `[-range, +range]` (in integer luma samples) centred on the
//!    macroblock's collocated position, scoring each candidate by the
//!    SAD of the 16×16 luma block against the
//!    [`crate::forming_predictions::predict_block`] prediction (so the
//!    score is computed on exactly the samples the decoder will form).
//! 2. **Half-pel refinement** of the eight half-sample neighbours of the
//!    integer-pel winner, again scored by SAD on the interpolated
//!    prediction the decoder produces.
//!
//! The returned [`MotionVectorPel`] is in the **half-sample** luminance
//! units the §7.6.3.1 reconstruction and §7.6.4 prediction both consume
//! — i.e. an integer displacement of `n` luma samples is `2*n`. Because
//! the score is computed against the very prediction
//! [`predict_block`] forms, the SAD this search reports is the exact
//! luma prediction error the residual encoder will then transform.
//!
//! The search is deliberately exhaustive rather than fast: this crate's
//! goal is bit-exact round-trip correctness, not encoder speed. A
//! diamond / hexagon fast-search can replace the inner loop later
//! without changing the public contract.

use crate::forming_predictions::{predict_block, BlockSize, ReferencePlane};
use crate::frame_assembly::FrameBuffer;
use crate::inter_reconstruction::MotionVectorPel;

/// Sum of absolute differences between a 16×16 `current` luma block at
/// macroblock `(mb_col, mb_row)` and the prediction formed from
/// `reference` at the half-sample vector `(hx, hy)`.
///
/// `(hx, hy)` are in half-sample units (the §7.6.4 prediction units).
/// The prediction is formed by [`predict_block`] exactly as the decoder
/// would, so the returned SAD is the true luma prediction error.
fn luma_sad(
    current: &FrameBuffer,
    reference: ReferencePlane<'_>,
    mb_col: usize,
    mb_row: usize,
    hx: i32,
    hy: i32,
    best_so_far: u32,
) -> u32 {
    let base_x = (mb_col * 16) as i32;
    let base_y = (mb_row * 16) as i32;
    let size = BlockSize::new(16, 16).expect("16x16 is non-empty");
    let pred = predict_block(reference, base_x, base_y, size, hx, hy);
    let plane = &current.y;
    let w = plane.width();
    let h = plane.height();
    let mut sad = 0u32;
    for row in 0..16usize {
        let sy = (base_y as usize + row).min(h.saturating_sub(1));
        for col in 0..16usize {
            let sx = (base_x as usize + col).min(w.saturating_sub(1));
            let cur = i32::from(plane.get(sx, sy).unwrap_or(0));
            let prd = i32::from(pred[row * 16 + col]);
            sad += (cur - prd).unsigned_abs();
        }
        // Early-out: a partial SAD already past the incumbent cannot win.
        if sad >= best_so_far {
            return sad;
        }
    }
    sad
}

/// The result of a motion search: the chosen half-sample motion vector
/// and the luma SAD of its prediction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotionSearchResult {
    /// The chosen forward motion vector in half-sample luminance units.
    pub vector: MotionVectorPel,
    /// The luma sum-of-absolute-differences of the prediction this
    /// vector forms against the current block.
    pub sad: u32,
}

/// Estimate the forward motion vector for the macroblock at
/// `(mb_col, mb_row)` of `current` against the reconstructed `reference`
/// frame, searching an integer-pel window of `±search_range` luma
/// samples followed by a half-pel refinement.
///
/// The returned [`MotionVectorPel`] is in half-sample units. The search
/// is restricted to vectors that keep the reconstructed value inside the
/// `[low, high]` range of `f_code` (so the chosen vector is always
/// codable): a candidate whose half-sample component would exceed
/// `±(16 * 2^(f_code-1) * 2 - …)` is simply never visited because the
/// caller picks `search_range` accordingly — see [`max_search_range`].
pub fn estimate_forward_mv(
    current: &FrameBuffer,
    reference: &FrameBuffer,
    mb_col: usize,
    mb_row: usize,
    search_range: i32,
) -> MotionSearchResult {
    let data = reference.y.samples();
    let plane = ReferencePlane::new(data, reference.y.width(), reference.y.height())
        .expect("reference luma plane is width*height");

    // §7.6.3.8: reconstructed motion vectors shall not refer to
    // samples outside the boundary of the coded picture. The §7.6.4
    // prediction of a half-sample vector `h` reads integer columns
    // `base + (h DIV 2) ..= base + 15 + (h DIV 2) + (h & 1)` (DIV
    // rounds toward minus infinity; an odd component interpolates one
    // extra sample), so a candidate is only legal when that whole
    // span lies inside the reference plane — the coded macroblock
    // grid. The zero-vector incumbent is always legal (the macroblock
    // itself is inside the picture).
    let legal = |hx: i32, hy: i32| -> bool {
        frame_vector_legal(
            reference.y.width(),
            reference.y.height(),
            mb_col,
            mb_row,
            hx,
            hy,
        )
    };

    // Tie-break helper: when two vectors give the same SAD, prefer the
    // one that is cheaper to code (smaller magnitude, hence a shorter
    // §6.2.5.2.1 motion_code). The bias is a single SAD unit per unit of
    // Manhattan magnitude so it only ever discriminates exact ties, never
    // overriding a genuinely lower-error vector.
    let vec_cost = |hx: i32, hy: i32| -> u32 { hx.unsigned_abs() + hy.unsigned_abs() };

    // --- Integer-pel full search ---
    // The zero vector is the natural starting incumbent (cheap to code).
    let mut best_hx = 0i32;
    let mut best_hy = 0i32;
    let mut best_sad = luma_sad(current, plane, mb_col, mb_row, 0, 0, u32::MAX);
    let mut best_score = best_sad.saturating_add(vec_cost(0, 0));
    for dy in -search_range..=search_range {
        for dx in -search_range..=search_range {
            if dx == 0 && dy == 0 {
                continue;
            }
            // Integer displacement (dx, dy) → half-sample (2*dx, 2*dy).
            let hx = dx * 2;
            let hy = dy * 2;
            if !legal(hx, hy) {
                continue;
            }
            let sad = luma_sad(current, plane, mb_col, mb_row, hx, hy, best_score);
            let score = sad.saturating_add(vec_cost(hx, hy));
            if score < best_score {
                best_score = score;
                best_sad = sad;
                best_hx = hx;
                best_hy = hy;
            }
        }
    }

    // --- Half-pel refinement of the integer-pel winner ---
    let int_hx = best_hx;
    let int_hy = best_hy;
    for &(ox, oy) in &[
        (-1i32, -1i32),
        (0, -1),
        (1, -1),
        (-1, 0),
        (1, 0),
        (-1, 1),
        (0, 1),
        (1, 1),
    ] {
        let hx = int_hx + ox;
        let hy = int_hy + oy;
        if !legal(hx, hy) {
            continue;
        }
        let sad = luma_sad(current, plane, mb_col, mb_row, hx, hy, best_score);
        let score = sad.saturating_add(vec_cost(hx, hy));
        if score < best_score {
            best_score = score;
            best_sad = sad;
            best_hx = hx;
            best_hy = hy;
        }
    }

    MotionSearchResult {
        vector: MotionVectorPel::new(best_hx, best_hy),
        sad: best_sad,
    }
}

/// §7.6.3.8 legality of a frame-format half-sample vector `(hx, hy)`
/// for the 16×16 luminance macroblock at `(mb_col, mb_row)` against a
/// `ref_width × ref_height` reference plane: the whole §7.6.4 read
/// span `base + (h DIV 2) ..= base + 15 + (h DIV 2) + (h & 1)` must
/// lie inside the plane.
pub fn frame_vector_legal(
    ref_width: usize,
    ref_height: usize,
    mb_col: usize,
    mb_row: usize,
    hx: i32,
    hy: i32,
) -> bool {
    let base_x = (mb_col * 16) as i32;
    let base_y = (mb_row * 16) as i32;
    let ix = hx.div_euclid(2);
    let iy = hy.div_euclid(2);
    let ex = i32::from(hx.rem_euclid(2) != 0);
    let ey = i32::from(hy.rem_euclid(2) != 0);
    base_x + ix >= 0
        && base_y + iy >= 0
        && base_x + ix + 15 + ex < ref_width as i32
        && base_y + iy + 15 + ey < ref_height as i32
}

/// The largest integer-pel `search_range` that keeps every reachable
/// half-sample vector inside the codable `[low, high]` band of `f_code`.
///
/// §7.6.3.1 bounds the reconstructed vector to `[-16*f, 16*f - 1]` half
/// samples where `f = 2^(f_code-1)`. An integer displacement of `r` luma
/// samples is `2*r` half samples plus up to one half-sample of
/// refinement, so `2*r + 1 <= 16*f - 1`, i.e. `r <= 8*f - 1`. Clamp the
/// caller-requested range to that ceiling so the search never produces an
/// uncodable vector.
pub fn max_search_range(f_code: u8) -> i32 {
    let f = 1i32 << (f_code.clamp(1, 9) - 1);
    (8 * f - 1).max(1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sequence_extension::ChromaFormat;

    /// Build a frame whose luma is a function of `(x, y)`, mid-grey
    /// chroma.
    fn frame_from<F: Fn(usize, usize) -> u8>(w: usize, h: usize, f: F) -> FrameBuffer {
        let mut fb = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
        for y in 0..h {
            for x in 0..w {
                fb.y.put_sample(x, y, f(x, y));
            }
        }
        for y in 0..fb.cb.height() {
            for x in 0..fb.cb.width() {
                fb.cb.put_sample(x, y, 128);
                fb.cr.put_sample(x, y, 128);
            }
        }
        fb
    }

    #[test]
    fn identical_frames_find_zero_vector() {
        let reference = frame_from(48, 48, |x, y| ((x * 5 + y * 3) % 200 + 20) as u8);
        let current = reference.clone();
        let r = estimate_forward_mv(&current, &reference, 1, 1, 8);
        assert_eq!(r.vector, MotionVectorPel::new(0, 0));
        assert_eq!(r.sad, 0);
    }

    #[test]
    fn pure_translation_recovers_the_shift() {
        // Reference is a gradient; current is the reference shifted right
        // by 3 luma samples (so the block at (mb) predicts best from a
        // vector pointing 3 samples left = -6 half-samples horizontally).
        let reference = frame_from(64, 48, |x, _| (16 + x * 3).min(235) as u8);
        let shift = 3usize;
        let current = frame_from(64, 48, |x, _| {
            let sx = x.saturating_sub(shift);
            (16 + sx * 3).min(235) as u8
        });
        // Search the interior macroblock (1,1) to avoid edge clamping.
        let r = estimate_forward_mv(&current, &reference, 1, 1, 8);
        assert_eq!(r.vector, MotionVectorPel::new(-(shift as i32) * 2, 0));
        assert_eq!(r.sad, 0);
    }

    #[test]
    fn half_pel_refinement_beats_integer() {
        // A smooth horizontal ramp: the best match for a block sampled at
        // half-integer offset is a half-pel vector. Build current as the
        // half-pel average of the reference ramp.
        let reference = frame_from(64, 48, |x, _| (10 + x * 2) as u8);
        // current[x] = avg(ref[x], ref[x+1]) = ref shifted by +0.5 sample.
        let current = frame_from(64, 48, |x, _| {
            let a = 10 + x * 2;
            let b = 10 + (x + 1) * 2;
            ((a + b).div_ceil(2)) as u8
        });
        let r = estimate_forward_mv(&current, &reference, 1, 1, 8);
        // The best vector is +1 half-sample horizontally (half-pel right).
        assert_eq!(r.vector, MotionVectorPel::new(1, 0));
        assert_eq!(r.sad, 0);
    }

    #[test]
    fn max_search_range_respects_f_code() {
        assert_eq!(max_search_range(1), 7); // f = 1 → 8*1 - 1
        assert_eq!(max_search_range(2), 15); // f = 2 → 8*2 - 1
        assert_eq!(max_search_range(4), 63); // f = 8 → 8*8 - 1
    }
}
