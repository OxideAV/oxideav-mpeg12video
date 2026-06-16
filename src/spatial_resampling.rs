//! §7.7.3.5 / §7.7.3.6 spatial-scalable lower-layer resampling per
//! **ISO/IEC 13818-2 (Recommendation ITU-T H.262)**, pages 113–114 of
//! the 1995 base text — the linear-interpolation upsampling that takes a
//! progressive lower-layer frame (`prog_pic`) and resamples it onto the
//! enhancement-layer sample grid, producing the spatial prediction the
//! §7.7.4 combiner ([`crate::spatial_temporal_combine`]) consumes as
//! `pel_pred_spat`.
//!
//! ## Where this sits in §7.7.3
//!
//! §7.7.3 *"Formation of spatial prediction"* upsamples the lower-layer
//! reconstructed frame `d_lower[y][x]` to the enhancement-layer grid in
//! a pipeline (§7.7.3.2, Figure 7-14):
//!
//! ```text
//!   d_lower → [§7.7.3.4 deinterlace] → prog_pic
//!           → [§7.7.3.5 vertical resample]   → vert_pic
//!           → [§7.7.3.6 horizontal resample] → hor_pic
//!           → [§7.7.3.7 reinterlace]         → spat_pred_pic
//! ```
//!
//! This module implements the **two resampling stages** — §7.7.3.5 and
//! §7.7.3.6 — which are the linear-interpolation core of the pipeline.
//! For the dominant progressive-to-progressive case
//! (`lower_layer_progressive_frame == 1`, `progressive_frame == 1`) no
//! deinterlace (§7.7.3.4) or reinterlace (§7.7.3.7) step is applied
//! (Table 7-15 row 3: *"Apply deinterlace process = no"*), so `prog_pic`
//! is the lower-layer reconstructed frame directly and `hor_pic` is
//! renamed to `spat_pred_pic` unchanged (§7.7.3.7, *"If hor_pic was
//! derived from a lower layer progressive frame, hor_pic is copied to
//! spat_pred_pic"*). The deinterlace / reinterlace filters that bracket
//! these two stages for the interlaced cases are out of scope here.
//!
//! ## The two stages (pages 113–114)
//!
//! Both stages perform the same phase-based linear interpolation between
//! two adjacent lower-layer sample sites, but the vertical stage defers
//! its `// 16` normalisation so the horizontal stage's single `// 256`
//! removes both stages' ×16 scaling at once.
//!
//! ### §7.7.3.5 Vertical resampling (`prog_pic` → `vert_pic`)
//!
//! ```text
//! vert_pic[yh + ll_v_offset][x] = (16 - phase) * prog_pic[y1][x]
//!                               +  phase        * prog_pic[y2][x]
//!   y1    = (yh * v_subs_m) / v_subs_n
//!   y2    = y1 + 1   if y1 < ll_v_size - 1, else y1
//!   phase = (16 * ((yh * v_subs_m) % v_subs_n)) // v_subs_n
//! ```
//!
//! The output is **not** divided here: `vert_pic` carries a ×16 scale
//! into the horizontal stage. `yh` is the output row index relative to
//! `ll_v_offset`; `yh + ll_v_offset` is the absolute row in `vert_pic`.
//!
//! ### §7.7.3.6 Horizontal resampling (`vert_pic` → `hor_pic`)
//!
//! ```text
//! hor_pic[y][xh + ll_h_offset] = ((16 - phase) * vert_pic[y][x1]
//!                               +  phase        * vert_pic[y][x2]) // 256
//!   x1    = (xh * h_subs_m) / h_subs_n
//!   x2    = x1 + 1   if x1 < ll_h_size - 1, else x1
//!   phase = (16 * ((xh * h_subs_m) % h_subs_n)) // h_subs_n
//! ```
//!
//! The `// 256` (= `// (16 * 16)`) folds the vertical-stage ×16 and the
//! horizontal-stage ×16 into one round-to-nearest-half-away-from-zero
//! division (§4.1 `//`).
//!
//! ## §4.1 operators (page 11)
//!
//! * `/` truncates **toward zero**; `%` is the paired modulus
//!   (*"defined only for positive numbers"*). Every operand in the index
//!   / phase formulae above is non-negative (sizes, subsampling factors,
//!   and output coordinates are all `>= 0`), so `/` and `%` reduce to
//!   plain unsigned integer `div` / `rem`.
//! * `//` rounds to the **nearest integer, half away from zero**. For a
//!   non-negative numerator `s` over a positive divisor `d` that is the
//!   canonical `(s + d/2) / d` (here `d/2` itself uses the `/`
//!   truncation, e.g. `256/2 = 128`).
//!
//! ## Border extension (pages 113–114)
//!
//! Both stages note: *"Samples which lie outside the lower layer
//! reconstructed frame which are required for upsampling are obtained by
//! border extension of the lower layer reconstructed frame."* The `y2` /
//! `x2` clamps (`y1 < ll_v_size - 1` etc.) already keep the *paired*
//! interpolation site inside the frame, but `y1` / `x1` themselves can
//! reach `ll_*_size - 1` when the enhancement grid extends past the
//! upsampled frame, so the sample reads clamp the index into
//! `0 ..= size - 1` (`PadEdge`). This matches the §7.6.4 pel reader's
//! pad-to-edge boundary mode.
//!
//! ## Table 7-16 local variables (page 111)
//!
//! The stages are driven by the six Table 7-16 local variables
//! (`ll_h_size`, `ll_v_size`, `ll_h_offset`, `ll_v_offset`, the four
//! `*_subs_*` factors) whose values differ for luminance vs chrominance
//! processing because the two components sit on different sampling grids
//! (Tables 7-17 / 7-18). [`ResampleParams::luminance`] and
//! [`ResampleParams::chrominance`] derive those values from the raw
//! `sequence_scalable_extension()` /
//! `picture_spatial_scalable_extension()` fields plus the lower- and
//! enhancement-layer `chroma_format`s; [`ResampleParams`] can also be
//! built field-by-field for the luminance case or for tests.
//!
//! Spec citations refer to **ISO/IEC 13818-2 (H.262)** §7.7.3.2 (the
//! pipeline overview + Figure 7-14), §7.7.3.3 + Tables 7-16 / 7-17 /
//! 7-18 (the local-variable derivation), §7.7.3.5 (vertical resampling),
//! §7.7.3.6 (horizontal resampling), and the §4.1 arithmetic operators.

use crate::sequence_extension::ChromaFormat;
use crate::{Error, Result};

/// The §7.7.3.3 / Table 7-16 local variables that drive one resampling
/// pass (one component — luminance or one chrominance plane).
///
/// All fields are post-Table-7-16 *output* values: for chrominance
/// processing the caller (or [`ResampleParams::chrominance`]) has
/// already applied the Table 7-16 `chroma_ratio` / `format_ratio`
/// divisions and multiplications, so the resampling math below treats
/// luma and chroma identically.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResampleParams {
    /// `ll_h_size` — Table 7-16: the lower-layer frame width (in
    /// samples of this component) the resampling reads from. For
    /// luminance this is `lower_layer_prediction_horizontal_size`; for
    /// chrominance it is divided by `chroma_ratio_horizontal[lower]`.
    pub ll_h_size: u32,
    /// `ll_v_size` — Table 7-16: the lower-layer frame height (in
    /// samples of this component).
    pub ll_v_size: u32,
    /// `ll_h_offset` — Table 7-16: the horizontal position (in
    /// enhancement-layer samples of this component) of the upsampled
    /// lower-layer frame's top-left corner within the enhancement
    /// frame. From `lower_layer_horizontal_offset`
    /// (`picture_spatial_scalable_extension()`), divided by
    /// `chroma_ratio_horizontal[enhance]` for chrominance.
    pub ll_h_offset: u32,
    /// `ll_v_offset` — Table 7-16: the vertical position of the
    /// upsampled frame's top-left corner.
    pub ll_v_offset: u32,
    /// `h_subs_m` — Table 7-16: the horizontal subsampling numerator
    /// (`horizontal_subsampling_factor_m`, unchanged for chroma).
    pub h_subs_m: u32,
    /// `h_subs_n` — Table 7-16: the horizontal subsampling denominator
    /// (`horizontal_subsampling_factor_n`, times `format_ratio_horizontal`
    /// for chroma).
    pub h_subs_n: u32,
    /// `v_subs_m` — Table 7-16: the vertical subsampling numerator
    /// (`vertical_subsampling_factor_m`, unchanged for chroma).
    pub v_subs_m: u32,
    /// `v_subs_n` — Table 7-16: the vertical subsampling denominator
    /// (`vertical_subsampling_factor_n`, times `format_ratio_vertical`
    /// for chroma).
    pub v_subs_n: u32,
}

/// Table 7-17 `chroma_ratio_horizontal[layer]` /
/// `chroma_ratio_vertical[layer]` for one chrominance format.
fn chroma_ratio(format: ChromaFormat) -> (u32, u32) {
    match format {
        // 4:2:0 → (2, 2); 4:2:2 → (2, 1); 4:4:4 → (1, 1).
        ChromaFormat::Yuv420 => (2, 2),
        ChromaFormat::Yuv422 => (2, 1),
        ChromaFormat::Yuv444 => (1, 1),
    }
}

/// Table 7-18 `(format_ratio_horizontal, format_ratio_vertical)` for the
/// `(lower, enhancement)` chrominance-format pair. The six listed rows
/// cover every `lower <= enhance` upsampling pair; any other pair is a
/// bitstream / configuration error.
fn format_ratio(lower: ChromaFormat, enhance: ChromaFormat) -> Result<(u32, u32)> {
    use ChromaFormat::{Yuv420, Yuv422, Yuv444};
    Ok(match (lower, enhance) {
        (Yuv420, Yuv420) => (1, 1),
        (Yuv420, Yuv422) => (1, 2),
        (Yuv420, Yuv444) => (2, 2),
        (Yuv422, Yuv422) => (1, 1),
        (Yuv422, Yuv444) => (2, 1),
        (Yuv444, Yuv444) => (1, 1),
        _ => {
            return Err(Error::InvalidBitstream(
                "spatial resampling: (lower, enhancement) chroma_format pair not in Table 7-18",
            ))
        }
    })
}

impl ResampleParams {
    /// Build a luminance-processing [`ResampleParams`] (Table 7-16
    /// *"value for luminance processing"* column) from the raw
    /// `sequence_scalable_extension()` /
    /// `picture_spatial_scalable_extension()` fields.
    ///
    /// `ll_h_offset` / `ll_v_offset` are the
    /// `lower_layer_horizontal_offset` / `lower_layer_vertical_offset`
    /// from `picture_spatial_scalable_extension()`. They are 15-bit
    /// signed twos-complement in the bitstream; per §7.7.3 a negative
    /// offset would place the upsampled frame's origin above / left of
    /// the enhancement frame, but the §7.7.3.5 / §7.7.3.6 output index
    /// `yh + ll_v_offset` / `xh + ll_h_offset` and the `%` operator
    /// (*"defined only for positive numbers"*) are framed for
    /// non-negative coordinates, so this constructor takes the offsets
    /// as `u32` and the caller resolves any negative-offset cropping
    /// before calling. The four subsampling factors are the
    /// `*_subsampling_factor_*` fields (each `1..=31`, zero forbidden
    /// per §6.3.7).
    #[allow(clippy::too_many_arguments)]
    pub fn luminance(
        ll_h_size: u32,
        ll_v_size: u32,
        ll_h_offset: u32,
        ll_v_offset: u32,
        h_subs_m: u32,
        h_subs_n: u32,
        v_subs_m: u32,
        v_subs_n: u32,
    ) -> Result<Self> {
        Self::new_checked(
            ll_h_size,
            ll_v_size,
            ll_h_offset,
            ll_v_offset,
            h_subs_m,
            h_subs_n,
            v_subs_m,
            v_subs_n,
        )
    }

    /// Build a chrominance-processing [`ResampleParams`] (Table 7-16
    /// *"value for chrominance processing"* column) by applying the
    /// Tables 7-17 / 7-18 `chroma_ratio` / `format_ratio` adjustments
    /// to the luminance-grid inputs.
    ///
    /// Per Table 7-16:
    /// * `ll_h_size /= chroma_ratio_horizontal[lower]`,
    ///   `ll_v_size /= chroma_ratio_vertical[lower]`;
    /// * `ll_h_offset /= chroma_ratio_horizontal[enhance]`,
    ///   `ll_v_offset /= chroma_ratio_vertical[enhance]`;
    /// * `h_subs_n *= format_ratio_horizontal`,
    ///   `v_subs_n *= format_ratio_vertical` (the `*_subs_m` factors
    ///   are unchanged);
    ///
    /// where `chroma_ratio` comes from Table 7-17 (keyed on the named
    /// layer's `chroma_format`) and `format_ratio` from Table 7-18
    /// (keyed on the `(lower, enhancement)` pair).
    ///
    /// Per §7.7.3.2, *"the lower layer offsets are limited to even
    /// values when the chrominance in the enhancement layer is
    /// subsampled in that dimension"* — i.e. the `ll_*_offset /
    /// chroma_ratio_*[enhance]` divisions are exact for conforming
    /// streams. The `/` truncation is applied regardless so a
    /// non-conforming odd offset still yields a defined (truncated)
    /// result rather than panicking.
    #[allow(clippy::too_many_arguments)]
    pub fn chrominance(
        ll_h_size: u32,
        ll_v_size: u32,
        ll_h_offset: u32,
        ll_v_offset: u32,
        h_subs_m: u32,
        h_subs_n: u32,
        v_subs_m: u32,
        v_subs_n: u32,
        lower_format: ChromaFormat,
        enhance_format: ChromaFormat,
    ) -> Result<Self> {
        let (cr_h_lower, cr_v_lower) = chroma_ratio(lower_format);
        let (cr_h_enh, cr_v_enh) = chroma_ratio(enhance_format);
        let (fr_h, fr_v) = format_ratio(lower_format, enhance_format)?;
        Self::new_checked(
            ll_h_size / cr_h_lower,
            ll_v_size / cr_v_lower,
            ll_h_offset / cr_h_enh,
            ll_v_offset / cr_v_enh,
            h_subs_m,
            h_subs_n * fr_h,
            v_subs_m,
            v_subs_n * fr_v,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn new_checked(
        ll_h_size: u32,
        ll_v_size: u32,
        ll_h_offset: u32,
        ll_v_offset: u32,
        h_subs_m: u32,
        h_subs_n: u32,
        v_subs_m: u32,
        v_subs_n: u32,
    ) -> Result<Self> {
        if ll_h_size == 0 || ll_v_size == 0 {
            return Err(Error::InvalidBitstream(
                "spatial resampling: lower-layer frame size must be non-zero (§7.7.3.5/.6)",
            ));
        }
        if h_subs_m == 0 || h_subs_n == 0 || v_subs_m == 0 || v_subs_n == 0 {
            return Err(Error::InvalidBitstream(
                "spatial resampling: subsampling factor zero is forbidden (§6.3.7)",
            ));
        }
        Ok(Self {
            ll_h_size,
            ll_v_size,
            ll_h_offset,
            ll_v_offset,
            h_subs_m,
            h_subs_n,
            v_subs_m,
            v_subs_n,
        })
    }
}

/// A simple row-major sample plane used as the resampling input /
/// intermediate. Sample type is `i32` so the ×16-scaled `vert_pic`
/// intermediate (range up to `255 * 16 = 4080`) fits without overflow
/// while the lower-layer `prog_pic` samples stay in `[0, 255]`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Plane {
    width: u32,
    height: u32,
    samples: Vec<i32>,
}

impl Plane {
    /// Build a plane from a row-major sample buffer. Returns
    /// [`Error::InvalidBitstream`] when `samples.len() != width *
    /// height` or either dimension is zero.
    pub fn new(width: u32, height: u32, samples: Vec<i32>) -> Result<Self> {
        if width == 0 || height == 0 {
            return Err(Error::InvalidBitstream(
                "spatial resampling: plane dimension must be non-zero",
            ));
        }
        let expected =
            (width as usize)
                .checked_mul(height as usize)
                .ok_or(Error::InvalidBitstream(
                    "spatial resampling: plane dimensions overflow usize",
                ))?;
        if samples.len() != expected {
            return Err(Error::InvalidBitstream(
                "spatial resampling: plane sample count does not match width * height",
            ));
        }
        Ok(Self {
            width,
            height,
            samples,
        })
    }

    /// Plane width in samples.
    pub fn width(&self) -> u32 {
        self.width
    }

    /// Plane height in samples.
    pub fn height(&self) -> u32 {
        self.height
    }

    /// Row-major sample slice.
    pub fn samples(&self) -> &[i32] {
        &self.samples
    }

    /// Sample at `(x, y)` with pad-to-edge clamping (border extension).
    /// `x` / `y` outside `[0, width)` / `[0, height)` clamp to the
    /// nearest in-bounds sample per the §7.7.3.5 / §7.7.3.6 border-
    /// extension rule.
    #[inline]
    fn sample_clamped(&self, x: i64, y: i64) -> i32 {
        let xc = x.clamp(0, i64::from(self.width) - 1) as u32;
        let yc = y.clamp(0, i64::from(self.height) - 1) as u32;
        self.samples[(yc as usize) * (self.width as usize) + (xc as usize)]
    }
}

/// §4.1 `//`: integer division rounding to the nearest integer, half
/// away from zero, for a **non-negative** numerator over a positive
/// divisor. (Every `//` site in §7.7.3.5 / §7.7.3.6 has a non-negative
/// numerator.)
#[inline]
fn div_round_half_up(numerator: i64, divisor: i64) -> i64 {
    debug_assert!(numerator >= 0 && divisor > 0);
    (numerator + divisor / 2) / divisor
}

/// §7.7.3.5 Vertical resampling: resample `prog_pic` onto the
/// enhancement-layer vertical sampling grid, producing the ×16-scaled
/// `vert_pic` field.
///
/// The output plane has the same width as `prog_pic` and a height of
/// `out_height` rows (the number of enhancement-layer rows of this
/// component the caller needs; for a whole-frame resample this is the
/// upsampled frame height `ll_v_size * v_subs_n / v_subs_m`, but the
/// caller passes it explicitly since for macroblock-granular decode only
/// the 16 rows covering a macroblock are formed, per §7.7.3).
///
/// Output row `yh` (`0..out_height`) maps to absolute `vert_pic` row
/// `yh + ll_v_offset` in the spec; this function returns a plane whose
/// row `0` corresponds to spec row `ll_v_offset` (i.e. the offset is the
/// caller's coordinate base, not stored as leading blank rows).
///
/// # Errors
/// * [`Error::InvalidBitstream`] if `out_height` is zero or the output
///   geometry overflows.
pub fn vertical_resample(
    prog_pic: &Plane,
    params: &ResampleParams,
    out_height: u32,
) -> Result<Plane> {
    if out_height == 0 {
        return Err(Error::InvalidBitstream(
            "vertical_resample: out_height must be non-zero (§7.7.3.5)",
        ));
    }
    let width = prog_pic.width();
    let v_subs_m = i64::from(params.v_subs_m);
    let v_subs_n = i64::from(params.v_subs_n);
    let ll_v_size = i64::from(params.ll_v_size);

    let mut out = vec![0_i32; (width as usize) * (out_height as usize)];
    for yh in 0..i64::from(out_height) {
        // y1 = (yh * v_subs_m) / v_subs_n  (§4.1 `/`, truncate toward 0;
        // non-negative so == floor).
        let prod = yh * v_subs_m;
        let y1 = prod / v_subs_n;
        // y2 = y1 + 1 if y1 < ll_v_size - 1 else y1.
        let y2 = if y1 < ll_v_size - 1 { y1 + 1 } else { y1 };
        // phase = (16 * ((yh * v_subs_m) % v_subs_n)) // v_subs_n.
        let rem = prod % v_subs_n;
        let phase = div_round_half_up(16 * rem, v_subs_n);
        let w1 = 16 - phase;
        let w0 = phase;
        let row_base = (yh as usize) * (width as usize);
        for x in 0..i64::from(width) {
            let s1 = i64::from(prog_pic.sample_clamped(x, y1));
            let s2 = i64::from(prog_pic.sample_clamped(x, y2));
            // §7.7.3.5: NO division here — vert_pic carries the ×16 scale.
            let v = w1 * s1 + w0 * s2;
            out[row_base + (x as usize)] = v as i32;
        }
    }
    Plane::new(width, out_height, out)
}

/// §7.7.3.6 Horizontal resampling: resample the ×16-scaled `vert_pic`
/// onto the enhancement-layer horizontal sampling grid, producing
/// `hor_pic` with the final `// 256` normalisation that removes both the
/// vertical and the horizontal ×16 scaling.
///
/// The output plane has the same height as `vert_pic` and a width of
/// `out_width` samples (see [`vertical_resample`] for the symmetric
/// row-count argument). Output column `xh` maps to spec column
/// `xh + ll_h_offset`. Output samples are saturated into `[0, 255]`
/// (the §7.7.3.4 deinterlace stage saturates to `[0:255]` and the
/// progressive-path `prog_pic` is itself an `[0, 255]` reconstructed
/// frame, so the bilinear blend of two such samples is already in range;
/// the clamp guards against any rounding excursion at the bounds).
///
/// # Errors
/// * [`Error::InvalidBitstream`] if `out_width` is zero or the output
///   geometry overflows.
pub fn horizontal_resample(
    vert_pic: &Plane,
    params: &ResampleParams,
    out_width: u32,
) -> Result<Plane> {
    if out_width == 0 {
        return Err(Error::InvalidBitstream(
            "horizontal_resample: out_width must be non-zero (§7.7.3.6)",
        ));
    }
    let height = vert_pic.height();
    let h_subs_m = i64::from(params.h_subs_m);
    let h_subs_n = i64::from(params.h_subs_n);
    let ll_h_size = i64::from(params.ll_h_size);

    let mut out = vec![0_i32; (out_width as usize) * (height as usize)];
    // Precompute per-column x1 / x2 / phase (they do not depend on y).
    let mut cols: Vec<(i64, i64, i64)> = Vec::with_capacity(out_width as usize);
    for xh in 0..i64::from(out_width) {
        let prod = xh * h_subs_m;
        let x1 = prod / h_subs_n;
        let x2 = if x1 < ll_h_size - 1 { x1 + 1 } else { x1 };
        let rem = prod % h_subs_n;
        let phase = div_round_half_up(16 * rem, h_subs_n);
        cols.push((x1, x2, phase));
    }
    for y in 0..i64::from(height) {
        let row_base = (y as usize) * (out_width as usize);
        for (xh_idx, &(x1, x2, phase)) in cols.iter().enumerate() {
            let s1 = i64::from(vert_pic.sample_clamped(x1, y));
            let s2 = i64::from(vert_pic.sample_clamped(x2, y));
            // §7.7.3.6: ((16 - phase)*s1 + phase*s2) // 256, removing
            // the vertical-stage ×16 and the horizontal-stage ×16.
            let num = (16 - phase) * s1 + phase * s2;
            let v = div_round_half_up(num, 256);
            out[row_base + xh_idx] = v.clamp(0, 255) as i32;
        }
    }
    Plane::new(out_width, height, out)
}

/// §7.7.3.5 + §7.7.3.6 composed: resample a progressive lower-layer
/// frame (`prog_pic`) into the enhancement-layer spatial prediction
/// region of size `out_width × out_height` samples of one component.
///
/// For the progressive-to-progressive case (Table 7-15 row 3, no
/// deinterlace / reinterlace), the result *is* `spat_pred_pic`
/// (§7.7.3.7, *"hor_pic is copied to spat_pred_pic"*) — the
/// `pel_pred_spat` input to the §7.7.4 combiner
/// ([`crate::spatial_temporal_combine`]). Output samples are in
/// `[0, 255]`.
///
/// # Errors
/// * Propagates the [`Error::InvalidBitstream`] geometry errors of the
///   two stages.
pub fn resample_progressive(
    prog_pic: &Plane,
    params: &ResampleParams,
    out_width: u32,
    out_height: u32,
) -> Result<Plane> {
    let vert = vertical_resample(prog_pic, params, out_height)?;
    horizontal_resample(&vert, params, out_width)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plane(width: u32, height: u32, samples: &[i32]) -> Plane {
        Plane::new(width, height, samples.to_vec()).expect("plane")
    }

    #[test]
    fn div_round_half_up_matches_spec_examples() {
        // §4.1: 3//2 = 2, and the non-negative round-half-away-from-zero
        // shape on /256 and /1.
        assert_eq!(div_round_half_up(3, 2), 2);
        assert_eq!(div_round_half_up(4, 2), 2);
        assert_eq!(div_round_half_up(5, 2), 3); // 2.5 -> 3
        assert_eq!(div_round_half_up(255, 1), 255);
        assert_eq!(div_round_half_up(128, 256), 1); // 0.5 -> 1
        assert_eq!(div_round_half_up(127, 256), 0); // 0.496 -> 0
    }

    #[test]
    fn identity_resample_1_to_1_reproduces_input() {
        // subs_m == subs_n == 1 and matching sizes: every phase is 0,
        // y1 == yh, x1 == xh, so hor_pic[y][x] = (16*s1)//256... wait:
        // vert = 16*s; hor = (16*16*s)//256 = s. Identity.
        let src = plane(3, 2, &[10, 20, 30, 40, 50, 60]);
        let params = ResampleParams::luminance(3, 2, 0, 0, 1, 1, 1, 1).expect("params");
        let out = resample_progressive(&src, &params, 3, 2).expect("resample");
        assert_eq!(out.width(), 3);
        assert_eq!(out.height(), 2);
        assert_eq!(out.samples(), &[10, 20, 30, 40, 50, 60]);
    }

    #[test]
    fn vertical_2x_upsample_midpoint_is_average() {
        // Upsample a 1-wide, 2-tall column [0, 100] by 2x vertically.
        // subs_m = 1, subs_n = 2 → out rows map: yh*1/2.
        //   yh=0: y1=0, phase=(16*0)//2=0  -> 16*src[0] = 0
        //   yh=1: y1=0, phase=(16*1)//2=8  -> 8*src[0] + 8*src[1] = 800
        //   yh=2: y1=1, phase=(16*0)//2=0  -> 16*src[1] = 1600
        //   yh=3: y1=1 (== ll_v_size-1 so y2=y1), phase=(16*1)//2=8
        //         -> 8*src[1] + 8*src[1] = 1600 (border extension)
        let src = plane(1, 2, &[0, 100]);
        let params = ResampleParams::luminance(1, 2, 0, 0, 1, 1, 1, 2).expect("params");
        let vert = vertical_resample(&src, &params, 4).expect("vert");
        assert_eq!(vert.samples(), &[0, 800, 1600, 1600]);
        // Horizontal stage with phase==0 multiplies by (16 - phase) = 16,
        // so num = 16 * vert; // 256 = vert / 16, recovering the true
        // sample scale. 0/16=0, 800*16//256 = 12800//256 = 50,
        // 1600*16//256 = 25600//256 = 100, 1600 -> 100. The yh=1 row is
        // the exact midpoint of [0, 100].
        let out = horizontal_resample(&vert, &params, 1).expect("hor");
        assert_eq!(out.samples(), &[0, 50, 100, 100]);
    }

    #[test]
    fn horizontal_2x_upsample_midpoint_is_average() {
        // 2-wide, 1-tall row [0, 160] upsampled 2x horizontally. The
        // vertical stage is identity (subs 1/1, ll_v_size 1) so vert =
        // 16*src. Horizontal subs_m=1, subs_n=2:
        //   xh=0: x1=0, phase=0  -> (16*0) = 0;          //256 = 0
        //   xh=1: x1=0, phase=8  -> 8*0 + 8*2560 = 20480; //256 = 80
        //   xh=2: x1=1, phase=0  -> 16*2560 = 40960;      //256 = 160
        //   xh=3: x1=1 (border), phase=8 -> 16*2560;      //256 = 160
        // where 16*160 = 2560.
        let src = plane(2, 1, &[0, 160]);
        let params = ResampleParams::luminance(2, 1, 0, 0, 1, 2, 1, 1).expect("params");
        let out = resample_progressive(&src, &params, 4, 1).expect("resample");
        assert_eq!(out.samples(), &[0, 80, 160, 160]);
    }

    #[test]
    fn border_extension_clamps_past_frame_edge() {
        // out_height beyond the upsampled frame must clamp y1/y2 to the
        // last lower-layer row (border extension), not index out of
        // bounds.
        let src = plane(1, 2, &[40, 80]);
        let params = ResampleParams::luminance(1, 2, 0, 0, 1, 1, 1, 1).expect("params");
        // Ask for 4 rows from a 2-row source at 1:1 → rows 2,3 clamp.
        let vert = vertical_resample(&src, &params, 4).expect("vert");
        // yh=0:y1=0 ->16*40; yh=1:y1=1 ->16*80; yh=2:y1=2 clamp->row1=80
        //   ->16*80; yh=3:y1=3 clamp->80 ->16*80.
        assert_eq!(vert.samples(), &[640, 1280, 1280, 1280]);
    }

    #[test]
    fn chrominance_applies_table_7_16_ratios_420_to_420() {
        // 4:2:0 lower & enhance: chroma_ratio = (2,2) both layers,
        // format_ratio = (1,1). A luma ll size of (8, 8), offset (4, 4)
        // becomes chroma ll size (4, 4), offset (2, 2); subs unchanged.
        let params = ResampleParams::chrominance(
            8,
            8,
            4,
            4,
            1,
            1,
            1,
            1,
            ChromaFormat::Yuv420,
            ChromaFormat::Yuv420,
        )
        .expect("chroma params");
        assert_eq!(params.ll_h_size, 4);
        assert_eq!(params.ll_v_size, 4);
        assert_eq!(params.ll_h_offset, 2);
        assert_eq!(params.ll_v_offset, 2);
        assert_eq!(params.h_subs_n, 1);
        assert_eq!(params.v_subs_n, 1);
    }

    #[test]
    fn chrominance_420_to_444_format_ratio_scales_subs_n() {
        // Table 7-18: (4:2:0 lower, 4:4:4 enhance) → format_ratio (2,2).
        // chroma_ratio[lower=420] = (2,2); chroma_ratio[enh=444] = (1,1).
        // ll size /2 (lower), offset /1 (enhance), subs_n *2.
        let params = ResampleParams::chrominance(
            16,
            16,
            4,
            4,
            1,
            3,
            1,
            3,
            ChromaFormat::Yuv420,
            ChromaFormat::Yuv444,
        )
        .expect("chroma params");
        assert_eq!(params.ll_h_size, 8); // 16 / 2
        assert_eq!(params.ll_v_size, 8);
        assert_eq!(params.ll_h_offset, 4); // 4 / 1
        assert_eq!(params.ll_v_offset, 4);
        assert_eq!(params.h_subs_n, 6); // 3 * 2
        assert_eq!(params.v_subs_n, 6);
        assert_eq!(params.h_subs_m, 1); // unchanged
        assert_eq!(params.v_subs_m, 1);
    }

    #[test]
    fn rejects_unlisted_chroma_format_pair() {
        // (4:4:4 lower, 4:2:0 enhance) is a downsample, not in Table 7-18.
        let err = ResampleParams::chrominance(
            8,
            8,
            0,
            0,
            1,
            1,
            1,
            1,
            ChromaFormat::Yuv444,
            ChromaFormat::Yuv420,
        )
        .unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_zero_subsampling_factor() {
        let err = ResampleParams::luminance(4, 4, 0, 0, 0, 1, 1, 1).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_zero_lower_layer_size() {
        let err = ResampleParams::luminance(0, 4, 0, 0, 1, 1, 1, 1).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn plane_rejects_mismatched_sample_count() {
        let err = Plane::new(2, 2, vec![1, 2, 3]).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_zero_out_dimensions() {
        let src = plane(2, 2, &[1, 2, 3, 4]);
        let params = ResampleParams::luminance(2, 2, 0, 0, 1, 1, 1, 1).expect("params");
        assert!(matches!(
            vertical_resample(&src, &params, 0),
            Err(Error::InvalidBitstream(_))
        ));
        let vert = vertical_resample(&src, &params, 2).expect("vert");
        assert!(matches!(
            horizontal_resample(&vert, &params, 0),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn full_2x2_upsample_blends_both_axes() {
        // 2x2 source upsampled to 4x4 (subs 1/2 both axes). Spot-check
        // the true interior midpoint (xh=1, yh=1) which blends all four
        // corners. src = [[0,160],[160,0]] (a checkerboard).
        // vert stage (subs_m=1,subs_n=2, ll_v_size=2):
        //   yh=0:y1=0,phase=0 -> 16*row0
        //   yh=1:y1=0,phase=8 -> 8*row0 + 8*row1
        //   yh=2:y1=1,phase=0 -> 16*row1
        //   yh=3:y1=1(border),phase=8 -> 16*row1
        // row0=[0,160], row1=[160,0].
        //   vert row1 (yh=1) = 8*[0,160] + 8*[160,0] = [1280, 1280]
        // hor stage on vert row1 (subs_m=1,subs_n=2, ll_h_size=2):
        //   xh=1: x1=0,phase=8 -> (8*1280 + 8*1280)//256 = 20480//256=80
        let src = plane(2, 2, &[0, 160, 160, 0]);
        let params = ResampleParams::luminance(2, 2, 0, 0, 1, 2, 1, 2).expect("params");
        let out = resample_progressive(&src, &params, 4, 4).expect("resample");
        assert_eq!(out.width(), 4);
        assert_eq!(out.height(), 4);
        // Centre sample (row 1, col 1) is the four-corner blend = 80.
        let centre = out.samples()[4 + 1];
        assert_eq!(centre, 80);
        // Corner (0,0) is exact source corner 0.
        assert_eq!(out.samples()[0], 0);
        // (yh=2,xh=2) maps to source (1,1) exactly = 0.
        assert_eq!(out.samples()[2 * 4 + 2], 0);
    }
}
