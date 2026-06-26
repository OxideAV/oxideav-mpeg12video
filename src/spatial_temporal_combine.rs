//! §7.7.4 Selection and combination of spatial and temporal predictions
//! per ISO/IEC 13818-2 (Recommendation ITU-T H.262), pages 114–115 of
//! the 1995 base text — the *"precise method for predictor calculation"*
//! that combines a temporal enhancement-layer prediction
//! (`pel_pred_temp`) with a spatial lower-layer prediction
//! (`pel_pred_spat`) into the final enhancement-layer prediction
//! (`pel_pred`) using the `spatial_temporal_weight` resolved from
//! Table 7-21.
//!
//! ## What §7.7.4 specifies
//!
//! In a spatially-scalable enhancement layer the prediction for a
//! macroblock can be temporal-only, spatial-only, or a per-field
//! weighted blend of the two. The weighting is the
//! `spatial_temporal_weight(s)` column of Table 7-21 — resolved by
//! [`crate::macroblock_modes::SpatialTemporalWeight`] from
//! `(spatial_temporal_weight_code_table_index,
//! spatial_temporal_weight_code)` — where each weight is the proportion
//! of the prediction taken from the **spatial** (lower-layer)
//! prediction. The three legal weight values and their per-sample
//! formulae are (page 115):
//!
//! ```text
//! weight 0   →  pel_pred[y][x] = pel_pred_temp[y][x];                          (temporal-only)
//! weight 1   →  pel_pred[y][x] = pel_pred_spat[y][x];                          (spatial-only)
//! weight 0,5 →  pel_pred[y][x] = (pel_pred_temp[y][x] + pel_pred_spat[y][x])//2; (average)
//! ```
//!
//! The `//` operator is §4.1 integer division rounding half-integer
//! values away from zero; for the sum of two unsigned-8-bit samples
//! (range `[0, 510]`) this is the canonical `(sum + 1) >> 1` — the same
//! `avg2` the §7.6.7 bidirectional combiner uses, kept identical here.
//!
//! ## Two weight forms (Table 7-21)
//!
//! * **Single `(a)` form** (`spatial_temporal_weight_code_table_index ==
//!   '00'`, [`SpatialTemporalWeight::is_single`]): one weight applies to
//!   the whole macroblock prediction — *"`a` gives the proportion of the
//!   prediction for the picture which is derived from the spatial
//!   prediction for that picture"*. Use [`combine_uniform`].
//! * **Per-field `(a; b)` form** (the other table indices): *"`a` gives
//!   the proportion of the prediction for the top field … and `b` gives
//!   the proportion … for the bottom field"*. Within the macroblock the
//!   top-field lines are the even rows (`y` = 0, 2, 4, …) and the
//!   bottom-field lines the odd rows. Use [`combine_field_interleaved`].
//!   Per the page-115 note, *"When progressive_frame == 0 chrominance is
//!   treated as interlaced"* — the same even/odd-row split applies to a
//!   chroma block; the caller selects [`combine_field_interleaved`] for
//!   chroma exactly when `progressive_frame == 0`.
//!
//! The §7.7.4 weight values are restricted to `{0, 0,5, 1}` (Table 7-21
//! lists no other), which this module encodes as the sixteenths
//! `{0, 8, 16}` carried by `SpatialTemporalWeight`. Any other sixteenths
//! value is a caller/table bug and is rejected.
//!
//! After this combination, *"Addition of prediction and coefficient
//! data is then done as in 7.6.8"* — i.e. the produced `pel_pred` block
//! feeds [`crate::add_coefficients`] unchanged, exactly as a non-scalable
//! prediction would.
//!
//! ## Scope
//!
//! This module performs only the §7.7.4 per-sample *combination*. It
//! does **not** form `pel_pred_temp` (that is §7.6, via
//! [`crate::forming_predictions`] / [`crate::combine_predictions`]) nor
//! `pel_pred_spat` (the §7.7.3 resampling of the lower-layer frame,
//! not yet composed in this crate). Both inputs are supplied as
//! equal-length row-major sample blocks of the same geometry.
//!
//! Spec citations refer to **ISO/IEC 13818-2 (H.262) §7.7.4** plus the
//! §4.1 arithmetic operators (`//`) and the Table 7-21 weight column.

use crate::macroblock_modes::SpatialTemporalWeight;
use crate::{Error, Result};

/// One §7.7.4 spatial weight — the proportion of a prediction taken
/// from the **spatial** (lower-layer) prediction. Table 7-21 admits
/// only the three values `{0, 0,5, 1}`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpatialWeight {
    /// Weight `0`: temporal-only — `pel_pred = pel_pred_temp`.
    Temporal,
    /// Weight `0,5`: the `// 2` average of the two predictions.
    Half,
    /// Weight `1`: spatial-only — `pel_pred = pel_pred_spat`.
    Spatial,
}

impl SpatialWeight {
    /// Map a Table 7-21 sixteenths weight (`0`, `8` = 0,5, `16` = 1)
    /// to a [`SpatialWeight`]. Returns [`Error::InvalidBitstream`] for
    /// any other value — Table 7-21 lists no weight outside
    /// `{0, 0,5, 1}`.
    pub fn from_sixteenths(weight: u8) -> Result<Self> {
        match weight {
            0 => Ok(SpatialWeight::Temporal),
            8 => Ok(SpatialWeight::Half),
            16 => Ok(SpatialWeight::Spatial),
            _ => Err(Error::InvalidBitstream(
                "spatial_temporal_weight: not one of {0, 0.5, 1} (Table 7-21, §7.7.4)",
            )),
        }
    }

    /// Combine one temporal sample with one spatial sample under this
    /// weight, per the page-115 per-sample formulae.
    #[inline]
    pub fn combine_sample(self, temporal: u8, spatial: u8) -> u8 {
        match self {
            SpatialWeight::Temporal => temporal,
            SpatialWeight::Spatial => spatial,
            // §7.7.4 / §4.1: (temp + spat) // 2 — half-integers rounded
            // away from zero; on a non-negative sum that is (sum+1)>>1.
            SpatialWeight::Half => ((temporal as u16 + spatial as u16 + 1) >> 1) as u8,
        }
    }
}

/// §7.7.4 single `(a)` form: apply one [`SpatialWeight`] uniformly to
/// every sample of the macroblock prediction. Used for the
/// `spatial_temporal_weight_code_table_index == '00'` row, and for any
/// block where both fields share the same weight.
///
/// `temporal` and `spatial` must be the same length (the same §7.6.4 /
/// §7.7.3 block geometry); the output is allocated fresh with that
/// length. Returns [`Error::InvalidBitstream`] on a length mismatch
/// (a caller bug — both predictions are co-located, same geometry).
pub fn combine_uniform(weight: SpatialWeight, temporal: &[u8], spatial: &[u8]) -> Result<Vec<u8>> {
    if temporal.len() != spatial.len() {
        return Err(Error::InvalidBitstream(
            "spatial/temporal prediction blocks differ in length (§7.7.4)",
        ));
    }
    Ok(temporal
        .iter()
        .zip(spatial.iter())
        .map(|(&t, &s)| weight.combine_sample(t, s))
        .collect())
}

/// §7.7.4 per-field `(a; b)` form: apply `top_weight` to the top-field
/// lines (even rows `y = 0, 2, 4, …`) and `bottom_weight` to the
/// bottom-field lines (odd rows), per *"`a` … for the top field … `b`
/// … for the bottom field"*.
///
/// The block is row-major with `width` samples per row; its length must
/// be a multiple of `width` and equal for both inputs. Returns
/// [`Error::InvalidBitstream`] on a geometry mismatch or a zero width.
///
/// For chrominance this is invoked exactly when `progressive_frame ==
/// 0` (the page-115 note: *"chrominance is treated as interlaced"*);
/// for luminance the per-field form always uses this row split.
pub fn combine_field_interleaved(
    top_weight: SpatialWeight,
    bottom_weight: SpatialWeight,
    width: usize,
    temporal: &[u8],
    spatial: &[u8],
) -> Result<Vec<u8>> {
    if width == 0 {
        return Err(Error::InvalidBitstream(
            "spatial/temporal combine: zero block width (§7.7.4)",
        ));
    }
    if temporal.len() != spatial.len() {
        return Err(Error::InvalidBitstream(
            "spatial/temporal prediction blocks differ in length (§7.7.4)",
        ));
    }
    if temporal.len() % width != 0 {
        return Err(Error::InvalidBitstream(
            "spatial/temporal combine: block length not a multiple of width (§7.7.4)",
        ));
    }
    let mut out = Vec::with_capacity(temporal.len());
    for (row, (t_row, s_row)) in temporal
        .chunks_exact(width)
        .zip(spatial.chunks_exact(width))
        .enumerate()
    {
        // Even rows = top field; odd rows = bottom field.
        let weight = if row % 2 == 0 {
            top_weight
        } else {
            bottom_weight
        };
        for (&t, &s) in t_row.iter().zip(s_row.iter()) {
            out.push(weight.combine_sample(t, s));
        }
    }
    Ok(out)
}

/// §7.7.4 driver: combine `pel_pred_temp` with `pel_pred_spat` for one
/// macroblock-component block under a resolved Table 7-21
/// [`SpatialTemporalWeight`].
///
/// * For the single `(a)` form ([`SpatialTemporalWeight::is_single`])
///   the `top_weight` is applied uniformly via [`combine_uniform`];
///   `width` is unused.
/// * For the per-field `(a; b)` form `top_weight` / `bottom_weight` are
///   applied to even / odd rows via [`combine_field_interleaved`].
///
/// `width` is the block's row stride in samples (e.g. 16 for a luma
/// macroblock, 8 for a 4:2:0 chroma block); it is consulted only for
/// the per-field form. The §7.7.4 weight class `0` / `4` cases
/// (temporal-only / spatial-only signalled by `macroblock_type`, not by
/// a `spatial_temporal_weight_code`) are not represented by a
/// `SpatialTemporalWeight` and are handled by the caller before this
/// driver — a resolved `SpatialTemporalWeight` only ever carries the
/// Table 7-21 weight classes `1`, `2`, `3`.
pub fn combine_spatial_temporal(
    weight: &SpatialTemporalWeight,
    width: usize,
    temporal: &[u8],
    spatial: &[u8],
) -> Result<Vec<u8>> {
    let top = SpatialWeight::from_sixteenths(weight.top_weight)?;
    if weight.is_single {
        return combine_uniform(top, temporal, spatial);
    }
    let bottom = SpatialWeight::from_sixteenths(weight.bottom_weight)?;
    combine_field_interleaved(top, bottom, width, temporal, spatial)
}

/// Extract `pel_pred_spat` for one macroblock-sized region — the §7.7.4
/// *"appropriate samples, co-located with the current macroblock
/// position, from spat_pred_pic"* step.
///
/// `spat_pred_pic` is one component plane of the enhancement-grid
/// [`crate::SpatialPredictionPicture`]; `(base_x, base_y)` is the
/// top-left sample coordinate of the region in that plane (the
/// macroblock's pixel position for luma, or its chroma-subsampled
/// position for a chroma plane); `(width, height)` the region size
/// (16×16 for a luma macroblock, the chroma footprint otherwise).
///
/// Samples are read with §7.7.3.5 / §7.7.3.6 pad-to-edge border
/// extension ([`crate::ResamplePlane::get_clamped`]) so a macroblock
/// whose footprint runs past the bottom/right edge of `spat_pred_pic`
/// reads the edge sample, matching the spatial-prediction picture's own
/// border-extension convention. Output is row-major, clamped to the
/// `[0, 255]` sample range (the spatial prediction is always an 8-bit
/// reconstructed plane).
pub fn extract_colocated_spatial(
    spat_pred_pic: &crate::ResamplePlane,
    base_x: usize,
    base_y: usize,
    width: usize,
    height: usize,
) -> Vec<u8> {
    let mut out = Vec::with_capacity(width * height);
    for dy in 0..height {
        for dx in 0..width {
            let s = spat_pred_pic.get_clamped((base_x + dx) as i64, (base_y + dy) as i64);
            out.push(s.clamp(0, 255) as u8);
        }
    }
    out
}

/// §7.7.4 per-macroblock driver: combine a macroblock's temporal
/// prediction (`pel_pred_temp`, formed in the enhancement layer per
/// §7.6) with the co-located spatial prediction extracted from
/// `spat_pred_pic`, under a resolved Table 7-21
/// [`SpatialTemporalWeight`].
///
/// This composes [`extract_colocated_spatial`] with
/// [`combine_spatial_temporal`]: it reads the `width × height` region of
/// `spat_pred_pic` at the macroblock's `(base_x, base_y)` position and
/// blends it with `temporal` per the weight (single `(a)` or per-field
/// `(a; b)` form). `temporal` must hold `width * height` row-major
/// samples.
///
/// The §7.7.4 weight-class `0` (temporal-only) and `4` (spatial-only)
/// cases are signalled by `macroblock_type`, not by a
/// `spatial_temporal_weight_code`, so they are not represented by a
/// [`SpatialTemporalWeight`] and are handled by the caller (a class-0
/// macroblock keeps `temporal` unchanged; a class-4 macroblock uses the
/// extracted spatial block directly). A resolved
/// [`SpatialTemporalWeight`] here only carries weight classes `1`, `2`,
/// `3`.
///
/// # Errors
/// * [`Error::InvalidBitstream`] if `temporal.len() != width * height`,
///   or on a geometry error propagated from [`combine_spatial_temporal`].
#[allow(clippy::too_many_arguments)]
pub fn combine_macroblock_spatial_temporal(
    weight: &SpatialTemporalWeight,
    spat_pred_pic: &crate::ResamplePlane,
    base_x: usize,
    base_y: usize,
    width: usize,
    height: usize,
    temporal: &[u8],
) -> Result<Vec<u8>> {
    if temporal.len() != width * height {
        return Err(Error::InvalidBitstream(
            "combine_macroblock_spatial_temporal: temporal block size != width * height (§7.7.4)",
        ));
    }
    let spatial = extract_colocated_spatial(spat_pred_pic, base_x, base_y, width, height);
    combine_spatial_temporal(weight, width, temporal, &spatial)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- SpatialWeight::from_sixteenths ----

    #[test]
    fn from_sixteenths_maps_the_three_legal_values() {
        assert_eq!(
            SpatialWeight::from_sixteenths(0).unwrap(),
            SpatialWeight::Temporal
        );
        assert_eq!(
            SpatialWeight::from_sixteenths(8).unwrap(),
            SpatialWeight::Half
        );
        assert_eq!(
            SpatialWeight::from_sixteenths(16).unwrap(),
            SpatialWeight::Spatial
        );
    }

    #[test]
    fn from_sixteenths_rejects_other_values() {
        for w in [1u8, 4, 7, 9, 15, 17, 32, 255] {
            assert!(matches!(
                SpatialWeight::from_sixteenths(w),
                Err(Error::InvalidBitstream(_))
            ));
        }
    }

    // ---- combine_sample: page-115 formulae ----

    #[test]
    fn combine_sample_temporal_returns_temporal() {
        assert_eq!(SpatialWeight::Temporal.combine_sample(40, 200), 40);
    }

    #[test]
    fn combine_sample_spatial_returns_spatial() {
        assert_eq!(SpatialWeight::Spatial.combine_sample(40, 200), 200);
    }

    #[test]
    fn combine_sample_half_is_rounded_average() {
        // (40 + 200) // 2 = 120
        assert_eq!(SpatialWeight::Half.combine_sample(40, 200), 120);
        // (10 + 11) // 2 = 11 (half-integer rounds away from zero)
        assert_eq!(SpatialWeight::Half.combine_sample(10, 11), 11);
        // (255 + 255) // 2 = 255 (no overflow)
        assert_eq!(SpatialWeight::Half.combine_sample(255, 255), 255);
        // (0 + 1) // 2 = 1
        assert_eq!(SpatialWeight::Half.combine_sample(0, 1), 1);
    }

    #[test]
    fn combine_sample_half_matches_avg_formula_exhaustively() {
        for t in 0u8..=255 {
            for s in 0u8..=255 {
                let expected = ((t as u16 + s as u16 + 1) >> 1) as u8;
                assert_eq!(SpatialWeight::Half.combine_sample(t, s), expected);
            }
        }
    }

    // ---- combine_uniform ----

    #[test]
    fn uniform_temporal_only_copies_temporal() {
        let t = vec![1u8, 2, 3, 4];
        let s = vec![100u8, 100, 100, 100];
        let out = combine_uniform(SpatialWeight::Temporal, &t, &s).unwrap();
        assert_eq!(out, t);
    }

    #[test]
    fn uniform_spatial_only_copies_spatial() {
        let t = vec![1u8, 2, 3, 4];
        let s = vec![100u8, 110, 120, 130];
        let out = combine_uniform(SpatialWeight::Spatial, &t, &s).unwrap();
        assert_eq!(out, s);
    }

    #[test]
    fn uniform_half_averages_each_sample() {
        let t = vec![10u8, 20, 30, 40];
        let s = vec![20u8, 30, 40, 50];
        let out = combine_uniform(SpatialWeight::Half, &t, &s).unwrap();
        assert_eq!(out, vec![15, 25, 35, 45]);
    }

    #[test]
    fn uniform_rejects_length_mismatch() {
        let t = vec![1u8, 2, 3];
        let s = vec![4u8, 5];
        assert!(matches!(
            combine_uniform(SpatialWeight::Half, &t, &s),
            Err(Error::InvalidBitstream(_))
        ));
    }

    // ---- combine_field_interleaved ----

    #[test]
    fn interleaved_applies_top_weight_to_even_rows() {
        // 2 rows × 2 cols. Row 0 = top (spatial-only), row 1 = bottom
        // (temporal-only).
        let t = vec![10u8, 11, /* row1 */ 12, 13];
        let s = vec![200u8, 201, /* row1 */ 202, 203];
        let out = combine_field_interleaved(
            SpatialWeight::Spatial,  // top
            SpatialWeight::Temporal, // bottom
            2,
            &t,
            &s,
        )
        .unwrap();
        // row0 → spatial; row1 → temporal.
        assert_eq!(out, vec![200, 201, 12, 13]);
    }

    #[test]
    fn interleaved_half_top_temporal_bottom() {
        // row0 = half-average, row1 = temporal-only.
        let t = vec![10u8, 20, /* row1 */ 30, 40];
        let s = vec![20u8, 30, /* row1 */ 200, 210];
        let out =
            combine_field_interleaved(SpatialWeight::Half, SpatialWeight::Temporal, 2, &t, &s)
                .unwrap();
        // row0 → (10+20)//2, (20+30)//2 = 15, 25; row1 → 30, 40.
        assert_eq!(out, vec![15, 25, 30, 40]);
    }

    #[test]
    fn interleaved_four_rows_alternate() {
        // 4 rows × 1 col; top=spatial, bottom=temporal.
        let t = vec![1u8, 2, 3, 4];
        let s = vec![91u8, 92, 93, 94];
        let out =
            combine_field_interleaved(SpatialWeight::Spatial, SpatialWeight::Temporal, 1, &t, &s)
                .unwrap();
        // rows 0,2 spatial; rows 1,3 temporal.
        assert_eq!(out, vec![91, 2, 93, 4]);
    }

    #[test]
    fn interleaved_rejects_zero_width() {
        assert!(matches!(
            combine_field_interleaved(SpatialWeight::Half, SpatialWeight::Half, 0, &[], &[]),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn interleaved_rejects_length_mismatch() {
        let t = vec![1u8, 2, 3, 4];
        let s = vec![1u8, 2];
        assert!(matches!(
            combine_field_interleaved(SpatialWeight::Half, SpatialWeight::Half, 2, &t, &s),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn interleaved_rejects_non_multiple_of_width() {
        let t = vec![1u8, 2, 3];
        let s = vec![1u8, 2, 3];
        assert!(matches!(
            combine_field_interleaved(SpatialWeight::Half, SpatialWeight::Half, 2, &t, &s),
            Err(Error::InvalidBitstream(_))
        ));
    }

    // ---- combine_spatial_temporal driver ----

    fn single(top: u8) -> SpatialTemporalWeight {
        SpatialTemporalWeight {
            weight_class: 1,
            top_weight: top,
            bottom_weight: 0,
            is_single: true,
            integer_weight: false,
        }
    }

    fn paired(top: u8, bottom: u8, class: u8) -> SpatialTemporalWeight {
        SpatialTemporalWeight {
            weight_class: class,
            top_weight: top,
            bottom_weight: bottom,
            is_single: false,
            integer_weight: false,
        }
    }

    #[test]
    fn driver_single_form_applies_uniformly() {
        // table_index 00 row: single (0,5) → class 1 half average.
        let w = single(8);
        let t = vec![10u8, 20, 30, 40];
        let s = vec![20u8, 30, 40, 50];
        let out = combine_spatial_temporal(&w, 2, &t, &s).unwrap();
        assert_eq!(out, vec![15, 25, 35, 45]);
    }

    #[test]
    fn driver_single_form_ignores_width() {
        // is_single → width unused; a "wrong" width still yields the
        // uniform result.
        let w = single(16); // spatial-only
        let t = vec![1u8, 2, 3];
        let s = vec![7u8, 8, 9];
        let out = combine_spatial_temporal(&w, 999, &t, &s).unwrap();
        assert_eq!(out, s);
    }

    #[test]
    fn driver_paired_form_splits_by_row_parity() {
        // Table 7-21 index 10 code 00: (1; 0) class 2 — top spatial,
        // bottom temporal. top_weight 16, bottom_weight 0.
        let w = paired(16, 0, 2);
        let t = vec![10u8, 11, /* row1 */ 12, 13];
        let s = vec![200u8, 201, /* row1 */ 202, 203];
        let out = combine_spatial_temporal(&w, 2, &t, &s).unwrap();
        // row0 spatial, row1 temporal.
        assert_eq!(out, vec![200, 201, 12, 13]);
    }

    #[test]
    fn driver_paired_form_matches_table_7_21_index_01_code_00() {
        // index 01 code 00: (0; 1) class 3 — top temporal, bottom
        // spatial. top_weight 0, bottom_weight 16.
        let w = paired(0, 16, 3);
        let t = vec![5u8, 6, /* row1 */ 7, 8];
        let s = vec![55u8, 66, /* row1 */ 77, 88];
        let out = combine_spatial_temporal(&w, 2, &t, &s).unwrap();
        // row0 temporal (5,6); row1 spatial (77,88).
        assert_eq!(out, vec![5, 6, 77, 88]);
    }

    #[test]
    fn driver_rejects_illegal_weight_in_resolved_struct() {
        let mut w = single(8);
        w.top_weight = 5; // not 0/8/16
        let t = vec![1u8, 2];
        let s = vec![3u8, 4];
        assert!(matches!(
            combine_spatial_temporal(&w, 2, &t, &s),
            Err(Error::InvalidBitstream(_))
        ));
    }

    // ---- extract_colocated_spatial / combine_macroblock_spatial_temporal ----

    fn rplane(width: u32, height: u32, samples: &[i32]) -> crate::ResamplePlane {
        crate::ResamplePlane::new(width, height, samples.to_vec()).expect("plane")
    }

    #[test]
    fn extract_colocated_reads_the_region_at_the_macroblock_position() {
        // 4×3 picture; extract the 2×2 region at (1, 1).
        let pic = rplane(4, 3, &[0, 1, 2, 3, 10, 11, 12, 13, 20, 21, 22, 23]);
        let blk = extract_colocated_spatial(&pic, 1, 1, 2, 2);
        assert_eq!(blk, vec![11, 12, 21, 22]);
    }

    #[test]
    fn extract_colocated_pads_to_edge_past_the_picture_bounds() {
        // 2×2 picture; a 3×3 region at (1, 1) runs off the bottom/right
        // edge — the off-edge samples clamp to the nearest in-bounds one.
        let pic = rplane(2, 2, &[1, 2, 3, 4]);
        let blk = extract_colocated_spatial(&pic, 1, 1, 3, 3);
        // Row at y=1: x=1->4, x=2->4(clamp), x=3->4(clamp).
        // Rows y=2,3 clamp to y=1.
        assert_eq!(blk, vec![4, 4, 4, 4, 4, 4, 4, 4, 4]);
    }

    #[test]
    fn extract_colocated_clamps_negative_overshoot_samples_to_u8() {
        // A resampled plane sample below 0 / above 255 (shouldn't occur
        // for a valid reconstruction, but the extractor is defensive).
        let pic = rplane(2, 1, &[-5, 300]);
        let blk = extract_colocated_spatial(&pic, 0, 0, 2, 1);
        assert_eq!(blk, vec![0, 255]);
    }

    #[test]
    fn macroblock_combine_single_half_averages_colocated_block() {
        // 2×2 spat_pred_pic; single 0,5 weight averages temporal with the
        // co-located spatial block.
        let pic = rplane(2, 2, &[100, 100, 100, 100]);
        let w = single(8); // 0,5
        let temporal = vec![0u8, 50, 200, 254];
        let out = combine_macroblock_spatial_temporal(&w, &pic, 0, 0, 2, 2, &temporal).unwrap();
        // (0+100)//2=50, (50+100)//2=75, (200+100)//2=150, (254+100)//2=177
        assert_eq!(out, vec![50, 75, 150, 177]);
    }

    #[test]
    fn macroblock_combine_single_spatial_uses_colocated_block_directly() {
        let pic = rplane(4, 4, &(0..16).collect::<Vec<i32>>());
        let w = single(16); // weight 1 — spatial-only
                            // Extract the 2×2 region at (2, 2): rows {10,11},{14,15}.
        let temporal = vec![0u8; 4];
        let out = combine_macroblock_spatial_temporal(&w, &pic, 2, 2, 2, 2, &temporal).unwrap();
        assert_eq!(out, vec![10, 11, 14, 15]);
    }

    #[test]
    fn macroblock_combine_paired_splits_colocated_block_by_field() {
        // 2×2 region; paired (1; 0): top row spatial, bottom row temporal.
        let pic = rplane(2, 2, &[200, 201, 202, 203]);
        let w = paired(16, 0, 2);
        let temporal = vec![10u8, 11, 12, 13];
        let out = combine_macroblock_spatial_temporal(&w, &pic, 0, 0, 2, 2, &temporal).unwrap();
        // row0 spatial (200,201); row1 temporal (12,13).
        assert_eq!(out, vec![200, 201, 12, 13]);
    }

    #[test]
    fn macroblock_combine_rejects_wrong_temporal_size() {
        let pic = rplane(2, 2, &[1, 2, 3, 4]);
        let w = single(8);
        let temporal = vec![1u8, 2, 3]; // 3 != 2*2
        assert!(matches!(
            combine_macroblock_spatial_temporal(&w, &pic, 0, 0, 2, 2, &temporal),
            Err(Error::InvalidBitstream(_))
        ));
    }
}
