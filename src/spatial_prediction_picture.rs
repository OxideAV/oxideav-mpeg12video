//! §7.7.3 picture-level spatial-prediction driver.
//!
//! The per-component §7.7.3.1 driver [`crate::upsample_spatial_prediction`]
//! forms `spat_pred_pic` for **one** colour component, given that
//! component's lower-layer plane plus the Table 7-16 local variables for
//! that component. This module composes it across a **whole picture**: it
//!
//! 1. derives the luminance + chrominance [`ResampleParams`] (Table 7-16 /
//!    7-17 / 7-18) from the parsed `sequence_scalable_extension()`
//!    [`SpatialScalabilityParams`] and the
//!    `picture_spatial_scalable_extension()`
//!    [`PictureSpatialScalableExtension`] together with the two layers'
//!    chrominance formats (§7.7.3.3), then
//! 2. selects the Table 7-15 upsampling case
//!    ([`UpsampleCase::select`]) from the three progressiveness flags, and
//! 3. runs [`upsample_spatial_prediction`] over the lower-layer
//!    reconstructed frame's Y / Cb / Cr planes to emit the
//!    enhancement-grid [`SpatialPredictionPicture`] (`spat_pred_pic` for
//!    every component) that the §7.7.4 spatial/temporal combiner
//!    ([`crate::combine_spatial_temporal`]) consumes per macroblock.
//!
//! ## Geometry
//!
//! `lower_layer_prediction_horizontal_size` /
//! `lower_layer_prediction_vertical_size` (§6.3.7) are the lower-layer
//! **luminance** frame dimensions the resampling reads (Table 7-16
//! `ll_h_size` / `ll_v_size`). The §7.7.3.5 / §7.7.3.6 output extent — the
//! upsampled-frame region size on the enhancement grid — is supplied by
//! the caller as `out_width` / `out_height` (the enhancement-layer
//! `horizontal_size` / `vertical_size`); the chrominance output extent is
//! the luma output extent subsampled by the **enhancement** layer's
//! [`crate::chroma_shift`].
//!
//! `lower_layer_horizontal_offset` / `lower_layer_vertical_offset`
//! (`picture_spatial_scalable_extension()`) are 15-bit signed offsets; per
//! §7.7.3 a negative offset crops the upsampled frame above / left of the
//! enhancement frame. The §7.7.3.5 / §7.7.3.6 output index
//! `yh + ll_v_offset` / `xh + ll_h_offset` and the `%` operator are framed
//! for non-negative coordinates, so this driver rejects negative offsets
//! (the caller is expected to resolve any negative-offset cropping before
//! calling — a documented limitation, not a §7.7.3 deviation).

use crate::frame_assembly::{chroma_shift, FrameBuffer};
use crate::picture_spatial_scalable_extension::PictureSpatialScalableExtension;
use crate::sequence_extension::ChromaFormat;
use crate::sequence_scalable_extension::SpatialScalabilityParams;
use crate::spatial_resampling::{
    upsample_spatial_prediction, Plane as ResamplePlane, ResampleParams, UpsampleCase,
};
use crate::{Error, Result};

/// The enhancement-grid spatial prediction picture (`spat_pred_pic` for
/// all three components), the output of the §7.7.3 spatial-prediction
/// process for a whole picture. Each plane holds samples in `[0, 255]`
/// ready for the §7.7.4 combiner.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SpatialPredictionPicture {
    /// `spat_pred_pic` for the luminance component (enhancement-grid
    /// `out_width × out_height`).
    pub y: ResamplePlane,
    /// `spat_pred_pic` for the Cb chrominance component.
    pub cb: ResamplePlane,
    /// `spat_pred_pic` for the Cr chrominance component.
    pub cr: ResamplePlane,
}

/// Convert a reconstructed [`FrameBuffer`] component plane (8-bit
/// samples) into the resampling [`ResamplePlane`] (i32 samples) the
/// §7.7.3 stages operate on.
fn lower_layer_plane(plane: &crate::frame_assembly::Plane) -> Result<ResamplePlane> {
    let samples: Vec<i32> = plane.samples().iter().map(|&s| s as i32).collect();
    ResamplePlane::new(plane.width() as u32, plane.height() as u32, samples)
}

/// Derive the luminance [`ResampleParams`] for the §7.7.3 resampling from
/// the parsed spatial-scalability fields (Table 7-16 *"value for
/// luminance processing"* column).
///
/// `ll_h_offset` / `ll_v_offset` are the
/// `picture_spatial_scalable_extension()`
/// `lower_layer_horizontal_offset` / `lower_layer_vertical_offset`
/// resolved to non-negative enhancement-grid coordinates by the caller
/// (see module docs).
fn derive_luma_params(
    seq: &SpatialScalabilityParams,
    ll_h_offset: u32,
    ll_v_offset: u32,
) -> Result<ResampleParams> {
    ResampleParams::luminance(
        seq.lower_layer_prediction_horizontal_size as u32,
        seq.lower_layer_prediction_vertical_size as u32,
        ll_h_offset,
        ll_v_offset,
        seq.horizontal_subsampling_factor_m as u32,
        seq.horizontal_subsampling_factor_n as u32,
        seq.vertical_subsampling_factor_m as u32,
        seq.vertical_subsampling_factor_n as u32,
    )
}

/// Derive the chrominance [`ResampleParams`] (Table 7-16 *"value for
/// chrominance processing"* column) for the §7.7.3 resampling, applying
/// the Tables 7-17 / 7-18 `chroma_ratio` / `format_ratio` adjustments
/// keyed on the `(lower, enhance)` chrominance-format pair (§7.7.3.3).
fn derive_chroma_params(
    seq: &SpatialScalabilityParams,
    ll_h_offset: u32,
    ll_v_offset: u32,
    lower_format: ChromaFormat,
    enhance_format: ChromaFormat,
) -> Result<ResampleParams> {
    ResampleParams::chrominance(
        seq.lower_layer_prediction_horizontal_size as u32,
        seq.lower_layer_prediction_vertical_size as u32,
        ll_h_offset,
        ll_v_offset,
        seq.horizontal_subsampling_factor_m as u32,
        seq.horizontal_subsampling_factor_n as u32,
        seq.vertical_subsampling_factor_m as u32,
        seq.vertical_subsampling_factor_n as u32,
        lower_format,
        enhance_format,
    )
}

/// Resolve a 15-bit signed lower-layer offset to a non-negative
/// enhancement-grid coordinate, rejecting the negative-offset cropping
/// case this driver does not yet handle (see module docs).
fn non_negative_offset(value: i32, axis: &'static str) -> Result<u32> {
    if value < 0 {
        return Err(Error::InvalidBitstream(match axis {
            "horizontal" => {
                "spatial prediction: negative lower_layer_horizontal_offset \
                 (cropping above/left of the enhancement frame) is not supported"
            }
            _ => {
                "spatial prediction: negative lower_layer_vertical_offset \
                 (cropping above/left of the enhancement frame) is not supported"
            }
        }));
    }
    Ok(value as u32)
}

/// §7.7.3 picture-level spatial-prediction driver: form the
/// enhancement-grid `spat_pred_pic` for **all three** colour components
/// of a whole picture by composing the per-component §7.7.3.1 stages over
/// the lower-layer reconstructed frame.
///
/// * `lower_frame` — the reconstructed lower-layer frame (Y / Cb / Cr
///   planes, §7.7.3.1 `dlower`). Its luminance dimensions are expected to
///   match `lower_layer_prediction_horizontal_size` /
///   `lower_layer_prediction_vertical_size`.
/// * `seq` — the `sequence_scalable_extension()` spatial-scalability
///   parameter block (§6.3.7).
/// * `pss` — the `picture_spatial_scalable_extension()` for this picture
///   (the offsets + the three progressiveness flags, §6.3.10).
/// * `lower_format` / `enhance_format` — the two layers' chrominance
///   formats (§7.7.3.3, Tables 7-17 / 7-18).
/// * `enhance_progressive_frame` — the enhancement-layer `progressive_frame`
///   (`sequence_extension()`), feeding the Table 7-15 case dispatch.
/// * `enhance_width` / `enhance_height` — the enhancement-layer luminance
///   `horizontal_size` / `vertical_size`, the §7.7.3.5 / §7.7.3.6 output
///   extent.
/// * `frame_picture` — `true` when the enhancement picture being predicted
///   is a frame picture (selects the §7.7.3.4 luma deinterlace aperture).
///
/// Returns the [`SpatialPredictionPicture`] (`spat_pred_pic` per
/// component) ready for the §7.7.4 combiner.
///
/// # Errors
/// * [`Error::InvalidBitstream`] for a negative lower-layer offset (see
///   module docs), an invalid `(lower, enhance)` chroma pair
///   (Table 7-18), the Table 7-15 forbidden flag combinations, or a
///   geometry error propagated from a composed §7.7.3 stage.
#[allow(clippy::too_many_arguments)]
pub fn spatial_prediction_picture(
    lower_frame: &FrameBuffer,
    seq: &SpatialScalabilityParams,
    pss: &PictureSpatialScalableExtension,
    lower_format: ChromaFormat,
    enhance_format: ChromaFormat,
    enhance_progressive_frame: bool,
    enhance_width: u32,
    enhance_height: u32,
    frame_picture: bool,
) -> Result<SpatialPredictionPicture> {
    let ll_h_offset = non_negative_offset(pss.lower_layer_horizontal_offset, "horizontal")?;
    let ll_v_offset = non_negative_offset(pss.lower_layer_vertical_offset, "vertical")?;

    // §7.7.3.1 / Table 7-15: the upsampling case is identical for every
    // component (it depends only on the three progressiveness flags).
    let case = UpsampleCase::select(
        pss.lower_layer_deinterlaced_field_select,
        pss.lower_layer_progressive_frame,
        enhance_progressive_frame,
    )?;

    // Luminance: full-resolution enhancement grid.
    let luma_params = derive_luma_params(seq, ll_h_offset, ll_v_offset)?;
    let y = upsample_spatial_prediction(
        case,
        &lower_layer_plane(&lower_frame.y)?,
        &luma_params,
        enhance_width,
        enhance_height,
        frame_picture,
        true,
    )?;

    // Chrominance: the enhancement-grid chroma output extent is the luma
    // extent subsampled by the enhancement layer's chroma_shift. The
    // chroma planes always use the one-field deinterlace aperture
    // (is_luma = false).
    let (cx, cy) = chroma_shift(enhance_format);
    let chroma_w = (enhance_width + ((1 << cx) - 1)) >> cx;
    let chroma_h = (enhance_height + ((1 << cy) - 1)) >> cy;
    let chroma_params =
        derive_chroma_params(seq, ll_h_offset, ll_v_offset, lower_format, enhance_format)?;
    let cb = upsample_spatial_prediction(
        case,
        &lower_layer_plane(&lower_frame.cb)?,
        &chroma_params,
        chroma_w,
        chroma_h,
        frame_picture,
        false,
    )?;
    let cr = upsample_spatial_prediction(
        case,
        &lower_layer_plane(&lower_frame.cr)?,
        &chroma_params,
        chroma_w,
        chroma_h,
        frame_picture,
        false,
    )?;

    Ok(SpatialPredictionPicture { y, cb, cr })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn seq_params() -> SpatialScalabilityParams {
        SpatialScalabilityParams {
            lower_layer_prediction_horizontal_size: 4,
            lower_layer_prediction_vertical_size: 4,
            horizontal_subsampling_factor_m: 1,
            horizontal_subsampling_factor_n: 1,
            vertical_subsampling_factor_m: 1,
            vertical_subsampling_factor_n: 1,
        }
    }

    fn pss(progressive_lower: bool, field_select: bool) -> PictureSpatialScalableExtension {
        PictureSpatialScalableExtension {
            lower_layer_temporal_reference: 0,
            lower_layer_horizontal_offset: 0,
            lower_layer_vertical_offset: 0,
            spatial_temporal_weight_code_table_index: 0,
            lower_layer_progressive_frame: progressive_lower,
            lower_layer_deinterlaced_field_select: field_select,
        }
    }

    /// Fill a lower-layer frame's planes with a ramp so we can verify the
    /// identity (1:1, no subsampling, matching sizes) resample reproduces
    /// the input across all three components.
    fn ramp_frame(format: ChromaFormat) -> FrameBuffer {
        let mut frame = FrameBuffer::new(4, 4, format);
        for y in 0..4 {
            for x in 0..4 {
                frame.y.put_sample(x, y, (y * 4 + x) as u8 + 1);
            }
        }
        let cw = frame.cb.width();
        let ch = frame.cb.height();
        for y in 0..ch {
            for x in 0..cw {
                frame.cb.put_sample(x, y, (y * cw + x) as u8 + 100);
                frame.cr.put_sample(x, y, (y * cw + x) as u8 + 200);
            }
        }
        frame
    }

    #[test]
    fn identity_resample_reproduces_all_three_planes_444() {
        // 4:4:4 → every plane is full-resolution 4×4; subs 1:1; offsets 0;
        // progressive lower + progressive enhancement (row 3). The
        // resample is the identity, so spat_pred_pic == dlower per plane.
        let frame = ramp_frame(ChromaFormat::Yuv444);
        let out = spatial_prediction_picture(
            &frame,
            &seq_params(),
            &pss(true, true),
            ChromaFormat::Yuv444,
            ChromaFormat::Yuv444,
            true,
            4,
            4,
            true,
        )
        .expect("spatial prediction");

        assert_eq!(out.y.width(), 4);
        assert_eq!(out.y.height(), 4);
        let expect_y: Vec<i32> = (0..16).map(|i| i + 1).collect();
        assert_eq!(out.y.samples(), &expect_y[..]);
        let expect_cb: Vec<i32> = (0..16).map(|i| i + 100).collect();
        assert_eq!(out.cb.samples(), &expect_cb[..]);
        let expect_cr: Vec<i32> = (0..16).map(|i| i + 200).collect();
        assert_eq!(out.cr.samples(), &expect_cr[..]);
    }

    #[test]
    fn chroma_output_extent_follows_enhancement_subsampling_420() {
        // 4:2:0 lower + 4:2:0 enhancement: the luma output is 4×4, the
        // chroma output extent is 2×2 (half/half). Identity subs so the
        // chroma planes reproduce the lower-layer 2×2 chroma exactly.
        let frame = ramp_frame(ChromaFormat::Yuv420);
        let out = spatial_prediction_picture(
            &frame,
            &seq_params(),
            &pss(true, true),
            ChromaFormat::Yuv420,
            ChromaFormat::Yuv420,
            true,
            4,
            4,
            true,
        )
        .expect("spatial prediction");

        assert_eq!((out.y.width(), out.y.height()), (4, 4));
        assert_eq!((out.cb.width(), out.cb.height()), (2, 2));
        assert_eq!((out.cr.width(), out.cr.height()), (2, 2));
        let expect_cb: Vec<i32> = (0..4).map(|i| i + 100).collect();
        assert_eq!(out.cb.samples(), &expect_cb[..]);
    }

    #[test]
    fn negative_horizontal_offset_is_rejected() {
        let frame = ramp_frame(ChromaFormat::Yuv444);
        let mut p = pss(true, true);
        p.lower_layer_horizontal_offset = -1;
        let err = spatial_prediction_picture(
            &frame,
            &seq_params(),
            &p,
            ChromaFormat::Yuv444,
            ChromaFormat::Yuv444,
            true,
            4,
            4,
            true,
        )
        .unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn progressive_lower_with_field_select_zero_is_rejected() {
        // Table 7-15: lower_layer_deinterlaced_field_select shall be '1'
        // when lower_layer_progressive_frame is '1'.
        let frame = ramp_frame(ChromaFormat::Yuv444);
        let err = spatial_prediction_picture(
            &frame,
            &seq_params(),
            &pss(true, false),
            ChromaFormat::Yuv444,
            ChromaFormat::Yuv444,
            true,
            4,
            4,
            true,
        )
        .unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }
}
