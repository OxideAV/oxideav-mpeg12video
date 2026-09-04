//! §7.7 **Spatial scalability** — the top-level two-layer loop that
//! decodes a spatial enhancement-layer bitstream against the lower
//! layer's decoded (and §7.7.3 resampled) frames, and the self-made
//! enhancement-layer **encoder** that is this crate's only oracle for
//! the layer (no black-box reference in reach produces or consumes a
//! spatial enhancement layer; the lower layer stays an ordinary
//! ISO/IEC 13818-2 stream any decoder accepts).
//!
//! # The decoding process (§7.7)
//!
//! Every enhancement picture carries a
//! `picture_spatial_scalable_extension()` naming the lower-layer frame
//! (`lower_layer_temporal_reference`, §7.7.3.1) whose reconstruction
//! is resampled onto the enhancement grid by the §7.7.3 process
//! ([`crate::spatial_prediction_picture`]: Table 7-16 parameters from
//! the `sequence_scalable_extension()`, Table 7-15 case selection,
//! the §7.7.3.4 – §7.7.3.7 resampling chain) into `spat_pred_pic`.
//! Macroblocks use Tables B-5 / B-6 / B-7 and the
//! `spatial_temporal_weight_class` (§7.7.4): class 0 is ordinary
//! temporal (or intra) coding, class 4 is spatial-only prediction —
//! `p[y][x] = pel_pred_spat[y][x]`, no motion vectors, predictors
//! reset (§7.7.5.1) — and class 1 (the only class of weight table
//! `00`, Table 7-21: no `spatial_temporal_weight_code` on the wire)
//! averages the two: `p = (pel_pred_temp + pel_pred_spat) // 2`.
//! Skipped macroblocks are spatial-only in I pictures and
//! temporal-only in P / B pictures (§7.7.6); the §7.7.5 predictor
//! rules for frame-based prediction coincide with Table 7-10 plus the
//! class-4 reset. The residual is added per §7.6.8.
//!
//! **Scope**: both layers progressive frame pictures with
//! `spatial_temporal_weight_code_table_index = 00` (Table 7-20's
//! intended index for a progressive enhancement layer) — the
//! interlaced weight tables `01` / `10` / `11` (per-field weights,
//! field prediction under class 2 / 3, Tables 7-22 / 7-24 / 7-25) and
//! field pictures are not composed; a picture without a
//! `picture_spatial_scalable_extension()` is decoded non-scalably
//! (§6.3.7). Lower-layer offsets are non-negative (the §7.7.3 driver's
//! documented limitation).
//!
//! # The encoder
//!
//! [`encode_spatial_enhancement_layer`] mirrors the lower layer's GOP
//! structure picture for picture (same coded order, picture types and
//! temporal references — the coincident lower frame is the spatial
//! reference) and chooses per macroblock among intra / temporal /
//! spatial-only / half-weight prediction by luminance SAD, coding the
//! residual as an ordinary macroblock. Its enhancement references are
//! the decoder's exact reconstructions, so
//! [`decode_spatial_scalable_sequence`] reproduces its frames sample
//! for sample.

// The block arithmetic indexes 8×8 / 16×16 arrays by the same loop
// variables the spec's formulae use; the macroblock_type literals are
// grouped the way Tables B-5 / B-6 / B-7 print their codewords.
#![allow(clippy::needless_range_loop, clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitWriter;

use crate::coded_block_pattern::encode_coded_block_pattern;
use crate::frame_assembly::{
    block_placement, place_intra_macroblock, FrameBuffer, IntraPictureParams,
};
use crate::inter_reconstruction::{
    chroma_mb_extent, predict_frame_macroblock_planes, FrameMotion, MotionVectorPel,
    ReferenceFrames,
};
use crate::macroblock_modes::PredictionType;
use crate::macroblock_type::MacroblockTypeTable;
use crate::motion_estimation::{estimate_forward_mv, max_search_range};
use crate::motion_vector::encode_motion_component;
use crate::mpeg2_block_dc::ColourComponent;
use crate::mpeg2_dequantize::{intra_dc_mult, quantiser_scale, DEFAULT_NON_INTRA_WEIGHT};
use crate::mpeg2_macroblock_blocks::{block_component, block_count};
use crate::p_picture_encoder::{
    encode_intra_mb, gather_residual, intra_activity, nonintra_block_has_cbp_slot,
    quantise_inter_block, reconstruct_inter_mb, wrap_delta, write_inter_block_coeffs, InterBlock,
    IntraDcPred,
};
use crate::picture_header::{
    Mpeg2PictureHeader, PictureCodingExtension, PictureCodingType, PictureStructure,
};
use crate::picture_reconstruction::{
    frame_motion_from_reconstructed, reconstruct_one_macroblock, reconstruct_skipped_macroblock,
    InterDirection, PicturePredictionParams,
};
use crate::picture_spatial_scalable_extension::{
    write_picture_spatial_scalable_extension, PictureSpatialScalableExtension,
    PICTURE_SPATIAL_SCALABLE_EXTENSION_ID,
};
use crate::quant_matrix_extension::QuantiserMatrixState;
use crate::sequence_extension::{ChromaFormat, Mpeg2Sequence, Mpeg2SequenceExtension};
use crate::sequence_scalable_extension::{
    write_sequence_scalable_extension, ScalableMode, SequenceScalableExtension,
    SpatialScalabilityParams, SEQUENCE_SCALABLE_EXTENSION_ID,
};
use crate::slice_header::{SliceContext, SliceHeader};
use crate::slice_macroblock_walk::{
    reconstruct_slice_motion_vectors, walk_slice_at, MacroblockRecord, SliceWalkContext,
};
use crate::spatial_prediction_picture::{spatial_prediction_picture, SpatialPredictionPicture};
use crate::spatial_temporal_combine::{extract_colocated_spatial, SpatialWeight};
use crate::stream_writer::{
    write_picture_coding_extension, write_picture_header, write_sequence_header,
    write_slice_header_in, PictureCodingExtensionParams, SequenceHeaderParams, SEQUENCE_END_CODE,
};
use crate::video_sequence::{
    apply_quant_matrix_extensions, decode_video_sequence, display_indices_from_coded_pictures,
    DecodedFrame,
};
use crate::{Error, Result};

// -------------------------------------------------------------------
// Stream scanning
// -------------------------------------------------------------------

fn scan_start_codes(buf: &[u8]) -> Vec<(usize, u8)> {
    let mut out = Vec::new();
    let mut i = 0usize;
    while i + 3 < buf.len() {
        if buf[i] == 0 && buf[i + 1] == 0 && buf[i + 2] == 1 {
            out.push((i, buf[i + 3]));
            i += 4;
        } else {
            i += 1;
        }
    }
    out
}

fn is_slice_code(code: u8) -> bool {
    (0x01..=0xAF).contains(&code)
}

/// Pictures (`start`, `end_with_terminator`) and GOP headers
/// (`picture_index_before`, `start`, `end`) of an elementary stream.
fn layout(stream: &[u8]) -> (Vec<(usize, usize)>, Vec<(usize, usize, usize)>) {
    let codes = scan_start_codes(stream);
    let mut pictures = Vec::new();
    let mut gops = Vec::new();
    for (k, &(off, code)) in codes.iter().enumerate() {
        match code {
            0x00 => {
                let boundary = codes[k + 1..]
                    .iter()
                    .find(|&&(_, c)| matches!(c, 0x00 | 0xB8 | 0xB3 | 0xB7))
                    .map(|&(o, _)| (o + 4).min(stream.len()))
                    .unwrap_or(stream.len());
                pictures.push((off, boundary));
            }
            0xB8 => {
                let end = codes.get(k + 1).map(|&(o, _)| o).unwrap_or(stream.len());
                gops.push((pictures.len(), off, end));
            }
            _ => {}
        }
    }
    (pictures, gops)
}

fn geometry_of(seq: &Mpeg2Sequence) -> IntraPictureParams {
    IntraPictureParams {
        width: seq.horizontal_size as usize,
        height: seq.vertical_size as usize,
        chroma_format: seq.extension.chroma_format,
        frame_pred_frame_dct: true,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        progressive_sequence: seq.extension.progressive_sequence,
    }
}

fn initial_matrices(seq: &Mpeg2Sequence) -> QuantiserMatrixState {
    let mut matrices = QuantiserMatrixState::default();
    let loads = crate::quant_matrix_extension::QuantMatrixExtension {
        intra: seq
            .header
            .intra_quant
            .map(|zz| crate::quant_matrix_extension::QuantiserMatrixPayload { bytes: zz }),
        non_intra: seq
            .header
            .non_intra_quant
            .map(|zz| crate::quant_matrix_extension::QuantiserMatrixPayload { bytes: zz }),
        chroma_intra: None,
        chroma_non_intra: None,
    };
    loads.apply(&mut matrices, seq.extension.chroma_format);
    matrices
}

fn spatial_params(stream: &[u8]) -> Result<SpatialScalabilityParams> {
    for &(off, code) in &scan_start_codes(stream) {
        if code == 0x00 || code == 0xB8 {
            break;
        }
        if code == 0xB5
            && stream.get(off + 4).map(|b| b >> 4) == Some(SEQUENCE_SCALABLE_EXTENSION_ID as u8)
        {
            let sse = SequenceScalableExtension::parse(&stream[off..])?;
            return match sse.scalable_mode {
                ScalableMode::SpatialScalability(p) if sse.layer_id == 1 => Ok(p),
                _ => Err(Error::InvalidBitstream(
                    "spatial scalability: enhancement layer shall declare scalable_mode = spatial scalability, layer_id = 1 (§6.3.7)",
                )),
            };
        }
    }
    Err(Error::InvalidBitstream(
        "spatial scalability: enhancement layer carries no sequence_scalable_extension()",
    ))
}

fn spatial_extension(region: &[u8]) -> Result<Option<PictureSpatialScalableExtension>> {
    for &(off, code) in &scan_start_codes(region) {
        if is_slice_code(code) {
            break;
        }
        if code == 0xB5
            && region.get(off + 4).map(|b| b >> 4)
                == Some(PICTURE_SPATIAL_SCALABLE_EXTENSION_ID as u8)
        {
            return PictureSpatialScalableExtension::parse(&region[off..]).map(Some);
        }
    }
    Ok(None)
}

// -------------------------------------------------------------------
// Block helpers
// -------------------------------------------------------------------

/// The colocated `pel_pred_spat` planes of a macroblock (luma 16×16,
/// chroma per [`chroma_mb_extent`]).
fn spatial_macroblock_planes(
    spat: &SpatialPredictionPicture,
    chroma_format: ChromaFormat,
    mb_col: usize,
    mb_row: usize,
) -> (Vec<u8>, Vec<u8>, Vec<u8>) {
    let (cw, ch) = chroma_mb_extent(chroma_format);
    let luma = extract_colocated_spatial(&spat.y, mb_col * 16, mb_row * 16, 16, 16);
    let cb = extract_colocated_spatial(&spat.cb, mb_col * cw, mb_row * ch, cw, ch);
    let cr = extract_colocated_spatial(&spat.cr, mb_col * cw, mb_row * ch, cw, ch);
    (luma, cb, cr)
}

/// `(temporal + spatial) // 2` per sample (§7.7.4, class 1).
fn half_planes(temporal: &[u8], spatial: &[u8]) -> Vec<u8> {
    temporal
        .iter()
        .zip(spatial)
        .map(|(&t, &s)| SpatialWeight::Half.combine_sample(t, s))
        .collect()
}

/// `clamp(p + f_pel)` for the prediction planes of one macroblock and
/// the coded residual blocks of `record` (frame / field DCT per
/// `dct_type`), written into `frame`.
fn add_residual_to_prediction(
    frame: &mut FrameBuffer,
    record: &MacroblockRecord,
    mb_col: usize,
    mb_row: usize,
    luma_pred: &[u8],
    cb_pred: &[u8],
    cr_pred: &[u8],
) -> Result<()> {
    let chroma_format = frame.chroma_format;
    let nblocks = block_count(chroma_format);
    let (cmb_w, cmb_h) = chroma_mb_extent(chroma_format);
    let field_dct = record.dct_type == Some(true);
    for i in 0..nblocks {
        let placement = block_placement(i, chroma_format, mb_col, mb_row, field_dct)
            .ok_or(Error::InvalidBitstream("spatial: bad block placement"))?;
        let component = block_component(i, chroma_format)
            .ok_or(Error::InvalidBitstream("spatial: bad block index"))?;
        let (pred, pred_w, origin_x, origin_y) = match component {
            ColourComponent::Y => (luma_pred, 16usize, mb_col * 16, mb_row * 16),
            ColourComponent::Cb => (cb_pred, cmb_w, mb_col * cmb_w, mb_row * cmb_h),
            ColourComponent::Cr => (cr_pred, cmb_w, mb_col * cmb_w, mb_row * cmb_h),
        };
        let f_pel = record
            .decoded_blocks
            .as_ref()
            .and_then(|blocks| blocks.iter().find(|b| usize::from(b.block_index) == i))
            .map(|b| b.decoded.f_pel);
        let stride = placement.row_stride();
        let local_x0 = placement.base_x - origin_x;
        let local_y0 = placement.base_y - origin_y;
        let plane = match component {
            ColourComponent::Y => &mut frame.y,
            ColourComponent::Cb => &mut frame.cb,
            ColourComponent::Cr => &mut frame.cr,
        };
        for r in 0..8 {
            let py = local_y0 + r * stride;
            for c in 0..8 {
                let p = i32::from(pred[py * pred_w + local_x0 + c]);
                let d = f_pel.map(|f| i32::from(f[r][c])).unwrap_or(0);
                plane.put_sample(
                    placement.base_x + c,
                    placement.base_y + r * stride,
                    (p + d).clamp(0, 255) as u8,
                );
            }
        }
    }
    Ok(())
}

fn effective_class(record: &MacroblockRecord) -> u8 {
    record
        .spatial_temporal_weight
        .map(|w| w.weight_class)
        .or(record.macroblock_type.spatial_temporal_weight_class)
        .unwrap_or(0)
}

// -------------------------------------------------------------------
// Decoding
// -------------------------------------------------------------------

/// The result of [`decode_spatial_scalable_sequence`].
#[derive(Debug, Clone)]
pub struct SpatialScalableDecoded {
    /// The lower layer's frames in display order.
    pub lower: Vec<DecodedFrame>,
    /// The enhancement layer's frames in display order.
    pub enhancement: Vec<DecodedFrame>,
}

/// Decode a §7.7 spatial-scalable pair — the lower-layer stream `base`
/// (an ordinary ISO/IEC 13818-2 elementary stream) and its spatial
/// `enhancement` layer — into both layers' frames.
///
/// # Errors
/// [`Error::InvalidBitstream`] when the lower layer is not an ISO/IEC
/// 13818-2 stream, when the enhancement layer lacks a spatial
/// `sequence_scalable_extension()` or its `lower_layer_prediction_*`
/// sizes disagree with the lower layer (§6.3.7), for the interlaced
/// weight tables / field pictures this loop does not compose, when
/// a `lower_layer_temporal_reference` disagrees with the coincident
/// lower frame, or for any syntax error; [`Error::ShortHeader`] on
/// truncation.
pub fn decode_spatial_scalable_sequence(
    base: &[u8],
    enhancement: &[u8],
) -> Result<SpatialScalableDecoded> {
    let lower_seq = Mpeg2Sequence::from_buf(base).map_err(|_| {
        Error::InvalidBitstream(
            "spatial scalability: the lower layer shall conform to ISO/IEC 13818-2 (an ISO/IEC 11172-2 lower layer is not composed)",
        )
    })?;
    let lower = decode_video_sequence(base)?;

    let seq = Mpeg2Sequence::from_buf(enhancement).map_err(|_| {
        Error::InvalidBitstream(
            "spatial scalability: enhancement layer sequence_header / sequence_extension missing or malformed",
        )
    })?;
    let sp = spatial_params(enhancement)?;
    if sp.lower_layer_prediction_horizontal_size != lower_seq.horizontal_size
        || sp.lower_layer_prediction_vertical_size != lower_seq.vertical_size
    {
        return Err(Error::InvalidBitstream(
            "spatial scalability: lower_layer_prediction_horizontal/vertical_size shall equal the lower layer's horizontal/vertical_size (§6.3.7)",
        ));
    }
    if lower_seq.extension.chroma_format as u8 > seq.extension.chroma_format as u8 {
        return Err(Error::InvalidBitstream(
            "spatial scalability: the enhancement chroma_format shall not be lower than the lower layer's (Table 7-18)",
        ));
    }
    if !seq.extension.progressive_sequence {
        return Err(Error::InvalidBitstream(
            "spatial scalability: interlaced enhancement layers (weight tables 01 / 10 / 11, field pictures) are not composed",
        ));
    }
    let base_geometry = geometry_of(&seq);
    let mut matrices = initial_matrices(&seq);

    let (pictures, _) = layout(enhancement);
    let mut coded: Vec<(u16, PictureCodingType)> = Vec::with_capacity(pictures.len());
    for &(start, end) in &pictures {
        let header = Mpeg2PictureHeader::parse(&enhancement[start..end])?;
        coded.push((header.temporal_reference, header.picture_coding_type));
    }
    let display_index = display_indices_from_coded_pictures(&coded);

    let mut output: Vec<DecodedFrame> = Vec::new();
    let mut held_anchor: Option<DecodedFrame> = None;
    let mut forward_anchor: Option<FrameBuffer> = None;
    let mut backward_anchor: Option<FrameBuffer> = None;

    for (index, &(start, end)) in pictures.iter().enumerate() {
        let region = &enhancement[start..end];
        let (header, ext) = Mpeg2PictureHeader::parse_with_extension(region)?;
        if ext.picture_structure != PictureStructure::Frame {
            return Err(Error::InvalidBitstream(
                "spatial scalability: field pictures are not composed by this loop (frame pictures only)",
            ));
        }
        apply_quant_matrix_extensions(region, base_geometry.chroma_format, &mut matrices)?;
        let geometry = IntraPictureParams {
            frame_pred_frame_dct: ext.frame_pred_frame_dct,
            intra_dc_precision: ext.intra_dc_precision,
            intra_vlc_format: ext.intra_vlc_format,
            alternate_scan: ext.alternate_scan,
            q_scale_type: ext.q_scale_type,
            ..base_geometry
        };
        let references =
            match header.picture_coding_type {
                PictureCodingType::Intra => ReferenceFrames {
                    forward: None,
                    backward: None,
                },
                PictureCodingType::Predictive => ReferenceFrames::forward_only(
                    backward_anchor.as_ref().ok_or(Error::InvalidBitstream(
                        "§6.1.1.11: P-picture before any I/P anchor exists",
                    ))?,
                ),
                PictureCodingType::Bidirectional => ReferenceFrames::bidirectional(
                    forward_anchor.as_ref().ok_or(Error::InvalidBitstream(
                        "§6.1.1.11: B-picture before two I/P anchors exist",
                    ))?,
                    backward_anchor.as_ref().ok_or(Error::InvalidBitstream(
                        "§6.1.1.11: B-picture before two I/P anchors exist",
                    ))?,
                ),
                PictureCodingType::DcIntra => return Err(Error::InvalidBitstream(
                    "picture_coding_type: 100 (D-picture) shall not be used in MPEG-2 (Table 6-12)",
                )),
            };

        // §6.3.7: a picture without picture_spatial_scalable_extension()
        // is decoded non-scalably.
        let spat = match spatial_extension(region)? {
            None => None,
            Some(pss) => {
                if pss.spatial_temporal_weight_code_table_index != 0 {
                    return Err(Error::InvalidBitstream(
                        "spatial scalability: spatial_temporal_weight_code_table_index 01 / 10 / 11 (interlaced weight tables) is not composed",
                    ));
                }
                let d = display_index[index] as usize;
                let lower_frame = lower.get(d).ok_or(Error::InvalidBitstream(
                    "spatial scalability: no coincident lower-layer frame for the enhancement picture",
                ))?;
                if lower_frame.temporal_reference & 0x3FF != pss.lower_layer_temporal_reference {
                    return Err(Error::InvalidBitstream(
                        "spatial scalability: lower_layer_temporal_reference disagrees with the coincident lower-layer frame (§7.7.3.1)",
                    ));
                }
                Some((
                    pss,
                    spatial_prediction_picture(
                        &lower_frame.frame,
                        &sp,
                        &pss,
                        lower_seq.extension.chroma_format,
                        seq.extension.chroma_format,
                        ext.progressive_frame,
                        geometry.width as u32,
                        geometry.height as u32,
                        true,
                    )?,
                ))
            }
        };

        let frame = reconstruct_spatial_picture(
            region,
            &header,
            &ext,
            geometry,
            references,
            &matrices,
            spat.as_ref().map(|(p, s)| (p, s)),
        )?;

        let decoded = DecodedFrame {
            frame,
            temporal_reference: header.temporal_reference,
            picture_coding_type: header.picture_coding_type,
            top_field_first: ext.top_field_first,
            repeat_first_field: ext.repeat_first_field,
            progressive_frame: ext.progressive_frame,
        };
        match header.picture_coding_type {
            PictureCodingType::Bidirectional => output.push(decoded),
            _ => {
                if let Some(prev) = held_anchor.take() {
                    output.push(prev);
                }
                forward_anchor = backward_anchor.take();
                backward_anchor = Some(decoded.frame.clone());
                held_anchor = Some(decoded);
            }
        }
    }
    if let Some(prev) = held_anchor.take() {
        output.push(prev);
    }
    Ok(SpatialScalableDecoded {
        lower,
        enhancement: output,
    })
}

/// Reconstruct one enhancement frame picture, spatially scalable when
/// `spat` is present.
#[allow(clippy::too_many_arguments)]
fn reconstruct_spatial_picture(
    region: &[u8],
    header: &Mpeg2PictureHeader,
    ext: &PictureCodingExtension,
    geometry: IntraPictureParams,
    references: ReferenceFrames<'_>,
    matrices: &QuantiserMatrixState,
    spat: Option<(&PictureSpatialScalableExtension, &SpatialPredictionPicture)>,
) -> Result<FrameBuffer> {
    let mut frame = geometry.new_frame_buffer();
    let mb_width = geometry.mb_width() as u32;
    let mb_height = geometry.mb_height();
    let chroma_format = geometry.chroma_format;
    let slice_ctx = SliceContext::non_scalable(geometry.height as u32);
    let intra_picture = header.picture_coding_type == PictureCodingType::Intra;
    let params = PicturePredictionParams {
        geometry,
        picture_coding_type: header.picture_coding_type,
        f_code_fwd_horiz: ext.f_code_fwd_horiz,
        f_code_fwd_vert: ext.f_code_fwd_vert,
        f_code_bwd_horiz: ext.f_code_bwd_horiz,
        f_code_bwd_vert: ext.f_code_bwd_vert,
        concealment_motion_vectors: ext.concealment_motion_vectors,
        top_field_first: ext.top_field_first,
    };
    let (table, index) = match spat {
        Some((pss, _)) => (
            MacroblockTypeTable::SpatialScalable,
            pss.spatial_temporal_weight_code_table_index,
        ),
        None => (MacroblockTypeTable::NonScalable, 0),
    };

    let codes = scan_start_codes(region);
    let mut placed = 0usize;
    for (k, &(off, code)) in codes.iter().enumerate() {
        if !is_slice_code(code) {
            continue;
        }
        let end = codes
            .get(k + 1)
            .map(|&(o, _)| (o + 4).min(region.len()))
            .unwrap_or(region.len());
        let slice_buf = &region[off..end];
        let sh = SliceHeader::parse(slice_buf, slice_ctx)?;
        let mb_row = sh.mb_row();
        let (f_bwd_h, f_bwd_v) = if intra_picture {
            (15, 15)
        } else {
            (ext.f_code_bwd_horiz, ext.f_code_bwd_vert)
        };
        let ctx = SliceWalkContext::first_slice_with_block_decoding(
            mb_width,
            mb_row,
            header.picture_coding_type,
            sh.quantiser_scale_code,
            PictureStructure::Frame,
            geometry.frame_pred_frame_dct,
            ext.f_code_fwd_horiz,
            ext.f_code_fwd_vert,
            f_bwd_h,
            f_bwd_v,
            ext.concealment_motion_vectors,
            chroma_format,
            geometry.intra_vlc_format,
            geometry.alternate_scan,
            geometry.intra_dc_precision,
            geometry.q_scale_type,
        )
        .with_quantiser_matrices(*matrices)
        .with_scalable_tables(table, index);
        let walk = walk_slice_at(slice_buf, sh.body_bit_position, ctx)?;
        // Motion reconstruction for P / B pictures (spatial-only and
        // intra macroblocks reset the predictors through the same
        // Table 7-10 / 7-23 rows).
        let motion = if intra_picture {
            None
        } else {
            Some(reconstruct_slice_motion_vectors(&walk, &ctx)?)
        };

        let mut previous_inter_direction: Option<InterDirection> = None;
        for (r, record) in walk.macroblocks.iter().enumerate() {
            let address = record.macroblock_address;
            let mb_col = address as usize % mb_width as usize;
            let mb_row_of = address as usize / mb_width as usize;
            let motion_record = motion.as_ref().map(|m| &m.records[r]);

            // §7.7.6 skipped macroblocks: spatial-only in I pictures,
            // temporal-only in P / B pictures.
            let skipped = record.skipped_macroblock_count;
            for k in 0..skipped {
                let s_address = address - skipped + k;
                if intra_picture {
                    let (_, s) = spat.ok_or(Error::InvalidBitstream(
                        "§6.3.17.1: skipped macroblocks in an I picture need spatial scalability (§7.7.6)",
                    ))?;
                    let (l, cb, cr) = spatial_macroblock_planes(
                        s,
                        chroma_format,
                        s_address as usize % mb_width as usize,
                        s_address as usize / mb_width as usize,
                    );
                    let mut empty = record.clone();
                    empty.decoded_blocks = None;
                    empty.dct_type = None;
                    add_residual_to_prediction(
                        &mut frame,
                        &empty,
                        s_address as usize % mb_width as usize,
                        s_address as usize / mb_width as usize,
                        &l,
                        &cb,
                        &cr,
                    )?;
                    placed += 1;
                } else {
                    let mr = motion_record.ok_or(Error::InvalidBitstream(
                        "spatial: skipped macroblock without motion state",
                    ))?;
                    placed += reconstruct_skipped_macroblock(
                        &mut frame,
                        references,
                        s_address as usize,
                        mb_width as usize,
                        header.picture_coding_type,
                        previous_inter_direction,
                        &mr.pmv_before,
                    )?;
                }
            }

            let class = effective_class(record);
            match (class, spat) {
                (0, _) | (_, None) => {
                    if record.macroblock_type.macroblock_intra {
                        place_intra_macroblock(
                            &mut frame,
                            record,
                            mb_width as usize,
                            chroma_format,
                        );
                        placed += 1;
                    } else {
                        let mr = motion_record.ok_or(Error::InvalidBitstream(
                            "spatial: non-intra class-0 macroblock in an I picture",
                        ))?;
                        placed += reconstruct_one_macroblock(
                            &mut frame,
                            references,
                            record,
                            &mr.reconstructed,
                            mb_width as usize,
                            chroma_format,
                            params.top_field_first,
                            &mut previous_inter_direction,
                        )?;
                    }
                }
                (4, Some((_, s))) => {
                    // Spatial-only: p = pel_pred_spat.
                    let (l, cb, cr) =
                        spatial_macroblock_planes(s, chroma_format, mb_col, mb_row_of);
                    add_residual_to_prediction(
                        &mut frame, record, mb_col, mb_row_of, &l, &cb, &cr,
                    )?;
                    placed += 1;
                }
                (1, Some((_, s))) => {
                    // Half weight: p = (pel_pred_temp + pel_pred_spat) // 2.
                    let mr = motion_record.ok_or(Error::InvalidBitstream(
                        "spatial: class-1 macroblock in an I picture",
                    ))?;
                    let prediction_type = record
                        .motion_type
                        .map(|mt| mt.prediction_type)
                        .unwrap_or(PredictionType::FrameBased);
                    if prediction_type != PredictionType::FrameBased {
                        return Err(Error::InvalidBitstream(
                            "spatial scalability: field-based / dual-prime temporal prediction under a spatial weight is not composed (frame-based only)",
                        ));
                    }
                    let motion = frame_motion_from_reconstructed(&mr.reconstructed);
                    let (tl, tcb, tcr) = predict_frame_macroblock_planes(
                        &frame, references, mb_col, mb_row_of, motion,
                    )
                    .map_err(Error::from)?;
                    let (sl, scb, scr) =
                        spatial_macroblock_planes(s, chroma_format, mb_col, mb_row_of);
                    add_residual_to_prediction(
                        &mut frame,
                        record,
                        mb_col,
                        mb_row_of,
                        &half_planes(&tl, &sl),
                        &half_planes(&tcb, &scb),
                        &half_planes(&tcr, &scr),
                    )?;
                    previous_inter_direction = Some(InterDirection {
                        forward: record.macroblock_type.macroblock_motion_forward,
                        backward: record.macroblock_type.macroblock_motion_backward,
                    });
                    placed += 1;
                }
                (other, _) => {
                    return Err(Error::InvalidBitstream(if other == 2 || other == 3 {
                        "spatial scalability: spatial_temporal_weight_class 2 / 3 (interlaced weight tables) is not composed"
                    } else {
                        "spatial scalability: unknown spatial_temporal_weight_class"
                    }))
                }
            }
        }
    }
    if placed != mb_width as usize * mb_height {
        return Err(Error::InvalidBitstream(
            "§6.1.2.2: the picture's slices do not enclose every macroblock exactly once (restricted slice structure, Table 8-5)",
        ));
    }
    Ok(frame)
}

// -------------------------------------------------------------------
// Encoding
// -------------------------------------------------------------------

/// Configuration of [`encode_spatial_enhancement_layer`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpatialLayerConfig {
    /// The per-slice `quantiser_scale_code` (`1..=31`).
    pub quantiser_scale_code: u8,
    /// Motion-vector `f_code` for both directions (`1..=9`).
    pub f_code: u8,
}

impl Default for SpatialLayerConfig {
    fn default() -> Self {
        Self {
            quantiser_scale_code: 6,
            f_code: 3,
        }
    }
}

/// Per-macroblock decision counts of [`encode_spatial_enhancement_layer`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SpatialStats {
    /// Intra macroblocks (class 0 in I pictures, intra fallbacks).
    pub intra: usize,
    /// Temporal-only macroblocks (class 0).
    pub temporal: usize,
    /// Spatial-only macroblocks (class 4).
    pub spatial_only: usize,
    /// Half-weight macroblocks (class 1).
    pub half_weight: usize,
}

/// The output of [`encode_spatial_enhancement_layer`].
#[derive(Debug, Clone)]
pub struct SpatialEncoded {
    /// The enhancement-layer elementary stream.
    pub stream: Vec<u8>,
    /// The lower layer's decoded frames (display order).
    pub lower: Vec<DecodedFrame>,
    /// The enhancement frames' reconstruction in display order — what
    /// [`decode_spatial_scalable_sequence`] reproduces.
    pub enhancement: Vec<DecodedFrame>,
    /// Macroblock decision counts.
    pub stats: SpatialStats,
}

fn write_sequence_extension_fields(bw: &mut BitWriter, e: &Mpeg2SequenceExtension) {
    use crate::sequence_extension::{EXTENSION_START_CODE, SEQUENCE_EXTENSION_ID};
    bw.write_u32(EXTENSION_START_CODE, 32);
    bw.write_u32(SEQUENCE_EXTENSION_ID, 4);
    bw.write_u32(u32::from(e.profile_and_level), 8);
    bw.write_bit(e.progressive_sequence);
    bw.write_u32(
        match e.chroma_format {
            ChromaFormat::Yuv420 => 0b01,
            ChromaFormat::Yuv422 => 0b10,
            ChromaFormat::Yuv444 => 0b11,
        },
        2,
    );
    bw.write_u32(u32::from(e.horizontal_size_extension), 2);
    bw.write_u32(u32::from(e.vertical_size_extension), 2);
    bw.write_u32(u32::from(e.bit_rate_extension), 12);
    bw.write_bit(true); // marker_bit
    bw.write_u32(u32::from(e.vbv_buffer_size_extension), 8);
    bw.write_bit(e.low_delay);
    bw.write_u32(u32::from(e.frame_rate_extension_n), 2);
    bw.write_u32(u32::from(e.frame_rate_extension_d), 5);
    bw.align_to_byte();
}

fn aspect_ratio_code(a: crate::sequence_header::AspectRatio) -> u8 {
    use crate::sequence_header::AspectRatio;
    match a {
        AspectRatio::Square => 0b0001,
        AspectRatio::Dar3x4 => 0b0010,
        AspectRatio::Dar9x16 => 0b0011,
        AspectRatio::Dar1x221 => 0b0100,
        AspectRatio::Reserved(v) => v,
    }
}

fn gcd(a: u32, b: u32) -> u32 {
    if b == 0 {
        a
    } else {
        gcd(b, a % b)
    }
}

/// The macroblock kinds the encoder emits (a subset of the Table B-5 /
/// B-6 / B-7 rows).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    Intra,
    /// Class 0, no motion vector (P "No MC"), never emitted — kept for
    /// table completeness.
    Temporal {
        forward: bool,
        backward: bool,
    },
    /// Class 1: forward (P) / forward-only (B) temporal + spatial average.
    HalfWeight {
        forward: bool,
        backward: bool,
    },
    SpatialOnly,
}

/// Write the Table B-5 / B-6 / B-7 `macroblock_type` codeword.
fn write_macroblock_type(
    bw: &mut BitWriter,
    picture: PictureCodingType,
    kind: Kind,
    coded: bool,
) -> Result<()> {
    let (code, bits): (u32, u32) = match (picture, kind, coded) {
        // Table B-5.
        (PictureCodingType::Intra, Kind::Intra, _) => (0b0011, 4),
        (PictureCodingType::Intra, Kind::SpatialOnly, true) => (0b1, 1),
        (PictureCodingType::Intra, Kind::SpatialOnly, false) => (0b0001, 4),
        // Table B-6.
        (PictureCodingType::Predictive, Kind::Intra, _) => (0b0000_111, 7),
        (PictureCodingType::Predictive, Kind::Temporal { forward: true, .. }, true) => (0b10, 2),
        (PictureCodingType::Predictive, Kind::Temporal { forward: true, .. }, false) => (0b0010, 4),
        (PictureCodingType::Predictive, Kind::Temporal { forward: false, .. }, true) => {
            (0b0000_100, 7)
        }
        (PictureCodingType::Predictive, Kind::HalfWeight { forward: true, .. }, true) => (0b011, 3),
        (PictureCodingType::Predictive, Kind::HalfWeight { forward: true, .. }, false) => {
            (0b0011, 4)
        }
        (PictureCodingType::Predictive, Kind::HalfWeight { forward: false, .. }, true) => {
            (0b0001_11, 6)
        }
        (PictureCodingType::Predictive, Kind::HalfWeight { forward: false, .. }, false) => {
            (0b0001_10, 6)
        }
        (PictureCodingType::Predictive, Kind::SpatialOnly, true) => (0b0000_101, 7),
        (PictureCodingType::Predictive, Kind::SpatialOnly, false) => (0b0000_011, 7),
        // Table B-7.
        (PictureCodingType::Bidirectional, Kind::Intra, _) => (0b0000_110, 7),
        (
            PictureCodingType::Bidirectional,
            Kind::Temporal {
                forward: true,
                backward: true,
            },
            c,
        ) => {
            if c {
                (0b11, 2)
            } else {
                (0b10, 2)
            }
        }
        (
            PictureCodingType::Bidirectional,
            Kind::Temporal {
                forward: false,
                backward: true,
            },
            c,
        ) => {
            if c {
                (0b011, 3)
            } else {
                (0b010, 3)
            }
        }
        (
            PictureCodingType::Bidirectional,
            Kind::Temporal {
                forward: true,
                backward: false,
            },
            c,
        ) => {
            if c {
                (0b0011, 4)
            } else {
                (0b0010, 4)
            }
        }
        (
            PictureCodingType::Bidirectional,
            Kind::HalfWeight {
                forward: true,
                backward: false,
            },
            c,
        ) => {
            if c {
                (0b0001_01, 6)
            } else {
                (0b0001_00, 6)
            }
        }
        (
            PictureCodingType::Bidirectional,
            Kind::HalfWeight {
                forward: false,
                backward: true,
            },
            c,
        ) => {
            if c {
                (0b0001_11, 6)
            } else {
                (0b0001_10, 6)
            }
        }
        (PictureCodingType::Bidirectional, Kind::SpatialOnly, true) => (0b0000_0111_1, 9),
        (PictureCodingType::Bidirectional, Kind::SpatialOnly, false) => (0b0000_0111_0, 9),
        _ => {
            return Err(Error::InvalidBitstream(
                "spatial encoder: macroblock kind has no Table B-5 / B-6 / B-7 row",
            ))
        }
    };
    bw.write_u32(code, bits);
    Ok(())
}

fn sad(current: &[u8], pred: &[u8]) -> u32 {
    current
        .iter()
        .zip(pred)
        .map(|(&a, &b)| u32::from(a.abs_diff(b)))
        .sum()
}

/// The current macroblock's luma as a 16×16 block (edge clamped).
fn current_luma(frame: &FrameBuffer, mb_col: usize, mb_row: usize) -> Vec<u8> {
    let w = frame.y.width();
    let h = frame.y.height();
    let mut out = Vec::with_capacity(256);
    for r in 0..16 {
        let sy = (mb_row * 16 + r).min(h.saturating_sub(1));
        for c in 0..16 {
            let sx = (mb_col * 16 + c).min(w.saturating_sub(1));
            out.push(frame.y.get(sx, sy).unwrap_or(0));
        }
    }
    out
}

/// Quantise the residual of a macroblock against its prediction planes.
#[allow(clippy::too_many_arguments)]
fn quantise_residual(
    current: &FrameBuffer,
    luma_pred: &[u8],
    cb_pred: &[u8],
    cr_pred: &[u8],
    mb_col: usize,
    mb_row: usize,
    qscale: u8,
    chroma_format: ChromaFormat,
) -> (Vec<InterBlock>, [bool; 12]) {
    let nblocks = block_count(chroma_format);
    let (cmb_w, cmb_h) = chroma_mb_extent(chroma_format);
    let mut blocks = Vec::with_capacity(nblocks);
    let mut coded_flags = [false; 12];
    for i in 0..nblocks {
        let placement =
            block_placement(i, chroma_format, mb_col, mb_row, false).expect("valid block index");
        let component = block_component(i, chroma_format).expect("valid component");
        let (cur_plane, pred, pred_w, origin_x, origin_y) = match component {
            ColourComponent::Y => (&current.y, luma_pred, 16usize, mb_col * 16, mb_row * 16),
            ColourComponent::Cb => (&current.cb, cb_pred, cmb_w, mb_col * cmb_w, mb_row * cmb_h),
            ColourComponent::Cr => (&current.cr, cr_pred, cmb_w, mb_col * cmb_w, mb_row * cmb_h),
        };
        let residual = gather_residual(
            cur_plane,
            pred,
            pred_w,
            placement.base_x,
            placement.base_y,
            placement.base_x - origin_x,
            placement.base_y - origin_y,
        );
        let block = if nonintra_block_has_cbp_slot(i, chroma_format) {
            quantise_inter_block(&residual, qscale, &DEFAULT_NON_INTRA_WEIGHT)
        } else {
            InterBlock::uncoded()
        };
        coded_flags[i] = block.is_coded();
        blocks.push(block);
    }
    (blocks, coded_flags)
}

/// Encode one enhancement picture's slices, returning its
/// reconstruction.
#[allow(clippy::too_many_arguments)]
fn encode_spatial_picture(
    bw: &mut BitWriter,
    source: &FrameBuffer,
    spat: &SpatialPredictionPicture,
    kind: PictureCodingType,
    forward: Option<&FrameBuffer>,
    backward: Option<&FrameBuffer>,
    geometry: IntraPictureParams,
    config: &SpatialLayerConfig,
    stats: &mut SpatialStats,
) -> Result<FrameBuffer> {
    let qscale = quantiser_scale(config.quantiser_scale_code, geometry.q_scale_type)?;
    let dc_mult = intra_dc_mult(geometry.intra_dc_precision)?;
    let search_range = max_search_range(config.f_code).min(16);
    let chroma_format = geometry.chroma_format;
    let nblocks = block_count(chroma_format);
    let mb_width = geometry.mb_width();
    let mb_height = geometry.mb_height();
    let mut recon = geometry.new_frame_buffer();

    for mb_row in 0..mb_height {
        write_slice_header_in(
            bw,
            mb_row as u32,
            config.quantiser_scale_code,
            geometry.height as u32,
        );
        let mut pmv_fwd = (0i32, 0i32);
        let mut pmv_bwd = (0i32, 0i32);
        let mut intra_pred = IntraDcPred::reset(geometry.intra_dc_precision);
        let slice_first_addr = (mb_row * mb_width) as i32;
        let mut past_intra_address = slice_first_addr - 2;

        for mb_col in 0..mb_width {
            let mb_address = slice_first_addr + mb_col as i32;
            let cur = current_luma(source, mb_col, mb_row);
            let (sl, scb, scr) = spatial_macroblock_planes(spat, chroma_format, mb_col, mb_row);
            let spatial_sad = sad(&cur, &sl);
            let intra_cost = intra_activity(source, mb_col, mb_row);

            // Temporal candidates.
            let fwd_search =
                forward.map(|f| estimate_forward_mv(source, f, mb_col, mb_row, search_range));
            let bwd_search =
                backward.map(|b| estimate_forward_mv(source, b, mb_col, mb_row, search_range));

            // Build the candidate list: (kind, luma sad, planes, vectors).
            struct Candidate {
                kind: Kind,
                cost: u32,
                planes: (Vec<u8>, Vec<u8>, Vec<u8>),
                fwd: Option<MotionVectorPel>,
                bwd: Option<MotionVectorPel>,
            }
            let mut candidates: Vec<Candidate> = Vec::new();
            if kind != PictureCodingType::Intra {
                candidates.push(Candidate {
                    kind: Kind::SpatialOnly,
                    cost: spatial_sad,
                    planes: (sl.clone(), scb.clone(), scr.clone()),
                    fwd: None,
                    bwd: None,
                });
            }
            let refs = ReferenceFrames { forward, backward };
            if let Some(fs) = &fwd_search {
                let planes = predict_frame_macroblock_planes(
                    &recon,
                    refs,
                    mb_col,
                    mb_row,
                    FrameMotion::forward(fs.vector),
                )
                .map_err(Error::from)?;
                let t = sad(&cur, &planes.0);
                let half = (
                    half_planes(&planes.0, &sl),
                    half_planes(&planes.1, &scb),
                    half_planes(&planes.2, &scr),
                );
                let h = sad(&cur, &half.0);
                candidates.push(Candidate {
                    kind: Kind::Temporal {
                        forward: true,
                        backward: false,
                    },
                    cost: t,
                    planes,
                    fwd: Some(fs.vector),
                    bwd: None,
                });
                candidates.push(Candidate {
                    kind: Kind::HalfWeight {
                        forward: true,
                        backward: false,
                    },
                    cost: h,
                    planes: half,
                    fwd: Some(fs.vector),
                    bwd: None,
                });
            }
            if let (Some(fs), Some(bs)) = (&fwd_search, &bwd_search) {
                let planes = predict_frame_macroblock_planes(
                    &recon,
                    refs,
                    mb_col,
                    mb_row,
                    FrameMotion {
                        forward: Some(fs.vector),
                        backward: Some(bs.vector),
                    },
                )
                .map_err(Error::from)?;
                let t = sad(&cur, &planes.0);
                candidates.push(Candidate {
                    kind: Kind::Temporal {
                        forward: true,
                        backward: true,
                    },
                    cost: t,
                    planes,
                    fwd: Some(fs.vector),
                    bwd: Some(bs.vector),
                });
            }
            if let Some(bs) = &bwd_search {
                let planes = predict_frame_macroblock_planes(
                    &recon,
                    refs,
                    mb_col,
                    mb_row,
                    FrameMotion {
                        forward: None,
                        backward: Some(bs.vector),
                    },
                )
                .map_err(Error::from)?;
                let t = sad(&cur, &planes.0);
                candidates.push(Candidate {
                    kind: Kind::Temporal {
                        forward: false,
                        backward: true,
                    },
                    cost: t,
                    planes,
                    fwd: None,
                    bwd: Some(bs.vector),
                });
            }
            if kind == PictureCodingType::Intra {
                // I pictures: spatial-only versus intra.
                candidates.push(Candidate {
                    kind: Kind::SpatialOnly,
                    cost: spatial_sad,
                    planes: (sl.clone(), scb.clone(), scr.clone()),
                    fwd: None,
                    bwd: None,
                });
            }
            let best = candidates
                .into_iter()
                .min_by_key(|c| c.cost)
                .expect("at least the spatial candidate");

            // Intra fallback when every prediction is poor.
            if best.cost > intra_cost.saturating_mul(2).saturating_add(512)
                || (kind == PictureCodingType::Intra && best.cost > intra_cost)
            {
                if mb_address - past_intra_address > 1 {
                    intra_pred = IntraDcPred::reset(geometry.intra_dc_precision);
                }
                bw.write_bit(true); // macroblock_address_increment = 1
                write_macroblock_type(bw, kind, Kind::Intra, true)?;
                encode_intra_mb(
                    bw,
                    source,
                    &mut recon,
                    mb_col,
                    mb_row,
                    qscale,
                    dc_mult,
                    &mut intra_pred,
                    nblocks,
                    chroma_format,
                    crate::mpeg2_dct_coeff::TableSelection::from_context(
                        geometry.intra_vlc_format,
                        true,
                    ),
                    geometry.alternate_scan,
                    &QuantiserMatrixState::defaults(),
                );
                pmv_fwd = (0, 0);
                pmv_bwd = (0, 0);
                past_intra_address = mb_address;
                stats.intra += 1;
                continue;
            }

            let (blocks, coded_flags) = quantise_residual(
                source,
                &best.planes.0,
                &best.planes.1,
                &best.planes.2,
                mb_col,
                mb_row,
                qscale,
                chroma_format,
            );
            let coded = coded_flags.iter().any(|&f| f);
            bw.write_bit(true); // macroblock_address_increment = 1
            write_macroblock_type(bw, kind, best.kind, coded)?;
            // (weight table 00: no spatial_temporal_weight_code;
            // frame_pred_frame_dct = 1: no frame_motion_type / dct_type.)
            match best.kind {
                Kind::SpatialOnly => {
                    // §7.7.5.1: predictors reset.
                    pmv_fwd = (0, 0);
                    pmv_bwd = (0, 0);
                    stats.spatial_only += 1;
                }
                Kind::Temporal { forward, backward } | Kind::HalfWeight { forward, backward } => {
                    if forward {
                        let mv = best.fwd.expect("forward vector");
                        let dx = wrap_delta(mv.horizontal - pmv_fwd.0, config.f_code)?;
                        let dy = wrap_delta(mv.vertical - pmv_fwd.1, config.f_code)?;
                        encode_motion_component(bw, dx, config.f_code);
                        encode_motion_component(bw, dy, config.f_code);
                        pmv_fwd = (mv.horizontal, mv.vertical);
                    }
                    if backward {
                        let mv = best.bwd.expect("backward vector");
                        let dx = wrap_delta(mv.horizontal - pmv_bwd.0, config.f_code)?;
                        let dy = wrap_delta(mv.vertical - pmv_bwd.1, config.f_code)?;
                        encode_motion_component(bw, dx, config.f_code);
                        encode_motion_component(bw, dy, config.f_code);
                        pmv_bwd = (mv.horizontal, mv.vertical);
                    }
                    if !forward && !backward {
                        pmv_fwd = (0, 0);
                        pmv_bwd = (0, 0);
                    }
                    if matches!(best.kind, Kind::HalfWeight { .. }) {
                        stats.half_weight += 1;
                    } else {
                        stats.temporal += 1;
                    }
                }
                Kind::Intra => unreachable!("handled above"),
            }
            if coded {
                encode_coded_block_pattern(bw, &coded_flags[..nblocks], chroma_format)?;
                for b in &blocks {
                    if let Some(qf) = b.qf_ref() {
                        write_inter_block_coeffs(bw, qf, geometry.alternate_scan);
                    }
                }
            }
            reconstruct_inter_mb(
                &mut recon,
                mb_col,
                mb_row,
                &best.planes.0,
                &best.planes.1,
                &best.planes.2,
                &blocks,
            );
        }
        bw.align_to_byte_zero();
    }
    Ok(recon)
}

/// Encode a §7.7 spatial **enhancement layer** for the lower-layer
/// stream `base`: `sources` are the full-resolution frames in display
/// order (one per lower-layer picture, an integer multiple of the
/// lower geometry in each axis, any chroma format not lower than the
/// lower layer's). The enhancement layer mirrors the lower layer's GOP
/// structure (coded order, picture types, temporal references, GOP
/// headers); the coincident lower frame is every picture's spatial
/// reference.
///
/// # Errors
/// [`Error::InvalidBitstream`] for a lower layer this loop does not
/// compose (ISO/IEC 11172-2, interlaced, field pictures), a source
/// list of the wrong length / geometry, or out-of-range configuration.
pub fn encode_spatial_enhancement_layer(
    base: &[u8],
    sources: &[FrameBuffer],
    config: &SpatialLayerConfig,
) -> Result<SpatialEncoded> {
    if !(1..=31).contains(&config.quantiser_scale_code) {
        return Err(Error::InvalidBitstream(
            "spatial encoder: quantiser_scale_code must be in 1..=31",
        ));
    }
    if !(1..=9).contains(&config.f_code) {
        return Err(Error::InvalidBitstream(
            "spatial encoder: f_code must be in 1..=9",
        ));
    }
    let lower_seq = Mpeg2Sequence::from_buf(base).map_err(|_| {
        Error::InvalidBitstream(
            "spatial encoder: the lower layer shall conform to ISO/IEC 13818-2 (an ISO/IEC 11172-2 lower layer is not composed)",
        )
    })?;
    if !lower_seq.extension.progressive_sequence {
        return Err(Error::InvalidBitstream(
            "spatial encoder: interlaced lower layers (weight tables 01 / 10 / 11) are not composed",
        ));
    }
    let lower = decode_video_sequence(base)?;
    let (pictures, gops) = layout(base);
    if lower.is_empty() || sources.len() != lower.len() || pictures.len() != lower.len() {
        return Err(Error::InvalidBitstream(
            "spatial encoder: one source frame per lower-layer picture",
        ));
    }
    let first = &sources[0];
    let (ew, eh) = (first.width, first.height);
    let (lw, lh) = (
        lower_seq.horizontal_size as usize,
        lower_seq.vertical_size as usize,
    );
    if ew < lw || eh < lh || ew % lw != 0 && ew * 1 != lw || ew == 0 || eh == 0 {
        return Err(Error::InvalidBitstream(
            "spatial encoder: enhancement geometry must be at least the lower layer's",
        ));
    }
    if (first.chroma_format as u8) < (lower_seq.extension.chroma_format as u8) {
        return Err(Error::InvalidBitstream(
            "spatial encoder: enhancement chroma_format shall not be lower than the lower layer's (Table 7-18)",
        ));
    }
    for s in sources {
        if s.width != ew || s.height != eh || s.chroma_format != first.chroma_format {
            return Err(Error::InvalidBitstream(
                "spatial encoder: every source frame must share one geometry / chroma format",
            ));
        }
    }
    if ew > 4095 || eh > 4095 {
        return Err(Error::InvalidBitstream(
            "spatial encoder: enhancement geometry beyond the 12-bit sequence header",
        ));
    }

    // Table 7-16 subsampling factors: upsampled size = lower * n / m.
    let g_h = gcd(ew as u32, lw as u32);
    let g_v = gcd(eh as u32, lh as u32);
    let sp = SpatialScalabilityParams {
        lower_layer_prediction_horizontal_size: lw as u16,
        lower_layer_prediction_vertical_size: lh as u16,
        horizontal_subsampling_factor_m: (lw as u32 / g_h) as u8,
        horizontal_subsampling_factor_n: (ew as u32 / g_h) as u8,
        vertical_subsampling_factor_m: (lh as u32 / g_v) as u8,
        vertical_subsampling_factor_n: (eh as u32 / g_v) as u8,
    };
    if [
        sp.horizontal_subsampling_factor_m,
        sp.horizontal_subsampling_factor_n,
        sp.vertical_subsampling_factor_m,
        sp.vertical_subsampling_factor_n,
    ]
    .iter()
    .any(|&f| f == 0 || f > 31)
    {
        return Err(Error::InvalidBitstream(
            "spatial encoder: subsampling factors must fit the 5-bit §6.2.2.5 fields",
        ));
    }

    let geometry = IntraPictureParams {
        width: ew,
        height: eh,
        chroma_format: first.chroma_format,
        frame_pred_frame_dct: true,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        progressive_sequence: true,
    };

    // Pre-scan the lower pictures: coded order, types, display indices.
    let mut coded: Vec<(u16, PictureCodingType)> = Vec::with_capacity(pictures.len());
    for &(start, end) in &pictures {
        let header = Mpeg2PictureHeader::parse(&base[start..end])?;
        coded.push((header.temporal_reference, header.picture_coding_type));
    }
    let display_index = display_indices_from_coded_pictures(&coded);

    let mut bw = BitWriter::new();
    write_sequence_header(
        &mut bw,
        &SequenceHeaderParams {
            horizontal_size: ew as u16,
            vertical_size: eh as u16,
            aspect_ratio_code: aspect_ratio_code(lower_seq.header.aspect_ratio),
            frame_rate_code: lower_seq.header.frame_rate_code,
            bit_rate_value: lower_seq.header.bit_rate,
            vbv_buffer_size_value: lower_seq.header.vbv_buffer_size,
            intra_quantiser_matrix: None,
            non_intra_quantiser_matrix: None,
        },
    );
    write_sequence_extension_fields(
        &mut bw,
        &Mpeg2SequenceExtension {
            profile_and_level: crate::stream_writer::profile_and_level_indication(
                first.chroma_format,
            ),
            chroma_format: first.chroma_format,
            ..lower_seq.extension
        },
    );
    write_sequence_scalable_extension(
        &mut bw,
        &SequenceScalableExtension {
            scalable_mode: ScalableMode::SpatialScalability(sp),
            layer_id: 1,
        },
    );

    let mut stats = SpatialStats::default();
    let mut recons: Vec<Option<DecodedFrame>> = vec![None; sources.len()];
    let mut forward_anchor: Option<FrameBuffer> = None;
    let mut backward_anchor: Option<FrameBuffer> = None;
    let mut next_gop = 0usize;

    for (index, &(tref, kind)) in coded.iter().enumerate() {
        while let Some(&(before, start, end)) = gops.get(next_gop) {
            if before > index {
                break;
            }
            bw.write_bytes(&base[start..end]);
            next_gop += 1;
        }
        let d = display_index[index] as usize;
        let source = &sources[d];
        let lower_frame = lower.get(d).ok_or(Error::InvalidBitstream(
            "spatial encoder: lower-layer display index beyond its frames",
        ))?;
        let pss = PictureSpatialScalableExtension {
            lower_layer_temporal_reference: lower_frame.temporal_reference & 0x3FF,
            lower_layer_horizontal_offset: 0,
            lower_layer_vertical_offset: 0,
            spatial_temporal_weight_code_table_index: 0,
            lower_layer_progressive_frame: true,
            // Table 7-15: a progressive lower frame is used whole
            // (lower_layer_deinterlaced_field_select = 1).
            lower_layer_deinterlaced_field_select: true,
        };
        let spat = spatial_prediction_picture(
            &lower_frame.frame,
            &sp,
            &pss,
            lower_seq.extension.chroma_format,
            first.chroma_format,
            true,
            ew as u32,
            eh as u32,
            true,
        )?;

        // Picture layer.
        write_picture_header(&mut bw, tref, kind, 0b111, 0b111);
        let (fwd_code, bwd_code) = match kind {
            PictureCodingType::Intra => (15, 15),
            PictureCodingType::Predictive => (config.f_code, 15),
            _ => (config.f_code, config.f_code),
        };
        write_picture_coding_extension(
            &mut bw,
            &PictureCodingExtensionParams {
                forward_f_code: fwd_code,
                backward_f_code: bwd_code,
                intra_dc_precision: 0,
                frame_pred_frame_dct: true,
                q_scale_type: false,
                intra_vlc_format: false,
                alternate_scan: false,
                progressive_frame: true,
                top_field_first: false,
                repeat_first_field: false,
                concealment_motion_vectors: false,
                chroma_format: first.chroma_format,
            },
        );
        write_picture_spatial_scalable_extension(&mut bw, &pss);

        let (forward, backward) = match kind {
            PictureCodingType::Intra => (None, None),
            PictureCodingType::Predictive => (
                Some(backward_anchor.as_ref().ok_or(Error::InvalidBitstream(
                    "spatial encoder: P picture before an anchor",
                ))?),
                None,
            ),
            PictureCodingType::Bidirectional => (
                Some(forward_anchor.as_ref().ok_or(Error::InvalidBitstream(
                    "spatial encoder: B picture before two anchors",
                ))?),
                Some(backward_anchor.as_ref().ok_or(Error::InvalidBitstream(
                    "spatial encoder: B picture before two anchors",
                ))?),
            ),
            PictureCodingType::DcIntra => {
                return Err(Error::InvalidBitstream(
                    "spatial encoder: D-pictures do not occur in ISO/IEC 13818-2",
                ))
            }
        };
        let recon = encode_spatial_picture(
            &mut bw, source, &spat, kind, forward, backward, geometry, config, &mut stats,
        )?;
        if kind != PictureCodingType::Bidirectional {
            forward_anchor = backward_anchor.take();
            backward_anchor = Some(recon.clone());
        }
        recons[d] = Some(DecodedFrame {
            frame: recon,
            temporal_reference: tref,
            picture_coding_type: kind,
            top_field_first: false,
            repeat_first_field: false,
            progressive_frame: true,
        });
    }

    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
    let enhancement: Vec<DecodedFrame> = recons
        .into_iter()
        .map(|r| {
            r.ok_or(Error::InvalidBitstream(
                "spatial encoder: a display slot was never coded",
            ))
        })
        .collect::<Result<_>>()?;
    Ok(SpatialEncoded {
        stream,
        lower,
        enhancement,
        stats,
    })
}
