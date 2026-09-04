//! §7.9 **Temporal scalability** — the top-level two-layer loop that
//! decodes a temporal enhancement-layer bitstream against the lower
//! layer's decoded frames, the §6.3.7 remultiplex, and the self-made
//! enhancement-layer **encoder** that is this crate's only oracle for
//! the layer (no black-box reference in reach produces or consumes a
//! temporal enhancement layer; the lower layer stays an ordinary
//! ISO/IEC 13818-2 stream any decoder accepts).
//!
//! # The decoding process (§7.9)
//!
//! Both layers share one spatial resolution. Enhancement pictures are
//! decoded exactly as ordinary pictures (§7.1 – §7.6) except for the
//! **prediction reference selection** (§7.6.2): every picture carries
//! a `picture_temporal_scalable_extension()` whose
//! `reference_select_code` picks, per Tables 7-28 / 7-29, the most
//! recent decoded enhancement picture, the most recent lower-layer
//! frame in display order, or the next lower-layer frame in display
//! order (a backward-in-time forward reference, or a forward-in-time
//! backward reference, is allowed); `forward_temporal_reference` /
//! `backward_temporal_reference` name the lower-layer frames
//! (§6.3.13). Backward prediction never comes from the enhancement
//! layer, so enhancement pictures are output in coded order (no
//! reorder), and a decoded B picture may itself be the "most recent
//! enhancement picture" for what follows.
//!
//! "Most recent" and "next" lower-layer frames are resolved by the
//! picture's **position in the multiplex** — the
//! `picture_mux_order` / `picture_mux_factor` of the enhancement
//! layer's `sequence_scalable_extension()` (`picture_mux_enable = 1`
//! is required: without it the temporal alignment of the layers is a
//! systems-layer matter, §7.9.2) — and the extension's temporal
//! references are checked against the frames so found. Frame pictures
//! only (both layers); the lower layer must conform to ISO/IEC 13818-2
//! (§7.9.1).
//!
//! # The encoder
//!
//! [`encode_temporal_enhancement_layer`] codes the enhancement
//! pictures that sit between the lower layer's frames: the
//! `picture_mux_order` pictures before the first lower frame as P
//! pictures predicted from the *next* lower frame
//! (`reference_select_code = 10`), and the `picture_mux_factor`
//! pictures between two lower frames as B pictures — the first from
//! the two surrounding lower frames (`11`), the rest optionally from
//! the most recent enhancement picture forward and the next lower
//! frame backward (`10`). Every reference is the decoder's exact
//! reconstruction, so [`decode_temporal_scalable_sequence`] reproduces
//! the returned enhancement frames sample for sample.

use oxideav_core::bits::BitWriter;

use crate::frame_assembly::{
    decode_intra_picture_with_context, FrameBuffer, IntraDecodeContext, IntraPictureParams,
};
use crate::inter_reconstruction::ReferenceFrames;
use crate::picture_header::{Mpeg2PictureHeader, PictureCodingType, PictureStructure};
use crate::picture_reconstruction::{decode_inter_picture_with_matrices, PicturePredictionParams};
use crate::picture_temporal_scalable_extension::{
    write_picture_temporal_scalable_extension, PictureReferences, PictureTemporalScalableExtension,
    ReferenceSource, PICTURE_TEMPORAL_SCALABLE_EXTENSION_ID,
};
use crate::quant_matrix_extension::QuantiserMatrixState;
use crate::sequence_extension::{ChromaFormat, Mpeg2Sequence, Mpeg2SequenceExtension};
use crate::sequence_scalable_extension::{
    write_sequence_scalable_extension, ScalableMode, SequenceScalableExtension,
    TemporalScalabilityParams, SEQUENCE_SCALABLE_EXTENSION_ID,
};
use crate::stream_writer::{write_sequence_header, SequenceHeaderParams, SEQUENCE_END_CODE};
use crate::video_sequence::{apply_quant_matrix_extensions, decode_video_sequence, DecodedFrame};
use crate::{Error, Result};

// -------------------------------------------------------------------
// Stream scanning (shared shape with the SNR loop)
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

/// `(start, end_with_terminator)` of every picture in coded order.
fn picture_spans(stream: &[u8]) -> Vec<(usize, usize)> {
    let codes = scan_start_codes(stream);
    let mut out = Vec::new();
    for (k, &(off, code)) in codes.iter().enumerate() {
        if code != 0x00 {
            continue;
        }
        let boundary = codes[k + 1..]
            .iter()
            .find(|&&(_, c)| matches!(c, 0x00 | 0xB8 | 0xB3 | 0xB7))
            .map(|&(o, _)| (o + 4).min(stream.len()))
            .unwrap_or(stream.len());
        out.push((off, boundary));
    }
    out
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

/// The temporal-scalability parameters of an enhancement layer's
/// leading `sequence_scalable_extension()`.
fn temporal_params(stream: &[u8]) -> Result<TemporalScalabilityParams> {
    let codes = scan_start_codes(stream);
    for &(off, code) in &codes {
        if code == 0x00 || code == 0xB8 {
            break;
        }
        if code == 0xB5
            && stream.get(off + 4).map(|b| b >> 4) == Some(SEQUENCE_SCALABLE_EXTENSION_ID as u8)
        {
            let sse = SequenceScalableExtension::parse(&stream[off..])?;
            return match sse.scalable_mode {
                ScalableMode::TemporalScalability(p) if sse.layer_id == 1 => Ok(p),
                _ => Err(Error::InvalidBitstream(
                    "temporal scalability: enhancement layer shall declare scalable_mode = temporal scalability, layer_id = 1 (§7.9.1)",
                )),
            };
        }
    }
    Err(Error::InvalidBitstream(
        "temporal scalability: enhancement layer carries no sequence_scalable_extension()",
    ))
}

/// The `picture_temporal_scalable_extension()` inside a picture region.
fn temporal_extension(region: &[u8]) -> Result<PictureTemporalScalableExtension> {
    let codes = scan_start_codes(region);
    for &(off, code) in &codes {
        if (0x01..=0xAF).contains(&code) {
            break;
        }
        if code == 0xB5
            && region.get(off + 4).map(|b| b >> 4)
                == Some(PICTURE_TEMPORAL_SCALABLE_EXTENSION_ID as u8)
        {
            return PictureTemporalScalableExtension::parse(&region[off..]);
        }
    }
    Err(Error::InvalidBitstream(
        "temporal scalability: picture_temporal_scalable_extension() shall be present for each enhancement picture (§7.9.1)",
    ))
}

/// The lower-layer frames an enhancement picture at multiplex
/// position `e` (0-based, coded = display order) sits between.
fn neighbours(e: usize, mux: &TemporalScalabilityParams) -> (Option<usize>, usize) {
    let order = usize::from(mux.picture_mux_order);
    let factor = usize::from(mux.picture_mux_factor).max(1);
    if e < order {
        (None, 0)
    } else {
        let j = (e - order) / factor;
        (Some(j), j + 1)
    }
}

// -------------------------------------------------------------------
// Decoding
// -------------------------------------------------------------------

/// The result of [`decode_temporal_scalable_sequence`].
#[derive(Debug, Clone)]
pub struct TemporalScalableDecoded {
    /// The lower layer's frames in display order.
    pub lower: Vec<DecodedFrame>,
    /// The enhancement layer's frames in coded (= display) order.
    pub enhancement: Vec<DecodedFrame>,
    /// The enhancement layer's §6.3.7 multiplex parameters.
    pub mux: TemporalScalabilityParams,
}

impl TemporalScalableDecoded {
    /// The §6.3.7 remultiplex: `picture_mux_order` enhancement frames,
    /// the first lower frame, then `picture_mux_factor` enhancement
    /// frames between consecutive lower frames — the full-rate display
    /// sequence.
    pub fn remultiplex(&self) -> Vec<&DecodedFrame> {
        remultiplex(&self.lower, &self.enhancement, &self.mux)
    }
}

/// See [`TemporalScalableDecoded::remultiplex`].
pub fn remultiplex<'a>(
    lower: &'a [DecodedFrame],
    enhancement: &'a [DecodedFrame],
    mux: &TemporalScalabilityParams,
) -> Vec<&'a DecodedFrame> {
    let order = usize::from(mux.picture_mux_order);
    let factor = usize::from(mux.picture_mux_factor).max(1);
    let mut out = Vec::with_capacity(lower.len() + enhancement.len());
    let mut e = enhancement.iter();
    for _ in 0..order {
        if let Some(f) = e.next() {
            out.push(f);
        }
    }
    for (j, l) in lower.iter().enumerate() {
        out.push(l);
        if j + 1 < lower.len() {
            for _ in 0..factor {
                if let Some(f) = e.next() {
                    out.push(f);
                }
            }
        }
    }
    out.extend(e);
    out
}

/// Decode a §7.9 temporal-scalable pair — the lower-layer stream
/// `base` (an ordinary ISO/IEC 13818-2 elementary stream) and its
/// temporal `enhancement` layer — into both layers' frames.
///
/// # Errors
/// [`Error::InvalidBitstream`] when the lower layer is not an ISO/IEC
/// 13818-2 stream, when the enhancement layer's sequence layer
/// disagrees with the lower layer's geometry (§7.9.1), lacks a temporal
/// `sequence_scalable_extension()` with `picture_mux_enable = 1`, when
/// a picture lacks its `picture_temporal_scalable_extension()`, names
/// a reference that does not exist or whose temporal reference
/// disagrees, or for any syntax error in either layer;
/// [`Error::ShortHeader`] on truncation.
pub fn decode_temporal_scalable_sequence(
    base: &[u8],
    enhancement: &[u8],
) -> Result<TemporalScalableDecoded> {
    let lower_seq = Mpeg2Sequence::from_buf(base).map_err(|_| {
        Error::InvalidBitstream(
            "temporal scalability: the lower layer shall conform to ISO/IEC 13818-2 (§7.9.1)",
        )
    })?;
    let lower = decode_video_sequence(base)?;

    let seq = Mpeg2Sequence::from_buf(enhancement).map_err(|_| {
        Error::InvalidBitstream(
            "temporal scalability: enhancement layer sequence_header / sequence_extension missing or malformed",
        )
    })?;
    if seq.horizontal_size != lower_seq.horizontal_size
        || seq.vertical_size != lower_seq.vertical_size
    {
        return Err(Error::InvalidBitstream(
            "temporal scalability: enhancement layer horizontal_size / vertical_size differ from the lower layer (§7.9.1)",
        ));
    }
    if seq.extension.chroma_format != lower_seq.extension.chroma_format {
        return Err(Error::InvalidBitstream(
            "temporal scalability: enhancement sequence_extension shall match the lower layer's chroma_format (§7.9.1)",
        ));
    }
    let mux = temporal_params(enhancement)?;
    if !mux.picture_mux_enable {
        return Err(Error::InvalidBitstream(
            "temporal scalability: picture_mux_enable = 0 leaves the layers' temporal alignment to the systems layer (§7.9.2); not composed",
        ));
    }
    if mux.picture_mux_factor == 0 {
        return Err(Error::InvalidBitstream(
            "picture_mux_factor: '000' is reserved (§6.3.7)",
        ));
    }
    match (seq.extension.progressive_sequence, mux.mux_to_progressive_sequence) {
        (true, Some(false)) | (true, None) => {
            return Err(Error::InvalidBitstream(
                "temporal scalability: progressive_sequence = 1 with mux_to_progressive_sequence = 0 shall not occur (§7.9.1)",
            ))
        }
        _ => {}
    }

    let base_geometry = geometry_of(&seq);
    let mut matrices = initial_matrices(&seq);
    let mut output: Vec<DecodedFrame> = Vec::new();
    let mut most_recent: Option<FrameBuffer> = None;

    for (e, &(start, end)) in picture_spans(enhancement).iter().enumerate() {
        let region = &enhancement[start..end];
        let (header, ext) = Mpeg2PictureHeader::parse_with_extension(region)?;
        if ext.picture_structure != PictureStructure::Frame {
            return Err(Error::InvalidBitstream(
                "temporal scalability: field pictures are not composed by this loop (frame pictures only)",
            ));
        }
        apply_quant_matrix_extensions(region, base_geometry.chroma_format, &mut matrices)?;
        let tse = temporal_extension(region)?;
        tse.validate(header.picture_coding_type)?;
        let refs = tse.resolve_references(header.picture_coding_type)?;
        let geometry = IntraPictureParams {
            frame_pred_frame_dct: ext.frame_pred_frame_dct,
            intra_dc_precision: ext.intra_dc_precision,
            intra_vlc_format: ext.intra_vlc_format,
            alternate_scan: ext.alternate_scan,
            q_scale_type: ext.q_scale_type,
            ..base_geometry
        };

        let (recent_lower, next_lower) = neighbours(e, &mux);
        let lower_frame = |which: ReferenceSource, tref: u16| -> Result<&FrameBuffer> {
            let index = match which {
                ReferenceSource::MostRecentLowerLayer => recent_lower.ok_or(Error::InvalidBitstream(
                    "temporal scalability: no most-recent lower-layer frame before the first lower picture (Table 7-28 / 7-29)",
                ))?,
                ReferenceSource::NextLowerLayer => next_lower,
                ReferenceSource::MostRecentEnhancement => unreachable!("lower-layer sources only"),
            };
            let frame = lower.get(index).ok_or(Error::InvalidBitstream(
                "temporal scalability: the lower layer has no frame at the multiplex position the enhancement picture references",
            ))?;
            if frame.temporal_reference & 0x3FF != tref {
                return Err(Error::InvalidBitstream(
                    "temporal scalability: forward/backward_temporal_reference disagrees with the lower-layer frame at that multiplex position (§6.3.13)",
                ));
            }
            Ok(&frame.frame)
        };
        let resolve = |source: ReferenceSource, tref: u16| -> Result<&FrameBuffer> {
            match source {
                ReferenceSource::MostRecentEnhancement => {
                    most_recent.as_ref().ok_or(Error::InvalidBitstream(
                        "temporal scalability: no most-recent enhancement picture to predict from",
                    ))
                }
                other => lower_frame(other, tref),
            }
        };

        let frame = match refs {
            PictureReferences::Intra => {
                let (frame, placed) = decode_intra_picture_with_context(
                    region,
                    geometry,
                    &matrices,
                    IntraDecodeContext {
                        concealment_motion_vectors: ext.concealment_motion_vectors,
                        f_code_fwd_horiz: ext.f_code_fwd_horiz,
                        f_code_fwd_vert: ext.f_code_fwd_vert,
                    },
                )?;
                full_coverage(placed, &geometry)?;
                frame
            }
            PictureReferences::Predictive { forward } => {
                let fwd = resolve(forward, tse.forward_temporal_reference)?;
                let params = prediction_params(&header, &ext, geometry);
                let (frame, placed) = decode_inter_picture_with_matrices(
                    region,
                    params,
                    ReferenceFrames::forward_only(fwd),
                    &matrices,
                )?;
                full_coverage(placed, &geometry)?;
                frame
            }
            PictureReferences::Bidirectional { forward, backward } => {
                let fwd = resolve(forward, tse.forward_temporal_reference)?;
                let bwd = lower_frame(backward, tse.backward_temporal_reference)?;
                let params = prediction_params(&header, &ext, geometry);
                let (frame, placed) = decode_inter_picture_with_matrices(
                    region,
                    params,
                    ReferenceFrames::bidirectional(fwd, bwd),
                    &matrices,
                )?;
                full_coverage(placed, &geometry)?;
                frame
            }
        };
        most_recent = Some(frame.clone());
        output.push(DecodedFrame {
            frame,
            temporal_reference: header.temporal_reference,
            picture_coding_type: header.picture_coding_type,
            top_field_first: ext.top_field_first,
            repeat_first_field: ext.repeat_first_field,
            progressive_frame: ext.progressive_frame,
        });
    }

    Ok(TemporalScalableDecoded {
        lower,
        enhancement: output,
        mux,
    })
}

fn full_coverage(placed: usize, geometry: &IntraPictureParams) -> Result<()> {
    if placed != geometry.mb_width() * geometry.mb_height() {
        return Err(Error::InvalidBitstream(
            "§6.1.2.2: the picture's slices do not enclose every macroblock exactly once (restricted slice structure, Table 8-5)",
        ));
    }
    Ok(())
}

fn prediction_params(
    header: &Mpeg2PictureHeader,
    ext: &crate::picture_header::PictureCodingExtension,
    geometry: IntraPictureParams,
) -> PicturePredictionParams {
    PicturePredictionParams {
        geometry,
        picture_coding_type: header.picture_coding_type,
        f_code_fwd_horiz: ext.f_code_fwd_horiz,
        f_code_fwd_vert: ext.f_code_fwd_vert,
        f_code_bwd_horiz: ext.f_code_bwd_horiz,
        f_code_bwd_vert: ext.f_code_bwd_vert,
        concealment_motion_vectors: ext.concealment_motion_vectors,
        top_field_first: ext.top_field_first,
    }
}

// -------------------------------------------------------------------
// Encoding
// -------------------------------------------------------------------

/// Configuration of [`encode_temporal_enhancement_layer`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TemporalLayerConfig {
    /// The per-slice `quantiser_scale_code` (`1..=31`).
    pub quantiser_scale_code: u8,
    /// Motion-vector `f_code` for both directions (`1..=9`).
    pub f_code: u8,
    /// §6.3.7 `picture_mux_order`: enhancement pictures before the
    /// first lower-layer frame (`0..=7`).
    pub picture_mux_order: u8,
    /// §6.3.7 `picture_mux_factor`: enhancement pictures between
    /// consecutive lower-layer frames (`1..=7`).
    pub picture_mux_factor: u8,
    /// Predict the second and later pictures between two lower frames
    /// from the most recent enhancement picture (Table 7-29
    /// `reference_select_code = 10`) instead of the two lower frames
    /// (`11`).
    pub use_enhancement_references: bool,
}

impl Default for TemporalLayerConfig {
    fn default() -> Self {
        Self {
            quantiser_scale_code: 6,
            f_code: 3,
            picture_mux_order: 0,
            picture_mux_factor: 1,
            use_enhancement_references: false,
        }
    }
}

/// The output of [`encode_temporal_enhancement_layer`].
#[derive(Debug, Clone)]
pub struct TemporalEncoded {
    /// The enhancement-layer elementary stream.
    pub stream: Vec<u8>,
    /// The lower layer's decoded frames (display order) the encoder
    /// predicted from.
    pub lower: Vec<DecodedFrame>,
    /// The enhancement pictures' reconstruction, coded (= display)
    /// order — what [`decode_temporal_scalable_sequence`] reproduces.
    pub enhancement: Vec<DecodedFrame>,
    /// The `reference_select_code` of every enhancement picture.
    pub reference_select_codes: Vec<u8>,
}

/// Write a `sequence_extension()` carrying every parsed field.
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

/// Splice a `picture_temporal_scalable_extension()` into a coded
/// picture (header + coding extension + slices) right before its
/// first slice, where the writers left the stream byte-aligned.
fn splice_temporal_extension(picture: &[u8], tse: &PictureTemporalScalableExtension) -> Vec<u8> {
    let first_slice = scan_start_codes(picture)
        .into_iter()
        .find(|&(_, c)| (0x01..=0xAF).contains(&c))
        .map(|(o, _)| o)
        .unwrap_or(picture.len());
    let mut bw = BitWriter::new();
    write_picture_temporal_scalable_extension(&mut bw, tse);
    let ext = bw.finish();
    let mut out = Vec::with_capacity(picture.len() + ext.len());
    out.extend_from_slice(&picture[..first_slice]);
    out.extend_from_slice(&ext);
    out.extend_from_slice(&picture[first_slice..]);
    out
}

/// Encode a §7.9 temporal **enhancement layer** for the lower-layer
/// stream `base`: `sources` are the enhancement pictures in display
/// order — `picture_mux_order` of them before the first lower frame,
/// then `picture_mux_factor` between each pair of consecutive lower
/// frames (exactly `order + factor × (n_lower − 1)` frames, same
/// geometry and chroma format as the lower layer).
///
/// # Errors
/// [`Error::InvalidBitstream`] for a lower layer this loop does not
/// compose (ISO/IEC 11172-2, field pictures), a source list of the
/// wrong length / geometry, or out-of-range configuration.
pub fn encode_temporal_enhancement_layer(
    base: &[u8],
    sources: &[FrameBuffer],
    config: &TemporalLayerConfig,
) -> Result<TemporalEncoded> {
    if !(1..=31).contains(&config.quantiser_scale_code) {
        return Err(Error::InvalidBitstream(
            "temporal encoder: quantiser_scale_code must be in 1..=31",
        ));
    }
    if !(1..=9).contains(&config.f_code) {
        return Err(Error::InvalidBitstream(
            "temporal encoder: f_code must be in 1..=9",
        ));
    }
    if config.picture_mux_order > 7 || !(1..=7).contains(&config.picture_mux_factor) {
        return Err(Error::InvalidBitstream(
            "temporal encoder: picture_mux_order in 0..=7, picture_mux_factor in 1..=7 (§6.2.2.5)",
        ));
    }
    let lower_seq = Mpeg2Sequence::from_buf(base).map_err(|_| {
        Error::InvalidBitstream(
            "temporal encoder: the lower layer shall conform to ISO/IEC 13818-2 (§7.9.1)",
        )
    })?;
    let lower = decode_video_sequence(base)?;
    if lower.is_empty() {
        return Err(Error::InvalidBitstream(
            "temporal encoder: the lower layer has no pictures",
        ));
    }
    let order = usize::from(config.picture_mux_order);
    let factor = usize::from(config.picture_mux_factor);
    let expected = order + factor * (lower.len() - 1);
    if sources.len() != expected {
        return Err(Error::InvalidBitstream(
            "temporal encoder: sources must hold picture_mux_order + picture_mux_factor * (lower frames - 1) pictures",
        ));
    }
    let geometry = geometry_of(&lower_seq);
    for s in sources {
        if s.width != geometry.width
            || s.height != geometry.height
            || s.chroma_format != geometry.chroma_format
        {
            return Err(Error::InvalidBitstream(
                "temporal encoder: source geometry / chroma format does not match the lower layer",
            ));
        }
    }
    if expected > 1023 {
        return Err(Error::InvalidBitstream(
            "temporal encoder: at most 1023 enhancement pictures (10-bit temporal_reference in one GOP)",
        ));
    }

    let mux = TemporalScalabilityParams {
        picture_mux_enable: true,
        // §7.9.1: a progressive lower layer multiplexes to a
        // progressive sequence; an interlaced one keeps top_field_first
        // and the mux factor free (mux_to_progressive_sequence = 0).
        mux_to_progressive_sequence: Some(lower_seq.extension.progressive_sequence),
        picture_mux_order: config.picture_mux_order,
        picture_mux_factor: config.picture_mux_factor,
    };

    let mut bw = BitWriter::new();
    write_sequence_header(
        &mut bw,
        &SequenceHeaderParams {
            horizontal_size: lower_seq.header.width,
            vertical_size: lower_seq.header.height,
            aspect_ratio_code: aspect_ratio_code(lower_seq.header.aspect_ratio),
            frame_rate_code: lower_seq.header.frame_rate_code,
            bit_rate_value: lower_seq.header.bit_rate,
            vbv_buffer_size_value: lower_seq.header.vbv_buffer_size,
            intra_quantiser_matrix: None,
            non_intra_quantiser_matrix: None,
        },
    );
    write_sequence_extension_fields(&mut bw, &lower_seq.extension);
    write_sequence_scalable_extension(
        &mut bw,
        &SequenceScalableExtension {
            scalable_mode: ScalableMode::TemporalScalability(mux),
            layer_id: 1,
        },
    );
    // One GOP for the whole enhancement layer: temporal_reference
    // counts enhancement pictures in display (= coded) order.
    crate::gop_header::write_gop_header(
        &mut bw,
        &crate::gop_header::Mpeg2Gop {
            time_code: crate::gop_header::TimeCode::from_display_index(
                0,
                lower_seq.header.frame_rate_code,
            )?,
            closed_gop: false,
            broken_link: false,
        },
    );

    let matrix_state = QuantiserMatrixState::default();
    let options = crate::encode_options::FrameEncodeOptions::default();
    let mut enhancement: Vec<DecodedFrame> = Vec::with_capacity(sources.len());
    let mut codes: Vec<u8> = Vec::with_capacity(sources.len());
    let mut most_recent: Option<FrameBuffer> = None;

    for (e, source) in sources.iter().enumerate() {
        let (recent_lower, next_lower) = neighbours(e, &mux);
        let tref = e as u16;
        let mut pic = BitWriter::new();
        let (recon, tse, kind) = match recent_lower {
            None => {
                // Before the first lower frame: P from the next lower
                // frame (Table 7-28 '10').
                let next = &lower[next_lower];
                let (recon, _) = crate::p_picture_encoder::encode_p_picture_with_stats(
                    &mut pic,
                    source,
                    &next.frame,
                    geometry,
                    tref,
                    config.quantiser_scale_code,
                    config.f_code,
                    &matrix_state,
                    options,
                )?;
                (
                    recon,
                    PictureTemporalScalableExtension {
                        reference_select_code: 0b10,
                        forward_temporal_reference: next.temporal_reference & 0x3FF,
                        backward_temporal_reference: 0,
                    },
                    PictureCodingType::Predictive,
                )
            }
            Some(j) => {
                let prev = &lower[j];
                let next = lower.get(next_lower).ok_or(Error::InvalidBitstream(
                    "temporal encoder: no next lower-layer frame",
                ))?;
                let position_in_gap = (e - order) % factor;
                let use_enh = config.use_enhancement_references
                    && position_in_gap > 0
                    && most_recent.is_some();
                let forward: &FrameBuffer = if use_enh {
                    most_recent.as_ref().expect("checked")
                } else {
                    &prev.frame
                };
                let _stats = crate::b_picture_encoder::encode_b_picture_with_stats(
                    &mut pic,
                    source,
                    forward,
                    &next.frame,
                    geometry,
                    tref,
                    config.quantiser_scale_code,
                    config.f_code,
                    config.f_code,
                    &matrix_state,
                    options,
                )?;
                // The B encoder returns statistics only; its
                // reconstruction is the decoder's, which we form by
                // decoding the picture we just wrote.
                let coded = pic.finish();
                pic = BitWriter::new();
                pic.write_bytes(&coded);
                let mut with_terminator = coded.clone();
                with_terminator.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
                let (header, ext) = Mpeg2PictureHeader::parse_with_extension(&with_terminator)?;
                let (decoded, placed) = decode_inter_picture_with_matrices(
                    &with_terminator,
                    prediction_params(&header, &ext, geometry),
                    ReferenceFrames::bidirectional(forward, &next.frame),
                    &matrix_state,
                )?;
                full_coverage(placed, &geometry)?;
                let recon = decoded;
                (
                    recon,
                    PictureTemporalScalableExtension {
                        reference_select_code: if use_enh { 0b10 } else { 0b11 },
                        forward_temporal_reference: if use_enh {
                            0
                        } else {
                            prev.temporal_reference & 0x3FF
                        },
                        backward_temporal_reference: next.temporal_reference & 0x3FF,
                    },
                    PictureCodingType::Bidirectional,
                )
            }
        };
        let picture = splice_temporal_extension(&pic.finish(), &tse);
        bw.write_bytes(&picture);
        codes.push(tse.reference_select_code);
        most_recent = Some(recon.clone());
        enhancement.push(DecodedFrame {
            frame: recon,
            temporal_reference: tref,
            picture_coding_type: kind,
            top_field_first: false,
            repeat_first_field: false,
            progressive_frame: geometry.progressive_sequence,
        });
    }

    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
    Ok(TemporalEncoded {
        stream,
        lower,
        enhancement,
        reference_select_codes: codes,
    })
}
