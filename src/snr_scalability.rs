//! §7.8 **SNR scalability** — the top-level two-layer loop that
//! demultiplexes an SNR enhancement-layer bitstream and combines it
//! with the lower layer picture by picture, and the self-made
//! enhancement-layer **encoder** that is this crate's only oracle for
//! the layer (no black-box reference in reach produces or consumes SNR
//! enhancement layers; the lower layer stays an ordinary ISO/IEC
//! 13818-2 stream any decoder accepts).
//!
//! # The decoding process (§7.8)
//!
//! The enhancement layer carries only refinement DCT coefficients. Its
//! macroblocks use Table B-8 (`Coded` / `Coded, Quant` / `Not Coded`
//! — never intra, never motion), its blocks are VLC-decoded and
//! inverse-quantised **as non-intra blocks** (§7.8.3.1 / §7.8.3.3,
//! the enhancement layer's own `q_scale_type` / `alternate_scan` and
//! non-intra matrices), and the two layers' inverse-quantised
//! coefficients are added before the §7.4.3 saturation (§7.8.3.4):
//!
//! ```text
//! F''[v][u] = F''lower[v][u] + F''enhance[v][u]
//! ```
//!
//! after which the remaining steps — saturation, mismatch control,
//! IDCT, motion compensation — run once on the sum (§7.8.3.5), with
//! the prediction `p[y][x]` taken from the lower layer's macroblock
//! syntax (motion type, vectors, skips) and formed from the **combined**
//! reconstruction in the frame store (Figure 7-15: one frame store
//! feeds both layers). A macroblock skipped or `Not Coded` in the
//! enhancement layer contributes `F''enhance = 0`; one skipped in the
//! lower layer but coded in the enhancement layer takes
//! `F''lower = 0` (§7.8.2.2). `dct_type`, when present in both layers,
//! is the same (§7.8.2.1); when the lower layer left it unsignalled
//! (a `Not Coded` / skipped macroblock) the enhancement layer's value
//! governs the block organisation.
//!
//! The §7.8.1 header restrictions the loop relies on: the enhancement
//! sequence header / sequence extension match the lower layer's
//! geometry and chroma format (the `chroma_simulcast` case of Table
//! 7-26 is **not** composed — a differing chroma format is rejected),
//! the sequence scalable extension declares `scalable_mode = SNR`,
//! GOP headers and picture headers coincide (same picture types in the
//! same coded order), slices are coincident with the lower layer's.
//! Lower layers conforming to ISO/IEC 11172-2 and field pictures are
//! outside this loop (frame pictures only — both
//! `frame_pred_frame_dct` values).
//!
//! # The encoder
//!
//! [`encode_snr_enhancement_layer`] runs the same combined loop over
//! an existing lower-layer stream with the original source frames in
//! hand: for every block it forms the in-loop lower-only
//! reconstruction `clamp(p + IDCT(F'lower))`, DCTs the remaining error
//! against the source, quantises it as a non-intra block at the
//! enhancement quantiser, and emits Table B-8 macroblocks in slices
//! coincident with the lower layer's. Its returned frames are the
//! combined reconstruction — exactly what
//! [`decode_snr_scalable_sequence`] reproduces, sample for sample.

// The block arithmetic indexes 8×8 arrays by the same `[v][u]` /
// `[y][x]` loop variables the spec's formulae use.
#![allow(clippy::needless_range_loop)]

use oxideav_core::bits::{BitReader, BitWriter};

use crate::coded_block_pattern::{encode_coded_block_pattern, CodedBlockPattern};
use crate::forward_dct::fdct_8x8;
use crate::forward_quant::forward_quantise_block;
use crate::frame_assembly::{block_placement, BlockPlacement, FrameBuffer, IntraPictureParams};
use crate::idct::idct_8x8_from_i32;
use crate::inter_reconstruction::ReferenceFrames;
use crate::macroblock_type::{MacroblockType, MacroblockTypeTable};
use crate::mb_address_increment::{MbAddressIncrement, MbAddressIncrementContext};
use crate::mpeg2_block_dc::{ColourComponent, DcPredictors};
use crate::mpeg2_block_decoder::{decode_block, BlockContext};
use crate::mpeg2_dequantize::{
    intra_dc_mult, inverse_quantise_arithmetic, quantiser_scale, saturate_and_mismatch, BlockCoding,
};
use crate::mpeg2_macroblock_blocks::{block_component, block_count};
use crate::p_picture_encoder::{nonintra_block_has_cbp_slot, write_inter_block_coeffs};
use crate::picture_header::{
    Mpeg2PictureHeader, PictureCodingExtension, PictureCodingType, PictureStructure,
};
use crate::picture_reconstruction::{
    reconstruct_one_macroblock, reconstruct_skipped_macroblock, InterDirection,
    PicturePredictionParams,
};
use crate::quant_matrix_extension::QuantiserMatrixState;
use crate::sequence_extension::{ChromaFormat, Mpeg2Sequence, Mpeg2SequenceExtension};
use crate::sequence_scalable_extension::{
    write_sequence_scalable_extension, ScalableMode, SequenceScalableExtension,
    SEQUENCE_SCALABLE_EXTENSION_ID,
};
use crate::slice_header::{SliceContext, SliceHeader};
use crate::slice_macroblock_walk::{
    reconstruct_slice_motion_vectors, walk_slice_at, SliceWalkContext,
};
use crate::stream_writer::{
    write_picture_coding_extension, write_picture_header, write_sequence_header,
    write_slice_header_in, PictureCodingExtensionParams, SequenceHeaderParams, SEQUENCE_END_CODE,
};
use crate::video_sequence::{
    apply_quant_matrix_extensions, display_indices_from_coded_pictures, DecodedFrame,
};
use crate::{Error, Result};

const START_CODE_PICTURE: u8 = 0x00;
const START_CODE_SEQUENCE_HEADER: u8 = 0xB3;
const START_CODE_EXTENSION: u8 = 0xB5;
const START_CODE_SEQUENCE_END: u8 = 0xB7;
const START_CODE_GOP: u8 = 0xB8;

// -------------------------------------------------------------------
// Start-code scanning
// -------------------------------------------------------------------

/// `(offset, start_code_value)` of every start code in `buf`.
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

/// A picture region: from its `picture_start_code` up to (and
/// including the 4 bytes of) the next picture / GOP / sequence /
/// sequence-end start code, so the slice walkers see their §5.2.3
/// terminator.
#[derive(Debug, Clone, Copy)]
struct PictureSpan {
    start: usize,
    /// End including the boundary start code's four bytes.
    end_with_terminator: usize,
}

/// The picture regions of an elementary stream, in coded order, plus
/// the byte range of every GOP header and the leading sequence layer.
#[derive(Debug, Clone)]
struct StreamLayout {
    pictures: Vec<PictureSpan>,
    /// `(picture_index_before_which_it_appears, start, end)` of every
    /// `group_of_pictures_header()`.
    gops: Vec<(usize, usize, usize)>,
    /// `(picture_index_before_which_it_appears, offset)` of every
    /// `sequence_header_code`.
    sequence_headers: Vec<(usize, usize)>,
}

fn layout(stream: &[u8]) -> StreamLayout {
    let codes = scan_start_codes(stream);
    let mut pictures = Vec::new();
    let mut gops = Vec::new();
    let mut sequence_headers = Vec::new();
    for (k, &(off, code)) in codes.iter().enumerate() {
        match code {
            START_CODE_PICTURE => {
                // Boundary: next non-slice, non-extension, non-user-data
                // start code (picture / GOP / sequence / end).
                let boundary = codes[k + 1..]
                    .iter()
                    .find(|&&(_, c)| {
                        matches!(
                            c,
                            START_CODE_PICTURE
                                | START_CODE_GOP
                                | START_CODE_SEQUENCE_HEADER
                                | START_CODE_SEQUENCE_END
                        )
                    })
                    .map(|&(o, _)| o);
                let end_with_terminator = match boundary {
                    Some(b) => (b + 4).min(stream.len()),
                    None => stream.len(),
                };
                pictures.push(PictureSpan {
                    start: off,
                    end_with_terminator,
                });
            }
            START_CODE_GOP => {
                let end = codes.get(k + 1).map(|&(o, _)| o).unwrap_or(stream.len());
                gops.push((pictures.len(), off, end));
            }
            START_CODE_SEQUENCE_HEADER => sequence_headers.push((pictures.len(), off)),
            _ => {}
        }
    }
    StreamLayout {
        pictures,
        gops,
        sequence_headers,
    }
}

// -------------------------------------------------------------------
// Block helpers
// -------------------------------------------------------------------

fn plane_of(frame: &FrameBuffer, component: ColourComponent) -> &crate::frame_assembly::Plane {
    match component {
        ColourComponent::Y => &frame.y,
        ColourComponent::Cb => &frame.cb,
        ColourComponent::Cr => &frame.cr,
    }
}

fn plane_of_mut(
    frame: &mut FrameBuffer,
    component: ColourComponent,
) -> &mut crate::frame_assembly::Plane {
    match component {
        ColourComponent::Y => &mut frame.y,
        ColourComponent::Cb => &mut frame.cb,
        ColourComponent::Cr => &mut frame.cr,
    }
}

/// Read the 8×8 samples of a placed block (field-DCT stride honoured).
fn read_block(frame: &FrameBuffer, placement: BlockPlacement) -> [[u8; 8]; 8] {
    let plane = plane_of(frame, placement.component);
    let stride = placement.row_stride();
    let mut out = [[0u8; 8]; 8];
    for r in 0..8 {
        for c in 0..8 {
            out[r][c] = plane
                .get(placement.base_x + c, placement.base_y + r * stride)
                .unwrap_or(0);
        }
    }
    out
}

/// Write the 8×8 samples of a placed block.
fn write_block(frame: &mut FrameBuffer, placement: BlockPlacement, samples: &[[u8; 8]; 8]) {
    let plane = plane_of_mut(frame, placement.component);
    let stride = placement.row_stride();
    for r in 0..8 {
        for c in 0..8 {
            plane.put_sample(
                placement.base_x + c,
                placement.base_y + r * stride,
                samples[r][c],
            );
        }
    }
}

/// `clamp(p + IDCT(F))` per sample, `F` already saturated / mismatch
/// controlled (`f_pel` is the §A IDCT output).
fn add_residual(prediction: &[[u8; 8]; 8], f_pel: &[[i16; 8]; 8]) -> [[u8; 8]; 8] {
    let mut out = [[0u8; 8]; 8];
    for r in 0..8 {
        for c in 0..8 {
            out[r][c] = (i32::from(prediction[r][c]) + i32::from(f_pel[r][c])).clamp(0, 255) as u8;
        }
    }
    out
}

fn is_zero(f: &[[i32; 8]; 8]) -> bool {
    f.iter().all(|row| row.iter().all(|&c| c == 0))
}

fn add_coeffs(a: &[[i32; 8]; 8], b: &[[i32; 8]; 8]) -> [[i32; 8]; 8] {
    let mut out = *a;
    for v in 0..8 {
        for u in 0..8 {
            out[v][u] += b[v][u];
        }
    }
    out
}

/// The weight matrix Table 7-5 selects for a block.
fn weight_for(
    matrices: &QuantiserMatrixState,
    coding: BlockCoding,
    component: ColourComponent,
) -> &[[u8; 8]; 8] {
    match (coding, component) {
        (BlockCoding::Intra, ColourComponent::Y) => &matrices.intra_luma,
        (BlockCoding::Intra, _) => &matrices.intra_chroma,
        (BlockCoding::NonIntra, ColourComponent::Y) => &matrices.non_intra_luma,
        (BlockCoding::NonIntra, _) => &matrices.non_intra_chroma,
    }
}

// -------------------------------------------------------------------
// The layer hook
// -------------------------------------------------------------------

/// What the combined loop tells the enhancement layer about the
/// picture it is about to process.
#[derive(Debug, Clone, Copy)]
struct PictureContext<'a> {
    /// Index in coded order.
    index: usize,
    header: &'a Mpeg2PictureHeader,
    ext: &'a PictureCodingExtension,
    /// The lower layer's per-picture geometry (extension flags folded
    /// in).
    geometry: IntraPictureParams,
}

/// The enhancement layer as the combined loop sees it — a parsed
/// bitstream (decoding) or an encoder producing one.
trait SnrLayer {
    /// The (first / repeated) sequence layer of the lower layer.
    fn begin_sequence(&mut self, seq: &Mpeg2Sequence, header_bytes: &[u8]) -> Result<()>;
    /// A lower-layer `group_of_pictures_header()` (raw bytes).
    fn gop_header(&mut self, bytes: &[u8]) -> Result<()>;
    /// A lower-layer picture is about to be processed.
    fn begin_picture(&mut self, ctx: &PictureContext<'_>) -> Result<()>;
    /// A lower-layer slice starts at macroblock row `mb_row`.
    fn begin_slice(&mut self, mb_row: u32) -> Result<()>;
    /// The `dct_type` the enhancement layer uses for a macroblock the
    /// lower layer left unsignalled.
    fn enhancement_field_dct(&mut self, mb_address: u32) -> bool;
    /// `F''enhance` for one block, given `F''lower` and the in-loop
    /// lower-only reconstruction of the block (`None` = all zero).
    fn refine_block(
        &mut self,
        mb_address: u32,
        block_index: usize,
        field_dct: bool,
        lower: &[[i32; 8]; 8],
        lower_only: &[[u8; 8]; 8],
    ) -> Result<Option<[[i32; 8]; 8]>>;
    /// Every block of the macroblock has been refined.
    fn end_macroblock(&mut self, mb_address: u32, field_dct: bool) -> Result<()>;
    /// The slice is complete.
    fn end_slice(&mut self) -> Result<()>;
    /// The picture is complete.
    fn end_picture(&mut self) -> Result<()>;
    /// The stream is complete.
    fn end_sequence(&mut self) -> Result<()>;
}

// -------------------------------------------------------------------
// The combined loop
// -------------------------------------------------------------------

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

fn parse_sequence(buf: &[u8]) -> Result<Mpeg2Sequence> {
    Mpeg2Sequence::from_buf(buf).map_err(|_| {
        Error::InvalidBitstream(
            "SNR scalability: the lower layer must be an ISO/IEC 13818-2 sequence (sequence_header + sequence_extension); an ISO/IEC 11172-2 lower layer is not composed",
        )
    })
}

/// Run the §7.8 combined decoding loop over the lower-layer `base`
/// stream with `layer` supplying (or producing) the enhancement
/// coefficients. Returns the combined frames in display order.
fn run_combined_loop(base: &[u8], layer: &mut dyn SnrLayer) -> Result<Vec<DecodedFrame>> {
    let lay = layout(base);
    let Some(&(_, first_seq)) = lay.sequence_headers.first() else {
        return Err(Error::InvalidBitstream(
            "SNR scalability: lower layer has no sequence_header_code",
        ));
    };
    let mut seq = parse_sequence(&base[first_seq..])?;
    let mut base_geometry = geometry_of(&seq);
    let mut matrices = initial_matrices(&seq);
    let first_seq_end = lay
        .pictures
        .first()
        .map(|p| p.start)
        .unwrap_or(base.len())
        .min(lay.gops.first().map(|g| g.1).unwrap_or(base.len()));
    layer.begin_sequence(&seq, &base[first_seq..first_seq_end])?;

    let mut output: Vec<DecodedFrame> = Vec::new();
    let mut held_anchor: Option<DecodedFrame> = None;
    let mut forward_anchor: Option<FrameBuffer> = None;
    let mut backward_anchor: Option<FrameBuffer> = None;

    let mut next_seq = 1usize; // index into lay.sequence_headers
    let mut next_gop = 0usize;

    for (index, span) in lay.pictures.iter().enumerate() {
        // Repeated sequence headers / GOP headers preceding this
        // picture, in stream order.
        while let Some(&(before, off)) = lay.sequence_headers.get(next_seq) {
            if before > index {
                break;
            }
            seq = parse_sequence(&base[off..])?;
            base_geometry = geometry_of(&seq);
            matrices = initial_matrices(&seq);
            let end = lay
                .gops
                .iter()
                .find(|g| g.0 == before && g.1 > off)
                .map(|g| g.1)
                .unwrap_or(span.start);
            layer.begin_sequence(&seq, &base[off..end])?;
            next_seq += 1;
        }
        while let Some(&(before, start, end)) = lay.gops.get(next_gop) {
            if before > index {
                break;
            }
            layer.gop_header(&base[start..end])?;
            next_gop += 1;
        }

        let region = &base[span.start..span.end_with_terminator];
        let (header, ext) = Mpeg2PictureHeader::parse_with_extension(region)?;
        if ext.picture_structure != PictureStructure::Frame {
            return Err(Error::InvalidBitstream(
                "SNR scalability: field pictures are not composed by this loop (frame pictures only)",
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
        layer.begin_picture(&PictureContext {
            index,
            header: &header,
            ext: &ext,
            geometry,
        })?;

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

        let frame = reconstruct_refined_picture(
            region, &header, &ext, geometry, references, &matrices, layer,
        )?;
        layer.end_picture()?;

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
    layer.end_sequence()?;
    Ok(output)
}

/// Reconstruct one lower-layer frame picture with the enhancement
/// layer's coefficients folded in per §7.8.3.
#[allow(clippy::too_many_arguments)]
fn reconstruct_refined_picture(
    region: &[u8],
    header: &Mpeg2PictureHeader,
    ext: &PictureCodingExtension,
    geometry: IntraPictureParams,
    references: ReferenceFrames<'_>,
    matrices: &QuantiserMatrixState,
    layer: &mut dyn SnrLayer,
) -> Result<FrameBuffer> {
    let mut frame = geometry.new_frame_buffer();
    let mb_width = geometry.mb_width() as u32;
    let mb_height = geometry.mb_height();
    let nblocks = block_count(geometry.chroma_format);
    let slice_ctx = SliceContext::non_scalable(geometry.height as u32);
    let intra_picture = header.picture_coding_type == PictureCodingType::Intra;
    let dc_mult = intra_dc_mult(ext.intra_dc_precision)?;
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
        let (f_fwd_h, f_fwd_v, f_bwd_h, f_bwd_v) = if intra_picture {
            (ext.f_code_fwd_horiz, ext.f_code_fwd_vert, 15, 15)
        } else {
            (
                ext.f_code_fwd_horiz,
                ext.f_code_fwd_vert,
                ext.f_code_bwd_horiz,
                ext.f_code_bwd_vert,
            )
        };
        let ctx = SliceWalkContext::first_slice_with_block_decoding(
            mb_width,
            mb_row,
            header.picture_coding_type,
            sh.quantiser_scale_code,
            PictureStructure::Frame,
            geometry.frame_pred_frame_dct,
            f_fwd_h,
            f_fwd_v,
            f_bwd_h,
            f_bwd_v,
            ext.concealment_motion_vectors,
            geometry.chroma_format,
            geometry.intra_vlc_format,
            geometry.alternate_scan,
            geometry.intra_dc_precision,
            geometry.q_scale_type,
        )
        .with_quantiser_matrices(*matrices);
        let walk = walk_slice_at(slice_buf, sh.body_bit_position, ctx)?;
        let motion = if intra_picture {
            None
        } else {
            Some(reconstruct_slice_motion_vectors(&walk, &ctx)?)
        };

        layer.begin_slice(mb_row)?;
        let mut previous_inter_direction: Option<InterDirection> = None;
        for (r, record) in walk.macroblocks.iter().enumerate() {
            let motion_record = motion.as_ref().map(|m| &m.records[r]);

            // §7.6.6 skipped macroblocks preceding this coded one:
            // prediction only from the lower layer, F''lower = 0.
            if let Some(mr) = motion_record {
                for k in 0..mr.skipped_before {
                    let address = record.macroblock_address - mr.skipped_before + k;
                    placed += reconstruct_skipped_macroblock(
                        &mut frame,
                        references,
                        address as usize,
                        mb_width as usize,
                        header.picture_coding_type,
                        previous_inter_direction,
                        &mr.pmv_before,
                    )?;
                    let field_dct = layer.enhancement_field_dct(address);
                    refine_macroblock_in_place(
                        &mut frame,
                        address,
                        mb_width as usize,
                        geometry.chroma_format,
                        field_dct,
                        nblocks,
                        |_, _| None,
                        layer,
                    )?;
                    layer.end_macroblock(address, field_dct)?;
                }
            }

            let address = record.macroblock_address;
            let mb_col = address as usize % mb_width as usize;
            let mb_row_of = address as usize / mb_width as usize;
            let qs = quantiser_scale(record.quantiser_scale_code, geometry.q_scale_type)?;
            let coding = if record.macroblock_type.macroblock_intra {
                BlockCoding::Intra
            } else {
                BlockCoding::NonIntra
            };
            // F''lower per block (pre-saturation arithmetic, §7.8.3).
            let lower_of = |i: usize, component: ColourComponent| -> Option<[[i32; 8]; 8]> {
                record.decoded_blocks.as_ref().and_then(|blocks| {
                    blocks
                        .iter()
                        .find(|b| usize::from(b.block_index) == i)
                        .map(|b| {
                            inverse_quantise_arithmetic(
                                &b.decoded.qf,
                                coding,
                                weight_for(matrices, coding, component),
                                qs,
                                dc_mult,
                            )
                        })
                })
            };

            if record.macroblock_type.macroblock_intra {
                // §7.6.8: no prediction; combined = sat(IDCT(sat_mm(F''lower + F''enh))).
                let field_dct = record.dct_type == Some(true);
                for i in 0..nblocks {
                    let component = block_component(i, geometry.chroma_format)
                        .ok_or(Error::InvalidBitstream("SNR: bad block index"))?;
                    let placement =
                        block_placement(i, geometry.chroma_format, mb_col, mb_row_of, field_dct)
                            .ok_or(Error::InvalidBitstream("SNR: bad block placement"))?;
                    let lower = lower_of(i, component).unwrap_or([[0; 8]; 8]);
                    let lower_only = samples_of(&[[0u8; 8]; 8], &lower, true);
                    let enh = layer.refine_block(address, i, field_dct, &lower, &lower_only)?;
                    let combined = match enh {
                        Some(e) => samples_of(&[[0u8; 8]; 8], &add_coeffs(&lower, &e), true),
                        None => lower_only,
                    };
                    write_block(&mut frame, placement, &combined);
                }
                placed += 1;
                layer.end_macroblock(address, field_dct)?;
                continue;
            }

            // Inter: prediction-only pass (a record without residual
            // blocks writes exactly p), then refine in place.
            let mr = motion_record.ok_or(Error::InvalidBitstream(
                "SNR: inter macroblock in an I picture",
            ))?;
            let mut prediction_only = record.clone();
            prediction_only.decoded_blocks = None;
            placed += reconstruct_one_macroblock(
                &mut frame,
                references,
                &prediction_only,
                &mr.reconstructed,
                mb_width as usize,
                geometry.chroma_format,
                params.top_field_first,
                &mut previous_inter_direction,
            )?;
            let field_dct = match record.dct_type {
                Some(d) => d,
                None => layer.enhancement_field_dct(address),
            };
            refine_macroblock_in_place(
                &mut frame,
                address,
                mb_width as usize,
                geometry.chroma_format,
                field_dct,
                nblocks,
                lower_of,
                layer,
            )?;
            layer.end_macroblock(address, field_dct)?;
        }
        layer.end_slice()?;
    }
    if placed != mb_width as usize * mb_height {
        return Err(Error::InvalidBitstream(
            "§6.1.2.2: the picture's slices do not enclose every macroblock exactly once (restricted slice structure, Table 8-5)",
        ));
    }
    Ok(frame)
}

/// `clamp(p + IDCT(sat_mm(F'')))` — for `intra` blocks the IDCT output
/// is placed directly (no prediction, `p` ignored).
fn samples_of(
    prediction: &[[u8; 8]; 8],
    f_double_prime: &[[i32; 8]; 8],
    intra: bool,
) -> [[u8; 8]; 8] {
    let f = saturate_and_mismatch(f_double_prime);
    let f_pel = idct_8x8_from_i32(&f);
    if intra {
        let mut out = [[0u8; 8]; 8];
        for r in 0..8 {
            for c in 0..8 {
                out[r][c] = i32::from(f_pel[r][c]).clamp(0, 255) as u8;
            }
        }
        out
    } else {
        add_residual(prediction, &f_pel)
    }
}

/// With the prediction `p` of an inter macroblock already in `frame`,
/// fold `F''lower + F''enhance` into every block.
#[allow(clippy::too_many_arguments)]
fn refine_macroblock_in_place(
    frame: &mut FrameBuffer,
    address: u32,
    mb_width: usize,
    chroma_format: ChromaFormat,
    field_dct: bool,
    nblocks: usize,
    lower_of: impl Fn(usize, ColourComponent) -> Option<[[i32; 8]; 8]>,
    layer: &mut dyn SnrLayer,
) -> Result<()> {
    let mb_col = address as usize % mb_width;
    let mb_row = address as usize / mb_width;
    for i in 0..nblocks {
        let component = block_component(i, chroma_format)
            .ok_or(Error::InvalidBitstream("SNR: bad block index"))?;
        let placement = block_placement(i, chroma_format, mb_col, mb_row, field_dct)
            .ok_or(Error::InvalidBitstream("SNR: bad block placement"))?;
        let prediction = read_block(frame, placement);
        let lower = lower_of(i, component).unwrap_or([[0; 8]; 8]);
        let lower_only = if is_zero(&lower) {
            prediction
        } else {
            samples_of(&prediction, &lower, false)
        };
        let enh = layer.refine_block(address, i, field_dct, &lower, &lower_only)?;
        let combined = match enh {
            Some(e) => samples_of(&prediction, &add_coeffs(&lower, &e), false),
            None => lower_only,
        };
        write_block(frame, placement, &combined);
    }
    Ok(())
}

// -------------------------------------------------------------------
// Decoding: the enhancement layer as a parsed bitstream
// -------------------------------------------------------------------

/// One parsed enhancement-layer macroblock: `F''enhance` per block.
#[derive(Debug, Clone)]
struct EnhancementMacroblock {
    field_dct: bool,
    blocks: Vec<Option<[[i32; 8]; 8]>>,
}

struct EnhancementPicture {
    header: Mpeg2PictureHeader,
    ext: PictureCodingExtension,
    /// Indexed by macroblock address.
    macroblocks: Vec<Option<EnhancementMacroblock>>,
}

struct DecodingLayer<'e> {
    enhancement: &'e [u8],
    layout: StreamLayout,
    seq: Option<Mpeg2Sequence>,
    matrices: QuantiserMatrixState,
    next_seq: usize,
    current: Option<EnhancementPicture>,
}

impl<'e> DecodingLayer<'e> {
    fn new(enhancement: &'e [u8]) -> Self {
        Self {
            enhancement,
            layout: layout(enhancement),
            seq: None,
            matrices: QuantiserMatrixState::default(),
            next_seq: 0,
            current: None,
        }
    }

    /// Validate the enhancement sequence layer against the lower one
    /// (§7.8.1) and pick up its scalable extension.
    fn check_sequence(&mut self, lower: &Mpeg2Sequence, offset: usize) -> Result<()> {
        let buf = &self.enhancement[offset..];
        let seq = Mpeg2Sequence::from_buf(buf).map_err(|_| {
            Error::InvalidBitstream(
                "SNR scalability: enhancement layer sequence_header / sequence_extension missing or malformed",
            )
        })?;
        if seq.horizontal_size != lower.horizontal_size || seq.vertical_size != lower.vertical_size
        {
            return Err(Error::InvalidBitstream(
                "SNR scalability: enhancement layer geometry differs from the lower layer (§7.8.1)",
            ));
        }
        if seq.extension.chroma_format != lower.extension.chroma_format {
            return Err(Error::InvalidBitstream(
                "SNR scalability: chroma_simulcast (Table 7-26 differing chroma_format) is not composed",
            ));
        }
        if seq.extension.progressive_sequence != lower.extension.progressive_sequence {
            return Err(Error::InvalidBitstream(
                "SNR scalability: enhancement sequence_extension shall match the lower layer's progressive_sequence (§7.8.1)",
            ));
        }
        if seq.header.intra_quant.is_some() {
            return Err(Error::InvalidBitstream(
                "SNR scalability: load_intra_quantiser_matrix shall be zero in the enhancement layer (§7.8.1)",
            ));
        }
        // The sequence_scalable_extension() follows the sequence
        // extension: scalable_mode = SNR, layer_id = 1.
        let codes = scan_start_codes(buf);
        let mut declared = false;
        for &(off, code) in &codes {
            if code == START_CODE_PICTURE || code == START_CODE_GOP {
                break;
            }
            if code == START_CODE_EXTENSION
                && buf.get(off + 4).map(|b| b >> 4) == Some(SEQUENCE_SCALABLE_EXTENSION_ID as u8)
            {
                let sse = SequenceScalableExtension::parse(&buf[off..])?;
                if sse.scalable_mode != ScalableMode::SnrScalability || sse.layer_id != 1 {
                    return Err(Error::InvalidBitstream(
                        "SNR scalability: enhancement layer shall declare scalable_mode = SNR scalability, layer_id = 1 (§7.8.1)",
                    ));
                }
                declared = true;
            }
        }
        if !declared {
            return Err(Error::InvalidBitstream(
                "SNR scalability: enhancement layer carries no sequence_scalable_extension()",
            ));
        }
        // §7.8.1: only the non-intra matrices are used; the enhancement
        // sequence header may load its own non-intra matrix.
        self.matrices = initial_matrices(&seq);
        self.seq = Some(seq);
        Ok(())
    }

    /// Parse enhancement picture `index` into the per-macroblock table.
    fn parse_picture(&mut self, index: usize, ctx: &PictureContext<'_>) -> Result<()> {
        let span = *self
            .layout
            .pictures
            .get(index)
            .ok_or(Error::InvalidBitstream(
                "SNR scalability: the enhancement layer has fewer pictures than the lower layer",
            ))?;
        let region = &self.enhancement[span.start..span.end_with_terminator];
        let (header, ext) = Mpeg2PictureHeader::parse_with_extension(region)?;
        if header.picture_coding_type != ctx.header.picture_coding_type
            || header.temporal_reference != ctx.header.temporal_reference
        {
            return Err(Error::InvalidBitstream(
                "SNR scalability: enhancement picture header disagrees with the lower layer's (§7.8.1)",
            ));
        }
        if ext.picture_structure != PictureStructure::Frame
            || ext.frame_pred_frame_dct != ctx.ext.frame_pred_frame_dct
        {
            return Err(Error::InvalidBitstream(
                "SNR scalability: enhancement picture_coding_extension disagrees with the lower layer's (§7.8.1)",
            ));
        }
        let chroma_format = ctx.geometry.chroma_format;
        apply_quant_matrix_extensions(region, chroma_format, &mut self.matrices)?;

        let mb_width = ctx.geometry.mb_width();
        let mb_height = ctx.geometry.mb_height();
        let nblocks = block_count(chroma_format);
        let mut macroblocks: Vec<Option<EnhancementMacroblock>> = vec![None; mb_width * mb_height];
        let slice_ctx = SliceContext::non_scalable(ctx.geometry.height as u32);

        let codes = scan_start_codes(region);
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
            let mb_row = sh.mb_row() as usize;
            if mb_row >= mb_height {
                return Err(Error::InvalidBitstream(
                    "SNR scalability: enhancement slice_vertical_position beyond the picture",
                ));
            }
            let mut br = BitReader::new(slice_buf);
            br.skip(sh.body_bit_position as u32)
                .map_err(|_| Error::ShortHeader)?;
            let mut quantiser_scale_code = sh.quantiser_scale_code;
            let mut previous_address: i64 = (mb_row * mb_width) as i64 - 1;
            let mut first = true;
            loop {
                // §6.2.4: the slice ends at the next start code (23 zeros).
                match br.peek_u32(23) {
                    Ok(0) => break,
                    Ok(_) => {}
                    Err(_) => {
                        let remaining = br.bits_remaining().min(22) as u32;
                        if remaining == 0 || matches!(br.peek_u32(remaining), Ok(0)) {
                            break;
                        }
                    }
                }
                let increment =
                    MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2())?;
                // §6.3.17.1: a slice's first increment positions the
                // macroblock within the row (no skips implied).
                let _ = first;
                first = false;
                let address = previous_address + i64::from(increment.value);
                if address < 0 || address as usize >= mb_width * mb_height {
                    return Err(Error::InvalidBitstream(
                        "SNR scalability: enhancement macroblock_address beyond the picture",
                    ));
                }
                previous_address = address;
                let address = address as usize;

                // Table B-8.
                let mt = MacroblockType::parse_with_table(
                    &mut br,
                    ctx.header.picture_coding_type,
                    MacroblockTypeTable::SnrScalable,
                )?;
                // §6.2.5.1: dct_type iff frame picture, frame_pred_frame_dct
                // = 0 and (intra || pattern) — never intra here.
                let field_dct = if !ctx.ext.frame_pred_frame_dct && mt.macroblock_pattern {
                    br.read_bit().map_err(|_| Error::ShortHeader)?
                } else {
                    false
                };
                if mt.macroblock_quant {
                    quantiser_scale_code = br.read_u32(5).map_err(|_| Error::ShortHeader)? as u8;
                    if quantiser_scale_code == 0 {
                        return Err(Error::InvalidBitstream(
                            "quantiser_scale_code: 0 is forbidden (§6.3.17.4)",
                        ));
                    }
                }
                let mut blocks: Vec<Option<[[i32; 8]; 8]>> = vec![None; nblocks];
                if mt.macroblock_pattern {
                    let cbp = CodedBlockPattern::parse(&mut br, chroma_format)?;
                    let pattern = cbp.pattern_code(false, true);
                    let qs = quantiser_scale(quantiser_scale_code, ext.q_scale_type)?;
                    let block_ctx = BlockContext {
                        intra_vlc_format: false,
                        alternate_scan: ext.alternate_scan,
                        intra_dc_precision: 0,
                        quantiser_scale_value: qs,
                    };
                    let mut dc = DcPredictors::new(0)?;
                    for (i, block) in blocks.iter_mut().enumerate() {
                        if !pattern[i] {
                            continue;
                        }
                        let component = block_component(i, chroma_format)
                            .ok_or(Error::InvalidBitstream("SNR: bad block index"))?;
                        let weight = weight_for(&self.matrices, BlockCoding::NonIntra, component);
                        // §7.8.3.1 / §7.8.3.3: decoded and inverse-quantised as
                        // a non-intra block; the arithmetic-only result is
                        // F''enhance (the saturation waits for the sum).
                        let decoded =
                            decode_block(&mut br, &block_ctx, &mut dc, component, false, weight)?;
                        let enh = inverse_quantise_arithmetic(
                            &decoded.qf,
                            BlockCoding::NonIntra,
                            weight,
                            qs,
                            1,
                        );
                        *block = Some(enh);
                    }
                }
                macroblocks[address] = Some(EnhancementMacroblock { field_dct, blocks });
            }
        }
        self.current = Some(EnhancementPicture {
            header,
            ext,
            macroblocks,
        });
        Ok(())
    }
}

impl SnrLayer for DecodingLayer<'_> {
    fn begin_sequence(&mut self, seq: &Mpeg2Sequence, _header_bytes: &[u8]) -> Result<()> {
        let Some(&(_, off)) = self.layout.sequence_headers.get(self.next_seq) else {
            return Err(Error::InvalidBitstream(
                "SNR scalability: the enhancement layer has fewer sequence headers than the lower layer",
            ));
        };
        self.next_seq += 1;
        self.check_sequence(seq, off)
    }

    fn gop_header(&mut self, _bytes: &[u8]) -> Result<()> {
        Ok(())
    }

    fn begin_picture(&mut self, ctx: &PictureContext<'_>) -> Result<()> {
        self.parse_picture(ctx.index, ctx)
    }

    fn begin_slice(&mut self, _mb_row: u32) -> Result<()> {
        Ok(())
    }

    fn enhancement_field_dct(&mut self, mb_address: u32) -> bool {
        self.current
            .as_ref()
            .and_then(|p| p.macroblocks.get(mb_address as usize))
            .and_then(|m| m.as_ref())
            .map(|m| m.field_dct)
            .unwrap_or(false)
    }

    fn refine_block(
        &mut self,
        mb_address: u32,
        block_index: usize,
        _field_dct: bool,
        _lower: &[[i32; 8]; 8],
        _lower_only: &[[u8; 8]; 8],
    ) -> Result<Option<[[i32; 8]; 8]>> {
        Ok(self
            .current
            .as_ref()
            .and_then(|p| p.macroblocks.get(mb_address as usize))
            .and_then(|m| m.as_ref())
            .and_then(|m| m.blocks.get(block_index).copied().flatten()))
    }

    fn end_macroblock(&mut self, _mb_address: u32, _field_dct: bool) -> Result<()> {
        Ok(())
    }

    fn end_slice(&mut self) -> Result<()> {
        Ok(())
    }

    fn end_picture(&mut self) -> Result<()> {
        let _ = self.current.take().map(|p| (p.header, p.ext));
        Ok(())
    }

    fn end_sequence(&mut self) -> Result<()> {
        Ok(())
    }
}

/// Decode a §7.8 SNR-scalable pair — the lower-layer stream `base`
/// (an ordinary ISO/IEC 13818-2 elementary stream of frame pictures)
/// and its `enhancement` layer — into the **combined** frames in
/// display order.
///
/// # Errors
/// [`Error::InvalidBitstream`] when the lower layer is not an ISO/IEC
/// 13818-2 frame-picture stream, when the enhancement layer's sequence
/// / picture headers disagree with the lower layer's (§7.8.1), when
/// the two chroma formats differ (`chroma_simulcast` is not composed),
/// or for any syntax error in either layer; [`Error::ShortHeader`] on
/// truncation.
pub fn decode_snr_scalable_sequence(base: &[u8], enhancement: &[u8]) -> Result<Vec<DecodedFrame>> {
    let mut layer = DecodingLayer::new(enhancement);
    run_combined_loop(base, &mut layer)
}

// -------------------------------------------------------------------
// Encoding: the enhancement layer as an encoder
// -------------------------------------------------------------------

/// The output of [`encode_snr_enhancement_layer`].
#[derive(Debug, Clone)]
pub struct SnrEncoded {
    /// The enhancement-layer elementary stream (sequence header +
    /// extension + `sequence_scalable_extension()`, coincident GOP /
    /// picture / slice layers, `sequence_end_code`).
    pub stream: Vec<u8>,
    /// The combined (lower + enhancement) reconstruction in display
    /// order — what [`decode_snr_scalable_sequence`] reproduces.
    pub recon: Vec<DecodedFrame>,
    /// Coded / not-coded enhancement macroblock counts.
    pub coded_macroblocks: usize,
    /// See `coded_macroblocks`.
    pub not_coded_macroblocks: usize,
}

struct EncodingLayer<'s> {
    sources: &'s [FrameBuffer],
    display_index: Vec<u64>,
    quantiser_scale_code: u8,
    out: BitWriter,
    matrices: QuantiserMatrixState,
    // Per-picture state.
    source: Option<&'s FrameBuffer>,
    frame_pred_frame_dct: bool,
    q_scale_type: bool,
    alternate_scan: bool,
    chroma_format: ChromaFormat,
    mb_width: usize,
    vertical_size: u32,
    // Per-macroblock accumulator.
    pending: Vec<Option<[[i32; 8]; 8]>>,
    coded_macroblocks: usize,
    not_coded_macroblocks: usize,
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

impl<'s> EncodingLayer<'s> {
    fn source_block(&self, placement: BlockPlacement) -> Result<[[u8; 8]; 8]> {
        let source = self.source.ok_or(Error::InvalidBitstream(
            "SNR encoder: no source frame for this picture",
        ))?;
        Ok(read_block(source, placement))
    }
}

impl SnrLayer for EncodingLayer<'_> {
    fn begin_sequence(&mut self, seq: &Mpeg2Sequence, _header_bytes: &[u8]) -> Result<()> {
        // §7.8.1: identical to the lower layer's except bit_rate /
        // vbv_buffer_size / the matrix loads; load_intra shall be 0.
        write_sequence_header(
            &mut self.out,
            &SequenceHeaderParams {
                horizontal_size: seq.header.width,
                vertical_size: seq.header.height,
                aspect_ratio_code: aspect_ratio_code(seq.header.aspect_ratio),
                frame_rate_code: seq.header.frame_rate_code,
                bit_rate_value: seq.header.bit_rate,
                vbv_buffer_size_value: seq.header.vbv_buffer_size,
                intra_quantiser_matrix: None,
                non_intra_quantiser_matrix: None,
            },
        );
        write_sequence_extension_fields(&mut self.out, &seq.extension);
        write_sequence_scalable_extension(
            &mut self.out,
            &SequenceScalableExtension {
                scalable_mode: ScalableMode::SnrScalability,
                layer_id: 1,
            },
        );
        self.chroma_format = seq.extension.chroma_format;
        self.vertical_size = u32::from(seq.vertical_size);
        self.matrices = QuantiserMatrixState::default();
        Ok(())
    }

    fn gop_header(&mut self, bytes: &[u8]) -> Result<()> {
        // §7.8.1: identical to the lower layer's.
        self.out.write_bytes(bytes);
        Ok(())
    }

    fn begin_picture(&mut self, ctx: &PictureContext<'_>) -> Result<()> {
        let display = *self
            .display_index
            .get(ctx.index)
            .ok_or(Error::InvalidBitstream(
                "SNR encoder: picture index beyond the pre-scan",
            ))?;
        let source = self
            .sources
            .get(display as usize)
            .ok_or(Error::InvalidBitstream(
                "SNR encoder: fewer source frames than lower-layer pictures",
            ))?;
        if source.width != ctx.geometry.width
            || source.height != ctx.geometry.height
            || source.chroma_format != ctx.geometry.chroma_format
        {
            return Err(Error::InvalidBitstream(
                "SNR encoder: source frame geometry / chroma format does not match the lower layer",
            ));
        }
        self.source = Some(source);
        self.frame_pred_frame_dct = ctx.ext.frame_pred_frame_dct;
        self.q_scale_type = ctx.ext.q_scale_type;
        self.alternate_scan = ctx.ext.alternate_scan;
        self.mb_width = ctx.geometry.mb_width();

        // §7.8.1: picture header identical except vbv_delay; the
        // picture coding extension identical except q_scale_type /
        // alternate_scan (kept equal).
        let ext = ctx.ext;
        if ext.f_code_fwd_horiz != ext.f_code_fwd_vert
            || ext.f_code_bwd_horiz != ext.f_code_bwd_vert
        {
            return Err(Error::InvalidBitstream(
                "SNR encoder: lower-layer pictures with differing horizontal / vertical f_codes are not mirrored",
            ));
        }
        write_picture_header(
            &mut self.out,
            ctx.header.temporal_reference,
            ctx.header.picture_coding_type,
            0b111,
            0b111,
        );
        write_picture_coding_extension(
            &mut self.out,
            &PictureCodingExtensionParams {
                forward_f_code: ext.f_code_fwd_horiz,
                backward_f_code: ext.f_code_bwd_horiz,
                intra_dc_precision: ext.intra_dc_precision,
                frame_pred_frame_dct: ext.frame_pred_frame_dct,
                q_scale_type: ext.q_scale_type,
                intra_vlc_format: ext.intra_vlc_format,
                alternate_scan: ext.alternate_scan,
                progressive_frame: ext.progressive_frame,
                top_field_first: ext.top_field_first,
                repeat_first_field: ext.repeat_first_field,
                concealment_motion_vectors: ext.concealment_motion_vectors,
                chroma_format: ctx.geometry.chroma_format,
            },
        );
        Ok(())
    }

    fn begin_slice(&mut self, mb_row: u32) -> Result<()> {
        write_slice_header_in(
            &mut self.out,
            mb_row,
            self.quantiser_scale_code,
            self.vertical_size,
        );
        Ok(())
    }

    fn enhancement_field_dct(&mut self, _mb_address: u32) -> bool {
        false
    }

    fn refine_block(
        &mut self,
        mb_address: u32,
        block_index: usize,
        field_dct: bool,
        _lower: &[[i32; 8]; 8],
        lower_only: &[[u8; 8]; 8],
    ) -> Result<Option<[[i32; 8]; 8]>> {
        let nblocks = block_count(self.chroma_format);
        if self.pending.len() != nblocks {
            self.pending = vec![None; nblocks];
        }
        // §6.3.17.4 (printed derivation): 4:4:4 non-intra blocks 6 / 7
        // have no coded_block_pattern slot in the Table B-8 macroblock.
        if !nonintra_block_has_cbp_slot(block_index, self.chroma_format) {
            self.pending[block_index] = None;
            return Ok(None);
        }
        let mb_col = mb_address as usize % self.mb_width;
        let mb_row = mb_address as usize / self.mb_width;
        let placement = block_placement(block_index, self.chroma_format, mb_col, mb_row, field_dct)
            .ok_or(Error::InvalidBitstream("SNR encoder: bad block placement"))?;
        let component = block_component(block_index, self.chroma_format)
            .ok_or(Error::InvalidBitstream("SNR encoder: bad block index"))?;
        let source = self.source_block(placement)?;
        let mut residual = [[0i16; 8]; 8];
        for r in 0..8 {
            for c in 0..8 {
                residual[r][c] = i16::from(source[r][c]) - i16::from(lower_only[r][c]);
            }
        }
        let weight = *weight_for(&self.matrices, BlockCoding::NonIntra, component);
        let qs = quantiser_scale(self.quantiser_scale_code, self.q_scale_type)?;
        let f = fdct_8x8(&residual);
        let qf = forward_quantise_block(&f, BlockCoding::NonIntra, &weight, qs, 1);
        if is_zero(&qf) {
            self.pending[block_index] = None;
            return Ok(None);
        }
        self.pending[block_index] = Some(qf);
        Ok(Some(inverse_quantise_arithmetic(
            &qf,
            BlockCoding::NonIntra,
            &weight,
            qs,
            1,
        )))
    }

    fn end_macroblock(&mut self, _mb_address: u32, field_dct: bool) -> Result<()> {
        let nblocks = block_count(self.chroma_format);
        if self.pending.len() != nblocks {
            self.pending = vec![None; nblocks];
        }
        let coded_flags: Vec<bool> = self.pending.iter().map(|b| b.is_some()).collect();
        let coded = coded_flags.iter().any(|&f| f);
        // macroblock_address_increment = 1 (Table B-1).
        self.out.write_bit(true);
        if coded {
            self.coded_macroblocks += 1;
            self.out.write_bit(true); // Table B-8 "Coded"
            if !self.frame_pred_frame_dct {
                self.out.write_bit(field_dct); // dct_type (§6.2.5.1)
            }
            let mut flags = [false; 12];
            flags[..nblocks].copy_from_slice(&coded_flags);
            encode_coded_block_pattern(&mut self.out, &flags[..nblocks], self.chroma_format)?;
            for qf in self.pending.iter().flatten() {
                write_inter_block_coeffs(&mut self.out, qf, self.alternate_scan);
            }
        } else {
            self.not_coded_macroblocks += 1;
            self.out.write_u32(0b001, 3); // Table B-8 "Not Coded"
        }
        for slot in self.pending.iter_mut() {
            *slot = None;
        }
        Ok(())
    }

    fn end_slice(&mut self) -> Result<()> {
        self.out.align_to_byte_zero();
        Ok(())
    }

    fn end_picture(&mut self) -> Result<()> {
        self.source = None;
        Ok(())
    }

    fn end_sequence(&mut self) -> Result<()> {
        Ok(())
    }
}

/// Encode a §7.8 SNR **enhancement layer** for the lower-layer stream
/// `base` from the original `sources` (display order, one per
/// lower-layer picture, same geometry and chroma format) at the
/// enhancement `quantiser_scale_code` (`1..=31`, typically finer than
/// the lower layer's).
///
/// The layer refines every macroblock of every picture: the encoder
/// runs the §7.8 combined loop, forms each block's in-loop lower-only
/// reconstruction, DCT-codes the remaining error against the source
/// as a non-intra block and emits Table B-8 macroblocks in slices
/// coincident with the lower layer's (`Not Coded` where nothing
/// survives quantisation). The returned `recon` is the combined
/// reconstruction the decoder reproduces exactly.
///
/// # Errors
/// [`Error::InvalidBitstream`] for a lower layer this loop does not
/// compose (ISO/IEC 11172-2, field pictures), a source list that does
/// not match the lower layer, or an out-of-range quantiser.
pub fn encode_snr_enhancement_layer(
    base: &[u8],
    sources: &[FrameBuffer],
    quantiser_scale_code: u8,
) -> Result<SnrEncoded> {
    if !(1..=31).contains(&quantiser_scale_code) {
        return Err(Error::InvalidBitstream(
            "SNR encoder: quantiser_scale_code must be in 1..=31",
        ));
    }
    // Pre-scan the lower layer's pictures for their display indices.
    let lay = layout(base);
    let mut coded: Vec<(u16, PictureCodingType)> = Vec::with_capacity(lay.pictures.len());
    for span in &lay.pictures {
        let header = Mpeg2PictureHeader::parse(&base[span.start..span.end_with_terminator])?;
        coded.push((header.temporal_reference, header.picture_coding_type));
    }
    let display_index = display_indices_from_coded_pictures(&coded);
    if sources.len() < lay.pictures.len() {
        return Err(Error::InvalidBitstream(
            "SNR encoder: fewer source frames than lower-layer pictures",
        ));
    }

    let mut layer = EncodingLayer {
        sources,
        display_index,
        quantiser_scale_code,
        out: BitWriter::new(),
        matrices: QuantiserMatrixState::default(),
        source: None,
        frame_pred_frame_dct: true,
        q_scale_type: false,
        alternate_scan: false,
        chroma_format: ChromaFormat::Yuv420,
        mb_width: 0,
        vertical_size: 0,
        pending: Vec::new(),
        coded_macroblocks: 0,
        not_coded_macroblocks: 0,
    };
    let recon = run_combined_loop(base, &mut layer)?;
    let mut stream = layer.out.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
    Ok(SnrEncoded {
        stream,
        recon,
        coded_macroblocks: layer.coded_macroblocks,
        not_coded_macroblocks: layer.not_coded_macroblocks,
    })
}
