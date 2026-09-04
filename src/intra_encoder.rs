//! Intra (I) picture encoder — assembles a complete MPEG-2 elementary
//! stream for an all-intra frame picture from a [`FrameBuffer`].
//!
//! ## Pipeline (the inverse of the decoder's I-picture path)
//!
//! For each macroblock, for each of its §6.1.1.8 blocks:
//!
//! 1. **Gather** the 8×8 spatial samples from the component plane
//!    ([`crate::frame_assembly::block_placement`] resolves the block's
//!    top-left sample coordinate; frame-DCT organisation is used since
//!    the encoder writes `frame_pred_frame_dct = 1`).
//! 2. **Forward DCT** ([`crate::forward_dct::fdct_8x8`]) on the raw
//!    8-bit samples → `F[v][u]`. The MPEG-2 intra path does **not**
//!    level-shift before the DCT: the §7.4.1
//!    `F''[0][0] = intra_dc_mult * QFS[0]` reconstruction carries the
//!    absolute DC level and the §7.2.1 Table 7-2 predictor reset (128
//!    for `intra_dc_precision = 0`) seeds the DC chain.
//! 4. **Forward-quantise** ([`crate::forward_quant::forward_quantise_block`])
//!    → `QF[v][u]` levels.
//! 5. **Entropy-code**: the §7.2.1 DC differential
//!    ([`crate::mpeg2_block_dc::encode_intra_dc`]) against the running
//!    per-component DC predictor, then the §7.2.2 AC run-level VLCs in
//!    zig-zag scan order
//!    ([`crate::mpeg2_dct_coeff::encode_dct_coeff`]) terminated by
//!    `end_of_block`.
//!
//! The macroblock layer wraps each block run with
//! `macroblock_address_increment = 1` (Table B-1 `1`) and
//! `macroblock_type = Intra` (Table B-2 `1`). The slice / picture /
//! sequence headers come from [`crate::stream_writer`]. One slice per
//! macroblock row.
//!
//! ## Round-trip correctness
//!
//! The encoder is lossy (the quantiser discards high-frequency detail),
//! but the *bitstream* it produces is exactly what the decoder reads:
//! `tests/encode_intra_roundtrip.rs` encodes a frame, decodes the
//! resulting stream with [`crate::decode_video_sequence`], and checks
//! the reconstruction matches the encoder's own decode-side
//! reconstruction sample-for-sample (the encoder and decoder share the
//! same inverse-quantise + IDCT path, so the match is exact, not merely
//! approximate).

use oxideav_core::bits::BitWriter;

use crate::encode_options::FrameEncodeOptions;
use crate::forward_dct::fdct_8x8;
use crate::forward_quant::forward_quantise_block;
use crate::frame_assembly::{block_placement, FrameBuffer, IntraPictureParams};
use crate::mpeg2_block_dc::{encode_intra_dc, ColourComponent, DcComponent};
use crate::mpeg2_dct_coeff::{
    encode_dct_coeff, encode_end_of_block, CoefficientPosition, TableSelection,
};
use crate::mpeg2_dequantize::{
    intra_dc_mult, quantiser_scale, select_weighting_matrix_index, BlockCoding, Component,
};
use crate::mpeg2_inverse_scan::inverse_scan_table;
use crate::mpeg2_macroblock_blocks::{block_component, block_count};
use crate::picture_header::PictureCodingType;
use crate::quant_matrix_extension::{write_quant_matrix_extension, QuantMatrixExtension};
use crate::stream_writer::{
    write_picture_coding_extension, write_picture_header, write_sequence_extension,
    write_sequence_header, write_slice_header_in, SequenceHeaderParams, SEQUENCE_END_CODE,
};
use crate::{Error, Result};

/// Per-component running DC predictor for the §7.2.1 intra DC chain.
struct DcPredictorState {
    luma: i32,
    cb: i32,
    cr: i32,
}

impl DcPredictorState {
    /// Reset to the §7.2.1 Table 7-2 value for the `intra_dc_precision`.
    fn reset(intra_dc_precision: u8) -> Self {
        // Table 7-2: reset = 2^(8 + intra_dc_precision - 1) ... but the
        // listed values are {128, 256, 512, 1024} for precision 0..=3.
        let v = 128i32 << intra_dc_precision;
        Self {
            luma: v,
            cb: v,
            cr: v,
        }
    }

    fn get(&self, c: ColourComponent) -> i32 {
        match c {
            ColourComponent::Y => self.luma,
            ColourComponent::Cb => self.cb,
            ColourComponent::Cr => self.cr,
        }
    }

    fn set(&mut self, c: ColourComponent, value: i32) {
        match c {
            ColourComponent::Y => self.luma = value,
            ColourComponent::Cb => self.cb = value,
            ColourComponent::Cr => self.cr = value,
        }
    }
}

/// Map a [`ColourComponent`] to its DC VLC table component.
fn dc_table_component(c: ColourComponent) -> DcComponent {
    match c {
        ColourComponent::Y => DcComponent::Luminance,
        ColourComponent::Cb | ColourComponent::Cr => DcComponent::Chrominance,
    }
}

/// Gather an 8×8 frame-DCT block of samples from the plane for the
/// component, returning the raw `[y][x]` `u8` samples. Out-of-plane
/// reads (edge macroblocks padded to the 16-grid) repeat the last valid
/// sample by clamping coordinates.
fn gather_block(
    frame: &FrameBuffer,
    c: ColourComponent,
    base_x: usize,
    base_y: usize,
) -> [[i16; 8]; 8] {
    let plane = match c {
        ColourComponent::Y => &frame.y,
        ColourComponent::Cb => &frame.cb,
        ColourComponent::Cr => &frame.cr,
    };
    let w = plane.width();
    let h = plane.height();
    let mut out = [[0i16; 8]; 8];
    for (ry, row) in out.iter_mut().enumerate() {
        let sy = (base_y + ry).min(h.saturating_sub(1));
        for (rx, cell) in row.iter_mut().enumerate() {
            let sx = (base_x + rx).min(w.saturating_sub(1));
            *cell = i16::from(plane.get(sx, sy).unwrap_or(128));
        }
    }
    out
}

/// Encode a single intra block: gather → FDCT → quantise →
/// entropy-code into `bw`. Updates `pred` with the new DC value.
#[allow(clippy::too_many_arguments)]
fn encode_intra_block(
    bw: &mut BitWriter,
    frame: &FrameBuffer,
    c: ColourComponent,
    base_x: usize,
    base_y: usize,
    quantiser_scale_value: u8,
    intra_dc_mult_value: i32,
    pred: &mut DcPredictorState,
    table: TableSelection,
    alternate_scan: bool,
    weight: &[[u8; 8]; 8],
) {
    // 1. Gather the raw 8-bit samples. Unlike many codecs, the MPEG-2
    // intra path does NOT level-shift before the DCT: the §7.4.1
    // `F''[0][0] = intra_dc_mult * QFS[0]` reconstruction carries the
    // absolute DC level, and the §7.2.1 Table 7-2 predictor reset (128
    // for intra_dc_precision 0) seeds QFS[0] so a flat-128 block maps to
    // QFS[0] = 128 directly. The encoder therefore forward-transforms
    // the samples as-is.
    let raw = gather_block(frame, c, base_x, base_y);

    // 2. Forward DCT.
    let f = fdct_8x8(&raw);

    // 3. Forward-quantise (intra) against the active §7.4.2.1
    // weighting matrix (w = 0 luminance / w = 2 chrominance at
    // 4:2:2 / 4:4:4 — the caller resolves Table 7-5).
    let qf = forward_quantise_block(
        &f,
        BlockCoding::Intra,
        weight,
        quantiser_scale_value,
        intra_dc_mult_value,
    );

    // 4a. DC differential (§7.2.1): QFS[0] = qf[0][0]; transmit
    // QFS[0] - predictor; update predictor.
    let qfs_zero = qf[0][0];
    let dct_diff = qfs_zero - pred.get(c);
    encode_intra_dc(bw, dc_table_component(c), dct_diff);
    pred.set(c, qfs_zero);

    // 4b. AC coefficients (§7.2.2) in the §7.3 scan the picture
    // declares. Intra blocks use CoefficientPosition::Next throughout
    // (the DC field stood in for the first coefficient); `table` is
    // the §7.2.2.1 Table 7-3 selection (Table B-15 when the picture
    // declares intra_vlc_format = 1).
    let scan = inverse_scan_table(alternate_scan);
    // Collect non-zero AC levels in scan order, tracking the run of
    // zeros preceding each.
    let mut run = 0u8;
    for &(v, u) in scan.iter().skip(1) {
        let level = qf[v as usize][u as usize];
        if level == 0 {
            run += 1;
            continue;
        }
        let signed = level.clamp(-2047, 2047) as i16;
        encode_dct_coeff(bw, table, CoefficientPosition::Next, run, signed);
        run = 0;
    }
    // 4c. end_of_block terminates every coded block.
    encode_end_of_block(bw, table);
}

/// Encode an all-intra frame picture from `frame` into a complete
/// MPEG-2 elementary stream (leading `sequence_header` through the
/// trailing `sequence_end_code`).
///
/// `params` carries the geometry + the picture-coding-extension flags
/// the decoder needs to read the stream back, and every flag is
/// honoured on the wire: `q_scale_type` (Table 7-6 non-linear scale),
/// `intra_dc_precision` (Table 6-13, 8..=11-bit DC),
/// `intra_vlc_format` (Table 7-3 → Table B-15 intra AC), and
/// `alternate_scan` (§7.3 Figure 7-2 scan). The encoder writes a frame
/// picture with `frame_pred_frame_dct = 1` and default quantiser
/// matrices. `quantiser_scale_code` (`1..=31`) is the per-slice
/// quantiser scale.
///
/// # Errors
/// * [`Error::InvalidBitstream`] if `quantiser_scale_code` is out of
///   range or `intra_dc_precision` is invalid.
pub fn encode_intra_picture(
    frame: &FrameBuffer,
    params: IntraPictureParams,
    temporal_reference: u16,
    quantiser_scale_code: u8,
) -> Result<Vec<u8>> {
    encode_intra_picture_with_matrices(
        frame,
        params,
        temporal_reference,
        quantiser_scale_code,
        &QuantMatrixExtension::default(),
    )
}

/// [`encode_intra_picture`] with §6.3.11 **downloadable quantiser
/// matrices**: the `intra` / `non_intra` payloads are emitted through
/// the sequence header's `load_*_quantiser_matrix` slots (replacing
/// the luminance matrix and — at 4:2:2 / 4:4:4 — the chrominance one
/// too), while the `chroma_intra` / `chroma_non_intra` payloads travel
/// in a `quant_matrix_extension()` written between the
/// `picture_coding_extension()` and the first slice (they have no
/// sequence-header slot). The forward quantiser uses exactly the
/// Table 7-5 matrix bank the decoder resolves from those downloads.
///
/// # Errors
/// [`Error::InvalidBitstream`] on out-of-range parameters or a payload
/// violating §6.3.11 (zero byte, intra first value != 8, chroma
/// downloads at 4:2:0).
pub fn encode_intra_picture_with_matrices(
    frame: &FrameBuffer,
    params: IntraPictureParams,
    temporal_reference: u16,
    quantiser_scale_code: u8,
    matrices: &QuantMatrixExtension,
) -> Result<Vec<u8>> {
    encode_intra_picture_with_options(
        frame,
        params,
        temporal_reference,
        quantiser_scale_code,
        matrices,
        FrameEncodeOptions::default(),
        15,
        None,
    )
}

/// [`encode_intra_picture_with_matrices`] with the optional
/// [`FrameEncodeOptions`] behaviours. I-pictures never skip
/// macroblocks (§7.6.6), so only the §6.3.10 output-cadence flags and
/// **concealment motion vectors** (§7.6.3.9) apply: with
/// `concealment_motion_vectors` set every macroblock carries a
/// frame-format forward vector coded against `PMV[0][0]` with
/// `f_code[0][*] = forward_f_code` (which must then be `1..=9`) plus
/// a `marker_bit`. The vector is the one best suited to the
/// macroblock below, searched against `concealment_reference` (the
/// previous anchor a decoder would conceal from); with no reference
/// every vector is `(0, 0)`. Without the flag `forward_f_code` is
/// ignored and `f_code[0][*]` is written as `15` (unused).
///
/// # Errors
/// [`Error::InvalidBitstream`] on out-of-range parameters, a §6.3.11
/// matrix-payload violation, a §6.3.10 flag violation, or a
/// concealment `forward_f_code` outside `1..=9`.
#[allow(clippy::too_many_arguments)]
pub fn encode_intra_picture_with_options(
    frame: &FrameBuffer,
    params: IntraPictureParams,
    temporal_reference: u16,
    quantiser_scale_code: u8,
    matrices: &QuantMatrixExtension,
    options: FrameEncodeOptions,
    forward_f_code: u8,
    concealment_reference: Option<&FrameBuffer>,
) -> Result<Vec<u8>> {
    if options.concealment_motion_vectors && !(1..=9).contains(&forward_f_code) {
        return Err(Error::InvalidBitstream(
            "encode_intra_picture: concealment motion vectors need f_code[0][*] in 1..=9",
        ));
    }
    if let Some(reference) = concealment_reference {
        if reference.width != params.width || reference.height != params.height {
            return Err(Error::InvalidBitstream(
                "encode_intra_picture: concealment reference dimensions do not match params",
            ));
        }
    }
    if frame.width != params.width || frame.height != params.height {
        return Err(Error::InvalidBitstream(
            "encode_intra_picture: frame dimensions do not match params",
        ));
    }
    let q_value = quantiser_scale(quantiser_scale_code, params.q_scale_type)?;
    let dc_mult = intra_dc_mult(params.intra_dc_precision)?;
    matrices.validate_for_emission(params.chroma_format)?;
    // The Table 7-5 matrix bank the decoder will hold after the
    // downloads below; the forward quantiser must use the same one.
    let matrix_state = matrices.resolved_state(params.chroma_format);
    // §7.4.2.1 selector sanity: luminance intra is always w == 0.
    debug_assert_eq!(
        select_weighting_matrix_index(
            BlockCoding::Intra,
            Component::Luminance,
            params.chroma_format
        ),
        0
    );

    let mut bw = BitWriter::new();

    // ----- Sequence layer -----
    write_sequence_header(
        &mut bw,
        &SequenceHeaderParams {
            horizontal_size: params.width as u16,
            vertical_size: params.height as u16,
            intra_quantiser_matrix: matrices.intra,
            non_intra_quantiser_matrix: matrices.non_intra,
            ..Default::default()
        },
    );
    // §6.3.5/§6.3.3: this encoder codes progressive frame pictures on
    // the Ceil(h/16) macroblock grid, so it declares progressive_sequence.
    write_sequence_extension(&mut bw, params.chroma_format, params.progressive_sequence);

    // ----- Picture layer -----
    write_picture_header(
        &mut bw,
        temporal_reference,
        PictureCodingType::Intra,
        15,
        15,
    );
    // §6.3.11: f_code[0][*] is unused (15) in an I-picture unless
    // concealment motion vectors are coded.
    let f_code = if options.concealment_motion_vectors {
        forward_f_code
    } else {
        15
    };
    let pce = options.picture_coding_extension(&params, f_code, 15)?;
    write_picture_coding_extension(&mut bw, &pce);
    let search_range = crate::motion_estimation::max_search_range(f_code).min(16);
    // §6.2.3.2: chroma-specific matrix downloads have no sequence-
    // header slot — they travel in a quant_matrix_extension() between
    // the picture_coding_extension() and the first slice. The luma
    // slots already rode the sequence header, so only the chroma pair
    // is re-declared here.
    if matrices.has_chroma_downloads() {
        write_quant_matrix_extension(
            &mut bw,
            &QuantMatrixExtension {
                intra: None,
                non_intra: None,
                chroma_intra: matrices.chroma_intra,
                chroma_non_intra: matrices.chroma_non_intra,
            },
        );
    }

    let mb_width = params.mb_width();
    let mb_height = params.mb_height();
    let nblocks = block_count(params.chroma_format);

    // ----- Slice layer: one slice per macroblock row -----
    for mb_row in 0..mb_height {
        write_slice_header_in(
            &mut bw,
            mb_row as u32,
            quantiser_scale_code,
            params.height as u32,
        );
        // §7.2.1: the DC predictor resets at the start of every slice.
        let mut pred = DcPredictorState::reset(params.intra_dc_precision);
        // §7.6.3.4: PMV[0][0] resets at slice start (only consumed by
        // the concealment vectors).
        let mut pmv = (0i32, 0i32);

        for mb_col in 0..mb_width {
            // macroblock_address_increment = 1 (Table B-1 `1`).
            bw.write_bit(true);
            // macroblock_type = Intra (Table B-2 `1`).
            bw.write_bit(true);
            // No quantiser_scale (macroblock_quant clear), no dct_type
            // (frame_pred_frame_dct = 1 → frame DCT default).
            if options.concealment_motion_vectors {
                // §7.6.3.9: motion_vectors(0) — a Frame-based forward
                // vector against PMV[0][0] (Table 7-9 `Frame-based‡
                // intra` row writes it back to both forward slots) —
                // then the marker_bit.
                let cmv = match concealment_reference {
                    Some(reference) => crate::p_picture_encoder::concealment_vector(
                        frame,
                        reference,
                        mb_col,
                        mb_row,
                        mb_height,
                        search_range,
                    ),
                    None => crate::inter_reconstruction::MotionVectorPel::new(0, 0),
                };
                let dx = crate::p_picture_encoder::wrap_delta(cmv.horizontal - pmv.0, f_code)?;
                let dy = crate::p_picture_encoder::wrap_delta(cmv.vertical - pmv.1, f_code)?;
                crate::motion_vector::encode_motion_component(&mut bw, dx, f_code);
                crate::motion_vector::encode_motion_component(&mut bw, dy, f_code);
                bw.write_bit(true); // marker_bit
                pmv = (cmv.horizontal, cmv.vertical);
            }

            for i in 0..nblocks {
                let placement = block_placement(i, params.chroma_format, mb_col, mb_row, false)
                    .ok_or(Error::InvalidBitstream(
                        "encode_intra_picture: invalid block index",
                    ))?;
                let component = block_component(i, params.chroma_format).ok_or(
                    Error::InvalidBitstream("encode_intra_picture: invalid block component"),
                )?;
                // Table 7-5: luminance intra → w 0; chrominance intra
                // shares it at 4:2:0 and uses w 2 at 4:2:2 / 4:4:4 —
                // resolved_state already collapsed the 4:2:0 case.
                let weight = match component {
                    ColourComponent::Y => &matrix_state.intra_luma,
                    ColourComponent::Cb | ColourComponent::Cr => &matrix_state.intra_chroma,
                };
                encode_intra_block(
                    &mut bw,
                    frame,
                    component,
                    placement.base_x,
                    placement.base_y,
                    q_value,
                    dc_mult,
                    &mut pred,
                    TableSelection::from_context(params.intra_vlc_format, true),
                    params.alternate_scan,
                    weight,
                );
            }
        }
        // §6.2.4: slices end on a byte boundary (the next start code is
        // byte-aligned). Pad with zeros to the byte.
        bw.align_to_byte_zero();
    }

    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
    Ok(stream)
}

/// [`encode_intra_picture`] with an arbitrary **slice structure**: every
/// slice holds `macroblocks_per_slice` consecutive macroblocks, cut at
/// row ends (§6.1.2: the first and last macroblock of a slice lie in
/// one row; slices may start anywhere in a row). A slice that starts
/// mid-row codes its first `macroblock_address_increment` as the
/// column plus one against the §6.3.17.1 reset
/// `previous_macroblock_address = mb_row * mb_width - 1`; the §7.2.1
/// DC predictors reset at every slice start. `macroblocks_per_slice =
/// mb_width` (or more) reproduces the one-slice-per-row layout.
///
/// # Errors
/// As [`encode_intra_picture`]; `macroblocks_per_slice = 0` is
/// rejected.
pub fn encode_intra_picture_with_slice_length(
    frame: &FrameBuffer,
    params: IntraPictureParams,
    temporal_reference: u16,
    quantiser_scale_code: u8,
    macroblocks_per_slice: usize,
) -> Result<Vec<u8>> {
    if macroblocks_per_slice == 0 {
        return Err(Error::InvalidBitstream(
            "encode_intra_picture_with_slice_length: macroblocks_per_slice must be >= 1",
        ));
    }
    if frame.width != params.width || frame.height != params.height {
        return Err(Error::InvalidBitstream(
            "encode_intra_picture: frame dimensions do not match params",
        ));
    }
    let q_value = quantiser_scale(quantiser_scale_code, params.q_scale_type)?;
    let dc_mult = intra_dc_mult(params.intra_dc_precision)?;
    let matrix_state = QuantMatrixExtension::default().resolved_state(params.chroma_format);

    let mut bw = BitWriter::new();
    write_sequence_header(
        &mut bw,
        &SequenceHeaderParams {
            horizontal_size: params.width as u16,
            vertical_size: params.height as u16,
            ..Default::default()
        },
    );
    write_sequence_extension(&mut bw, params.chroma_format, params.progressive_sequence);
    write_picture_header(
        &mut bw,
        temporal_reference,
        PictureCodingType::Intra,
        15,
        15,
    );
    let pce = FrameEncodeOptions::default().picture_coding_extension(&params, 15, 15)?;
    write_picture_coding_extension(&mut bw, &pce);

    let mb_width = params.mb_width();
    let mb_height = params.mb_height();
    let nblocks = block_count(params.chroma_format);
    for mb_row in 0..mb_height {
        let mut mb_col = 0usize;
        while mb_col < mb_width {
            let slice_len = macroblocks_per_slice.min(mb_width - mb_col);
            write_slice_header_in(
                &mut bw,
                mb_row as u32,
                quantiser_scale_code,
                params.height as u32,
            );
            let mut pred = DcPredictorState::reset(params.intra_dc_precision);
            for k in 0..slice_len {
                let col = mb_col + k;
                if k == 0 {
                    // First macroblock: increment = column + 1.
                    crate::mb_address_increment::encode_mb_address_increment(
                        &mut bw,
                        col as u32 + 1,
                    );
                } else {
                    bw.write_bit(true);
                }
                bw.write_bit(true); // Table B-2 Intra
                for i in 0..nblocks {
                    let placement = block_placement(i, params.chroma_format, col, mb_row, false)
                        .ok_or(Error::InvalidBitstream(
                            "encode_intra_picture: invalid block index",
                        ))?;
                    let component = block_component(i, params.chroma_format).ok_or(
                        Error::InvalidBitstream("encode_intra_picture: invalid block component"),
                    )?;
                    let weight = match component {
                        ColourComponent::Y => &matrix_state.intra_luma,
                        ColourComponent::Cb | ColourComponent::Cr => &matrix_state.intra_chroma,
                    };
                    encode_intra_block(
                        &mut bw,
                        frame,
                        component,
                        placement.base_x,
                        placement.base_y,
                        q_value,
                        dc_mult,
                        &mut pred,
                        TableSelection::from_context(params.intra_vlc_format, true),
                        params.alternate_scan,
                        weight,
                    );
                }
            }
            bw.align_to_byte_zero();
            mb_col += slice_len;
        }
    }

    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
    Ok(stream)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sequence_extension::ChromaFormat;

    fn flat_frame(width: usize, height: usize, value: u8) -> FrameBuffer {
        let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
        for y in 0..height {
            for x in 0..width {
                f.y.put_sample(x, y, value);
            }
        }
        for y in 0..f.cb.height() {
            for x in 0..f.cb.width() {
                f.cb.put_sample(x, y, 128);
                f.cr.put_sample(x, y, 128);
            }
        }
        f
    }

    fn params_for(width: usize, height: usize) -> IntraPictureParams {
        IntraPictureParams {
            // progressive sequence: Ceil(h/16) macroblock grid (§6.3.3)
            progressive_sequence: true,
            width,
            height,
            chroma_format: ChromaFormat::Yuv420,
            frame_pred_frame_dct: true,
            intra_dc_precision: 0,
            intra_vlc_format: false,
            alternate_scan: false,
            q_scale_type: false,
        }
    }

    #[test]
    fn encodes_a_decodable_stream() {
        let frame = flat_frame(16, 16, 128);
        let params = params_for(16, 16);
        let stream = encode_intra_picture(&frame, params, 0, 8).expect("encode");
        // The stream must begin with a sequence header and end with the
        // sequence end code.
        assert_eq!(&stream[0..4], &[0x00, 0x00, 0x01, 0xB3]);
        assert_eq!(&stream[stream.len() - 4..], &[0x00, 0x00, 0x01, 0xB7]);
    }

    #[test]
    fn flat_grey_frame_roundtrips_exactly() {
        // A flat 128 luma / 128 chroma frame quantises to all-zero AC +
        // DC differential 0, so it reconstructs to exactly 128.
        let frame = flat_frame(16, 16, 128);
        let params = params_for(16, 16);
        let stream = encode_intra_picture(&frame, params, 0, 8).expect("encode");
        let frames = crate::decode_video_sequence(&stream).expect("decode");
        assert_eq!(frames.len(), 1);
        let out = &frames[0].frame;
        for y in 0..16 {
            for x in 0..16 {
                assert_eq!(out.y.get(x, y), Some(128), "luma ({x},{y})");
            }
        }
    }
}
