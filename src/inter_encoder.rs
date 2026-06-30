//! Inter (P) picture encoder primitives — the non-intra residual block
//! encoder and a zero-motion-vector P-picture assembler.
//!
//! ## Non-intra residual block (§6.2.6 / §7.2.2)
//!
//! A non-intra block carries **no DC prelude**: every one of the 64
//! coefficients is coded by the §7.2.2 run-level VLC, with the very
//! first symbol read as `dct_coeff_first` (`CoefficientPosition::First`,
//! so the Table B-14 `(0, ±1)` codeword is the 1-bit `1s` form) and the
//! rest as `dct_coeff_next`. [`encode_nonintra_block`] forward-
//! transforms a residual block (`current - prediction`), forward-
//! quantises it with the dead-zone non-intra quantiser, and emits the
//! run-level VLCs in zig-zag order terminated by `end_of_block`.
//!
//! ## Zero-MV P-picture (§7.6)
//!
//! [`encode_p_copy_picture`] writes a P-picture every macroblock of
//! which is "MC, Not Coded" (Table B-3 `001`) with a zero forward
//! motion vector — a verbatim copy of the forward anchor. This is the
//! minimal valid P-picture and exercises the full sequence/picture/
//! slice assembly with the §6.2.5.2.1 motion-vector emission. It
//! round-trips through [`crate::decode_video_sequence`] to reproduce the
//! forward anchor exactly.

#![allow(clippy::needless_range_loop)]

use oxideav_core::bits::BitWriter;

use crate::forward_dct::fdct_8x8;
use crate::forward_quant::forward_quantise_block;
use crate::frame_assembly::IntraPictureParams;
use crate::motion_vector::encode_motion_vector;
use crate::mpeg2_dct_coeff::{
    encode_dct_coeff, encode_end_of_block, CoefficientPosition, TableSelection,
};
use crate::mpeg2_dequantize::{quantiser_scale, BlockCoding, DEFAULT_NON_INTRA_WEIGHT};
use crate::mpeg2_inverse_scan::inverse_scan_table;
use crate::picture_header::PictureCodingType;
use crate::stream_writer::{
    write_picture_coding_extension, write_picture_header, write_sequence_extension,
    write_sequence_header, write_slice_header, PictureCodingExtensionParams, SequenceHeaderParams,
    SEQUENCE_END_CODE,
};
use crate::{Error, Result};

/// Forward-quantise + entropy-code one non-intra residual block into
/// `bw`. `residual[y][x]` is the signed `current - prediction` block
/// (range `[-255, +255]`). Returns `true` if the block carried any
/// non-zero coefficient (i.e. it is "coded"); a block that quantises to
/// all zeros writes **nothing** and returns `false` — the caller then
/// clears the block's `coded_block_pattern` bit so the decoder skips it.
pub fn encode_nonintra_block(
    bw: &mut BitWriter,
    residual: &[[i16; 8]; 8],
    quantiser_scale_value: u8,
) -> bool {
    // Forward DCT of the residual + dead-zone non-intra quantise.
    let f = fdct_8x8(residual);
    let qf = forward_quantise_block(
        &f,
        BlockCoding::NonIntra,
        &DEFAULT_NON_INTRA_WEIGHT,
        quantiser_scale_value,
        1, // intra_dc_mult unused for non-intra
    );

    // Collect the zig-zag-ordered coefficient list (all 64; non-intra
    // has no DC prelude).
    let scan = inverse_scan_table(false);
    let table = TableSelection::TableZero;
    // Determine whether any coefficient is non-zero.
    let any = scan.iter().any(|&(v, u)| qf[v as usize][u as usize] != 0);
    if !any {
        return false;
    }

    let mut run = 0u8;
    let mut first = true;
    for &(v, u) in scan.iter() {
        let level = qf[v as usize][u as usize];
        if level == 0 {
            run += 1;
            continue;
        }
        let position = if first {
            CoefficientPosition::First
        } else {
            CoefficientPosition::Next
        };
        let signed = level.clamp(-2047, 2047) as i16;
        encode_dct_coeff(bw, table, position, run, signed);
        run = 0;
        first = false;
    }
    encode_end_of_block(bw, table);
    true
}

/// Encode a zero-motion-vector P-picture: every macroblock is "MC, Not
/// Coded" (Table B-3 `001`) with a zero forward MV, producing a
/// verbatim copy of the forward anchor when decoded.
///
/// The `params` geometry must match the anchor the decoder holds. The
/// returned bytes are the picture layer (`picture_header` through the
/// final slice's byte alignment) — they are appended to a stream whose
/// sequence layer + I-anchor were written earlier. `forward_f_code` is
/// the §6.2.3.1 `f_code[0][*]` the picture-coding extension advertises
/// (the zero MV is codable at any f_code).
pub fn encode_p_copy_picture(
    bw: &mut BitWriter,
    params: IntraPictureParams,
    temporal_reference: u16,
    quantiser_scale_code: u8,
    forward_f_code: u8,
) {
    write_picture_header(
        bw,
        temporal_reference,
        PictureCodingType::Predictive,
        forward_f_code,
        15,
    );
    write_picture_coding_extension(
        bw,
        &PictureCodingExtensionParams {
            forward_f_code,
            frame_pred_frame_dct: params.frame_pred_frame_dct,
            intra_dc_precision: params.intra_dc_precision,
            q_scale_type: params.q_scale_type,
            intra_vlc_format: params.intra_vlc_format,
            alternate_scan: params.alternate_scan,
            ..Default::default()
        },
    );

    let mb_width = params.mb_width();
    let mb_height = params.mb_height();
    for mb_row in 0..mb_height {
        write_slice_header(bw, mb_row as u32, quantiser_scale_code);
        for _ in 0..mb_width {
            // macroblock_address_increment = 1 (Table B-1 `1`).
            bw.write_bit(true);
            // macroblock_type = "MC, Not Coded" (Table B-3 `001`).
            bw.write_u32(0b001, 3);
            // frame_pred_frame_dct = 1 → frame-based motion, one MV.
            // Zero forward motion vector (dx = dy = 0).
            encode_motion_vector(bw, 0, 0, forward_f_code, forward_f_code);
        }
        bw.align_to_byte_zero();
    }
}

/// Build a complete two-picture I→P elementary stream: an all-intra
/// anchor (from [`crate::intra_encoder::encode_intra_picture`]'s body
/// path) followed by a zero-MV P-picture copy of it. Convenience entry
/// used by the round-trip test; the P-picture reproduces the decoded
/// I-anchor exactly.
pub fn encode_i_then_p_copy(
    anchor: &crate::frame_assembly::FrameBuffer,
    params: IntraPictureParams,
    quantiser_scale_code: u8,
) -> Result<Vec<u8>> {
    let _ = quantiser_scale(quantiser_scale_code, params.q_scale_type)?;

    // Reuse the intra encoder for the I-anchor, but we need the two
    // pictures in one stream, so build the sequence + I-picture inline
    // mirroring intra_encoder, then append the P-picture.
    let mut bw = BitWriter::new();
    write_sequence_header(
        &mut bw,
        &SequenceHeaderParams {
            horizontal_size: params.width as u16,
            vertical_size: params.height as u16,
            ..Default::default()
        },
    );
    write_sequence_extension(&mut bw, params.chroma_format, false);

    // I-picture: delegate to the intra encoder's picture writer by
    // re-encoding into a temporary buffer and splicing out its picture
    // layer (everything after the sequence layer, before the end code).
    let i_stream =
        crate::intra_encoder::encode_intra_picture(anchor, params, 0, quantiser_scale_code)?;
    // i_stream = [seq_header][seq_ext][picture...][seq_end]. Find the
    // first picture_start_code and copy from there up to the seq_end.
    let pic_start = find_start(&i_stream, 0x0000_0100).expect("I picture start code");
    let end = i_stream.len() - 4; // strip the trailing sequence_end_code
    bw.write_bytes(&i_stream[pic_start..end]);

    // P-picture copy, temporal_reference = 1.
    encode_p_copy_picture(&mut bw, params, 1, quantiser_scale_code, 1);

    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
    Ok(stream)
}

/// Build a complete two-picture I→P elementary stream where the
/// P-picture is **motion-compensated**: an all-intra anchor followed by a
/// predictive picture that motion-searches `target` against the decoded
/// anchor and codes the residual.
///
/// The reference the P-encoder predicts from is the **decoder's own**
/// reconstruction of the I-anchor (obtained by decoding the I-only
/// stream), so the encoder and decoder share an identical reference and
/// the round-trip is exact: decoding the returned stream yields the I
/// anchor followed by the encoder's P-reconstruction
/// sample-for-sample.
///
/// `anchor` is the I-picture source; `target` is the frame the P-picture
/// approximates. `forward_f_code` (1..=9) sets the motion-vector range.
///
/// # Errors
/// Propagates encode / decode errors; returns
/// [`Error::InvalidBitstream`] if the spliced anchor lacks a picture
/// start code or the intermediate decode fails to produce the anchor.
pub fn encode_i_then_p(
    anchor: &crate::frame_assembly::FrameBuffer,
    target: &crate::frame_assembly::FrameBuffer,
    params: IntraPictureParams,
    quantiser_scale_code: u8,
    forward_f_code: u8,
) -> Result<Vec<u8>> {
    // Encode the I anchor on its own, then decode it back to recover the
    // exact reference the decoder will hold for the P-picture.
    let i_stream =
        crate::intra_encoder::encode_intra_picture(anchor, params, 0, quantiser_scale_code)?;
    let decoded_anchor = crate::decode_video_sequence(&i_stream)?;
    let reference =
        decoded_anchor
            .first()
            .map(|d| d.frame.clone())
            .ok_or(Error::InvalidBitstream(
                "encode_i_then_p: I anchor decode produced no frame",
            ))?;

    // Build the combined stream: sequence layer, the anchor's picture
    // layer, then the motion-compensated P-picture.
    let mut bw = BitWriter::new();
    write_sequence_header(
        &mut bw,
        &SequenceHeaderParams {
            horizontal_size: params.width as u16,
            vertical_size: params.height as u16,
            ..Default::default()
        },
    );
    write_sequence_extension(&mut bw, params.chroma_format, false);

    let pic_start = find_start(&i_stream, 0x0000_0100).ok_or(Error::InvalidBitstream(
        "encode_i_then_p: I picture start code missing",
    ))?;
    let end = i_stream.len() - 4; // strip the trailing sequence_end_code
    bw.write_bytes(&i_stream[pic_start..end]);

    crate::p_picture_encoder::encode_p_picture(
        &mut bw,
        target,
        &reference,
        params,
        1,
        quantiser_scale_code,
        forward_f_code,
    )?;

    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
    Ok(stream)
}

/// Build a complete I→P→P→… elementary stream: an all-intra anchor
/// followed by one predictive picture per entry of `targets`, each
/// predicting from the **reconstruction of the previous picture** (the
/// frame the decoder will hold). This exercises the multi-picture
/// reference chain — every P's reconstruction becomes the next P's
/// forward anchor, exactly as the decoder rotates its anchors.
///
/// `anchor` is the I source; `targets[k]` is the source for the `k`-th
/// P-picture (`temporal_reference = k + 1`). The returned stream decodes
/// to `1 + targets.len()` frames in coded == display order (no B frames).
///
/// # Errors
/// Propagates encode / decode errors.
pub fn encode_i_p_chain(
    anchor: &crate::frame_assembly::FrameBuffer,
    targets: &[crate::frame_assembly::FrameBuffer],
    params: IntraPictureParams,
    quantiser_scale_code: u8,
    forward_f_code: u8,
) -> Result<Vec<u8>> {
    let i_stream =
        crate::intra_encoder::encode_intra_picture(anchor, params, 0, quantiser_scale_code)?;
    let mut reference = crate::decode_video_sequence(&i_stream)?
        .first()
        .map(|d| d.frame.clone())
        .ok_or(Error::InvalidBitstream(
            "encode_i_p_chain: I anchor decode produced no frame",
        ))?;

    let mut bw = BitWriter::new();
    write_sequence_header(
        &mut bw,
        &SequenceHeaderParams {
            horizontal_size: params.width as u16,
            vertical_size: params.height as u16,
            ..Default::default()
        },
    );
    write_sequence_extension(&mut bw, params.chroma_format, false);
    let pic_start = find_start(&i_stream, 0x0000_0100).ok_or(Error::InvalidBitstream(
        "encode_i_p_chain: I picture start code missing",
    ))?;
    let end = i_stream.len() - 4;
    bw.write_bytes(&i_stream[pic_start..end]);

    for (k, target) in targets.iter().enumerate() {
        // Each P predicts from the previous picture's reconstruction and
        // its own reconstruction becomes the next P's reference.
        reference = crate::p_picture_encoder::encode_p_picture(
            &mut bw,
            target,
            &reference,
            params,
            (k + 1) as u16,
            quantiser_scale_code,
            forward_f_code,
        )?;
    }

    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
    Ok(stream)
}

/// Build a complete three-picture I→P→B elementary stream in **coded
/// order** (`I`, `P`, `B`) — the display order is `I`, `B`, `P` with
/// `temporal_reference` 0, 1, 2.
///
/// * The `I` anchor is encoded all-intra (`temporal_reference = 0`).
/// * The `P` picture (`temporal_reference = 2`) is motion-compensated
///   against the **decoded** `I` anchor.
/// * The `B` picture (`temporal_reference = 1`) is bidirectionally
///   predicted from the decoded `I` (forward / past) and the encoder's
///   reconstruction of the `P` (backward / future).
///
/// Because every reference the B-encoder predicts from is the exact frame
/// the decoder reconstructs, the whole stream round-trips faithfully:
/// decoding it reproduces, in display order, the `I` anchor, then the
/// `B` reconstruction, then the `P` reconstruction.
///
/// `i_frame` / `p_frame` / `b_frame` are the three source frames in
/// **display** order. `forward_f_code` / `backward_f_code` (1..=9) set
/// the motion-vector ranges.
///
/// # Errors
/// Propagates encode / decode errors; returns [`Error::InvalidBitstream`]
/// if an intermediate decode fails to produce its anchor.
pub fn encode_i_p_b(
    i_frame: &crate::frame_assembly::FrameBuffer,
    b_frame: &crate::frame_assembly::FrameBuffer,
    p_frame: &crate::frame_assembly::FrameBuffer,
    params: IntraPictureParams,
    quantiser_scale_code: u8,
    forward_f_code: u8,
    backward_f_code: u8,
) -> Result<Vec<u8>> {
    // 1. Encode + decode the I anchor to get the decoder's exact I
    //    reference.
    let i_stream =
        crate::intra_encoder::encode_intra_picture(i_frame, params, 0, quantiser_scale_code)?;
    let i_ref = crate::decode_video_sequence(&i_stream)?
        .first()
        .map(|d| d.frame.clone())
        .ok_or(Error::InvalidBitstream(
            "encode_i_p_b: I anchor decode produced no frame",
        ))?;

    // 2. Assemble the stream: sequence layer + I picture layer.
    let mut bw = BitWriter::new();
    write_sequence_header(
        &mut bw,
        &SequenceHeaderParams {
            horizontal_size: params.width as u16,
            vertical_size: params.height as u16,
            ..Default::default()
        },
    );
    write_sequence_extension(&mut bw, params.chroma_format, false);
    let pic_start = find_start(&i_stream, 0x0000_0100).ok_or(Error::InvalidBitstream(
        "encode_i_p_b: I picture start code missing",
    ))?;
    let end = i_stream.len() - 4;
    bw.write_bytes(&i_stream[pic_start..end]);

    // 3. P picture (temporal_reference = 2) against the decoded I.
    //    Its returned reconstruction is the decoder's P reconstruction
    //    (the encoder predicted from the decoder's exact reference).
    let p_ref = crate::p_picture_encoder::encode_p_picture(
        &mut bw,
        p_frame,
        &i_ref,
        params,
        2,
        quantiser_scale_code,
        forward_f_code,
    )?;

    // 4. B picture (temporal_reference = 1) between them.
    crate::b_picture_encoder::encode_b_picture(
        &mut bw,
        b_frame,
        &i_ref,
        &p_ref,
        params,
        1,
        quantiser_scale_code,
        forward_f_code,
        backward_f_code,
    )?;

    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
    Ok(stream)
}

/// Find the first 4-byte big-endian `code` start-code in `buf`.
fn find_start(buf: &[u8], code: u32) -> Option<usize> {
    buf.windows(4).position(|w| {
        (u32::from(w[0]) << 24 | u32::from(w[1]) << 16 | u32::from(w[2]) << 8 | u32::from(w[3]))
            == code
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame_assembly::FrameBuffer;
    use crate::mpeg2_block_dc::{ColourComponent, DcPredictors};
    use crate::mpeg2_block_decoder::{decode_block, BlockContext};
    use crate::sequence_extension::ChromaFormat;
    use oxideav_core::bits::BitReader;

    #[test]
    fn nonintra_residual_block_roundtrips() {
        // A structured residual block: forward-quantise + encode, then
        // decode with macroblock_intra = false and confirm the
        // reconstructed residual tracks the input within the quantiser
        // step.
        let mut residual = [[0i16; 8]; 8];
        for v in 0..8 {
            for u in 0..8 {
                residual[v][u] = (((v * 8 + u) as i16 * 5) % 120) - 60;
            }
        }
        let qscale = quantiser_scale(8, false).unwrap();
        let mut bw = BitWriter::new();
        let coded = encode_nonintra_block(&mut bw, &residual, qscale);
        assert!(coded, "structured residual must be coded");
        // Pad for the reader.
        for _ in 0..4 {
            bw.write_byte(0);
        }
        let bytes = bw.finish();

        let mut br = BitReader::new(&bytes);
        let ctx = BlockContext {
            intra_vlc_format: false,
            alternate_scan: false,
            intra_dc_precision: 0,
            quantiser_scale_value: qscale,
        };
        let mut preds = DcPredictors::new(0).unwrap();
        let block = decode_block(
            &mut br,
            &ctx,
            &mut preds,
            ColourComponent::Y,
            false, // macroblock_intra
            &DEFAULT_NON_INTRA_WEIGHT,
        )
        .expect("decode non-intra block");

        // The decoded residual (f_pel) should reconstruct the input to
        // within a bounded error.
        let mut max_err = 0i32;
        for v in 0..8 {
            for u in 0..8 {
                let e = (i32::from(block.f_pel[v][u]) - i32::from(residual[v][u])).abs();
                max_err = max_err.max(e);
            }
        }
        assert!(max_err <= 24, "non-intra residual max error {max_err}");
    }

    #[test]
    fn all_zero_residual_is_not_coded() {
        let mut bw = BitWriter::new();
        let coded = encode_nonintra_block(&mut bw, &[[0i16; 8]; 8], 8);
        assert!(!coded);
        assert_eq!(bw.bit_position(), 0, "uncoded block writes nothing");
    }

    fn params(width: usize, height: usize) -> IntraPictureParams {
        IntraPictureParams {
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
    fn p_copy_picture_reproduces_anchor() {
        // Build a gradient I-anchor; the zero-MV P-picture copy must
        // decode to exactly the same reconstruction as the anchor.
        let mut f = FrameBuffer::new(32, 32, ChromaFormat::Yuv420);
        for y in 0..32 {
            for x in 0..32 {
                f.y.put_sample(x, y, (16 + x * 6).min(235) as u8);
            }
        }
        for y in 0..16 {
            for x in 0..16 {
                f.cb.put_sample(x, y, 128);
                f.cr.put_sample(x, y, 128);
            }
        }
        let stream = encode_i_then_p_copy(&f, params(32, 32), 8).expect("encode I+P");
        let frames = crate::decode_video_sequence(&stream).expect("decode");
        assert_eq!(frames.len(), 2);
        // No B frames → coded order == display order: I (tr 0), P (tr 1).
        assert_eq!(frames[0].temporal_reference, 0);
        assert_eq!(frames[1].temporal_reference, 1);
        // The P frame copies the I anchor sample-for-sample.
        for y in 0..32 {
            for x in 0..32 {
                assert_eq!(
                    frames[1].frame.y.get(x, y),
                    frames[0].frame.y.get(x, y),
                    "P copy luma ({x},{y})"
                );
            }
        }
    }
}
