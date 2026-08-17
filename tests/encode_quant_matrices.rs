//! §6.3.11 **downloadable quantiser matrix emission**: the encoder's
//! sequence-header `load_*_quantiser_matrix` slots and the
//! `quant_matrix_extension()` writer (chroma tables, 4:2:2/4:4:4 only),
//! round-tripped through `decode_video_sequence`'s r413 matrix
//! threading.
//!
//! The tests pin three facts:
//!
//! * downloading the §6.3.7 **defaults explicitly** is a semantic
//!   no-op (the decode is sample-identical to the load-free stream);
//! * a custom matrix **changes both sides identically** — the forward
//!   quantiser uses exactly the Table 7-5 bank the decoder resolves,
//!   proven by decoder-exactness against the encoder's returned
//!   reconstruction;
//! * **chroma-specific tables** (w = 2 / w = 3) reach only the chroma
//!   blocks: with a coarse `chroma_intra` download the luma decode is
//!   bit-identical to the download-free stream while chroma moves.

use oxideav_mpeg12video::mpeg2_dequantize::{DEFAULT_INTRA_WEIGHT, DEFAULT_NON_INTRA_WEIGHT};
use oxideav_mpeg12video::mpeg2_inverse_scan::inverse_scan_table;
use oxideav_mpeg12video::quant_matrix_extension::{QuantMatrixExtension, QuantiserMatrixPayload};
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    decode_video_sequence, encode_display_order_gop_sequence,
    encode_display_order_gop_sequence_with_matrices, encode_intra_picture,
    encode_intra_picture_with_matrices, encode_p_picture_with_matrices, FrameBuffer,
    IntraPictureParams,
};

/// Zigzag-serialise a row-major `W[v][u]` matrix into the §6.3.11
/// wire payload order.
fn payload_of(matrix: &[[u8; 8]; 8]) -> QuantiserMatrixPayload {
    let inv = inverse_scan_table(false);
    let mut bytes = [0u8; 64];
    for (i, &(v, u)) in inv.iter().enumerate() {
        bytes[i] = matrix[v as usize][u as usize];
    }
    QuantiserMatrixPayload { bytes }
}

/// A legal coarse intra matrix: zigzag first value 8 (§6.3.11), every
/// other slot `weight`.
fn coarse_intra_payload(weight: u8) -> QuantiserMatrixPayload {
    let mut bytes = [weight; 64];
    bytes[0] = 8;
    QuantiserMatrixPayload { bytes }
}

fn params(width: usize, height: usize, chroma: ChromaFormat) -> IntraPictureParams {
    IntraPictureParams {
        progressive_sequence: true,
        width,
        height,
        chroma_format: chroma,
        frame_pred_frame_dct: true,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
    }
}

fn busy_frame(width: usize, height: usize, chroma: ChromaFormat, shift: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, chroma);
    for y in 0..height {
        for x in 0..width {
            let sx = x + shift;
            let g = 24 + ((sx * 3 + y * 5) % 192);
            let c = if (sx / 4 + y / 4) % 2 == 0 { 12 } else { 0 };
            f.y.put_sample(x, y, (g + c).min(235) as u8);
        }
    }
    let (cw, ch) = f.visible_chroma_dims();
    for y in 0..ch {
        for x in 0..cw {
            f.cb.put_sample(x, y, (64 + (x * 5 + y * 7 + shift) % 128) as u8);
            f.cr.put_sample(x, y, (192u8).saturating_sub(((x * 3 + y * 5) % 128) as u8));
        }
    }
    f
}

fn planes_equal(a: &FrameBuffer, b: &FrameBuffer) -> (bool, bool) {
    let (cw, ch) = a.visible_chroma_dims();
    let mut luma_eq = true;
    let mut chroma_eq = true;
    for y in 0..a.height {
        for x in 0..a.width {
            if a.y.get(x, y) != b.y.get(x, y) {
                luma_eq = false;
            }
        }
    }
    for y in 0..ch {
        for x in 0..cw {
            if a.cb.get(x, y) != b.cb.get(x, y) || a.cr.get(x, y) != b.cr.get(x, y) {
                chroma_eq = false;
            }
        }
    }
    (luma_eq, chroma_eq)
}

#[test]
fn explicit_default_downloads_are_a_semantic_noop() {
    let f = busy_frame(64, 48, ChromaFormat::Yuv420, 0);
    let p = params(64, 48, ChromaFormat::Yuv420);
    let plain = encode_intra_picture(&f, p, 0, 6).expect("plain encode");
    let loaded = encode_intra_picture_with_matrices(
        &f,
        p,
        0,
        6,
        &QuantMatrixExtension {
            intra: Some(payload_of(&DEFAULT_INTRA_WEIGHT)),
            non_intra: Some(payload_of(&DEFAULT_NON_INTRA_WEIGHT)),
            chroma_intra: None,
            chroma_non_intra: None,
        },
    )
    .expect("loaded encode");
    assert_ne!(plain, loaded, "the loads must appear on the wire");
    let d_plain = decode_video_sequence(&plain).expect("decode plain");
    let d_loaded = decode_video_sequence(&loaded).expect("decode loaded");
    let (luma_eq, chroma_eq) = planes_equal(&d_plain[0].frame, &d_loaded[0].frame);
    assert!(
        luma_eq && chroma_eq,
        "default download must not change the decode"
    );
}

#[test]
fn coarse_intra_matrix_changes_both_sides_identically() {
    let f = busy_frame(64, 48, ChromaFormat::Yuv420, 0);
    let p = params(64, 48, ChromaFormat::Yuv420);
    let ext = QuantMatrixExtension {
        intra: Some(coarse_intra_payload(64)),
        non_intra: None,
        chroma_intra: None,
        chroma_non_intra: None,
    };
    let coarse = encode_intra_picture_with_matrices(&f, p, 0, 6, &ext).expect("coarse encode");
    let plain = encode_intra_picture(&f, p, 0, 6).expect("plain encode");
    let d_coarse = decode_video_sequence(&coarse).expect("decode coarse");
    let d_plain = decode_video_sequence(&plain).expect("decode plain");
    let (luma_eq, _) = planes_equal(&d_coarse[0].frame, &d_plain[0].frame);
    assert!(
        !luma_eq,
        "an all-64 AC intra matrix must move the reconstruction"
    );
    // The coarse stream still decodes to a recognisable picture
    // (bounded MAE) — encoder and decoder agreed on the matrix.
    let mut sum = 0u64;
    for y in 0..48 {
        for x in 0..64 {
            sum += u64::from(
                (i32::from(f.y.get(x, y).unwrap())
                    - i32::from(d_coarse[0].frame.y.get(x, y).unwrap()))
                .unsigned_abs(),
            );
        }
    }
    assert!(
        (sum as f64) / (64.0 * 48.0) < 16.0,
        "coarse-matrix decode stays bounded"
    );
}

#[test]
fn chroma_intra_download_reaches_only_chroma_blocks_at_422() {
    let f = busy_frame(64, 48, ChromaFormat::Yuv422, 0);
    let p = params(64, 48, ChromaFormat::Yuv422);
    let ext = QuantMatrixExtension {
        intra: None,
        non_intra: None,
        chroma_intra: Some(coarse_intra_payload(96)),
        chroma_non_intra: None,
    };
    let plain = encode_intra_picture(&f, p, 0, 4).expect("plain encode");
    let loaded = encode_intra_picture_with_matrices(&f, p, 0, 4, &ext).expect("chroma encode");
    let d_plain = decode_video_sequence(&plain).expect("decode plain");
    let d_loaded = decode_video_sequence(&loaded).expect("decode loaded");
    let (luma_eq, chroma_eq) = planes_equal(&d_plain[0].frame, &d_loaded[0].frame);
    assert!(
        luma_eq,
        "a chroma_intra download (w = 2) must leave luminance blocks (w = 0) untouched"
    );
    assert!(
        !chroma_eq,
        "an all-96 chroma intra matrix must move the chroma reconstruction"
    );
}

#[test]
fn gop_with_chroma_matrices_is_decoder_exact_at_422() {
    // I P at 4:2:2 with coarse chroma intra + non-intra tables: the
    // decoded P frame must equal the reconstruction
    // encode_p_picture_with_matrices returned under the same resolved
    // state — the full two-sided §6.3.11 agreement.
    let f0 = busy_frame(64, 48, ChromaFormat::Yuv422, 0);
    let f1 = busy_frame(64, 48, ChromaFormat::Yuv422, 3);
    let p = params(64, 48, ChromaFormat::Yuv422);
    let ext = QuantMatrixExtension {
        intra: None,
        non_intra: None,
        chroma_intra: Some(coarse_intra_payload(48)),
        chroma_non_intra: Some(QuantiserMatrixPayload { bytes: [40u8; 64] }),
    };
    let stream = encode_display_order_gop_sequence_with_matrices(
        &[f0.clone(), f1.clone()],
        0,
        4,
        p,
        6,
        2,
        2,
        &ext,
    )
    .expect("encode GOP");
    let frames = decode_video_sequence(&stream).expect("decode GOP");
    assert_eq!(frames.len(), 2);

    // Rebuild the assembler's exact P call to recover its recon.
    let i_stream = encode_intra_picture_with_matrices(&f0, p, 0, 6, &ext).expect("I");
    let i_ref = decode_video_sequence(&i_stream).expect("decode I")[0]
        .frame
        .clone();
    let state = ext.resolved_state(p.chroma_format);
    let mut scratch = oxideav_core::bits::BitWriter::new();
    let recon =
        encode_p_picture_with_matrices(&mut scratch, &f1, &i_ref, p, 1, 6, 2, &state).expect("P");
    let (luma_eq, chroma_eq) = planes_equal(&frames[1].frame, &recon);
    assert!(
        luma_eq && chroma_eq,
        "decoded P must equal the encoder's reconstruction"
    );

    // And the chroma tables genuinely bit — the same GOP without the
    // downloads decodes to different chroma.
    let stream_plain =
        encode_display_order_gop_sequence(&[f0, f1], 0, 4, p, 6, 2, 2).expect("plain GOP");
    let frames_plain = decode_video_sequence(&stream_plain).expect("decode plain GOP");
    let (_, chroma_eq_plain) = planes_equal(&frames[1].frame, &frames_plain[1].frame);
    assert!(
        !chroma_eq_plain,
        "coarse chroma tables must change the P chroma"
    );
}

#[test]
fn emission_validation_rejects_illegal_payloads() {
    let f = busy_frame(32, 32, ChromaFormat::Yuv420, 0);
    let p420 = params(32, 32, ChromaFormat::Yuv420);

    // Chroma download at 4:2:0 (§6.3.11 "shall take the value '0'").
    let chroma_at_420 = QuantMatrixExtension {
        chroma_intra: Some(coarse_intra_payload(32)),
        ..Default::default()
    };
    assert!(encode_intra_picture_with_matrices(&f, p420, 0, 6, &chroma_at_420).is_err());

    // Intra payload whose first zigzag value is not 8.
    let mut bad_first = coarse_intra_payload(32);
    bad_first.bytes[0] = 16;
    let ext_bad_first = QuantMatrixExtension {
        intra: Some(bad_first),
        ..Default::default()
    };
    assert!(encode_intra_picture_with_matrices(&f, p420, 0, 6, &ext_bad_first).is_err());

    // A zero byte anywhere.
    let mut zero_byte = QuantiserMatrixPayload { bytes: [16u8; 64] };
    zero_byte.bytes[13] = 0;
    let ext_zero = QuantMatrixExtension {
        non_intra: Some(zero_byte),
        ..Default::default()
    };
    assert!(encode_intra_picture_with_matrices(&f, p420, 0, 6, &ext_zero).is_err());
}
