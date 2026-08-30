//! The optional frame-picture encode behaviours
//! (`FrameEncodeOptions`): §7.6.6 skipped-macroblock emission,
//! §7.6.3.9 concealment motion vectors, and the §6.3.10
//! `top_field_first` / `repeat_first_field` / `progressive_frame`
//! output-cadence signalling — each proven by an encode → decode
//! round trip through `decode_video_sequence`.

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::picture_header::Mpeg2PictureHeader;
use oxideav_mpeg12video::quant_matrix_extension::{QuantMatrixExtension, QuantiserMatrixState};
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    decode_video_sequence, encode_b_picture_with_stats, encode_display_order_gop_sequence,
    encode_display_order_gop_sequence_with_options, encode_intra_picture,
    encode_intra_picture_with_options, encode_p_picture_with_options, encode_p_picture_with_stats,
    FrameBuffer, FrameEncodeOptions, IntraPictureParams,
};

fn params(width: usize, height: usize, progressive: bool) -> IntraPictureParams {
    IntraPictureParams {
        width,
        height,
        chroma_format: ChromaFormat::Yuv420,
        frame_pred_frame_dct: true,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        progressive_sequence: progressive,
    }
}

/// A mostly-static frame: flat background with a small high-contrast
/// box at `(bx, by)` — everything away from the box is predictable
/// with a zero vector and a zero residual, the skip sweet spot.
fn boxed_frame(width: usize, height: usize, bx: usize, by: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            f.y.put_sample(x, y, 96);
        }
    }
    for y in by..(by + 8).min(height) {
        for x in bx..(bx + 8).min(width) {
            f.y.put_sample(x, y, if (x + y) % 2 == 0 { 16 } else { 235 });
        }
    }
    for y in 0..height.div_ceil(2) {
        for x in 0..width.div_ceil(2) {
            f.cb.put_sample(x, y, 112);
            f.cr.put_sample(x, y, 144);
        }
    }
    f
}

/// Splice a `picture layer` bit-writer onto an I-picture elementary
/// stream: `i_stream` minus its `sequence_end_code`, plus `bw`, plus
/// the end code.
fn splice(i_stream: &[u8], bw: BitWriter) -> Vec<u8> {
    let mut stream = i_stream[..i_stream.len() - 4].to_vec();
    stream.extend_from_slice(&bw.finish());
    stream.extend_from_slice(&0x0000_01B7u32.to_be_bytes());
    stream
}

fn max_luma_delta(a: &FrameBuffer, b: &FrameBuffer) -> i32 {
    let mut max = 0i32;
    for y in 0..a.height {
        for x in 0..a.width {
            let d = (i32::from(a.y.get(x, y).unwrap()) - i32::from(b.y.get(x, y).unwrap())).abs();
            max = max.max(d);
        }
    }
    max
}

#[test]
fn p_picture_skips_static_macroblocks_and_decodes_exactly() {
    let p = params(64, 48, true);
    let anchor = boxed_frame(64, 48, 8, 8);
    let current = boxed_frame(64, 48, 24, 8); // box moved: most MBs static
    let i_stream = encode_intra_picture(&anchor, p, 0, 6).expect("I");
    let reference = decode_video_sequence(&i_stream).expect("decode I")[0]
        .frame
        .clone();

    let mut bw_skip = BitWriter::new();
    let (recon, stats) = encode_p_picture_with_stats(
        &mut bw_skip,
        &current,
        &reference,
        p,
        1,
        6,
        3,
        &QuantiserMatrixState::defaults(),
        FrameEncodeOptions {
            skipped_macroblocks: true,
            ..Default::default()
        },
    )
    .expect("P with skips");
    assert!(
        stats.skipped > 0,
        "a static background must produce skipped macroblocks: {stats:?}"
    );
    assert_eq!(stats.total(), 4 * 3, "whole macroblock grid counted");

    // The skip stream decodes sample-exactly to the encoder's own
    // reconstruction.
    let stream = splice(&i_stream, bw_skip);
    let frames = decode_video_sequence(&stream).expect("skip stream decodes");
    assert_eq!(frames.len(), 2);
    assert_eq!(max_luma_delta(&frames[1].frame, &recon), 0);

    // And it is strictly smaller than the same picture without skips.
    let mut bw_plain = BitWriter::new();
    encode_p_picture_with_options(
        &mut bw_plain,
        &current,
        &reference,
        p,
        1,
        6,
        3,
        &QuantiserMatrixState::defaults(),
        FrameEncodeOptions::default(),
    )
    .expect("P without skips");
    assert!(
        bw_skip_len(&stream, &i_stream) < bw_plain.byte_len(),
        "skips must shrink the picture layer"
    );
}

/// Length of the spliced P picture layer (skip stream minus the I
/// prefix minus the end code).
fn bw_skip_len(stream: &[u8], i_stream: &[u8]) -> usize {
    stream.len() - (i_stream.len() - 4) - 4
}

#[test]
fn b_picture_skips_inherit_direction_and_decode_exactly() {
    let p = params(64, 48, true);
    let anchor = boxed_frame(64, 48, 8, 8);
    let i_stream = encode_intra_picture(&anchor, p, 0, 6).expect("I");
    let reference = decode_video_sequence(&i_stream).expect("decode I")[0]
        .frame
        .clone();

    // Backward anchor: the same P the decoder will hold.
    let future = boxed_frame(64, 48, 40, 24);
    let mut bw = BitWriter::new();
    let (backward, _) = encode_p_picture_with_stats(
        &mut bw,
        &future,
        &reference,
        p,
        2,
        6,
        3,
        &QuantiserMatrixState::defaults(),
        FrameEncodeOptions::default(),
    )
    .expect("P anchor");

    // The B frame equals the forward anchor away from the two boxes —
    // its macroblocks quantise to zero under the PMV/previous-direction
    // prediction and skip.
    let b_frame = boxed_frame(64, 48, 16, 8);
    let stats = encode_b_picture_with_stats(
        &mut bw,
        &b_frame,
        &reference,
        &backward,
        p,
        1,
        6,
        3,
        3,
        &QuantiserMatrixState::defaults(),
        FrameEncodeOptions {
            skipped_macroblocks: true,
            ..Default::default()
        },
    )
    .expect("B with skips");
    assert!(
        stats.skipped > 0,
        "a static background must skip B macroblocks: {stats:?}"
    );

    let stream = splice(&i_stream, bw);
    let frames = decode_video_sequence(&stream).expect("skip stream decodes");
    assert_eq!(frames.len(), 3, "display order I, B, P");
    // The B decode approximates its input (the skip reconstruction is
    // the prediction, which tracks the static content).
    assert!(
        max_luma_delta(&frames[1].frame, &b_frame) <= 40,
        "B skip reconstruction diverged"
    );
}

#[test]
fn gop_assembler_with_skips_shrinks_stream_and_round_trips() {
    let p = params(64, 48, true);
    let frames: Vec<FrameBuffer> = (0..5).map(|i| boxed_frame(64, 48, 8 + 8 * i, 16)).collect();
    let plain = encode_display_order_gop_sequence(&frames, 2, 4, p, 6, 3, 3).expect("plain stream");
    let (skippy, stats) = encode_display_order_gop_sequence_with_options(
        &frames,
        2,
        4,
        p,
        6,
        3,
        3,
        &QuantMatrixExtension::default(),
        &|_| FrameEncodeOptions {
            skipped_macroblocks: true,
            ..Default::default()
        },
    )
    .expect("skip stream");
    assert!(stats.skipped > 0, "no skips emitted: {stats:?}");
    assert!(
        skippy.len() < plain.len(),
        "skips must shrink the stream ({} vs {})",
        skippy.len(),
        plain.len()
    );
    let decoded = decode_video_sequence(&skippy).expect("skip stream decodes");
    assert_eq!(decoded.len(), 5);
    for (i, d) in decoded.iter().enumerate() {
        assert!(
            max_luma_delta(&d.frame, &frames[i]) <= 48,
            "frame {i} diverged"
        );
    }
}

#[test]
fn concealment_vectors_do_not_change_the_reconstruction() {
    let p = params(64, 48, true);
    // The moving stamp forces intra-fallback macroblocks in the P
    // pictures, which then carry concealment vectors.
    let frames: Vec<FrameBuffer> = (0..4)
        .map(|i| boxed_frame(64, 48, 8 + 16 * i, 24))
        .collect();
    let plain = encode_display_order_gop_sequence(&frames, 1, 4, p, 6, 3, 3).expect("plain stream");
    let (concealed, _stats) = encode_display_order_gop_sequence_with_options(
        &frames,
        1,
        4,
        p,
        6,
        3,
        3,
        &QuantMatrixExtension::default(),
        &|_| FrameEncodeOptions {
            concealment_motion_vectors: true,
            ..Default::default()
        },
    )
    .expect("concealment stream");
    assert_ne!(plain, concealed, "concealment vectors must move bits");

    // The picture_coding_extension() flag is on the wire.
    let pic = find_code(&concealed, 0x0000_0100).expect("picture start");
    let (_hdr, ext) =
        Mpeg2PictureHeader::parse_with_extension(&concealed[pic..]).expect("parse I header");
    assert!(ext.concealment_motion_vectors);
    assert_eq!(ext.f_code_fwd_horiz, 3, "concealment f_code[0][*]");

    // Concealment vectors are display-process hints: the decoded
    // samples are identical with and without them.
    let a = decode_video_sequence(&plain).expect("plain decodes");
    let b = decode_video_sequence(&concealed).expect("concealment decodes");
    assert_eq!(a.len(), b.len());
    for (fa, fb) in a.iter().zip(b.iter()) {
        assert_eq!(max_luma_delta(&fa.frame, &fb.frame), 0);
    }
}

#[test]
fn intra_picture_with_concealment_round_trips_standalone() {
    let p = params(64, 48, true);
    let reference = boxed_frame(64, 48, 8, 8);
    let frame = boxed_frame(64, 48, 16, 8);
    let plain = encode_intra_picture(&frame, p, 0, 6).expect("plain I");
    let concealed = encode_intra_picture_with_options(
        &frame,
        p,
        0,
        6,
        &QuantMatrixExtension::default(),
        FrameEncodeOptions {
            concealment_motion_vectors: true,
            ..Default::default()
        },
        3,
        Some(&reference),
    )
    .expect("concealed I");
    assert_ne!(plain, concealed);
    let a = decode_video_sequence(&plain).expect("plain decodes");
    let b = decode_video_sequence(&concealed).expect("concealed decodes");
    assert_eq!(max_luma_delta(&a[0].frame, &b[0].frame), 0);
}

#[test]
fn pulldown_flags_ride_the_wire_and_surface_on_decoded_frames() {
    // Interlaced sequence (progressive_sequence = 0), height a
    // multiple of 32 so both grids agree; every frame progressive
    // content with the classic 3:2 repeat pattern.
    let p = params(64, 64, false);
    let frames: Vec<FrameBuffer> = (0..4).map(|i| boxed_frame(64, 64, 8 + 4 * i, 16)).collect();
    let (stream, _stats) = encode_display_order_gop_sequence_with_options(
        &frames,
        1,
        4,
        p,
        6,
        3,
        3,
        &QuantMatrixExtension::default(),
        &|i| FrameEncodeOptions::pulldown_32(i),
    )
    .expect("pulldown stream");
    let decoded = decode_video_sequence(&stream).expect("pulldown stream decodes");
    assert_eq!(decoded.len(), 4);
    let mut fields = 0u32;
    for (i, d) in decoded.iter().enumerate() {
        let o = FrameEncodeOptions::pulldown_32(i);
        assert_eq!(d.top_field_first, o.top_field_first, "frame {i} tff");
        assert_eq!(d.repeat_first_field, o.repeat_first_field, "frame {i} rff");
        assert!(d.progressive_frame, "frame {i} progressive_frame");
        fields += d.output_field_count();
    }
    assert_eq!(
        fields, 10,
        "3:2 pulldown period is ten fields per four frames"
    );
}

#[test]
fn section_6_3_10_flag_violations_are_rejected() {
    let p = params(64, 48, true);
    let frame = boxed_frame(64, 48, 8, 8);
    // Progressive sequence: top_field_first without repeat_first_field.
    let err = encode_intra_picture_with_options(
        &frame,
        p,
        0,
        6,
        &QuantMatrixExtension::default(),
        FrameEncodeOptions {
            top_field_first: true,
            ..Default::default()
        },
        15,
        None,
    );
    assert!(err.is_err(), "tff without rff must be rejected");
    // Concealment vectors need a real f_code.
    let err = encode_intra_picture_with_options(
        &frame,
        p,
        0,
        6,
        &QuantMatrixExtension::default(),
        FrameEncodeOptions {
            concealment_motion_vectors: true,
            ..Default::default()
        },
        15,
        None,
    );
    assert!(err.is_err(), "concealment with f_code 15 must be rejected");
}

fn find_code(buf: &[u8], code: u32) -> Option<usize> {
    buf.windows(4).position(|w| {
        (u32::from(w[0]) << 24 | u32::from(w[1]) << 16 | u32::from(w[2]) << 8 | u32::from(w[3]))
            == code
    })
}
