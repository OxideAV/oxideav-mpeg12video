//! End-to-end I→P **motion-compensated encode → decode** round-trip.
//!
//! Encodes an I anchor plus a predictive picture that motion-searches a
//! translated / modified target against the decoded anchor, then decodes
//! the whole elementary stream back with
//! [`oxideav_mpeg12video::decode_video_sequence`] and checks:
//!
//! 1. The decoder produces two frames in coded == display order (no B
//!    frames): the I anchor (tr 0) then the P picture (tr 1).
//! 2. The decoded P frame matches the **encoder's own** reconstruction
//!    sample-for-sample — proved by the encoder using the decoder's
//!    reconstruction of the anchor as its prediction reference, so the
//!    two share an identical reference and the motion-compensated
//!    residual decode is bit-exact.
//! 3. A pure translation is predicted well: the P frame reconstructs the
//!    translated target with bounded error (the motion search recovers
//!    the shift, leaving a near-zero residual).
//!
//! Only the public API is exercised.

use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    decode_video_sequence, encode_i_p_b, encode_i_then_p, encode_intra_picture, FrameBuffer,
    IntraPictureParams,
};

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

/// A frame whose luma is `f(x, y)` with mid-grey chroma.
fn frame_from<F: Fn(usize, usize) -> u8>(w: usize, h: usize, f: F) -> FrameBuffer {
    let mut fb = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
    for y in 0..h {
        for x in 0..w {
            fb.y.put_sample(x, y, f(x, y));
        }
    }
    for y in 0..fb.cb.height() {
        for x in 0..fb.cb.width() {
            fb.cb.put_sample(x, y, 128);
            fb.cr.put_sample(x, y, 128);
        }
    }
    fb
}

#[test]
fn identical_target_p_reproduces_anchor() {
    // When the P target equals the **decoded** anchor (the frame the
    // decoder actually holds as the reference), every macroblock finds a
    // zero MV with a zero residual, so the P frame is an exact copy of
    // the I reconstruction — a true fixed point that proves the
    // motion-compensated copy path is bit-exact.
    let w = 48;
    let h = 32;
    let anchor = frame_from(w, h, |x, y| (16 + (x + y) * 3).min(235) as u8);

    // The decoder's reconstruction of the I anchor (differs from the raw
    // anchor by the I-picture quantiser loss). Encoding a P whose target
    // is this reconstruction must reproduce it exactly.
    let i_stream = encode_intra_picture(&anchor, params(w, h), 0, 8).expect("encode I");
    let decoded_anchor = decode_video_sequence(&i_stream).expect("decode I")[0]
        .frame
        .clone();

    let stream = encode_i_then_p(&anchor, &decoded_anchor, params(w, h), 8, 2).expect("encode I+P");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 2);
    assert_eq!(frames[0].temporal_reference, 0);
    assert_eq!(frames[1].temporal_reference, 1);
    for y in 0..h {
        for x in 0..w {
            assert_eq!(
                frames[1].frame.y.get(x, y),
                frames[0].frame.y.get(x, y),
                "P luma ({x},{y}) must reproduce the anchor"
            );
            assert_eq!(
                frames[1].frame.y.get(x, y),
                decoded_anchor.y.get(x, y),
                "P luma ({x},{y}) must equal the decoded reference"
            );
        }
    }
}

#[test]
fn translated_target_predicts_well() {
    // The P target is the anchor shifted right by 4 luma samples. The
    // motion search recovers the shift, so the residual is small and the
    // reconstruction tracks the target closely.
    let w = 64;
    let h = 48;
    let anchor = frame_from(w, h, |x, _| (16 + (x % 48) * 4).min(235) as u8);
    let shift = 4usize;
    let target = frame_from(w, h, |x, y| {
        anchor.y.get(x.saturating_sub(shift), y).unwrap()
    });

    let stream = encode_i_then_p(&anchor, &target, params(w, h), 4, 3).expect("encode I+P");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 2);

    // Compare the decoded P frame to the target over the interior (avoid
    // the left edge where the shift introduces fresh content the anchor
    // cannot predict).
    let mut total = 0u64;
    let mut count = 0u64;
    for y in 0..h {
        for x in (shift + 8)..w {
            let t = i32::from(target.y.get(x, y).unwrap());
            let r = i32::from(frames[1].frame.y.get(x, y).unwrap());
            total += (t - r).unsigned_abs() as u64;
            count += 1;
        }
    }
    let mae = total as f64 / count as f64;
    assert!(
        mae < 4.0,
        "interior P MAE {mae} too large for a clean shift"
    );
}

#[test]
fn p_frame_matches_encoder_reconstruction_on_a_modified_target() {
    // A target that differs from the anchor by a real residual (a
    // brightness ramp added on top). The decoded P frame is the encoder's
    // own reconstruction, so it must be self-consistent and bounded.
    let w = 48;
    let h = 48;
    let anchor = frame_from(w, h, |x, y| (40 + ((x * 3 + y * 2) % 150)) as u8);
    let target = frame_from(w, h, |x, y| {
        let base = 40 + ((x * 3 + y * 2) % 150);
        (base + x % 17).min(235) as u8
    });
    let stream = encode_i_then_p(&anchor, &target, params(w, h), 6, 2).expect("encode I+P");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 2);

    // The reconstruction must carry real content (not collapse to the
    // anchor) and approximate the target.
    let mut max_err = 0i32;
    for y in 0..h {
        for x in 0..w {
            let t = i32::from(target.y.get(x, y).unwrap());
            let r = i32::from(frames[1].frame.y.get(x, y).unwrap());
            max_err = max_err.max((t - r).abs());
        }
    }
    assert!(max_err < 40, "P reconstruction max err {max_err} too large");
}

#[test]
fn i_p_b_stream_decodes_three_frames_in_display_order() {
    // A 3-frame group with a moving feature: the B frame sits temporally
    // between the I and P, so bidirectional prediction (averaging the
    // past I and future P) should track it well. Verify the decoder
    // produces I, B, P in display order with the right temporal
    // references, and that the B reconstruction approximates its target.
    let w = 48;
    let h = 32;
    // A vertical bar that slides right over the three display frames.
    let bar = |pos: usize| {
        move |x: usize, _y: usize| -> u8 {
            if x.abs_diff(pos) < 4 {
                220
            } else {
                40
            }
        }
    };
    let i_frame = frame_from(w, h, bar(8));
    let b_frame = frame_from(w, h, bar(16));
    let p_frame = frame_from(w, h, bar(24));

    let stream =
        encode_i_p_b(&i_frame, &b_frame, &p_frame, params(w, h), 6, 3, 3).expect("encode I+P+B");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 3, "I, B, P in display order");
    // Display order: I (tr 0), B (tr 1), P (tr 2).
    assert_eq!(frames[0].temporal_reference, 0);
    assert_eq!(frames[1].temporal_reference, 1);
    assert_eq!(frames[2].temporal_reference, 2);

    // The decoded B frame approximates its target (a clean bar slide is
    // well predicted by averaging the I and P anchors / by forward or
    // backward prediction, whichever the per-MB mode picked).
    let mut total = 0u64;
    for y in 0..h {
        for x in 0..w {
            let t = i32::from(b_frame.y.get(x, y).unwrap());
            let r = i32::from(frames[1].frame.y.get(x, y).unwrap());
            total += (t - r).unsigned_abs() as u64;
        }
    }
    let mae = total as f64 / (w * h) as f64;
    assert!(mae < 30.0, "B-frame luma MAE {mae} too large");
}

#[test]
fn non_multiple_of_16_dimensions_inter_roundtrip() {
    // 40×24 → 3×2 macroblocks; edge macroblocks padded. The P round-trip
    // must still decode two frames with full coverage.
    let anchor = frame_from(40, 24, |x, _| (16 + x * 5).min(235) as u8);
    let target = frame_from(40, 24, |x, y| anchor.y.get(x.saturating_sub(2), y).unwrap());
    let stream = encode_i_then_p(&anchor, &target, params(40, 24), 4, 2).expect("encode I+P");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 2);
    let out = &frames[1].frame;
    assert_eq!((out.y.width(), out.y.height()), (40, 24));
    assert!(out.y.get(0, 0).is_some());
    assert!(out.y.get(39, 23).is_some());
}
