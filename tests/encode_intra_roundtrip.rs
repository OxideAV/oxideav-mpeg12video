//! End-to-end I-picture **encode → decode** round-trip.
//!
//! Builds a synthetic frame, encodes it with
//! [`oxideav_mpeg12video::encode_intra_picture`] into a complete MPEG-2
//! elementary stream, decodes that stream back with
//! [`oxideav_mpeg12video::decode_video_sequence`], and checks the
//! reconstruction:
//!
//! 1. A flat frame round-trips exactly (no quantiser loss for a DC-only
//!    block).
//! 2. A structured (gradient / checker) frame round-trips to a faithful
//!    approximation — bounded reconstruction error, full spatial
//!    coverage, and correct geometry.
//! 3. The encoder is **reconstruction-idempotent**: decoding, then
//!    re-encoding the decoded frame, then decoding again yields a
//!    pixel-identical frame (the second pass has nothing left to
//!    quantise away, so encode∘decode is a fixed point).
//!
//! Only the public API surface is exercised; no decoder internals are
//! reached into.

use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    decode_video_sequence, encode_intra_picture, FrameBuffer, IntraPictureParams,
};

fn params(width: usize, height: usize) -> IntraPictureParams {
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

/// A horizontal luma gradient with mid-grey chroma.
fn gradient_frame(width: usize, height: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            let v = (16 + (x * 220 / width.max(1))).min(235) as u8;
            f.y.put_sample(x, y, v);
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

#[test]
fn flat_frame_roundtrips_exactly() {
    let mut f = FrameBuffer::new(32, 32, ChromaFormat::Yuv420);
    for y in 0..32 {
        for x in 0..32 {
            f.y.put_sample(x, y, 100);
        }
    }
    for y in 0..16 {
        for x in 0..16 {
            f.cb.put_sample(x, y, 128);
            f.cr.put_sample(x, y, 128);
        }
    }
    let stream = encode_intra_picture(&f, params(32, 32), 0, 8).expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 1);
    let out = &frames[0].frame;
    for y in 0..32 {
        for x in 0..32 {
            assert_eq!(out.y.get(x, y), Some(100), "luma ({x},{y})");
        }
    }
}

#[test]
fn gradient_frame_roundtrips_faithfully() {
    let f = gradient_frame(64, 48);
    let stream = encode_intra_picture(&f, params(64, 48), 0, 4).expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 1);
    let out = &frames[0].frame;
    assert_eq!((out.y.width(), out.y.height()), (64, 48));

    // Bounded reconstruction error: compute mean absolute error over the
    // luma plane. A smooth gradient at quantiser_scale 4 should
    // reconstruct with small error.
    let mut total = 0u64;
    let mut max_err = 0u32;
    for y in 0..48 {
        for x in 0..64 {
            let orig = i32::from(f.y.get(x, y).unwrap());
            let rec = i32::from(out.y.get(x, y).unwrap());
            let e = (orig - rec).unsigned_abs();
            total += u64::from(e);
            max_err = max_err.max(e);
        }
    }
    let mae = total as f64 / (64.0 * 48.0);
    assert!(mae < 4.0, "luma MAE {mae} too large for a smooth gradient");
    assert!(max_err < 32, "luma max error {max_err} too large");

    // The reconstruction must not be flat (it carries the gradient).
    let samples = out.y.samples();
    let lo = *samples.iter().min().unwrap();
    let hi = *samples.iter().max().unwrap();
    assert!(
        hi - lo > 100,
        "gradient dynamic range collapsed: {lo}..{hi}"
    );
}

#[test]
fn encode_is_reconstruction_idempotent() {
    // Decoding then re-encoding the decoded frame, then decoding again,
    // must be a fixed point: the first decode produced a frame already
    // on the quantiser lattice, so the second round adds no further
    // loss. This proves the encoder's forward quantiser and the
    // decoder's inverse quantiser are exact inverses on the lattice.
    let f = gradient_frame(48, 32);
    let stream1 = encode_intra_picture(&f, params(48, 32), 0, 6).expect("encode 1");
    let dec1 = decode_video_sequence(&stream1).expect("decode 1");
    let frame1 = dec1[0].frame.clone();

    let stream2 = encode_intra_picture(&frame1, params(48, 32), 0, 6).expect("encode 2");
    let dec2 = decode_video_sequence(&stream2).expect("decode 2");
    let frame2 = &dec2[0].frame;

    for y in 0..32 {
        for x in 0..48 {
            assert_eq!(
                frame1.y.get(x, y),
                frame2.y.get(x, y),
                "luma fixed-point mismatch at ({x},{y})"
            );
        }
    }
    for y in 0..16 {
        for x in 0..24 {
            assert_eq!(frame1.cb.get(x, y), frame2.cb.get(x, y), "cb ({x},{y})");
            assert_eq!(frame1.cr.get(x, y), frame2.cr.get(x, y), "cr ({x},{y})");
        }
    }
}

#[test]
fn non_multiple_of_16_dimensions_roundtrip() {
    // 40×24 → 3×2 macroblocks (48×32 coded), edge macroblocks padded.
    let f = gradient_frame(40, 24);
    let stream = encode_intra_picture(&f, params(40, 24), 0, 4).expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 1);
    let out = &frames[0].frame;
    // The visible extent is 40×24; the plane storage covers the full
    // 3×2 macroblock grid (48×32).
    assert_eq!((out.width, out.height), (40, 24));
    assert_eq!((out.y.width(), out.y.height()), (48, 32));
    // Corners must be written (full coverage).
    assert!(out.y.get(0, 0).is_some());
    assert!(out.y.get(39, 23).is_some());
}
