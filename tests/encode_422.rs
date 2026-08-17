//! The 4:2:2-profile **encoder** leg: end-to-end encode → decode
//! round-trips for `chroma_format == 4:2:2` streams (ISO/IEC 13818-2
//! §6.3.5, Figure 6-11 eight-block macroblock structure, §6.2.5.3
//! `coded_block_pattern_1`).
//!
//! The decoder side has handled 4:2:2 since the Figure 6-11 block
//! numbering fix; these tests hold the encoder to it:
//!
//! * the emitted `sequence_extension()` declares `chroma_format = 10`
//!   and the High@Main `profile_and_level_indication` (Table 8-5 —
//!   the one 1995-text profile row admitting 4:2:2 chroma);
//! * intra macroblocks carry eight blocks in Figure 6-11 order (the
//!   §7.2.1 DC predictor chain interleaves Cb/Cr as 4,5,6,7);
//! * full-height chroma detail survives the round trip (a 4:2:0
//!   regression would halve the chroma rows);
//! * encode ∘ decode is a fixed point on its own reconstruction.

use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    decode_video_sequence, encode_intra_picture, FrameBuffer, IntraPictureParams,
};

fn params_422(width: usize, height: usize) -> IntraPictureParams {
    IntraPictureParams {
        progressive_sequence: true,
        width,
        height,
        chroma_format: ChromaFormat::Yuv422,
        frame_pred_frame_dct: true,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
    }
}

/// Deterministic 4:2:2 content: diagonal luma gradient + checker, and
/// chroma with genuine **vertical** detail (full-height 4:2:2 chroma
/// planes; every chroma row differs from its neighbour).
fn frame_422(width: usize, height: usize, shift: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv422);
    for y in 0..height {
        for x in 0..width {
            let sx = x + shift;
            let g = 24 + ((sx * 3 + y * 5) % 192);
            let c = if (sx / 4 + y / 4) % 2 == 0 { 12 } else { 0 };
            f.y.put_sample(x, y, (g + c).min(235) as u8);
        }
    }
    // 4:2:2 chroma: half width, FULL height.
    for y in 0..height {
        for x in 0..width / 2 {
            f.cb.put_sample(x, y, (64 + (x * 2 + y * 7 + shift) % 128) as u8);
            f.cr.put_sample(x, y, (192u8).saturating_sub(((x * 3 + y * 5) % 128) as u8));
        }
    }
    f
}

fn mae(a: &oxideav_mpeg12video::Plane, b: &oxideav_mpeg12video::Plane, w: usize, h: usize) -> f64 {
    let mut sum = 0u64;
    for y in 0..h {
        for x in 0..w {
            sum += u64::from(
                (i32::from(a.get(x, y).unwrap()) - i32::from(b.get(x, y).unwrap())).unsigned_abs(),
            );
        }
    }
    sum as f64 / (w * h) as f64
}

#[test]
fn intra_422_flat_frame_roundtrips_exactly() {
    let mut f = FrameBuffer::new(32, 32, ChromaFormat::Yuv422);
    for y in 0..32 {
        for x in 0..32 {
            f.y.put_sample(x, y, 100);
        }
    }
    for y in 0..32 {
        for x in 0..16 {
            f.cb.put_sample(x, y, 90);
            f.cr.put_sample(x, y, 170);
        }
    }
    let stream = encode_intra_picture(&f, params_422(32, 32), 0, 8).expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 1);
    let out = &frames[0].frame;
    assert_eq!(out.chroma_format, ChromaFormat::Yuv422);
    for y in 0..32 {
        for x in 0..32 {
            assert_eq!(out.y.get(x, y), Some(100), "luma ({x},{y})");
        }
        for x in 0..16 {
            assert_eq!(out.cb.get(x, y), Some(90), "cb ({x},{y})");
            assert_eq!(out.cr.get(x, y), Some(170), "cr ({x},{y})");
        }
    }
}

#[test]
fn intra_422_structured_frame_roundtrips_faithfully() {
    let f = frame_422(64, 48, 0);
    let stream = encode_intra_picture(&f, params_422(64, 48), 0, 4).expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 1);
    let out = &frames[0].frame;
    assert_eq!((out.width, out.height), (64, 48));
    assert_eq!(out.chroma_format, ChromaFormat::Yuv422);
    let (cw, ch) = out.visible_chroma_dims();
    assert_eq!(
        (cw, ch),
        (32, 48),
        "4:2:2 chroma is half-width, full-height"
    );
    assert!(mae(&f.y, &out.y, 64, 48) < 4.0, "luma MAE");
    assert!(mae(&f.cb, &out.cb, 32, 48) < 4.0, "cb MAE");
    assert!(mae(&f.cr, &out.cr, 32, 48) < 4.0, "cr MAE");
    // Vertical chroma detail must survive: the source's per-row chroma
    // ramp means adjacent chroma rows differ; a 4:2:0 collapse (or a
    // block-numbering swap) would flatten or shear this. Compare the
    // reconstruction against the source row-by-row rather than against
    // an averaged pair.
    let mut worst_row_mae = 0.0f64;
    for y in 0..48 {
        let mut sum = 0u64;
        for x in 0..32 {
            sum += u64::from(
                (i32::from(f.cb.get(x, y).unwrap()) - i32::from(out.cb.get(x, y).unwrap()))
                    .unsigned_abs(),
            );
        }
        worst_row_mae = worst_row_mae.max(sum as f64 / 32.0);
    }
    assert!(worst_row_mae < 8.0, "worst chroma-row MAE {worst_row_mae}");
}

#[test]
fn intra_422_encode_is_reconstruction_idempotent() {
    let f = frame_422(48, 32, 3);
    let p = params_422(48, 32);
    let stream1 = encode_intra_picture(&f, p, 0, 6).expect("encode 1");
    let dec1 = decode_video_sequence(&stream1).expect("decode 1");
    assert_eq!(dec1.len(), 1);
    let stream2 = encode_intra_picture(&dec1[0].frame, p, 0, 6).expect("encode 2");
    let dec2 = decode_video_sequence(&stream2).expect("decode 2");
    assert_eq!(dec2.len(), 1);
    let (a, b) = (&dec1[0].frame, &dec2[0].frame);
    for y in 0..32 {
        for x in 0..48 {
            assert_eq!(a.y.get(x, y), b.y.get(x, y), "luma ({x},{y})");
        }
        for x in 0..24 {
            assert_eq!(a.cb.get(x, y), b.cb.get(x, y), "cb ({x},{y})");
            assert_eq!(a.cr.get(x, y), b.cr.get(x, y), "cr ({x},{y})");
        }
    }
}

#[test]
fn intra_422_non_multiple_of_16_dimensions_roundtrip() {
    // 100×62: right/bottom edge macroblocks overhang the visible
    // picture; 4:2:2 chroma is 50×62 visible.
    let f = frame_422(100, 62, 0);
    let stream = encode_intra_picture(&f, params_422(100, 62), 0, 6).expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 1);
    let out = &frames[0].frame;
    assert_eq!((out.width, out.height), (100, 62));
    assert_eq!(out.visible_chroma_dims(), (50, 62));
    assert!(mae(&f.y, &out.y, 100, 62) < 4.0, "luma MAE");
    assert!(mae(&f.cb, &out.cb, 50, 62) < 4.0, "cb MAE");
}
