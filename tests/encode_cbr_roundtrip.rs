//! CBR rate control round-trip: the Annex C VBV-regulated MPEG-2 GOP
//! assembler (`encode_cbr_gop_sequence`) must produce streams that
//!
//! 1. **verify** against the exact Annex C model
//!    (`vbv::verify_cbr_stream`): C.5 / C.6 occupancy bounds at every
//!    removal and C.3.1-consistent coded `vbv_delay` values (never the
//!    `0xFFFF` variable-rate sentinel);
//! 2. **decode** through `decode_video_sequence` to the right frame
//!    count / geometry / display order, with bounded distortion against
//!    the synthetic inputs;
//! 3. show the *controller working*: the quantiser adapts under rate
//!    pressure, and easy content draws zero-byte stuffing to hold the
//!    C.5 overflow bound.

use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::vbv::{verify_cbr_stream, VbvStandard};
use oxideav_mpeg12video::{
    decode_video_sequence, encode_cbr_gop_sequence, CbrConfig, FrameBuffer, IntraPictureParams,
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
        progressive_sequence: true,
    }
}

/// Deterministic detailed content: a moving diagonal gradient with a
/// checker overlay (enough AC energy that rate control has real work).
fn busy_frame(width: usize, height: usize, t: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            let g = 20 + ((x * 5 + y * 3 + t * 7) % 200);
            let c = if ((x / 2 + t) / 2 + y / 2) % 2 == 0 {
                20
            } else {
                0
            };
            f.y.put_sample(x, y, (g + c).min(235) as u8);
        }
    }
    for y in 0..height.div_ceil(2) {
        for x in 0..width.div_ceil(2) {
            f.cb.put_sample(x, y, (80 + (x + t) % 96) as u8);
            f.cr.put_sample(x, y, (200u8).saturating_sub(((y + t * 2) % 96) as u8));
        }
    }
    f
}

/// Near-flat content: tiny pictures, so the stream must stuff to hold
/// the C.5 overflow bound.
fn flat_frame(width: usize, height: usize, t: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            f.y.put_sample(x, y, (100 + (t % 3)) as u8);
        }
    }
    for y in 0..height.div_ceil(2) {
        for x in 0..width.div_ceil(2) {
            f.cb.put_sample(x, y, 128);
            f.cr.put_sample(x, y, 128);
        }
    }
    f
}

#[test]
fn cbr_stream_verifies_and_decodes() {
    let (w, h) = (64usize, 48usize);
    let frames: Vec<FrameBuffer> = (0..9).map(|t| busy_frame(w, h, t)).collect();
    let cbr = CbrConfig {
        bit_rate_value: 375, // 150 kbit/s
        vbv_buffer_size_value: 4,
        frame_rate_code: 3,
        initial_quantiser_scale_code: 6,
    };
    let enc = encode_cbr_gop_sequence(&frames, 2, 2, params(w, h), &cbr, 3, 3).expect("CBR encode");

    // 1. Annex C conformance against the declared parameters.
    let report = verify_cbr_stream(&enc.stream, VbvStandard::Mpeg2).expect("VBV conformant");
    assert_eq!(report.bit_rate, 150_000);
    assert_eq!(report.buffer_size_bits, 4 * 16 * 1024);
    assert_eq!(report.pictures.len(), 9);
    assert!(report.max_occupancy_before_bits as u64 <= report.buffer_size_bits);
    assert!(report.min_occupancy_after_bits >= 0);
    for p in &report.pictures {
        assert_ne!(p.vbv_delay, 0xFFFF, "CBR stream must code real delays");
    }

    // 2. Decode round-trip: 9 frames in display order, bounded error.
    let decoded = decode_video_sequence(&enc.stream).expect("decode");
    assert_eq!(decoded.len(), 9);
    for (t, d) in decoded.iter().enumerate() {
        assert_eq!((d.frame.width, d.frame.height), (w, h));
        let src = busy_frame(w, h, t);
        let mut sum = 0u64;
        for y in 0..h {
            for x in 0..w {
                sum += u64::from(
                    d.frame
                        .y
                        .get(x, y)
                        .unwrap()
                        .abs_diff(src.y.get(x, y).unwrap()),
                );
            }
        }
        let mae = sum as f64 / (w * h) as f64;
        assert!(mae < 24.0, "frame {t} luma MAE {mae}");
    }

    // 3. Rate pressure: at 6 kbit/frame this content cannot hold the
    // initial quantiser everywhere — the controller must have adapted.
    let qs = &enc.quantiser_scale_codes;
    assert!(
        qs.iter().any(|&q| q != qs[0]),
        "quantiser never adapted: {qs:?}"
    );
}

#[test]
fn cbr_flat_content_stuffs_against_overflow() {
    let (w, h) = (64usize, 48usize);
    let frames: Vec<FrameBuffer> = (0..6).map(|t| flat_frame(w, h, t)).collect();
    let cbr = CbrConfig {
        bit_rate_value: 250, // 100 kbit/s — far more than flat frames need
        vbv_buffer_size_value: 2,
        frame_rate_code: 3,
        initial_quantiser_scale_code: 4,
    };
    let enc = encode_cbr_gop_sequence(&frames, 1, 2, params(w, h), &cbr, 2, 2).expect("CBR encode");
    assert!(
        enc.stuffing_bytes > 0,
        "flat content at a generous rate must stuff to hold C.5"
    );
    let report = verify_cbr_stream(&enc.stream, VbvStandard::Mpeg2).expect("VBV conformant");
    assert_eq!(report.pictures.len(), 6);

    // The stuffed stream still decodes cleanly.
    let decoded = decode_video_sequence(&enc.stream).expect("decode");
    assert_eq!(decoded.len(), 6);
    // Flat content reconstructs almost exactly.
    let src = flat_frame(w, h, 0);
    let d0 = &decoded[0].frame;
    let max = (0..h)
        .flat_map(|y| (0..w).map(move |x| (x, y)))
        .map(|(x, y)| d0.y.get(x, y).unwrap().abs_diff(src.y.get(x, y).unwrap()))
        .max()
        .unwrap();
    assert!(max <= 4, "flat reconstruction max err {max}");
}

#[test]
fn cbr_single_frame_and_tail_anchor_edge_cases() {
    let (w, h) = (48usize, 32usize);
    // Single frame: one I picture, sequence_end_code included in its
    // picture data (Annex C C.5).
    let one = vec![busy_frame(w, h, 0)];
    let cbr = CbrConfig {
        bit_rate_value: 500,
        vbv_buffer_size_value: 4,
        ..Default::default()
    };
    let enc = encode_cbr_gop_sequence(&one, 2, 1, params(w, h), &cbr, 3, 3).expect("encode");
    verify_cbr_stream(&enc.stream, VbvStandard::Mpeg2).expect("VBV conformant");
    assert_eq!(decode_video_sequence(&enc.stream).unwrap().len(), 1);

    // 5 frames with b_between = 2: the tail anchor clamps (I B B P B/P
    // pattern ends on an anchor), and the last coded picture is a B.
    let five: Vec<FrameBuffer> = (0..5).map(|t| busy_frame(w, h, t)).collect();
    let enc = encode_cbr_gop_sequence(&five, 2, 4, params(w, h), &cbr, 3, 3).expect("encode");
    verify_cbr_stream(&enc.stream, VbvStandard::Mpeg2).expect("VBV conformant");
    let decoded = decode_video_sequence(&enc.stream).unwrap();
    assert_eq!(decoded.len(), 5);
}

#[test]
fn cbr_rejects_impossible_configs() {
    let (w, h) = (64usize, 48usize);
    let frames = vec![busy_frame(w, h, 0)];
    // vbv_delay unrepresentable: 90000 * B / R > 0xFFFE.
    let bad = CbrConfig {
        bit_rate_value: 25, // 10 kbit/s
        vbv_buffer_size_value: 20,
        ..Default::default()
    };
    assert!(encode_cbr_gop_sequence(&frames, 0, 1, params(w, h), &bad, 3, 3).is_err());

    // Rate far too small for even q=31 content.
    let starved = CbrConfig {
        bit_rate_value: 3, // 1200 bit/s
        vbv_buffer_size_value: 1,
        ..Default::default()
    };
    assert!(encode_cbr_gop_sequence(&frames, 0, 1, params(w, h), &starved, 3, 3).is_err());
}
