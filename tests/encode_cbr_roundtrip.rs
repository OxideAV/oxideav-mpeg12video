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
use oxideav_mpeg12video::vbv::{VbvStandard, verify_cbr_stream};
use oxideav_mpeg12video::{
    CbrConfig, FrameBuffer, IntraPictureParams, decode_video_sequence, encode_cbr_gop_sequence,
};

fn params(width: usize, height: usize) -> IntraPictureParams {
    IntraPictureParams::new(
        width,
        height,
        ChromaFormat::Yuv420,
        true,
        0,
        false,
        false,
        false,
        true,
    )
}

/// Deterministic detailed content: a moving diagonal gradient with a
/// checker overlay (enough AC energy that rate control has real work).
fn busy_frame(width: usize, height: usize, t: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            let g = 20 + ((x * 5 + y * 3 + t * 7) % 200);
            let c = if ((x / 2 + t) / 2 + y / 2).is_multiple_of(2) {
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
    let cbr = CbrConfig::default()
        .with_bit_rate_value(375) // 150 kbit/s
        .with_vbv_buffer_size_value(4)
        .with_frame_rate_code(3)
        .with_initial_quantiser_scale_code(6);
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
    let cbr = CbrConfig::default()
        .with_bit_rate_value(250) // 100 kbit/s — far more than flat frames need
        .with_vbv_buffer_size_value(2)
        .with_frame_rate_code(3)
        .with_initial_quantiser_scale_code(4);
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
    let cbr = CbrConfig::default()
        .with_bit_rate_value(500)
        .with_vbv_buffer_size_value(4);
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
    let bad = CbrConfig::default()
        .with_bit_rate_value(25) // 10 kbit/s
        .with_vbv_buffer_size_value(20);
    assert!(encode_cbr_gop_sequence(&frames, 0, 1, params(w, h), &bad, 3, 3).is_err());

    // Rate far too small for even q=31 content.
    let starved = CbrConfig::default()
        .with_bit_rate_value(3) // 1200 bit/s
        .with_vbv_buffer_size_value(1);
    assert!(encode_cbr_gop_sequence(&frames, 0, 1, params(w, h), &starved, 3, 3).is_err());
}

#[test]
fn cbr_field_coded_sequence_verifies_and_decodes() {
    // Interlaced field-picture pairs under the C.11 field-period
    // cadence: every field picture is its own VBV picture with a real
    // vbv_delay; the verifier reads picture_structure from each
    // picture coding extension and applies the field interval.
    use oxideav_mpeg12video::encode_field_cbr_gop_sequence;
    let (w, h) = (48usize, 64usize);
    let mut fp = params(w, h);
    fp.frame_pred_frame_dct = false;
    fp.progressive_sequence = false;

    let frames: Vec<FrameBuffer> = (0..6)
        .map(|t| {
            let mut f = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
            for y in 0..h {
                for x in 0..w {
                    let v = 30 + ((x * 4 + y * 7 + t * 3) % 180);
                    let line = if y % 2 == 0 { 12 } else { 0 };
                    f.y.put_sample(x, y, (v + line).min(235) as u8);
                }
            }
            for y in 0..h / 2 {
                for x in 0..w / 2 {
                    f.cb.put_sample(x, y, (90 + (x + t) % 80) as u8);
                    f.cr.put_sample(x, y, (190u8).saturating_sub(((y + 2 * t) % 80) as u8));
                }
            }
            f
        })
        .collect();

    let cbr = CbrConfig::default()
        .with_bit_rate_value(375) // 150 kbit/s
        .with_vbv_buffer_size_value(4)
        .with_frame_rate_code(3)
        .with_initial_quantiser_scale_code(6);
    let enc =
        encode_field_cbr_gop_sequence(&frames, 1, 2, &fp, &cbr, 3, 3).expect("field CBR encode");

    // Annex C conformance: 12 field pictures on the field cadence.
    let report = verify_cbr_stream(&enc.stream, VbvStandard::Mpeg2).expect("VBV conformant");
    assert_eq!(report.pictures.len(), 12);
    assert!(report.max_occupancy_before_bits as u64 <= report.buffer_size_bits);
    assert!(report.min_occupancy_after_bits >= 0);
    for p in &report.pictures {
        assert_ne!(p.vbv_delay, 0xFFFF);
    }
    assert_eq!(enc.quantiser_scale_codes.len(), 12);

    // Decode round-trip: 6 assembled frames in display order.
    let decoded = decode_video_sequence(&enc.stream).expect("decode");
    assert_eq!(decoded.len(), 6);
    for d in &decoded {
        assert_eq!((d.frame.width, d.frame.height), (w, h));
    }
}

/// Pseudo-random content no prediction can track: predicted pictures
/// stay large at every quantiser.
fn noise_frame(width: usize, height: usize, t: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    let mut state = 0x1234_5678u32 ^ (t as u32).wrapping_mul(0x9E37_79B9);
    let mut next = || {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (state >> 24) as u8
    };
    for y in 0..height {
        for x in 0..width {
            f.y.put_sample(x, y, 64 + next() / 2);
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

/// Found by the black-box matrix: at a low rate the controller spent
/// the buffer on a GOP's predicted pictures and the next I-picture
/// then failed the C.6 bound at every quantiser. Predicted pictures
/// now leave a reserve, and one that fits at no quantiser is replaced
/// by a picture repeating its reference instead of failing the encode.
#[test]
fn cbr_low_rate_survives_content_that_fits_at_no_quantiser() {
    let (w, h) = (64usize, 48usize);
    let frames: Vec<FrameBuffer> = (0..26).map(|t| noise_frame(w, h, t)).collect();
    let cbr = CbrConfig::default()
        .with_bit_rate_value(75) // 30 kbit/s
        .with_vbv_buffer_size_value(1)
        .with_frame_rate_code(3)
        .with_initial_quantiser_scale_code(6);
    let enc = encode_cbr_gop_sequence(&frames, 2, 4, params(w, h), &cbr, 3, 3).expect("CBR encode");
    let report = verify_cbr_stream(&enc.stream, VbvStandard::Mpeg2).expect("VBV conformant");
    assert_eq!(report.pictures.len(), 26);
    assert!(report.min_occupancy_after_bits >= 0);
    // Some predicted picture was replaced by a repeat picture (a few
    // bytes per macroblock row).
    let smallest = report.pictures.iter().map(|p| p.data_bits).min().unwrap();
    assert!(
        smallest < 600,
        "no repeat picture: smallest {smallest} bits"
    );
    assert_eq!(
        decode_video_sequence(&enc.stream).expect("decode").len(),
        26
    );

    let seq = oxideav_mpeg12video::Mpeg1SequenceParams::default()
        .with_horizontal_size(w as u16)
        .with_vertical_size(h as u16)
        .with_picture_rate_code(3)
        .with_bit_rate_value(150)
        .with_vbv_buffer_size_value(2);
    let enc = oxideav_mpeg12video::encode_mpeg1_cbr_sequence(&frames, 2, 4, &seq, 6, 3, 3)
        .expect("MPEG-1 CBR encode");
    let report = verify_cbr_stream(&enc.stream, VbvStandard::Mpeg1).expect("VBV conformant");
    assert_eq!(report.pictures.len(), 26);
    assert_eq!(
        decode_video_sequence(&enc.stream).expect("decode").len(),
        26
    );
}

/// A repeat picture decodes to a copy of its forward reference.
#[test]
fn repeat_pictures_decode_to_the_reference() {
    use oxideav_core::bits::BitWriter;
    let (w, h) = (48usize, 32usize);
    let p = params(w, h);
    let intra = oxideav_mpeg12video::encode_intra_picture(&busy_frame(w, h, 0), p, 0, 4).unwrap();
    let mut stream = intra[..intra.len() - 4].to_vec();
    // Coded order I P B: display I B P.
    let mut bw = BitWriter::new();
    oxideav_mpeg12video::encode_repeat_p_picture(&mut bw, p, 2, 3).unwrap();
    oxideav_mpeg12video::encode_repeat_b_picture(&mut bw, p, 1, 3, 3).unwrap();
    stream.extend_from_slice(&bw.finish());
    stream.extend_from_slice(&[0, 0, 1, 0xB7]);
    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 3);
    for d in &decoded[1..] {
        assert_eq!(d.frame.y.samples(), decoded[0].frame.y.samples());
        assert_eq!(d.frame.cb.samples(), decoded[0].frame.cb.samples());
        assert_eq!(d.frame.cr.samples(), decoded[0].frame.cr.samples());
    }
}

/// The CBR controllers can emit skipped macroblocks: the streams stay
/// Annex C conformant, decode to the right frame count, and a static
/// scene spends the bits it saves on a finer quantiser.
#[test]
fn cbr_with_skipped_macroblocks_verifies_and_decodes() {
    let (w, h) = (96usize, 64usize);
    // A static, detailed scene.
    let frames: Vec<FrameBuffer> = (0..13).map(|_| busy_frame(w, h, 0)).collect();
    let base = CbrConfig::default()
        .with_bit_rate_value(250) // 100 kbit/s
        .with_vbv_buffer_size_value(4)
        .with_frame_rate_code(3)
        .with_initial_quantiser_scale_code(10);
    let plain = encode_cbr_gop_sequence(&frames, 2, 4, params(w, h), &base, 3, 3).expect("CBR");
    let skips = encode_cbr_gop_sequence(
        &frames,
        2,
        4,
        params(w, h),
        &base.with_skipped_macroblocks(true),
        3,
        3,
    )
    .expect("CBR with skips");
    for enc in [&plain, &skips] {
        let report = verify_cbr_stream(&enc.stream, VbvStandard::Mpeg2).expect("VBV conformant");
        assert_eq!(report.pictures.len(), 13);
        assert_eq!(
            decode_video_sequence(&enc.stream).expect("decode").len(),
            13
        );
    }
    let mean = |qs: &[u8]| qs.iter().map(|&q| u32::from(q)).sum::<u32>() as f64 / qs.len() as f64;
    assert!(
        mean(&skips.quantiser_scale_codes) <= mean(&plain.quantiser_scale_codes),
        "skips {:?} plain {:?}",
        skips.quantiser_scale_codes,
        plain.quantiser_scale_codes
    );

    let seq = oxideav_mpeg12video::Mpeg1SequenceParams::default()
        .with_horizontal_size(w as u16)
        .with_vertical_size(h as u16)
        .with_picture_rate_code(3)
        .with_bit_rate_value(250)
        .with_vbv_buffer_size_value(4);
    let enc = oxideav_mpeg12video::encode_mpeg1_cbr_sequence_with_skips(
        &frames, 2, 4, &seq, 10, 3, 3, true,
    )
    .expect("MPEG-1 CBR with skips");
    let report = verify_cbr_stream(&enc.stream, VbvStandard::Mpeg1).expect("VBV conformant");
    assert_eq!(report.pictures.len(), 13);
    assert_eq!(
        decode_video_sequence(&enc.stream).expect("decode").len(),
        13
    );
}
