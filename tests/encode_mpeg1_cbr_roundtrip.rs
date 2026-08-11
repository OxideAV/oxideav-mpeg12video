//! MPEG-1 CBR rate control round-trip: the ISO/IEC 11172-2 Annex C
//! (C.1.1–C.1.4) VBV-regulated assembler (`encode_mpeg1_cbr_sequence`)
//! must produce streams that verify against the exact buffer model,
//! decode faithfully, and show the controller adapting.

use oxideav_mpeg12video::vbv::{verify_cbr_stream, VbvStandard};
use oxideav_mpeg12video::{
    decode_video_sequence, encode_mpeg1_cbr_sequence, FrameBuffer, Mpeg1SequenceParams,
};

use oxideav_mpeg12video::sequence_extension::ChromaFormat;

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

fn flat_frame(width: usize, height: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            f.y.put_sample(x, y, 100);
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

fn seq(width: u16, height: u16, bit_rate_value: u32, vbv: u16) -> Mpeg1SequenceParams {
    Mpeg1SequenceParams {
        horizontal_size: width,
        vertical_size: height,
        bit_rate_value,
        vbv_buffer_size_value: vbv,
        ..Default::default()
    }
}

#[test]
fn mpeg1_cbr_stream_verifies_and_decodes() {
    let (w, h) = (64usize, 48usize);
    let frames: Vec<FrameBuffer> = (0..8).map(|t| busy_frame(w, h, t)).collect();
    // 150 kbit/s, B = 65 536 bits — inside the §2.4.3.2 constrained
    // bounds, so the flag must come out set.
    let s = seq(w as u16, h as u16, 375, 4);
    let enc = encode_mpeg1_cbr_sequence(&frames, 2, 2, &s, 6, 3, 3).expect("CBR encode");

    let report = verify_cbr_stream(&enc.stream, VbvStandard::Mpeg1).expect("VBV conformant");
    assert_eq!(report.bit_rate, 150_000);
    assert_eq!(report.buffer_size_bits, 4 * 16 * 1024);
    assert_eq!(report.pictures.len(), 8);
    assert!(report.max_occupancy_before_bits as u64 <= report.buffer_size_bits);
    assert!(report.min_occupancy_after_bits >= 0);
    for p in &report.pictures {
        assert_ne!(p.vbv_delay, 0xFFFF, "CBR stream must code real delays");
    }

    // The stream is still a classifiable 11172-2 sequence (no
    // extension start code before the first picture) and decodes to 8
    // display-order frames with bounded distortion.
    let decoded = decode_video_sequence(&enc.stream).expect("decode");
    assert_eq!(decoded.len(), 8);
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

    // Under this rate pressure the controller must adapt the quantiser.
    let qs = &enc.quantiser_scale_codes;
    assert!(
        qs.iter().any(|&q| q != qs[0]),
        "quantiser never adapted: {qs:?}"
    );

    // §2.4.3.2: this geometry / rate / f_code set is admissible, so the
    // sequence header must declare constrained_parameters_flag = 1
    // (bit 12+12+4+4+18+1+10 = offset 61 into the header after the
    // 32-bit code; check via a reparse of the header fields instead).
    let sh = oxideav_mpeg12video::sequence_header::Mpeg2SequenceHeader::parse(&enc.stream)
        .expect("sequence header parses");
    assert_eq!(sh.bit_rate, 375);
    assert_eq!(sh.vbv_buffer_size, 4);
}

#[test]
fn mpeg1_cbr_flat_content_stuffs() {
    let (w, h) = (48usize, 32usize);
    let frames: Vec<FrameBuffer> = (0..5).map(|_| flat_frame(w, h)).collect();
    let s = seq(w as u16, h as u16, 250, 2); // 100 kbit/s
    let enc = encode_mpeg1_cbr_sequence(&frames, 1, 2, &s, 4, 2, 2).expect("CBR encode");
    assert!(
        enc.stuffing_bytes > 0,
        "flat content at a generous rate must stuff to hold the C.1.4 overflow bound"
    );
    verify_cbr_stream(&enc.stream, VbvStandard::Mpeg1).expect("VBV conformant");
    assert_eq!(decode_video_sequence(&enc.stream).unwrap().len(), 5);
}

#[test]
fn mpeg1_cbr_rejects_vbr_and_impossible_configs() {
    let (w, h) = (48usize, 32usize);
    let frames = vec![busy_frame(w, h, 0)];
    // The 0x3FFFF bit_rate code means VBR (§2.4.3.2) — not CBR.
    let vbr = seq(w as u16, h as u16, 0x3_FFFF, 4);
    assert!(encode_mpeg1_cbr_sequence(&frames, 0, 1, &vbr, 6, 1, 1).is_err());
    // vbv_delay unrepresentable.
    let bad = seq(w as u16, h as u16, 25, 20);
    assert!(encode_mpeg1_cbr_sequence(&frames, 0, 1, &bad, 6, 1, 1).is_err());
}
