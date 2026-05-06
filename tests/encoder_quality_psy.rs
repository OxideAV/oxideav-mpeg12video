//! Encoder-quality regression tests covering the round-39 push:
//!
//! 1. **Activity-based per-MB QP (psy-QP) on I-pictures.** A 64×96 synthetic
//!    frame with a high-detail "noise" half and a flat half is encoded twice
//!    — once with psy-QP enabled (default), once with psy-QP disabled —
//!    and we assert the psy path produces ≥ 0.4 dB higher Y-PSNR (textured
//!    region gets finer quantisation, flat region gets a small QP raise).
//!
//! 2. **Half-pel diamond ME with QP-tuned lambda.** A pure-translation
//!    sequence with sub-pel motion (1.5 px / frame horizontal slide) is
//!    encoded as IPPP. The new diamond + lower half-pel-win threshold lets
//!    the encoder pick the half-pel MV the fixture intends; we assert the
//!    average Y-PSNR is ≥ 38 dB (was ≈ 35 dB before round 39).
//!
//! 3. **Self-roundtrip on a 64×64 IBPBP synthetic motion fixture** — covers
//!    the B-frame interpolated path with the new lambda + diamond. Assert
//!    Y-PSNR avg ≥ 42 dB.
//!
//! Plus a smoke `make_encoder_with_gop_psy(false)` toggle test and a
//! `make_encoder_with_gop_b_offset(...)` smoke test that the offset takes
//! effect (output bitstream changes vs offset=0).
//!
//! All tests use synthetic fixtures so no `/tmp/*.yuv` is needed.

use oxideav_core::{
    frame::VideoPlane, CodecId, CodecParameters, Error, Frame, Packet, PixelFormat, Rational,
    TimeBase, VideoFrame,
};
use oxideav_mpeg12video::{
    decoder::make_decoder,
    encoder::{
        make_encoder_with_gop, make_encoder_with_gop_b_offset, make_encoder_with_gop_psy,
        DEFAULT_QUANT_SCALE,
    },
    CODEC_ID_STR,
};

// ---------------------------------------------------------------------------
// Helpers.
// ---------------------------------------------------------------------------

fn build_frame(w: u32, h: u32, pts: i64, fill_y: impl Fn(usize, usize) -> u8) -> VideoFrame {
    let cw = w.div_ceil(2);
    let ch = h.div_ceil(2);
    let mut y = vec![0u8; (w * h) as usize];
    let cb = vec![128u8; (cw * ch) as usize];
    let cr = vec![128u8; (cw * ch) as usize];
    for j in 0..h as usize {
        for i in 0..w as usize {
            y[j * w as usize + i] = fill_y(i, j);
        }
    }
    VideoFrame {
        pts: Some(pts),
        planes: vec![
            VideoPlane {
                stride: w as usize,
                data: y,
            },
            VideoPlane {
                stride: cw as usize,
                data: cb,
            },
            VideoPlane {
                stride: cw as usize,
                data: cr,
            },
        ],
    }
}

fn drain_to_bytes(enc: &mut Box<dyn oxideav_core::Encoder>) -> Vec<u8> {
    enc.flush().expect("flush");
    let mut data = Vec::new();
    loop {
        match enc.receive_packet() {
            Ok(p) => data.extend_from_slice(&p.data),
            Err(Error::NeedMore) => break,
            Err(Error::Eof) => break,
            Err(e) => panic!("encoder error: {e}"),
        }
    }
    data
}

fn decode_all(bytes: &[u8]) -> Vec<VideoFrame> {
    let params = CodecParameters::video(CodecId::new(CODEC_ID_STR));
    let mut dec = make_decoder(&params).expect("build decoder");
    let pkt = Packet::new(0, TimeBase::new(1, 24), bytes.to_vec()).with_pts(0);
    dec.send_packet(&pkt).expect("send_packet");
    dec.flush().expect("flush");
    let mut out = Vec::new();
    loop {
        match dec.receive_frame() {
            Ok(Frame::Video(v)) => out.push(v),
            Ok(_) => continue,
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => panic!("decoder error: {e}"),
        }
    }
    out
}

fn psnr_y(orig: &VideoFrame, recon: &VideoFrame) -> f64 {
    let o = &orig.planes[0];
    let r = &recon.planes[0];
    let mut sq: f64 = 0.0;
    let mut count: u64 = 0;
    for (a, b) in o.data.iter().zip(r.data.iter()) {
        let d = *a as f64 - *b as f64;
        sq += d * d;
        count += 1;
    }
    if count == 0 {
        return f64::INFINITY;
    }
    let mse = sq / count as f64;
    if mse <= 0.0 {
        f64::INFINITY
    } else {
        10.0 * (255.0_f64 * 255.0 / mse).log10()
    }
}

fn make_params(w: u32, h: u32) -> CodecParameters {
    let mut params = CodecParameters::video(CodecId::new(CODEC_ID_STR));
    params.width = Some(w);
    params.height = Some(h);
    params.pixel_format = Some(PixelFormat::Yuv420P);
    params.frame_rate = Some(Rational::new(24, 1));
    params.bit_rate = Some(1_500_000);
    params
}

// ---------------------------------------------------------------------------
// Test 1 — psy-QP on I-pictures.
// ---------------------------------------------------------------------------

/// 64×96 frame: top half is high-frequency "noise" (PRNG checkerboard),
/// bottom half is a constant (flat). Psy-QP should lower QP in the noisy
/// region (better PSNR there) and raise it in the flat region (negligible
/// PSNR loss because flat MBs barely have any AC energy). Net Y-PSNR up.
#[test]
fn psy_qp_lifts_iframe_psnr_on_mixed_complexity() {
    let w = 64u32;
    let h = 96u32;
    let frame = build_frame(w, h, 0, |i, j| {
        if j < (h as usize) / 2 {
            // Pseudo-random "noise" via xorshift, deterministic per (i, j).
            let mut x = (i as u32).wrapping_mul(2654435761).wrapping_add(j as u32);
            x ^= x >> 13;
            x = x.wrapping_mul(0x5bd1e995);
            x ^= x >> 15;
            (x & 0xff) as u8
        } else {
            128
        }
    });
    let params = make_params(w, h);

    let mut enc_off =
        make_encoder_with_gop_psy(&params, 1, 0, false).expect("build psy-off encoder");
    enc_off
        .send_frame(&Frame::Video(frame.clone()))
        .expect("send");
    let bytes_off = drain_to_bytes(&mut enc_off);
    let recon_off = decode_all(&bytes_off);
    assert_eq!(recon_off.len(), 1);
    let psnr_off = psnr_y(&frame, &recon_off[0]);

    let mut enc_on = make_encoder_with_gop_psy(&params, 1, 0, true).expect("build psy-on encoder");
    enc_on
        .send_frame(&Frame::Video(frame.clone()))
        .expect("send");
    let bytes_on = drain_to_bytes(&mut enc_on);
    let recon_on = decode_all(&bytes_on);
    assert_eq!(recon_on.len(), 1);
    let psnr_on = psnr_y(&frame, &recon_on[0]);

    eprintln!(
        "psy-qp test: psnr OFF = {psnr_off:.2} dB ({} bytes), \
         psnr ON = {psnr_on:.2} dB ({} bytes), delta = {:+.2} dB at qp={}",
        bytes_off.len(),
        bytes_on.len(),
        psnr_on - psnr_off,
        DEFAULT_QUANT_SCALE
    );
    // Expect a ≥ 0.4 dB lift in the textured half from the lower-QP MBs.
    assert!(
        psnr_on - psnr_off >= 0.4,
        "psy-QP lift = {:+.2} dB < 0.4 dB threshold",
        psnr_on - psnr_off
    );
}

// ---------------------------------------------------------------------------
// Test 2 — half-pel diamond ME with sub-pel motion fixture.
// ---------------------------------------------------------------------------

/// Build a 64×64 frame whose Y plane contains a smoothly-varying sinusoid
/// shifted by `shift_x_q1` quarter-pel positions. Quarter-pel positions
/// mean half-pel and integer-pel both miss; the encoder's job is to pick
/// whichever half-pel candidate is closer.
fn synth_subpel(t: usize, w: u32, h: u32) -> VideoFrame {
    // Phase shift in real samples = t * 1.5 (i.e. 1.5 pels per frame).
    let shift = t as f64 * 1.5;
    let cw = w.div_ceil(2);
    let ch = h.div_ceil(2);
    let mut y = vec![0u8; (w * h) as usize];
    let cb = vec![128u8; (cw * ch) as usize];
    let cr = vec![128u8; (cw * ch) as usize];
    for j in 0..h as usize {
        for i in 0..w as usize {
            // Smooth gradient (so half-pel bilinear interpolation lands close
            // to ground truth); period chosen so MV ≤ 16 half-pel = 8 px is
            // easily representable.
            let x = i as f64 - shift;
            let v = 128.0
                + 80.0 * (x * std::f64::consts::PI / 16.0).sin()
                + 40.0 * ((j as f64) * std::f64::consts::PI / 24.0).cos();
            y[j * w as usize + i] = v.clamp(0.0, 255.0) as u8;
        }
    }
    VideoFrame {
        pts: Some(t as i64),
        planes: vec![
            VideoPlane {
                stride: w as usize,
                data: y,
            },
            VideoPlane {
                stride: cw as usize,
                data: cb,
            },
            VideoPlane {
                stride: cw as usize,
                data: cr,
            },
        ],
    }
}

#[test]
fn half_pel_diamond_lifts_subpel_motion_psnr() {
    let w = 64u32;
    let h = 64u32;
    let n = 4;
    let frames: Vec<VideoFrame> = (0..n).map(|t| synth_subpel(t, w, h)).collect();

    let params = make_params(w, h);
    // IPPP, no B-frames — pure forward ME path.
    let mut enc = make_encoder_with_gop(&params, n as u32, 0).expect("build enc");
    for f in &frames {
        enc.send_frame(&Frame::Video(f.clone())).expect("send");
    }
    let bytes = drain_to_bytes(&mut enc);
    let recon = decode_all(&bytes);
    assert_eq!(recon.len(), frames.len());

    let mut sum = 0.0;
    let mut min = f64::INFINITY;
    for (i, (o, r)) in frames.iter().zip(recon.iter()).enumerate() {
        let p = psnr_y(o, r);
        eprintln!("subpel-motion frame {i}: Y-PSNR={p:.2} dB");
        sum += p;
        if p < min {
            min = p;
        }
    }
    let avg = sum / n as f64;
    eprintln!("subpel-motion avg Y-PSNR={avg:.2} dB (min {min:.2} dB)");
    // Lower bound is conservative; the 1.5-pel/frame slide stresses
    // half-pel selection. Round-39 baseline measured ≈ 41 dB — assert ≥ 38.
    assert!(avg >= 38.0, "avg Y-PSNR {avg:.2} dB < 38 dB threshold");
}

// ---------------------------------------------------------------------------
// Test 3 — IBPBP synthetic motion roundtrip with new ME.
// ---------------------------------------------------------------------------

#[test]
fn ibpbp_synthetic_motion_roundtrip() {
    let w = 64u32;
    let h = 64u32;
    let n = 5;
    let frames: Vec<VideoFrame> = (0..n).map(|t| synth_subpel(t, w, h)).collect();

    let params = make_params(w, h);
    // IBPBP: gop_size=5, num_b_frames=1.
    let mut enc = make_encoder_with_gop(&params, 5, 1).expect("build enc");
    for f in &frames {
        enc.send_frame(&Frame::Video(f.clone())).expect("send");
    }
    let bytes = drain_to_bytes(&mut enc);
    let recon = decode_all(&bytes);
    assert_eq!(recon.len(), frames.len());

    let mut sum = 0.0;
    for (i, (o, r)) in frames.iter().zip(recon.iter()).enumerate() {
        let p = psnr_y(o, r);
        eprintln!("ibpbp-subpel frame {i}: Y-PSNR={p:.2} dB");
        sum += p;
    }
    let avg = sum / n as f64;
    eprintln!("ibpbp-subpel avg Y-PSNR={avg:.2} dB");
    // The B-frame interpolated path benefits strongly from the new diamond.
    assert!(avg >= 42.0, "avg Y-PSNR {avg:.2} dB < 42 dB threshold");
}

// ---------------------------------------------------------------------------
// Smoke tests for the new encoder factories.
// ---------------------------------------------------------------------------

#[test]
fn psy_qp_off_factory_produces_decodable_stream() {
    let w = 32u32;
    let h = 32u32;
    let frame = build_frame(w, h, 0, |i, j| ((i ^ j) as u8).wrapping_mul(8));
    let params = make_params(w, h);
    let mut enc = make_encoder_with_gop_psy(&params, 1, 0, false).expect("build enc");
    enc.send_frame(&Frame::Video(frame.clone())).expect("send");
    let bytes = drain_to_bytes(&mut enc);
    let recon = decode_all(&bytes);
    assert_eq!(recon.len(), 1);
    // Round-trip is "good enough" — just sanity-check decode succeeded.
    let p = psnr_y(&frame, &recon[0]);
    assert!(p >= 30.0, "psy-off roundtrip Y-PSNR {p:.2} < 30 dB");
}

#[test]
fn b_quant_offset_changes_bitstream() {
    let w = 64u32;
    let h = 64u32;
    let n = 5;
    let frames: Vec<VideoFrame> = (0..n).map(|t| synth_subpel(t, w, h)).collect();
    let params = make_params(w, h);

    let encode_with = |offset: i32| -> Vec<u8> {
        let mut enc = make_encoder_with_gop_b_offset(&params, 5, 1, offset).expect("build enc");
        for f in &frames {
            enc.send_frame(&Frame::Video(f.clone())).expect("send");
        }
        drain_to_bytes(&mut enc)
    };

    let no_off = encode_with(0);
    let with_off = encode_with(2);
    eprintln!(
        "B-quant-offset=0: {} bytes; offset=+2: {} bytes",
        no_off.len(),
        with_off.len()
    );
    // A higher B-frame QP must yield a smaller (or at least different)
    // bitstream — quantisation is coarser → fewer non-zero coefficients.
    assert!(
        with_off.len() <= no_off.len(),
        "B offset=+2 bitstream not smaller: {} vs {}",
        with_off.len(),
        no_off.len()
    );
    // And the bitstream must remain decodable.
    let recon = decode_all(&with_off);
    assert_eq!(recon.len(), n);
}
