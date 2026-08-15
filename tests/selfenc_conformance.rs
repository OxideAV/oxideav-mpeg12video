//! Self-encoded conformance corpus (`tests/fixtures/selfenc/`): the
//! encoder's output pinned bit-exactly, and its decodability by an
//! external black-box reference decoder pinned via the committed
//! `.ref.yuv` decodes. See the fixture README for the generation
//! record.
//!
//! Three assertions per stream:
//!
//! 1. **Bit-stability** — regenerating the stream from the same
//!    deterministic synthetic input reproduces the committed bytes
//!    exactly, so any encoder change that moves bits must refresh the
//!    fixture *and* re-run the black-box validation.
//! 2. **External conformance** — our decode of the committed stream
//!    matches the committed black-box reference decode within the
//!    corpus contract (|Δ| ≤ 3 per sample — Annex A IDCT rounding
//!    freedom — and < 5 % of samples differing; measured max |Δ| = 2).
//! 3. **Round-trip fidelity** — the decode approximates the original
//!    synthetic input (bounded luma MAE), so the corpus can't drift
//!    into "conformant garbage".

use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::vbv::{verify_cbr_stream, VbvStandard};
use oxideav_mpeg12video::{
    decode_video_sequence, encode_cbr_gop_sequence, encode_display_order_gop_sequence,
    encode_display_order_sequence, encode_ff_display_order_gop_sequence, encode_i_p_b,
    encode_i_p_chain, encode_intra_picture, encode_mpeg1_cbr_sequence, encode_mpeg1_d_sequence,
    encode_mpeg1_display_order_sequence, encode_mpeg1_intra_stream, CbrConfig, DecodedFrame,
    FrameBuffer, IntraPictureParams, Mpeg1SequenceParams,
};

const MAX_ABS_DELTA: i32 = 3;
const MAX_DIFF_PER_MILLE: u64 = 50;

/// Deterministic synthetic content — must stay in lock-step with
/// `examples/gen_selfenc_corpus.rs` (the generator that produced the
/// committed fixtures).
fn frame_at(width: usize, height: usize, dx: usize, dy: usize, stamp: bool) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            let sx = x + dx;
            let sy = y + dy;
            let g = 24 + ((sx * 3 + sy * 5) % 192);
            let c = if (sx / 4 + sy / 4) % 2 == 0 { 16 } else { 0 };
            f.y.put_sample(x, y, (g + c).min(235) as u8);
        }
    }
    if stamp {
        for y in 8..20.min(height) {
            for x in 8..20.min(width) {
                f.y.put_sample(x, y, if (x + y) % 2 == 0 { 16 } else { 235 });
            }
        }
    }
    for y in 0..height.div_ceil(2) {
        for x in 0..width.div_ceil(2) {
            f.cb.put_sample(x, y, (96 + (x + dx / 2 + y) % 64) as u8);
            f.cr.put_sample(x, y, (160u8).saturating_sub(((x + y + dy / 2) % 64) as u8));
        }
    }
    f
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
        progressive_sequence: true,
    }
}

fn fixture(name: &str) -> (Vec<u8>, Vec<u8>) {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/selfenc/");
    let stream = std::fs::read(format!("{dir}{name}")).expect("stream fixture present");
    let reference = std::fs::read(format!("{dir}{name}.ref.yuv")).expect("reference present");
    (stream, reference)
}

/// Pack a decoded frame's visible rectangle as planar 4:2:0 bytes —
/// the layout the black-box reference decode uses.
fn packed(frame: &DecodedFrame) -> Vec<u8> {
    let fb = &frame.frame;
    let (cw, ch) = fb.visible_chroma_dims();
    let mut out = fb.y.packed_rect(fb.width, fb.height);
    out.extend_from_slice(&fb.cb.packed_rect(cw, ch));
    out.extend_from_slice(&fb.cr.packed_rect(cw, ch));
    out
}

/// Assertions 2 + 3: decode `stream`, compare against the committed
/// black-box reference decode, and bound the luma MAE against the
/// original input frames (in display order).
fn assert_reference_conformant(
    name: &str,
    stream: &[u8],
    reference: &[u8],
    display_inputs: &[&FrameBuffer],
) {
    let frames = decode_video_sequence(stream).expect("self-encoded stream decodes");
    assert_eq!(frames.len(), display_inputs.len(), "{name}: frame count");

    let frame_bytes = reference.len() / display_inputs.len();
    for (index, frame) in frames.iter().enumerate() {
        let ours = packed(frame);
        assert_eq!(ours.len(), frame_bytes, "{name}: frame {index} size");
        let ref_frame = &reference[index * frame_bytes..(index + 1) * frame_bytes];

        let mut diff_count = 0u64;
        for (pos, (&a, &b)) in ours.iter().zip(ref_frame.iter()).enumerate() {
            let delta = (i32::from(a) - i32::from(b)).abs();
            if delta != 0 {
                diff_count += 1;
                assert!(
                    delta <= MAX_ABS_DELTA,
                    "{name}: frame {index} byte {pos}: |{a} - {b}| = {delta} exceeds the IDCT bound"
                );
            }
        }
        let per_mille = diff_count * 1000 / frame_bytes as u64;
        assert!(
            per_mille <= MAX_DIFF_PER_MILLE,
            "{name}: frame {index}: {per_mille}‰ samples differ — structural divergence"
        );

        // Round-trip fidelity: bounded mean absolute luma error
        // against the synthetic input this frame encodes.
        let input = display_inputs[index];
        let mut total = 0u64;
        let mut count = 0u64;
        for y in 0..input.height {
            for x in 0..input.width {
                let a = i64::from(input.y.get(x, y).unwrap());
                let b = i64::from(frame.frame.y.get(x, y).unwrap());
                total += a.abs_diff(b);
                count += 1;
            }
        }
        let mae = total as f64 / count as f64;
        assert!(
            mae < 8.0,
            "{name}: frame {index} luma MAE {mae:.2} — round-trip fidelity lost"
        );
    }
}

#[test]
fn selfenc_intra_64x48_is_pinned_and_reference_conformant() {
    let (stream, reference) = fixture("selfenc-intra-64x48.m2v");
    let input = frame_at(64, 48, 0, 0, false);
    let regenerated = encode_intra_picture(&input, params(64, 48), 0, 6).expect("intra re-encode");
    assert_eq!(
        regenerated, stream,
        "encoder output moved — refresh the fixture and re-run the black-box validation"
    );
    assert_reference_conformant("selfenc-intra-64x48", &stream, &reference, &[&input]);
}

#[test]
fn selfenc_intra_100x62_is_pinned_and_reference_conformant() {
    let (stream, reference) = fixture("selfenc-intra-100x62.m2v");
    let input = frame_at(100, 62, 0, 0, false);
    let regenerated = encode_intra_picture(&input, params(100, 62), 0, 5).expect("intra re-encode");
    assert_eq!(
        regenerated, stream,
        "encoder output moved — refresh the fixture and re-run the black-box validation"
    );
    assert_reference_conformant("selfenc-intra-100x62", &stream, &reference, &[&input]);
}

#[test]
fn selfenc_ip_chain_is_pinned_and_reference_conformant() {
    let (stream, reference) = fixture("selfenc-ipchain-64x48.m2v");
    let anchor = frame_at(64, 48, 0, 0, false);
    let targets = [
        frame_at(64, 48, 2, 1, false),
        frame_at(64, 48, 4, 2, true),
        frame_at(64, 48, 6, 3, true),
    ];
    let regenerated =
        encode_i_p_chain(&anchor, &targets, params(64, 48), 6, 3).expect("chain re-encode");
    assert_eq!(
        regenerated, stream,
        "encoder output moved — refresh the fixture and re-run the black-box validation"
    );
    assert_reference_conformant(
        "selfenc-ipchain-64x48",
        &stream,
        &reference,
        &[&anchor, &targets[0], &targets[1], &targets[2]],
    );
}

#[test]
fn selfenc_ibbp_sequence_is_pinned_and_reference_conformant() {
    let (stream, reference) = fixture("selfenc-ibbp-64x48.m2v");
    let display: Vec<FrameBuffer> = (0..7).map(|k| frame_at(64, 48, 2 * k, k, k == 3)).collect();
    let regenerated = encode_display_order_sequence(&display, 2, params(64, 48), 6, 3, 3)
        .expect("ibbp re-encode");
    assert_eq!(
        regenerated, stream,
        "encoder output moved — refresh the fixture and re-run the black-box validation"
    );
    let inputs: Vec<&FrameBuffer> = display.iter().collect();
    assert_reference_conformant("selfenc-ibbp-64x48", &stream, &reference, &inputs);
}

#[test]
fn selfenc_mpeg2_gop_sequence_is_pinned_and_reference_conformant() {
    let (stream, reference) = fixture("selfenc-gops-48x32.m2v");
    let display: Vec<FrameBuffer> = (0..8).map(|k| frame_at(48, 32, 2 * k, k, false)).collect();
    let regenerated = encode_display_order_gop_sequence(&display, 1, 2, params(48, 32), 6, 3, 3)
        .expect("gop re-encode");
    assert_eq!(
        regenerated, stream,
        "encoder output moved — refresh the fixture and re-run the black-box validation"
    );
    let inputs: Vec<&FrameBuffer> = display.iter().collect();
    assert_reference_conformant("selfenc-gops-48x32", &stream, &reference, &inputs);
}

fn mpeg1_seq(width: u16, height: u16) -> Mpeg1SequenceParams {
    Mpeg1SequenceParams {
        horizontal_size: width,
        vertical_size: height,
        ..Default::default()
    }
}

#[test]
fn selfenc_mpeg1_intra_is_pinned_and_reference_conformant() {
    let (stream, reference) = fixture("selfenc-mpeg1-intra-64x48.m1v");
    let input = frame_at(64, 48, 0, 0, false);
    let regenerated =
        encode_mpeg1_intra_stream(&input, &mpeg1_seq(64, 48), 6).expect("mpeg1 intra re-encode");
    assert_eq!(
        regenerated, stream,
        "encoder output moved — refresh the fixture and re-run the black-box validation"
    );
    assert_reference_conformant("selfenc-mpeg1-intra-64x48", &stream, &reference, &[&input]);
}

#[test]
fn selfenc_mpeg1_ippp_chain_is_pinned_and_reference_conformant() {
    let (stream, reference) = fixture("selfenc-mpeg1-ippp-64x48.m1v");
    let display = [
        frame_at(64, 48, 0, 0, false),
        frame_at(64, 48, 2, 1, false),
        frame_at(64, 48, 4, 2, true),
        frame_at(64, 48, 6, 3, true),
    ];
    let regenerated =
        encode_mpeg1_display_order_sequence(&display, 0, 3, &mpeg1_seq(64, 48), 6, 3, 3)
            .expect("mpeg1 ippp re-encode");
    assert_eq!(
        regenerated, stream,
        "encoder output moved — refresh the fixture and re-run the black-box validation"
    );
    let inputs: Vec<&FrameBuffer> = display.iter().collect();
    assert_reference_conformant("selfenc-mpeg1-ippp-64x48", &stream, &reference, &inputs);
}

#[test]
fn selfenc_mpeg1_two_gop_ibbp_is_pinned_and_reference_conformant() {
    let (stream, reference) = fixture("selfenc-mpeg1-ibbp2gop-64x48.m1v");
    let display: Vec<FrameBuffer> = (0..8).map(|k| frame_at(64, 48, 2 * k, k, k == 5)).collect();
    let regenerated =
        encode_mpeg1_display_order_sequence(&display, 2, 1, &mpeg1_seq(64, 48), 6, 3, 3)
            .expect("mpeg1 ibbp re-encode");
    assert_eq!(
        regenerated, stream,
        "encoder output moved — refresh the fixture and re-run the black-box validation"
    );
    let inputs: Vec<&FrameBuffer> = display.iter().collect();
    assert_reference_conformant("selfenc-mpeg1-ibbp2gop-64x48", &stream, &reference, &inputs);
}

#[test]
fn selfenc_mpeg2_cbr_is_pinned_reference_and_vbv_conformant() {
    let (stream, reference) = fixture("selfenc-cbr-64x48.m2v");
    let display: Vec<FrameBuffer> = (0..8).map(|k| frame_at(64, 48, 2 * k, k, k == 4)).collect();
    let cbr = CbrConfig {
        bit_rate_value: 600,
        vbv_buffer_size_value: 4,
        frame_rate_code: 3,
        initial_quantiser_scale_code: 6,
    };
    let regenerated =
        encode_cbr_gop_sequence(&display, 1, 2, params(64, 48), &cbr, 3, 3).expect("cbr re-encode");
    assert_eq!(
        regenerated.stream, stream,
        "encoder output moved — refresh the fixture and re-run the black-box validation"
    );
    // Annex C: the committed stream satisfies the bit_rate /
    // vbv_buffer_size it declares, with C.3.1-consistent vbv_delay in
    // every picture header.
    let report = verify_cbr_stream(&stream, VbvStandard::Mpeg2).expect("VBV conformant");
    assert_eq!(report.bit_rate, 240_000);
    assert_eq!(report.buffer_size_bits, 65_536);
    assert_eq!(report.pictures.len(), 8);
    let inputs: Vec<&FrameBuffer> = display.iter().collect();
    assert_reference_conformant("selfenc-cbr-64x48", &stream, &reference, &inputs);
}

#[test]
fn selfenc_mpeg1_cbr_is_pinned_reference_and_vbv_conformant() {
    let (stream, reference) = fixture("selfenc-mpeg1-cbr-64x48.m1v");
    let display: Vec<FrameBuffer> = (0..8).map(|k| frame_at(64, 48, 2 * k, k, k == 5)).collect();
    let seq_cbr = Mpeg1SequenceParams {
        bit_rate_value: 600,
        vbv_buffer_size_value: 4,
        ..mpeg1_seq(64, 48)
    };
    let regenerated =
        encode_mpeg1_cbr_sequence(&display, 2, 1, &seq_cbr, 6, 3, 3).expect("mpeg1 cbr re-encode");
    assert_eq!(
        regenerated.stream, stream,
        "encoder output moved — refresh the fixture and re-run the black-box validation"
    );
    let report = verify_cbr_stream(&stream, VbvStandard::Mpeg1).expect("VBV conformant");
    assert_eq!(report.bit_rate, 240_000);
    assert_eq!(report.buffer_size_bits, 65_536);
    assert_eq!(report.pictures.len(), 8);
    let inputs: Vec<&FrameBuffer> = display.iter().collect();
    assert_reference_conformant("selfenc-mpeg1-cbr-64x48", &stream, &reference, &inputs);
}

/// The interlaced field-sequence fixture's synthetic content — in
/// lock-step with `examples/gen_selfenc_corpus.rs` stream 13.
fn field_frame_at(t: usize) -> FrameBuffer {
    let (w, h) = (48usize, 64usize);
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
}

#[test]
fn selfenc_field_sequence_is_pinned_and_reference_conformant() {
    let (stream, reference) = fixture("selfenc-fieldseq-48x64.m2v");
    let display: Vec<FrameBuffer> = (0..5).map(field_frame_at).collect();
    let field_params = IntraPictureParams {
        width: 48,
        height: 64,
        chroma_format: ChromaFormat::Yuv420,
        frame_pred_frame_dct: false,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        progressive_sequence: false,
    };
    let regenerated = oxideav_mpeg12video::encode_field_display_order_gop_sequence(
        &display,
        1,
        2,
        &field_params,
        6,
        3,
        3,
    )
    .expect("field sequence re-encode");
    assert_eq!(
        regenerated, stream,
        "encoder output moved — refresh the fixture and re-run the black-box validation"
    );
    let inputs: Vec<&FrameBuffer> = display.iter().collect();
    assert_reference_conformant("selfenc-fieldseq-48x64", &stream, &reference, &inputs);
}

/// The frame-picture field-based parameters — in lock-step with
/// `examples/gen_selfenc_corpus.rs` streams 14–15.
fn ff_params_64() -> IntraPictureParams {
    IntraPictureParams {
        width: 64,
        height: 64,
        chroma_format: ChromaFormat::Yuv420,
        frame_pred_frame_dct: false,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        progressive_sequence: false,
    }
}

/// Stream 14's synthetic content: per-frame opposite-direction field
/// pans with an alternating-field brightness offset.
fn ff_frame_at(t: usize) -> FrameBuffer {
    let (w, h) = (64usize, 64usize);
    let mut f = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
    for y in 0..h {
        let dx = if y % 2 == 0 {
            2 * t as i32
        } else {
            -2 * (t as i32)
        };
        for x in 0..w {
            let sx = (x as i32 - dx).rem_euclid(w as i32) as usize;
            let v = 40 + ((sx * 5 + (y / 2) * 9) % 160);
            let line = if y % 2 == 0 { 10 } else { 0 };
            f.y.put_sample(x, y, (v + line).min(235) as u8);
        }
    }
    for y in 0..h / 2 {
        for x in 0..w / 2 {
            f.cb.put_sample(x, y, (100 + (x + 2 * t) % 72) as u8);
            f.cr.put_sample(x, y, (180u8).saturating_sub(((y + 3 * t) % 72) as u8));
        }
    }
    f
}

#[test]
fn selfenc_frame_field_sequence_is_pinned_and_reference_conformant() {
    let (stream, reference) = fixture("selfenc-framefield-64x64.m2v");
    let display: Vec<FrameBuffer> = (0..5).map(ff_frame_at).collect();
    let (regenerated, stats) =
        encode_ff_display_order_gop_sequence(&display, 1, 2, &ff_params_64(), 6, 3, 3, false)
            .expect("frame-field re-encode");
    assert_eq!(
        regenerated, stream,
        "encoder output moved — refresh the fixture and re-run the black-box validation"
    );
    // The stream genuinely exercises the frame_pred_frame_dct = 0
    // surface: field-based macroblocks and field-DCT macroblocks.
    assert!(stats.field_mc > 0, "field MC coded: {stats:?}");
    assert!(stats.field_dct > 0, "field DCT coded: {stats:?}");
    let inputs: Vec<&FrameBuffer> = display.iter().collect();
    assert_reference_conformant("selfenc-framefield-64x64", &stream, &reference, &inputs);
}

/// Stream 15's synthetic content: column-constant base, per-sample
/// noise on the I reference only.
fn dp_frame_at(t: usize) -> FrameBuffer {
    let noise = |x: usize, y: usize, seed: usize| -> i32 {
        let h = x
            .wrapping_mul(31)
            .wrapping_add(y.wrapping_mul(97))
            .wrapping_add(seed.wrapping_mul(131));
        ((h % 17) as i32) - 8
    };
    let (w, h) = (64usize, 64usize);
    let mut f = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
    for y in 0..h {
        for x in 0..w {
            let base = 90 + ((x * 7) % 100) as i32;
            let v = if t == 0 { base + noise(x, y, 1) } else { base };
            f.y.put_sample(x, y, v.clamp(0, 255) as u8);
        }
    }
    for y in 0..h / 2 {
        for x in 0..w / 2 {
            f.cb.put_sample(x, y, 128);
            f.cr.put_sample(x, y, 128);
        }
    }
    f
}

#[test]
fn selfenc_dual_prime_sequence_is_pinned_and_reference_conformant() {
    let (stream, reference) = fixture("selfenc-dualprime-64x64.m2v");
    let display: Vec<FrameBuffer> = (0..3).map(dp_frame_at).collect();
    let (regenerated, stats) =
        encode_ff_display_order_gop_sequence(&display, 0, 2, &ff_params_64(), 6, 3, 3, true)
            .expect("dual-prime re-encode");
    assert_eq!(
        regenerated, stream,
        "encoder output moved — refresh the fixture and re-run the black-box validation"
    );
    assert!(stats.dual_prime > 0, "dual-prime coded: {stats:?}");
    let inputs: Vec<&FrameBuffer> = display.iter().collect();
    assert_reference_conformant("selfenc-dualprime-64x64", &stream, &reference, &inputs);
}

/// Stream 16's synthetic content — the D-picture staircase.
fn d_frame_at(t: usize) -> FrameBuffer {
    let (w, h) = (48usize, 32usize);
    let mut f = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
    for y in 0..h {
        for x in 0..w {
            let mb = (y / 16) * w.div_ceil(16) + x / 16;
            let v = 40 + 23 * (mb % 8) + 7 * t + (x + y) % 5;
            f.y.put_sample(x, y, v.min(235) as u8);
        }
    }
    for y in 0..h / 2 {
        for x in 0..w / 2 {
            f.cb.put_sample(x, y, (90 + x + 3 * t).min(240) as u8);
            f.cr.put_sample(x, y, (170usize.saturating_sub(y + 2 * t)).max(16) as u8);
        }
    }
    f
}

#[test]
fn selfenc_mpeg1_d_sequence_is_pinned_and_self_conformant() {
    // No black-box reference decode exists for picture_coding_type 4
    // (the reference binary emits zero frames — the same limitation
    // recorded for the mpeg1-dpics conformance fixture), so this
    // stream is pinned bit-exactly and its decode held sample-exact
    // against the encoder's own §2.4.4.1 reconstruction.
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/selfenc/");
    let stream =
        std::fs::read(format!("{dir}selfenc-mpeg1-dpics-48x32.m1v")).expect("fixture present");
    let display: Vec<FrameBuffer> = (0..4).map(d_frame_at).collect();
    let regenerated =
        encode_mpeg1_d_sequence(&display, &mpeg1_seq(48, 32), 8, 2).expect("mpeg1 d re-encode");
    assert_eq!(
        regenerated, stream,
        "encoder output moved — refresh the fixture"
    );

    let frames = decode_video_sequence(&stream).expect("D stream decodes");
    assert_eq!(frames.len(), 4);
    for (i, (decoded, input)) in frames.iter().zip(display.iter()).enumerate() {
        // DC-only coding: each 8x8 block is flat at its quantised
        // mean; the staircase content is flat per block, so the
        // decode stays within DC quantisation of the input.
        let mut max_err = 0i64;
        for y in 0..input.height {
            for x in 0..input.width {
                let a = i64::from(input.y.get(x, y).unwrap());
                let b = i64::from(decoded.frame.y.get(x, y).unwrap());
                max_err = max_err.max((a - b).abs());
            }
        }
        assert!(max_err <= 4, "D frame {i} luma max err {max_err}");
    }
}

#[test]
fn selfenc_mpeg1_loaded_matrices_is_pinned_and_reference_conformant() {
    let (stream, reference) = fixture("selfenc-mpeg1-qmat-48x32.m1v");
    let mut intra = [8u8; 64];
    for (i, v) in intra.iter_mut().enumerate().skip(1) {
        *v = 12 + (i as u8 % 8);
    }
    let seq_qmat = Mpeg1SequenceParams {
        intra_quant_matrix: Some(intra),
        non_intra_quant_matrix: Some([20u8; 64]),
        ..mpeg1_seq(48, 32)
    };
    let display: Vec<FrameBuffer> = (0..3).map(|k| frame_at(48, 32, 2 * k, k, false)).collect();
    let regenerated = encode_mpeg1_display_order_sequence(&display, 1, 1, &seq_qmat, 6, 3, 3)
        .expect("mpeg1 qmat re-encode");
    assert_eq!(
        regenerated, stream,
        "encoder output moved — refresh the fixture and re-run the black-box validation"
    );
    let inputs: Vec<&FrameBuffer> = display.iter().collect();
    assert_reference_conformant("selfenc-mpeg1-qmat-48x32", &stream, &reference, &inputs);
}

#[test]
fn selfenc_ipb_group_is_pinned_and_reference_conformant() {
    let (stream, reference) = fixture("selfenc-ipb-64x48.m2v");
    let i_frame = frame_at(64, 48, 0, 0, false);
    let b_frame = frame_at(64, 48, 2, 1, false);
    let p_frame = frame_at(64, 48, 4, 2, false);
    let regenerated = encode_i_p_b(&i_frame, &b_frame, &p_frame, params(64, 48), 6, 3, 3)
        .expect("i-p-b re-encode");
    assert_eq!(
        regenerated, stream,
        "encoder output moved — refresh the fixture and re-run the black-box validation"
    );
    // Display order: I, B, P.
    assert_reference_conformant(
        "selfenc-ipb-64x48",
        &stream,
        &reference,
        &[&i_frame, &b_frame, &p_frame],
    );
}
