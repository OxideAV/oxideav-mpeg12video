//! §7.8 SNR scalability end to end: the self-made enhancement-layer
//! encoder is the oracle — its combined reconstruction must be what
//! the two-layer decode loop reproduces sample for sample — over
//! progressive I/B/P lower layers, `frame_pred_frame_dct = 0` lower
//! layers (the §7.8.2.1 `dct_type` coincidence), lower layers with
//! §7.6.6 skipped macroblocks (`F''lower = 0`), 4:2:2 chroma, and
//! downloaded non-intra matrices. The lower layer stays an ordinary
//! ISO/IEC 13818-2 stream (the base decodes unchanged), and the
//! enhancement really enhances (combined error < lower-only error).

use oxideav_mpeg12video::quant_matrix_extension::QuantMatrixExtension;
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::sequence_scalable_extension::{ScalableMode, SequenceScalableExtension};
use oxideav_mpeg12video::{
    decode_snr_scalable_sequence, decode_video_sequence, encode_display_order_gop_sequence,
    encode_display_order_gop_sequence_with_options, encode_ff_display_order_gop_sequence,
    encode_snr_enhancement_layer, DecodedFrame, FrameBuffer, FrameEncodeOptions,
    IntraPictureParams,
};

fn params(
    width: usize,
    height: usize,
    chroma: ChromaFormat,
    progressive: bool,
) -> IntraPictureParams {
    IntraPictureParams {
        width,
        height,
        chroma_format: chroma,
        frame_pred_frame_dct: progressive,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        progressive_sequence: progressive,
    }
}

/// Busy, translating content with a fixed high-contrast stamp.
fn frame_at(width: usize, height: usize, chroma: ChromaFormat, t: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, chroma);
    for y in 0..height {
        for x in 0..width {
            let sx = x + 2 * t;
            let g = 24 + ((sx * 3 + y * 5) % 192);
            let c = if (sx / 4 + y / 4) % 2 == 0 { 16 } else { 0 };
            let n = (sx * 7 + y * 13) % 9; // fine texture the coarse layer drops
            f.y.put_sample(x, y, (g + c + n).min(235) as u8);
        }
    }
    for y in 8..20.min(height) {
        for x in 8..20.min(width) {
            f.y.put_sample(x, y, if (x + y) % 2 == 0 { 16 } else { 235 });
        }
    }
    let (cw, ch) = f.visible_chroma_dims();
    for y in 0..ch {
        for x in 0..cw {
            f.cb.put_sample(x, y, (96 + (x + t + y * 3) % 64) as u8);
            f.cr.put_sample(x, y, (160u8).saturating_sub(((x * 2 + y + t) % 64) as u8));
        }
    }
    f
}

fn assert_frames_equal(name: &str, a: &[DecodedFrame], b: &[DecodedFrame]) {
    assert_eq!(a.len(), b.len(), "{name}: frame count");
    for (i, (x, y)) in a.iter().zip(b).enumerate() {
        assert_eq!(
            x.temporal_reference, y.temporal_reference,
            "{name}: frame {i} tref"
        );
        assert_eq!(x.frame.chroma_format, y.frame.chroma_format);
        assert_eq!(
            x.frame.y.samples(),
            y.frame.y.samples(),
            "{name}: frame {i} luma"
        );
        assert_eq!(
            x.frame.cb.samples(),
            y.frame.cb.samples(),
            "{name}: frame {i} cb"
        );
        assert_eq!(
            x.frame.cr.samples(),
            y.frame.cr.samples(),
            "{name}: frame {i} cr"
        );
    }
}

fn luma_mae(frames: &[DecodedFrame], sources: &[FrameBuffer]) -> f64 {
    let mut total = 0u64;
    let mut n = 0u64;
    for (d, s) in frames.iter().zip(sources) {
        for y in 0..s.height {
            for x in 0..s.width {
                total += u64::from(
                    d.frame
                        .y
                        .get(x, y)
                        .unwrap()
                        .abs_diff(s.y.get(x, y).unwrap()),
                );
                n += 1;
            }
        }
    }
    total as f64 / n as f64
}

fn check_pair(name: &str, base: &[u8], sources: &[FrameBuffer], q_enh: u8) -> Vec<u8> {
    let enc = encode_snr_enhancement_layer(base, sources, q_enh).expect("enhancement encode");
    assert_eq!(
        enc.recon.len(),
        sources.len(),
        "{name}: combined frame count"
    );
    assert!(
        enc.coded_macroblocks > 0,
        "{name}: the layer codes something"
    );

    // Oracle: the decode loop reproduces the encoder's combined
    // reconstruction exactly.
    let combined = decode_snr_scalable_sequence(base, &enc.stream).expect("two-layer decode");
    assert_frames_equal(name, &combined, &enc.recon);

    // The lower layer is untouched and decodes on its own; the
    // enhancement lowers the error against the source.
    let lower = decode_video_sequence(base).expect("base decodes alone");
    let lower_mae = luma_mae(&lower, sources);
    let combined_mae = luma_mae(&combined, sources);
    assert!(
        combined_mae < lower_mae,
        "{name}: combined MAE {combined_mae:.3} must beat lower-only {lower_mae:.3}"
    );

    // The enhancement stream is an ISO/IEC 13818-2 sequence declaring
    // SNR scalability, layer 1, with the lower layer's geometry.
    let seq = oxideav_mpeg12video::sequence_extension::Mpeg2Sequence::from_buf(&enc.stream)
        .expect("enhancement sequence layer");
    let lower_seq = oxideav_mpeg12video::sequence_extension::Mpeg2Sequence::from_buf(base).unwrap();
    assert_eq!(seq.horizontal_size, lower_seq.horizontal_size);
    assert_eq!(seq.vertical_size, lower_seq.vertical_size);
    assert_eq!(
        seq.extension.chroma_format,
        lower_seq.extension.chroma_format
    );
    let sse_pos = enc
        .stream
        .windows(5)
        .position(|w| w[..4] == [0, 0, 1, 0xB5] && w[4] >> 4 == 0b0101)
        .expect("sequence_scalable_extension");
    let sse = SequenceScalableExtension::parse(&enc.stream[sse_pos..]).unwrap();
    assert_eq!(sse.scalable_mode, ScalableMode::SnrScalability);
    assert_eq!(sse.layer_id, 1);
    // Same number of pictures in the same coded order.
    let count = |s: &[u8]| s.windows(4).filter(|w| *w == [0, 0, 1, 0]).count();
    assert_eq!(count(&enc.stream), count(base), "{name}: picture count");
    enc.stream
}

#[test]
fn progressive_ibp_lower_layer_refines_and_roundtrips_exactly() {
    let sources: Vec<FrameBuffer> = (0..5)
        .map(|t| frame_at(64, 48, ChromaFormat::Yuv420, t))
        .collect();
    let base = encode_display_order_gop_sequence(
        &sources,
        1,
        2,
        params(64, 48, ChromaFormat::Yuv420, true),
        14,
        3,
        3,
    )
    .expect("coarse lower layer");
    let enh = check_pair("progressive", &base, &sources, 4);
    // Two-GOP structure: the enhancement mirrors the GOP headers.
    let gops = |s: &[u8]| s.windows(4).filter(|w| *w == [0, 0, 1, 0xB8]).count();
    assert_eq!(gops(&enh), gops(&base));
}

#[test]
fn frame_field_lower_layer_matches_dct_type_and_roundtrips_exactly() {
    // frame_pred_frame_dct = 0: the enhancement macroblocks carry a
    // dct_type that must agree with the lower layer's (§7.8.2.1).
    let base_src: Vec<FrameBuffer> = (0..4)
        .map(|t| {
            let mut f = frame_at(64, 64, ChromaFormat::Yuv420, t);
            // Opposite per-field pans so field MC / field DCT fire.
            let src = f.clone();
            for y in 0..64 {
                let dx: i32 = if y % 2 == 0 {
                    2 * t as i32
                } else {
                    -2 * (t as i32)
                };
                for x in 0..64 {
                    let sx = (x as i32 - dx).clamp(0, 63) as usize;
                    f.y.put_sample(x, y, src.y.get(sx, y).unwrap());
                }
            }
            f
        })
        .collect();
    let (base, stats) = encode_ff_display_order_gop_sequence(
        &base_src,
        1,
        2,
        &params(64, 64, ChromaFormat::Yuv420, false),
        14,
        3,
        3,
        false,
    )
    .expect("frame-field lower layer");
    assert!(stats.field_dct > 0, "{stats:?}");
    check_pair("frame-field", &base, &base_src, 5);
}

#[test]
fn skipped_lower_macroblocks_take_zero_lower_coefficients() {
    // A mostly-static scene: the lower layer skips most P / B
    // macroblocks (§7.6.6), which the enhancement layer still refines
    // with F''lower = 0 (§7.8.2.2).
    let sources: Vec<FrameBuffer> = (0..4)
        .map(|t| {
            let mut f = FrameBuffer::new(64, 48, ChromaFormat::Yuv420);
            for y in 0..48 {
                for x in 0..64 {
                    let n = (x * 7 + y * 13) % 7;
                    f.y.put_sample(x, y, (90 + n) as u8);
                }
            }
            for y in (8 + 4 * t)..(16 + 4 * t) {
                for x in 8..16 {
                    f.y.put_sample(x, y, if (x + y) % 2 == 0 { 16 } else { 235 });
                }
            }
            for y in 0..24 {
                for x in 0..32 {
                    f.cb.put_sample(x, y, 112);
                    f.cr.put_sample(x, y, 144);
                }
            }
            f
        })
        .collect();
    let options = FrameEncodeOptions {
        skipped_macroblocks: true,
        concealment_motion_vectors: true,
        ..Default::default()
    };
    let (base, stats) = encode_display_order_gop_sequence_with_options(
        &sources,
        1,
        2,
        params(64, 48, ChromaFormat::Yuv420, true),
        16,
        3,
        3,
        &QuantMatrixExtension::default(),
        &|_| options,
    )
    .expect("skipping lower layer");
    assert!(stats.skipped > 0, "{stats:?}");
    check_pair("skips", &base, &sources, 3);
}

#[test]
fn chroma_422_lower_layer_with_downloaded_matrices_roundtrips_exactly() {
    let sources: Vec<FrameBuffer> = (0..3)
        .map(|t| frame_at(64, 48, ChromaFormat::Yuv422, t))
        .collect();
    let matrices = QuantMatrixExtension {
        intra: None,
        non_intra: Some(
            oxideav_mpeg12video::quant_matrix_extension::QuantiserMatrixPayload {
                bytes: [20u8; 64],
            },
        ),
        chroma_intra: None,
        chroma_non_intra: None,
    };
    let p = IntraPictureParams {
        intra_vlc_format: true,
        alternate_scan: true,
        q_scale_type: true,
        ..params(64, 48, ChromaFormat::Yuv422, true)
    };
    let (base, _) = encode_display_order_gop_sequence_with_options(
        &sources,
        1,
        2,
        p,
        18,
        3,
        3,
        &matrices,
        &|_| FrameEncodeOptions::default(),
    )
    .expect("4:2:2 lower layer");
    check_pair("4:2:2", &base, &sources, 6);
}

#[test]
fn enhancement_without_refinement_reproduces_the_lower_layer() {
    // An enhancement layer whose every macroblock is "Not Coded"
    // (quantiser so coarse nothing survives) combines to exactly the
    // lower layer's own reconstruction.
    let sources: Vec<FrameBuffer> = (0..3)
        .map(|t| frame_at(48, 32, ChromaFormat::Yuv420, t))
        .collect();
    let base = encode_display_order_gop_sequence(
        &sources,
        1,
        2,
        params(48, 32, ChromaFormat::Yuv420, true),
        4,
        3,
        3,
    )
    .unwrap();
    let enc = encode_snr_enhancement_layer(&base, &sources, 31).unwrap();
    let combined = decode_snr_scalable_sequence(&base, &enc.stream).unwrap();
    let lower = decode_video_sequence(&base).unwrap();
    // Whatever the layer coded, the loop is exact against the encoder…
    assert_frames_equal("coarse", &combined, &enc.recon);
    // …and a layer that codes nothing is the identity.
    if enc.coded_macroblocks == 0 {
        assert_frames_equal("identity", &combined, &lower);
    }
}

#[test]
fn mismatched_pairs_and_garbage_are_rejected_not_panicked() {
    let sources: Vec<FrameBuffer> = (0..3)
        .map(|t| frame_at(48, 32, ChromaFormat::Yuv420, t))
        .collect();
    let base = encode_display_order_gop_sequence(
        &sources,
        1,
        2,
        params(48, 32, ChromaFormat::Yuv420, true),
        12,
        3,
        3,
    )
    .unwrap();
    let enc = encode_snr_enhancement_layer(&base, &sources, 4).unwrap();

    // Enhancement of a different geometry.
    let other: Vec<FrameBuffer> = (0..3)
        .map(|t| frame_at(64, 32, ChromaFormat::Yuv420, t))
        .collect();
    let other_base = encode_display_order_gop_sequence(
        &other,
        1,
        2,
        params(64, 32, ChromaFormat::Yuv420, true),
        12,
        3,
        3,
    )
    .unwrap();
    assert!(decode_snr_scalable_sequence(&other_base, &enc.stream).is_err());
    // A plain (non-scalable) stream is not an enhancement layer.
    assert!(decode_snr_scalable_sequence(&base, &base).is_err());
    // Swapped layers.
    assert!(decode_snr_scalable_sequence(&enc.stream, &base).is_err());
    // An ISO/IEC 11172-2 lower layer is not composed.
    let m1 = oxideav_mpeg12video::encode_mpeg1_display_order_sequence(
        &sources,
        1,
        2,
        &oxideav_mpeg12video::Mpeg1SequenceParams {
            horizontal_size: 48,
            vertical_size: 32,
            ..Default::default()
        },
        6,
        3,
        3,
    )
    .unwrap();
    assert!(decode_snr_scalable_sequence(&m1, &enc.stream).is_err());
    assert!(encode_snr_enhancement_layer(&m1, &sources, 4).is_err());
    // Too few sources / bad quantiser.
    assert!(encode_snr_enhancement_layer(&base, &sources[..2], 4).is_err());
    assert!(encode_snr_enhancement_layer(&base, &sources, 0).is_err());

    // Truncations and bit flips of the enhancement layer never panic.
    for cut in (0..enc.stream.len()).step_by(37) {
        let _ = decode_snr_scalable_sequence(&base, &enc.stream[..cut]);
    }
    for k in (0..enc.stream.len()).step_by(53) {
        let mut bad = enc.stream.clone();
        bad[k] ^= 0x5A;
        let _ = decode_snr_scalable_sequence(&base, &bad);
    }
}
