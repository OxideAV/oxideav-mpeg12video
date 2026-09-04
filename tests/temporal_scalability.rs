//! §7.9 temporal scalability end to end: the self-made enhancement-
//! layer encoder is the oracle — the two-layer decode loop must
//! reproduce its enhancement pictures sample for sample — over
//! progressive and interlaced (frame-picture) lower layers, every
//! Table 7-28 / 7-29 reference selection the encoder emits (`10` P
//! before the first lower frame, `11` and `10` B pictures), several
//! multiplex shapes, the §6.3.7 remultiplex, and the rejection paths.

use oxideav_mpeg12video::picture_temporal_scalable_extension::PictureTemporalScalableExtension;
use oxideav_mpeg12video::sequence_extension::{ChromaFormat, Mpeg2Sequence};
use oxideav_mpeg12video::sequence_scalable_extension::{ScalableMode, SequenceScalableExtension};
use oxideav_mpeg12video::{
    decode_temporal_scalable_sequence, decode_video_sequence, encode_display_order_gop_sequence,
    encode_temporal_enhancement_layer, DecodedFrame, FrameBuffer, IntraPictureParams,
    PictureCodingType, TemporalLayerConfig,
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

/// Content at "time" `t2` in half-frame units: the lower layer takes
/// the even instants, the enhancement layer the odd ones.
fn frame_at_half(width: usize, height: usize, t2: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            let sx = x + t2;
            let g = 24 + ((sx * 3 + y * 5) % 192);
            let c = if (sx / 4 + y / 4) % 2 == 0 { 16 } else { 0 };
            f.y.put_sample(x, y, (g + c).min(235) as u8);
        }
    }
    for y in 0..height.div_ceil(2) {
        for x in 0..width.div_ceil(2) {
            f.cb.put_sample(x, y, (96 + (x + t2 / 2 + y) % 64) as u8);
            f.cr.put_sample(x, y, (160u8).saturating_sub(((x + y + t2) % 64) as u8));
        }
    }
    f
}

fn assert_frames_equal(name: &str, a: &[DecodedFrame], b: &[DecodedFrame]) {
    assert_eq!(a.len(), b.len(), "{name}: frame count");
    for (i, (x, y)) in a.iter().zip(b).enumerate() {
        assert_eq!(
            x.temporal_reference, y.temporal_reference,
            "{name}: {i} tref"
        );
        assert_eq!(
            x.picture_coding_type, y.picture_coding_type,
            "{name}: {i} type"
        );
        assert_eq!(x.frame.y.samples(), y.frame.y.samples(), "{name}: {i} luma");
        assert_eq!(x.frame.cb.samples(), y.frame.cb.samples(), "{name}: {i} cb");
        assert_eq!(x.frame.cr.samples(), y.frame.cr.samples(), "{name}: {i} cr");
    }
}

fn luma_mae(a: &FrameBuffer, b: &FrameBuffer) -> f64 {
    let mut total = 0u64;
    for y in 0..a.height {
        for x in 0..a.width {
            total += u64::from(a.y.get(x, y).unwrap().abs_diff(b.y.get(x, y).unwrap()));
        }
    }
    total as f64 / (a.width * a.height) as f64
}

/// Every `reference_select_code` in the enhancement stream, in order.
fn select_codes(stream: &[u8]) -> Vec<u8> {
    stream
        .windows(5)
        .enumerate()
        .filter(|(_, w)| w[..4] == [0, 0, 1, 0xB5] && w[4] >> 4 == 0b1010)
        .map(|(i, _)| {
            PictureTemporalScalableExtension::parse(&stream[i..])
                .unwrap()
                .reference_select_code
        })
        .collect()
}

/// Build the pair for `n_lower` lower frames and a mux shape; return
/// `(base, sources, encoded)`.
fn build(
    n_lower: usize,
    progressive: bool,
    config: TemporalLayerConfig,
) -> (
    Vec<u8>,
    Vec<FrameBuffer>,
    oxideav_mpeg12video::TemporalEncoded,
) {
    let (w, h) = (64, 48);
    let order = usize::from(config.picture_mux_order);
    let factor = usize::from(config.picture_mux_factor);
    // Lower frames at instants order + j * (factor + 1); enhancement
    // frames fill the instants in between.
    let lower_src: Vec<FrameBuffer> = (0..n_lower)
        .map(|j| frame_at_half(w, h, order + j * (factor + 1)))
        .collect();
    let base =
        encode_display_order_gop_sequence(&lower_src, 1, 2, params(w, h, progressive), 8, 3, 3)
            .expect("lower layer");
    let mut sources = Vec::new();
    for t in 0..order {
        sources.push(frame_at_half(w, h, t));
    }
    for j in 0..n_lower - 1 {
        for k in 1..=factor {
            sources.push(frame_at_half(w, h, order + j * (factor + 1) + k));
        }
    }
    let enc = encode_temporal_enhancement_layer(&base, &sources, &config).expect("enhancement");
    (base, sources, enc)
}

#[test]
fn doubling_the_frame_rate_roundtrips_exactly_and_remultiplexes() {
    let config = TemporalLayerConfig {
        quantiser_scale_code: 6,
        f_code: 3,
        picture_mux_order: 0,
        picture_mux_factor: 1,
        use_enhancement_references: false,
    };
    let (base, sources, enc) = build(5, true, config);
    assert_eq!(enc.enhancement.len(), 4);
    assert_eq!(
        enc.reference_select_codes,
        vec![0b11; 4],
        "B from both lower frames"
    );

    let decoded = decode_temporal_scalable_sequence(&base, &enc.stream).expect("two-layer decode");
    assert_frames_equal("enh", &decoded.enhancement, &enc.enhancement);
    assert_frames_equal(
        "lower",
        &decoded.lower,
        &decode_video_sequence(&base).unwrap(),
    );
    assert!(decoded
        .enhancement
        .iter()
        .all(|d| d.picture_coding_type == PictureCodingType::Bidirectional));

    // The remultiplex interleaves L E L E L E L E L (identity, not
    // coding type: the lower layer is itself I B P B P).
    let muxed = decoded.remultiplex();
    assert_eq!(muxed.len(), 9);
    for (i, f) in muxed.iter().enumerate() {
        let from_lower = decoded.lower.iter().any(|l| std::ptr::eq(l, *f));
        assert_eq!(from_lower, i % 2 == 0, "position {i}");
        if from_lower {
            assert!(std::ptr::eq(&decoded.lower[i / 2], *f));
        } else {
            assert!(std::ptr::eq(&decoded.enhancement[i / 2], *f));
        }
    }
    // Every enhancement picture is a faithful reconstruction of its
    // source (the in-between instants).
    for (d, s) in decoded.enhancement.iter().zip(&sources) {
        let mae = luma_mae(&d.frame, s);
        assert!(mae < 6.0, "enhancement luma MAE {mae:.2}");
    }

    // Stream declarations (§7.9.1 / §6.3.7).
    let seq = Mpeg2Sequence::from_buf(&enc.stream).unwrap();
    let lower_seq = Mpeg2Sequence::from_buf(&base).unwrap();
    assert_eq!(seq.horizontal_size, lower_seq.horizontal_size);
    assert_eq!(seq.vertical_size, lower_seq.vertical_size);
    assert!(seq.extension.progressive_sequence);
    let sse_pos = enc
        .stream
        .windows(5)
        .position(|w| w[..4] == [0, 0, 1, 0xB5] && w[4] >> 4 == 0b0101)
        .unwrap();
    let sse = SequenceScalableExtension::parse(&enc.stream[sse_pos..]).unwrap();
    assert_eq!(sse.layer_id, 1);
    match sse.scalable_mode {
        ScalableMode::TemporalScalability(p) => {
            assert!(p.picture_mux_enable);
            assert_eq!(p.mux_to_progressive_sequence, Some(true));
            assert_eq!((p.picture_mux_order, p.picture_mux_factor), (0, 1));
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(select_codes(&enc.stream), enc.reference_select_codes);
}

#[test]
fn leading_pictures_and_enhancement_references_cover_the_reference_codes() {
    // order = 2 (two P pictures from the *next* lower frame, code 10),
    // factor = 3 with enhancement references: B 11 then B 10, B 10.
    let config = TemporalLayerConfig {
        quantiser_scale_code: 5,
        f_code: 3,
        picture_mux_order: 2,
        picture_mux_factor: 3,
        use_enhancement_references: true,
    };
    let (base, _sources, enc) = build(3, true, config);
    assert_eq!(enc.enhancement.len(), 2 + 3 * 2);
    assert_eq!(
        enc.reference_select_codes,
        vec![0b10, 0b10, 0b11, 0b10, 0b10, 0b11, 0b10, 0b10]
    );
    assert_eq!(
        enc.enhancement[0].picture_coding_type,
        PictureCodingType::Predictive
    );
    assert_eq!(
        enc.enhancement[2].picture_coding_type,
        PictureCodingType::Bidirectional
    );

    let decoded = decode_temporal_scalable_sequence(&base, &enc.stream).expect("two-layer decode");
    assert_frames_equal("mixed refs", &decoded.enhancement, &enc.enhancement);
    let muxed = decoded.remultiplex();
    assert_eq!(muxed.len(), 3 + 8);
    // E E L E E E L E E E L
    let lower_positions: Vec<usize> = muxed
        .iter()
        .enumerate()
        .filter(|(_, f)| decoded.lower.iter().any(|l| std::ptr::eq(l, **f)))
        .map(|(i, _)| i)
        .collect();
    assert_eq!(lower_positions, vec![2, 6, 10]);
}

#[test]
fn interlaced_frame_picture_lower_layer_keeps_the_mux_flags_consistent() {
    let config = TemporalLayerConfig::default();
    let (base, _sources, enc) = build(4, false, config);
    let decoded = decode_temporal_scalable_sequence(&base, &enc.stream).expect("two-layer decode");
    assert_frames_equal("interlaced", &decoded.enhancement, &enc.enhancement);
    // §7.9.1: progressive_sequence = 0 → mux_to_progressive_sequence = 0
    // and progressive_frame = progressive_sequence on every picture.
    assert_eq!(decoded.mux.mux_to_progressive_sequence, Some(false));
    assert!(decoded.enhancement.iter().all(|d| !d.progressive_frame));
    let seq = Mpeg2Sequence::from_buf(&enc.stream).unwrap();
    assert!(!seq.extension.progressive_sequence);
}

#[test]
fn mismatched_pairs_and_garbage_are_rejected_not_panicked() {
    let config = TemporalLayerConfig::default();
    let (base, sources, enc) = build(3, true, config);

    // Wrong source count / geometry / configuration.
    assert!(encode_temporal_enhancement_layer(&base, &sources[..1], &config).is_err());
    let bad_cfg = TemporalLayerConfig {
        picture_mux_factor: 0,
        ..config
    };
    assert!(encode_temporal_enhancement_layer(&base, &sources, &bad_cfg).is_err());
    let m1 = oxideav_mpeg12video::encode_mpeg1_display_order_sequence(
        &sources
            .iter()
            .cloned()
            .chain(sources.iter().cloned())
            .take(3)
            .collect::<Vec<_>>(),
        1,
        2,
        &oxideav_mpeg12video::Mpeg1SequenceParams {
            horizontal_size: 64,
            vertical_size: 48,
            ..Default::default()
        },
        6,
        3,
        3,
    )
    .unwrap();
    assert!(encode_temporal_enhancement_layer(&m1, &sources[..2], &config).is_err());
    assert!(decode_temporal_scalable_sequence(&m1, &enc.stream).is_err());

    // A plain stream is not an enhancement layer; swapped layers fail.
    assert!(decode_temporal_scalable_sequence(&base, &base).is_err());
    assert!(decode_temporal_scalable_sequence(&enc.stream, &base).is_err());
    // A lower layer of a different geometry.
    let other: Vec<FrameBuffer> = (0..3).map(|t| frame_at_half(48, 48, 2 * t)).collect();
    let other_base =
        encode_display_order_gop_sequence(&other, 1, 2, params(48, 48, true), 8, 3, 3).unwrap();
    assert!(decode_temporal_scalable_sequence(&other_base, &enc.stream).is_err());
    // A lower layer with too few frames for the multiplex positions.
    let short: Vec<FrameBuffer> = (0..2).map(|t| frame_at_half(64, 48, 2 * t)).collect();
    let short_base =
        encode_display_order_gop_sequence(&short, 1, 2, params(64, 48, true), 8, 3, 3).unwrap();
    assert!(decode_temporal_scalable_sequence(&short_base, &enc.stream).is_err());

    // Truncations and bit flips never panic.
    for cut in (0..enc.stream.len()).step_by(41) {
        let _ = decode_temporal_scalable_sequence(&base, &enc.stream[..cut]);
    }
    for k in (0..enc.stream.len()).step_by(47) {
        let mut bad = enc.stream.clone();
        bad[k] ^= 0xA5;
        let _ = decode_temporal_scalable_sequence(&base, &bad);
    }
}
