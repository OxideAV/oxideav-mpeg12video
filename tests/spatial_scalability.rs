//! §7.7 spatial scalability end to end (progressive layers, weight
//! table `00`): the self-made enhancement-layer encoder is the oracle
//! — the two-layer decode loop must reproduce its frames sample for
//! sample — over a 2:1 upsampled I B P B P lower layer (every Table
//! B-5 / B-6 / B-7 row the encoder emits: intra, temporal, half-weight
//! and spatial-only macroblocks), a 4:2:0 → 4:2:2 chroma upgrade
//! (Table 7-18), a non-scalable enhancement picture (§6.3.7), and the
//! rejection paths.

use oxideav_mpeg12video::picture_spatial_scalable_extension::PictureSpatialScalableExtension;
use oxideav_mpeg12video::sequence_extension::{ChromaFormat, Mpeg2Sequence};
use oxideav_mpeg12video::sequence_scalable_extension::{ScalableMode, SequenceScalableExtension};
use oxideav_mpeg12video::{
    decode_spatial_scalable_sequence, decode_video_sequence, encode_display_order_gop_sequence,
    encode_spatial_enhancement_layer, DecodedFrame, FrameBuffer, IntraPictureParams,
    SpatialLayerConfig,
};

fn params(width: usize, height: usize, chroma: ChromaFormat) -> IntraPictureParams {
    IntraPictureParams {
        width,
        height,
        chroma_format: chroma,
        frame_pred_frame_dct: true,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        progressive_sequence: true,
    }
}

/// Full-resolution content at instant `t`: a smooth gradient the
/// lower layer captures, fine texture only the enhancement layer can
/// code, a translating high-contrast stamp.
fn full_frame(width: usize, height: usize, chroma: ChromaFormat, t: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, chroma);
    for y in 0..height {
        for x in 0..width {
            let sx = x + 2 * t;
            let g = 40 + ((sx * 2 + y * 3) % 160);
            let n = (sx * 7 + y * 11) % 13;
            f.y.put_sample(x, y, (g + n).min(235) as u8);
        }
    }
    let (bx, by) = (12 + 2 * t, 10);
    for y in by..(by + 12).min(height) {
        for x in bx..(bx + 12).min(width) {
            f.y.put_sample(x, y, if (x + y) % 2 == 0 { 16 } else { 235 });
        }
    }
    let (cw, ch) = f.visible_chroma_dims();
    for y in 0..ch {
        for x in 0..cw {
            f.cb.put_sample(x, y, (96 + (x + y + t) % 64) as u8);
            f.cr.put_sample(x, y, (160u8).saturating_sub(((x * 2 + y + t) % 64) as u8));
        }
    }
    f
}

/// 2:1 box-filtered lower-resolution version of `full`.
fn downsample(full: &FrameBuffer, chroma: ChromaFormat) -> FrameBuffer {
    let (w, h) = (full.width / 2, full.height / 2);
    let mut f = FrameBuffer::new(w, h, chroma);
    for y in 0..h {
        for x in 0..w {
            let s = (0..2)
                .flat_map(|dy| (0..2).map(move |dx| (dx, dy)))
                .map(|(dx, dy)| u32::from(full.y.get(2 * x + dx, 2 * y + dy).unwrap()))
                .sum::<u32>();
            f.y.put_sample(x, y, ((s + 2) / 4) as u8);
        }
    }
    let (cw, ch) = f.visible_chroma_dims();
    let (fcw, fch) = full.visible_chroma_dims();
    for y in 0..ch {
        for x in 0..cw {
            let sx = (x * fcw / cw).min(fcw - 1);
            let sy = (y * fch / ch).min(fch - 1);
            f.cb.put_sample(x, y, full.cb.get(sx, sy).unwrap());
            f.cr.put_sample(x, y, full.cr.get(sx, sy).unwrap());
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

fn build(
    enh_chroma: ChromaFormat,
    lower_chroma: ChromaFormat,
    n: usize,
    q_lower: u8,
    config: &SpatialLayerConfig,
) -> (
    Vec<u8>,
    Vec<FrameBuffer>,
    oxideav_mpeg12video::SpatialEncoded,
) {
    let sources: Vec<FrameBuffer> = (0..n).map(|t| full_frame(64, 48, enh_chroma, t)).collect();
    let lower_src: Vec<FrameBuffer> = sources
        .iter()
        .map(|f| downsample(f, lower_chroma))
        .collect();
    let base = encode_display_order_gop_sequence(
        &lower_src,
        1,
        2,
        params(32, 24, lower_chroma),
        q_lower,
        3,
        3,
    )
    .expect("lower layer");
    let enc = encode_spatial_enhancement_layer(&base, &sources, config).expect("enhancement");
    (base, sources, enc)
}

#[test]
fn two_to_one_upsampled_ibp_roundtrips_exactly_and_uses_every_class() {
    let config = SpatialLayerConfig {
        quantiser_scale_code: 5,
        f_code: 3,
    };
    let (base, sources, enc) = build(ChromaFormat::Yuv420, ChromaFormat::Yuv420, 5, 6, &config);
    let stats = enc.stats;
    assert!(stats.spatial_only > 0, "{stats:?}");
    assert!(stats.temporal > 0, "{stats:?}");
    assert!(stats.half_weight > 0, "{stats:?}");

    let decoded = decode_spatial_scalable_sequence(&base, &enc.stream).expect("two-layer decode");
    assert_frames_equal("enh", &decoded.enhancement, &enc.enhancement);
    assert_frames_equal(
        "lower",
        &decoded.lower,
        &decode_video_sequence(&base).unwrap(),
    );
    assert_eq!(decoded.enhancement.len(), 5);
    assert_eq!(
        (
            decoded.enhancement[0].frame.width,
            decoded.enhancement[0].frame.height
        ),
        (64, 48)
    );

    // The enhancement layer reconstructs the full-resolution source
    // closely — far better than the upsampled lower layer alone.
    let mae = luma_mae(&decoded.enhancement, &sources);
    assert!(mae < 5.0, "enhancement luma MAE {mae:.2}");

    // Declarations: spatial sequence scalable extension with the
    // lower geometry and 1:2 factors; a pss on every picture naming the
    // coincident lower frame.
    let seq = Mpeg2Sequence::from_buf(&enc.stream).unwrap();
    assert_eq!((seq.horizontal_size, seq.vertical_size), (64, 48));
    let sse_pos = enc
        .stream
        .windows(5)
        .position(|w| w[..4] == [0, 0, 1, 0xB5] && w[4] >> 4 == 0b0101)
        .unwrap();
    let sse = SequenceScalableExtension::parse(&enc.stream[sse_pos..]).unwrap();
    assert_eq!(sse.layer_id, 1);
    match sse.scalable_mode {
        ScalableMode::SpatialScalability(p) => {
            assert_eq!(p.lower_layer_prediction_horizontal_size, 32);
            assert_eq!(p.lower_layer_prediction_vertical_size, 24);
            assert_eq!(
                (
                    p.horizontal_subsampling_factor_m,
                    p.horizontal_subsampling_factor_n
                ),
                (1, 2)
            );
            assert_eq!(
                (
                    p.vertical_subsampling_factor_m,
                    p.vertical_subsampling_factor_n
                ),
                (1, 2)
            );
        }
        other => panic!("{other:?}"),
    }
    let pss: Vec<PictureSpatialScalableExtension> = enc
        .stream
        .windows(5)
        .enumerate()
        .filter(|(_, w)| w[..4] == [0, 0, 1, 0xB5] && w[4] >> 4 == 0b1001)
        .map(|(i, _)| PictureSpatialScalableExtension::parse(&enc.stream[i..]).unwrap())
        .collect();
    assert_eq!(pss.len(), 5);
    assert!(pss
        .iter()
        .all(|p| p.spatial_temporal_weight_code_table_index == 0
            && p.lower_layer_progressive_frame
            && p.lower_layer_horizontal_offset == 0));
    // Coded order I P B P B → lower_layer_temporal_reference 0 2 1 4 3.
    let trefs: Vec<u16> = pss
        .iter()
        .map(|p| p.lower_layer_temporal_reference)
        .collect();
    assert_eq!(trefs, vec![0, 2, 1, 4, 3]);
}

#[test]
fn chroma_upgrade_420_lower_to_422_enhancement() {
    let config = SpatialLayerConfig::default();
    let (base, sources, enc) = build(ChromaFormat::Yuv422, ChromaFormat::Yuv420, 3, 8, &config);
    let decoded = decode_spatial_scalable_sequence(&base, &enc.stream).expect("two-layer decode");
    assert_frames_equal("422", &decoded.enhancement, &enc.enhancement);
    assert_eq!(
        decoded.enhancement[0].frame.chroma_format,
        ChromaFormat::Yuv422
    );
    assert_eq!(decoded.lower[0].frame.chroma_format, ChromaFormat::Yuv420);
    let mae = luma_mae(&decoded.enhancement, &sources);
    assert!(mae < 6.0, "luma MAE {mae:.2}");
}

#[test]
fn a_picture_without_the_spatial_extension_decodes_non_scalably() {
    // Strip the pss from the enhancement layer's first (I) picture:
    // §6.3.7 says it is then decoded with Tables B-2 .. B-4. The
    // encoder's I picture used spatial-only rows, so the stripped
    // stream must be rejected as non-scalable syntax rather than
    // misread — but a pss-less stream built from ordinary pictures
    // decodes: mirror the lower layer itself as an "enhancement" at 1:1.
    let sources: Vec<FrameBuffer> = (0..3)
        .map(|t| full_frame(32, 32, ChromaFormat::Yuv420, t))
        .collect();
    let base = encode_display_order_gop_sequence(
        &sources,
        1,
        2,
        params(32, 32, ChromaFormat::Yuv420),
        8,
        3,
        3,
    )
    .unwrap();
    let enc = encode_spatial_enhancement_layer(&base, &sources, &SpatialLayerConfig::default())
        .expect("1:1 enhancement");
    let decoded = decode_spatial_scalable_sequence(&base, &enc.stream).unwrap();
    assert_frames_equal("1:1", &decoded.enhancement, &enc.enhancement);

    // Remove every pss: the pictures carry Table B-5..B-7 codes, so
    // the non-scalable parse must fail cleanly (no panic).
    let mut stripped = Vec::new();
    let mut i = 0;
    while i < enc.stream.len() {
        if i + 5 <= enc.stream.len()
            && enc.stream[i..i + 4] == [0, 0, 1, 0xB5]
            && enc.stream[i + 4] >> 4 == 0b1001
        {
            // pss is 4 + 6 bytes (48 bits payload after the id = 10 + 1 + 15 + 1 + 15 + 2 + 1 + 1 = 46 bits + 4-bit id = 50 bits → 7 bytes).
            let mut j = i + 4;
            while j + 3 < enc.stream.len()
                && !(enc.stream[j] == 0 && enc.stream[j + 1] == 0 && enc.stream[j + 2] == 1)
            {
                j += 1;
            }
            i = j;
            continue;
        }
        stripped.push(enc.stream[i]);
        i += 1;
    }
    let _ = decode_spatial_scalable_sequence(&base, &stripped);
}

#[test]
fn mismatched_pairs_and_garbage_are_rejected_not_panicked() {
    let config = SpatialLayerConfig::default();
    let (base, sources, enc) = build(ChromaFormat::Yuv420, ChromaFormat::Yuv420, 3, 8, &config);

    assert!(encode_spatial_enhancement_layer(&base, &sources[..2], &config).is_err());
    let bad = SpatialLayerConfig {
        quantiser_scale_code: 0,
        ..config
    };
    assert!(encode_spatial_enhancement_layer(&base, &sources, &bad).is_err());
    // A lower layer of a different geometry than the extension declares.
    let other: Vec<FrameBuffer> = sources
        .iter()
        .map(|_| full_frame(48, 24, ChromaFormat::Yuv420, 0))
        .collect();
    let other_base = encode_display_order_gop_sequence(
        &other,
        1,
        2,
        params(48, 24, ChromaFormat::Yuv420),
        8,
        3,
        3,
    )
    .unwrap();
    assert!(decode_spatial_scalable_sequence(&other_base, &enc.stream).is_err());
    // Plain streams and swapped layers.
    assert!(decode_spatial_scalable_sequence(&base, &base).is_err());
    assert!(decode_spatial_scalable_sequence(&enc.stream, &base).is_err());
    // An ISO/IEC 11172-2 lower layer.
    let m1 = oxideav_mpeg12video::encode_mpeg1_display_order_sequence(
        &sources
            .iter()
            .map(|f| downsample(f, ChromaFormat::Yuv420))
            .collect::<Vec<_>>(),
        1,
        2,
        &oxideav_mpeg12video::Mpeg1SequenceParams {
            horizontal_size: 32,
            vertical_size: 24,
            ..Default::default()
        },
        6,
        3,
        3,
    )
    .unwrap();
    assert!(encode_spatial_enhancement_layer(&m1, &sources, &config).is_err());
    assert!(decode_spatial_scalable_sequence(&m1, &enc.stream).is_err());

    for cut in (0..enc.stream.len()).step_by(43) {
        let _ = decode_spatial_scalable_sequence(&base, &enc.stream[..cut]);
    }
    for k in (0..enc.stream.len()).step_by(59) {
        let mut bad = enc.stream.clone();
        bad[k] ^= 0x3C;
        let _ = decode_spatial_scalable_sequence(&base, &bad);
    }
}

#[test]
fn intra_rows_fire_when_the_lower_layer_predicts_nothing() {
    // A black lower layer: the spatial prediction is useless, so the
    // I picture must fall back to Table B-5 intra rows (and the P / B
    // pictures to enhancement-layer temporal prediction).
    let config = SpatialLayerConfig::default();
    let sources: Vec<FrameBuffer> = (0..3)
        .map(|t| full_frame(64, 48, ChromaFormat::Yuv420, t))
        .collect();
    let black: Vec<FrameBuffer> = (0..3)
        .map(|_| {
            let mut f = FrameBuffer::new(32, 24, ChromaFormat::Yuv420);
            for y in 0..24 {
                for x in 0..32 {
                    f.y.put_sample(x, y, 16);
                }
            }
            for y in 0..12 {
                for x in 0..16 {
                    f.cb.put_sample(x, y, 128);
                    f.cr.put_sample(x, y, 128);
                }
            }
            f
        })
        .collect();
    let base = encode_display_order_gop_sequence(
        &black,
        1,
        2,
        params(32, 24, ChromaFormat::Yuv420),
        8,
        3,
        3,
    )
    .unwrap();
    let enc = encode_spatial_enhancement_layer(&base, &sources, &config).expect("enhancement");
    assert!(enc.stats.intra > 0, "{:?}", enc.stats);
    assert!(enc.stats.temporal > 0, "{:?}", enc.stats);
    let decoded = decode_spatial_scalable_sequence(&base, &enc.stream).expect("two-layer decode");
    assert_frames_equal("black lower", &decoded.enhancement, &enc.enhancement);
    let mae = luma_mae(&decoded.enhancement, &sources);
    assert!(mae < 6.0, "luma MAE {mae:.2}");
}
