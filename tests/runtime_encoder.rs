//! End-to-end runtime [`oxideav_core::Encoder`] wiring for MPEG-1 /
//! MPEG-2 video: the frame-to-packet encoder adapter — both the direct
//! [`oxideav_mpeg12video::make_encoder`] factory and the registry path
//! — proves the elementary stream it emits decodes back through the
//! crate's own runtime decoder and `decode_video_sequence` with the
//! frame count, geometry, and bounded round-trip error intact.

use oxideav_core::{
    CodecId, CodecParameters, Encoder, Error, Frame, PixelFormat, RuntimeContext, VideoFrame,
    VideoPlane,
};
use oxideav_mpeg12video::{
    decode_video_sequence, make_decoder, make_encoder, MPEG1_CODEC_ID_STR, MPEG2_CODEC_ID_STR,
};

/// A deterministic panning 4:2:0 planar [`VideoFrame`].
fn video_frame(width: usize, height: usize, t: usize) -> VideoFrame {
    let mut y = vec![0u8; width * height];
    for r in 0..height {
        for c in 0..width {
            y[r * width + c] = (30 + ((c + 2 * t) * 3 + r * 5) % 200) as u8;
        }
    }
    let (cw, ch) = (width / 2, height / 2);
    let mut cb = vec![0u8; cw * ch];
    let mut cr = vec![0u8; cw * ch];
    for r in 0..ch {
        for c in 0..cw {
            cb[r * cw + c] = (96 + (c + t) % 64) as u8;
            cr[r * cw + c] = (160 - ((r + t) % 64)) as u8;
        }
    }
    VideoFrame {
        pts: Some(t as i64),
        planes: vec![
            VideoPlane {
                stride: width,
                data: y,
            },
            VideoPlane {
                stride: cw,
                data: cb,
            },
            VideoPlane {
                stride: cw,
                data: cr,
            },
        ],
    }
}

fn encoder_params(id: &str, width: u32, height: u32) -> CodecParameters {
    let mut p = CodecParameters::video(CodecId::new(id));
    p.width = Some(width);
    p.height = Some(height);
    p.pixel_format = Some(PixelFormat::Yuv420P);
    p
}

/// Feed `n` frames, flush, and return the single emitted elementary
/// stream packet payload.
fn encode_n(enc: &mut dyn Encoder, width: usize, height: usize, n: usize) -> Vec<u8> {
    for t in 0..n {
        enc.send_frame(&Frame::Video(video_frame(width, height, t)))
            .expect("send_frame");
    }
    assert!(
        matches!(enc.receive_packet(), Err(Error::NeedMore)),
        "no packet before flush (whole-stream framing)"
    );
    enc.flush().expect("flush");
    let packet = enc.receive_packet().expect("stream packet");
    assert!(
        packet.flags.keyframe,
        "stream packet starts at an I picture"
    );
    assert_eq!(packet.duration, Some(n as i64));
    assert!(
        matches!(enc.receive_packet(), Err(Error::Eof)),
        "drained after the single stream packet"
    );
    packet.data
}

#[test]
fn registry_installs_encoders_under_both_codec_ids() {
    let mut ctx = RuntimeContext::new();
    oxideav_mpeg12video::register(&mut ctx);
    for id in [MPEG1_CODEC_ID_STR, MPEG2_CODEC_ID_STR] {
        let params = encoder_params(id, 64, 48);
        let enc = ctx
            .codecs
            .first_encoder(&params)
            .unwrap_or_else(|_| panic!("{id} encoder factory"));
        assert_eq!(enc.codec_id().as_str(), id);
        assert_eq!(enc.output_params().width, Some(64));
        assert_eq!(enc.output_params().height, Some(48));
        assert_eq!(enc.output_params().pixel_format, Some(PixelFormat::Yuv420P));
    }
}

#[test]
fn mpeg2_runtime_encode_roundtrips_through_both_decode_paths() {
    let params = encoder_params(MPEG2_CODEC_ID_STR, 64, 48);
    let mut enc = make_encoder(&params).expect("make_encoder");
    let stream = encode_n(enc.as_mut(), 64, 48, 5);

    // Whole-stream driver.
    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 5);

    // Runtime decoder adapter.
    let mut dec = make_decoder(&params).expect("make_decoder");
    dec.send_packet(&oxideav_core::Packet::new(
        0,
        oxideav_core::TimeBase::new(1, 25),
        stream,
    ))
    .expect("send_packet");
    dec.flush().expect("decoder flush");
    let mut count = 0usize;
    let mut mae_sum = 0f64;
    loop {
        match dec.receive_frame() {
            Ok(Frame::Video(vf)) => {
                let input = video_frame(64, 48, count);
                assert_eq!(vf.planes[0].data.len(), input.planes[0].data.len());
                let total: u64 = vf.planes[0]
                    .data
                    .iter()
                    .zip(input.planes[0].data.iter())
                    .map(|(&a, &b)| u64::from(a.abs_diff(b)))
                    .sum();
                mae_sum += total as f64 / vf.planes[0].data.len() as f64;
                count += 1;
            }
            Ok(_) => panic!("expected video frames"),
            Err(Error::Eof) => break,
            Err(e) => panic!("decode error: {e:?}"),
        }
    }
    assert_eq!(count, 5, "display-order frame count");
    let mae = mae_sum / 5.0;
    assert!(mae < 8.0, "round-trip luma MAE {mae}");
}

#[test]
fn mpeg1_runtime_encode_emits_an_11172_2_stream_that_roundtrips() {
    let params = encoder_params(MPEG1_CODEC_ID_STR, 64, 48);
    let mut enc = make_encoder(&params).expect("make_encoder");
    let stream = encode_n(enc.as_mut(), 64, 48, 4);

    // 11172-2 classification: no extension start code anywhere.
    assert!(
        !stream.windows(4).any(|w| w == [0x00, 0x00, 0x01, 0xB5]),
        "an ISO/IEC 11172-2 stream carries no extension_start_code"
    );

    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 4);
}

#[test]
fn encoder_options_are_honoured_and_validated() {
    // b_between = 0 with a 3-frame input yields I P P — every picture
    // an anchor.
    let mut params = encoder_params(MPEG2_CODEC_ID_STR, 48, 32);
    params.options.insert("b_between", "0");
    params.options.insert("quantiser_scale_code", "8");
    params.options.insert("f_code", "2");
    let mut enc = make_encoder(&params).expect("make_encoder");
    let stream = encode_n(enc.as_mut(), 48, 32, 3);
    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 3);
    // With no B pictures the display order equals coded order and every
    // temporal_reference increments within the GOP.
    let trefs: Vec<u16> = decoded.iter().map(|d| d.temporal_reference).collect();
    assert_eq!(trefs, vec![0, 1, 2]);

    // Out-of-range / garbage options are rejected at construction.
    for (key, value) in [
        ("quantiser_scale_code", "0"),
        ("quantiser_scale_code", "32"),
        ("quantiser_scale_code", "abc"),
        ("f_code", "8"),
        ("anchors_per_gop", "0"),
    ] {
        let mut bad = encoder_params(MPEG2_CODEC_ID_STR, 48, 32);
        bad.options.insert(key, value);
        assert!(
            make_encoder(&bad).is_err(),
            "option {key}={value} must be rejected"
        );
    }
}

#[test]
fn encoder_rejects_bad_geometry_and_formats() {
    // Missing geometry.
    let bare = CodecParameters::video(CodecId::new(MPEG2_CODEC_ID_STR));
    assert!(make_encoder(&bare).is_err(), "missing width/height");

    // Unsupported pixel format.
    let mut p = encoder_params(MPEG2_CODEC_ID_STR, 64, 48);
    p.pixel_format = Some(PixelFormat::Yuv444P);
    assert!(make_encoder(&p).is_err(), "non-4:2:0 pixel format");

    // Frame geometry mismatch at send_frame.
    let params = encoder_params(MPEG2_CODEC_ID_STR, 64, 48);
    let mut enc = make_encoder(&params).expect("make_encoder");
    let wrong = video_frame(32, 32, 0);
    assert!(
        enc.send_frame(&Frame::Video(wrong)).is_err(),
        "undersized planes must be rejected"
    );

    // send_frame after flush is rejected.
    let mut enc = make_encoder(&params).expect("make_encoder");
    enc.flush().expect("flush");
    assert!(enc
        .send_frame(&Frame::Video(video_frame(64, 48, 0)))
        .is_err());
}
