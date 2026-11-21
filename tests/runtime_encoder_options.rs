//! The typed option schema of the runtime [`oxideav_core::Encoder`]
//! adapter (`Mpeg12EncoderOptions`): every documented option name
//! round-trips through the direct `make_encoder` factory / the registry
//! into the assembler it names — picture structure (field pairs,
//! frame-field, adaptive field modes), chroma format from the pixel
//! format, the entropy flags, `FrameEncodeOptions` (skips,
//! concealment vectors, cadence flags, 3:2 pulldown), dual-prime, the
//! Annex C CBR controllers, §7.10 data partitioning and MPEG-1
//! D-pictures — and every stream decodes back through the crate's own
//! decoder.

use oxideav_core::{
    CodecId, CodecOptionsStruct, CodecParameters, Encoder, Error, Frame, Packet, PixelFormat,
    Rational, RuntimeContext, VideoFrame, VideoPlane,
};
use oxideav_mpeg12video::picture_header::{Mpeg2PictureHeader, PictureStructure};
use oxideav_mpeg12video::sequence_extension::{ChromaFormat, Mpeg2Sequence};
use oxideav_mpeg12video::vbv::{VbvStandard, verify_cbr_stream};
use oxideav_mpeg12video::{
    MPEG1_CODEC_ID_STR, MPEG2_CODEC_ID_STR, Mpeg12EncoderOptions, PictureCodingType,
    decode_video_sequence, make_decoder, make_encoder, merge_data_partitions,
};

/// A deterministic interlaced-looking planar frame in `format`.
fn video_frame(width: usize, height: usize, t: usize, format: PixelFormat) -> VideoFrame {
    let (cw, ch) = match format {
        PixelFormat::Yuv420P => (width.div_ceil(2), height.div_ceil(2)),
        PixelFormat::Yuv422P => (width.div_ceil(2), height),
        PixelFormat::Yuv444P => (width, height),
        other => panic!("test helper: {other:?}"),
    };
    let mut y = vec![0u8; width * height];
    for r in 0..height {
        for c in 0..width {
            let line = if r % 2 == 0 { 10 } else { 0 };
            y[r * width + c] = (30 + ((c + 2 * t) * 3 + (r / 2) * 5 + line) % 200) as u8;
        }
    }
    let mut cb = vec![0u8; cw * ch];
    let mut cr = vec![0u8; cw * ch];
    for r in 0..ch {
        for c in 0..cw {
            cb[r * cw + c] = (96 + (c + t + r) % 64) as u8;
            cr[r * cw + c] = (160 - ((r + 2 * t) % 64)) as u8;
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

fn params(id: &str, width: u32, height: u32, format: PixelFormat) -> CodecParameters {
    let mut p = CodecParameters::video(CodecId::new(id));
    p.width = Some(width);
    p.height = Some(height);
    p.pixel_format = Some(format);
    p
}

fn with(p: &CodecParameters, opts: &[(&str, &str)]) -> CodecParameters {
    let mut q = p.clone();
    for (k, v) in opts {
        q.options.insert(*k, *v);
    }
    q
}

/// Feed `n` frames, flush, and return every emitted packet.
fn encode_packets(
    enc: &mut dyn Encoder,
    width: usize,
    height: usize,
    n: usize,
    format: PixelFormat,
) -> Vec<Packet> {
    let mut out = Vec::new();
    for t in 0..n {
        enc.send_frame(&Frame::Video(video_frame(width, height, t, format)))
            .expect("send_frame");
        // A finished GOP's packets are available before the flush.
        while let Ok(packet) = enc.receive_packet() {
            out.push(packet);
        }
    }
    assert!(matches!(enc.receive_packet(), Err(Error::NeedMore)));
    enc.flush().expect("flush");
    loop {
        match enc.receive_packet() {
            Ok(p) => out.push(p),
            Err(Error::Eof) => break,
            Err(e) => panic!("receive_packet: {e:?}"),
        }
    }
    out
}

fn encode_one(p: &CodecParameters, n: usize) -> Vec<u8> {
    let format = p.pixel_format.unwrap();
    let (w, h) = (p.width.unwrap() as usize, p.height.unwrap() as usize);
    let mut enc = make_encoder(p).expect("make_encoder");
    let packets = encode_packets(enc.as_mut(), w, h, n, format);
    assert_eq!(packets.len(), n, "one packet per coded frame");
    assert!(packets[0].flags.keyframe);
    assert!(packets.iter().all(|p| p.duration == Some(1)));
    packets.into_iter().flat_map(|p| p.data).collect()
}

/// The first picture's header + coding extension.
fn first_picture(
    stream: &[u8],
) -> (
    Mpeg2PictureHeader,
    oxideav_mpeg12video::PictureCodingExtension,
) {
    let pos = stream
        .windows(4)
        .position(|w| w == [0, 0, 1, 0])
        .expect("picture_start_code");
    Mpeg2PictureHeader::parse_with_extension(&stream[pos..]).expect("picture layer")
}

#[test]
fn every_schema_key_is_documented_and_defaults_match() {
    // The typed struct's defaults agree with the schema defaults for
    // every scalar key, and every key applies.
    let defaults = Mpeg12EncoderOptions::default();
    for field in Mpeg12EncoderOptions::SCHEMA {
        let mut o = Mpeg12EncoderOptions::default();
        assert!(!field.help.is_empty(), "{}: help text", field.name);
        o.apply(field.name, &field.default)
            .or_else(|e| {
                // Enum defaults are the empty placeholder; the typed
                // default is the first value.
                if matches!(field.kind, oxideav_core::OptionKind::Enum(_)) {
                    Ok(())
                } else {
                    Err(e)
                }
            })
            .unwrap_or_else(|e| panic!("{}: apply default: {e:?}", field.name));
        assert_eq!(
            o, defaults,
            "{}: schema default == typed default",
            field.name
        );
    }
    let bad = Mpeg12EncoderOptions::default().apply("bogus", &oxideav_core::OptionValue::U32(1));
    assert!(bad.is_err());
}

#[test]
fn chroma_format_follows_the_pixel_format_on_every_structure() {
    for (format, chroma) in [
        (PixelFormat::Yuv422P, ChromaFormat::Yuv422),
        (PixelFormat::Yuv444P, ChromaFormat::Yuv444),
    ] {
        for structure in ["frame", "field", "frame_field", "field_adaptive"] {
            let p = with(
                &params(MPEG2_CODEC_ID_STR, 48, 64, format),
                &[
                    ("picture_structure", structure),
                    ("b_between", "1"),
                    ("anchors_per_gop", "2"),
                ],
            );
            let stream = encode_one(&p, 4);
            let seq = Mpeg2Sequence::from_buf(&stream).expect("sequence");
            assert_eq!(
                seq.extension.chroma_format, chroma,
                "{structure} {format:?}"
            );
            let decoded = decode_video_sequence(&stream).expect("decode");
            assert_eq!(decoded.len(), 4, "{structure} {format:?}");
            assert_eq!(decoded[0].frame.chroma_format, chroma);

            // The runtime decoder hands back planes of the same format.
            let mut dec = make_decoder(&p).expect("make_decoder");
            dec.send_packet(&Packet::new(0, oxideav_core::TimeBase::new(1, 25), stream))
                .unwrap();
            dec.flush().unwrap();
            let Ok(Frame::Video(vf)) = dec.receive_frame() else {
                panic!("frame");
            };
            let (cw, ch) = decoded[0].frame.visible_chroma_dims();
            assert_eq!(
                vf.planes[1].data.len(),
                cw * ch,
                "{structure} {format:?} chroma plane"
            );
        }
    }
}

#[test]
fn picture_structure_selects_the_assembler() {
    let base = params(MPEG2_CODEC_ID_STR, 48, 64, PixelFormat::Yuv420P);

    // frame (default): progressive frame pictures.
    let stream = encode_one(&with(&base, &[("b_between", "1")]), 3);
    let (_, ext) = first_picture(&stream);
    assert_eq!(ext.picture_structure, PictureStructure::Frame);
    assert!(ext.frame_pred_frame_dct && ext.progressive_frame);
    assert!(
        Mpeg2Sequence::from_buf(&stream)
            .unwrap()
            .extension
            .progressive_sequence
    );

    // interlaced frame pictures keep frame_pred_frame_dct = 1.
    let stream = encode_one(
        &with(&base, &[("interlaced", "true"), ("b_between", "1")]),
        3,
    );
    let (_, ext) = first_picture(&stream);
    assert!(ext.frame_pred_frame_dct && !ext.progressive_frame);
    assert!(
        !Mpeg2Sequence::from_buf(&stream)
            .unwrap()
            .extension
            .progressive_sequence
    );

    // field pairs.
    let stream = encode_one(
        &with(&base, &[("picture_structure", "field"), ("b_between", "1")]),
        3,
    );
    let (_, ext) = first_picture(&stream);
    assert_eq!(ext.picture_structure, PictureStructure::TopField);
    assert_eq!(decode_video_sequence(&stream).unwrap().len(), 3);

    // frame-field: frame pictures with frame_pred_frame_dct = 0.
    let stream = encode_one(
        &with(
            &base,
            &[("picture_structure", "frame_field"), ("b_between", "1")],
        ),
        3,
    );
    let (_, ext) = first_picture(&stream);
    assert_eq!(ext.picture_structure, PictureStructure::Frame);
    assert!(!ext.frame_pred_frame_dct);
    assert_eq!(decode_video_sequence(&stream).unwrap().len(), 3);

    // adaptive field modes with dual-prime (b_between = 0).
    let stream = encode_one(
        &with(
            &base,
            &[
                ("picture_structure", "field_adaptive"),
                ("b_between", "0"),
                ("dual_prime", "true"),
            ],
        ),
        3,
    );
    let (_, ext) = first_picture(&stream);
    assert_eq!(ext.picture_structure, PictureStructure::TopField);
    assert_eq!(decode_video_sequence(&stream).unwrap().len(), 3);

    // frame-field with dual-prime.
    let stream = encode_one(
        &with(
            &base,
            &[
                ("picture_structure", "frame_field"),
                ("b_between", "0"),
                ("dual_prime", "true"),
            ],
        ),
        3,
    );
    assert_eq!(decode_video_sequence(&stream).unwrap().len(), 3);

    // field pairs need a 32-line-multiple height.
    let odd = with(
        &params(MPEG2_CODEC_ID_STR, 48, 48, PixelFormat::Yuv420P),
        &[("picture_structure", "field")],
    );
    assert!(make_encoder(&odd).is_err());
}

#[test]
fn entropy_flags_reach_the_picture_coding_extension() {
    let p = with(
        &params(MPEG2_CODEC_ID_STR, 48, 32, PixelFormat::Yuv420P),
        &[
            ("intra_vlc_format", "true"),
            ("alternate_scan", "1"),
            ("q_scale_type", "yes"),
            ("intra_dc_precision", "2"),
            ("f_code", "2"),
            ("backward_f_code", "4"),
            ("b_between", "1"),
        ],
    );
    let stream = encode_one(&p, 3);
    let (_, ext) = first_picture(&stream);
    assert!(ext.intra_vlc_format && ext.alternate_scan && ext.q_scale_type);
    assert_eq!(ext.intra_dc_precision, 2);
    // The B picture (third in coded order: I P B) carries both f_codes.
    let third = stream
        .windows(4)
        .enumerate()
        .filter(|(_, w)| *w == [0, 0, 1, 0])
        .nth(2)
        .map(|(i, _)| i)
        .unwrap();
    let (hdr, bext) = Mpeg2PictureHeader::parse_with_extension(&stream[third..]).unwrap();
    assert_eq!(hdr.picture_coding_type, PictureCodingType::Bidirectional);
    assert_eq!((bext.f_code_fwd_horiz, bext.f_code_bwd_horiz), (2, 4));
    assert_eq!(decode_video_sequence(&stream).unwrap().len(), 3);

    for (k, v) in [
        ("intra_dc_precision", "4"),
        ("backward_f_code", "10"),
        ("f_code", "0"),
    ] {
        assert!(make_encoder(&with(&p, &[(k, v)])).is_err(), "{k}={v}");
    }
}

#[test]
fn frame_encode_options_skips_concealment_and_cadence() {
    let base = params(MPEG2_CODEC_ID_STR, 64, 48, PixelFormat::Yuv420P);
    let p = with(
        &base,
        &[
            ("skipped_macroblocks", "true"),
            ("concealment_motion_vectors", "true"),
            ("b_between", "1"),
        ],
    );
    let stream = encode_one(&p, 4);
    let (_, ext) = first_picture(&stream);
    assert!(ext.concealment_motion_vectors, "I picture carries the flag");
    assert_eq!(decode_video_sequence(&stream).unwrap().len(), 4);

    // Constant cadence flags in a progressive sequence: tff needs rff.
    let stream = encode_one(
        &with(
            &base,
            &[("top_field_first", "true"), ("repeat_first_field", "true")],
        ),
        2,
    );
    let decoded = decode_video_sequence(&stream).unwrap();
    assert!(
        decoded
            .iter()
            .all(|d| d.top_field_first && d.repeat_first_field)
    );
    assert!(make_encoder(&with(&base, &[("top_field_first", "true")])).is_ok());
    let mut enc = make_encoder(&with(&base, &[("top_field_first", "true")])).unwrap();
    enc.send_frame(&Frame::Video(video_frame(64, 48, 0, PixelFormat::Yuv420P)))
        .unwrap();
    assert!(
        enc.flush().is_err(),
        "§6.3.10: top_field_first without repeat_first_field is rejected in a progressive sequence"
    );

    // 3:2 pulldown over an interlaced frame sequence.
    let stream = encode_one(
        &with(
            &base,
            &[
                ("interlaced", "true"),
                ("pulldown", "3:2"),
                ("b_between", "1"),
            ],
        ),
        8,
    );
    let decoded = decode_video_sequence(&stream).unwrap();
    let fields: u32 = decoded.iter().map(|d| d.output_field_count()).sum();
    assert_eq!(fields, 20, "8 frames at 3:2 = 20 fields");
    assert!(decoded.iter().all(|d| d.progressive_frame));
    // Pulldown needs an interlaced sequence.
    assert!(make_encoder(&with(&base, &[("pulldown", "3:2")])).is_err());
    // FrameEncodeOptions belong to frame pictures.
    assert!(
        make_encoder(&with(
            &base,
            &[
                ("picture_structure", "field"),
                ("concealment_motion_vectors", "true")
            ]
        ))
        .is_err()
    );
}

#[test]
fn cbr_rate_control_holds_the_vbv_on_frame_field_and_mpeg1() {
    // MPEG-2 frame pictures: bit_rate from CodecParameters::bit_rate.
    let mut p = with(
        &params(MPEG2_CODEC_ID_STR, 64, 48, PixelFormat::Yuv420P),
        &[("rate_control", "cbr"), ("b_between", "1")],
    );
    p.bit_rate = Some(240_000);
    p.frame_rate = Some(Rational::new(30_000, 1001));
    let mut enc = make_encoder(&p).expect("make_encoder");
    assert_eq!(enc.output_params().bit_rate, Some(240_000));
    let packets = encode_packets(enc.as_mut(), 64, 48, 5, PixelFormat::Yuv420P);
    let stream: Vec<u8> = packets.iter().flat_map(|p| p.data.clone()).collect();
    let stream = &stream;
    assert_eq!(
        packets[0].time_base,
        oxideav_core::TimeBase::new(1001, 30_000)
    );
    let seq = Mpeg2Sequence::from_buf(stream).unwrap();
    assert_eq!(seq.header.bit_rate, 600, "240 kbit/s in 400 bit/s units");
    assert_eq!(seq.header.frame_rate_code, 4, "29.97 = Table 6-4 code 4");
    let report = verify_cbr_stream(stream, VbvStandard::Mpeg2).expect("VBV conformant");
    assert_eq!(report.pictures.len(), 5);

    // Field pictures under CBR, explicit bit_rate_value / vbv size.
    let p = with(
        &params(MPEG2_CODEC_ID_STR, 48, 64, PixelFormat::Yuv422P),
        &[
            ("rate_control", "cbr"),
            ("picture_structure", "field"),
            ("bit_rate_value", "600"),
            ("vbv_buffer_size_value", "4"),
            ("b_between", "1"),
        ],
    );
    let stream = encode_one(&p, 4);
    let report = verify_cbr_stream(&stream, VbvStandard::Mpeg2).expect("VBV conformant");
    assert_eq!(report.pictures.len(), 8, "one record per field");
    assert_eq!(report.buffer_size_bits, 4 * 16 * 1024);

    // MPEG-1 CBR.
    let p = with(
        &params(MPEG1_CODEC_ID_STR, 64, 48, PixelFormat::Yuv420P),
        &[
            ("rate_control", "cbr"),
            ("bit_rate_value", "600"),
            ("vbv_buffer_size_value", "4"),
            ("b_between", "2"),
        ],
    );
    let stream = encode_one(&p, 6);
    verify_cbr_stream(&stream, VbvStandard::Mpeg1).expect("11172-2 VBV conformant");
    assert_eq!(decode_video_sequence(&stream).unwrap().len(), 6);

    // No CBR controller for the frame-field / adaptive assemblers.
    assert!(
        make_encoder(&with(
            &params(MPEG2_CODEC_ID_STR, 64, 64, PixelFormat::Yuv420P),
            &[
                ("rate_control", "cbr"),
                ("picture_structure", "frame_field")
            ]
        ))
        .is_err()
    );
    // Unknown frame rate.
    let mut p = params(MPEG2_CODEC_ID_STR, 64, 48, PixelFormat::Yuv420P);
    p.frame_rate = Some(Rational::new(7, 1));
    assert!(make_encoder(&p).is_err());
}

#[test]
fn data_partitioning_emits_two_partition_packets() {
    let base = params(MPEG2_CODEC_ID_STR, 64, 48, PixelFormat::Yuv420P);
    let plain = encode_one(&with(&base, &[("b_between", "1")]), 4);
    let p = with(&base, &[("b_between", "1"), ("data_partitioning", "64")]);
    let mut enc = make_encoder(&p).expect("make_encoder");
    let packets = encode_packets(enc.as_mut(), 64, 48, 4, PixelFormat::Yuv420P);
    assert_eq!(packets.len(), 2, "partition 0 then partition 1");
    assert!(
        packets
            .iter()
            .all(|p| p.pts == Some(0) && p.duration == Some(4))
    );
    let merged = merge_data_partitions(&packets[0].data, &packets[1].data).expect("merge");
    assert_eq!(
        merged, plain,
        "the partition pair re-forms the plain stream"
    );
    for bad in ["4", "63", "128"] {
        assert!(
            make_encoder(&with(&base, &[("data_partitioning", bad)])).is_err(),
            "{bad}"
        );
    }
    assert!(
        make_encoder(&with(
            &params(MPEG1_CODEC_ID_STR, 64, 48, PixelFormat::Yuv420P),
            &[("data_partitioning", "1")]
        ))
        .is_err()
    );
}

#[test]
fn mpeg1_options_d_pictures_and_rejections() {
    let base = params(MPEG1_CODEC_ID_STR, 48, 32, PixelFormat::Yuv420P);
    let stream = encode_one(
        &with(
            &base,
            &[("mpeg1_d_pictures", "true"), ("anchors_per_gop", "2")],
        ),
        4,
    );
    let decoded = decode_video_sequence(&stream).unwrap();
    assert_eq!(decoded.len(), 4);
    assert!(
        decoded
            .iter()
            .all(|d| d.picture_coding_type == PictureCodingType::DcIntra)
    );
    assert!(
        !stream.windows(4).any(|w| w == [0, 0, 1, 0xB5]),
        "11172-2: no extension"
    );

    for (k, v) in [
        ("picture_structure", "field"),
        ("interlaced", "true"),
        ("intra_vlc_format", "true"),
        ("concealment_motion_vectors", "true"),
        ("dual_prime", "true"),
        ("f_code", "8"),
    ] {
        assert!(
            make_encoder(&with(&base, &[(k, v)])).is_err(),
            "mpeg1video {k}={v}"
        );
    }
    assert!(
        make_encoder(&with(
            &params(MPEG2_CODEC_ID_STR, 48, 32, PixelFormat::Yuv420P),
            &[("mpeg1_d_pictures", "true")]
        ))
        .is_err()
    );
}

#[test]
fn registry_path_honours_the_options() {
    let mut ctx = RuntimeContext::new();
    oxideav_mpeg12video::register(&mut ctx);
    let p = with(
        &params(MPEG2_CODEC_ID_STR, 48, 64, PixelFormat::Yuv422P),
        &[
            ("picture_structure", "field"),
            ("b_between", "1"),
            ("intra_vlc_format", "true"),
        ],
    );
    let mut enc = ctx.codecs.first_encoder(&p).expect("registry encoder");
    assert_eq!(enc.output_params().pixel_format, Some(PixelFormat::Yuv422P));
    let packets = encode_packets(enc.as_mut(), 48, 64, 3, PixelFormat::Yuv422P);
    let stream: Vec<u8> = packets.iter().flat_map(|p| p.data.clone()).collect();
    let stream = &stream;
    let (_, ext) = first_picture(stream);
    assert_eq!(ext.picture_structure, PictureStructure::TopField);
    assert!(ext.intra_vlc_format);
    assert_eq!(
        Mpeg2Sequence::from_buf(stream)
            .unwrap()
            .extension
            .chroma_format,
        ChromaFormat::Yuv422
    );
    assert_eq!(decode_video_sequence(stream).unwrap().len(), 3);
}

/// Found by the black-box matrix: `repeat_first_field` in an
/// interlaced sequence was rejected (§6.3.10 requires
/// `progressive_frame = 1` for a repeated field, and the adapter left
/// it at the interlaced default); the adapter now sets it.
#[test]
fn repeat_first_field_in_an_interlaced_sequence_sets_progressive_frame() {
    let p = params(MPEG2_CODEC_ID_STR, 48, 32, PixelFormat::Yuv420P);
    let stream = encode_one(
        &with(
            &p,
            &[
                ("interlaced", "true"),
                ("top_field_first", "true"),
                ("repeat_first_field", "true"),
            ],
        ),
        4,
    );
    let (_, ext) = first_picture(&stream);
    assert!(ext.repeat_first_field && ext.top_field_first && ext.progressive_frame);
    assert_eq!(decode_video_sequence(&stream).expect("decode").len(), 4);
}

/// Found by the black-box matrix: at 15 Mbit/s the default VBV size
/// (20 × 16 kbit) held less than one picture period of the rate, so no
/// stream could satisfy it and the controller failed. The default now
/// holds at least four picture periods.
#[test]
fn cbr_default_buffer_holds_several_pictures_at_high_rates() {
    for (id, standard) in [
        (MPEG2_CODEC_ID_STR, VbvStandard::Mpeg2),
        (MPEG1_CODEC_ID_STR, VbvStandard::Mpeg1),
    ] {
        let mut p = params(id, 48, 32, PixelFormat::Yuv420P);
        p.bit_rate = Some(15_000_000);
        let stream = encode_one(&with(&p, &[("rate_control", "cbr")]), 6);
        let report = verify_cbr_stream(&stream, standard).expect("VBV conformant");
        assert!(report.buffer_size_bits >= 4 * 15_000_000 / 25);
    }
}

/// ISO/IEC 11172-2 §2.4.3.5 has 175 slice rows; the encoder starts a
/// slice per macroblock row, so taller pictures are refused instead of
/// producing a stream no decoder accepts (found by the black-box
/// matrix at 34×4095).
#[test]
fn mpeg1_pictures_taller_than_175_slice_rows_are_refused() {
    let p = params(MPEG1_CODEC_ID_STR, 16, 2816, PixelFormat::Yuv420P);
    let mut enc = make_encoder(&p).expect("make_encoder");
    enc.send_frame(&Frame::Video(video_frame(
        16,
        2816,
        0,
        PixelFormat::Yuv420P,
    )))
    .expect("send_frame");
    assert!(enc.flush().is_err());
    let p = params(MPEG1_CODEC_ID_STR, 16, 2800, PixelFormat::Yuv420P);
    assert_eq!(decode_video_sequence(&encode_one(&p, 1)).unwrap().len(), 1);
}

/// The encoder releases a closed GOP as soon as the next one starts:
/// one packet per frame in coded order, keyframes at the GOP starts
/// (each with its own sequence header and a running GOP time code),
/// the input frames' presentation times on the packets — and the
/// concatenation decodes to what one whole-sequence encode gives.
#[test]
fn gops_are_released_while_frames_arrive() {
    let (w, h, n) = (48usize, 32usize, 20usize);
    let p = with(
        &params(MPEG2_CODEC_ID_STR, w as u32, h as u32, PixelFormat::Yuv420P),
        &[("b_between", "2"), ("anchors_per_gop", "2")],
    );
    let gop = 7usize; // I + 2 × (B B P)
    let mut enc = make_encoder(&p).expect("make_encoder");
    let mut packets: Vec<Packet> = Vec::new();
    let mut released_before_flush = 0;
    for t in 0..n {
        let mut frame = video_frame(w, h, t, PixelFormat::Yuv420P);
        frame.pts = Some(1000 + 40 * t as i64);
        enc.send_frame(&Frame::Video(frame)).expect("send_frame");
        while let Ok(packet) = enc.receive_packet() {
            packets.push(packet);
        }
        // GOP k is complete when frame (k + 1) * gop has arrived.
        assert_eq!(packets.len(), (t / gop) * gop, "after frame {t}");
        released_before_flush = packets.len();
    }
    assert_eq!(released_before_flush, 14);
    enc.flush().expect("flush");
    while let Ok(packet) = enc.receive_packet() {
        packets.push(packet);
    }
    assert_eq!(packets.len(), n);

    let keyframes: Vec<usize> = (0..n).filter(|&i| packets[i].flags.keyframe).collect();
    assert_eq!(keyframes, vec![0, 7, 14]);
    for &k in &keyframes {
        assert_eq!(packets[k].data[..4], [0, 0, 1, 0xB3], "sequence header");
    }
    // Coded order I P B B P B B: presentation times are the frames'.
    let pts: Vec<i64> = packets.iter().map(|p| p.pts.unwrap()).collect();
    let display = [0, 3, 1, 2, 6, 4, 5];
    for (i, &t) in pts.iter().enumerate().take(14) {
        let frame = (i / gop) * gop + display[i % gop];
        assert_eq!(t, 1000 + 40 * frame as i64, "packet {i}");
    }
    let dts: Vec<i64> = packets.iter().map(|p| p.dts.unwrap()).collect();
    assert!(dts.windows(2).all(|w| w[0] < w[1]));
    assert!(dts.iter().zip(&pts).all(|(d, p)| d <= p));

    let stream: Vec<u8> = packets.iter().flat_map(|p| p.data.clone()).collect();
    assert_eq!(stream[stream.len() - 4..], [0, 0, 1, 0xB7]);
    assert_eq!(
        stream.windows(4).filter(|w| *w == [0, 0, 1, 0xB7]).count(),
        1,
        "one sequence_end_code"
    );
    // GOP time codes count display frames (25 frames/s: 0, 7, 14).
    let time_codes: Vec<u32> = stream
        .windows(4)
        .enumerate()
        .filter(|(_, w)| *w == [0, 0, 1, 0xB8])
        .map(|(i, _)| {
            let gop = oxideav_mpeg12video::gop_header::Mpeg2Gop::parse(&stream[i..]).unwrap();
            assert!(gop.closed_gop && !gop.broken_link);
            u32::from(gop.time_code.seconds) * 25 + u32::from(gop.time_code.pictures)
        })
        .collect();
    assert_eq!(time_codes, vec![0, 7, 14]);

    // The same frames through the whole-sequence assembler.
    let frames: Vec<oxideav_mpeg12video::FrameBuffer> = (0..n)
        .map(|t| {
            let v = video_frame(w, h, t, PixelFormat::Yuv420P);
            let mut f = oxideav_mpeg12video::FrameBuffer::new(w, h, ChromaFormat::Yuv420);
            for y in 0..h {
                for x in 0..w {
                    f.y.put_sample(x, y, v.planes[0].data[y * w + x]);
                }
            }
            for y in 0..h / 2 {
                for x in 0..w / 2 {
                    f.cb.put_sample(x, y, v.planes[1].data[y * (w / 2) + x]);
                    f.cr.put_sample(x, y, v.planes[2].data[y * (w / 2) + x]);
                }
            }
            f
        })
        .collect();
    let (whole, _) = oxideav_mpeg12video::encode_display_order_gop_sequence_with_options(
        &frames,
        2,
        2,
        oxideav_mpeg12video::IntraPictureParams::new(
            w,
            h,
            ChromaFormat::Yuv420,
            true,
            0,
            false,
            false,
            false,
            true,
        ),
        6,
        3,
        3,
        &oxideav_mpeg12video::quant_matrix_extension::QuantMatrixExtension::default(),
        &|_| oxideav_mpeg12video::FrameEncodeOptions::default().with_skipped_macroblocks(true),
    )
    .expect("whole-sequence encode");
    let a = decode_video_sequence(&stream).expect("decode packets");
    let b = decode_video_sequence(&whole).expect("decode whole");
    assert_eq!(a.len(), n);
    for (x, y) in a.iter().zip(&b) {
        assert_eq!(x.frame.y.samples(), y.frame.y.samples());
        assert_eq!(x.frame.cb.samples(), y.frame.cb.samples());
        assert_eq!(x.frame.cr.samples(), y.frame.cr.samples());
    }
}
