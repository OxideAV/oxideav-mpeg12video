//! End-to-end runtime [`oxideav_core::Decoder`] wiring for MPEG-1 /
//! MPEG-2 video.
//!
//! These tests exercise the packet-oriented decoder adapter — both the
//! direct [`oxideav_mpeg12video::make_decoder`] factory and the
//! [`oxideav_core::register`] registry path — and prove the frames it
//! emits are **sample-exact** with the whole-stream driver
//! [`oxideav_mpeg12video::decode_video_sequence`] they wrap.
//!
//! Clean-room: the fixture is an opaque black-box-encoded elementary
//! stream (its encoder source is not read).

use oxideav_core::{
    CodecId, CodecParameters, Decoder, Error, Frame, Packet, PixelFormat, RuntimeContext, TimeBase,
};
use oxideav_mpeg12video::{
    FrameBuffer, MPEG1_CODEC_ID_STR, MPEG2_CODEC_ID_STR, Mpeg12Decoder, decode_video_sequence,
    make_decoder,
};

/// A real 352×240 4:2:0 MPEG-2 elementary stream from an opaque
/// black-box encoder (its source is not read). Its single coded picture
/// is an I-picture.
const FIXTURE: &[u8] = include_bytes!("fixtures/ffmpeg-352x240-25fps.m2v");

fn tb() -> TimeBase {
    TimeBase::new(1, 25)
}

/// Assert a decoded [`Frame::Video`] carries exactly the planar samples
/// of the reference [`FrameBuffer`].
fn assert_frame_matches(frame: &Frame, reference: &FrameBuffer) {
    let Frame::Video(vf) = frame else {
        panic!("expected a video frame");
    };
    assert_eq!(vf.planes.len(), 3, "Y/Cb/Cr planes");
    let expect = [
        (reference.y.width(), reference.y.samples()),
        (reference.cb.width(), reference.cb.samples()),
        (reference.cr.width(), reference.cr.samples()),
    ];
    for (i, (plane, (w, samples))) in vf.planes.iter().zip(expect).enumerate() {
        assert_eq!(plane.stride, w, "plane {i} stride == width");
        assert_eq!(plane.data.as_slice(), samples, "plane {i} samples exact");
    }
}

#[test]
fn make_decoder_reports_requested_codec_id() {
    for id in [MPEG1_CODEC_ID_STR, MPEG2_CODEC_ID_STR] {
        let params = CodecParameters::video(CodecId::new(id));
        let dec = make_decoder(&params).expect("make_decoder");
        assert_eq!(dec.codec_id().as_str(), id);
    }
}

#[test]
fn registry_installs_both_codec_ids() {
    let mut ctx = RuntimeContext::new();
    oxideav_mpeg12video::register(&mut ctx);
    for id in [MPEG1_CODEC_ID_STR, MPEG2_CODEC_ID_STR] {
        let params = CodecParameters::video(CodecId::new(id));
        let dec = ctx
            .codecs
            .first_decoder(&params)
            .unwrap_or_else(|_| panic!("{id} decoder factory"));
        assert_eq!(dec.codec_id().as_str(), id);
    }
}

#[test]
fn decoder_needs_flush_before_committing_frames() {
    // Before flush the §6.1.1.11 reorder cannot commit — the trailing
    // anchors are unknown, so the decoder asks for more input.
    let params = CodecParameters::video(CodecId::new(MPEG2_CODEC_ID_STR));
    let mut dec = make_decoder(&params).expect("make_decoder");
    dec.send_packet(&Packet::new(0, tb(), FIXTURE.to_vec()))
        .expect("send_packet");
    assert!(matches!(dec.receive_frame(), Err(Error::NeedMore)));
    dec.flush().expect("flush");
    assert!(dec.receive_frame().is_ok(), "frame available after flush");
    assert!(matches!(dec.receive_frame(), Err(Error::Eof)), "then Eof");
}

#[test]
fn decoder_sample_exact_on_real_fixture() {
    let reference = decode_video_sequence(FIXTURE).expect("reference decode");
    assert_eq!(reference.len(), 1, "fixture has one coded picture");

    let params = CodecParameters::video(CodecId::new(MPEG2_CODEC_ID_STR));
    let mut dec = make_decoder(&params).expect("make_decoder");
    dec.send_packet(&Packet::new(0, tb(), FIXTURE.to_vec()))
        .expect("send_packet");
    dec.flush().expect("flush");

    let frame = dec.receive_frame().expect("frame");
    assert_frame_matches(&frame, &reference[0].frame);

    // 352×240 4:2:0 geometry survives the plane conversion, and the
    // single frame is stamped with display index 0.
    let Frame::Video(vf) = &frame else {
        unreachable!()
    };
    assert_eq!(vf.pts, Some(0));
    assert_eq!(
        (vf.planes[0].stride, vf.planes[0].data.len()),
        (352, 352 * 240)
    );
    assert_eq!(vf.planes[1].stride, 176);
    assert_eq!(vf.planes[1].data.len(), 176 * 120);
    assert!(matches!(dec.receive_frame(), Err(Error::Eof)));
}

#[test]
fn multi_packet_input_is_concatenated() {
    let reference = decode_video_sequence(FIXTURE).expect("reference decode");

    // Split the elementary stream across two packets at an arbitrary
    // byte boundary — the decoder must concatenate them before decode.
    let split = FIXTURE.len() / 2;
    let mut dec = Mpeg12Decoder::new(CodecId::new(MPEG2_CODEC_ID_STR));
    dec.send_packet(&Packet::new(0, tb(), FIXTURE[..split].to_vec()))
        .expect("send_packet 1");
    dec.send_packet(&Packet::new(0, tb(), FIXTURE[split..].to_vec()))
        .expect("send_packet 2");
    dec.flush().expect("flush");

    let frame = dec.receive_frame().expect("frame");
    assert_frame_matches(&frame, &reference[0].frame);
    assert!(matches!(dec.receive_frame(), Err(Error::Eof)));
}

#[test]
fn reset_clears_accumulated_state() {
    let reference = decode_video_sequence(FIXTURE).expect("reference");
    let mut dec = Mpeg12Decoder::new(CodecId::new(MPEG2_CODEC_ID_STR));

    // Feed a stream and drain it to completion.
    dec.send_packet(&Packet::new(0, tb(), FIXTURE.to_vec()))
        .expect("send_packet");
    dec.flush().expect("flush");
    assert_frame_matches(&dec.receive_frame().expect("frame"), &reference[0].frame);
    assert!(matches!(dec.receive_frame(), Err(Error::Eof)));

    // reset() returns the decoder to a fresh state — dropping the
    // consumed stream — so a new stream decodes as if it were the first.
    dec.reset().expect("reset");
    dec.send_packet(&Packet::new(0, tb(), FIXTURE.to_vec()))
        .expect("re-send");
    dec.flush().expect("re-flush");
    let frame = dec.receive_frame().expect("frame");
    assert_frame_matches(&frame, &reference[0].frame);
    assert!(matches!(dec.receive_frame(), Err(Error::Eof)));
}

/// A multi-picture I B B P stream (black-box encoded, see
/// `fixtures/conformance/README.md`).
const IBBP: &[u8] = include_bytes!("fixtures/conformance/mpeg2-ibbp-96x64.m2v");

#[test]
fn decoder_releases_frames_while_packets_arrive() {
    // A long stream must not be held whole: once a picture's end has
    // arrived and the reorder can commit it, it is available before
    // flush, sample-exact with the whole-stream driver.
    let reference = decode_video_sequence(IBBP).expect("reference decode");
    let params = CodecParameters::video(CodecId::new(MPEG2_CODEC_ID_STR));
    let mut dec = make_decoder(&params).expect("make_decoder");
    let mut frames = Vec::new();
    for piece in IBBP.chunks(2048) {
        dec.send_packet(&Packet::new(0, tb(), piece.to_vec()))
            .expect("send_packet");
        while let Ok(frame) = dec.receive_frame() {
            frames.push(frame);
        }
    }
    let before_flush = frames.len();
    dec.flush().expect("flush");
    loop {
        match dec.receive_frame() {
            Ok(frame) => frames.push(frame),
            Err(Error::Eof) => break,
            Err(e) => panic!("receive_frame: {e:?}"),
        }
    }
    assert!(
        before_flush + 2 >= reference.len(),
        "{before_flush} of {} before flush",
        reference.len()
    );
    assert_eq!(frames.len(), reference.len());
    for (n, (frame, want)) in frames.iter().zip(&reference).enumerate() {
        assert_frame_matches(frame, &want.frame);
        let Frame::Video(vf) = frame else {
            unreachable!()
        };
        assert_eq!(vf.pts, Some(n as i64), "display index stamp");
    }
}

const IBBP_64X48: &[u8] = include_bytes!("fixtures/selfenc/selfenc-ibbp-64x48.m2v");
const INTRA_100X62: &[u8] = include_bytes!("fixtures/selfenc/selfenc-intra-100x62.m2v");
const YUV422_64X48: &[u8] = include_bytes!("fixtures/selfenc/selfenc-422-ibbp-64x48.m2v");

/// Drain every frame (and event) the decoder holds.
fn drain(dec: &mut dyn oxideav_core::Decoder) -> Vec<Frame> {
    let mut out = Vec::new();
    loop {
        match dec.receive_frame() {
            Ok(frame) => out.push(frame),
            Err(Error::Eof | Error::NeedMore) => return out,
            Err(e) => panic!("receive_frame: {e:?}"),
        }
    }
}

/// The byte ranges of the coded pictures of `stream`: each starts at
/// the sequence / GOP header that precedes its picture start code (or
/// at the picture start code) and ends where the next one starts.
fn access_units(stream: &[u8]) -> Vec<&[u8]> {
    let mut starts = Vec::new();
    let mut unit_open = false;
    let mut i = 0;
    while i + 4 <= stream.len() {
        if stream[i..i + 3] == [0, 0, 1] {
            match stream[i + 3] {
                0xB3 | 0xB8 if !unit_open => {
                    starts.push(i);
                    unit_open = true;
                }
                0x00 => {
                    if !unit_open {
                        starts.push(i);
                    }
                    unit_open = false;
                }
                _ => {}
            }
            i += 4;
        } else {
            i += 1;
        }
    }
    starts
        .iter()
        .enumerate()
        .map(|(n, &s)| &stream[s..starts.get(n + 1).copied().unwrap_or(stream.len())])
        .collect()
}

/// Size, pixel format and frame rate are stream parameters: stated by
/// `output_params` once the first frame exists, re-announced in band
/// (with a generation step) when a sequence header changes them.
#[test]
fn output_params_follow_the_sequence_headers() {
    let params = CodecParameters::video(CodecId::new(MPEG2_CODEC_ID_STR));
    let mut dec = make_decoder(&params).expect("make_decoder");
    assert!(dec.output_params().is_none(), "nothing known before data");
    assert_eq!(dec.output_params_generation(), 0);

    let stream = [IBBP_64X48, INTRA_100X62, YUV422_64X48].concat();
    dec.send_packet(&Packet::new(0, tb(), stream)).unwrap();
    dec.flush().unwrap();
    let out = drain(dec.as_mut());

    let first = decode_video_sequence(IBBP_64X48).unwrap().len();
    let second = decode_video_sequence(INTRA_100X62).unwrap().len();
    let third = decode_video_sequence(YUV422_64X48).unwrap().len();
    assert_eq!(out.len(), first + second + third + 2, "two events");
    assert!(out[..first].iter().all(|f| matches!(f, Frame::Video(_))));

    let event = out[first].stream_params().expect("size-change event");
    assert_eq!((event.width, event.height), (Some(100), Some(62)));
    assert_eq!(event.pixel_format, Some(PixelFormat::Yuv420P));
    let Frame::Video(vf) = &out[first + 1] else {
        panic!("a frame follows the event");
    };
    assert_eq!(vf.planes[0].stride, 100);

    let event = out[first + 1 + second]
        .stream_params()
        .expect("chroma-format-change event");
    assert_eq!((event.width, event.height), (Some(64), Some(48)));
    assert_eq!(event.pixel_format, Some(PixelFormat::Yuv422P));

    assert_eq!(dec.output_params_generation(), 2);
    let now = dec.output_params().expect("params after frames");
    assert_eq!(now.pixel_format, Some(PixelFormat::Yuv422P));
    assert_eq!(now.frame_rate, Some(oxideav_core::Rational::new(25, 1)));
    assert_eq!(dec.output_pixel_format(), Some(PixelFormat::Yuv422P));
}

/// Each frame carries the presentation time of the packet that held
/// its picture, so container times survive the display reorder.
#[test]
fn frames_carry_their_packets_presentation_times() {
    let reference = decode_video_sequence(IBBP_64X48).unwrap();
    let units = access_units(IBBP_64X48);
    assert_eq!(units.len(), reference.len());
    // Coded order I P B B P B B: a container stamps each access unit
    // with its display time. Recover that from the display-order
    // decode: the n-th coded picture is the one whose temporal
    // reference a coded-order decode reports.
    let mut coded = oxideav_mpeg12video::SequenceDecoder::new().coded_order();
    coded.push(IBBP_64X48);
    coded.finish();
    let mut display_index = Vec::new();
    while let Some(frame) = coded.next_frame().unwrap() {
        display_index.push(i64::from(frame.temporal_reference));
    }
    assert_eq!(display_index.len(), units.len());

    let params = CodecParameters::video(CodecId::new(MPEG2_CODEC_ID_STR));
    let mut dec = make_decoder(&params).expect("make_decoder");
    let mut pts = Vec::new();
    for (unit, index) in units.iter().zip(&display_index) {
        let mut packet = Packet::new(0, tb(), unit.to_vec());
        packet.pts = Some(1000 + 40 * index);
        dec.send_packet(&packet).unwrap();
        pts.extend(drain(dec.as_mut()).iter().map(Frame::pts));
    }
    dec.flush().unwrap();
    pts.extend(drain(dec.as_mut()).iter().map(Frame::pts));
    let expect: Vec<Option<i64>> = (0..reference.len() as i64)
        .map(|n| Some(1000 + 40 * n))
        .collect();
    assert_eq!(pts, expect, "display order with the packets' times");
}

/// A stream entered mid-way (an open GOP's leading B-pictures have no
/// forward reference) and a damaged picture decode by default; the
/// `strict` option turns both into errors.
#[test]
fn damage_is_concealed_unless_strict() {
    let units = access_units(IBBP_64X48);
    let total = units.len();
    // Drop the P-picture the first two B-pictures predict from.
    let damaged: Vec<u8> = units
        .iter()
        .enumerate()
        .filter(|(n, _)| *n != 1)
        .flat_map(|(_, u)| u.iter().copied())
        .collect();

    let params = CodecParameters::video(CodecId::new(MPEG2_CODEC_ID_STR));
    let mut dec = make_decoder(&params).expect("make_decoder");
    dec.send_packet(&Packet::new(0, tb(), damaged.clone()))
        .expect("concealing decode");
    dec.flush().expect("flush");
    let frames = drain(dec.as_mut());
    assert!(!frames.is_empty() && frames.len() < total);

    let mut strict = params.clone();
    strict.options.insert("strict", "true");
    let mut dec = make_decoder(&strict).expect("make_decoder");
    let result = dec
        .send_packet(&Packet::new(0, tb(), damaged))
        .and_then(|()| dec.flush());
    assert!(result.is_err(), "strict mode reports the missing reference");

    // Options that are not this decoder's (a stream's parameter set
    // may carry its encoder's) are ignored; a malformed value is not.
    let mut foreign = params.clone();
    foreign.options.insert("picture_structure", "field");
    assert!(make_decoder(&foreign).is_ok());
    let mut malformed = params.clone();
    malformed.options.insert("strict", "perhaps");
    assert!(make_decoder(&malformed).is_err());
}
