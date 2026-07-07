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
    CodecId, CodecParameters, Decoder, Error, Frame, Packet, RuntimeContext, TimeBase,
};
use oxideav_mpeg12video::{
    decode_video_sequence, make_decoder, FrameBuffer, Mpeg12Decoder, MPEG1_CODEC_ID_STR,
    MPEG2_CODEC_ID_STR,
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
