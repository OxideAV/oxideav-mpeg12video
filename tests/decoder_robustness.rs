//! Robustness of the runtime MPEG-1 / MPEG-2 video [`Decoder`] against
//! malformed input: truncated and corrupted elementary streams must
//! produce a clean error (or empty output), never a panic. A decoder is
//! fed attacker-controlled container payloads, so a bounds-check slip or
//! an arithmetic overflow anywhere in the parse / reconstruct path is a
//! denial-of-service bug, not a cosmetic one.
//!
//! Clean-room: the only real input is the opaque black-box fixture (its
//! encoder source is not read); every other input is a mechanical
//! mutation of it or a deterministic pseudo-random byte stream.

use oxideav_core::{CodecId, CodecParameters, Packet, TimeBase};
use oxideav_mpeg12video::{make_decoder, MPEG1_CODEC_ID_STR, MPEG2_CODEC_ID_STR};

const FIXTURE: &[u8] = include_bytes!("fixtures/ffmpeg-352x240-25fps.m2v");

fn tb() -> TimeBase {
    TimeBase::new(1, 25)
}

/// Feed `data` through the whole `Decoder` lifecycle. Never asserts on
/// the result — the contract under test is simply "does not panic".
fn drive(codec_id: &str, data: &[u8]) {
    let params = CodecParameters::video(CodecId::new(codec_id));
    let mut dec = make_decoder(&params).expect("make_decoder");
    let _ = dec.send_packet(&Packet::new(0, tb(), data.to_vec()));
    let _ = dec.flush();
    // Bounded drain — a well-behaved decoder terminates in NeedMore/Eof.
    for _ in 0..64 {
        if dec.receive_frame().is_err() {
            break;
        }
    }
}

#[test]
fn no_panic_on_truncated_streams() {
    // Every byte-level truncation through the header region (cheap: the
    // parse fails before any reconstruction), then a coarse sweep over
    // the whole stream (each surviving prefix may reconstruct a picture).
    for n in 0..128.min(FIXTURE.len()) {
        drive(MPEG2_CODEC_ID_STR, &FIXTURE[..n]);
    }
    let mut n = 0;
    while n <= FIXTURE.len() {
        drive(MPEG2_CODEC_ID_STR, &FIXTURE[..n]);
        n += 101;
    }
}

#[test]
fn no_panic_on_bit_flips() {
    // Flip one byte at a stride of positions across the stream; a single
    // corrupt VLC / length / start code must not derail into a panic.
    for pos in (0..FIXTURE.len()).step_by(53) {
        let mut buf = FIXTURE.to_vec();
        buf[pos] ^= 0xFF;
        drive(MPEG2_CODEC_ID_STR, &buf);
    }
}

#[test]
fn no_panic_on_garbage_and_empty() {
    drive(MPEG2_CODEC_ID_STR, &[]);
    drive(MPEG1_CODEC_ID_STR, &[]);
    drive(MPEG2_CODEC_ID_STR, &[0x00, 0x00, 0x01, 0xB3]);

    // Deterministic pseudo-random bytes salted with valid-looking
    // start-code prefixes, both alone and behind a real sequence header.
    let mut v = Vec::new();
    let mut x: u32 = 0x1234_5678;
    for i in 0..8000u32 {
        x = x.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        if i % 37 == 0 {
            v.extend_from_slice(&[0x00, 0x00, 0x01, (x >> 8) as u8]);
        }
        v.push((x >> 16) as u8);
    }
    drive(MPEG2_CODEC_ID_STR, &v);
    let mut salted = FIXTURE[..20].to_vec();
    salted.extend_from_slice(&v);
    drive(MPEG2_CODEC_ID_STR, &salted);
}

/// Self-encoded ISO/IEC 11172-2 streams (this crate's own MPEG-1
/// encoder output — see `tests/fixtures/selfenc/`): the §2.4 decode
/// path (GOP layer, MPEG-1 picture/slice/block drivers) must survive
/// truncation and corruption of a stream it normally accepts.
const MPEG1_SELFENC: &[u8] = include_bytes!("fixtures/selfenc/selfenc-mpeg1-ibbp2gop-64x48.m1v");

#[test]
fn no_panic_on_truncated_mpeg1_streams() {
    for n in 0..160.min(MPEG1_SELFENC.len()) {
        drive(MPEG1_CODEC_ID_STR, &MPEG1_SELFENC[..n]);
    }
    let mut n = 0;
    while n <= MPEG1_SELFENC.len() {
        drive(MPEG1_CODEC_ID_STR, &MPEG1_SELFENC[..n]);
        n += 61;
    }
}

#[test]
fn no_panic_on_mpeg1_bit_flips() {
    for pos in (0..MPEG1_SELFENC.len()).step_by(31) {
        let mut buf = MPEG1_SELFENC.to_vec();
        buf[pos] ^= 0xFF;
        drive(MPEG1_CODEC_ID_STR, &buf);
    }
    // Byte-precise flips through the sequence + GOP + first picture
    // header region, where a flip lands in length/flag fields.
    for pos in 0..64.min(MPEG1_SELFENC.len()) {
        let mut buf = MPEG1_SELFENC.to_vec();
        buf[pos] ^= 0x55;
        drive(MPEG1_CODEC_ID_STR, &buf);
    }
}

/// Round-453 fuzz finding: a 282-byte input whose pictures carry no
/// slices used to mint one full-size (4019×2549) frame per
/// `picture_start_code` — gigabytes of output from a few header bytes.
/// §6.1.2.2 (restricted slice structure, Table 8-5) requires every
/// macroblock to be enclosed in a slice, so such pictures are rejected.
#[test]
fn zero_slice_pictures_are_rejected_not_allocated() {
    let bytes = std::fs::read(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/hostile/zero-slice-pictures-282b.bin"
    ))
    .expect("hostile fixture");
    let result = oxideav_mpeg12video::decode_video_sequence(&bytes);
    assert!(result.is_err(), "slice-less pictures must be rejected");
}

/// A conformant picture with its last slice removed leaves the bottom
/// macroblock row uncovered — rejected per §6.1.2.2 (restricted slice
/// structure), while the intact stream decodes.
#[test]
fn partial_slice_coverage_is_rejected() {
    let stream = std::fs::read(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/selfenc/selfenc-intra-64x48.m2v"
    ))
    .expect("fixture");
    assert!(oxideav_mpeg12video::decode_video_sequence(&stream).is_ok());
    // Locate the last slice start code (0x000001 0x01..=0xAF) and cut
    // the stream there, re-appending the sequence_end_code.
    let last_slice = stream
        .windows(4)
        .enumerate()
        .filter(|(_, w)| w[0] == 0 && w[1] == 0 && w[2] == 1 && (1..=0xAF).contains(&w[3]))
        .map(|(i, _)| i)
        .next_back()
        .expect("slice present");
    let mut cut = stream[..last_slice].to_vec();
    cut.extend_from_slice(&[0, 0, 1, 0xB7]);
    assert!(
        oxideav_mpeg12video::decode_video_sequence(&cut).is_err(),
        "a picture missing a slice row must be rejected"
    );
}
