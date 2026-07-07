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
