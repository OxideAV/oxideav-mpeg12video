//! Coverage-guided decode target: attacker-controlled bytes through
//! the whole-elementary-stream driver `decode_video_sequence`.
//!
//! Three feeds per iteration so the deep layers are reached often:
//!
//! 1. the raw attacker bytes (start-code scan, sequence-layer
//!    classification, truncation handling);
//! 2. a valid MPEG-2 sequence-header + extension + picture-header
//!    skeleton (borrowed from the crate's own intra encoder output)
//!    with the attacker bytes spliced in as slice/macroblock payload —
//!    the slice walk, VLC decoders, inverse quant, IDCT and motion
//!    compensation see structured garbage;
//! 3. the same splice over an ISO/IEC 11172-2 skeleton so the MPEG-1
//!    block/motion pipeline is exercised too.
//!
//! The contract is panic-freedom: every outcome (Ok or Err) is
//! acceptable, crashing is not.
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    FrameBuffer, IntraPictureParams, Mpeg1SequenceParams, SequenceDecoder, decode_video_sequence,
    encode_i_then_p, encode_intra_picture, encode_mpeg1_intra_stream,
};
use std::sync::OnceLock;

/// A valid stream prefix ending right where slice data begins (the
/// first slice start code is kept, the slice body is the fuzzer's).
fn prefixes() -> &'static (Vec<u8>, Vec<u8>) {
    static PREFIXES: OnceLock<(Vec<u8>, Vec<u8>)> = OnceLock::new();
    PREFIXES.get_or_init(|| {
        let mut frame = FrameBuffer::new(48, 32, ChromaFormat::Yuv420);
        for y in 0..32 {
            for x in 0..48 {
                frame.y.put_sample(x, y, ((x * 5 + y * 3) % 200) as u8);
            }
        }
        for y in 0..16 {
            for x in 0..24 {
                frame.cb.put_sample(x, y, 128);
                frame.cr.put_sample(x, y, 128);
            }
        }
        let params = IntraPictureParams::new(
            48,
            32,
            ChromaFormat::Yuv420,
            true,
            0,
            false,
            false,
            false,
            true,
        );
        let m2v = encode_intra_picture(&frame, params, 0, 6).expect("valid intra stream");
        let m1v = encode_mpeg1_intra_stream(
            &frame,
            &Mpeg1SequenceParams::default()
                .with_horizontal_size(48)
                .with_vertical_size(32),
            6,
        )
        .expect("valid mpeg1 stream");
        let cut = |s: &[u8]| -> Vec<u8> {
            // Keep everything through the first slice start code
            // (0x000001 [0x01..=0xAF]) plus one byte of header.
            for i in 0..s.len().saturating_sub(4) {
                if s[i] == 0 && s[i + 1] == 0 && s[i + 2] == 1 && (1..=0xAF).contains(&s[i + 3]) {
                    return s[..(i + 5).min(s.len())].to_vec();
                }
            }
            s.to_vec()
        };
        (cut(&m2v), cut(&m1v))
    })
}

fuzz_target!(|data: &[u8]| {
    // 1. Raw bytes.
    let _ = decode_video_sequence(data);

    // 2. MPEG-2 skeleton + attacker slice payload.
    let (m2v, m1v) = prefixes();
    let mut spliced = m2v.clone();
    spliced.extend_from_slice(data);
    spliced.extend_from_slice(&[0x00, 0x00, 0x01, 0xB7]); // sequence_end_code
    let _ = decode_video_sequence(&spliced);

    // 3. MPEG-1 skeleton + attacker slice payload.
    let mut spliced1 = m1v.clone();
    spliced1.extend_from_slice(data);
    spliced1.extend_from_slice(&[0x00, 0x00, 0x01, 0xB7]);
    let _ = decode_video_sequence(&spliced1);

    // 4. An intact I-picture, then a P-picture whose slices are the
    //    attacker's: decoded serially and under a thread budget (the
    //    slices of a P-picture are decoded in row bands), with and
    //    without concealment. The outcome must not depend on the
    //    budget.
    let mut spliced_p = inter_prefix().clone();
    spliced_p.extend_from_slice(data);
    spliced_p.extend_from_slice(&[0x00, 0x00, 0x01, 0xB7]);
    for conceal in [false, true] {
        let run = |threads: usize| {
            let mut decoder = if conceal {
                SequenceDecoder::with_concealment()
            } else {
                SequenceDecoder::new()
            }
            .with_threads(threads);
            decoder.push(&spliced_p);
            decoder.finish();
            let mut frames = Vec::new();
            loop {
                match decoder.next_frame() {
                    Ok(Some(frame)) => frames.push(Ok(frame.frame)),
                    Ok(None) => break,
                    Err(e) => {
                        frames.push(Err(format!("{e:?}")));
                        break;
                    }
                }
            }
            frames
        };
        assert_eq!(run(1), run(4), "thread budget changed the decode");
    }
});

/// A valid I-picture followed by a P-picture cut right after its first
/// slice start code (the slice bodies are the fuzzer's).
fn inter_prefix() -> &'static Vec<u8> {
    static PREFIX: OnceLock<Vec<u8>> = OnceLock::new();
    PREFIX.get_or_init(|| {
        let frame = |shift: usize| {
            let mut frame = FrameBuffer::new(48, 64, ChromaFormat::Yuv420);
            for y in 0..64 {
                for x in 0..48 {
                    frame
                        .y
                        .put_sample(x, y, (((x + shift) * 5 + y * 3) % 200) as u8);
                }
            }
            for y in 0..32 {
                for x in 0..24 {
                    frame.cb.put_sample(x, y, 128);
                    frame.cr.put_sample(x, y, 128);
                }
            }
            frame
        };
        let params = IntraPictureParams::new(
            48,
            64,
            ChromaFormat::Yuv420,
            true,
            0,
            false,
            false,
            false,
            true,
        );
        let stream = encode_i_then_p(&frame(0), &frame(2), params, 6, 2).expect("valid I P stream");
        // The P-picture's first slice start code is the first one
        // after the second picture_start_code.
        let second_picture = stream
            .windows(4)
            .enumerate()
            .filter(|(_, w)| *w == [0, 0, 1, 0])
            .map(|(i, _)| i)
            .nth(1)
            .expect("two pictures");
        let slice = stream[second_picture + 4..]
            .windows(4)
            .position(|w| w[0] == 0 && w[1] == 0 && w[2] == 1 && (1..=0xAF).contains(&w[3]))
            .expect("a slice")
            + second_picture
            + 4;
        stream[..slice + 5].to_vec()
    })
}
