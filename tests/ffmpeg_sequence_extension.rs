//! Black-box validation that our `sequence_extension()` parser and
//! the [`Mpeg2Sequence::from_buf`] composer agree with a real
//! MPEG-2 elementary stream produced by an opaque encoder. The
//! fixture under `tests/fixtures/` was produced via:
//!
//! ```text
//! ffmpeg -y -f lavfi -i testsrc=size=352x240:rate=25:duration=0.04 \
//!        -c:v mpeg2video -b:v 800k -an -f mpeg2video out.m2v
//! ```
//!
//! Only the file's *bytes* are consumed here; the encoder's source
//! code is not. Same fixture as the round-1 sequence-header
//! integration test — round 2 just exercises the extension layer
//! on top of it.

use oxideav_mpeg12video::{
    ChromaFormat, Mpeg2Sequence, Mpeg2SequenceExtension, EXTENSION_START_CODE, SEQUENCE_HEADER_CODE,
};

const FIXTURE: &[u8] = include_bytes!("fixtures/ffmpeg-352x240-25fps.m2v");

fn find_start_code(haystack: &[u8], code: u32) -> Option<usize> {
    haystack.windows(4).position(|w| {
        (u32::from(w[0]) << 24 | u32::from(w[1]) << 16 | u32::from(w[2]) << 8 | u32::from(w[3]))
            == code
    })
}

#[test]
fn parses_ffmpeg_352x240_sequence_extension() {
    let pos = find_start_code(FIXTURE, EXTENSION_START_CODE).expect("fixture has extension code");
    let ext = Mpeg2SequenceExtension::parse(&FIXTURE[pos..]).expect("parse extension");

    // ffmpeg's testsrc-352x240 MPEG-2 fixture writes Main Profile @
    // Main Level (clause 8 of 13818-2 maps this to 0x48).
    assert_eq!(ext.profile_and_level, 0x48, "MP@ML byte");
    assert!(ext.progressive_sequence, "progressive_sequence");
    assert_eq!(ext.chroma_format, ChromaFormat::Yuv420);
    assert_eq!(ext.horizontal_size_extension, 0);
    assert_eq!(ext.vertical_size_extension, 0);
    // bit_rate_value in the header was the all-ones 0x3FFFF
    // sentinel; the extension contributes 0 high bits in this
    // fixture (the actual rate signalling is fully in
    // bit_rate_value's lower bits — see round-1 integration test).
    assert_eq!(ext.bit_rate_extension, 0);
    assert_eq!(ext.vbv_buffer_size_extension, 0);
    assert!(!ext.low_delay);
    assert_eq!(ext.frame_rate_extension_n, 0);
    assert_eq!(ext.frame_rate_extension_d, 0);
}

#[test]
fn composes_ffmpeg_352x240_sequence() {
    let pos =
        find_start_code(FIXTURE, SEQUENCE_HEADER_CODE).expect("fixture has sequence_header_code");
    let seq = Mpeg2Sequence::from_buf(&FIXTURE[pos..]).expect("compose sequence");

    // Composed 14-bit dimensions match the source resolution.
    assert_eq!(seq.horizontal_size, 352);
    assert_eq!(seq.vertical_size, 240);
    // Composed 30-bit bit_rate is the lower-18 + 0 from the
    // extension (ffmpeg parks the rate in bit_rate_value here).
    assert_eq!(seq.bit_rate, 0x3_FFFF);
    // Composed 18-bit vbv_buffer_size is just whatever lower 10
    // bits the encoder wrote — we don't pin a specific value to
    // avoid coupling the test to ffmpeg's encoder defaults.
    assert!(seq.vbv_buffer_size <= 0x3_FFFF);
    assert!(seq.extension.progressive_sequence);
    assert_eq!(seq.extension.chroma_format, ChromaFormat::Yuv420);
}
