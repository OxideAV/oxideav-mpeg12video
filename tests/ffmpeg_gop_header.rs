//! Black-box validation that our `group_of_pictures_header()` parser
//! agrees with a real MPEG-2 elementary stream produced by an opaque
//! encoder. The fixture under `tests/fixtures/` was produced via:
//!
//! ```text
//! ffmpeg -y -f lavfi -i testsrc=size=352x240:rate=25:duration=0.04 \
//!        -c:v mpeg2video -b:v 800k -an -f mpeg2video out.m2v
//! ```
//!
//! Only the file's *bytes* are consumed here; the encoder's source
//! code is not. We hex-locate the `00 00 01 B8` `group_start_code` in
//! the fixture and parse from there; the four payload bytes recovered
//! from offset 0x1C..=0x1F of the fixture are `00 08 00 40`, which
//! per Table 6-11 decodes to the zero time-code with `closed_gop = 1`
//! and `broken_link = 0`.

use oxideav_mpeg12video::{Mpeg2Gop, GROUP_START_CODE};

const FIXTURE: &[u8] = include_bytes!("fixtures/ffmpeg-352x240-25fps.m2v");

fn find_start_code(haystack: &[u8], code: u32) -> Option<usize> {
    haystack.windows(4).position(|w| {
        (u32::from(w[0]) << 24 | u32::from(w[1]) << 16 | u32::from(w[2]) << 8 | u32::from(w[3]))
            == code
    })
}

#[test]
fn parses_ffmpeg_352x240_group_of_pictures_header() {
    let pos = find_start_code(FIXTURE, GROUP_START_CODE).expect("fixture has GOP start code");
    let gop = Mpeg2Gop::parse(&FIXTURE[pos..]).expect("parse GOP header");

    // The fixture is a single-GOP encode of the first 25fps frame, so
    // the time-code at the head of the GOP is the zero point and
    // ffmpeg leaves drop_frame_flag cleared (the frame rate is 25 Hz,
    // not 29.97 Hz, so drop-frame is illegal here anyway).
    assert!(!gop.time_code.drop_frame, "drop_frame_flag");
    assert_eq!(gop.time_code.hours, 0);
    assert_eq!(gop.time_code.minutes, 0);
    assert_eq!(gop.time_code.seconds, 0);
    assert_eq!(gop.time_code.pictures, 0);

    // ffmpeg's mpeg2video encoder marks every GOP as closed by
    // default (`-flags +cgop` is the default; the leading B-pictures
    // -- if any -- are encoded with backward prediction only).
    assert!(gop.closed_gop, "closed_gop");
    // The encoder never edits a stream after writing, so broken_link
    // must be zero per §6.3.8.
    assert!(!gop.broken_link, "broken_link");
}
