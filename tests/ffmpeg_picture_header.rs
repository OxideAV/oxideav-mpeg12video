//! Black-box validation that our `picture_header()` parser agrees
//! with a real MPEG-2 elementary stream produced by an opaque
//! encoder. The fixture under `tests/fixtures/` was produced via:
//!
//! ```text
//! ffmpeg -y -f lavfi -i testsrc=size=352x240:rate=25:duration=0.04 \
//!        -c:v mpeg2video -b:v 800k -an -f mpeg2video out.m2v
//! ```
//!
//! Only the file's *bytes* are consumed here; the encoder's source
//! code is not. We hex-locate the `00 00 01 00` `picture_start_code`
//! in the fixture and parse from there. Manual inspection at the
//! ffmpeg-emitted offset (0x1e) yields the four payload bytes
//! `00 0f ff f8` which per §6.2.3 decode to `temporal_reference =
//! 0`, `picture_coding_type = 001 (I-picture)`, `vbv_delay = 0xFFFF`
//! (VBR sentinel), and a zero terminating `extra_bit_picture`.

use oxideav_mpeg12video::{
    Mpeg2PictureHeader, PictureCodingType, PictureStructure, PICTURE_START_CODE,
};

const FIXTURE: &[u8] = include_bytes!("fixtures/ffmpeg-352x240-25fps.m2v");

fn find_start_code(haystack: &[u8], code: u32) -> Option<usize> {
    haystack.windows(4).position(|w| {
        (u32::from(w[0]) << 24 | u32::from(w[1]) << 16 | u32::from(w[2]) << 8 | u32::from(w[3]))
            == code
    })
}

#[test]
fn parses_ffmpeg_352x240_picture_header() {
    let pos = find_start_code(FIXTURE, PICTURE_START_CODE).expect("fixture has picture start code");
    let pic = Mpeg2PictureHeader::parse(&FIXTURE[pos..]).expect("parse picture header");

    // First coded picture after the GOP header is the I-frame whose
    // temporal_reference is reset to zero (§6.3.10, §6.3.8).
    assert_eq!(pic.temporal_reference, 0);
    assert_eq!(pic.picture_coding_type, PictureCodingType::Intra);

    // The fixture is a single-frame encode; ffmpeg writes the VBR
    // sentinel 0xFFFF for vbv_delay rather than a real fill time.
    assert_eq!(pic.vbv_delay, 0xFFFF);

    // I-pictures have no forward/backward motion-vector hints.
    assert!(pic.full_pel_forward_vector.is_none());
    assert!(pic.fwd_f_code.is_none());
    assert!(pic.full_pel_backward_vector.is_none());
    assert!(pic.bwd_f_code.is_none());

    // A conforming MPEG-2 stream "shall not contain" extra picture
    // information bytes (§6.3.10).
    assert!(pic.extra_information_picture.is_empty());
}

#[test]
fn parses_ffmpeg_352x240_picture_header_with_extension() {
    let pos = find_start_code(FIXTURE, PICTURE_START_CODE).expect("fixture has picture start code");
    let (pic, ext) = Mpeg2PictureHeader::parse_with_extension(&FIXTURE[pos..])
        .expect("compose picture_header + picture_coding_extension");

    assert_eq!(pic.picture_coding_type, PictureCodingType::Intra);

    // For an I-picture in a non-scalable progressive-frame encode,
    // all four f_code[s][t] sub-fields shall be 15 (unused) per
    // §6.3.11.
    assert_eq!(ext.f_code_fwd_horiz, 15);
    assert_eq!(ext.f_code_fwd_vert, 15);
    assert_eq!(ext.f_code_bwd_horiz, 15);
    assert_eq!(ext.f_code_bwd_vert, 15);

    // ffmpeg's mpeg2video encoder defaults to frame pictures (no
    // field coding) on a progressive testsrc input.
    assert_eq!(ext.picture_structure, PictureStructure::Frame);
}
