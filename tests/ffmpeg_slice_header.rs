//! Black-box validation that our `slice()` header parser agrees with
//! a real MPEG-2 elementary stream produced by an opaque encoder. The
//! fixture under `tests/fixtures/` was produced via:
//!
//! ```text
//! ffmpeg -y -f lavfi -i testsrc=size=352x240:rate=25:duration=0.04 \
//!        -c:v mpeg2video -b:v 800k -an -f mpeg2video out.m2v
//! ```
//!
//! Only the file's *bytes* are consumed here; the encoder's source
//! code is not. The first `slice_start_code` (`00 00 01 01`) sits
//! immediately after the picture-coding-extension at byte offset
//! `0x2E` of this 352x240 fixture and carries
//! `slice_vertical_position = 0x01` (first row of macroblocks, per
//! §6.3.16). The 5-bit `quantiser_scale_code` is the encoder's
//! choice; we don't hard-pin it, but we do require it to be in the
//! spec-mandated range `1..=31` (§6.3.16), to come *before* the
//! macroblock body, and for the slice to *not* carry the optional
//! intra_slice prelude (`nextbits() == '0'` is the dominant case for
//! ffmpeg-emitted streams).

use oxideav_mpeg12video::slice_header::{SLICE_VERTICAL_POSITION_MAX, SLICE_VERTICAL_POSITION_MIN};
use oxideav_mpeg12video::{SliceContext, SliceHeader};

const FIXTURE: &[u8] = include_bytes!("fixtures/ffmpeg-352x240-25fps.m2v");

/// Locate the first `slice_start_code` in the fixture: a 24-bit
/// prefix `00 00 01` followed by a byte in `0x01..=0xAF`.
fn find_first_slice_start_code(haystack: &[u8]) -> Option<usize> {
    haystack.windows(4).position(|w| {
        w[0] == 0x00
            && w[1] == 0x00
            && w[2] == 0x01
            && (SLICE_VERTICAL_POSITION_MIN..=SLICE_VERTICAL_POSITION_MAX).contains(&w[3])
    })
}

#[test]
fn parses_ffmpeg_352x240_first_slice_header() {
    let pos = find_first_slice_start_code(FIXTURE).expect("fixture contains a slice start code");
    // ffmpeg's mpeg2video encoder packs every row as its own slice;
    // the first slice's vertical position is therefore 1 (first row).
    assert_eq!(FIXTURE[pos + 3], 0x01);

    let sh = SliceHeader::parse(
        &FIXTURE[pos..],
        // 352x240 stream — vertical_size = 240, not > 2800, no
        // sequence_scalable_extension() → no priority_breakpoint.
        SliceContext::non_scalable(240),
    )
    .expect("parse slice header");

    assert_eq!(sh.slice_vertical_position, 0x01);
    assert!(sh.slice_vertical_position_extension.is_none());
    assert!(sh.priority_breakpoint.is_none());

    // §6.3.16 — quantiser_scale_code is 1..=31.
    assert!((1..=31).contains(&sh.quantiser_scale_code));

    // ffmpeg's mpeg2video encoder does not emit the optional
    // intra_slice prelude (the spec calls extra_information_slice
    // "Reserved" / "shall not be present in a conforming bitstream").
    assert!(sh.intra_slice_flag.is_none());
    assert!(sh.intra_slice.is_none());
    assert!(sh.extra_information_slice.is_empty());

    // mb_row = svp - 1 = 0 for the first slice.
    assert_eq!(sh.mb_row(), 0);

    // Without the prelude or scalable extras, the header is exactly
    // 24 + 8 + 5 + 1 = 38 bits long.
    assert_eq!(sh.body_bit_position, 38);
}

#[test]
fn fixture_carries_multiple_slice_start_codes() {
    // The 352x240 testsrc encode emits at least one slice per
    // 16-line row, so at least a handful of slice_start_codes
    // exist. Walk the buffer and count them.
    let mut cursor = 0;
    let mut count = 0;
    while let Some(rel) = find_first_slice_start_code(&FIXTURE[cursor..]) {
        count += 1;
        cursor += rel + 4;
    }
    // 240 / 16 = 15 rows of macroblocks. ffmpeg's mpeg2video encoder
    // emits one slice per row by default, so we expect at least 5
    // (cheap lower bound that still catches a regression where the
    // search loop breaks after the first hit).
    assert!(count >= 5, "expected >=5 slice start codes, got {count}");
}
