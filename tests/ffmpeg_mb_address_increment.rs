//! Black-box validation that our `macroblock_address_increment`
//! parser agrees with a real MPEG-2 elementary stream produced by an
//! opaque encoder.
//!
//! The fixture under `tests/fixtures/` was produced via:
//!
//! ```text
//! ffmpeg -y -f lavfi -i testsrc=size=352x240:rate=25:duration=0.04 \
//!        -c:v mpeg2video -b:v 800k -an -f mpeg2video out.m2v
//! ```
//!
//! Only the file's *bytes* are consumed here; the encoder's source
//! code is not.
//!
//! After the first `slice_start_code` (`00 00 01 01`) the bitstream
//! enters its `macroblock()` loop. The very first macroblock in a
//! slice must encode `macroblock_address_increment = 1`
//! (ISO/IEC 13818-2 §6.3.17.1) because `previous_macroblock_address`
//! is initialised to `(slice_vertical_position - 1) * mb_width - 1`
//! and `macroblock_address` therefore must increment by exactly 1
//! to point at the first macroblock of the row. The Table B-1 VLC
//! for `value = 1` is the single bit `'1'`.

use oxideav_core::bits::BitReader;
use oxideav_mpeg12video::slice_header::{SLICE_VERTICAL_POSITION_MAX, SLICE_VERTICAL_POSITION_MIN};
use oxideav_mpeg12video::{
    MbAddressIncrement, MbAddressIncrementContext, SliceContext, SliceHeader,
};

const FIXTURE: &[u8] = include_bytes!("fixtures/ffmpeg-352x240-25fps.m2v");

fn find_first_slice_start_code(haystack: &[u8]) -> Option<usize> {
    haystack.windows(4).position(|w| {
        w[0] == 0x00
            && w[1] == 0x00
            && w[2] == 0x01
            && (SLICE_VERTICAL_POSITION_MIN..=SLICE_VERTICAL_POSITION_MAX).contains(&w[3])
    })
}

#[test]
fn first_macroblock_increment_in_ffmpeg_fixture_is_one() {
    let pos = find_first_slice_start_code(FIXTURE).expect("fixture contains a slice start code");
    let slice = &FIXTURE[pos..];

    // Parse the slice header first to discover where the macroblock
    // body begins (bit-aligned, not byte-aligned).
    let sh = SliceHeader::parse(slice, SliceContext::non_scalable(240)).expect("slice header");

    // Seek a BitReader to the body start.
    let mut br = BitReader::new(slice);
    br.skip(sh.body_bit_position as u32).expect("skip header");

    // Per §6.3.17.1 the first macroblock of a slice carries
    // increment = 1. The Table B-1 code for 1 is the single bit '1'.
    let mi = MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2())
        .expect("parse first mb_address_increment");
    assert_eq!(
        mi.value, 1,
        "first macroblock of slice should increment by 1"
    );
    assert_eq!(mi.escape_count, 0);
    assert_eq!(mi.stuffing_count, 0);
    // The first '1' bit lives right after `body_bit_position`, so
    // bit_position_after should be exactly body_bit_position + 1.
    assert_eq!(mi.bit_position_after, sh.body_bit_position + 1);
}

#[test]
fn ffmpeg_fixture_does_not_emit_macroblock_stuffing() {
    // MPEG-2 streams shall not contain the MPEG-1 stuffing code.
    // Confirm: parse with mpeg2 context and check stuffing_count
    // == 0 for the first macroblock_address_increment of the slice.
    let pos = find_first_slice_start_code(FIXTURE).expect("fixture contains a slice start code");
    let slice = &FIXTURE[pos..];
    let sh = SliceHeader::parse(slice, SliceContext::non_scalable(240)).expect("slice header");
    let mut br = BitReader::new(slice);
    br.skip(sh.body_bit_position as u32).expect("skip header");
    let mi = MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2())
        .expect("parse mb_address_increment");
    assert_eq!(mi.stuffing_count, 0);
}
