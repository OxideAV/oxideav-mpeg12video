//! Black-box validation that our `macroblock_type` parser agrees with
//! a real MPEG-2 elementary stream produced by an opaque encoder.
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
//! The fixture's first picture is an I-picture
//! (`picture_coding_type = 1`). After the first slice header and the
//! first `macroblock_address_increment` (`value = 1`), the next field
//! is `macroblock_type` from Annex B Table B-2. The first coded
//! macroblock of an intra picture is therefore `Intra`
//! (`macroblock_intra = 1`, every other flag `0`) — the single-bit
//! Table B-2 code `'1'`.

use oxideav_core::bits::BitReader;
use oxideav_mpeg12video::picture_header::PictureCodingType;
use oxideav_mpeg12video::slice_header::{SLICE_VERTICAL_POSITION_MAX, SLICE_VERTICAL_POSITION_MIN};
use oxideav_mpeg12video::{
    MacroblockType, MbAddressIncrement, MbAddressIncrementContext, SliceContext, SliceHeader,
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
fn first_macroblock_type_in_ffmpeg_i_picture_is_intra() {
    let pos = find_first_slice_start_code(FIXTURE).expect("fixture contains a slice start code");
    let slice = &FIXTURE[pos..];

    // Parse the slice header to find where the macroblock body begins.
    let sh = SliceHeader::parse(slice, SliceContext::non_scalable(240)).expect("slice header");

    let mut br = BitReader::new(slice);
    br.skip(sh.body_bit_position as u32).expect("skip header");

    // First macroblock_address_increment of the slice is 1.
    let mi = MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2())
        .expect("parse mb_address_increment");
    assert_eq!(mi.value, 1);

    // The picture is an I-picture, so macroblock_type uses Table B-2.
    let mt =
        MacroblockType::parse(&mut br, PictureCodingType::Intra).expect("parse macroblock_type");

    // The first coded macroblock of an intra slice is plain "Intra".
    assert!(mt.macroblock_intra, "first I-picture macroblock is intra");
    assert!(!mt.macroblock_quant);
    assert!(!mt.macroblock_motion_forward);
    assert!(!mt.macroblock_motion_backward);
    assert!(!mt.macroblock_pattern);
    assert!(!mt.spatial_temporal_weight_code_flag);

    // The Table B-2 "Intra" code is the single bit '1', so the
    // cursor advances by exactly one bit past the increment.
    assert_eq!(mt.bit_position_after, mi.bit_position_after + 1);
}

#[test]
fn macroblock_type_advances_cursor_one_bit_in_i_picture() {
    // Sanity check that the parsed macroblock_type for the I-picture
    // consumes exactly one bit (the single-bit Table B-2 "Intra"
    // code), confirming we did not over- or under-read the VLC.
    let pos = find_first_slice_start_code(FIXTURE).expect("slice start code");
    let slice = &FIXTURE[pos..];
    let sh = SliceHeader::parse(slice, SliceContext::non_scalable(240)).expect("slice header");
    let mut br = BitReader::new(slice);
    br.skip(sh.body_bit_position as u32).expect("skip header");
    let mi = MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2())
        .expect("mb_address_increment");
    let before = mi.bit_position_after;
    let mt = MacroblockType::parse(&mut br, PictureCodingType::Intra).expect("macroblock_type");
    assert_eq!(mt.bit_position_after - before, 1);
}
