//! Black-box validation of the `macroblock_modes()` tail parser
//! (`frame_motion_type` / `field_motion_type` + `dct_type`) against a
//! real MPEG-2 elementary stream produced by an opaque encoder.
//!
//! The fixture under `tests/fixtures/` was produced via:
//!
//! ```text
//! ffmpeg -y -f lavfi -i testsrc=size=352x240:rate=25:duration=0.04 \
//!        -c:v mpeg2video -b:v 800k -an -f mpeg2video out.m2v
//! ```
//!
//! Only the file's *bytes* are consumed here; the encoder's source code
//! is not.
//!
//! The fixture's first picture is an I-picture in a frame structure
//! whose first coded macroblock is plain `Intra` — neither motion flag
//! is set. Per ISO/IEC 13818-2 §6.2.5.1 a motion-type code is present
//! only when `macroblock_motion_forward || macroblock_motion_backward`,
//! so this macroblock carries **none** — the spec-correct absent-field
//! behaviour the first test asserts. Whether the trailing `dct_type`
//! is present is gated on the picture's own `frame_pred_frame_dct`, so
//! the test reads that flag out of the fixture's
//! `picture_coding_extension()` and asserts the field's presence
//! accordingly rather than hard-coding the encoder's choice.

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_mpeg12video::macroblock_modes::MacroblockModesTail;
use oxideav_mpeg12video::picture_header::{
    Mpeg2PictureHeader, PictureCodingType, PictureStructure,
};
use oxideav_mpeg12video::slice_header::{SLICE_VERTICAL_POSITION_MAX, SLICE_VERTICAL_POSITION_MIN};
use oxideav_mpeg12video::{
    MacroblockModesContext, MacroblockType, MbAddressIncrement, MbAddressIncrementContext,
    QuantizerScale, SliceContext, SliceHeader, PICTURE_START_CODE,
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

fn find_start_code(haystack: &[u8], code: u32) -> Option<usize> {
    haystack.windows(4).position(|w| {
        (u32::from(w[0]) << 24 | u32::from(w[1]) << 16 | u32::from(w[2]) << 8 | u32::from(w[3]))
            == code
    })
}

#[test]
fn first_i_picture_macroblock_modes_tail_matches_spec_gates() {
    // Pull the picture-level gates straight out of the fixture's own
    // picture_coding_extension().
    let pic_pos = find_start_code(FIXTURE, PICTURE_START_CODE).expect("picture start code");
    let (_pic, ext) = Mpeg2PictureHeader::parse_with_extension(&FIXTURE[pic_pos..])
        .expect("picture_header + picture_coding_extension");
    assert_eq!(ext.picture_structure, PictureStructure::Frame);
    let ctx = MacroblockModesContext::new(ext.picture_structure, ext.frame_pred_frame_dct);

    // Walk into the first macroblock of the first slice.
    let pos = find_first_slice_start_code(FIXTURE).expect("fixture contains a slice start code");
    let slice = &FIXTURE[pos..];
    let sh = SliceHeader::parse(slice, SliceContext::non_scalable(240)).expect("slice header");
    let mut br = BitReader::new(slice);
    br.skip(sh.body_bit_position as u32).expect("skip header");

    let mi = MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2())
        .expect("mb_address_increment");
    assert_eq!(mi.value, 1);

    let mt =
        MacroblockType::parse(&mut br, PictureCodingType::Intra).expect("parse macroblock_type");
    assert!(mt.macroblock_intra);
    assert!(!mt.macroblock_motion_forward);
    assert!(!mt.macroblock_motion_backward);

    let qs = QuantizerScale::parse_after_type(&mut br, &mt).expect("quantizer_scale");
    assert_eq!(qs.quantizer_scale, None);

    let before = br.bit_position();
    let tail = MacroblockModesTail::parse(&mut br, &mt, &ctx).expect("macroblock_modes tail");

    // Plain Intra → no motion flags → motion-type code is always absent.
    assert_eq!(tail.motion_type, None);

    // dct_type is present iff (frame picture && frame_pred_frame_dct ==
    // 0 && (intra || pattern)). The macroblock is intra in a frame
    // picture, so presence reduces to !frame_pred_frame_dct.
    if ext.frame_pred_frame_dct {
        assert_eq!(tail.dct_type, None, "frame_pred_frame_dct=1 omits dct_type");
        assert_eq!(tail.bit_position_after, before, "no bits consumed");
    } else {
        assert!(
            tail.dct_type.is_some(),
            "frame_pred_frame_dct=0 carries dct_type for an intra macroblock"
        );
        assert_eq!(tail.bit_position_after, before + 1, "one dct_type bit");
    }
}

#[test]
fn spliced_inter_frame_macroblock_modes_tail_decodes_motion_and_dct() {
    // Build a P-picture frame macroblock prefix: a Table B-3 "MC,
    // Coded" macroblock_type (code '1', macroblock_motion_forward = 1,
    // macroblock_pattern = 1), then a frame_motion_type '10'
    // (Frame-based) and a dct_type '1'. With frame_pred_frame_dct = 0
    // in a frame picture both trailing fields are present.
    let mut bw = BitWriter::new();
    bw.write_u32(0b1, 1); // macroblock_type: MC, Coded
    bw.write_u32(0b10, 2); // frame_motion_type: Frame-based
    bw.write_u32(0b1, 1); // dct_type: field DCT coded
    bw.write_bit(true); // padding the parser never reads
    bw.align_to_byte();
    let buf = bw.finish();

    let mut br = BitReader::new(&buf);
    let mt =
        MacroblockType::parse(&mut br, PictureCodingType::Predictive).expect("macroblock_type");
    assert!(mt.macroblock_motion_forward);
    assert!(mt.macroblock_pattern);

    let ctx = MacroblockModesContext::new(PictureStructure::Frame, false);
    let tail = MacroblockModesTail::parse(&mut br, &mt, &ctx).expect("macroblock_modes tail");

    let motion = tail.motion_type.expect("motion type present");
    assert_eq!(motion.code, 0b10);
    assert_eq!(tail.dct_type, Some(true));
    // 1 bit type + 2 bits motion + 1 bit dct = 4.
    assert_eq!(tail.bit_position_after, 4);
}
