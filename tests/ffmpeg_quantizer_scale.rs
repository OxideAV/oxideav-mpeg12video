//! Black-box validation of the macroblock-layer `quantizer_scale`
//! parser against a real MPEG-2 elementary stream produced by an opaque
//! encoder.
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
//! The fixture's first picture is an I-picture
//! (`picture_coding_type = 1`) whose first coded macroblock is plain
//! `Intra` (`macroblock_quant = 0`). Per ISO/IEC 11172-2:1993 §2.4.2.7
//! a macroblock-layer `quantizer_scale` is present **only** when
//! `macroblock_quant == 1`, so the fixture's first macroblock carries
//! none — the spec-correct absent-field behaviour the first test below
//! asserts (the parser consumes zero bits and the cursor stays on the
//! bit that begins `coded_block_pattern()` / the block loop).
//!
//! The second test splices a synthetic `macroblock_quant`-set
//! `macroblock_type` followed by a 5-bit `quantizer_scale` and confirms
//! the field is decoded with the right value and bit accounting.

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_mpeg12video::picture_header::PictureCodingType;
use oxideav_mpeg12video::slice_header::{SLICE_VERTICAL_POSITION_MAX, SLICE_VERTICAL_POSITION_MIN};
use oxideav_mpeg12video::{
    MacroblockType, MbAddressIncrement, MbAddressIncrementContext, QuantizerScale, SliceContext,
    SliceHeader,
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
fn first_i_picture_macroblock_has_no_quantizer_scale() {
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
    // First I-picture macroblock is plain Intra → macroblock_quant = 0.
    assert!(
        !mt.macroblock_quant,
        "plain Intra carries no quantizer_scale"
    );

    // With macroblock_quant clear the field is absent; the parser must
    // consume no bits, leaving the cursor exactly where macroblock_type
    // left it.
    let before = br.bit_position();
    let qs = QuantizerScale::parse_after_type(&mut br, &mt).expect("quantizer_scale");
    assert_eq!(qs.quantizer_scale, None);
    assert_eq!(qs.bit_position_after, before);
    assert_eq!(br.bit_position(), before, "no bits consumed when absent");
}

#[test]
fn spliced_quant_macroblock_decodes_quantizer_scale() {
    // Build a tiny macroblock prefix: an I-picture "Intra, Quant"
    // macroblock_type (Table B.2a code '01', macroblock_quant = 1)
    // followed by a 5-bit quantizer_scale of 11.
    let mut bw = BitWriter::new();
    bw.write_u32(0b01, 2); // macroblock_type: Intra, Quant
    bw.write_u32(11, 5); // quantizer_scale
    bw.write_bit(true); // padding the parser never reads
    bw.align_to_byte();
    let buf = bw.finish();

    let mut br = BitReader::new(&buf);
    let mt = MacroblockType::parse(&mut br, PictureCodingType::Intra).expect("macroblock_type");
    assert!(mt.macroblock_quant);

    let qs = QuantizerScale::parse_after_type(&mut br, &mt).expect("quantizer_scale");
    assert_eq!(qs.quantizer_scale, Some(11));
    // 2 bits of type + 5 bits of scale = 7.
    assert_eq!(qs.bit_position_after, 7);
}
