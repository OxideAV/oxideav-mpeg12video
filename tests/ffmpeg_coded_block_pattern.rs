//! Black-box validation of the `coded_block_pattern()` parser against
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
//! (`picture_coding_type = 1`) and the first coded macroblock is plain
//! `Intra` (`macroblock_pattern = 0`). Per §6.2.5.3 a
//! `coded_block_pattern()` is present **only** when
//! `macroblock_pattern == 1`, so the fixture's first macroblock has
//! none — the spec-correct behaviour the first test below asserts.
//!
//! The second test pins the fixture's chroma format (4:2:0, so no
//! `coded_block_pattern_1` / `coded_block_pattern_2` extension) and
//! confirms a Table B-9 codeword spliced onto the real stream's
//! chroma format decodes to the expected `cbp`.

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_mpeg12video::picture_header::PictureCodingType;
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::slice_header::{SLICE_VERTICAL_POSITION_MAX, SLICE_VERTICAL_POSITION_MIN};
use oxideav_mpeg12video::{
    CodedBlockPattern, MacroblockType, MbAddressIncrement, MbAddressIncrementContext,
    Mpeg2Sequence, SliceContext, SliceHeader,
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

fn find_sequence_header(haystack: &[u8]) -> Option<usize> {
    haystack
        .windows(4)
        .position(|w| w == [0x00, 0x00, 0x01, 0xB3])
}

#[test]
fn first_i_picture_macroblock_has_no_coded_block_pattern() {
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

    // The first I-picture macroblock is plain Intra: macroblock_pattern
    // is 0, so per §6.2.5.3 the bitstream carries NO coded_block_pattern()
    // for this macroblock. The pattern_code[] for such an intra block is
    // therefore all-ones (every block coded) by §6.3.17.4 defaulting.
    assert!(
        !mt.macroblock_pattern,
        "intra macroblock has no cbp present"
    );
    assert!(mt.macroblock_intra);
}

#[test]
fn fixture_chroma_is_420_and_table_b9_decodes_against_it() {
    // Confirm the fixture's sequence declares 4:2:0 chroma — so a real
    // coded_block_pattern() in this stream would be a bare Table B-9
    // VLC with no 4:2:2/4:4:4 fixed-length extension.
    let seq_pos = find_sequence_header(FIXTURE).expect("fixture has a sequence header");
    let seq = Mpeg2Sequence::from_buf(&FIXTURE[seq_pos..]).expect("parse sequence");
    assert_eq!(
        seq.extension.chroma_format,
        ChromaFormat::Yuv420,
        "ffmpeg's default mpeg2video output is 4:2:0"
    );

    // Splice a known Table B-9 codeword (cbp 60, the 3-bit code '111')
    // and decode it against the fixture's own chroma format. With 4:2:0
    // there is no trailing extension, so the parser must consume exactly
    // the 3 VLC bits.
    let mut bw = BitWriter::new();
    bw.write_u32(0b111, 3);
    bw.write_bit(true);
    bw.align_to_byte();
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let cbp = CodedBlockPattern::parse(&mut br, seq.extension.chroma_format).expect("parse cbp");
    assert_eq!(cbp.cbp, 60);
    assert_eq!(cbp.coded_block_pattern_1, None);
    assert_eq!(cbp.coded_block_pattern_2, None);
    assert_eq!(cbp.bit_position_after, 3);
}
