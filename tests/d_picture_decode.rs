//! ISO/IEC 11172-2 **D-picture** (dc intra-coded,
//! `picture_coding_type == 4`, §2.4.3.4) decode: hand-built streams
//! exercising the §2.4.2.7 macroblock layer (Table B.2d
//! `macroblock_type`, `end_of_macroblock`) and the §2.4.2.8 DC-only
//! block layer, both through the per-picture driver
//! (`decode_mpeg1_d_picture`) and the whole-stream
//! `decode_video_sequence` path.
//!
//! Every bit written here is transcribed from the ISO/IEC 11172-2:1993
//! syntax tables: §2.4.2.5 (picture layer), §2.4.2.6 (slice layer),
//! §2.4.2.7 (macroblock layer), §2.4.2.8 (block layer), Annex B
//! Tables B.1 (macroblock_address_increment), B.2d (macroblock_type in
//! D-pictures), B.5a / B.5b (dct_dc_size).

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::mpeg1_picture::{decode_mpeg1_d_picture, Mpeg1PictureParams};
use oxideav_mpeg12video::picture_header::PictureCodingType;
use oxideav_mpeg12video::{decode_video_sequence, DEFAULT_INTRA_QUANT};

/// Write an ISO/IEC 11172-2 §2.4.2.3 sequence header (no extension
/// follows — that absence is what classifies the stream as MPEG-1).
fn write_mpeg1_sequence_header(bw: &mut BitWriter, width: u16, height: u16) {
    bw.write_u32(0x0000_01B3, 32); // sequence_header_code
    bw.write_u32(u32::from(width), 12); // horizontal_size
    bw.write_u32(u32::from(height), 12); // vertical_size
    bw.write_u32(0b0001, 4); // pel_aspect_ratio = 1.0 (Table 2-5)
    bw.write_u32(0b0011, 4); // picture_rate = 25 Hz (Table 2-6)
    bw.write_u32(0x3FFFF, 18); // bit_rate = variable (all ones)
    bw.write_bit(true); // marker_bit
    bw.write_u32(16, 10); // vbv_buffer_size
    bw.write_bit(false); // constrained_parameters_flag
    bw.write_bit(false); // load_intra_quantizer_matrix
    bw.write_bit(false); // load_non_intra_quantizer_matrix
    bw.align_to_byte();
}

/// Write a §2.4.2.5 picture header for a D-picture: no f_codes (the
/// `if (picture_coding_type == 2 || == 3)` gates skip type 4).
fn write_d_picture_header(bw: &mut BitWriter, temporal_reference: u16) {
    bw.write_u32(0x0000_0100, 32); // picture_start_code
    bw.write_u32(u32::from(temporal_reference), 10);
    bw.write_u32(0b100, 3); // picture_coding_type = 4 (dc intra-coded)
    bw.write_u32(0xFFFF, 16); // vbv_delay
    bw.write_bit(false); // extra_bit_picture = '0'
    bw.align_to_byte();
}

/// Write the §2.4.2.6 slice prelude for `mb_row` (0-based).
fn write_d_slice_header(bw: &mut BitWriter, mb_row: u32, quantizer_scale: u8) {
    bw.write_u32(0x0000_0101 + mb_row, 32); // slice_start_code
    bw.write_u32(u32::from(quantizer_scale), 5);
    bw.write_bit(false); // extra_bit_slice = '0'
}

/// Write the §2.4.3.7 DC prelude for one luminance block:
/// Table B.5a `dct_dc_size_luminance` + `dct_dc_differential`.
/// Only the sizes this test needs are transcribed.
fn write_luma_dc(bw: &mut BitWriter, differential: i32) {
    match differential {
        0 => bw.write_u32(0b100, 3), // size 0, no differential bits
        7 => {
            bw.write_u32(0b101, 3); // size 3
            bw.write_u32(0b111, 3); // positive: bits = value
        }
        -7 => {
            bw.write_u32(0b101, 3); // size 3
                                    // §2.4.3.7 negative rule: dct_zz[0] = ((-1) << size) |
                                    // (differential + 1) → bits '000' decode to -7.
            bw.write_u32(0b000, 3);
        }
        other => panic!("unsupported test differential {other}"),
    }
}

/// Write the §2.4.3.7 DC prelude for one chrominance block:
/// Table B.5b `dct_dc_size_chrominance` + `dct_dc_differential`.
fn write_chroma_dc(bw: &mut BitWriter, differential: i32) {
    match differential {
        0 => bw.write_u32(0b00, 2), // size 0
        3 => {
            bw.write_u32(0b10, 2); // size 2
            bw.write_u32(0b11, 2); // positive: bits = value
        }
        other => panic!("unsupported test differential {other}"),
    }
}

/// Write one §2.4.2.7 D-picture macroblock: increment '1' (Table B.1),
/// macroblock_type '1' (Table B.2d), six DC-only blocks, and the
/// `end_of_macroblock` '1' bit.
fn write_d_macroblock(bw: &mut BitWriter, y0_diff: i32, cb_diff: i32, cr_diff: i32) {
    bw.write_bit(true); // macroblock_address_increment = 1
    bw.write_bit(true); // macroblock_type = '1' (Table B.2d)
    write_luma_dc(bw, y0_diff);
    write_luma_dc(bw, 0); // Y1 inherits
    write_luma_dc(bw, 0); // Y2
    write_luma_dc(bw, 0); // Y3
    write_chroma_dc(bw, cb_diff);
    write_chroma_dc(bw, cr_diff);
    bw.write_bit(true); // end_of_macroblock = '1'
}

/// A whole 32×16 D-picture (2 macroblocks) as one slice, ending
/// byte-aligned with zero padding (the §5.2.3 zero-stuffed stop
/// pattern the slice walker recognises).
fn write_d_picture(bw: &mut BitWriter, temporal_reference: u16, y0_diff: i32) {
    write_d_picture_header(bw, temporal_reference);
    write_d_slice_header(bw, 0, 8);
    write_d_macroblock(bw, y0_diff, 3, 0);
    write_d_macroblock(bw, 0, 0, 0); // inherits both predictors
    bw.align_to_byte();
}

/// Hand-computed §2.4.4.1 expectations for `write_d_picture`:
///
/// * Y0 differential +7 → `dct_recon[0][0] = 1024 + 7·8 = 1080` →
///   DC-only IDCT flat `round(1080/8) = 135`; the other three
///   luminance blocks inherit via `dct_dc_y_past`.
/// * Cb differential +3 → `1024 + 24 = 1048` → flat `131`.
/// * Cr differential 0 → `1024` → flat `128`.
/// * Y0 differential −7 → `1024 − 56 = 968` → flat `121`.
const FLAT_LUMA_PLUS7: u8 = 135;
const FLAT_CB_PLUS3: u8 = 131;
const FLAT_CR_ZERO: u8 = 128;
const FLAT_LUMA_MINUS7: u8 = 121;

#[test]
fn d_picture_driver_reconstructs_flat_planes() {
    let mut bw = BitWriter::new();
    write_d_picture(&mut bw, 0, 7);
    let picture = bw.into_bytes();

    let params = Mpeg1PictureParams {
        width: 32,
        height: 16,
        intra_quant: DEFAULT_INTRA_QUANT,
        non_intra_quant: [[16u8; 8]; 8],
    };
    let (frame, placed) = decode_mpeg1_d_picture(&picture, &params).expect("D-picture decodes");
    assert_eq!(placed, 2, "two macroblocks decoded");

    // Both macroblocks carry the same predictor-inherited DC values.
    for y in 0..16 {
        for x in 0..32 {
            assert_eq!(frame.y.get(x, y), Some(FLAT_LUMA_PLUS7), "Y at ({x},{y})");
        }
    }
    for y in 0..8 {
        for x in 0..16 {
            assert_eq!(frame.cb.get(x, y), Some(FLAT_CB_PLUS3), "Cb at ({x},{y})");
            assert_eq!(frame.cr.get(x, y), Some(FLAT_CR_ZERO), "Cr at ({x},{y})");
        }
    }
}

#[test]
fn whole_stream_d_sequence_decodes_in_coded_order() {
    // sequence_header + two D-pictures + sequence_end_code — a legal
    // 11172-2 D-only sequence (§2.4.1: D-pictures shall not be in a
    // sequence containing any other picture types).
    let mut bw = BitWriter::new();
    write_mpeg1_sequence_header(&mut bw, 32, 16);
    write_d_picture(&mut bw, 0, 7);
    write_d_picture(&mut bw, 1, -7);
    bw.write_u32(0x0000_01B7, 32); // sequence_end_code
    let stream = bw.into_bytes();

    let frames = decode_video_sequence(&stream).expect("D-only sequence decodes");
    assert_eq!(frames.len(), 2);
    assert_eq!(frames[0].temporal_reference, 0);
    assert_eq!(frames[1].temporal_reference, 1);
    for f in &frames {
        assert_eq!(f.picture_coding_type, PictureCodingType::DcIntra);
        assert_eq!((f.frame.width, f.frame.height), (32, 16));
    }
    assert_eq!(frames[0].frame.y.get(0, 0), Some(FLAT_LUMA_PLUS7));
    assert_eq!(frames[1].frame.y.get(0, 0), Some(FLAT_LUMA_MINUS7));
    assert_eq!(frames[1].frame.cb.get(0, 0), Some(FLAT_CB_PLUS3));
}

/// The pinned fixture `mpeg1-dpics-48x32.m1v` (see the conformance
/// README for generation notes + SHA-256): four D-pictures, 3×2
/// macroblocks, per-macroblock DC staircases. The oracle is the
/// generator's arithmetic: `luma = 40 + 32·mb + 9·pic`,
/// `cb = 96 + 12·mb + 5·pic`, `cr = 160 − 10·mb − 3·pic` — every
/// sample of each macroblock is exactly that flat value (a DC-only
/// block IDCTs to `dct_recon[0][0] / 8` with zero rounding error).
#[test]
fn pinned_d_picture_fixture_decodes_exactly() {
    let stream = std::fs::read(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/conformance/mpeg1-dpics-48x32.m1v"
    ))
    .expect("fixture present");

    let frames = decode_video_sequence(&stream).expect("D fixture decodes");
    assert_eq!(frames.len(), 4);
    for (pic, f) in frames.iter().enumerate() {
        assert_eq!(f.picture_coding_type, PictureCodingType::DcIntra);
        assert_eq!((f.frame.width, f.frame.height), (48, 32));
        assert_eq!(f.temporal_reference, pic as u16);
        for mb_row in 0..2usize {
            for mb_col in 0..3usize {
                let mb = mb_row * 3 + mb_col;
                let luma = (40 + 32 * mb + 9 * pic).min(235) as u8;
                let cb = (96 + 12 * mb + 5 * pic).min(240) as u8;
                let cr = (160 - 10 * mb as i32 - 3 * pic as i32).max(16) as u8;
                for y in 0..16 {
                    for x in 0..16 {
                        assert_eq!(
                            f.frame.y.get(mb_col * 16 + x, mb_row * 16 + y),
                            Some(luma),
                            "pic {pic} mb {mb} Y at ({x},{y})"
                        );
                    }
                }
                for y in 0..8 {
                    for x in 0..8 {
                        assert_eq!(
                            f.frame.cb.get(mb_col * 8 + x, mb_row * 8 + y),
                            Some(cb),
                            "pic {pic} mb {mb} Cb at ({x},{y})"
                        );
                        assert_eq!(
                            f.frame.cr.get(mb_col * 8 + x, mb_row * 8 + y),
                            Some(cr),
                            "pic {pic} mb {mb} Cr at ({x},{y})"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn d_macroblock_type_zero_bit_is_rejected() {
    let mut bw = BitWriter::new();
    write_d_picture_header(&mut bw, 0);
    write_d_slice_header(&mut bw, 0, 8);
    bw.write_bit(true); // macroblock_address_increment = 1
    bw.write_bit(false); // not a Table B.2d codeword
    bw.write_u32(0xFFFF, 16); // keep the stop-pattern peek non-zero
    bw.align_to_byte();
    let picture = bw.into_bytes();

    let params = Mpeg1PictureParams {
        width: 16,
        height: 16,
        intra_quant: DEFAULT_INTRA_QUANT,
        non_intra_quant: [[16u8; 8]; 8],
    };
    let err = decode_mpeg1_d_picture(&picture, &params).unwrap_err();
    assert!(err.to_string().contains("B.2d"), "unexpected error: {err}");
}

#[test]
fn d_end_of_macroblock_zero_is_rejected() {
    let mut bw = BitWriter::new();
    write_d_picture_header(&mut bw, 0);
    write_d_slice_header(&mut bw, 0, 8);
    bw.write_bit(true); // increment
    bw.write_bit(true); // macroblock_type
    write_luma_dc(&mut bw, 7);
    for _ in 0..3 {
        write_luma_dc(&mut bw, 0);
    }
    write_chroma_dc(&mut bw, 0);
    write_chroma_dc(&mut bw, 0);
    bw.write_bit(false); // end_of_macroblock = '0' — illegal
    bw.write_u32(0xFFFF, 16); // keep the stop-pattern peek non-zero
    bw.align_to_byte();
    let picture = bw.into_bytes();

    let params = Mpeg1PictureParams {
        width: 16,
        height: 16,
        intra_quant: DEFAULT_INTRA_QUANT,
        non_intra_quant: [[16u8; 8]; 8],
    };
    let err = decode_mpeg1_d_picture(&picture, &params).unwrap_err();
    assert!(
        err.to_string().contains("end_of_macroblock"),
        "unexpected error: {err}"
    );
}

#[test]
fn skipped_macroblock_in_d_picture_is_rejected() {
    let mut bw = BitWriter::new();
    write_d_picture_header(&mut bw, 0);
    write_d_slice_header(&mut bw, 0, 8);
    // First macroblock is fine.
    write_d_macroblock(&mut bw, 7, 0, 0);
    // Second macroblock skips one slot: increment 2 = '011' (Table B.1).
    bw.write_u32(0b011, 3);
    bw.write_bit(true); // macroblock_type
    bw.write_u32(0xFFFF, 16); // keep the stop-pattern peek non-zero
    bw.align_to_byte();
    let picture = bw.into_bytes();

    let params = Mpeg1PictureParams {
        width: 48,
        height: 16,
        intra_quant: DEFAULT_INTRA_QUANT,
        non_intra_quant: [[16u8; 8]; 8],
    };
    let err = decode_mpeg1_d_picture(&picture, &params).unwrap_err();
    assert!(
        err.to_string().contains("2.4.4.4"),
        "unexpected error: {err}"
    );
}

#[test]
fn mpeg2_stream_with_d_picture_is_rejected() {
    use oxideav_mpeg12video::picture_header::Mpeg2PictureHeader;

    // A bare picture header with coding type 4 parses (11172-2 path)…
    let mut bw = BitWriter::new();
    write_d_picture_header(&mut bw, 0);
    // …then a picture_coding_extension start to satisfy the MPEG-2
    // chained parser's start-code scan.
    bw.write_u32(0x0000_01B5, 32);
    let buf = bw.into_bytes();

    let header = Mpeg2PictureHeader::parse(&buf).expect("bare 11172-2 parse accepts D");
    assert_eq!(header.picture_coding_type, PictureCodingType::DcIntra);

    let err = Mpeg2PictureHeader::parse_with_extension(&buf).unwrap_err();
    assert!(
        err.to_string().contains("Table 6-12"),
        "unexpected error: {err}"
    );
}
