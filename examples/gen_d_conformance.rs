//! Generate the hand-built **D-picture** conformance fixture
//! (`tests/fixtures/conformance/mpeg1-dpics-48x32.m1v`): an ISO/IEC
//! 11172-2 elementary stream coded entirely as dc intra-coded pictures
//! (`picture_coding_type == 4`, §2.4.3.4) — the picture type no
//! black-box encoder in reach emits. Each picture carries a
//! per-macroblock luma/chroma DC staircase so a wrong macroblock
//! address, a mis-chained §2.4.4.1 `dct_dc_*_past` predictor, or a
//! spurious AC/`end_of_block` read (D-blocks have neither, §2.4.2.8)
//! shows as a pixel difference.
//!
//! Geometry: 48x32 -> 3x2 macroblocks, one slice per macroblock row,
//! four pictures. Every macroblock is the Table B.2d `'1'` type with
//! six DC-only blocks and the `end_of_macroblock` `'1'` bit.
//!
//! Usage: `gen_d_conformance <out.m1v>`

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::picture_header::PICTURE_START_CODE;
use oxideav_mpeg12video::sequence_header::SEQUENCE_HEADER_CODE;

const MB_COLS: usize = 3;
const MB_ROWS: usize = 2;
const WIDTH: u32 = (MB_COLS as u32) * 16;
const HEIGHT: u32 = (MB_ROWS as u32) * 16;
const PICTURES: usize = 4;

/// §2.4.2.3 sequence header, no extension follows (11172-2).
fn write_seq(bw: &mut BitWriter) {
    bw.write_u32(SEQUENCE_HEADER_CODE, 32);
    bw.write_u32(WIDTH, 12); // horizontal_size
    bw.write_u32(HEIGHT, 12); // vertical_size
    bw.write_u32(0b0001, 4); // pel_aspect_ratio 1.0 (Table 2-5)
    bw.write_u32(0b0011, 4); // picture_rate 25 Hz (Table 2-6)
    bw.write_u32(0x3FFFF, 18); // bit_rate: variable
    bw.write_bit(true); // marker_bit
    bw.write_u32(16, 10); // vbv_buffer_size
    bw.write_bit(false); // constrained_parameters_flag
    bw.write_bit(false); // load_intra_quantizer_matrix
    bw.write_bit(false); // load_non_intra_quantizer_matrix
    bw.align_to_byte();
}

/// §2.4.2.5 picture header for a D-picture (no f_code fields).
fn write_pic_header(bw: &mut BitWriter, tr: u32) {
    bw.write_u32(PICTURE_START_CODE, 32);
    bw.write_u32(tr, 10);
    bw.write_u32(0b100, 3); // picture_coding_type = 4 (dc intra-coded)
    bw.write_u32(0xFFFF, 16); // vbv_delay
    bw.write_bit(false); // extra_bit_picture
    bw.align_to_byte();
}

/// Table B.5a (luma) / B.5b (chroma) `dct_dc_size` codes, indexed by
/// size 0..=8.
const LUMA_SIZE_CODES: [(u32, u32); 9] = [
    (0b100, 3),
    (0b00, 2),
    (0b01, 2),
    (0b101, 3),
    (0b110, 3),
    (0b1110, 4),
    (0b1_1110, 5),
    (0b11_1110, 6),
    (0b111_1110, 7),
];
const CHROMA_SIZE_CODES: [(u32, u32); 9] = [
    (0b00, 2),
    (0b01, 2),
    (0b10, 2),
    (0b110, 3),
    (0b1110, 4),
    (0b1_1110, 5),
    (0b11_1110, 6),
    (0b111_1110, 7),
    (0b1111_1110, 8),
];

/// Encode one §2.4.3.7 DC prelude: size VLC + differential payload.
/// Inverts the parse rule: positive → payload = value; negative →
/// `dct_zz[0] = ((-1) << size) | (payload + 1)`.
fn write_dc(bw: &mut BitWriter, dct_zz0: i32, luma: bool) {
    let mag = dct_zz0.unsigned_abs();
    let size = (32 - mag.leading_zeros()) as usize;
    assert!(size <= 8, "differential out of Table B.5 range");
    let (code, bits) = if luma {
        LUMA_SIZE_CODES[size]
    } else {
        CHROMA_SIZE_CODES[size]
    };
    bw.write_u32(code, bits);
    if size > 0 {
        let payload = if dct_zz0 >= 0 {
            dct_zz0 as u32
        } else {
            (dct_zz0 - (((-1i32) << size) + 1)) as u32
        };
        bw.write_u32(payload, size as u32);
    }
}

/// Flat target values per (picture, macroblock index): a staircase
/// that differs across macroblocks and pictures.
fn luma_value(pic: usize, mb: usize) -> i32 {
    (40 + 32 * mb as i32 + 9 * pic as i32).min(235)
}
fn cb_value(pic: usize, mb: usize) -> i32 {
    (96 + 12 * mb as i32 + 5 * pic as i32).min(240)
}
fn cr_value(pic: usize, mb: usize) -> i32 {
    (160 - 10 * mb as i32 - 3 * pic as i32).max(16)
}

fn main() {
    let out = std::env::args()
        .nth(1)
        .expect("usage: gen_d_conformance <out.m1v>");

    let mut bw = BitWriter::new();
    write_seq(&mut bw);

    for pic in 0..PICTURES {
        write_pic_header(&mut bw, pic as u32);
        for row in 0..MB_ROWS {
            // §2.4.2.6 slice: slice_start_code for this row,
            // quantizer_scale, extra_bit_slice '0'.
            bw.write_u32(0x0000_0101 + row as u32, 32);
            bw.write_u32(8, 5);
            bw.write_bit(false);

            // §2.4.4.1: predictors reset at slice start.
            let mut y_pred = 1024i32;
            let mut cb_pred = 1024i32;
            let mut cr_pred = 1024i32;

            for col in 0..MB_COLS {
                let mb = row * MB_COLS + col;
                bw.write_bit(true); // macroblock_address_increment = 1
                bw.write_bit(true); // macroblock_type '1' (Table B.2d)

                // Four luminance blocks: step to the target on Y0,
                // hold via the predictor chain on Y1..Y3.
                let y_recon = luma_value(pic, mb) * 8;
                write_dc(&mut bw, (y_recon - y_pred) / 8, true);
                y_pred = y_recon;
                for _ in 1..4 {
                    write_dc(&mut bw, 0, true);
                }
                let cb_recon = cb_value(pic, mb) * 8;
                write_dc(&mut bw, (cb_recon - cb_pred) / 8, false);
                cb_pred = cb_recon;
                let cr_recon = cr_value(pic, mb) * 8;
                write_dc(&mut bw, (cr_recon - cr_pred) / 8, false);
                cr_pred = cr_recon;

                bw.write_bit(true); // end_of_macroblock
            }
            bw.align_to_byte();
        }
    }
    bw.write_u32(0x0000_01B7, 32); // sequence_end_code
    std::fs::write(&out, bw.into_bytes()).expect("write fixture");
    eprintln!("wrote {out}: {WIDTH}x{HEIGHT}, {PICTURES} D-pictures");
}
