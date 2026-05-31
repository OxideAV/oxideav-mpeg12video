//! Integration tests for the MPEG-2 §7.2.1 intra-block DC prelude.
//!
//! Spec: ISO/IEC 13818-2 (ITU-T H.262) §7.2.1, Tables B-12 and B-13
//! (DC size VLCs), Table 7-2 (`intra_dc_precision` → reset value),
//! Table 7-1 (colour-component `cc`).
//!
//! These tests drive the public re-exports
//! (`mpeg2_decode_dc_block`, `Mpeg2DcPredictors`,
//! `Mpeg2ColourComponent`, `mpeg2_dc_pred_reset_value`,
//! `mpeg2_qfs_zero_max`) end-to-end through synthetic bit-buffers to
//! confirm the spec invariants survive the crate boundary.

#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::{BitReader, BitWriter};

use oxideav_mpeg12video::{
    mpeg2_dc_pred_reset_value, mpeg2_decode_dc_block, mpeg2_qfs_zero_max, Mpeg2ColourComponent,
    Mpeg2DcPredictors, MPEG2_MAX_DC_SIZE,
};

/// Hand-table of (precision, reset, max). Pulled from Table 7-2 +
/// the §7.2.1 `[0, 2^(8 + intra_dc_precision) - 1]` constraint.
const PRECISION_TABLE: [(u8, i32, i32); 4] = [
    (0, 128, 255),
    (1, 256, 511),
    (2, 512, 1023),
    (3, 1024, 2047),
];

#[test]
fn re_exported_reset_table_matches_table_7_2() {
    for &(precision, reset, max) in &PRECISION_TABLE {
        assert_eq!(mpeg2_dc_pred_reset_value(precision).unwrap(), reset);
        assert_eq!(mpeg2_qfs_zero_max(precision).unwrap(), max);
        let predictors = Mpeg2DcPredictors::new(precision).unwrap();
        assert_eq!(predictors.luma, reset);
        assert_eq!(predictors.cb, reset);
        assert_eq!(predictors.cr, reset);
    }
}

#[test]
fn mpeg2_max_dc_size_is_eleven() {
    assert_eq!(MPEG2_MAX_DC_SIZE, 11);
}

/// Decode a four-block sequence (Y, Y, Cb, Cr) all in the same
/// intra macroblock and check that:
///  * the Y predictor chains across the two Y blocks
///  * the Cb predictor is independent of Y
///  * the Cr predictor is independent of both
///
/// All four blocks use Table B-12 / B-13 size 3; we craft raw
/// differentials of (`+1`, `-2`, `+3`, `-4`) so the resulting
/// predictor chain is:
///
/// | block | size | raw | dct_diff | pred-before | QFS[0] = pred-after |
/// |-------|------|-----|----------|-------------|---------------------|
/// | Y     | 3    | 101 |   +5     | 128         | 133                 |
/// | Y     | 3    | 010 |   -5     | 133         | 128                 |
/// | Cb    | 3    | 100 |   +4     | 128         | 132                 |
/// | Cr    | 3    | 011 |   -4     | 128         | 124                 |
#[test]
fn four_block_yycbcr_predictor_chain() {
    let mut bw = BitWriter::new();
    // Y block 0: B-12 size=3 → '101'; raw '101' = 5 → dct_diff = +5.
    bw.write_u32(0b101, 3);
    bw.write_u32(0b101, 3);
    // Y block 1: B-12 size=3 → '101'; raw '010' = 2 → dct_diff = -5
    //   (half_range=4; 2 < 4 → dct_diff = (2 + 1) - 8 = -5).
    bw.write_u32(0b101, 3);
    bw.write_u32(0b010, 3);
    // Cb block: B-13 size=3 → '110'; raw '100' = 4 → dct_diff = +4.
    bw.write_u32(0b110, 3);
    bw.write_u32(0b100, 3);
    // Cr block: B-13 size=3 → '110'; raw '011' = 3 → dct_diff = -4
    //   (half_range=4; 3 < 4 → dct_diff = (3 + 1) - 8 = -4).
    bw.write_u32(0b110, 3);
    bw.write_u32(0b011, 3);
    bw.write_bit(false);
    bw.align_to_byte();
    let buf = bw.finish();

    let mut predictors = Mpeg2DcPredictors::new(0).unwrap();
    let mut br = BitReader::new(&buf);

    let y0 = mpeg2_decode_dc_block(&mut br, &mut predictors, Mpeg2ColourComponent::Y).unwrap();
    assert_eq!(y0.dct_diff, 5);
    assert_eq!(y0.qfs_zero, 133);
    assert_eq!(predictors.luma, 133);
    assert_eq!(predictors.cb, 128);
    assert_eq!(predictors.cr, 128);

    let y1 = mpeg2_decode_dc_block(&mut br, &mut predictors, Mpeg2ColourComponent::Y).unwrap();
    assert_eq!(y1.dct_diff, -5);
    assert_eq!(y1.qfs_zero, 128);
    assert_eq!(predictors.luma, 128);
    assert_eq!(predictors.cb, 128);
    assert_eq!(predictors.cr, 128);

    let cb = mpeg2_decode_dc_block(&mut br, &mut predictors, Mpeg2ColourComponent::Cb).unwrap();
    assert_eq!(cb.dct_diff, 4);
    assert_eq!(cb.qfs_zero, 132);
    assert_eq!(predictors.luma, 128);
    assert_eq!(predictors.cb, 132);
    assert_eq!(predictors.cr, 128);

    let cr = mpeg2_decode_dc_block(&mut br, &mut predictors, Mpeg2ColourComponent::Cr).unwrap();
    assert_eq!(cr.dct_diff, -4);
    assert_eq!(cr.qfs_zero, 124);
    assert_eq!(predictors.luma, 128);
    assert_eq!(predictors.cb, 132);
    assert_eq!(predictors.cr, 124);
}

/// §7.2.1 mandates a reset of all three predictors at the start of
/// every slice, on every non-intra macroblock, and on every skipped
/// macroblock (`macroblock_address_increment > 1`). The reset value
/// is `dc_pred_reset_value(intra_dc_precision)`. Walk through a
/// non-trivial state, reset, and confirm we land back on Table 7-2.
#[test]
fn reset_returns_to_table_7_2() {
    let mut p = Mpeg2DcPredictors::new(2).unwrap(); // reset = 512
    assert_eq!(p.luma, 512);
    p.luma = 100;
    p.cb = 200;
    p.cr = 300;
    p.reset();
    assert_eq!(p.luma, 512);
    assert_eq!(p.cb, 512);
    assert_eq!(p.cr, 512);
}

/// At intra_dc_precision = 1 (reset = 256, max = 511) the bitstream
/// range is `[0, 511]`. A size-9 B-12 code (`1111 1110`) followed
/// by raw = 0 yields dct_diff = -511, which from the reset
/// predictor 256 lands at -255 — must fail.
#[test]
fn size_9_underflow_at_precision_1_is_rejected() {
    let mut bw = BitWriter::new();
    bw.write_u32(0b1111_1110, 8); // B-12 size 9
    for _ in 0..9 {
        bw.write_bit(false); // raw = 0 → dct_diff = -511
    }
    bw.write_bit(false);
    bw.align_to_byte();
    let buf = bw.finish();

    let mut p = Mpeg2DcPredictors::new(1).unwrap();
    let mut br = BitReader::new(&buf);
    let err = mpeg2_decode_dc_block(&mut br, &mut p, Mpeg2ColourComponent::Y).unwrap_err();
    let msg = format!("{err:?}");
    assert!(msg.contains("QFS[0]"), "unexpected error: {msg}");
}

/// Picking up size 10 / 11 confirms the B-12 / B-13 long-prefix
/// codes parse end-to-end. At intra_dc_precision = 3 the
/// [0, 2047] window is wide enough that prediction `2047 - 1023 =
/// 1024` works for a size-10 raw of `00_0000_0000`:
/// half_range = 512, raw 0 < half_range → dct_diff = 1 - 1024 = -1023.
/// 2047 + (-1023) = 1024 → in range.
#[test]
fn size_10_b13_long_prefix_round_trips() {
    let mut bw = BitWriter::new();
    bw.write_u32(0b11_1111_1110, 10); // B-13 size 10
    for _ in 0..10 {
        bw.write_bit(false); // raw = 0 → dct_diff = -1023
    }
    bw.write_bit(false);
    bw.align_to_byte();
    let buf = bw.finish();

    let mut p = Mpeg2DcPredictors::new(3).unwrap();
    p.cb = 2047;
    let mut br = BitReader::new(&buf);
    let dc = mpeg2_decode_dc_block(&mut br, &mut p, Mpeg2ColourComponent::Cb).unwrap();
    assert_eq!(dc.dct_dc_size, 10);
    assert_eq!(dc.dct_diff, -1023);
    assert_eq!(dc.qfs_zero, 1024);
    assert_eq!(p.cb, 1024);
}

/// The longest B-12 codeword `1_1111_1111` (size 11) parses
/// cleanly. Use intra_dc_precision = 3 so the [0, 2047] window
/// can accommodate a +2047 differential atop a reset of 0.
#[test]
fn size_11_b12_long_prefix_round_trips() {
    let mut bw = BitWriter::new();
    bw.write_u32(0b1_1111_1111, 9); // B-12 size 11
    bw.write_u32(0b111_1111_1111, 11); // raw = 2047 → dct_diff = +2047
    bw.write_bit(false);
    bw.align_to_byte();
    let buf = bw.finish();

    let mut p = Mpeg2DcPredictors::new(3).unwrap();
    p.luma = 0;
    let mut br = BitReader::new(&buf);
    let dc = mpeg2_decode_dc_block(&mut br, &mut p, Mpeg2ColourComponent::Y).unwrap();
    assert_eq!(dc.dct_dc_size, 11);
    assert_eq!(dc.dct_diff, 2047);
    assert_eq!(dc.qfs_zero, 2047);
    assert_eq!(p.luma, 2047);
    // Bit-position check: 9 (code) + 11 (raw) = 20.
    assert_eq!(dc.bit_position_after, 20);
}
