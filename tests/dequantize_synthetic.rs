//! End-to-end synthetic-fixture test for the MPEG-1 (ISO/IEC
//! 11172-2:1993) §2.4.4.1 / §2.4.4.2 dequantiser, fed directly by
//! the §2.4.3.7 `dct_coeff_first` / `dct_coeff_next` walker output.
//!
//! These tests assemble a small `(run, level)` stream of the kind
//! the round-17 walker emits, populate `dct_zz[]` per the §2.4.3.7
//! position rules (`i = run` for FIRST, `i += run + 1` for NEXT,
//! and `i` is reset to 0 at the start of an intra block per the
//! spec footnote on `dct_coeff_next`), then run the round-18
//! dequantiser and compare against the §2.4.4.1 closed-form
//! arithmetic.
//!
//! Spec basis: ISO/IEC 11172-2:1993 §2.4.3.2 (default quantiser
//! matrices, page 25), §2.4.3.7 (zig-zag position update), §2.4.4.1
//! (intra dequantiser, page 32), §2.4.4.2 (non-intra dequantiser,
//! page 35). The §A.1 IDCT is *not* exercised — these tests stop at
//! `dct_recon[m][n]`.

// The bit groupings in the synthetic codeword writes mirror the
// MPEG-1 spec's printed bit strings (`00101 s`, `0101 s`, `10`)
// rather than nibble-aligned groups, so a reader can match the
// helper line-by-line against the §2.4.3.7 / Table B.5c examples.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_mpeg12video::{
    dequantize_intra_block, dequantize_non_intra_block, finalise_intra_macroblock,
    CoefficientPosition, DctCoeff, DctCoeffStep, IntraBlockKind, IntraDcPredictors,
    DEFAULT_INTRA_QUANT, DEFAULT_NON_INTRA_QUANT, INVERSE_SCAN,
};

/// Walk the bitstream as a non-intra block per §2.4.3.7 and fill
/// the zig-zag-ordered `dct_zz[64]` array. Starts at FIRST, then
/// loops as NEXT until end_of_block.
fn walk_non_intra_block(br: &mut BitReader<'_>) -> [i32; 64] {
    let mut dct_zz = [0i32; 64];
    // §2.4.3.7 (FIRST): i = run.
    let first = DctCoeffStep::parse(br, CoefficientPosition::First).unwrap();
    let mut i: usize = match first.symbol {
        DctCoeff::RunLevel {
            run, signed_level, ..
        } => {
            let pos = usize::from(run);
            dct_zz[pos] = i32::from(signed_level);
            pos
        }
        DctCoeff::EndOfBlock => unreachable!("EoB illegal at FIRST per Table B.5c note 2"),
    };
    // §2.4.3.7 (NEXT): i += run + 1.
    loop {
        let step = DctCoeffStep::parse(br, CoefficientPosition::Next).unwrap();
        match step.symbol {
            DctCoeff::RunLevel {
                run, signed_level, ..
            } => {
                i += usize::from(run) + 1;
                assert!(i < 64, "spec forbids i > 63 (§2.4.3.7)");
                dct_zz[i] = i32::from(signed_level);
            }
            DctCoeff::EndOfBlock => break,
        }
    }
    dct_zz
}

/// Walk the bitstream as an intra block per §2.4.3.7: skip the DC
/// (caller already inserted `dct_zz[0]` from the §2.4.3.7 DC
/// prelude), then loop NEXT until end_of_block. The §2.4.3.7
/// footnote on `dct_coeff_next` says: *"If macroblock_intra == 1
/// then the term i shall be set to zero before the first
/// dct_coeff_next of the block."* So the AC stream uses `i += run +
/// 1` starting from `i = 0`.
fn walk_intra_block_ac(br: &mut BitReader<'_>, dct_zz: &mut [i32; 64]) {
    let mut i: usize = 0;
    loop {
        let step = DctCoeffStep::parse(br, CoefficientPosition::Next).unwrap();
        match step.symbol {
            DctCoeff::RunLevel {
                run, signed_level, ..
            } => {
                i += usize::from(run) + 1;
                assert!(i < 64, "spec forbids i > 63 (§2.4.3.7)");
                dct_zz[i] = i32::from(signed_level);
            }
            DctCoeff::EndOfBlock => break,
        }
    }
}

#[test]
fn non_intra_walker_feeds_dequantizer_end_to_end() {
    // Assemble a small non-intra block:
    //   FIRST: (run=0, level=+3) → code `00101`, sign=0 (6 bits)
    //   NEXT:  (run=2, level=-1) → code `0101`,  sign=1 (5 bits)
    //   NEXT:  EoB                → `10`              (2 bits)
    let mut bw = BitWriter::new();
    bw.write_u32(0b0010_1, 5);
    bw.write_bit(false);
    bw.write_u32(0b0101, 4);
    bw.write_bit(true);
    bw.write_u32(0b10, 2);
    for _ in 0..3 {
        bw.write_byte(0);
    }
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);

    let dct_zz = walk_non_intra_block(&mut br);
    assert_eq!(dct_zz[0], 3);
    // Second coefficient: i += run + 1 = 0 + 2 + 1 = 3, level = -1.
    assert_eq!(dct_zz[3], -1);

    let recon = dequantize_non_intra_block(&dct_zz, 8, &DEFAULT_NON_INTRA_QUANT).unwrap();

    // Spec-pinned closed forms for both non-zero coefficients (qs=8,
    // q=16, default non-intra matrix):
    //   At i = 0 -> (m, n) = (0, 0), dct_zz = +3:
    //     numerator = (2*3 + Sign(3)) * 8 * 16 = 7 * 128 = 896
    //     896 / 16 = 56, even -> 56 - 1 = 55, sat -> 55.
    //   At i = 3 -> (m, n) = (1, 1), dct_zz = -1:
    //     numerator = (2*-1 + Sign(-1)) * 8 * 16 = -3 * 128 = -384
    //     -384 / 16 = -24, even -> -24 - (-1) = -23, sat -> -23.
    let (m0, n0) = INVERSE_SCAN[0];
    let (m1, n1) = INVERSE_SCAN[3];
    assert_eq!(recon[m0 as usize][n0 as usize], 55);
    assert_eq!(recon[m1 as usize][n1 as usize], -23);

    // All other coefficients zero: the §2.4.4.2 zeroing pass fires.
    for (m, row) in recon.iter().enumerate() {
        for (n, &cell) in row.iter().enumerate() {
            if (m, n) == (m0 as usize, n0 as usize) || (m, n) == (m1 as usize, n1 as usize) {
                continue;
            }
            assert_eq!(cell, 0, "expected zero at ({m},{n})");
        }
    }
}

#[test]
fn intra_walker_with_dc_prelude_feeds_dequantizer_end_to_end() {
    // Intra block: caller sets dct_zz[0] from the §2.4.3.7 DC
    // prelude (here a synthetic +5), then we walk the AC body. The
    // bitstream below is the AC body only.
    //
    //   NEXT: (run=0, level=+3) → code `00101`, sign=0 (6 bits)
    //   NEXT: EoB → `10` (2 bits)
    let mut bw = BitWriter::new();
    bw.write_u32(0b0010_1, 5);
    bw.write_bit(false);
    bw.write_u32(0b10, 2);
    for _ in 0..3 {
        bw.write_byte(0);
    }
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);

    let mut dct_zz = [0i32; 64];
    dct_zz[0] = 5; // §2.4.3.7 DC prelude reconstruction.
    walk_intra_block_ac(&mut br, &mut dct_zz);
    // First AC: i = 0 + 0 + 1 = 1, level = +3.
    assert_eq!(dct_zz[1], 3);

    // Slice-start predictors (all 1024, past_intra_address = -2).
    let mut pred = IntraDcPredictors::at_slice_start();
    // First luma block, macroblock_address = 0:
    //   (0 - (-2)) = 2 > 1 → reset branch:
    //   DC = 128*8 + dct_zz[0]*8 = 1024 + 40 = 1064.
    // AC at i = 1, (m, n) = (0, 1), qs = 8, intra_quant[0][1] = 16:
    //   raw = 2 * 3 * 8 * 16 / 16 = 48, even -> 48 - 1 = 47,
    //   sat -> 47.
    let recon = dequantize_intra_block(
        &dct_zz,
        8,
        &DEFAULT_INTRA_QUANT,
        IntraBlockKind::LuminanceFirst,
        &mut pred,
        0,
    )
    .unwrap();
    assert_eq!(recon[0][0], 1064);
    let (m1, n1) = INVERSE_SCAN[1];
    assert_eq!(recon[m1 as usize][n1 as usize], 47);
    assert_eq!(pred.y_past, 1064);

    // Close the macroblock.
    finalise_intra_macroblock(&mut pred, 0);
    assert_eq!(pred.past_intra_address, 0);
}
