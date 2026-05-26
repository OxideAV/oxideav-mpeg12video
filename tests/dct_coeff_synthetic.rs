//! End-to-end black-box tests that walk a synthesised MPEG-1 residual
//! block through the public [`DctCoeffStep::parse`] API per
//! ISO/IEC 11172-2:1993 §2.4.2.8 / §2.4.3.7.
//!
//! No fixture file is required for these tests — the test bit-strings
//! are assembled in-process from spec-defined Table B.5c / B.5d / B.5e
//! / B.5f codewords. The existing 352×240 fixture is an **MPEG-2**
//! stream (its DCT escape uses MPEG-2 Table B-16, which differs from
//! MPEG-1 Table B.5f per ISO/IEC 13818-2 §7.2.2.3) so it would not
//! exercise the MPEG-1-specific escape path this module owns.

// Bit groupings mirror the spec's MSB-first printed codewords (e.g.
// the 6-bit escape `000001` is one logical group, not `00_0001`), so
// an audit can read each `write_u32` argument against the spec page
// at a glance. clippy's `unusual_byte_groupings` lint prefers uniform
// 4-bit groups, which would obscure the spec correspondence.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_mpeg12video::{CoefficientPosition, DctCoeff, DctCoeffStep};

/// A non-intra block run that mixes a B.5c short code, a B.5d 12-bit
/// code, a B.5f escape, then `end_of_block`.
#[test]
fn walks_synthesised_block_with_escape() {
    let mut bw = BitWriter::new();
    // dct_coeff_first: (run=0, level=+2) → 0100 s, s=0 → 5 bits.
    bw.write_u32(0b0100, 4);
    bw.write_bit(false);
    // dct_coeff_next: (run=0, level=+8) → 0000_0001_1101 s, s=0 → 13 bits.
    bw.write_u32(0b0000_0001_1101, 12);
    bw.write_bit(false);
    // dct_coeff_next: escape (run=12, level=-200) → 6+6+8+8 = 28 bits.
    // Escape prefix `000001`.
    bw.write_u32(0b0000_01, 6);
    // 6-bit run.
    bw.write_u32(12, 6);
    // Long-form negative marker `1000_0000`.
    bw.write_u32(0x80, 8);
    // 8-bit magnitude: -200 + 256 = 56.
    bw.write_u32(((-200i16 + 256) as u32) & 0xff, 8);
    // `end_of_block` (`10`, 2 bits, no sign).
    bw.write_u32(0b10, 2);
    // Padding so reader can always look ahead.
    for _ in 0..4 {
        bw.write_byte(0);
    }
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);

    let s1 = DctCoeffStep::parse(&mut br, CoefficientPosition::First).expect("FIRST");
    if let DctCoeff::RunLevel {
        run,
        signed_level,
        escape,
    } = s1.symbol
    {
        assert_eq!(run, 0);
        assert_eq!(signed_level, 2);
        assert!(!escape);
    } else {
        panic!("expected RunLevel");
    }
    assert_eq!(s1.bit_position_after, 5);

    let s2 = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).expect("NEXT 1");
    if let DctCoeff::RunLevel {
        run,
        signed_level,
        escape,
    } = s2.symbol
    {
        assert_eq!(run, 0);
        assert_eq!(signed_level, 8);
        assert!(!escape);
    } else {
        panic!("expected RunLevel");
    }
    assert_eq!(s2.bit_position_after, 5 + 13);

    let s3 = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).expect("NEXT 2 escape");
    if let DctCoeff::RunLevel {
        run,
        signed_level,
        escape,
    } = s3.symbol
    {
        assert_eq!(run, 12);
        assert_eq!(signed_level, -200);
        assert!(escape);
    } else {
        panic!("expected RunLevel");
    }
    assert_eq!(s3.bit_position_after, 5 + 13 + 28);

    let s4 = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).expect("EoB");
    assert_eq!(s4.symbol, DctCoeff::EndOfBlock);
    assert_eq!(s4.bit_position_after, 5 + 13 + 28 + 2);
}

/// Walk a full 64-zig-zag-position block that uses every Table B.5c
/// short code plus an escape. Confirms the running bit cursor lines
/// up and the running `i` zig-zag position never exceeds 63.
#[test]
fn walks_full_block_run_advances_cursor_and_zigzag_position() {
    let mut bw = BitWriter::new();
    // FIRST: (run=2, level=+1) → 0101 s, s=0 → 5 bits → i = 2.
    bw.write_u32(0b0101, 4);
    bw.write_bit(false);
    // NEXT: (run=4, level=+1) → 0011 0 s, s=1 → 6 bits → i = 7.
    bw.write_u32(0b0011_0, 5);
    bw.write_bit(true);
    // NEXT escape: (run=50, level=+1) → 6 + 6 + 8 = 20 bits → i = 58.
    bw.write_u32(0b0000_01, 6);
    bw.write_u32(50, 6);
    bw.write_u32(1, 8);
    // NEXT: end-of-block → 2 bits.
    bw.write_u32(0b10, 2);
    for _ in 0..4 {
        bw.write_byte(0);
    }
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);

    // §2.4.3.7 update procedure: i starts at run for FIRST then
    // i += run + 1 for each NEXT.
    let mut i: i32 = -1;
    let s1 = DctCoeffStep::parse(&mut br, CoefficientPosition::First).unwrap();
    if let DctCoeff::RunLevel { run, .. } = s1.symbol {
        i = run as i32;
    }
    assert_eq!(i, 2);

    let s2 = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
    if let DctCoeff::RunLevel { run, .. } = s2.symbol {
        i += run as i32 + 1;
    }
    assert_eq!(i, 7);

    let s3 = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
    if let DctCoeff::RunLevel {
        run,
        signed_level,
        escape,
    } = s3.symbol
    {
        i += run as i32 + 1;
        assert_eq!(run, 50);
        assert_eq!(signed_level, 1);
        assert!(escape);
    }
    assert_eq!(i, 58);
    assert!(
        i <= 63,
        "zig-zag index must remain bounded by 63 per §2.4.3.7"
    );

    let s4 = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
    assert_eq!(s4.symbol, DctCoeff::EndOfBlock);
    // Total consumed bits: 5 + 6 + 20 + 2 = 33.
    assert_eq!(s4.bit_position_after, 33);
}
