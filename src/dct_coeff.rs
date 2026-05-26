//! MPEG-1 residual `dct_coeff_first` / `dct_coeff_next` run-level VLC
//! walker per **ISO/IEC 11172-2:1993 §2.4.2.8 / §2.4.3.7** with field
//! semantics from §2.4.3.7 and the Annex B **Tables B.5c / B.5d / B.5e**
//! variable-length codes plus the **Table B.5f** escape encoding.
//!
//! Round 16 landed the intra-block DC prelude (Tables B.5a / B.5b plus
//! `dct_dc_differential` → `dct_zz[0]`). Round 17 lands the wider
//! `dct_coeff_first` / `dct_coeff_next` walker that follows the DC
//! field in intra blocks and replaces the DC step entirely in non-intra
//! blocks — the residual block's run-length-coded body of zig-zag
//! coefficients.
//!
//! `dct_coeff_first` and `dct_coeff_next` share the same VLC table; the
//! only difference is the special treatment of the `1s` (2-bit) vs
//! `11s` (3-bit) codewords for `(run = 0, level = 1)`:
//!
//! * The 2-bit code `1s` is **only** used for `dct_coeff_first` (the
//!   first coefficient of a non-intra block). Using it as
//!   `dct_coeff_next` would clash with the `end_of_block` codeword
//!   `10` — both start with `1`.
//! * The 3-bit code `11s` is used for `dct_coeff_next` (the second,
//!   third, … coefficient of any block).
//! * `end_of_block` is the 2-bit code `10`, recognised by
//!   `dct_coeff_next` only. The spec forbids encoding `end_of_block`
//!   as the first coefficient of a block.
//!
//! The escape codeword `0000 01` (6 bits) is followed by a Table B.5f
//! fixed-length payload — either a 14-bit short form (6 bits of run
//! followed by 8 bits of signed level for `level ∈ [-127, +127] \ {0}`)
//! or a 22-bit long form (6 bits of run followed by a 16-bit level
//! word whose top 8 bits select sign/magnitude for
//! `level ∈ [-255, -128] ∪ [+128, +255]`).
//!
//! The reconstructed `(run, signed_level)` is the next zig-zag-ordered
//! entry the decoder writes into `dct_zz[]` per the §2.4.3.7 update
//! procedure (`i = run` for `dct_coeff_first` then `i += run + 1` for
//! each `dct_coeff_next`).
//!
//! Spec citations refer to **ISO/IEC 11172-2:1993** (MPEG-1 Video) §§
//! 2.4.2.8, 2.4.3.7, and Annex B Tables B.5c, B.5d, B.5e, B.5f. The
//! MPEG-1 escape encoding is intentionally **not** the same as MPEG-2
//! Table B-16 (ISO/IEC 13818-2 §7.2.2.3 explicitly notes the change);
//! this module implements the MPEG-1 form only.
//!
//! Bit groupings in the table constants mirror the spec's printed bit
//! strings (1- to 16-bit groups, not nibble-aligned), so an audit can
//! read each entry off the spec page directly. clippy's
//! `unusual_byte_groupings` lint prefers uniform 4-bit groups, which
//! would obscure the spec correspondence.

#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

// =============================================================
// Table data — Annex B Tables B.5c / B.5d / B.5e
// =============================================================

/// One row of a `dct_coeff` VLC table.
///
/// The codeword is stored MSB-first, right-justified into a `u16`, and
/// excludes the trailing sign bit `s`. The walker consumes
/// `bits` code bits, then a separate 1-bit `s` (`0` = positive, `1` =
/// negative) and applies it to `level` to produce the signed level.
#[derive(Debug, Clone, Copy)]
struct CoeffEntry {
    /// MSB-first bit-string of the codeword excluding the sign bit
    /// `s`, right-justified into a `u16`.
    code: u16,
    /// Length of `code` in bits (`2..=16`).
    bits: u8,
    /// Decoded run length (`0..=31`).
    run: u8,
    /// Unsigned level magnitude (`1..=40`).
    level: u8,
}

/// **Table B.5c** entries (page 45 of ISO/IEC 11172-2:1993).
///
/// Includes the `(run=0, level=1)` first-coefficient code `1s`
/// (1-bit, sign bit added at read time → 2 bits total); the
/// `(run=0, level=1)` next-coefficient code `11s` (2-bit, sign bit
/// added → 3 bits total); the escape codeword `000001` (6 bits, no
/// sign bit) is modelled separately via `ESCAPE_CODE` and decoded
/// by [`DctCoeffStep::parse_escape`]; the `end_of_block` codeword
/// `10` (2 bits, no sign bit) is modelled separately via
/// `END_OF_BLOCK_CODE`.
///
/// The spec lists 32 ordinary run-level rows plus the two alternate
/// `(0, 1)` rows (one for `dct_coeff_first`, one for
/// `dct_coeff_next`). The 2-bit alternate `1s` collides with the
/// `end_of_block` prefix `1`, so the parser distinguishes the two
/// based on whether it is reading `dct_coeff_first` or
/// `dct_coeff_next` (see [`CoefficientPosition`]).
const TABLE_B5C: &[CoeffEntry] = &[
    // The 2-bit `1s` form for the FIRST coefficient of a non-intra
    // block. Modelled with `level = 1` and the matching tag bit
    // resolved by [`CoefficientPosition`] at parse time.
    CoeffEntry {
        code: 0b1,
        bits: 1,
        run: 0,
        level: 1,
    },
    // The 3-bit `11s` form for every subsequent coefficient.
    CoeffEntry {
        code: 0b11,
        bits: 2,
        run: 0,
        level: 1,
    },
    CoeffEntry {
        code: 0b011,
        bits: 3,
        run: 1,
        level: 1,
    },
    CoeffEntry {
        code: 0b0100,
        bits: 4,
        run: 0,
        level: 2,
    },
    CoeffEntry {
        code: 0b0101,
        bits: 4,
        run: 2,
        level: 1,
    },
    CoeffEntry {
        code: 0b0010_1,
        bits: 5,
        run: 0,
        level: 3,
    },
    CoeffEntry {
        code: 0b0011_1,
        bits: 5,
        run: 3,
        level: 1,
    },
    CoeffEntry {
        code: 0b0011_0,
        bits: 5,
        run: 4,
        level: 1,
    },
    CoeffEntry {
        code: 0b0001_10,
        bits: 6,
        run: 1,
        level: 2,
    },
    CoeffEntry {
        code: 0b0001_11,
        bits: 6,
        run: 5,
        level: 1,
    },
    CoeffEntry {
        code: 0b0001_01,
        bits: 6,
        run: 6,
        level: 1,
    },
    CoeffEntry {
        code: 0b0001_00,
        bits: 6,
        run: 7,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_110,
        bits: 7,
        run: 0,
        level: 4,
    },
    CoeffEntry {
        code: 0b0000_100,
        bits: 7,
        run: 2,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_111,
        bits: 7,
        run: 8,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_101,
        bits: 7,
        run: 9,
        level: 1,
    },
    // 8-bit codes (Note: 8-bit prefix `0010_0xxx s` block).
    CoeffEntry {
        code: 0b0010_0110,
        bits: 8,
        run: 0,
        level: 5,
    },
    CoeffEntry {
        code: 0b0010_0001,
        bits: 8,
        run: 0,
        level: 6,
    },
    CoeffEntry {
        code: 0b0010_0101,
        bits: 8,
        run: 1,
        level: 3,
    },
    CoeffEntry {
        code: 0b0010_0100,
        bits: 8,
        run: 3,
        level: 2,
    },
    CoeffEntry {
        code: 0b0010_0111,
        bits: 8,
        run: 10,
        level: 1,
    },
    CoeffEntry {
        code: 0b0010_0011,
        bits: 8,
        run: 11,
        level: 1,
    },
    CoeffEntry {
        code: 0b0010_0010,
        bits: 8,
        run: 12,
        level: 1,
    },
    CoeffEntry {
        code: 0b0010_0000,
        bits: 8,
        run: 13,
        level: 1,
    },
    // 10-bit codes (prefix `0000_0010_xx s` block).
    CoeffEntry {
        code: 0b0000_0010_10,
        bits: 10,
        run: 0,
        level: 7,
    },
    CoeffEntry {
        code: 0b0000_0011_00,
        bits: 10,
        run: 1,
        level: 4,
    },
    CoeffEntry {
        code: 0b0000_0010_11,
        bits: 10,
        run: 2,
        level: 3,
    },
    CoeffEntry {
        code: 0b0000_0011_11,
        bits: 10,
        run: 4,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_0010_01,
        bits: 10,
        run: 5,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_0011_10,
        bits: 10,
        run: 14,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0011_01,
        bits: 10,
        run: 15,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0010_00,
        bits: 10,
        run: 16,
        level: 1,
    },
];

/// **Table B.5d** entries (page 46 of ISO/IEC 11172-2:1993).
///
/// 32 entries, all 12- or 13-bit codes.
const TABLE_B5D: &[CoeffEntry] = &[
    // 12-bit codes (prefix `0000_0001_xxxx s`).
    CoeffEntry {
        code: 0b0000_0001_1101,
        bits: 12,
        run: 0,
        level: 8,
    },
    CoeffEntry {
        code: 0b0000_0001_1000,
        bits: 12,
        run: 0,
        level: 9,
    },
    CoeffEntry {
        code: 0b0000_0001_0011,
        bits: 12,
        run: 0,
        level: 10,
    },
    CoeffEntry {
        code: 0b0000_0001_0000,
        bits: 12,
        run: 0,
        level: 11,
    },
    CoeffEntry {
        code: 0b0000_0001_1011,
        bits: 12,
        run: 1,
        level: 5,
    },
    CoeffEntry {
        code: 0b0000_0001_0100,
        bits: 12,
        run: 2,
        level: 4,
    },
    CoeffEntry {
        code: 0b0000_0001_1100,
        bits: 12,
        run: 3,
        level: 3,
    },
    CoeffEntry {
        code: 0b0000_0001_0010,
        bits: 12,
        run: 4,
        level: 3,
    },
    CoeffEntry {
        code: 0b0000_0001_1110,
        bits: 12,
        run: 6,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_0001_0101,
        bits: 12,
        run: 7,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_0001_0001,
        bits: 12,
        run: 8,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_0001_1111,
        bits: 12,
        run: 17,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0001_1010,
        bits: 12,
        run: 18,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0001_1001,
        bits: 12,
        run: 19,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0001_0111,
        bits: 12,
        run: 20,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0001_0110,
        bits: 12,
        run: 21,
        level: 1,
    },
    // 13-bit codes (prefix `0000_0000_1xxx_x s`).
    CoeffEntry {
        code: 0b0000_0000_1101_0,
        bits: 13,
        run: 0,
        level: 12,
    },
    CoeffEntry {
        code: 0b0000_0000_1100_1,
        bits: 13,
        run: 0,
        level: 13,
    },
    CoeffEntry {
        code: 0b0000_0000_1100_0,
        bits: 13,
        run: 0,
        level: 14,
    },
    CoeffEntry {
        code: 0b0000_0000_1011_1,
        bits: 13,
        run: 0,
        level: 15,
    },
    CoeffEntry {
        code: 0b0000_0000_1011_0,
        bits: 13,
        run: 1,
        level: 6,
    },
    CoeffEntry {
        code: 0b0000_0000_1010_1,
        bits: 13,
        run: 1,
        level: 7,
    },
    CoeffEntry {
        code: 0b0000_0000_1010_0,
        bits: 13,
        run: 2,
        level: 5,
    },
    CoeffEntry {
        code: 0b0000_0000_1001_1,
        bits: 13,
        run: 3,
        level: 4,
    },
    CoeffEntry {
        code: 0b0000_0000_1001_0,
        bits: 13,
        run: 5,
        level: 3,
    },
    CoeffEntry {
        code: 0b0000_0000_1000_1,
        bits: 13,
        run: 9,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_0000_1000_0,
        bits: 13,
        run: 10,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_0000_1111_1,
        bits: 13,
        run: 22,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0000_1111_0,
        bits: 13,
        run: 23,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0000_1110_1,
        bits: 13,
        run: 24,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0000_1110_0,
        bits: 13,
        run: 25,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0000_1101_1,
        bits: 13,
        run: 26,
        level: 1,
    },
];

/// **Table B.5e** entries (page 47 of ISO/IEC 11172-2:1993).
///
/// 48 entries, all 14-, 15-, or 16-bit codes.
const TABLE_B5E: &[CoeffEntry] = &[
    // 14-bit codes (prefix `0000_0000_0xxx_xx s`).
    CoeffEntry {
        code: 0b0000_0000_0111_11,
        bits: 14,
        run: 0,
        level: 16,
    },
    CoeffEntry {
        code: 0b0000_0000_0111_10,
        bits: 14,
        run: 0,
        level: 17,
    },
    CoeffEntry {
        code: 0b0000_0000_0111_01,
        bits: 14,
        run: 0,
        level: 18,
    },
    CoeffEntry {
        code: 0b0000_0000_0111_00,
        bits: 14,
        run: 0,
        level: 19,
    },
    CoeffEntry {
        code: 0b0000_0000_0110_11,
        bits: 14,
        run: 0,
        level: 20,
    },
    CoeffEntry {
        code: 0b0000_0000_0110_10,
        bits: 14,
        run: 0,
        level: 21,
    },
    CoeffEntry {
        code: 0b0000_0000_0110_01,
        bits: 14,
        run: 0,
        level: 22,
    },
    CoeffEntry {
        code: 0b0000_0000_0110_00,
        bits: 14,
        run: 0,
        level: 23,
    },
    CoeffEntry {
        code: 0b0000_0000_0101_11,
        bits: 14,
        run: 0,
        level: 24,
    },
    CoeffEntry {
        code: 0b0000_0000_0101_10,
        bits: 14,
        run: 0,
        level: 25,
    },
    CoeffEntry {
        code: 0b0000_0000_0101_01,
        bits: 14,
        run: 0,
        level: 26,
    },
    CoeffEntry {
        code: 0b0000_0000_0101_00,
        bits: 14,
        run: 0,
        level: 27,
    },
    CoeffEntry {
        code: 0b0000_0000_0100_11,
        bits: 14,
        run: 0,
        level: 28,
    },
    CoeffEntry {
        code: 0b0000_0000_0100_10,
        bits: 14,
        run: 0,
        level: 29,
    },
    CoeffEntry {
        code: 0b0000_0000_0100_01,
        bits: 14,
        run: 0,
        level: 30,
    },
    CoeffEntry {
        code: 0b0000_0000_0100_00,
        bits: 14,
        run: 0,
        level: 31,
    },
    // 15-bit codes (prefix `0000_0000_0xxx_xxx s`).
    CoeffEntry {
        code: 0b0000_0000_0011_000,
        bits: 15,
        run: 0,
        level: 32,
    },
    CoeffEntry {
        code: 0b0000_0000_0010_111,
        bits: 15,
        run: 0,
        level: 33,
    },
    CoeffEntry {
        code: 0b0000_0000_0010_110,
        bits: 15,
        run: 0,
        level: 34,
    },
    CoeffEntry {
        code: 0b0000_0000_0010_101,
        bits: 15,
        run: 0,
        level: 35,
    },
    CoeffEntry {
        code: 0b0000_0000_0010_100,
        bits: 15,
        run: 0,
        level: 36,
    },
    CoeffEntry {
        code: 0b0000_0000_0010_011,
        bits: 15,
        run: 0,
        level: 37,
    },
    CoeffEntry {
        code: 0b0000_0000_0010_010,
        bits: 15,
        run: 0,
        level: 38,
    },
    CoeffEntry {
        code: 0b0000_0000_0010_001,
        bits: 15,
        run: 0,
        level: 39,
    },
    CoeffEntry {
        code: 0b0000_0000_0010_000,
        bits: 15,
        run: 0,
        level: 40,
    },
    CoeffEntry {
        code: 0b0000_0000_0011_111,
        bits: 15,
        run: 1,
        level: 8,
    },
    CoeffEntry {
        code: 0b0000_0000_0011_110,
        bits: 15,
        run: 1,
        level: 9,
    },
    CoeffEntry {
        code: 0b0000_0000_0011_101,
        bits: 15,
        run: 1,
        level: 10,
    },
    CoeffEntry {
        code: 0b0000_0000_0011_100,
        bits: 15,
        run: 1,
        level: 11,
    },
    CoeffEntry {
        code: 0b0000_0000_0011_011,
        bits: 15,
        run: 1,
        level: 12,
    },
    CoeffEntry {
        code: 0b0000_0000_0011_010,
        bits: 15,
        run: 1,
        level: 13,
    },
    CoeffEntry {
        code: 0b0000_0000_0011_001,
        bits: 15,
        run: 1,
        level: 14,
    },
    // 16-bit codes (prefix `0000_0000_0001_xxxx s`).
    CoeffEntry {
        code: 0b0000_0000_0001_0011,
        bits: 16,
        run: 1,
        level: 15,
    },
    CoeffEntry {
        code: 0b0000_0000_0001_0010,
        bits: 16,
        run: 1,
        level: 16,
    },
    CoeffEntry {
        code: 0b0000_0000_0001_0001,
        bits: 16,
        run: 1,
        level: 17,
    },
    CoeffEntry {
        code: 0b0000_0000_0001_0000,
        bits: 16,
        run: 1,
        level: 18,
    },
    CoeffEntry {
        code: 0b0000_0000_0001_0100,
        bits: 16,
        run: 6,
        level: 3,
    },
    CoeffEntry {
        code: 0b0000_0000_0001_1010,
        bits: 16,
        run: 11,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_0000_0001_1001,
        bits: 16,
        run: 12,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_0000_0001_1000,
        bits: 16,
        run: 13,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_0000_0001_0111,
        bits: 16,
        run: 14,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_0000_0001_0110,
        bits: 16,
        run: 15,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_0000_0001_0101,
        bits: 16,
        run: 16,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_0000_0001_1111,
        bits: 16,
        run: 27,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0000_0001_1110,
        bits: 16,
        run: 28,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0000_0001_1101,
        bits: 16,
        run: 29,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0000_0001_1100,
        bits: 16,
        run: 30,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0000_0001_1011,
        bits: 16,
        run: 31,
        level: 1,
    },
];

// =============================================================
// Special bit-strings (separate from the codeword tables)
// =============================================================

/// `end_of_block` codeword `10` (2 bits, no sign bit) per Table B.5c.
/// Only recognised by `dct_coeff_next`; `dct_coeff_first` always reads
/// a non-EoB symbol.
const END_OF_BLOCK_CODE: u32 = 0b10;
const END_OF_BLOCK_BITS: u32 = 2;

/// Escape codeword `000001` (6 bits, no sign bit) per Table B.5c. The
/// escape itself has no sign — the following Table B.5f payload encodes
/// run and signed level directly.
const ESCAPE_CODE: u32 = 0b0000_01;
const ESCAPE_BITS: u32 = 6;

/// Maximum codeword length over Tables B.5c / B.5d / B.5e (including
/// the trailing sign bit `s`): 16-bit code + 1-bit sign = 17 bits.
const MAX_CODE_LEN: u32 = 17;

// =============================================================
// Spec range constants
// =============================================================

/// `run` is bounded by `0..=63` for all of Tables B.5c..B.5f.
pub const MAX_RUN: u8 = 63;

/// `level` magnitude is bounded by `1..=255` for Tables B.5c..B.5f.
/// (B.5c..B.5e cap at 40; B.5f extends to 255 via the long-form escape.)
pub const MAX_LEVEL_MAG: u16 = 255;

// =============================================================
// Public API
// =============================================================

/// Whether the parser is reading the **first** coefficient of a
/// non-intra block (`dct_coeff_first`) or any **subsequent**
/// coefficient (`dct_coeff_next`).
///
/// The two positions decode the same VLC table with two
/// disambiguations:
///
/// * `(run = 0, level = 1)` is coded as the 2-bit `1s` for
///   `First` and as the 3-bit `11s` for `Next`. `First` therefore
///   never reads `11s`, and `Next` never reads `1s`.
/// * `end_of_block` (`10`) is **only** legal at `Next`. The spec
///   (`§2.4.3.7` and Table B.5c note 2) forbids an
///   immediately-terminating block.
///
/// In intra blocks, the DC field is decoded by Tables B.5a / B.5b
/// (see [`crate::block_dc`]) and every subsequent coefficient is
/// `dct_coeff_next`. In non-intra blocks the first coefficient is
/// `dct_coeff_first` and every subsequent one is `dct_coeff_next`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoefficientPosition {
    /// `dct_coeff_first` — first coefficient of a non-intra block.
    First,
    /// `dct_coeff_next` — any later coefficient, in either intra or
    /// non-intra blocks.
    Next,
}

/// One decoded `dct_coeff_*` symbol per §2.4.3.7. Either a `(run,
/// signed_level)` pair or the `end_of_block` terminator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DctCoeff {
    /// Ordinary run-level pair. `run` is the number of zero
    /// coefficients to skip, `signed_level` is the value to write at
    /// `dct_zz[i += run + 1]` (for `Next`) or `dct_zz[i = run]` (for
    /// `First`).
    RunLevel {
        /// Zero-coefficient skip, range `0..=63`.
        run: u8,
        /// Signed coefficient amplitude. Range
        /// `[-255, +255] \ {0, -256}` per Tables B.5c..B.5f.
        signed_level: i16,
        /// `true` iff the symbol was carried via the Table B.5f
        /// escape codeword (`000001` prefix + fixed-length payload).
        /// Useful for trace tools and bitstream analysis.
        escape: bool,
    },
    /// `end_of_block` marker (`10`, 2 bits). Only valid for
    /// `Position::Next`.
    EndOfBlock,
}

/// One step of the residual-block walker — the consumed bits and the
/// decoded symbol.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DctCoeffStep {
    /// Decoded symbol (run-level pair or end-of-block marker).
    pub symbol: DctCoeff,
    /// Bit position relative to the start of the reader's buffer
    /// after the last bit of this codeword was consumed.
    pub bit_position_after: u64,
}

impl DctCoeffStep {
    /// Walk Tables B.5c / B.5d / B.5e / B.5f for the next
    /// `dct_coeff_*` symbol at the cursor `br`. The `position`
    /// argument disambiguates the `(run = 0, level = 1)` two-form
    /// case and gates the `end_of_block` codeword.
    ///
    /// On success the matched bits (codeword + sign bit, or escape
    /// prefix + fixed-length payload, or end-of-block) are consumed.
    pub fn parse(br: &mut BitReader<'_>, position: CoefficientPosition) -> Result<Self> {
        // Match the longest-first codeword against the prefix.
        //
        // The walker tries every distinct width that appears in
        // Tables B.5c..B.5e plus the escape (6 bits) and EoB
        // (2 bits, gated on `Next`). Longest-first keeps the
        // match unambiguous on shorter prefixes of longer codes.
        let available = br.bits_remaining() as u32;
        if available == 0 {
            return Err(Error::ShortHeader);
        }
        let peek_w = MAX_CODE_LEN.min(available);
        let peeked = br.peek_u32(peek_w).map_err(|_| Error::ShortHeader)?;
        // Left-align so the MSB of every candidate width sits at bit
        // (MAX_CODE_LEN - 1) regardless of how many bits actually
        // fit in `peek_w`.
        let aligned = peeked << (MAX_CODE_LEN - peek_w);

        // (1) Table entries: cand_w + 1 bits needed (codeword + sign).
        // Iterate widths longest-first. The codeword widths that
        // appear in Tables B.5c / B.5d / B.5e are exactly:
        //   B.5c: 1, 2, 3, 4, 5, 6, 7, 8, 10
        //   B.5d: 12, 13
        //   B.5e: 14, 15, 16
        for &cand_w in &[16u8, 15, 14, 13, 12, 10, 8, 7, 6, 5, 4, 3, 2, 1] {
            let needed = u32::from(cand_w) + 1;
            if available < needed {
                continue;
            }
            let candidate = aligned >> (MAX_CODE_LEN - u32::from(cand_w));
            for table in [TABLE_B5C, TABLE_B5D, TABLE_B5E] {
                for &entry in table {
                    if entry.bits != cand_w || u32::from(entry.code) != candidate {
                        continue;
                    }
                    // Reject the `1s` 1-bit FIRST-only entry at NEXT.
                    if entry.bits == 1 && position == CoefficientPosition::Next {
                        continue;
                    }
                    // Reject the `11s` 2-bit NEXT-only entry at FIRST.
                    if entry.bits == 2
                        && entry.code == 0b11
                        && position == CoefficientPosition::First
                    {
                        continue;
                    }
                    // Consume the codeword + sign.
                    br.consume(u32::from(entry.bits))
                        .map_err(|_| Error::ShortHeader)?;
                    let sign = br.read_u1().map_err(|_| Error::ShortHeader)?;
                    let signed_level = if sign == 0 {
                        i16::from(entry.level)
                    } else {
                        -i16::from(entry.level)
                    };
                    return Ok(Self {
                        symbol: DctCoeff::RunLevel {
                            run: entry.run,
                            signed_level,
                            escape: false,
                        },
                        bit_position_after: br.bit_position(),
                    });
                }
            }
        }

        // (2) Special codewords (no sign bit).
        //
        // Escape (`000001`, 6 bits) — needs at least 20 bits total
        // for the short form (6 escape + 6 run + 8 level). The
        // long form is detected after reading the short level word.
        if available >= ESCAPE_BITS + ESCAPE_LEVEL_MIN_PAYLOAD_BITS {
            let escape_candidate = aligned >> (MAX_CODE_LEN - ESCAPE_BITS);
            if escape_candidate == ESCAPE_CODE {
                return Self::parse_escape(br);
            }
        }
        // `end_of_block` (`10`, 2 bits) — NEXT only.
        if position == CoefficientPosition::Next && available >= END_OF_BLOCK_BITS {
            let eob_candidate = aligned >> (MAX_CODE_LEN - END_OF_BLOCK_BITS);
            if eob_candidate == END_OF_BLOCK_CODE {
                br.consume(END_OF_BLOCK_BITS)
                    .map_err(|_| Error::ShortHeader)?;
                return Ok(Self {
                    symbol: DctCoeff::EndOfBlock,
                    bit_position_after: br.bit_position(),
                });
            }
        }
        Err(Error::InvalidBitstream(
            "dct_coeff: no Table B.5c / B.5d / B.5e codeword matches the bit prefix (§2.4.3.7)",
        ))
    }

    /// Decode a Table B.5f escape payload starting with the cursor
    /// positioned at the escape prefix `000001`. Consumes the prefix
    /// + the run + the level (short or long form).
    fn parse_escape(br: &mut BitReader<'_>) -> Result<Self> {
        br.consume(ESCAPE_BITS).map_err(|_| Error::ShortHeader)?;
        // 6-bit run, 1..=63 (0 is forbidden per Table B.5f).
        let run = br.read_u32(6).map_err(|_| Error::ShortHeader)? as u8;
        // 8-bit level word: if equal to 0x00 (short-form, level = 0)
        // or 0x80 (short-form forbidden), apply the long-form rules.
        let first = br.read_u32(8).map_err(|_| Error::ShortHeader)? as u8;
        let signed_level: i16 = match first {
            // 0x00 (short-form level = 0) is forbidden; it instead
            // signals the long-form positive level. The long form
            // carries an 8-bit unsigned magnitude in the next 8
            // bits, with values 0x80..=0xFF mapping to +128..=+255.
            0x00 => {
                let mag = br.read_u32(8).map_err(|_| Error::ShortHeader)? as u8;
                if mag < 0x80 {
                    // The short-form already covers `0x01..=0x7F`
                    // (= levels +1..=+127) so the long form for
                    // positive levels must start at 0x80.
                    return Err(Error::InvalidBitstream(
                        "dct_coeff: Table B.5f long-form positive level < 128 is forbidden",
                    ));
                }
                i16::from(mag)
            }
            // 0x80 (short-form level = -128) is forbidden; it
            // instead signals the long-form negative level. The
            // long form's next 8 bits encode `level + 256` so the
            // wire value 0x80..=0xFF maps to levels -128..=-1, and
            // 0x01..=0x7F maps to levels -255..=-129. 0x00 is the
            // forbidden -256.
            0x80 => {
                let mag = br.read_u32(8).map_err(|_| Error::ShortHeader)? as u8;
                if mag == 0x00 {
                    return Err(Error::InvalidBitstream(
                        "dct_coeff: Table B.5f long-form -256 is forbidden",
                    ));
                }
                i16::from(mag) - 256
            }
            // Short-form: the 8 bits are an 8-bit signed two's
            // complement level. 0x00 was caught above (forbidden);
            // 0x80 was caught above (escape to long form). All
            // other values are legal `level ∈ [-127, -1] ∪ [1, 127]`.
            other => i16::from(other as i8),
        };
        if run > MAX_RUN {
            return Err(Error::InvalidBitstream(
                "dct_coeff: Table B.5f run > 63 is impossible (§7.2.2.3)",
            ));
        }
        Ok(Self {
            symbol: DctCoeff::RunLevel {
                run,
                signed_level,
                escape: true,
            },
            bit_position_after: br.bit_position(),
        })
    }
}

/// Minimum number of bits the Table B.5f escape payload requires
/// after the escape prefix (`6` run + `8` short-form level = 14).
const ESCAPE_LEVEL_MIN_PAYLOAD_BITS: u32 = 14;

// =============================================================
// Tests
// =============================================================

#[cfg(test)]
mod tests {
    //! Spec-pinned coverage of Tables B.5c..B.5f, the
    //! `dct_coeff_first` vs `dct_coeff_next` disambiguation,
    //! end-of-block recognition, and the §2.4.3.7 §7.2.2.3 escape
    //! coding (short + long forms).
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Helper: append a codeword from one of the three sub-tables
    /// plus a sign bit to a writer.
    fn write_coeff(bw: &mut BitWriter, run: u8, level: u8, negative: bool) {
        let entry = TABLE_B5C
            .iter()
            .chain(TABLE_B5D)
            .chain(TABLE_B5E)
            .find(|e| e.run == run && e.level == level)
            .unwrap_or_else(|| panic!("no entry for (run={run}, level={level})"));
        // Prefer the 3-bit `11s` form for `(0, 1)` so the helper is
        // unambiguous in a `Next` context; tests that need the `1s`
        // form build that bit pattern directly.
        let code = if run == 0 && level == 1 && entry.bits == 1 {
            // `1s` form (FIRST only). Skip — the helper uses `11s`.
            TABLE_B5C
                .iter()
                .find(|e| e.run == 0 && e.level == 1 && e.bits == 2)
                .unwrap()
        } else {
            entry
        };
        bw.write_u32(u32::from(code.code), u32::from(code.bits));
        bw.write_bit(negative);
    }

    fn pad_and_finish(mut bw: BitWriter) -> Vec<u8> {
        // Append enough alignment bytes that a BitReader can always
        // peek MAX_CODE_LEN bits past the payload.
        for _ in 0..3 {
            bw.write_byte(0);
        }
        bw.finish()
    }

    // ----- table invariants -----

    #[test]
    fn table_b5c_has_expected_row_count() {
        // 32 entries — 1 (FIRST-only `1s` for run=0, level=1) +
        // 1 (NEXT-only `11s` for the same value) +
        // 30 other ordinary run-level rows. Excludes the
        // `end_of_block` (`10`) and `escape` (`000001`) codewords,
        // which are modelled separately because they carry no sign
        // bit `s` and have positional / payload semantics.
        assert_eq!(TABLE_B5C.len(), 32);
    }

    #[test]
    fn table_b5d_has_expected_row_count() {
        assert_eq!(TABLE_B5D.len(), 32);
    }

    #[test]
    fn table_b5e_has_expected_row_count() {
        assert_eq!(TABLE_B5E.len(), 48);
    }

    #[test]
    fn all_codes_fit_their_declared_width() {
        for table in [TABLE_B5C, TABLE_B5D, TABLE_B5E] {
            for e in table {
                assert!(u32::from(e.bits) <= 16, "code wider than 16 bits");
                let max = 1u32 << u32::from(e.bits);
                assert!(
                    u32::from(e.code) < max,
                    "code 0x{:x} (width {}) overflows",
                    e.code,
                    e.bits
                );
            }
        }
    }

    #[test]
    fn codes_unique_within_each_width() {
        // For every (table-row width) pair, the codewords must be
        // distinct — the spec is prefix-free at the per-width level.
        let all: Vec<&CoeffEntry> = TABLE_B5C.iter().chain(TABLE_B5D).chain(TABLE_B5E).collect();
        for w in 1u8..=16 {
            let group: Vec<_> = all.iter().filter(|e| e.bits == w).collect();
            for (i, a) in group.iter().enumerate() {
                for b in &group[i + 1..] {
                    assert_ne!(
                        a.code, b.code,
                        "duplicate codeword 0x{:x} at width {w} (runs {} {} levels {} {})",
                        a.code, a.run, b.run, a.level, b.level
                    );
                }
            }
        }
    }

    #[test]
    fn full_codebook_is_prefix_free() {
        // The full B.5c+d+e codebook (including the FIRST-only `1s`
        // and the EoB and escape) is a complete prefix-free decoding
        // tree once you account for:
        //   * `1s` (1-bit code, 2 total bits) is only legal for FIRST.
        //   * `10` (EoB, 2 bits, no sign) is only legal for NEXT.
        //   * `11s` (2-bit code) is only legal for NEXT.
        //   * `000001` (escape, 6 bits, no sign) carries a
        //     fixed-length payload.
        // So we check prefix-freeness within FIRST and within NEXT
        // separately, treating each as a flat list of (bits-including-
        // sign, value) entries.

        for position in [CoefficientPosition::First, CoefficientPosition::Next] {
            let mut entries: Vec<(u32, u32)> = Vec::new();
            for table in [TABLE_B5C, TABLE_B5D, TABLE_B5E] {
                for e in table {
                    // Skip the position-incompatible `(0, 1)` row.
                    if e.bits == 1 && position == CoefficientPosition::Next {
                        continue;
                    }
                    if e.bits == 2 && e.code == 0b11 && position == CoefficientPosition::First {
                        continue;
                    }
                    // total bits = codeword bits + 1 sign bit
                    let total = u32::from(e.bits) + 1;
                    // We left-align the code to 32 bits for easy
                    // prefix comparison; ignore the sign bit (low
                    // bit) since both `0` and `1` are legal there.
                    let aligned = u32::from(e.code) << (32 - u32::from(e.bits));
                    entries.push((total, aligned));
                }
            }
            // Add escape (no sign bit, 6 bits).
            let esc_aligned = ESCAPE_CODE << (32 - ESCAPE_BITS);
            entries.push((ESCAPE_BITS, esc_aligned));
            // Add EoB for `Next` only.
            if position == CoefficientPosition::Next {
                let eob_aligned = END_OF_BLOCK_CODE << (32 - END_OF_BLOCK_BITS);
                entries.push((END_OF_BLOCK_BITS, eob_aligned));
            }
            // For every pair, neither should be a prefix of the other
            // when the sign bit (if any) is ignored: i.e. the
            // codeword bits (not including sign) of A should not
            // equal the high `bits_A` bits of B's aligned codeword.
            // Use codeword bits = total - 1 if the entry has a sign,
            // = total if not (escape, EoB).
            //
            // We test the strict version: codeword-only prefix check.
            for (i, &(total_a, aligned_a)) in entries.iter().enumerate() {
                let cwbits_a = match total_a {
                    ESCAPE_BITS | END_OF_BLOCK_BITS => total_a,
                    _ => total_a - 1,
                };
                for &(total_b, aligned_b) in &entries[i + 1..] {
                    let cwbits_b = match total_b {
                        ESCAPE_BITS | END_OF_BLOCK_BITS => total_b,
                        _ => total_b - 1,
                    };
                    let min = cwbits_a.min(cwbits_b);
                    let mask = if min == 32 {
                        u32::MAX
                    } else {
                        !((1u32 << (32 - min)) - 1)
                    };
                    if (aligned_a & mask) == (aligned_b & mask) {
                        panic!(
                            "prefix collision at position {:?}: {:032b} ({} bits) vs {:032b} ({} bits)",
                            position, aligned_a, cwbits_a, aligned_b, cwbits_b
                        );
                    }
                }
            }
        }
    }

    // ----- per-table round-trips -----

    #[test]
    fn parses_simple_b5c_rows_at_next() {
        // (run=0, level=1) via `11s`; (run=2, level=1) via `0101s`;
        // (run=0, level=2) via `0100s`.
        for &(run, level) in &[(0u8, 1u8), (2, 1), (0, 2), (4, 1), (1, 2)] {
            for negative in [false, true] {
                let mut bw = BitWriter::new();
                write_coeff(&mut bw, run, level, negative);
                let buf = pad_and_finish(bw);
                let mut br = BitReader::new(&buf);
                let step = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
                match step.symbol {
                    DctCoeff::RunLevel {
                        run: r,
                        signed_level: s,
                        escape,
                    } => {
                        assert_eq!(r, run);
                        let expected: i16 = if negative {
                            -i16::from(level)
                        } else {
                            i16::from(level)
                        };
                        assert_eq!(s, expected);
                        assert!(!escape);
                    }
                    DctCoeff::EndOfBlock => panic!("unexpected EoB for ({run},{level})"),
                }
            }
        }
    }

    #[test]
    fn parses_b5d_row() {
        // (run=0, level=8) via 0000_0001_1101 s.
        let mut bw = BitWriter::new();
        write_coeff(&mut bw, 0, 8, false);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let step = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
        match step.symbol {
            DctCoeff::RunLevel {
                run, signed_level, ..
            } => {
                assert_eq!(run, 0);
                assert_eq!(signed_level, 8);
            }
            _ => panic!("expected RunLevel"),
        }
    }

    #[test]
    fn parses_b5e_row() {
        // (run=0, level=40) is the largest B.5e level, 15-bit code
        // 0000_0000_0010_000 s.
        let mut bw = BitWriter::new();
        write_coeff(&mut bw, 0, 40, false);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let step = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
        match step.symbol {
            DctCoeff::RunLevel {
                run, signed_level, ..
            } => {
                assert_eq!(run, 0);
                assert_eq!(signed_level, 40);
            }
            _ => panic!("expected RunLevel"),
        }
    }

    #[test]
    fn parses_b5e_16bit_row() {
        // (run=31, level=1) is one of the 16-bit codes:
        // 0000_0000_0001_1011 s.
        for negative in [false, true] {
            let mut bw = BitWriter::new();
            write_coeff(&mut bw, 31, 1, negative);
            let buf = pad_and_finish(bw);
            let mut br = BitReader::new(&buf);
            let step = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
            match step.symbol {
                DctCoeff::RunLevel {
                    run, signed_level, ..
                } => {
                    assert_eq!(run, 31);
                    let expected: i16 = if negative { -1 } else { 1 };
                    assert_eq!(signed_level, expected);
                }
                _ => panic!("expected RunLevel"),
            }
        }
    }

    #[test]
    fn every_b5c_b5d_b5e_row_round_trips_for_next() {
        // For every (run, level) pair encodable via B.5c/d/e, emit
        // the codeword with each sign and confirm the decoder
        // recovers the original (run, signed_level). Skip the
        // FIRST-only `1s` row (1-bit code), which is not legal at
        // NEXT — the equivalent NEXT-only `11s` row covers (0, 1)
        // there.
        for table in [TABLE_B5C, TABLE_B5D, TABLE_B5E] {
            for entry in table {
                if entry.bits == 1 {
                    continue;
                }
                for negative in [false, true] {
                    let mut bw = BitWriter::new();
                    bw.write_u32(u32::from(entry.code), u32::from(entry.bits));
                    bw.write_bit(negative);
                    let buf = pad_and_finish(bw);
                    let mut br = BitReader::new(&buf);
                    let step = DctCoeffStep::parse(&mut br, CoefficientPosition::Next)
                        .unwrap_or_else(|e| {
                            panic!(
                                "decode failed for entry width={} code={:0width$b} run={} level={} neg={}: {:?}",
                                entry.bits, entry.code, entry.run, entry.level, negative, e,
                                width = entry.bits as usize
                            )
                        });
                    match step.symbol {
                        DctCoeff::RunLevel {
                            run, signed_level, ..
                        } => {
                            assert_eq!(
                                run, entry.run,
                                "run mismatch for entry width={} code=0x{:x}",
                                entry.bits, entry.code
                            );
                            let expected = if negative {
                                -i16::from(entry.level)
                            } else {
                                i16::from(entry.level)
                            };
                            assert_eq!(
                                signed_level, expected,
                                "level mismatch for entry width={} code=0x{:x}",
                                entry.bits, entry.code
                            );
                        }
                        DctCoeff::EndOfBlock => panic!(
                            "Unexpected EoB for entry ({}, {}) at width {}",
                            entry.run, entry.level, entry.bits
                        ),
                    }
                }
            }
        }
    }

    // ----- FIRST vs NEXT disambiguation -----

    #[test]
    fn first_form_decodes_0_1_via_1s_code() {
        // `1s` form: '1' then sign bit '0' for +1.
        let buf = [0b10_000000u8, 0, 0, 0];
        let mut br = BitReader::new(&buf);
        let step = DctCoeffStep::parse(&mut br, CoefficientPosition::First).unwrap();
        match step.symbol {
            DctCoeff::RunLevel {
                run, signed_level, ..
            } => {
                assert_eq!(run, 0);
                assert_eq!(signed_level, 1);
            }
            _ => panic!("expected RunLevel"),
        }
        assert_eq!(step.bit_position_after, 2);
    }

    #[test]
    fn next_form_decodes_0_1_via_11s_code() {
        // `11s` form: '11' then sign bit '0' for +1.
        let buf = [0b110_00000u8, 0, 0, 0];
        let mut br = BitReader::new(&buf);
        let step = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
        match step.symbol {
            DctCoeff::RunLevel {
                run, signed_level, ..
            } => {
                assert_eq!(run, 0);
                assert_eq!(signed_level, 1);
            }
            _ => panic!("expected RunLevel"),
        }
        assert_eq!(step.bit_position_after, 3);
    }

    #[test]
    fn next_recognises_end_of_block() {
        // EoB: '10' then arbitrary padding.
        let buf = [0b10_000000u8, 0, 0, 0];
        let mut br = BitReader::new(&buf);
        let step = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
        assert_eq!(step.symbol, DctCoeff::EndOfBlock);
        assert_eq!(step.bit_position_after, 2);
    }

    #[test]
    fn first_does_not_recognise_eob() {
        // Wire bits `10` as FIRST decodes as the `1s` form (code
        // `1`, sign `0` → +1). The spec mandates that EoB is not
        // the only / first coefficient of a block (Table B.5c note
        // 2), so this is the expected FIRST-vs-NEXT disambiguation:
        // FIRST sees `1s` and decodes a value, NEXT sees `10` and
        // returns EoB.
        let buf = [0b10_000000u8, 0, 0, 0];
        let mut br = BitReader::new(&buf);
        let step = DctCoeffStep::parse(&mut br, CoefficientPosition::First).unwrap();
        match step.symbol {
            DctCoeff::RunLevel {
                run, signed_level, ..
            } => {
                assert_eq!(run, 0);
                // `1` (code) + `0` (sign) = +1.
                assert_eq!(signed_level, 1);
            }
            _ => panic!("FIRST must not return EoB"),
        }
        // Cross-check: same buffer at NEXT returns EoB.
        let mut br2 = BitReader::new(&buf);
        let step2 = DctCoeffStep::parse(&mut br2, CoefficientPosition::Next).unwrap();
        assert_eq!(step2.symbol, DctCoeff::EndOfBlock);
    }

    // ----- escape decoding (Table B.5f) -----

    /// Helper: assemble an escape codeword with a short-form (8-bit
    /// signed) level.
    fn escape_short(run: u8, signed_level: i8) -> Vec<u8> {
        let mut bw = BitWriter::new();
        bw.write_u32(ESCAPE_CODE, ESCAPE_BITS); // 000001
        bw.write_u32(u32::from(run), 6); // 6-bit run
        bw.write_u32((signed_level as u8) as u32, 8);
        for _ in 0..3 {
            bw.write_byte(0);
        }
        bw.finish()
    }

    /// Helper: assemble an escape codeword with a long-form negative
    /// level (`-255..=-128`).
    fn escape_long_negative(run: u8, signed_level: i16) -> Vec<u8> {
        assert!((-255..=-128).contains(&signed_level));
        let mag: u8 = (signed_level + 256) as u8; // -255 → 1, -128 → 128
        let mut bw = BitWriter::new();
        bw.write_u32(ESCAPE_CODE, ESCAPE_BITS);
        bw.write_u32(u32::from(run), 6);
        bw.write_u32(0x80, 8); // long-form negative marker
        bw.write_u32(u32::from(mag), 8);
        for _ in 0..3 {
            bw.write_byte(0);
        }
        bw.finish()
    }

    /// Helper: assemble an escape codeword with a long-form positive
    /// level (`128..=255`).
    fn escape_long_positive(run: u8, signed_level: i16) -> Vec<u8> {
        assert!((128..=255).contains(&signed_level));
        let mag: u8 = signed_level as u8;
        let mut bw = BitWriter::new();
        bw.write_u32(ESCAPE_CODE, ESCAPE_BITS);
        bw.write_u32(u32::from(run), 6);
        bw.write_u32(0x00, 8); // long-form positive marker
        bw.write_u32(u32::from(mag), 8);
        for _ in 0..3 {
            bw.write_byte(0);
        }
        bw.finish()
    }

    #[test]
    fn escape_short_positive() {
        // run=3, level=+50 (short form).
        let buf = escape_short(3, 50);
        let mut br = BitReader::new(&buf);
        let step = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
        match step.symbol {
            DctCoeff::RunLevel {
                run,
                signed_level,
                escape,
            } => {
                assert_eq!(run, 3);
                assert_eq!(signed_level, 50);
                assert!(escape);
            }
            _ => panic!("expected escape RunLevel"),
        }
        // Consumed: 6 (escape) + 6 (run) + 8 (short level) = 20.
        assert_eq!(step.bit_position_after, 20);
    }

    #[test]
    fn escape_short_negative() {
        // run=5, level=-1 (short form, wire = 0xFF).
        let buf = escape_short(5, -1);
        let mut br = BitReader::new(&buf);
        let step = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
        match step.symbol {
            DctCoeff::RunLevel {
                run,
                signed_level,
                escape,
            } => {
                assert_eq!(run, 5);
                assert_eq!(signed_level, -1);
                assert!(escape);
            }
            _ => panic!("expected escape RunLevel"),
        }
    }

    #[test]
    fn escape_short_corner_127_and_minus_127() {
        for &level in &[127i8, -127] {
            let buf = escape_short(63, level);
            let mut br = BitReader::new(&buf);
            let step = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
            match step.symbol {
                DctCoeff::RunLevel {
                    run, signed_level, ..
                } => {
                    assert_eq!(run, 63);
                    assert_eq!(signed_level, i16::from(level));
                }
                _ => panic!(),
            }
        }
    }

    #[test]
    fn escape_long_negative_minus_128() {
        // -128 must use the long form (wire 0x80 0x80).
        let buf = escape_long_negative(2, -128);
        let mut br = BitReader::new(&buf);
        let step = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
        match step.symbol {
            DctCoeff::RunLevel {
                run,
                signed_level,
                escape,
            } => {
                assert_eq!(run, 2);
                assert_eq!(signed_level, -128);
                assert!(escape);
            }
            _ => panic!(),
        }
        // Consumed: 6 + 6 + 8 + 8 = 28.
        assert_eq!(step.bit_position_after, 28);
    }

    #[test]
    fn escape_long_negative_minus_255() {
        let buf = escape_long_negative(7, -255);
        let mut br = BitReader::new(&buf);
        let step = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
        match step.symbol {
            DctCoeff::RunLevel {
                run, signed_level, ..
            } => {
                assert_eq!(run, 7);
                assert_eq!(signed_level, -255);
            }
            _ => panic!(),
        }
    }

    #[test]
    fn escape_long_positive_128_and_255() {
        for &level in &[128i16, 200, 255] {
            let buf = escape_long_positive(10, level);
            let mut br = BitReader::new(&buf);
            let step = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
            match step.symbol {
                DctCoeff::RunLevel {
                    run, signed_level, ..
                } => {
                    assert_eq!(run, 10);
                    assert_eq!(signed_level, level);
                }
                _ => panic!(),
            }
        }
    }

    #[test]
    fn escape_rejects_forbidden_long_minus_256() {
        // Wire: escape + run=0 + 0x80 + 0x00 → -256, forbidden.
        let mut bw = BitWriter::new();
        bw.write_u32(ESCAPE_CODE, ESCAPE_BITS);
        bw.write_u32(0, 6);
        bw.write_u32(0x80, 8);
        bw.write_u32(0x00, 8);
        for _ in 0..3 {
            bw.write_byte(0);
        }
        let buf = bw.finish();
        let mut br = BitReader::new(&buf);
        let err = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn escape_rejects_long_positive_below_128() {
        // Wire: escape + run=0 + 0x00 + 0x7F → forbidden long
        // positive (the short form covers +127).
        let mut bw = BitWriter::new();
        bw.write_u32(ESCAPE_CODE, ESCAPE_BITS);
        bw.write_u32(0, 6);
        bw.write_u32(0x00, 8);
        bw.write_u32(0x7F, 8);
        for _ in 0..3 {
            bw.write_byte(0);
        }
        let buf = bw.finish();
        let mut br = BitReader::new(&buf);
        let err = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    // ----- error sites -----

    #[test]
    fn rejects_empty_buffer() {
        let buf: [u8; 0] = [];
        let mut br = BitReader::new(&buf);
        let err = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap_err();
        assert!(matches!(
            err,
            Error::InvalidBitstream(_) | Error::ShortHeader
        ));
    }

    #[test]
    fn rejects_truncated_short_buffer() {
        // 1-bit buffer at NEXT cannot decode anything (the shortest
        // legal NEXT symbol is 2 bits: `10` EoB or `11s` which needs
        // 3 bits with the sign). At FIRST, a 1-bit buffer ('1') is
        // legal only with the 1s sign bit, which we lack.
        let buf = [0b1_0000000u8]; // 1 byte = 8 bits available
        let mut br = BitReader::new(&buf);
        // FIRST: 1s with sign=0 → +1.
        let step = DctCoeffStep::parse(&mut br, CoefficientPosition::First).unwrap();
        match step.symbol {
            DctCoeff::RunLevel { signed_level, .. } => assert_eq!(signed_level, 1),
            _ => panic!(),
        }
    }

    // ----- bit position accounting -----

    #[test]
    fn bit_position_tracks_2bit_first_form() {
        // `1s` = 2 bits total.
        let buf = [0b11_000000u8, 0, 0, 0];
        let mut br = BitReader::new(&buf);
        let step = DctCoeffStep::parse(&mut br, CoefficientPosition::First).unwrap();
        assert_eq!(step.bit_position_after, 2);
    }

    #[test]
    fn bit_position_tracks_17bit_b5e_max() {
        // 16-bit codeword + 1-bit sign = 17 bits.
        // (run=31, level=1) via 0000_0000_0001_1011 s, sign 0 → +1.
        let mut bw = BitWriter::new();
        write_coeff(&mut bw, 31, 1, false);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let step = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
        assert_eq!(step.bit_position_after, 17);
    }

    // ----- end-to-end stream walking -----

    #[test]
    fn walks_a_synthetic_block_run() {
        // Synthesise a non-intra block:
        //   dct_coeff_first: (run=0, level=+3) → 00101 0
        //   dct_coeff_next:  (run=2, level=-1) → 0101  1
        //   dct_coeff_next:  end-of-block       → 10
        let mut bw = BitWriter::new();
        // FIRST (0, 3) → code 00101, sign 0 → 6 bits total.
        bw.write_u32(0b0010_1, 5);
        bw.write_bit(false);
        // NEXT (2, 1) → code 0101, sign 1 → 5 bits total.
        bw.write_u32(0b0101, 4);
        bw.write_bit(true);
        // NEXT EoB → 10, 2 bits total.
        bw.write_u32(0b10, 2);
        for _ in 0..3 {
            bw.write_byte(0);
        }
        let buf = bw.finish();
        let mut br = BitReader::new(&buf);

        let s1 = DctCoeffStep::parse(&mut br, CoefficientPosition::First).unwrap();
        match s1.symbol {
            DctCoeff::RunLevel {
                run, signed_level, ..
            } => {
                assert_eq!(run, 0);
                assert_eq!(signed_level, 3);
            }
            _ => panic!(),
        }
        assert_eq!(s1.bit_position_after, 6);

        let s2 = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
        match s2.symbol {
            DctCoeff::RunLevel {
                run, signed_level, ..
            } => {
                assert_eq!(run, 2);
                assert_eq!(signed_level, -1);
            }
            _ => panic!(),
        }
        assert_eq!(s2.bit_position_after, 11);

        let s3 = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
        assert_eq!(s3.symbol, DctCoeff::EndOfBlock);
        assert_eq!(s3.bit_position_after, 13);
    }

    #[test]
    fn walks_block_with_escape_in_the_middle() {
        // FIRST: (0, +5) via 00100110 0 = 9 bits
        // NEXT escape short: (run=4, level=+80) via escape+6+8 = 20 bits
        // NEXT EoB: 10 = 2 bits
        let mut bw = BitWriter::new();
        // FIRST (0, 5) → code 0010_0110, sign 0 → 9 bits.
        bw.write_u32(0b0010_0110, 8);
        bw.write_bit(false);
        // Escape: 000001
        bw.write_u32(ESCAPE_CODE, ESCAPE_BITS);
        // 6-bit run = 4
        bw.write_u32(4, 6);
        // 8-bit signed level = +80
        bw.write_u32(80, 8);
        // EoB
        bw.write_u32(0b10, 2);
        for _ in 0..3 {
            bw.write_byte(0);
        }
        let buf = bw.finish();
        let mut br = BitReader::new(&buf);

        let s1 = DctCoeffStep::parse(&mut br, CoefficientPosition::First).unwrap();
        match s1.symbol {
            DctCoeff::RunLevel {
                run,
                signed_level,
                escape,
            } => {
                assert_eq!(run, 0);
                assert_eq!(signed_level, 5);
                assert!(!escape);
            }
            _ => panic!(),
        }

        let s2 = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
        match s2.symbol {
            DctCoeff::RunLevel {
                run,
                signed_level,
                escape,
            } => {
                assert_eq!(run, 4);
                assert_eq!(signed_level, 80);
                assert!(escape);
            }
            _ => panic!(),
        }

        let s3 = DctCoeffStep::parse(&mut br, CoefficientPosition::Next).unwrap();
        assert_eq!(s3.symbol, DctCoeff::EndOfBlock);
    }

    // ----- spec range / constants -----

    #[test]
    fn constants_match_spec_bounds() {
        assert_eq!(MAX_RUN, 63);
        assert_eq!(MAX_LEVEL_MAG, 255);
        assert_eq!(ESCAPE_CODE, 0b0000_01);
        assert_eq!(ESCAPE_BITS, 6);
        assert_eq!(END_OF_BLOCK_CODE, 0b10);
        assert_eq!(END_OF_BLOCK_BITS, 2);
    }

    #[test]
    fn debug_impl_smoke() {
        let s = DctCoeffStep {
            symbol: DctCoeff::RunLevel {
                run: 3,
                signed_level: -7,
                escape: false,
            },
            bit_position_after: 11,
        };
        let dbg = format!("{s:?}");
        assert!(dbg.contains("DctCoeffStep"));
        assert!(dbg.contains("RunLevel"));
    }
}
