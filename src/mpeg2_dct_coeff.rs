//! MPEG-2 residual `dct_coeff_first` / `dct_coeff_next` walker per
//! **ISO/IEC 13818-2 (ITU-T H.262) §7.2.2** with field semantics from
//! §6.2.6 and §7.2.2.4 plus the Annex B **Tables B-14**, **B-15**, and
//! **B-16** variable-length codes.
//!
//! MPEG-2 ships **two** run-level VLC tables for the §6.2.6 residual
//! block body. The selector is the §6.3.10 `intra_vlc_format` picture-
//! coding-extension flag, resolved through §7.2.2.1 Table 7-3:
//!
//! | `intra_vlc_format` | intra blocks | non-intra blocks |
//! |--------------------|--------------|------------------|
//! | `0`                | **B-14**     | **B-14**         |
//! | `1`                | **B-15**     | **B-14**         |
//!
//! Both tables share the same Annex B **Table B-16** escape encoding
//! (6-bit run + 12-bit signed_level, which §7.2.2.3 explicitly notes is
//! **different** from the MPEG-1 §2.4.3.7 / Table B.5f form). The
//! MPEG-1 walker in [`crate::dct_coeff`] implements the older format;
//! this module is its MPEG-2 sibling.
//!
//! ## FIRST vs NEXT
//!
//! §7.2.2.2 ("Table selection for the first coefficient of a non-intra
//! block") modifies Table B-14 by NOTE 2 / NOTE 3 at its foot:
//!
//! * The 2-bit code `1s` ("NOTE 2") encodes `(run=0, level=±1)` for the
//!   FIRST coefficient of a non-intra block.
//! * The 3-bit code `11s` ("NOTE 3") encodes the same `(0, ±1)` for
//!   every later coefficient.
//!
//! The §7.2.2.2 note clarifies that this modification does **not**
//! apply when Table B-14 is used for an intra block, because the first
//! coefficient of an intra block is the DC value coded by §7.2.1
//! (Tables B-12 / B-13 — handled by [`crate::block_dc`]), so the
//! residual walker always starts at the second coefficient there.
//! Table B-15 is therefore only ever entered at NEXT (intra block,
//! `intra_vlc_format = 1`); its `(0, ±1)` codeword is the 2-bit `10s`,
//! and there is no NOTE 2 / NOTE 3 split.
//!
//! ## End of block
//!
//! `end_of_block` is `10` (2 bits, no sign bit) for Table B-14 — the
//! same encoding as MPEG-1. Table B-15 uses `0110` (4 bits, no sign
//! bit) instead, which is one of the key shape differences between the
//! two MPEG-2 tables.
//!
//! ## Escape
//!
//! Both tables share the **`0000 01`** (6-bit, no sign bit) escape
//! prefix. The Table B-16 payload that follows is:
//!
//! * 6-bit fixed-length `run` in `0..=63`.
//! * 12-bit fixed-length `signed_level` in `[-2047, +2047] \ {0}`
//!   (the all-zeros word is forbidden).
//!
//! Spec citations refer to **ISO/IEC 13818-2** (ITU-T H.262) §§7.2.2,
//! 7.2.2.1 (Table 7-3), 7.2.2.2 (FIRST / NEXT modification), 7.2.2.3
//! (escape, Table B-16), 7.2.2.4 (decoder pseudo-code), and Annex B
//! Tables B-14 and B-15. Bit groupings in the constants below mirror
//! the spec's printed bit strings so an audit can read each entry
//! against the spec page directly. clippy's `unusual_byte_groupings`
//! lint prefers uniform 4-bit groups, which would obscure the spec
//! correspondence.

#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

// =============================================================
// Codeword storage
// =============================================================

/// One row of an MPEG-2 §7.2.2 VLC table.
///
/// The codeword is stored MSB-first, right-justified into a `u16`,
/// excluding the trailing sign bit `s`. The walker consumes `bits`
/// code bits, then a separate 1-bit `s` (`0` = positive, `1` =
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

// =============================================================
// Annex B Table B-14 --- DCT coefficients Table zero
// =============================================================
//
// 113 ordinary rows (32 + 32 + 32 + 16 + the FIRST-only `1s` form),
// transcribed against ISO/IEC 13818-2 Annex B Table B-14, page 147
// ("Table B-14 --- DCT coefficients Table zero"), continuing on
// pages 148, 149, and 150 (concluded).
//
// The two `(0, 1)` forms (`1s` and `11s`) are stored as separate rows;
// the walker resolves the FIRST / NEXT disambiguation per
// §7.2.2.2 NOTE 2 / NOTE 3.

/// Table B-14 entries with codeword width `<= 8` bits (excluding sign).
///
/// Includes the FIRST-only `1s` (1-bit) and NEXT-only `11s` (2-bit)
/// alternates for `(run=0, level=1)` per §7.2.2.2 NOTE 2 / NOTE 3.
/// The 2-bit `10` `end_of_block` codeword and the 6-bit `000001`
/// escape codeword are modelled separately via `EOB_B14_*` and
/// `ESCAPE_CODE` / `ESCAPE_BITS`.
const TABLE_B14_PAGE1: &[CoeffEntry] = &[
    // 1-bit NOTE 2 form (FIRST coefficient of a non-intra block).
    CoeffEntry {
        code: 0b1,
        bits: 1,
        run: 0,
        level: 1,
    },
    // 2-bit NOTE 3 form (every subsequent coefficient).
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
    // 8-bit codes (prefix `0010_0xxx s`).
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
    // 10-bit codes (prefix `0000_0010_xx s` and `0000_0011_xx s`).
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

/// Table B-14 page 2 entries (12-bit and 13-bit codes).
const TABLE_B14_PAGE2: &[CoeffEntry] = &[
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

/// Table B-14 page 3 entries (14-bit and 15-bit codes).
const TABLE_B14_PAGE3: &[CoeffEntry] = &[
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
    // 15-bit codes (prefix `0000_0000_001x_xxx s`).
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
];

/// Table B-14 page 4 entries (16-bit codes, concluded).
const TABLE_B14_PAGE4: &[CoeffEntry] = &[
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
// Annex B Table B-15 --- DCT coefficients Table one
// =============================================================
//
// Used only for intra blocks when `intra_vlc_format == 1`. The first
// coefficient of an intra block is the §7.2.1 DC value (Tables B-12 /
// B-13 in [`crate::block_dc`]), so this table is always entered at the
// "Next" position — there is no NOTE 2 / NOTE 3 alternate form for
// `(0, 1)`.
//
// 111 ordinary rows transcribed against ISO/IEC 13818-2 Annex B
// Table B-15, pages 151, 152, 153, and 154 (concluded).
//
// `end_of_block` is the 4-bit codeword `0110` (no sign bit) and the
// escape prefix is `0000 01` (6 bits, no sign bit), both modelled
// separately via `EOB_B15_*` and `ESCAPE_CODE`.

/// Table B-15 page 1 entries — variable widths.
const TABLE_B15_PAGE1: &[CoeffEntry] = &[
    // 3-bit codes.
    CoeffEntry {
        code: 0b10,
        bits: 2,
        run: 0,
        level: 1,
    },
    CoeffEntry {
        code: 0b010,
        bits: 3,
        run: 1,
        level: 1,
    },
    CoeffEntry {
        code: 0b110,
        bits: 3,
        run: 0,
        level: 2,
    },
    // 5-bit codes.
    CoeffEntry {
        code: 0b0010_1,
        bits: 5,
        run: 2,
        level: 1,
    },
    CoeffEntry {
        code: 0b0111,
        bits: 4,
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
        code: 0b0001_10,
        bits: 6,
        run: 4,
        level: 1,
    },
    CoeffEntry {
        code: 0b0011_0,
        bits: 5,
        run: 1,
        level: 2,
    },
    CoeffEntry {
        code: 0b0001_11,
        bits: 6,
        run: 5,
        level: 1,
    },
    // 7-bit codes.
    CoeffEntry {
        code: 0b0000_110,
        bits: 7,
        run: 6,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_100,
        bits: 7,
        run: 7,
        level: 1,
    },
    CoeffEntry {
        code: 0b1110_0,
        bits: 5,
        run: 0,
        level: 4,
    },
    CoeffEntry {
        code: 0b0000_111,
        bits: 7,
        run: 2,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_101,
        bits: 7,
        run: 8,
        level: 1,
    },
    CoeffEntry {
        code: 0b1111_000,
        bits: 7,
        run: 9,
        level: 1,
    },
    // 5-bit `1110 1 s` and the 6-bit `0001 01 s`.
    CoeffEntry {
        code: 0b1110_1,
        bits: 5,
        run: 0,
        level: 5,
    },
    CoeffEntry {
        code: 0b0001_01,
        bits: 6,
        run: 0,
        level: 6,
    },
    CoeffEntry {
        code: 0b1111_001,
        bits: 7,
        run: 1,
        level: 3,
    },
    CoeffEntry {
        code: 0b0010_0110,
        bits: 8,
        run: 3,
        level: 2,
    },
    CoeffEntry {
        code: 0b1111_010,
        bits: 7,
        run: 10,
        level: 1,
    },
    CoeffEntry {
        code: 0b0010_0001,
        bits: 8,
        run: 11,
        level: 1,
    },
    CoeffEntry {
        code: 0b0010_0101,
        bits: 8,
        run: 12,
        level: 1,
    },
    CoeffEntry {
        code: 0b0010_0100,
        bits: 8,
        run: 13,
        level: 1,
    },
    CoeffEntry {
        code: 0b0001_00,
        bits: 6,
        run: 0,
        level: 7,
    },
    CoeffEntry {
        code: 0b0010_0111,
        bits: 8,
        run: 1,
        level: 4,
    },
    CoeffEntry {
        code: 0b1111_1100,
        bits: 8,
        run: 2,
        level: 3,
    },
    CoeffEntry {
        code: 0b1111_1101,
        bits: 8,
        run: 4,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_0010_0,
        bits: 9,
        run: 5,
        level: 2,
    },
    CoeffEntry {
        code: 0b0000_0010_1,
        bits: 9,
        run: 14,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0011_1,
        bits: 9,
        run: 15,
        level: 1,
    },
    CoeffEntry {
        code: 0b0000_0011_01,
        bits: 10,
        run: 16,
        level: 1,
    },
];

/// Table B-15 page 2 entries — 8-bit + 10-bit + 12-bit + 13-bit codes.
const TABLE_B15_PAGE2: &[CoeffEntry] = &[
    CoeffEntry {
        code: 0b1111_011,
        bits: 7,
        run: 0,
        level: 8,
    },
    CoeffEntry {
        code: 0b1111_100,
        bits: 7,
        run: 0,
        level: 9,
    },
    CoeffEntry {
        code: 0b0010_0011,
        bits: 8,
        run: 0,
        level: 10,
    },
    CoeffEntry {
        code: 0b0010_0010,
        bits: 8,
        run: 0,
        level: 11,
    },
    CoeffEntry {
        code: 0b0010_0000,
        bits: 8,
        run: 1,
        level: 5,
    },
    CoeffEntry {
        code: 0b0000_0011_00,
        bits: 10,
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
    CoeffEntry {
        code: 0b1111_1010,
        bits: 8,
        run: 0,
        level: 12,
    },
    CoeffEntry {
        code: 0b1111_1011,
        bits: 8,
        run: 0,
        level: 13,
    },
    CoeffEntry {
        code: 0b1111_1110,
        bits: 8,
        run: 0,
        level: 14,
    },
    CoeffEntry {
        code: 0b1111_1111,
        bits: 8,
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

/// Table B-15 page 3 entries — 14-bit + 15-bit codes.
const TABLE_B15_PAGE3: &[CoeffEntry] = &[
    // 14-bit codes.
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
    // 15-bit codes.
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
];

/// Table B-15 page 4 entries — 16-bit codes (concluded).
const TABLE_B15_PAGE4: &[CoeffEntry] = &[
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

/// Table B-14 `end_of_block` codeword `10` (2 bits, no sign).
const EOB_B14_CODE: u32 = 0b10;
const EOB_B14_BITS: u32 = 2;

/// Table B-15 `end_of_block` codeword `0110` (4 bits, no sign).
const EOB_B15_CODE: u32 = 0b0110;
const EOB_B15_BITS: u32 = 4;

/// Common escape codeword `000001` (6 bits, no sign).
const ESCAPE_CODE: u32 = 0b0000_01;
const ESCAPE_BITS: u32 = 6;

/// Table B-16 escape payload: 6-bit run + 12-bit signed_level.
const ESCAPE_RUN_BITS: u32 = 6;
const ESCAPE_LEVEL_BITS: u32 = 12;
const ESCAPE_PAYLOAD_BITS: u32 = ESCAPE_RUN_BITS + ESCAPE_LEVEL_BITS;

/// Maximum codeword length over Tables B-14 / B-15 (including the
/// trailing sign bit `s`): 16-bit code + 1-bit sign = 17 bits.
const MAX_CODE_LEN: u32 = 17;

// =============================================================
// Spec range constants
// =============================================================

/// `run` is bounded by `0..=63` for both VLC tables and the escape.
pub const MAX_RUN: u8 = 63;

/// Escape `signed_level` is bounded by `[-2047, +2047] \ {0}` per
/// Table B-16. The wire word value `0x000` is forbidden.
pub const ESCAPE_SIGNED_LEVEL_MIN: i16 = -2047;
pub const ESCAPE_SIGNED_LEVEL_MAX: i16 = 2047;

// =============================================================
// Public API
// =============================================================

/// Selector for which Table B-14 or B-15 the walker should use.
///
/// Resolved per §7.2.2.1 Table 7-3 from the §6.3.10
/// `intra_vlc_format` flag and `macroblock_intra`:
///
/// * `(intra_vlc_format = 0, *)`             → `TableZero` (Table B-14)
/// * `(intra_vlc_format = 1, intra)`         → `TableOne`  (Table B-15)
/// * `(intra_vlc_format = 1, non-intra)`     → `TableZero` (Table B-14)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TableSelection {
    /// Annex B **Table B-14** ("Table zero").
    TableZero,
    /// Annex B **Table B-15** ("Table one"). Intra-only.
    TableOne,
}

impl TableSelection {
    /// Resolve §7.2.2.1 Table 7-3 from the picture-coding-extension
    /// `intra_vlc_format` flag and the macroblock's `macroblock_intra`
    /// flag.
    pub fn from_context(intra_vlc_format: bool, macroblock_intra: bool) -> Self {
        match (intra_vlc_format, macroblock_intra) {
            (true, true) => Self::TableOne,
            _ => Self::TableZero,
        }
    }
}

/// Whether the parser is reading the **first** coefficient of a
/// non-intra block (`dct_coeff_first`) or any **subsequent**
/// coefficient (`dct_coeff_next`).
///
/// Per §7.2.2.2:
///
/// * Only Table B-14 used for a non-intra block sees the NOTE 2 / NOTE
///   3 modification — the `(0, ±1)` codeword is `1s` for FIRST and
///   `11s` for NEXT.
/// * Table B-15 is intra-only and its walker always starts at NEXT
///   (the §7.2.1 DC field has already been consumed).
/// * Table B-14 used for an intra block also always starts at NEXT
///   for the same reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoefficientPosition {
    /// `dct_coeff_first` — first coefficient of a non-intra block.
    First,
    /// `dct_coeff_next` — any later coefficient, or any coefficient
    /// of an intra block.
    Next,
}

/// One decoded `dct_coeff_*` symbol per §7.2.2. Either a `(run,
/// signed_level)` pair or the `end_of_block` terminator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DctCoeff {
    /// Ordinary run-level pair. `run` is the number of zero
    /// coefficients to skip, `signed_level` is the value to write
    /// at the next zig-zag position. `escape` records whether the
    /// symbol was carried via the Table B-16 escape.
    RunLevel {
        /// Zero-coefficient skip, range `0..=63`.
        run: u8,
        /// Signed coefficient amplitude. Range `[-2047, +2047] \ {0}`
        /// via the escape; `[-40, +40] \ {0}` via the VLC body.
        signed_level: i16,
        /// `true` iff the symbol was carried via the Table B-16
        /// escape (`000001` prefix + 6-bit run + 12-bit signed_level).
        escape: bool,
    },
    /// `end_of_block` marker (2 bits `10` for Table B-14, 4 bits
    /// `0110` for Table B-15). Only valid for `Position::Next`.
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
    /// Walk Tables B-14 / B-15 / B-16 for the next `dct_coeff_*`
    /// symbol at the cursor `br`. `table` selects the §7.2.2.1
    /// Table 7-3 row and `position` disambiguates `(0, ±1)` /
    /// gates the `end_of_block` codeword.
    ///
    /// On success the matched bits (codeword + sign bit, or escape
    /// prefix + Table B-16 payload, or end-of-block) are consumed.
    pub fn parse(
        br: &mut BitReader<'_>,
        table: TableSelection,
        position: CoefficientPosition,
    ) -> Result<Self> {
        let available = br.bits_remaining() as u32;
        if available == 0 {
            return Err(Error::ShortHeader);
        }
        let peek_w = MAX_CODE_LEN.min(available);
        let peeked = br.peek_u32(peek_w).map_err(|_| Error::ShortHeader)?;
        let aligned = peeked << (MAX_CODE_LEN - peek_w);

        // (1) Per-table codeword walk, longest-first.
        let pages: [&[CoeffEntry]; 4] = match table {
            TableSelection::TableZero => [
                TABLE_B14_PAGE1,
                TABLE_B14_PAGE2,
                TABLE_B14_PAGE3,
                TABLE_B14_PAGE4,
            ],
            TableSelection::TableOne => [
                TABLE_B15_PAGE1,
                TABLE_B15_PAGE2,
                TABLE_B15_PAGE3,
                TABLE_B15_PAGE4,
            ],
        };

        // Widths to try: every distinct codeword width that appears
        // anywhere in the selected table. We iterate longest-first so
        // a longer match wins over a shorter prefix.
        for &cand_w in &[16u8, 15, 14, 13, 12, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1] {
            let needed = u32::from(cand_w) + 1;
            if available < needed {
                continue;
            }
            let candidate = aligned >> (MAX_CODE_LEN - u32::from(cand_w));
            for page in pages {
                for &entry in page {
                    if entry.bits != cand_w || u32::from(entry.code) != candidate {
                        continue;
                    }
                    // Table B-14 FIRST/NEXT gating (Table B-15 has
                    // no NOTE 2/3 alternate, so its `(0, 1)` row is
                    // always legal).
                    if table == TableSelection::TableZero {
                        // `1s` (1-bit) is FIRST-only.
                        if entry.bits == 1 && position == CoefficientPosition::Next {
                            continue;
                        }
                        // `11s` (2-bit) is NEXT-only — only the `(0, 1)`
                        // entry. Other 2-bit entries (none exist in
                        // Table B-14) would not be filtered.
                        if entry.bits == 2
                            && entry.code == 0b11
                            && position == CoefficientPosition::First
                        {
                            continue;
                        }
                    }
                    // Consume codeword + sign bit.
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

        // (2) Escape (`000001`, 6 bits) — needs 6 prefix + 6 run +
        // 12 level = 24 bits total.
        if available >= ESCAPE_BITS + ESCAPE_PAYLOAD_BITS {
            let escape_candidate = aligned >> (MAX_CODE_LEN - ESCAPE_BITS);
            if escape_candidate == ESCAPE_CODE {
                return Self::parse_escape(br);
            }
        }
        // (3) `end_of_block` — table-dependent, NEXT-only.
        let (eob_code, eob_bits) = match table {
            TableSelection::TableZero => (EOB_B14_CODE, EOB_B14_BITS),
            TableSelection::TableOne => (EOB_B15_CODE, EOB_B15_BITS),
        };
        if position == CoefficientPosition::Next && available >= eob_bits {
            let eob_candidate = aligned >> (MAX_CODE_LEN - eob_bits);
            if eob_candidate == eob_code {
                br.consume(eob_bits).map_err(|_| Error::ShortHeader)?;
                return Ok(Self {
                    symbol: DctCoeff::EndOfBlock,
                    bit_position_after: br.bit_position(),
                });
            }
        }

        Err(Error::InvalidBitstream(
            "mpeg2_dct_coeff: no Table B-14 / B-15 / B-16 codeword matches the bit prefix (§7.2.2)",
        ))
    }

    /// Decode a Table B-16 escape payload with the cursor positioned at
    /// the escape prefix `000001`. Consumes the prefix, then the 6-bit
    /// run, then the 12-bit signed_level. The all-zeros wire word for
    /// the signed_level field is the spec's forbidden value.
    fn parse_escape(br: &mut BitReader<'_>) -> Result<Self> {
        br.consume(ESCAPE_BITS).map_err(|_| Error::ShortHeader)?;
        let run = br
            .read_u32(ESCAPE_RUN_BITS)
            .map_err(|_| Error::ShortHeader)? as u8;
        let level_word = br
            .read_u32(ESCAPE_LEVEL_BITS)
            .map_err(|_| Error::ShortHeader)?;
        // 12-bit two's complement; values 0x000..=0x7FF map to
        // +0..=+2047, and 0x800..=0xFFF map to -2048..=-1. The spec
        // forbids the wire word `0x000` (signed_level = 0).
        if level_word == 0 {
            return Err(Error::InvalidBitstream(
                "mpeg2_dct_coeff: Table B-16 signed_level = 0 is forbidden",
            ));
        }
        let signed_level: i16 = if level_word & 0x800 != 0 {
            // Negative — sign-extend the 12-bit two's complement.
            (level_word as i32 - 0x1000) as i16
        } else {
            level_word as i16
        };
        if run > MAX_RUN {
            // Defensive — the 6-bit field already bounds this, but the
            // assertion documents the spec range.
            return Err(Error::InvalidBitstream(
                "mpeg2_dct_coeff: Table B-16 run > 63 is impossible",
            ));
        }
        // The MPEG-2 spec range pegs the wire value as -2048..=+2047.
        // The wire word -2048 (`0x800`) would map to signed_level
        // -2048 which falls outside the §7.2.2.3 documented range
        // `-2047..=+2047`. The spec lists the wire entries explicitly:
        // `1000 0000 0000` → -2048 is not in the table — the table
        // goes from `1000 0000 0001` (-2047) up. Reject -2048.
        if signed_level < ESCAPE_SIGNED_LEVEL_MIN {
            return Err(Error::InvalidBitstream(
                "mpeg2_dct_coeff: Table B-16 signed_level wire word 0x800 (= -2048) is not listed",
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

// =============================================================
// Tests
// =============================================================

#[cfg(test)]
mod tests {
    //! Spec-pinned coverage of Annex B Tables B-14, B-15, and the
    //! Table B-16 escape — the FIRST / NEXT disambiguation,
    //! end-of-block recognition (table-dependent), and the §7.2.2.1
    //! Table 7-3 `(intra_vlc_format, macroblock_intra)` resolution.
    use super::*;
    use oxideav_core::bits::BitWriter;

    fn pad_and_finish(mut bw: BitWriter) -> Vec<u8> {
        for _ in 0..4 {
            bw.write_byte(0);
        }
        bw.finish()
    }

    fn all_b14() -> impl Iterator<Item = &'static CoeffEntry> {
        TABLE_B14_PAGE1
            .iter()
            .chain(TABLE_B14_PAGE2)
            .chain(TABLE_B14_PAGE3)
            .chain(TABLE_B14_PAGE4)
    }

    fn all_b15() -> impl Iterator<Item = &'static CoeffEntry> {
        TABLE_B15_PAGE1
            .iter()
            .chain(TABLE_B15_PAGE2)
            .chain(TABLE_B15_PAGE3)
            .chain(TABLE_B15_PAGE4)
    }

    // ----- §7.2.2.1 Table 7-3 selector -----

    #[test]
    fn table_selection_follows_spec_table_7_3() {
        // Per Table 7-3:
        // intra_vlc_format = 0, intra blocks       → B-14
        // intra_vlc_format = 0, non-intra blocks   → B-14
        // intra_vlc_format = 1, intra blocks       → B-15
        // intra_vlc_format = 1, non-intra blocks   → B-14
        assert_eq!(
            TableSelection::from_context(false, true),
            TableSelection::TableZero
        );
        assert_eq!(
            TableSelection::from_context(false, false),
            TableSelection::TableZero
        );
        assert_eq!(
            TableSelection::from_context(true, true),
            TableSelection::TableOne
        );
        assert_eq!(
            TableSelection::from_context(true, false),
            TableSelection::TableZero
        );
    }

    // ----- table-shape invariants -----

    #[test]
    fn table_b14_has_112_rows() {
        // 32 (page 1, including both the NOTE 2 `1s` FIRST-only row
        // and the NOTE 3 `11s` NEXT-only row for `(0, 1)`) + 32 (page
        // 2, 12-bit + 13-bit) + 32 (page 3, 14-bit + 15-bit) + 16
        // (page 4, 16-bit) = 112 rows total in the codeword table.
        // The §7.2.2.2 NOTE 2 / NOTE 3 distinction is the two
        // alternate (run=0, level=1) rows, both already counted here.
        assert_eq!(all_b14().count(), 112);
    }

    #[test]
    fn table_b15_has_111_rows() {
        // The page1 row count is 31 (no NOTE 2 / NOTE 3 alternate),
        // pages 2..=4 are 32 + 32 + 16. Total 31 + 32 + 32 + 16 = 111.
        assert_eq!(all_b15().count(), 111);
    }

    #[test]
    fn every_b14_code_fits_its_declared_width() {
        for e in all_b14() {
            assert!(u32::from(e.bits) <= 16);
            let max = 1u32 << u32::from(e.bits);
            assert!(u32::from(e.code) < max);
        }
    }

    #[test]
    fn every_b15_code_fits_its_declared_width() {
        for e in all_b15() {
            assert!(u32::from(e.bits) <= 16);
            let max = 1u32 << u32::from(e.bits);
            assert!(u32::from(e.code) < max);
        }
    }

    #[test]
    fn b14_codes_unique_within_each_width() {
        for w in 1u8..=16 {
            let group: Vec<_> = all_b14().filter(|e| e.bits == w).collect();
            for (i, a) in group.iter().enumerate() {
                for b in &group[i + 1..] {
                    assert_ne!(
                        a.code, b.code,
                        "B-14 dup at width {w}: (run {} lvl {}) vs (run {} lvl {})",
                        a.run, a.level, b.run, b.level
                    );
                }
            }
        }
    }

    #[test]
    fn b15_codes_unique_within_each_width() {
        for w in 1u8..=16 {
            let group: Vec<_> = all_b15().filter(|e| e.bits == w).collect();
            for (i, a) in group.iter().enumerate() {
                for b in &group[i + 1..] {
                    assert_ne!(
                        a.code, b.code,
                        "B-15 dup at width {w}: (run {} lvl {}) vs (run {} lvl {})",
                        a.run, a.level, b.run, b.level
                    );
                }
            }
        }
    }

    /// For each codeword + sign, neither codeword should be a prefix
    /// of the other (when EoB / escape are also included).
    fn assert_prefix_free<'a, I>(
        entries: I,
        eob_code: u32,
        eob_bits: u32,
        include_first_only_1bit: bool,
        include_next_only_2bit: bool,
    ) where
        I: Iterator<Item = &'a CoeffEntry>,
    {
        let mut items: Vec<(u32, u32)> = Vec::new(); // (bits, aligned)
        for e in entries {
            // Skip the FIRST-only `1s` row when checking NEXT, and
            // skip the NEXT-only `11s` row when checking FIRST.
            if e.bits == 1 && !include_first_only_1bit {
                continue;
            }
            if e.bits == 2 && e.code == 0b11 && !include_next_only_2bit {
                continue;
            }
            // codeword bits (no sign included for prefix check; both
            // signs are legal on the trailing bit so they're not a
            // disambiguator).
            let cw_bits = u32::from(e.bits);
            let aligned = u32::from(e.code) << (32 - cw_bits);
            items.push((cw_bits, aligned));
        }
        // Escape (no sign).
        items.push((ESCAPE_BITS, ESCAPE_CODE << (32 - ESCAPE_BITS)));
        // EoB (no sign) — only on NEXT.
        if !include_first_only_1bit {
            items.push((eob_bits, eob_code << (32 - eob_bits)));
        }

        for (i, &(bits_a, al_a)) in items.iter().enumerate() {
            for &(bits_b, al_b) in &items[i + 1..] {
                let min = bits_a.min(bits_b);
                let mask = if min == 32 {
                    u32::MAX
                } else {
                    !((1u32 << (32 - min)) - 1)
                };
                assert_ne!(
                    al_a & mask,
                    al_b & mask,
                    "prefix collision: {:032b} ({} bits) vs {:032b} ({} bits)",
                    al_a,
                    bits_a,
                    al_b,
                    bits_b
                );
            }
        }
    }

    #[test]
    fn b14_codebook_is_prefix_free_at_first_and_next() {
        // FIRST excludes `11s`; NEXT excludes `1s` and includes EoB.
        assert_prefix_free(all_b14(), EOB_B14_CODE, EOB_B14_BITS, true, false);
        assert_prefix_free(all_b14(), EOB_B14_CODE, EOB_B14_BITS, false, true);
    }

    #[test]
    fn b15_codebook_is_prefix_free_at_next() {
        // B-15 has no NOTE 2 / NOTE 3 alternate; only NEXT semantics.
        // Include neither the `1s` nor the `11s` filter (B-15 has no
        // 1-bit code at all and its 2-bit `10` is the legitimate
        // (run=0, level=1) row).
        assert_prefix_free(all_b15(), EOB_B15_CODE, EOB_B15_BITS, true, true);
    }

    // ----- per-table round-trips -----

    fn write_entry(bw: &mut BitWriter, e: &CoeffEntry, negative: bool) {
        bw.write_u32(u32::from(e.code), u32::from(e.bits));
        bw.write_bit(negative);
    }

    #[test]
    fn every_b14_row_round_trips_at_next() {
        for entry in all_b14() {
            if entry.bits == 1 {
                // FIRST-only — handled in its own test.
                continue;
            }
            for negative in [false, true] {
                let mut bw = BitWriter::new();
                write_entry(&mut bw, entry, negative);
                let buf = pad_and_finish(bw);
                let mut br = BitReader::new(&buf);
                let step = DctCoeffStep::parse(
                    &mut br,
                    TableSelection::TableZero,
                    CoefficientPosition::Next,
                )
                .unwrap_or_else(|e| {
                    panic!(
                        "decode failed for B-14 entry width={} code=0x{:x} run={} level={} neg={}: {:?}",
                        entry.bits, entry.code, entry.run, entry.level, negative, e
                    )
                });
                match step.symbol {
                    DctCoeff::RunLevel {
                        run,
                        signed_level,
                        escape,
                    } => {
                        assert_eq!(run, entry.run);
                        let expected = if negative {
                            -i16::from(entry.level)
                        } else {
                            i16::from(entry.level)
                        };
                        assert_eq!(signed_level, expected);
                        assert!(!escape);
                    }
                    _ => panic!("expected RunLevel for ({}, {})", entry.run, entry.level),
                }
            }
        }
    }

    #[test]
    fn every_b15_row_round_trips_at_next() {
        for entry in all_b15() {
            for negative in [false, true] {
                let mut bw = BitWriter::new();
                write_entry(&mut bw, entry, negative);
                let buf = pad_and_finish(bw);
                let mut br = BitReader::new(&buf);
                let step = DctCoeffStep::parse(
                    &mut br,
                    TableSelection::TableOne,
                    CoefficientPosition::Next,
                )
                .unwrap_or_else(|e| {
                    panic!(
                        "decode failed for B-15 entry width={} code=0x{:x} run={} level={} neg={}: {:?}",
                        entry.bits, entry.code, entry.run, entry.level, negative, e
                    )
                });
                match step.symbol {
                    DctCoeff::RunLevel {
                        run,
                        signed_level,
                        escape,
                    } => {
                        assert_eq!(run, entry.run);
                        let expected = if negative {
                            -i16::from(entry.level)
                        } else {
                            i16::from(entry.level)
                        };
                        assert_eq!(signed_level, expected);
                        assert!(!escape);
                    }
                    _ => panic!("expected RunLevel for ({}, {})", entry.run, entry.level),
                }
            }
        }
    }

    #[test]
    fn b14_first_only_1s_form_legal_at_first() {
        // `1s` for FIRST: (run=0, level=±1).
        for negative in [false, true] {
            let mut bw = BitWriter::new();
            bw.write_u32(0b1, 1);
            bw.write_bit(negative);
            let buf = pad_and_finish(bw);
            let mut br = BitReader::new(&buf);
            let step = DctCoeffStep::parse(
                &mut br,
                TableSelection::TableZero,
                CoefficientPosition::First,
            )
            .unwrap();
            match step.symbol {
                DctCoeff::RunLevel {
                    run, signed_level, ..
                } => {
                    assert_eq!(run, 0);
                    assert_eq!(signed_level, if negative { -1 } else { 1 });
                }
                _ => panic!("expected RunLevel"),
            }
        }
    }

    #[test]
    fn b14_next_only_11s_form_legal_at_next_not_first() {
        // `11s`. At NEXT it decodes to (0, ±1). At FIRST it is
        // forbidden — the walker must instead match the `1s` 1-bit
        // form and the trailing `1` becomes the sign bit, producing
        // (0, -1) instead.
        for negative in [false, true] {
            let mut bw = BitWriter::new();
            bw.write_u32(0b11, 2);
            bw.write_bit(negative);
            let buf = pad_and_finish(bw);
            // NEXT — `11s` → (0, ±1).
            let mut br = BitReader::new(&buf);
            let step = DctCoeffStep::parse(
                &mut br,
                TableSelection::TableZero,
                CoefficientPosition::Next,
            )
            .unwrap();
            match step.symbol {
                DctCoeff::RunLevel {
                    run, signed_level, ..
                } => {
                    assert_eq!(run, 0);
                    assert_eq!(signed_level, if negative { -1 } else { 1 });
                }
                _ => panic!("expected RunLevel"),
            }
        }
    }

    #[test]
    fn b14_eob_at_next() {
        let mut bw = BitWriter::new();
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let step = DctCoeffStep::parse(
            &mut br,
            TableSelection::TableZero,
            CoefficientPosition::Next,
        )
        .unwrap();
        assert!(matches!(step.symbol, DctCoeff::EndOfBlock));
        assert_eq!(step.bit_position_after, u64::from(EOB_B14_BITS));
    }

    #[test]
    fn b15_eob_at_next() {
        let mut bw = BitWriter::new();
        bw.write_u32(EOB_B15_CODE, EOB_B15_BITS);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let step =
            DctCoeffStep::parse(&mut br, TableSelection::TableOne, CoefficientPosition::Next)
                .unwrap();
        assert!(matches!(step.symbol, DctCoeff::EndOfBlock));
        assert_eq!(step.bit_position_after, u64::from(EOB_B15_BITS));
    }

    // ----- Table B-16 escape -----

    fn write_escape(bw: &mut BitWriter, run: u8, signed_level: i16) {
        // 6-bit escape prefix `000001`.
        bw.write_u32(ESCAPE_CODE, ESCAPE_BITS);
        // 6-bit run.
        bw.write_u32(u32::from(run), ESCAPE_RUN_BITS);
        // 12-bit signed_level (two's complement).
        let word = if signed_level < 0 {
            (signed_level as i32 + 0x1000) as u32
        } else {
            signed_level as u32
        };
        bw.write_u32(word & 0xFFF, ESCAPE_LEVEL_BITS);
    }

    #[test]
    fn b16_escape_round_trip_positive() {
        let cases: &[(u8, i16)] = &[(0, 1), (5, 128), (10, 2047), (63, 100), (12, 500)];
        for &(run, level) in cases {
            let mut bw = BitWriter::new();
            write_escape(&mut bw, run, level);
            let buf = pad_and_finish(bw);
            let mut br = BitReader::new(&buf);
            let step = DctCoeffStep::parse(
                &mut br,
                TableSelection::TableZero,
                CoefficientPosition::Next,
            )
            .unwrap();
            match step.symbol {
                DctCoeff::RunLevel {
                    run: r,
                    signed_level,
                    escape,
                } => {
                    assert_eq!(r, run);
                    assert_eq!(signed_level, level);
                    assert!(escape);
                }
                _ => panic!("expected RunLevel escape"),
            }
        }
    }

    #[test]
    fn b16_escape_round_trip_negative() {
        let cases: &[(u8, i16)] = &[(0, -1), (7, -128), (12, -500), (33, -2047), (63, -64)];
        for &(run, level) in cases {
            let mut bw = BitWriter::new();
            write_escape(&mut bw, run, level);
            let buf = pad_and_finish(bw);
            let mut br = BitReader::new(&buf);
            let step = DctCoeffStep::parse(
                &mut br,
                TableSelection::TableZero,
                CoefficientPosition::Next,
            )
            .unwrap();
            match step.symbol {
                DctCoeff::RunLevel {
                    run: r,
                    signed_level,
                    escape,
                } => {
                    assert_eq!(r, run);
                    assert_eq!(signed_level, level);
                    assert!(escape);
                }
                _ => panic!("expected RunLevel escape"),
            }
        }
    }

    #[test]
    fn b16_escape_signed_level_zero_is_forbidden() {
        let mut bw = BitWriter::new();
        bw.write_u32(ESCAPE_CODE, ESCAPE_BITS);
        bw.write_u32(0, ESCAPE_RUN_BITS);
        bw.write_u32(0, ESCAPE_LEVEL_BITS);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let err = DctCoeffStep::parse(
            &mut br,
            TableSelection::TableZero,
            CoefficientPosition::Next,
        )
        .unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn b16_escape_signed_level_neg2048_is_forbidden() {
        // Wire word 0x800 = -2048. Spec lists the table from -2047
        // (`1000 0000 0001`) up. -2048 is not listed.
        let mut bw = BitWriter::new();
        bw.write_u32(ESCAPE_CODE, ESCAPE_BITS);
        bw.write_u32(0, ESCAPE_RUN_BITS);
        bw.write_u32(0x800, ESCAPE_LEVEL_BITS);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let err = DctCoeffStep::parse(
            &mut br,
            TableSelection::TableZero,
            CoefficientPosition::Next,
        )
        .unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn b16_escape_extremes_decode() {
        // Wire word 0x801 = -2047 — the most negative listed.
        let mut bw = BitWriter::new();
        bw.write_u32(ESCAPE_CODE, ESCAPE_BITS);
        bw.write_u32(63, ESCAPE_RUN_BITS);
        bw.write_u32(0x801, ESCAPE_LEVEL_BITS);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let step = DctCoeffStep::parse(
            &mut br,
            TableSelection::TableZero,
            CoefficientPosition::Next,
        )
        .unwrap();
        match step.symbol {
            DctCoeff::RunLevel {
                run,
                signed_level,
                escape,
            } => {
                assert_eq!(run, 63);
                assert_eq!(signed_level, -2047);
                assert!(escape);
            }
            _ => panic!(),
        }

        // Wire word 0x7FF = +2047 — the most positive listed.
        let mut bw = BitWriter::new();
        bw.write_u32(ESCAPE_CODE, ESCAPE_BITS);
        bw.write_u32(0, ESCAPE_RUN_BITS);
        bw.write_u32(0x7FF, ESCAPE_LEVEL_BITS);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let step = DctCoeffStep::parse(
            &mut br,
            TableSelection::TableZero,
            CoefficientPosition::Next,
        )
        .unwrap();
        match step.symbol {
            DctCoeff::RunLevel {
                run,
                signed_level,
                escape,
            } => {
                assert_eq!(run, 0);
                assert_eq!(signed_level, 2047);
                assert!(escape);
            }
            _ => panic!(),
        }
    }

    // ----- end-to-end block walk -----

    #[test]
    fn b14_full_block_walk() {
        // Emit a small non-intra block:
        //   FIRST: (run=0, level=+3) via `0010 1 s` = `00101 0`
        //   NEXT:  (run=2, level=-1) via `0101 s`  = `0101 1`
        //   NEXT:  escape (run=4, level=+1500)
        //   NEXT:  EoB                              = `10`
        let mut bw = BitWriter::new();
        bw.write_u32(0b0010_1, 5);
        bw.write_bit(false);
        bw.write_u32(0b0101, 4);
        bw.write_bit(true);
        write_escape(&mut bw, 4, 1500);
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        let buf = pad_and_finish(bw);

        let mut br = BitReader::new(&buf);
        // FIRST.
        let s0 = DctCoeffStep::parse(
            &mut br,
            TableSelection::TableZero,
            CoefficientPosition::First,
        )
        .unwrap();
        match s0.symbol {
            DctCoeff::RunLevel {
                run,
                signed_level,
                escape,
            } => {
                assert_eq!(run, 0);
                assert_eq!(signed_level, 3);
                assert!(!escape);
            }
            _ => panic!(),
        }
        // NEXT (2, -1).
        let s1 = DctCoeffStep::parse(
            &mut br,
            TableSelection::TableZero,
            CoefficientPosition::Next,
        )
        .unwrap();
        match s1.symbol {
            DctCoeff::RunLevel {
                run,
                signed_level,
                escape,
            } => {
                assert_eq!(run, 2);
                assert_eq!(signed_level, -1);
                assert!(!escape);
            }
            _ => panic!(),
        }
        // NEXT escape.
        let s2 = DctCoeffStep::parse(
            &mut br,
            TableSelection::TableZero,
            CoefficientPosition::Next,
        )
        .unwrap();
        match s2.symbol {
            DctCoeff::RunLevel {
                run,
                signed_level,
                escape,
            } => {
                assert_eq!(run, 4);
                assert_eq!(signed_level, 1500);
                assert!(escape);
            }
            _ => panic!(),
        }
        // NEXT EoB.
        let s3 = DctCoeffStep::parse(
            &mut br,
            TableSelection::TableZero,
            CoefficientPosition::Next,
        )
        .unwrap();
        assert!(matches!(s3.symbol, DctCoeff::EndOfBlock));
    }

    #[test]
    fn b15_intra_block_walk() {
        // Emit an intra block (B-15, NEXT-only after DC):
        //   NEXT: (run=0, level=+1) via `10 s`     = `10 0`
        //   NEXT: (run=0, level=+2) via `110 s`    = `110 0`
        //   NEXT: escape (run=20, level=-1234)
        //   NEXT: EoB `0110`.
        let mut bw = BitWriter::new();
        bw.write_u32(0b10, 2);
        bw.write_bit(false);
        bw.write_u32(0b110, 3);
        bw.write_bit(false);
        write_escape(&mut bw, 20, -1234);
        bw.write_u32(EOB_B15_CODE, EOB_B15_BITS);
        let buf = pad_and_finish(bw);

        let mut br = BitReader::new(&buf);
        let s0 = DctCoeffStep::parse(&mut br, TableSelection::TableOne, CoefficientPosition::Next)
            .unwrap();
        match s0.symbol {
            DctCoeff::RunLevel {
                run, signed_level, ..
            } => {
                assert_eq!(run, 0);
                assert_eq!(signed_level, 1);
            }
            _ => panic!(),
        }
        let s1 = DctCoeffStep::parse(&mut br, TableSelection::TableOne, CoefficientPosition::Next)
            .unwrap();
        match s1.symbol {
            DctCoeff::RunLevel {
                run, signed_level, ..
            } => {
                assert_eq!(run, 0);
                assert_eq!(signed_level, 2);
            }
            _ => panic!(),
        }
        let s2 = DctCoeffStep::parse(&mut br, TableSelection::TableOne, CoefficientPosition::Next)
            .unwrap();
        match s2.symbol {
            DctCoeff::RunLevel {
                run,
                signed_level,
                escape,
            } => {
                assert_eq!(run, 20);
                assert_eq!(signed_level, -1234);
                assert!(escape);
            }
            _ => panic!(),
        }
        let s3 = DctCoeffStep::parse(&mut br, TableSelection::TableOne, CoefficientPosition::Next)
            .unwrap();
        assert!(matches!(s3.symbol, DctCoeff::EndOfBlock));
    }

    // ----- error paths -----

    #[test]
    fn truncated_buffer_returns_short_header() {
        // A single 0 bit isn't a complete codeword (the longest legal
        // prefix is `1s` / `10`, both 2 bits min). All buffers <= 1
        // bit must error.
        let buf: &[u8] = &[];
        let mut br = BitReader::new(buf);
        let err = DctCoeffStep::parse(
            &mut br,
            TableSelection::TableZero,
            CoefficientPosition::Next,
        )
        .unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    #[test]
    fn unrecognised_prefix_returns_invalid_bitstream() {
        // 17 zero bits — none of the table codes nor the escape can
        // start with that many zeros (the longest-prefix-of-zero is
        // the 16-bit codeword `0000_0000_0001_xxxx s` whose 12 leading
        // zeros are followed by a `1`, so 16 leading zeros violates
        // every entry).
        let buf: &[u8] = &[0x00, 0x00, 0x00];
        let mut br = BitReader::new(buf);
        let err = DctCoeffStep::parse(
            &mut br,
            TableSelection::TableZero,
            CoefficientPosition::Next,
        )
        .unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }
}
