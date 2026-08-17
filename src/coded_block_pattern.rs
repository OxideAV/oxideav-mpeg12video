//! Parser for `coded_block_pattern()` per ISO/IEC 13818-2
//! (Recommendation ITU-T H.262) §6.2.5.3, with the field semantics
//! from §6.3.17.4 and the Annex B Table B-9 variable-length codes.
//!
//! Round 8 advances the macroblock loop one syntax element past
//! round 7's `macroblock_type`. `coded_block_pattern()` is present in
//! the bitstream exactly when `macroblock_pattern` (derived from
//! `macroblock_type`, §6.3.17.1) is `1`. It tells the decoder which of
//! the macroblock's blocks carry coded transform coefficients.
//!
//! The syntax (§6.2.5.3) is:
//!
//! ```text
//! coded_block_pattern() {
//!     coded_block_pattern_420                    3-9 bits  vlclbf  (Table B-9)
//!     if (chroma_format == 4:2:2)
//!         coded_block_pattern_1                  2 bits    uimsbf
//!     if (chroma_format == 4:4:4)
//!         coded_block_pattern_2                  6 bits    uimsbf
//! }
//! ```
//!
//! `coded_block_pattern_420` is a VLC decoded against Table B-9 into a
//! 6-bit value `cbp` (0..=63). For 4:2:2 and 4:4:4 chroma the pattern
//! is extended by a fixed-length `coded_block_pattern_1` (2 bits) or
//! `coded_block_pattern_2` (6 bits) respectively.
//!
//! §6.3.17.4 derives the 12-entry `pattern_code[i]` array used by the
//! block loop:
//!
//! ```text
//! for (i = 0; i < 12; i++)
//!     pattern_code[i] = macroblock_intra ? 1 : 0;
//! if (macroblock_pattern) {
//!     for (i = 0; i < 6; i++)
//!         if (cbp & (1 << (5 - i))) pattern_code[i] = 1;
//!     if (chroma_format == 4:2:2)
//!         for (i = 6; i < 8; i++)
//!             if (coded_block_pattern_1 & (1 << (7 - i))) pattern_code[i] = 1;
//!     if (chroma_format == 4:4:4)
//!         for (i = 8; i < 12; i++)
//!             if (coded_block_pattern_2 & (1 << (11 - i))) pattern_code[i] = 1;
//! }
//! ```
//!
//! This module decodes the wire fields (`cbp`,
//! `coded_block_pattern_1`, `coded_block_pattern_2`) and exposes the
//! §6.3.17.4 derivation through [`CodedBlockPattern::pattern_code`].
//! The block loop itself (`block()`, the DCT coefficient VLCs, and the
//! IDCT) is out of scope for round 8.
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)) §6.2.5.3, §6.3.17.4, and
//! Annex B Table B-9.

// The Annex B Table B-9 codewords are kept in the spec's MSB-first
// visual grouping (e.g. `0b0010_111` for the 7-bit `cbp = 5` code) so
// an audit can read each constant straight against the printed table.
// clippy's `unusual_byte_groupings` lint would prefer uniform 4-bit
// groups, which would obscure that mapping.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitReader;

use crate::sequence_extension::ChromaFormat;
use crate::{Error, Result};

/// The decoded `coded_block_pattern()` fields per §6.2.5.3.
///
/// `cbp` is the 6-bit value derived from `coded_block_pattern_420`
/// (Table B-9). The optional `coded_block_pattern_1` /
/// `coded_block_pattern_2` extensions are `Some` only for 4:2:2 /
/// 4:4:4 chroma respectively (and `None` otherwise).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CodedBlockPattern {
    /// The 6-bit `cbp` (0..=63) decoded from `coded_block_pattern_420`
    /// via Table B-9.
    pub cbp: u8,
    /// `coded_block_pattern_1` (2-bit FLC), present only for 4:2:2.
    pub coded_block_pattern_1: Option<u8>,
    /// `coded_block_pattern_2` (6-bit FLC), present only for 4:4:4.
    pub coded_block_pattern_2: Option<u8>,
    /// Bit position (relative to the start of the buffer the
    /// [`BitReader`] was created from) right after the consumed
    /// `coded_block_pattern()`. Lets callers chain into the block loop
    /// without losing the partial-byte cursor.
    pub bit_position_after: u64,
}

/// One Table B-9 row: a right-justified MSB-first VLC code, its bit
/// length, and the resulting 6-bit `cbp`.
#[derive(Clone, Copy)]
struct Row {
    /// VLC code right-justified into a `u16` — e.g. `0010 111` becomes
    /// `0b0_0010_111` with `bits == 7`.
    code: u16,
    /// Length of `code` in bits (`3..=9` across Table B-9).
    bits: u8,
    /// The `cbp` value (0..=63) this codeword decodes to.
    cbp: u8,
}

/// Table B-9 — variable length codes for `coded_block_pattern`.
///
/// All 64 `cbp` values (0..=63) are listed, transcribed verbatim from
/// the spec's two-column layout (left column then right column). The
/// `cbp = 0` entry (`0000 0000 1`) carries the spec NOTE that it shall
/// not be used with 4:2:0 chrominance; the syntax still defines its
/// codeword, so the walker accepts it — higher-layer 4:2:0 conformance
/// checking is left to the caller.
const TABLE_B9: &[Row] = &[
    // Left column (top to bottom).
    Row {
        code: 0b111,
        bits: 3,
        cbp: 60,
    },
    Row {
        code: 0b1101,
        bits: 4,
        cbp: 4,
    },
    Row {
        code: 0b1100,
        bits: 4,
        cbp: 8,
    },
    Row {
        code: 0b1011,
        bits: 4,
        cbp: 16,
    },
    Row {
        code: 0b1010,
        bits: 4,
        cbp: 32,
    },
    Row {
        code: 0b1001_1,
        bits: 5,
        cbp: 12,
    },
    Row {
        code: 0b1001_0,
        bits: 5,
        cbp: 48,
    },
    Row {
        code: 0b1000_1,
        bits: 5,
        cbp: 20,
    },
    Row {
        code: 0b1000_0,
        bits: 5,
        cbp: 40,
    },
    Row {
        code: 0b0111_1,
        bits: 5,
        cbp: 28,
    },
    Row {
        code: 0b0111_0,
        bits: 5,
        cbp: 44,
    },
    Row {
        code: 0b0110_1,
        bits: 5,
        cbp: 52,
    },
    Row {
        code: 0b0110_0,
        bits: 5,
        cbp: 56,
    },
    Row {
        code: 0b0101_1,
        bits: 5,
        cbp: 1,
    },
    Row {
        code: 0b0101_0,
        bits: 5,
        cbp: 61,
    },
    Row {
        code: 0b0100_1,
        bits: 5,
        cbp: 2,
    },
    Row {
        code: 0b0100_0,
        bits: 5,
        cbp: 62,
    },
    Row {
        code: 0b0011_11,
        bits: 6,
        cbp: 24,
    },
    Row {
        code: 0b0011_10,
        bits: 6,
        cbp: 36,
    },
    Row {
        code: 0b0011_01,
        bits: 6,
        cbp: 3,
    },
    Row {
        code: 0b0011_00,
        bits: 6,
        cbp: 63,
    },
    Row {
        code: 0b0010_111,
        bits: 7,
        cbp: 5,
    },
    Row {
        code: 0b0010_110,
        bits: 7,
        cbp: 9,
    },
    Row {
        code: 0b0010_101,
        bits: 7,
        cbp: 17,
    },
    Row {
        code: 0b0010_100,
        bits: 7,
        cbp: 33,
    },
    Row {
        code: 0b0010_011,
        bits: 7,
        cbp: 6,
    },
    Row {
        code: 0b0010_010,
        bits: 7,
        cbp: 10,
    },
    Row {
        code: 0b0010_001,
        bits: 7,
        cbp: 18,
    },
    Row {
        code: 0b0010_000,
        bits: 7,
        cbp: 34,
    },
    Row {
        code: 0b0001_1111,
        bits: 8,
        cbp: 7,
    },
    Row {
        code: 0b0001_1110,
        bits: 8,
        cbp: 11,
    },
    Row {
        code: 0b0001_1101,
        bits: 8,
        cbp: 19,
    },
    // Right column (top to bottom).
    Row {
        code: 0b0001_1100,
        bits: 8,
        cbp: 35,
    },
    Row {
        code: 0b0001_1011,
        bits: 8,
        cbp: 13,
    },
    Row {
        code: 0b0001_1010,
        bits: 8,
        cbp: 49,
    },
    Row {
        code: 0b0001_1001,
        bits: 8,
        cbp: 21,
    },
    Row {
        code: 0b0001_1000,
        bits: 8,
        cbp: 41,
    },
    Row {
        code: 0b0001_0111,
        bits: 8,
        cbp: 14,
    },
    Row {
        code: 0b0001_0110,
        bits: 8,
        cbp: 50,
    },
    Row {
        code: 0b0001_0101,
        bits: 8,
        cbp: 22,
    },
    Row {
        code: 0b0001_0100,
        bits: 8,
        cbp: 42,
    },
    Row {
        code: 0b0001_0011,
        bits: 8,
        cbp: 15,
    },
    Row {
        code: 0b0001_0010,
        bits: 8,
        cbp: 51,
    },
    Row {
        code: 0b0001_0001,
        bits: 8,
        cbp: 23,
    },
    Row {
        code: 0b0001_0000,
        bits: 8,
        cbp: 43,
    },
    Row {
        code: 0b0000_1111,
        bits: 8,
        cbp: 25,
    },
    Row {
        code: 0b0000_1110,
        bits: 8,
        cbp: 37,
    },
    Row {
        code: 0b0000_1101,
        bits: 8,
        cbp: 26,
    },
    Row {
        code: 0b0000_1100,
        bits: 8,
        cbp: 38,
    },
    Row {
        code: 0b0000_1011,
        bits: 8,
        cbp: 29,
    },
    Row {
        code: 0b0000_1010,
        bits: 8,
        cbp: 45,
    },
    Row {
        code: 0b0000_1001,
        bits: 8,
        cbp: 53,
    },
    Row {
        code: 0b0000_1000,
        bits: 8,
        cbp: 57,
    },
    Row {
        code: 0b0000_0111,
        bits: 8,
        cbp: 30,
    },
    Row {
        code: 0b0000_0110,
        bits: 8,
        cbp: 46,
    },
    Row {
        code: 0b0000_0101,
        bits: 8,
        cbp: 54,
    },
    Row {
        code: 0b0000_0100,
        bits: 8,
        cbp: 58,
    },
    Row {
        code: 0b0000_0011_1,
        bits: 9,
        cbp: 31,
    },
    Row {
        code: 0b0000_0011_0,
        bits: 9,
        cbp: 47,
    },
    Row {
        code: 0b0000_0010_1,
        bits: 9,
        cbp: 55,
    },
    Row {
        code: 0b0000_0010_0,
        bits: 9,
        cbp: 59,
    },
    Row {
        code: 0b0000_0001_1,
        bits: 9,
        cbp: 27,
    },
    Row {
        code: 0b0000_0001_0,
        bits: 9,
        cbp: 39,
    },
    Row {
        code: 0b0000_0000_1,
        bits: 9,
        cbp: 0,
    },
];

/// Walk Table B-9 longest-first so a shorter codeword can never be
/// matched on the leading bits of a longer one.
fn match_cbp(br: &mut BitReader<'_>) -> Result<Row> {
    // Distinct code widths across Table B-9, descending. Walking
    // longest-first guarantees an exact full-width equality match.
    for &width in &[9u8, 8, 7, 6, 5, 4, 3] {
        if br.bits_remaining() < u64::from(width) {
            continue;
        }
        let peeked = br
            .peek_u32(u32::from(width))
            .map_err(|_| Error::ShortHeader)? as u16;
        for &row in TABLE_B9.iter().filter(|r| r.bits == width) {
            if row.code == peeked {
                br.consume(u32::from(width))
                    .map_err(|_| Error::ShortHeader)?;
                return Ok(row);
            }
        }
    }
    Err(Error::InvalidBitstream(
        "coded_block_pattern: no Table B-9 codeword matches the bit prefix (§6.2.5.3)",
    ))
}

/// Emit the §6.2.5.3 `coded_block_pattern_420` Table B-9 VLC for the
/// 6-bit `cbp` value (bit `5 - i` set means block `i` is coded). The
/// 4:2:2 / 4:4:4 extension fields are the caller's responsibility.
///
/// # Panics
/// Panics if `cbp` has no Table B-9 codeword (only `cbp == 0` is
/// unlisted — a macroblock with no coded blocks is signalled via
/// `macroblock_pattern == 0`, not an all-zero cbp).
pub fn encode_cbp420(bw: &mut oxideav_core::bits::BitWriter, cbp: u8) {
    let row = TABLE_B9
        .iter()
        .find(|r| r.cbp == cbp)
        .expect("cbp must have a Table B-9 codeword (cbp != 0)");
    bw.write_u32(u32::from(row.code), u32::from(row.bits));
}

/// Emit a whole §6.2.5.3 `coded_block_pattern()` for a **non-intra**
/// macroblock with `macroblock_pattern == 1`: the Table B-9
/// `coded_block_pattern_420` VLC for blocks `0..=5`, then the 4:2:2
/// two-bit `coded_block_pattern_1` (blocks 6..=7, mask bit `7 - i`) or
/// the 4:4:4 six-bit `coded_block_pattern_2` per §6.3.17.4.
///
/// `coded[i]` says whether block `i` (Figures 6-10/6-11/6-12 numbering)
/// carries coded coefficients; its length must equal the chroma
/// format's block count (6 / 8 / 12).
///
/// Errors:
/// * an all-clear `coded` — a macroblock with no coded block is
///   signalled through `macroblock_pattern == 0`, never through an
///   empty pattern (and the `cbp == 0` codeword carries the Table B-9
///   NOTE "shall not be used with 4:2:0 chrominance structure");
/// * a 4:4:4 pattern with block 6 or 7 set: the printed §6.3.17.4
///   derivation drives only `pattern_code[8..12]` from
///   `coded_block_pattern_2` (mask bits `11 - i`, i.e. bits 3..0),
///   leaving non-intra blocks 6 and 7 with no wire representation, so
///   the encoder refuses to emit a pattern it cannot signal (callers
///   drop those residuals or code the macroblock intra).
pub fn encode_coded_block_pattern(
    bw: &mut oxideav_core::bits::BitWriter,
    coded: &[bool],
    chroma_format: ChromaFormat,
) -> crate::Result<()> {
    let nblocks = crate::mpeg2_macroblock_blocks::block_count(chroma_format);
    debug_assert_eq!(coded.len(), nblocks, "coded[] must cover every block");
    if coded.iter().all(|&b| !b) {
        return Err(Error::InvalidBitstream(
            "coded_block_pattern: empty pattern — signal macroblock_pattern = 0 instead (§6.3.17.4)",
        ));
    }
    let mut cbp = 0u8;
    for (i, &c) in coded.iter().enumerate().take(6) {
        if c {
            cbp |= 1 << (5 - i);
        }
    }
    match chroma_format {
        ChromaFormat::Yuv420 => {
            if cbp == 0 {
                return Err(Error::InvalidBitstream(
                    "coded_block_pattern: cbp 0 shall not be used with 4:2:0 (Table B-9 NOTE)",
                ));
            }
            encode_cbp420(bw, cbp);
        }
        ChromaFormat::Yuv422 => {
            // §6.3.17.4: pattern_code[i] for i in 6..8 reads
            // coded_block_pattern_1 bit (7 - i) — block 6 → bit 1,
            // block 7 → bit 0.
            let cbp1 = (u32::from(coded[6]) << 1) | u32::from(coded[7]);
            encode_cbp420(bw, cbp);
            bw.write_u32(cbp1, 2);
        }
        ChromaFormat::Yuv444 => {
            if coded[6] || coded[7] {
                return Err(Error::InvalidBitstream(
                    "coded_block_pattern: 4:4:4 blocks 6/7 have no non-intra wire representation (§6.3.17.4)",
                ));
            }
            // §6.3.17.4: pattern_code[i] for i in 8..12 reads
            // coded_block_pattern_2 bit (11 - i) — bits 3..0; bits 5
            // and 4 of the six-bit field select nothing and stay 0.
            let mut cbp2 = 0u32;
            for (i, &c) in coded.iter().enumerate().take(12).skip(8) {
                if c {
                    cbp2 |= 1 << (11 - i);
                }
            }
            encode_cbp420(bw, cbp);
            bw.write_u32(cbp2, 6);
        }
    }
    Ok(())
}

impl CodedBlockPattern {
    /// Parse one `coded_block_pattern()` starting at the current
    /// position of `br`. `chroma_format` (from
    /// `sequence_extension()`) selects whether the 4:2:2 / 4:4:4
    /// fixed-length extensions follow the `coded_block_pattern_420`
    /// VLC. Consumes from `br` on success.
    ///
    /// Errors:
    /// * [`Error::InvalidBitstream`] if no Table B-9 codeword matches
    ///   the upcoming bits.
    /// * [`Error::ShortHeader`] if the bitstream ends before the VLC
    ///   (or a required FLC extension) could be read.
    pub fn parse(br: &mut BitReader<'_>, chroma_format: ChromaFormat) -> Result<Self> {
        let row = match_cbp(br)?;

        let coded_block_pattern_1 = if chroma_format == ChromaFormat::Yuv422 {
            Some(br.read_u32(2).map_err(|_| Error::ShortHeader)? as u8)
        } else {
            None
        };
        let coded_block_pattern_2 = if chroma_format == ChromaFormat::Yuv444 {
            Some(br.read_u32(6).map_err(|_| Error::ShortHeader)? as u8)
        } else {
            None
        };

        Ok(Self {
            cbp: row.cbp,
            coded_block_pattern_1,
            coded_block_pattern_2,
            bit_position_after: br.bit_position(),
        })
    }

    /// Derive the 12-entry `pattern_code[i]` array per §6.3.17.4.
    ///
    /// `macroblock_intra` and `macroblock_pattern` come from the
    /// macroblock's `macroblock_type` ([`crate::MacroblockType`]).
    ///
    /// * For an intra macroblock every entry starts at `1`.
    /// * When `macroblock_pattern` is set, the low six entries are
    ///   driven by `cbp` (bit `5 - i`), and — for 4:2:2 / 4:4:4
    ///   chroma — entries 6..8 / 8..12 are driven by
    ///   `coded_block_pattern_1` / `coded_block_pattern_2`.
    ///
    /// `pattern_code[i] == true` means block `i` carries coded
    /// coefficients.
    pub fn pattern_code(&self, macroblock_intra: bool, macroblock_pattern: bool) -> [bool; 12] {
        let mut pattern_code = [macroblock_intra; 12];

        if macroblock_pattern {
            for (i, slot) in pattern_code.iter_mut().enumerate().take(6) {
                if self.cbp & (1 << (5 - i)) != 0 {
                    *slot = true;
                }
            }
            if let Some(cbp1) = self.coded_block_pattern_1 {
                // i in 6..8 → mask bit (7 - i): bit 1 then bit 0.
                for (i, slot) in pattern_code.iter_mut().enumerate().take(8).skip(6) {
                    if cbp1 & (1 << (7 - i)) != 0 {
                        *slot = true;
                    }
                }
            }
            if let Some(cbp2) = self.coded_block_pattern_2 {
                // i in 8..12 → mask bit (11 - i): bits 3,2,1,0.
                for (i, slot) in pattern_code.iter_mut().enumerate().take(12).skip(8) {
                    if cbp2 & (1 << (11 - i)) != 0 {
                        *slot = true;
                    }
                }
            }
        }

        pattern_code
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact round-trips covering every row of Table
    //! B-9, the 4:2:2 / 4:4:4 fixed-length extensions, the §6.3.17.4
    //! `pattern_code` derivation, and the rejection / truncation
    //! paths.
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Emit a code into a fresh buffer, padded with trailing `'1'`
    /// bits to a byte boundary (the parser only reads the bits it
    /// needs; Table B-9 is prefix-free so the padding is never
    /// confused with the codeword).
    fn buf_for(code: u32, bits: u32) -> Vec<u8> {
        let mut bw = BitWriter::new();
        bw.write_u32(code, bits);
        bw.write_bit(true);
        bw.align_to_byte();
        bw.finish()
    }

    fn parse_420(code: u32, bits: u32) -> CodedBlockPattern {
        let buf = buf_for(code, bits);
        let mut br = BitReader::new(&buf);
        CodedBlockPattern::parse(&mut br, ChromaFormat::Yuv420).expect("codeword should parse")
    }

    #[test]
    fn every_table_b9_row_round_trips() {
        for row in TABLE_B9 {
            let cbp = parse_420(u32::from(row.code), u32::from(row.bits));
            assert_eq!(cbp.cbp, row.cbp, "cbp for code {:b}", row.code);
            assert_eq!(cbp.coded_block_pattern_1, None);
            assert_eq!(cbp.coded_block_pattern_2, None);
            assert_eq!(cbp.bit_position_after, u64::from(row.bits));
        }
    }

    #[test]
    fn spot_check_spec_listed_codes() {
        // A handful read straight off the printed Table B-9.
        assert_eq!(parse_420(0b111, 3).cbp, 60);
        assert_eq!(parse_420(0b1101, 4).cbp, 4);
        assert_eq!(parse_420(0b0101_1, 5).cbp, 1);
        assert_eq!(parse_420(0b0011_01, 6).cbp, 3);
        assert_eq!(parse_420(0b0010_111, 7).cbp, 5);
        assert_eq!(parse_420(0b0001_1111, 8).cbp, 7);
        assert_eq!(parse_420(0b0000_0000_1, 9).cbp, 0);
    }

    #[test]
    fn all_64_cbp_values_present_exactly_once() {
        let mut seen = [false; 64];
        for row in TABLE_B9 {
            assert!(
                !seen[row.cbp as usize],
                "cbp {} appears more than once",
                row.cbp
            );
            seen[row.cbp as usize] = true;
        }
        assert!(
            seen.iter().all(|&b| b),
            "every cbp 0..=63 must appear in Table B-9"
        );
    }

    #[test]
    fn table_b9_is_prefix_free_and_fits_widths() {
        for r in TABLE_B9 {
            let max = 1u32 << u32::from(r.bits);
            assert!(
                u32::from(r.code) < max,
                "code {:b} does not fit in {} bits",
                r.code,
                r.bits
            );
        }
        for (i, a) in TABLE_B9.iter().enumerate() {
            for b in &TABLE_B9[i + 1..] {
                let (short, long) = if a.bits <= b.bits { (a, b) } else { (b, a) };
                let shift = long.bits - short.bits;
                let long_prefix = u32::from(long.code) >> u32::from(shift);
                assert_ne!(
                    long_prefix,
                    u32::from(short.code),
                    "code {:b} ({}b) is a prefix of {:b} ({}b)",
                    short.code,
                    short.bits,
                    long.code,
                    long.bits
                );
            }
        }
    }

    #[test]
    fn table_b9_has_64_rows() {
        assert_eq!(TABLE_B9.len(), 64);
    }

    #[test]
    fn longest_first_does_not_misread_prefixes() {
        // '0010 111' (7 bits, cbp 5) shares its leading '00...' with
        // many shorter codes that begin '0011..'/'0010..'. The
        // longest-first walk must decode the full 7-bit codeword.
        let cbp = parse_420(0b0010_111, 7);
        assert_eq!(cbp.cbp, 5);
        assert_eq!(cbp.bit_position_after, 7);
        // '0000 0000 1' (9 bits, cbp 0) is the longest code; nothing
        // shorter must steal it.
        let cbp0 = parse_420(0b0000_0000_1, 9);
        assert_eq!(cbp0.cbp, 0);
        assert_eq!(cbp0.bit_position_after, 9);
    }

    #[test]
    fn yuv422_appends_two_bit_extension() {
        // VLC '111' (cbp 60) followed by coded_block_pattern_1 = '10'.
        let mut bw = BitWriter::new();
        bw.write_u32(0b111, 3);
        bw.write_u32(0b10, 2);
        bw.write_bit(true);
        bw.align_to_byte();
        let buf = bw.finish();
        let mut br = BitReader::new(&buf);
        let cbp = CodedBlockPattern::parse(&mut br, ChromaFormat::Yuv422).expect("parse 4:2:2");
        assert_eq!(cbp.cbp, 60);
        assert_eq!(cbp.coded_block_pattern_1, Some(0b10));
        assert_eq!(cbp.coded_block_pattern_2, None);
        assert_eq!(cbp.bit_position_after, 3 + 2);
    }

    #[test]
    fn yuv444_appends_six_bit_extension() {
        // VLC '1101' (cbp 4) followed by coded_block_pattern_2 = '101010'.
        let mut bw = BitWriter::new();
        bw.write_u32(0b1101, 4);
        bw.write_u32(0b101010, 6);
        bw.write_bit(true);
        bw.align_to_byte();
        let buf = bw.finish();
        let mut br = BitReader::new(&buf);
        let cbp = CodedBlockPattern::parse(&mut br, ChromaFormat::Yuv444).expect("parse 4:4:4");
        assert_eq!(cbp.cbp, 4);
        assert_eq!(cbp.coded_block_pattern_1, None);
        assert_eq!(cbp.coded_block_pattern_2, Some(0b101010));
        assert_eq!(cbp.bit_position_after, 4 + 6);
    }

    #[test]
    fn pattern_code_intra_all_ones_when_no_pattern() {
        // An intra macroblock with macroblock_pattern == 0: every
        // pattern_code entry is 1 regardless of cbp.
        let cbp = parse_420(0b111, 3); // cbp 60, but unused here
        let pc = cbp.pattern_code(/*intra=*/ true, /*pattern=*/ false);
        assert_eq!(pc, [true; 12]);
    }

    #[test]
    fn pattern_code_non_intra_all_zero_when_no_pattern() {
        let cbp = parse_420(0b111, 3);
        let pc = cbp.pattern_code(/*intra=*/ false, /*pattern=*/ false);
        assert_eq!(pc, [false; 12]);
    }

    #[test]
    fn pattern_code_420_drives_low_six_from_cbp() {
        // cbp = 60 = 0b111100 → bits 5,4,3,2 set, bits 1,0 clear.
        // pattern_code[i] = (cbp & (1 << (5 - i))).
        let cbp = parse_420(0b111, 3); // cbp 60
        let pc = cbp.pattern_code(/*intra=*/ false, /*pattern=*/ true);
        // i: 0 1 2 3 4 5  → mask bit 5 4 3 2 1 0
        assert_eq!(
            pc,
            [
                true, true, true, true, false, false, // luma+chroma 4:2:0 blocks
                false, false, false, false, false, false, // 4:2:2/4:4:4 blocks unset
            ]
        );
    }

    #[test]
    fn pattern_code_intra_pattern_ors_cbp_over_ones() {
        // An intra macroblock still starts all-ones; setting pattern
        // only keeps them set (cbp can never clear an intra block).
        let cbp = parse_420(0b0101_1, 5); // cbp 1 = 0b000001 → only bit 0
        let pc = cbp.pattern_code(/*intra=*/ true, /*pattern=*/ true);
        assert_eq!(pc[0..6], [true; 6]);
        // Blocks 6..12 are never touched in 4:2:0, and intra left them 1.
        assert_eq!(pc[6..12], [true; 6]);
    }

    #[test]
    fn pattern_code_422_extension_drives_blocks_six_and_seven() {
        // cbp 0 (all luma/chroma-420 blocks clear) + cbp_1 = '10'
        // → block 6 set (mask bit 1), block 7 clear (mask bit 0).
        let mut bw = BitWriter::new();
        bw.write_u32(0b0000_0000_1, 9); // cbp 0
        bw.write_u32(0b10, 2); // cbp_1
        bw.write_bit(true);
        bw.align_to_byte();
        let buf = bw.finish();
        let mut br = BitReader::new(&buf);
        let cbp = CodedBlockPattern::parse(&mut br, ChromaFormat::Yuv422).expect("parse");
        let pc = cbp.pattern_code(false, true);
        assert_eq!(pc[0..6], [false; 6]);
        assert!(pc[6], "cbp_1 bit 1 sets block 6");
        assert!(!pc[7], "cbp_1 bit 0 clears block 7");
        assert_eq!(pc[8..12], [false; 4]);
    }

    #[test]
    fn pattern_code_444_extension_drives_blocks_eight_to_eleven() {
        // cbp 0 + cbp_2 = '1010' (6-bit value 0b001010)
        // → for i in 8..12 mask bit (11 - i): bits 3,2,1,0.
        //   value 0b001010 has bits 3 and 1 set → blocks 8 and 10.
        let mut bw = BitWriter::new();
        bw.write_u32(0b0000_0000_1, 9); // cbp 0
        bw.write_u32(0b001010, 6); // cbp_2
        bw.write_bit(true);
        bw.align_to_byte();
        let buf = bw.finish();
        let mut br = BitReader::new(&buf);
        let cbp = CodedBlockPattern::parse(&mut br, ChromaFormat::Yuv444).expect("parse");
        let pc = cbp.pattern_code(false, true);
        assert_eq!(pc[0..6], [false; 6]);
        assert!(pc[8], "cbp_2 bit 3 sets block 8");
        assert!(!pc[9], "cbp_2 bit 2 clears block 9");
        assert!(pc[10], "cbp_2 bit 1 sets block 10");
        assert!(!pc[11], "cbp_2 bit 0 clears block 11");
    }

    #[test]
    fn rejects_unknown_codeword() {
        // The all-zero 9-bit prefix '0000 0000 0' matches no Table B-9
        // codeword (the longest valid all-zero code is '0000 0000 1').
        let buf = [0u8; 2];
        let mut br = BitReader::new(&buf);
        let err = CodedBlockPattern::parse(&mut br, ChromaFormat::Yuv420).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_truncated_buffer() {
        let buf: [u8; 0] = [];
        let mut br = BitReader::new(&buf);
        let err = CodedBlockPattern::parse(&mut br, ChromaFormat::Yuv420).unwrap_err();
        assert!(matches!(
            err,
            Error::ShortHeader | Error::InvalidBitstream(_)
        ));
    }

    #[test]
    fn rejects_truncated_444_extension() {
        // A valid 3-bit VLC ('111', cbp 60) leaves 5 bits in a single
        // byte — fewer than the 6-bit `coded_block_pattern_2` the
        // 4:4:4 path requires, so the FLC read must short-fault.
        let one_byte = [0b1110_0000u8];
        let mut br = BitReader::new(&one_byte[..]);
        let err = CodedBlockPattern::parse(&mut br, ChromaFormat::Yuv444).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    #[test]
    fn accepts_well_formed_422_extension_at_byte_edge() {
        // '111' (3-bit VLC) + '00' (cbp_1) = 5 bits all inside one
        // byte — the complementary success case to the truncation
        // test above.
        let one_byte = [0b1110_0000u8];
        let mut br = BitReader::new(&one_byte[..]);
        let cbp = CodedBlockPattern::parse(&mut br, ChromaFormat::Yuv422).expect("5 bits fit");
        assert_eq!(cbp.cbp, 60);
        assert_eq!(cbp.coded_block_pattern_1, Some(0b00));
    }

    #[test]
    fn debug_impl_smoke() {
        let cbp = CodedBlockPattern {
            cbp: 60,
            coded_block_pattern_1: Some(2),
            coded_block_pattern_2: None,
            bit_position_after: 5,
        };
        let s = format!("{cbp:?}");
        assert!(s.contains("CodedBlockPattern"));
        assert!(s.contains("cbp"));
    }

    #[test]
    fn encode_coded_block_pattern_422_roundtrips_all_patterns() {
        // Every non-empty 8-bit pattern must parse back to the same
        // pattern_code[0..8] through the §6.3.17.4 derivation.
        for bits in 1u32..256 {
            let coded: Vec<bool> = (0..8).map(|i| bits & (1 << (7 - i)) != 0).collect();
            let mut bw = BitWriter::new();
            super::encode_coded_block_pattern(&mut bw, &coded, ChromaFormat::Yuv422)
                .expect("non-empty 4:2:2 pattern encodes");
            bw.write_bit(true);
            bw.align_to_byte();
            let bytes = bw.finish();
            let mut br = BitReader::new(&bytes);
            let parsed =
                CodedBlockPattern::parse(&mut br, ChromaFormat::Yuv422).expect("parse back");
            let pc = parsed.pattern_code(false, true);
            for i in 0..8 {
                assert_eq!(pc[i], coded[i], "block {i} of pattern {bits:#010b}");
            }
            assert_eq!(pc[8..12], [false; 4]);
        }
    }

    #[test]
    fn encode_coded_block_pattern_422_chroma_extension_only() {
        // Only blocks 6/7 coded: cbp 0 ('0000 0000 1') + cbp_1 '11' —
        // legal for 4:2:2 (the Table B-9 NOTE bars cbp 0 for 4:2:0 only).
        let coded = [false, false, false, false, false, false, true, true];
        let mut bw = BitWriter::new();
        super::encode_coded_block_pattern(&mut bw, &coded, ChromaFormat::Yuv422).expect("encode");
        bw.write_bit(true);
        bw.align_to_byte();
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let parsed = CodedBlockPattern::parse(&mut br, ChromaFormat::Yuv422).expect("parse");
        assert_eq!(parsed.cbp, 0);
        assert_eq!(parsed.coded_block_pattern_1, Some(0b11));
        let pc = parsed.pattern_code(false, true);
        assert_eq!(pc[0..6], [false; 6]);
        assert!(pc[6] && pc[7]);
    }

    #[test]
    fn encode_coded_block_pattern_444_roundtrips_supported_patterns() {
        // 4:4:4 patterns over blocks 0..6 and 8..12 (blocks 6/7 have no
        // §6.3.17.4 non-intra wire representation).
        for bits in 1u32..1024 {
            let mut coded = [false; 12];
            for (i, slot) in coded.iter_mut().enumerate().take(6) {
                *slot = bits & (1 << (9 - i)) != 0;
            }
            for (i, slot) in coded.iter_mut().enumerate().take(12).skip(8) {
                *slot = bits & (1 << (11 - i)) != 0;
            }
            if coded.iter().all(|&b| !b) {
                continue;
            }
            let mut bw = BitWriter::new();
            super::encode_coded_block_pattern(&mut bw, &coded, ChromaFormat::Yuv444)
                .expect("supported 4:4:4 pattern encodes");
            bw.write_bit(true);
            bw.align_to_byte();
            let bytes = bw.finish();
            let mut br = BitReader::new(&bytes);
            let parsed =
                CodedBlockPattern::parse(&mut br, ChromaFormat::Yuv444).expect("parse back");
            let pc = parsed.pattern_code(false, true);
            for i in 0..12 {
                assert_eq!(pc[i], coded[i], "block {i} of pattern {bits:#012b}");
            }
        }
    }

    #[test]
    fn encode_coded_block_pattern_rejects_empty_and_444_blocks_6_7() {
        let mut bw = BitWriter::new();
        assert!(
            super::encode_coded_block_pattern(&mut bw, &[false; 6], ChromaFormat::Yuv420).is_err(),
            "empty pattern must be rejected"
        );
        assert!(
            super::encode_coded_block_pattern(&mut bw, &[false; 8], ChromaFormat::Yuv422).is_err(),
            "empty 4:2:2 pattern must be rejected"
        );
        let mut only_chroma420_empty = [false; 6];
        only_chroma420_empty[0] = true;
        assert!(super::encode_coded_block_pattern(
            &mut bw,
            &only_chroma420_empty,
            ChromaFormat::Yuv420
        )
        .is_ok());
        let mut block6 = [false; 12];
        block6[6] = true;
        block6[0] = true;
        assert!(
            super::encode_coded_block_pattern(&mut bw, &block6, ChromaFormat::Yuv444).is_err(),
            "4:4:4 block 6 has no non-intra representation"
        );
    }

    #[test]
    fn encode_cbp420_roundtrips_every_listed_value() {
        // Encoding each Table B-9 cbp then decoding via parse must
        // recover the same 6-bit pattern.
        for &row in TABLE_B9 {
            let mut bw = BitWriter::new();
            super::encode_cbp420(&mut bw, row.cbp);
            bw.write_bit(true);
            bw.align_to_byte();
            let bytes = bw.finish();
            let mut br = BitReader::new(&bytes);
            let parsed =
                CodedBlockPattern::parse(&mut br, ChromaFormat::Yuv420).expect("parse cbp");
            assert_eq!(parsed.cbp, row.cbp, "cbp {} mismatch", row.cbp);
        }
    }
}
