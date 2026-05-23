//! Parser for `macroblock_type` — the leading VLC of
//! `macroblock_modes()` per ISO/IEC 13818-2 (Recommendation ITU-T
//! H.262) §6.2.5.1, with the field semantics from §6.3.17.1 and the
//! Annex B Table B-2 / B-3 / B-4 codeword sets.
//!
//! Round 7 advances the macroblock loop one field past round 6's
//! `macroblock_address_increment`. Given a
//! [`oxideav_core::bits::BitReader`] sitting at the first bit of
//! `macroblock_modes()` (i.e. right after a parsed
//! [`crate::MbAddressIncrement`]), the parser walks the
//! `macroblock_type` VLC for the current `picture_coding_type` and
//! returns the six derived flags the spec lists in §6.3.17.1:
//!
//! * `macroblock_quant` — when set, a `quantiser_scale_code` follows
//!   in the bitstream (§6.2.5 macroblock()).
//! * `macroblock_motion_forward` / `macroblock_motion_backward` —
//!   control whether `motion_vectors(0)` / `motion_vectors(1)` are
//!   present and which prediction is formed.
//! * `macroblock_pattern` — when set, `coded_block_pattern()` follows.
//! * `macroblock_intra` — selects intra coding for the macroblock.
//! * `spatial_temporal_weight_code_flag` — for the non-scalable
//!   B-2 / B-3 / B-4 tables this is always `0`; it is surfaced for
//!   completeness so a later scalable-mode round can reuse the type.
//!
//! Round 7 covers **only** the non-scalable tables B-2 (I-pictures),
//! B-3 (P-pictures) and B-4 (B-pictures). Per Table 6-10 those are
//! exactly the tables a decoder selects when no
//! `sequence_scalable_extension()` is present — which is every stream
//! this crate can currently parse. The scalable tables B-5 .. B-8
//! (spatial / SNR scalability) require `sequence_scalable_extension()`
//! parsing that no earlier round has landed; they are out of scope.
//! The fields *after* `macroblock_type` inside `macroblock_modes()`
//! (`spatial_temporal_weight_code`, `frame_motion_type`,
//! `field_motion_type`, `dct_type`) are likewise deferred — they
//! depend on `picture_coding_extension()` state (`frame_pred_frame_dct`,
//! `picture_structure`) that is best threaded through in a later round.
//!
//! D-pictures (MPEG-1 `picture_coding_type == 4`) have no
//! `macroblock_type` VLC of this form and are rejected by the
//! [`crate::PictureCodingType`] parser already; this module therefore
//! only accepts `I`, `P`, and `B`.
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)) §6.2.5.1, §6.3.17.1, Table
//! 6-10, and Annex B Tables B-2, B-3, B-4.

// Bit-group widths follow the spec's MSB-first visual layout of the
// Annex B tables (e.g. `0b0000_01` for the 6-bit P-picture
// "Intra, Quant" code) so an audit can read each constant against the
// printed table at a glance. clippy's `unusual_byte_groupings` lint
// prefers equal-size 4-bit groups, which would obscure the spec
// mapping.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitReader;

use crate::picture_header::PictureCodingType;
use crate::{Error, Result};

/// The six boolean flags derived from a `macroblock_type` VLC per
/// §6.3.17.1, together with the `spatial_temporal_weight_code_flag`.
///
/// For the non-scalable Tables B-2 / B-3 / B-4 the
/// `spatial_temporal_weight_code_flag` is always `false`; it is kept
/// here so the same type can be reused once the scalable tables land.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MacroblockType {
    /// `macroblock_quant`: a `quantiser_scale_code` follows in the
    /// bitstream (§6.2.5).
    pub macroblock_quant: bool,
    /// `macroblock_motion_forward`: forward motion vectors are present
    /// / forward prediction is formed.
    pub macroblock_motion_forward: bool,
    /// `macroblock_motion_backward`: backward motion vectors are
    /// present / backward prediction is formed.
    pub macroblock_motion_backward: bool,
    /// `macroblock_pattern`: a `coded_block_pattern()` follows in the
    /// bitstream (§6.2.5).
    pub macroblock_pattern: bool,
    /// `macroblock_intra`: the macroblock is intra-coded.
    pub macroblock_intra: bool,
    /// `spatial_temporal_weight_code_flag`: always `false` for the
    /// non-scalable Tables B-2 / B-3 / B-4 (§6.3.17.1).
    pub spatial_temporal_weight_code_flag: bool,
    /// Bit position (relative to the start of the buffer the
    /// [`BitReader`] was created from) right after the consumed
    /// `macroblock_type` VLC. Lets callers chain into the next
    /// `macroblock_modes()` field without losing the partial-byte
    /// cursor.
    pub bit_position_after: u64,
}

/// One Annex B table row: a right-justified MSB-first VLC code, its
/// bit length, and the five spec flag columns. The
/// `spatial_temporal_weight_code_flag` column is `0` for every row of
/// B-2 / B-3 / B-4, so it is not stored per-row — it is reported as
/// `false` by [`MacroblockType`].
#[derive(Clone, Copy)]
struct Row {
    /// VLC code right-justified into a `u8` — e.g. the bit string
    /// `0000 01` becomes `0b00_0001` (decimal 1) with `bits == 6`.
    code: u8,
    /// Length of `code` in bits (`1..=6` across B-2 / B-3 / B-4).
    bits: u8,
    /// `macroblock_quant` column.
    quant: bool,
    /// `macroblock_motion_forward` column.
    fwd: bool,
    /// `macroblock_motion_backward` column.
    bwd: bool,
    /// `macroblock_pattern` column.
    pattern: bool,
    /// `macroblock_intra` column.
    intra: bool,
}

/// Table B-2 — `macroblock_type` in I-pictures.
///
/// ```text
/// VLC  quant fwd bwd pat intra  Description
/// 1     0     0   0   0   1     Intra
/// 01    1     0   0   0   1     Intra, Quant
/// ```
const TABLE_B2_I: &[Row] = &[
    Row {
        code: 0b1,
        bits: 1,
        quant: false,
        fwd: false,
        bwd: false,
        pattern: false,
        intra: true,
    },
    Row {
        code: 0b01,
        bits: 2,
        quant: true,
        fwd: false,
        bwd: false,
        pattern: false,
        intra: true,
    },
];

/// Table B-3 — `macroblock_type` in P-pictures.
///
/// ```text
/// VLC      quant fwd bwd pat intra  Description
/// 1         0     1   0   1   0     MC, Coded
/// 01        0     0   0   1   0     No MC, Coded
/// 001       0     1   0   0   0     MC, Not Coded
/// 0001 1    0     0   0   0   1     Intra
/// 0001 0    1     1   0   1   0     MC, Coded, Quant
/// 0000 1    1     0   0   1   0     No MC, Coded, Quant
/// 0000 01   1     0   0   0   1     Intra, Quant
/// ```
const TABLE_B3_P: &[Row] = &[
    Row {
        code: 0b1,
        bits: 1,
        quant: false,
        fwd: true,
        bwd: false,
        pattern: true,
        intra: false,
    },
    Row {
        code: 0b01,
        bits: 2,
        quant: false,
        fwd: false,
        bwd: false,
        pattern: true,
        intra: false,
    },
    Row {
        code: 0b001,
        bits: 3,
        quant: false,
        fwd: true,
        bwd: false,
        pattern: false,
        intra: false,
    },
    Row {
        code: 0b0001_1,
        bits: 5,
        quant: false,
        fwd: false,
        bwd: false,
        pattern: false,
        intra: true,
    },
    Row {
        code: 0b0001_0,
        bits: 5,
        quant: true,
        fwd: true,
        bwd: false,
        pattern: true,
        intra: false,
    },
    Row {
        code: 0b0000_1,
        bits: 5,
        quant: true,
        fwd: false,
        bwd: false,
        pattern: true,
        intra: false,
    },
    Row {
        code: 0b0000_01,
        bits: 6,
        quant: true,
        fwd: false,
        bwd: false,
        pattern: false,
        intra: true,
    },
];

/// Table B-4 — `macroblock_type` in B-pictures.
///
/// ```text
/// VLC      quant fwd bwd pat intra  Description
/// 10        0     1   1   0   0     Interp, Not Coded
/// 11        0     1   1   1   0     Interp, Coded
/// 010       0     0   1   0   0     Bwd, Not Coded
/// 011       0     0   1   1   0     Bwd, Coded
/// 0010      0     1   0   0   0     Fwd, Not Coded
/// 0011      0     1   0   1   0     Fwd, Coded
/// 0001 1    0     0   0   0   1     Intra
/// 0001 0    1     1   1   1   0     Interp, Coded, Quant
/// 0000 11   1     1   0   1   0     Fwd, Coded, Quant
/// 0000 10   1     0   1   1   0     Bwd, Coded, Quant
/// 0000 01   1     0   0   0   1     Intra, Quant
/// ```
const TABLE_B4_B: &[Row] = &[
    Row {
        code: 0b10,
        bits: 2,
        quant: false,
        fwd: true,
        bwd: true,
        pattern: false,
        intra: false,
    },
    Row {
        code: 0b11,
        bits: 2,
        quant: false,
        fwd: true,
        bwd: true,
        pattern: true,
        intra: false,
    },
    Row {
        code: 0b010,
        bits: 3,
        quant: false,
        fwd: false,
        bwd: true,
        pattern: false,
        intra: false,
    },
    Row {
        code: 0b011,
        bits: 3,
        quant: false,
        fwd: false,
        bwd: true,
        pattern: true,
        intra: false,
    },
    Row {
        code: 0b0010,
        bits: 4,
        quant: false,
        fwd: true,
        bwd: false,
        pattern: false,
        intra: false,
    },
    Row {
        code: 0b0011,
        bits: 4,
        quant: false,
        fwd: true,
        bwd: false,
        pattern: true,
        intra: false,
    },
    Row {
        code: 0b0001_1,
        bits: 5,
        quant: false,
        fwd: false,
        bwd: false,
        pattern: false,
        intra: true,
    },
    Row {
        code: 0b0001_0,
        bits: 5,
        quant: true,
        fwd: true,
        bwd: true,
        pattern: true,
        intra: false,
    },
    Row {
        code: 0b0000_11,
        bits: 6,
        quant: true,
        fwd: true,
        bwd: false,
        pattern: true,
        intra: false,
    },
    Row {
        code: 0b0000_10,
        bits: 6,
        quant: true,
        fwd: false,
        bwd: true,
        pattern: true,
        intra: false,
    },
    Row {
        code: 0b0000_01,
        bits: 6,
        quant: true,
        fwd: false,
        bwd: false,
        pattern: false,
        intra: true,
    },
];

/// Select the non-scalable `macroblock_type` table for a picture
/// coding type per Table 6-10 (no `sequence_scalable_extension()`).
fn table_for(picture_coding_type: PictureCodingType) -> Result<&'static [Row]> {
    match picture_coding_type {
        PictureCodingType::Intra => Ok(TABLE_B2_I),
        PictureCodingType::Predictive => Ok(TABLE_B3_P),
        PictureCodingType::Bidirectional => Ok(TABLE_B4_B),
    }
}

/// Walk the picture-type-selected table longest-first so a shorter
/// codeword can never be matched on the high bits of a longer one.
fn match_row(br: &mut BitReader<'_>, table: &'static [Row]) -> Result<Row> {
    // Distinct code widths across B-2 / B-3 / B-4, descending. Walking
    // longest-first guarantees an exact full-width equality match.
    for &width in &[6u8, 5, 4, 3, 2, 1] {
        if br.bits_remaining() < u64::from(width) {
            continue;
        }
        let peeked = br
            .peek_u32(u32::from(width))
            .map_err(|_| Error::ShortHeader)? as u8;
        for &row in table.iter().filter(|r| r.bits == width) {
            if row.code == peeked {
                br.consume(u32::from(width))
                    .map_err(|_| Error::ShortHeader)?;
                return Ok(row);
            }
        }
    }
    Err(Error::InvalidBitstream(
        "macroblock_type: no Table B-2/B-3/B-4 codeword matches the bit prefix (§6.2.5.1)",
    ))
}

impl MacroblockType {
    /// Parse one `macroblock_type` VLC starting at the current
    /// position of `br`, selecting the table from
    /// `picture_coding_type` per Table 6-10 (non-scalable streams).
    /// Consumes from `br` on success.
    ///
    /// Errors:
    /// * [`Error::InvalidBitstream`] if no codeword in the selected
    ///   table matches the upcoming bits.
    /// * [`Error::ShortHeader`] if the bitstream ends before a full
    ///   codeword could be read.
    pub fn parse(br: &mut BitReader<'_>, picture_coding_type: PictureCodingType) -> Result<Self> {
        let table = table_for(picture_coding_type)?;
        let row = match_row(br, table)?;
        Ok(Self {
            macroblock_quant: row.quant,
            macroblock_motion_forward: row.fwd,
            macroblock_motion_backward: row.bwd,
            macroblock_pattern: row.pattern,
            macroblock_intra: row.intra,
            // Always 0 for the non-scalable B-2 / B-3 / B-4 tables.
            spatial_temporal_weight_code_flag: false,
            bit_position_after: br.bit_position(),
        })
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact round-trips covering every row of Tables
    //! B-2, B-3 and B-4 plus the rejection / truncation paths.
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Emit a code into a fresh buffer, padded with a trailing `'1'`
    /// (so the reader has a valid trailing byte the parser will never
    /// confuse with the codeword under test — every table here is
    /// prefix-free).
    fn buf_for(code: u32, bits: u32) -> Vec<u8> {
        let mut bw = BitWriter::new();
        bw.write_u32(code, bits);
        // Pad to a byte boundary with 1-bits; the parser only reads
        // `bits` of them.
        bw.write_bit(true);
        bw.align_to_byte();
        bw.finish()
    }

    fn parse(code: u32, bits: u32, pct: PictureCodingType) -> MacroblockType {
        let buf = buf_for(code, bits);
        let mut br = BitReader::new(&buf);
        MacroblockType::parse(&mut br, pct).expect("codeword should parse")
    }

    #[test]
    fn i_picture_intra() {
        // Table B-2 row '1': Intra (intra only).
        let mt = parse(0b1, 1, PictureCodingType::Intra);
        assert!(mt.macroblock_intra);
        assert!(!mt.macroblock_quant);
        assert!(!mt.macroblock_motion_forward);
        assert!(!mt.macroblock_motion_backward);
        assert!(!mt.macroblock_pattern);
        assert!(!mt.spatial_temporal_weight_code_flag);
        assert_eq!(mt.bit_position_after, 1);
    }

    #[test]
    fn i_picture_intra_quant() {
        // Table B-2 row '01': Intra, Quant.
        let mt = parse(0b01, 2, PictureCodingType::Intra);
        assert!(mt.macroblock_intra);
        assert!(mt.macroblock_quant);
        assert!(!mt.macroblock_motion_forward);
        assert!(!mt.macroblock_motion_backward);
        assert!(!mt.macroblock_pattern);
        assert_eq!(mt.bit_position_after, 2);
    }

    #[test]
    fn p_picture_all_rows() {
        // (code, bits, quant, fwd, bwd, pattern, intra)
        let cases: &[(u32, u32, bool, bool, bool, bool, bool)] = &[
            (0b1, 1, false, true, false, true, false),    // MC, Coded
            (0b01, 2, false, false, false, true, false),  // No MC, Coded
            (0b001, 3, false, true, false, false, false), // MC, Not Coded
            (0b0001_1, 5, false, false, false, false, true), // Intra
            (0b0001_0, 5, true, true, false, true, false), // MC, Coded, Quant
            (0b0000_1, 5, true, false, false, true, false), // No MC, Coded, Quant
            (0b0000_01, 6, true, false, false, false, true), // Intra, Quant
        ];
        for &(code, bits, quant, fwd, bwd, pattern, intra) in cases {
            let mt = parse(code, bits, PictureCodingType::Predictive);
            assert_eq!(mt.macroblock_quant, quant, "quant for code {code:b}");
            assert_eq!(mt.macroblock_motion_forward, fwd, "fwd for code {code:b}");
            assert_eq!(mt.macroblock_motion_backward, bwd, "bwd for code {code:b}");
            assert_eq!(mt.macroblock_pattern, pattern, "pattern for code {code:b}");
            assert_eq!(mt.macroblock_intra, intra, "intra for code {code:b}");
            assert!(!mt.spatial_temporal_weight_code_flag);
            assert_eq!(mt.bit_position_after, u64::from(bits));
        }
    }

    #[test]
    fn b_picture_all_rows() {
        let cases: &[(u32, u32, bool, bool, bool, bool, bool)] = &[
            (0b10, 2, false, true, true, false, false), // Interp, Not Coded
            (0b11, 2, false, true, true, true, false),  // Interp, Coded
            (0b010, 3, false, false, true, false, false), // Bwd, Not Coded
            (0b011, 3, false, false, true, true, false), // Bwd, Coded
            (0b0010, 4, false, true, false, false, false), // Fwd, Not Coded
            (0b0011, 4, false, true, false, true, false), // Fwd, Coded
            (0b0001_1, 5, false, false, false, false, true), // Intra
            (0b0001_0, 5, true, true, true, true, false), // Interp, Coded, Quant
            (0b0000_11, 6, true, true, false, true, false), // Fwd, Coded, Quant
            (0b0000_10, 6, true, false, true, true, false), // Bwd, Coded, Quant
            (0b0000_01, 6, true, false, false, false, true), // Intra, Quant
        ];
        for &(code, bits, quant, fwd, bwd, pattern, intra) in cases {
            let mt = parse(code, bits, PictureCodingType::Bidirectional);
            assert_eq!(mt.macroblock_quant, quant, "quant for code {code:b}");
            assert_eq!(mt.macroblock_motion_forward, fwd, "fwd for code {code:b}");
            assert_eq!(mt.macroblock_motion_backward, bwd, "bwd for code {code:b}");
            assert_eq!(mt.macroblock_pattern, pattern, "pattern for code {code:b}");
            assert_eq!(mt.macroblock_intra, intra, "intra for code {code:b}");
            assert!(!mt.spatial_temporal_weight_code_flag);
            assert_eq!(mt.bit_position_after, u64::from(bits));
        }
    }

    #[test]
    fn longest_first_match_does_not_misread_prefixes() {
        // In a P-picture, '1' is a 1-bit code (MC, Coded). The 5-bit
        // '0001 1' (Intra) and the 6-bit '0000 01' (Intra, Quant)
        // both *start* with prefixes of shorter codes. Decoding each
        // full codeword must not be hijacked by a shorter prefix.
        let intra = parse(0b0001_1, 5, PictureCodingType::Predictive);
        assert!(intra.macroblock_intra);
        assert_eq!(intra.bit_position_after, 5);

        let intra_q = parse(0b0000_01, 6, PictureCodingType::Predictive);
        assert!(intra_q.macroblock_intra && intra_q.macroblock_quant);
        assert_eq!(intra_q.bit_position_after, 6);
    }

    #[test]
    fn rejects_unknown_codeword() {
        // In an I-picture, the only valid prefixes are '1' and '01'.
        // A leading '00...' matches neither.
        let buf = [0u8; 2];
        let mut br = BitReader::new(&buf);
        let err = MacroblockType::parse(&mut br, PictureCodingType::Intra).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_truncated_buffer() {
        // An empty buffer offers no bits; the parser cannot read even
        // the shortest 1-bit I-picture codeword.
        let buf: [u8; 0] = [];
        let mut br = BitReader::new(&buf);
        let err = MacroblockType::parse(&mut br, PictureCodingType::Intra).unwrap_err();
        assert!(matches!(
            err,
            Error::ShortHeader | Error::InvalidBitstream(_)
        ));
    }

    #[test]
    fn rejects_all_zero_prefix_in_p_picture() {
        // '000000' (6 bits) matches no Table B-3 codeword — every
        // valid 5- and 6-bit P-table code begins '0001' or '0000 1'.
        let buf = [0u8; 2];
        let mut br = BitReader::new(&buf);
        let err = MacroblockType::parse(&mut br, PictureCodingType::Predictive).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn table_b2_is_prefix_free() {
        assert_prefix_free(TABLE_B2_I);
    }

    #[test]
    fn table_b3_is_prefix_free() {
        assert_prefix_free(TABLE_B3_P);
    }

    #[test]
    fn table_b4_is_prefix_free() {
        assert_prefix_free(TABLE_B4_B);
    }

    /// Every code must (a) fit in its declared bit width and (b) be
    /// prefix-free: no codeword is a bit-prefix of another. A
    /// non-prefix-free table would make `match_row`'s longest-first
    /// walk ambiguous.
    fn assert_prefix_free(table: &[Row]) {
        for r in table {
            let max = 1u32 << u32::from(r.bits);
            assert!(
                u32::from(r.code) < max,
                "code {:b} does not fit in {} bits",
                r.code,
                r.bits
            );
        }
        for (i, a) in table.iter().enumerate() {
            for b in &table[i + 1..] {
                // Compare the shorter against the high bits of the
                // longer.
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
    fn table_sizes_match_spec() {
        // B-2 has 2 rows, B-3 has 7, B-4 has 11 (Annex B).
        assert_eq!(TABLE_B2_I.len(), 2);
        assert_eq!(TABLE_B3_P.len(), 7);
        assert_eq!(TABLE_B4_B.len(), 11);
    }

    #[test]
    fn debug_impl_smoke() {
        let mt = MacroblockType {
            macroblock_quant: true,
            macroblock_motion_forward: false,
            macroblock_motion_backward: false,
            macroblock_pattern: false,
            macroblock_intra: true,
            spatial_temporal_weight_code_flag: false,
            bit_position_after: 2,
        };
        let s = format!("{mt:?}");
        assert!(s.contains("MacroblockType"));
        assert!(s.contains("macroblock_intra"));
    }
}
