//! Parser for `macroblock_address_increment` per ISO/IEC 13818-2
//! (Recommendation ITU-T H.262) **Annex B Table B-1** with the field
//! semantics from §6.3.17.1.
//!
//! Round 6 extends the macroblock-loop entry without touching the
//! macroblock body. Given a [`oxideav_core::bits::BitReader`] sitting
//! at the start of a `macroblock()` (i.e. right after a slice
//! header's `body_bit_position`), the parser walks Table B-1 and
//! returns:
//!
//! * the decoded `macroblock_address_increment` value
//!   (`1..=33` plus any number of leading `macroblock_escape`
//!   codes that each contribute `+33`);
//! * the count of consumed `macroblock_escape` codes (for callers
//!   that want to validate spec-imposed limits);
//! * the number of MPEG-1 `macroblock_stuffing` (`0000 0001 111`)
//!   codes consumed before the increment proper — always zero for
//!   MPEG-2 streams; non-zero MPEG-1 streams may emit any number of
//!   stuffing codes at the head of each macroblock (§D.5.5.1 of
//!   ISO/IEC 11172-2:1993).
//!
//! Table B-1 enumerates 33 increment values plus the
//! `macroblock_escape` sentinel. ISO/IEC 13818-2 removed
//! `macroblock_stuffing`; ISO/IEC 11172-2:1993 still has it. The
//! parser supports both via [`MbAddressIncrementContext::mpeg1`].
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)) §6.2.5, §6.3.17.1, and
//! Annex B Table B-1; ISO/IEC 11172-2:1993 §2.4.3.5, §D.5.5.1, and
//! Annex B Table B.1.

// Bit-group widths follow the spec's MSB-first visual layout of
// Table B-1 — e.g. `0b0000_0001_000` for the 11-bit
// `macroblock_escape` code — so an audit can read each constant
// against the printed table at a glance. clippy's
// `unusual_byte_groupings` lint prefers equal-size 4-bit groups,
// which would obscure the spec mapping.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// One entry of Table B-1: a (right-justified) MSB-first VLC code,
/// its bit length, and the symbol it decodes to.
#[derive(Debug, Clone, Copy)]
struct Entry {
    /// VLC code right-justified into a `u16` — i.e. the bit string
    /// `0000 0001 000` becomes `0b0000_0001_000` (decimal 8) with
    /// `bits == 11`.
    code: u16,
    /// Length of `code` in bits, `1..=11` for Table B-1.
    bits: u8,
    /// What the VLC means.
    symbol: TableSymbol,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TableSymbol {
    /// A direct `macroblock_address_increment` value in `1..=33`.
    Value(u8),
    /// `macroblock_escape` — caller adds 33 to the increment value
    /// that the following codeword decodes to.
    Escape,
    /// MPEG-1 `macroblock_stuffing` — discarded by the decoder. Not
    /// present in ISO/IEC 13818-2.
    Stuffing,
}

/// Table B-1 in spec order. The lookup walker scans linearly, so the
/// order here only affects search cost. Including `Stuffing` is
/// harmless for MPEG-2 — its code `0000 0001 111` does not collide
/// with any other Table B-1 entry and we gate emission on
/// [`MbAddressIncrementContext::mpeg1`].
const TABLE_B1: &[Entry] = &[
    // --- 1-bit code ---
    Entry {
        code: 0b1,
        bits: 1,
        symbol: TableSymbol::Value(1),
    },
    // --- 3-bit codes ---
    Entry {
        code: 0b011,
        bits: 3,
        symbol: TableSymbol::Value(2),
    },
    Entry {
        code: 0b010,
        bits: 3,
        symbol: TableSymbol::Value(3),
    },
    // --- 4-bit codes ---
    Entry {
        code: 0b0011,
        bits: 4,
        symbol: TableSymbol::Value(4),
    },
    Entry {
        code: 0b0010,
        bits: 4,
        symbol: TableSymbol::Value(5),
    },
    // --- 5-bit codes ---
    Entry {
        code: 0b00011,
        bits: 5,
        symbol: TableSymbol::Value(6),
    },
    Entry {
        code: 0b00010,
        bits: 5,
        symbol: TableSymbol::Value(7),
    },
    // --- 7-bit codes ---
    Entry {
        code: 0b0000_111,
        bits: 7,
        symbol: TableSymbol::Value(8),
    },
    Entry {
        code: 0b0000_110,
        bits: 7,
        symbol: TableSymbol::Value(9),
    },
    // --- 8-bit codes ---
    Entry {
        code: 0b0000_1011,
        bits: 8,
        symbol: TableSymbol::Value(10),
    },
    Entry {
        code: 0b0000_1010,
        bits: 8,
        symbol: TableSymbol::Value(11),
    },
    Entry {
        code: 0b0000_1001,
        bits: 8,
        symbol: TableSymbol::Value(12),
    },
    Entry {
        code: 0b0000_1000,
        bits: 8,
        symbol: TableSymbol::Value(13),
    },
    Entry {
        code: 0b0000_0111,
        bits: 8,
        symbol: TableSymbol::Value(14),
    },
    Entry {
        code: 0b0000_0110,
        bits: 8,
        symbol: TableSymbol::Value(15),
    },
    // --- 10-bit codes ---
    Entry {
        code: 0b0000_0101_11,
        bits: 10,
        symbol: TableSymbol::Value(16),
    },
    Entry {
        code: 0b0000_0101_10,
        bits: 10,
        symbol: TableSymbol::Value(17),
    },
    Entry {
        code: 0b0000_0101_01,
        bits: 10,
        symbol: TableSymbol::Value(18),
    },
    Entry {
        code: 0b0000_0101_00,
        bits: 10,
        symbol: TableSymbol::Value(19),
    },
    Entry {
        code: 0b0000_0100_11,
        bits: 10,
        symbol: TableSymbol::Value(20),
    },
    Entry {
        code: 0b0000_0100_10,
        bits: 10,
        symbol: TableSymbol::Value(21),
    },
    // --- 11-bit codes ---
    Entry {
        code: 0b0000_0100_011,
        bits: 11,
        symbol: TableSymbol::Value(22),
    },
    Entry {
        code: 0b0000_0100_010,
        bits: 11,
        symbol: TableSymbol::Value(23),
    },
    Entry {
        code: 0b0000_0100_001,
        bits: 11,
        symbol: TableSymbol::Value(24),
    },
    Entry {
        code: 0b0000_0100_000,
        bits: 11,
        symbol: TableSymbol::Value(25),
    },
    Entry {
        code: 0b0000_0011_111,
        bits: 11,
        symbol: TableSymbol::Value(26),
    },
    Entry {
        code: 0b0000_0011_110,
        bits: 11,
        symbol: TableSymbol::Value(27),
    },
    Entry {
        code: 0b0000_0011_101,
        bits: 11,
        symbol: TableSymbol::Value(28),
    },
    Entry {
        code: 0b0000_0011_100,
        bits: 11,
        symbol: TableSymbol::Value(29),
    },
    Entry {
        code: 0b0000_0011_011,
        bits: 11,
        symbol: TableSymbol::Value(30),
    },
    Entry {
        code: 0b0000_0011_010,
        bits: 11,
        symbol: TableSymbol::Value(31),
    },
    Entry {
        code: 0b0000_0011_001,
        bits: 11,
        symbol: TableSymbol::Value(32),
    },
    Entry {
        code: 0b0000_0011_000,
        bits: 11,
        symbol: TableSymbol::Value(33),
    },
    // --- 11-bit special codes ---
    Entry {
        code: 0b0000_0001_000,
        bits: 11,
        symbol: TableSymbol::Escape,
    },
    // MPEG-1 only — gated by `MbAddressIncrementContext::mpeg1`.
    // Listed here so the same walker handles both; the matcher
    // refuses to emit `Stuffing` when `mpeg1 == false`.
    Entry {
        code: 0b0000_0001_111,
        bits: 11,
        symbol: TableSymbol::Stuffing,
    },
];

/// Sanity-checked Table B-1 lookup: peek `bits` bits, compare for
/// equality. The walker tries entries longest-first so that a
/// shorter codeword can never be falsely matched by a longer
/// codeword's prefix.
fn match_b1(br: &mut BitReader<'_>) -> Result<Entry> {
    // Walk from longest to shortest. Equality check on the full
    // codeword bit-width is required because shorter codes will
    // match the high bits of unrelated longer codewords.
    for &width in &[11u8, 10, 8, 7, 5, 4, 3, 1] {
        // Bail out if the bitstream cannot supply `width` bits.
        if br.bits_remaining() < u64::from(width) {
            continue;
        }
        let peeked = br
            .peek_u32(u32::from(width))
            .map_err(|_| Error::ShortHeader)? as u16;
        for &entry in TABLE_B1.iter().filter(|e| e.bits == width) {
            if entry.code == peeked {
                // Consume the matched bits and return the entry.
                br.consume(u32::from(width))
                    .map_err(|_| Error::ShortHeader)?;
                return Ok(entry);
            }
        }
    }
    Err(Error::InvalidBitstream(
        "macroblock_address_increment: no Table B-1 codeword matches the bit prefix (§6.2.5)",
    ))
}

/// Caller-supplied configuration for [`MbAddressIncrement::parse`].
///
/// MPEG-2 (ISO/IEC 13818-2) defines no `macroblock_stuffing` — its
/// Table B-1 has 33 increment codes plus the escape sentinel.
/// MPEG-1 (ISO/IEC 11172-2:1993) additionally recognises
/// `0000 0001 111` as `macroblock_stuffing`, a no-op spacing code
/// (§D.5.5.1 of 11172-2). The default is MPEG-2.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct MbAddressIncrementContext {
    /// `true` to recognise the MPEG-1 `macroblock_stuffing` code.
    /// Defaults to `false` (MPEG-2 / 13818-2 behaviour).
    pub mpeg1: bool,
}

impl MbAddressIncrementContext {
    /// MPEG-1 stream context (recognises `macroblock_stuffing`).
    pub const fn mpeg1() -> Self {
        Self { mpeg1: true }
    }

    /// MPEG-2 stream context (rejects `macroblock_stuffing`).
    pub const fn mpeg2() -> Self {
        Self { mpeg1: false }
    }
}

/// The decoded `macroblock_address_increment` plus the metadata the
/// caller may need to validate spec-defined limits and stuffing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MbAddressIncrement {
    /// Final increment value (`1..=33 + 33 * escape_count`).
    pub value: u16,
    /// Number of `macroblock_escape` codewords consumed before the
    /// terminating `Value(...)` entry.
    pub escape_count: u8,
    /// Number of MPEG-1 `macroblock_stuffing` codewords consumed
    /// before the increment proper. Always `0` for MPEG-2 streams.
    pub stuffing_count: u8,
    /// Bit position (relative to the start of the buffer the parser
    /// was handed) right after the last consumed VLC bit. Useful
    /// for chaining into the next macroblock field.
    pub bit_position_after: u64,
}

impl MbAddressIncrement {
    /// Parse one `macroblock_address_increment` starting at the
    /// current position of `br`. Consumes from `br` on success.
    ///
    /// Per §6.3.17.1 the maximum *decoded* increment is bounded by
    /// the slice's `mb_width * (mb_height - mb_row)` macroblocks
    /// remaining in the picture; this parser does not know those
    /// numbers, so it accepts any combination of escapes the
    /// bitstream presents and lets the caller validate. The escape
    /// count *is* surfaced (see [`MbAddressIncrement::escape_count`])
    /// so the higher layer can do the bounds check.
    pub fn parse(br: &mut BitReader<'_>, ctx: MbAddressIncrementContext) -> Result<Self> {
        let mut stuffing_count: u8 = 0;
        let mut escape_count: u8 = 0;

        // Phase 1 — consume any MPEG-1 macroblock_stuffing codes.
        // §D.5.5.1 of 11172-2: any number of stuffing codes may
        // precede the increment proper. For MPEG-2 these will not
        // appear; if one does we reject the bitstream.
        loop {
            let entry = match_b1(br)?;
            match entry.symbol {
                TableSymbol::Stuffing => {
                    if !ctx.mpeg1 {
                        return Err(Error::InvalidBitstream(
                            "macroblock_stuffing: code 0000 0001 111 is not defined in ISO/IEC 13818-2 (§6.3.17.1 note)",
                        ));
                    }
                    // Cap on stuffing count: §D.5.5.1 says "any
                    // number" without a hard bound, but a 1-byte
                    // counter would overflow on an 11-bit-per-code
                    // stream of >5800 stuffing codes against a tiny
                    // payload. Saturate rather than panic.
                    stuffing_count = stuffing_count.saturating_add(1);
                    continue;
                }
                TableSymbol::Escape => {
                    escape_count = escape_count.saturating_add(1);
                    break;
                }
                TableSymbol::Value(v) => {
                    return Ok(Self {
                        value: u16::from(v),
                        escape_count: 0,
                        stuffing_count,
                        bit_position_after: br.bit_position(),
                    });
                }
            }
        }

        // Phase 2 — we just consumed one escape. The spec allows
        // chained escapes (e.g. "two macroblock_escape codewords
        // preceding the macroblock_address_increment, then 66 is
        // added"). Keep consuming escapes until we hit a Value.
        // §D.5.5.1 of 11172-2 + §6.3.17.1 of 13818-2 agree on this.
        loop {
            let entry = match_b1(br)?;
            match entry.symbol {
                TableSymbol::Escape => {
                    escape_count = escape_count.checked_add(1).ok_or(Error::InvalidBitstream(
                        "macroblock_escape: chain longer than 255 codewords (§6.3.17.1)",
                    ))?;
                    continue;
                }
                TableSymbol::Value(v) => {
                    // Total increment is 33 * escape_count + v.
                    let value = u16::from(v)
                        .checked_add(u16::from(escape_count).saturating_mul(33))
                        .ok_or(Error::InvalidBitstream(
                            "macroblock_address_increment: composite overflow (§6.3.17.1)",
                        ))?;
                    return Ok(Self {
                        value,
                        escape_count,
                        stuffing_count,
                        bit_position_after: br.bit_position(),
                    });
                }
                TableSymbol::Stuffing => {
                    // Stuffing after an escape is not described by
                    // either spec. Reject — the bitstream is
                    // malformed.
                    return Err(Error::InvalidBitstream(
                        "macroblock_stuffing: appeared after a macroblock_escape (§D.5.5.1 only allows stuffing before the increment)",
                    ));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact Table B-1 round-trips covering every
    //! 33-value entry plus the escape and (MPEG-1) stuffing codes.
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Emit a Table B-1 code into a [`BitWriter`] given the desired
    /// increment value `1..=33`.
    fn write_value_code(bw: &mut BitWriter, value: u8) {
        let entry = TABLE_B1
            .iter()
            .find(|e| matches!(e.symbol, TableSymbol::Value(v) if v == value))
            .expect("value in 1..=33");
        bw.write_u32(u32::from(entry.code), u32::from(entry.bits));
    }

    fn write_escape(bw: &mut BitWriter) {
        bw.write_u32(0b0000_0001_000, 11);
    }

    fn write_stuffing(bw: &mut BitWriter) {
        bw.write_u32(0b0000_0001_111, 11);
    }

    /// Pad the writer with a single `'0'` bit then byte-align so the
    /// BitReader has at least one valid trailing byte to load. The
    /// `'0'` keeps the parser from confusing trailing padding with
    /// new VLC codes.
    fn pad_and_finish(mut bw: BitWriter) -> Vec<u8> {
        bw.write_bit(false);
        bw.align_to_byte();
        bw.finish()
    }

    #[test]
    fn parses_every_value_1_through_33() {
        for v in 1u8..=33 {
            let mut bw = BitWriter::new();
            write_value_code(&mut bw, v);
            let buf = pad_and_finish(bw);
            let mut br = BitReader::new(&buf);
            let mi = MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2())
                .unwrap_or_else(|_| panic!("value {v} should parse"));
            assert_eq!(mi.value, u16::from(v), "value {v}");
            assert_eq!(mi.escape_count, 0);
            assert_eq!(mi.stuffing_count, 0);
        }
    }

    #[test]
    fn parses_single_escape_then_value_1_is_34() {
        // escape + '1' → 33 + 1 = 34.
        let mut bw = BitWriter::new();
        write_escape(&mut bw);
        write_value_code(&mut bw, 1);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let mi = MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2())
            .expect("parse 34");
        assert_eq!(mi.value, 34);
        assert_eq!(mi.escape_count, 1);
        assert_eq!(mi.stuffing_count, 0);
    }

    #[test]
    fn parses_two_escapes_then_value_4_is_70() {
        // Spec example: "An increment of 70 would be coded as two
        // escape codes followed by the code for an increment of 4."
        // (ISO/IEC 11172-2 §D.5.5.2; carried over verbatim in
        // 13818-2 §6.3.17.1.)
        let mut bw = BitWriter::new();
        write_escape(&mut bw);
        write_escape(&mut bw);
        write_value_code(&mut bw, 4);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let mi = MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2())
            .expect("parse 70");
        assert_eq!(mi.value, 70);
        assert_eq!(mi.escape_count, 2);
    }

    #[test]
    fn parses_single_escape_then_value_33_is_66() {
        // Spec example mentioned at §6.3.17.1: two escapes adds 66
        // to the trailing value. We verify a single escape + 33 →
        // 66 here to keep the arithmetic obvious.
        let mut bw = BitWriter::new();
        write_escape(&mut bw);
        write_value_code(&mut bw, 33);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let mi = MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2())
            .expect("parse 66");
        assert_eq!(mi.value, 66);
        assert_eq!(mi.escape_count, 1);
    }

    #[test]
    fn rejects_stuffing_in_mpeg2_context() {
        // ISO/IEC 13818-2 dropped macroblock_stuffing; emitting it
        // is a bitstream violation in MPEG-2.
        let mut bw = BitWriter::new();
        write_stuffing(&mut bw);
        write_value_code(&mut bw, 1);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let err =
            MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2()).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn accepts_leading_stuffing_in_mpeg1_context() {
        let mut bw = BitWriter::new();
        write_stuffing(&mut bw);
        write_stuffing(&mut bw);
        write_stuffing(&mut bw);
        write_value_code(&mut bw, 7);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let mi = MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg1())
            .expect("parse stuffing+7");
        assert_eq!(mi.value, 7);
        assert_eq!(mi.escape_count, 0);
        assert_eq!(mi.stuffing_count, 3);
    }

    #[test]
    fn mpeg1_stuffing_then_escape_then_value() {
        // Stuffing+escape+value chain — stuffing must precede escape
        // per §D.5.5.1.
        let mut bw = BitWriter::new();
        write_stuffing(&mut bw);
        write_escape(&mut bw);
        write_value_code(&mut bw, 5);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let mi = MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg1())
            .expect("parse stuffing+escape+5");
        assert_eq!(mi.value, 38); // 33 + 5
        assert_eq!(mi.escape_count, 1);
        assert_eq!(mi.stuffing_count, 1);
    }

    #[test]
    fn rejects_stuffing_after_escape_even_in_mpeg1() {
        // §D.5.5.1 places stuffing strictly *before* the increment;
        // after an escape the parser is mid-increment and any
        // stuffing is malformed.
        let mut bw = BitWriter::new();
        write_escape(&mut bw);
        write_stuffing(&mut bw);
        write_value_code(&mut bw, 1);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let err =
            MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg1()).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_garbage_prefix() {
        // 11 zero bits is not a Table B-1 code (it would be `0000
        // 0000 000`, which is not in the table).
        let buf = [0u8; 4];
        let mut br = BitReader::new(&buf);
        let err =
            MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2()).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_truncated_buffer() {
        // Half a byte of zeros — cannot complete any 11-bit code.
        let buf = [0u8; 1];
        let mut br = BitReader::new(&buf);
        let err =
            MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2()).unwrap_err();
        // Either ShortHeader (out of bits before a match was found)
        // or InvalidBitstream (matcher decided no entry fits). Both
        // are acceptable failure modes.
        assert!(matches!(
            err,
            Error::ShortHeader | Error::InvalidBitstream(_)
        ));
    }

    #[test]
    fn bit_position_after_tracks_consumption() {
        // value=1 is the 1-bit code '1'. After parsing the bit
        // position must be exactly 1.
        let mut bw = BitWriter::new();
        write_value_code(&mut bw, 1);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let mi = MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2())
            .expect("parse 1");
        assert_eq!(mi.bit_position_after, 1);

        // value=33 is the 11-bit code '0000 0011 000'.
        let mut bw = BitWriter::new();
        write_value_code(&mut bw, 33);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let mi = MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2())
            .expect("parse 33");
        assert_eq!(mi.bit_position_after, 11);

        // Escape + value=1 = 11 + 1 = 12 bits.
        let mut bw = BitWriter::new();
        write_escape(&mut bw);
        write_value_code(&mut bw, 1);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let mi = MbAddressIncrement::parse(&mut br, MbAddressIncrementContext::mpeg2())
            .expect("parse 34");
        assert_eq!(mi.bit_position_after, 12);
    }

    #[test]
    fn table_b1_has_33_value_entries_plus_two_specials() {
        let values = TABLE_B1
            .iter()
            .filter(|e| matches!(e.symbol, TableSymbol::Value(_)))
            .count();
        let escapes = TABLE_B1
            .iter()
            .filter(|e| matches!(e.symbol, TableSymbol::Escape))
            .count();
        let stuffings = TABLE_B1
            .iter()
            .filter(|e| matches!(e.symbol, TableSymbol::Stuffing))
            .count();
        assert_eq!(values, 33);
        assert_eq!(escapes, 1);
        assert_eq!(stuffings, 1);
    }

    #[test]
    fn table_b1_entries_have_unique_bit_strings_per_width() {
        // No two entries of the same bit-width should share the
        // same code, otherwise the linear matcher would emit
        // ambiguous output.
        for &width in &[1u8, 3, 4, 5, 7, 8, 10, 11] {
            let entries: Vec<_> = TABLE_B1.iter().filter(|e| e.bits == width).collect();
            for (i, a) in entries.iter().enumerate() {
                for b in &entries[i + 1..] {
                    assert_ne!(
                        a.code, b.code,
                        "duplicate code {:b} at width {width}",
                        a.code
                    );
                }
            }
        }
    }

    #[test]
    fn table_b1_codes_fit_their_declared_width() {
        // Each `code` must be < 2^bits — otherwise we'd be storing
        // junk in the high bits that peek_u32 will never see.
        for e in TABLE_B1 {
            let max = 1u32 << u32::from(e.bits);
            assert!(
                u32::from(e.code) < max,
                "code {:b} (val {}) does not fit in {} bits",
                e.code,
                e.code,
                e.bits
            );
        }
    }

    #[test]
    fn ctx_helpers_are_correct() {
        assert!(!MbAddressIncrementContext::mpeg2().mpeg1);
        assert!(MbAddressIncrementContext::mpeg1().mpeg1);
        assert!(!MbAddressIncrementContext::default().mpeg1);
    }

    #[test]
    fn debug_impl_smoke() {
        let mi = MbAddressIncrement {
            value: 34,
            escape_count: 1,
            stuffing_count: 0,
            bit_position_after: 12,
        };
        let s = format!("{mi:?}");
        assert!(s.contains("MbAddressIncrement"));
        assert!(s.contains("value"));
    }
}
