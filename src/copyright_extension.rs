//! Parser for the MPEG-2 video `copyright_extension()` element.
//!
//! Implements the bitstream syntax in ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262) §6.2.3.6 and the field semantics in §6.3.15. The
//! extension marks the copyright status of *"the source video material
//! encoded in all the coded pictures following the copyright
//! extension, in coding order, up to the next copyright extension or
//! end of sequence code"* (§6.3.15). Nothing in it affects the
//! decoding process; the parser still consumes the bits exactly so
//! the cursor advances correctly to the trailing `next_start_code()`
//! (§5.2.3) and so a re-encoder can preserve the fields.
//!
//! ## Wire shape (§6.2.3.6)
//!
//! ```text
//! copyright_extension() {
//!     extension_start_code_identifier        4
//!     copyright_flag                         1
//!     copyright_identifier                   8
//!     original_or_copy                       1
//!     reserved                               7
//!     marker_bit                             1
//!     copyright_number_1                    20
//!     marker_bit                             1
//!     copyright_number_2                    22
//!     marker_bit                             1
//!     copyright_number_3                    22
//!     next_start_code()
//! }
//! ```
//!
//! The 32-bit `extension_start_code` (value `0x000001B5` per §6.3.4)
//! precedes the syntax above; the parser consumes the four start-code
//! bytes plus the 4-bit identifier (Table 6-2 entry `0100`, see
//! [`COPYRIGHT_EXTENSION_ID`]) so a caller can hand it a slice
//! starting at the start-code prefix exactly as the other
//! `*_extension()` parsers in this crate expect. The element is 120
//! bits — exactly 15 bytes, so the trailing `next_start_code()`
//! contributes no stuffing bits of its own.
//!
//! ## Semantic constraints enforced (§6.3.15)
//!
//! * `reserved` — *"This is a 7-bit integer, reserved for future
//!   extension. It shall have the value zero."* Non-zero is rejected.
//! * The three `marker_bit`s shall be `'1'`.
//! * *"When `copyright_flag` is set to '0', `copyright_identifier`
//!   has no meaning and shall have the value 0"* and *"When
//!   `copyright_flag` is set to '0', `copyright_number` has no
//!   meaning and shall have the value 0"* — a clear flag with a
//!   non-zero identifier or number is rejected.
//! * *"The value of `copyright_number` shall be zero when
//!   `copyright_identifier` is equal to zero"* — a zero identifier
//!   (*"this information is not available"*) with a non-zero number
//!   is rejected.
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)).

use oxideav_core::bits::BitReader;

use crate::sequence_extension::EXTENSION_START_CODE;
use crate::{Error, Result};

/// `extension_start_code_identifier` value for
/// `copyright_extension()` (Table 6-2 entry `0100`).
pub const COPYRIGHT_EXTENSION_ID: u32 = 0b0100;

/// Parsed `copyright_extension()` (§6.2.3.6 / §6.3.15).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CopyrightExtension {
    /// `copyright_flag` — `'1'`: the source material of the pictures
    /// this extension governs *"is copyrighted"*, identified by
    /// `copyright_identifier` + `copyright_number`. `'0'`: *"it does
    /// not indicate whether the source video material … is
    /// copyrighted or not"* (§6.3.15).
    pub copyright_flag: bool,
    /// `copyright_identifier` — 8-bit integer identifying *"a
    /// Registration Authority as designated by ISO/IEC JTC1/SC29"*;
    /// *"Value zero indicates that this information is not
    /// available"* (§6.3.15).
    pub copyright_identifier: u8,
    /// `original_or_copy` — `'1'`: *"the material is an original"*;
    /// `'0'`: *"it is a copy"* (§6.3.15).
    pub original_or_copy: bool,
    /// `copyright_number_1` — 20-bit *"bits 44 to 63 of
    /// `copyright_number`"* (§6.3.15).
    pub copyright_number_1: u32,
    /// `copyright_number_2` — 22-bit *"bits 22 to 43 of
    /// `copyright_number`"* (§6.3.15).
    pub copyright_number_2: u32,
    /// `copyright_number_3` — 22-bit *"bits 0 to 21 of
    /// `copyright_number`"* (§6.3.15).
    pub copyright_number_3: u32,
}

impl CopyrightExtension {
    /// Parse a `copyright_extension()` from a slice starting with the
    /// four start-code bytes `00 00 01 B5`. The element is exactly 15
    /// bytes; the trailing `next_start_code()` (§5.2.3) is not
    /// consumed.
    pub fn parse(buf: &[u8]) -> Result<Self> {
        let mut br = BitReader::new(buf);
        Self::parse_with_reader(&mut br)
    }

    /// Parse from an existing [`BitReader`] positioned at the start of
    /// `copyright_extension()` (i.e. its 32-bit
    /// `extension_start_code`).
    pub fn parse_with_reader(br: &mut BitReader<'_>) -> Result<Self> {
        // §6.2.3.6 / §6.3.4: extension_start_code = 0x000001B5.
        let code = br.read_u32(32).map_err(|_| Error::ShortHeader)?;
        if code != EXTENSION_START_CODE {
            return Err(Error::InvalidBitstream(
                "extension_start_code: expected 0x000001B5 (§6.3.4)",
            ));
        }
        // 4-bit extension_start_code_identifier; Copyright Extension
        // ID is '0100' per Table 6-2.
        let id = br.read_u32(4).map_err(|_| Error::ShortHeader)?;
        if id != COPYRIGHT_EXTENSION_ID {
            return Err(Error::InvalidBitstream(
                "extension_start_code_identifier: expected '0100' Copyright Extension ID (Table 6-2)",
            ));
        }

        let copyright_flag = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let copyright_identifier = br.read_u32(8).map_err(|_| Error::ShortHeader)? as u8;
        let original_or_copy = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;

        // §6.3.15: reserved "shall have the value zero".
        let reserved = br.read_u32(7).map_err(|_| Error::ShortHeader)?;
        if reserved != 0 {
            return Err(Error::InvalidBitstream(
                "reserved: shall have the value zero (§6.3.15)",
            ));
        }

        let read_marker = |br: &mut BitReader<'_>| -> Result<()> {
            if br.read_u32(1).map_err(|_| Error::ShortHeader)? != 1 {
                return Err(Error::InvalidBitstream(
                    "marker_bit in copyright_extension() (§6.2.3.6)",
                ));
            }
            Ok(())
        };
        read_marker(br)?;
        let copyright_number_1 = br.read_u32(20).map_err(|_| Error::ShortHeader)?;
        read_marker(br)?;
        let copyright_number_2 = br.read_u32(22).map_err(|_| Error::ShortHeader)?;
        read_marker(br)?;
        let copyright_number_3 = br.read_u32(22).map_err(|_| Error::ShortHeader)?;

        let out = Self {
            copyright_flag,
            copyright_identifier,
            original_or_copy,
            copyright_number_1,
            copyright_number_2,
            copyright_number_3,
        };

        // §6.3.15: with copyright_flag '0' both copyright_identifier
        // and copyright_number "shall have the value 0".
        if !copyright_flag && (copyright_identifier != 0 || out.copyright_number() != 0) {
            return Err(Error::InvalidBitstream(
                "copyright_identifier / copyright_number: shall be 0 when copyright_flag is '0' (§6.3.15)",
            ));
        }
        // §6.3.15: "The value of copyright_number shall be zero when
        // copyright_identifier is equal to zero."
        if copyright_identifier == 0 && out.copyright_number() != 0 {
            return Err(Error::InvalidBitstream(
                "copyright_number: shall be zero when copyright_identifier is zero (§6.3.15)",
            ));
        }

        Ok(out)
    }

    /// The 64-bit `copyright_number` per the §6.3.15 derivation:
    /// *"`copyright_number` = (`copyright_number_1` << 44) +
    /// (`copyright_number_2` << 22) + `copyright_number_3`"*. Its
    /// meaning *"is defined only when `copyright_flag` is set to
    /// '1'"*; the value `0` indicates the identification number *"is
    /// not available"*.
    pub fn copyright_number(&self) -> u64 {
        (u64::from(self.copyright_number_1) << 44)
            + (u64::from(self.copyright_number_2) << 22)
            + u64::from(self.copyright_number_3)
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact `copyright_extension()` fixtures plus
    //! negative cases for every §6.2.3.6 / §6.3.15 rejection site
    //! this parser introduces.
    use super::*;
    use oxideav_core::bits::BitWriter;

    struct Fixture {
        flag: bool,
        identifier: u8,
        original: bool,
        reserved: u32,
        markers: [bool; 3],
        numbers: (u32, u32, u32),
    }

    impl Default for Fixture {
        fn default() -> Self {
            Self {
                flag: false,
                identifier: 0,
                original: true,
                reserved: 0,
                markers: [true; 3],
                numbers: (0, 0, 0),
            }
        }
    }

    fn build(fx: &Fixture) -> Vec<u8> {
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(COPYRIGHT_EXTENSION_ID, 4);
        bw.write_bit(fx.flag);
        bw.write_u32(fx.identifier as u32, 8);
        bw.write_bit(fx.original);
        bw.write_u32(fx.reserved, 7);
        bw.write_bit(fx.markers[0]);
        bw.write_u32(fx.numbers.0, 20);
        bw.write_bit(fx.markers[1]);
        bw.write_u32(fx.numbers.1, 22);
        bw.write_bit(fx.markers[2]);
        bw.write_u32(fx.numbers.2, 22);
        bw.finish()
    }

    // ---- Positive wire parses --------------------------------------

    #[test]
    fn parses_copyrighted_work() {
        let bytes = build(&Fixture {
            flag: true,
            identifier: 0x12,
            original: true,
            numbers: (0xABCDE, 0x155555, 0x2AAAAA),
            ..Fixture::default()
        });
        let ext = CopyrightExtension::parse(&bytes).expect("parse");
        assert!(ext.copyright_flag);
        assert_eq!(ext.copyright_identifier, 0x12);
        assert!(ext.original_or_copy);
        assert_eq!(ext.copyright_number_1, 0xABCDE);
        assert_eq!(ext.copyright_number_2, 0x155555);
        assert_eq!(ext.copyright_number_3, 0x2AAAAA);
    }

    #[test]
    fn parses_unasserted_copyright_status() {
        // copyright_flag '0' with identifier and number zero —
        // §6.3.15's only conformant shape for a clear flag.
        let bytes = build(&Fixture {
            original: false,
            ..Fixture::default()
        });
        let ext = CopyrightExtension::parse(&bytes).expect("parse");
        assert!(!ext.copyright_flag);
        assert_eq!(ext.copyright_identifier, 0);
        assert!(!ext.original_or_copy);
        assert_eq!(ext.copyright_number(), 0);
    }

    #[test]
    fn parses_flagged_work_with_unavailable_identification() {
        // §6.3.15: identifier 0 = "this information is not available";
        // copyright_number shall then be 0 too.
        let bytes = build(&Fixture {
            flag: true,
            ..Fixture::default()
        });
        let ext = CopyrightExtension::parse(&bytes).expect("parse");
        assert!(ext.copyright_flag);
        assert_eq!(ext.copyright_identifier, 0);
        assert_eq!(ext.copyright_number(), 0);
    }

    #[test]
    fn copyright_number_concatenation() {
        // §6.3.15: number = (n1 << 44) + (n2 << 22) + n3, with n1 the
        // most significant 20 bits of the 64-bit value.
        let bytes = build(&Fixture {
            flag: true,
            identifier: 1,
            numbers: (0xFFFFF, 0x3FFFFF, 0x3FFFFF),
            ..Fixture::default()
        });
        let ext = CopyrightExtension::parse(&bytes).expect("parse");
        assert_eq!(ext.copyright_number(), 0xFFFF_FFFF_FFFF_FFFF);

        let bytes = build(&Fixture {
            flag: true,
            identifier: 1,
            numbers: (1, 2, 3),
            ..Fixture::default()
        });
        let ext = CopyrightExtension::parse(&bytes).expect("parse");
        assert_eq!(ext.copyright_number(), (1u64 << 44) + (2u64 << 22) + 3);
    }

    #[test]
    fn encoded_length_is_fifteen_bytes() {
        // 32 + 4 + 1 + 8 + 1 + 7 + 1 + 20 + 1 + 22 + 1 + 22 = 120
        // bits — the element is byte-aligned by itself.
        assert_eq!(build(&Fixture::default()).len(), 15);
    }

    // ---- Rejection sites -------------------------------------------

    #[test]
    fn rejects_wrong_extension_start_code() {
        let mut bytes = build(&Fixture::default());
        bytes[3] = 0xB3; // sequence_header_code instead
        assert!(matches!(
            CopyrightExtension::parse(&bytes),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn rejects_wrong_extension_identifier() {
        // Identifier '0011' (Quant Matrix Extension ID) instead of
        // '0100'.
        let mut bytes = build(&Fixture::default());
        bytes[4] = (bytes[4] & 0x0F) | 0b0011 << 4;
        assert!(matches!(
            CopyrightExtension::parse(&bytes),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn rejects_non_zero_reserved_field() {
        let bytes = build(&Fixture {
            reserved: 0b0000001,
            ..Fixture::default()
        });
        assert!(matches!(
            CopyrightExtension::parse(&bytes),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn rejects_each_zero_marker_bit() {
        for zeroed in 0..3 {
            let mut markers = [true; 3];
            markers[zeroed] = false;
            let bytes = build(&Fixture {
                markers,
                ..Fixture::default()
            });
            assert!(
                matches!(
                    CopyrightExtension::parse(&bytes),
                    Err(Error::InvalidBitstream(_))
                ),
                "marker index {zeroed}"
            );
        }
    }

    #[test]
    fn rejects_clear_flag_with_non_zero_identifier() {
        let bytes = build(&Fixture {
            identifier: 7,
            ..Fixture::default()
        });
        assert!(matches!(
            CopyrightExtension::parse(&bytes),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn rejects_clear_flag_with_non_zero_number() {
        // Each of the three number fields violates the rule alone.
        for numbers in [(1, 0, 0), (0, 1, 0), (0, 0, 1)] {
            let bytes = build(&Fixture {
                numbers,
                ..Fixture::default()
            });
            assert!(
                matches!(
                    CopyrightExtension::parse(&bytes),
                    Err(Error::InvalidBitstream(_))
                ),
                "numbers {numbers:?}"
            );
        }
    }

    #[test]
    fn rejects_zero_identifier_with_non_zero_number() {
        // §6.3.15: "The value of copyright_number shall be zero when
        // copyright_identifier is equal to zero."
        let bytes = build(&Fixture {
            flag: true,
            numbers: (0, 0, 5),
            ..Fixture::default()
        });
        assert!(matches!(
            CopyrightExtension::parse(&bytes),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn short_buffer_returns_short_header() {
        let bytes = [0u8, 0u8];
        assert!(matches!(
            CopyrightExtension::parse(&bytes),
            Err(Error::ShortHeader)
        ));

        // Truncated mid-payload: cut inside copyright_number_2.
        let full = build(&Fixture::default());
        assert!(matches!(
            CopyrightExtension::parse(&full[..10]),
            Err(Error::ShortHeader)
        ));
    }
}
