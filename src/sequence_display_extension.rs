//! Parser for the MPEG-2 video `sequence_display_extension()` element.
//!
//! Implements the bitstream syntax in ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262) §6.2.2.4 and the field semantics in §6.3.6. The
//! sequence-display extension describes the source material (video
//! format + colourimetry) and the intended display's active rectangle;
//! §6.3.6 opens by observing that *"This specification does not define
//! the display process. The information in this extension does not
//! affect the decoding process and may be ignored by decoders that
//! conform to this specification."* The parser still consumes the bits
//! exactly so the cursor advances correctly to the trailing
//! `next_start_code()` (§5.2.3) and so a re-encoder can preserve the
//! fields.
//!
//! ## Wire shape (§6.2.2.4)
//!
//! ```text
//! sequence_display_extension() {
//!     extension_start_code_identifier        4
//!     video_format                           3
//!     colour_description                     1
//!     if ( colour_description ) {
//!         colour_primaries                   8
//!         transfer_characteristics           8
//!         matrix_coefficients                8
//!     }
//!     display_horizontal_size               14
//!     marker_bit                             1
//!     display_vertical_size                 14
//!     next_start_code()
//! }
//! ```
//!
//! The 32-bit `extension_start_code` (value `0x000001B5` per §6.3.4)
//! precedes the syntax above; the parser consumes the four start-code
//! bytes plus the 4-bit identifier (Table 6-2 entry `0010`, see
//! [`SEQUENCE_DISPLAY_EXTENSION_ID`]) so a caller can hand it a slice
//! starting at the start-code prefix exactly as the other
//! `*_extension()` parsers in this crate expect.
//!
//! ## Occurrence constraint (§6.3.5)
//!
//! §6.3.5 binds the extension's appearance across repeat sequence
//! headers: *"If a `sequence_display_extension()` occurs after the
//! first `sequence_header()` all subsequent sequence headers shall be
//! followed by `sequence_display_extension()` in which all data
//! elements are the same as in the first
//! `sequence_display_extension()`. Conversely if no
//! `sequence_display_extension()` occurs between the first
//! `sequence_header()` and the first `picture_header()` then
//! `sequence_display_extension()` shall not occur in the bitstream."*
//! That cross-element rule is enforced by
//! [`crate::SequenceDisplayOrderDriver`]; this module supplies the
//! parsed value the driver compares. The §6.3.12 ordering constraint
//! on `picture_display_extension()` (*"… shall not occur unless a
//! `sequence_display_extension()` followed the previous
//! `sequence_header()`"*) keys off the same presence fact, also
//! enforced by [`crate::SequenceDisplayOrderDriver`].
//!
//! ## Defaults when the extension is absent (§6.3.6)
//!
//! §6.3.6 names a default for every described property when the
//! extension is missing (or `colour_description == 0`):
//!
//! * `video_format` *"may be assumed to be 'Unspecified video
//!   format'"* — [`VideoFormat::Unspecified`], also the
//!   `Default::default()` of [`VideoFormat`].
//! * `colour_primaries` / `transfer_characteristics` /
//!   `matrix_coefficients` are each *"assumed to be that corresponding
//!   to … having the value 1"* — [`ColourDescription::ASSUMED`].
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)).

use oxideav_core::bits::BitReader;

use crate::sequence_extension::EXTENSION_START_CODE;
use crate::{Error, Result};

/// `extension_start_code_identifier` value for
/// `sequence_display_extension()` (Table 6-2 entry `0010`).
pub const SEQUENCE_DISPLAY_EXTENSION_ID: u32 = 0b0010;

/// `video_format` per Table 6-6 — *"a three bit integer indicating the
/// representation of the pictures before being coded in accordance
/// with this specification"* (§6.3.6).
///
/// Codes `110` and `111` are reserved by Table 6-6; they are preserved
/// as [`VideoFormat::Reserved`] with the raw 3-bit value so the caller
/// has the original code available (the same policy as
/// [`crate::AspectRatio::Reserved`], since §6.3.6 says the field does
/// not affect the decoding process).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum VideoFormat {
    /// `000` — component.
    Component,
    /// `001` — PAL.
    Pal,
    /// `010` — NTSC.
    Ntsc,
    /// `011` — SECAM.
    Secam,
    /// `100` — MAC.
    Mac,
    /// `101` — *"Unspecified video format"*. Also the §6.3.6 value to
    /// assume when no `sequence_display_extension()` is present, hence
    /// the `Default` derive marks this variant.
    #[default]
    Unspecified,
    /// `110` / `111` — reserved by Table 6-6; raw code preserved.
    Reserved(u8),
}

impl VideoFormat {
    /// Decode a 3-bit `video_format` code per Table 6-6.
    pub fn from_code(code: u8) -> Self {
        match code {
            0b000 => Self::Component,
            0b001 => Self::Pal,
            0b010 => Self::Ntsc,
            0b011 => Self::Secam,
            0b100 => Self::Mac,
            0b101 => Self::Unspecified,
            other => Self::Reserved(other),
        }
    }
}

/// The optional colourimetry triple carried when
/// `colour_description == 1` (§6.2.2.4).
///
/// Each component is the raw 8-bit integer the wire carries; the value
/// `0` is the *"(forbidden)"* row of each defining table and is
/// rejected at parse time. Values above the highest described row are
/// *"reserved"* by the same tables and are preserved raw (the field
/// set does not affect the decoding process per §6.3.6).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ColourDescription {
    /// `colour_primaries` — *"describes the chromaticity coordinates
    /// of the source primaries"*, Table 6-7 (`1` = Rec. ITU-R BT.709;
    /// `0` forbidden; `3`, `8..=255` reserved).
    pub colour_primaries: u8,
    /// `transfer_characteristics` — *"describes the opto-electronic
    /// transfer characteristic of the source picture"*, Table 6-8
    /// (`1` = Rec. ITU-R BT.709; `0` forbidden; `3`, `9..=255`
    /// reserved).
    pub transfer_characteristics: u8,
    /// `matrix_coefficients` — *"describes the matrix coefficients
    /// used in deriving luminance and chrominance signals from the
    /// green, blue, and red primaries"*, Table 6-9 (`1` = Rec. ITU-R
    /// BT.709; `0` forbidden; `3`, `8..=255` reserved).
    pub matrix_coefficients: u8,
}

impl ColourDescription {
    /// The §6.3.6 absence default: *"In the case that
    /// `sequence_display_extension()` is not present in the bitstream
    /// or `colour_description` is zero the …"* chromaticity, transfer
    /// characteristics, and matrix coefficients *"are assumed to be
    /// those corresponding to … having the value 1"* (stated once per
    /// field under Tables 6-7, 6-8, and 6-9 respectively).
    pub const ASSUMED: Self = Self {
        colour_primaries: 1,
        transfer_characteristics: 1,
        matrix_coefficients: 1,
    };
}

/// Parsed `sequence_display_extension()` (§6.2.2.4 / §6.3.6).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SequenceDisplayExtension {
    /// `video_format` per Table 6-6.
    pub video_format: VideoFormat,
    /// The colourimetry triple, present iff the wire's
    /// `colour_description` flag was `1`. When `None`, §6.3.6 says to
    /// assume [`ColourDescription::ASSUMED`] — see
    /// [`Self::effective_colour_description`].
    pub colour_description: Option<ColourDescription>,
    /// `display_horizontal_size` — 14-bit width of the *"intended
    /// display's"* active region, *"in the same units as
    /// `horizontal_size` (samples of the encoded frames)"* (§6.3.6).
    pub display_horizontal_size: u16,
    /// `display_vertical_size` — 14-bit height of the intended
    /// display's active region, *"in the same units as `vertical_size`
    /// (lines of the encoded frames)"* (§6.3.6).
    pub display_vertical_size: u16,
}

impl SequenceDisplayExtension {
    /// Parse a `sequence_display_extension()` from a slice starting
    /// with the four start-code bytes `00 00 01 B5`. The trailing
    /// `next_start_code()` (§5.2.3) byte-align is not consumed.
    pub fn parse(buf: &[u8]) -> Result<Self> {
        let mut br = BitReader::new(buf);
        Self::parse_with_reader(&mut br)
    }

    /// Parse from an existing [`BitReader`] positioned at the start of
    /// `sequence_display_extension()` (i.e. its 32-bit
    /// `extension_start_code`).
    pub fn parse_with_reader(br: &mut BitReader<'_>) -> Result<Self> {
        // §6.2.2.4 / §6.3.4: extension_start_code = 0x000001B5.
        let code = br.read_u32(32).map_err(|_| Error::ShortHeader)?;
        if code != EXTENSION_START_CODE {
            return Err(Error::InvalidBitstream(
                "extension_start_code: expected 0x000001B5 (§6.3.4)",
            ));
        }
        // 4-bit extension_start_code_identifier; Sequence Display
        // Extension ID is '0010' per Table 6-2.
        let id = br.read_u32(4).map_err(|_| Error::ShortHeader)?;
        if id != SEQUENCE_DISPLAY_EXTENSION_ID {
            return Err(Error::InvalidBitstream(
                "extension_start_code_identifier: expected '0010' Sequence Display Extension ID (Table 6-2)",
            ));
        }

        // 3-bit video_format (Table 6-6).
        let video_format =
            VideoFormat::from_code(br.read_u32(3).map_err(|_| Error::ShortHeader)? as u8);

        // 1-bit colour_description flag — "if set to '1' indicates the
        // presence of colour_primaries, transfer_characteristics and
        // matrix_coefficients in the bitstream" (§6.3.6).
        let colour_description = if br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1 {
            let colour_primaries = br.read_u32(8).map_err(|_| Error::ShortHeader)? as u8;
            if colour_primaries == 0 {
                return Err(Error::InvalidBitstream(
                    "colour_primaries: value 0 is forbidden (Table 6-7)",
                ));
            }
            let transfer_characteristics = br.read_u32(8).map_err(|_| Error::ShortHeader)? as u8;
            if transfer_characteristics == 0 {
                return Err(Error::InvalidBitstream(
                    "transfer_characteristics: value 0 is forbidden (Table 6-8)",
                ));
            }
            let matrix_coefficients = br.read_u32(8).map_err(|_| Error::ShortHeader)? as u8;
            if matrix_coefficients == 0 {
                return Err(Error::InvalidBitstream(
                    "matrix_coefficients: value 0 is forbidden (Table 6-9)",
                ));
            }
            Some(ColourDescription {
                colour_primaries,
                transfer_characteristics,
                matrix_coefficients,
            })
        } else {
            None
        };

        // 14-bit display_horizontal_size, marker_bit, 14-bit
        // display_vertical_size (§6.2.2.4).
        let display_horizontal_size = br.read_u32(14).map_err(|_| Error::ShortHeader)? as u16;
        let marker = br.read_u32(1).map_err(|_| Error::ShortHeader)?;
        if marker != 1 {
            return Err(Error::InvalidBitstream(
                "marker_bit after display_horizontal_size (§6.2.2.4)",
            ));
        }
        let display_vertical_size = br.read_u32(14).map_err(|_| Error::ShortHeader)? as u16;

        // §6.2.2.4: 32-bit start code + 4-bit identifier + 3 + 1
        // (+ 24 when colour_description) + 14 + 1 + 14 = 69 or 93
        // bits — neither byte-aligned. The trailing next_start_code()
        // (§5.2.3) supplies the zero stuffing back to a byte
        // boundary; we therefore do NOT assert byte-alignment here.
        Ok(Self {
            video_format,
            colour_description,
            display_horizontal_size,
            display_vertical_size,
        })
    }

    /// The colourimetry triple after applying the §6.3.6 absence rule:
    /// the parsed triple when `colour_description` was `1`, otherwise
    /// [`ColourDescription::ASSUMED`] (every component *"assumed to be
    /// … having the value 1"*). A caller modelling a stream with no
    /// `sequence_display_extension()` at all gets the same answer by
    /// reading `ColourDescription::ASSUMED` directly.
    pub fn effective_colour_description(&self) -> ColourDescription {
        self.colour_description
            .unwrap_or(ColourDescription::ASSUMED)
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact `sequence_display_extension()` fixtures
    //! plus negative cases for every §6.2.2.4 / §6.3.6 rejection site
    //! this parser introduces.
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Emit a `sequence_display_extension()` with the given fields.
    fn write_sequence_display_extension(
        bw: &mut BitWriter,
        video_format: u8,
        colour: Option<(u8, u8, u8)>,
        dhs: u16,
        dvs: u16,
    ) {
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(SEQUENCE_DISPLAY_EXTENSION_ID, 4);
        bw.write_u32(video_format as u32, 3);
        match colour {
            Some((cp, tc, mc)) => {
                bw.write_bit(true);
                bw.write_u32(cp as u32, 8);
                bw.write_u32(tc as u32, 8);
                bw.write_u32(mc as u32, 8);
            }
            None => bw.write_bit(false),
        }
        bw.write_u32(dhs as u32, 14);
        bw.write_bit(true);
        bw.write_u32(dvs as u32, 14);
        bw.align_to_byte();
    }

    fn build(video_format: u8, colour: Option<(u8, u8, u8)>, dhs: u16, dvs: u16) -> Vec<u8> {
        let mut bw = BitWriter::new();
        write_sequence_display_extension(&mut bw, video_format, colour, dhs, dvs);
        bw.finish()
    }

    // ---- Positive wire parses --------------------------------------

    #[test]
    fn parses_without_colour_description() {
        let bytes = build(0b010, None, 352, 240);
        let ext = SequenceDisplayExtension::parse(&bytes).expect("parse");
        assert_eq!(ext.video_format, VideoFormat::Ntsc);
        assert_eq!(ext.colour_description, None);
        assert_eq!(ext.display_horizontal_size, 352);
        assert_eq!(ext.display_vertical_size, 240);
    }

    #[test]
    fn parses_with_colour_description() {
        // Table 6-7 value 5 (Rec. ITU-R BT.470-2 System B, G), Table
        // 6-8 value 4 (System M, assumed display gamma 2,2), Table 6-9
        // value 5 (System B, G).
        let bytes = build(0b001, Some((5, 4, 5)), 720, 576);
        let ext = SequenceDisplayExtension::parse(&bytes).expect("parse");
        assert_eq!(ext.video_format, VideoFormat::Pal);
        assert_eq!(
            ext.colour_description,
            Some(ColourDescription {
                colour_primaries: 5,
                transfer_characteristics: 4,
                matrix_coefficients: 5,
            })
        );
        assert_eq!(ext.display_horizontal_size, 720);
        assert_eq!(ext.display_vertical_size, 576);
    }

    #[test]
    fn parses_every_described_video_format() {
        // Table 6-6 rows 000..=101.
        let expect = [
            (0b000u8, VideoFormat::Component),
            (0b001, VideoFormat::Pal),
            (0b010, VideoFormat::Ntsc),
            (0b011, VideoFormat::Secam),
            (0b100, VideoFormat::Mac),
            (0b101, VideoFormat::Unspecified),
        ];
        for (code, vf) in expect {
            let bytes = build(code, None, 1, 1);
            let ext = SequenceDisplayExtension::parse(&bytes).expect("parse");
            assert_eq!(ext.video_format, vf, "code {code:#05b}");
        }
    }

    #[test]
    fn preserves_reserved_video_format_codes() {
        // Table 6-6 rows 110 / 111 are reserved; the raw code is
        // preserved (the field does not affect decoding per §6.3.6).
        for code in [0b110u8, 0b111] {
            let bytes = build(code, None, 1, 1);
            let ext = SequenceDisplayExtension::parse(&bytes).expect("parse");
            assert_eq!(ext.video_format, VideoFormat::Reserved(code));
        }
    }

    #[test]
    fn parses_maximum_display_sizes() {
        // Both display sizes are 14-bit fields; 0x3FFF round-trips.
        let bytes = build(0b000, None, 0x3FFF, 0x3FFF);
        let ext = SequenceDisplayExtension::parse(&bytes).expect("parse");
        assert_eq!(ext.display_horizontal_size, 0x3FFF);
        assert_eq!(ext.display_vertical_size, 0x3FFF);
    }

    #[test]
    fn parses_reserved_colour_values_above_described_rows() {
        // Tables 6-7 / 6-8 / 6-9 mark 8-255 (9-255 for Table 6-8)
        // reserved, not forbidden — the raw bytes are preserved.
        let bytes = build(0b101, Some((255, 9, 8)), 100, 100);
        let ext = SequenceDisplayExtension::parse(&bytes).expect("parse");
        assert_eq!(
            ext.colour_description,
            Some(ColourDescription {
                colour_primaries: 255,
                transfer_characteristics: 9,
                matrix_coefficients: 8,
            })
        );
    }

    // ---- Encoded-length accounting ---------------------------------

    #[test]
    fn encoded_length_without_colour_description() {
        // 32 + 4 + 3 + 1 + 14 + 1 + 14 = 69 bits = 8 bytes + 5 bits;
        // the writer pads to byte boundary -> 9 bytes.
        let bytes = build(0b000, None, 0, 0);
        assert_eq!(bytes.len(), 9);
    }

    #[test]
    fn encoded_length_with_colour_description() {
        // 69 + 24 = 93 bits = 11 bytes + 5 bits; padded -> 12 bytes.
        let bytes = build(0b000, Some((1, 1, 1)), 0, 0);
        assert_eq!(bytes.len(), 12);
    }

    // ---- Rejection sites -------------------------------------------

    #[test]
    fn rejects_wrong_extension_start_code() {
        let mut bytes = build(0b000, None, 1, 1);
        bytes[3] = 0xB3; // sequence_header_code instead
        let err = SequenceDisplayExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_wrong_extension_identifier() {
        // Identifier '0001' (Sequence Extension ID) instead of '0010'.
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(0b0001, 4);
        bw.write_u32(0, 3);
        bw.write_bit(false);
        bw.write_u32(1, 14);
        bw.write_bit(true);
        bw.write_u32(1, 14);
        bw.align_to_byte();
        let bytes = bw.finish();
        let err = SequenceDisplayExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_forbidden_zero_colour_primaries() {
        let bytes = build(0b000, Some((0, 1, 1)), 1, 1);
        let err = SequenceDisplayExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_forbidden_zero_transfer_characteristics() {
        let bytes = build(0b000, Some((1, 0, 1)), 1, 1);
        let err = SequenceDisplayExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_forbidden_zero_matrix_coefficients() {
        let bytes = build(0b000, Some((1, 1, 0)), 1, 1);
        let err = SequenceDisplayExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_zero_marker_bit() {
        // Build manually with the marker_bit forced to '0'.
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(SEQUENCE_DISPLAY_EXTENSION_ID, 4);
        bw.write_u32(0, 3);
        bw.write_bit(false);
        bw.write_u32(1, 14);
        bw.write_bit(false); // forbidden: marker_bit must be '1'
        bw.write_u32(1, 14);
        bw.align_to_byte();
        let bytes = bw.finish();
        let err = SequenceDisplayExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn short_buffer_returns_short_header() {
        let bytes = [0u8, 0u8];
        let err = SequenceDisplayExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));

        // Also truncated mid-payload: cut after the identifier nibble.
        let full = build(0b000, Some((1, 1, 1)), 1, 1);
        let err = SequenceDisplayExtension::parse(&full[..5]).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    // ---- §6.3.6 absence defaults -----------------------------------

    #[test]
    fn video_format_default_is_unspecified() {
        // §6.3.6: "If the sequence_display_extension() is not present
        // in the bitstream then the video format may be assumed to be
        // 'Unspecified video format'."
        assert_eq!(VideoFormat::default(), VideoFormat::Unspecified);
        assert_eq!(VideoFormat::from_code(0b101), VideoFormat::default());
    }

    #[test]
    fn assumed_colour_description_is_all_value_one() {
        // §6.3.6 under Tables 6-7 / 6-8 / 6-9: absence (or
        // colour_description == 0) means every component is assumed
        // to be the table's value-1 row.
        assert_eq!(
            ColourDescription::ASSUMED,
            ColourDescription {
                colour_primaries: 1,
                transfer_characteristics: 1,
                matrix_coefficients: 1,
            }
        );
    }

    #[test]
    fn effective_colour_description_applies_absence_rule() {
        // colour_description == 0 -> the §6.3.6 assumed triple.
        let bytes = build(0b000, None, 1, 1);
        let ext = SequenceDisplayExtension::parse(&bytes).expect("parse");
        assert_eq!(
            ext.effective_colour_description(),
            ColourDescription::ASSUMED
        );

        // colour_description == 1 -> the parsed triple wins.
        let bytes = build(0b000, Some((6, 6, 6)), 1, 1);
        let ext = SequenceDisplayExtension::parse(&bytes).expect("parse");
        assert_eq!(
            ext.effective_colour_description(),
            ColourDescription {
                colour_primaries: 6,
                transfer_characteristics: 6,
                matrix_coefficients: 6,
            }
        );
    }
}
