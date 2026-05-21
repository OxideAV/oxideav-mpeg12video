//! Parser for the MPEG-1 / MPEG-2 video sequence header.
//!
//! Implements the bitstream syntax in ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262) §6.2.2.1 and the field semantics defined in §6.3.3.
//!
//! Scope is deliberately structural: this module returns the decoded
//! field values from a single `sequence_header()` element. It does
//! not chain into `sequence_extension()` or any picture-layer parser.
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)).

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// The 32-bit start code that introduces a `sequence_header()`:
/// the byte string `00 00 01 B3` (§6.3.3, §6.2.2.1).
pub const SEQUENCE_HEADER_CODE: u32 = 0x0000_01B3;

/// Sample / display aspect ratio code from Table 6-3.
///
/// Code `0000` is the spec-defined `forbidden` value and is rejected
/// during parsing. Codes `0101..=1111` are spec-reserved and are
/// surfaced as [`AspectRatio::Reserved`] so the caller can decide
/// whether to reject the stream or proceed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AspectRatio {
    /// `0001` — Sample Aspect Ratio is `1,0` (square samples).
    Square,
    /// `0010` — Display Aspect Ratio is `3 ÷ 4`.
    Dar3x4,
    /// `0011` — Display Aspect Ratio is `9 ÷ 16`.
    Dar9x16,
    /// `0100` — Display Aspect Ratio is `1 ÷ 2,21`.
    Dar1x221,
    /// `0101..=1111` — reserved by the spec; preserved as raw 4-bit
    /// value so the caller has the original code available.
    Reserved(u8),
}

impl AspectRatio {
    fn from_code(code: u32) -> Result<Self> {
        Ok(match code {
            0b0000 => {
                return Err(Error::InvalidBitstream(
                    "aspect_ratio_information: forbidden value 0000 (Table 6-3)",
                ));
            }
            0b0001 => Self::Square,
            0b0010 => Self::Dar3x4,
            0b0011 => Self::Dar9x16,
            0b0100 => Self::Dar1x221,
            other => Self::Reserved(other as u8),
        })
    }
}

/// Parsed result of `sequence_header()` (ISO/IEC 13818-2 §6.2.2.1).
///
/// The fields are stored as the lower-bits values that the bare
/// sequence header carries. The companion `sequence_extension()`
/// (§6.2.2.3) supplies the upper bits of `horizontal_size`,
/// `vertical_size`, `bit_rate`, and `vbv_buffer_size`; combining the
/// two layers is the job of a higher-level parser, not this module.
#[derive(Debug, Clone)]
pub struct Mpeg2SequenceHeader {
    /// Width of the displayable luminance component, in samples
    /// (low 12 bits of `horizontal_size`). Guaranteed non-zero
    /// because §6.3.3 forbids `horizontal_size_value == 0` (start-code
    /// emulation prevention).
    pub width: u16,
    /// Height of the displayable luminance component, in lines
    /// (low 12 bits of `vertical_size`). Guaranteed non-zero per
    /// §6.3.3.
    pub height: u16,
    /// Decoded `aspect_ratio_information` (Table 6-3).
    pub aspect_ratio: AspectRatio,
    /// Raw 4-bit `frame_rate_code` (Table 6-4). The actual
    /// `frame_rate_value` lookup is left to the caller because it
    /// depends on `frame_rate_extension_n/_d` from
    /// `sequence_extension()`.
    pub frame_rate_code: u8,
    /// `bit_rate` low 18 bits — measured in units of 400 bit/s
    /// (§6.3.3). Combined with `bit_rate_extension` (12 bits) from
    /// `sequence_extension()` to form the full 30-bit `bit_rate`.
    /// Guaranteed non-zero by §6.3.3.
    pub bit_rate: u32,
    /// `vbv_buffer_size` low 10 bits. Combined with the 8-bit
    /// `vbv_buffer_size_extension` to form an 18-bit field;
    /// `B = 16 * 1024 * vbv_buffer_size` bits.
    pub vbv_buffer_size: u16,
    /// `constrained_parameters_flag` — has no meaning in 13818-2
    /// (§6.3.3) and shall be `'0'`. We surface the bit so callers
    /// who want to distinguish a strict-MPEG-1 stream can do so.
    pub constrained_parameters: bool,
    /// 64-byte intra-block quantiser matrix in default zigzag scan
    /// order (§6.3.11). `Some(_)` iff `load_intra_quantiser_matrix`
    /// was `'1'`; `None` means "use default matrix" (§7.3.1).
    pub intra_quant: Option<[u8; 64]>,
    /// 64-byte non-intra quantiser matrix, same convention as
    /// `intra_quant`.
    pub non_intra_quant: Option<[u8; 64]>,
}

impl Mpeg2SequenceHeader {
    /// Parse a `sequence_header()` from a slice that starts with the
    /// four start-code bytes `00 00 01 B3`. The trailing
    /// `next_start_code()` byte-align + zero-byte stuffing (§5.2.3) is
    /// not consumed — the caller is in a better position to chain
    /// into the next layer.
    ///
    /// On success the returned struct mirrors the lower-bit values
    /// from the bitstream; higher-level synthesis (combining with
    /// `sequence_extension()`) is the caller's responsibility.
    pub fn parse(buf: &[u8]) -> Result<Self> {
        let mut br = BitReader::new(buf);

        // §6.2.2.1: 32-bit sequence_header_code, value 0x000001B3.
        let code = br.read_u32(32).map_err(|_| Error::ShortHeader)?;
        if code != SEQUENCE_HEADER_CODE {
            return Err(Error::InvalidBitstream(
                "sequence_header_code: expected 0x000001B3 (§6.3.3)",
            ));
        }

        let horizontal = br.read_u32(12).map_err(|_| Error::ShortHeader)? as u16;
        if horizontal == 0 {
            // §6.3.3: horizontal_size_value shall not be zero
            // (start-code emulation prevention).
            return Err(Error::InvalidBitstream(
                "horizontal_size_value: forbidden value 0 (§6.3.3)",
            ));
        }

        let vertical = br.read_u32(12).map_err(|_| Error::ShortHeader)? as u16;
        if vertical == 0 {
            return Err(Error::InvalidBitstream(
                "vertical_size_value: forbidden value 0 (§6.3.3)",
            ));
        }

        let aspect = AspectRatio::from_code(br.read_u32(4).map_err(|_| Error::ShortHeader)?)?;

        let frame_rate_code = br.read_u32(4).map_err(|_| Error::ShortHeader)? as u8;
        // §6.3.3 Table 6-4: code 0000 is forbidden; 1001..=1111 are
        // reserved. Reject the forbidden value here; reserved values
        // are surfaced as-is so a strict downstream caller may decide.
        if frame_rate_code == 0 {
            return Err(Error::InvalidBitstream(
                "frame_rate_code: forbidden value 0 (Table 6-4)",
            ));
        }

        let bit_rate = br.read_u32(18).map_err(|_| Error::ShortHeader)?;
        if bit_rate == 0 {
            // §6.3.3 forbids bit_rate == 0.
            return Err(Error::InvalidBitstream(
                "bit_rate_value: forbidden value 0 (§6.3.3)",
            ));
        }

        // §6.2.2.1: marker_bit, shall be '1'.
        let marker = br.read_u32(1).map_err(|_| Error::ShortHeader)?;
        if marker != 1 {
            return Err(Error::InvalidBitstream(
                "marker_bit after bit_rate_value: expected '1' (§6.2.2.1)",
            ));
        }

        let vbv_buffer_size = br.read_u32(10).map_err(|_| Error::ShortHeader)? as u16;
        let constrained_parameters = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;

        let load_intra = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let intra_quant = if load_intra {
            let mut m = [0u8; 64];
            for slot in &mut m {
                *slot = br.read_u32(8).map_err(|_| Error::ShortHeader)? as u8;
            }
            Some(m)
        } else {
            None
        };

        let load_non_intra = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let non_intra_quant = if load_non_intra {
            let mut m = [0u8; 64];
            for slot in &mut m {
                *slot = br.read_u32(8).map_err(|_| Error::ShortHeader)? as u8;
            }
            Some(m)
        } else {
            None
        };

        Ok(Self {
            width: horizontal,
            height: vertical,
            aspect_ratio: aspect,
            frame_rate_code,
            bit_rate,
            vbv_buffer_size,
            constrained_parameters,
            intra_quant,
            non_intra_quant,
        })
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact sequence headers. Each fixture is
    //! constructed by emitting the §6.2.2.1 syntax elements through a
    //! local MSB-first BitWriter so the bytes are derived purely from
    //! the published spec.
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Build the minimum-shape `sequence_header()`: no quantiser
    /// matrices loaded, no extension data.
    fn build_minimal(
        width: u16,
        height: u16,
        aspect_code: u32,
        frame_rate_code: u32,
        bit_rate: u32,
        vbv: u32,
        constrained: bool,
    ) -> Vec<u8> {
        let mut bw = BitWriter::new();
        bw.write_u32(SEQUENCE_HEADER_CODE, 32);
        bw.write_u32(u32::from(width), 12);
        bw.write_u32(u32::from(height), 12);
        bw.write_u32(aspect_code, 4);
        bw.write_u32(frame_rate_code, 4);
        bw.write_u32(bit_rate, 18);
        bw.write_bit(true); // marker_bit
        bw.write_u32(vbv, 10);
        bw.write_bit(constrained);
        bw.write_bit(false); // load_intra_quantiser_matrix
        bw.write_bit(false); // load_non_intra_quantiser_matrix
        bw.align_to_byte();
        bw.finish()
    }

    #[test]
    fn parses_minimal_pal_like_header() {
        // 720x576 PAL-shape stream, square SAR, 25 fps code (0011),
        // 1 Mbit/s ≙ bit_rate_value = 1_000_000 / 400 = 2500.
        let bytes = build_minimal(720, 576, 0b0001, 0b0011, 2500, 112, false);
        let sh = Mpeg2SequenceHeader::parse(&bytes).expect("parse");
        assert_eq!(sh.width, 720);
        assert_eq!(sh.height, 576);
        assert_eq!(sh.aspect_ratio, AspectRatio::Square);
        assert_eq!(sh.frame_rate_code, 0b0011);
        assert_eq!(sh.bit_rate, 2500);
        assert_eq!(sh.vbv_buffer_size, 112);
        assert!(!sh.constrained_parameters);
        assert!(sh.intra_quant.is_none());
        assert!(sh.non_intra_quant.is_none());
    }

    #[test]
    fn rejects_wrong_start_code() {
        let mut bytes = build_minimal(720, 576, 0b0001, 0b0011, 2500, 112, false);
        bytes[3] = 0xB8; // flip last start-code byte
        let err = Mpeg2SequenceHeader::parse(&bytes).unwrap_err();
        matches!(err, Error::InvalidBitstream(_));
    }

    #[test]
    fn rejects_forbidden_horizontal_size_zero() {
        // horizontal_size_value == 0 per §6.3.3.
        let bytes = build_minimal(0, 576, 0b0001, 0b0011, 2500, 112, false);
        let err = Mpeg2SequenceHeader::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_forbidden_vertical_size_zero() {
        let bytes = build_minimal(720, 0, 0b0001, 0b0011, 2500, 112, false);
        let err = Mpeg2SequenceHeader::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_forbidden_aspect_ratio_code() {
        let bytes = build_minimal(720, 576, 0b0000, 0b0011, 2500, 112, false);
        let err = Mpeg2SequenceHeader::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_forbidden_frame_rate_code() {
        let bytes = build_minimal(720, 576, 0b0001, 0b0000, 2500, 112, false);
        let err = Mpeg2SequenceHeader::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_zero_bit_rate() {
        let bytes = build_minimal(720, 576, 0b0001, 0b0011, 0, 112, false);
        let err = Mpeg2SequenceHeader::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_zero_marker_bit() {
        // Construct a header by hand and zero the marker bit.
        let mut bw = BitWriter::new();
        bw.write_u32(SEQUENCE_HEADER_CODE, 32);
        bw.write_u32(720, 12);
        bw.write_u32(576, 12);
        bw.write_u32(0b0001, 4);
        bw.write_u32(0b0011, 4);
        bw.write_u32(2500, 18);
        bw.write_bit(false); // marker_bit forced to 0
        bw.write_u32(112, 10);
        bw.write_bit(false);
        bw.write_bit(false);
        bw.write_bit(false);
        bw.align_to_byte();
        let bytes = bw.finish();
        let err = Mpeg2SequenceHeader::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn captures_aspect_ratio_codes() {
        for (code, expected) in [
            (0b0001u32, AspectRatio::Square),
            (0b0010, AspectRatio::Dar3x4),
            (0b0011, AspectRatio::Dar9x16),
            (0b0100, AspectRatio::Dar1x221),
            (0b0101, AspectRatio::Reserved(0b0101)),
            (0b1111, AspectRatio::Reserved(0b1111)),
        ] {
            let bytes = build_minimal(720, 576, code, 0b0011, 2500, 112, false);
            let sh = Mpeg2SequenceHeader::parse(&bytes).expect("parse");
            assert_eq!(sh.aspect_ratio, expected, "code {code:04b}");
        }
    }

    #[test]
    fn loads_intra_quantiser_matrix() {
        let mut bw = BitWriter::new();
        bw.write_u32(SEQUENCE_HEADER_CODE, 32);
        bw.write_u32(640, 12);
        bw.write_u32(480, 12);
        bw.write_u32(0b0001, 4);
        bw.write_u32(0b0101, 4); // 30 fps
        bw.write_u32(4000, 18); // 1.6 Mbit/s
        bw.write_bit(true); // marker_bit
        bw.write_u32(20, 10);
        bw.write_bit(false); // constrained_parameters
        bw.write_bit(true); // load_intra_quantiser_matrix
        for i in 0..64u32 {
            // Synthetic ramp 1..=64 — easy to recognise in failures.
            bw.write_u32(i + 1, 8);
        }
        bw.write_bit(false); // load_non_intra_quantiser_matrix
        bw.align_to_byte();
        let bytes = bw.finish();

        let sh = Mpeg2SequenceHeader::parse(&bytes).expect("parse");
        let q = sh.intra_quant.expect("intra matrix loaded");
        for (i, value) in q.iter().enumerate() {
            assert_eq!(*value, (i as u8) + 1, "intra slot {i}");
        }
        assert!(sh.non_intra_quant.is_none());
    }

    #[test]
    fn loads_both_quantiser_matrices() {
        let mut bw = BitWriter::new();
        bw.write_u32(SEQUENCE_HEADER_CODE, 32);
        bw.write_u32(352, 12);
        bw.write_u32(288, 12);
        bw.write_u32(0b0010, 4); // DAR 3:4
        bw.write_u32(0b0011, 4); // 25 fps
        bw.write_u32(1500, 18);
        bw.write_bit(true);
        bw.write_u32(40, 10);
        bw.write_bit(false);
        bw.write_bit(true); // load_intra
        for _ in 0..64 {
            bw.write_u32(8, 8);
        }
        bw.write_bit(true); // load_non_intra
        for _ in 0..64 {
            bw.write_u32(16, 8);
        }
        bw.align_to_byte();
        let bytes = bw.finish();

        let sh = Mpeg2SequenceHeader::parse(&bytes).expect("parse");
        let i = sh.intra_quant.expect("intra loaded");
        let n = sh.non_intra_quant.expect("non-intra loaded");
        assert!(i.iter().all(|&v| v == 8));
        assert!(n.iter().all(|&v| v == 16));
        assert_eq!(sh.aspect_ratio, AspectRatio::Dar3x4);
    }

    #[test]
    fn rejects_truncated_buffer() {
        let bytes = vec![0x00, 0x00, 0x01, 0xB3, 0x00]; // start code + 1 byte
        let err = Mpeg2SequenceHeader::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }
}
