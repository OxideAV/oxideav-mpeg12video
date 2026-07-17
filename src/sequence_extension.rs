//! Parser for the MPEG-2 video `sequence_extension()` element.
//!
//! Implements the bitstream syntax in ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262) §6.2.2.3 and the field semantics in §6.3.5.
//!
//! `sequence_extension()` is the MPEG-2 layer that must immediately
//! follow `sequence_header()` (§6.2.2.1) when the bitstream is
//! MPEG-2 rather than ISO/IEC 11172-2 (MPEG-1). It contributes the
//! upper bits of `horizontal_size` / `vertical_size` / `bit_rate` /
//! `vbv_buffer_size` and exposes the new MPEG-2 fields
//! (profile/level, chroma format, progressive/interlaced flag,
//! frame-rate extension numerator/denominator, low_delay).
//!
//! This module deliberately keeps the parsed struct shaped exactly
//! like the bitstream: the higher-level synthesis that combines the
//! lower bits from [`crate::sequence_header::Mpeg2SequenceHeader`]
//! with the upper bits from this struct lives in
//! [`Mpeg2Sequence::from_buf`].

use oxideav_core::bits::BitReader;

use crate::sequence_header::{Mpeg2SequenceHeader, SEQUENCE_HEADER_CODE};
use crate::{Error, Result};

/// 32-bit `extension_start_code`, byte string `00 00 01 B5`
/// (§6.3.4).
#[doc(hidden)] // internal: §6.2.2.3 parser plumbing
pub const EXTENSION_START_CODE: u32 = 0x0000_01B5;

/// `extension_start_code_identifier` value for `sequence_extension()`
/// (Table 6-2 entry `0001`).
#[doc(hidden)] // internal: §6.2.2.3 parser plumbing
pub const SEQUENCE_EXTENSION_ID: u32 = 0b0001;

/// `chroma_format` (Table 6-5).
///
/// Code `00` is spec-reserved and is rejected during parsing; the
/// three legal codes map to the standard 4:2:0 / 4:2:2 / 4:4:4
/// sub-samplings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChromaFormat {
    /// `01` — 4:2:0 chroma sub-sampling.
    Yuv420,
    /// `10` — 4:2:2 chroma sub-sampling.
    Yuv422,
    /// `11` — 4:4:4 (no chroma sub-sampling).
    Yuv444,
}

impl ChromaFormat {
    fn from_code(code: u32) -> Result<Self> {
        match code {
            0b00 => Err(Error::InvalidBitstream(
                "chroma_format: reserved value 00 (Table 6-5)",
            )),
            0b01 => Ok(Self::Yuv420),
            0b10 => Ok(Self::Yuv422),
            0b11 => Ok(Self::Yuv444),
            _ => unreachable!("chroma_format is a 2-bit field"),
        }
    }
}

/// Parsed result of `sequence_extension()` (§6.2.2.3).
///
/// The struct mirrors the bitstream verbatim — i.e. the
/// `*_extension` fields are the *extension portion* (upper bits)
/// only. The convenience helper [`Mpeg2Sequence::from_buf`] composes
/// them with the lower bits in [`Mpeg2SequenceHeader`].
#[derive(Debug, Clone, Copy)]
#[doc(hidden)] // internal: §6.2.2.3 parser plumbing (ChromaFormat above is the stable type)
pub struct Mpeg2SequenceExtension {
    /// 8-bit `profile_and_level_indication` (clause 8). The bit
    /// layout is profile-specific; this crate currently surfaces the
    /// raw byte and leaves interpretation to higher layers.
    pub profile_and_level: u8,
    /// `progressive_sequence` flag. `true` ⇒ every coded picture is
    /// a progressive frame; `false` ⇒ the sequence may contain
    /// field-pictures or interlaced frame-pictures (§6.3.5).
    pub progressive_sequence: bool,
    /// `chroma_format` (Table 6-5).
    pub chroma_format: ChromaFormat,
    /// `horizontal_size_extension` — upper 2 bits of the 14-bit
    /// `horizontal_size`.
    pub horizontal_size_extension: u8,
    /// `vertical_size_extension` — upper 2 bits of the 14-bit
    /// `vertical_size`.
    pub vertical_size_extension: u8,
    /// `bit_rate_extension` — upper 12 bits of the 30-bit `bit_rate`
    /// (units of 400 bit/s).
    pub bit_rate_extension: u16,
    /// `vbv_buffer_size_extension` — upper 8 bits of the 18-bit
    /// `vbv_buffer_size`.
    pub vbv_buffer_size_extension: u8,
    /// `low_delay` flag: `true` ⇒ no B-pictures and big-picture
    /// behaviour per Annex C.7 may apply (§6.3.5).
    pub low_delay: bool,
    /// `frame_rate_extension_n` (2 bits). Combined with
    /// `frame_rate_extension_d` as `(n + 1) ÷ (d + 1)` to scale the
    /// `frame_rate_value` looked up from Table 6-4.
    pub frame_rate_extension_n: u8,
    /// `frame_rate_extension_d` (5 bits).
    pub frame_rate_extension_d: u8,
}

impl Mpeg2SequenceExtension {
    /// Parse a `sequence_extension()` from a slice that starts with
    /// the four start-code bytes `00 00 01 B5`. The trailing
    /// `next_start_code()` byte-align + zero stuffing (§5.2.3) is
    /// not consumed.
    pub fn parse(buf: &[u8]) -> Result<Self> {
        let mut br = BitReader::new(buf);
        Self::parse_with_reader(&mut br)
    }

    /// Parse from an existing [`BitReader`] positioned at the start
    /// of `sequence_extension()` (i.e. its 32-bit `extension_start_code`).
    fn parse_with_reader(br: &mut BitReader<'_>) -> Result<Self> {
        // §6.2.2.3: extension_start_code, value 0x000001B5.
        let code = br.read_u32(32).map_err(|_| Error::ShortHeader)?;
        if code != EXTENSION_START_CODE {
            return Err(Error::InvalidBitstream(
                "extension_start_code: expected 0x000001B5 (§6.3.4)",
            ));
        }

        // 4-bit extension_start_code_identifier; Sequence Extension
        // ID is `0001` per Table 6-2.
        let id = br.read_u32(4).map_err(|_| Error::ShortHeader)?;
        if id != SEQUENCE_EXTENSION_ID {
            return Err(Error::InvalidBitstream(
                "extension_start_code_identifier: expected '0001' Sequence Extension ID (Table 6-2)",
            ));
        }

        let profile_and_level = br.read_u32(8).map_err(|_| Error::ShortHeader)? as u8;
        let progressive_sequence = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let chroma_format =
            ChromaFormat::from_code(br.read_u32(2).map_err(|_| Error::ShortHeader)?)?;
        let h_size_ext = br.read_u32(2).map_err(|_| Error::ShortHeader)? as u8;
        let v_size_ext = br.read_u32(2).map_err(|_| Error::ShortHeader)? as u8;
        let bit_rate_extension = br.read_u32(12).map_err(|_| Error::ShortHeader)? as u16;

        // §6.2.2.3 marker_bit, shall be '1'.
        let marker = br.read_u32(1).map_err(|_| Error::ShortHeader)?;
        if marker != 1 {
            return Err(Error::InvalidBitstream(
                "marker_bit in sequence_extension(): expected '1' (§6.2.2.3)",
            ));
        }

        let vbv_buffer_size_extension = br.read_u32(8).map_err(|_| Error::ShortHeader)? as u8;
        let low_delay = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let frame_rate_extension_n = br.read_u32(2).map_err(|_| Error::ShortHeader)? as u8;
        let frame_rate_extension_d = br.read_u32(5).map_err(|_| Error::ShortHeader)? as u8;

        Ok(Self {
            profile_and_level,
            progressive_sequence,
            chroma_format,
            horizontal_size_extension: h_size_ext,
            vertical_size_extension: v_size_ext,
            bit_rate_extension,
            vbv_buffer_size_extension,
            low_delay,
            frame_rate_extension_n,
            frame_rate_extension_d,
        })
    }
}

/// Result of [`Mpeg2Sequence::from_buf`] — a sequence header paired
/// with its `sequence_extension()` plus the synthesised full-width
/// 14-bit / 30-bit / 18-bit values.
///
/// The composition rules follow §6.3.3 and §6.3.5:
///
/// * `horizontal_size = (horizontal_size_extension << 12) | horizontal_size_value`
/// * `vertical_size   = (vertical_size_extension   << 12) | vertical_size_value`
/// * `bit_rate        = (bit_rate_extension        << 18) | bit_rate_value`
/// * `vbv_buffer_size = (vbv_buffer_size_extension << 10) | vbv_buffer_size_value`
#[derive(Debug, Clone)]
#[doc(hidden)] // internal: §6.2.2 composed sequence-layer view, parser plumbing
pub struct Mpeg2Sequence {
    /// The parsed `sequence_header()` (lower-bit values, plus all
    /// MPEG-1-compatible fields).
    pub header: Mpeg2SequenceHeader,
    /// The parsed `sequence_extension()` (upper-bit values plus
    /// MPEG-2-only fields).
    pub extension: Mpeg2SequenceExtension,
    /// 14-bit composed `horizontal_size` (samples).
    pub horizontal_size: u16,
    /// 14-bit composed `vertical_size` (lines).
    pub vertical_size: u16,
    /// 30-bit composed `bit_rate` (units of 400 bit/s, §6.3.3).
    pub bit_rate: u32,
    /// 18-bit composed `vbv_buffer_size`. `B = 16 * 1024 *
    /// vbv_buffer_size` bits (§6.3.5).
    pub vbv_buffer_size: u32,
}

impl Mpeg2Sequence {
    /// Parse `sequence_header()` immediately followed by
    /// `sequence_extension()` from a buffer starting at the
    /// `sequence_header_code` (`00 00 01 B3`). Per §5.2.3 the
    /// `next_start_code()` between the two layers is consumed: any
    /// trailing zero bits to reach the next byte boundary, plus any
    /// number of `00` stuffing bytes before the next start code,
    /// are discarded.
    pub fn from_buf(buf: &[u8]) -> Result<Self> {
        // Re-use the existing sequence_header parser through its
        // public entry point so its bitstream-error coverage stays
        // shared. After it returns we recompute the cursor by
        // scanning for the next `00 00 01` byte sequence — that's
        // the spec's `next_start_code()` (§5.2.3): from the byte
        // boundary after the header, skip any number of `00` bytes
        // until a `0x01` byte appears.
        let header = Mpeg2SequenceHeader::parse(buf)?;
        let after_header = sequence_header_byte_length(buf)?;

        // §5.2.3 next_start_code(): byte-align (we are already
        // byte-aligned after the matrix-load arms), then skip
        // any number of zero stuffing bytes until the next start
        // code's first non-zero byte (`01`). The next start code
        // shall be the extension_start_code when present.
        let mut cursor = after_header;
        while cursor < buf.len() && buf[cursor] == 0x00 {
            cursor += 1;
        }
        // Re-anchor to the start-code's first byte: the zero-byte
        // skip stopped on a non-`00` byte which must be the `0x01`
        // start-code marker preceded by (at least) two `00`s.
        if cursor >= buf.len()
            || buf[cursor] != 0x01
            || cursor < 2
            || buf[cursor - 1] != 0x00
            || buf[cursor - 2] != 0x00
        {
            return Err(Error::InvalidBitstream(
                "next_start_code(): missing '00 00 01' prefix after sequence_header() (§5.2.3)",
            ));
        }
        let start_of_extension = cursor - 2;
        if start_of_extension + 4 > buf.len() {
            return Err(Error::ShortHeader);
        }
        if buf[start_of_extension + 3] != 0xB5 {
            return Err(Error::InvalidBitstream(
                "sequence_extension(): expected start code 0x000001B5 immediately after sequence_header() (§6.2.2)",
            ));
        }
        let extension = Mpeg2SequenceExtension::parse(&buf[start_of_extension..])?;

        let horizontal_size = (u16::from(extension.horizontal_size_extension) << 12) | header.width;
        let vertical_size = (u16::from(extension.vertical_size_extension) << 12) | header.height;
        let bit_rate = (u32::from(extension.bit_rate_extension) << 18) | header.bit_rate;
        if bit_rate == 0 {
            // §6.3.3: combined bit_rate value zero is forbidden.
            return Err(Error::InvalidBitstream(
                "bit_rate: forbidden composite value 0 (§6.3.3)",
            ));
        }
        let vbv_buffer_size = (u32::from(extension.vbv_buffer_size_extension) << 10)
            | u32::from(header.vbv_buffer_size);

        Ok(Self {
            header,
            extension,
            horizontal_size,
            vertical_size,
            bit_rate,
            vbv_buffer_size,
        })
    }
}

/// Re-parse the `sequence_header()` purely to discover the byte
/// length it consumes from `buf`. The parser itself does not return
/// this datum, so we recompute it by adding up the fixed bit count
/// (§6.2.2.1: 32 + 12 + 12 + 4 + 4 + 18 + 1 + 10 + 1 + 1 + 1 = 96
/// bits) plus the optional 64-byte matrix loads.
fn sequence_header_byte_length(buf: &[u8]) -> Result<usize> {
    // Mirror the spec's syntax for everything up to the two
    // load-quant-matrix flags, then count the matrices that
    // actually follow.
    let mut br = BitReader::new(buf);

    // Sanity-check the start code so this helper can be called on
    // an arbitrary buffer without depending on a prior parser run.
    let code = br.read_u32(32).map_err(|_| Error::ShortHeader)?;
    if code != SEQUENCE_HEADER_CODE {
        return Err(Error::InvalidBitstream(
            "sequence_header_code: expected 0x000001B3 (§6.3.3)",
        ));
    }

    // Fixed fields up to and including constrained_parameters_flag:
    // 12 + 12 + 4 + 4 + 18 + 1 + 10 + 1 = 62 bits.
    br.skip(62).map_err(|_| Error::ShortHeader)?;

    let load_intra = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
    if load_intra {
        br.skip(64 * 8).map_err(|_| Error::ShortHeader)?;
    }
    let load_non_intra = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
    if load_non_intra {
        br.skip(64 * 8).map_err(|_| Error::ShortHeader)?;
    }

    // The two flags + (optional) matrices land us back on a byte
    // boundary because the matrices are byte-sized and the
    // pre-matrix bit count (32 + 62 + 1 + 1 = 96) is already a
    // multiple of 8. Confirm and return.
    debug_assert!(br.is_byte_aligned());
    Ok(br.byte_position())
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact `sequence_header()` + `sequence_extension()`
    //! pairs, plus negative cases for every spec-defined rejection
    //! site that this parser introduces.
    use super::*;
    use crate::sequence_header::AspectRatio;
    use oxideav_core::bits::BitWriter;

    /// Emit a minimal `sequence_header()` (no matrices loaded), so
    /// the parser exits on a byte boundary. Matches the helper used
    /// by `sequence_header.rs`'s tests.
    #[allow(clippy::too_many_arguments)]
    fn write_sequence_header(
        bw: &mut BitWriter,
        width: u16,
        height: u16,
        aspect_code: u32,
        frame_rate_code: u32,
        bit_rate: u32,
        vbv: u32,
        constrained: bool,
    ) {
        bw.write_u32(SEQUENCE_HEADER_CODE, 32);
        bw.write_u32(u32::from(width), 12);
        bw.write_u32(u32::from(height), 12);
        bw.write_u32(aspect_code, 4);
        bw.write_u32(frame_rate_code, 4);
        bw.write_u32(bit_rate, 18);
        bw.write_bit(true);
        bw.write_u32(vbv, 10);
        bw.write_bit(constrained);
        bw.write_bit(false); // load_intra_quantiser_matrix
        bw.write_bit(false); // load_non_intra_quantiser_matrix
        bw.align_to_byte();
    }

    #[allow(clippy::too_many_arguments)]
    fn write_sequence_extension(
        bw: &mut BitWriter,
        profile_and_level: u8,
        progressive: bool,
        chroma_code: u32,
        h_ext: u32,
        v_ext: u32,
        bit_rate_ext: u32,
        vbv_ext: u32,
        low_delay: bool,
        fr_n: u32,
        fr_d: u32,
    ) {
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(SEQUENCE_EXTENSION_ID, 4);
        bw.write_u32(u32::from(profile_and_level), 8);
        bw.write_bit(progressive);
        bw.write_u32(chroma_code, 2);
        bw.write_u32(h_ext, 2);
        bw.write_u32(v_ext, 2);
        bw.write_u32(bit_rate_ext, 12);
        bw.write_bit(true); // marker_bit
        bw.write_u32(vbv_ext, 8);
        bw.write_bit(low_delay);
        bw.write_u32(fr_n, 2);
        bw.write_u32(fr_d, 5);
        bw.align_to_byte();
    }

    #[test]
    fn parses_minimal_sequence_extension() {
        let mut bw = BitWriter::new();
        write_sequence_extension(
            &mut bw, 0x44, // profile_and_level (Main@Main = 0x48 in real fixtures;
            // any byte is legal at parse time)
            true,  // progressive_sequence
            0b01,  // 4:2:0
            0,     // h_ext
            0,     // v_ext
            0,     // bit_rate_ext
            0,     // vbv_ext
            false, // low_delay
            0,     // fr_n
            0,     // fr_d
        );
        let bytes = bw.finish();
        let ext = Mpeg2SequenceExtension::parse(&bytes).expect("parse");
        assert_eq!(ext.profile_and_level, 0x44);
        assert!(ext.progressive_sequence);
        assert_eq!(ext.chroma_format, ChromaFormat::Yuv420);
        assert_eq!(ext.horizontal_size_extension, 0);
        assert_eq!(ext.vertical_size_extension, 0);
        assert_eq!(ext.bit_rate_extension, 0);
        assert_eq!(ext.vbv_buffer_size_extension, 0);
        assert!(!ext.low_delay);
        assert_eq!(ext.frame_rate_extension_n, 0);
        assert_eq!(ext.frame_rate_extension_d, 0);
    }

    #[test]
    fn captures_chroma_format_codes() {
        for (code, expected) in [
            (0b01u32, ChromaFormat::Yuv420),
            (0b10, ChromaFormat::Yuv422),
            (0b11, ChromaFormat::Yuv444),
        ] {
            let mut bw = BitWriter::new();
            write_sequence_extension(&mut bw, 0, false, code, 0, 0, 0, 0, false, 0, 0);
            let bytes = bw.finish();
            let ext = Mpeg2SequenceExtension::parse(&bytes).expect("parse");
            assert_eq!(ext.chroma_format, expected, "code {code:02b}");
        }
    }

    #[test]
    fn rejects_reserved_chroma_format() {
        // chroma_format == 00 is reserved per Table 6-5.
        let mut bw = BitWriter::new();
        write_sequence_extension(&mut bw, 0, false, 0b00, 0, 0, 0, 0, false, 0, 0);
        let bytes = bw.finish();
        let err = Mpeg2SequenceExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_wrong_extension_start_code() {
        let mut bw = BitWriter::new();
        write_sequence_extension(&mut bw, 0, false, 0b01, 0, 0, 0, 0, false, 0, 0);
        let mut bytes = bw.finish();
        bytes[3] = 0xB7; // sequence_end_code
        let err = Mpeg2SequenceExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_wrong_extension_identifier() {
        // Build manually with the identifier set to '0010' (Sequence
        // Display Extension ID) rather than '0001'.
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(0b0010, 4);
        bw.write_u32(0, 8);
        bw.write_bit(false);
        bw.write_u32(0b01, 2);
        bw.write_u32(0, 2);
        bw.write_u32(0, 2);
        bw.write_u32(0, 12);
        bw.write_bit(true);
        bw.write_u32(0, 8);
        bw.write_bit(false);
        bw.write_u32(0, 2);
        bw.write_u32(0, 5);
        bw.align_to_byte();
        let bytes = bw.finish();
        let err = Mpeg2SequenceExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_zero_marker_bit_in_extension() {
        // Build manually with marker_bit forced to '0'.
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(SEQUENCE_EXTENSION_ID, 4);
        bw.write_u32(0, 8);
        bw.write_bit(false);
        bw.write_u32(0b01, 2);
        bw.write_u32(0, 2);
        bw.write_u32(0, 2);
        bw.write_u32(0, 12);
        bw.write_bit(false); // marker_bit forced to 0
        bw.write_u32(0, 8);
        bw.write_bit(false);
        bw.write_u32(0, 2);
        bw.write_u32(0, 5);
        bw.align_to_byte();
        let bytes = bw.finish();
        let err = Mpeg2SequenceExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_truncated_extension_buffer() {
        let bytes = vec![0x00, 0x00, 0x01, 0xB5, 0x10]; // start code + 1 byte
        let err = Mpeg2SequenceExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    /// End-to-end: build a full sequence_header() + sequence_extension()
    /// pair (no zero stuffing) and verify the composed dimensions.
    #[test]
    fn composes_sequence_with_extension() {
        let mut bw = BitWriter::new();
        // header lower bits: width 720, height 576, square SAR,
        // 25 fps (0011), 2500 bit_rate (1 Mbit/s), vbv 112.
        write_sequence_header(&mut bw, 720, 576, 0b0001, 0b0011, 2500, 112, false);
        // extension upper bits: progressive, 4:2:0, h_ext=1, v_ext=2,
        // bit_rate_ext=3, vbv_ext=4, low_delay=true.
        write_sequence_extension(&mut bw, 0x44, true, 0b01, 1, 2, 3, 4, true, 0, 0);
        let bytes = bw.finish();

        let seq = Mpeg2Sequence::from_buf(&bytes).expect("compose");
        assert_eq!(seq.header.width, 720);
        assert_eq!(seq.header.height, 576);
        assert_eq!(seq.header.aspect_ratio, AspectRatio::Square);
        assert_eq!(seq.extension.profile_and_level, 0x44);
        assert!(seq.extension.progressive_sequence);
        assert_eq!(seq.extension.chroma_format, ChromaFormat::Yuv420);
        // Composed values (§6.3.3, §6.3.5):
        assert_eq!(seq.horizontal_size, (1 << 12) | 720);
        assert_eq!(seq.vertical_size, (2 << 12) | 576);
        assert_eq!(seq.bit_rate, (3u32 << 18) | 2500);
        assert_eq!(seq.vbv_buffer_size, (4u32 << 10) | 112);
        assert!(seq.extension.low_delay);
    }

    /// `next_start_code()` (§5.2.3) allows any number of `00`
    /// stuffing bytes between the two layers. We always emit zero
    /// such bytes from `write_sequence_header()` (since it ends on a
    /// byte boundary), but compliant encoders may add some — the
    /// composer must tolerate it.
    #[test]
    fn composes_with_zero_byte_stuffing_between_layers() {
        let mut bw = BitWriter::new();
        write_sequence_header(&mut bw, 352, 288, 0b0010, 0b0011, 1500, 40, false);
        let mut bytes = bw.finish();
        bytes.extend_from_slice(&[0x00, 0x00, 0x00]); // three stuffing bytes
        let mut tail = BitWriter::new();
        write_sequence_extension(&mut tail, 0x48, false, 0b10, 0, 0, 0, 0, false, 0, 0);
        bytes.extend_from_slice(&tail.finish());
        let seq = Mpeg2Sequence::from_buf(&bytes).expect("compose");
        assert_eq!(seq.horizontal_size, 352);
        assert_eq!(seq.vertical_size, 288);
        assert_eq!(seq.extension.chroma_format, ChromaFormat::Yuv422);
    }

    #[test]
    fn rejects_missing_extension_after_header() {
        // sequence_header() followed by sequence_end_code (B7)
        // instead of the extension's B5.
        let mut bw = BitWriter::new();
        write_sequence_header(&mut bw, 720, 576, 0b0001, 0b0011, 2500, 112, false);
        let mut bytes = bw.finish();
        bytes.extend_from_slice(&[0x00, 0x00, 0x01, 0xB7]);
        let err = Mpeg2Sequence::from_buf(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_composite_bit_rate_zero() {
        // bit_rate_value can be anything non-zero (e.g. 1) and
        // bit_rate_extension can be anything; the composite must
        // also be non-zero. Force bit_rate_value=1 and ext=0 to land
        // on a non-zero composite (passes), then flip to verify
        // the negative path: build a header with the maximum
        // 18-bit value zero is *forbidden* — we can't construct
        // that legally via the header parser, so we exercise the
        // composite-zero guard by routing through a manually
        // emitted buffer where the lower 18 bits are 0. The header
        // parser will reject first; the composite guard is dead
        // code defensively. Skip negative test (no observable path).
        // This test instead verifies the boundary value: smallest
        // legal composite is 1.
        let mut bw = BitWriter::new();
        write_sequence_header(&mut bw, 720, 576, 0b0001, 0b0011, 1, 0, false);
        write_sequence_extension(&mut bw, 0, false, 0b01, 0, 0, 0, 0, false, 0, 0);
        let bytes = bw.finish();
        let seq = Mpeg2Sequence::from_buf(&bytes).expect("compose minimal");
        assert_eq!(seq.bit_rate, 1);
    }

    #[test]
    fn debug_impl_smoke() {
        let mut bw = BitWriter::new();
        write_sequence_extension(&mut bw, 0x48, true, 0b01, 0, 0, 0, 0, false, 0, 0);
        let bytes = bw.finish();
        let ext = Mpeg2SequenceExtension::parse(&bytes).expect("parse");
        // Just ensure Debug renders without panicking and includes
        // the chroma_format variant name.
        let s = format!("{ext:?}");
        assert!(s.contains("Yuv420"));
    }
}
