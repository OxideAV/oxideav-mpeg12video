//! Parser for the MPEG-1 / MPEG-2 video `picture_header()` syntax
//! element plus the optional MPEG-2 `picture_coding_extension()`.
//!
//! Implements the bitstream syntax in ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262) §6.2.3 and the field semantics in §6.3.10. The
//! companion `picture_coding_extension()` (§6.2.3.1 / §6.3.11) is
//! exposed through [`Mpeg2PictureHeader::parse_with_extension`] for
//! callers that want the full picture-layer view in a single call.
//!
//! Per §6.3.10, `picture_header()` opens with the 32-bit
//! `picture_start_code` `0x00000100`. The 10-bit `temporal_reference`
//! is incremented (modulo 1024) for each input frame; the 3-bit
//! `picture_coding_type` identifies whether the picture is I-/P-/B-
//! coded (Table 6-12); the 16-bit `vbv_delay` measures the time the
//! VBV buffer must wait, in periods of the 90 kHz system clock,
//! before this picture is removed (or carries the sentinel `0xFFFF`
//! for variable-bitrate streams).
//!
//! For predictive-coded pictures (P, type `2`) and
//! bidirectionally-predictive-coded pictures (B, type `3`) the spec
//! preserves the MPEG-1 `full_pel_forward_vector` flag and
//! `forward_f_code` field; for B-pictures it additionally preserves
//! `full_pel_backward_vector` and `backward_f_code`. In MPEG-2 these
//! four sub-fields are mandated by §6.3.10 to be `0` / `0b111` (`7`)
//! respectively; the parser surfaces the raw values so MPEG-1
//! callers see the truth and MPEG-2 callers can validate the spec
//! constraint at a higher layer.
//!
//! The trailing `extra_information_picture` loop is byte-aligned by
//! the parser: each `extra_bit_picture == 1` introduces 8 bits of
//! reserved data that "shall be ignored" by a conforming decoder
//! (§6.3.10). The bytes are collected so encoders that round-trip the
//! bitstream do not lose them, even though the spec disallows their
//! presence in a conforming stream.
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)).

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// The 32-bit start code that introduces a `picture_header()`: the
/// byte string `00 00 01 00` (§6.3.10).
#[doc(hidden)] // internal: §6.2.3 parser plumbing
pub const PICTURE_START_CODE: u32 = 0x0000_0100;

/// `extension_start_code_identifier` value for
/// `picture_coding_extension()` (Table 6-2 entry `1000`).
#[doc(hidden)] // internal: §6.2.3 parser plumbing
pub const PICTURE_CODING_EXTENSION_ID: u32 = 0b1000;

/// `picture_coding_type` (§6.3.10, Table 6-12).
///
/// The three usable codes map to I- / P- / B- pictures. Code `0b000`
/// is spec-forbidden; `0b100` is the MPEG-1 D-picture type that
/// "shall not be used" in MPEG-2; codes `0b101..=0b111` are reserved.
/// All four illegal codes are rejected at parse time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PictureCodingType {
    /// `001` — intra-coded picture (I).
    Intra,
    /// `010` — predictive-coded picture (P).
    Predictive,
    /// `011` — bidirectionally predictive-coded picture (B).
    Bidirectional,
    /// `100` — dc intra-coded picture (D), ISO/IEC 11172-2 only
    /// (§2.4.3.4). Of the DCT coefficients only the dc ones are
    /// present, and a D-picture sequence shall contain no other
    /// picture types (§2.4.1). ISO/IEC 13818-2 Table 6-12 marks the
    /// code *"shall not be used"*, so the MPEG-2 header+extension
    /// parser ([`Mpeg2PictureHeader::parse_with_extension`]) rejects
    /// it; the bare [`Mpeg2PictureHeader::parse`] accepts it for the
    /// 11172-2 decode path.
    DcIntra,
}

impl PictureCodingType {
    fn from_code(code: u32) -> Result<Self> {
        match code {
            0b000 => Err(Error::InvalidBitstream(
                "picture_coding_type: forbidden value 000 (Table 6-12)",
            )),
            0b001 => Ok(Self::Intra),
            0b010 => Ok(Self::Predictive),
            0b011 => Ok(Self::Bidirectional),
            // dc intra-coded (D) — legal in ISO/IEC 11172-2 only; the
            // MPEG-2 gate lives in `parse_with_extension` (Table 6-12).
            0b100 => Ok(Self::DcIntra),
            0b101..=0b111 => Err(Error::InvalidBitstream(
                "picture_coding_type: reserved value (Table 6-12)",
            )),
            _ => unreachable!("picture_coding_type is a 3-bit field"),
        }
    }

    /// Returns `true` for picture types whose `picture_header()`
    /// carries a `forward_f_code` (P, B). Used by both the parser
    /// and round-trip writers.
    pub const fn uses_forward(self) -> bool {
        matches!(self, Self::Predictive | Self::Bidirectional)
    }

    /// Returns `true` for picture types whose `picture_header()`
    /// carries a `backward_f_code` (B only).
    pub const fn uses_backward(self) -> bool {
        matches!(self, Self::Bidirectional)
    }
}

/// Parsed result of `picture_header()` (§6.2.3).
///
/// `fwd_*` / `bwd_*` fields are present only when the picture type
/// requires them per the spec's `if (picture_coding_type ==
/// 2 || picture_coding_type == 3)` (forward) and `if
/// (picture_coding_type == 3)` (backward) conditional gates.
///
/// `extra_information_picture` is the optional reserved-byte payload
/// after the conditional motion-vector hints. §6.3.10 says any
/// conforming stream "shall not contain this syntax element", so this
/// vector is empty for every legal MPEG-2 bitstream we observe; the
/// parser keeps the bytes for callers that need to faithfully
/// reproduce a non-conforming input.
#[derive(Debug, Clone, PartialEq, Eq)]
#[doc(hidden)] // internal: §6.2.3 parser plumbing (PictureCodingType above is the stable type)
pub struct Mpeg2PictureHeader {
    /// 10-bit `temporal_reference`, modulo-1024 frame index
    /// (§6.3.10).
    pub temporal_reference: u16,
    /// `picture_coding_type` (Table 6-12), I/P/B.
    pub picture_coding_type: PictureCodingType,
    /// 16-bit `vbv_delay`. For non-constant-bitrate streams this is
    /// the sentinel `0xFFFF` (§6.3.10).
    pub vbv_delay: u16,
    /// `full_pel_forward_vector` flag, present when
    /// `picture_coding_type ∈ {P, B}` (§6.2.3). Per §6.3.10 this
    /// MPEG-1 flag "shall have the value zero" in MPEG-2.
    pub full_pel_forward_vector: Option<bool>,
    /// 3-bit `forward_f_code`, present when
    /// `picture_coding_type ∈ {P, B}`. Per §6.3.10 this MPEG-1
    /// parameter "shall have the value seven (all ones)" in MPEG-2.
    pub fwd_f_code: Option<u8>,
    /// `full_pel_backward_vector` flag, present only when
    /// `picture_coding_type == B`. Same MPEG-2 spec constraint as
    /// `full_pel_forward_vector`.
    pub full_pel_backward_vector: Option<bool>,
    /// 3-bit `backward_f_code`, present only when
    /// `picture_coding_type == B`. Same MPEG-2 spec constraint as
    /// `forward_f_code` (must be `0b111`).
    pub bwd_f_code: Option<u8>,
    /// `extra_information_picture` bytes accumulated from the
    /// `while ( nextbits() == '1' )` loop. Empty for every
    /// MPEG-2-compliant stream (§6.3.10).
    pub extra_information_picture: Vec<u8>,
}

impl Mpeg2PictureHeader {
    /// Parse a `picture_header()` from a slice that starts with the
    /// four start-code bytes `00 00 01 00`.
    ///
    /// The trailing `next_start_code()` byte-align + zero stuffing
    /// (§5.2.3) is *not* consumed; the caller is in a better
    /// position to chain into the following `picture_coding_extension()`
    /// (§6.2.3.1), `picture_data()` (§6.2.3.6), or whatever layer
    /// the elementary stream presents next.
    pub fn parse(buf: &[u8]) -> Result<Self> {
        let mut br = BitReader::new(buf);
        Self::parse_with_reader(&mut br)
    }

    fn parse_with_reader(br: &mut BitReader<'_>) -> Result<Self> {
        // §6.3.10: 32-bit picture_start_code, value 0x00000100.
        let code = br.read_u32(32).map_err(|_| Error::ShortHeader)?;
        if code != PICTURE_START_CODE {
            return Err(Error::InvalidBitstream(
                "picture_start_code: expected 0x00000100 (§6.3.10)",
            ));
        }

        // 10-bit temporal_reference (§6.3.10).
        let temporal_reference = br.read_u32(10).map_err(|_| Error::ShortHeader)? as u16;

        // 3-bit picture_coding_type (Table 6-12).
        let coding_code = br.read_u32(3).map_err(|_| Error::ShortHeader)?;
        let picture_coding_type = PictureCodingType::from_code(coding_code)?;

        // 16-bit vbv_delay (§6.3.10). No range constraint other than
        // the FFFF sentinel for VBR — we preserve the raw value.
        let vbv_delay = br.read_u32(16).map_err(|_| Error::ShortHeader)? as u16;

        // §6.2.3: forward motion-vector hint for P / B pictures.
        let (full_pel_forward_vector, fwd_f_code) = if picture_coding_type.uses_forward() {
            let full_pel = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
            let f_code = br.read_u32(3).map_err(|_| Error::ShortHeader)? as u8;
            (Some(full_pel), Some(f_code))
        } else {
            (None, None)
        };

        // §6.2.3: backward motion-vector hint for B pictures.
        let (full_pel_backward_vector, bwd_f_code) = if picture_coding_type.uses_backward() {
            let full_pel = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
            let f_code = br.read_u32(3).map_err(|_| Error::ShortHeader)? as u8;
            (Some(full_pel), Some(f_code))
        } else {
            (None, None)
        };

        // §6.2.3: while ( nextbits() == '1' ) { extra_bit_picture;
        // extra_information_picture[8] }. The terminating
        // extra_bit_picture is read after the loop exits.
        let mut extra_information_picture = Vec::new();
        loop {
            let bit = br.read_u32(1).map_err(|_| Error::ShortHeader)?;
            if bit == 0 {
                // Terminating extra_bit_picture == '0'.
                break;
            }
            let extra = br.read_u32(8).map_err(|_| Error::ShortHeader)? as u8;
            extra_information_picture.push(extra);
        }

        Ok(Self {
            temporal_reference,
            picture_coding_type,
            vbv_delay,
            full_pel_forward_vector,
            fwd_f_code,
            full_pel_backward_vector,
            bwd_f_code,
            extra_information_picture,
        })
    }

    /// Parse a `picture_header()` immediately followed by the
    /// `picture_coding_extension()` mandated by §6.2.2.5 whenever the
    /// surrounding video sequence carries a `sequence_extension()`
    /// (i.e. is an MPEG-2 sequence). The `next_start_code()` byte
    /// alignment + zero-stuffing between the two layers is consumed
    /// per §5.2.3.
    ///
    /// The returned [`PictureCodingExtension`] only models the
    /// fixed 4 + 4 + 4 + 4 + 4 = 20-bit `extension_start_code`
    /// header and the four `f_code[s][t]` sub-fields plus
    /// `intra_dc_precision`, `picture_structure`, and the small
    /// trailing flags up to `composite_display_flag`. Higher-layer
    /// callers needing the composite-display sub-fields can rerun
    /// the extension parser directly once we land them.
    pub fn parse_with_extension(buf: &[u8]) -> Result<(Self, PictureCodingExtension)> {
        let header = Self::parse(buf)?;
        // Table 6-12: picture_coding_type '100' (dc intra-coded)
        // "shall not be used" in an ISO/IEC 13818-2 stream — only the
        // 11172-2 path (bare `parse`, no picture_coding_extension)
        // may see it.
        if header.picture_coding_type == PictureCodingType::DcIntra {
            return Err(Error::InvalidBitstream(
                "picture_coding_type: 100 (D-picture) shall not be used in MPEG-2 (Table 6-12)",
            ));
        }
        // Re-walk the buffer with a fresh BitReader to discover the
        // exact byte length consumed; the public parser does not
        // return the cursor and the trailing extra-info loop makes
        // it impossible to compute from the field shape alone.
        let after_header = picture_header_byte_length(buf)?;

        // §5.2.3 next_start_code(): skip zero stuffing bytes until
        // the next start-code prefix `00 00 01` is seen.
        let mut cursor = after_header;
        while cursor < buf.len() && buf[cursor] == 0x00 {
            cursor += 1;
        }
        if cursor >= buf.len()
            || buf[cursor] != 0x01
            || cursor < 2
            || buf[cursor - 1] != 0x00
            || buf[cursor - 2] != 0x00
        {
            return Err(Error::InvalidBitstream(
                "next_start_code(): missing '00 00 01' prefix after picture_header() (§5.2.3)",
            ));
        }
        let start_of_extension = cursor - 2;
        if start_of_extension + 4 > buf.len() {
            return Err(Error::ShortHeader);
        }
        if buf[start_of_extension + 3] != 0xB5 {
            return Err(Error::InvalidBitstream(
                "picture_coding_extension(): expected start code 0x000001B5 after picture_header() (§6.2.3.1)",
            ));
        }
        let extension = PictureCodingExtension::parse(&buf[start_of_extension..])?;
        Ok((header, extension))
    }
}

/// Re-walk the buffer to compute the exact byte length of a
/// `picture_header()`. The variable-length `extra_information_picture`
/// loop forces a second parse pass — the helper only returns the
/// cursor position, not a new struct, so it is cheap and the parse
/// errors are guaranteed identical to [`Mpeg2PictureHeader::parse`].
fn picture_header_byte_length(buf: &[u8]) -> Result<usize> {
    let mut br = BitReader::new(buf);
    let code = br.read_u32(32).map_err(|_| Error::ShortHeader)?;
    if code != PICTURE_START_CODE {
        return Err(Error::InvalidBitstream(
            "picture_start_code: expected 0x00000100 (§6.3.10)",
        ));
    }
    // temporal_reference (10) + picture_coding_type (3) + vbv_delay (16)
    let _tr = br.read_u32(10).map_err(|_| Error::ShortHeader)?;
    let coding_code = br.read_u32(3).map_err(|_| Error::ShortHeader)?;
    let pct = PictureCodingType::from_code(coding_code)?;
    let _vbv = br.read_u32(16).map_err(|_| Error::ShortHeader)?;
    if pct.uses_forward() {
        br.skip(4).map_err(|_| Error::ShortHeader)?;
    }
    if pct.uses_backward() {
        br.skip(4).map_err(|_| Error::ShortHeader)?;
    }
    loop {
        let bit = br.read_u32(1).map_err(|_| Error::ShortHeader)?;
        if bit == 0 {
            break;
        }
        br.skip(8).map_err(|_| Error::ShortHeader)?;
    }
    // §5.2.3 next_start_code() begins by byte-aligning with zero
    // stuffing bits. The pre-loop bit count for an I-picture is
    // 32 + 10 + 3 + 16 = 61 bits + the final 1-bit extra_bit_picture
    // = 62 bits → 6 zero pad bits to land on a byte boundary. For P
    // pictures: 62 + 4 = 66 → 6 pad bits. For B: 62 + 8 = 70 → 2.
    // Each accepted extra-info byte adds 9 bits to the running tally
    // but the loop's final '0' bit keeps the total a multiple of 1.
    // Independent of which arm we took, the spec says we now skip
    // bits until we reach a byte boundary.
    let bit_pos = br.bit_position();
    let pad = (8 - (bit_pos % 8)) % 8;
    if pad != 0 {
        br.skip(pad as u32).map_err(|_| Error::ShortHeader)?;
    }
    debug_assert!(br.is_byte_aligned());
    Ok(br.byte_position())
}

/// Parsed result of `picture_coding_extension()` (§6.2.3.1).
///
/// Only the leading fixed-shape portion of the extension is captured
/// here — everything up to and including `composite_display_flag`.
/// The optional five composite-display sub-fields are surfaced via
/// the [`PictureCodingExtension::composite_display`] accessor as a
/// raw bit-slice for now; full structural decoding is deferred to a
/// later round that needs the values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[doc(hidden)] // internal: §6.2.3.1 parser plumbing
pub struct PictureCodingExtension {
    /// `f_code[0][0]` — 4-bit forward horizontal motion-vector range.
    /// `15` (`0b1111`) means "unused" per §6.3.11.
    pub f_code_fwd_horiz: u8,
    /// `f_code[0][1]` — 4-bit forward vertical motion-vector range.
    pub f_code_fwd_vert: u8,
    /// `f_code[1][0]` — 4-bit backward horizontal range.
    pub f_code_bwd_horiz: u8,
    /// `f_code[1][1]` — 4-bit backward vertical range.
    pub f_code_bwd_vert: u8,
    /// 2-bit `intra_dc_precision` (Table 6-13). Maps to 8 / 9 / 10 /
    /// 11 DC bits as the integer increases.
    pub intra_dc_precision: u8,
    /// 2-bit `picture_structure` (Table 6-14). `0b00` is reserved
    /// and rejected.
    pub picture_structure: PictureStructure,
    /// `top_field_first` flag.
    pub top_field_first: bool,
    /// `frame_pred_frame_dct` flag.
    pub frame_pred_frame_dct: bool,
    /// `concealment_motion_vectors` flag.
    pub concealment_motion_vectors: bool,
    /// `q_scale_type` flag — affects the inverse quantisation
    /// (§7.4.2.2).
    pub q_scale_type: bool,
    /// `intra_vlc_format` flag — selects the coefficient VLC table.
    pub intra_vlc_format: bool,
    /// `alternate_scan` flag — selects the zigzag scan order.
    pub alternate_scan: bool,
    /// `repeat_first_field` flag.
    pub repeat_first_field: bool,
    /// `chroma_420_type` flag.
    pub chroma_420_type: bool,
    /// `progressive_frame` flag — when `1`, the two fields are
    /// co-temporal and a number of other restrictions apply
    /// (§6.3.11).
    pub progressive_frame: bool,
    /// `composite_display_flag`. The five trailing sub-fields are
    /// only present in the bitstream when this bit is `1`.
    pub composite_display_flag: bool,
}

/// `picture_structure` (§6.3.11, Table 6-14).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[doc(hidden)] // internal: §6.3.11 field of the hidden PictureCodingExtension
pub enum PictureStructure {
    /// `01` — top field only.
    TopField,
    /// `10` — bottom field only.
    BottomField,
    /// `11` — frame picture (both fields).
    Frame,
}

impl PictureStructure {
    fn from_code(code: u32) -> Result<Self> {
        match code {
            0b00 => Err(Error::InvalidBitstream(
                "picture_structure: reserved value 00 (Table 6-14)",
            )),
            0b01 => Ok(Self::TopField),
            0b10 => Ok(Self::BottomField),
            0b11 => Ok(Self::Frame),
            _ => unreachable!("picture_structure is a 2-bit field"),
        }
    }
}

impl PictureCodingExtension {
    /// Parse a `picture_coding_extension()` from a slice starting
    /// with the four start-code bytes `00 00 01 B5`.
    ///
    /// The trailing `next_start_code()` byte-align is not consumed.
    pub fn parse(buf: &[u8]) -> Result<Self> {
        let mut br = BitReader::new(buf);
        // §6.2.3.1: 32-bit extension_start_code, value 0x000001B5.
        let code = br.read_u32(32).map_err(|_| Error::ShortHeader)?;
        if code != 0x0000_01B5 {
            return Err(Error::InvalidBitstream(
                "extension_start_code: expected 0x000001B5 (§6.2.3.1)",
            ));
        }
        // 4-bit extension_start_code_identifier; Picture Coding
        // Extension ID is `1000` per Table 6-2.
        let id = br.read_u32(4).map_err(|_| Error::ShortHeader)?;
        if id != PICTURE_CODING_EXTENSION_ID {
            return Err(Error::InvalidBitstream(
                "extension_start_code_identifier: expected '1000' Picture Coding Extension ID (Table 6-2)",
            ));
        }

        let f_code_fwd_horiz = br.read_u32(4).map_err(|_| Error::ShortHeader)? as u8;
        let f_code_fwd_vert = br.read_u32(4).map_err(|_| Error::ShortHeader)? as u8;
        let f_code_bwd_horiz = br.read_u32(4).map_err(|_| Error::ShortHeader)? as u8;
        let f_code_bwd_vert = br.read_u32(4).map_err(|_| Error::ShortHeader)? as u8;
        // §6.3.11: the value zero is forbidden for every f_code[s][t].
        for (label, value) in [
            ("f_code[0][0]", f_code_fwd_horiz),
            ("f_code[0][1]", f_code_fwd_vert),
            ("f_code[1][0]", f_code_bwd_horiz),
            ("f_code[1][1]", f_code_bwd_vert),
        ] {
            if value == 0 {
                let _ = label;
                return Err(Error::InvalidBitstream(
                    "f_code[s][t]: forbidden value 0 (§6.3.11)",
                ));
            }
        }

        let intra_dc_precision = br.read_u32(2).map_err(|_| Error::ShortHeader)? as u8;
        let picture_structure =
            PictureStructure::from_code(br.read_u32(2).map_err(|_| Error::ShortHeader)?)?;
        let top_field_first = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let frame_pred_frame_dct = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let concealment_motion_vectors = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let q_scale_type = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let intra_vlc_format = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let alternate_scan = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let repeat_first_field = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let chroma_420_type = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let progressive_frame = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let composite_display_flag = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;

        Ok(Self {
            f_code_fwd_horiz,
            f_code_fwd_vert,
            f_code_bwd_horiz,
            f_code_bwd_vert,
            intra_dc_precision,
            picture_structure,
            top_field_first,
            frame_pred_frame_dct,
            concealment_motion_vectors,
            q_scale_type,
            intra_vlc_format,
            alternate_scan,
            repeat_first_field,
            chroma_420_type,
            progressive_frame,
            composite_display_flag,
        })
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact `picture_header()` fixtures plus negative
    //! cases for every spec-defined rejection site that this parser
    //! introduces. Composition with `picture_coding_extension()` is
    //! covered separately.
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Emit a minimal `picture_header()` for an I-frame, given the
    /// caller's `temporal_reference` and `vbv_delay`.
    fn write_picture_header_i(bw: &mut BitWriter, temporal_reference: u32, vbv_delay: u32) {
        bw.write_u32(PICTURE_START_CODE, 32);
        bw.write_u32(temporal_reference, 10);
        bw.write_u32(0b001, 3); // I
        bw.write_u32(vbv_delay, 16);
        // No forward/backward sub-fields.
        bw.write_bit(false); // terminating extra_bit_picture == 0
        bw.align_to_byte();
    }

    fn write_picture_header_p(
        bw: &mut BitWriter,
        temporal_reference: u32,
        vbv_delay: u32,
        full_pel_fwd: bool,
        fwd_f_code: u32,
    ) {
        bw.write_u32(PICTURE_START_CODE, 32);
        bw.write_u32(temporal_reference, 10);
        bw.write_u32(0b010, 3); // P
        bw.write_u32(vbv_delay, 16);
        bw.write_bit(full_pel_fwd);
        bw.write_u32(fwd_f_code, 3);
        bw.write_bit(false); // terminating extra_bit_picture == 0
        bw.align_to_byte();
    }

    #[allow(clippy::too_many_arguments)]
    fn write_picture_header_b(
        bw: &mut BitWriter,
        temporal_reference: u32,
        vbv_delay: u32,
        full_pel_fwd: bool,
        fwd_f_code: u32,
        full_pel_bwd: bool,
        bwd_f_code: u32,
    ) {
        bw.write_u32(PICTURE_START_CODE, 32);
        bw.write_u32(temporal_reference, 10);
        bw.write_u32(0b011, 3); // B
        bw.write_u32(vbv_delay, 16);
        bw.write_bit(full_pel_fwd);
        bw.write_u32(fwd_f_code, 3);
        bw.write_bit(full_pel_bwd);
        bw.write_u32(bwd_f_code, 3);
        bw.write_bit(false); // terminating extra_bit_picture == 0
        bw.align_to_byte();
    }

    #[test]
    fn parses_intra_picture_zero_temporal_reference() {
        // §6.3.10: the first picture after a GOP header has
        // temporal_reference 0. vbv_delay 0xFFFF marks VBR.
        let mut bw = BitWriter::new();
        write_picture_header_i(&mut bw, 0, 0xFFFF);
        let bytes = bw.finish();
        let pic = Mpeg2PictureHeader::parse(&bytes).expect("parse");
        assert_eq!(pic.temporal_reference, 0);
        assert_eq!(pic.picture_coding_type, PictureCodingType::Intra);
        assert_eq!(pic.vbv_delay, 0xFFFF);
        assert!(pic.full_pel_forward_vector.is_none());
        assert!(pic.fwd_f_code.is_none());
        assert!(pic.full_pel_backward_vector.is_none());
        assert!(pic.bwd_f_code.is_none());
        assert!(pic.extra_information_picture.is_empty());
    }

    #[test]
    fn parses_predictive_picture_with_fwd_fcode() {
        // P picture with the spec-mandated MPEG-2 sentinel value of
        // forward_f_code (0b111 = 7) and full_pel_forward_vector = 0.
        let mut bw = BitWriter::new();
        write_picture_header_p(&mut bw, 3, 0x1234, false, 0b111);
        let bytes = bw.finish();
        let pic = Mpeg2PictureHeader::parse(&bytes).expect("parse");
        assert_eq!(pic.temporal_reference, 3);
        assert_eq!(pic.picture_coding_type, PictureCodingType::Predictive);
        assert_eq!(pic.vbv_delay, 0x1234);
        assert_eq!(pic.full_pel_forward_vector, Some(false));
        assert_eq!(pic.fwd_f_code, Some(0b111));
        assert!(pic.full_pel_backward_vector.is_none());
        assert!(pic.bwd_f_code.is_none());
    }

    #[test]
    fn parses_bidirectional_picture_with_both_fcodes() {
        // B picture. Both directions carry sentinel `0b111` / 0.
        let mut bw = BitWriter::new();
        write_picture_header_b(&mut bw, 5, 0xCAFE, false, 0b111, false, 0b111);
        let bytes = bw.finish();
        let pic = Mpeg2PictureHeader::parse(&bytes).expect("parse");
        assert_eq!(pic.temporal_reference, 5);
        assert_eq!(pic.picture_coding_type, PictureCodingType::Bidirectional);
        assert_eq!(pic.vbv_delay, 0xCAFE);
        assert_eq!(pic.fwd_f_code, Some(0b111));
        assert_eq!(pic.bwd_f_code, Some(0b111));
    }

    #[test]
    fn captures_mpeg1_full_pel_flags() {
        // MPEG-1 may set full_pel_*_vector to 1; we preserve the
        // raw bit. Use a P picture so only forward is present.
        let mut bw = BitWriter::new();
        write_picture_header_p(&mut bw, 7, 0, true, 0b100);
        let bytes = bw.finish();
        let pic = Mpeg2PictureHeader::parse(&bytes).expect("parse");
        assert_eq!(pic.full_pel_forward_vector, Some(true));
        assert_eq!(pic.fwd_f_code, Some(0b100));
    }

    #[test]
    fn rejects_wrong_start_code() {
        let mut bw = BitWriter::new();
        write_picture_header_i(&mut bw, 0, 0);
        let mut bytes = bw.finish();
        bytes[3] = 0xB3; // sequence_header_code
        let err = Mpeg2PictureHeader::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_forbidden_picture_coding_type() {
        let mut bw = BitWriter::new();
        bw.write_u32(PICTURE_START_CODE, 32);
        bw.write_u32(0, 10);
        bw.write_u32(0b000, 3); // forbidden
        bw.write_u32(0, 16);
        bw.write_bit(false);
        bw.align_to_byte();
        let bytes = bw.finish();
        let err = Mpeg2PictureHeader::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn d_picture_coding_type_parses_bare_but_not_with_extension() {
        // 0b100 is the ISO/IEC 11172-2 dc intra-coded (D) picture: the
        // bare header parse accepts it (no f_code fields follow, per
        // the §2.4.2.5 type-2/3 gates), while the MPEG-2 chained
        // parser rejects it ("shall not be used", Table 6-12).
        let mut bw = BitWriter::new();
        bw.write_u32(PICTURE_START_CODE, 32);
        bw.write_u32(0, 10);
        bw.write_u32(0b100, 3);
        bw.write_u32(0, 16);
        bw.write_bit(false);
        bw.align_to_byte();
        // A following extension start-code prefix so the chained
        // parser reaches its own Table 6-12 gate.
        bw.write_u32(0x0000_01B5, 32);
        let bytes = bw.finish();

        let hdr = Mpeg2PictureHeader::parse(&bytes).expect("11172-2 path accepts D");
        assert_eq!(hdr.picture_coding_type, PictureCodingType::DcIntra);
        assert_eq!(hdr.fwd_f_code, None);
        assert_eq!(hdr.bwd_f_code, None);

        let err = Mpeg2PictureHeader::parse_with_extension(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_reserved_picture_coding_type() {
        for code in [0b101u32, 0b110, 0b111] {
            let mut bw = BitWriter::new();
            bw.write_u32(PICTURE_START_CODE, 32);
            bw.write_u32(0, 10);
            bw.write_u32(code, 3);
            bw.write_u32(0, 16);
            bw.write_bit(false);
            bw.align_to_byte();
            let bytes = bw.finish();
            let err = Mpeg2PictureHeader::parse(&bytes).unwrap_err();
            assert!(matches!(err, Error::InvalidBitstream(_)), "code {code:03b}");
        }
    }

    #[test]
    fn rejects_truncated_buffer() {
        // 4 bytes start code + 1 byte: parser needs at least 32 + 10
        // + 3 + 16 + 1 = 62 bits = 8 bytes (plus alignment).
        let bytes = vec![0x00, 0x00, 0x01, 0x00, 0x00];
        let err = Mpeg2PictureHeader::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    #[test]
    fn parses_extra_information_picture_byte() {
        // Force one non-conforming extra_bit_picture=1 byte to walk
        // the while-loop, then terminator. Two extra-info bytes
        // exercise the loop's repetition.
        let mut bw = BitWriter::new();
        bw.write_u32(PICTURE_START_CODE, 32);
        bw.write_u32(42, 10);
        bw.write_u32(0b001, 3); // I
        bw.write_u32(0xFFFF, 16);
        // First extra: bit=1, byte=0xA5.
        bw.write_bit(true);
        bw.write_u32(0xA5, 8);
        // Second extra: bit=1, byte=0x5A.
        bw.write_bit(true);
        bw.write_u32(0x5A, 8);
        // Terminator.
        bw.write_bit(false);
        bw.align_to_byte();
        let bytes = bw.finish();
        let pic = Mpeg2PictureHeader::parse(&bytes).expect("parse");
        assert_eq!(pic.temporal_reference, 42);
        assert_eq!(pic.extra_information_picture, vec![0xA5, 0x5A]);
    }

    #[test]
    fn picture_coding_type_helpers() {
        assert!(!PictureCodingType::Intra.uses_forward());
        assert!(!PictureCodingType::Intra.uses_backward());
        assert!(PictureCodingType::Predictive.uses_forward());
        assert!(!PictureCodingType::Predictive.uses_backward());
        assert!(PictureCodingType::Bidirectional.uses_forward());
        assert!(PictureCodingType::Bidirectional.uses_backward());
    }

    // --- picture_coding_extension() tests -------------------------

    #[allow(clippy::too_many_arguments)]
    fn write_picture_coding_extension(
        bw: &mut BitWriter,
        f00: u32,
        f01: u32,
        f10: u32,
        f11: u32,
        intra_dc_precision: u32,
        picture_structure: u32,
        top_field_first: bool,
        frame_pred_frame_dct: bool,
        concealment_motion_vectors: bool,
        q_scale_type: bool,
        intra_vlc_format: bool,
        alternate_scan: bool,
        repeat_first_field: bool,
        chroma_420_type: bool,
        progressive_frame: bool,
        composite_display_flag: bool,
    ) {
        bw.write_u32(0x0000_01B5, 32);
        bw.write_u32(PICTURE_CODING_EXTENSION_ID, 4);
        bw.write_u32(f00, 4);
        bw.write_u32(f01, 4);
        bw.write_u32(f10, 4);
        bw.write_u32(f11, 4);
        bw.write_u32(intra_dc_precision, 2);
        bw.write_u32(picture_structure, 2);
        bw.write_bit(top_field_first);
        bw.write_bit(frame_pred_frame_dct);
        bw.write_bit(concealment_motion_vectors);
        bw.write_bit(q_scale_type);
        bw.write_bit(intra_vlc_format);
        bw.write_bit(alternate_scan);
        bw.write_bit(repeat_first_field);
        bw.write_bit(chroma_420_type);
        bw.write_bit(progressive_frame);
        bw.write_bit(composite_display_flag);
        bw.align_to_byte();
    }

    #[test]
    fn parses_minimal_picture_coding_extension() {
        // I-picture's MPEG-2 PCE: f_code = 15 (unused) across the
        // board, intra_dc_precision = 0 (8 bits), frame picture,
        // top_field_first = 1, frame_pred_frame_dct = 1,
        // progressive_frame = 1, composite_display_flag = 0.
        let mut bw = BitWriter::new();
        write_picture_coding_extension(
            &mut bw, 15, 15, 15, 15, 0, 0b11, true, true, false, false, false, false, false, false,
            true, false,
        );
        let bytes = bw.finish();
        let ext = PictureCodingExtension::parse(&bytes).expect("parse");
        assert_eq!(ext.f_code_fwd_horiz, 15);
        assert_eq!(ext.f_code_fwd_vert, 15);
        assert_eq!(ext.f_code_bwd_horiz, 15);
        assert_eq!(ext.f_code_bwd_vert, 15);
        assert_eq!(ext.intra_dc_precision, 0);
        assert_eq!(ext.picture_structure, PictureStructure::Frame);
        assert!(ext.top_field_first);
        assert!(ext.frame_pred_frame_dct);
        assert!(!ext.concealment_motion_vectors);
        assert!(ext.progressive_frame);
        assert!(!ext.composite_display_flag);
    }

    #[test]
    fn rejects_zero_f_code_in_extension() {
        let mut bw = BitWriter::new();
        write_picture_coding_extension(
            &mut bw, 0, 15, 15, 15, 0, 0b11, true, true, false, false, false, false, false, false,
            true, false,
        );
        let bytes = bw.finish();
        let err = PictureCodingExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_reserved_picture_structure() {
        let mut bw = BitWriter::new();
        write_picture_coding_extension(
            &mut bw, 15, 15, 15, 15, 0, 0b00, // reserved
            true, true, false, false, false, false, false, false, true, false,
        );
        let bytes = bw.finish();
        let err = PictureCodingExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_wrong_extension_id_for_pce() {
        // Build a buffer with Sequence Extension ID (0001) instead
        // of Picture Coding Extension ID (1000).
        let mut bw = BitWriter::new();
        bw.write_u32(0x0000_01B5, 32);
        bw.write_u32(0b0001, 4);
        // Pad the rest of a PCE-shaped buffer with zeros so the
        // parser fails at the id check, not on EOF.
        for _ in 0..8 {
            bw.write_u32(0, 4);
        }
        bw.align_to_byte();
        let bytes = bw.finish();
        let err = PictureCodingExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_truncated_picture_coding_extension() {
        let bytes = vec![0x00, 0x00, 0x01, 0xB5, 0x80]; // start code + id nibble
        let err = PictureCodingExtension::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    #[test]
    fn picture_structure_decodes_all_legal_codes() {
        for (code, expected) in [
            (0b01u32, PictureStructure::TopField),
            (0b10, PictureStructure::BottomField),
            (0b11, PictureStructure::Frame),
        ] {
            let mut bw = BitWriter::new();
            write_picture_coding_extension(
                &mut bw, 15, 15, 15, 15, 0, code, true, true, false, false, false, false, false,
                false, true, false,
            );
            let bytes = bw.finish();
            let ext = PictureCodingExtension::parse(&bytes).expect("parse");
            assert_eq!(ext.picture_structure, expected);
        }
    }

    // --- composition ------------------------------------------------

    #[test]
    fn composes_picture_header_with_extension() {
        let mut bw = BitWriter::new();
        write_picture_header_i(&mut bw, 11, 0xFFFF);
        // The picture_header() writers above leave us byte-aligned,
        // so the extension follows immediately.
        write_picture_coding_extension(
            &mut bw, 15, 15, 15, 15, 0, 0b11, true, true, false, false, false, false, false, false,
            true, false,
        );
        let bytes = bw.finish();
        let (hdr, ext) =
            Mpeg2PictureHeader::parse_with_extension(&bytes).expect("compose header + extension");
        assert_eq!(hdr.temporal_reference, 11);
        assert_eq!(hdr.picture_coding_type, PictureCodingType::Intra);
        assert_eq!(ext.f_code_fwd_horiz, 15);
        assert_eq!(ext.picture_structure, PictureStructure::Frame);
    }

    #[test]
    fn composes_picture_header_with_extension_after_zero_stuffing() {
        // §5.2.3 allows zero-byte stuffing between any two layers.
        let mut bw = BitWriter::new();
        write_picture_header_p(&mut bw, 1, 0, false, 0b111);
        let mut bytes = bw.finish();
        bytes.extend_from_slice(&[0x00, 0x00]); // two stuffing bytes
        let mut tail = BitWriter::new();
        write_picture_coding_extension(
            &mut tail, 15, 15, 15, 15, 0, 0b11, true, true, false, false, false, false, false,
            false, true, false,
        );
        bytes.extend_from_slice(&tail.finish());
        let (hdr, ext) =
            Mpeg2PictureHeader::parse_with_extension(&bytes).expect("compose with stuffing");
        assert_eq!(hdr.picture_coding_type, PictureCodingType::Predictive);
        assert_eq!(ext.intra_dc_precision, 0);
    }

    #[test]
    fn rejects_missing_extension_after_picture_header() {
        let mut bw = BitWriter::new();
        write_picture_header_i(&mut bw, 0, 0xFFFF);
        let mut bytes = bw.finish();
        // Sequence_end_code instead of extension_start_code.
        bytes.extend_from_slice(&[0x00, 0x00, 0x01, 0xB7]);
        let err = Mpeg2PictureHeader::parse_with_extension(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn debug_impl_smoke() {
        let mut bw = BitWriter::new();
        write_picture_header_i(&mut bw, 0, 0xFFFF);
        let bytes = bw.finish();
        let pic = Mpeg2PictureHeader::parse(&bytes).expect("parse");
        let s = format!("{pic:?}");
        assert!(s.contains("Intra"));
        assert!(s.contains("Mpeg2PictureHeader") || s.contains("temporal_reference"));
    }
}
