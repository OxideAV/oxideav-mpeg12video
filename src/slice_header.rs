//! Parser for the MPEG-1 / MPEG-2 video `slice()` header bits per
//! ISO/IEC 13818-2 (Recommendation ITU-T H.262) §6.2.4 with the field
//! semantics from §6.3.16.
//!
//! Round 5 limits its scope to the *header* of a slice — everything
//! the spec spells out *before* the macroblock loop:
//!
//! 1. The 32-bit `slice_start_code`. Per §6.3.16 the first 24 bits are
//!    the start-code prefix `00 00 01`; the trailing byte is the
//!    `slice_vertical_position` and shall be in the range
//!    `0x01..=0xAF` inclusive (Table 6-1).
//! 2. The optional 3-bit `slice_vertical_position_extension`, gated on
//!    a `sequence_header()` carrying `vertical_size > 2800`. The
//!    caller must pass that fact in (via [`SliceContext::vertical_size`])
//!    so the parser knows whether the bit is present.
//! 3. The optional 7-bit `priority_breakpoint`, gated on
//!    `sequence_scalable_extension()` being present *and*
//!    `scalable_mode == "data partitioning"` (§6.3.16). Round 5 does
//!    not yet parse `sequence_scalable_extension()`, so the caller
//!    must signal this through [`SliceContext::priority_breakpoint_present`].
//! 4. The mandatory 5-bit `quantiser_scale_code`, with the
//!    spec-defined forbidden-zero check (§6.3.16).
//! 5. The optional intra-slice prelude `intra_slice_flag` +
//!    `intra_slice` + `reserved_bits` + the `extra_information_slice`
//!    byte loop. The prelude is present iff `nextbits() == '1'`.
//!    `reserved_bits` is required to be zero (§6.3.16).
//! 6. The terminating `extra_bit_slice` bit (required to be `'0'`).
//!
//! The macroblock loop that follows is **out of scope for round 5**;
//! the parser stops as soon as the terminating `extra_bit_slice` is
//! consumed and returns a [`SliceHeader`] together with the byte
//! position at which the body begins (`body_bit_position` measured in
//! *bits* from the start of the slice, since the body is not
//! byte-aligned in MPEG-2 streams).
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)).

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// Lower bound of the valid range of `slice_vertical_position` per
/// Table 6-1: `0x01`. `0x00` is `picture_start_code`.
pub const SLICE_VERTICAL_POSITION_MIN: u8 = 0x01;

/// Upper bound of the valid range of `slice_vertical_position` per
/// Table 6-1: `0xAF`. `0xB0`+ are reserved or other start codes.
pub const SLICE_VERTICAL_POSITION_MAX: u8 = 0xAF;

/// Caller-supplied context that decides which optional fields appear
/// in the slice header. Both bits flow from layers *above* `slice()`
/// in the bitstream — the slice header itself does not signal them —
/// so the caller is required to forward them in.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SliceContext {
    /// The full 14-bit `vertical_size` synthesised by
    /// [`crate::Mpeg2Sequence`] (§6.3.3 + §6.3.5). The 3-bit
    /// `slice_vertical_position_extension` is present iff this value
    /// is strictly greater than 2800 (§6.2.4).
    pub vertical_size: u32,
    /// `true` when a `sequence_scalable_extension()` is present *and*
    /// its `scalable_mode == "data partitioning"` (§6.2.4). Round 5
    /// does not parse the scalable extension, so the parser cannot
    /// derive this internally; the caller passes it in. Defaults to
    /// `false`, which matches every non-scalable MPEG-2 stream.
    pub priority_breakpoint_present: bool,
}

impl SliceContext {
    /// Convenience constructor for the dominant non-scalable case:
    /// vertical size ≤ 2800 and no data-partitioning scalable
    /// extension, i.e. neither `slice_vertical_position_extension`
    /// nor `priority_breakpoint` is in the bitstream.
    pub const fn non_scalable(vertical_size: u32) -> Self {
        Self {
            vertical_size,
            priority_breakpoint_present: false,
        }
    }
}

/// Parsed result of the `slice()` header bits — everything before the
/// macroblock loop.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SliceHeader {
    /// Byte taken from the low 8 bits of the `slice_start_code`. The
    /// spec guarantees `0x01..=0xAF` (Table 6-1, §6.3.16).
    pub slice_vertical_position: u8,
    /// Optional 3-bit extension surfaced only when
    /// `vertical_size > 2800` (§6.2.4 / §6.3.16). The composite
    /// `mb_row` is `((extension as u32) << 7) + svp - 1` per §6.3.16.
    pub slice_vertical_position_extension: Option<u8>,
    /// Optional 7-bit `priority_breakpoint`, present only when the
    /// surrounding sequence is data-partitioned (§6.3.16 / Table
    /// 7-30 — Table 7-30 itself is *not* decoded here; the raw value
    /// is surfaced for the higher layer).
    pub priority_breakpoint: Option<u8>,
    /// 5-bit `quantiser_scale_code` in `1..=31` (§6.3.16, zero
    /// forbidden).
    pub quantiser_scale_code: u8,
    /// `intra_slice_flag`, surfaced only when the conditional
    /// `nextbits() == '1'` prelude is present (§6.2.4). When the
    /// prelude is absent §6.3.16 says `intra_slice` "shall be
    /// assumed to have the value zero".
    pub intra_slice_flag: Option<bool>,
    /// `intra_slice`, surfaced only when the conditional prelude is
    /// present. See `intra_slice_flag` above for the absent-case
    /// semantics.
    pub intra_slice: Option<bool>,
    /// Bytes accumulated by the `while ( nextbits() == '1' )`
    /// `extra_information_slice` loop. Per §6.3.16 a conforming
    /// stream "shall not contain this syntax element"; we preserve
    /// the bytes anyway for round-trip fidelity. The loop only runs
    /// when the intra-slice prelude was present (`nextbits() == '1'`
    /// is checked inside that block in §6.2.4).
    pub extra_information_slice: Vec<u8>,
    /// Position (in *bits*, counted from the start of the slice
    /// buffer the parser was handed) at which the first
    /// `macroblock()` begins. The slice header is **not** byte-aligned
    /// in MPEG-2 streams, so a byte cursor would lose the partial-byte
    /// remainder. Round-5 callers can use this to seed a downstream
    /// macroblock-layer parser.
    pub body_bit_position: u64,
}

impl SliceHeader {
    /// Parse the `slice()` header from a slice that starts with the
    /// four bytes of `slice_start_code` (`00 00 01 XX` with
    /// `XX ∈ 0x01..=0xAF`).
    ///
    /// `ctx` carries the two pieces of state the header itself does
    /// not signal: `vertical_size` (for
    /// `slice_vertical_position_extension`) and
    /// `priority_breakpoint_present` (for the data-partitioning
    /// `priority_breakpoint`). See [`SliceContext`].
    pub fn parse(buf: &[u8], ctx: SliceContext) -> Result<Self> {
        let mut br = BitReader::new(buf);

        // §6.3.16: 32-bit slice_start_code with the prefix 0x000001
        // in the top three bytes and slice_vertical_position in the
        // low byte. We validate the prefix and the SVP range
        // separately so the error message points at the right field.
        let prefix = br.read_u32(24).map_err(|_| Error::ShortHeader)?;
        if prefix != 0x00_00_01 {
            return Err(Error::InvalidBitstream(
                "slice_start_code: expected 24-bit prefix 0x000001 (§6.3.16)",
            ));
        }
        let slice_vertical_position = br.read_u32(8).map_err(|_| Error::ShortHeader)? as u8;
        if !(SLICE_VERTICAL_POSITION_MIN..=SLICE_VERTICAL_POSITION_MAX)
            .contains(&slice_vertical_position)
        {
            return Err(Error::InvalidBitstream(
                "slice_vertical_position: outside 0x01..=0xAF (Table 6-1, §6.3.16)",
            ));
        }

        // §6.2.4: 3-bit slice_vertical_position_extension iff
        // vertical_size > 2800. When present, §6.3.16 constrains
        // slice_vertical_position to 1..=128 — checked here.
        let slice_vertical_position_extension = if ctx.vertical_size > 2800 {
            let ext = br.read_u32(3).map_err(|_| Error::ShortHeader)? as u8;
            if !(1..=128).contains(&slice_vertical_position) {
                return Err(Error::InvalidBitstream(
                    "slice_vertical_position: shall be in [1:128] when the extension is present (§6.3.16)",
                ));
            }
            Some(ext)
        } else {
            None
        };

        // §6.2.4: 7-bit priority_breakpoint iff sequence_scalable_extension()
        // is present with scalable_mode == "data partitioning". The
        // caller carries this fact in `ctx`.
        let priority_breakpoint = if ctx.priority_breakpoint_present {
            Some(br.read_u32(7).map_err(|_| Error::ShortHeader)? as u8)
        } else {
            None
        };

        // §6.3.16: 5-bit quantiser_scale_code in 1..=31 (zero
        // forbidden).
        let quantiser_scale_code = br.read_u32(5).map_err(|_| Error::ShortHeader)? as u8;
        if quantiser_scale_code == 0 {
            return Err(Error::InvalidBitstream(
                "quantiser_scale_code: forbidden value 0 (§6.3.16)",
            ));
        }

        // §6.2.4: `if ( nextbits() == '1' ) { intra_slice_flag,
        // intra_slice, reserved_bits, while ( nextbits() == '1' )
        // { extra_bit_slice; extra_information_slice } }`.
        let (intra_slice_flag, intra_slice, extra_information_slice) =
            if br.peek_u32(1).map_err(|_| Error::ShortHeader)? == 1 {
                // intra_slice_flag is the very '1' we just peeked at,
                // consumed here.
                let intra_slice_flag = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
                debug_assert!(
                    intra_slice_flag,
                    "intra_slice_flag must be 1 to reach this branch (§6.2.4)"
                );
                let intra_slice = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
                let reserved_bits = br.read_u32(7).map_err(|_| Error::ShortHeader)?;
                if reserved_bits != 0 {
                    return Err(Error::InvalidBitstream(
                        "reserved_bits: shall have the value zero (§6.3.16)",
                    ));
                }
                let mut bytes = Vec::new();
                loop {
                    let bit = br.read_u32(1).map_err(|_| Error::ShortHeader)?;
                    if bit == 0 {
                        // This zero bit *is* the terminating
                        // `extra_bit_slice == '0'` of §6.2.4. The
                        // outer while-loop and the trailing extra_bit_slice
                        // share the same bit string.
                        return Ok(Self {
                            slice_vertical_position,
                            slice_vertical_position_extension,
                            priority_breakpoint,
                            quantiser_scale_code,
                            intra_slice_flag: Some(intra_slice_flag),
                            intra_slice: Some(intra_slice),
                            extra_information_slice: bytes,
                            body_bit_position: br.bit_position(),
                        });
                    }
                    let extra = br.read_u32(8).map_err(|_| Error::ShortHeader)? as u8;
                    bytes.push(extra);
                }
            } else {
                (None, None, Vec::new())
            };

        // §6.2.4: terminating `extra_bit_slice` with value '0' when
        // the conditional prelude was absent. (When the prelude was
        // present we already returned above, because the same zero
        // bit doubles as the loop terminator and the extra_bit_slice.)
        let bit = br.read_u32(1).map_err(|_| Error::ShortHeader)?;
        if bit != 0 {
            return Err(Error::InvalidBitstream(
                "extra_bit_slice: expected terminating '0' (§6.3.16)",
            ));
        }

        Ok(Self {
            slice_vertical_position,
            slice_vertical_position_extension,
            priority_breakpoint,
            quantiser_scale_code,
            intra_slice_flag,
            intra_slice,
            extra_information_slice,
            body_bit_position: br.bit_position(),
        })
    }

    /// Compute `mb_row` per §6.3.16. The math is `mb_row =
    /// (extension << 7) + svp - 1` when the extension is present and
    /// `mb_row = svp - 1` otherwise.
    pub fn mb_row(&self) -> u32 {
        match self.slice_vertical_position_extension {
            Some(ext) => (u32::from(ext) << 7) + u32::from(self.slice_vertical_position) - 1,
            None => u32::from(self.slice_vertical_position) - 1,
        }
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact `slice()` header fixtures + negative tests
    //! for every spec-defined rejection site that this parser
    //! introduces.
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Emit a minimal slice-header byte stream for a 5-bit
    /// `quantiser_scale_code` with no scalable extras and no
    /// intra_slice prelude.
    fn write_minimal_slice_header(bw: &mut BitWriter, svp: u8, q_scale: u32) {
        bw.write_u32(0x00_00_01, 24);
        bw.write_u32(u32::from(svp), 8);
        bw.write_u32(q_scale, 5);
        // No intra_slice prelude => terminating extra_bit_slice == '0'.
        bw.write_bit(false);
        bw.align_to_byte();
    }

    #[test]
    fn parses_minimal_slice_header_first_row() {
        let mut bw = BitWriter::new();
        write_minimal_slice_header(&mut bw, 0x01, 8);
        let bytes = bw.finish();
        let sh = SliceHeader::parse(&bytes, SliceContext::non_scalable(240)).expect("parse");
        assert_eq!(sh.slice_vertical_position, 0x01);
        assert!(sh.slice_vertical_position_extension.is_none());
        assert!(sh.priority_breakpoint.is_none());
        assert_eq!(sh.quantiser_scale_code, 8);
        assert!(sh.intra_slice_flag.is_none());
        assert!(sh.intra_slice.is_none());
        assert!(sh.extra_information_slice.is_empty());
        assert_eq!(sh.mb_row(), 0);
        // 24 + 8 + 5 + 1 = 38 bits.
        assert_eq!(sh.body_bit_position, 38);
    }

    #[test]
    fn parses_slice_header_with_intra_slice_prelude() {
        // intra_slice_flag = 1, intra_slice = 1, reserved_bits = 0,
        // no extra_information_slice. The intra-slice block ends
        // with a zero bit which doubles as the terminating
        // extra_bit_slice per §6.2.4.
        let mut bw = BitWriter::new();
        bw.write_u32(0x00_00_01, 24);
        bw.write_u32(0x10, 8); // svp
        bw.write_u32(31, 5); // q_scale = max
        bw.write_bit(true); // intra_slice_flag (the nextbits() == '1' check)
        bw.write_bit(true); // intra_slice
        bw.write_u32(0, 7); // reserved_bits
        bw.write_bit(false); // extra_bit_slice == '0' (loop terminator)
        bw.align_to_byte();
        let bytes = bw.finish();
        let sh = SliceHeader::parse(&bytes, SliceContext::non_scalable(240)).expect("parse");
        assert_eq!(sh.slice_vertical_position, 0x10);
        assert_eq!(sh.quantiser_scale_code, 31);
        assert_eq!(sh.intra_slice_flag, Some(true));
        assert_eq!(sh.intra_slice, Some(true));
        assert!(sh.extra_information_slice.is_empty());
    }

    #[test]
    fn parses_slice_header_with_extra_information_slice_bytes() {
        let mut bw = BitWriter::new();
        bw.write_u32(0x00_00_01, 24);
        bw.write_u32(0x05, 8);
        bw.write_u32(4, 5);
        bw.write_bit(true); // intra_slice_flag
        bw.write_bit(false); // intra_slice
        bw.write_u32(0, 7); // reserved_bits
                            // Two non-conforming extra_bit_slice=1 bytes, then terminator.
        bw.write_bit(true);
        bw.write_u32(0xAB, 8);
        bw.write_bit(true);
        bw.write_u32(0xCD, 8);
        bw.write_bit(false);
        bw.align_to_byte();
        let bytes = bw.finish();
        let sh = SliceHeader::parse(&bytes, SliceContext::non_scalable(240)).expect("parse");
        assert_eq!(sh.intra_slice_flag, Some(true));
        assert_eq!(sh.intra_slice, Some(false));
        assert_eq!(sh.extra_information_slice, vec![0xAB, 0xCD]);
    }

    #[test]
    fn parses_slice_header_with_priority_breakpoint() {
        let mut bw = BitWriter::new();
        bw.write_u32(0x00_00_01, 24);
        bw.write_u32(0x02, 8);
        bw.write_u32(0x42, 7); // priority_breakpoint
        bw.write_u32(7, 5); // q_scale
        bw.write_bit(false); // no intra_slice prelude
        bw.align_to_byte();
        let bytes = bw.finish();
        let ctx = SliceContext {
            vertical_size: 720,
            priority_breakpoint_present: true,
        };
        let sh = SliceHeader::parse(&bytes, ctx).expect("parse");
        assert_eq!(sh.slice_vertical_position, 0x02);
        assert_eq!(sh.priority_breakpoint, Some(0x42));
        assert_eq!(sh.quantiser_scale_code, 7);
    }

    #[test]
    fn parses_slice_header_with_vertical_position_extension() {
        let mut bw = BitWriter::new();
        bw.write_u32(0x00_00_01, 24);
        bw.write_u32(0x80, 8); // svp = 128 (max with extension)
        bw.write_u32(0b011, 3); // slice_vertical_position_extension = 3
        bw.write_u32(2, 5); // q_scale
        bw.write_bit(false); // no intra_slice prelude
        bw.align_to_byte();
        let bytes = bw.finish();
        let ctx = SliceContext::non_scalable(2880); // > 2800 triggers extension
        let sh = SliceHeader::parse(&bytes, ctx).expect("parse");
        assert_eq!(sh.slice_vertical_position, 0x80);
        assert_eq!(sh.slice_vertical_position_extension, Some(3));
        // mb_row = (3 << 7) + 128 - 1 = 384 + 128 - 1 = 511.
        assert_eq!(sh.mb_row(), 511);
    }

    #[test]
    fn rejects_wrong_start_code_prefix() {
        // Use 0x000000 (not a valid start prefix).
        let mut bw = BitWriter::new();
        bw.write_u32(0x00_00_00, 24);
        bw.write_u32(0x01, 8);
        bw.write_u32(1, 5);
        bw.write_bit(false);
        bw.align_to_byte();
        let bytes = bw.finish();
        let err = SliceHeader::parse(&bytes, SliceContext::non_scalable(240)).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_zero_slice_vertical_position() {
        // svp = 0x00 is picture_start_code (Table 6-1), not a slice.
        let mut bw = BitWriter::new();
        bw.write_u32(0x00_00_01, 24);
        bw.write_u32(0x00, 8);
        bw.write_u32(1, 5);
        bw.write_bit(false);
        bw.align_to_byte();
        let bytes = bw.finish();
        let err = SliceHeader::parse(&bytes, SliceContext::non_scalable(240)).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_out_of_range_slice_vertical_position() {
        // svp = 0xB0 is reserved (Table 6-1).
        let mut bw = BitWriter::new();
        bw.write_u32(0x00_00_01, 24);
        bw.write_u32(0xB0, 8);
        bw.write_u32(1, 5);
        bw.write_bit(false);
        bw.align_to_byte();
        let bytes = bw.finish();
        let err = SliceHeader::parse(&bytes, SliceContext::non_scalable(240)).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_slice_vertical_position_above_128_when_extension_present() {
        // §6.3.16 says svp shall be in [1:128] when extension is
        // present. 129 trips the second range check.
        let mut bw = BitWriter::new();
        bw.write_u32(0x00_00_01, 24);
        bw.write_u32(129, 8);
        bw.write_u32(0, 3); // extension
        bw.write_u32(1, 5);
        bw.write_bit(false);
        bw.align_to_byte();
        let bytes = bw.finish();
        let err = SliceHeader::parse(&bytes, SliceContext::non_scalable(2880)).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_zero_quantiser_scale_code() {
        let mut bw = BitWriter::new();
        bw.write_u32(0x00_00_01, 24);
        bw.write_u32(0x01, 8);
        bw.write_u32(0, 5); // forbidden
        bw.write_bit(false);
        bw.align_to_byte();
        let bytes = bw.finish();
        let err = SliceHeader::parse(&bytes, SliceContext::non_scalable(240)).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_non_zero_reserved_bits() {
        let mut bw = BitWriter::new();
        bw.write_u32(0x00_00_01, 24);
        bw.write_u32(0x01, 8);
        bw.write_u32(8, 5);
        bw.write_bit(true); // intra_slice_flag
        bw.write_bit(false); // intra_slice
        bw.write_u32(0x01, 7); // reserved_bits != 0
        bw.write_bit(false);
        bw.align_to_byte();
        let bytes = bw.finish();
        let err = SliceHeader::parse(&bytes, SliceContext::non_scalable(240)).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_truncated_slice_header() {
        // Only the four start-code bytes, no quantiser_scale_code.
        let bytes = vec![0x00, 0x00, 0x01, 0x01];
        let err = SliceHeader::parse(&bytes, SliceContext::non_scalable(240)).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    #[test]
    fn slice_context_non_scalable_helper() {
        let ctx = SliceContext::non_scalable(1080);
        assert_eq!(ctx.vertical_size, 1080);
        assert!(!ctx.priority_breakpoint_present);
    }

    #[test]
    fn mb_row_without_extension() {
        // svp = 5 (no extension) → mb_row = 4.
        let mut bw = BitWriter::new();
        write_minimal_slice_header(&mut bw, 5, 1);
        let bytes = bw.finish();
        let sh = SliceHeader::parse(&bytes, SliceContext::non_scalable(240)).expect("parse");
        assert_eq!(sh.mb_row(), 4);
    }

    #[test]
    fn slice_vertical_position_bounds_are_table_6_1() {
        // Confirm the constants match Table 6-1 literally.
        assert_eq!(SLICE_VERTICAL_POSITION_MIN, 0x01);
        assert_eq!(SLICE_VERTICAL_POSITION_MAX, 0xAF);
    }

    #[test]
    fn debug_impl_smoke() {
        let mut bw = BitWriter::new();
        write_minimal_slice_header(&mut bw, 0x01, 8);
        let bytes = bw.finish();
        let sh = SliceHeader::parse(&bytes, SliceContext::non_scalable(240)).expect("parse");
        let s = format!("{sh:?}");
        assert!(s.contains("SliceHeader"));
        assert!(s.contains("slice_vertical_position"));
    }
}
