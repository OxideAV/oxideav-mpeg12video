//! Parser for the macroblock-layer `quantizer_scale` per ISO/IEC
//! 11172-2:1993 (MPEG-1 Video) §2.4.2.7 (syntax) with the field
//! semantics from §2.4.3.6.
//!
//! Round 9 fills the macroblock-layer gap that sits between round 7's
//! [`crate::MacroblockType`] and the motion-vector / coded-block-pattern
//! fields. Within `macroblock()` the spec reads, immediately after
//! `macroblock_type`:
//!
//! ```text
//! macroblock_type                                     1-6   vlclbf
//! if ( macroblock_quant )
//!     quantizer_scale                                 5     uimsbf
//! ```
//!
//! `macroblock_quant` is one of the boolean flags derived from
//! `macroblock_type` (Tables B.2a..B.2d, §2.4.3.6). When it is set a
//! fresh 5-bit `quantizer_scale` overrides the value the decoder has
//! been carrying since the slice layer; when it is clear the field is
//! absent and the previously established scale persists.
//!
//! Per §2.4.3.6 `quantizer_scale` is *"an unsigned integer in the range
//! 1 to 31 used to scale the reconstruction level of the retrieved DCT
//! coefficient levels. The value zero is forbidden. The decoder shall
//! use this value until another `quantizer_scale` is encountered either
//! at the slice layer or the macroblock layer."*
//!
//! This module decodes the wire field only; threading the live
//! quantiser value through the decode state (the slice-layer
//! `quantizer_scale` of §2.4.2.6 vs. the macroblock-layer override of
//! §2.4.2.7) and the dequantisation arithmetic itself are out of scope
//! for round 9. The field is returned together with the post-field bit
//! position so callers can chain into the motion-vector fields without
//! losing the partial-byte cursor.
//!
//! Spec citations refer to ISO/IEC 11172-2:1993 (MPEG-1 Video)
//! §2.4.2.7 and §2.4.3.6.

use oxideav_core::bits::BitReader;

use crate::macroblock_type::MacroblockType;
use crate::{Error, Result};

/// The smallest legal `quantizer_scale`. `0` is forbidden (§2.4.3.6).
pub const QUANTIZER_SCALE_MIN: u8 = 1;

/// The largest legal `quantizer_scale`. The field is 5 bits, so the
/// maximum representable value `31` is also the spec maximum (§2.4.3.6).
pub const QUANTIZER_SCALE_MAX: u8 = 31;

/// The macroblock-layer `quantizer_scale` decision and (when present)
/// the decoded value.
///
/// `quantizer_scale` is `Some(v)` with `v ∈ 1..=31` exactly when
/// `macroblock_quant` was set; otherwise it is `None` and the decoder
/// keeps using the value established by an earlier slice- or
/// macroblock-layer `quantizer_scale` (§2.4.3.6).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuantizerScale {
    /// The decoded 5-bit `quantizer_scale` in `1..=31` when
    /// `macroblock_quant` was set; `None` when the field was absent.
    pub quantizer_scale: Option<u8>,
    /// Bit position (relative to the start of the buffer the
    /// [`BitReader`] was created from) right after the field. When the
    /// field was absent this equals the reader's position on entry —
    /// no bits are consumed.
    pub bit_position_after: u64,
}

impl QuantizerScale {
    /// Parse the macroblock-layer `quantizer_scale` field conditional on
    /// `macroblock_quant`.
    ///
    /// When `macroblock_quant` is `false` no bits are read and
    /// [`QuantizerScale::quantizer_scale`] is `None`. When it is `true`
    /// the next 5 bits are read as an unsigned integer; the forbidden
    /// value `0` is rejected (§2.4.3.6).
    ///
    /// Errors:
    /// * [`Error::InvalidBitstream`] if `macroblock_quant` was set but
    ///   the decoded `quantizer_scale` is the forbidden value `0`.
    /// * [`Error::ShortHeader`] if `macroblock_quant` was set but the
    ///   bitstream ends before the 5-bit field could be read.
    pub fn parse(br: &mut BitReader<'_>, macroblock_quant: bool) -> Result<Self> {
        if !macroblock_quant {
            return Ok(Self {
                quantizer_scale: None,
                bit_position_after: br.bit_position(),
            });
        }

        let value = br.read_u32(5).map_err(|_| Error::ShortHeader)? as u8;
        if value == 0 {
            return Err(Error::InvalidBitstream(
                "quantizer_scale: forbidden value 0 (§2.4.3.6)",
            ));
        }
        debug_assert!(
            (QUANTIZER_SCALE_MIN..=QUANTIZER_SCALE_MAX).contains(&value),
            "a non-zero 5-bit field is always within 1..=31"
        );

        Ok(Self {
            quantizer_scale: Some(value),
            bit_position_after: br.bit_position(),
        })
    }

    /// Parse the field using the `macroblock_quant` flag carried by a
    /// previously decoded [`MacroblockType`]. Convenience wrapper that
    /// pairs the two macroblock-layer fields the spec reads back to
    /// back (§2.4.2.7).
    pub fn parse_after_type(br: &mut BitReader<'_>, mb_type: &MacroblockType) -> Result<Self> {
        Self::parse(br, mb_type.macroblock_quant)
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact round-trips covering the present/absent
    //! branches, the full legal value range, and the forbidden-zero and
    //! truncation rejection paths.
    use super::*;
    use oxideav_core::bits::BitWriter;

    fn parse_present(value: u32) -> Result<QuantizerScale> {
        let mut bw = BitWriter::new();
        bw.write_u32(value, 5);
        // A trailing '1' bit gives a valid byte the parser never reads.
        bw.write_bit(true);
        bw.align_to_byte();
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        QuantizerScale::parse(&mut br, true)
    }

    #[test]
    fn absent_when_macroblock_quant_clear() {
        // No bits should be consumed when macroblock_quant is false.
        let buf = [0xFFu8; 1];
        let mut br = BitReader::new(&buf);
        let qs = QuantizerScale::parse(&mut br, false).expect("absent parse");
        assert_eq!(qs.quantizer_scale, None);
        assert_eq!(qs.bit_position_after, 0);
        // Reader is untouched.
        assert_eq!(br.bit_position(), 0);
    }

    #[test]
    fn present_minimum_value() {
        let qs = parse_present(1).expect("min parse");
        assert_eq!(qs.quantizer_scale, Some(1));
        assert_eq!(qs.bit_position_after, 5);
    }

    #[test]
    fn present_maximum_value() {
        let qs = parse_present(31).expect("max parse");
        assert_eq!(qs.quantizer_scale, Some(31));
        assert_eq!(qs.bit_position_after, 5);
    }

    #[test]
    fn present_every_legal_value() {
        for v in QUANTIZER_SCALE_MIN..=QUANTIZER_SCALE_MAX {
            let qs = parse_present(u32::from(v)).expect("legal value parse");
            assert_eq!(qs.quantizer_scale, Some(v), "value {v}");
            assert_eq!(qs.bit_position_after, 5);
        }
    }

    #[test]
    fn rejects_forbidden_zero() {
        let err = parse_present(0).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_truncated_field() {
        // Only 4 bits available, but the field needs 5.
        let buf = [0b1000u8]; // bit_position will run out after 8 bits,
                              // but we hand the reader a buffer that is
                              // pre-exhausted to 4 remaining bits.
        let mut br = BitReader::new(&buf);
        br.skip(4).expect("skip");
        let err = QuantizerScale::parse(&mut br, true).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    #[test]
    fn empty_buffer_is_short_when_present() {
        let buf: [u8; 0] = [];
        let mut br = BitReader::new(&buf);
        let err = QuantizerScale::parse(&mut br, true).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    #[test]
    fn empty_buffer_is_fine_when_absent() {
        // macroblock_quant false reads nothing, so even an empty buffer
        // is acceptable.
        let buf: [u8; 0] = [];
        let mut br = BitReader::new(&buf);
        let qs = QuantizerScale::parse(&mut br, false).expect("absent on empty");
        assert_eq!(qs.quantizer_scale, None);
        assert_eq!(qs.bit_position_after, 0);
    }

    #[test]
    fn parse_after_type_threads_macroblock_quant_true() {
        // Build a macroblock_type with macroblock_quant set, then a
        // 5-bit quantizer_scale right after it.
        let mut bw = BitWriter::new();
        // Table B.2a I-picture "Intra, Quant" code is '01' (2 bits) —
        // macroblock_quant = 1.
        bw.write_u32(0b01, 2);
        bw.write_u32(9, 5); // quantizer_scale
        bw.write_bit(true);
        bw.align_to_byte();
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let mt = MacroblockType::parse(&mut br, crate::PictureCodingType::Intra)
            .expect("macroblock_type");
        assert!(mt.macroblock_quant);
        let qs = QuantizerScale::parse_after_type(&mut br, &mt).expect("quantizer_scale");
        assert_eq!(qs.quantizer_scale, Some(9));
        // 2 bits of type + 5 bits of scale = 7.
        assert_eq!(qs.bit_position_after, 7);
    }

    #[test]
    fn parse_after_type_threads_macroblock_quant_false() {
        // Table B.2a I-picture "Intra" code is '1' (1 bit) —
        // macroblock_quant = 0, so no quantizer_scale follows.
        let mut bw = BitWriter::new();
        bw.write_u32(0b1, 1);
        // Junk after, which the absent branch must not read.
        bw.write_u32(0b11111, 5);
        bw.align_to_byte();
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let mt = MacroblockType::parse(&mut br, crate::PictureCodingType::Intra)
            .expect("macroblock_type");
        assert!(!mt.macroblock_quant);
        let before = br.bit_position();
        let qs = QuantizerScale::parse_after_type(&mut br, &mt).expect("quantizer_scale");
        assert_eq!(qs.quantizer_scale, None);
        // Field absent: cursor unchanged at 1 bit (just the type code).
        assert_eq!(qs.bit_position_after, before);
        assert_eq!(qs.bit_position_after, 1);
    }

    #[test]
    fn bounds_constants_match_spec() {
        assert_eq!(QUANTIZER_SCALE_MIN, 1);
        assert_eq!(QUANTIZER_SCALE_MAX, 31);
    }

    #[test]
    fn debug_impl_smoke() {
        let qs = QuantizerScale {
            quantizer_scale: Some(16),
            bit_position_after: 5,
        };
        let s = format!("{qs:?}");
        assert!(s.contains("QuantizerScale"));
        assert!(s.contains("quantizer_scale"));
    }
}
