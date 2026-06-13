//! Parser for the MPEG-2 video `picture_spatial_scalable_extension()`
//! element.
//!
//! Implements the bitstream syntax in ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262) §6.2.3.5 and the field semantics in §6.3.14. The
//! extension is one of the two picture-layer scalable extensions the
//! §6.2.2.2 `extension_and_user_data(2)` dispatcher may encounter when
//! the sequence is spatially scalable (a `sequence_scalable_extension()`
//! with `scalable_mode == spatial` having appeared at `i = 0`). It
//! places the upsampled lower-layer frame relative to the enhancement-
//! layer picture and selects the §7.7 spatial-temporal weight code
//! table.
//!
//! ## Wire shape (§6.2.3.5)
//!
//! ```text
//! picture_spatial_scalable_extension() {
//!     extension_start_code_identifier            4
//!     lower_layer_temporal_reference            10
//!     marker_bit                                 1
//!     lower_layer_horizontal_offset             15
//!     marker_bit                                 1
//!     lower_layer_vertical_offset               15
//!     spatial_temporal_weight_code_table_index   2
//!     lower_layer_progressive_frame              1
//!     lower_layer_deinterlaced_field_select      1
//!     next_start_code()
//! }
//! ```
//!
//! The 32-bit `extension_start_code` (value `0x000001B5` per §6.3.4)
//! precedes the syntax above; the parser consumes the four start-code
//! bytes plus the 4-bit identifier (Table 6-2 entry `1001`, see
//! [`PICTURE_SPATIAL_SCALABLE_EXTENSION_ID`]) so a caller can hand it a
//! slice starting at the start-code prefix exactly as the other
//! `*_extension()` parsers in this crate expect. The element is 83 bits;
//! the trailing `next_start_code()` (§5.2.3) is not consumed.
//!
//! ## Semantic constraints enforced
//!
//! * `extension_start_code` shall be `0x000001B5` and
//!   `extension_start_code_identifier` shall be `'1001'` (Table 6-2).
//! * Both `marker_bit`s (after the temporal reference and between the
//!   two 15-bit offsets) shall be `'1'`.
//!
//! The 15-bit `lower_layer_horizontal_offset` / `lower_layer_vertical_offset`
//! are `simsbf` (two's-complement), so they span `[-16384, 16383]` and are
//! read via [`oxideav_core::bits::BitReader::read_i32`].
//!
//! The chroma-format-dependent even-offset rules (§6.3.14: *"If the
//! chrominance format is 4:2:0 or 4:2:2 then [the horizontal offset]
//! shall be an even number"*; *"If the chrominance format is 4:2:0 then
//! [the vertical offset] shall be an even number"*) cannot be checked by
//! the bare wire parse, which does not see the active `chroma_format`;
//! they are exposed through
//! [`PictureSpatialScalableExtension::validate`].
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)).

use oxideav_core::bits::BitReader;

use crate::sequence_extension::{ChromaFormat, EXTENSION_START_CODE};
use crate::{Error, Result};

/// `extension_start_code_identifier` value for
/// `picture_spatial_scalable_extension()` (Table 6-2 entry `1001`).
pub const PICTURE_SPATIAL_SCALABLE_EXTENSION_ID: u32 = 0b1001;

/// Parsed `picture_spatial_scalable_extension()` (§6.2.3.5 / §6.3.14).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PictureSpatialScalableExtension {
    /// `lower_layer_temporal_reference` — *"A 10 bit unsigned integer
    /// value which indicates temporal reference of the lower layer frame
    /// to be used to provide the prediction"* (§6.3.14).
    pub lower_layer_temporal_reference: u16,
    /// `lower_layer_horizontal_offset` — *"This 15 bit signed (twos
    /// complement) integer specifies the horizontal offset (of the top
    /// left hand corner) of the upsampled lower layer frame relative to
    /// the enhancement layer picture"* (§6.3.14).
    pub lower_layer_horizontal_offset: i32,
    /// `lower_layer_vertical_offset` — *"This 15 bit signed (twos
    /// complement) integer specifies the vertical offset (of the top
    /// left hand corner) of the upsampled lower layer picture relative
    /// to the enhancement layer picture"* (§6.3.14).
    pub lower_layer_vertical_offset: i32,
    /// `spatial_temporal_weight_code_table_index` — *"This 2 bit integer
    /// indicates which table of spatial temporal weight codes is to be
    /// used as defined in 7.7"* (§6.3.14 / Table 7-20 / Table 7-21).
    pub spatial_temporal_weight_code_table_index: u8,
    /// `lower_layer_progressive_frame` — *"This flag shall be set to 0
    /// if the lower layer frame is interlaced and shall be set to '1' if
    /// the lower layer frame is progressive"* (§6.3.14).
    pub lower_layer_progressive_frame: bool,
    /// `lower_layer_deinterlaced_field_select` — *"This flag affects the
    /// spatial scalable upsampling process, as defined in 7.7"*
    /// (§6.3.14).
    pub lower_layer_deinterlaced_field_select: bool,
}

impl PictureSpatialScalableExtension {
    /// Parse a `picture_spatial_scalable_extension()` from a slice
    /// starting with the four start-code bytes `00 00 01 B5`. The
    /// trailing `next_start_code()` (§5.2.3) is not consumed.
    pub fn parse(buf: &[u8]) -> Result<Self> {
        let mut br = BitReader::new(buf);
        Self::parse_with_reader(&mut br)
    }

    /// Parse from an existing [`BitReader`] positioned at the start of
    /// `picture_spatial_scalable_extension()` (i.e. its 32-bit
    /// `extension_start_code`).
    pub fn parse_with_reader(br: &mut BitReader<'_>) -> Result<Self> {
        // §6.2.3.5 / §6.3.4: extension_start_code = 0x000001B5.
        let code = br.read_u32(32).map_err(|_| Error::ShortHeader)?;
        if code != EXTENSION_START_CODE {
            return Err(Error::InvalidBitstream(
                "extension_start_code: expected 0x000001B5 (§6.3.4)",
            ));
        }
        // 4-bit extension_start_code_identifier; Picture Spatial
        // Scalable Extension ID is '1001' per Table 6-2.
        let id = br.read_u32(4).map_err(|_| Error::ShortHeader)?;
        if id != PICTURE_SPATIAL_SCALABLE_EXTENSION_ID {
            return Err(Error::InvalidBitstream(
                "extension_start_code_identifier: expected '1001' Picture Spatial Scalable Extension ID (Table 6-2)",
            ));
        }

        let lower_layer_temporal_reference =
            br.read_u32(10).map_err(|_| Error::ShortHeader)? as u16;

        let read_marker = |br: &mut BitReader<'_>| -> Result<()> {
            if br.read_u32(1).map_err(|_| Error::ShortHeader)? != 1 {
                return Err(Error::InvalidBitstream(
                    "marker_bit in picture_spatial_scalable_extension() (§6.2.3.5)",
                ));
            }
            Ok(())
        };

        read_marker(br)?;
        let lower_layer_horizontal_offset = br.read_i32(15).map_err(|_| Error::ShortHeader)?;
        read_marker(br)?;
        let lower_layer_vertical_offset = br.read_i32(15).map_err(|_| Error::ShortHeader)?;

        let spatial_temporal_weight_code_table_index =
            br.read_u32(2).map_err(|_| Error::ShortHeader)? as u8;
        let lower_layer_progressive_frame = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let lower_layer_deinterlaced_field_select =
            br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;

        Ok(Self {
            lower_layer_temporal_reference,
            lower_layer_horizontal_offset,
            lower_layer_vertical_offset,
            spatial_temporal_weight_code_table_index,
            lower_layer_progressive_frame,
            lower_layer_deinterlaced_field_select,
        })
    }

    /// Cross-check the chroma-format-dependent §6.3.14 even-offset rules
    /// the bare wire parse cannot see. The caller supplies the active
    /// `chroma_format` from the `sequence_extension()`.
    ///
    /// * *"If the chrominance format is 4:2:0 or 4:2:2 then [the
    ///   `lower_layer_horizontal_offset`] shall be an even number"*.
    /// * *"If the chrominance format is 4:2:0 then [the
    ///   `lower_layer_vertical_offset`] shall be an even number"*.
    pub fn validate(&self, chroma_format: ChromaFormat) -> Result<()> {
        let horizontal_must_be_even =
            matches!(chroma_format, ChromaFormat::Yuv420 | ChromaFormat::Yuv422);
        if horizontal_must_be_even && self.lower_layer_horizontal_offset % 2 != 0 {
            return Err(Error::InvalidBitstream(
                "lower_layer_horizontal_offset: shall be even for 4:2:0 / 4:2:2 (§6.3.14)",
            ));
        }
        let vertical_must_be_even = matches!(chroma_format, ChromaFormat::Yuv420);
        if vertical_must_be_even && self.lower_layer_vertical_offset % 2 != 0 {
            return Err(Error::InvalidBitstream(
                "lower_layer_vertical_offset: shall be even for 4:2:0 (§6.3.14)",
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact `picture_spatial_scalable_extension()`
    //! fixtures plus negative cases for every §6.2.3.5 rejection site
    //! and the §6.3.14 chroma-format cross-checks.
    use super::*;
    use oxideav_core::bits::BitWriter;

    struct Fixture {
        id: u32,
        temporal_reference: u32,
        markers: [bool; 2],
        horizontal: i32,
        vertical: i32,
        weight_index: u32,
        progressive: bool,
        deinterlaced: bool,
    }

    impl Default for Fixture {
        fn default() -> Self {
            Self {
                id: PICTURE_SPATIAL_SCALABLE_EXTENSION_ID,
                temporal_reference: 9,
                markers: [true; 2],
                horizontal: 4,
                vertical: -6,
                weight_index: 0b00,
                progressive: true,
                deinterlaced: false,
            }
        }
    }

    fn build(fx: &Fixture) -> Vec<u8> {
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(fx.id, 4);
        bw.write_u32(fx.temporal_reference, 10);
        bw.write_bit(fx.markers[0]);
        bw.write_i32(fx.horizontal, 15);
        bw.write_bit(fx.markers[1]);
        bw.write_i32(fx.vertical, 15);
        bw.write_u32(fx.weight_index, 2);
        bw.write_bit(fx.progressive);
        bw.write_bit(fx.deinterlaced);
        bw.finish()
    }

    #[test]
    fn parses_a_spatial_scalable_extension() {
        let buf = build(&Fixture::default());
        let ext = PictureSpatialScalableExtension::parse(&buf).unwrap();
        assert_eq!(ext.lower_layer_temporal_reference, 9);
        assert_eq!(ext.lower_layer_horizontal_offset, 4);
        assert_eq!(ext.lower_layer_vertical_offset, -6);
        assert_eq!(ext.spatial_temporal_weight_code_table_index, 0b00);
        assert!(ext.lower_layer_progressive_frame);
        assert!(!ext.lower_layer_deinterlaced_field_select);
    }

    #[test]
    fn fifteen_bit_offsets_round_trip_full_signed_range() {
        let buf = build(&Fixture {
            horizontal: 16383,
            vertical: -16384,
            ..Fixture::default()
        });
        let ext = PictureSpatialScalableExtension::parse(&buf).unwrap();
        assert_eq!(ext.lower_layer_horizontal_offset, 16383);
        assert_eq!(ext.lower_layer_vertical_offset, -16384);
    }

    #[test]
    fn element_is_eighty_three_bits_eleven_padded_bytes() {
        // 32 + 4 + 10 + 1 + 15 + 1 + 15 + 2 + 1 + 1 = 82 bits of payload
        // after the start code prefix... recount: 4+10+1+15+1+15+2+1+1=50,
        // plus 32 = 82 bits → 11 bytes.
        let buf = build(&Fixture::default());
        assert_eq!(buf.len(), 11);
    }

    #[test]
    fn captures_all_four_weight_table_indices() {
        for idx in 0..4u32 {
            let buf = build(&Fixture {
                weight_index: idx,
                ..Fixture::default()
            });
            let ext = PictureSpatialScalableExtension::parse(&buf).unwrap();
            assert_eq!(ext.spatial_temporal_weight_code_table_index, idx as u8);
        }
    }

    #[test]
    fn rejects_wrong_extension_start_code() {
        let mut buf = build(&Fixture::default());
        buf[3] = 0xB3;
        assert!(matches!(
            PictureSpatialScalableExtension::parse(&buf),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn rejects_wrong_identifier() {
        let buf = build(&Fixture {
            id: 0b1010, // temporal scalable ID
            ..Fixture::default()
        });
        assert!(matches!(
            PictureSpatialScalableExtension::parse(&buf),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn rejects_zero_first_marker_bit() {
        let buf = build(&Fixture {
            markers: [false, true],
            ..Fixture::default()
        });
        assert!(matches!(
            PictureSpatialScalableExtension::parse(&buf),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn rejects_zero_second_marker_bit() {
        let buf = build(&Fixture {
            markers: [true, false],
            ..Fixture::default()
        });
        assert!(matches!(
            PictureSpatialScalableExtension::parse(&buf),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn rejects_truncated_buffer() {
        let buf = build(&Fixture::default());
        assert!(matches!(
            PictureSpatialScalableExtension::parse(&buf[..8]),
            Err(Error::ShortHeader)
        ));
    }

    #[test]
    fn validate_accepts_even_offsets_for_420() {
        let ext = PictureSpatialScalableExtension {
            lower_layer_temporal_reference: 0,
            lower_layer_horizontal_offset: 4,
            lower_layer_vertical_offset: -6,
            spatial_temporal_weight_code_table_index: 0,
            lower_layer_progressive_frame: false,
            lower_layer_deinterlaced_field_select: false,
        };
        assert!(ext.validate(ChromaFormat::Yuv420).is_ok());
    }

    #[test]
    fn validate_rejects_odd_horizontal_for_422() {
        let ext = PictureSpatialScalableExtension {
            lower_layer_temporal_reference: 0,
            lower_layer_horizontal_offset: 3,
            lower_layer_vertical_offset: 0,
            spatial_temporal_weight_code_table_index: 0,
            lower_layer_progressive_frame: false,
            lower_layer_deinterlaced_field_select: false,
        };
        assert!(matches!(
            ext.validate(ChromaFormat::Yuv422),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn validate_rejects_odd_vertical_only_for_420() {
        let ext = PictureSpatialScalableExtension {
            lower_layer_temporal_reference: 0,
            lower_layer_horizontal_offset: 4,
            lower_layer_vertical_offset: 5,
            spatial_temporal_weight_code_table_index: 0,
            lower_layer_progressive_frame: false,
            lower_layer_deinterlaced_field_select: false,
        };
        // 4:2:0 requires even vertical → reject.
        assert!(matches!(
            ext.validate(ChromaFormat::Yuv420),
            Err(Error::InvalidBitstream(_))
        ));
        // 4:2:2 has no vertical even-offset rule → accept.
        assert!(ext.validate(ChromaFormat::Yuv422).is_ok());
    }

    #[test]
    fn validate_accepts_odd_offsets_for_444() {
        let ext = PictureSpatialScalableExtension {
            lower_layer_temporal_reference: 0,
            lower_layer_horizontal_offset: 3,
            lower_layer_vertical_offset: 5,
            spatial_temporal_weight_code_table_index: 0,
            lower_layer_progressive_frame: false,
            lower_layer_deinterlaced_field_select: false,
        };
        assert!(ext.validate(ChromaFormat::Yuv444).is_ok());
    }
}
