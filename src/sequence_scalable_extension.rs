//! Parser for the MPEG-2 video `sequence_scalable_extension()` element.
//!
//! Implements the bitstream syntax in ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262) §6.2.2.5 and the field semantics in §6.3.7. The
//! extension declares that the video sequence is one layer of a
//! scalable hierarchy and selects the scalability type; §6.3.7: *"The
//! `scalable_mode` indicates the type of scalability used in the video
//! sequence. If no `sequence_scalable_extension()` is present in the
//! bitstream then no scalability is used for that sequence."*
//!
//! ## Wire shape (§6.2.2.5)
//!
//! ```text
//! sequence_scalable_extension() {
//!     extension_start_code_identifier              4
//!     scalable_mode                                2
//!     layer_id                                     4
//!     if ( scalable_mode == "spatial scalability" ) {
//!         lower_layer_prediction_horizontal_size  14
//!         marker_bit                               1
//!         lower_layer_prediction_vertical_size    14
//!         horizontal_subsampling_factor_m          5
//!         horizontal_subsampling_factor_n          5
//!         vertical_subsampling_factor_m            5
//!         vertical_subsampling_factor_n            5
//!     }
//!     if ( scalable_mode == "temporal scalability" ) {
//!         picture_mux_enable                       1
//!         if ( picture_mux_enable )
//!             mux_to_progressive_sequence         1
//!         picture_mux_order                        3
//!         picture_mux_factor                       3
//!     }
//!     next_start_code()
//! }
//! ```
//!
//! The 32-bit `extension_start_code` (value `0x000001B5` per §6.3.4)
//! precedes the syntax above; the parser consumes the four start-code
//! bytes plus the 4-bit identifier (Table 6-2 entry `0101`, see
//! [`SEQUENCE_SCALABLE_EXTENSION_ID`]) so a caller can hand it a
//! slice starting at the start-code prefix exactly as the other
//! `*_extension()` parsers in this crate expect.
//!
//! ## `layer_id` constraints (§6.1 / §6.3.7)
//!
//! §6.3.7: *"The base layer always has `layer_id` = 0. However the
//! base layer of a scalable hierarchy does not carry a
//! `sequence_scalable_extension()` and hence `layer_id`, except in
//! the case of data partitioning. Each successive layer has a
//! `layer_id` which is one greater than the layer for which it is an
//! enhancement."* Combined with §6.1 (*"In all cases apart from Data
//! partitioning, the base layer does not contain a
//! `sequence_scalable_extension()`. Enhancement layers always contain
//! `sequence_scalable_extension()`."*) this pins:
//!
//! * data partitioning — *"`layer_id` shall be zero for partition
//!   zero and `layer_id` shall be one for partition one"* (§6.3.7);
//!   any other value is rejected.
//! * spatial / SNR / temporal scalability — the extension can only
//!   appear in an enhancement layer, whose `layer_id` is at least
//!   one; `layer_id == 0` is rejected.
//!
//! ## Occurrence constraint (§6.1.1.6 / §6.3.7)
//!
//! §6.3.7 opens: *"It is a syntactic restriction that if a
//! `sequence_scalable_extension()` is present in the bitstream
//! following a given `sequence_extension()` then
//! `sequence_scalable_extension()` shall follow every other occurrence
//! of `sequence_extension()`. Thus a bitstream is either scalable or
//! it is not scalable."* §6.1.1.6 states the same repeat-header rule
//! in the `sequence_display_extension()` shape (all data elements
//! equal across repeats, all-or-nothing presence). That cross-element
//! rule needs a sequence-layer driver like
//! [`crate::SequenceDisplayOrderDriver`]; this module supplies the
//! parsed value such a driver would compare. The driver is a
//! follow-up.
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)).

use oxideav_core::bits::BitReader;

use crate::sequence_extension::EXTENSION_START_CODE;
use crate::{Error, Result};

/// `extension_start_code_identifier` value for
/// `sequence_scalable_extension()` (Table 6-2 entry `0101`).
pub const SEQUENCE_SCALABLE_EXTENSION_ID: u32 = 0b0101;

/// The spatial-scalability parameter block carried when
/// `scalable_mode == "spatial scalability"` (§6.2.2.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpatialScalabilityParams {
    /// `lower_layer_prediction_horizontal_size` — 14-bit *"horizontal
    /// size of the lower layer frame which is used for prediction"*;
    /// *"shall contain the value contained in `horizontal_size` … in
    /// the lower layer bitstream"* (§6.3.7).
    pub lower_layer_prediction_horizontal_size: u16,
    /// `lower_layer_prediction_vertical_size` — 14-bit vertical
    /// counterpart (§6.3.7).
    pub lower_layer_prediction_vertical_size: u16,
    /// `horizontal_subsampling_factor_m` — 5-bit factor for the §7.7.2
    /// spatial scalable upsampling process. *"The value zero is
    /// forbidden"* (§6.3.7).
    pub horizontal_subsampling_factor_m: u8,
    /// `horizontal_subsampling_factor_n` — 5-bit factor (§6.3.7, zero
    /// forbidden).
    pub horizontal_subsampling_factor_n: u8,
    /// `vertical_subsampling_factor_m` — 5-bit factor (§6.3.7, zero
    /// forbidden).
    pub vertical_subsampling_factor_m: u8,
    /// `vertical_subsampling_factor_n` — 5-bit factor (§6.3.7, zero
    /// forbidden).
    pub vertical_subsampling_factor_n: u8,
}

/// The temporal-scalability parameter block carried when
/// `scalable_mode == "temporal scalability"` (§6.2.2.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TemporalScalabilityParams {
    /// `picture_mux_enable` — *"If set to 1, `picture_mux_order` and
    /// `picture_mux_factor` are used for remultiplexing prior to
    /// display"* (§6.3.7).
    pub picture_mux_enable: bool,
    /// `mux_to_progressive_sequence` — present iff
    /// `picture_mux_enable` (§6.2.2.5). `'1'`: the two layers'
    /// decoded pictures *"shall be temporally multiplexed to generate
    /// a progressive sequence for display"*; for an interlaced
    /// multiplex the flag *"shall be '0'"* (§6.3.7).
    pub mux_to_progressive_sequence: Option<bool>,
    /// `picture_mux_order` — 3-bit *"number of enhancement layer
    /// pictures prior to the first base layer picture"* (§6.3.7).
    pub picture_mux_order: u8,
    /// `picture_mux_factor` — 3-bit *"number of enhancement layer
    /// pictures between consecutive base layer pictures"* (§6.3.7).
    /// *"The value '000' is reserved"* — rejected at parse time when
    /// `picture_mux_enable` is set (the only case §6.3.7 says the
    /// field is used); preserved raw otherwise.
    pub picture_mux_factor: u8,
}

/// `scalable_mode` per Table 6-10, carrying the mode-conditional
/// parameter block from §6.2.2.5. All four 2-bit codes are defined —
/// Table 6-10 has no reserved row.
///
/// Table 6-10 also binds the macroblock_type tables: B-2/B-3/B-4 for
/// data partitioning and temporal scalability (and when no
/// `sequence_scalable_extension()` is present at all), B-5/B-6/B-7
/// for spatial scalability with a `picture_spatial_scalable_extension()`
/// present (B-2/B-3/B-4 when absent — §6.3.7: such a picture *"shall
/// be decoded in a non-scalable manner"*), and B-8 for SNR
/// scalability.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScalableMode {
    /// `00` — data partitioning.
    DataPartitioning,
    /// `01` — spatial scalability.
    SpatialScalability(SpatialScalabilityParams),
    /// `10` — SNR scalability.
    SnrScalability,
    /// `11` — temporal scalability.
    TemporalScalability(TemporalScalabilityParams),
}

impl ScalableMode {
    /// The raw 2-bit `scalable_mode` code (Table 6-10).
    pub fn code(&self) -> u8 {
        match self {
            Self::DataPartitioning => 0b00,
            Self::SpatialScalability(_) => 0b01,
            Self::SnrScalability => 0b10,
            Self::TemporalScalability(_) => 0b11,
        }
    }
}

/// Parsed `sequence_scalable_extension()` (§6.2.2.5 / §6.3.7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SequenceScalableExtension {
    /// `scalable_mode` (Table 6-10) plus its mode-conditional
    /// parameters.
    pub scalable_mode: ScalableMode,
    /// `layer_id` — 4-bit *"integer which identifies the layers in a
    /// scalable hierarchy"* (§6.3.7). See the module docs for the
    /// per-mode value constraints enforced at parse time.
    pub layer_id: u8,
}

impl SequenceScalableExtension {
    /// Parse a `sequence_scalable_extension()` from a slice starting
    /// with the four start-code bytes `00 00 01 B5`. The trailing
    /// `next_start_code()` (§5.2.3) byte-align is not consumed.
    pub fn parse(buf: &[u8]) -> Result<Self> {
        let mut br = BitReader::new(buf);
        Self::parse_with_reader(&mut br)
    }

    /// Parse from an existing [`BitReader`] positioned at the start of
    /// `sequence_scalable_extension()` (i.e. its 32-bit
    /// `extension_start_code`).
    pub fn parse_with_reader(br: &mut BitReader<'_>) -> Result<Self> {
        // §6.2.2.5 / §6.3.4: extension_start_code = 0x000001B5.
        let code = br.read_u32(32).map_err(|_| Error::ShortHeader)?;
        if code != EXTENSION_START_CODE {
            return Err(Error::InvalidBitstream(
                "extension_start_code: expected 0x000001B5 (§6.3.4)",
            ));
        }
        // 4-bit extension_start_code_identifier; Sequence Scalable
        // Extension ID is '0101' per Table 6-2.
        let id = br.read_u32(4).map_err(|_| Error::ShortHeader)?;
        if id != SEQUENCE_SCALABLE_EXTENSION_ID {
            return Err(Error::InvalidBitstream(
                "extension_start_code_identifier: expected '0101' Sequence Scalable Extension ID (Table 6-2)",
            ));
        }

        // 2-bit scalable_mode (Table 6-10) + 4-bit layer_id.
        let mode_code = br.read_u32(2).map_err(|_| Error::ShortHeader)? as u8;
        let layer_id = br.read_u32(4).map_err(|_| Error::ShortHeader)? as u8;

        let scalable_mode = match mode_code {
            0b00 => ScalableMode::DataPartitioning,
            0b01 => ScalableMode::SpatialScalability(Self::parse_spatial_params(br)?),
            0b10 => ScalableMode::SnrScalability,
            0b11 => ScalableMode::TemporalScalability(Self::parse_temporal_params(br)?),
            _ => unreachable!("2-bit field"),
        };

        // §6.3.7 layer_id constraints (see module docs):
        match scalable_mode {
            ScalableMode::DataPartitioning => {
                // "In the case of data partitioning layer_id shall be
                // zero for partition zero and layer_id shall be one
                // for partition one."
                if layer_id > 1 {
                    return Err(Error::InvalidBitstream(
                        "layer_id: data partitioning uses 0 (partition zero) or 1 (partition one) (§6.3.7)",
                    ));
                }
            }
            _ => {
                // §6.1: apart from data partitioning the base layer
                // (layer_id = 0) does not contain a
                // sequence_scalable_extension(); §6.3.7: each
                // enhancement layer's layer_id is one greater than
                // the layer it enhances, hence at least one here.
                if layer_id == 0 {
                    return Err(Error::InvalidBitstream(
                        "layer_id: 0 names the base layer, which does not carry sequence_scalable_extension() outside data partitioning (§6.1 / §6.3.7)",
                    ));
                }
            }
        }

        // §6.2.2.5: 42 bits (data partitioning / SNR), 91 bits
        // (spatial), or 49 / 50 bits (temporal) — the trailing
        // next_start_code() (§5.2.3) supplies the zero stuffing back
        // to a byte boundary; we therefore do NOT assert
        // byte-alignment here.
        Ok(Self {
            scalable_mode,
            layer_id,
        })
    }

    /// The `if ( scalable_mode == "spatial scalability" )` block of
    /// §6.2.2.5.
    fn parse_spatial_params(br: &mut BitReader<'_>) -> Result<SpatialScalabilityParams> {
        let lower_layer_prediction_horizontal_size =
            br.read_u32(14).map_err(|_| Error::ShortHeader)? as u16;
        let marker = br.read_u32(1).map_err(|_| Error::ShortHeader)?;
        if marker != 1 {
            return Err(Error::InvalidBitstream(
                "marker_bit after lower_layer_prediction_horizontal_size (§6.2.2.5)",
            ));
        }
        let lower_layer_prediction_vertical_size =
            br.read_u32(14).map_err(|_| Error::ShortHeader)? as u16;

        // §6.3.7 marks zero forbidden for all four subsampling
        // factors (each feeds the §7.7.2 upsampling ratios as a
        // numerator or denominator).
        let mut factors = [0u8; 4];
        const FACTOR_REJECTS: [&str; 4] = [
            "horizontal_subsampling_factor_m: the value zero is forbidden (§6.3.7)",
            "horizontal_subsampling_factor_n: the value zero is forbidden (§6.3.7)",
            "vertical_subsampling_factor_m: the value zero is forbidden (§6.3.7)",
            "vertical_subsampling_factor_n: the value zero is forbidden (§6.3.7)",
        ];
        for (factor, reject) in factors.iter_mut().zip(FACTOR_REJECTS) {
            *factor = br.read_u32(5).map_err(|_| Error::ShortHeader)? as u8;
            if *factor == 0 {
                return Err(Error::InvalidBitstream(reject));
            }
        }

        Ok(SpatialScalabilityParams {
            lower_layer_prediction_horizontal_size,
            lower_layer_prediction_vertical_size,
            horizontal_subsampling_factor_m: factors[0],
            horizontal_subsampling_factor_n: factors[1],
            vertical_subsampling_factor_m: factors[2],
            vertical_subsampling_factor_n: factors[3],
        })
    }

    /// The `if ( scalable_mode == "temporal scalability" )` block of
    /// §6.2.2.5.
    fn parse_temporal_params(br: &mut BitReader<'_>) -> Result<TemporalScalabilityParams> {
        let picture_mux_enable = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        // §6.2.2.5: mux_to_progressive_sequence is present iff
        // picture_mux_enable.
        let mux_to_progressive_sequence = if picture_mux_enable {
            Some(br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1)
        } else {
            None
        };
        let picture_mux_order = br.read_u32(3).map_err(|_| Error::ShortHeader)? as u8;
        let picture_mux_factor = br.read_u32(3).map_err(|_| Error::ShortHeader)? as u8;
        // §6.3.7: "The value '000' is reserved." The field is only
        // used (for inverting the encoder's temporal demultiplex)
        // when picture_mux_enable is set — reject the reserved code
        // there; with picture_mux_enable clear the field is unused
        // and the raw value is preserved.
        if picture_mux_enable && picture_mux_factor == 0 {
            return Err(Error::InvalidBitstream(
                "picture_mux_factor: the value '000' is reserved (§6.3.7)",
            ));
        }
        Ok(TemporalScalabilityParams {
            picture_mux_enable,
            mux_to_progressive_sequence,
            picture_mux_order,
            picture_mux_factor,
        })
    }
}

/// Write a §6.2.2.5 `sequence_scalable_extension()` — the
/// `extension_start_code`, the `'0101'` identifier, `scalable_mode`,
/// `layer_id` and the mode-specific fields — byte-aligned with zero
/// stuffing (§5.2.3 `next_start_code()`).
pub fn write_sequence_scalable_extension(
    bw: &mut oxideav_core::bits::BitWriter,
    ext: &SequenceScalableExtension,
) {
    bw.write_u32(EXTENSION_START_CODE, 32);
    bw.write_u32(SEQUENCE_SCALABLE_EXTENSION_ID, 4);
    bw.write_u32(u32::from(ext.scalable_mode.code()), 2);
    bw.write_u32(u32::from(ext.layer_id & 0xF), 4);
    match ext.scalable_mode {
        ScalableMode::DataPartitioning | ScalableMode::SnrScalability => {}
        ScalableMode::SpatialScalability(p) => {
            bw.write_u32(u32::from(p.lower_layer_prediction_horizontal_size), 14);
            bw.write_bit(true); // marker_bit
            bw.write_u32(u32::from(p.lower_layer_prediction_vertical_size), 14);
            bw.write_u32(u32::from(p.horizontal_subsampling_factor_m), 5);
            bw.write_u32(u32::from(p.horizontal_subsampling_factor_n), 5);
            bw.write_u32(u32::from(p.vertical_subsampling_factor_m), 5);
            bw.write_u32(u32::from(p.vertical_subsampling_factor_n), 5);
        }
        ScalableMode::TemporalScalability(p) => {
            bw.write_bit(p.picture_mux_enable);
            if p.picture_mux_enable {
                bw.write_bit(p.mux_to_progressive_sequence.unwrap_or(false));
            }
            bw.write_u32(u32::from(p.picture_mux_order), 3);
            bw.write_u32(u32::from(p.picture_mux_factor), 3);
        }
    }
    bw.align_to_byte_zero();
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact `sequence_scalable_extension()` fixtures
    //! plus negative cases for every §6.2.2.5 / §6.3.7 rejection site
    //! this parser introduces.
    use super::*;
    use oxideav_core::bits::BitWriter;

    fn write_prelude(bw: &mut BitWriter, mode: u8, layer_id: u8) {
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(SEQUENCE_SCALABLE_EXTENSION_ID, 4);
        bw.write_u32(mode as u32, 2);
        bw.write_u32(layer_id as u32, 4);
    }

    fn build_plain(mode: u8, layer_id: u8) -> Vec<u8> {
        let mut bw = BitWriter::new();
        write_prelude(&mut bw, mode, layer_id);
        bw.align_to_byte();
        bw.finish()
    }

    fn build_spatial(layer_id: u8, sizes: (u16, u16), factors: [u8; 4], marker: bool) -> Vec<u8> {
        let mut bw = BitWriter::new();
        write_prelude(&mut bw, 0b01, layer_id);
        bw.write_u32(sizes.0 as u32, 14);
        bw.write_bit(marker);
        bw.write_u32(sizes.1 as u32, 14);
        for f in factors {
            bw.write_u32(f as u32, 5);
        }
        bw.align_to_byte();
        bw.finish()
    }

    fn build_temporal(layer_id: u8, enable: Option<bool>, order: u8, factor: u8) -> Vec<u8> {
        let mut bw = BitWriter::new();
        write_prelude(&mut bw, 0b11, layer_id);
        match enable {
            Some(mux_to_progressive) => {
                bw.write_bit(true);
                bw.write_bit(mux_to_progressive);
            }
            None => bw.write_bit(false),
        }
        bw.write_u32(order as u32, 3);
        bw.write_u32(factor as u32, 3);
        bw.align_to_byte();
        bw.finish()
    }

    // ---- Positive wire parses --------------------------------------

    #[test]
    fn parses_data_partitioning_partition_zero_and_one() {
        // §6.3.7: layer_id 0 = partition zero, 1 = partition one.
        for layer_id in [0u8, 1] {
            let ext = SequenceScalableExtension::parse(&build_plain(0b00, layer_id))
                .expect("data partitioning");
            assert_eq!(ext.scalable_mode, ScalableMode::DataPartitioning);
            assert_eq!(ext.layer_id, layer_id);
        }
    }

    #[test]
    fn parses_snr_scalability() {
        let ext = SequenceScalableExtension::parse(&build_plain(0b10, 1)).expect("SNR");
        assert_eq!(ext.scalable_mode, ScalableMode::SnrScalability);
        assert_eq!(ext.layer_id, 1);
    }

    #[test]
    fn parses_spatial_scalability_parameters() {
        let bytes = build_spatial(1, (352, 288), [1, 2, 1, 2], true);
        let ext = SequenceScalableExtension::parse(&bytes).expect("spatial");
        assert_eq!(
            ext.scalable_mode,
            ScalableMode::SpatialScalability(SpatialScalabilityParams {
                lower_layer_prediction_horizontal_size: 352,
                lower_layer_prediction_vertical_size: 288,
                horizontal_subsampling_factor_m: 1,
                horizontal_subsampling_factor_n: 2,
                vertical_subsampling_factor_m: 1,
                vertical_subsampling_factor_n: 2,
            })
        );
        assert_eq!(ext.layer_id, 1);
    }

    #[test]
    fn parses_spatial_maximum_field_values() {
        // 14-bit sizes and 5-bit factors at their maxima round-trip.
        let bytes = build_spatial(15, (0x3FFF, 0x3FFF), [31, 31, 31, 31], true);
        let ext = SequenceScalableExtension::parse(&bytes).expect("spatial maxima");
        let ScalableMode::SpatialScalability(p) = ext.scalable_mode else {
            panic!("expected spatial mode");
        };
        assert_eq!(p.lower_layer_prediction_horizontal_size, 0x3FFF);
        assert_eq!(p.lower_layer_prediction_vertical_size, 0x3FFF);
        assert_eq!(p.vertical_subsampling_factor_n, 31);
        assert_eq!(ext.layer_id, 15);
    }

    #[test]
    fn parses_temporal_with_picture_mux_enabled() {
        let bytes = build_temporal(1, Some(true), 2, 3);
        let ext = SequenceScalableExtension::parse(&bytes).expect("temporal");
        assert_eq!(
            ext.scalable_mode,
            ScalableMode::TemporalScalability(TemporalScalabilityParams {
                picture_mux_enable: true,
                mux_to_progressive_sequence: Some(true),
                picture_mux_order: 2,
                picture_mux_factor: 3,
            })
        );
    }

    #[test]
    fn parses_temporal_without_picture_mux() {
        // §6.2.2.5: mux_to_progressive_sequence absent when
        // picture_mux_enable == 0; the unused order/factor raw values
        // (including the otherwise-reserved factor '000') are
        // preserved.
        let bytes = build_temporal(2, None, 0, 0);
        let ext = SequenceScalableExtension::parse(&bytes).expect("temporal, mux disabled");
        assert_eq!(
            ext.scalable_mode,
            ScalableMode::TemporalScalability(TemporalScalabilityParams {
                picture_mux_enable: false,
                mux_to_progressive_sequence: None,
                picture_mux_order: 0,
                picture_mux_factor: 0,
            })
        );
    }

    #[test]
    fn scalable_mode_codes_round_trip() {
        // Table 6-10 code accessor.
        for (mode, expected) in [(0b00u8, 0b00u8), (0b10, 0b10)] {
            let ext = SequenceScalableExtension::parse(&build_plain(mode, 1)).expect("parse");
            assert_eq!(ext.scalable_mode.code(), expected);
        }
        let spatial = SequenceScalableExtension::parse(&build_spatial(1, (1, 1), [1; 4], true))
            .expect("spatial");
        assert_eq!(spatial.scalable_mode.code(), 0b01);
        let temporal =
            SequenceScalableExtension::parse(&build_temporal(1, None, 1, 1)).expect("temporal");
        assert_eq!(temporal.scalable_mode.code(), 0b11);
    }

    // ---- Encoded-length accounting ---------------------------------

    #[test]
    fn encoded_lengths_per_mode() {
        // 32 + 4 + 2 + 4 = 42 bits -> 6 bytes (data partitioning /
        // SNR); + 49 = 91 bits -> 12 bytes (spatial); + 7 = 49 bits
        // -> 7 bytes (temporal, mux disabled); + 8 = 50 bits -> 7
        // bytes (temporal, mux enabled).
        assert_eq!(build_plain(0b00, 0).len(), 6);
        assert_eq!(build_plain(0b10, 1).len(), 6);
        assert_eq!(build_spatial(1, (1, 1), [1; 4], true).len(), 12);
        assert_eq!(build_temporal(1, None, 1, 1).len(), 7);
        assert_eq!(build_temporal(1, Some(false), 1, 1).len(), 7);
    }

    // ---- Rejection sites -------------------------------------------

    #[test]
    fn rejects_wrong_extension_start_code() {
        let mut bytes = build_plain(0b00, 0);
        bytes[3] = 0xB3; // sequence_header_code instead
        assert!(matches!(
            SequenceScalableExtension::parse(&bytes),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn rejects_wrong_extension_identifier() {
        // Identifier '0010' (Sequence Display Extension ID) instead
        // of '0101'.
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(0b0010, 4);
        bw.write_u32(0, 2);
        bw.write_u32(0, 4);
        bw.align_to_byte();
        assert!(matches!(
            SequenceScalableExtension::parse(&bw.finish()),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn rejects_data_partitioning_layer_id_above_one() {
        // §6.3.7: partitions are layer_id 0 and 1 only.
        assert!(matches!(
            SequenceScalableExtension::parse(&build_plain(0b00, 2)),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn rejects_layer_id_zero_outside_data_partitioning() {
        // §6.1 / §6.3.7: the base layer (layer_id 0) carries no
        // sequence_scalable_extension() in the other three modes.
        assert!(matches!(
            SequenceScalableExtension::parse(&build_plain(0b10, 0)),
            Err(Error::InvalidBitstream(_))
        ));
        assert!(matches!(
            SequenceScalableExtension::parse(&build_spatial(0, (1, 1), [1; 4], true)),
            Err(Error::InvalidBitstream(_))
        ));
        assert!(matches!(
            SequenceScalableExtension::parse(&build_temporal(0, None, 1, 1)),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn rejects_zero_marker_bit_in_spatial_block() {
        assert!(matches!(
            SequenceScalableExtension::parse(&build_spatial(1, (1, 1), [1; 4], false)),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn rejects_each_zero_subsampling_factor() {
        // §6.3.7 forbids zero for all four factors independently.
        for zeroed in 0..4 {
            let mut factors = [1u8; 4];
            factors[zeroed] = 0;
            assert!(
                matches!(
                    SequenceScalableExtension::parse(&build_spatial(1, (1, 1), factors, true)),
                    Err(Error::InvalidBitstream(_))
                ),
                "factor index {zeroed}"
            );
        }
    }

    #[test]
    fn rejects_reserved_picture_mux_factor_when_mux_enabled() {
        // §6.3.7: "The value '000' is reserved" — and with
        // picture_mux_enable set the field is in use.
        assert!(matches!(
            SequenceScalableExtension::parse(&build_temporal(1, Some(false), 1, 0)),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn short_buffer_returns_short_header() {
        let bytes = [0u8, 0u8];
        assert!(matches!(
            SequenceScalableExtension::parse(&bytes),
            Err(Error::ShortHeader)
        ));

        // Truncated mid-spatial-block: cut after the identifier byte
        // plus one payload byte.
        let full = build_spatial(1, (352, 288), [1, 2, 1, 2], true);
        assert!(matches!(
            SequenceScalableExtension::parse(&full[..6]),
            Err(Error::ShortHeader)
        ));

        // Truncated mid-temporal-block.
        let full = build_temporal(1, Some(true), 2, 3);
        assert!(matches!(
            SequenceScalableExtension::parse(&full[..6]),
            Err(Error::ShortHeader)
        ));
    }
}
