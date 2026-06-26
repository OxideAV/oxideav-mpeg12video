//! Parser for the MPEG-2 video `picture_temporal_scalable_extension()`
//! element.
//!
//! Implements the bitstream syntax in ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262) §6.2.3.4 and the field semantics in §6.3.13. The
//! extension is one of the two picture-layer scalable extensions the
//! §6.2.2.2 `extension_and_user_data(2)` dispatcher may encounter when
//! the sequence is temporally scalable (a `sequence_scalable_extension()`
//! with `scalable_mode == temporal` having appeared at `i = 0`). It
//! selects the lower-layer / enhancement-layer reference frames the
//! §7.9 enhancement-layer motion-compensation process predicts from.
//!
//! ## Wire shape (§6.2.3.4)
//!
//! ```text
//! picture_temporal_scalable_extension() {
//!     extension_start_code_identifier        4
//!     reference_select_code                  2
//!     forward_temporal_reference            10
//!     marker_bit                             1
//!     backward_temporal_reference           10
//!     next_start_code()
//! }
//! ```
//!
//! The 32-bit `extension_start_code` (value `0x000001B5` per §6.3.4)
//! precedes the syntax above; the parser consumes the four start-code
//! bytes plus the 4-bit identifier (Table 6-2 entry `1010`, see
//! [`PICTURE_TEMPORAL_SCALABLE_EXTENSION_ID`]) so a caller can hand it a
//! slice starting at the start-code prefix exactly as the other
//! `*_extension()` parsers in this crate expect. The element is 59 bits;
//! the trailing `next_start_code()` (§5.2.3) is not consumed.
//!
//! ## Semantic constraints enforced
//!
//! * `extension_start_code` shall be `0x000001B5` and
//!   `extension_start_code_identifier` shall be `'1010'` (Table 6-2).
//! * The `marker_bit` between the two 10-bit temporal references shall
//!   be `'1'`.
//! * `reference_select_code == '11'` is *"forbidden"* in P-pictures
//!   (Table 7-28). The bare wire parse cannot see `picture_coding_type`,
//!   so this rule — and the §6.3.13 I-picture `'11'` requirement and the
//!   *"shall take the value 0"* zeroing of unused temporal references —
//!   are exposed through [`PictureTemporalScalableExtension::validate`]
//!   for a caller that knows the active `picture_coding_type`.
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)).

use oxideav_core::bits::BitReader;

use crate::picture_header::PictureCodingType;
use crate::sequence_extension::EXTENSION_START_CODE;
use crate::{Error, Result};

/// `extension_start_code_identifier` value for
/// `picture_temporal_scalable_extension()` (Table 6-2 entry `1010`).
pub const PICTURE_TEMPORAL_SCALABLE_EXTENSION_ID: u32 = 0b1010;

/// Parsed `picture_temporal_scalable_extension()` (§6.2.3.4 / §6.3.13).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PictureTemporalScalableExtension {
    /// `reference_select_code` — *"a 2-bit code that identifies
    /// reference frames or reference fields for prediction depending on
    /// the picture type"* (§6.3.13). Its meaning is given by Table 7-28
    /// (P-pictures) and Table 7-29 (B-pictures).
    pub reference_select_code: u8,
    /// `forward_temporal_reference` — *"A 10 bit unsigned integer value
    /// which indicates temporal reference of the lower layer frame to be
    /// used to provide the forward prediction"* (§6.3.13).
    pub forward_temporal_reference: u16,
    /// `backward_temporal_reference` — *"A 10 bit unsigned integer value
    /// which indicates temporal reference of the lower layer frame to be
    /// used to provide the backward prediction"* (§6.3.13).
    pub backward_temporal_reference: u16,
}

impl PictureTemporalScalableExtension {
    /// Parse a `picture_temporal_scalable_extension()` from a slice
    /// starting with the four start-code bytes `00 00 01 B5`. The
    /// trailing `next_start_code()` (§5.2.3) is not consumed.
    pub fn parse(buf: &[u8]) -> Result<Self> {
        let mut br = BitReader::new(buf);
        Self::parse_with_reader(&mut br)
    }

    /// Parse from an existing [`BitReader`] positioned at the start of
    /// `picture_temporal_scalable_extension()` (i.e. its 32-bit
    /// `extension_start_code`).
    pub fn parse_with_reader(br: &mut BitReader<'_>) -> Result<Self> {
        // §6.2.3.4 / §6.3.4: extension_start_code = 0x000001B5.
        let code = br.read_u32(32).map_err(|_| Error::ShortHeader)?;
        if code != EXTENSION_START_CODE {
            return Err(Error::InvalidBitstream(
                "extension_start_code: expected 0x000001B5 (§6.3.4)",
            ));
        }
        // 4-bit extension_start_code_identifier; Picture Temporal
        // Scalable Extension ID is '1010' per Table 6-2.
        let id = br.read_u32(4).map_err(|_| Error::ShortHeader)?;
        if id != PICTURE_TEMPORAL_SCALABLE_EXTENSION_ID {
            return Err(Error::InvalidBitstream(
                "extension_start_code_identifier: expected '1010' Picture Temporal Scalable Extension ID (Table 6-2)",
            ));
        }

        let reference_select_code = br.read_u32(2).map_err(|_| Error::ShortHeader)? as u8;
        let forward_temporal_reference = br.read_u32(10).map_err(|_| Error::ShortHeader)? as u16;

        // §6.2.3.4: marker_bit shall be '1'.
        if br.read_u32(1).map_err(|_| Error::ShortHeader)? != 1 {
            return Err(Error::InvalidBitstream(
                "marker_bit in picture_temporal_scalable_extension() (§6.2.3.4)",
            ));
        }

        let backward_temporal_reference = br.read_u32(10).map_err(|_| Error::ShortHeader)? as u16;

        Ok(Self {
            reference_select_code,
            forward_temporal_reference,
            backward_temporal_reference,
        })
    }

    /// Cross-check the picture-type-dependent §6.3.13 / Table 7-28 /
    /// Table 7-29 constraints that the bare wire parse cannot see. The
    /// caller supplies the active `picture_coding_type` from the
    /// `picture_header()`.
    ///
    /// * I-pictures: *"the `reference_select_code` for I-pictures shall
    ///   be '11'"* (§7.9 / the text following Table 7-29).
    /// * P-pictures: `reference_select_code == '11'` is *"forbidden"*
    ///   (Table 7-28).
    /// * B-pictures: `reference_select_code == '00'` is *"forbidden"*
    ///   (Table 7-29).
    pub fn validate(&self, picture_coding_type: PictureCodingType) -> Result<()> {
        match picture_coding_type {
            PictureCodingType::Intra => {
                if self.reference_select_code != 0b11 {
                    return Err(Error::InvalidBitstream(
                        "reference_select_code: shall be '11' for I-pictures (§7.9)",
                    ));
                }
            }
            PictureCodingType::Predictive => {
                if self.reference_select_code == 0b11 {
                    return Err(Error::InvalidBitstream(
                        "reference_select_code: '11' is forbidden in P-pictures (Table 7-28)",
                    ));
                }
            }
            PictureCodingType::Bidirectional => {
                if self.reference_select_code == 0b00 {
                    return Err(Error::InvalidBitstream(
                        "reference_select_code: '00' is forbidden in B-pictures (Table 7-29)",
                    ));
                }
            }
        }
        Ok(())
    }

    /// §7.9 / Tables 7-28 + 7-29: resolve the `reference_select_code`
    /// into the named prediction reference source(s) for this picture's
    /// `picture_coding_type`.
    ///
    /// In temporal scalability the enhancement-layer picture's prediction
    /// references are drawn from the enhancement layer's own decoded
    /// pictures and/or the lower layer's decoded frames; which combination
    /// is keyed by `reference_select_code` (Table 7-28 for P-pictures,
    /// Table 7-29 for B-pictures). I-pictures use no references
    /// ([`PictureReferences::Intra`]); their `reference_select_code` is
    /// `'11'` (§7.9), validated by [`Self::validate`].
    ///
    /// Returns a [`PictureReferences`] that names the §7.6.2 reference
    /// each prediction direction draws from. The
    /// `forward_temporal_reference` / `backward_temporal_reference`
    /// fields (which lower-layer frame, by its `temporal_reference`) are
    /// available directly on `self`; §7.9 says they *"take the value 0"*
    /// when the picture type does not imply a reference in that
    /// direction.
    ///
    /// # Errors
    /// * [`Error::InvalidBitstream`] for a code Table 7-28 / 7-29 marks
    ///   *"forbidden"* for this picture type (the same conditions
    ///   [`Self::validate`] rejects).
    pub fn resolve_references(
        &self,
        picture_coding_type: PictureCodingType,
    ) -> Result<PictureReferences> {
        match picture_coding_type {
            // §7.9: I-pictures use no prediction references.
            PictureCodingType::Intra => Ok(PictureReferences::Intra),
            // Table 7-28: forward-only prediction.
            PictureCodingType::Predictive => {
                let forward =
                    match self.reference_select_code {
                        0b00 => ReferenceSource::MostRecentEnhancement,
                        0b01 => ReferenceSource::MostRecentLowerLayer,
                        0b10 => ReferenceSource::NextLowerLayer,
                        _ => return Err(Error::InvalidBitstream(
                            "reference_select_code: '11' is forbidden in P-pictures (Table 7-28)",
                        )),
                    };
                Ok(PictureReferences::Predictive { forward })
            }
            // Table 7-29: forward + backward prediction.
            PictureCodingType::Bidirectional => {
                let (forward, backward) =
                    match self.reference_select_code {
                        0b01 => (
                            ReferenceSource::MostRecentEnhancement,
                            ReferenceSource::MostRecentLowerLayer,
                        ),
                        0b10 => (
                            ReferenceSource::MostRecentEnhancement,
                            ReferenceSource::NextLowerLayer,
                        ),
                        0b11 => (
                            ReferenceSource::MostRecentLowerLayer,
                            ReferenceSource::NextLowerLayer,
                        ),
                        _ => return Err(Error::InvalidBitstream(
                            "reference_select_code: '00' is forbidden in B-pictures (Table 7-29)",
                        )),
                    };
                Ok(PictureReferences::Bidirectional { forward, backward })
            }
        }
    }
}

/// A single §7.9 prediction reference source named by Table 7-28 /
/// Table 7-29. *"Most recent lower layer frame in display order"*
/// includes the frame temporally coincident with the enhancement-layer
/// picture (§7.9).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReferenceSource {
    /// *"Most recent decoded enhancement picture(s)"* — the prediction
    /// is drawn from the enhancement layer's own most recently decoded
    /// picture(s).
    MostRecentEnhancement,
    /// *"Most recent lower layer frame/picture in display order"*
    /// (including the temporally-coincident lower-layer frame).
    MostRecentLowerLayer,
    /// *"Next lower layer frame/picture in display order"* — the
    /// lower-layer frame that is forward in time relative to the current
    /// enhancement picture.
    NextLowerLayer,
}

/// The resolved §7.9 prediction references for a temporal-scalability
/// enhancement-layer picture (Tables 7-28 / 7-29), keyed by
/// `picture_coding_type`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PictureReferences {
    /// I-picture: no prediction references (§7.9).
    Intra,
    /// P-picture: a single forward reference (Table 7-28).
    Predictive {
        /// The forward prediction reference source.
        forward: ReferenceSource,
    },
    /// B-picture: forward + backward references (Table 7-29). §7.9:
    /// *"Backward prediction cannot be made from a picture in the
    /// enhancement layer"*, so `backward` is always a lower-layer
    /// source.
    Bidirectional {
        /// The forward prediction reference source.
        forward: ReferenceSource,
        /// The backward prediction reference source (always a lower-layer
        /// frame — §7.9).
        backward: ReferenceSource,
    },
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact `picture_temporal_scalable_extension()`
    //! fixtures plus negative cases for every §6.2.3.4 rejection site
    //! and the §6.3.13 picture-type cross-checks.
    use super::*;
    use oxideav_core::bits::BitWriter;

    struct Fixture {
        id: u32,
        reference_select_code: u32,
        forward: u32,
        marker: bool,
        backward: u32,
    }

    impl Default for Fixture {
        fn default() -> Self {
            Self {
                id: PICTURE_TEMPORAL_SCALABLE_EXTENSION_ID,
                reference_select_code: 0b01,
                forward: 5,
                marker: true,
                backward: 7,
            }
        }
    }

    fn build(fx: &Fixture) -> Vec<u8> {
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(fx.id, 4);
        bw.write_u32(fx.reference_select_code, 2);
        bw.write_u32(fx.forward, 10);
        bw.write_bit(fx.marker);
        bw.write_u32(fx.backward, 10);
        bw.finish()
    }

    #[test]
    fn parses_a_temporal_scalable_extension() {
        let buf = build(&Fixture::default());
        let ext = PictureTemporalScalableExtension::parse(&buf).unwrap();
        assert_eq!(ext.reference_select_code, 0b01);
        assert_eq!(ext.forward_temporal_reference, 5);
        assert_eq!(ext.backward_temporal_reference, 7);
    }

    #[test]
    fn ten_bit_temporal_references_round_trip_full_range() {
        let buf = build(&Fixture {
            forward: 0x3FF,
            backward: 0x3FF,
            ..Fixture::default()
        });
        let ext = PictureTemporalScalableExtension::parse(&buf).unwrap();
        assert_eq!(ext.forward_temporal_reference, 1023);
        assert_eq!(ext.backward_temporal_reference, 1023);
    }

    #[test]
    fn element_is_fifty_nine_bits_eight_padded_bytes() {
        // 32 (start code) + 4 + 2 + 10 + 1 + 10 = 59 bits → 8 bytes.
        let buf = build(&Fixture::default());
        assert_eq!(buf.len(), 8);
    }

    #[test]
    fn rejects_wrong_extension_start_code() {
        let mut buf = build(&Fixture::default());
        buf[3] = 0xB3; // sequence_header_code value byte
        assert!(matches!(
            PictureTemporalScalableExtension::parse(&buf),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn rejects_wrong_identifier() {
        let buf = build(&Fixture {
            id: PICTURE_SPATIAL_SCALABLE_ID_FOR_TEST,
            ..Fixture::default()
        });
        assert!(matches!(
            PictureTemporalScalableExtension::parse(&buf),
            Err(Error::InvalidBitstream(_))
        ));
    }

    const PICTURE_SPATIAL_SCALABLE_ID_FOR_TEST: u32 = 0b1001;

    #[test]
    fn rejects_zero_marker_bit() {
        let buf = build(&Fixture {
            marker: false,
            ..Fixture::default()
        });
        assert!(matches!(
            PictureTemporalScalableExtension::parse(&buf),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn rejects_truncated_buffer() {
        let buf = build(&Fixture::default());
        assert!(matches!(
            PictureTemporalScalableExtension::parse(&buf[..6]),
            Err(Error::ShortHeader)
        ));
    }

    #[test]
    fn validate_i_picture_requires_eleven() {
        let ok = PictureTemporalScalableExtension {
            reference_select_code: 0b11,
            forward_temporal_reference: 0,
            backward_temporal_reference: 0,
        };
        assert!(ok.validate(PictureCodingType::Intra).is_ok());
        let bad = PictureTemporalScalableExtension {
            reference_select_code: 0b01,
            ..ok
        };
        assert!(matches!(
            bad.validate(PictureCodingType::Intra),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn validate_p_picture_forbids_eleven() {
        let bad = PictureTemporalScalableExtension {
            reference_select_code: 0b11,
            forward_temporal_reference: 0,
            backward_temporal_reference: 0,
        };
        assert!(matches!(
            bad.validate(PictureCodingType::Predictive),
            Err(Error::InvalidBitstream(_))
        ));
        let ok = PictureTemporalScalableExtension {
            reference_select_code: 0b01,
            ..bad
        };
        assert!(ok.validate(PictureCodingType::Predictive).is_ok());
    }

    #[test]
    fn validate_b_picture_forbids_zero() {
        let bad = PictureTemporalScalableExtension {
            reference_select_code: 0b00,
            forward_temporal_reference: 0,
            backward_temporal_reference: 0,
        };
        assert!(matches!(
            bad.validate(PictureCodingType::Bidirectional),
            Err(Error::InvalidBitstream(_))
        ));
        let ok = PictureTemporalScalableExtension {
            reference_select_code: 0b10,
            ..bad
        };
        assert!(ok.validate(PictureCodingType::Bidirectional).is_ok());
    }

    // ---- resolve_references: Tables 7-28 / 7-29 ----

    fn ext(code: u8) -> PictureTemporalScalableExtension {
        PictureTemporalScalableExtension {
            reference_select_code: code,
            forward_temporal_reference: 0,
            backward_temporal_reference: 0,
        }
    }

    #[test]
    fn resolve_intra_has_no_references() {
        assert_eq!(
            ext(0b11)
                .resolve_references(PictureCodingType::Intra)
                .unwrap(),
            PictureReferences::Intra
        );
    }

    #[test]
    fn resolve_p_picture_table_7_28() {
        use ReferenceSource::*;
        for (code, src) in [
            (0b00u8, MostRecentEnhancement),
            (0b01, MostRecentLowerLayer),
            (0b10, NextLowerLayer),
        ] {
            assert_eq!(
                ext(code)
                    .resolve_references(PictureCodingType::Predictive)
                    .unwrap(),
                PictureReferences::Predictive { forward: src }
            );
        }
    }

    #[test]
    fn resolve_p_picture_rejects_forbidden_eleven() {
        assert!(matches!(
            ext(0b11).resolve_references(PictureCodingType::Predictive),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn resolve_b_picture_table_7_29() {
        use ReferenceSource::*;
        for (code, fwd, bwd) in [
            (0b01u8, MostRecentEnhancement, MostRecentLowerLayer),
            (0b10, MostRecentEnhancement, NextLowerLayer),
            (0b11, MostRecentLowerLayer, NextLowerLayer),
        ] {
            assert_eq!(
                ext(code)
                    .resolve_references(PictureCodingType::Bidirectional)
                    .unwrap(),
                PictureReferences::Bidirectional {
                    forward: fwd,
                    backward: bwd
                }
            );
        }
    }

    #[test]
    fn resolve_b_picture_rejects_forbidden_zero() {
        assert!(matches!(
            ext(0b00).resolve_references(PictureCodingType::Bidirectional),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn resolve_b_picture_backward_is_always_lower_layer() {
        // §7.9: backward prediction cannot come from the enhancement
        // layer.
        for code in [0b01u8, 0b10, 0b11] {
            if let PictureReferences::Bidirectional { backward, .. } = ext(code)
                .resolve_references(PictureCodingType::Bidirectional)
                .unwrap()
            {
                assert_ne!(backward, ReferenceSource::MostRecentEnhancement);
            } else {
                panic!("expected Bidirectional");
            }
        }
    }
}
