//! Parser for the remainder of `macroblock_modes()` after
//! `macroblock_type` — the `frame_motion_type` / `field_motion_type`
//! prediction-mode codes and the `dct_type` flag — per ISO/IEC
//! 13818-2 (Recommendation ITU-T H.262) §6.2.5.1, with field
//! semantics from §6.3.17.1 and the meaning tables 6-17, 6-18 and
//! 6-19.
//!
//! Round 7 landed [`crate::MacroblockType`], the leading VLC of
//! `macroblock_modes()`. Round 9 landed the macroblock-layer
//! `quantizer_scale` that follows it inside `macroblock()`. Round 10
//! finishes `macroblock_modes()` itself: the two fixed-length
//! 2-bit motion-type codes and the 1-bit `dct_type` flag. The
//! `motion_vectors()` block (Tables B-10 / B-11) and the residual
//! block loop remain out of scope.
//!
//! The `macroblock_modes()` syntax (§6.2.5.1) reads, after
//! `macroblock_type`:
//!
//! ```text
//! if ( ( spatial_temporal_weight_code_flag == 1 ) &&
//!      ( spatial_temporal_weight_code_table_index != '00' ) )
//!     spatial_temporal_weight_code                  2   uimsbf
//! if ( macroblock_motion_forward ||
//!      macroblock_motion_backward ) {
//!     if ( picture_structure == 'frame' ) {
//!         if ( frame_pred_frame_dct == 0 )
//!             frame_motion_type                     2   uimsbf
//!     } else {
//!         field_motion_type                         2   uimsbf
//!     }
//! }
//! if ( ( picture_structure == "Frame picture" ) &&
//!      ( frame_pred_frame_dct == 0 ) &&
//!      ( macroblock_intra || macroblock_pattern ) )
//!     dct_type                                      1   uimsbf
//! ```
//!
//! `spatial_temporal_weight_code` is gated on
//! `spatial_temporal_weight_code_flag`, which is always `0` for the
//! non-scalable Tables B-2 / B-3 / B-4 that round 7 decodes (no
//! `sequence_scalable_extension()` is parsed by this crate). It is
//! therefore never present in any stream this crate can currently
//! reach, and this module does not read it; a later scalable round
//! will add it.
//!
//! ## Motion-type presence and the concealment-MV special case
//!
//! The motion-type code is present iff either motion flag from the
//! `macroblock_type` is set. §6.3.17.1 additionally notes that for an
//! **intra** macroblock with `concealment_motion_vectors == 1` the
//! motion-type code is *not* in the bitstream — but in that case the
//! macroblock_type itself sets neither motion flag (intra rows of
//! Tables B-2 / B-3 / B-4 have `fwd == bwd == 0`), so the
//! `motion_forward || motion_backward` gate already excludes it. The
//! decoder then proceeds *as if* the omitted type were "Frame-based" /
//! "Field-based"; that defaulting is a motion-vector-decode concern and
//! is left to a later round.
//!
//! ## `dct_type` defaulting
//!
//! When `dct_type` is absent its effective value is derived from
//! Table 6-19: `0` ("frame") when `frame_pred_frame_dct == 1`, and
//! "unused" in a field picture or for a not-coded macroblock. This
//! module reports the field as [`Option`]; deriving the effective
//! value for the absent case is a block-organisation concern deferred
//! to the residual round.
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)) §6.2.5.1, §6.3.17.1, and
//! Tables 6-17, 6-18, 6-19.

use oxideav_core::bits::BitReader;

use crate::macroblock_type::MacroblockType;
use crate::picture_header::PictureStructure;
use crate::{Error, Result};

/// The prediction type a `frame_motion_type` / `field_motion_type`
/// code selects (Tables 6-17 / 6-18).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PredictionType {
    /// `Field-based` — prediction from one or two reference fields,
    /// field motion vectors.
    FieldBased,
    /// `Frame-based` — prediction from a reference frame, frame motion
    /// vectors. Only reachable through `frame_motion_type` (Table 6-17).
    FrameBased,
    /// `16x8 MC` — two field motion vectors covering the upper and
    /// lower halves of the macroblock. Only reachable through
    /// `field_motion_type` (Table 6-18).
    SixteenByEight,
    /// `Dual-Prime` — a single field motion vector plus a differential
    /// `dmvector`. Sets `dmv == 1`.
    DualPrime,
}

/// `mv_format` — whether the macroblock's motion vectors are field- or
/// frame-organised (§6.3.17.2, derived from Table 6-17 / 6-18).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MvFormat {
    /// Field motion vectors.
    Field,
    /// Frame motion vectors. Only `Frame-based` prediction produces it.
    Frame,
}

/// A decoded `frame_motion_type` / `field_motion_type` code together
/// with the three derived quantities §6.3.17.2 reads out of Tables
/// 6-17 / 6-18: `motion_vector_count`, `mv_format`, and `dmv`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotionType {
    /// The raw 2-bit code as read from the bitstream (`01`, `10`, `11`;
    /// `00` is reserved and rejected).
    pub code: u8,
    /// The prediction type the code selects.
    pub prediction_type: PredictionType,
    /// `motion_vector_count` — `1` or `2` (Table 6-17 / 6-18).
    pub motion_vector_count: u8,
    /// `mv_format` — field or frame motion vectors.
    pub mv_format: MvFormat,
    /// `dmv` — `1` for Dual-Prime, `0` otherwise.
    pub dmv: bool,
}

impl MotionType {
    /// Decode a `frame_motion_type` code per Table 6-17.
    ///
    /// The `spatial_temporal_weight_class` distinguishes the two
    /// `Field-based` rows (`0,1` → 2 vectors; `2,3` → 1 vector). For
    /// the non-scalable tables this class is always `0`, so the
    /// 2-vector row applies; this module accepts a caller-supplied
    /// class so a later scalable round can reuse it.
    fn from_frame_code(code: u8, spatial_temporal_weight_class: u8) -> Result<Self> {
        let (prediction_type, motion_vector_count, mv_format, dmv) = match code {
            0b00 => {
                return Err(Error::InvalidBitstream(
                    "frame_motion_type: reserved code 00 (Table 6-17)",
                ))
            }
            0b01 => {
                // Field-based: 2 vectors for class 0/1, 1 vector for 2/3.
                let count = if spatial_temporal_weight_class <= 1 {
                    2
                } else {
                    1
                };
                (PredictionType::FieldBased, count, MvFormat::Field, false)
            }
            0b10 => (PredictionType::FrameBased, 1, MvFormat::Frame, false),
            0b11 => (PredictionType::DualPrime, 1, MvFormat::Field, true),
            _ => unreachable!("frame_motion_type is a 2-bit field"),
        };
        Ok(Self {
            code,
            prediction_type,
            motion_vector_count,
            mv_format,
            dmv,
        })
    }

    /// Decode a `field_motion_type` code per Table 6-18.
    fn from_field_code(code: u8) -> Result<Self> {
        let (prediction_type, motion_vector_count, mv_format, dmv) = match code {
            0b00 => {
                return Err(Error::InvalidBitstream(
                    "field_motion_type: reserved code 00 (Table 6-18)",
                ))
            }
            0b01 => (PredictionType::FieldBased, 1, MvFormat::Field, false),
            0b10 => (PredictionType::SixteenByEight, 2, MvFormat::Field, false),
            0b11 => (PredictionType::DualPrime, 1, MvFormat::Field, true),
            _ => unreachable!("field_motion_type is a 2-bit field"),
        };
        Ok(Self {
            code,
            prediction_type,
            motion_vector_count,
            mv_format,
            dmv,
        })
    }
}

/// The fields of `macroblock_modes()` that follow `macroblock_type`:
/// the optional motion-type code and the optional `dct_type` flag.
///
/// Both are `Option`: each is present only when the §6.2.5.1 gate
/// conditions hold. When absent the parser reads zero bits for that
/// field and the decoder applies the §6.3.17.1 / Table 6-19 defaults
/// (deferred to later rounds).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MacroblockModesTail {
    /// The decoded `frame_motion_type` (frame pictures) or
    /// `field_motion_type` (field pictures) when either motion flag is
    /// set and the field is not omitted by `frame_pred_frame_dct`;
    /// `None` otherwise.
    pub motion_type: Option<MotionType>,
    /// `dct_type` when present (frame picture, `frame_pred_frame_dct ==
    /// 0`, and the macroblock is intra or has a coded pattern);
    /// `Some(true)` = field DCT coded, `Some(false)` = frame DCT coded.
    /// `None` when the field is absent (Table 6-19 supplies the
    /// effective value).
    pub dct_type: Option<bool>,
    /// Bit position (relative to the start of the buffer the
    /// [`BitReader`] was created from) right after the consumed fields.
    /// Equal to the entry position when both fields are absent.
    pub bit_position_after: u64,
}

/// Picture-level state, read once per picture, that gates which
/// `macroblock_modes()` fields are present (§6.2.5.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MacroblockModesContext {
    /// `picture_structure` from `picture_coding_extension()` (§6.3.11,
    /// Table 6-14).
    pub picture_structure: PictureStructure,
    /// `frame_pred_frame_dct` from `picture_coding_extension()`. When
    /// `1`, both the motion-type code (in frame pictures) and
    /// `dct_type` are omitted.
    pub frame_pred_frame_dct: bool,
    /// `spatial_temporal_weight_class` for the macroblock. Always `0`
    /// for the non-scalable Tables B-2 / B-3 / B-4 this crate decodes;
    /// it selects between the two `Field-based` rows of Table 6-17.
    pub spatial_temporal_weight_class: u8,
}

impl MacroblockModesContext {
    /// A non-scalable context: `spatial_temporal_weight_class == 0`,
    /// taking the `picture_structure` and `frame_pred_frame_dct`
    /// straight from a parsed `picture_coding_extension()`.
    pub fn new(picture_structure: PictureStructure, frame_pred_frame_dct: bool) -> Self {
        Self {
            picture_structure,
            frame_pred_frame_dct,
            spatial_temporal_weight_class: 0,
        }
    }
}

impl MacroblockModesTail {
    /// Parse the post-`macroblock_type` fields of `macroblock_modes()`
    /// from the current position of `br`.
    ///
    /// `mb_type` supplies the motion / intra / pattern flags decoded by
    /// [`MacroblockType`]; `ctx` supplies the picture-level gates.
    ///
    /// `spatial_temporal_weight_code` is **not** read: it is gated on
    /// `spatial_temporal_weight_code_flag`, which is always `0` for the
    /// non-scalable tables this crate can decode. Callers that pass a
    /// `mb_type` with `spatial_temporal_weight_code_flag == true`
    /// (only reachable once the scalable tables land) are rejected so
    /// the bitstream cursor is never silently misaligned.
    ///
    /// Errors:
    /// * [`Error::InvalidBitstream`] if a reserved motion-type code
    ///   `00` is read, or if `mb_type` claims a
    ///   `spatial_temporal_weight_code_flag` this round cannot handle.
    /// * [`Error::ShortHeader`] if the bitstream ends before a present
    ///   field could be read.
    pub fn parse(
        br: &mut BitReader<'_>,
        mb_type: &MacroblockType,
        ctx: &MacroblockModesContext,
    ) -> Result<Self> {
        if mb_type.spatial_temporal_weight_code_flag {
            return Err(Error::InvalidBitstream(
                "macroblock_modes: spatial_temporal_weight_code_flag set requires the scalable \
                 tables, not yet supported (§6.2.5.1)",
            ));
        }

        let motion_type = if mb_type.macroblock_motion_forward || mb_type.macroblock_motion_backward
        {
            match ctx.picture_structure {
                PictureStructure::Frame => {
                    if ctx.frame_pred_frame_dct {
                        // Omitted; the decoder assumes Frame-based.
                        None
                    } else {
                        let code = br.read_u32(2).map_err(|_| Error::ShortHeader)? as u8;
                        Some(MotionType::from_frame_code(
                            code,
                            ctx.spatial_temporal_weight_class,
                        )?)
                    }
                }
                PictureStructure::TopField | PictureStructure::BottomField => {
                    let code = br.read_u32(2).map_err(|_| Error::ShortHeader)? as u8;
                    Some(MotionType::from_field_code(code)?)
                }
            }
        } else {
            None
        };

        let dct_type_present = ctx.picture_structure == PictureStructure::Frame
            && !ctx.frame_pred_frame_dct
            && (mb_type.macroblock_intra || mb_type.macroblock_pattern);

        let dct_type = if dct_type_present {
            Some(br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1)
        } else {
            None
        };

        Ok(Self {
            motion_type,
            dct_type,
            bit_position_after: br.bit_position(),
        })
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact round-trips covering the motion-type tables,
    //! the `dct_type` presence matrix, and the field-omission paths.
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Build a `MacroblockType` with explicit flags (the parser only
    /// reads the five flags we exercise here).
    fn mb_type(fwd: bool, bwd: bool, intra: bool, pattern: bool, quant: bool) -> MacroblockType {
        MacroblockType {
            macroblock_quant: quant,
            macroblock_motion_forward: fwd,
            macroblock_motion_backward: bwd,
            macroblock_pattern: pattern,
            macroblock_intra: intra,
            spatial_temporal_weight_code_flag: false,
            bit_position_after: 0,
        }
    }

    /// Emit `bits` codewords back to back, pad to a byte with a
    /// trailing `1`, and return a reader over the buffer.
    fn reader(codes: &[(u32, u32)]) -> Vec<u8> {
        let mut bw = BitWriter::new();
        for &(code, n) in codes {
            bw.write_u32(code, n);
        }
        bw.write_bit(true);
        bw.align_to_byte();
        bw.finish()
    }

    #[test]
    fn frame_motion_type_field_based_two_vectors() {
        // frame_motion_type '01', class 0 → Field-based, 2 vectors.
        let buf = reader(&[(0b01, 2)]);
        let mut br = BitReader::new(&buf);
        let mt = mb_type(true, false, false, true, false);
        let ctx = MacroblockModesContext::new(PictureStructure::Frame, false);
        let tail = MacroblockModesTail::parse(&mut br, &mt, &ctx).expect("parse");
        let motion = tail.motion_type.expect("motion type present");
        assert_eq!(motion.code, 0b01);
        assert_eq!(motion.prediction_type, PredictionType::FieldBased);
        assert_eq!(motion.motion_vector_count, 2);
        assert_eq!(motion.mv_format, MvFormat::Field);
        assert!(!motion.dmv);
    }

    #[test]
    fn frame_motion_type_field_based_one_vector_class_two() {
        // frame_motion_type '01', class 2 → Field-based, 1 vector.
        let buf = reader(&[(0b01, 2)]);
        let mut br = BitReader::new(&buf);
        let mt = mb_type(true, false, false, true, false);
        let ctx = MacroblockModesContext {
            picture_structure: PictureStructure::Frame,
            frame_pred_frame_dct: false,
            spatial_temporal_weight_class: 2,
        };
        let tail = MacroblockModesTail::parse(&mut br, &mt, &ctx).expect("parse");
        let motion = tail.motion_type.expect("motion type present");
        assert_eq!(motion.prediction_type, PredictionType::FieldBased);
        assert_eq!(motion.motion_vector_count, 1);
    }

    #[test]
    fn frame_motion_type_frame_based() {
        let buf = reader(&[(0b10, 2)]);
        let mut br = BitReader::new(&buf);
        let mt = mb_type(true, true, false, true, false);
        let ctx = MacroblockModesContext::new(PictureStructure::Frame, false);
        let tail = MacroblockModesTail::parse(&mut br, &mt, &ctx).expect("parse");
        let motion = tail.motion_type.expect("present");
        assert_eq!(motion.prediction_type, PredictionType::FrameBased);
        assert_eq!(motion.motion_vector_count, 1);
        assert_eq!(motion.mv_format, MvFormat::Frame);
        assert!(!motion.dmv);
    }

    #[test]
    fn frame_motion_type_dual_prime() {
        let buf = reader(&[(0b11, 2)]);
        let mut br = BitReader::new(&buf);
        let mt = mb_type(true, false, false, true, false);
        let ctx = MacroblockModesContext::new(PictureStructure::Frame, false);
        let tail = MacroblockModesTail::parse(&mut br, &mt, &ctx).expect("parse");
        let motion = tail.motion_type.expect("present");
        assert_eq!(motion.prediction_type, PredictionType::DualPrime);
        assert_eq!(motion.motion_vector_count, 1);
        assert_eq!(motion.mv_format, MvFormat::Field);
        assert!(motion.dmv);
    }

    #[test]
    fn frame_motion_type_reserved_code_rejected() {
        let buf = reader(&[(0b00, 2)]);
        let mut br = BitReader::new(&buf);
        let mt = mb_type(true, false, false, true, false);
        let ctx = MacroblockModesContext::new(PictureStructure::Frame, false);
        let err = MacroblockModesTail::parse(&mut br, &mt, &ctx).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn field_motion_type_field_based() {
        let buf = reader(&[(0b01, 2)]);
        let mut br = BitReader::new(&buf);
        let mt = mb_type(true, false, false, true, false);
        let ctx = MacroblockModesContext::new(PictureStructure::TopField, false);
        let tail = MacroblockModesTail::parse(&mut br, &mt, &ctx).expect("parse");
        let motion = tail.motion_type.expect("present");
        assert_eq!(motion.prediction_type, PredictionType::FieldBased);
        assert_eq!(motion.motion_vector_count, 1);
        assert_eq!(motion.mv_format, MvFormat::Field);
        assert!(!motion.dmv);
        // dct_type is never present in a field picture.
        assert_eq!(tail.dct_type, None);
    }

    #[test]
    fn field_motion_type_16x8() {
        let buf = reader(&[(0b10, 2)]);
        let mut br = BitReader::new(&buf);
        let mt = mb_type(true, false, false, true, false);
        let ctx = MacroblockModesContext::new(PictureStructure::BottomField, false);
        let tail = MacroblockModesTail::parse(&mut br, &mt, &ctx).expect("parse");
        let motion = tail.motion_type.expect("present");
        assert_eq!(motion.prediction_type, PredictionType::SixteenByEight);
        assert_eq!(motion.motion_vector_count, 2);
        assert_eq!(motion.mv_format, MvFormat::Field);
    }

    #[test]
    fn field_motion_type_dual_prime() {
        let buf = reader(&[(0b11, 2)]);
        let mut br = BitReader::new(&buf);
        let mt = mb_type(false, true, false, true, false);
        let ctx = MacroblockModesContext::new(PictureStructure::TopField, false);
        let tail = MacroblockModesTail::parse(&mut br, &mt, &ctx).expect("parse");
        let motion = tail.motion_type.expect("present");
        assert_eq!(motion.prediction_type, PredictionType::DualPrime);
        assert!(motion.dmv);
    }

    #[test]
    fn field_motion_type_reserved_code_rejected() {
        let buf = reader(&[(0b00, 2)]);
        let mut br = BitReader::new(&buf);
        let mt = mb_type(true, false, false, true, false);
        let ctx = MacroblockModesContext::new(PictureStructure::TopField, false);
        let err = MacroblockModesTail::parse(&mut br, &mt, &ctx).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn motion_type_absent_when_no_motion_flags() {
        // Intra macroblock: no motion flags, no motion type. But a
        // frame-picture intra macroblock with frame_pred_frame_dct == 0
        // *does* carry dct_type.
        let buf = reader(&[(0b1, 1)]); // dct_type bit '1'
        let mut br = BitReader::new(&buf);
        let mt = mb_type(false, false, true, false, false);
        let ctx = MacroblockModesContext::new(PictureStructure::Frame, false);
        let tail = MacroblockModesTail::parse(&mut br, &mt, &ctx).expect("parse");
        assert_eq!(tail.motion_type, None);
        assert_eq!(tail.dct_type, Some(true));
        assert_eq!(tail.bit_position_after, 1);
    }

    #[test]
    fn motion_type_omitted_when_frame_pred_frame_dct() {
        // frame picture, frame_pred_frame_dct == 1 → motion type omitted
        // even though motion flags are set; dct_type also omitted.
        let buf = reader(&[]); // nothing read
        let mut br = BitReader::new(&buf);
        let mt = mb_type(true, true, false, true, false);
        let ctx = MacroblockModesContext::new(PictureStructure::Frame, true);
        let tail = MacroblockModesTail::parse(&mut br, &mt, &ctx).expect("parse");
        assert_eq!(tail.motion_type, None);
        assert_eq!(tail.dct_type, None);
        assert_eq!(tail.bit_position_after, 0);
    }

    #[test]
    fn dct_type_present_for_coded_inter_macroblock() {
        // Frame picture, fpfd==0, inter with coded pattern → motion
        // type then dct_type.
        let buf = reader(&[(0b10, 2), (0b1, 1)]); // frame_motion_type + dct_type
        let mut br = BitReader::new(&buf);
        let mt = mb_type(true, false, false, true, false);
        let ctx = MacroblockModesContext::new(PictureStructure::Frame, false);
        let tail = MacroblockModesTail::parse(&mut br, &mt, &ctx).expect("parse");
        assert!(tail.motion_type.is_some());
        assert_eq!(tail.dct_type, Some(true));
        assert_eq!(tail.bit_position_after, 3);
    }

    #[test]
    fn dct_type_absent_for_not_coded_inter_macroblock() {
        // Inter macroblock with no pattern and not intra → dct_type
        // absent (the !(intra||pattern) row of Table 6-19).
        let buf = reader(&[(0b10, 2)]); // only frame_motion_type
        let mut br = BitReader::new(&buf);
        let mt = mb_type(true, false, false, false, false);
        let ctx = MacroblockModesContext::new(PictureStructure::Frame, false);
        let tail = MacroblockModesTail::parse(&mut br, &mt, &ctx).expect("parse");
        assert!(tail.motion_type.is_some());
        assert_eq!(tail.dct_type, None);
        assert_eq!(tail.bit_position_after, 2);
    }

    #[test]
    fn dct_type_zero_decodes_frame_coded() {
        let buf = reader(&[(0b0, 1)]);
        let mut br = BitReader::new(&buf);
        let mt = mb_type(false, false, true, false, false);
        let ctx = MacroblockModesContext::new(PictureStructure::Frame, false);
        let tail = MacroblockModesTail::parse(&mut br, &mt, &ctx).expect("parse");
        assert_eq!(tail.dct_type, Some(false));
    }

    #[test]
    fn all_fields_absent_consumes_zero_bits() {
        // Field picture, intra (no motion flags), so motion type absent;
        // dct_type never present in a field picture.
        let buf = [0xFFu8; 1];
        let mut br = BitReader::new(&buf);
        let mt = mb_type(false, false, true, false, false);
        let ctx = MacroblockModesContext::new(PictureStructure::TopField, false);
        let tail = MacroblockModesTail::parse(&mut br, &mt, &ctx).expect("parse");
        assert_eq!(tail.motion_type, None);
        assert_eq!(tail.dct_type, None);
        assert_eq!(tail.bit_position_after, 0);
        assert_eq!(br.bit_position(), 0);
    }

    #[test]
    fn scalable_flag_is_rejected() {
        let buf = reader(&[(0b01, 2)]);
        let mut br = BitReader::new(&buf);
        let mut mt = mb_type(true, false, false, true, false);
        mt.spatial_temporal_weight_code_flag = true;
        let ctx = MacroblockModesContext::new(PictureStructure::Frame, false);
        let err = MacroblockModesTail::parse(&mut br, &mt, &ctx).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn truncated_motion_type_is_short() {
        // Only 1 bit available but motion type needs 2.
        let buf = [0b1000_0000u8];
        let mut br = BitReader::new(&buf);
        br.skip(7).expect("skip");
        let mt = mb_type(true, false, false, true, false);
        let ctx = MacroblockModesContext::new(PictureStructure::Frame, false);
        let err = MacroblockModesTail::parse(&mut br, &mt, &ctx).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    #[test]
    fn truncated_dct_type_is_short() {
        // Empty buffer, intra frame-picture macroblock → dct_type
        // present but no bit to read.
        let buf: [u8; 0] = [];
        let mut br = BitReader::new(&buf);
        let mt = mb_type(false, false, true, false, false);
        let ctx = MacroblockModesContext::new(PictureStructure::Frame, false);
        let err = MacroblockModesTail::parse(&mut br, &mt, &ctx).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    #[test]
    fn context_new_sets_zero_class() {
        let ctx = MacroblockModesContext::new(PictureStructure::Frame, true);
        assert_eq!(ctx.spatial_temporal_weight_class, 0);
        assert_eq!(ctx.picture_structure, PictureStructure::Frame);
        assert!(ctx.frame_pred_frame_dct);
    }

    #[test]
    fn debug_impls_smoke() {
        let tail = MacroblockModesTail {
            motion_type: Some(MotionType {
                code: 0b10,
                prediction_type: PredictionType::FrameBased,
                motion_vector_count: 1,
                mv_format: MvFormat::Frame,
                dmv: false,
            }),
            dct_type: Some(true),
            bit_position_after: 3,
        };
        let s = format!("{tail:?}");
        assert!(s.contains("MacroblockModesTail"));
        assert!(s.contains("FrameBased"));
        assert!(format!("{:?}", MvFormat::Field).contains("Field"));
        assert!(format!("{:?}", PredictionType::DualPrime).contains("DualPrime"));
    }
}
