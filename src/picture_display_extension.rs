//! Parser for the MPEG-2 video `picture_display_extension()` element.
//!
//! Implements the bitstream syntax in ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262) §6.2.3.3 and the field semantics in §6.3.12. The
//! picture-display extension carries up to three `(horizontal,
//! vertical)` frame-centre offset pairs that the display process can
//! use to implement pan-scan; §6.3.12 explicitly observes that the
//! information *"does not affect the decoding process and may be
//! ignored by decoders that conform to this specification"*. The
//! parser still consumes the bits exactly so that the byte cursor
//! advances correctly to the trailing `next_start_code()` (§5.2.3)
//! and so encoders that round-trip a stream can preserve the offsets.
//!
//! ## Wire shape (§6.2.3.3)
//!
//! ```text
//! picture_display_extension() {
//!     extension_start_code_identifier        4
//!     for (i = 0; i < number_of_frame_centre_offsets; i++) {
//!         frame_centre_horizontal_offset    16  (signed)
//!         marker_bit                         1
//!         frame_centre_vertical_offset      16  (signed)
//!         marker_bit                         1
//!     }
//!     next_start_code()
//! }
//! ```
//!
//! The 32-bit `extension_start_code` (value `0x000001B5` per §6.3.4)
//! precedes the syntax above; the parser consumes the four start-code
//! bytes plus the 4-bit identifier (Table 6-2 entry `0111`, see
//! [`PICTURE_DISPLAY_EXTENSION_ID`]) so a caller can hand it a slice
//! starting at the start-code prefix exactly as the other
//! `*_extension()` parsers expect.
//!
//! ## `number_of_frame_centre_offsets` (§6.3.12)
//!
//! `number_of_frame_centre_offsets` is **not** carried in the wire
//! payload; the decoder derives it from the active
//! `sequence_extension()` flag `progressive_sequence` plus the active
//! `picture_coding_extension()` flags `picture_structure`,
//! `repeat_first_field`, and `top_field_first` per the §6.3.12 body
//! pseudocode:
//!
//! ```text
//! if (progressive_sequence == 1) {
//!     if (repeat_first_field == '1') {
//!         if (top_field_first == '1')
//!             number_of_frame_centre_offsets = 3
//!         else
//!             number_of_frame_centre_offsets = 2
//!     } else {
//!         number_of_frame_centre_offsets = 1
//!     }
//! } else {
//!     if (picture_structure == "field") {
//!         number_of_frame_centre_offsets = 1
//!     } else {
//!         if (repeat_first_field == '1')
//!             number_of_frame_centre_offsets = 3
//!         else
//!             number_of_frame_centre_offsets = 2
//!     }
//! }
//! ```
//!
//! That derivation is exposed as
//! [`number_of_frame_centre_offsets`] so a caller can compute the
//! value once from already-parsed picture-layer state and hand it to
//! [`PictureDisplayExtension::parse`].
//!
//! ## Marker bits (§6.2.3.3)
//!
//! Per §5.2.1 a `marker_bit` shall be `'1'`. The parser rejects a `0`
//! as `InvalidBitstream` to keep parity with every other §6.2.x
//! marker-bit site in this crate.
//!
//! ## State carry-over (§6.3.12)
//!
//! §6.3.12 also defines two state-carry rules:
//!
//! * *"In the case that a given picture does not have a
//!   `picture_display_extension()` then the most recently decoded
//!   frame centre offset shall be used."*
//! * *"Following a `sequence_header()` the value zero shall be used
//!   for all frame centre offsets until a `picture_display_extension()`
//!   defines non-zero values."*
//!
//! Both rules are exposed by [`FrameCentreOffsetState`]. The shared
//! current pair (a single `(horizontal, vertical)`) is reset to
//! `(0, 0)` on construction (equivalent to the post-`sequence_header()`
//! reset) and updated by [`FrameCentreOffsetState::apply`] whenever a
//! parsed [`PictureDisplayExtension`] arrives. §6.3.12's NOTE clarifies
//! that even when the extension carries two or three offset pairs, the
//! "missing offsets" between pictures all have the same value, so a
//! single carried pair is the correct model for the *between-picture*
//! state.
//!
//! ## Order constraint
//!
//! §6.3.12 requires that *"a `picture_display_extension()` shall not
//! occur unless a `sequence_display_extension()` followed the previous
//! `sequence_header()`"*. That ordering constraint binds two
//! sequence-layer elements that this parser does not yet handle, so
//! enforcement is left to the sequence-layer driver. [`FieldUsage`]
//! captures the §6.3.12 application context so a later driver can
//! cross-check the constraint without re-reading the spec.
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)).

use oxideav_core::bits::BitReader;

use crate::picture_header::PictureStructure;
use crate::sequence_extension::EXTENSION_START_CODE;
use crate::{Error, Result};

/// `extension_start_code_identifier` value for
/// `picture_display_extension()` (Table 6-2 entry `0111`).
pub const PICTURE_DISPLAY_EXTENSION_ID: u32 = 0b0111;

/// Maximum number of frame-centre offset pairs §6.3.12 may produce
/// (three pairs, the `progressive_sequence == 1 &&
/// repeat_first_field == 1 && top_field_first == 1` branch and the
/// `progressive_sequence == 0 && picture_structure == frame &&
/// repeat_first_field == 1` branch both reach `3`).
pub const MAX_FRAME_CENTRE_OFFSETS: usize = 3;

/// One `(frame_centre_horizontal_offset, frame_centre_vertical_offset)`
/// pair per the §6.2.3.3 loop body.
///
/// Both components are 16-bit signed (`simsbf`) values measured in
/// units of 1/16th sample per §6.3.12.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FrameCentreOffset {
    /// `frame_centre_horizontal_offset` — positive ⇒ frame centre is
    /// to the right of the display-rectangle centre.
    pub horizontal: i32,
    /// `frame_centre_vertical_offset` — positive ⇒ frame centre is
    /// below the display-rectangle centre. NOTE 2 of §6.3.12: the
    /// vertical offset is in 1/16ths of a *frame* line even for a
    /// field picture.
    pub vertical: i32,
}

/// Parsed `picture_display_extension()` (§6.2.3.3).
///
/// The `offsets` array holds the `count` decoded pairs in transmission
/// order; entries beyond `count` are zeroed-out placeholders. Storing
/// the fixed-capacity array (rather than a `Vec`) keeps the struct
/// `Copy` and suits the §6.3.12 cap of three.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PictureDisplayExtension {
    /// Number of valid offset pairs in [`Self::offsets`] (1, 2, or 3).
    pub count: u8,
    /// Decoded `(horizontal, vertical)` pairs in transmission order.
    pub offsets: [FrameCentreOffset; MAX_FRAME_CENTRE_OFFSETS],
}

impl PictureDisplayExtension {
    /// Slice accessor returning only the populated `count` entries.
    pub fn offsets(&self) -> &[FrameCentreOffset] {
        &self.offsets[..self.count as usize]
    }
}

impl Default for PictureDisplayExtension {
    fn default() -> Self {
        Self {
            count: 0,
            offsets: [FrameCentreOffset::default(); MAX_FRAME_CENTRE_OFFSETS],
        }
    }
}

/// The four picture-layer flags §6.3.12 needs to derive
/// `number_of_frame_centre_offsets`.
///
/// All four bits originate elsewhere:
///
/// * `progressive_sequence` is on
///   [`crate::sequence_extension::Mpeg2SequenceExtension`].
/// * `picture_structure`, `repeat_first_field`, and `top_field_first`
///   are on [`crate::picture_header::PictureCodingExtension`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PictureDisplayContext {
    /// `progressive_sequence` from the active
    /// `sequence_extension()`.
    pub progressive_sequence: bool,
    /// `picture_structure` from the active
    /// `picture_coding_extension()`.
    pub picture_structure: PictureStructure,
    /// `repeat_first_field` from the active
    /// `picture_coding_extension()`.
    pub repeat_first_field: bool,
    /// `top_field_first` from the active
    /// `picture_coding_extension()`.
    pub top_field_first: bool,
}

/// Compute `number_of_frame_centre_offsets` per the §6.3.12
/// pseudocode.
///
/// Returns 1, 2, or 3.
pub fn number_of_frame_centre_offsets(ctx: PictureDisplayContext) -> u8 {
    if ctx.progressive_sequence {
        if ctx.repeat_first_field {
            if ctx.top_field_first {
                3
            } else {
                2
            }
        } else {
            1
        }
    } else if matches!(
        ctx.picture_structure,
        PictureStructure::TopField | PictureStructure::BottomField
    ) {
        1
    } else if ctx.repeat_first_field {
        3
    } else {
        2
    }
}

impl PictureDisplayExtension {
    /// Parse a `picture_display_extension()` from a slice starting
    /// with the four start-code bytes `00 00 01 B5`. The trailing
    /// `next_start_code()` (§5.2.3) byte-align is not consumed.
    ///
    /// `ctx` carries the four picture-layer flags §6.3.12 needs to
    /// derive `number_of_frame_centre_offsets`.
    pub fn parse(buf: &[u8], ctx: PictureDisplayContext) -> Result<Self> {
        let mut br = BitReader::new(buf);
        Self::parse_with_reader(&mut br, ctx)
    }

    /// Parse from an existing [`BitReader`] positioned at the start
    /// of `picture_display_extension()` (i.e. its 32-bit
    /// `extension_start_code`).
    pub fn parse_with_reader(br: &mut BitReader<'_>, ctx: PictureDisplayContext) -> Result<Self> {
        // §6.2.3.3 / §6.3.4: extension_start_code = 0x000001B5.
        let code = br.read_u32(32).map_err(|_| Error::ShortHeader)?;
        if code != EXTENSION_START_CODE {
            return Err(Error::InvalidBitstream(
                "extension_start_code: expected 0x000001B5 (§6.3.4)",
            ));
        }
        // 4-bit extension_start_code_identifier; Picture Display
        // Extension ID is '0111' per Table 6-2.
        let id = br.read_u32(4).map_err(|_| Error::ShortHeader)?;
        if id != PICTURE_DISPLAY_EXTENSION_ID {
            return Err(Error::InvalidBitstream(
                "extension_start_code_identifier: expected '0111' Picture Display Extension ID (Table 6-2)",
            ));
        }

        let count = number_of_frame_centre_offsets(ctx);
        debug_assert!((1..=MAX_FRAME_CENTRE_OFFSETS as u8).contains(&count));

        let mut offsets = [FrameCentreOffset::default(); MAX_FRAME_CENTRE_OFFSETS];
        for slot in offsets.iter_mut().take(count as usize) {
            let horizontal = br.read_i32(16).map_err(|_| Error::ShortHeader)?;
            expect_marker_bit(
                br,
                "marker_bit after frame_centre_horizontal_offset (§6.2.3.3)",
            )?;
            let vertical = br.read_i32(16).map_err(|_| Error::ShortHeader)?;
            expect_marker_bit(
                br,
                "marker_bit after frame_centre_vertical_offset (§6.2.3.3)",
            )?;
            *slot = FrameCentreOffset {
                horizontal,
                vertical,
            };
        }

        // §6.2.3.3 / §6.3.4: the 32-bit start code + 4-bit identifier
        // = 36 bits; each loop iteration is 16 + 1 + 16 + 1 = 34
        // bits. The post-loop bit total is 36 + 34 * count, which for
        // counts 1 / 2 / 3 reaches 70 / 104 / 138 bits — none of
        // which are byte-aligned. The trailing `next_start_code()`
        // (§5.2.3) supplies the zero stuffing that gets the cursor
        // back to a byte boundary; we therefore do NOT assert
        // byte-alignment here.
        let _ = br;
        Ok(Self { count, offsets })
    }
}

fn expect_marker_bit(br: &mut BitReader<'_>, msg: &'static str) -> Result<()> {
    let bit = br.read_u32(1).map_err(|_| Error::ShortHeader)?;
    if bit == 1 {
        Ok(())
    } else {
        Err(Error::InvalidBitstream(msg))
    }
}

/// Application context for [`FrameCentreOffsetState`].
///
/// §6.3.12 implicitly distinguishes the "between pictures" case (where
/// the most recently decoded offset is reused) from the per-picture
/// case (where up to three offsets are surfaced for this picture only).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FieldUsage {
    /// The carried pair is the most recently decoded offset, applied
    /// to every picture that does not present its own
    /// `picture_display_extension()`.
    BetweenPictures,
    /// The pair is the explicit per-picture extension; up to three
    /// offsets in [`PictureDisplayExtension::offsets`] are valid.
    PerPicture,
}

/// Per-sequence frame-centre-offset state for §6.3.12 carry-over.
///
/// §6.3.12 imposes two rules:
///
/// 1. *"Following a `sequence_header()` the value zero shall be used
///    for all frame centre offsets until a
///    `picture_display_extension()` defines non-zero values."*
/// 2. *"In the case that a given picture does not have a
///    `picture_display_extension()` then the most recently decoded
///    frame centre offset shall be used."*
///
/// The §6.3.12 NOTE further clarifies that *"each of the missing
/// frame centre offsets have the same value (even if two or three
/// frame centre offsets would have been contained in the
/// `picture_display_extension()` had been present)"*, so the carried
/// state is a single `FrameCentreOffset`.
///
/// [`Self::reset_to_zero`] performs the §6.3.12 post-`sequence_header()`
/// reset; [`Self::apply`] absorbs a parsed
/// [`PictureDisplayExtension`], adopting its first offset pair as the
/// new "most recently decoded" value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FrameCentreOffsetState {
    /// Most recently decoded `(horizontal, vertical)` frame-centre
    /// offset. Reset to `(0, 0)` on `sequence_header()` (§6.3.12).
    pub current: FrameCentreOffset,
}

impl FrameCentreOffsetState {
    /// Construct a fresh state at the §6.3.12 post-`sequence_header()`
    /// zero baseline.
    pub const fn zero() -> Self {
        Self {
            current: FrameCentreOffset {
                horizontal: 0,
                vertical: 0,
            },
        }
    }

    /// §6.3.12: *"Following a `sequence_header()` the value zero
    /// shall be used for all frame centre offsets until a
    /// `picture_display_extension()` defines non-zero values."*
    pub fn reset_to_zero(&mut self) {
        *self = Self::zero();
    }

    /// Absorb a parsed [`PictureDisplayExtension`]: the first offset
    /// in transmission order becomes the new
    /// "most recently decoded frame centre offset" §6.3.12 quotes.
    /// Subsequent pictures that omit the extension reuse this single
    /// pair per the §6.3.12 NOTE on missing offsets.
    pub fn apply(&mut self, ext: &PictureDisplayExtension) {
        if ext.count > 0 {
            self.current = ext.offsets[0];
        }
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact `picture_display_extension()` fixtures
    //! plus negative cases for every §6.2.3.3 / §6.3.12 rejection
    //! site this parser introduces.
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Build the four-flag picture context for the §6.3.12 derivation.
    fn ctx(
        progressive_sequence: bool,
        picture_structure: PictureStructure,
        repeat_first_field: bool,
        top_field_first: bool,
    ) -> PictureDisplayContext {
        PictureDisplayContext {
            progressive_sequence,
            picture_structure,
            repeat_first_field,
            top_field_first,
        }
    }

    /// Emit a `picture_display_extension()` with the four loop
    /// iterations dictated by `offsets.len()`. Caller is responsible
    /// for matching the count to the §6.3.12 derivation.
    fn write_picture_display_extension(bw: &mut BitWriter, offsets: &[(i32, i32)]) {
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(PICTURE_DISPLAY_EXTENSION_ID, 4);
        for &(h, v) in offsets {
            bw.write_i32(h, 16);
            bw.write_bit(true);
            bw.write_i32(v, 16);
            bw.write_bit(true);
        }
        bw.align_to_byte();
    }

    // ---- §6.3.12 derivation ---------------------------------------

    #[test]
    fn derivation_progressive_no_rff_is_one() {
        // progressive_sequence == 1 && repeat_first_field == 0 -> 1.
        let c = ctx(true, PictureStructure::Frame, false, false);
        assert_eq!(number_of_frame_centre_offsets(c), 1);
    }

    #[test]
    fn derivation_progressive_rff_tff_is_three() {
        let c = ctx(true, PictureStructure::Frame, true, true);
        assert_eq!(number_of_frame_centre_offsets(c), 3);
    }

    #[test]
    fn derivation_progressive_rff_no_tff_is_two() {
        let c = ctx(true, PictureStructure::Frame, true, false);
        assert_eq!(number_of_frame_centre_offsets(c), 2);
    }

    #[test]
    fn derivation_interlaced_field_is_one() {
        let c = ctx(false, PictureStructure::TopField, true, true);
        assert_eq!(number_of_frame_centre_offsets(c), 1);
        let c = ctx(false, PictureStructure::BottomField, false, false);
        assert_eq!(number_of_frame_centre_offsets(c), 1);
    }

    #[test]
    fn derivation_interlaced_frame_rff_is_three() {
        let c = ctx(false, PictureStructure::Frame, true, false);
        assert_eq!(number_of_frame_centre_offsets(c), 3);
    }

    #[test]
    fn derivation_interlaced_frame_no_rff_is_two() {
        let c = ctx(false, PictureStructure::Frame, false, true);
        assert_eq!(number_of_frame_centre_offsets(c), 2);
    }

    // ---- Wire parses ----------------------------------------------

    #[test]
    fn parses_single_offset_pair() {
        let pairs = [(123, -456)];
        let mut bw = BitWriter::new();
        write_picture_display_extension(&mut bw, &pairs);
        let bytes = bw.finish();
        let c = ctx(true, PictureStructure::Frame, false, false);
        let ext = PictureDisplayExtension::parse(&bytes, c).expect("parse");
        assert_eq!(ext.count, 1);
        assert_eq!(ext.offsets()[0].horizontal, 123);
        assert_eq!(ext.offsets()[0].vertical, -456);
    }

    #[test]
    fn parses_two_offset_pairs() {
        let pairs = [(10, 20), (-30, -40)];
        let mut bw = BitWriter::new();
        write_picture_display_extension(&mut bw, &pairs);
        let bytes = bw.finish();
        let c = ctx(true, PictureStructure::Frame, true, false);
        let ext = PictureDisplayExtension::parse(&bytes, c).expect("parse");
        assert_eq!(ext.count, 2);
        assert_eq!(
            ext.offsets()[0],
            FrameCentreOffset {
                horizontal: 10,
                vertical: 20
            }
        );
        assert_eq!(
            ext.offsets()[1],
            FrameCentreOffset {
                horizontal: -30,
                vertical: -40,
            }
        );
    }

    #[test]
    fn parses_three_offset_pairs() {
        let pairs = [(1, 2), (3, 4), (5, 6)];
        let mut bw = BitWriter::new();
        write_picture_display_extension(&mut bw, &pairs);
        let bytes = bw.finish();
        let c = ctx(true, PictureStructure::Frame, true, true);
        let ext = PictureDisplayExtension::parse(&bytes, c).expect("parse");
        assert_eq!(ext.count, 3);
        for (i, &(h, v)) in pairs.iter().enumerate() {
            assert_eq!(
                ext.offsets()[i],
                FrameCentreOffset {
                    horizontal: h,
                    vertical: v,
                }
            );
        }
    }

    #[test]
    fn parses_extreme_signed_offsets() {
        // Boundary values for 16-bit signed integers.
        let pairs = [(i16::MIN as i32, i16::MAX as i32)];
        let mut bw = BitWriter::new();
        write_picture_display_extension(&mut bw, &pairs);
        let bytes = bw.finish();
        let c = ctx(true, PictureStructure::Frame, false, false);
        let ext = PictureDisplayExtension::parse(&bytes, c).expect("parse");
        assert_eq!(ext.offsets()[0].horizontal, i16::MIN as i32);
        assert_eq!(ext.offsets()[0].vertical, i16::MAX as i32);
    }

    // ---- Rejection sites -----------------------------------------

    #[test]
    fn rejects_wrong_extension_start_code() {
        let pairs = [(0, 0)];
        let mut bw = BitWriter::new();
        write_picture_display_extension(&mut bw, &pairs);
        let mut bytes = bw.finish();
        bytes[3] = 0xB3; // sequence_header_code instead
        let c = ctx(true, PictureStructure::Frame, false, false);
        let err = PictureDisplayExtension::parse(&bytes, c).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_wrong_extension_identifier() {
        // Build manually with the identifier set to '0001'
        // (Sequence Extension ID) rather than '0111'.
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(0b0001, 4);
        bw.write_i32(0, 16);
        bw.write_bit(true);
        bw.write_i32(0, 16);
        bw.write_bit(true);
        bw.align_to_byte();
        let bytes = bw.finish();
        let c = ctx(true, PictureStructure::Frame, false, false);
        let err = PictureDisplayExtension::parse(&bytes, c).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_zero_marker_bit_after_horizontal() {
        // Build manually with the first marker_bit forced to '0'.
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(PICTURE_DISPLAY_EXTENSION_ID, 4);
        bw.write_i32(0, 16);
        bw.write_bit(false); // forbidden: marker_bit must be '1'
        bw.write_i32(0, 16);
        bw.write_bit(true);
        bw.align_to_byte();
        let bytes = bw.finish();
        let c = ctx(true, PictureStructure::Frame, false, false);
        let err = PictureDisplayExtension::parse(&bytes, c).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_zero_marker_bit_after_vertical() {
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(PICTURE_DISPLAY_EXTENSION_ID, 4);
        bw.write_i32(0, 16);
        bw.write_bit(true);
        bw.write_i32(0, 16);
        bw.write_bit(false); // forbidden: marker_bit must be '1'
        bw.align_to_byte();
        let bytes = bw.finish();
        let c = ctx(true, PictureStructure::Frame, false, false);
        let err = PictureDisplayExtension::parse(&bytes, c).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn short_buffer_returns_short_header() {
        let bytes = [0u8, 0u8];
        let c = ctx(true, PictureStructure::Frame, false, false);
        let err = PictureDisplayExtension::parse(&bytes, c).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    // ---- State carry-over ----------------------------------------

    #[test]
    fn state_starts_at_zero() {
        let s = FrameCentreOffsetState::zero();
        assert_eq!(s.current, FrameCentreOffset::default());
    }

    #[test]
    fn state_apply_adopts_first_offset() {
        let pairs = [(10, 20), (30, 40)];
        let mut bw = BitWriter::new();
        write_picture_display_extension(&mut bw, &pairs);
        let bytes = bw.finish();
        let c = ctx(true, PictureStructure::Frame, true, false);
        let ext = PictureDisplayExtension::parse(&bytes, c).expect("parse");
        let mut state = FrameCentreOffsetState::zero();
        state.apply(&ext);
        assert_eq!(
            state.current,
            FrameCentreOffset {
                horizontal: 10,
                vertical: 20
            }
        );
    }

    #[test]
    fn state_reset_clears_to_zero() {
        let mut state = FrameCentreOffsetState::zero();
        state.current.horizontal = 99;
        state.current.vertical = -99;
        state.reset_to_zero();
        assert_eq!(state, FrameCentreOffsetState::zero());
    }

    #[test]
    fn between_pictures_field_usage_round_trips() {
        // FieldUsage is an enum-bag; cover both arms in equality
        // checks so the discriminants are pinned.
        assert_eq!(FieldUsage::BetweenPictures, FieldUsage::BetweenPictures);
        assert_eq!(FieldUsage::PerPicture, FieldUsage::PerPicture);
        assert_ne!(FieldUsage::BetweenPictures, FieldUsage::PerPicture);
    }

    #[test]
    fn parses_first_byte_after_extension_for_three_offsets() {
        // 36 + 34 * 3 = 138 bits = 17 bytes + 2 bits. The writer
        // pads to byte boundary, so the resulting buffer is 18 bytes.
        let pairs = [(0, 0), (0, 0), (0, 0)];
        let mut bw = BitWriter::new();
        write_picture_display_extension(&mut bw, &pairs);
        let bytes = bw.finish();
        assert_eq!(bytes.len(), 18);
    }

    #[test]
    fn parses_first_byte_after_extension_for_two_offsets() {
        // 36 + 34 * 2 = 104 bits = 13 bytes. Already byte-aligned.
        let pairs = [(0, 0), (0, 0)];
        let mut bw = BitWriter::new();
        write_picture_display_extension(&mut bw, &pairs);
        let bytes = bw.finish();
        assert_eq!(bytes.len(), 13);
    }

    #[test]
    fn parses_first_byte_after_extension_for_one_offset() {
        // 36 + 34 = 70 bits = 8 bytes + 6 bits. The writer pads to
        // byte boundary, so the resulting buffer is 9 bytes.
        let pairs = [(0, 0)];
        let mut bw = BitWriter::new();
        write_picture_display_extension(&mut bw, &pairs);
        let bytes = bw.finish();
        assert_eq!(bytes.len(), 9);
    }
}
