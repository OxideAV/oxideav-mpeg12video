//! §6.2.2.2 `extension_and_user_data(i)` dispatcher, §6.2.2.2.1
//! `extension_data(i)`, and §6.2.2.2.2 `user_data()` (ISO/IEC
//! 13818-2 | ITU-T H.262).
//!
//! The §6.2.2 `video_sequence()` syntax invokes
//! `extension_and_user_data(i)` at three points:
//!
//! * `i = 0` — immediately after `sequence_extension()`,
//! * `i = 1` — immediately after `group_of_pictures_header()`,
//! * `i = 2` — immediately after `picture_coding_extension()`.
//!
//! The element is a loop over byte-aligned start codes:
//!
//! ```text
//! extension_and_user_data( i ) {
//!     while ( ( nextbits() == extension_start_code ) ||
//!             ( nextbits() == user_data_start_code ) ) {
//!         if ( ( i != 1 ) && ( nextbits() == extension_start_code ) )
//!             extension_data( i )
//!         if ( nextbits() == user_data_start_code )
//!             user_data()
//!     }
//! }
//! ```
//!
//! `extension_data(i)` then dispatches on the 4-bit
//! `extension_start_code_identifier` (Table 6-2) that follows each
//! `extension_start_code`: at `i == 0` the allowed set is
//! `sequence_display_extension()` / `sequence_scalable_extension()`;
//! at `i == 2` it is `quant_matrix_extension()` /
//! `copyright_extension()` / `picture_display_extension()` /
//! `picture_spatial_scalable_extension()` /
//! `picture_temporal_scalable_extension()`. The §6.2.2.2.1 NOTE
//! pins `i == 1`: *"i never takes the value 1 because
//! extension_data() never follows a group_of_pictures_header()"* —
//! an `extension_start_code` at that point is therefore a bitstream
//! that the pseudocode can never consume, rejected here as
//! [`Error::InvalidBitstream`].
//!
//! Two §6.3.1 semantic rules are enforced on top of the syntax:
//!
//! * *"At each point where extensions are allowed in the bitstream
//!   any number of the extensions from the defined allowable set may
//!   be included. However each type of extension shall not occur
//!   more than once."* — a repeated `extension_start_code_identifier`
//!   within one `extension_and_user_data(i)` is rejected.
//! * *"In the case that a decoder encounters an extension with an
//!   extension identification that is described as 'reserved' in
//!   this specification the decoder shall discard all subsequent
//!   data until the next start code."* — reserved identifiers
//!   (`0000`, `0110`, `1011`..`1111`) are skipped to the next
//!   start code and recorded in
//!   [`ExtensionAndUserData::discarded_reserved_ids`].
//!
//! An identifier that names a *defined* extension outside the
//! location's allowable set (e.g. a Sequence Display Extension ID at
//! `i = 2`) is rejected — §6.3.1 *"The set of allowed extensions is
//! different at each different point in the syntax"*. Every Table 6-2
//! extension this dispatcher can reach now has a parser, including the
//! two picture-layer scalable extensions
//! (`picture_spatial_scalable_extension()` /
//! `picture_temporal_scalable_extension()`).
//!
//! `user_data()` (§6.2.2.2.2 / §6.3.4.1) collects the 8-bit
//! `user_data` bytes that follow a `user_data_start_code`
//! (`0x000001B2`): *"The user data continues until receipt of
//! another start code"*, with the §6.3.4.1 constraint *"In the
//! series of consecutive user_data bytes there shall not be a
//! string of 23 or more consecutive zero bits"* (start-code
//! emulation guard) enforced as a rejection site.
//!
//! Between elements the §5.2.3 `next_start_code()` discipline holds:
//! start codes are byte aligned and may be preceded only by zero
//! stuffing bits / zero stuffing bytes; a non-zero byte that is not
//! part of a `00 00 01` prefix is rejected.
//!
//! The `i = 0` result's [`ExtensionAndUserData::sequence_display_extension`]
//! is exactly the `Option<SequenceDisplayExtension>` the r271
//! [`crate::SequenceDisplayOrderDriver::on_sequence_header_window`]
//! consumes, so a sequence-layer driver can feed the §6.3.5 /
//! §6.3.12 ordering checks straight from this dispatcher.

use oxideav_core::bits::BitReader;

use crate::copyright_extension::{CopyrightExtension, COPYRIGHT_EXTENSION_ID};
use crate::picture_display_extension::{
    PictureDisplayContext, PictureDisplayExtension, PICTURE_DISPLAY_EXTENSION_ID,
};
use crate::picture_spatial_scalable_extension::PictureSpatialScalableExtension;
use crate::picture_temporal_scalable_extension::PictureTemporalScalableExtension;
use crate::quant_matrix_extension::{QuantMatrixExtension, QUANT_MATRIX_EXTENSION_ID};
use crate::sequence_display_extension::{SequenceDisplayExtension, SEQUENCE_DISPLAY_EXTENSION_ID};
use crate::sequence_extension::{ChromaFormat, EXTENSION_START_CODE};
use crate::sequence_scalable_extension::{
    SequenceScalableExtension, SEQUENCE_SCALABLE_EXTENSION_ID,
};
use crate::{Error, Result};

/// 32-bit `user_data_start_code`, byte string `00 00 01 B2`
/// (§6.3.4.1 / Table 6-1).
pub const USER_DATA_START_CODE: u32 = 0x0000_01B2;

pub use crate::picture_spatial_scalable_extension::PICTURE_SPATIAL_SCALABLE_EXTENSION_ID;
pub use crate::picture_temporal_scalable_extension::PICTURE_TEMPORAL_SCALABLE_EXTENSION_ID;

/// The `i` argument of `extension_and_user_data(i)` — which of the
/// three §6.2.2 invocation points this dispatch services. The
/// `i = 2` arm carries the two pieces of picture-layer state its
/// allowable extensions need to parse:
/// `quant_matrix_extension()` needs the active `chroma_format`
/// (§6.3.11 `4:2:0 ⇒ load_chroma_* == '0'`), and
/// `picture_display_extension()` needs the §6.3.12
/// [`PictureDisplayContext`] to derive
/// `number_of_frame_centre_offsets`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExtensionLocation {
    /// `i = 0` — follows `sequence_extension()`. Allowable
    /// extensions: Sequence Display Extension ID (`0010`),
    /// Sequence Scalable Extension ID (`0101`).
    AfterSequenceExtension,
    /// `i = 1` — follows `group_of_pictures_header()`. Only
    /// `user_data()` may occur (§6.2.2.2.1 NOTE).
    AfterGroupOfPicturesHeader,
    /// `i = 2` — follows `picture_coding_extension()`. Allowable
    /// extensions: Quant Matrix (`0011`), Copyright (`0100`),
    /// Picture Display (`0111`), Picture Spatial Scalable (`1001`),
    /// Picture Temporal Scalable (`1010`).
    AfterPictureCodingExtension {
        /// `chroma_format` from the active `sequence_extension()`
        /// (Table 6-5).
        chroma_format: ChromaFormat,
        /// The §6.3.12 picture-layer flag bundle.
        picture_display: PictureDisplayContext,
    },
}

/// One `user_data()` element (§6.2.2.2.2).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct UserData {
    /// The raw `user_data` bytes — *"an 8 bit integer, an arbitrary
    /// number of which may follow one another. User data is defined
    /// by users for their specific applications"* (§6.3.4.1). May be
    /// empty (a `user_data_start_code` immediately followed by
    /// another start code).
    pub bytes: Vec<u8>,
}

/// Everything one `extension_and_user_data(i)` invocation parsed.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ExtensionAndUserData {
    /// `sequence_display_extension()` when present (`i = 0` only).
    /// Feed this straight into
    /// [`crate::SequenceDisplayOrderDriver::on_sequence_header_window`].
    pub sequence_display_extension: Option<SequenceDisplayExtension>,
    /// `sequence_scalable_extension()` when present (`i = 0` only).
    pub sequence_scalable_extension: Option<SequenceScalableExtension>,
    /// `quant_matrix_extension()` when present (`i = 2` only).
    pub quant_matrix_extension: Option<QuantMatrixExtension>,
    /// `copyright_extension()` when present (`i = 2` only).
    pub copyright_extension: Option<CopyrightExtension>,
    /// `picture_display_extension()` when present (`i = 2` only).
    pub picture_display_extension: Option<PictureDisplayExtension>,
    /// `picture_spatial_scalable_extension()` when present (`i = 2`
    /// only — legal only in a spatially-scalable sequence).
    pub picture_spatial_scalable_extension: Option<PictureSpatialScalableExtension>,
    /// `picture_temporal_scalable_extension()` when present (`i = 2`
    /// only — legal only in a temporally-scalable sequence).
    pub picture_temporal_scalable_extension: Option<PictureTemporalScalableExtension>,
    /// Every `user_data()` element, in bitstream order. `user_data()`
    /// is not an extension, so the §6.3.1 at-most-once rule does not
    /// bound it.
    pub user_data: Vec<UserData>,
    /// The Table 6-2 *reserved* identifiers whose extensions were
    /// discarded up to the next start code per §6.3.1.
    pub discarded_reserved_ids: Vec<u8>,
    /// Byte offset (into the parsed slice) of the first byte of the
    /// `00 00 01` prefix of the start code that terminated the
    /// `while` loop — i.e. where the caller resumes the §6.2.2
    /// `video_sequence()` walk. Equals the slice length when the
    /// buffer ended in (possibly zero bytes of) zero stuffing.
    pub byte_position_after: usize,
}

/// Locate the next byte-aligned start code at or after `pos`,
/// enforcing the §5.2.3 `next_start_code()` discipline: every byte
/// before the `00 00 01` prefix must be a zero stuffing byte.
///
/// Returns `Ok(Some(prefix_pos))` with `buf[prefix_pos..]` starting
/// `00 00 01`, or `Ok(None)` when the remainder of the buffer is
/// all zero stuffing (no further start code).
fn locate_start_code(buf: &[u8], pos: usize) -> Result<Option<usize>> {
    let mut j = pos;
    while j < buf.len() && buf[j] == 0x00 {
        j += 1;
    }
    if j >= buf.len() {
        // Trailing zero stuffing (possibly empty) and no further
        // start code — the caller's window ends here.
        return Ok(None);
    }
    if buf[j] != 0x01 || j - pos < 2 {
        return Err(Error::InvalidBitstream(
            "next_start_code(): only zero stuffing may precede a start code (§5.2.3)",
        ));
    }
    Ok(Some(j - 2))
}

/// Scan for the next `00 00 01` prefix at or after `pos` with **no**
/// constraint on the intervening bytes. This is the §6.3.1
/// reserved-extension recovery: *"the decoder shall discard all
/// subsequent data until the next start code"* — the discarded
/// payload is arbitrary.
fn scan_start_code(buf: &[u8], pos: usize) -> Option<usize> {
    if buf.len() < 3 {
        return None;
    }
    (pos..buf.len() - 2).find(|&j| buf[j] == 0x00 && buf[j + 1] == 0x00 && buf[j + 2] == 0x01)
}

/// Longest run of consecutive zero **bits** in `bytes` viewed as a
/// contiguous bit string (MSB first), for the §6.3.4.1 constraint
/// *"there shall not be a string of 23 or more consecutive zero
/// bits"*.
fn longest_zero_bit_run(bytes: &[u8]) -> u32 {
    let mut longest = 0u32;
    let mut run = 0u32;
    for &b in bytes {
        for bit in (0..8).rev() {
            if b & (1 << bit) == 0 {
                run += 1;
                longest = longest.max(run);
            } else {
                run = 0;
            }
        }
    }
    longest
}

impl UserData {
    /// Parse one `user_data()` element (§6.2.2.2.2) from a slice
    /// whose first four bytes are the `user_data_start_code`
    /// `00 00 01 B2`. Returns the element plus the byte offset of
    /// the terminating start code's `00 00 01` prefix (*"The user
    /// data continues until receipt of another start code"*,
    /// §6.3.4.1) — the trailing `next_start_code()` is **not**
    /// consumed beyond locating that prefix.
    pub fn parse(buf: &[u8]) -> Result<(Self, usize)> {
        if buf.len() < 4 {
            return Err(Error::ShortHeader);
        }
        let code = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);
        if code != USER_DATA_START_CODE {
            return Err(Error::InvalidBitstream(
                "user_data_start_code: expected 0x000001B2 (§6.3.4.1)",
            ));
        }
        // §6.2.2.2.2: while ( nextbits() != '0000 0000 0000 0000
        // 0000 0001' ) user_data (8 bits). The 24-bit lookahead is
        // exactly the `00 00 01` start-code prefix.
        let mut end = 4usize;
        loop {
            if end + 3 > buf.len() {
                // The element is only terminated by "receipt of
                // another start code" (§6.3.4.1); running out of
                // buffer first is a truncation.
                return Err(Error::ShortHeader);
            }
            if buf[end] == 0x00 && buf[end + 1] == 0x00 && buf[end + 2] == 0x01 {
                break;
            }
            end += 1;
        }
        let bytes = buf[4..end].to_vec();
        // §6.3.4.1: "In the series of consecutive user_data bytes
        // there shall not be a string of 23 or more consecutive
        // zero bits" (start-code emulation guard).
        if longest_zero_bit_run(&bytes) >= 23 {
            return Err(Error::InvalidBitstream(
                "user_data: string of 23 or more consecutive zero bits (§6.3.4.1)",
            ));
        }
        Ok((Self { bytes }, end))
    }
}

impl ExtensionAndUserData {
    /// Parse one `extension_and_user_data(i)` invocation (§6.2.2.2)
    /// from a slice positioned at the byte boundary right after the
    /// preceding syntax element's last data byte (i.e. at the start
    /// of the §5.2.3 zero stuffing, if any). The loop consumes
    /// every `extension_data(i)` / `user_data()` element it finds
    /// and stops — without consuming anything — at the first start
    /// code that is neither `extension_start_code` nor
    /// `user_data_start_code`, reporting that position in
    /// [`Self::byte_position_after`].
    pub fn parse(buf: &[u8], location: ExtensionLocation) -> Result<Self> {
        let mut out = Self::default();
        // §6.3.1: "each type of extension shall not occur more than
        // once" at each point — one presence bit per Table 6-2
        // identifier.
        let mut seen_ids: u16 = 0;
        let mut pos = 0usize;

        loop {
            let Some(sc_pos) = locate_start_code(buf, pos)? else {
                out.byte_position_after = buf.len();
                return Ok(out);
            };
            if sc_pos + 4 > buf.len() {
                // `00 00 01` prefix with the start-code value byte
                // missing.
                return Err(Error::ShortHeader);
            }
            let code = u32::from_be_bytes([
                buf[sc_pos],
                buf[sc_pos + 1],
                buf[sc_pos + 2],
                buf[sc_pos + 3],
            ]);

            if code == USER_DATA_START_CODE {
                let (user_data, end) = UserData::parse(&buf[sc_pos..])?;
                out.user_data.push(user_data);
                pos = sc_pos + end;
                continue;
            }
            if code != EXTENSION_START_CODE {
                // §6.2.2.2 while-condition fails: the element is
                // over. The caller resumes at this start code.
                out.byte_position_after = sc_pos;
                return Ok(out);
            }

            // extension_start_code. §6.2.2.2.1 NOTE: "i never takes
            // the value 1 because extension_data() never follows a
            // group_of_pictures_header()" — the §6.2.2.2 loop body
            // has no arm that could consume it at i = 1.
            if location == ExtensionLocation::AfterGroupOfPicturesHeader {
                return Err(Error::InvalidBitstream(
                    "extension_data() shall not follow a group_of_pictures_header() (§6.2.2.2.1 NOTE)",
                ));
            }
            if sc_pos + 5 > buf.len() {
                return Err(Error::ShortHeader);
            }
            let id = u32::from(buf[sc_pos + 4] >> 4);
            if seen_ids & (1 << id) != 0 {
                return Err(Error::InvalidBitstream(
                    "each type of extension shall not occur more than once (§6.3.1)",
                ));
            }
            seen_ids |= 1 << id;

            // Table 6-2 reserved identifiers: 0000, 0110, 1011..1111.
            // §6.3.1: "the decoder shall discard all subsequent data
            // until the next start code".
            if matches!(id, 0b0000 | 0b0110 | 0b1011..=0b1111) {
                out.discarded_reserved_ids.push(id as u8);
                match scan_start_code(buf, sc_pos + 4) {
                    Some(next) => {
                        pos = next;
                        continue;
                    }
                    None => {
                        out.byte_position_after = buf.len();
                        return Ok(out);
                    }
                }
            }

            // Defined identifier: dispatch per the §6.2.2.2.1
            // location-dependent allowable set.
            let mut br = BitReader::with_position(buf, sc_pos);
            match location {
                ExtensionLocation::AfterSequenceExtension => match id {
                    SEQUENCE_DISPLAY_EXTENSION_ID => {
                        out.sequence_display_extension =
                            Some(SequenceDisplayExtension::parse_with_reader(&mut br)?);
                    }
                    SEQUENCE_SCALABLE_EXTENSION_ID => {
                        out.sequence_scalable_extension =
                            Some(SequenceScalableExtension::parse_with_reader(&mut br)?);
                    }
                    _ => {
                        return Err(Error::InvalidBitstream(
                            "extension not in the allowable set after sequence_extension() (§6.2.2.2.1 / §6.3.1)",
                        ));
                    }
                },
                ExtensionLocation::AfterPictureCodingExtension {
                    chroma_format,
                    picture_display,
                } => match id {
                    QUANT_MATRIX_EXTENSION_ID => {
                        out.quant_matrix_extension = Some(QuantMatrixExtension::parse_with_reader(
                            &mut br,
                            chroma_format,
                        )?);
                    }
                    PICTURE_DISPLAY_EXTENSION_ID => {
                        out.picture_display_extension = Some(
                            PictureDisplayExtension::parse_with_reader(&mut br, picture_display)?,
                        );
                    }
                    COPYRIGHT_EXTENSION_ID => {
                        out.copyright_extension =
                            Some(CopyrightExtension::parse_with_reader(&mut br)?);
                    }
                    PICTURE_SPATIAL_SCALABLE_EXTENSION_ID => {
                        out.picture_spatial_scalable_extension =
                            Some(PictureSpatialScalableExtension::parse_with_reader(&mut br)?);
                    }
                    PICTURE_TEMPORAL_SCALABLE_EXTENSION_ID => {
                        out.picture_temporal_scalable_extension = Some(
                            PictureTemporalScalableExtension::parse_with_reader(&mut br)?,
                        );
                    }
                    _ => {
                        return Err(Error::InvalidBitstream(
                            "extension not in the allowable set after picture_coding_extension() (§6.2.2.2.1 / §6.3.1)",
                        ));
                    }
                },
                ExtensionLocation::AfterGroupOfPicturesHeader => unreachable!("rejected above"),
            }

            // The extension parsers stop after their last syntax
            // field, mid-byte when the bit count is not a multiple
            // of eight. §5.2.3: the remaining bits of that byte are
            // zero stuffing bits ("while ( !bytealigned() )
            // zero_bit '0'").
            let bit_pos = br.bit_position();
            let trailing_bits = (bit_pos % 8) as u32;
            let mut next_pos = (bit_pos / 8) as usize;
            if trailing_bits != 0 {
                let mask = 0xffu8 >> trailing_bits;
                if buf[next_pos] & mask != 0 {
                    return Err(Error::InvalidBitstream(
                        "next_start_code(): non-zero stuffing bits after extension (§5.2.3)",
                    ));
                }
                next_pos += 1;
            }
            pos = next_pos;
        }
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact fixtures for the §6.2.2.2 dispatcher,
    //! §6.2.2.2.2 `user_data()`, and every §6.3.1 / §6.3.4.1 /
    //! §5.2.3 rejection site this module introduces.
    use super::*;
    use crate::picture_header::PictureStructure;
    use oxideav_core::bits::BitWriter;

    fn frame_picture_ctx() -> PictureDisplayContext {
        PictureDisplayContext {
            progressive_sequence: false,
            picture_structure: PictureStructure::Frame,
            repeat_first_field: false,
            top_field_first: false,
        }
    }

    fn i2_location() -> ExtensionLocation {
        ExtensionLocation::AfterPictureCodingExtension {
            chroma_format: ChromaFormat::Yuv420,
            picture_display: frame_picture_ctx(),
        }
    }

    /// Minimal `sequence_display_extension()`: no colour
    /// description, 720×576 display size. 69 bits → 9 bytes with
    /// §5.2.3 zero stuffing.
    fn write_sequence_display_extension(bw: &mut BitWriter) {
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(SEQUENCE_DISPLAY_EXTENSION_ID, 4);
        bw.write_u32(0b001, 3); // video_format = PAL
        bw.write_u32(0, 1); // colour_description = 0
        bw.write_u32(720, 14); // display_horizontal_size
        bw.write_u32(1, 1); // marker_bit
        bw.write_u32(576, 14); // display_vertical_size
        bw.align_to_byte();
    }

    /// Minimal `quant_matrix_extension()`: all four load flags
    /// clear. 40 bits → 5 bytes.
    fn write_quant_matrix_extension(bw: &mut BitWriter) {
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(QUANT_MATRIX_EXTENSION_ID, 4);
        bw.write_u32(0, 4); // four load flags, all '0'
    }

    /// `picture_display_extension()` carrying the given offsets.
    /// With [`frame_picture_ctx`] (interlaced frame picture,
    /// `repeat_first_field = 0`) the §6.3.12 derivation expects
    /// exactly two `(horizontal, vertical)` pairs.
    fn write_picture_display_extension(bw: &mut BitWriter, offsets: &[(i32, i32)]) {
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(PICTURE_DISPLAY_EXTENSION_ID, 4);
        for &(h, v) in offsets {
            bw.write_i32(h, 16);
            bw.write_u32(1, 1); // marker_bit
            bw.write_i32(v, 16);
            bw.write_u32(1, 1); // marker_bit
        }
        bw.align_to_byte();
    }

    fn write_user_data(bw: &mut BitWriter, bytes: &[u8]) {
        bw.write_u32(USER_DATA_START_CODE, 32);
        bw.write_bytes(bytes);
    }

    /// `picture_start_code` — a start code outside the §6.2.2.2
    /// while-condition, terminating the element.
    fn write_picture_start_code_prefix(bw: &mut BitWriter) {
        bw.write_u32(0x0000_0100, 32);
    }

    // ---- user_data() unit surface ----

    #[test]
    fn user_data_collects_bytes_until_next_start_code() {
        let mut bw = BitWriter::new();
        write_user_data(&mut bw, b"OxideAV");
        write_picture_start_code_prefix(&mut bw);
        let buf = bw.into_bytes();
        let (ud, end) = UserData::parse(&buf).expect("user_data");
        assert_eq!(ud.bytes, b"OxideAV");
        assert_eq!(end, 4 + 7);
    }

    #[test]
    fn user_data_may_be_empty() {
        let mut bw = BitWriter::new();
        write_user_data(&mut bw, &[]);
        write_picture_start_code_prefix(&mut bw);
        let (ud, end) = UserData::parse(&bw.into_bytes()).expect("empty user_data");
        assert!(ud.bytes.is_empty());
        assert_eq!(end, 4);
    }

    #[test]
    fn user_data_rejects_wrong_start_code() {
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        write_picture_start_code_prefix(&mut bw);
        assert!(matches!(
            UserData::parse(&bw.into_bytes()),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn user_data_rejects_truncation_without_terminating_start_code() {
        // §6.3.4.1: "The user data continues until receipt of
        // another start code" — EOF first is a truncation.
        let mut bw = BitWriter::new();
        write_user_data(&mut bw, b"abc");
        assert_eq!(UserData::parse(&bw.into_bytes()), Err(Error::ShortHeader));
    }

    #[test]
    fn user_data_rejects_23_consecutive_zero_bits() {
        // 0x80 00 00 FF: bit 1, then exactly 23 zeros, then 8 ones —
        // the §6.3.4.1 forbidden run at its lower bound, without
        // emulating a start code (the 24-bit lookahead never sees
        // 0x000001 inside the byte series).
        let mut bw = BitWriter::new();
        write_user_data(&mut bw, &[0x80, 0x00, 0x00, 0xff]);
        write_picture_start_code_prefix(&mut bw);
        assert!(matches!(
            UserData::parse(&bw.into_bytes()),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn user_data_accepts_22_consecutive_zero_bits() {
        // 0x80 00 01: 1, then 7 + 8 + 7 = 22 zeros, then a final 1 —
        // the longest legal zero run.
        let mut bw = BitWriter::new();
        write_user_data(&mut bw, &[0x80, 0x00, 0x01]);
        write_picture_start_code_prefix(&mut bw);
        let (ud, _) = UserData::parse(&bw.into_bytes()).expect("22-zero-bit run is legal");
        assert_eq!(ud.bytes, [0x80, 0x00, 0x01]);
    }

    #[test]
    fn longest_zero_bit_run_spans_byte_boundaries() {
        assert_eq!(longest_zero_bit_run(&[]), 0);
        assert_eq!(longest_zero_bit_run(&[0xff]), 0);
        assert_eq!(longest_zero_bit_run(&[0x01, 0x80]), 7);
        // 0x01 ends in '1' and 0x80 starts with '1', so the longest
        // run is exactly the 8 zeros of the middle byte.
        assert_eq!(longest_zero_bit_run(&[0x01, 0x00, 0x80]), 8);
        // 0xfe ends in a zero: run spans the byte boundary.
        assert_eq!(longest_zero_bit_run(&[0xfe, 0x00, 0xff]), 1 + 8);
        assert_eq!(longest_zero_bit_run(&[0x00, 0x00, 0x00]), 24);
    }

    // ---- extension_and_user_data(i) positive parses ----

    #[test]
    fn empty_element_terminates_on_foreign_start_code() {
        let mut bw = BitWriter::new();
        write_picture_start_code_prefix(&mut bw);
        let buf = bw.into_bytes();
        let out = ExtensionAndUserData::parse(&buf, ExtensionLocation::AfterSequenceExtension)
            .expect("empty element");
        assert_eq!(out, ExtensionAndUserData::default());
    }

    #[test]
    fn empty_element_with_empty_buffer() {
        let out = ExtensionAndUserData::parse(&[], ExtensionLocation::AfterSequenceExtension)
            .expect("empty buffer = empty element");
        assert_eq!(out.byte_position_after, 0);
    }

    #[test]
    fn i0_parses_sequence_display_extension() {
        let mut bw = BitWriter::new();
        write_sequence_display_extension(&mut bw);
        write_picture_start_code_prefix(&mut bw);
        let buf = bw.into_bytes();
        let out = ExtensionAndUserData::parse(&buf, ExtensionLocation::AfterSequenceExtension)
            .expect("i=0 with sequence_display_extension");
        let sde = out.sequence_display_extension.expect("parsed extension");
        assert_eq!(sde.display_horizontal_size, 720);
        assert_eq!(sde.display_vertical_size, 576);
        // 69 bits → 9 bytes; the picture start code follows directly.
        assert_eq!(out.byte_position_after, 9);
        assert!(out.user_data.is_empty());
    }

    #[test]
    fn i0_parses_user_data_then_extension() {
        // §6.2.2.2 allows the two element kinds in any order.
        let mut bw = BitWriter::new();
        write_user_data(&mut bw, b"hi");
        write_sequence_display_extension(&mut bw);
        write_picture_start_code_prefix(&mut bw);
        let buf = bw.into_bytes();
        let out = ExtensionAndUserData::parse(&buf, ExtensionLocation::AfterSequenceExtension)
            .expect("user_data + extension");
        assert_eq!(out.user_data.len(), 1);
        assert_eq!(out.user_data[0].bytes, b"hi");
        assert!(out.sequence_display_extension.is_some());
        assert_eq!(out.byte_position_after, 6 + 9);
    }

    #[test]
    fn i1_parses_user_data_only() {
        let mut bw = BitWriter::new();
        write_user_data(&mut bw, &[0xaa, 0xbb]);
        write_user_data(&mut bw, &[0xcc]);
        write_picture_start_code_prefix(&mut bw);
        let buf = bw.into_bytes();
        let out = ExtensionAndUserData::parse(&buf, ExtensionLocation::AfterGroupOfPicturesHeader)
            .expect("i=1 user_data");
        assert_eq!(out.user_data.len(), 2);
        assert_eq!(out.user_data[0].bytes, [0xaa, 0xbb]);
        assert_eq!(out.user_data[1].bytes, [0xcc]);
        assert_eq!(out.byte_position_after, 6 + 5);
    }

    #[test]
    fn i2_parses_quant_matrix_and_picture_display_extensions() {
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw);
        // frame_picture_ctx: interlaced frame picture, RFF = 0 ⇒
        // number_of_frame_centre_offsets = 2 (§6.3.12).
        write_picture_display_extension(&mut bw, &[(16, -16), (32, -32)]);
        write_picture_start_code_prefix(&mut bw);
        let buf = bw.into_bytes();
        let out = ExtensionAndUserData::parse(&buf, i2_location())
            .expect("i=2 quant matrix + picture display");
        let qme = out.quant_matrix_extension.expect("quant matrix");
        assert_eq!(qme, QuantMatrixExtension::default());
        let pde = out.picture_display_extension.expect("picture display");
        assert_eq!(pde.offsets().len(), 2);
        assert_eq!(
            (pde.offsets()[0].horizontal, pde.offsets()[0].vertical),
            (16, -16)
        );
        // 5 bytes quant matrix + ceil((36 + 2*34)/8) = 13 bytes.
        assert_eq!(out.byte_position_after, 5 + 13);
    }

    #[test]
    fn zero_stuffing_bytes_between_elements_are_skipped() {
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw);
        bw.write_bytes(&[0x00, 0x00, 0x00]); // §5.2.3 zero_byte stuffing
        write_user_data(&mut bw, b"x");
        write_picture_start_code_prefix(&mut bw);
        let buf = bw.into_bytes();
        let out = ExtensionAndUserData::parse(&buf, i2_location()).expect("stuffed element");
        assert!(out.quant_matrix_extension.is_some());
        assert_eq!(out.user_data.len(), 1);
        assert_eq!(out.byte_position_after, 5 + 3 + 5);
    }

    #[test]
    fn leading_zero_stuffing_before_first_element() {
        let mut bw = BitWriter::new();
        bw.write_bytes(&[0x00, 0x00]);
        write_sequence_display_extension(&mut bw);
        write_picture_start_code_prefix(&mut bw);
        let buf = bw.into_bytes();
        let out = ExtensionAndUserData::parse(&buf, ExtensionLocation::AfterSequenceExtension)
            .expect("leading stuffing");
        assert!(out.sequence_display_extension.is_some());
        assert_eq!(out.byte_position_after, 2 + 9);
    }

    #[test]
    fn trailing_zero_stuffing_to_end_of_buffer_is_a_clean_stop() {
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw);
        bw.write_bytes(&[0x00, 0x00]);
        let buf = bw.into_bytes();
        let out = ExtensionAndUserData::parse(&buf, i2_location()).expect("trailing stuffing");
        assert!(out.quant_matrix_extension.is_some());
        assert_eq!(out.byte_position_after, buf.len());
    }

    #[test]
    fn reserved_extension_id_is_discarded_to_next_start_code() {
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(0b0110, 4); // Table 6-2 reserved
        bw.write_u32(0xdead, 20); // arbitrary payload the decoder discards
        write_quant_matrix_extension(&mut bw);
        write_picture_start_code_prefix(&mut bw);
        let buf = bw.into_bytes();
        let out = ExtensionAndUserData::parse(&buf, i2_location()).expect("reserved discarded");
        assert_eq!(out.discarded_reserved_ids, [0b0110]);
        assert!(out.quant_matrix_extension.is_some());
        assert_eq!(out.byte_position_after, 7 + 5);
    }

    #[test]
    fn reserved_extension_discard_may_run_to_end_of_buffer() {
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(0b1111, 4);
        bw.write_u32(0xff, 12); // payload, no further start code
        let buf = bw.into_bytes();
        let out = ExtensionAndUserData::parse(&buf, ExtensionLocation::AfterSequenceExtension)
            .expect("reserved to EOF");
        assert_eq!(out.discarded_reserved_ids, [0b1111]);
        assert_eq!(out.byte_position_after, buf.len());
    }

    // ---- rejection sites ----

    #[test]
    fn i1_rejects_extension_start_code() {
        // §6.2.2.2.1 NOTE: extension_data() never follows a
        // group_of_pictures_header().
        let mut bw = BitWriter::new();
        write_sequence_display_extension(&mut bw);
        write_picture_start_code_prefix(&mut bw);
        assert!(matches!(
            ExtensionAndUserData::parse(
                &bw.into_bytes(),
                ExtensionLocation::AfterGroupOfPicturesHeader
            ),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn i0_rejects_picture_layer_extension_id() {
        // Quant Matrix Extension ID is not in the i=0 allowable set.
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw);
        write_picture_start_code_prefix(&mut bw);
        assert!(matches!(
            ExtensionAndUserData::parse(
                &bw.into_bytes(),
                ExtensionLocation::AfterSequenceExtension
            ),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn i2_rejects_sequence_layer_extension_id() {
        let mut bw = BitWriter::new();
        write_sequence_display_extension(&mut bw);
        write_picture_start_code_prefix(&mut bw);
        assert!(matches!(
            ExtensionAndUserData::parse(&bw.into_bytes(), i2_location()),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn i0_parses_sequence_scalable_extension() {
        use crate::sequence_scalable_extension::ScalableMode;

        // SNR scalability, layer_id 1 — 42 bits → 6 bytes with §5.2.3
        // zero stuffing.
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(SEQUENCE_SCALABLE_EXTENSION_ID, 4);
        bw.write_u32(0b10, 2); // scalable_mode = SNR (Table 6-10)
        bw.write_u32(1, 4); // layer_id
        bw.align_to_byte();
        write_picture_start_code_prefix(&mut bw);
        let buf = bw.into_bytes();
        let out = ExtensionAndUserData::parse(&buf, ExtensionLocation::AfterSequenceExtension)
            .expect("i=0 with sequence_scalable_extension");
        let sse = out.sequence_scalable_extension.expect("parsed extension");
        assert_eq!(sse.scalable_mode, ScalableMode::SnrScalability);
        assert_eq!(sse.layer_id, 1);
        assert_eq!(out.byte_position_after, 6);
    }

    #[test]
    fn i2_parses_copyright_extension() {
        // copyright_flag '0' shape: identifier and number all zero
        // (§6.3.15) — 120 bits, exactly 15 bytes.
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(COPYRIGHT_EXTENSION_ID, 4);
        bw.write_u32(0, 1); // copyright_flag
        bw.write_u32(0, 8); // copyright_identifier
        bw.write_u32(1, 1); // original_or_copy
        bw.write_u32(0, 7); // reserved
        bw.write_u32(1, 1); // marker_bit
        bw.write_u32(0, 20); // copyright_number_1
        bw.write_u32(1, 1); // marker_bit
        bw.write_u32(0, 22); // copyright_number_2
        bw.write_u32(1, 1); // marker_bit
        bw.write_u32(0, 22); // copyright_number_3
        write_picture_start_code_prefix(&mut bw);
        let buf = bw.into_bytes();
        let out =
            ExtensionAndUserData::parse(&buf, i2_location()).expect("i=2 copyright_extension");
        let ce = out.copyright_extension.expect("parsed extension");
        assert!(!ce.copyright_flag);
        assert!(ce.original_or_copy);
        assert_eq!(ce.copyright_number(), 0);
        assert_eq!(out.byte_position_after, 15);
    }

    #[test]
    fn i0_parses_both_sequence_layer_extensions() {
        // §6.3.1: any number of extensions from the allowable set, in
        // any order — both i=0 extensions in one window.
        let mut bw = BitWriter::new();
        write_sequence_display_extension(&mut bw);
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(SEQUENCE_SCALABLE_EXTENSION_ID, 4);
        bw.write_u32(0b00, 2); // scalable_mode = data partitioning
        bw.write_u32(0, 4); // layer_id = partition zero
        bw.align_to_byte();
        write_picture_start_code_prefix(&mut bw);
        let buf = bw.into_bytes();
        let out = ExtensionAndUserData::parse(&buf, ExtensionLocation::AfterSequenceExtension)
            .expect("i=0 with both extensions");
        assert!(out.sequence_display_extension.is_some());
        let sse = out.sequence_scalable_extension.expect("scalable");
        assert_eq!(sse.layer_id, 0);
        assert_eq!(out.byte_position_after, 9 + 6);
    }

    #[test]
    fn i0_rejects_copyright_extension_id() {
        // Copyright Extension ID is in the i=2 allowable set only.
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(COPYRIGHT_EXTENSION_ID, 4);
        bw.write_u32(0, 4);
        write_picture_start_code_prefix(&mut bw);
        assert!(matches!(
            ExtensionAndUserData::parse(
                &bw.into_bytes(),
                ExtensionLocation::AfterSequenceExtension
            ),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn i2_rejects_sequence_scalable_extension_id() {
        // Sequence Scalable Extension ID is in the i=0 allowable set
        // only.
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(SEQUENCE_SCALABLE_EXTENSION_ID, 4);
        bw.write_u32(0b10, 2);
        bw.write_u32(1, 4);
        bw.align_to_byte();
        write_picture_start_code_prefix(&mut bw);
        assert!(matches!(
            ExtensionAndUserData::parse(&bw.into_bytes(), i2_location()),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn i2_parses_picture_spatial_scalable_extension() {
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(PICTURE_SPATIAL_SCALABLE_EXTENSION_ID, 4);
        bw.write_u32(9, 10); // lower_layer_temporal_reference
        bw.write_bit(true); // marker_bit
        bw.write_i32(4, 15); // lower_layer_horizontal_offset (even for 4:2:0)
        bw.write_bit(true); // marker_bit
        bw.write_i32(-6, 15); // lower_layer_vertical_offset (even for 4:2:0)
        bw.write_u32(0b00, 2); // spatial_temporal_weight_code_table_index
        bw.write_bit(true); // lower_layer_progressive_frame
        bw.write_bit(false); // lower_layer_deinterlaced_field_select
        bw.align_to_byte();
        write_picture_start_code_prefix(&mut bw);
        let out = ExtensionAndUserData::parse(&bw.into_bytes(), i2_location()).unwrap();
        let ext = out.picture_spatial_scalable_extension.unwrap();
        assert_eq!(ext.lower_layer_temporal_reference, 9);
        assert_eq!(ext.lower_layer_horizontal_offset, 4);
        assert_eq!(ext.lower_layer_vertical_offset, -6);
        assert!(ext.lower_layer_progressive_frame);
    }

    #[test]
    fn i2_parses_picture_temporal_scalable_extension() {
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(PICTURE_TEMPORAL_SCALABLE_EXTENSION_ID, 4);
        bw.write_u32(0b01, 2); // reference_select_code
        bw.write_u32(5, 10); // forward_temporal_reference
        bw.write_bit(true); // marker_bit
        bw.write_u32(7, 10); // backward_temporal_reference
        bw.align_to_byte();
        write_picture_start_code_prefix(&mut bw);
        let out = ExtensionAndUserData::parse(&bw.into_bytes(), i2_location()).unwrap();
        let ext = out.picture_temporal_scalable_extension.unwrap();
        assert_eq!(ext.reference_select_code, 0b01);
        assert_eq!(ext.forward_temporal_reference, 5);
        assert_eq!(ext.backward_temporal_reference, 7);
    }

    #[test]
    fn duplicate_extension_type_is_rejected() {
        // §6.3.1: "each type of extension shall not occur more than
        // once".
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw);
        write_quant_matrix_extension(&mut bw);
        write_picture_start_code_prefix(&mut bw);
        assert!(matches!(
            ExtensionAndUserData::parse(&bw.into_bytes(), i2_location()),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn non_zero_garbage_before_start_code_is_rejected() {
        // §5.2.3: only zero stuffing may precede a start code.
        let mut bw = BitWriter::new();
        bw.write_bytes(&[0x00, 0x42]);
        write_quant_matrix_extension(&mut bw);
        assert!(matches!(
            ExtensionAndUserData::parse(&bw.into_bytes(), i2_location()),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn short_prefix_before_start_code_is_rejected() {
        // A lone `00 01` run-in (only one zero byte before the 0x01)
        // is not a valid start-code prefix.
        let buf = [0x00, 0x01, 0x00];
        assert!(matches!(
            ExtensionAndUserData::parse(&buf, i2_location()),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn truncated_start_code_value_byte_is_rejected() {
        let buf = [0x00, 0x00, 0x01];
        assert_eq!(
            ExtensionAndUserData::parse(&buf, i2_location()),
            Err(Error::ShortHeader)
        );
    }

    #[test]
    fn truncated_extension_identifier_is_rejected() {
        let buf = [0x00, 0x00, 0x01, 0xb5];
        assert_eq!(
            ExtensionAndUserData::parse(&buf, ExtensionLocation::AfterSequenceExtension),
            Err(Error::ShortHeader)
        );
    }

    #[test]
    fn non_zero_stuffing_bits_after_extension_are_rejected() {
        // sequence_display_extension() ends after 69 bits — bits
        // 70..72 of its last byte are §5.2.3 zero stuffing bits.
        // Force one of them to '1'.
        let mut bw = BitWriter::new();
        write_sequence_display_extension(&mut bw);
        write_picture_start_code_prefix(&mut bw);
        let mut buf = bw.into_bytes();
        buf[8] |= 0x01; // last stuffing bit of the 9-byte extension
        assert!(matches!(
            ExtensionAndUserData::parse(&buf, ExtensionLocation::AfterSequenceExtension),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn inner_extension_parse_errors_propagate() {
        // Zero marker_bit inside sequence_display_extension().
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(SEQUENCE_DISPLAY_EXTENSION_ID, 4);
        bw.write_u32(0b001, 3);
        bw.write_u32(0, 1);
        bw.write_u32(720, 14);
        bw.write_u32(0, 1); // marker_bit violated
        bw.write_u32(576, 14);
        bw.align_to_byte();
        write_picture_start_code_prefix(&mut bw);
        assert!(matches!(
            ExtensionAndUserData::parse(
                &bw.into_bytes(),
                ExtensionLocation::AfterSequenceExtension
            ),
            Err(Error::InvalidBitstream(_))
        ));
    }

    // ---- §6.3.5 / §6.3.12 driver hand-off ----

    #[test]
    fn i0_result_feeds_sequence_display_order_driver() {
        use crate::sequence_display_order::SequenceDisplayOrderDriver;

        let mut bw = BitWriter::new();
        write_sequence_display_extension(&mut bw);
        write_picture_start_code_prefix(&mut bw);
        let buf = bw.into_bytes();
        let out = ExtensionAndUserData::parse(&buf, ExtensionLocation::AfterSequenceExtension)
            .expect("i=0 window");

        let mut driver = SequenceDisplayOrderDriver::new();
        driver
            .on_sequence_header_window(out.sequence_display_extension)
            .expect("first window pins RequiredEqual");
        assert!(driver.picture_display_extension_permitted());

        // An absent-extension i=0 window (element terminates
        // immediately) must now violate the §6.3.5 pin.
        let mut bw2 = BitWriter::new();
        write_picture_start_code_prefix(&mut bw2);
        let buf2 = bw2.into_bytes();
        let out2 = ExtensionAndUserData::parse(&buf2, ExtensionLocation::AfterSequenceExtension)
            .expect("absent window");
        assert!(out2.sequence_display_extension.is_none());
        assert!(driver
            .on_sequence_header_window(out2.sequence_display_extension)
            .is_err());
    }
}
