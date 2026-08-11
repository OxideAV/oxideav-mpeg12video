//! Bitstream layer writers — the encoder side of the §6.2 MPEG-2 video
//! syntax (sequence / sequence-extension / picture / picture-coding-
//! extension / slice headers, plus the sequence-end code).
//!
//! Each writer emits exactly the syntax its decoder counterpart parses,
//! using the same start codes and field widths:
//!
//! | Writer | Syntax | Decoder counterpart |
//! |--------|--------|---------------------|
//! | [`write_sequence_header`] | §6.2.2.1 `sequence_header()` | [`crate::sequence_header::Mpeg2SequenceHeader::parse`] |
//! | [`write_sequence_extension`] | §6.2.2.3 `sequence_extension()` | [`crate::sequence_extension::Mpeg2SequenceExtension`] |
//! | [`write_picture_header`] | §6.2.3 `picture_header()` | [`crate::picture_header::Mpeg2PictureHeader`] |
//! | [`write_picture_coding_extension`] | §6.2.3.1 `picture_coding_extension()` | [`crate::picture_header::PictureCodingExtension`] |
//! | [`write_slice_header`] | §6.2.4 `slice()` header | [`crate::slice_header::SliceHeader::parse`] |
//!
//! The writers cover the **baseline frame-picture** path an I/P-picture
//! encoder needs: default (non-loaded) quantiser matrices,
//! `picture_structure = Frame`, `frame_pred_frame_dct = 1`. They are
//! validated by the encode→decode round-trip in
//! `tests/encode_intra_roundtrip.rs`, where every field written here is
//! read back by the decoder and the reconstructed frame is compared to
//! the encoder's input.

use oxideav_core::bits::BitWriter;

use crate::picture_header::{PictureCodingType, PICTURE_START_CODE};
use crate::sequence_extension::{ChromaFormat, EXTENSION_START_CODE};
use crate::sequence_header::SEQUENCE_HEADER_CODE;

/// §6.2.2 `sequence_end_code` (`0x000001B7`).
pub const SEQUENCE_END_CODE: u32 = 0x0000_01B7;

/// §6.2.3.1 picture-coding-extension start id (Table 6-2 `Picture
/// Coding Extension ID = 1000`).
const PICTURE_CODING_EXTENSION_ID: u32 = 0b1000;

/// §6.2.2.3 sequence-extension start id (Table 6-2 `Sequence Extension
/// ID = 0001`).
const SEQUENCE_EXTENSION_ID: u32 = 0b0001;

/// Parameters for a §6.2.2.1 `sequence_header()` write. Only the
/// baseline fields an encoder varies are exposed; the quantiser
/// matrices are left at their defaults (both `load_*_quantiser_matrix`
/// flags `0`).
#[derive(Debug, Clone, Copy)]
pub struct SequenceHeaderParams {
    /// `horizontal_size_value` (lower 12 bits of the picture width).
    pub horizontal_size: u16,
    /// `vertical_size_value` (lower 12 bits of the picture height).
    pub vertical_size: u16,
    /// `aspect_ratio_information` (Table 6-3, 4 bits; `1` = square).
    pub aspect_ratio_code: u8,
    /// `frame_rate_code` (Table 6-4, 4 bits; `3` = 25 fps).
    pub frame_rate_code: u8,
    /// `bit_rate_value` (18 bits, must be non-zero).
    pub bit_rate_value: u32,
    /// `vbv_buffer_size_value` (10 bits).
    pub vbv_buffer_size_value: u16,
}

impl Default for SequenceHeaderParams {
    fn default() -> Self {
        Self {
            horizontal_size: 16,
            vertical_size: 16,
            aspect_ratio_code: 0b0001,
            frame_rate_code: 0b0011,
            bit_rate_value: 2500,
            vbv_buffer_size_value: 112,
        }
    }
}

/// Write a §6.2.2.1 `sequence_header()` with both quantiser-matrix load
/// flags clear (the §6.3.7 defaults apply).
pub fn write_sequence_header(bw: &mut BitWriter, p: &SequenceHeaderParams) {
    bw.write_u32(SEQUENCE_HEADER_CODE, 32);
    bw.write_u32(u32::from(p.horizontal_size), 12);
    bw.write_u32(u32::from(p.vertical_size), 12);
    bw.write_u32(u32::from(p.aspect_ratio_code), 4);
    bw.write_u32(u32::from(p.frame_rate_code), 4);
    bw.write_u32(p.bit_rate_value, 18);
    bw.write_bit(true); // marker_bit
    bw.write_u32(u32::from(p.vbv_buffer_size_value), 10);
    bw.write_bit(false); // constrained_parameters_flag
    bw.write_bit(false); // load_intra_quantiser_matrix
    bw.write_bit(false); // load_non_intra_quantiser_matrix
    bw.align_to_byte();
}

/// Map a [`ChromaFormat`] to its §6.3.5 2-bit `chroma_format` code.
fn chroma_format_code(chroma: ChromaFormat) -> u32 {
    match chroma {
        ChromaFormat::Yuv420 => 0b01,
        ChromaFormat::Yuv422 => 0b10,
        ChromaFormat::Yuv444 => 0b11,
    }
}

/// Write a §6.2.2.3 `sequence_extension()` for a non-scalable,
/// non-progressive sequence with all size / rate extension fields zero
/// (the §6.2.2.1 base values carry the full geometry).
pub fn write_sequence_extension(bw: &mut BitWriter, chroma: ChromaFormat, progressive: bool) {
    bw.write_u32(EXTENSION_START_CODE, 32);
    bw.write_u32(SEQUENCE_EXTENSION_ID, 4);
    bw.write_u32(0x48, 8); // profile_and_level_indication (Main@Main)
    bw.write_bit(progressive); // progressive_sequence
    bw.write_u32(chroma_format_code(chroma), 2);
    bw.write_u32(0, 2); // horizontal_size_extension
    bw.write_u32(0, 2); // vertical_size_extension
    bw.write_u32(0, 12); // bit_rate_extension
    bw.write_bit(true); // marker_bit
    bw.write_u32(0, 8); // vbv_buffer_size_extension
    bw.write_bit(false); // low_delay
    bw.write_u32(0, 2); // frame_rate_extension_n
    bw.write_u32(0, 5); // frame_rate_extension_d
    bw.align_to_byte();
}

/// Write a §6.2.3 `picture_header()` for the given coding type and
/// `temporal_reference`.
///
/// `forward_f_code` / `backward_f_code` carry the MPEG-1-style
/// `forward_f_code` / `backward_f_code` placeholders that follow
/// `vbv_delay` for P / B pictures (the MPEG-2 per-direction f_codes
/// live in the picture-coding extension; these 3-bit fields here are
/// the legacy ones the §6.2.3 syntax still emits, set to `0b111` =
/// unused when MPEG-2 motion is signalled in the extension).
pub fn write_picture_header(
    bw: &mut BitWriter,
    temporal_reference: u16,
    coding_type: PictureCodingType,
    forward_f_code: u8,
    backward_f_code: u8,
) {
    bw.write_u32(PICTURE_START_CODE, 32);
    bw.write_u32(u32::from(temporal_reference), 10);
    let ct_code = match coding_type {
        PictureCodingType::Intra => 0b001,
        PictureCodingType::Predictive => 0b010,
        PictureCodingType::Bidirectional => 0b011,
        // dc intra-coded (D), ISO/IEC 11172-2 §2.4.3.4 — carries no
        // f_code fields (the P/B conditionals below skip it).
        PictureCodingType::DcIntra => 0b100,
    };
    bw.write_u32(ct_code, 3);
    bw.write_u32(0xFFFF, 16); // vbv_delay
    if matches!(
        coding_type,
        PictureCodingType::Predictive | PictureCodingType::Bidirectional
    ) {
        bw.write_bit(false); // full_pel_forward_vector
        bw.write_u32(u32::from(forward_f_code), 3);
    }
    if coding_type == PictureCodingType::Bidirectional {
        bw.write_bit(false); // full_pel_backward_vector
        bw.write_u32(u32::from(backward_f_code), 3);
    }
    bw.write_bit(false); // extra_bit_picture = 0
    bw.align_to_byte();
}

/// Parameters for the §6.2.3.1 `picture_coding_extension()` write.
#[derive(Debug, Clone, Copy)]
pub struct PictureCodingExtensionParams {
    /// `f_code[0][0]` / `f_code[0][1]` forward f_code (4 bits; 15 =
    /// unused).
    pub forward_f_code: u8,
    /// `f_code[1][0]` / `f_code[1][1]` backward f_code (4 bits; 15 =
    /// unused).
    pub backward_f_code: u8,
    /// `intra_dc_precision` (Table 6-13, `0..=3`).
    pub intra_dc_precision: u8,
    /// `frame_pred_frame_dct`.
    pub frame_pred_frame_dct: bool,
    /// `q_scale_type`.
    pub q_scale_type: bool,
    /// `intra_vlc_format`.
    pub intra_vlc_format: bool,
    /// `alternate_scan`.
    pub alternate_scan: bool,
    /// `progressive_frame`.
    pub progressive_frame: bool,
}

impl Default for PictureCodingExtensionParams {
    fn default() -> Self {
        Self {
            forward_f_code: 15,
            backward_f_code: 15,
            intra_dc_precision: 0,
            frame_pred_frame_dct: true,
            q_scale_type: false,
            intra_vlc_format: false,
            alternate_scan: false,
            progressive_frame: false,
        }
    }
}

/// Write a §6.2.3.1 `picture_coding_extension()` for a frame picture
/// (`picture_structure = Frame`, `top_field_first = 1`).
pub fn write_picture_coding_extension(bw: &mut BitWriter, p: &PictureCodingExtensionParams) {
    bw.write_u32(EXTENSION_START_CODE, 32);
    bw.write_u32(PICTURE_CODING_EXTENSION_ID, 4);
    bw.write_u32(u32::from(p.forward_f_code), 4); // f_code[0][0]
    bw.write_u32(u32::from(p.forward_f_code), 4); // f_code[0][1]
    bw.write_u32(u32::from(p.backward_f_code), 4); // f_code[1][0]
    bw.write_u32(u32::from(p.backward_f_code), 4); // f_code[1][1]
    bw.write_u32(u32::from(p.intra_dc_precision), 2);
    bw.write_u32(0b11, 2); // picture_structure = Frame
    bw.write_bit(true); // top_field_first
    bw.write_bit(p.frame_pred_frame_dct);
    bw.write_bit(false); // concealment_motion_vectors
    bw.write_bit(p.q_scale_type);
    bw.write_bit(p.intra_vlc_format);
    bw.write_bit(p.alternate_scan);
    bw.write_bit(false); // repeat_first_field
    bw.write_bit(false); // chroma_420_type
    bw.write_bit(p.progressive_frame);
    bw.write_bit(false); // composite_display_flag
    bw.align_to_byte();
}

/// Write a §6.2.3.1 `picture_coding_extension()` for a **field
/// picture** (`picture_structure` = Top Field `01` / Bottom Field `10`
/// per Table 6-14).
///
/// The §6.3.10 field-picture constraints are applied here rather than
/// exposed as parameters: `top_field_first = 0` (*"in a field picture
/// top_field_first shall have the value '0'"*), `repeat_first_field =
/// 0` (*"in a field picture it shall be set to zero"*),
/// `frame_pred_frame_dct = 0` (the flag only applies to frame
/// pictures; a field picture always reads `field_motion_type` and has
/// no frame/field DCT distinction, §6.2.5.1 / Table 6-19),
/// `progressive_frame = 0` (field pictures only occur in interlaced
/// sequences, §6.3.5), and `chroma_420_type = progressive_frame = 0`
/// (§6.3.11).
///
/// # Panics
/// Debug-asserts that `structure` is not [`PictureStructure::Frame`]
/// (use [`write_picture_coding_extension`] for frame pictures).
pub fn write_field_picture_coding_extension(
    bw: &mut BitWriter,
    p: &PictureCodingExtensionParams,
    structure: crate::picture_header::PictureStructure,
) {
    use crate::picture_header::PictureStructure;
    debug_assert_ne!(structure, PictureStructure::Frame);
    let structure_code = match structure {
        PictureStructure::TopField => 0b01,
        PictureStructure::BottomField => 0b10,
        PictureStructure::Frame => 0b11,
    };
    bw.write_u32(EXTENSION_START_CODE, 32);
    bw.write_u32(PICTURE_CODING_EXTENSION_ID, 4);
    bw.write_u32(u32::from(p.forward_f_code), 4); // f_code[0][0]
    bw.write_u32(u32::from(p.forward_f_code), 4); // f_code[0][1]
    bw.write_u32(u32::from(p.backward_f_code), 4); // f_code[1][0]
    bw.write_u32(u32::from(p.backward_f_code), 4); // f_code[1][1]
    bw.write_u32(u32::from(p.intra_dc_precision), 2);
    bw.write_u32(structure_code, 2); // picture_structure (Table 6-14)
    bw.write_bit(false); // top_field_first (§6.3.10: '0' in a field picture)
    bw.write_bit(false); // frame_pred_frame_dct (frame pictures only)
    bw.write_bit(false); // concealment_motion_vectors
    bw.write_bit(p.q_scale_type);
    bw.write_bit(p.intra_vlc_format);
    bw.write_bit(p.alternate_scan);
    bw.write_bit(false); // repeat_first_field (§6.3.10: zero in a field picture)
    bw.write_bit(false); // chroma_420_type (= progressive_frame, §6.3.11)
    bw.write_bit(false); // progressive_frame (interlaced sequence)
    bw.write_bit(false); // composite_display_flag
    bw.align_to_byte();
}

/// Write a §6.2.4 `slice()` header for the macroblock row `mb_row`
/// (slice_vertical_position = `mb_row + 1`), for the baseline
/// non-scalable, `vertical_size <= 2800` case (no
/// `slice_vertical_position_extension`, no `priority_breakpoint`, no
/// `intra_slice` prelude).
///
/// `quantiser_scale_code` is the 5-bit per-slice quantiser scale code
/// (`1..=31`). After this header the caller writes the macroblock body
/// at the now non-byte-aligned bit position.
pub fn write_slice_header(bw: &mut BitWriter, mb_row: u32, quantiser_scale_code: u8) {
    // slice_start_code = 0x00000100 + slice_vertical_position; the
    // slice_vertical_position is mb_row + 1 (1..=174 for the 8-bit
    // start-code byte range 0x01..=0xAF).
    bw.write_u32(0x00_00_01, 24); // start-code prefix
    bw.write_u32(mb_row + 1, 8); // slice_vertical_position byte
    bw.write_u32(u32::from(quantiser_scale_code), 5);
    bw.write_bit(false); // extra_bit_slice = 0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sequence_extension::Mpeg2Sequence;
    use crate::sequence_header::Mpeg2SequenceHeader;

    #[test]
    fn sequence_header_roundtrips() {
        let p = SequenceHeaderParams {
            horizontal_size: 352,
            vertical_size: 240,
            ..Default::default()
        };
        let mut bw = BitWriter::new();
        write_sequence_header(&mut bw, &p);
        let bytes = bw.finish();
        let sh = Mpeg2SequenceHeader::parse(&bytes).expect("parse sequence header");
        assert_eq!(sh.width, 352);
        assert_eq!(sh.height, 240);
    }

    #[test]
    fn sequence_header_plus_extension_roundtrips() {
        let mut bw = BitWriter::new();
        write_sequence_header(
            &mut bw,
            &SequenceHeaderParams {
                horizontal_size: 64,
                vertical_size: 48,
                ..Default::default()
            },
        );
        write_sequence_extension(&mut bw, ChromaFormat::Yuv420, false);
        let bytes = bw.finish();
        let seq = Mpeg2Sequence::from_buf(&bytes).expect("parse sequence");
        assert_eq!(seq.horizontal_size, 64);
        assert_eq!(seq.vertical_size, 48);
        assert_eq!(seq.extension.chroma_format, ChromaFormat::Yuv420);
    }

    #[test]
    fn chroma_format_codes_match_spec() {
        assert_eq!(chroma_format_code(ChromaFormat::Yuv420), 0b01);
        assert_eq!(chroma_format_code(ChromaFormat::Yuv422), 0b10);
        assert_eq!(chroma_format_code(ChromaFormat::Yuv444), 0b11);
    }

    #[test]
    fn picture_header_and_extension_roundtrip() {
        use crate::picture_header::{Mpeg2PictureHeader, PictureStructure};
        let mut bw = BitWriter::new();
        write_picture_header(&mut bw, 7, PictureCodingType::Intra, 15, 15);
        write_picture_coding_extension(
            &mut bw,
            &PictureCodingExtensionParams {
                intra_dc_precision: 1,
                ..Default::default()
            },
        );
        let bytes = bw.finish();
        let (hdr, ext) =
            Mpeg2PictureHeader::parse_with_extension(&bytes).expect("parse picture header");
        assert_eq!(hdr.temporal_reference, 7);
        assert_eq!(hdr.picture_coding_type, PictureCodingType::Intra);
        assert_eq!(ext.picture_structure, PictureStructure::Frame);
        assert_eq!(ext.intra_dc_precision, 1);
        assert!(ext.frame_pred_frame_dct);
    }

    #[test]
    fn slice_header_roundtrips() {
        use crate::slice_header::{SliceContext, SliceHeader};
        let mut bw = BitWriter::new();
        write_slice_header(&mut bw, 4, 8);
        // Pad with a stop so the parser has a body; align then a byte.
        bw.write_bit(true); // macroblock_address_increment placeholder
        bw.align_to_byte_zero();
        let bytes = bw.finish();
        let hdr = SliceHeader::parse(&bytes, SliceContext::non_scalable(240))
            .expect("parse slice header");
        assert_eq!(hdr.slice_vertical_position, 5); // mb_row 4 + 1
        assert_eq!(hdr.quantiser_scale_code, 8);
    }
}
