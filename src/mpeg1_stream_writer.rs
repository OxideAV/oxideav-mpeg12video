//! ISO/IEC 11172-2 (MPEG-1 Video) sequence-layer **writer** — the
//! encoder side of the §2.4.2.3 `sequence_header()` syntax, plus the
//! §2.4.3.2 constrained-parameters admissibility check.
//!
//! An MPEG-1 sequence header carries the same 32 + 12 + 12 + 4 + 4 +
//! 18 + 1 + 10 + 1 + 1 + 1 field layout as the ISO/IEC 13818-2
//! §6.2.2.1 base header (13818-2 §6.3.3 documents the field
//! correspondence), but the MPEG-1 field *semantics* differ in the
//! ways an encoder must honour:
//!
//! * `pel_aspect_ratio` (§2.4.3.2) is the MPEG-1 aspect table, not
//!   the Table 6-3 display-aspect codes (`0001` = square pels in
//!   both).
//! * `picture_rate` is the §2.4.3.2 table (codes `1..=8`, identical
//!   numeric rates to Table 6-4).
//! * `constrained_parameters_flag` is **meaningful** (13818-2 §6.3.3
//!   requires it to be `'0'`): it may be set to `'1'` only when the
//!   §2.4.3.2 constrained-parameters bounds hold —
//!   [`constrained_parameters_admissible`] evaluates them.
//! * No `sequence_extension()` follows: the very *absence* of the
//!   extension start code after the header is what classifies the
//!   stream as ISO/IEC 11172-2 (13818-2 §6.1.1.6 conversely mandates
//!   the extension for MPEG-2 streams).
//!
//! The MPEG-1 picture header (§2.4.2.5) and slice layer (§2.4.2.6)
//! reuse [`crate::stream_writer::write_picture_header`] /
//! [`crate::stream_writer::write_slice_header`] verbatim: the wire
//! layouts are identical, and for MPEG-1 the picture header's
//! `full_pel_*_vector` = `'0'` + real 3-bit `forward_f_code` /
//! `backward_f_code` fields are exactly what those writers emit (an
//! MPEG-2 encoder passes the §6.3.10 `'111'` placeholders instead).
//!
//! Spec citations refer to ISO/IEC 11172-2:1993 §2.4.2.3 / §2.4.3.2
//! unless MPEG-2 clause numbers are named explicitly.

use oxideav_core::bits::BitWriter;

use crate::gop_header::nominal_pictures_per_second;
use crate::sequence_header::SEQUENCE_HEADER_CODE;

/// Parameters for a §2.4.2.3 MPEG-1 `sequence_header()` write. Both
/// `load_*_quantizer_matrix` flags are written `'0'` (the §2.4.3.2
/// default matrices apply).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mpeg1SequenceParams {
    /// `horizontal_size` (12 bits) — width of the displayable
    /// luminance in pels. Zero is forbidden.
    pub horizontal_size: u16,
    /// `vertical_size` (12 bits) — height in lines. Zero is forbidden.
    pub vertical_size: u16,
    /// `pel_aspect_ratio` (4 bits, §2.4.3.2; `1` = square pels;
    /// `0` forbidden, `15` reserved).
    pub pel_aspect_ratio_code: u8,
    /// `picture_rate` (4 bits, §2.4.3.2 table; codes `1..=8`;
    /// `3` = 25 pictures/s).
    pub picture_rate_code: u8,
    /// `bit_rate` (18 bits) in units of 400 bit/s rounded upwards;
    /// `0` forbidden; `0x3FFFF` = variable bit rate.
    pub bit_rate_value: u32,
    /// `vbv_buffer_size` (10 bits): the minimum VBV buffer is
    /// `B = 16 * 1024 * vbv_buffer_size` bits (annex C).
    pub vbv_buffer_size_value: u16,
    /// `constrained_parameters_flag` — may be `'1'` only when the
    /// §2.4.3.2 bounds hold (see
    /// [`constrained_parameters_admissible`]).
    pub constrained_parameters_flag: bool,
}

impl Default for Mpeg1SequenceParams {
    /// A square-pel 25 pictures/s baseline at 1 856 000 bit/s
    /// (`4640` × 400) with the constrained-parameters maximum VBV
    /// buffer (`20` × 16 384 bits); the flag itself defaults to `'0'`
    /// and is set by callers that verified admissibility.
    fn default() -> Self {
        Self {
            horizontal_size: 16,
            vertical_size: 16,
            pel_aspect_ratio_code: 1,
            picture_rate_code: 3,
            bit_rate_value: 4640,
            vbv_buffer_size_value: 20,
            constrained_parameters_flag: false,
        }
    }
}

/// The §2.4.3.2 constrained-parameters bit-rate ceiling: *"the
/// bit_rate field shall indicate a coded data rate less than or equal
/// to 1 856 000 bits/s"* — in the header's 400 bit/s units, `4640`.
pub const CPB_MAX_BIT_RATE_VALUE: u32 = 1_856_000 / 400;

/// The §2.4.3.2 constrained-parameters VBV ceiling: *"a VBV buffer
/// size less than or equal to 327 680 bits (20*1024*16)"* — a
/// `vbv_buffer_size` field value of `20`.
pub const CPB_MAX_VBV_BUFFER_SIZE_VALUE: u16 = 20;

/// Evaluate the §2.4.3.2 constrained-parameters bounds for a sequence
/// with the given geometry/rate parameters and the largest
/// `forward_f_code` / `backward_f_code` any picture in the sequence
/// uses (pass `1` for a direction never coded):
///
/// * `horizontal_size <= 768` pels, `vertical_size <= 576` pels;
/// * `((horizontal_size+15)/16) * ((vertical_size+15)/16) <= 396`;
/// * that macroblock count × `picture_rate <= 396*25`;
/// * `picture_rate <= 30` pictures/s;
/// * `forward_f_code <= 4` and `backward_f_code <= 4` (§2.4.3.4);
/// * `bit_rate <= 1 856 000` bit/s (and not the `0x3FFFF` VBR code);
/// * `vbv_buffer_size <= 20` (`B <= 327 680` bits).
///
/// The picture-rate factor uses the nominal integral rate
/// ([`nominal_pictures_per_second`]); an undefined `picture_rate`
/// code is not admissible.
pub fn constrained_parameters_admissible(
    p: &Mpeg1SequenceParams,
    max_forward_f_code: u8,
    max_backward_f_code: u8,
) -> bool {
    let Ok(rate) = nominal_pictures_per_second(p.picture_rate_code) else {
        return false;
    };
    let mb_count =
        u64::from(p.horizontal_size.div_ceil(16)) * u64::from(p.vertical_size.div_ceil(16));
    p.horizontal_size <= 768
        && p.vertical_size <= 576
        && mb_count <= 396
        && mb_count * u64::from(rate) <= 396 * 25
        && rate <= 30
        && max_forward_f_code <= 4
        && max_backward_f_code <= 4
        && p.bit_rate_value != 0x3_FFFF
        && p.bit_rate_value <= CPB_MAX_BIT_RATE_VALUE
        && p.vbv_buffer_size_value <= CPB_MAX_VBV_BUFFER_SIZE_VALUE
}

/// Write a §2.4.2.3 MPEG-1 `sequence_header()` with both quantiser-
/// matrix load flags `'0'` (the §2.4.3.2 default matrices apply),
/// padded to the byte boundary for `next_start_code()`.
///
/// The caller supplies field values inside their syntactic ranges
/// (the companion parser rejects the §2.4.3.2 forbidden values).
/// **No `sequence_extension()` may follow** — its absence is what
/// makes the stream ISO/IEC 11172-2.
pub fn write_mpeg1_sequence_header(bw: &mut BitWriter, p: &Mpeg1SequenceParams) {
    bw.write_u32(SEQUENCE_HEADER_CODE, 32);
    bw.write_u32(u32::from(p.horizontal_size), 12);
    bw.write_u32(u32::from(p.vertical_size), 12);
    bw.write_u32(u32::from(p.pel_aspect_ratio_code), 4);
    bw.write_u32(u32::from(p.picture_rate_code), 4);
    bw.write_u32(p.bit_rate_value, 18);
    bw.write_bit(true); // marker_bit
    bw.write_u32(u32::from(p.vbv_buffer_size_value), 10);
    bw.write_bit(p.constrained_parameters_flag);
    bw.write_bit(false); // load_intra_quantizer_matrix
    bw.write_bit(false); // load_non_intra_quantizer_matrix
    bw.align_to_byte();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sequence_header::{AspectRatio, Mpeg2SequenceHeader};

    fn sif_params() -> Mpeg1SequenceParams {
        Mpeg1SequenceParams {
            horizontal_size: 352,
            vertical_size: 240,
            picture_rate_code: 5, // 30 pictures/s
            constrained_parameters_flag: true,
            ..Default::default()
        }
    }

    #[test]
    fn sequence_header_roundtrips_through_the_parser() {
        let p = sif_params();
        let mut bw = BitWriter::new();
        write_mpeg1_sequence_header(&mut bw, &p);
        let bytes = bw.finish();
        // 32+12+12+4+4+18+1+10+1+1+1 = 96 bits = 12 bytes exactly.
        assert_eq!(bytes.len(), 12);

        let h = Mpeg2SequenceHeader::parse(&bytes).expect("parse");
        assert_eq!(h.width, 352);
        assert_eq!(h.height, 240);
        assert_eq!(h.aspect_ratio, AspectRatio::Square);
        assert_eq!(h.frame_rate_code, 5);
        assert_eq!(h.bit_rate, 4640);
        assert_eq!(h.vbv_buffer_size, 20);
        assert!(h.constrained_parameters);
        assert!(h.intra_quant.is_none());
        assert!(h.non_intra_quant.is_none());
    }

    #[test]
    fn no_sequence_extension_follows_the_header() {
        // The MPEG-1 classification hinges on the *absence* of an
        // extension start code after the header: the chained MPEG-2
        // parser must fail on the bare header while the bare parser
        // succeeds.
        let mut bw = BitWriter::new();
        write_mpeg1_sequence_header(&mut bw, &sif_params());
        // Follow with a GOP start code, as a real MPEG-1 stream does.
        bw.write_u32(crate::gop_header::GROUP_START_CODE, 32);
        let bytes = bw.finish();
        assert!(Mpeg2SequenceHeader::parse(&bytes).is_ok());
        assert!(crate::sequence_extension::Mpeg2Sequence::from_buf(&bytes).is_err());
    }

    // ---- §2.4.3.2 constrained-parameters matrix --------------------

    #[test]
    fn cpb_sif_rates_are_admissible() {
        // 352x240 @ 29,97 or 30 Hz and 352x288 @ 25 Hz are the classic
        // constrained-parameters operating points.
        for (w, h, rate) in [(352u16, 240u16, 4u8), (352, 240, 5), (352, 288, 3)] {
            let p = Mpeg1SequenceParams {
                horizontal_size: w,
                vertical_size: h,
                picture_rate_code: rate,
                ..Default::default()
            };
            assert!(
                constrained_parameters_admissible(&p, 4, 4),
                "{w}x{h} rate code {rate}"
            );
        }
    }

    #[test]
    fn cpb_rejects_each_violated_bound() {
        let base = sif_params();
        // horizontal_size > 768.
        let p = Mpeg1SequenceParams {
            horizontal_size: 769,
            ..base
        };
        assert!(!constrained_parameters_admissible(&p, 1, 1));
        // vertical_size > 576.
        let p = Mpeg1SequenceParams {
            vertical_size: 577,
            ..base
        };
        assert!(!constrained_parameters_admissible(&p, 1, 1));
        // Macroblock count: 768x592 would pass the raw size bounds
        // only via the 396-MB cap (48*37 = 1776 > 396).
        let p = Mpeg1SequenceParams {
            horizontal_size: 768,
            vertical_size: 576,
            ..base
        };
        assert!(!constrained_parameters_admissible(&p, 1, 1));
        // MB*rate: 396 MBs at 30 Hz exceeds 396*25.
        let p = Mpeg1SequenceParams {
            horizontal_size: 352,
            vertical_size: 288, // 22*18 = 396 MBs
            picture_rate_code: 5,
            ..base
        };
        assert!(!constrained_parameters_admissible(&p, 1, 1));
        // …but the same geometry at 25 Hz is exactly on the bound.
        let p = Mpeg1SequenceParams {
            horizontal_size: 352,
            vertical_size: 288,
            picture_rate_code: 3,
            ..base
        };
        assert!(constrained_parameters_admissible(&p, 1, 1));
        // picture_rate > 30 (50 Hz).
        let p = Mpeg1SequenceParams {
            picture_rate_code: 6,
            ..base
        };
        assert!(!constrained_parameters_admissible(&p, 1, 1));
        // f_codes above 4.
        assert!(!constrained_parameters_admissible(&base, 5, 1));
        assert!(!constrained_parameters_admissible(&base, 1, 5));
        assert!(constrained_parameters_admissible(&base, 4, 4));
        // Bit rate above 1 856 000 bit/s, or VBR.
        let p = Mpeg1SequenceParams {
            bit_rate_value: CPB_MAX_BIT_RATE_VALUE + 1,
            ..base
        };
        assert!(!constrained_parameters_admissible(&p, 1, 1));
        let p = Mpeg1SequenceParams {
            bit_rate_value: 0x3_FFFF,
            ..base
        };
        assert!(!constrained_parameters_admissible(&p, 1, 1));
        // VBV buffer above 327 680 bits.
        let p = Mpeg1SequenceParams {
            vbv_buffer_size_value: CPB_MAX_VBV_BUFFER_SIZE_VALUE + 1,
            ..base
        };
        assert!(!constrained_parameters_admissible(&p, 1, 1));
        // Undefined picture-rate code.
        let p = Mpeg1SequenceParams {
            picture_rate_code: 9,
            ..base
        };
        assert!(!constrained_parameters_admissible(&p, 1, 1));
    }

    #[test]
    fn default_params_are_admissible_at_16x16() {
        assert!(constrained_parameters_admissible(
            &Mpeg1SequenceParams::default(),
            4,
            4
        ));
    }
}
