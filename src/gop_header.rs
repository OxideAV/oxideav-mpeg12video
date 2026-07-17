//! Parser **and writer** for the MPEG-1 / MPEG-2 video
//! `group_of_pictures_header()` syntax element.
//!
//! Implements the bitstream syntax in ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262) §6.2.2.6 and the field semantics in §6.3.8. The same
//! wire syntax is the ISO/IEC 11172-2 (MPEG-1) §2.4.2.4 group-of-
//! pictures layer with the §2.4.3.3 field semantics — one
//! group_start_code, the 25-bit time_code, and the `closed_gop` /
//! `broken_link` flags — so the parser and [`write_gop_header`] writer
//! here serve both standards (MPEG-1 makes the layer mandatory:
//! §2.4.2.2 `video_sequence()` loops `group_of_pictures()` directly
//! under each `sequence_header()`).
//!
//! `group_of_pictures_header()` is an optional layer that may precede
//! a `picture_header()`. When present it sets a 25-bit time-code (per
//! IEC publication 461) for the next coded picture whose
//! `temporal_reference` is zero, plus two single-bit flags that govern
//! how a decoder may handle the leading B-pictures of the GOP after
//! editing splices:
//!
//! * `closed_gop` -- the first consecutive B-pictures following the
//!   first coded I-frame in this GOP were encoded with backward
//!   prediction or intra coding only (so they are safely decodable
//!   without any preceding reference).
//! * `broken_link` -- shall be `'0'` during encoding; an editor may
//!   set it to `'1'` to mark that the preceding reference frame has
//!   been removed and the leading B-pictures of this GOP may not be
//!   correctly decoded.
//!
//! Per §6.3.8 the information carried by `time_code` plays no part in
//! the decoding process; we surface every sub-field nevertheless so
//! callers that propagate timecode metadata (e.g. demuxers) can use
//! it.
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)).

use oxideav_core::bits::{BitReader, BitWriter};

use crate::{Error, Result};

/// The 32-bit start code that introduces a
/// `group_of_pictures_header()`: the byte string `00 00 01 B8`
/// (§6.2.1 Table 6-1, §6.3.8).
pub const GROUP_START_CODE: u32 = 0x0000_01B8;

/// Decoded 25-bit `time_code` sub-fields per Table 6-11 of §6.3.8.
///
/// The numeric ranges enforced by the parser match the spec's
/// "range of value" column:
///
/// * `time_code_hours` ∈ 0..=23
/// * `time_code_minutes` ∈ 0..=59
/// * `time_code_seconds` ∈ 0..=59
/// * `time_code_pictures` ∈ 0..=59
///
/// `drop_frame_flag` may be either `'0'` or `'1'`; per §6.3.8 it may
/// be set to `'1'` only if the frame rate is 29,97 Hz. Enforcing the
/// frame-rate cross-check requires the `sequence_header()` /
/// `sequence_extension()` companion data, which is the caller's
/// responsibility (see [`crate::sequence_extension::Mpeg2Sequence`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimeCode {
    /// `drop_frame_flag`: when `'1'`, picture numbers 0 and 1 are
    /// dropped at the start of each minute except minutes 0, 10, 20,
    /// 30, 40, 50 (NTSC drop-frame convention).
    pub drop_frame: bool,
    /// `time_code_hours` ∈ 0..=23 (5-bit field).
    pub hours: u8,
    /// `time_code_minutes` ∈ 0..=59 (6-bit field).
    pub minutes: u8,
    /// `time_code_seconds` ∈ 0..=59 (6-bit field).
    pub seconds: u8,
    /// `time_code_pictures` ∈ 0..=59 (6-bit field).
    pub pictures: u8,
}

impl TimeCode {
    /// Derive the time code of the display frame at continuous display
    /// index `display_index` for the sequence header's 4-bit
    /// `picture_rate` / `frame_rate_code` (Table 6-4 of §6.3.3 /
    /// ISO/IEC 11172-2 §2.4.3.2), with `drop_frame = 0`.
    ///
    /// Per §6.3.8 (and 11172-2 §2.4.3.3), with `drop_frame_flag == 0`
    /// *"pictures are counted assuming rounding to the nearest integral
    /// number of pictures per second"* — so the 23,976 / 29,97 / 59,94
    /// rates count as 24 / 30 / 60 pictures per second. The hours field
    /// wraps modulo 24 (its Table 6-11 range is `0..=23`).
    ///
    /// This is the encoder-side helper for GOP emission: the §6.3.8
    /// time code *"refers to the first picture in the group of pictures
    /// that has a `temporal_reference` of zero"* (the GOP's earliest
    /// picture in display order), so a display-order assembler passes
    /// the continuous display index of each GOP's first frame.
    ///
    /// # Errors
    /// [`Error::InvalidBitstream`] for a `picture_rate` code outside
    /// the defined `1..=8` range.
    pub fn from_display_index(display_index: u64, picture_rate_code: u8) -> Result<Self> {
        let fps = nominal_pictures_per_second(picture_rate_code)?;
        let fps = u64::from(fps);
        let pictures = (display_index % fps) as u8;
        let total_seconds = display_index / fps;
        let seconds = (total_seconds % 60) as u8;
        let total_minutes = total_seconds / 60;
        let minutes = (total_minutes % 60) as u8;
        let hours = ((total_minutes / 60) % 24) as u8;
        Ok(Self {
            drop_frame: false,
            hours,
            minutes,
            seconds,
            pictures,
        })
    }
}

/// The Table 6-4 (§6.3.3) / ISO/IEC 11172-2 §2.4.3.2 picture-rate
/// table, rounded to the nearest integral number of pictures per
/// second as the `drop_frame == 0` counting rule prescribes (§6.3.8):
/// codes 1..=8 → 24, 24, 25, 30, 30, 50, 60, 60.
///
/// # Errors
/// [`Error::InvalidBitstream`] for code `0` (forbidden) or the
/// reserved codes `9..=15`.
pub fn nominal_pictures_per_second(picture_rate_code: u8) -> Result<u8> {
    Ok(match picture_rate_code {
        1 | 2 => 24, // 23,976 (24 000/1001) rounds to 24; 24 exact
        3 => 25,
        4 | 5 => 30, // 29,97 (30 000/1001) rounds to 30; 30 exact
        6 => 50,
        7 | 8 => 60, // 59,94 (60 000/1001) rounds to 60; 60 exact
        _ => {
            return Err(Error::InvalidBitstream(
                "picture_rate: only codes 1..=8 are defined (Table 6-4 / §2.4.3.2)",
            ))
        }
    })
}

/// Write a `group_of_pictures_header()` — the §6.2.2.6 / ISO/IEC
/// 11172-2 §2.4.2.4 syntax: `group_start_code`, the 25-bit `time_code`
/// (with its embedded marker bit), `closed_gop`, `broken_link`, then
/// zero-padding to the next byte boundary for `next_start_code()`.
///
/// The writer emits exactly the fields [`Mpeg2Gop::parse`] reads back;
/// the caller is responsible for the Table 6-11 / §2.4.3.3 value
/// ranges (out-of-range values would be rejected by the parser).
pub fn write_gop_header(bw: &mut BitWriter, gop: &Mpeg2Gop) {
    bw.write_u32(GROUP_START_CODE, 32);
    bw.write_bit(gop.time_code.drop_frame);
    bw.write_u32(u32::from(gop.time_code.hours), 5);
    bw.write_u32(u32::from(gop.time_code.minutes), 6);
    bw.write_bit(true); // marker_bit (Table 6-11)
    bw.write_u32(u32::from(gop.time_code.seconds), 6);
    bw.write_u32(u32::from(gop.time_code.pictures), 6);
    bw.write_bit(gop.closed_gop);
    bw.write_bit(gop.broken_link);
    bw.align_to_byte();
}

/// Parsed result of `group_of_pictures_header()` (§6.2.2.6).
///
/// The trailing `next_start_code()` byte-align + zero stuffing
/// (§5.2.3) is *not* consumed: the caller is in a better position to
/// chain into the following `picture_header()` (§6.2.3) or whatever
/// layer comes next in the elementary stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mpeg2Gop {
    /// Decoded 25-bit `time_code`.
    pub time_code: TimeCode,
    /// `closed_gop` flag (§6.3.8).
    pub closed_gop: bool,
    /// `broken_link` flag (§6.3.8).
    pub broken_link: bool,
}

impl Mpeg2Gop {
    /// Parse a `group_of_pictures_header()` from a slice that starts
    /// with the four start-code bytes `00 00 01 B8`.
    ///
    /// On success the returned struct mirrors the bitstream sub-fields
    /// from Table 6-11 plus the two GOP flags. The trailing
    /// `next_start_code()` is not consumed.
    pub fn parse(buf: &[u8]) -> Result<Self> {
        let mut br = BitReader::new(buf);

        // §6.2.2.6: 32-bit group_start_code, value 0x000001B8.
        let code = br.read_u32(32).map_err(|_| Error::ShortHeader)?;
        if code != GROUP_START_CODE {
            return Err(Error::InvalidBitstream(
                "group_start_code: expected 0x000001B8 (§6.2.2.6)",
            ));
        }

        // §6.2.2.6 / Table 6-11: 25-bit time_code.
        // Read the sub-fields in spec order so the marker_bit lands
        // between minutes and seconds.
        let drop_frame = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;

        let hours = br.read_u32(5).map_err(|_| Error::ShortHeader)? as u8;
        if hours > 23 {
            return Err(Error::InvalidBitstream(
                "time_code_hours: out of range 0..=23 (Table 6-11)",
            ));
        }

        let minutes = br.read_u32(6).map_err(|_| Error::ShortHeader)? as u8;
        if minutes > 59 {
            return Err(Error::InvalidBitstream(
                "time_code_minutes: out of range 0..=59 (Table 6-11)",
            ));
        }

        // Table 6-11 marker_bit, shall be '1'.
        let marker = br.read_u32(1).map_err(|_| Error::ShortHeader)?;
        if marker != 1 {
            return Err(Error::InvalidBitstream(
                "marker_bit in time_code: expected '1' (Table 6-11)",
            ));
        }

        let seconds = br.read_u32(6).map_err(|_| Error::ShortHeader)? as u8;
        if seconds > 59 {
            return Err(Error::InvalidBitstream(
                "time_code_seconds: out of range 0..=59 (Table 6-11)",
            ));
        }

        let pictures = br.read_u32(6).map_err(|_| Error::ShortHeader)? as u8;
        if pictures > 59 {
            return Err(Error::InvalidBitstream(
                "time_code_pictures: out of range 0..=59 (Table 6-11)",
            ));
        }

        let closed_gop = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
        let broken_link = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;

        Ok(Self {
            time_code: TimeCode {
                drop_frame,
                hours,
                minutes,
                seconds,
                pictures,
            },
            closed_gop,
            broken_link,
        })
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact `group_of_pictures_header()` fixtures plus
    //! negative cases for every spec-defined rejection site that this
    //! parser introduces.
    use super::*;

    /// Emit a `group_of_pictures_header()` through the public writer.
    /// Since the syntax consumes 32 + 25 + 1 + 1 = 59 bits, the writer
    /// pads with 5 zero bits to a byte boundary.
    #[allow(clippy::too_many_arguments)]
    fn write_gop_header(
        bw: &mut BitWriter,
        drop_frame: bool,
        hours: u32,
        minutes: u32,
        seconds: u32,
        pictures: u32,
        closed_gop: bool,
        broken_link: bool,
    ) {
        super::write_gop_header(
            bw,
            &Mpeg2Gop {
                time_code: TimeCode {
                    drop_frame,
                    hours: hours as u8,
                    minutes: minutes as u8,
                    seconds: seconds as u8,
                    pictures: pictures as u8,
                },
                closed_gop,
                broken_link,
            },
        );
    }

    #[test]
    fn parses_zero_timecode_open_gop() {
        let mut bw = BitWriter::new();
        write_gop_header(&mut bw, false, 0, 0, 0, 0, false, false);
        let bytes = bw.finish();
        let gop = Mpeg2Gop::parse(&bytes).expect("parse");
        assert_eq!(
            gop.time_code,
            TimeCode {
                drop_frame: false,
                hours: 0,
                minutes: 0,
                seconds: 0,
                pictures: 0,
            }
        );
        assert!(!gop.closed_gop);
        assert!(!gop.broken_link);
    }

    #[test]
    fn parses_full_timecode_closed_broken() {
        // A non-trivial timecode at the upper edge of every range:
        // 23:59:59;59 with drop_frame set; closed_gop=1; broken_link=1.
        let mut bw = BitWriter::new();
        write_gop_header(&mut bw, true, 23, 59, 59, 59, true, true);
        let bytes = bw.finish();
        let gop = Mpeg2Gop::parse(&bytes).expect("parse");
        assert_eq!(
            gop.time_code,
            TimeCode {
                drop_frame: true,
                hours: 23,
                minutes: 59,
                seconds: 59,
                pictures: 59,
            }
        );
        assert!(gop.closed_gop);
        assert!(gop.broken_link);
    }

    #[test]
    fn parses_mid_range_timecode() {
        // 12:34:56;15 -- a value that exercises every byte of the
        // 25-bit time_code instead of all-zero / all-one boundary.
        let mut bw = BitWriter::new();
        write_gop_header(&mut bw, false, 12, 34, 56, 15, true, false);
        let bytes = bw.finish();
        let gop = Mpeg2Gop::parse(&bytes).expect("parse");
        assert_eq!(gop.time_code.hours, 12);
        assert_eq!(gop.time_code.minutes, 34);
        assert_eq!(gop.time_code.seconds, 56);
        assert_eq!(gop.time_code.pictures, 15);
        assert!(gop.closed_gop);
        assert!(!gop.broken_link);
    }

    #[test]
    fn rejects_wrong_start_code() {
        let mut bw = BitWriter::new();
        write_gop_header(&mut bw, false, 0, 0, 0, 0, false, false);
        let mut bytes = bw.finish();
        bytes[3] = 0xB7; // sequence_end_code
        let err = Mpeg2Gop::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_hours_out_of_range() {
        // 5-bit field allows 0..=31; spec range is 0..=23. Build a
        // header with hours = 24 (smallest illegal value).
        let mut bw = BitWriter::new();
        write_gop_header(&mut bw, false, 24, 0, 0, 0, false, false);
        let bytes = bw.finish();
        let err = Mpeg2Gop::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_minutes_out_of_range() {
        // 6-bit field allows 0..=63; spec range is 0..=59.
        let mut bw = BitWriter::new();
        write_gop_header(&mut bw, false, 0, 60, 0, 0, false, false);
        let bytes = bw.finish();
        let err = Mpeg2Gop::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_seconds_out_of_range() {
        let mut bw = BitWriter::new();
        write_gop_header(&mut bw, false, 0, 0, 60, 0, false, false);
        let bytes = bw.finish();
        let err = Mpeg2Gop::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_pictures_out_of_range() {
        let mut bw = BitWriter::new();
        write_gop_header(&mut bw, false, 0, 0, 0, 60, false, false);
        let bytes = bw.finish();
        let err = Mpeg2Gop::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_zero_marker_bit() {
        // Construct manually with marker_bit forced to '0'.
        let mut bw = BitWriter::new();
        bw.write_u32(GROUP_START_CODE, 32);
        bw.write_bit(false); // drop_frame
        bw.write_u32(0, 5); // hours
        bw.write_u32(0, 6); // minutes
        bw.write_bit(false); // marker_bit forced to 0
        bw.write_u32(0, 6); // seconds
        bw.write_u32(0, 6); // pictures
        bw.write_bit(false); // closed_gop
        bw.write_bit(false); // broken_link
        bw.align_to_byte();
        let bytes = bw.finish();
        let err = Mpeg2Gop::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_truncated_buffer() {
        // Start code + one byte: parser needs 4 + 4 = 8 bytes (60 bits
        // of header + 4-bit align) so any 5-byte slice is short.
        let bytes = vec![0x00, 0x00, 0x01, 0xB8, 0x00];
        let err = Mpeg2Gop::parse(&bytes).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    #[test]
    fn captures_closed_and_broken_flag_matrix() {
        // Exercise all four combinations of the two single-bit flags.
        for (closed, broken) in [(false, false), (false, true), (true, false), (true, true)] {
            let mut bw = BitWriter::new();
            write_gop_header(&mut bw, false, 1, 2, 3, 4, closed, broken);
            let bytes = bw.finish();
            let gop = Mpeg2Gop::parse(&bytes).expect("parse");
            assert_eq!(
                gop.closed_gop, closed,
                "closed_gop bit at {closed},{broken}"
            );
            assert_eq!(
                gop.broken_link, broken,
                "broken_link bit at {closed},{broken}"
            );
        }
    }

    // ---- Writer round-trip -----------------------------------------

    #[test]
    fn writer_output_is_byte_aligned_and_parses_back() {
        // 59 syntax bits + 5 pad bits = 8 bytes exactly.
        let gop = Mpeg2Gop {
            time_code: TimeCode {
                drop_frame: false,
                hours: 7,
                minutes: 41,
                seconds: 13,
                pictures: 22,
            },
            closed_gop: true,
            broken_link: false,
        };
        let mut bw = BitWriter::new();
        super::write_gop_header(&mut bw, &gop);
        let bytes = bw.finish();
        assert_eq!(bytes.len(), 8, "59 bits pad to 8 bytes");
        assert_eq!(Mpeg2Gop::parse(&bytes).expect("parse"), gop);
    }

    // ---- TimeCode::from_display_index ------------------------------

    #[test]
    fn nominal_rates_match_table_6_4_rounding() {
        // §6.3.8 drop_frame == 0: count at the nearest integral rate.
        let expect = [
            (1u8, 24u8),
            (2, 24),
            (3, 25),
            (4, 30),
            (5, 30),
            (6, 50),
            (7, 60),
            (8, 60),
        ];
        for (code, fps) in expect {
            assert_eq!(
                nominal_pictures_per_second(code).unwrap(),
                fps,
                "code {code}"
            );
        }
        assert!(nominal_pictures_per_second(0).is_err());
        assert!(nominal_pictures_per_second(9).is_err());
        assert!(nominal_pictures_per_second(15).is_err());
    }

    #[test]
    fn from_display_index_zero_is_all_zero() {
        let tc = TimeCode::from_display_index(0, 3).unwrap();
        assert_eq!(
            tc,
            TimeCode {
                drop_frame: false,
                hours: 0,
                minutes: 0,
                seconds: 0,
                pictures: 0,
            }
        );
    }

    #[test]
    fn from_display_index_splits_at_25fps() {
        // Display frame 3661*25 + 7 = 1 h 1 min 1 s, picture 7 at 25 fps.
        let idx = (3600u64 + 60 + 1) * 25 + 7;
        let tc = TimeCode::from_display_index(idx, 3).unwrap();
        assert_eq!(
            (tc.hours, tc.minutes, tc.seconds, tc.pictures),
            (1, 1, 1, 7)
        );
        assert!(!tc.drop_frame);
    }

    #[test]
    fn from_display_index_counts_29_97_as_30() {
        // Code 4 (29,97 Hz) counts as 30 pictures/s with drop_frame 0.
        let tc = TimeCode::from_display_index(30, 4).unwrap();
        assert_eq!((tc.seconds, tc.pictures), (1, 0));
        let tc = TimeCode::from_display_index(29, 4).unwrap();
        assert_eq!((tc.seconds, tc.pictures), (0, 29));
    }

    #[test]
    fn from_display_index_hours_wrap_modulo_24() {
        // 25 h at 24 fps: hours field wraps into the 0..=23 range.
        let idx = 25u64 * 3600 * 24;
        let tc = TimeCode::from_display_index(idx, 2).unwrap();
        assert_eq!(tc.hours, 1);
        assert_eq!((tc.minutes, tc.seconds, tc.pictures), (0, 0, 0));
    }

    #[test]
    fn from_display_index_rejects_undefined_rate_codes() {
        assert!(TimeCode::from_display_index(0, 0).is_err());
        assert!(TimeCode::from_display_index(0, 9).is_err());
    }

    #[test]
    fn from_display_index_always_parses_back() {
        // Every derived time code is in the Table 6-11 ranges, so the
        // writer→parser loop must accept it. Spot a spread of indices
        // and rates.
        for &(idx, code) in &[
            (0u64, 3u8),
            (1, 1),
            (59, 8),
            (60, 8),
            (89_999, 4),
            (90_000, 5),
            (12_345_678, 6),
        ] {
            let tc = TimeCode::from_display_index(idx, code).unwrap();
            let gop = Mpeg2Gop {
                time_code: tc,
                closed_gop: false,
                broken_link: false,
            };
            let mut bw = BitWriter::new();
            super::write_gop_header(&mut bw, &gop);
            let bytes = bw.finish();
            assert_eq!(
                Mpeg2Gop::parse(&bytes).expect("parse"),
                gop,
                "idx {idx} code {code}"
            );
        }
    }

    #[test]
    fn debug_impl_smoke() {
        let mut bw = BitWriter::new();
        write_gop_header(&mut bw, true, 1, 2, 3, 4, true, false);
        let bytes = bw.finish();
        let gop = Mpeg2Gop::parse(&bytes).expect("parse");
        let s = format!("{gop:?}");
        // Debug renderer must mention the type names.
        assert!(s.contains("TimeCode"));
        assert!(s.contains("Mpeg2Gop") || s.contains("closed_gop"));
    }
}
