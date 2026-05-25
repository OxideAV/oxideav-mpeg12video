//! MPEG-1 (ISO/IEC 11172-2:1993) `motion_vector(s)` parsing for the
//! `forward` and `backward` macroblock motion-vector fields per §2.4.2.7
//! (syntax) and §2.4.3.6 (field semantics), driven by Annex B Table B.4
//! (`motion_code` VLC).
//!
//! MPEG-1's motion-vector layer is markedly simpler than MPEG-2's
//! §6.2.5.2: the one-bit `motion_vertical_field_select` flag is absent,
//! the `mv_format` / `dmv` macroblock-mode toggles do not exist (frame
//! pictures and full-frame motion-compensation only — no field-of-frame
//! prediction, no 16×8, no dual-prime), and a "set" of motion vectors
//! is exactly one (horizontal, vertical) pair per direction. The wire
//! shape of `motion_vector(s)` is therefore:
//!
//! ```text
//! motion_horizontal_<dir>_code   1-11 vlclbf  (Table B.4)
//! if (<dir>_f != 1 && motion_horizontal_<dir>_code != 0)
//!     motion_horizontal_<dir>_r  1-7  uimsbf  (<dir>_r_size bits)
//! motion_vertical_<dir>_code     1-11 vlclbf  (Table B.4)
//! if (<dir>_f != 1 && motion_vertical_<dir>_code != 0)
//!     motion_vertical_<dir>_r    1-7  uimsbf  (<dir>_r_size bits)
//! ```
//!
//! where `<dir>` is `forward` or `backward`, the residual width comes
//! from `<dir>_r_size = <dir>_f_code - 1`, and `<dir>_f = 1 <<
//! <dir>_r_size` (§2.4.4.2 / §2.4.4.3). MPEG-1's `forward_f_code` /
//! `backward_f_code` are constrained to `1..=7` (§2.4.3.4: "An unsigned
//! integer taking values 1 through 7. The value zero is forbidden.").
//!
//! Table B.4 (single VLC reused for all four `motion_*_code` fields per
//! the column header of B.4) decodes a signed value in `-16..=16` from
//! a 1- to 11-bit codeword. The numerical mapping matches MPEG-2 Annex
//! B Table B-10 row-for-row; the [`match_motion_code`] walker is shared
//! via a `pub(crate)` accessor in [`crate::motion_vector`] so the
//! per-row constants do not get retyped.
//!
//! This module's contract ends at "the bits the §2.4.2.7 syntax says
//! are present have been read into typed fields and the cursor has
//! advanced exactly that far". The §2.4.4.2 reconstruction of
//! `recon_right_for` / `recon_down_for` (and the §2.4.4.3 backward
//! analogues) — which folds in the predictor state and the
//! `full_pel_*_vector` flag — is the next-round concern, mirroring the
//! split between MPEG-2 §6.2.5.2 (parser) and §7.6.3.1
//! (reconstruction).
//!
//! Spec citations refer to ISO/IEC 11172-2:1993 (MPEG-1 Video) §2.4.2.7,
//! §2.4.3.4, §2.4.3.6, §2.4.4.2, §2.4.4.3, Annex B Table B.4.

// Table B.4 codewords are transcribed (in the test module) with the
// uneven bit-group widths the spec prints (e.g. `0b0000_0011_001` for
// the 11-bit `-16` entry, `0b0000_0011_000` for `+16`). clippy's
// `unusual_byte_groupings` prefers equal 4-bit groups, which would
// obscure the visual mapping back to the printed Annex B table.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitReader;

use crate::motion_vector::match_motion_code;
use crate::{Error, Result};

/// `s` index — which direction this `motion_vector(s)` element decodes.
/// MPEG-1 §2.4.2.7 lists the forward and backward pairs side by side
/// under the `if (macroblock_motion_forward)` / `if
/// (macroblock_motion_backward)` macroblock-type gates; the same wire
/// shape covers both, parameterised on the matching `<dir>_f_code` and
/// `full_pel_<dir>_vector` flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mpeg1MotionDirection {
    /// `forward` — references the past picture (P- and B-pictures).
    Forward,
    /// `backward` — references the future picture (B-pictures only).
    Backward,
}

/// One parsed `motion_vector(s)` element per ISO/IEC 11172-2 §2.4.2.7 —
/// a `(motion_*_code, motion_*_r)` pair per component.
///
/// All four fields here are raw bitstream values. The signed
/// `*_code` ranges over `-16..=16` per Table B.4; the unsigned `*_r`
/// values range over `0..(1 << r_size)` and are only present when the
/// matching residual gate `f != 1 && code != 0` from §2.4.3.6 fires
/// (otherwise [`Option::None`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mpeg1MotionVector {
    /// Which direction (forward or backward) this vector decodes.
    pub direction: Mpeg1MotionDirection,
    /// `motion_horizontal_<dir>_code` decoded against Table B.4
    /// (`-16..=16`).
    pub horizontal_code: i8,
    /// `motion_horizontal_<dir>_r` — `Some(r)` when `<dir>_f != 1 &&
    /// horizontal_code != 0`, else `None`. Width is `<dir>_r_size =
    /// <dir>_f_code - 1` bits (range `1..=6` since `f_code ∈ 1..=7`).
    pub horizontal_r: Option<u8>,
    /// `motion_vertical_<dir>_code` decoded against Table B.4
    /// (`-16..=16`).
    pub vertical_code: i8,
    /// `motion_vertical_<dir>_r` — `Some(r)` when `<dir>_f != 1 &&
    /// vertical_code != 0`, else `None`.
    pub vertical_r: Option<u8>,
    /// Bit position right after the consumed bits of this
    /// `motion_vector(s)` call.
    pub bit_position_after: u64,
}

impl Mpeg1MotionVector {
    /// Parse one `motion_vector(s)` element per ISO/IEC 11172-2
    /// §2.4.2.7 for the requested `direction`.
    ///
    /// `f_code` is the matching `forward_f_code` or `backward_f_code`
    /// from the surrounding `picture_header()` (§2.4.2.3 / §2.4.3.4):
    /// an unsigned integer in `1..=7`. Zero is rejected here mirroring
    /// the spec's "zero is forbidden" guard; values above `7` are
    /// rejected as the unused range. (The `r_size = f_code - 1` field
    /// width otherwise would overflow the §2.4.3.6 "1 through 7" bound
    /// on `r_size + 1` and bit-shift past the spec's residual range.)
    ///
    /// The residual presence rule comes from §2.4.3.6:
    /// `motion_horizontal_<dir>_r` (resp. `motion_vertical_<dir>_r`)
    /// appears iff `<dir>_f != 1 && motion_horizontal_<dir>_code != 0`
    /// (resp. `motion_vertical_<dir>_code != 0`). With `<dir>_f = 1 <<
    /// (<dir>_f_code - 1)`, `<dir>_f == 1` iff `<dir>_f_code == 1`,
    /// matching the spec text "If `forward_f_code == 1`, then no
    /// further bits are needed beyond the `motion_*_forward_code`".
    ///
    /// Errors:
    /// * [`Error::InvalidBitstream`] if the `f_code` is out of range
    ///   (`!= 1..=7`) or a `motion_*_code` prefix matches no Table B.4
    ///   row.
    /// * [`Error::ShortHeader`] if the bitstream ends mid-element.
    pub fn parse(
        br: &mut BitReader<'_>,
        direction: Mpeg1MotionDirection,
        f_code: u8,
    ) -> Result<Self> {
        // §2.4.3.4: forward_f_code / backward_f_code take values
        // 1 through 7; zero is forbidden, and values above 7 are
        // outside the spec's range.
        if !(1..=7).contains(&f_code) {
            return Err(Error::InvalidBitstream(
                "mpeg1 motion_vector: f_code outside the §2.4.3.4 1..=7 range",
            ));
        }
        let r_size = u32::from(f_code - 1);
        let horizontal_code = match_motion_code(br)?;
        let horizontal_r = read_residual(br, f_code, r_size, horizontal_code)?;
        let vertical_code = match_motion_code(br)?;
        let vertical_r = read_residual(br, f_code, r_size, vertical_code)?;

        Ok(Self {
            direction,
            horizontal_code,
            horizontal_r,
            vertical_code,
            vertical_r,
            bit_position_after: br.bit_position(),
        })
    }
}

/// Read the `r_size`-bit residual when §2.4.3.6 says it should be
/// present (`f != 1 && code != 0`), or return `None`.
fn read_residual(br: &mut BitReader<'_>, f_code: u8, r_size: u32, code: i8) -> Result<Option<u8>> {
    if f_code == 1 || code == 0 {
        return Ok(None);
    }
    // r_size ∈ 1..=6 since f_code ∈ 2..=7 here, so the read width fits
    // in a `u8`.
    let r = br.read_u32(r_size).map_err(|_| Error::ShortHeader)? as u8;
    Ok(Some(r))
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact tests pinning Table B.4 row by row and the
    //! §2.4.2.7 / §2.4.3.6 presence matrix.
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// All 33 entries of Table B.4 in spec order, transcribed from
    /// ISO/IEC 11172-2:1993 Annex B page 43. Used by the per-row test
    /// to confirm every codeword decodes to its tabulated signed value
    /// with the exact bit width the spec prints.
    const TABLE_B4: &[(u32, u8, i8)] = &[
        // (code, bits, value)
        (0b0000_0011_001, 11, -16),
        (0b0000_0011_011, 11, -15),
        (0b0000_0011_101, 11, -14),
        (0b0000_0011_111, 11, -13),
        (0b0000_0100_001, 11, -12),
        (0b0000_0100_011, 11, -11),
        (0b0000_0100_11, 10, -10),
        (0b0000_0101_01, 10, -9),
        (0b0000_0101_11, 10, -8),
        (0b0000_0111, 8, -7),
        (0b0000_1001, 8, -6),
        (0b0000_1011, 8, -5),
        (0b0000_111, 7, -4),
        (0b0001_1, 5, -3),
        (0b0011, 4, -2),
        (0b011, 3, -1),
        (0b1, 1, 0),
        (0b010, 3, 1),
        (0b0010, 4, 2),
        (0b0001_0, 5, 3),
        (0b0000_110, 7, 4),
        (0b0000_1010, 8, 5),
        (0b0000_1000, 8, 6),
        (0b0000_0110, 8, 7),
        (0b0000_0101_10, 10, 8),
        (0b0000_0101_00, 10, 9),
        (0b0000_0100_10, 10, 10),
        (0b0000_0100_010, 11, 11),
        (0b0000_0100_000, 11, 12),
        (0b0000_0011_110, 11, 13),
        (0b0000_0011_100, 11, 14),
        (0b0000_0011_010, 11, 15),
        (0b0000_0011_000, 11, 16),
    ];

    fn buf(codes: &[(u32, u32)]) -> Vec<u8> {
        let mut bw = BitWriter::new();
        for &(code, n) in codes {
            bw.write_u32(code, n);
        }
        // Padding so the BitReader doesn't run dry mid-element when a
        // test only writes the syntactic minimum. The padding bit `1`
        // also doubles as the "code 0" entry of Table B.4 for tests
        // that need a follow-up motion_code.
        bw.write_bit(true);
        bw.align_to_byte();
        bw.finish()
    }

    #[test]
    fn table_b4_has_33_unique_entries_covering_minus16_to_plus16() {
        // Sanity check on the transcription: 33 rows, every value in
        // -16..=+16 exactly once.
        assert_eq!(TABLE_B4.len(), 33);
        let mut values: Vec<i8> = TABLE_B4.iter().map(|&(_, _, v)| v).collect();
        values.sort_unstable();
        let expected: Vec<i8> = (-16..=16).collect();
        assert_eq!(values, expected);
    }

    #[test]
    fn table_b4_every_row_decodes_via_match_motion_code() {
        for &(code, bits, value) in TABLE_B4 {
            let data = buf(&[(code, u32::from(bits))]);
            let mut br = BitReader::new(&data);
            let got = match_motion_code(&mut br)
                .unwrap_or_else(|err| panic!("row value={value} bits={bits} failed: {err:?}"));
            assert_eq!(got, value, "row value={value} bits={bits} code={code:b}");
            assert_eq!(br.bit_position(), u64::from(bits));
        }
    }

    #[test]
    fn table_b4_zero_is_one_bit() {
        let data = buf(&[(0b1, 1)]);
        let mut br = BitReader::new(&data);
        assert_eq!(match_motion_code(&mut br).unwrap(), 0);
        assert_eq!(br.bit_position(), 1);
    }

    #[test]
    fn table_b4_extremes_minus16_and_plus16() {
        let neg = buf(&[(0b0000_0011_001, 11)]);
        let mut br = BitReader::new(&neg);
        assert_eq!(match_motion_code(&mut br).unwrap(), -16);

        let pos = buf(&[(0b0000_0011_000, 11)]);
        let mut br = BitReader::new(&pos);
        assert_eq!(match_motion_code(&mut br).unwrap(), 16);
    }

    #[test]
    fn table_b4_unknown_prefix_rejected() {
        // 11 zero bits: not in Table B.4 (any genuine entry has a
        // trailing '1' somewhere in its first 11 bits).
        let data = buf(&[(0, 11)]);
        let mut br = BitReader::new(&data);
        let err = match_motion_code(&mut br).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn parse_rejects_f_code_zero() {
        let data = buf(&[(0b1, 1), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let err = Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Forward, 0).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn parse_rejects_f_code_above_seven() {
        // f_code = 8 is past the §2.4.3.4 "1 through 7" range.
        let data = buf(&[(0b1, 1), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let err = Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Forward, 8).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn parse_minimal_f_code_one_no_residual() {
        // f_code = 1 → forward_f = 1 → §2.4.3.6 says no `*_r` field
        // regardless of the code value. Two `1` bits = two code-zero
        // motion vectors.
        let data = buf(&[(0b1, 1), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let mv =
            Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Forward, 1).expect("parse");
        assert_eq!(mv.direction, Mpeg1MotionDirection::Forward);
        assert_eq!(mv.horizontal_code, 0);
        assert_eq!(mv.horizontal_r, None);
        assert_eq!(mv.vertical_code, 0);
        assert_eq!(mv.vertical_r, None);
        assert_eq!(mv.bit_position_after, 2);
    }

    #[test]
    fn parse_residual_only_when_code_nonzero_and_f_code_gt_one() {
        // f_code = 2 → r_size = 1. horizontal_code = -1 (3 bits '011')
        // → residual present (1 bit '1' → 1). vertical_code = 0 →
        // residual absent.
        let data = buf(&[(0b011, 3), (0b1, 1), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let mv =
            Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Forward, 2).expect("parse");
        assert_eq!(mv.horizontal_code, -1);
        assert_eq!(mv.horizontal_r, Some(1));
        assert_eq!(mv.vertical_code, 0);
        assert_eq!(mv.vertical_r, None);
        // 3 (horiz code) + 1 (horiz residual) + 1 (vert code) = 5 bits.
        assert_eq!(mv.bit_position_after, 5);
    }

    #[test]
    fn parse_residual_width_tracks_f_code() {
        // f_code = 5 → r_size = 4. horizontal_code = +1 ('010', 3 bits),
        // horizontal_r = 4 bits '1010' = 10. vertical_code = 0 → no r.
        let data = buf(&[(0b010, 3), (0b1010, 4), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let mv =
            Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Forward, 5).expect("parse");
        assert_eq!(mv.horizontal_code, 1);
        assert_eq!(mv.horizontal_r, Some(10));
        assert_eq!(mv.vertical_code, 0);
        assert_eq!(mv.vertical_r, None);
        assert_eq!(mv.bit_position_after, 8);
    }

    #[test]
    fn parse_residual_absent_when_motion_code_zero_even_with_large_f_code() {
        // f_code = 7 → r_size = 6. Both motion codes are zero so
        // §2.4.3.6 suppresses both residuals despite f != 1.
        let data = buf(&[(0b1, 1), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let mv =
            Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Forward, 7).expect("parse");
        assert_eq!(mv.horizontal_code, 0);
        assert_eq!(mv.horizontal_r, None);
        assert_eq!(mv.vertical_code, 0);
        assert_eq!(mv.vertical_r, None);
        assert_eq!(mv.bit_position_after, 2);
    }

    #[test]
    fn parse_widest_residual_six_bits_when_f_code_is_seven() {
        // f_code = 7 → r_size = 6. horizontal_code = -16 (11 bits),
        // horizontal_r = 6 bits all-ones = 63. vertical_code = +16
        // (11 bits), vertical_r = 6 bits all-zeros = 0.
        let data = buf(&[
            (0b0000_0011_001, 11),
            (0b111111, 6),
            (0b0000_0011_000, 11),
            (0b000000, 6),
        ]);
        let mut br = BitReader::new(&data);
        let mv =
            Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Backward, 7).expect("parse");
        assert_eq!(mv.direction, Mpeg1MotionDirection::Backward);
        assert_eq!(mv.horizontal_code, -16);
        assert_eq!(mv.horizontal_r, Some(63));
        assert_eq!(mv.vertical_code, 16);
        assert_eq!(mv.vertical_r, Some(0));
        // 11 + 6 + 11 + 6 = 34 bits.
        assert_eq!(mv.bit_position_after, 34);
    }

    #[test]
    fn parse_truncated_horizontal_code_is_invalid_bitstream() {
        // 11 zeros = no Table B.4 row matches → InvalidBitstream.
        let data = [0u8; 2];
        let mut br = BitReader::new(&data);
        let err = Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Forward, 1).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn parse_truncated_residual_is_short() {
        // f_code = 7 → r_size = 6. horizontal_code = -1 (3 bits '011').
        // Put just the 3-bit code into a single-byte buffer (5 zero
        // pad bits left → 5 bits remain) and pre-skip so we leave only
        // 4 bits ahead — read_u32(6) then runs short.
        let mut bw = BitWriter::new();
        bw.write_u32(0b011, 3);
        bw.align_to_byte();
        let mut data_vec = bw.finish();
        // Truncate trailing padding so the BitReader genuinely runs
        // out of bits when read_u32(6) tries to read the residual.
        // After the 3-bit code we have 5 zero pad bits in the single
        // byte; read_u32(6) needs 6 — exactly one bit short.
        data_vec.truncate(1);
        let mut br = BitReader::new(&data_vec);
        let err = Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Forward, 7).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    #[test]
    fn parse_backward_direction_records_backward_tag() {
        let data = buf(&[(0b1, 1), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let mv =
            Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Backward, 1).expect("parse");
        assert_eq!(mv.direction, Mpeg1MotionDirection::Backward);
        assert_eq!(mv.horizontal_code, 0);
        assert_eq!(mv.vertical_code, 0);
    }

    #[test]
    fn parse_both_components_can_carry_residual() {
        // f_code = 3 → r_size = 2. Both codes non-zero so both
        // residuals appear: horiz_code = -1 ('011'), horiz_r = '10' =
        // 2; vert_code = +1 ('010'), vert_r = '01' = 1.
        let data = buf(&[(0b011, 3), (0b10, 2), (0b010, 3), (0b01, 2)]);
        let mut br = BitReader::new(&data);
        let mv =
            Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Forward, 3).expect("parse");
        assert_eq!(mv.horizontal_code, -1);
        assert_eq!(mv.horizontal_r, Some(2));
        assert_eq!(mv.vertical_code, 1);
        assert_eq!(mv.vertical_r, Some(1));
        // 3 + 2 + 3 + 2 = 10 bits.
        assert_eq!(mv.bit_position_after, 10);
    }

    #[test]
    fn parse_mixed_components_one_residual_then_one_bare() {
        // f_code = 4 → r_size = 3. horiz_code = +2 ('0010'), horiz_r =
        // '101' = 5; vert_code = 0 (1 bit '1'), no vert_r.
        let data = buf(&[(0b0010, 4), (0b101, 3), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let mv =
            Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Forward, 4).expect("parse");
        assert_eq!(mv.horizontal_code, 2);
        assert_eq!(mv.horizontal_r, Some(5));
        assert_eq!(mv.vertical_code, 0);
        assert_eq!(mv.vertical_r, None);
        // 4 + 3 + 1 = 8 bits.
        assert_eq!(mv.bit_position_after, 8);
    }

    #[test]
    fn parse_zero_horiz_nonzero_vert_skips_horiz_residual_only() {
        // f_code = 4 → r_size = 3. horiz_code = 0 (1 bit '1'), no
        // horiz_r. vert_code = -2 ('0011'), vert_r = '110' = 6.
        let data = buf(&[(0b1, 1), (0b0011, 4), (0b110, 3)]);
        let mut br = BitReader::new(&data);
        let mv =
            Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Forward, 4).expect("parse");
        assert_eq!(mv.horizontal_code, 0);
        assert_eq!(mv.horizontal_r, None);
        assert_eq!(mv.vertical_code, -2);
        assert_eq!(mv.vertical_r, Some(6));
        // 1 + 4 + 3 = 8 bits.
        assert_eq!(mv.bit_position_after, 8);
    }

    #[test]
    fn direction_debug_smoke() {
        let f = format!("{:?}", Mpeg1MotionDirection::Forward);
        assert!(f.contains("Forward"));
        let b = format!("{:?}", Mpeg1MotionDirection::Backward);
        assert!(b.contains("Backward"));
    }

    #[test]
    fn motion_vector_debug_smoke() {
        let mv = Mpeg1MotionVector {
            direction: Mpeg1MotionDirection::Forward,
            horizontal_code: -3,
            horizontal_r: Some(1),
            vertical_code: 0,
            vertical_r: None,
            bit_position_after: 7,
        };
        let s = format!("{mv:?}");
        assert!(s.contains("Mpeg1MotionVector"));
        assert!(s.contains("Forward"));
        assert!(s.contains("-3"));
    }
}
