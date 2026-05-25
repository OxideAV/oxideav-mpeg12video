//! Parsers for the `motion_vectors()` / `motion_vector()` syntax of
//! ISO/IEC 13818-2 (Recommendation ITU-T H.262) §6.2.5.2 and §6.2.5.2.1
//! together with the underlying Annex B Tables B-10 (`motion_code`) and
//! B-11 (`dmvector`).
//!
//! Round 10 closed `macroblock_modes()`; round 11 fills the next gap in
//! the macroblock body — the motion-vector fields proper. The residual
//! block layer (the Tables B-12..B-16 DCT-coefficient VLCs and the IDCT)
//! remains out of scope.
//!
//! The §6.2.5.2 wrapper picks one or two `motion_vector(r, s)` calls
//! based on `motion_vector_count` and toggles the optional
//! `motion_vertical_field_select[r][s]` flag based on `mv_format` and
//! `dmv`. The §6.2.5.2.1 inner reads, for each component `t ∈ {0, 1}`:
//!
//! ```text
//! motion_code[r][s][t]                       1-11   vlclbf   (Table B-10)
//! if (f_code[s][t] != 1 && motion_code[r][s][t] != 0)
//!     motion_residual[r][s][t]                1-8    uimsbf   (r_size = f_code-1 bits)
//! if (dmv == 1)
//!     dmvector[t]                             1-2    vlclbf   (Table B-11)
//! ```
//!
//! The numerical reconstruction of `vector'[r][s][t]` from
//! `motion_code` + `motion_residual` + prior PMV (§7.6.3.1) is **not**
//! performed by this module — it is the next-round concern. This
//! module's contract ends at "the bits the syntax says are present have
//! been read into typed Option-tagged fields and the cursor has
//! advanced exactly that far". Reconstruction needs PMV state we don't
//! carry yet.
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)) §6.2.5.2, §6.2.5.2.1,
//! §6.3.17.2, §6.3.17.3, and Annex B Tables B-10 and B-11.

// Bit-group widths match the spec's MSB-first visual layout of
// Tables B-10 and B-11 (e.g. `0b0000_0011_001` for the 11-bit
// motion_code -16 entry), so an audit can read each constant
// against the printed table at a glance. clippy's
// `unusual_byte_groupings` lint prefers equal-size 4-bit groups,
// which would obscure the spec mapping.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitReader;

use crate::macroblock_modes::{MotionType, MvFormat};
use crate::{Error, Result};

/// One row of Table B-10: a right-justified MSB-first VLC code, its bit
/// length, and the signed `motion_code` value it decodes to.
#[derive(Debug, Clone, Copy)]
struct MotionCodeEntry {
    /// VLC code right-justified into a `u16`.
    code: u16,
    /// Length of `code` in bits (`1..=11` for Table B-10).
    bits: u8,
    /// Signed motion code value, range `-16..=16`.
    value: i8,
}

/// Table B-10 in spec order. The walker scans longest-first so that a
/// shorter code can never falsely match the high bits of a longer one.
const TABLE_B10: &[MotionCodeEntry] = &[
    // --- 11-bit codes (negative half) ---
    MotionCodeEntry {
        code: 0b0000_0011_001,
        bits: 11,
        value: -16,
    },
    MotionCodeEntry {
        code: 0b0000_0011_011,
        bits: 11,
        value: -15,
    },
    MotionCodeEntry {
        code: 0b0000_0011_101,
        bits: 11,
        value: -14,
    },
    MotionCodeEntry {
        code: 0b0000_0011_111,
        bits: 11,
        value: -13,
    },
    MotionCodeEntry {
        code: 0b0000_0100_001,
        bits: 11,
        value: -12,
    },
    MotionCodeEntry {
        code: 0b0000_0100_011,
        bits: 11,
        value: -11,
    },
    // --- 10-bit codes (negative half) ---
    MotionCodeEntry {
        code: 0b0000_0100_11,
        bits: 10,
        value: -10,
    },
    MotionCodeEntry {
        code: 0b0000_0101_01,
        bits: 10,
        value: -9,
    },
    MotionCodeEntry {
        code: 0b0000_0101_11,
        bits: 10,
        value: -8,
    },
    // --- 8-bit codes (negative half) ---
    MotionCodeEntry {
        code: 0b0000_0111,
        bits: 8,
        value: -7,
    },
    MotionCodeEntry {
        code: 0b0000_1001,
        bits: 8,
        value: -6,
    },
    MotionCodeEntry {
        code: 0b0000_1011,
        bits: 8,
        value: -5,
    },
    // --- 7-bit code ---
    MotionCodeEntry {
        code: 0b0000_111,
        bits: 7,
        value: -4,
    },
    // --- 5-bit code ---
    MotionCodeEntry {
        code: 0b0001_1,
        bits: 5,
        value: -3,
    },
    // --- 4-bit code ---
    MotionCodeEntry {
        code: 0b0011,
        bits: 4,
        value: -2,
    },
    // --- 3-bit codes ---
    MotionCodeEntry {
        code: 0b011,
        bits: 3,
        value: -1,
    },
    // --- 1-bit code: zero ---
    MotionCodeEntry {
        code: 0b1,
        bits: 1,
        value: 0,
    },
    // --- 3-bit code ---
    MotionCodeEntry {
        code: 0b010,
        bits: 3,
        value: 1,
    },
    // --- 4-bit code ---
    MotionCodeEntry {
        code: 0b0010,
        bits: 4,
        value: 2,
    },
    // --- 5-bit code ---
    MotionCodeEntry {
        code: 0b0001_0,
        bits: 5,
        value: 3,
    },
    // --- 7-bit code ---
    MotionCodeEntry {
        code: 0b0000_110,
        bits: 7,
        value: 4,
    },
    // --- 8-bit codes (positive half) ---
    MotionCodeEntry {
        code: 0b0000_1010,
        bits: 8,
        value: 5,
    },
    MotionCodeEntry {
        code: 0b0000_1000,
        bits: 8,
        value: 6,
    },
    MotionCodeEntry {
        code: 0b0000_0110,
        bits: 8,
        value: 7,
    },
    // --- 10-bit codes (positive half) ---
    MotionCodeEntry {
        code: 0b0000_0101_10,
        bits: 10,
        value: 8,
    },
    MotionCodeEntry {
        code: 0b0000_0101_00,
        bits: 10,
        value: 9,
    },
    MotionCodeEntry {
        code: 0b0000_0100_10,
        bits: 10,
        value: 10,
    },
    // --- 11-bit codes (positive half) ---
    MotionCodeEntry {
        code: 0b0000_0100_010,
        bits: 11,
        value: 11,
    },
    MotionCodeEntry {
        code: 0b0000_0100_000,
        bits: 11,
        value: 12,
    },
    MotionCodeEntry {
        code: 0b0000_0011_110,
        bits: 11,
        value: 13,
    },
    MotionCodeEntry {
        code: 0b0000_0011_100,
        bits: 11,
        value: 14,
    },
    MotionCodeEntry {
        code: 0b0000_0011_010,
        bits: 11,
        value: 15,
    },
    MotionCodeEntry {
        code: 0b0000_0011_000,
        bits: 11,
        value: 16,
    },
];

/// Walk Table B-10 longest-first and return the matching entry's signed
/// `motion_code` value. Bits are consumed iff a match is found.
///
/// MPEG-1 Table B.4 of ISO/IEC 11172-2:1993 lists the same 33-entry
/// codeword → signed-value mapping as MPEG-2 Annex B Table B-10. This
/// `pub(crate)` accessor lets the MPEG-1 parser ([`crate::mpeg1_motion_vector`])
/// reuse the walker by Table B.4 citation without duplicating the data
/// constants.
pub(crate) fn match_motion_code(br: &mut BitReader<'_>) -> Result<i8> {
    match_b10(br).map(|entry| entry.value)
}

/// Walk Table B-10 longest-first and return the matching entry. Bits are
/// consumed iff a match is found.
fn match_b10(br: &mut BitReader<'_>) -> Result<MotionCodeEntry> {
    for &width in &[11u8, 10, 8, 7, 5, 4, 3, 1] {
        if br.bits_remaining() < u64::from(width) {
            continue;
        }
        let peeked = br
            .peek_u32(u32::from(width))
            .map_err(|_| Error::ShortHeader)? as u16;
        for &entry in TABLE_B10.iter().filter(|e| e.bits == width) {
            if entry.code == peeked {
                br.consume(u32::from(width))
                    .map_err(|_| Error::ShortHeader)?;
                return Ok(entry);
            }
        }
    }
    Err(Error::InvalidBitstream(
        "motion_code: no Table B-10 codeword matches the bit prefix (§6.2.5.2.1)",
    ))
}

/// Walk Table B-11 (`dmvector[t]`) and return the signed value.
/// Table B-11 entries (per §B.4 of the 1995 spec):
///
/// ```text
/// 11 -> -1
///  0 ->  0
/// 10 -> +1
/// ```
fn match_b11(br: &mut BitReader<'_>) -> Result<i8> {
    // First bit '0' is the standalone `0` value.
    let first = br.read_bit().map_err(|_| Error::ShortHeader)?;
    if !first {
        return Ok(0);
    }
    let second = br.read_bit().map_err(|_| Error::ShortHeader)?;
    Ok(if second { -1 } else { 1 })
}

/// A decoded `motion_vector(r, s)` per §6.2.5.2.1 — the per-component
/// `motion_code` / `motion_residual` / `dmvector` triplets for the
/// horizontal (`t = 0`) and vertical (`t = 1`) components.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotionVector {
    /// `motion_code[r][s][0]` decoded against Table B-10 (`-16..=16`).
    pub motion_code_horiz: i8,
    /// `motion_residual[r][s][0]` when `f_code[s][0] != 1 &&
    /// motion_code != 0`, else `None`. Bit-width is `f_code[s][0] - 1`
    /// (range `1..=8`).
    pub motion_residual_horiz: Option<u8>,
    /// `dmvector[0]` when the surrounding macroblock is Dual-Prime
    /// (`dmv == 1`), else `None`. Values in `{-1, 0, +1}`.
    pub dmvector_horiz: Option<i8>,
    /// `motion_code[r][s][1]`.
    pub motion_code_vert: i8,
    /// `motion_residual[r][s][1]`.
    pub motion_residual_vert: Option<u8>,
    /// `dmvector[1]`.
    pub dmvector_vert: Option<i8>,
    /// Bit position right after the consumed bits of this
    /// `motion_vector(r, s)` call.
    pub bit_position_after: u64,
}

impl MotionVector {
    /// Parse a `motion_vector(r, s)` from the bitstream per §6.2.5.2.1.
    ///
    /// `f_code_horiz` is `f_code[s][0]`, `f_code_vert` is `f_code[s][1]`;
    /// `dmv` is the macroblock-level Dual-Prime flag (derived from the
    /// `frame_motion_type` / `field_motion_type` via Tables 6-17 / 6-18).
    ///
    /// The §6.3.11 forbidden-zero / value-15-unused contract on
    /// `f_code[s][t]` is the upstream parser's responsibility; this
    /// function only enforces that the residual width is non-zero (i.e.
    /// `f_code >= 2`) before reading bits, mirroring the spec's
    /// `f_code != 1` gate.
    ///
    /// Errors:
    /// * [`Error::InvalidBitstream`] if a `motion_code` prefix does not
    ///   match any Table B-10 codeword.
    /// * [`Error::ShortHeader`] if the bitstream ends before a required
    ///   field could be read.
    pub fn parse(
        br: &mut BitReader<'_>,
        f_code_horiz: u8,
        f_code_vert: u8,
        dmv: bool,
    ) -> Result<Self> {
        let (motion_code_horiz, motion_residual_horiz, dmvector_horiz) =
            Self::parse_component(br, f_code_horiz, dmv)?;
        let (motion_code_vert, motion_residual_vert, dmvector_vert) =
            Self::parse_component(br, f_code_vert, dmv)?;

        Ok(Self {
            motion_code_horiz,
            motion_residual_horiz,
            dmvector_horiz,
            motion_code_vert,
            motion_residual_vert,
            dmvector_vert,
            bit_position_after: br.bit_position(),
        })
    }

    /// One `t`-loop iteration of `motion_vector(r, s)` from §6.2.5.2.1.
    fn parse_component(
        br: &mut BitReader<'_>,
        f_code: u8,
        dmv: bool,
    ) -> Result<(i8, Option<u8>, Option<i8>)> {
        let entry = match_b10(br)?;
        let motion_code = entry.value;

        // motion_residual present iff f_code != 1 && motion_code != 0.
        let motion_residual = if f_code != 1 && motion_code != 0 {
            // r_size = f_code - 1, the bit width of motion_residual.
            // f_code is constrained by §6.3.11 to `1..=9` (15 = unused);
            // here we treat any f_code <= 9 as legal and reject the
            // overflowing widths defensively.
            if !(2..=9).contains(&f_code) {
                return Err(Error::InvalidBitstream(
                    "motion_vector: f_code outside the §6.3.11 1..=9 range cannot drive motion_residual width",
                ));
            }
            let r_size = u32::from(f_code - 1);
            let residual = br.read_u32(r_size).map_err(|_| Error::ShortHeader)? as u8;
            Some(residual)
        } else {
            None
        };

        let dmvector = if dmv { Some(match_b11(br)?) } else { None };

        Ok((motion_code, motion_residual, dmvector))
    }
}

/// `s`-index — the `motion_vectors(s)` argument selecting between the
/// forward (`s = 0`) and backward (`s = 1`) reference (Table 7-7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionVectorsKind {
    /// Forward prediction (`s = 0`).
    Forward,
    /// Backward prediction (`s = 1`). Only used inside B-pictures.
    Backward,
}

/// Caller-supplied state that gates which fields of `motion_vectors(s)`
/// are present (§6.2.5.2). All four `f_code` values are picked up here
/// rather than per-call so the caller threads them straight from a
/// parsed `picture_coding_extension()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotionVectorsContext {
    /// `f_code[0][0]` — forward horizontal `f_code`.
    pub f_code_fwd_horiz: u8,
    /// `f_code[0][1]` — forward vertical `f_code`.
    pub f_code_fwd_vert: u8,
    /// `f_code[1][0]` — backward horizontal `f_code`.
    pub f_code_bwd_horiz: u8,
    /// `f_code[1][1]` — backward vertical `f_code`.
    pub f_code_bwd_vert: u8,
}

impl MotionVectorsContext {
    /// Pick the `(f_code_horiz, f_code_vert)` pair for the given `s`.
    fn for_kind(&self, kind: MotionVectorsKind) -> (u8, u8) {
        match kind {
            MotionVectorsKind::Forward => (self.f_code_fwd_horiz, self.f_code_fwd_vert),
            MotionVectorsKind::Backward => (self.f_code_bwd_horiz, self.f_code_bwd_vert),
        }
    }
}

/// One entry of `motion_vectors(s)`'s `r`-loop: an optional
/// `motion_vertical_field_select[r][s]` flag plus a parsed
/// [`MotionVector`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotionVectorEntry {
    /// `motion_vertical_field_select[r][s]` — top reference field
    /// (`false` = 0) or bottom reference field (`true` = 1); `None`
    /// when the flag is absent (`motion_vector_count == 1` &&
    /// `mv_format == frame`, or `dmv == 1`). §6.2.5.2.
    pub vertical_field_select: Option<bool>,
    /// The parsed `motion_vector(r, s)` per §6.2.5.2.1.
    pub motion_vector: MotionVector,
}

/// The full `motion_vectors(s)` element per §6.2.5.2: one or two
/// `MotionVectorEntry` rows plus the post-cursor bit position.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MotionVectors {
    /// `s` — forward (`0`) or backward (`1`) — for which this
    /// `motion_vectors(s)` was parsed.
    pub kind: MotionVectorsKind,
    /// One or two `(motion_vertical_field_select, motion_vector)` rows
    /// driven by `motion_vector_count` (Table 6-17 / 6-18).
    pub entries: Vec<MotionVectorEntry>,
    /// Bit position right after the consumed bits.
    pub bit_position_after: u64,
}

impl MotionVectors {
    /// Parse a `motion_vectors(s)` from §6.2.5.2.
    ///
    /// `kind` is the spec's `s` index. `motion_type` carries the
    /// derived `motion_vector_count` / `mv_format` / `dmv` values from
    /// Tables 6-17 / 6-18 that gate the syntax. `ctx` carries the
    /// `f_code[s][t]` matrix from `picture_coding_extension()`.
    pub fn parse(
        br: &mut BitReader<'_>,
        kind: MotionVectorsKind,
        motion_type: &MotionType,
        ctx: &MotionVectorsContext,
    ) -> Result<Self> {
        let (f_code_horiz, f_code_vert) = ctx.for_kind(kind);
        let mut entries: Vec<MotionVectorEntry> = Vec::with_capacity(2);

        match motion_type.motion_vector_count {
            1 => {
                // §6.2.5.2: vertical_field_select present iff
                // (mv_format == field) && (dmv != 1).
                let vfs_present = motion_type.mv_format == MvFormat::Field && !motion_type.dmv;
                let vertical_field_select = if vfs_present {
                    Some(br.read_bit().map_err(|_| Error::ShortHeader)?)
                } else {
                    None
                };
                let mv = MotionVector::parse(br, f_code_horiz, f_code_vert, motion_type.dmv)?;
                entries.push(MotionVectorEntry {
                    vertical_field_select,
                    motion_vector: mv,
                });
            }
            2 => {
                // §6.2.5.2: both rows carry vertical_field_select
                // unconditionally; dmv is incompatible with count == 2
                // because Tables 6-17 / 6-18 only emit dmv with
                // count == 1.
                for _ in 0..2 {
                    let vertical_field_select =
                        Some(br.read_bit().map_err(|_| Error::ShortHeader)?);
                    let mv = MotionVector::parse(br, f_code_horiz, f_code_vert, motion_type.dmv)?;
                    entries.push(MotionVectorEntry {
                        vertical_field_select,
                        motion_vector: mv,
                    });
                }
            }
            other => {
                let _ = other;
                return Err(Error::InvalidBitstream(
                    "motion_vectors: motion_vector_count must be 1 or 2 (Tables 6-17 / 6-18)",
                ));
            }
        }

        Ok(Self {
            kind,
            entries,
            bit_position_after: br.bit_position(),
        })
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact round-trips for Tables B-10 and B-11, plus
    //! the §6.2.5.2 / §6.2.5.2.1 presence matrix.
    use super::*;
    use crate::macroblock_modes::PredictionType;
    use oxideav_core::bits::BitWriter;

    /// Emit `bits` codewords back to back, pad to a byte with a
    /// trailing `1`, and return the resulting buffer.
    fn buf(codes: &[(u32, u32)]) -> Vec<u8> {
        let mut bw = BitWriter::new();
        for &(code, n) in codes {
            bw.write_u32(code, n);
        }
        bw.write_bit(true);
        bw.align_to_byte();
        bw.finish()
    }

    fn frame_based(count: u8) -> MotionType {
        MotionType {
            code: 0b10,
            prediction_type: PredictionType::FrameBased,
            motion_vector_count: count,
            mv_format: MvFormat::Frame,
            dmv: false,
        }
    }

    fn field_based_one() -> MotionType {
        MotionType {
            code: 0b01,
            prediction_type: PredictionType::FieldBased,
            motion_vector_count: 1,
            mv_format: MvFormat::Field,
            dmv: false,
        }
    }

    fn sixteen_by_eight() -> MotionType {
        MotionType {
            code: 0b10,
            prediction_type: PredictionType::SixteenByEight,
            motion_vector_count: 2,
            mv_format: MvFormat::Field,
            dmv: false,
        }
    }

    fn dual_prime() -> MotionType {
        MotionType {
            code: 0b11,
            prediction_type: PredictionType::DualPrime,
            motion_vector_count: 1,
            mv_format: MvFormat::Field,
            dmv: true,
        }
    }

    fn ctx_all_one() -> MotionVectorsContext {
        MotionVectorsContext {
            f_code_fwd_horiz: 1,
            f_code_fwd_vert: 1,
            f_code_bwd_horiz: 1,
            f_code_bwd_vert: 1,
        }
    }

    #[test]
    fn b10_zero_decodes_to_zero_with_one_bit() {
        // Bit '1' alone = motion_code 0.
        let data = buf(&[(0b1, 1)]);
        let mut br = BitReader::new(&data);
        let entry = match_b10(&mut br).expect("match");
        assert_eq!(entry.value, 0);
        assert_eq!(entry.bits, 1);
        assert_eq!(br.bit_position(), 1);
    }

    #[test]
    fn b10_three_bit_negative_one_and_one() {
        let n = buf(&[(0b011, 3)]);
        let mut br = BitReader::new(&n);
        assert_eq!(match_b10(&mut br).unwrap().value, -1);

        let p = buf(&[(0b010, 3)]);
        let mut br = BitReader::new(&p);
        assert_eq!(match_b10(&mut br).unwrap().value, 1);
    }

    #[test]
    fn b10_extremes_minus16_and_plus16() {
        let neg = buf(&[(0b0000_0011_001, 11)]);
        let mut br = BitReader::new(&neg);
        let entry = match_b10(&mut br).unwrap();
        assert_eq!(entry.value, -16);
        assert_eq!(entry.bits, 11);

        let pos = buf(&[(0b0000_0011_000, 11)]);
        let mut br = BitReader::new(&pos);
        let entry = match_b10(&mut br).unwrap();
        assert_eq!(entry.value, 16);
        assert_eq!(entry.bits, 11);
    }

    #[test]
    fn b10_table_has_33_unique_entries() {
        // -16..=+16 inclusive = 33 values.
        assert_eq!(TABLE_B10.len(), 33);
        let mut values: Vec<i8> = TABLE_B10.iter().map(|e| e.value).collect();
        values.sort_unstable();
        let expected: Vec<i8> = (-16..=16).collect();
        assert_eq!(values, expected);
    }

    #[test]
    fn b10_table_is_prefix_free() {
        // Every codeword, extended into a u32 left-justified at bit 31,
        // must not be a prefix of any other entry.
        for &a in TABLE_B10 {
            for &b in TABLE_B10 {
                if a.bits == b.bits && a.code == b.code {
                    continue; // same row
                }
                if a.bits < b.bits {
                    let b_prefix = b.code >> (b.bits - a.bits);
                    assert!(
                        b_prefix != a.code,
                        "Table B-10: row (value={}) prefix of (value={})",
                        a.value,
                        b.value
                    );
                }
            }
        }
    }

    #[test]
    fn b10_every_code_fits_its_width() {
        for &e in TABLE_B10 {
            assert!(
                u32::from(e.code) < (1u32 << e.bits),
                "code {:b} does not fit in {} bits",
                e.code,
                e.bits
            );
        }
    }

    #[test]
    fn b10_walks_each_row_individually() {
        for &e in TABLE_B10 {
            let data = buf(&[(u32::from(e.code), u32::from(e.bits))]);
            let mut br = BitReader::new(&data);
            let got = match_b10(&mut br).unwrap_or_else(|err| {
                panic!("row value={} bits={} failed: {err:?}", e.value, e.bits)
            });
            assert_eq!(got.value, e.value);
            assert_eq!(got.bits, e.bits);
            assert_eq!(br.bit_position(), u64::from(e.bits));
        }
    }

    #[test]
    fn b10_unknown_prefix_is_rejected() {
        // 11 zero bits = `0000 0000 000`, not in Table B-10.
        let data = buf(&[(0, 11)]);
        let mut br = BitReader::new(&data);
        let err = match_b10(&mut br).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn b10_truncated_short_buffer_rejected() {
        let data: [u8; 0] = [];
        let mut br = BitReader::new(&data);
        let err = match_b10(&mut br).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn b11_decodes_zero_with_one_bit() {
        let data = buf(&[(0b0, 1)]);
        let mut br = BitReader::new(&data);
        assert_eq!(match_b11(&mut br).unwrap(), 0);
        assert_eq!(br.bit_position(), 1);
    }

    #[test]
    fn b11_decodes_plus_one_and_minus_one() {
        let pos = buf(&[(0b10, 2)]);
        let mut br = BitReader::new(&pos);
        assert_eq!(match_b11(&mut br).unwrap(), 1);
        assert_eq!(br.bit_position(), 2);

        let neg = buf(&[(0b11, 2)]);
        let mut br = BitReader::new(&neg);
        assert_eq!(match_b11(&mut br).unwrap(), -1);
        assert_eq!(br.bit_position(), 2);
    }

    #[test]
    fn b11_truncated_after_first_one_bit_is_short() {
        // Only a single '1' bit available, but Table B-11 needs a
        // second bit to disambiguate +1 from -1.
        let data = [0b0000_0001u8];
        let mut br = BitReader::new(&data);
        br.skip(7).expect("skip");
        // Now exactly 1 bit left, value '1'.
        let err = match_b11(&mut br).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    #[test]
    fn motion_vector_minimal_f_code_one_no_residual() {
        // motion_code = 0 (1 bit '1'), motion_code_vert = 0,
        // f_code=1 for both components → no residuals, no dmvector.
        let data = buf(&[(0b1, 1), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let mv = MotionVector::parse(&mut br, 1, 1, false).expect("parse");
        assert_eq!(mv.motion_code_horiz, 0);
        assert_eq!(mv.motion_code_vert, 0);
        assert_eq!(mv.motion_residual_horiz, None);
        assert_eq!(mv.motion_residual_vert, None);
        assert_eq!(mv.dmvector_horiz, None);
        assert_eq!(mv.dmvector_vert, None);
        assert_eq!(mv.bit_position_after, 2);
    }

    #[test]
    fn motion_vector_residual_only_when_code_nonzero_and_f_code_gt_one() {
        // f_code = 2 → r_size = 1; motion_code = -1 (3 bits '011') →
        // residual present (1 bit). Then code_vert = 0 → no residual.
        let data = buf(&[(0b011, 3), (0b1, 1), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let mv = MotionVector::parse(&mut br, 2, 1, false).expect("parse");
        assert_eq!(mv.motion_code_horiz, -1);
        assert_eq!(mv.motion_residual_horiz, Some(1));
        assert_eq!(mv.motion_code_vert, 0);
        assert_eq!(mv.motion_residual_vert, None);
        assert_eq!(mv.bit_position_after, 5);
    }

    #[test]
    fn motion_vector_residual_width_tracks_f_code() {
        // f_code = 5 → r_size = 4. motion_code = +1 (3 bits '010') →
        // residual is 4 bits, value `1010` = 10.
        let data = buf(&[(0b010, 3), (0b1010, 4), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let mv = MotionVector::parse(&mut br, 5, 1, false).expect("parse");
        assert_eq!(mv.motion_code_horiz, 1);
        assert_eq!(mv.motion_residual_horiz, Some(10));
        assert_eq!(mv.motion_code_vert, 0);
        assert_eq!(mv.bit_position_after, 8);
    }

    #[test]
    fn motion_vector_residual_absent_when_motion_code_zero_even_with_f_code_gt_one() {
        // f_code = 3 but motion_code = 0 → no residual.
        let data = buf(&[(0b1, 1), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let mv = MotionVector::parse(&mut br, 3, 3, false).expect("parse");
        assert_eq!(mv.motion_code_horiz, 0);
        assert_eq!(mv.motion_residual_horiz, None);
        assert_eq!(mv.bit_position_after, 2);
    }

    #[test]
    fn motion_vector_dmvector_consumed_when_dmv_set() {
        // f_code = 1 → no residual. motion_code = 0 (1 bit '1') for
        // both components. dmv = 1 → dmvector after each.
        // dmvector: '11' → -1, then '0' → 0.
        let data = buf(&[(0b1, 1), (0b11, 2), (0b1, 1), (0b0, 1)]);
        let mut br = BitReader::new(&data);
        let mv = MotionVector::parse(&mut br, 1, 1, true).expect("parse");
        assert_eq!(mv.motion_code_horiz, 0);
        assert_eq!(mv.dmvector_horiz, Some(-1));
        assert_eq!(mv.motion_code_vert, 0);
        assert_eq!(mv.dmvector_vert, Some(0));
        assert_eq!(mv.bit_position_after, 5);
    }

    #[test]
    fn motion_vector_rejects_out_of_range_f_code() {
        // f_code = 0 is forbidden upstream but tolerated here unless we
        // would actually use r_size — the gate `f_code != 1` plus our
        // own residual-width sanity check catches it.
        let data = buf(&[(0b011, 3), (0b0, 1), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let err = MotionVector::parse(&mut br, 0, 1, false).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));

        // f_code = 10 is also out of range (max is 9; 15 = unused).
        let data = buf(&[(0b011, 3), (0b0, 1), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let err = MotionVector::parse(&mut br, 10, 1, false).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn motion_vector_truncated_residual_is_short() {
        // f_code = 4 → r_size = 3. motion_code = -1 (3 bits '011').
        // Only the 3-bit code present, residual is missing.
        let data = buf(&[(0b011, 3)]);
        let mut br = BitReader::new(&data);
        // Consume the padding '1' bit so the next read sees a short
        // buffer for the residual.
        br.read_bit().ok(); // skip nothing — actually we want the buf
                            // to terminate after the 3 bits, so just
                            // build it explicitly without padding.
        let mut bw = BitWriter::new();
        bw.write_u32(0b011, 3);
        bw.align_to_byte();
        let data = bw.finish();
        let mut br = BitReader::new(&data);
        // 5 bits of padding remain (zeros) — enough for r_size=3
        // residual, then nothing for the vertical code.
        let err = MotionVector::parse(&mut br, 4, 1, false).unwrap_err();
        // After reading horiz code (3) + horiz residual (3) = 6, then
        // vertical code needs at least 1 bit; only 2 zero bits left so
        // the only matching B-10 row is '1' (value 0) but the high bit
        // is 0 — falls through to the unknown-prefix path.
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn motion_vectors_count_one_frame_no_vfs() {
        // Frame-based, count = 1, mv_format = frame → no
        // vertical_field_select; motion_vector(0, 0) only.
        // motion_code horiz = 0, motion_code vert = 0; f_code = 1.
        let data = buf(&[(0b1, 1), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let mt = frame_based(1);
        let ctx = ctx_all_one();
        let mvs =
            MotionVectors::parse(&mut br, MotionVectorsKind::Forward, &mt, &ctx).expect("parse");
        assert_eq!(mvs.entries.len(), 1);
        assert_eq!(mvs.entries[0].vertical_field_select, None);
        assert_eq!(mvs.bit_position_after, 2);
    }

    #[test]
    fn motion_vectors_count_one_field_with_vfs() {
        // Field-based, count = 1, mv_format = field, dmv = 0 → VFS bit
        // present.
        let data = buf(&[(0b1, 1), (0b1, 1), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let mt = field_based_one();
        let ctx = ctx_all_one();
        let mvs =
            MotionVectors::parse(&mut br, MotionVectorsKind::Forward, &mt, &ctx).expect("parse");
        assert_eq!(mvs.entries.len(), 1);
        assert_eq!(mvs.entries[0].vertical_field_select, Some(true));
        assert_eq!(mvs.bit_position_after, 3);
    }

    #[test]
    fn motion_vectors_dual_prime_no_vfs() {
        // dmv = 1 → VFS suppressed even for mv_format == field
        // (§6.2.5.2). Each component carries a dmvector after its
        // motion_code.
        let data = buf(&[(0b1, 1), (0b0, 1), (0b1, 1), (0b0, 1)]);
        let mut br = BitReader::new(&data);
        let mt = dual_prime();
        let ctx = ctx_all_one();
        let mvs =
            MotionVectors::parse(&mut br, MotionVectorsKind::Forward, &mt, &ctx).expect("parse");
        assert_eq!(mvs.entries.len(), 1);
        assert_eq!(mvs.entries[0].vertical_field_select, None);
        assert_eq!(mvs.entries[0].motion_vector.dmvector_horiz, Some(0));
        assert_eq!(mvs.entries[0].motion_vector.dmvector_vert, Some(0));
        assert_eq!(mvs.bit_position_after, 4);
    }

    #[test]
    fn motion_vectors_count_two_emits_two_vfs() {
        // 16×8 MC: count == 2 → two VFS bits, each followed by a
        // motion_vector(r, s). With f_code = 1 and motion_code = 0
        // everywhere, the layout is `VFS | 1 | 1 | VFS | 1 | 1`.
        let data = buf(&[(0b0, 1), (0b1, 1), (0b1, 1), (0b1, 1), (0b1, 1), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let mt = sixteen_by_eight();
        let ctx = ctx_all_one();
        let mvs =
            MotionVectors::parse(&mut br, MotionVectorsKind::Forward, &mt, &ctx).expect("parse");
        assert_eq!(mvs.entries.len(), 2);
        assert_eq!(mvs.entries[0].vertical_field_select, Some(false));
        assert_eq!(mvs.entries[1].vertical_field_select, Some(true));
        assert_eq!(mvs.bit_position_after, 6);
    }

    #[test]
    fn motion_vectors_backward_picks_bwd_f_code() {
        // f_code_bwd_horiz = 2 → r_size = 1 for the horizontal residual
        // when motion_code != 0. Use a non-zero motion_code to confirm
        // the bwd table is in play.
        let data = buf(&[(0b011, 3), (0b1, 1), (0b1, 1)]);
        let mut br = BitReader::new(&data);
        let mt = frame_based(1);
        let ctx = MotionVectorsContext {
            f_code_fwd_horiz: 1,
            f_code_fwd_vert: 1,
            f_code_bwd_horiz: 2,
            f_code_bwd_vert: 1,
        };
        let mvs =
            MotionVectors::parse(&mut br, MotionVectorsKind::Backward, &mt, &ctx).expect("parse");
        assert_eq!(mvs.entries[0].motion_vector.motion_code_horiz, -1);
        assert_eq!(mvs.entries[0].motion_vector.motion_residual_horiz, Some(1));
        assert_eq!(mvs.entries[0].motion_vector.motion_code_vert, 0);
        assert_eq!(mvs.bit_position_after, 5);
    }

    #[test]
    fn motion_vectors_rejects_invalid_count() {
        // Synthesise a motion_type with count = 0 (only reachable if a
        // future scalable table emits a value the non-scalable ones do
        // not).
        let mt = MotionType {
            code: 0,
            prediction_type: PredictionType::FrameBased,
            motion_vector_count: 0,
            mv_format: MvFormat::Frame,
            dmv: false,
        };
        let ctx = ctx_all_one();
        let data = buf(&[(0b1, 1)]);
        let mut br = BitReader::new(&data);
        let err = MotionVectors::parse(&mut br, MotionVectorsKind::Forward, &mt, &ctx).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn motion_vectors_kind_resolves_correct_f_code_pair() {
        let ctx = MotionVectorsContext {
            f_code_fwd_horiz: 1,
            f_code_fwd_vert: 2,
            f_code_bwd_horiz: 3,
            f_code_bwd_vert: 4,
        };
        assert_eq!(ctx.for_kind(MotionVectorsKind::Forward), (1, 2));
        assert_eq!(ctx.for_kind(MotionVectorsKind::Backward), (3, 4));
    }

    #[test]
    fn motion_vectors_truncated_vfs_is_short() {
        // Field-based, count = 1 → VFS bit expected. Buffer empty.
        let data: [u8; 0] = [];
        let mut br = BitReader::new(&data);
        let mt = field_based_one();
        let ctx = ctx_all_one();
        let err = MotionVectors::parse(&mut br, MotionVectorsKind::Forward, &mt, &ctx).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    #[test]
    fn motion_vectors_truncated_motion_code_is_invalid_bitstream() {
        // VFS present, then a zero motion_code prefix that does not
        // match any Table B-10 row.
        let data = [0u8; 2]; // VFS = 0, then 11 zero bits → unknown.
        let mut br = BitReader::new(&data);
        let mt = field_based_one();
        let ctx = ctx_all_one();
        let err = MotionVectors::parse(&mut br, MotionVectorsKind::Forward, &mt, &ctx).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn debug_impls_smoke() {
        let mvs = MotionVectors {
            kind: MotionVectorsKind::Forward,
            entries: vec![MotionVectorEntry {
                vertical_field_select: Some(false),
                motion_vector: MotionVector {
                    motion_code_horiz: 0,
                    motion_residual_horiz: None,
                    dmvector_horiz: None,
                    motion_code_vert: 0,
                    motion_residual_vert: None,
                    dmvector_vert: None,
                    bit_position_after: 2,
                },
            }],
            bit_position_after: 3,
        };
        let s = format!("{mvs:?}");
        assert!(s.contains("MotionVectors"));
        assert!(s.contains("Forward"));
        let kind_s = format!("{:?}", MotionVectorsKind::Backward);
        assert!(kind_s.contains("Backward"));
    }
}
