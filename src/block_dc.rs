//! MPEG-1 intra-block DC prelude per **ISO/IEC 11172-2:1993
//! §2.4.2.8 / §2.4.3.7** — the `dct_dc_size_luminance` /
//! `dct_dc_size_chrominance` VLC walkers (**Annex B Tables B.5a /
//! B.5b**), the `dct_dc_differential` → `dct_zz[0]` reconstruction
//! formula, and the §2.4.4.1 8x8 `scan[m][n]` zig-zag ordering used
//! by every block-layer iterator.
//!
//! This module is the *prelude* to the residual block layer: the
//! pieces below feed into the wider `dct_coeff_first` /
//! `dct_coeff_next` (Tables B.5c..B.5f) decoder still ahead. They
//! are bounded and useful on their own — every MPEG-1 intra block
//! starts with one of these DC fields, and the `SCAN` matrix is
//! shared with every non-intra block as well.
//!
//! All spec citations refer to **ISO/IEC 11172-2:1993** (MPEG-1
//! Video). The companion MPEG-2 sibling fields (`dct_dc_size_*` /
//! the 13818-2 §6.3.17.5 zig-zag arrays) use different tables and
//! are intentionally not covered here — this module is MPEG-1 only.
//!
//! ## Layout of `SCAN`
//!
//! The spec prints `scan[m][n]` (page 32) with `m` the *vertical*
//! index and `n` the *horizontal* index — i.e. `scan[0][0] = 0`,
//! `scan[0][7] = 28`, `scan[7][7] = 63`. We store it row-major as
//! `[[u8; 8]; 8]` with the same `[row][col]` indexing.

// The VLC constants in Tables B.5a / B.5b are short (1..=9 bits)
// and printed in the spec as run-together bit strings rather than
// nibble-aligned groups; the bit groupings here mirror the spec
// printout for ease of audit.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

// =============================================================
// §2.4.4.1 zig-zag scan order
// =============================================================

/// `scan[m][n]` per ISO/IEC 11172-2:1993 §2.4.4.1 page 32.
///
/// Stored row-major: `SCAN[m][n]` is the spec's `scan[m][n]` with
/// `m` the vertical index and `n` the horizontal index.
///
/// The matrix maps a zig-zag-ordered position `(m, n)` of the
/// 8x8 block to its position in the raster-ordered `dct_recon`
/// matrix. The §2.4.4.1 dequantiser uses it as `i = SCAN[m][n]`
/// to fetch the matching `dct_zz[i]`.
pub const SCAN: [[u8; 8]; 8] = [
    [0, 1, 5, 6, 14, 15, 27, 28],
    [2, 4, 7, 13, 16, 26, 29, 42],
    [3, 8, 12, 17, 25, 30, 41, 43],
    [9, 11, 18, 24, 31, 40, 44, 53],
    [10, 19, 23, 32, 39, 45, 52, 54],
    [20, 22, 33, 38, 46, 51, 55, 60],
    [21, 34, 37, 47, 50, 56, 59, 61],
    [35, 36, 48, 49, 57, 58, 62, 63],
];

/// Inverse of [`SCAN`]: maps a zig-zag *index* `i` in `0..=63` to
/// the `(m, n)` cell in the raster matrix it loads.
///
/// Derived from [`SCAN`] at compile time. Useful for encoders /
/// trace tools that want to write a coefficient list out in
/// zig-zag order. Indexed as `INVERSE_SCAN[i] = (row, col)`.
pub const INVERSE_SCAN: [(u8, u8); 64] = build_inverse_scan();

const fn build_inverse_scan() -> [(u8, u8); 64] {
    let mut out = [(0u8, 0u8); 64];
    let mut m = 0usize;
    while m < 8 {
        let mut n = 0usize;
        while n < 8 {
            let i = SCAN[m][n] as usize;
            out[i] = (m as u8, n as u8);
            n += 1;
        }
        m += 1;
    }
    out
}

// =============================================================
// §2.4.2.8 / §2.4.3.7 — dct_dc_size VLC walkers (Tables B.5a / B.5b)
// =============================================================

/// Which intra-block kind the DC field belongs to.
///
/// MPEG-1 §2.4.2.8 has separate VLC tables for luminance
/// (Y) blocks (`dct_dc_size_luminance`, Table B.5a) and
/// chrominance (Cb, Cr) blocks (`dct_dc_size_chrominance`,
/// Table B.5b).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DcComponent {
    /// Luminance (Y) block — decode against Table B.5a.
    Luminance,
    /// Chrominance (Cb or Cr) block — decode against Table B.5b.
    Chrominance,
}

/// One entry of a `dct_dc_size_*` VLC table. The code is stored
/// right-justified in a `u16` and the spec mapping is *code →
/// `dc_size`* (the number of trailing differential bits).
#[derive(Debug, Clone, Copy)]
struct DcSizeEntry {
    /// MSB-first bit-string, right-justified into a `u16`.
    code: u16,
    /// Code length in bits.
    bits: u8,
    /// Decoded `dct_dc_size_*` value, `0..=8` for luminance and
    /// `0..=8` for chrominance per Tables B.5a / B.5b.
    size: u8,
}

/// **Table B.5a** — `dct_dc_size_luminance` VLC (page 44 of the
/// spec). Sizes `0..=8` cover 0..=11-bit DC differentials.
const TABLE_B5A: &[DcSizeEntry] = &[
    DcSizeEntry {
        code: 0b100,
        bits: 3,
        size: 0,
    },
    DcSizeEntry {
        code: 0b00,
        bits: 2,
        size: 1,
    },
    DcSizeEntry {
        code: 0b01,
        bits: 2,
        size: 2,
    },
    DcSizeEntry {
        code: 0b101,
        bits: 3,
        size: 3,
    },
    DcSizeEntry {
        code: 0b110,
        bits: 3,
        size: 4,
    },
    DcSizeEntry {
        code: 0b1110,
        bits: 4,
        size: 5,
    },
    DcSizeEntry {
        code: 0b1_1110,
        bits: 5,
        size: 6,
    },
    DcSizeEntry {
        code: 0b11_1110,
        bits: 6,
        size: 7,
    },
    DcSizeEntry {
        code: 0b111_1110,
        bits: 7,
        size: 8,
    },
];

/// **Table B.5b** — `dct_dc_size_chrominance` VLC (page 44 of the
/// spec). Sizes `0..=8` cover 0..=11-bit DC differentials. The
/// chrominance table is uniformly one bit longer than the
/// luminance table for the same size (the leading `'1'` prefix is
/// shifted right by one).
const TABLE_B5B: &[DcSizeEntry] = &[
    DcSizeEntry {
        code: 0b00,
        bits: 2,
        size: 0,
    },
    DcSizeEntry {
        code: 0b01,
        bits: 2,
        size: 1,
    },
    DcSizeEntry {
        code: 0b10,
        bits: 2,
        size: 2,
    },
    DcSizeEntry {
        code: 0b110,
        bits: 3,
        size: 3,
    },
    DcSizeEntry {
        code: 0b1110,
        bits: 4,
        size: 4,
    },
    DcSizeEntry {
        code: 0b1_1110,
        bits: 5,
        size: 5,
    },
    DcSizeEntry {
        code: 0b11_1110,
        bits: 6,
        size: 6,
    },
    DcSizeEntry {
        code: 0b111_1110,
        bits: 7,
        size: 7,
    },
    DcSizeEntry {
        code: 0b1111_1110,
        bits: 8,
        size: 8,
    },
];

/// Upper bound on `dct_dc_size_*` per Tables B.5a / B.5b
/// (`0..=8`). The matching `dct_dc_differential` is `dc_size` bits
/// wide.
pub const MAX_DC_SIZE: u8 = 8;

/// Walk Tables B.5a / B.5b for the next `dct_dc_size_*` codeword
/// starting at `br`. Consumes the matched bits on success.
///
/// Both tables are prefix-free; the walker tries widths from
/// longest to shortest to keep the equality check unambiguous on
/// shorter prefixes of longer codewords.
fn read_dc_size(br: &mut BitReader<'_>, component: DcComponent) -> Result<u8> {
    let (table, widths): (&[DcSizeEntry], &[u8]) = match component {
        DcComponent::Luminance => (TABLE_B5A, &[7u8, 6, 5, 4, 3, 2]),
        DcComponent::Chrominance => (TABLE_B5B, &[8u8, 7, 6, 5, 4, 3, 2]),
    };
    for &width in widths {
        if br.bits_remaining() < u64::from(width) {
            continue;
        }
        let peeked = br
            .peek_u32(u32::from(width))
            .map_err(|_| Error::ShortHeader)? as u16;
        for &entry in table.iter().filter(|e| e.bits == width) {
            if entry.code == peeked {
                br.consume(u32::from(width))
                    .map_err(|_| Error::ShortHeader)?;
                return Ok(entry.size);
            }
        }
    }
    Err(Error::InvalidBitstream(
        "dct_dc_size: no Table B.5a / B.5b codeword matches the bit prefix (§2.4.2.8)",
    ))
}

// =============================================================
// §2.4.3.7 — dct_dc_differential → dct_zz[0] reconstruction
// =============================================================

/// Reconstruct `dct_zz[0]` from a `dct_dc_size_*` value and the
/// following `dct_dc_differential` bits per **ISO/IEC 11172-2:1993
/// §2.4.3.7** (page 30).
///
/// Per the spec, if `dc_size == 0` the differential is *not* in
/// the bitstream and `dct_zz[0]` is `0`. Otherwise the
/// `dc_size`-bit unsigned integer is read MSB-first and converted
/// into a signed value:
///
/// ```text
/// if (dct_dc_differential & (1 << (dc_size - 1)))
///     dct_zz[0] =  dct_dc_differential ;
/// else
///     dct_zz[0] = ((-1) << dc_size) | (dct_dc_differential + 1) ;
/// ```
///
/// (The same formula is used for luminance and chrominance — only
/// the size-determining VLC differs.)
///
/// The signed result range is `[-(2^dc_size - 1), 2^dc_size - 1]`
/// — for `dc_size == 8` that is `[-255, 255]`.
fn reconstruct_dc_differential(dc_size: u8, dct_dc_differential: u32) -> i32 {
    if dc_size == 0 {
        return 0;
    }
    // dct_dc_differential is constrained to `dc_size` bits — the
    // caller (read_dc_differential) reads exactly that many bits
    // and the high bits are zero. The branch tests the MSB.
    let msb = 1u32 << (dc_size - 1);
    if dct_dc_differential & msb != 0 {
        // Positive branch: the value is already the magnitude.
        dct_dc_differential as i32
    } else {
        // Negative branch: spec rewrites as
        // `((-1) << dc_size) | (dct_dc_differential + 1)` which is
        // the two's-complement extension of the sign bit down to
        // `dc_size` bits, plus one. We compute it in i64 to avoid
        // any overflow concerns on the `+ 1` then cast.
        let extension: i64 = -(1i64 << dc_size);
        let value: i64 = extension | i64::from(dct_dc_differential + 1);
        value as i32
    }
}

/// Read the `dct_dc_differential` field (`dc_size` bits, MSB-first
/// unsigned) and reconstruct `dct_zz[0]` per §2.4.3.7.
fn read_dc_differential(br: &mut BitReader<'_>, dc_size: u8) -> Result<i32> {
    if dc_size == 0 {
        return Ok(0);
    }
    // dc_size is bounded by MAX_DC_SIZE = 8 in practice, so a
    // `u32` read is always wide enough. The guard is defensive.
    debug_assert!(dc_size <= MAX_DC_SIZE);
    let raw = br
        .peek_u32(u32::from(dc_size))
        .map_err(|_| Error::ShortHeader)?;
    br.consume(u32::from(dc_size))
        .map_err(|_| Error::ShortHeader)?;
    Ok(reconstruct_dc_differential(dc_size, raw))
}

// =============================================================
// Encoder side — §2.4.2.8 / §2.4.3.7 emission
// =============================================================

/// The minimal `dct_dc_size_*` for a signed `dct_zz[0]` differential:
/// `0` for zero, else the smallest `size` with
/// `|value| <= 2^size - 1` (§2.4.3.7 value ranges).
fn dc_size_for_value(dct_zz_0: i32) -> u8 {
    if dct_zz_0 == 0 {
        return 0;
    }
    let mag = dct_zz_0.unsigned_abs();
    let mut size = 1u8;
    while (1u32 << size) - 1 < mag {
        size += 1;
        debug_assert!(
            size <= MAX_DC_SIZE,
            "dct_zz[0] {dct_zz_0} exceeds the Table B.5a/B.5b size range"
        );
    }
    size
}

/// Emit a §2.4.2.8 intra-block DC prelude: the `dct_dc_size_*` VLC
/// (Table B.5a / B.5b by `component`) followed by the `dc_size`-bit
/// `dct_dc_differential` that reconstructs `dct_zz_0` exactly.
///
/// `dct_zz_0` is the signed DC differential the §2.4.4.1 predictor
/// chain requires (`(dct_recon[0][0] - dct_dc_<comp>_past) / 8`), in
/// the Table B.5a/B.5b range `[-255, +255]`. The function is the exact
/// inverse of the §2.4.3.7 reconstruction read back by
/// [`DcCoefficient::parse`]: positive values are transmitted verbatim;
/// a negative value `v` is transmitted as `v + 2^size - 1` (inverting
/// `dct_zz[0] = ((-1) << size) | (differential + 1)`).
///
/// # Panics (debug)
/// Debug-asserts `|dct_zz_0| <= 255`.
pub fn encode_dc_coefficient(
    bw: &mut oxideav_core::bits::BitWriter,
    component: DcComponent,
    dct_zz_0: i32,
) {
    let size = dc_size_for_value(dct_zz_0);
    let table = match component {
        DcComponent::Luminance => TABLE_B5A,
        DcComponent::Chrominance => TABLE_B5B,
    };
    let entry = table
        .iter()
        .find(|e| e.size == size)
        .expect("size <= MAX_DC_SIZE has a table row");
    bw.write_u32(u32::from(entry.code), u32::from(entry.bits));
    if size == 0 {
        return;
    }
    let differential = if dct_zz_0 > 0 {
        dct_zz_0
    } else {
        dct_zz_0 + (1i32 << size) - 1
    };
    bw.write_u32(differential as u32, u32::from(size));
}

// =============================================================
// Public entry point
// =============================================================

/// One parsed `(dct_dc_size_*, dct_dc_differential)` pair, the
/// per-component intra-block prelude per §2.4.2.8 + §2.4.3.7.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DcCoefficient {
    /// Which DC VLC table was used (Y vs Cb / Cr).
    pub component: DcComponent,
    /// `dct_dc_size_*` value in `0..=8`.
    pub dc_size: u8,
    /// Raw `dct_dc_differential` bits (`dc_size` bits wide,
    /// MSB-first). `0` when `dc_size == 0` (the field is absent).
    pub dct_dc_differential: u32,
    /// Reconstructed `dct_zz[0]` per §2.4.3.7. Range
    /// `[-(2^dc_size - 1), 2^dc_size - 1]`.
    pub dct_zz_0: i32,
    /// Bit position (relative to the start of the buffer the
    /// reader was handed) right after the last consumed bit of
    /// this DC field.
    pub bit_position_after: u64,
}

impl DcCoefficient {
    /// Parse the intra-block DC prelude (VLC + differential) for
    /// the given `component`. Returns the typed record on success.
    pub fn parse(br: &mut BitReader<'_>, component: DcComponent) -> Result<Self> {
        let dc_size = read_dc_size(br, component)?;
        // Capture the raw differential before reconstruction so
        // the caller can inspect / re-emit it bit-exactly.
        let raw = if dc_size == 0 {
            0u32
        } else {
            br.peek_u32(u32::from(dc_size))
                .map_err(|_| Error::ShortHeader)?
        };
        let dct_zz_0 = read_dc_differential(br, dc_size)?;
        Ok(Self {
            component,
            dc_size,
            dct_dc_differential: raw,
            dct_zz_0,
            bit_position_after: br.bit_position(),
        })
    }
}

#[cfg(test)]
mod tests {
    //! Spec-pinned coverage of Tables B.5a / B.5b, the §2.4.3.7
    //! differential→`dct_zz[0]` reconstruction (incl. the worked
    //! `dc_size == 3` example from page 30), and the §2.4.4.1
    //! zig-zag `SCAN` matrix.
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Helper: emit a Table B.5a code into a writer.
    fn write_b5a(bw: &mut BitWriter, size: u8) {
        let entry = TABLE_B5A
            .iter()
            .find(|e| e.size == size)
            .expect("size in 0..=8");
        bw.write_u32(u32::from(entry.code), u32::from(entry.bits));
    }

    /// Helper: emit a Table B.5b code into a writer.
    fn write_b5b(bw: &mut BitWriter, size: u8) {
        let entry = TABLE_B5B
            .iter()
            .find(|e| e.size == size)
            .expect("size in 0..=8");
        bw.write_u32(u32::from(entry.code), u32::from(entry.bits));
    }

    fn pad_and_finish(mut bw: BitWriter) -> Vec<u8> {
        // One '0' then byte-align, so a BitReader has at least one
        // trailing byte to load past the end of the payload.
        bw.write_bit(false);
        bw.align_to_byte();
        bw.finish()
    }

    // ----- Table B.5a / B.5b shape -----

    #[test]
    fn table_b5a_has_nine_entries_0_through_8() {
        let mut sizes: Vec<u8> = TABLE_B5A.iter().map(|e| e.size).collect();
        sizes.sort_unstable();
        assert_eq!(sizes, vec![0, 1, 2, 3, 4, 5, 6, 7, 8]);
    }

    #[test]
    fn table_b5b_has_nine_entries_0_through_8() {
        let mut sizes: Vec<u8> = TABLE_B5B.iter().map(|e| e.size).collect();
        sizes.sort_unstable();
        assert_eq!(sizes, vec![0, 1, 2, 3, 4, 5, 6, 7, 8]);
    }

    #[test]
    fn table_b5a_codes_fit_their_widths() {
        for e in TABLE_B5A {
            let max = 1u32 << u32::from(e.bits);
            assert!(u32::from(e.code) < max, "B.5a code {:b} too wide", e.code);
        }
    }

    #[test]
    fn table_b5b_codes_fit_their_widths() {
        for e in TABLE_B5B {
            let max = 1u32 << u32::from(e.bits);
            assert!(u32::from(e.code) < max, "B.5b code {:b} too wide", e.code);
        }
    }

    #[test]
    fn table_b5a_codes_unique_per_width() {
        for &width in &[2u8, 3, 4, 5, 6, 7] {
            let group: Vec<_> = TABLE_B5A.iter().filter(|e| e.bits == width).collect();
            for (i, a) in group.iter().enumerate() {
                for b in &group[i + 1..] {
                    assert_ne!(a.code, b.code, "B.5a duplicate code at width {width}");
                }
            }
        }
    }

    #[test]
    fn table_b5b_codes_unique_per_width() {
        for &width in &[2u8, 3, 4, 5, 6, 7, 8] {
            let group: Vec<_> = TABLE_B5B.iter().filter(|e| e.bits == width).collect();
            for (i, a) in group.iter().enumerate() {
                for b in &group[i + 1..] {
                    assert_ne!(a.code, b.code, "B.5b duplicate code at width {width}");
                }
            }
        }
    }

    // ----- VLC round-trips -----

    #[test]
    fn parses_every_b5a_size() {
        for size in 0u8..=8 {
            let mut bw = BitWriter::new();
            write_b5a(&mut bw, size);
            // Append a `dct_dc_differential` of all zeros so the
            // parser has bits to consume when size > 0.
            for _ in 0..size {
                bw.write_bit(false);
            }
            let buf = pad_and_finish(bw);
            let mut br = BitReader::new(&buf);
            let dc =
                DcCoefficient::parse(&mut br, DcComponent::Luminance).expect("Table B.5a parse");
            assert_eq!(dc.dc_size, size);
        }
    }

    #[test]
    fn parses_every_b5b_size() {
        for size in 0u8..=8 {
            let mut bw = BitWriter::new();
            write_b5b(&mut bw, size);
            for _ in 0..size {
                bw.write_bit(false);
            }
            let buf = pad_and_finish(bw);
            let mut br = BitReader::new(&buf);
            let dc =
                DcCoefficient::parse(&mut br, DcComponent::Chrominance).expect("Table B.5b parse");
            assert_eq!(dc.dc_size, size);
        }
    }

    #[test]
    fn rejects_garbage_b5a_prefix() {
        // A run of '1's longer than the longest B.5a code (7 bits)
        // must hit the no-match branch. Use 0xFF FF: every 7-bit
        // prefix is 0b111_1111 which is not in the table — the
        // longest valid prefix `111_1110` (size 8) ends in a `0`.
        let buf = [0xFFu8; 4];
        let mut br = BitReader::new(&buf);
        let err = DcCoefficient::parse(&mut br, DcComponent::Luminance).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_truncated_buffer_b5a() {
        // Empty buffer — no bits to peek.
        let buf: [u8; 0] = [];
        let mut br = BitReader::new(&buf);
        let err = DcCoefficient::parse(&mut br, DcComponent::Luminance).unwrap_err();
        assert!(matches!(
            err,
            Error::InvalidBitstream(_) | Error::ShortHeader
        ));
    }

    #[test]
    fn b5a_bit_position_tracks_size() {
        // size 0 — 3 code bits, 0 differential bits.
        let mut bw = BitWriter::new();
        write_b5a(&mut bw, 0);
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let dc = DcCoefficient::parse(&mut br, DcComponent::Luminance).unwrap();
        assert_eq!(dc.dc_size, 0);
        assert_eq!(dc.bit_position_after, 3);

        // size 8 — 7 code bits, 8 differential bits.
        let mut bw = BitWriter::new();
        write_b5a(&mut bw, 8);
        for _ in 0..8 {
            bw.write_bit(true);
        }
        let buf = pad_and_finish(bw);
        let mut br = BitReader::new(&buf);
        let dc = DcCoefficient::parse(&mut br, DcComponent::Luminance).unwrap();
        assert_eq!(dc.dc_size, 8);
        assert_eq!(dc.bit_position_after, 7 + 8);
    }

    // ----- §2.4.3.7 differential reconstruction -----

    #[test]
    fn size_zero_yields_zero_differential() {
        let v = reconstruct_dc_differential(0, 0);
        assert_eq!(v, 0);
    }

    /// Worked example transcribed from page 30 of ISO/IEC
    /// 11172-2:1993 for `dc_size = 3`:
    ///
    /// | `dct_dc_differential` | `dct_zz[0]` |
    /// |-----------------------|-------------|
    /// | 000                   | -7          |
    /// | 001                   | -6          |
    /// | 010                   | -5          |
    /// | 011                   | -4          |
    /// | 100                   |  4          |
    /// | 101                   |  5          |
    /// | 110                   |  6          |
    /// | 111                   |  7          |
    #[test]
    fn spec_table_dc_size_3_example_matches_page_30() {
        let expected = [-7, -6, -5, -4, 4, 5, 6, 7];
        for (raw, want) in (0u32..8).zip(expected.iter().copied()) {
            let got = reconstruct_dc_differential(3, raw);
            assert_eq!(got, want, "dc_size=3 raw={raw:03b}");
        }
    }

    #[test]
    fn size_1_yields_minus1_or_plus1() {
        assert_eq!(reconstruct_dc_differential(1, 0b0), -1);
        assert_eq!(reconstruct_dc_differential(1, 0b1), 1);
    }

    #[test]
    fn size_2_covers_the_four_values() {
        // The §2.4.3.7 formula for dc_size = 2 yields:
        // raw=00 → -3, raw=01 → -2, raw=10 → 2, raw=11 → 3.
        assert_eq!(reconstruct_dc_differential(2, 0b00), -3);
        assert_eq!(reconstruct_dc_differential(2, 0b01), -2);
        assert_eq!(reconstruct_dc_differential(2, 0b10), 2);
        assert_eq!(reconstruct_dc_differential(2, 0b11), 3);
    }

    #[test]
    fn size_8_corner_values() {
        // The §2.4.3.7 formula for dc_size = 8 has range
        // [-255, 255] with raw=0x00 → -255, raw=0xFF → 255,
        // raw=0x80 → 128 (MSB set), raw=0x7F → -128.
        assert_eq!(reconstruct_dc_differential(8, 0x00), -255);
        assert_eq!(reconstruct_dc_differential(8, 0xFF), 255);
        assert_eq!(reconstruct_dc_differential(8, 0x80), 128);
        assert_eq!(reconstruct_dc_differential(8, 0x7F), -128);
    }

    #[test]
    fn parse_returns_reconstructed_value_for_size_3_example() {
        // For each row of the spec page-30 example, hand-build
        // the wire bytes ("B.5a code for size 3" + raw 3-bit
        // differential) and confirm the parser yields the
        // expected signed dct_zz[0].
        let expected = [-7, -6, -5, -4, 4, 5, 6, 7];
        for (raw, want) in (0u32..8).zip(expected.iter().copied()) {
            let mut bw = BitWriter::new();
            write_b5a(&mut bw, 3);
            // dct_dc_differential is MSB-first. write_u32 packs
            // exactly that way.
            bw.write_u32(raw, 3);
            let buf = pad_and_finish(bw);
            let mut br = BitReader::new(&buf);
            let dc = DcCoefficient::parse(&mut br, DcComponent::Luminance).unwrap();
            assert_eq!(dc.dc_size, 3);
            assert_eq!(dc.dct_dc_differential, raw);
            assert_eq!(dc.dct_zz_0, want, "raw={raw:03b}");
        }
    }

    #[test]
    fn chrominance_parse_uses_b5b_codes() {
        // Pick a size only Table B.5b decodes via its specific
        // codeword: size 0 has B.5b code '00' (2 bits) which in
        // B.5a is *also* a 2-bit code but with a different value
        // (B.5a code '00' decodes to size 1). Reading the same
        // bytes against Luminance vs Chrominance must therefore
        // yield different `dc_size`s.
        let mut bw = BitWriter::new();
        write_b5b(&mut bw, 0); // B.5b: '00' → size 0
        let buf = pad_and_finish(bw);

        let mut br_y = BitReader::new(&buf);
        let dc_y = DcCoefficient::parse(&mut br_y, DcComponent::Luminance).unwrap();
        assert_eq!(dc_y.dc_size, 1); // B.5a: '00' → size 1

        let mut br_c = BitReader::new(&buf);
        let dc_c = DcCoefficient::parse(&mut br_c, DcComponent::Chrominance).unwrap();
        assert_eq!(dc_c.dc_size, 0);
    }

    // ----- §2.4.4.1 SCAN matrix -----

    #[test]
    fn scan_contains_every_index_exactly_once() {
        let mut seen = [false; 64];
        for row in SCAN.iter() {
            for &v in row {
                assert!(v < 64, "SCAN entry {v} out of range — must be in 0..=63");
                assert!(!seen[v as usize], "SCAN entry {v} duplicated");
                seen[v as usize] = true;
            }
        }
        assert!(seen.iter().all(|b| *b), "SCAN missed some index");
    }

    #[test]
    fn scan_corners_match_spec_page_32() {
        // Spec prints scan[0][0] = 0, scan[0][7] = 28,
        // scan[7][0] = 35, scan[7][7] = 63.
        assert_eq!(SCAN[0][0], 0);
        assert_eq!(SCAN[0][7], 28);
        assert_eq!(SCAN[7][0], 35);
        assert_eq!(SCAN[7][7], 63);
    }

    #[test]
    fn scan_first_diagonal_matches_spec() {
        // The classic JPEG-style zig-zag opens
        //   0
        //   1, 2
        //   3, 4, 5
        //   6, 7, 8, 9
        // which in the spec page-32 matrix means:
        // scan[0][1] = 1, scan[1][0] = 2, scan[2][0] = 3,
        // scan[1][1] = 4, scan[0][2] = 5, scan[0][3] = 6,
        // scan[1][2] = 7, scan[2][1] = 8, scan[3][0] = 9.
        assert_eq!(SCAN[0][1], 1);
        assert_eq!(SCAN[1][0], 2);
        assert_eq!(SCAN[2][0], 3);
        assert_eq!(SCAN[1][1], 4);
        assert_eq!(SCAN[0][2], 5);
        assert_eq!(SCAN[0][3], 6);
        assert_eq!(SCAN[1][2], 7);
        assert_eq!(SCAN[2][1], 8);
        assert_eq!(SCAN[3][0], 9);
    }

    #[test]
    fn inverse_scan_round_trips_against_scan() {
        for (m, row) in SCAN.iter().enumerate() {
            for (n, &cell) in row.iter().enumerate() {
                let i = cell as usize;
                let (rm, rn) = INVERSE_SCAN[i];
                assert_eq!(
                    (rm as usize, rn as usize),
                    (m, n),
                    "INVERSE_SCAN[{i}] should map back to ({m},{n})"
                );
            }
        }
    }

    #[test]
    fn debug_impl_smoke() {
        let dc = DcCoefficient {
            component: DcComponent::Luminance,
            dc_size: 3,
            dct_dc_differential: 0b101,
            dct_zz_0: 5,
            bit_position_after: 6,
        };
        let s = format!("{dc:?}");
        assert!(s.contains("DcCoefficient"));
        assert!(s.contains("dc_size"));
    }
}

#[cfg(test)]
mod encode_tests {
    //! Encoder-side coverage: the full §2.4.3.7 differential range
    //! round-trips through the Table B.5a / B.5b parser for both
    //! component tables.
    use super::*;
    use oxideav_core::bits::{BitReader, BitWriter};

    #[test]
    fn dc_size_matches_value_magnitude() {
        assert_eq!(dc_size_for_value(0), 0);
        assert_eq!(dc_size_for_value(1), 1);
        assert_eq!(dc_size_for_value(-1), 1);
        assert_eq!(dc_size_for_value(2), 2);
        assert_eq!(dc_size_for_value(3), 2);
        assert_eq!(dc_size_for_value(4), 3);
        assert_eq!(dc_size_for_value(127), 7);
        assert_eq!(dc_size_for_value(128), 8);
        assert_eq!(dc_size_for_value(-255), 8);
        assert_eq!(dc_size_for_value(255), 8);
    }

    #[test]
    fn every_differential_roundtrips_for_both_components() {
        for component in [DcComponent::Luminance, DcComponent::Chrominance] {
            for value in -255i32..=255 {
                let mut bw = BitWriter::new();
                encode_dc_coefficient(&mut bw, component, value);
                let written = bw.bit_position();
                bw.write_bit(false);
                bw.align_to_byte();
                bw.write_byte(0);
                let bytes = bw.finish();
                let mut br = BitReader::new(&bytes);
                let dc = DcCoefficient::parse(&mut br, component).expect("parse DC prelude");
                assert_eq!(dc.dct_zz_0, value, "{component:?} value {value}");
                assert_eq!(
                    dc.bit_position_after, written,
                    "{component:?} value {value}"
                );
            }
        }
    }
}
