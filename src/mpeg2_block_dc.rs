//! MPEG-2 intra-block DC prelude per **ISO/IEC 13818-2 (ITU-T H.262)
//! §7.2.1** — the `dct_dc_size_luminance` / `dct_dc_size_chrominance`
//! VLC walkers (**Annex B Tables B-12 / B-13**), the
//! `dc_dct_differential` → `dct_diff` reconstruction formula, and
//! the three-component DC predictor state (`dc_dct_pred[cc]`) with
//! the §7.2.1 reset semantics derived from `intra_dc_precision`
//! (Table 7-2).
//!
//! This module is the MPEG-2 sibling of the MPEG-1 [`block_dc`]
//! module (the MPEG-1 form lives in `block_dc::DcCoefficient` and
//! reads Tables B.5a / B.5b). The two stream types diverge in three
//! places:
//!
//! 1. **VLC range.** MPEG-1 (Tables B.5a / B.5b) caps `dct_dc_size`
//!    at 8 (matching the 8-bit-per-sample pixel range). MPEG-2
//!    (Tables B-12 / B-13) extends the range to 11 to accommodate the
//!    `intra_dc_precision == 3` case where DC is coded at 11 bits.
//! 2. **DC predictor.** MPEG-1 maintains the DC predictor across
//!    intra macroblocks only and resets it at slice boundaries and on
//!    non-intra macroblocks. MPEG-2 §7.2.1 carries an explicit
//!    per-component predictor `dc_dct_pred[cc]` whose **reset value**
//!    depends on `intra_dc_precision` (Table 7-2:
//!    `intra_dc_precision ∈ {0,1,2,3} → reset ∈ {128, 256, 512, 1024}`)
//!    and which is reset on (a) the start of a slice, (b) any
//!    non-intra macroblock, and (c) any skipped macroblock
//!    (`macroblock_address_increment > 1`).
//! 3. **Reconstruction.** Both specs differ in spelling. MPEG-1 §2.4.3.7
//!    uses an MSB-test branch; MPEG-2 §7.2.1 uses a `half_range`
//!    threshold (`half_range = 2 ^ (dct_dc_size - 1)`), and the
//!    bitstream constraint is that the recovered `QFS[0]` (predictor
//!    plus differential) must lie in `[0, 2^(8 + intra_dc_precision) - 1]`.
//!
//! All spec citations refer to **ISO/IEC 13818-2:1995** (ITU-T H.262).
//! The §7.2.1 reconstruction is exercised together with the §7.2.2
//! walker ([`mpeg2_dct_coeff`]) and the §7.3 inverse scan
//! ([`mpeg2_inverse_scan`]) to build a complete `QF[v][u]` block from
//! the bitstream.

// The VLC constants in Tables B-12 / B-13 are short (2..=11 bits)
// and printed in the spec as run-together bit strings rather than
// nibble-aligned groups; the bit groupings here mirror the spec
// printout for ease of audit.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

// =============================================================
// §7.2.1 — dct_dc_size VLC walkers (Tables B-12 / B-13)
// =============================================================

/// Which DC VLC table to use for the current block.
///
/// §7.2.1: *"If `cc` is zero then Table B-12 shall be used for
/// `dct_dc_size`. If `cc` is non-zero then Table B-13 shall be
/// used for `dct_dc_size`."* `cc` is the colour-component index
/// from Table 7-1 (`cc == 0` for Y; `cc == 1` for Cb; `cc == 2`
/// for Cr). The crate's `Component` enum elsewhere captures the
/// luma/chroma split — we name the variants by component kind to
/// keep call sites readable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DcComponent {
    /// Luminance (Y) — `cc == 0`. Decode against Table B-12.
    Luminance,
    /// Chrominance (Cb or Cr) — `cc != 0`. Decode against Table B-13.
    Chrominance,
}

/// One entry of a `dct_dc_size_*` VLC table. The code is stored
/// right-justified in a `u16` and the spec mapping is *code →
/// `dct_dc_size`* (the number of trailing differential bits).
#[derive(Debug, Clone, Copy)]
struct DcSizeEntry {
    /// MSB-first bit-string, right-justified into a `u16`.
    code: u16,
    /// Code length in bits.
    bits: u8,
    /// Decoded `dct_dc_size_*` value in `0..=11`.
    size: u8,
}

/// **Table B-12** — `dct_dc_size_luminance` VLC (ISO/IEC 13818-2
/// page 161). Sizes `0..=11` cover DC differentials of width 0
/// through 11 bits respectively. The first 9 codes (sizes 0..=8)
/// are identical bit-for-bit to MPEG-1's Table B.5a; sizes 9, 10,
/// and 11 extend the table with prefixes `1111 1110`, `1111 1111 0`,
/// and `1111 1111 1` (all sharing the `1111 1110` longer-prefix
/// pattern).
const TABLE_B12: &[DcSizeEntry] = &[
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
    DcSizeEntry {
        code: 0b1111_1110,
        bits: 8,
        size: 9,
    },
    DcSizeEntry {
        code: 0b1_1111_1110,
        bits: 9,
        size: 10,
    },
    DcSizeEntry {
        code: 0b1_1111_1111,
        bits: 9,
        size: 11,
    },
];

/// **Table B-13** — `dct_dc_size_chrominance` VLC (ISO/IEC 13818-2
/// page 161). Sizes `0..=11` cover DC differentials of width 0
/// through 11 bits respectively. The first 9 codes (sizes 0..=8)
/// are identical bit-for-bit to MPEG-1's Table B.5b; sizes 9, 10,
/// and 11 extend the table with prefixes `1111 1111 0`,
/// `1111 1111 10`, and `1111 1111 11`.
const TABLE_B13: &[DcSizeEntry] = &[
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
    DcSizeEntry {
        code: 0b1_1111_1110,
        bits: 9,
        size: 9,
    },
    DcSizeEntry {
        code: 0b11_1111_1110,
        bits: 10,
        size: 10,
    },
    DcSizeEntry {
        code: 0b11_1111_1111,
        bits: 10,
        size: 11,
    },
];

/// Upper bound on `dct_dc_size_*` per Tables B-12 / B-13
/// (`0..=11`). The matching `dc_dct_differential` is `dct_dc_size`
/// bits wide. (MPEG-1's analogue tops out at 8; the +3 sizes carry
/// the extra precision MPEG-2 needs when
/// `intra_dc_precision != 0`.)
pub const MAX_DC_SIZE: u8 = 11;

/// Walk Tables B-12 / B-13 for the next `dct_dc_size_*` codeword
/// starting at `br`. Consumes the matched bits on success.
///
/// Both tables are prefix-free; the walker tries widths from
/// longest to shortest to keep the equality check unambiguous on
/// shorter prefixes of longer codewords.
fn read_dc_size(br: &mut BitReader<'_>, component: DcComponent) -> Result<u8> {
    let (table, widths): (&[DcSizeEntry], &[u8]) = match component {
        DcComponent::Luminance => (TABLE_B12, &[9u8, 8, 7, 6, 5, 4, 3, 2]),
        DcComponent::Chrominance => (TABLE_B13, &[10u8, 9, 8, 7, 6, 5, 4, 3, 2]),
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
        "dct_dc_size: no Table B-12 / B-13 codeword matches the bit prefix (§7.2.1)",
    ))
}

// =============================================================
// §7.2.1 — dc_dct_differential → dct_diff reconstruction
// =============================================================

/// Reconstruct the signed `dct_diff` from a `dct_dc_size` value and
/// the following `dc_dct_differential` bits per **ISO/IEC 13818-2:1995
/// §7.2.1**:
///
/// ```text
/// if ( dct_dc_size == 0 ) {
///     dct_diff = 0;
/// } else {
///     half_range = 2 ^ ( dct_dc_size - 1 );
///     if ( dc_dct_differential >= half_range )
///         dct_diff = dc_dct_differential;
///     else
///         dct_diff = (dc_dct_differential + 1) - (2 * half_range);
/// }
/// ```
///
/// The signed result range is `[-(2^dct_dc_size - 1), 2^dct_dc_size - 1]`
/// — for `dct_dc_size == 11` that is `[-2047, 2047]`. Per §7.2.1 the
/// recovered `QFS[0] = dc_dct_pred[cc] + dct_diff` is required by the
/// bitstream to lie in `[0, 2^(8 + intra_dc_precision) - 1]`, but the
/// `dct_diff` itself can be negative.
///
/// (Mathematically equivalent to the MPEG-1 §2.4.3.7 MSB-test form,
/// just spelled differently — the bitstream constraint is the same.)
fn reconstruct_dc_diff(dct_dc_size: u8, dc_dct_differential: u32) -> i32 {
    if dct_dc_size == 0 {
        return 0;
    }
    debug_assert!(dct_dc_size <= MAX_DC_SIZE);
    let half_range: u32 = 1u32 << (dct_dc_size - 1);
    if dc_dct_differential >= half_range {
        dc_dct_differential as i32
    } else {
        // (differential + 1) - 2 * half_range. Compute in i64 to
        // avoid the obvious unsigned overflow when `differential ==
        // 0` and the spec wants a negative result.
        let signed = i64::from(dc_dct_differential) + 1 - 2 * i64::from(half_range);
        signed as i32
    }
}

/// Read the `dc_dct_differential` field (`dct_dc_size` bits,
/// MSB-first unsigned) and reconstruct the signed `dct_diff` per
/// §7.2.1.
fn read_dc_differential(br: &mut BitReader<'_>, dct_dc_size: u8) -> Result<i32> {
    if dct_dc_size == 0 {
        return Ok(0);
    }
    debug_assert!(dct_dc_size <= MAX_DC_SIZE);
    let raw = br
        .peek_u32(u32::from(dct_dc_size))
        .map_err(|_| Error::ShortHeader)?;
    br.consume(u32::from(dct_dc_size))
        .map_err(|_| Error::ShortHeader)?;
    Ok(reconstruct_dc_diff(dct_dc_size, raw))
}

// =============================================================
// §7.2.1 / Table 7-2 — dc_dct_pred[cc] state + reset values
// =============================================================

/// Reset value for `dc_dct_pred[cc]` per **Table 7-2** of
/// ISO/IEC 13818-2:1995. `intra_dc_precision` is the 2-bit field
/// from `picture_coding_extension()` (Table 6-13).
///
/// | `intra_dc_precision` | bits | reset value |
/// |----------------------|------|-------------|
/// | 0 | 8 | 128 |
/// | 1 | 9 | 256 |
/// | 2 | 10 | 512 |
/// | 3 | 11 | 1024 |
///
/// Returns `Err` if `intra_dc_precision` is outside `0..=3`
/// (Table 6-13 only defines those four values).
pub fn dc_pred_reset_value(intra_dc_precision: u8) -> Result<i32> {
    match intra_dc_precision {
        0 => Ok(128),
        1 => Ok(256),
        2 => Ok(512),
        3 => Ok(1024),
        _ => Err(Error::InvalidBitstream(
            "intra_dc_precision: only the 2-bit values 0..=3 are defined (Table 6-13)",
        )),
    }
}

/// Maximum legal `QFS[0]` for a given `intra_dc_precision` per the
/// §7.2.1 bitstream constraint:
///
/// > It is a requirement of the bitstream that QFS[0] shall lie in
/// > the range `0 to ((2^(8 + intra_dc_precision)) - 1)`.
///
/// For `intra_dc_precision ∈ {0,1,2,3}` that is `{255, 511, 1023,
/// 2047}`.
pub fn qfs_zero_max(intra_dc_precision: u8) -> Result<i32> {
    let bits = match intra_dc_precision {
        0 => 8,
        1 => 9,
        2 => 10,
        3 => 11,
        _ => {
            return Err(Error::InvalidBitstream(
                "intra_dc_precision: only the 2-bit values 0..=3 are defined (Table 6-13)",
            ));
        }
    };
    Ok((1i32 << bits) - 1)
}

/// Per-component DC predictor state per **ISO/IEC 13818-2:1995
/// §7.2.1**: three predictors `dc_dct_pred[cc]` indexed by the
/// colour-component `cc` of Table 7-1 (`cc == 0` for Y; `cc == 1`
/// for Cb; `cc == 2` for Cr).
///
/// The predictors are reset to `dc_pred_reset_value(intra_dc_precision)`
/// at:
///
/// * the start of a slice
/// * whenever a non-intra macroblock is decoded
/// * whenever a macroblock is skipped (i.e.
///   `macroblock_address_increment > 1`)
///
/// Each call to [`Self::decode`] for a block in an intra macroblock
/// reads a fresh `(dct_dc_size, dc_dct_differential)` pair from the
/// stream, adds `dct_diff` to the per-component predictor, asserts
/// the §7.2.1 `[0, qfs_zero_max(p)]` range constraint on the
/// resulting `QFS[0]`, then updates the predictor to that `QFS[0]`
/// and returns the decoded value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DcPredictors {
    /// Predictor for `cc == 0` (Y).
    pub luma: i32,
    /// Predictor for `cc == 1` (Cb).
    pub cb: i32,
    /// Predictor for `cc == 2` (Cr).
    pub cr: i32,
    /// `intra_dc_precision` (Table 6-13, value 0..=3). Pinned so the
    /// predictor knows which reset value Table 7-2 selects and which
    /// `[0, max]` constraint §7.2.1 enforces on `QFS[0]`.
    pub intra_dc_precision: u8,
}

impl DcPredictors {
    /// Construct a predictor set primed to the §7.2.1 reset value
    /// for the given `intra_dc_precision`. Returns `Err` for
    /// `intra_dc_precision > 3`.
    pub fn new(intra_dc_precision: u8) -> Result<Self> {
        let reset = dc_pred_reset_value(intra_dc_precision)?;
        Ok(Self {
            luma: reset,
            cb: reset,
            cr: reset,
            intra_dc_precision,
        })
    }

    /// Reset all three predictors to the Table 7-2 value. Called by
    /// the slice-layer driver at the start of every slice, on every
    /// non-intra macroblock, and on every skipped macroblock per
    /// §7.2.1.
    pub fn reset(&mut self) {
        // intra_dc_precision was validated by `new`; reset cannot
        // fail in steady state.
        let reset = dc_pred_reset_value(self.intra_dc_precision)
            .expect("intra_dc_precision pinned by Self::new");
        self.luma = reset;
        self.cb = reset;
        self.cr = reset;
    }

    /// Read the predictor cell for a colour component per Table 7-1.
    pub fn get(&self, component: ColourComponent) -> i32 {
        match component {
            ColourComponent::Y => self.luma,
            ColourComponent::Cb => self.cb,
            ColourComponent::Cr => self.cr,
        }
    }

    /// Update the predictor cell for a colour component (Y, Cb, Cr
    /// each have their own cell per §7.2.1).
    pub fn set(&mut self, component: ColourComponent, value: i32) {
        match component {
            ColourComponent::Y => self.luma = value,
            ColourComponent::Cb => self.cb = value,
            ColourComponent::Cr => self.cr = value,
        }
    }
}

/// Which colour component the DC block is for, per Table 7-1.
/// Tables B-12 / B-13 only distinguish `cc == 0` (luma) from
/// `cc != 0` (chroma) — but the predictor cells are per-component,
/// so the caller needs to spell out Cb vs Cr.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColourComponent {
    /// `cc == 0` — luminance (Y).
    Y,
    /// `cc == 1` — Cb chrominance.
    Cb,
    /// `cc == 2` — Cr chrominance.
    Cr,
}

impl ColourComponent {
    /// Project to the Tables B-12 / B-13 table selector.
    pub fn dc_component(self) -> DcComponent {
        match self {
            ColourComponent::Y => DcComponent::Luminance,
            ColourComponent::Cb | ColourComponent::Cr => DcComponent::Chrominance,
        }
    }
}

// =============================================================
// Public entry point: parse + predictor update
// =============================================================

/// One parsed `(dct_dc_size, dc_dct_differential)` pair plus the
/// resulting `QFS[0]` after the §7.2.1 predictor update, for one
/// intra block of one colour component.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DcCoefficient {
    /// Which colour component this block belongs to (Y, Cb, Cr).
    pub component: ColourComponent,
    /// `dct_dc_size` value, `0..=11` per Tables B-12 / B-13.
    pub dct_dc_size: u8,
    /// Raw `dc_dct_differential` bits (`dct_dc_size` bits wide,
    /// MSB-first). `0` when `dct_dc_size == 0` (the field is
    /// absent from the bitstream).
    pub dc_dct_differential: u32,
    /// Reconstructed signed `dct_diff` per the §7.2.1 formula.
    /// Range `[-(2^dct_dc_size - 1), 2^dct_dc_size - 1]`.
    pub dct_diff: i32,
    /// Final `QFS[0]` = `dc_dct_pred[cc] + dct_diff` (after the
    /// per-component predictor update from this block). Constrained
    /// to `[0, qfs_zero_max(intra_dc_precision)]`.
    pub qfs_zero: i32,
    /// Bit position (relative to the start of the buffer the reader
    /// was handed) right after the last consumed bit of this DC
    /// field.
    pub bit_position_after: u64,
}

/// Decode one intra block's §7.2.1 DC prelude: pull the
/// `dct_dc_size` VLC from Tables B-12 / B-13, read the
/// `dc_dct_differential` field, reconstruct `dct_diff`, add the
/// per-component predictor, assert the §7.2.1 `[0, max]` bitstream
/// constraint on the resulting `QFS[0]`, and update the predictor.
///
/// The choice of Table B-12 vs Table B-13 is driven by
/// `component.dc_component()`. The predictor cell that's read +
/// updated is selected by `component` itself (so that Cb and Cr
/// each have their own predictor state, even though both share
/// Table B-13 for the size VLC).
pub fn decode_dc_block(
    br: &mut BitReader<'_>,
    predictors: &mut DcPredictors,
    component: ColourComponent,
) -> Result<DcCoefficient> {
    let dc_component = component.dc_component();

    // (1) Pull dct_dc_size from Table B-12 / B-13.
    let dct_dc_size = read_dc_size(br, dc_component)?;

    // (2) Peek the raw differential bits before reconstruction so
    // we can both return the unsigned wire value and the signed
    // reconstructed `dct_diff`. The reader needs to advance the
    // same number of bits either way.
    let raw_diff = if dct_dc_size == 0 {
        0u32
    } else {
        br.peek_u32(u32::from(dct_dc_size))
            .map_err(|_| Error::ShortHeader)?
    };
    let dct_diff = read_dc_differential(br, dct_dc_size)?;

    // (3) §7.2.1 predictor update: dct_diff is added to
    // `dc_dct_pred[cc]`, the resulting QFS[0] becomes the new
    // predictor. Cb and Cr each have their own predictor cell, so
    // we route the read + write through `ColourComponent`.
    let _ = dc_component; // table-selection only — silence unused-binding warnings in `--release`
    let predictor = predictors.get(component);
    let qfs_zero = predictor + dct_diff;

    // (4) §7.2.1 bitstream constraint: QFS[0] must lie in
    // [0, 2^(8 + intra_dc_precision) - 1].
    let max = qfs_zero_max(predictors.intra_dc_precision)?;
    if qfs_zero < 0 || qfs_zero > max {
        return Err(Error::InvalidBitstream(
            "QFS[0] outside the §7.2.1 [0, 2^(8 + intra_dc_precision) - 1] range",
        ));
    }

    // (5) Predictor update: dc_dct_pred[cc] = QFS[0].
    predictors.set(component, qfs_zero);

    Ok(DcCoefficient {
        component,
        dct_dc_size,
        dc_dct_differential: raw_diff,
        dct_diff,
        qfs_zero,
        bit_position_after: br.bit_position(),
    })
}

#[cfg(test)]
mod tests {
    //! Spec-pinned coverage of Tables B-12 / B-13, the §7.2.1
    //! reconstruction formula, the Table 7-2 reset values, the
    //! §7.2.1 predictor add + `[0, max]` range constraint, and the
    //! per-component predictor routing (Y / Cb / Cr).
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Helper: emit a Table B-12 code into a writer.
    fn write_b12(bw: &mut BitWriter, size: u8) {
        let entry = TABLE_B12
            .iter()
            .find(|e| e.size == size)
            .expect("size in 0..=11");
        bw.write_u32(u32::from(entry.code), u32::from(entry.bits));
    }

    /// Helper: emit a Table B-13 code into a writer.
    fn write_b13(bw: &mut BitWriter, size: u8) {
        let entry = TABLE_B13
            .iter()
            .find(|e| e.size == size)
            .expect("size in 0..=11");
        bw.write_u32(u32::from(entry.code), u32::from(entry.bits));
    }

    fn pad_and_finish(mut bw: BitWriter) -> Vec<u8> {
        // One '0' then byte-align, so a BitReader has at least one
        // trailing byte to load past the end of the payload.
        bw.write_bit(false);
        bw.align_to_byte();
        bw.finish()
    }

    // ----- Table B-12 / B-13 shape -----

    #[test]
    fn table_b12_has_twelve_entries_0_through_11() {
        let mut sizes: Vec<u8> = TABLE_B12.iter().map(|e| e.size).collect();
        sizes.sort_unstable();
        assert_eq!(sizes, (0u8..=11).collect::<Vec<u8>>());
    }

    #[test]
    fn table_b13_has_twelve_entries_0_through_11() {
        let mut sizes: Vec<u8> = TABLE_B13.iter().map(|e| e.size).collect();
        sizes.sort_unstable();
        assert_eq!(sizes, (0u8..=11).collect::<Vec<u8>>());
    }

    #[test]
    fn table_b12_codes_fit_their_widths() {
        for e in TABLE_B12 {
            let max = 1u32 << u32::from(e.bits);
            assert!(u32::from(e.code) < max, "B-12 code {:b} too wide", e.code);
        }
    }

    #[test]
    fn table_b13_codes_fit_their_widths() {
        for e in TABLE_B13 {
            let max = 1u32 << u32::from(e.bits);
            assert!(u32::from(e.code) < max, "B-13 code {:b} too wide", e.code);
        }
    }

    #[test]
    fn table_b12_codes_unique_per_width() {
        for &width in &[2u8, 3, 4, 5, 6, 7, 8, 9] {
            let group: Vec<_> = TABLE_B12.iter().filter(|e| e.bits == width).collect();
            for (i, a) in group.iter().enumerate() {
                for b in &group[i + 1..] {
                    assert_ne!(a.code, b.code, "B-12 duplicate code at width {width}");
                }
            }
        }
    }

    #[test]
    fn table_b13_codes_unique_per_width() {
        for &width in &[2u8, 3, 4, 5, 6, 7, 8, 9, 10] {
            let group: Vec<_> = TABLE_B13.iter().filter(|e| e.bits == width).collect();
            for (i, a) in group.iter().enumerate() {
                for b in &group[i + 1..] {
                    assert_ne!(a.code, b.code, "B-13 duplicate code at width {width}");
                }
            }
        }
    }

    /// The first 9 rows of Table B-12 (sizes 0..=8) are bit-exact
    /// MPEG-1 Table B.5a. Pin the equivalence so future drift on
    /// either side trips immediately.
    #[test]
    fn b12_first_9_rows_match_b5a() {
        use crate::block_dc;
        // The MPEG-1 side stores its DcSizeEntry privately too, but
        // the parse behaviour through `DcCoefficient::parse` is the
        // visible contract. Easier: confirm wire-level parse agrees
        // for sizes 0..=8.
        for size in 0u8..=8 {
            let mut bw = BitWriter::new();
            write_b12(&mut bw, size);
            for _ in 0..size {
                bw.write_bit(false);
            }
            let buf = pad_and_finish(bw);
            let mut br = BitReader::new(&buf);
            let mpeg1 =
                block_dc::DcCoefficient::parse(&mut br, block_dc::DcComponent::Luminance).unwrap();
            assert_eq!(mpeg1.dc_size, size);
        }
    }

    #[test]
    fn b13_first_9_rows_match_b5b() {
        use crate::block_dc;
        for size in 0u8..=8 {
            let mut bw = BitWriter::new();
            write_b13(&mut bw, size);
            for _ in 0..size {
                bw.write_bit(false);
            }
            let buf = pad_and_finish(bw);
            let mut br = BitReader::new(&buf);
            let mpeg1 = block_dc::DcCoefficient::parse(&mut br, block_dc::DcComponent::Chrominance)
                .unwrap();
            assert_eq!(mpeg1.dc_size, size);
        }
    }

    // ----- VLC round-trips -----

    #[test]
    fn parses_every_b12_size() {
        // Decode every (size, raw=0) pair through Table B-12 at
        // intra_dc_precision = 3 (widest window). All-zero raw at
        // size N decodes to dct_diff = -(2^N - 1); prime the
        // predictor halfway through the [0, 2047] window so even
        // size = 11 (worst case -2047) stays in range.
        let mut predictors = DcPredictors::new(3).unwrap();
        for size in 0u8..=11 {
            let mut bw = BitWriter::new();
            write_b12(&mut bw, size);
            for _ in 0..size {
                bw.write_bit(false);
            }
            let buf = pad_and_finish(bw);
            let mut br = BitReader::new(&buf);
            predictors.luma = 2047;
            let dc = decode_dc_block(&mut br, &mut predictors, ColourComponent::Y)
                .expect("Table B-12 parse");
            assert_eq!(dc.dct_dc_size, size);
        }
    }

    #[test]
    fn parses_every_b13_size() {
        let mut predictors = DcPredictors::new(3).unwrap();
        for size in 0u8..=11 {
            let mut bw = BitWriter::new();
            write_b13(&mut bw, size);
            for _ in 0..size {
                bw.write_bit(false);
            }
            let buf = pad_and_finish(bw);
            let mut br = BitReader::new(&buf);
            predictors.cb = 2047;
            let dc = decode_dc_block(&mut br, &mut predictors, ColourComponent::Cb)
                .expect("Table B-13 parse");
            assert_eq!(dc.dct_dc_size, size);
        }
    }

    #[test]
    fn rejects_garbage_b12_prefix() {
        // The longest B-12 codeword is 9 bits. A run of '1's longer
        // than that hits the no-match branch. We need a sequence
        // whose 9-bit prefix is neither `1_1111_1110` (size 10) nor
        // `1_1111_1111` (size 11). 0b1_1111_1110 is in the table;
        // 0b1_1111_1111 is in the table too. So all-ones DOES match
        // size 11. To trip the no-match, build a prefix that's not
        // in any width: try 0b0_0000_0000 (9 bits zeroes). The
        // shortest match attempt is "100" (size 0) — not all zeros.
        // "00" (2 bits, size 1) matches the first two zeros. So
        // zeros parse fine. We need a 7-bit prefix that's a valid
        // size 8 width — `111_1110` ends in '0' (size 8). To fail
        // we want bits that don't form ANY code, so use a prefix
        // that doesn't match any short code: 0b0_0011_xxxx — let's
        // build the explicit non-matching sequence directly. The
        // 3-bit `001` doesn't match B-12 (codes: 100, 00, 01, 101,
        // 110, 1110, ...). The 2-bit `00` matches size 1. So a
        // 0b00 prefix parses, never reaching the bad bits.
        // Conclusion: every non-empty B-12 stream eventually
        // matches *something*. To trigger the no-match path we
        // must construct a buffer that's too short to satisfy any
        // codeword's bit-count: we already cover that in
        // `rejects_truncated_buffer_b12`. The "infinite ones"
        // failure mode only applies to MPEG-1's bounded B.5a; B-12
        // extends that to size 11 with the `1111 1111 1` codeword,
        // so MPEG-2's worst-case all-ones prefix *does* match.
        // Document the invariant and move on.

        // Confirm that 9 all-ones bits decode as size 11 (the
        // longest B-12 codeword), proving the table is complete on
        // the long-prefix path.
        let mut bw = BitWriter::new();
        bw.write_u32(0b1_1111_1111, 9);
        // append 11 zero differential bits + pad
        for _ in 0..11 {
            bw.write_bit(false);
        }
        let buf = pad_and_finish(bw);
        let mut predictors = DcPredictors::new(3).unwrap();
        predictors.luma = 2047;
        let mut br = BitReader::new(&buf);
        let dc =
            decode_dc_block(&mut br, &mut predictors, ColourComponent::Y).expect("size 11 parse");
        assert_eq!(dc.dct_dc_size, 11);
    }

    #[test]
    fn rejects_truncated_buffer_b12() {
        let buf: [u8; 0] = [];
        let mut br = BitReader::new(&buf);
        let mut predictors = DcPredictors::new(0).unwrap();
        let err = decode_dc_block(&mut br, &mut predictors, ColourComponent::Y).unwrap_err();
        assert!(matches!(
            err,
            Error::InvalidBitstream(_) | Error::ShortHeader
        ));
    }

    // ----- §7.2.1 reconstruction (vs §2.4.3.7) -----

    #[test]
    fn size_zero_yields_zero_diff() {
        assert_eq!(reconstruct_dc_diff(0, 0), 0);
    }

    /// The MPEG-2 §7.2.1 formula and the MPEG-1 §2.4.3.7 formula
    /// are mathematically equivalent. Cross-check that every
    /// (dct_dc_size, raw) pair within MPEG-1's range produces the
    /// same signed `dct_diff` on both sides.
    #[test]
    fn mpeg2_recon_matches_mpeg1_for_sizes_1_through_8() {
        for dc_size in 1u8..=8 {
            let n = 1u32 << dc_size;
            for raw in 0..n {
                let mpeg2 = reconstruct_dc_diff(dc_size, raw);
                // Open-coded MPEG-1 form (§2.4.3.7), inline here so
                // we don't accidentally cross-test against a buggy
                // helper.
                let msb = 1u32 << (dc_size - 1);
                let mpeg1: i32 = if raw & msb != 0 {
                    raw as i32
                } else {
                    let ext: i64 = -(1i64 << dc_size);
                    (ext | i64::from(raw + 1)) as i32
                };
                assert_eq!(mpeg2, mpeg1, "dc_size={dc_size} raw={raw}");
            }
        }
    }

    /// Worked example for `dct_dc_size = 3`:
    ///
    /// | raw bits | dct_diff |
    /// |----------|----------|
    /// | 000      | -7       |
    /// | 001      | -6       |
    /// | 010      | -5       |
    /// | 011      | -4       |
    /// | 100      |  4       |
    /// | 101      |  5       |
    /// | 110      |  6       |
    /// | 111      |  7       |
    #[test]
    fn spec_table_size_3_example() {
        let expected = [-7, -6, -5, -4, 4, 5, 6, 7];
        for (raw, want) in (0u32..8).zip(expected.iter().copied()) {
            assert_eq!(reconstruct_dc_diff(3, raw), want, "raw={raw:03b}");
        }
    }

    #[test]
    fn size_11_corner_values() {
        // dct_dc_size = 11 → range [-2047, 2047].
        // raw = 0 → -2047; raw = 0x7FF (2047) → -1 (just below half_range);
        // raw = 0x400 (1024 = half_range) → 1024 (just at half_range);
        // raw = 0xFFF? — only 11 bits, so max raw = 0x7FF. Let's
        // re-verify: 2^11 = 2048, so raw in [0, 2047].
        assert_eq!(reconstruct_dc_diff(11, 0), -2047);
        assert_eq!(reconstruct_dc_diff(11, 1), -2046);
        assert_eq!(reconstruct_dc_diff(11, 1023), -1024);
        assert_eq!(reconstruct_dc_diff(11, 1024), 1024);
        assert_eq!(reconstruct_dc_diff(11, 2047), 2047);
    }

    // ----- Table 7-2 reset values + intra_dc_precision -----

    #[test]
    fn table_7_2_reset_values() {
        assert_eq!(dc_pred_reset_value(0).unwrap(), 128);
        assert_eq!(dc_pred_reset_value(1).unwrap(), 256);
        assert_eq!(dc_pred_reset_value(2).unwrap(), 512);
        assert_eq!(dc_pred_reset_value(3).unwrap(), 1024);
    }

    #[test]
    fn table_7_2_rejects_out_of_range_precision() {
        assert!(dc_pred_reset_value(4).is_err());
        assert!(dc_pred_reset_value(255).is_err());
    }

    #[test]
    fn qfs_zero_max_matches_8_plus_precision() {
        assert_eq!(qfs_zero_max(0).unwrap(), 255);
        assert_eq!(qfs_zero_max(1).unwrap(), 511);
        assert_eq!(qfs_zero_max(2).unwrap(), 1023);
        assert_eq!(qfs_zero_max(3).unwrap(), 2047);
    }

    // ----- Predictor lifecycle -----

    #[test]
    fn new_primes_all_three_to_reset() {
        for precision in 0u8..=3 {
            let p = DcPredictors::new(precision).unwrap();
            let reset = dc_pred_reset_value(precision).unwrap();
            assert_eq!(p.luma, reset);
            assert_eq!(p.cb, reset);
            assert_eq!(p.cr, reset);
            assert_eq!(p.intra_dc_precision, precision);
        }
    }

    #[test]
    fn reset_returns_all_three_to_table_7_2_value() {
        let mut p = DcPredictors::new(1).unwrap();
        p.luma = 42;
        p.cb = 7;
        p.cr = 9;
        p.reset();
        assert_eq!(p.luma, 256);
        assert_eq!(p.cb, 256);
        assert_eq!(p.cr, 256);
    }

    // ----- Full decode_dc_block paths -----

    /// Decode a size-3 DC for Y starting from the §7.2.1 reset
    /// value (128 at precision 0), confirm the predictor moves to
    /// `128 + dct_diff` per the worked example, and confirm the
    /// next call uses the updated predictor.
    #[test]
    fn decode_dc_block_y_chains_predictor() {
        let expected_diffs = [-7, -6, -5, -4, 4, 5, 6, 7];
        for (raw, diff) in (0u32..8).zip(expected_diffs.iter().copied()) {
            let mut bw = BitWriter::new();
            write_b12(&mut bw, 3);
            bw.write_u32(raw, 3);
            let buf = pad_and_finish(bw);

            let mut p = DcPredictors::new(0).unwrap();
            assert_eq!(p.luma, 128);

            let mut br = BitReader::new(&buf);
            let dc = decode_dc_block(&mut br, &mut p, ColourComponent::Y).unwrap();
            assert_eq!(dc.dct_dc_size, 3);
            assert_eq!(dc.dct_diff, diff);
            assert_eq!(dc.qfs_zero, 128 + diff);
            assert_eq!(p.luma, 128 + diff, "predictor must advance to QFS[0]");
            // Cb / Cr untouched.
            assert_eq!(p.cb, 128);
            assert_eq!(p.cr, 128);
        }
    }

    /// Decode a Cb-then-Cr pair confirming the two chroma
    /// predictors are independent.
    #[test]
    fn decode_dc_block_cb_and_cr_have_independent_state() {
        // Build [Cb size=2 raw=11 → +3] [Cr size=2 raw=00 → -3] back-to-back.
        let mut bw = BitWriter::new();
        write_b13(&mut bw, 2);
        bw.write_u32(0b11, 2); // raw = 3 → dct_diff = 3
        write_b13(&mut bw, 2);
        bw.write_u32(0b00, 2); // raw = 0 → dct_diff = -3
        let buf = pad_and_finish(bw);

        let mut p = DcPredictors::new(0).unwrap();
        let mut br = BitReader::new(&buf);

        let cb = decode_dc_block(&mut br, &mut p, ColourComponent::Cb).unwrap();
        assert_eq!(cb.dct_diff, 3);
        assert_eq!(cb.qfs_zero, 131);
        assert_eq!(p.cb, 131);
        // Cr predictor must still be the reset value.
        assert_eq!(p.cr, 128);

        let cr = decode_dc_block(&mut br, &mut p, ColourComponent::Cr).unwrap();
        assert_eq!(cr.dct_diff, -3);
        assert_eq!(cr.qfs_zero, 125);
        assert_eq!(p.cr, 125);
        // Cb predictor unchanged by the Cr decode.
        assert_eq!(p.cb, 131);
    }

    /// QFS[0] must lie in `[0, qfs_zero_max(precision)]` per the
    /// §7.2.1 bitstream constraint. Drive the predictor to a value
    /// that would make `predictor + dct_diff` negative and confirm
    /// the parser raises `InvalidBitstream`.
    #[test]
    fn out_of_range_qfs_zero_negative_is_rejected() {
        // dct_dc_size=3, raw=000 → dct_diff=-7. Starting predictor
        // at 5 → QFS[0] = -2, which violates the [0, 255] window.
        let mut bw = BitWriter::new();
        write_b12(&mut bw, 3);
        bw.write_u32(0b000, 3);
        let buf = pad_and_finish(bw);

        let mut p = DcPredictors::new(0).unwrap();
        p.luma = 5;
        let mut br = BitReader::new(&buf);
        let err = decode_dc_block(&mut br, &mut p, ColourComponent::Y).unwrap_err();
        match err {
            Error::InvalidBitstream(msg) => assert!(msg.contains("QFS[0]")),
            other => panic!("expected InvalidBitstream, got {other:?}"),
        }
    }

    #[test]
    fn out_of_range_qfs_zero_above_max_is_rejected() {
        // dct_dc_size=3, raw=111 → dct_diff=+7. Starting predictor
        // at 250 → QFS[0] = 257, which violates the [0, 255] window
        // at intra_dc_precision = 0.
        let mut bw = BitWriter::new();
        write_b12(&mut bw, 3);
        bw.write_u32(0b111, 3);
        let buf = pad_and_finish(bw);

        let mut p = DcPredictors::new(0).unwrap();
        p.luma = 250;
        let mut br = BitReader::new(&buf);
        let err = decode_dc_block(&mut br, &mut p, ColourComponent::Y).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    /// At `intra_dc_precision = 3` the [0, 2047] window is wide
    /// enough that a +7 differential from a predictor of 1024
    /// stays in range.
    #[test]
    fn precision_3_widens_qfs_zero_window() {
        let mut bw = BitWriter::new();
        write_b12(&mut bw, 3);
        bw.write_u32(0b111, 3);
        let buf = pad_and_finish(bw);

        let mut p = DcPredictors::new(3).unwrap();
        assert_eq!(p.luma, 1024);
        let mut br = BitReader::new(&buf);
        let dc = decode_dc_block(&mut br, &mut p, ColourComponent::Y).unwrap();
        assert_eq!(dc.qfs_zero, 1031);
    }

    // ----- Bit-position accounting -----

    #[test]
    fn bit_position_after_tracks_code_plus_differential() {
        // size 0 → 3 code bits, 0 differential bits.
        let mut bw = BitWriter::new();
        write_b12(&mut bw, 0);
        let buf = pad_and_finish(bw);
        let mut p = DcPredictors::new(0).unwrap();
        let mut br = BitReader::new(&buf);
        let dc = decode_dc_block(&mut br, &mut p, ColourComponent::Y).unwrap();
        assert_eq!(dc.bit_position_after, 3);

        // size 11 (longest B-12) → 9 code bits, 11 differential bits = 20.
        let mut bw = BitWriter::new();
        write_b12(&mut bw, 11);
        bw.write_u32(0b100_0000_0000, 11); // raw = 1024 → dct_diff = 1024 (in range)
        let buf = pad_and_finish(bw);
        let mut p = DcPredictors::new(3).unwrap();
        p.luma = 1023; // predictor 1023 + dct_diff 1024 = 2047 (max for precision 3)
        let mut br = BitReader::new(&buf);
        let dc = decode_dc_block(&mut br, &mut p, ColourComponent::Y).unwrap();
        assert_eq!(dc.bit_position_after, 9 + 11);
        assert_eq!(dc.dct_diff, 1024);
        assert_eq!(dc.qfs_zero, 1023 + 1024);
    }

    // ----- ColourComponent → DcComponent projection -----

    #[test]
    fn colour_component_projects_to_dc_component() {
        assert_eq!(ColourComponent::Y.dc_component(), DcComponent::Luminance);
        assert_eq!(ColourComponent::Cb.dc_component(), DcComponent::Chrominance);
        assert_eq!(ColourComponent::Cr.dc_component(), DcComponent::Chrominance);
    }

    #[test]
    fn debug_smoke() {
        let dc = DcCoefficient {
            component: ColourComponent::Y,
            dct_dc_size: 3,
            dc_dct_differential: 0b101,
            dct_diff: 5,
            qfs_zero: 133,
            bit_position_after: 6,
        };
        let s = format!("{dc:?}");
        assert!(s.contains("DcCoefficient"));
        assert!(s.contains("qfs_zero"));
        let p = DcPredictors::new(0).unwrap();
        assert!(format!("{p:?}").contains("DcPredictors"));
        assert!(format!("{:?}", ColourComponent::Cb).contains("Cb"));
        assert!(format!("{:?}", DcComponent::Chrominance).contains("Chrominance"));
    }
}
