//! MPEG-2 §6.2.6 `block(i)` driver per **ISO/IEC 13818-2 (ITU-T
//! H.262) §6.2.6** — the per-block syntax that chains the
//! already-landed §7.2.1 DC prelude (intra blocks only), §7.2.2
//! residual VLC walker, §7.3 inverse scan, §7.4 inverse
//! quantisation, and §A 8×8 IDCT into a single
//! "bitstream → `f[y][x]` plane ready for §7.6.8 add-and-saturate"
//! entry point.
//!
//! ## What §6.2.6 specifies
//!
//! Page 53 of ISO/IEC 13818-2:1995 gives the wire-level grammar:
//!
//! ```text
//! block(i) {
//!     if (pattern_code[i]) {
//!         if (macroblock_intra) {
//!             if (cc == 0)            // Y component
//!                 dct_dc_size_luminance    // Table B-12
//!                 if (dct_dc_size_luminance != 0)
//!                     dc_dct_differential  // dct_dc_size bits
//!             else                    // Cb / Cr
//!                 dct_dc_size_chrominance  // Table B-13
//!                 if (dct_dc_size_chrominance != 0)
//!                     dc_dct_differential  // dct_dc_size bits
//!         } else {
//!             dct_coeff_first          // Tables B-14 / B-15 (FIRST)
//!         }
//!         while (nextbits() != End-of-block) {
//!             dct_coeff_next           // Tables B-14 / B-15 (NEXT)
//!         }
//!         end_of_block                 // Table-dependent EOB code
//!     }
//! }
//! ```
//!
//! The §6.2.6 driver this module exposes does five back-to-back
//! steps, each delegating to its already-landed sibling:
//!
//! 1. **§7.2.1 DC prelude** (intra blocks only) — via
//!    [`crate::mpeg2_block_dc::decode_dc_block`].
//! 2. **§7.2.2 residual walker** — repeated calls to
//!    [`crate::mpeg2_dct_coeff::DctCoeffStep::parse`] until
//!    `end_of_block`, with the §7.2.2.2 NOTE 2 / NOTE 3 FIRST /
//!    NEXT alternation: for an intra block the first call after
//!    the §7.2.1 DC is `Position::Next` (because the DC already
//!    consumed the §7.2.2 "first" slot at zig-zag index 0); for a
//!    non-intra block the very first call is `Position::First`.
//! 3. **§7.3 inverse scan** — runs the §7.3
//!    `for (v) for (u) QF[v][u] = QFS[scan[alternate_scan][v][u]]`
//!    loop via [`crate::mpeg2_inverse_scan::apply_inverse_scan`]
//!    after the walker fills `QFS[0..64]`.
//! 4. **§7.4 inverse quantisation** — via
//!    [`crate::mpeg2_dequantize::inverse_quantise_block`].
//! 5. **§A 8×8 IDCT** — via [`crate::idct::idct_8x8_from_i32`],
//!    producing the 9-bit signed pel plane `[-256, +255]` ready
//!    to be combined with the §7.6.4 prediction by §7.6.8
//!    add-and-saturate.
//!
//! Each step's spec citation is named at the call site so an audit
//! trail walks straight from the wire to the §6.2.6 step number
//! and from there to the relevant sub-section.
//!
//! ## What this driver does NOT do
//!
//! * It does **not** wrap the §7.6 macroblock pipeline (which
//!   already exists at [`crate::macroblock_pipeline`]). This is the
//!   block-level entry point; the macroblock-level driver dispatches
//!   to it once per coded block according to `pattern_code[12]`.
//! * It does **not** perform the §7.4.4 MPEG-2 mismatch control as
//!   a separate step — that's already inside
//!   [`crate::mpeg2_dequantize::inverse_quantise_block`].
//! * It does **not** handle MPEG-1 streams. The MPEG-1 §2.4.2 / §2.4.3
//!   syntax uses different VLC tables (B.5a..f vs B-12 / B-13 /
//!   B-14 / B-15 / B-16) and is the subject of a separate driver.
//! * It does **not** advance any caller-held position state besides
//!   the [`oxideav_core::bits::BitReader`] cursor and the DC
//!   predictor.
//!
//! Spec citations refer to **ISO/IEC 13818-2:1995** (Recommendation
//! ITU-T H.262 (1995 E)).

use oxideav_core::bits::BitReader;

use crate::idct::idct_8x8_from_i32;
use crate::mpeg2_block_dc::{decode_dc_block, ColourComponent, DcPredictors};
use crate::mpeg2_dct_coeff::{
    CoefficientPosition, DctCoeff, DctCoeffStep, TableSelection, MAX_RUN,
};
use crate::mpeg2_dequantize::{inverse_quantise_block, BlockCoding, Component as DequantComponent};
use crate::mpeg2_inverse_scan::apply_inverse_scan;
use crate::{Error, Result};

/// Bit position of `QFS[0]` (the DC slot) — the start of the
/// 64-entry coefficient array filled by the §7.3 inverse scan.
const QFS_DC_INDEX: usize = 0;

/// Total number of coefficients in an 8×8 block (`QFS[0..64]`).
const QFS_LEN: usize = 64;

/// Inputs that don't change across all blocks of a macroblock —
/// the picture-coding-extension flags plus the per-macroblock
/// quantiser scale and the per-block weighting matrix.
///
/// Groups together the parameters that are constant across a
/// macroblock's up-to-12 blocks: `intra_vlc_format`,
/// `alternate_scan`, `q_scale_type`, `quantiser_scale_code`, and
/// `intra_dc_precision`. Per-block fields
/// (`macroblock_intra`, `component`, `weight`) move with the
/// block.
#[derive(Debug, Clone, Copy)]
pub struct BlockContext {
    /// `intra_vlc_format` from `picture_coding_extension()`.
    /// Selects Table B-14 vs Table B-15 for intra blocks per
    /// §7.2.2.1 Table 7-3.
    pub intra_vlc_format: bool,
    /// `alternate_scan` from `picture_coding_extension()`. Selects
    /// Figure 7-2 vs Figure 7-3 in §7.3.
    pub alternate_scan: bool,
    /// `intra_dc_precision` from `picture_coding_extension()`.
    /// Drives Table 7-2 (DC predictor reset value) and
    /// `intra_dc_mult` (Table 7-4). Must be in `0..=3`.
    pub intra_dc_precision: u8,
    /// `quantiser_scale_value` resolved from
    /// `quantiser_scale_code` + `q_scale_type` per §7.4.2.2 (Table
    /// 7-6). Must be in `1..=112` (it is the resolved value, not
    /// the code).
    pub quantiser_scale_value: u8,
}

/// `intra_dc_mult` per Table 7-4 — `8 >> intra_dc_precision`. Local
/// helper that returns the value for the block at hand; intentionally
/// duplicates the spec arithmetic so this module is self-contained
/// for audit. The [`crate::mpeg2_dequantize::intra_dc_mult`] entry
/// point provides the validated form used at the public API surface.
fn intra_dc_mult_local(intra_dc_precision: u8) -> Result<i32> {
    match intra_dc_precision {
        0 => Ok(8),
        1 => Ok(4),
        2 => Ok(2),
        3 => Ok(1),
        _ => Err(Error::InvalidBitstream(
            "intra_dc_precision: only 0..=3 are defined (Table 6-13)",
        )),
    }
}

/// Decoded output of one §6.2.6 `block(i)` call.
///
/// Carries the post-IDCT pel plane `f[y][x]` (the entry point for
/// §7.6.8 add-and-saturate) plus the four intermediate planes a
/// caller might want for tracing/verification:
///
/// * [`Self::qfs`] — the `QFS[0..64]` array right after the §7.2.2
///   walker.
/// * [`Self::qf`] — the `QF[v][u]` matrix after the §7.3 inverse
///   scan.
/// * [`Self::f_quant`] — the `F[v][u]` matrix after the §7.4
///   inverse-quantisation pipeline (saturation + §7.4.4 mismatch
///   control included).
/// * [`Self::f_pel`] — the `f[y][x]` IDCT output in the §7.5
///   9-bit signed pel range `[-256, +255]`.
/// * [`Self::end_of_block_bit_position`] — bit cursor after the
///   EOB codeword was consumed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecodedBlock {
    /// The §7.2.2 walker output, in zig-zag order.
    pub qfs: [i32; QFS_LEN],
    /// The §7.3 inverse-scan output `QF[v][u]`.
    pub qf: [[i32; 8]; 8],
    /// The §7.4 inverse-quantisation output `F[v][u]`.
    pub f_quant: [[i32; 8]; 8],
    /// The §A IDCT output `f[y][x]`.
    pub f_pel: [[i16; 8]; 8],
    /// Bit position after `end_of_block` was consumed.
    pub end_of_block_bit_position: u64,
}

/// Decode one §6.2.6 `block(i)` from the bitstream cursor `br` into
/// a fully reconstructed `f[y][x]` plane.
///
/// `ctx` carries the constant macroblock-level parameters
/// ([`BlockContext`]); `dc_predictors` is the per-component DC
/// predictor state ([`DcPredictors`]); `component` is the colour
/// component (`Y` / `Cb` / `Cr`) per Table 7-1; `macroblock_intra`
/// is the §6.3.17.1 flag; `weight` is the 8×8 weighting matrix
/// `W[w][v][u]` selected by §7.4.2.1
/// ([`crate::mpeg2_dequantize::select_weighting_matrix_index`] is
/// the canonical resolver).
///
/// On success the cursor has been advanced past the
/// `end_of_block` codeword and `dc_predictors` is updated (intra
/// path only).
///
/// # Errors
///
/// Propagates the underlying VLC walker errors:
///
/// * [`Error::ShortHeader`] — the bitstream ended before the
///   driver had read every demanded bit.
/// * [`Error::InvalidBitstream`] — any §7.2.1 / §7.2.2 / §7.2.2.3
///   spec constraint was violated (wrong DC range, forbidden
///   escape level, FIRST/NEXT mismatch, run + position exceeded
///   63, etc.).
#[allow(clippy::too_many_arguments)]
pub fn decode_block(
    br: &mut BitReader<'_>,
    ctx: &BlockContext,
    dc_predictors: &mut DcPredictors,
    component: ColourComponent,
    macroblock_intra: bool,
    weight: &[[u8; 8]; 8],
) -> Result<DecodedBlock> {
    // ----- Step 0: validate the macroblock-level constants. ---------
    if ctx.intra_dc_precision > 3 {
        return Err(Error::InvalidBitstream(
            "intra_dc_precision: only 0..=3 are defined (Table 6-13)",
        ));
    }
    if ctx.quantiser_scale_value == 0 {
        return Err(Error::InvalidBitstream(
            "quantiser_scale_value: 0 is forbidden (Table 7-6 entry)",
        ));
    }
    if dc_predictors.intra_dc_precision != ctx.intra_dc_precision {
        return Err(Error::InvalidBitstream(
            "DcPredictors.intra_dc_precision must match BlockContext.intra_dc_precision (§7.2.1)",
        ));
    }

    // ----- Step 1: §7.2.1 DC prelude (intra blocks only). -----------
    //
    // For an intra block the DC coefficient `QFS[0]` is signalled
    // by `dct_dc_size_*` (Table B-12 / B-13) + an optional
    // `dc_dct_differential` field, added to the per-component
    // DC predictor `dc_dct_pred[cc]`.
    //
    // For a non-intra block there is no DC prelude — the §7.2.2
    // walker starts from `dct_coeff_first` at `QFS[0]`.
    let mut qfs = [0i32; QFS_LEN];
    let mut walker_index: usize;
    let mut walker_position: CoefficientPosition;

    if macroblock_intra {
        let dc = decode_dc_block(br, dc_predictors, component)?;
        qfs[QFS_DC_INDEX] = dc.qfs_zero;
        // The §7.2.2 walker advances past `QFS[0]`; the next
        // coefficient lives at zig-zag index 1 and (per §7.2.2.2)
        // is read with `Position::Next`.
        walker_index = 1;
        walker_position = CoefficientPosition::Next;
    } else {
        walker_index = 0;
        walker_position = CoefficientPosition::First;
    }

    // ----- Step 2: §7.2.2 residual walker. --------------------------
    //
    // Each iteration consumes one `dct_coeff_*` codeword, places
    // the resulting (run, signed_level) pair into `QFS[]`, and
    // advances `walker_index`. The loop terminates on
    // `end_of_block`.
    let table = TableSelection::from_context(ctx.intra_vlc_format, macroblock_intra);
    let end_of_block_bit_position: u64 = loop {
        let step = DctCoeffStep::parse(br, table, walker_position)?;
        match step.symbol {
            DctCoeff::RunLevel {
                run, signed_level, ..
            } => {
                // §7.2.2.4 cursor update: skip `run` zeros, then
                // place the next non-zero coefficient. The §7.2.2.3
                // wire range bounds `run` to `0..=63` already, but
                // a wire-legal `(run, position)` pair can still
                // overflow `QFS[0..64]` if the encoder mis-coded the
                // block; we reject that as a §7.2.2 spec violation
                // (catches the spec's "the position of the
                // coefficient ... shall not exceed 63" constraint).
                debug_assert!(run <= MAX_RUN);
                let target_index =
                    walker_index
                        .checked_add(run as usize)
                        .ok_or(Error::InvalidBitstream(
                            "§7.2.2: run + walker position overflowed usize",
                        ))?;
                if target_index >= QFS_LEN {
                    return Err(Error::InvalidBitstream(
                        "§7.2.2: walker position + run exceeded 63",
                    ));
                }
                qfs[target_index] = i32::from(signed_level);
                walker_index = target_index + 1;
                walker_position = CoefficientPosition::Next;
            }
            DctCoeff::EndOfBlock => {
                break step.bit_position_after;
            }
        }
        if walker_index > QFS_LEN {
            // Defensive: this branch is unreachable because the
            // `target_index >= QFS_LEN` check above catches the
            // exact-boundary case (target_index == 63 leaves
            // walker_index == 64, which is allowed only because
            // the very next iteration must be EOB; if it isn't,
            // the next `RunLevel` arrival will hit the overflow
            // check on its own `target_index >= QFS_LEN`).
            return Err(Error::InvalidBitstream(
                "§7.2.2: walker advanced past zig-zag index 63 without end_of_block",
            ));
        }
    };

    // ----- Step 3: §7.3 inverse scan. -------------------------------
    //
    // `apply_inverse_scan` takes the 64-entry `QFS[]` and places
    // each entry at its `(v, u)` cell per Figure 7-2 (zig-zag) or
    // Figure 7-3 (alternate), keyed off `alternate_scan`.
    // `apply_inverse_scan` is written for `i16`, so we narrow
    // each entry here. The §7.2.2 walker bounds signed_level into
    // the i16 range and the §7.2.1 DC reconstruction is bounded
    // by `qfs_zero_max(3) == 2047`, both of which fit in i16.
    let mut qfs_narrow = [0i16; QFS_LEN];
    for (dst, &src) in qfs_narrow.iter_mut().zip(qfs.iter()) {
        // Saturating-cast is a defensive no-op here; both inputs
        // are already in-range and the cast is identity. The
        // alternative (`as i16` with overflow on out-of-range
        // values) would silently wrap on the QFS[0] = 2048 corner
        // case (which is itself rejected upstream by §7.2.1's
        // qfs_zero_max check) — saturate keeps the invariant
        // explicit at the type level.
        *dst = src.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16;
    }
    let qf_narrow = apply_inverse_scan(&qfs_narrow, ctx.alternate_scan);
    let mut qf = [[0i32; 8]; 8];
    for v in 0..8 {
        for u in 0..8 {
            qf[v][u] = i32::from(qf_narrow[v][u]);
        }
    }

    // ----- Step 4: §7.4 inverse quantisation. -----------------------
    //
    // The dequantiser folds §7.4.2.3 reconstruction, §7.4.3
    // saturation to `[-2048, 2047]`, and §7.4.4 sum-parity
    // mismatch control on `F[7][7]` into one call.
    let coding = if macroblock_intra {
        BlockCoding::Intra
    } else {
        BlockCoding::NonIntra
    };
    // §7.4.1 intra DC multiplier: only consulted for intra blocks
    // but always available since it's a 0..=3 → 8/4/2/1 lookup.
    let intra_dc_mult_value = intra_dc_mult_local(ctx.intra_dc_precision)?;
    let _ = DequantComponent::Luminance; // documentation-only — caller selected `weight` for us
    let f_quant = inverse_quantise_block(
        &qf,
        coding,
        weight,
        ctx.quantiser_scale_value,
        intra_dc_mult_value,
    );

    // ----- Step 5: §A 8×8 IDCT. -------------------------------------
    //
    // The IDCT consumes `F[v][u]` in `[-2048, +2047]` and returns
    // `f[y][x]` saturated to the 9-bit signed pel range
    // `[-256, +255]` per §7.5.
    let f_pel = idct_8x8_from_i32(&f_quant);

    Ok(DecodedBlock {
        qfs,
        qf,
        f_quant,
        f_pel,
        end_of_block_bit_position,
    })
}

#[cfg(test)]
mod tests {
    //! §6.2.6 driver coverage — assembles a bitstream out of the
    //! already-spec-pinned VLC tables and re-runs the same DC + walker
    //! arithmetic the sibling modules already pin, asserting only the
    //! driver's composition behaviour (predictor update, cursor
    //! advance, EOB detection, inverse-scan placement).
    use super::*;

    use crate::mpeg2_block_dc::ColourComponent;
    use crate::mpeg2_dequantize::{
        select_weighting_matrix_index, BlockCoding as DequantBlockCoding, Component,
        DEFAULT_INTRA_WEIGHT, DEFAULT_NON_INTRA_WEIGHT,
    };
    use crate::sequence_extension::ChromaFormat;
    use oxideav_core::bits::BitWriter;

    /// Test-only re-export of the Table B-14 EOB code (`10`, 2 bits).
    /// This is the same constant the walker uses internally; named
    /// here so the test arithmetic is self-documenting.
    const EOB_B14_CODE: u32 = 0b10;
    const EOB_B14_BITS: u32 = 2;

    /// Test-only re-export of the Table B-15 EOB code (`0110`, 4
    /// bits).
    #[allow(dead_code)]
    const EOB_B15_CODE: u32 = 0b0110;
    #[allow(dead_code)]
    const EOB_B15_BITS: u32 = 4;

    /// Emit a `dct_dc_size_luminance` codeword for size 0 (`100`,
    /// 3 bits) and size 1 (`00`, 2 bits). Sized helper mirrors the
    /// Table B-12 entries pinned in `mpeg2_block_dc::TABLE_B12`.
    fn write_b12_size(bw: &mut BitWriter, size: u8) {
        match size {
            0 => bw.write_u32(0b100, 3),
            1 => bw.write_u32(0b00, 2),
            _ => panic!("test helper covers sizes 0..=1 only; observed {size}"),
        }
    }

    /// Emit a `dct_dc_size_chrominance` codeword for size 0
    /// (`00`, 2 bits). Mirrors Table B-13 entry pinned in
    /// `mpeg2_block_dc::TABLE_B13`.
    fn write_b13_size(bw: &mut BitWriter, size: u8) {
        match size {
            0 => bw.write_u32(0b00, 2),
            _ => panic!("test helper covers size 0 only; observed {size}"),
        }
    }

    /// Tail-pad with a `0` and align to a byte so the BitReader has
    /// at least one trailing byte to load past the payload (mirrors
    /// the sibling-module test helpers).
    fn pad(mut bw: BitWriter) -> Vec<u8> {
        bw.write_bit(false);
        bw.align_to_byte();
        bw.finish()
    }

    fn default_ctx() -> BlockContext {
        BlockContext {
            intra_vlc_format: false,
            alternate_scan: false,
            intra_dc_precision: 0,
            quantiser_scale_value: 8,
        }
    }

    #[test]
    fn intra_block_with_size_zero_dc_and_immediate_eob_decodes_to_zero_residual() {
        // For an intra block with dct_dc_size = 0 and the very next
        // codeword being EOB, the §7.2.2 walker reads one EOB and
        // returns. `QFS[]` ends with one entry (DC, = predictor reset
        // = 128) and 63 zeros.
        let mut bw = BitWriter::new();
        // §7.2.1: dct_dc_size_luminance = 0 → 3-bit Table B-12 code
        // `100`; no dc_dct_differential bits follow.
        write_b12_size(&mut bw, 0);
        // §7.2.2: immediate EOB (`10`, table B-14).
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);
        let ctx = default_ctx();
        let mut dc = DcPredictors::new(0).unwrap();
        let weight = DEFAULT_INTRA_WEIGHT;

        let out =
            decode_block(&mut br, &ctx, &mut dc, ColourComponent::Y, true, &weight).expect("ok");

        // §7.2.1: predictor was 128 (Table 7-2 reset at
        // intra_dc_precision = 0); dct_diff was 0; QFS[0] = 128.
        assert_eq!(out.qfs[0], 128);
        // All other entries untouched.
        for &q in &out.qfs[1..] {
            assert_eq!(q, 0);
        }
        // §7.3 inverse-scan places QFS[0] at QF[0][0].
        assert_eq!(out.qf[0][0], 128);
        // §7.4.1: F''[0][0] = intra_dc_mult * QF[0][0] = 8 * 128 = 1024,
        // and §7.4.3 saturates that into range (no change).
        assert_eq!(out.f_quant[0][0], 1024);
        // §A IDCT of a constant DC=1024 + zero AC plane is a constant
        // 128-equivalent pel plane after the §7.5 [-256, +255]
        // clamp. Exact value depends on the IDCT scaling but it must
        // be a constant plane.
        let constant = out.f_pel[0][0];
        for v in 0..8 {
            for u in 0..8 {
                assert_eq!(out.f_pel[v][u], constant, "{v},{u}");
            }
        }
        // §7.2.1: predictor was updated to QFS[0].
        assert_eq!(dc.get(ColourComponent::Y), 128);
    }

    #[test]
    fn intra_block_with_size_one_positive_dc_updates_predictor() {
        // §7.2.1: dct_dc_size = 1, dc_dct_differential = `1` →
        // dct_diff = +1, QFS[0] = 128 + 1 = 129.
        let mut bw = BitWriter::new();
        // dct_dc_size_luminance = 1 → 2-bit code `00` per Table B-12.
        write_b12_size(&mut bw, 1);
        bw.write_bit(true); // dc_dct_differential = 1
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);
        let mut dc = DcPredictors::new(0).unwrap();
        let out = decode_block(
            &mut br,
            &default_ctx(),
            &mut dc,
            ColourComponent::Y,
            true,
            &DEFAULT_INTRA_WEIGHT,
        )
        .expect("ok");
        assert_eq!(out.qfs[0], 129);
        assert_eq!(dc.get(ColourComponent::Y), 129);
    }

    #[test]
    fn non_intra_block_with_immediate_eob_panics_at_first_position() {
        // §7.2.2.2: a non-intra block uses `Position::First`. Table
        // B-14 has no FIRST-position EOB (EOB is `10`, which is
        // NEXT-only). An immediate `10` at a non-intra block must
        // therefore be rejected (the walker sees no codeword match
        // at FIRST and falls through to the NEXT-only EOB gate which
        // is not legal).
        let mut bw = BitWriter::new();
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        // The walker should fail before consuming any further bits;
        // tail-pad with a permissive byte either way.
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);
        let mut dc = DcPredictors::new(0).unwrap();
        let result = decode_block(
            &mut br,
            &default_ctx(),
            &mut dc,
            ColourComponent::Y,
            false, // non-intra
            &DEFAULT_NON_INTRA_WEIGHT,
        );
        assert!(matches!(result, Err(Error::InvalidBitstream(_))));
    }

    #[test]
    fn non_intra_block_with_first_one_then_eob_places_signed_level_at_index_zero() {
        // §7.2.2.2: the FIRST `(0, ±1)` codeword in Table B-14 is the
        // 1-bit `1` + 1-bit sign. With sign = 0 the level is +1 and
        // QFS[0] = +1. Immediately followed by an EOB.
        let mut bw = BitWriter::new();
        // First coefficient: `1s` = `10` (level +1, run = 0).
        bw.write_bit(true); // codeword `1`
        bw.write_bit(false); // sign = positive
                             // EOB.
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);
        let mut dc = DcPredictors::new(0).unwrap();
        let ctx = default_ctx();
        let out = decode_block(
            &mut br,
            &ctx,
            &mut dc,
            ColourComponent::Y,
            false,
            &DEFAULT_NON_INTRA_WEIGHT,
        )
        .expect("ok");
        assert_eq!(out.qfs[0], 1);
        for &q in &out.qfs[1..] {
            assert_eq!(q, 0);
        }
        // §7.3 inverse scan places QFS[0] at QF[0][0].
        assert_eq!(out.qf[0][0], 1);
        // §7.4 non-intra reconstruction:
        // F''[0][0] = (2*QF + Sign(QF)) * W * Qs / 32
        //           = (2*1 + 1) * 16 * 8 / 32
        //           = 12.
        // §7.4.4 mismatch control on F[7][7] alters the last cell
        // but not [0][0]; we check [0][0] only.
        assert_eq!(out.f_quant[0][0], 12);
    }

    #[test]
    fn cb_chroma_block_routes_to_b13_and_cb_predictor() {
        // §7.2.1: a Cb block reads the dct_dc_size VLC from
        // Table B-13 (not B-12) and updates `dc_dct_pred[cb]`
        // independently of Y / Cr.
        let mut bw = BitWriter::new();
        // dct_dc_size_chrominance = 0 → 2-bit Table B-13 code `00`.
        write_b13_size(&mut bw, 0);
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);
        let mut dc = DcPredictors::new(0).unwrap();
        let out = decode_block(
            &mut br,
            &default_ctx(),
            &mut dc,
            ColourComponent::Cb,
            true,
            &DEFAULT_INTRA_WEIGHT,
        )
        .expect("ok");
        // Cb predictor reset value is 128 (Table 7-2 at
        // intra_dc_precision = 0), unchanged from Y.
        assert_eq!(out.qfs[0], 128);
        // Cb predictor is now 128 (= reset value); Y and Cr stay
        // at their independent reset values too.
        assert_eq!(dc.get(ColourComponent::Cb), 128);
        assert_eq!(dc.get(ColourComponent::Y), 128);
        assert_eq!(dc.get(ColourComponent::Cr), 128);
    }

    #[test]
    fn quantiser_scale_value_zero_is_rejected() {
        // §7.4.2.2 Table 7-6 forbids `quantiser_scale_value == 0`;
        // the driver rejects the parameter set up-front.
        let mut bw = BitWriter::new();
        write_b12_size(&mut bw, 0);
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);
        let ctx = BlockContext {
            quantiser_scale_value: 0,
            ..default_ctx()
        };
        let mut dc = DcPredictors::new(0).unwrap();
        let result = decode_block(
            &mut br,
            &ctx,
            &mut dc,
            ColourComponent::Y,
            true,
            &DEFAULT_INTRA_WEIGHT,
        );
        assert!(matches!(result, Err(Error::InvalidBitstream(_))));
    }

    #[test]
    fn intra_dc_precision_above_three_is_rejected() {
        let mut bw = BitWriter::new();
        write_b12_size(&mut bw, 0);
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);
        let ctx = BlockContext {
            intra_dc_precision: 4,
            ..default_ctx()
        };
        // DcPredictors::new also rejects, so we must build it from
        // a valid precision and then mutate.
        let mut dc = DcPredictors::new(0).unwrap();
        dc.intra_dc_precision = 4;
        let result = decode_block(
            &mut br,
            &ctx,
            &mut dc,
            ColourComponent::Y,
            true,
            &DEFAULT_INTRA_WEIGHT,
        );
        assert!(matches!(result, Err(Error::InvalidBitstream(_))));
    }

    #[test]
    fn predictor_intra_dc_precision_mismatch_is_rejected() {
        // The driver requires `dc_predictors.intra_dc_precision`
        // match the `BlockContext.intra_dc_precision` so the §7.2.1
        // reset-value invariant holds; mismatching them is a
        // call-site bug, surfaced as InvalidBitstream so it's
        // detectable from production callsites too.
        let mut bw = BitWriter::new();
        write_b12_size(&mut bw, 0);
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);
        let ctx = BlockContext {
            intra_dc_precision: 1,
            ..default_ctx()
        };
        // dc_predictors at precision = 0 vs ctx at precision = 1.
        let mut dc = DcPredictors::new(0).unwrap();
        let result = decode_block(
            &mut br,
            &ctx,
            &mut dc,
            ColourComponent::Y,
            true,
            &DEFAULT_INTRA_WEIGHT,
        );
        assert!(matches!(result, Err(Error::InvalidBitstream(_))));
    }

    #[test]
    fn intra_block_with_one_runlevel_after_dc_places_signed_level_at_index_one() {
        // §7.2.2.2: an intra block uses `Position::Next` from the
        // start of the walker (because §7.2.1 already consumed
        // QFS[0]). The first walker codeword is therefore a
        // NEXT-only entry. Table B-14's NEXT-position `(0, 1)`
        // codeword is `11s` (2 bits + sign).
        let mut bw = BitWriter::new();
        write_b12_size(&mut bw, 0); // intra DC: size = 0
                                    // First residual: NEXT (0, +1) = `11` + sign `0`.
        bw.write_u32(0b11, 2);
        bw.write_bit(false); // positive
                             // EOB.
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);
        let mut dc = DcPredictors::new(0).unwrap();
        let out = decode_block(
            &mut br,
            &default_ctx(),
            &mut dc,
            ColourComponent::Y,
            true,
            &DEFAULT_INTRA_WEIGHT,
        )
        .expect("ok");
        assert_eq!(out.qfs[0], 128); // DC = predictor reset = 128
        assert_eq!(out.qfs[1], 1); // first NEXT placed at zig-zag index 1
        for &q in &out.qfs[2..] {
            assert_eq!(q, 0);
        }
    }

    #[test]
    fn intra_block_with_run_three_then_level_places_at_index_four() {
        // After §7.2.1 the walker starts at zig-zag index 1.
        // A (run = 3, level = +1) codeword skips indices 1..=3
        // and places level at index 4.
        //
        // From Table B-14, the `(3, 1)` entry is the 5-bit code
        // `0_0111` followed by a sign bit (level = +1).
        let mut bw = BitWriter::new();
        write_b12_size(&mut bw, 0);
        // `(3, 1)` codeword + positive sign.
        bw.write_u32(0b0_0111, 5);
        bw.write_bit(false);
        // EOB.
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);
        let mut dc = DcPredictors::new(0).unwrap();
        let out = decode_block(
            &mut br,
            &default_ctx(),
            &mut dc,
            ColourComponent::Y,
            true,
            &DEFAULT_INTRA_WEIGHT,
        )
        .expect("ok");
        assert_eq!(out.qfs[0], 128);
        assert_eq!(out.qfs[1], 0);
        assert_eq!(out.qfs[2], 0);
        assert_eq!(out.qfs[3], 0);
        assert_eq!(out.qfs[4], 1);
        for &q in &out.qfs[5..] {
            assert_eq!(q, 0);
        }
    }

    #[test]
    fn alternate_scan_remaps_runlevel_placement_in_qf_matrix() {
        // With alternate_scan = true, the §7.3 placement reads
        // Figure 7-3 instead of Figure 7-2. The `QFS[1]` entry
        // therefore goes to a different `(v, u)` cell from the
        // zig-zag case. Pin that mapping by parsing the same
        // bitstream as the index-one test under both scans.
        let mut bw = BitWriter::new();
        write_b12_size(&mut bw, 0);
        bw.write_u32(0b11, 2); // NEXT (0, 1)
        bw.write_bit(false); // positive
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);
        let ctx = BlockContext {
            alternate_scan: true,
            ..default_ctx()
        };
        let mut dc = DcPredictors::new(0).unwrap();
        let out = decode_block(
            &mut br,
            &ctx,
            &mut dc,
            ColourComponent::Y,
            true,
            &DEFAULT_INTRA_WEIGHT,
        )
        .expect("ok");
        assert_eq!(out.qfs[0], 128);
        assert_eq!(out.qfs[1], 1);
        // QF[0][0] still gets the DC (zig-zag index 0 → (0, 0)
        // under both Figure 7-2 and Figure 7-3).
        assert_eq!(out.qf[0][0], 128);
        // Under Figure 7-3 (alternate), zig-zag index 1 maps to a
        // cell other than `(0, 1)`. The exact target is governed
        // by `mpeg2_inverse_scan::ALTERNATE_INVERSE_SCAN`; we
        // assert non-equality with the zig-zag mapping.
        assert_ne!(
            out.qf[0][1], 1,
            "alternate_scan should NOT place QFS[1] at (0, 1)"
        );
    }

    #[test]
    fn end_of_block_bit_position_matches_buffer_position_after_walker() {
        // Round-trip the cursor accounting: after the walker has
        // consumed every codeword the driver reports the EOB bit
        // position, which must equal `br.bit_position()` once we
        // return.
        let mut bw = BitWriter::new();
        write_b12_size(&mut bw, 0); // 3 bits
                                    // EOB.
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS); // 2 bits
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);
        let mut dc = DcPredictors::new(0).unwrap();
        let out = decode_block(
            &mut br,
            &default_ctx(),
            &mut dc,
            ColourComponent::Y,
            true,
            &DEFAULT_INTRA_WEIGHT,
        )
        .expect("ok");
        // 3 (B-12 size = 0 codeword) + 2 (EOB) = 5 bits consumed.
        assert_eq!(out.end_of_block_bit_position, 5);
        assert_eq!(br.bit_position(), 5);
    }

    #[test]
    fn block_context_is_constructible_alongside_weighting_matrix_resolver() {
        // Sanity check that the public §7.4.2.1 weighting-matrix
        // resolver composes with [`BlockContext`] / [`decode_block`]
        // — i.e. callers can use `select_weighting_matrix_index` to
        // pick a per-block weight without an extra adapter.
        let idx = select_weighting_matrix_index(
            DequantBlockCoding::Intra,
            Component::Luminance,
            ChromaFormat::Yuv420,
        );
        // Spec-equivalent index for the §6.3.7 default intra matrix.
        assert_eq!(idx, 0);
    }

    #[test]
    fn intra_block_intra_dc_precision_one_uses_table_7_2_reset_256() {
        // §7.2.1 + Table 7-2: at `intra_dc_precision == 1` the DC
        // predictor resets to 256 and `intra_dc_mult` becomes 4
        // (Table 7-4). A size-0 DC therefore yields QFS[0] = 256
        // and F''[0][0] = 4 * 256 = 1024.
        let mut bw = BitWriter::new();
        write_b12_size(&mut bw, 0);
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);
        let ctx = BlockContext {
            intra_dc_precision: 1,
            ..default_ctx()
        };
        let mut dc = DcPredictors::new(1).unwrap();
        let out = decode_block(
            &mut br,
            &ctx,
            &mut dc,
            ColourComponent::Y,
            true,
            &DEFAULT_INTRA_WEIGHT,
        )
        .expect("ok");
        assert_eq!(out.qfs[0], 256);
        assert_eq!(out.qf[0][0], 256);
        assert_eq!(out.f_quant[0][0], 1024);
    }

    #[test]
    fn intra_dc_mult_local_matches_table_7_4() {
        assert_eq!(intra_dc_mult_local(0).unwrap(), 8);
        assert_eq!(intra_dc_mult_local(1).unwrap(), 4);
        assert_eq!(intra_dc_mult_local(2).unwrap(), 2);
        assert_eq!(intra_dc_mult_local(3).unwrap(), 1);
        assert!(intra_dc_mult_local(4).is_err());
    }

    #[test]
    fn run_plus_position_exceeding_63_is_rejected() {
        // Construct a non-intra block where the first RunLevel has
        // run = 63 (escape) and place it at walker_index = 1 to
        // overflow `QFS[]`. The driver must reject with
        // InvalidBitstream.
        //
        // Walker_index starts at 0 for non-intra; we want it at 1
        // before the offending codeword. Emit a leading FIRST
        // (0, +1) (the 1-bit `1` + sign `0`), then an escape with
        // run = 63, signed_level = +1.
        //
        // Table B-16 escape: `000001` prefix + 6-bit run + 12-bit
        // signed_level. signed_level +1 encodes as `0000 0000 0001`.
        let mut bw = BitWriter::new();
        bw.write_bit(true); // first FIRST codeword: `1` + sign
        bw.write_bit(false); // positive
                             // escape prefix
        bw.write_u32(0b000001, 6);
        bw.write_u32(63, 6); // run
        bw.write_u32(0b0000_0000_0001, 12); // signed_level = +1
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);
        let mut dc = DcPredictors::new(0).unwrap();
        let result = decode_block(
            &mut br,
            &default_ctx(),
            &mut dc,
            ColourComponent::Y,
            false,
            &DEFAULT_NON_INTRA_WEIGHT,
        );
        // walker_index after the leading (0, +1) = 1.
        // Second symbol: run = 63, target = 1 + 63 = 64 → reject.
        assert!(matches!(result, Err(Error::InvalidBitstream(_))));
    }
}
