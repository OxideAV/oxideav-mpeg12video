//! MPEG-2 §6.2.5 / §6.2.6 macroblock-level block-stream driver per
//! **ISO/IEC 13818-2 (ITU-T H.262)** — the wrapper that walks a
//! macroblock's `pattern_code[12]` array and dispatches the §6.2.6
//! `block(i)` driver
//! ([`crate::mpeg2_block_decoder::decode_block`]) once per coded
//! slot, returning a `Vec` of decoded blocks paired with their
//! §6.1.1.8 block-index position.
//!
//! ## What §6.2.5 / §6.2.6 specifies (driver shape)
//!
//! Page 53 of ISO/IEC 13818-2:1995 gives the macroblock-level loop:
//!
//! ```text
//! macroblock() {
//!     ...
//!     coded_block_pattern()   // derives pattern_code[12]
//!     for (i = 0; i < block_count; i++) {
//!         block(i)             // §6.2.6 — gated by pattern_code[i]
//!     }
//! }
//! ```
//!
//! Per **§6.1.1.8** the `block_count` per macroblock depends on the
//! [`crate::sequence_extension::ChromaFormat`] of the active
//! sequence:
//!
//! * **4:2:0** — 6 blocks. Figure 6-10 numbers them
//!   Y0=0, Y1=1, Y2=2, Y3=3, Cb=4, Cr=5.
//! * **4:2:2** — 8 blocks. Figure 6-11 numbers them
//!   Y0=0, Y1=1, Y2=2, Y3=3, Cb0=4, Cb1=5, Cr0=6, Cr1=7.
//! * **4:4:4** — 12 blocks. Figure 6-12 numbers them
//!   Y0=0, Y1=1, Y2=2, Y3=3, Cb0=4, Cb1=5, Cb2=6, Cb3=7,
//!   Cr0=8, Cr1=9, Cr2=10, Cr3=11.
//!
//! The figure-driven `i → component` mapping is exactly the one
//! [`block_component`] encodes; the same mapping bounds the choice
//! of Table B-12 vs Table B-13 in §7.2.1 (luma vs chroma) and the
//! §7.4.2.1 / Table 7-5 weighting-matrix selection (luma vs chroma
//! weighting matrix for 4:2:2 / 4:4:4).
//!
//! ## What this driver does NOT do
//!
//! * It does **not** parse the macroblock header bits
//!   (`macroblock_address_increment`, `macroblock_type`,
//!   `quantiser_scale_code`, motion vectors,
//!   `coded_block_pattern()`). Those already live in their own
//!   modules; this driver consumes the parsed `pattern_code[12]`
//!   bitmap and the per-macroblock flags.
//! * It does **not** run the §7.6.4 forming-predictions step or the
//!   §7.6.7 / §7.6.8 prediction-combine / add-and-saturate steps.
//!   The result of *this* driver is the post-IDCT `f[y][x]` plane
//!   per coded slot (the entry point for §7.6.8 add-and-saturate).
//!   Stitching the per-block `f[y][x]` into a full prediction +
//!   reconstruct pipeline is the next layer up — see
//!   [`crate::macroblock_pipeline`].
//! * It does **not** model `dct_type` (frame vs field DCT). The
//!   inverse-scan / inverse-quant / IDCT pipeline operates on an
//!   8×8 block in the same way for either DCT type; `dct_type` only
//!   affects how the sample plane is laid out in the macroblock,
//!   which is a higher-layer concern.
//! * It does **not** decode MPEG-1 streams. The MPEG-1 macroblock /
//!   block syntax uses different VLC tables and predictor reset
//!   semantics and is the subject of a separate driver.
//!
//! Spec citations refer to **ISO/IEC 13818-2:1995** (Recommendation
//! ITU-T H.262 (1995 E)) §6.1.1.8 (block ordering), §6.2.5 /
//! §6.2.6 (`macroblock()` / `block(i)` syntax), §6.3.17.4
//! (`pattern_code[12]` derivation), §7.2.1 / §7.2.2 / §7.3 / §7.4
//! / §A (the per-block reconstruction chain composed by
//! [`crate::mpeg2_block_decoder::decode_block`]).

use oxideav_core::bits::BitReader;

use crate::coded_block_pattern::CodedBlockPattern;
use crate::macroblock_type::MacroblockType;
use crate::mpeg2_block_dc::{ColourComponent, DcPredictors};
use crate::mpeg2_block_decoder::{
    decode_block as decode_block_inner, BlockContext, DecodedBlock as InnerDecodedBlock,
};
use crate::mpeg2_dequantize::{
    select_weighting_matrix_index, BlockCoding, Component as DequantComponent,
    DEFAULT_INTRA_WEIGHT, DEFAULT_NON_INTRA_WEIGHT,
};
use crate::sequence_extension::ChromaFormat;
use crate::{Error, Result};

/// Per-macroblock block count per **§6.1.1.8** (Figures 6-10 /
/// 6-11 / 6-12).
///
/// * `Yuv420` → 6 blocks.
/// * `Yuv422` → 8 blocks.
/// * `Yuv444` → 12 blocks.
///
/// Matches [`crate::macroblock_pipeline::blocks_per_macroblock`]
/// — repeated here so this driver is self-contained for audit and
/// callers depending on this module don't have to import the
/// §7.6 macroblock-pipeline module too.
pub const fn block_count(chroma: ChromaFormat) -> usize {
    match chroma {
        ChromaFormat::Yuv420 => 6,
        ChromaFormat::Yuv422 => 8,
        ChromaFormat::Yuv444 => 12,
    }
}

/// Map a §6.1.1.8 block index to its colour component per
/// Figure 6-10 / Figure 6-11 / Figure 6-12.
///
/// * `i ∈ 0..=3` → [`ColourComponent::Y`] for every chroma format.
/// * `i ∈ 4..=block_count(chroma)` → Cb or Cr per the figures:
///   * **4:2:0** — index 4 = Cb, index 5 = Cr.
///   * **4:2:2** — indices 4..=5 = Cb, indices 6..=7 = Cr.
///   * **4:4:4** — indices 4..=7 = Cb, indices 8..=11 = Cr.
///
/// Returns `None` for `i >= block_count(chroma)` (the trailing
/// `pattern_code[]` slots that don't exist in the current chroma
/// format).
pub fn block_component(i: usize, chroma: ChromaFormat) -> Option<ColourComponent> {
    if i >= block_count(chroma) {
        return None;
    }
    if i < 4 {
        return Some(ColourComponent::Y);
    }
    let chroma_pair_count = match chroma {
        ChromaFormat::Yuv420 => 1, // 1 Cb + 1 Cr
        ChromaFormat::Yuv422 => 2, // 2 Cb + 2 Cr
        ChromaFormat::Yuv444 => 4, // 4 Cb + 4 Cr
    };
    let cb_first = 4;
    let cr_first = cb_first + chroma_pair_count;
    if i < cr_first {
        Some(ColourComponent::Cb)
    } else {
        Some(ColourComponent::Cr)
    }
}

/// Macroblock-level constants this driver consumes once per
/// macroblock — the §6.2.5 / §6.3.17 parsed fields plus the
/// picture-coding-extension flags from §6.2.3.1.
///
/// Per-block fields (`component`, `macroblock_intra`, `weight`)
/// are derived per slot inside the driver from the parsed
/// [`MacroblockType`], the [`ChromaFormat`], and Table 7-5 via
/// [`select_weighting_matrix_index`].
///
/// `weight_matrices` carries the four §6.3.7 / §6.3.11.1
/// weighting matrices (`w ∈ {0, 1, 2, 3}` per Table 7-5). For
/// 4:2:0 only `w ∈ {0, 1}` are consulted (chroma shares the luma
/// matrix); for 4:2:2 / 4:4:4 all four are consulted. The default
/// matrices from §6.3.7 are exposed by
/// [`Self::with_default_weight_matrices`].
#[derive(Debug, Clone, Copy)]
pub struct MacroblockBlockContext<'w> {
    /// `intra_vlc_format` from `picture_coding_extension()` §6.2.3.1.
    /// Drives §7.2.2.1 Table 7-3 (B-14 vs B-15).
    pub intra_vlc_format: bool,
    /// `alternate_scan` from `picture_coding_extension()` §6.2.3.1.
    /// Drives §7.3 Figure 7-2 vs Figure 7-3.
    pub alternate_scan: bool,
    /// `intra_dc_precision` from `picture_coding_extension()`
    /// §6.2.3.1. Table 6-13 value, `0..=3`. Drives Table 7-2 (DC
    /// predictor reset) and Table 7-4 (`intra_dc_mult`).
    pub intra_dc_precision: u8,
    /// `quantiser_scale_value` — the resolved §7.4.2.2 Table 7-6
    /// value (post-lookup against `quantiser_scale_code` +
    /// `q_scale_type`). Must be in `1..=112`.
    pub quantiser_scale_value: u8,
    /// `chroma_format` from `sequence_extension()` §6.2.2.3. Bounds
    /// the block walk and drives Table 7-5 indexing.
    pub chroma_format: ChromaFormat,
    /// Four 8×8 weighting matrices, indexed by Table 7-5 `w`:
    /// `[0]` = intra luma, `[1]` = non-intra luma, `[2]` = intra
    /// chroma, `[3]` = non-intra chroma. The §6.3.7 defaults are
    /// the right thing for `Self::with_default_weight_matrices`.
    pub weight_matrices: &'w [[[u8; 8]; 8]; 4],
}

/// The default §6.3.7 weighting-matrix table (`w ∈ {0, 1, 2, 3}`)
/// in the order Table 7-5 indexes them: intra luma, non-intra luma,
/// intra chroma, non-intra chroma. All four chroma cells share the
/// same default matrix as their luma counterpart; encoders may
/// download the chroma matrices independently per §6.2.2.4 /
/// §6.3.11.1.
pub const DEFAULT_WEIGHT_MATRICES: [[[u8; 8]; 8]; 4] = [
    DEFAULT_INTRA_WEIGHT,
    DEFAULT_NON_INTRA_WEIGHT,
    DEFAULT_INTRA_WEIGHT,
    DEFAULT_NON_INTRA_WEIGHT,
];

impl<'w> MacroblockBlockContext<'w> {
    /// Construct a [`MacroblockBlockContext`] with the §6.3.7
    /// default weighting matrices. The static
    /// [`DEFAULT_WEIGHT_MATRICES`] table is referenced so the
    /// returned value's lifetime is `'static`.
    pub fn with_default_weight_matrices(
        intra_vlc_format: bool,
        alternate_scan: bool,
        intra_dc_precision: u8,
        quantiser_scale_value: u8,
        chroma_format: ChromaFormat,
    ) -> MacroblockBlockContext<'static> {
        MacroblockBlockContext {
            intra_vlc_format,
            alternate_scan,
            intra_dc_precision,
            quantiser_scale_value,
            chroma_format,
            weight_matrices: &DEFAULT_WEIGHT_MATRICES,
        }
    }
}

/// One decoded block plus its §6.1.1.8 position.
///
/// `block_index` is `0..=block_count(chroma_format) - 1` and maps
/// per [`block_component`] to a colour component (Y / Cb / Cr).
#[derive(Debug, Clone)]
pub struct DecodedBlock {
    /// Position of this block per Figures 6-10 / 6-11 / 6-12.
    pub block_index: u8,
    /// Colour component per [`block_component`].
    pub component: ColourComponent,
    /// Inner driver output: `QFS[]`, `QF[v][u]`, `F[v][u]`,
    /// `f[y][x]`, plus the post-EOB bit cursor.
    pub decoded: InnerDecodedBlock,
}

/// Drive the §6.2.5 / §6.2.6 macroblock-block loop from a
/// `BitReader` cursor positioned at the first block's syntax start.
///
/// The driver:
///
/// 1. Computes `pattern_code[12]` from `cbp.pattern_code(intra,
///    macroblock_pattern)`.
/// 2. Walks `i ∈ 0..block_count(chroma_format)`.
/// 3. For each `i` where `pattern_code[i] == true`:
///    * Derives the colour component `cc` per
///      [`block_component`] (Figures 6-10 / 6-11 / 6-12).
///    * Selects the §7.4.2.1 weighting matrix index `w` per
///      Table 7-5 via [`select_weighting_matrix_index`].
///    * Calls [`crate::mpeg2_block_decoder::decode_block`] to
///      consume one §6.2.6 `block(i)` from the bitstream cursor.
///    * Pushes a [`DecodedBlock`] with the block_index, component,
///      and the inner driver output.
///
/// Uncoded slots (`pattern_code[i] == false`) are not consumed
/// from the bitstream — their §6.2.6 body is empty per the
/// `if (pattern_code[i])` gate.
///
/// Per §7.2.1 (page 71) the DC predictors are reset:
///
/// * Before the first macroblock of a slice — the caller of this
///   driver handles that (this is a per-macroblock driver, so it
///   never crosses a slice boundary on its own).
/// * On every non-intra macroblock (i.e. when `mt.macroblock_intra
///   == false`) — the driver does this here, before walking any
///   blocks.
/// * On every skipped macroblock — also handled by the slice-layer
///   driver, not here (skipped MBs have no `macroblock()` syntax
///   to walk and thus never reach this entry point).
///
/// # Errors
///
/// * Propagates every [`Error::ShortHeader`] / [`Error::InvalidBitstream`]
///   raised by [`crate::mpeg2_block_decoder::decode_block`] on the
///   first failing coded block — no further blocks are walked.
/// * [`Error::InvalidBitstream`] if the caller passed
///   `dc_predictors.intra_dc_precision != intra_dc_precision`
///   (the inner driver validates this too, but failing fast at
///   the macroblock-block driver makes the call site obvious).
/// * [`Error::InvalidBitstream`] if `intra_dc_precision > 3` or
///   `quantiser_scale_value == 0` (Table 6-13 / Table 7-6
///   forbidden values).
pub fn decode_macroblock_blocks(
    br: &mut BitReader<'_>,
    ctx: &MacroblockBlockContext<'_>,
    dc_predictors: &mut DcPredictors,
    mt: &MacroblockType,
    cbp: &CodedBlockPattern,
) -> Result<Vec<DecodedBlock>> {
    // §7.2.1: validate macroblock-level constants up-front. The
    // inner driver re-validates these per block but failing fast
    // here points the diagnostic at the macroblock-block driver
    // call site (not the deeper §6.2.6 stack frame).
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
            "DcPredictors.intra_dc_precision must match MacroblockBlockContext.intra_dc_precision (§7.2.1)",
        ));
    }

    // §7.2.1: reset the DC predictors on every non-intra
    // macroblock. The slice-layer caller is responsible for the
    // "start of slice" and "skipped macroblock" resets — those
    // are events that happen outside the `macroblock()` syntax.
    if !mt.macroblock_intra {
        dc_predictors.reset();
    }

    let pattern_code = cbp.pattern_code(mt.macroblock_intra, mt.macroblock_pattern);
    let blocks = block_count(ctx.chroma_format);

    // Build the per-block context once; only the per-block
    // `(component, macroblock_intra, weight)` triplet changes
    // across the loop.
    let inner_ctx = BlockContext {
        intra_vlc_format: ctx.intra_vlc_format,
        alternate_scan: ctx.alternate_scan,
        intra_dc_precision: ctx.intra_dc_precision,
        quantiser_scale_value: ctx.quantiser_scale_value,
    };

    let coding = if mt.macroblock_intra {
        BlockCoding::Intra
    } else {
        BlockCoding::NonIntra
    };

    let mut out: Vec<DecodedBlock> = Vec::new();
    for (i, &coded) in pattern_code.iter().enumerate().take(blocks) {
        if !coded {
            continue;
        }
        // §6.1.1.8: block_index → colour component. block_count
        // guards the slice so block_component cannot fail here,
        // but unwrap is documented as defensive.
        let component = block_component(i, ctx.chroma_format).ok_or(Error::InvalidBitstream(
            "block_component: block_index out of range for chroma_format (§6.1.1.8)",
        ))?;
        // §7.4.2.1 Table 7-5: select the weighting matrix `w` from
        // (coding, component, chroma_format).
        let dequant_component = match component {
            ColourComponent::Y => DequantComponent::Luminance,
            ColourComponent::Cb | ColourComponent::Cr => DequantComponent::Chrominance,
        };
        let w = select_weighting_matrix_index(coding, dequant_component, ctx.chroma_format);
        let weight = &ctx.weight_matrices[w as usize];

        let decoded = decode_block_inner(
            br,
            &inner_ctx,
            dc_predictors,
            component,
            mt.macroblock_intra,
            weight,
        )?;
        out.push(DecodedBlock {
            block_index: i as u8,
            component,
            decoded,
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    //! §6.2.5 / §6.2.6 macroblock-block-driver coverage. The
    //! per-block §7.2.1 / §7.2.2 / §7.3 / §7.4 / §A arithmetic is
    //! already pinned by the sibling-module tests; these tests
    //! cover only the driver's composition behaviour:
    //!
    //! * §6.1.1.8 block-index → component mapping for all three
    //!   chroma formats.
    //! * `pattern_code[]` gating (uncoded slots are not consumed
    //!   from the bitstream).
    //! * Table 7-5 weighting-matrix dispatch (luma vs chroma).
    //! * §7.2.1 non-intra-macroblock predictor reset.
    //! * Argument validation up-front (forbidden
    //!   `intra_dc_precision` / `quantiser_scale_value`,
    //!   predictor / context precision mismatch).
    use super::*;

    use crate::macroblock_type::MacroblockType;
    use crate::sequence_extension::ChromaFormat;
    use oxideav_core::bits::BitWriter;

    // ---- Helpers (mirroring mpeg2_block_decoder's test layout) ----

    /// Table B-14 EOB = `10` (2 bits).
    const EOB_B14_CODE: u32 = 0b10;
    const EOB_B14_BITS: u32 = 2;

    /// Table B-12 (`dct_dc_size_luminance`) size 0 → `100` (3 bits).
    fn write_b12_size_zero(bw: &mut BitWriter) {
        bw.write_u32(0b100, 3);
    }

    /// Table B-13 (`dct_dc_size_chrominance`) size 0 → `00` (2 bits).
    fn write_b13_size_zero(bw: &mut BitWriter) {
        bw.write_u32(0b00, 2);
    }

    /// Tail-pad with a `0` and align to a byte so the BitReader has
    /// at least one trailing byte to load past the payload.
    fn pad(mut bw: BitWriter) -> Vec<u8> {
        bw.write_bit(false);
        bw.align_to_byte();
        bw.finish()
    }

    /// Emit the wire syntax for one intra block whose DC-size is 0
    /// (Table B-12 / B-13 `size = 0`) and whose residual is just an
    /// immediate end_of_block. Six bits total (Y) or five (Cb/Cr).
    fn write_size_zero_intra_block(bw: &mut BitWriter, component: ColourComponent) {
        match component {
            ColourComponent::Y => write_b12_size_zero(bw),
            ColourComponent::Cb | ColourComponent::Cr => write_b13_size_zero(bw),
        }
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
    }

    fn mt_intra() -> MacroblockType {
        MacroblockType {
            macroblock_quant: false,
            macroblock_motion_forward: false,
            macroblock_motion_backward: false,
            macroblock_pattern: false,
            macroblock_intra: true,
            spatial_temporal_weight_code_flag: false,
            bit_position_after: 0,
        }
    }

    fn mt_inter_pattern() -> MacroblockType {
        MacroblockType {
            macroblock_quant: false,
            macroblock_motion_forward: true,
            macroblock_motion_backward: false,
            macroblock_pattern: true,
            macroblock_intra: false,
            spatial_temporal_weight_code_flag: false,
            bit_position_after: 0,
        }
    }

    fn cbp_full() -> CodedBlockPattern {
        CodedBlockPattern {
            cbp: 0b111111,
            coded_block_pattern_1: None,
            coded_block_pattern_2: None,
            bit_position_after: 0,
        }
    }

    fn default_ctx(chroma: ChromaFormat) -> MacroblockBlockContext<'static> {
        MacroblockBlockContext::with_default_weight_matrices(
            false, // intra_vlc_format
            false, // alternate_scan
            0,     // intra_dc_precision
            8,     // quantiser_scale_value
            chroma,
        )
    }

    // ---- block_component coverage ----

    #[test]
    fn block_component_420_assigns_first_four_to_luma() {
        for i in 0..4 {
            assert_eq!(
                block_component(i, ChromaFormat::Yuv420),
                Some(ColourComponent::Y),
                "i={i}",
            );
        }
    }

    #[test]
    fn block_component_420_assigns_index_4_to_cb_index_5_to_cr() {
        assert_eq!(
            block_component(4, ChromaFormat::Yuv420),
            Some(ColourComponent::Cb),
        );
        assert_eq!(
            block_component(5, ChromaFormat::Yuv420),
            Some(ColourComponent::Cr),
        );
    }

    #[test]
    fn block_component_420_returns_none_past_block_count() {
        for i in 6..16 {
            assert_eq!(
                block_component(i, ChromaFormat::Yuv420),
                None,
                "i={i} is past the 4:2:0 block_count",
            );
        }
    }

    #[test]
    fn block_component_422_assigns_cb_to_4_5_and_cr_to_6_7() {
        for i in 4..=5 {
            assert_eq!(
                block_component(i, ChromaFormat::Yuv422),
                Some(ColourComponent::Cb),
                "i={i}",
            );
        }
        for i in 6..=7 {
            assert_eq!(
                block_component(i, ChromaFormat::Yuv422),
                Some(ColourComponent::Cr),
                "i={i}",
            );
        }
    }

    #[test]
    fn block_component_444_assigns_cb_to_4_7_and_cr_to_8_11() {
        for i in 4..=7 {
            assert_eq!(
                block_component(i, ChromaFormat::Yuv444),
                Some(ColourComponent::Cb),
                "i={i}",
            );
        }
        for i in 8..=11 {
            assert_eq!(
                block_component(i, ChromaFormat::Yuv444),
                Some(ColourComponent::Cr),
                "i={i}",
            );
        }
    }

    #[test]
    fn block_count_matches_section_6_1_1_8() {
        assert_eq!(block_count(ChromaFormat::Yuv420), 6);
        assert_eq!(block_count(ChromaFormat::Yuv422), 8);
        assert_eq!(block_count(ChromaFormat::Yuv444), 12);
    }

    // ---- Driver behaviour ----

    #[test]
    fn intra_macroblock_420_walks_six_blocks_yyyy_cb_cr() {
        // Build six §6.2.6 intra blocks back-to-back. Four luma
        // blocks (Table B-12 size 0 + EOB), then Cb (Table B-13
        // size 0 + EOB), then Cr (Table B-13 size 0 + EOB).
        let mut bw = BitWriter::new();
        for _ in 0..4 {
            write_size_zero_intra_block(&mut bw, ColourComponent::Y);
        }
        write_size_zero_intra_block(&mut bw, ColourComponent::Cb);
        write_size_zero_intra_block(&mut bw, ColourComponent::Cr);
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);

        let ctx = default_ctx(ChromaFormat::Yuv420);
        let mut dc = DcPredictors::new(0).unwrap();
        // Intra macroblock with `macroblock_pattern = false` — per
        // §6.3.17.4 the derivation collapses to "every slot coded"
        // for an intra MB, independent of the cbp byte.
        let mt = mt_intra();
        let cbp = CodedBlockPattern {
            cbp: 0,
            coded_block_pattern_1: None,
            coded_block_pattern_2: None,
            bit_position_after: 0,
        };

        let out =
            decode_macroblock_blocks(&mut br, &ctx, &mut dc, &mt, &cbp).expect("six blocks decode");
        assert_eq!(out.len(), 6, "4:2:0 intra MB has six coded blocks");
        // Block indices walk 0..=5 in order.
        let indices: Vec<u8> = out.iter().map(|b| b.block_index).collect();
        assert_eq!(indices, vec![0, 1, 2, 3, 4, 5]);
        // Components: Y, Y, Y, Y, Cb, Cr.
        let comps: Vec<ColourComponent> = out.iter().map(|b| b.component).collect();
        assert_eq!(
            comps,
            vec![
                ColourComponent::Y,
                ColourComponent::Y,
                ColourComponent::Y,
                ColourComponent::Y,
                ColourComponent::Cb,
                ColourComponent::Cr,
            ]
        );
        // All six QFS[0] equal the §7.2.1 Table 7-2 reset value
        // (128 at intra_dc_precision = 0), since every block has
        // dct_diff = 0 and the predictor walks 128 → 128.
        for db in &out {
            assert_eq!(db.decoded.qfs[0], 128, "block {}", db.block_index);
        }
        // Predictors landed at 128 for each independent cell.
        assert_eq!(dc.get(ColourComponent::Y), 128);
        assert_eq!(dc.get(ColourComponent::Cb), 128);
        assert_eq!(dc.get(ColourComponent::Cr), 128);
    }

    #[test]
    fn pattern_code_gates_skip_uncoded_slots_in_bitstream() {
        // 4:2:0 inter MB with pattern_code = [F,T,F,T,F,F] (cbp =
        // 0b010100 = 20 → bits 4 and 2 set → blocks 1 and 3 coded
        // per the §6.3.17.4 derivation's bit-numbering).
        //
        // Build only two blocks in the bitstream — one per coded
        // slot — and assert the driver returns exactly two
        // decoded blocks with indices [1, 3].
        let mut bw = BitWriter::new();
        // Two non-intra blocks: each one is a `(0, +1) FIRST`
        // codeword (`1` `0`) followed by EOB (`10`). The §7.4
        // non-intra reconstruction is irrelevant here; we just
        // need the cursor advance.
        for _ in 0..2 {
            bw.write_bit(true); // (0, +1) FIRST codeword `1`
            bw.write_bit(false); // sign +
            bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        }
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);

        let ctx = default_ctx(ChromaFormat::Yuv420);
        let mut dc = DcPredictors::new(0).unwrap();
        let mt = mt_inter_pattern();
        // CBP value: pattern_code is computed from the high-to-low
        // bit numbering of `cbp` (see CodedBlockPattern::pattern_code
        // tests in src/coded_block_pattern.rs). The pattern we want
        // is [F, T, F, T, F, F], i.e. blocks 1 and 3 set: 0b010100.
        let cbp = CodedBlockPattern {
            cbp: 0b010100,
            coded_block_pattern_1: None,
            coded_block_pattern_2: None,
            bit_position_after: 0,
        };

        let out = decode_macroblock_blocks(&mut br, &ctx, &mut dc, &mt, &cbp)
            .expect("two-block walk decodes");
        assert_eq!(out.len(), 2, "only the two coded slots are walked");
        let indices: Vec<u8> = out.iter().map(|b| b.block_index).collect();
        assert_eq!(indices, vec![1, 3]);
        // Both blocks were Y (indices < 4).
        for db in &out {
            assert_eq!(db.component, ColourComponent::Y);
        }
    }

    #[test]
    fn non_intra_macroblock_resets_dc_predictors_per_section_7_2_1() {
        // §7.2.1: every non-intra macroblock resets the predictors
        // back to the Table 7-2 value. Build a non-intra MB whose
        // `pattern_code = [F; 6]` so no blocks are walked from the
        // bitstream — the only side effect under test is the
        // predictor reset itself.
        let ctx = default_ctx(ChromaFormat::Yuv420);
        let mut dc = DcPredictors::new(0).unwrap();
        // Seed the predictors to a non-reset value so we can see
        // the reset happen.
        dc.luma = 500;
        dc.cb = 600;
        dc.cr = 700;
        // Non-intra MB with `macroblock_pattern = false` → cbp
        // doesn't contribute; pattern_code is all-false.
        let mt = MacroblockType {
            macroblock_quant: false,
            macroblock_motion_forward: true,
            macroblock_motion_backward: false,
            macroblock_pattern: false,
            macroblock_intra: false,
            spatial_temporal_weight_code_flag: false,
            bit_position_after: 0,
        };
        let cbp = CodedBlockPattern {
            cbp: 0,
            coded_block_pattern_1: None,
            coded_block_pattern_2: None,
            bit_position_after: 0,
        };
        let buf = [0u8; 4];
        let mut br = BitReader::new(&buf);
        let out = decode_macroblock_blocks(&mut br, &ctx, &mut dc, &mt, &cbp)
            .expect("empty walk decodes");
        assert!(out.is_empty(), "no coded blocks");
        // §7.2.1 reset value at intra_dc_precision = 0 is 128.
        assert_eq!(dc.luma, 128);
        assert_eq!(dc.cb, 128);
        assert_eq!(dc.cr, 128);
    }

    #[test]
    fn intra_macroblock_does_not_reset_predictors_at_macroblock_start() {
        // §7.2.1: the reset is on non-intra MBs and skipped MBs.
        // Intra MBs *update* the predictors per block; they do
        // NOT reset them at MB entry. (The slice-layer driver
        // handles the slice-start reset separately.)
        let ctx = default_ctx(ChromaFormat::Yuv420);
        let mut dc = DcPredictors::new(0).unwrap();
        // Seed Y to 200 — if the driver resets it, the size-0
        // intra block's QFS[0] will land at 128 (reset) + 0
        // (diff) = 128. If the driver preserves the predictor,
        // QFS[0] lands at 200 + 0 = 200.
        dc.luma = 200;

        let mut bw = BitWriter::new();
        write_size_zero_intra_block(&mut bw, ColourComponent::Y);
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);

        // Single-block intra MB: pattern_code is all-true for intra.
        // We can't easily build a single-block MB without 4+1+1
        // blocks for 4:2:0 — so instead, use the size-zero intra
        // block for Y, but we'd need 6 of them. Instead just
        // assert pre-walk that the driver does not invoke a reset:
        // check that the predictor's first-block update produces
        // QFS[0] == 200 (preserved), not 128 (reset).
        let mt = mt_intra();
        let cbp = CodedBlockPattern {
            cbp: 0,
            coded_block_pattern_1: None,
            coded_block_pattern_2: None,
            bit_position_after: 0,
        };

        // We'd need 6 blocks in the bitstream for a full intra
        // walk; instead we just check the first block's QFS[0]
        // by emitting 6 size-zero Y blocks (cheaper than emitting
        // distinct Cb/Cr blocks, and we only assert the FIRST
        // block's QFS[0]).
        let mut bw2 = BitWriter::new();
        for _ in 0..4 {
            write_size_zero_intra_block(&mut bw2, ColourComponent::Y);
        }
        write_size_zero_intra_block(&mut bw2, ColourComponent::Cb);
        write_size_zero_intra_block(&mut bw2, ColourComponent::Cr);
        let buf2 = pad(bw2);
        let mut br2 = BitReader::new(&buf2);
        // br was for the single-block sketch; not used further.
        let _ = &mut br;

        let out = decode_macroblock_blocks(&mut br2, &ctx, &mut dc, &mt, &cbp)
            .expect("six-block intra walk");
        assert_eq!(out.len(), 6);
        // First Y block's QFS[0]: predictor was 200, dct_diff = 0,
        // so QFS[0] = 200. If the driver had reset, this would be
        // 128 instead.
        assert_eq!(
            out[0].decoded.qfs[0], 200,
            "intra MB must preserve the §7.2.1 predictor at MB entry"
        );
    }

    #[test]
    fn weighting_matrix_dispatch_uses_chroma_matrix_for_chroma_block_at_4_4_4() {
        // §7.4.2.1 Table 7-5: at 4:4:4, the intra chroma block uses
        // w = 2, which (in this test's MacroblockBlockContext)
        // points at a deliberately-rigged chroma matrix whose
        // [0][0] cell is 32 (vs the default's 16). The §7.4.1 DC
        // arithmetic for the size-zero intra chroma block at
        // intra_dc_precision = 0 then yields F[0][0] =
        // intra_dc_mult * QF[0][0] = 8 * 128 = 1024 (independent
        // of W[0][0]: §7.4.1 intra-DC path doesn't multiply by W).
        // So a direct sanity check on F[0][0] doesn't tell us
        // which matrix was chosen.
        //
        // We use a non-intra Cb block instead: §7.4.2.3 uses W
        // multiplicatively. (2*QF + Sign(QF)) * W * Qs / 32 = 3 *
        // 32 * 8 / 32 = 24 vs the default-W path's 3 * 16 * 8 / 32
        // = 12.
        let mut intra_luma = DEFAULT_INTRA_WEIGHT;
        let mut non_intra_luma = DEFAULT_NON_INTRA_WEIGHT;
        let intra_chroma = DEFAULT_INTRA_WEIGHT;
        let mut non_intra_chroma = DEFAULT_NON_INTRA_WEIGHT;
        // Set every chroma-non-intra cell to 32 (vs the default
        // 16) so the §7.4 arithmetic surfaces the table choice.
        for row in non_intra_chroma.iter_mut() {
            for cell in row.iter_mut() {
                *cell = 32;
            }
        }
        // Leave luma matrices at the default so a luma block (if
        // we accidentally route there) is recognisable.
        for row in intra_luma.iter_mut() {
            for cell in row.iter_mut() {
                *cell = 16;
            }
        }
        for row in non_intra_luma.iter_mut() {
            for cell in row.iter_mut() {
                *cell = 16;
            }
        }
        let matrices = [intra_luma, non_intra_luma, intra_chroma, non_intra_chroma];

        let ctx = MacroblockBlockContext {
            intra_vlc_format: false,
            alternate_scan: false,
            intra_dc_precision: 0,
            quantiser_scale_value: 8,
            chroma_format: ChromaFormat::Yuv444,
            weight_matrices: &matrices,
        };

        // 4:4:4 non-intra MB; pattern_code with only block index 4
        // (first Cb block under Figure 6-12). cbp = 0b000010_000000
        // — but the §6.3.17.4 derivation in this crate's
        // implementation reads cbp + coded_block_pattern_1 +
        // coded_block_pattern_2 differently — we can verify the
        // mapping by toggling block_index 4 specifically.
        //
        // Simpler: build the input so block 4 is the only coded
        // slot, and look at the F_quant[0][0] of the returned
        // DecodedBlock (skipping the predictor question entirely:
        // non-intra has no DC path).
        let mut bw = BitWriter::new();
        // Block 4 body: `(0, +1) FIRST` `10` then EOB `10`.
        bw.write_bit(true); // codeword `1` for level +1 at run 0 FIRST
        bw.write_bit(false); // sign positive
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
        let buf = pad(bw);
        let mut br = BitReader::new(&buf);

        let mut dc = DcPredictors::new(0).unwrap();
        let mt = mt_inter_pattern();
        // Only block 4 (first chroma in 4:4:4) coded.
        // §6.3.17.4 in this crate: pattern_code[0..6] from cbp's
        // high-to-low bits, pattern_code[6..8] from
        // coded_block_pattern_1, pattern_code[8..12] from
        // coded_block_pattern_2. So pattern_code[4] = bit 1 of
        // cbp = 0b000010 = 2.
        let cbp = CodedBlockPattern {
            cbp: 0b000010,
            coded_block_pattern_1: None,
            coded_block_pattern_2: None,
            bit_position_after: 0,
        };

        let out =
            decode_macroblock_blocks(&mut br, &ctx, &mut dc, &mt, &cbp).expect("one-block walk");
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].block_index, 4);
        assert_eq!(out[0].component, ColourComponent::Cb);
        // §7.4.2.3 with QF[0][0] = 1, W = 32, Qs = 8:
        // F''[0][0] = (2*1 + 1) * 32 * 8 / 32 = 24.
        // If the driver had picked the (default) luma matrix
        // instead, F[0][0] would be (3 * 16 * 8) / 32 = 12.
        assert_eq!(
            out[0].decoded.f_quant[0][0], 24,
            "Table 7-5 must pick the chroma weighting matrix (w = 3) for a non-intra chroma block"
        );
    }

    #[test]
    fn intra_dc_precision_above_three_is_rejected() {
        let ctx = MacroblockBlockContext {
            intra_vlc_format: false,
            alternate_scan: false,
            intra_dc_precision: 4,
            quantiser_scale_value: 8,
            chroma_format: ChromaFormat::Yuv420,
            weight_matrices: &DEFAULT_WEIGHT_MATRICES,
        };
        let buf = [0u8; 1];
        let mut br = BitReader::new(&buf);
        let mut dc = DcPredictors::new(0).unwrap();
        let mt = mt_intra();
        let cbp = cbp_full();
        let result = decode_macroblock_blocks(&mut br, &ctx, &mut dc, &mt, &cbp);
        assert!(matches!(result, Err(Error::InvalidBitstream(_))));
    }

    #[test]
    fn quantiser_scale_value_zero_is_rejected() {
        let ctx = MacroblockBlockContext {
            intra_vlc_format: false,
            alternate_scan: false,
            intra_dc_precision: 0,
            quantiser_scale_value: 0,
            chroma_format: ChromaFormat::Yuv420,
            weight_matrices: &DEFAULT_WEIGHT_MATRICES,
        };
        let buf = [0u8; 1];
        let mut br = BitReader::new(&buf);
        let mut dc = DcPredictors::new(0).unwrap();
        let mt = mt_intra();
        let cbp = cbp_full();
        let result = decode_macroblock_blocks(&mut br, &ctx, &mut dc, &mt, &cbp);
        assert!(matches!(result, Err(Error::InvalidBitstream(_))));
    }

    #[test]
    fn predictor_context_precision_mismatch_is_rejected() {
        let ctx = MacroblockBlockContext {
            intra_vlc_format: false,
            alternate_scan: false,
            intra_dc_precision: 1, // ctx says 1
            quantiser_scale_value: 8,
            chroma_format: ChromaFormat::Yuv420,
            weight_matrices: &DEFAULT_WEIGHT_MATRICES,
        };
        let buf = [0u8; 1];
        let mut br = BitReader::new(&buf);
        let mut dc = DcPredictors::new(0).unwrap(); // predictor says 0
        let mt = mt_intra();
        let cbp = cbp_full();
        let result = decode_macroblock_blocks(&mut br, &ctx, &mut dc, &mt, &cbp);
        assert!(matches!(result, Err(Error::InvalidBitstream(_))));
    }

    #[test]
    fn first_failing_block_propagates_and_stops_the_walk() {
        // Build a bitstream where the FIRST block is well-formed
        // (size 0 + EOB) but the SECOND block is malformed — the
        // walker reads an immediate B-14 EOB (`10`) at FIRST
        // position, which is illegal for a non-intra block per
        // §7.2.2.2.
        //
        // Use an intra MB instead, where the failure mode is the
        // §7.2.2 wire-position constraint: the size-9 DC + a Table
        // B-16 escape with run = 63 at walker_index = 1 → reject
        // on the second block.
        //
        // Simpler shape: emit FOUR well-formed luma blocks (so the
        // walk reaches Cb), then leave the bitstream too short
        // for the Cb block → ShortHeader on block_index 4.
        let mut bw = BitWriter::new();
        for _ in 0..4 {
            write_size_zero_intra_block(&mut bw, ColourComponent::Y);
        }
        // Cb starts: emit a 1-bit prefix `0` and then truncate (no
        // pad). Cb size-0 codeword in B-13 is `00` (2 bits); we
        // emit only 1 bit and stop, leaving the BitReader to hit
        // ShortHeader on the second size bit.
        bw.write_bit(false);
        // Don't pad — drop the trailing byte alignment.
        let buf = bw.finish();
        let mut br = BitReader::new(&buf);

        let ctx = default_ctx(ChromaFormat::Yuv420);
        let mut dc = DcPredictors::new(0).unwrap();
        let mt = mt_intra();
        let cbp = cbp_full();
        let result = decode_macroblock_blocks(&mut br, &ctx, &mut dc, &mt, &cbp);
        assert!(
            matches!(
                result,
                Err(Error::ShortHeader) | Err(Error::InvalidBitstream(_))
            ),
            "the partial Cb block must surface as Short or InvalidBitstream"
        );
    }
}
