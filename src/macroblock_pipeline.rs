//! §7.6 Decoder macroblock pipeline per ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262), page 102 — the per-macroblock composition of the
//! already-landed prediction / combine / add steps into a single
//! "block in → decoded samples out" driver, keyed off the parsed
//! `macroblock_type` and `coded_block_pattern()` derivations.
//!
//! ## What §7.6 specifies (in driver order)
//!
//! The spec breaks decoder reconstruction of a single macroblock into
//! a fixed pipeline of stages (page 102, "7.6 Picture data"):
//!
//! 1. **§7.6.3** — motion-vector reconstruction (PMV chain). Landed.
//! 2. **§7.6.4** — forming predictions (per-component pel reader).
//!    Landed; see [`crate::forming_predictions`].
//! 3. **§7.6.5** — table 7-13 / 7-14 case selection (forward /
//!    backward / both / skipped). Captured by
//!    [`crate::combine_predictions::PredictionDirection`].
//! 4. **§7.6.6** — skipped-macroblock special cases. The skipped-MB
//!    prediction is what the §7.6.4 caller already produced — the
//!    driver passes it through.
//! 5. **§7.6.7** — combining predictions (bidirectional `// 2`
//!    average, or single-direction pass-through). Landed; see
//!    [`crate::combine_predictions`].
//! 6. **§7.6.5 / §6.2.6** — inverse quantisation + IDCT to produce the
//!    transform output `f[y][x]`. *The IDCT is not landed in this
//!    crate yet.* This driver therefore takes the post-IDCT `f[]` as
//!    an input rather than producing it; the caller plugs it in
//!    (today: from a stub IDCT or a fabricated test value; tomorrow:
//!    from the §A.1 implementation when it lands).
//! 7. **§7.6.8** — `d[y][x] = saturate(f[y][x] + p[y][x])`. Landed;
//!    see [`crate::add_coefficients`].
//!
//! Within a macroblock the same pipeline runs once per **coded block**
//! — the `pattern_code[12]` array from [`crate::CodedBlockPattern`]
//! tells the driver which of the up-to-12 blocks carry coefficients;
//! uncoded blocks contribute the prediction sample plane unchanged
//! (i.e. all-zero residual under §7.6.8) for inter macroblocks, and
//! are absent (skipped entirely, undefined) for intra macroblocks.
//!
//! ## What this module provides
//!
//! * [`MacroblockKind`] — one of the four §7.6.5 / §7.6.6 cases
//!   (`Intra` / `Inter{direction}` / `Skipped`). The intra case skips
//!   the prediction-formation step entirely (the prediction is
//!   conceptually all-zero per §7.4.1 + §7.6.8 — see
//!   [`crate::add_intra_block`]).
//! * [`BlockInputs`] — per-block payload the driver consumes for one
//!   coded block. For an intra macroblock the prediction sides are
//!   ignored; for an inter macroblock the relevant prediction sides
//!   carry the §7.6.4 pel-reader output. The transform buffer is
//!   always the post-IDCT `f[y][x]`.
//! * [`decode_block`] — the inner driver for one block. Returns the
//!   `[0, 255]`-clamped decoded sample plane.
//! * [`decode_macroblock`] — the outer driver: walks the up-to-12
//!   coded-block array described by a [`CodedBlockPattern`] +
//!   `macroblock_intra` / `macroblock_pattern` and calls
//!   [`decode_block`] per coded slot. Returns the per-coded-block
//!   decoded plane plus the indices of the coded slots, in
//!   §6.3.17.4 order.
//! * [`PipelineError`] — local error variants for the driver-level
//!   bugs (input-length mismatches, missing prediction side, etc.).
//!
//! ## What this module does NOT do
//!
//! * It does **not** run the §A.1 IDCT — `transform` enters
//!   pre-IDCT'd. (See workspace issue #1110.)
//! * It does **not** run `dequantize_*_block` — the caller is expected
//!   to have used [`crate::dequantize_intra_block`] /
//!   [`crate::dequantize_non_intra_block`] before computing the IDCT.
//! * It does **not** parse the bitstream — the inputs are already
//!   parsed structural fields ([`MacroblockType`], [`CodedBlockPattern`]).
//! * It does **not** form predictions — the caller is expected to
//!   have used [`crate::predict_block`] to populate per-block
//!   prediction planes before invoking the driver.
//!
//! The driver's contract is intentionally narrow: it stitches the
//! already-spec-traceable §7.6.7 + §7.6.8 endpoints onto a per-coded-
//! block dispatch loop driven by §6.3.17.4 `pattern_code[]`. That is
//! the missing piece between "we have a slice of parsed syntax
//! elements" and "we have a slice of decoded sample planes."
//!
//! Spec citations refer to **ISO/IEC 13818-2 (H.262)** §7.6.5 through
//! §7.6.8 plus §6.3.17.4 (`pattern_code[12]` derivation).

use crate::add_coefficients::{add_intra_block, add_prediction_and_coefficients};
use crate::coded_block_pattern::CodedBlockPattern;
use crate::combine_predictions::{combine_directional_predictions, PredictionDirection};
use crate::macroblock_type::MacroblockType;
use crate::sequence_extension::ChromaFormat;

/// Which §7.6.5 / §7.6.6 case applies to the macroblock as a whole.
///
/// Derived by the caller from the parsed [`MacroblockType`] flags:
///
/// * `macroblock_intra == 1` → [`MacroblockKind::Intra`] regardless
///   of the motion flags (an intra MB carries no motion data).
/// * `macroblock_intra == 0` and `(forward, backward) == (1, 1)` →
///   [`MacroblockKind::Inter`] with
///   [`PredictionDirection::Bidirectional`].
/// * `macroblock_intra == 0` and `(forward, backward) == (1, 0)` →
///   [`MacroblockKind::Inter`] with [`PredictionDirection::Forward`].
/// * `macroblock_intra == 0` and `(forward, backward) == (0, 1)` →
///   [`MacroblockKind::Inter`] with [`PredictionDirection::Backward`].
/// * `macroblock_intra == 0` and `(forward, backward) == (0, 0)` on
///   a P-picture → [`MacroblockKind::Inter`] with
///   [`PredictionDirection::Skipped`] (the §7.6.3.5 implicit zero-MV
///   prediction the caller must have built into the forward slot of
///   each block).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MacroblockKind {
    /// `macroblock_intra == 1` — no prediction step runs;
    /// reconstruction collapses to `d = saturate(f)`.
    Intra,
    /// `macroblock_intra == 0` — inter macroblock with the given
    /// §7.6.5 prediction direction.
    Inter(PredictionDirection),
}

impl MacroblockKind {
    /// Classify a parsed [`MacroblockType`] into a [`MacroblockKind`]
    /// per §7.6.5 / §7.6.6.
    ///
    /// Returns `Inter(PredictionDirection)` even when
    /// `(forward, backward) == (0, 0)` — the [`PredictionDirection::Skipped`]
    /// variant covers that case (§7.6.3.5 implicit zero-MV
    /// prediction). The caller decides whether to *form* that
    /// prediction, the driver only labels the case.
    pub fn from_macroblock_type(mt: &MacroblockType) -> Self {
        if mt.macroblock_intra {
            Self::Intra
        } else {
            let dir = match (mt.macroblock_motion_forward, mt.macroblock_motion_backward) {
                (true, true) => PredictionDirection::Bidirectional,
                (true, false) => PredictionDirection::Forward,
                (false, true) => PredictionDirection::Backward,
                (false, false) => PredictionDirection::Skipped,
            };
            Self::Inter(dir)
        }
    }
}

/// Per-block payload the driver consumes for one coded block.
///
/// All three slices must have the same length when present — that is
/// the per-block sample count (typically 64 for the §A.1 8×8 IDCT
/// output, but the driver is geometry-agnostic and any matched
/// length works).
///
/// For an [`MacroblockKind::Intra`] block the prediction slices are
/// ignored (a caller may pass `&[]`). For an
/// [`MacroblockKind::Inter`] block the relevant prediction slices —
/// per [`PredictionDirection`] — must carry the §7.6.4 pel-reader
/// output for the block. Specifically:
///
/// * [`PredictionDirection::Forward`] uses `prediction_forward`.
/// * [`PredictionDirection::Backward`] uses `prediction_backward`.
/// * [`PredictionDirection::Bidirectional`] uses both, with matched
///   lengths.
/// * [`PredictionDirection::Skipped`] uses `prediction_forward` (the
///   spec's §7.6.3.5 implicit-zero-MV block lives there).
#[derive(Debug, Clone, Copy)]
pub struct BlockInputs<'a> {
    /// Post-IDCT transform plane `f[y][x]` for this block (the §A.1
    /// output; this driver takes it as a parameter rather than
    /// computing it — see crate-level "What this module does NOT do").
    pub transform: &'a [i16],
    /// §7.6.4 forward-prediction sample plane for this block. Used
    /// when the macroblock kind is `Inter(Forward | Bidirectional |
    /// Skipped)`; ignored for `Inter(Backward)` and `Intra`.
    pub prediction_forward: &'a [u8],
    /// §7.6.4 backward-prediction sample plane for this block. Used
    /// when the macroblock kind is `Inter(Backward | Bidirectional)`;
    /// ignored for `Inter(Forward | Skipped)` and `Intra`.
    pub prediction_backward: &'a [u8],
}

impl<'a> BlockInputs<'a> {
    /// Build a [`BlockInputs`] for an intra block (prediction sides
    /// default to empty).
    pub fn intra(transform: &'a [i16]) -> Self {
        Self {
            transform,
            prediction_forward: &[],
            prediction_backward: &[],
        }
    }

    /// Build a [`BlockInputs`] for an inter block with only a forward
    /// prediction.
    pub fn forward(transform: &'a [i16], prediction_forward: &'a [u8]) -> Self {
        Self {
            transform,
            prediction_forward,
            prediction_backward: &[],
        }
    }

    /// Build a [`BlockInputs`] for an inter block with only a backward
    /// prediction.
    pub fn backward(transform: &'a [i16], prediction_backward: &'a [u8]) -> Self {
        Self {
            transform,
            prediction_forward: &[],
            prediction_backward,
        }
    }

    /// Build a [`BlockInputs`] for an inter block with both forward
    /// and backward predictions (bidirectional).
    pub fn bidirectional(
        transform: &'a [i16],
        prediction_forward: &'a [u8],
        prediction_backward: &'a [u8],
    ) -> Self {
        Self {
            transform,
            prediction_forward,
            prediction_backward,
        }
    }
}

/// Errors raised by the macroblock-pipeline driver.
///
/// These are caller-bug errors — the driver doesn't parse bitstreams,
/// so it cannot raise an `InvalidBitstream` of its own. Each variant
/// names the precondition the caller breached.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PipelineError {
    /// The transform and prediction slices for an inter block were of
    /// different lengths. §7.6.7 / §7.6.8 require pointwise alignment.
    LengthMismatch,
    /// The macroblock kind was `Inter(Forward)` or `Inter(Skipped)`
    /// but `prediction_forward` was empty.
    MissingForwardPrediction,
    /// The macroblock kind was `Inter(Backward)` but
    /// `prediction_backward` was empty.
    MissingBackwardPrediction,
    /// The macroblock kind was `Inter(Bidirectional)` but at least one
    /// of the two prediction sides was empty.
    MissingBidirectionalPrediction,
}

impl core::fmt::Display for PipelineError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::LengthMismatch => write!(
                f,
                "mpeg12video pipeline: transform and prediction slice lengths differ (§7.6.8)"
            ),
            Self::MissingForwardPrediction => write!(
                f,
                "mpeg12video pipeline: inter block needs a forward prediction (§7.6.5 forward / skipped row)"
            ),
            Self::MissingBackwardPrediction => write!(
                f,
                "mpeg12video pipeline: inter block needs a backward prediction (§7.6.5 backward row)"
            ),
            Self::MissingBidirectionalPrediction => write!(
                f,
                "mpeg12video pipeline: bidirectional inter block needs both forward and backward predictions (§7.6.7.1)"
            ),
        }
    }
}

impl std::error::Error for PipelineError {}

/// Run the §7.6.5 → §7.6.7 → §7.6.8 endpoints for a single coded
/// block.
///
/// Behaviour by [`MacroblockKind`]:
///
/// * [`MacroblockKind::Intra`] — call [`add_intra_block`] on
///   `inputs.transform`. Prediction sides are ignored.
/// * [`MacroblockKind::Inter`] — call [`combine_directional_predictions`]
///   on the prediction sides per the direction, then
///   [`add_prediction_and_coefficients`] of the combined prediction
///   and `inputs.transform`.
///
/// The returned `Vec<u8>` has the same length as `inputs.transform`
/// (which is the same length as the prediction sides for the inter
/// cases).
///
/// Errors per [`PipelineError`]:
///
/// * [`PipelineError::LengthMismatch`] — transform and prediction
///   slices differ in length for an inter block.
/// * [`PipelineError::MissingForwardPrediction`] — forward / skipped
///   inter kind but `prediction_forward` is empty.
/// * [`PipelineError::MissingBackwardPrediction`] — backward inter
///   kind but `prediction_backward` is empty.
/// * [`PipelineError::MissingBidirectionalPrediction`] — bidirectional
///   inter kind with at least one empty side.
pub fn decode_block(
    kind: MacroblockKind,
    inputs: BlockInputs<'_>,
) -> Result<Vec<u8>, PipelineError> {
    match kind {
        MacroblockKind::Intra => {
            // §7.6.8 intra shortcut: prediction conceptually zero,
            // d = saturate(f). add_intra_block is geometry-agnostic
            // and has no failure mode (the transform slice may be
            // empty, in which case the output is empty too).
            Ok(add_intra_block(inputs.transform))
        }
        MacroblockKind::Inter(direction) => decode_inter_block(direction, inputs),
    }
}

/// Inter-block driver, factored out of [`decode_block`] for clarity.
fn decode_inter_block(
    direction: PredictionDirection,
    inputs: BlockInputs<'_>,
) -> Result<Vec<u8>, PipelineError> {
    // Validate the required prediction side(s) are present and length-
    // matched against the transform. The driver mirrors the spec's
    // §7.6.5 table — each row requires a specific subset of (forward,
    // backward).
    match direction {
        PredictionDirection::Forward | PredictionDirection::Skipped => {
            if inputs.prediction_forward.is_empty() && !inputs.transform.is_empty() {
                return Err(PipelineError::MissingForwardPrediction);
            }
            if inputs.prediction_forward.len() != inputs.transform.len() {
                return Err(PipelineError::LengthMismatch);
            }
        }
        PredictionDirection::Backward => {
            if inputs.prediction_backward.is_empty() && !inputs.transform.is_empty() {
                return Err(PipelineError::MissingBackwardPrediction);
            }
            if inputs.prediction_backward.len() != inputs.transform.len() {
                return Err(PipelineError::LengthMismatch);
            }
        }
        PredictionDirection::Bidirectional => {
            if (inputs.prediction_forward.is_empty() || inputs.prediction_backward.is_empty())
                && !inputs.transform.is_empty()
            {
                return Err(PipelineError::MissingBidirectionalPrediction);
            }
            if inputs.prediction_forward.len() != inputs.transform.len()
                || inputs.prediction_backward.len() != inputs.transform.len()
            {
                return Err(PipelineError::LengthMismatch);
            }
        }
    }

    // §7.6.7: combine the up-to-two prediction sides into a single
    // sample plane.
    let combined = combine_directional_predictions(
        direction,
        inputs.prediction_forward,
        inputs.prediction_backward,
    )
    .ok_or(PipelineError::LengthMismatch)?;

    // §7.6.8: add the IDCT output to the combined prediction with
    // [0, 255] saturation.
    add_prediction_and_coefficients(inputs.transform, &combined)
        .ok_or(PipelineError::LengthMismatch)
}

/// Per-macroblock block count for a given [`ChromaFormat`] per §6.1.1.8
/// (Table 6-1) — the number of trailing entries of `pattern_code[12]`
/// that are actually walked by the macroblock-block loop.
///
/// * `Yuv420` → 6 (`Y0..Y3` + `Cb` + `Cr`).
/// * `Yuv422` → 8 (`Y0..Y3` + `Cb0..Cb1` + `Cr0..Cr1`).
/// * `Yuv444` → 12 (`Y0..Y3` + `Cb0..Cb3` + `Cr0..Cr3`).
///
/// The §6.3.17.4 `pattern_code[]` array is always 12 entries wide; the
/// trailing slots for narrower chroma formats are zero and the
/// per-block driver simply doesn't walk them.
pub const fn blocks_per_macroblock(chroma: ChromaFormat) -> usize {
    match chroma {
        ChromaFormat::Yuv420 => 6,
        ChromaFormat::Yuv422 => 8,
        ChromaFormat::Yuv444 => 12,
    }
}

/// One decoded block plus its position in §6.3.17.4 `pattern_code[]`.
///
/// The `block_index` is the 0..=11 index into the `pattern_code[]`
/// array — i.e. its position in the spec's per-MB block iteration
/// order: blocks 0..=3 are the four luma sub-blocks, then chroma
/// follows per the chroma format (one Cb + one Cr for 4:2:0; two of
/// each for 4:2:2; four of each for 4:4:4).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecodedBlock {
    /// Position of this block in §6.3.17.4 `pattern_code[]` (0..=11).
    pub block_index: u8,
    /// Final decoded samples for the block: `width * height`-shaped
    /// `u8` plane, length matching the matching `BlockInputs.transform`
    /// the caller passed in.
    pub samples: Vec<u8>,
}

/// Walk the macroblock's coded blocks per §6.3.17.4 `pattern_code[]`
/// and run the §7.6.7 / §7.6.8 endpoints per coded slot.
///
/// `kind` is the macroblock-wide §7.6.5 case (one
/// [`MacroblockKind`] applies to every coded block in the MB).
///
/// `block_inputs` is a fixed 12-entry table indexed by the spec's
/// `pattern_code[i]` position; only entries where `pattern_code[i]
/// == true` are consulted (the others may be `BlockInputs::intra(&[])`
/// or any placeholder).
///
/// Returns one [`DecodedBlock`] per coded slot, in §6.3.17.4 walk order
/// (`block_index` ascending). The slots not flagged by `pattern_code[]`
/// are skipped — they are either uncoded (residual conceptually zero;
/// the caller is responsible for the §7.6.8 `d = p` short-circuit if
/// it wants their samples too) or chroma slots that don't exist in
/// the current [`ChromaFormat`] (their `pattern_code[]` entry is
/// always false).
///
/// `chroma` bounds the walk: the driver only inspects `pattern_code[0
/// .. blocks_per_macroblock(chroma)]`.
///
/// Errors per [`PipelineError`] — propagated from [`decode_block`]
/// for the first failing coded block.
pub fn decode_macroblock(
    kind: MacroblockKind,
    cbp: &CodedBlockPattern,
    mt: &MacroblockType,
    chroma: ChromaFormat,
    block_inputs: &[BlockInputs<'_>; 12],
) -> Result<Vec<DecodedBlock>, PipelineError> {
    let pattern_code = cbp.pattern_code(mt.macroblock_intra, mt.macroblock_pattern);
    let block_count = blocks_per_macroblock(chroma);

    let mut out = Vec::new();
    for (i, &coded) in pattern_code.iter().enumerate().take(block_count) {
        if !coded {
            continue;
        }
        let samples = decode_block(kind, block_inputs[i])?;
        out.push(DecodedBlock {
            block_index: i as u8,
            samples,
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::macroblock_type::MacroblockType;

    // Helpers to fabricate MacroblockType / CodedBlockPattern values
    // without going through the bitstream parsers — the unit tests
    // focus on the driver, not the parsers.

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

    fn mt_inter(forward: bool, backward: bool, pattern: bool) -> MacroblockType {
        MacroblockType {
            macroblock_quant: false,
            macroblock_motion_forward: forward,
            macroblock_motion_backward: backward,
            macroblock_pattern: pattern,
            macroblock_intra: false,
            spatial_temporal_weight_code_flag: false,
            bit_position_after: 0,
        }
    }

    fn cbp(value: u8) -> CodedBlockPattern {
        CodedBlockPattern {
            cbp: value,
            coded_block_pattern_1: None,
            coded_block_pattern_2: None,
            bit_position_after: 0,
        }
    }

    // ---- MacroblockKind::from_macroblock_type ----

    #[test]
    fn kind_classifies_intra_regardless_of_motion_flags() {
        // Per §6.3.17.1 macroblock_intra=1 dominates — even if a
        // bitstream weirdly sets motion flags too.
        let mut mt = mt_intra();
        assert_eq!(
            MacroblockKind::from_macroblock_type(&mt),
            MacroblockKind::Intra
        );
        mt.macroblock_motion_forward = true;
        mt.macroblock_motion_backward = true;
        assert_eq!(
            MacroblockKind::from_macroblock_type(&mt),
            MacroblockKind::Intra
        );
    }

    #[test]
    fn kind_classifies_inter_directions() {
        let mt_fwd = mt_inter(true, false, true);
        assert_eq!(
            MacroblockKind::from_macroblock_type(&mt_fwd),
            MacroblockKind::Inter(PredictionDirection::Forward)
        );
        let mt_bwd = mt_inter(false, true, true);
        assert_eq!(
            MacroblockKind::from_macroblock_type(&mt_bwd),
            MacroblockKind::Inter(PredictionDirection::Backward)
        );
        let mt_bi = mt_inter(true, true, true);
        assert_eq!(
            MacroblockKind::from_macroblock_type(&mt_bi),
            MacroblockKind::Inter(PredictionDirection::Bidirectional)
        );
        // (0, 0, 0) = §7.6.3.5 implicit zero-MV (P-skipped).
        let mt_skip = mt_inter(false, false, false);
        assert_eq!(
            MacroblockKind::from_macroblock_type(&mt_skip),
            MacroblockKind::Inter(PredictionDirection::Skipped)
        );
    }

    // ---- decode_block: intra ----

    #[test]
    fn decode_block_intra_matches_add_intra_block() {
        // §7.6.8 intra shortcut: d = saturate(f). Driver output must
        // be bit-identical to add_intra_block of the same input.
        let transform: Vec<i16> = vec![-50, 0, 25, 255, 256, -1, 100, 1000];
        let direct = add_intra_block(&transform);
        let via_driver = decode_block(MacroblockKind::Intra, BlockInputs::intra(&transform))
            .expect("intra never errors");
        assert_eq!(direct, via_driver);
    }

    #[test]
    fn decode_block_intra_ignores_prediction_sides() {
        // Intra reconstruction doesn't touch the prediction sides;
        // passing arbitrary garbage there must yield the same output.
        let transform: Vec<i16> = vec![10, -10, 100, -100];
        let bogus_fwd = vec![42u8; 4];
        let bogus_bwd = vec![99u8; 4];
        let inputs = BlockInputs {
            transform: &transform,
            prediction_forward: &bogus_fwd,
            prediction_backward: &bogus_bwd,
        };
        let via_driver = decode_block(MacroblockKind::Intra, inputs).unwrap();
        let intra_clean = add_intra_block(&transform);
        assert_eq!(via_driver, intra_clean);
    }

    #[test]
    fn decode_block_intra_empty_transform_yields_empty() {
        let out = decode_block(MacroblockKind::Intra, BlockInputs::intra(&[])).unwrap();
        assert!(out.is_empty());
    }

    // ---- decode_block: inter forward ----

    #[test]
    fn decode_block_inter_forward_matches_combine_then_add() {
        // §7.6.5 forward-only row: combine returns the forward block
        // unchanged; §7.6.8 adds the transform to it.
        let prediction = vec![10u8, 20, 30, 40];
        let transform: Vec<i16> = vec![5, -5, 10, -10];
        // 10+5=15, 20-5=15, 30+10=40, 40-10=30.
        let expected = vec![15, 15, 40, 30];
        let out = decode_block(
            MacroblockKind::Inter(PredictionDirection::Forward),
            BlockInputs::forward(&transform, &prediction),
        )
        .unwrap();
        assert_eq!(out, expected);
    }

    #[test]
    fn decode_block_inter_forward_missing_side_errors() {
        let transform: Vec<i16> = vec![0, 0, 0, 0];
        let err = decode_block(
            MacroblockKind::Inter(PredictionDirection::Forward),
            BlockInputs {
                transform: &transform,
                prediction_forward: &[],
                prediction_backward: &[],
            },
        )
        .unwrap_err();
        assert_eq!(err, PipelineError::MissingForwardPrediction);
    }

    // ---- decode_block: inter backward ----

    #[test]
    fn decode_block_inter_backward_matches_combine_then_add() {
        let prediction = vec![100u8, 110, 120, 130];
        let transform: Vec<i16> = vec![0, 0, 0, 0];
        let out = decode_block(
            MacroblockKind::Inter(PredictionDirection::Backward),
            BlockInputs::backward(&transform, &prediction),
        )
        .unwrap();
        assert_eq!(out, prediction);
    }

    #[test]
    fn decode_block_inter_backward_missing_side_errors() {
        let transform: Vec<i16> = vec![0, 0, 0, 0];
        let err = decode_block(
            MacroblockKind::Inter(PredictionDirection::Backward),
            BlockInputs {
                transform: &transform,
                prediction_forward: &[],
                prediction_backward: &[],
            },
        )
        .unwrap_err();
        assert_eq!(err, PipelineError::MissingBackwardPrediction);
    }

    // ---- decode_block: inter bidirectional ----

    #[test]
    fn decode_block_inter_bidirectional_averages_then_adds() {
        // (10+20)//2=15, (20+30)//2=25, (30+40)//2=35, (40+50)//2=45.
        let forward = vec![10u8, 20, 30, 40];
        let backward = vec![20u8, 30, 40, 50];
        let transform: Vec<i16> = vec![5, -5, 10, -10];
        // 15+5=20, 25-5=20, 35+10=45, 45-10=35.
        let expected = vec![20, 20, 45, 35];
        let out = decode_block(
            MacroblockKind::Inter(PredictionDirection::Bidirectional),
            BlockInputs::bidirectional(&transform, &forward, &backward),
        )
        .unwrap();
        assert_eq!(out, expected);
    }

    #[test]
    fn decode_block_inter_bidirectional_missing_side_errors() {
        let transform: Vec<i16> = vec![0; 4];
        let only_fwd = vec![1u8; 4];
        let err = decode_block(
            MacroblockKind::Inter(PredictionDirection::Bidirectional),
            BlockInputs {
                transform: &transform,
                prediction_forward: &only_fwd,
                prediction_backward: &[],
            },
        )
        .unwrap_err();
        assert_eq!(err, PipelineError::MissingBidirectionalPrediction);
    }

    #[test]
    fn decode_block_inter_bidirectional_length_mismatch_errors() {
        let transform: Vec<i16> = vec![0; 4];
        let fwd = vec![1u8; 4];
        let bwd = vec![2u8; 3]; // wrong length
        let err = decode_block(
            MacroblockKind::Inter(PredictionDirection::Bidirectional),
            BlockInputs {
                transform: &transform,
                prediction_forward: &fwd,
                prediction_backward: &bwd,
            },
        )
        .unwrap_err();
        assert_eq!(err, PipelineError::LengthMismatch);
    }

    // ---- decode_block: inter skipped ----

    #[test]
    fn decode_block_inter_skipped_uses_forward_slot() {
        // §7.6.3.5 implicit-zero-MV: caller has built the (0,0) MV
        // prediction into the forward slot. Driver passes it through
        // §7.6.7 (Skipped branch returns forward unchanged) and adds
        // the transform.
        let forward = vec![50u8, 60, 70, 80];
        let transform: Vec<i16> = vec![5, -5, 10, -10];
        let expected = vec![55, 55, 80, 70];
        let out = decode_block(
            MacroblockKind::Inter(PredictionDirection::Skipped),
            BlockInputs::forward(&transform, &forward),
        )
        .unwrap();
        assert_eq!(out, expected);
    }

    // ---- decode_block: length mismatch on the single-side cases ----

    #[test]
    fn decode_block_inter_forward_length_mismatch_errors() {
        let transform: Vec<i16> = vec![0; 4];
        let fwd = vec![1u8; 3]; // wrong length
        let err = decode_block(
            MacroblockKind::Inter(PredictionDirection::Forward),
            BlockInputs::forward(&transform, &fwd),
        )
        .unwrap_err();
        assert_eq!(err, PipelineError::LengthMismatch);
    }

    #[test]
    fn decode_block_inter_backward_length_mismatch_errors() {
        let transform: Vec<i16> = vec![0; 4];
        let bwd = vec![1u8; 5]; // wrong length
        let err = decode_block(
            MacroblockKind::Inter(PredictionDirection::Backward),
            BlockInputs::backward(&transform, &bwd),
        )
        .unwrap_err();
        assert_eq!(err, PipelineError::LengthMismatch);
    }

    // ---- blocks_per_macroblock ----

    #[test]
    fn blocks_per_mb_matches_chroma_format() {
        assert_eq!(blocks_per_macroblock(ChromaFormat::Yuv420), 6);
        assert_eq!(blocks_per_macroblock(ChromaFormat::Yuv422), 8);
        assert_eq!(blocks_per_macroblock(ChromaFormat::Yuv444), 12);
    }

    // ---- decode_macroblock ----

    fn placeholder_inputs() -> [BlockInputs<'static>; 12] {
        [BlockInputs::intra(&[]); 12]
    }

    #[test]
    fn decode_macroblock_intra_walks_all_blocks_for_420() {
        // Intra MB in 4:2:0 → pattern_code = [1; 6], extra slots false.
        // The driver must emit exactly six decoded blocks, with
        // block_index 0..=5 in order.
        let mt = mt_intra();
        // CBP value is irrelevant for intra (pattern_code starts at
        // macroblock_intra = true and stays so since macroblock_pattern
        // is false for intra).
        let cbp_val = cbp(0);

        // Per-block transform buffers — small, distinct so we can
        // verify the driver routed each one correctly.
        let t0: Vec<i16> = vec![10, 20, 30, 40];
        let t1: Vec<i16> = vec![50, 60, 70, 80];
        let t2: Vec<i16> = vec![-10, -20, -30, -40];
        let t3: Vec<i16> = vec![100, 200, 300, 400]; // exercises 255-clamp
        let t4: Vec<i16> = vec![1, 2, 3, 4];
        let t5: Vec<i16> = vec![5, 6, 7, 8];
        let mut inputs = placeholder_inputs();
        inputs[0] = BlockInputs::intra(&t0);
        inputs[1] = BlockInputs::intra(&t1);
        inputs[2] = BlockInputs::intra(&t2);
        inputs[3] = BlockInputs::intra(&t3);
        inputs[4] = BlockInputs::intra(&t4);
        inputs[5] = BlockInputs::intra(&t5);

        let out = decode_macroblock(
            MacroblockKind::Intra,
            &cbp_val,
            &mt,
            ChromaFormat::Yuv420,
            &inputs,
        )
        .expect("intra never errors");

        assert_eq!(out.len(), 6, "4:2:0 intra MB has 6 coded blocks");
        let indices: Vec<u8> = out.iter().map(|b| b.block_index).collect();
        assert_eq!(indices, vec![0, 1, 2, 3, 4, 5]);
        // Spot-check the third block — saturation clamps 100, 200,
        // 255, 255.
        assert_eq!(out[3].samples, vec![100, 200, 255, 255]);
    }

    #[test]
    fn decode_macroblock_inter_only_walks_cbp_bits() {
        // Inter MB in 4:2:0 with macroblock_pattern=1 and cbp = 0b101010
        // (binary): blocks 0, 2, 4 coded; blocks 1, 3, 5 uncoded.
        //
        // bit position 5 is block 0, position 4 is block 1, ..., position
        // 0 is block 5. cbp=0b101010 = 42 → bits 5, 3, 1 set → blocks
        // 0, 2, 4 coded.
        let mt = mt_inter(true, false, true);
        let cbp_val = cbp(0b101010);
        let kind = MacroblockKind::Inter(PredictionDirection::Forward);

        let t0: Vec<i16> = vec![1, 2, 3, 4];
        let t2: Vec<i16> = vec![10, 20, 30, 40];
        let t4: Vec<i16> = vec![-5, -10, -15, -20];
        let p0 = vec![100u8, 100, 100, 100];
        let p2 = vec![50u8, 50, 50, 50];
        let p4 = vec![60u8, 60, 60, 60];
        let mut inputs = placeholder_inputs();
        inputs[0] = BlockInputs::forward(&t0, &p0);
        inputs[2] = BlockInputs::forward(&t2, &p2);
        inputs[4] = BlockInputs::forward(&t4, &p4);

        let out = decode_macroblock(kind, &cbp_val, &mt, ChromaFormat::Yuv420, &inputs).unwrap();

        assert_eq!(out.len(), 3);
        let indices: Vec<u8> = out.iter().map(|b| b.block_index).collect();
        assert_eq!(indices, vec![0, 2, 4]);
        // Block 0: 100+1=101, 100+2=102, 100+3=103, 100+4=104.
        assert_eq!(out[0].samples, vec![101, 102, 103, 104]);
        // Block 2: 50+10=60, 50+20=70, 50+30=80, 50+40=90.
        assert_eq!(out[1].samples, vec![60, 70, 80, 90]);
        // Block 4: 60-5=55, 60-10=50, 60-15=45, 60-20=40.
        assert_eq!(out[2].samples, vec![55, 50, 45, 40]);
    }

    #[test]
    fn decode_macroblock_skipped_inter_emits_no_blocks_when_pattern_false() {
        // Skipped inter MB: macroblock_pattern = false, macroblock_intra
        // = false → pattern_code is all-false → walker emits zero
        // decoded blocks.
        let mt = mt_inter(false, false, false);
        let cbp_val = cbp(0);
        let inputs = placeholder_inputs();
        let out = decode_macroblock(
            MacroblockKind::Inter(PredictionDirection::Skipped),
            &cbp_val,
            &mt,
            ChromaFormat::Yuv420,
            &inputs,
        )
        .unwrap();
        assert!(
            out.is_empty(),
            "skipped MB with pattern=false has no coded blocks"
        );
    }

    #[test]
    fn decode_macroblock_chroma_format_bounds_walk_to_8_for_422() {
        // 4:2:2 inter MB with all bits set in cbp + coded_block_pattern_1.
        // pattern_code[0..6] from cbp(=0b111111=63), pattern_code[6..8]
        // from coded_block_pattern_1(=0b11=3). pattern_code[8..12] stay
        // false because coded_block_pattern_2 is absent.
        let mt = mt_inter(true, false, true);
        let mut cbp_val = cbp(63);
        cbp_val.coded_block_pattern_1 = Some(0b11);
        let kind = MacroblockKind::Inter(PredictionDirection::Forward);
        let transform: Vec<i16> = vec![0; 4];
        let prediction = vec![100u8; 4];
        let mut inputs = placeholder_inputs();
        for slot in inputs.iter_mut() {
            *slot = BlockInputs::forward(&transform, &prediction);
        }
        let out = decode_macroblock(kind, &cbp_val, &mt, ChromaFormat::Yuv422, &inputs).unwrap();
        assert_eq!(out.len(), 8, "4:2:2 has 8 blocks per MB");
        // Indices must be 0..=7 in order.
        let indices: Vec<u8> = out.iter().map(|b| b.block_index).collect();
        assert_eq!(indices, vec![0, 1, 2, 3, 4, 5, 6, 7]);
    }

    #[test]
    fn decode_macroblock_chroma_format_walks_12_for_444_intra() {
        // Intra MB in 4:4:4: pattern_code starts at [macroblock_intra
        // = true; 12], so all twelve slots are coded regardless of
        // cbp. The walker must emit twelve decoded blocks for the
        // §6.1.1.8 4:4:4 geometry.
        let mt = mt_intra();
        let cbp_val = cbp(0);
        let transform: Vec<i16> = vec![10, 20, 30, 40];
        let mut inputs = placeholder_inputs();
        for slot in inputs.iter_mut() {
            *slot = BlockInputs::intra(&transform);
        }
        let out = decode_macroblock(
            MacroblockKind::Intra,
            &cbp_val,
            &mt,
            ChromaFormat::Yuv444,
            &inputs,
        )
        .unwrap();
        assert_eq!(out.len(), 12, "4:4:4 intra MB has 12 coded blocks");
        let indices: Vec<u8> = out.iter().map(|b| b.block_index).collect();
        assert_eq!(indices, (0u8..12).collect::<Vec<_>>());
    }

    #[test]
    fn decode_macroblock_propagates_inter_errors() {
        // First coded block has missing prediction → driver returns
        // MissingForwardPrediction; no later blocks are walked.
        let mt = mt_inter(true, false, true);
        let cbp_val = cbp(63); // all six blocks coded
        let kind = MacroblockKind::Inter(PredictionDirection::Forward);
        let transform: Vec<i16> = vec![0; 4];
        let mut inputs = placeholder_inputs();
        for slot in inputs.iter_mut() {
            *slot = BlockInputs {
                transform: &transform,
                prediction_forward: &[],
                prediction_backward: &[],
            };
        }
        let err =
            decode_macroblock(kind, &cbp_val, &mt, ChromaFormat::Yuv420, &inputs).unwrap_err();
        assert_eq!(err, PipelineError::MissingForwardPrediction);
    }
}
