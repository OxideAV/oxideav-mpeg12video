//! # oxideav-mpeg12video
//!
//! Clean-room MPEG-1 Video (ISO/IEC 11172-2) / MPEG-2 Video
//! (ITU-T H.262 / ISO/IEC 13818-2) decoder and encoder for the
//! [oxideav](https://github.com/OxideAV/oxideav) framework.
//!
//! **Status:** rebuild rounds 1–23 — structural sequence-layer
//! parsers, the `group_of_pictures_header()` layer, the
//! `picture_header()` (+ `picture_coding_extension()`) layer, the
//! `slice()` header bits, the macroblock-loop syntax through the end
//! of `macroblock_modes()`
//! (`macroblock_address_increment`, `macroblock_type`, the
//! macroblock-layer `quantizer_scale`, `coded_block_pattern()`, and
//! the `frame_motion_type` / `field_motion_type` / `dct_type` tail),
//! the `motion_vectors()` / `motion_vector()` syntax with the Annex B
//! Tables B-10 / B-11 VLCs that drive it, the §7.6.3.1
//! `vector'[r][s][t]` reconstruction (PMV state, wrap-around
//! arithmetic, vertical-half-pred rule), the §7.6.3.3 inter-vector
//! PMV-copy update (Tables 7-10 / 7-11), §7.6.3.4 reset, §7.6.3.7
//! chroma scaling, the MPEG-1 §2.4.2.8 / §2.4.3.7 intra-block DC
//! prelude (Tables B.5a / B.5b + the `dct_dc_differential`
//! reconstruction plus the §2.4.4.1 zig-zag `scan[m][n]`), and the
//! MPEG-1 §2.4.3.7 `dct_coeff_first` / `dct_coeff_next` walker
//! (Tables B.5c / B.5d / B.5e run-level VLCs + Table B.5f short and
//! long escape encodings + the FIRST-vs-NEXT `(0, 1)` two-form
//! disambiguation + `end_of_block` recognition), and the MPEG-1
//! §2.4.4.1 / §2.4.4.2 dequantiser (the four intra-block loops
//! with the `dct_dc_y_past` / `dct_dc_cb_past` / `dct_dc_cr_past`
//! predictor chain and the `past_intra_address > 1` reset branch,
//! the non-intra `(2*dct_zz[i] + Sign(dct_zz[i]))` dead-zone
//! arithmetic with the `dct_zz[i] == 0 -> 0` zeroing pass, the
//! `Sign(...)` even-value mismatch-prevention rule, and the
//! `[-2048, 2047]` saturation, driven by the §2.4.3.2 default
//! `intra_quant[m][n]` and `non_intra_quant[m][n]` matrices), and
//! the §7.6.3.6 MPEG-2 dual-prime additional arithmetic that derives
//! the opposite-parity motion vector(s) `vector'[r][0][1:0]` from the
//! decoded same-parity vector and the inline `dmvector[0..1]` via
//! Tables 7-12 (`m[parity_ref][parity_pred]`) / 7-13
//! (`e[parity_ref][parity_pred]`) under the §4.1 `//` integer
//! division-with-rounding-away-from-zero operator, and the §7.6.4
//! forming-predictions pel reader (per-component `int_vec` /
//! `half_flag` split with the §4.1 `DIV` floor-toward-minus-infinity
//! operator, the four-way half-pel switch on
//! `(half_flag[0], half_flag[1])`, and the bilinear two-sample /
//! four-sample `// 2` / `// 4` averaging, driving a dimensionless
//! `width x height` prediction block over a pad-to-edge reference
//! plane). The §A 8×8 IDCT is now in hand at
//! [`idct::idct_reference_f64`] (direct 4-D summation),
//! [`idct::idct_candidate_f64`] (separable 1-D-pass), and
//! [`idct::idct_8x8`] (integer output rounded + clamped to
//! `[-256, +255]`), with an IEEE Std 1180-1990 / P1180/D2 conformance
//! harness in `tests/idct_p1180_conformance.rs` covering the four
//! statistical metrics plus the two deterministic edge cases. The
//! public `register` symbol is still a no-op so that downstream
//! consumers can depend on the crate without the decoder being
//! inadvertently selected by the registry — the full
//! [`oxideav_core::Decoder`] / [`oxideav_core::Encoder`] glue still
//! needs the slice-decoding driver and bytestream entry points.
//!
//! The landed pieces so far are:
//!
//! * [`sequence_header::Mpeg2SequenceHeader`] — `sequence_header()`
//!   from ISO/IEC 13818-2 §6.2.2.1 (field semantics §6.3.3).
//! * [`sequence_extension::Mpeg2SequenceExtension`] —
//!   `sequence_extension()` from §6.2.2.3 (field semantics §6.3.5).
//! * [`sequence_extension::Mpeg2Sequence`] — composed view that
//!   pairs the two and synthesises the full 14-bit width/height,
//!   30-bit bit_rate, and 18-bit vbv_buffer_size.
//! * [`gop_header::Mpeg2Gop`] — `group_of_pictures_header()` from
//!   §6.2.2.6 (field semantics §6.3.8), including the 25-bit
//!   `time_code` decomposition and the `closed_gop` / `broken_link`
//!   editing flags.
//! * [`picture_header::Mpeg2PictureHeader`] — `picture_header()`
//!   from §6.2.3 (field semantics §6.3.10) plus the companion
//!   [`picture_header::PictureCodingExtension`] for §6.2.3.1 /
//!   §6.3.11.
//! * [`slice_header::SliceHeader`] — the start-code-aligned header
//!   bits of `slice()` from §6.2.4 (field semantics §6.3.16):
//!   `slice_vertical_position` (from the start code), optional
//!   `slice_vertical_position_extension` (when `vertical_size >
//!   2800`), optional `priority_breakpoint` (when the surrounding
//!   sequence is data-partitioned), `quantiser_scale_code`, the
//!   optional `intra_slice_flag` / `intra_slice` / `reserved_bits`
//!   prelude, and the `extra_information_slice` byte loop. The
//!   macroblock body is **not** yet decoded.
//! * [`mb_address_increment::MbAddressIncrement`] — the leading
//!   `macroblock_address_increment` of `macroblock()` per §6.2.5
//!   (field semantics §6.3.17.1), with the Annex B Table B-1 VLC
//!   walker plus the `macroblock_escape` chain and (when
//!   [`mb_address_increment::MbAddressIncrementContext::mpeg1`] is
//!   set) the MPEG-1 `macroblock_stuffing` no-op.
//! * [`macroblock_type::MacroblockType`] — the `macroblock_type` VLC
//!   that opens `macroblock_modes()` per §6.2.5.1 (field semantics
//!   §6.3.17.1), decoding the six derived flags from the
//!   non-scalable Annex B Tables B-2 (I), B-3 (P), and B-4 (B).
//! * [`quantizer_scale::QuantizerScale`] — the macroblock-layer
//!   `quantizer_scale` per ISO/IEC 11172-2:1993 (MPEG-1) §2.4.2.7
//!   (field semantics §2.4.3.6): the 5-bit field present when
//!   `macroblock_quant` is set, in the range `1..=31` (zero
//!   forbidden), with the absent-field no-op when the flag is clear.
//! * [`coded_block_pattern::CodedBlockPattern`] — the
//!   `coded_block_pattern()` syntax per §6.2.5.3 (field semantics
//!   §6.3.17.4): the Annex B Table B-9 `coded_block_pattern_420` VLC
//!   plus the 4:2:2 / 4:4:4 fixed-length extensions, and the
//!   §6.3.17.4 `pattern_code[12]` derivation.
//! * [`macroblock_modes::MacroblockModesTail`] — the remainder of
//!   `macroblock_modes()` after `macroblock_type` per §6.2.5.1 (field
//!   semantics §6.3.17.1): the `frame_motion_type` (Table 6-17) /
//!   `field_motion_type` (Table 6-18) prediction-mode codes with their
//!   derived `motion_vector_count` / `mv_format` / `dmv`, and the
//!   `dct_type` flag (Table 6-19), each gated by `picture_structure` /
//!   `frame_pred_frame_dct` / the macroblock flags.
//! * [`motion_vector::MotionVectors`] — the `motion_vectors(s)`
//!   wrapper per §6.2.5.2 and [`motion_vector::MotionVector`] for the
//!   inner `motion_vector(r, s)` per §6.2.5.2.1, including the
//!   Annex B Table B-10 `motion_code` VLC, the f_code-driven
//!   fixed-length `motion_residual`, the Table B-11 `dmvector` VLC,
//!   and the `motion_vertical_field_select` presence gates
//!   (§6.3.17.2 / §6.3.17.3).
//! * [`pmv::Pmv`] — the §7.6.3 motion-vector predictor state and the
//!   §7.6.3.1 `vector'[r][s][t]` reconstruction (`delta` derivation,
//!   PMV-based prediction, half-pred for the field-in-frame vertical
//!   case, wrap-around to `[low, high]`), the §7.6.3.3 inter-vector
//!   PMV-copy update table ([`pmv::update_predictors`] driving Tables
//!   7-10 / 7-11), §7.6.3.4 reset hooks, and §7.6.3.7 chrominance
//!   scaling for 4:2:0 / 4:2:2 / 4:4:4. The §7.6.3.6 dual-prime
//!   additional arithmetic itself lives in
//!   [`dual_prime::derive_opposite_parity_vector`].
//! * [`dual_prime::derive_opposite_parity_vector`] /
//!   [`dual_prime::derive_all`] — §7.6.3.6 MPEG-2 dual-prime
//!   additional arithmetic: derive the opposite-parity motion vector
//!   `vector'[r][0][1:0]` (`r = 2` for a field picture; `r ∈ {2, 3}`
//!   for a frame picture) from the decoded same-parity vector and the
//!   inline `dmvector[0..1]` via Table 7-12
//!   (`m[parity_ref][parity_pred]`, picture-structure /
//!   `top_field_first`-keyed field-distance factor) and Table 7-13
//!   (`e[parity_ref][parity_pred]`, vertical inter-field offset). The
//!   §4.1 `//` integer-division-with-rounding-away-from-zero operator
//!   is honoured for the `m`-scaling halving (`3//2 = 2`, `-3//2 =
//!   -2`). The derived vectors do not flow through the PMV slots
//!   (`r ∈ {2, 3}` are not PMV-backed per Table 7-7).
//! * [`mpeg1_motion_vector::Mpeg1MotionVector`] — the MPEG-1
//!   (ISO/IEC 11172-2:1993) `motion_vector(s)` element per §2.4.2.7
//!   with the §2.4.3.6 field semantics: the Annex B Table B.4
//!   `motion_*_code` VLC and the `<dir>_f_code`-driven fixed-length
//!   `motion_*_r` residual for both horizontal and vertical
//!   components, parameterised on the forward/backward direction.
//!   MPEG-1 has no `motion_vertical_field_select`, `mv_format`, or
//!   `dmv` toggles, so the wire shape is the four `(code, r)` pairs
//!   straight through.
//! * [`mpeg1_reconstruct::reconstruct`] — the MPEG-1
//!   (ISO/IEC 11172-2:1993) §2.4.4.2 / §2.4.4.3 motion-vector
//!   reconstruction (`recon_right_for` / `recon_down_for` with the
//!   `right_little` / `right_big` wrap-around arithmetic, the PMV
//!   update via [`mpeg1_reconstruct::Mpeg1Predictor`], the
//!   `full_pel_*_vector` post-PMV left-shift, and the §2.4.4.2
//!   closing table that splits `recon_*` into the luminance and
//!   chrominance whole/half-pel offsets). Companion helpers
//!   [`mpeg1_reconstruct::reconstruct_zero`] (the §2.4.4.2 P-picture
//!   "no MV" reset) and [`mpeg1_reconstruct::reconstruct_absent`]
//!   (the §2.4.4.3 B-picture PMV carry-over) close the two
//!   spec-defined absence paths.
//! * [`block_dc::DcCoefficient`] — the MPEG-1 intra-block DC
//!   prelude per **ISO/IEC 11172-2:1993 §2.4.2.8 / §2.4.3.7**: the
//!   Annex B Tables B.5a / B.5b VLC walker for
//!   `dct_dc_size_luminance` / `dct_dc_size_chrominance` plus the
//!   `dct_dc_differential` → `dct_zz[0]` reconstruction formula.
//!   The companion [`block_dc::SCAN`] / [`block_dc::INVERSE_SCAN`]
//!   constants encode the §2.4.4.1 8x8 zig-zag scan order shared
//!   by every block-layer iterator.
//! * [`dct_coeff::DctCoeffStep`] — the MPEG-1
//!   (ISO/IEC 11172-2:1993) `dct_coeff_first` / `dct_coeff_next`
//!   walker per §2.4.3.7 driven by Annex B Tables B.5c / B.5d /
//!   B.5e (the run-level codebook) and Table B.5f (the escape
//!   encoding's short 14-bit and long 22-bit forms). Includes the
//!   `(run = 0, level = 1)` FIRST / NEXT disambiguation
//!   ([`dct_coeff::CoefficientPosition`]) and `end_of_block`
//!   recognition.
//! * [`dequantize::dequantize_intra_block`] /
//!   [`dequantize::dequantize_non_intra_block`] — the MPEG-1
//!   §2.4.4.1 (page 32) / §2.4.4.2 (page 35) dequantiser bodies
//!   that consume the fully-populated `dct_zz[]` array from the
//!   §2.4.3.7 walker and produce the `dct_recon[m][n]` matrix the
//!   §A.1 IDCT operates on. The four §2.4.4.1 intra block-loops
//!   (first-luma / subsequent-luma / Cb / Cr) are folded into a
//!   single [`dequantize::IntraBlockKind`] selector that drives
//!   the `(macroblock_address - past_intra_address) > 1` reset
//!   branch against the [`dequantize::IntraDcPredictors`] chain;
//!   [`dequantize::finalise_intra_macroblock`] performs the
//!   per-macroblock `past_intra_address = macroblock_address`
//!   close-out. The shared arithmetic body applies the `2 *
//!   dct_zz[i] * quantizer_scale * intra_quant[m][n] / 16` (or, for
//!   non-intra, the `(2 * dct_zz[i] + Sign(dct_zz[i])) *
//!   quantizer_scale * non_intra_quant[m][n] / 16`) numerator, the
//!   `if (recon & 1) == 0 -> recon -= Sign(recon)` even-mismatch
//!   prevention rule, the `[-2048, 2047]` saturation, and (non-
//!   intra only) the `if dct_zz[i] == 0 -> dct_recon[m][n] = 0`
//!   zeroing pass. [`dequantize::DEFAULT_INTRA_QUANT`] and
//!   [`dequantize::DEFAULT_NON_INTRA_QUANT`] expose the §2.4.3.2
//!   page-25 default matrices used when the sequence header sets
//!   the matching `load_*_quantizer_matrix == 0`.
//! * [`forming_predictions::predict_block`] /
//!   [`forming_predictions::predict_sample`] — the §7.6.4 forming-
//!   predictions pel reader per ISO/IEC 13818-2 (H.262) page 88: per-
//!   component `int_vec[t] = vector[r][s][t] DIV 2` /
//!   `half_flag[t] = (vector - 2*int_vec) != 0` split (with `DIV`
//!   being the §4.1 floor-toward-minus-infinity operator),
//!   [`forming_predictions::HalfPattern`] enumeration of the four
//!   `(half_flag[0], half_flag[1])` outcomes, and the bilinear
//!   `// 2` / `// 4` averaging of two or four reference samples
//!   (§4.1 round-half-away-from-zero, identical to `(sum + d/2) / d`
//!   on non-negative sums). [`forming_predictions::ReferencePlane`]
//!   wraps a row-major sample buffer with a `PadEdge` boundary mode
//!   so that motion vectors reaching past the picture edge clip to
//!   the nearest in-bounds sample.
//! * [`combine_predictions::average_predictions`] /
//!   [`combine_predictions::combine_directional_predictions`] —
//!   the §7.6.7.1 / §7.6.7.2 / §7.6.7.4 combining step that turns
//!   the up-to-two §7.6.4 prediction blocks into the final
//!   per-component prediction plane: `pel_pred[y][x] =
//!   (pel_pred_forward[y][x] + pel_pred_backward[y][x]) // 2` for
//!   the bi-directional case, single-direction pass-through for
//!   forward-only / backward-only B-frame rows of Tables 7-13 /
//!   7-14, and the §7.6.3.5 implicit-zero-MV
//!   [`combine_predictions::PredictionDirection::Skipped`] pass-through.
//!   The dual-prime
//!   [`combine_predictions::average_dual_prime_predictions`] alias
//!   matches the §7.6.7.4 spelling.
//! * [`add_coefficients::add_prediction_and_coefficients`] /
//!   [`add_coefficients::add_intra_block`] — the §7.6.8 adding step:
//!   `d[y][x] = saturate(f[y][x] + p[y][x])` with the `[0, 255]`
//!   clamp, both for the inter / B-frame
//!   `prediction + transform` case and for the intra shortcut where
//!   the prediction is conceptually all-zero and `d = saturate(f)`.
//! * [`macroblock_pipeline::decode_block`] /
//!   [`macroblock_pipeline::decode_macroblock`] — the §7.6 per-
//!   macroblock driver that composes the already-landed §7.6.7
//!   combine-predictions and §7.6.8 add-and-saturate endpoints into a
//!   per-coded-block dispatch loop keyed off the §6.3.17.4
//!   `pattern_code[12]` array. Consumes a
//!   [`macroblock_pipeline::MacroblockKind`] (the §7.6.5 / §7.6.6
//!   case), per-block [`macroblock_pipeline::BlockInputs`] (post-IDCT
//!   transform plane + §7.6.4 prediction sides), and the
//!   [`sequence_extension::ChromaFormat`] that bounds the walk to
//!   [`macroblock_pipeline::blocks_per_macroblock`] blocks per MB
//!   (6 for 4:2:0, 8 for 4:2:2, 12 for 4:4:4). Does NOT run the
//!   §A.1 IDCT or the §7.6.4 pel reader — the IDCT in particular is
//!   still blocked by issue #1110.
//! * [`mpeg2_dequantize::inverse_quantise_block`] — the §7.4
//!   inverse-quantisation pipeline from ITU-T H.262 / ISO/IEC
//!   13818-2 page 73 onward: §7.4.1 intra DC via the Table 7-4
//!   `intra_dc_mult` lookup against `intra_dc_precision`, §7.4.2.1
//!   weighting-matrix selection through
//!   [`mpeg2_dequantize::select_weighting_matrix_index`]
//!   (Table 7-5), §7.4.2.2 `quantiser_scale_code → quantiser_scale`
//!   resolution via [`mpeg2_dequantize::quantiser_scale`] backed by
//!   the [`mpeg2_dequantize::QUANTISER_SCALE_LINEAR`] /
//!   [`mpeg2_dequantize::QUANTISER_SCALE_NONLINEAR`] Table 7-6
//!   columns and keyed on `q_scale_type`, §7.4.2.3 reconstruction
//!   (`F''[v][u] = ((2*QF + k) * W * quantiser_scale) / 32` with
//!   `k = 0` for intra and `k = Sign(QF[v][u])` for non-intra under
//!   the §4.1 round-toward-zero `/` operator), §7.4.3 saturation to
//!   `[-2048, 2047]`, and §7.4.4 mismatch control (sum-parity LSB
//!   toggle on `F[7][7]`). Companion constants
//!   [`mpeg2_dequantize::DEFAULT_INTRA_WEIGHT`] and
//!   [`mpeg2_dequantize::DEFAULT_NON_INTRA_WEIGHT`] expose the
//!   §6.3.7 defaults; [`mpeg2_dequantize::F_SATURATION_MIN`] /
//!   [`mpeg2_dequantize::F_SATURATION_MAX`] expose the §7.4.3
//!   clamp bounds. The §A.1 IDCT itself remains blocked behind
//!   issue #1110.

#![warn(missing_debug_implementations)]

use oxideav_core::RuntimeContext;

pub mod add_coefficients;
pub mod block_dc;
pub mod coded_block_pattern;
pub mod combine_predictions;
pub mod dct_coeff;
pub mod dequantize;
pub mod dual_prime;
pub mod forming_predictions;
pub mod gop_header;
pub mod idct;
pub mod macroblock_modes;
pub mod macroblock_pipeline;
pub mod macroblock_type;
pub mod mb_address_increment;
pub mod motion_vector;
pub mod mpeg1_motion_vector;
pub mod mpeg1_reconstruct;
pub mod mpeg2_dequantize;
pub mod picture_header;
pub mod pmv;
pub mod quantizer_scale;
pub mod sequence_extension;
pub mod sequence_header;
pub mod slice_header;

pub use add_coefficients::{
    add_intra_block, add_prediction_and_coefficients, add_prediction_and_coefficients_in_place,
    saturate as saturate_decoded_sample,
};
pub use block_dc::{DcCoefficient, DcComponent, INVERSE_SCAN, MAX_DC_SIZE, SCAN};
pub use coded_block_pattern::CodedBlockPattern;
pub use combine_predictions::{
    average_dual_prime_predictions, average_predictions, average_predictions_in_place,
    combine_directional_predictions, PredictionDirection,
};
pub use dct_coeff::{CoefficientPosition, DctCoeff, DctCoeffStep, MAX_LEVEL_MAG, MAX_RUN};
pub use dequantize::{
    dequantize_intra_block, dequantize_non_intra_block, finalise_intra_macroblock, IntraBlockKind,
    IntraDcPredictors, DCT_RECON_MAX, DCT_RECON_MIN, DC_PREDICTOR_RESET, DEFAULT_INTRA_QUANT,
    DEFAULT_NON_INTRA_QUANT,
};
pub use dual_prime::{
    derive_all as derive_dual_prime_all, derive_opposite_parity_vector as derive_dual_prime_vector,
    dual_prime_picture, e_offset, m_factor, DerivedDualPrimeVector, DualPrimePicture, FieldParity,
};
pub use forming_predictions::{
    predict_block, predict_sample, split_component, split_reconstructed, split_vector, BlockSize,
    BoundaryMode, ComponentSplit, HalfPattern, ReferencePlane, SplitVector,
};
pub use gop_header::{Mpeg2Gop, TimeCode, GROUP_START_CODE};
pub use idct::{
    idct_8x8, idct_8x8_from_i32, idct_candidate_f64, idct_reference_f64,
    saturate_input as saturate_idct_input, saturate_output as saturate_idct_output, F_INPUT_MAX,
    F_INPUT_MIN, F_OUTPUT_MAX, F_OUTPUT_MIN,
};
pub use macroblock_modes::{
    MacroblockModesContext, MacroblockModesTail, MotionType, MvFormat, PredictionType,
};
pub use macroblock_pipeline::{
    blocks_per_macroblock, decode_block as pipeline_decode_block,
    decode_macroblock as pipeline_decode_macroblock, BlockInputs, DecodedBlock, MacroblockKind,
    PipelineError,
};
pub use macroblock_type::MacroblockType;
pub use mb_address_increment::{MbAddressIncrement, MbAddressIncrementContext};
pub use motion_vector::{
    MotionVector, MotionVectorEntry, MotionVectors, MotionVectorsContext, MotionVectorsKind,
};
pub use mpeg1_motion_vector::{Mpeg1MotionDirection, Mpeg1MotionVector};
pub use mpeg1_reconstruct::{
    reconstruct as mpeg1_reconstruct, reconstruct_absent as mpeg1_reconstruct_absent,
    reconstruct_zero as mpeg1_reconstruct_zero, Mpeg1FrameMvContext, Mpeg1Predictor,
    Mpeg1ReconstructedMv,
};
pub use mpeg2_dequantize::{
    intra_dc_mult, intra_dc_mult_from_extension,
    inverse_quantise_block as mpeg2_inverse_quantise_block, quantiser_scale,
    saturate as mpeg2_saturate, select_weighting_matrix_index, sign as mpeg2_sign,
    BlockCoding as Mpeg2BlockCoding, Component as Mpeg2Component, DEFAULT_INTRA_WEIGHT,
    DEFAULT_NON_INTRA_WEIGHT, F_SATURATION_MAX, F_SATURATION_MIN, QUANTISER_SCALE_LINEAR,
    QUANTISER_SCALE_NONLINEAR,
};
pub use picture_header::{
    Mpeg2PictureHeader, PictureCodingExtension, PictureCodingType, PictureStructure,
    PICTURE_CODING_EXTENSION_ID, PICTURE_START_CODE,
};
pub use pmv::{
    compute_delta, reconstruct_component, reconstruct_motion_vector, scale_chroma,
    update_predictors, vector_range, Component, Direction, Pmv, PmvUpdateContext, PmvUpdateOutcome,
    ReconstructedComponent, ScaledMotionVector, VectorIndex,
};
pub use quantizer_scale::{QuantizerScale, QUANTIZER_SCALE_MAX, QUANTIZER_SCALE_MIN};
pub use sequence_extension::{
    ChromaFormat, Mpeg2Sequence, Mpeg2SequenceExtension, EXTENSION_START_CODE,
    SEQUENCE_EXTENSION_ID,
};
pub use sequence_header::{AspectRatio, Mpeg2SequenceHeader, SEQUENCE_HEADER_CODE};
pub use slice_header::{
    SliceContext, SliceHeader, SLICE_VERTICAL_POSITION_MAX, SLICE_VERTICAL_POSITION_MIN,
};

/// Crate-local error type. Each variant is raised at most by the
/// specific decoder stage named in its docstring; sites may grow as
/// future rounds add slice/macroblock layers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// A bitstream constraint defined by ISO/IEC 13818-2 was
    /// violated (forbidden value, marker_bit zero, wrong start code,
    /// reserved entry where reserved values are not allowed, etc.).
    /// The static message names the spec subclause.
    InvalidBitstream(&'static str),
    /// The input buffer ended before the parser had read every bit
    /// the syntax element demanded.
    ShortHeader,
    /// Placeholder for syntax paths that are spec-defined but not
    /// yet implemented in this crate (motion vectors, IDCT, slice
    /// decoding, …). No code path currently returns this — it is
    /// kept as the contract for the encoder/decoder traits that
    /// later rounds will wire up.
    NotImplemented,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidBitstream(detail) => {
                write!(f, "mpeg12video: invalid bitstream: {detail}")
            }
            Self::ShortHeader => {
                write!(f, "mpeg12video: short header (unexpected end of input)")
            }
            Self::NotImplemented => {
                write!(f, "mpeg12video: feature not implemented yet")
            }
        }
    }
}

impl std::error::Error for Error {}

/// Crate-local `Result` alias.
pub type Result<T> = core::result::Result<T, Error>;

/// No-op codec registration. Rounds 1–23 parse the sequence,
/// group-of-pictures, picture, and slice headers plus the
/// macroblock-loop syntax through the end of `motion_vectors()`, the
/// §7.6.3.1 MPEG-2 motion-vector reconstruction, the §7.6.3.3
/// inter-vector PMV-copy update table, the MPEG-1 §2.4.4.2 /
/// §2.4.4.3 motion-vector reconstruction, the MPEG-1 §2.4.2.8 /
/// §2.4.3.7 intra-block DC prelude (Tables B.5a / B.5b + the
/// `dct_dc_differential` reconstruction plus the §2.4.4.1 zig-zag
/// scan), the MPEG-1 §2.4.3.7 `dct_coeff_first` /
/// `dct_coeff_next` run-level walker (Tables B.5c / B.5d / B.5e +
/// the Table B.5f short and long escape encodings), and the MPEG-1
/// §2.4.4.1 / §2.4.4.2 dequantiser bodies (intra / non-intra
/// arithmetic with the `dct_dc_*_past` predictor chain, the
/// `past_intra_address > 1` reset branch, the `Sign(...)`
/// even-mismatch fix, the `[-2048, 2047]` saturation, the §2.4.3.2
/// default `intra_quant` / `non_intra_quant` matrices, and the
/// non-intra `dct_zz[i] == 0 -> 0` zeroing pass), and the §7.6.3.6
/// MPEG-2 dual-prime additional arithmetic (Tables 7-12 / 7-13 with
/// the `//` rounding-away-from-zero operator, both single-vector
/// field-picture and two-vector frame-picture derivations), and the
/// §7.6.4 forming-predictions pel reader (per-component
/// `int_vec[t]` / `half_flag[t]` split with the §4.1 `DIV` floor
/// operator, the four-way half-pel switch on
/// `(half_flag[0], half_flag[1])`, and the bilinear `// 2` / `// 4`
/// averaging over a pad-to-edge reference plane), the §7.6.7.1 /
/// §7.6.7.4 combine-predictions step (bidirectional `(forward +
/// backward) // 2` average, single-direction pass-through, and the
/// dual-prime same-parity / opposite-parity alias of the same
/// formula), and the §7.6.8 add-prediction-and-coefficients
/// reconstruction step (`d = saturate(f + p)` with `[0, 255]` clamp,
/// plus the intra shortcut `d = saturate(f)` for `macroblock_intra ==
/// 1` blocks), and the §7.6 per-macroblock pipeline driver
/// ([`macroblock_pipeline::decode_block`] /
/// [`macroblock_pipeline::decode_macroblock`]) that composes the
/// §7.6.7 + §7.6.8 endpoints onto a per-coded-block dispatch loop
/// keyed off the §6.3.17.4 `pattern_code[12]` derivation and the
/// [`ChromaFormat`]-bounded `blocks_per_macroblock` (6 / 8 / 12)
/// walk, and the §7.4 MPEG-2 inverse-quantisation pipeline
/// ([`mpeg2_dequantize::inverse_quantise_block`]) covering §7.4.1
/// intra DC via Table 7-4 `intra_dc_mult`, §7.4.2.1 weighting-matrix
/// selection (Table 7-5), §7.4.2.2 `quantiser_scale_code` via
/// Table 7-6 (both `q_scale_type` columns), §7.4.2.3 reconstruction
/// (`((2*QF + k) * W * quantiser_scale) / 32` with `k = 0` /
/// `k = Sign(QF)`), §7.4.3 saturation to `[-2048, 2047]`, and §7.4.4
/// mismatch control (sum-parity LSB toggle on `F[7][7]`), and the §A
/// 8×8 IDCT ([`idct::idct_8x8`] over [`idct::idct_candidate_f64`]
/// with [`idct::idct_reference_f64`] as the IEEE 1180 reference and
/// the conformance harness exercising `pmse` / `omse` / `pme` / `ome`
/// plus peak error against the staged bounds) — they do not yet
/// provide a complete [`oxideav_core::Decoder`] or
/// [`oxideav_core::Encoder`] (the slice-decoding driver wiring the
/// dequantiser + IDCT into a bytestream pipeline is still ahead), so
/// there is nothing to install in the registry.
pub fn register(_ctx: &mut RuntimeContext) {}

oxideav_core::register!("mpeg12video", register);
