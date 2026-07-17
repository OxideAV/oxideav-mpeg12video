//! # oxideav-mpeg12video
//!
//! Clean-room MPEG-1 Video (ISO/IEC 11172-2) / MPEG-2 Video
//! (ITU-T H.262 / ISO/IEC 13818-2) decoder and encoder for the
//! [oxideav](https://github.com/OxideAV/oxideav) framework.
//!
//! **Status:** rebuild rounds 1–251 — structural sequence-layer
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
//! MPEG-2 residual VLC walker per ISO/IEC 13818-2 §7.2.2 (Annex B
//! Tables B-14 / B-15 / B-16) is now in
//! [`mpeg2_dct_coeff::DctCoeffStep`], with the §7.2.2.1 Table 7-3
//! `(intra_vlc_format, macroblock_intra)` table selector
//! ([`mpeg2_dct_coeff::TableSelection::from_context`]), the §7.2.2.2
//! NOTE 2 / NOTE 3 FIRST / NEXT disambiguation, the table-dependent
//! `end_of_block` codeword (B-14 `10`, B-15 `0110`), and the Table
//! B-16 escape (`000001` prefix + 6-bit run + 12-bit signed_level,
//! `signed_level ∈ [-2047, +2047] \ {0}`). The §6.2.6 `block(i)`
//! driver at [`mpeg2_block_decoder::decode_block`] now chains the
//! §7.2.1 DC prelude (intra blocks only), §7.2.2 residual VLC
//! walker (with the §7.2.2.2 NOTE 2 / NOTE 3 FIRST / NEXT
//! alternation honoured), §7.3 inverse scan, §7.4 inverse
//! quantisation, and §A 8×8 IDCT into a single
//! "bitstream → `f[y][x]` plane ready for §7.6.8 add-and-saturate"
//! entry point — the missing block-level composition step the
//! macroblock-level [`macroblock_pipeline`] driver dispatches to
//! once per coded block per the §6.3.17.4 `pattern_code[12]`
//! derivation. The public `register` symbol is still a no-op so
//! that downstream consumers can depend on the crate without the
//! decoder being inadvertently selected by the registry — the full
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
//! * [`mpeg2_block_decoder::decode_block`] — the §6.2.6 `block(i)`
//!   driver per ISO/IEC 13818-2 §6.2.6: chains the §7.2.1 DC
//!   prelude (intra blocks only), §7.2.2 residual VLC walker
//!   (with §7.2.2.2 NOTE 2 / NOTE 3 FIRST / NEXT alternation),
//!   §7.3 inverse scan, §7.4 inverse-quantisation pipeline, and
//!   §A 8×8 IDCT into a single
//!   "bitstream → `f[y][x]` plane" call. Consumes a
//!   [`mpeg2_block_decoder::BlockContext`] (the per-macroblock
//!   constants `intra_vlc_format` / `alternate_scan` /
//!   `intra_dc_precision` / `quantiser_scale_value`) plus the
//!   per-block triplet `(component, macroblock_intra, weight)`
//!   and a mutable [`mpeg2_block_dc::DcPredictors`] reference;
//!   returns a [`mpeg2_block_decoder::DecodedBlock`] carrying
//!   the four intermediate planes (`QFS[]`, `QF[v][u]`,
//!   `F[v][u]`, `f[y][x]`) plus the post-EOB bit cursor. The
//!   §7.2.2 wire-position constraint *"the position of the
//!   coefficient ... shall not exceed 63"* is enforced as an
//!   `InvalidBitstream` rejection.
//! * [`slice_macroblock_walk::SliceWalkContext::quantiser_matrices`]
//!   — round 251 wires the §6.3.11
//!   [`quant_matrix_extension::QuantiserMatrixState`] through
//!   the §6.2.4 slice walker into the §6.2.6 `block(i)` driver
//!   so the §7.4.2.3 reconstruction step picks up user-downloaded
//!   weighting matrices verbatim. Each constructor seeds the
//!   field to [`quant_matrix_extension::QuantiserMatrixState::defaults`]
//!   (byte-identical to the prior `with_default_weight_matrices`
//!   path); callers that have parsed a `quant_matrix_extension()`
//!   chain it on via the new
//!   [`slice_macroblock_walk::SliceWalkContext::with_quantiser_matrices`]
//!   builder and the four Table 7-5 `w`-indexed matrices flow
//!   verbatim into [`mpeg2_macroblock_blocks::MacroblockBlockContext::weight_matrices`].
//! * [`quant_matrix_extension::QuantMatrixDriver`] — round 254
//!   adds the picture-level state machine that collapses the
//!   §6.3.11 lifecycle (`reset_to_defaults()` on every
//!   `sequence_header_code`, `extension.apply(...)` on every
//!   `quant_matrix_extension()`) behind two named event methods
//!   [`quant_matrix_extension::QuantMatrixDriver::on_sequence_header`]
//!   and
//!   [`quant_matrix_extension::QuantMatrixDriver::on_quant_matrix_extension`].
//!   The driver's
//!   [`quant_matrix_extension::QuantMatrixDriver::state`] accessor
//!   returns a [`quant_matrix_extension::QuantiserMatrixState`]
//!   snapshot the slice-walker builder
//!   [`slice_macroblock_walk::SliceWalkContext::with_quantiser_matrices`]
//!   consumes verbatim, so callers reading the §6.3.11 spec flow
//!   line-for-line no longer have to write out the
//!   `state.reset_to_defaults(); ext.apply(&mut state, cf);
//!   ctx.with_quantiser_matrices(state)` dance themselves.

#![warn(missing_debug_implementations)]
// The crate docs above deliberately link into `#[doc(hidden)]` internal
// modules (the §6/§7 stage-by-stage build log); keep those links valid
// without surfacing the internals in the rendered API docs.
#![allow(rustdoc::private_intra_doc_links)]

use oxideav_core::{
    CodecCapabilities, CodecId, CodecInfo, CodecRegistry, CodecTag, RuntimeContext,
};

// Modules marked `#[doc(hidden)]` below are internal §6 syntax-walker /
// §7 reconstruction / encoder-stage plumbing: they stay `pub` so the
// integration tests and cross-stage callers keep exercising them, but
// they are NOT part of the stable API surface and are excluded from
// semver analysis. The stable surface is: the whole-stream decode entry
// points (`video_sequence`), the encoder entry points (`intra_encoder`,
// `p_picture_encoder`, `b_picture_encoder`, `inter_encoder`), the
// registry glue (`decoder`, `register`, `register_codecs`), and the
// frame/error types those signatures expose (`frame_assembly`,
// `picture_header::PictureCodingType`, `sequence_extension::ChromaFormat`,
// `inter_reconstruction::InterError`).
#[doc(hidden)] // internal: §7.6.8 add-and-saturate stage
pub mod add_coefficients;
pub mod b_picture_encoder;
#[doc(hidden)] // internal: MPEG-1 §2.4.3.7 DC prelude walker
pub mod block_dc;
#[doc(hidden)] // internal: §6.2.5.3 CBP syntax walker
pub mod coded_block_pattern;
#[doc(hidden)] // internal: §7.6.7 prediction-combining stage
pub mod combine_predictions;
#[doc(hidden)] // internal: §6.2.3.6 extension parser
pub mod copyright_extension;
#[doc(hidden)] // internal: MPEG-1 §2.4.3.7 run-level VLC walker
pub mod dct_coeff;
pub mod decoder;
#[doc(hidden)] // internal: MPEG-1 §2.4.4 dequantiser stage
pub mod dequantize;
#[doc(hidden)] // internal: §7.6.3.6 dual-prime vector arithmetic
pub mod dual_prime;
#[doc(hidden)] // internal: §6.2.2.2 extension/user-data dispatcher
pub mod extension_and_user_data;
#[doc(hidden)] // internal: §7.6.4 forming-predictions pel reader
pub mod forming_predictions;
#[doc(hidden)] // internal: encoder forward-DCT stage
pub mod forward_dct;
#[doc(hidden)] // internal: encoder forward-quantiser stage
pub mod forward_quant;
pub mod frame_assembly;
#[doc(hidden)] // internal: §6.2.2.6 GOP-header parser
pub mod gop_header;
#[doc(hidden)] // internal: §A 8x8 IDCT stage
pub mod idct;
pub mod inter_encoder;
pub mod inter_reconstruction;
pub mod intra_encoder;
#[doc(hidden)] // internal: §6.2.5.1 macroblock-modes syntax walker
pub mod macroblock_modes;
#[doc(hidden)] // internal: §7.6 per-macroblock block-dispatch driver
pub mod macroblock_pipeline;
#[doc(hidden)] // internal: §6.2.5.1 macroblock_type VLC walker
pub mod macroblock_type;
#[doc(hidden)] // internal: §6.2.5 address-increment VLC walker
pub mod mb_address_increment;
#[doc(hidden)] // internal: encoder motion-search stage
pub mod motion_estimation;
#[doc(hidden)] // internal: §6.2.5.2 motion-vector syntax walker
pub mod motion_vector;
#[doc(hidden)] // internal: §7.6.5/§7.6.6 prediction-selection tables
pub mod motion_vector_selection;
#[doc(hidden)] // internal: MPEG-1 block(i) driver
pub mod mpeg1_block_decoder;
#[doc(hidden)] // internal: MPEG-1 §2.4.2.7 motion-vector walker
pub mod mpeg1_motion_vector;
#[doc(hidden)] // internal: MPEG-1 picture-layer parser
pub mod mpeg1_picture;
#[doc(hidden)] // internal: MPEG-1 §2.4.4.2/§2.4.4.3 MV reconstruction
pub mod mpeg1_reconstruct;
#[doc(hidden)] // internal: §7.2.1 DC prelude walker
pub mod mpeg2_block_dc;
#[doc(hidden)] // internal: §6.2.6 block(i) driver
pub mod mpeg2_block_decoder;
#[doc(hidden)] // internal: §7.2.2 run-level VLC walker
pub mod mpeg2_dct_coeff;
#[doc(hidden)] // internal: §7.4 inverse-quantisation stage
pub mod mpeg2_dequantize;
#[doc(hidden)] // internal: §7.3 inverse-scan stage
pub mod mpeg2_inverse_scan;
#[doc(hidden)] // internal: §6.2.5 per-macroblock block loop
pub mod mpeg2_macroblock_blocks;
pub mod p_picture_encoder;
#[doc(hidden)] // internal: §6.2.3.3 extension parser
pub mod picture_display_extension;
pub mod picture_header;
#[doc(hidden)] // internal: picture-level reconstruction driver
pub mod picture_reconstruction;
#[doc(hidden)] // internal: §6.2.3.5 extension parser
pub mod picture_spatial_scalable_extension;
#[doc(hidden)] // internal: §6.2.3.4 extension parser
pub mod picture_temporal_scalable_extension;
#[doc(hidden)] // internal: §7.6.3 motion-vector predictor state
pub mod pmv;
#[doc(hidden)] // internal: §6.2.3.1 quant-matrix extension state
pub mod quant_matrix_extension;
#[doc(hidden)] // internal: MPEG-1 §2.4.2.7 quantizer_scale field
pub mod quantizer_scale;
#[doc(hidden)] // internal: §6.2.2.4 extension parser
pub mod sequence_display_extension;
#[doc(hidden)] // internal: display-order verification driver
pub mod sequence_display_order;
pub mod sequence_extension;
#[doc(hidden)] // internal: §6.2.2.1 sequence-header parser
pub mod sequence_header;
#[doc(hidden)] // internal: §6.2.2.5 extension parser
pub mod sequence_scalable_extension;
#[doc(hidden)] // internal: §7.6.6 skipped-macroblock semantics
pub mod skipped_macroblock;
#[doc(hidden)] // internal: §6.2.4 slice-header parser
pub mod slice_header;
#[doc(hidden)] // internal: §6.2.4/§6.2.5 slice-macroblock walker
pub mod slice_macroblock_walk;
#[doc(hidden)] // internal: §7.8 SNR-scalability addition stage
pub mod snr_coefficient_addition;
#[doc(hidden)] // internal: §7.7 spatial-scalability prediction stage
pub mod spatial_prediction_picture;
#[doc(hidden)] // internal: §7.7 spatial-resampling stage
pub mod spatial_resampling;
#[doc(hidden)] // internal: §7.7 spatial/temporal weight combining
pub mod spatial_temporal_combine;
#[doc(hidden)] // internal: encoder bitstream-writer stage
pub mod stream_writer;
pub mod video_sequence;

// ---------------------------------------------------------------------
// Stable crate-root surface: whole-stream decode, encoder entry points,
// registry glue, and the frame/error types their signatures expose.
// ---------------------------------------------------------------------
pub use b_picture_encoder::encode_b_picture;
pub use decoder::{
    frame_buffer_to_video_frame, make_decoder, Mpeg12Decoder, MPEG1_CODEC_ID_STR,
    MPEG2_CODEC_ID_STR,
};
pub use frame_assembly::{FrameBuffer, IntraPictureParams, Plane};
pub use inter_encoder::{
    encode_display_order_sequence, encode_i_p_b, encode_i_p_chain, encode_i_then_p,
    encode_i_then_p_copy, encode_nonintra_block, encode_p_copy_picture,
};
pub use inter_reconstruction::InterError;
pub use intra_encoder::encode_intra_picture;
pub use p_picture_encoder::encode_p_picture;
pub use picture_header::PictureCodingType;
pub use sequence_extension::ChromaFormat;
pub use video_sequence::{
    decode_video_sequence, display_indices_from_coded_pictures,
    display_indices_from_temporal_references, verify_display_order,
    verify_display_order_with_types, DecodedFrame,
};

// ---------------------------------------------------------------------
// Internal re-exports (kept for test / historical callers, hidden from
// the documented API and from semver analysis; each mirrors a hidden
// module or a hidden item of a mixed module above).
// ---------------------------------------------------------------------
#[doc(hidden)] // internal: §7.6.8 stage re-exports
pub use add_coefficients::{
    add_intra_block, add_prediction_and_coefficients, add_prediction_and_coefficients_in_place,
    saturate as saturate_decoded_sample,
};
#[doc(hidden)] // internal: MPEG-1 DC-prelude re-exports
pub use block_dc::{DcCoefficient, DcComponent, INVERSE_SCAN, MAX_DC_SIZE, SCAN};
#[doc(hidden)] // internal: §6.2.5.3 CBP re-exports
pub use coded_block_pattern::{encode_cbp420, CodedBlockPattern};
#[doc(hidden)] // internal: §7.6.7 combining re-exports
pub use combine_predictions::{
    average_dual_prime_predictions, average_predictions, average_predictions_in_place,
    combine_directional_predictions, PredictionDirection,
};
#[doc(hidden)] // internal: §6.2.3.6 extension re-exports
pub use copyright_extension::{CopyrightExtension, COPYRIGHT_EXTENSION_ID};
#[doc(hidden)] // internal: MPEG-1 run-level VLC re-exports
pub use dct_coeff::{CoefficientPosition, DctCoeff, DctCoeffStep, MAX_LEVEL_MAG, MAX_RUN};
#[doc(hidden)] // internal: MPEG-1 dequantiser re-exports
pub use dequantize::{
    dequantize_intra_block, dequantize_non_intra_block, finalise_intra_macroblock, IntraBlockKind,
    IntraDcPredictors, DCT_RECON_MAX, DCT_RECON_MIN, DC_PREDICTOR_RESET, DEFAULT_INTRA_QUANT,
    DEFAULT_NON_INTRA_QUANT,
};
#[doc(hidden)] // internal: §6.2.2.2 dispatcher re-exports
pub use extension_and_user_data::{
    ExtensionAndUserData, ExtensionLocation, UserData, USER_DATA_START_CODE,
};

#[doc(hidden)] // internal: §7.6.3.6 dual-prime re-exports
pub use dual_prime::{
    derive_all as derive_dual_prime_all, derive_opposite_parity_vector as derive_dual_prime_vector,
    dual_prime_picture, e_offset, m_factor, DerivedDualPrimeVector, DualPrimePicture, FieldParity,
};
#[doc(hidden)] // internal: §7.6.4 pel-reader re-exports
pub use forming_predictions::{
    predict_block, predict_field_block, predict_field_sample, predict_sample, split_component,
    split_reconstructed, split_vector, BlockSize, BoundaryMode, ComponentSplit, FieldReference,
    HalfPattern, ReferencePlane, SplitVector,
};
#[doc(hidden)] // internal: encoder forward-DCT re-exports
pub use forward_dct::{fdct_8x8, fdct_candidate_f64, fdct_reference_f64};
#[doc(hidden)] // internal: encoder forward-quantiser re-exports
pub use forward_quant::{forward_quantise_block, forward_quantise_intra_dc, MAX_CODED_LEVEL};
#[doc(hidden)] // internal: picture-assembly plumbing re-exports
pub use frame_assembly::{
    assemble_frame_from_fields, block_placement, chroma_shift, decode_intra_picture,
    decode_intra_picture_with_matrices, place_intra_block, place_intra_macroblock, BlockPlacement,
};
#[doc(hidden)] // internal: §6.2.2.6 GOP-header re-exports
pub use gop_header::{
    nominal_pictures_per_second, write_gop_header, Mpeg2Gop, TimeCode, GROUP_START_CODE,
};
#[doc(hidden)] // internal: §A IDCT re-exports
pub use idct::{
    idct_8x8, idct_8x8_from_i32, idct_candidate_f64, idct_reference_f64,
    saturate_input as saturate_idct_input, saturate_output as saturate_idct_output, F_INPUT_MAX,
    F_INPUT_MIN, F_OUTPUT_MAX, F_OUTPUT_MIN,
};
#[doc(hidden)] // internal: §7.6 inter-reconstruction plumbing re-exports
pub use inter_reconstruction::{
    predict_field_based_macroblock_planes, predict_field_picture_16x8_macroblock_planes,
    predict_field_picture_macroblock_planes, predict_frame_macroblock_planes,
    reconstruct_field_based_macroblock, reconstruct_field_picture_16x8_macroblock,
    reconstruct_field_picture_macroblock, reconstruct_inter_macroblock, FieldBasedMotion,
    FieldPicture16x8Motion, FieldPictureMotion, FrameMotion, MotionVectorPel, ReferenceFrames,
    ResidualBlock,
};
#[doc(hidden)] // internal: §6.2.5.1 macroblock-modes re-exports
pub use macroblock_modes::{
    MacroblockModesContext, MacroblockModesTail, MotionType, MvFormat, PredictionType,
    SpatialTemporalWeight,
};
#[doc(hidden)] // internal: §7.6 macroblock-pipeline re-exports
pub use macroblock_pipeline::{
    blocks_per_macroblock, decode_block as pipeline_decode_block,
    decode_macroblock as pipeline_decode_macroblock, BlockInputs, DecodedBlock, MacroblockKind,
    PipelineError,
};
#[doc(hidden)] // internal: §6.2.5.1 macroblock_type re-exports
pub use macroblock_type::{MacroblockType, MacroblockTypeTable};
#[doc(hidden)] // internal: §6.2.5 address-increment re-exports
pub use mb_address_increment::{MbAddressIncrement, MbAddressIncrementContext};
#[doc(hidden)] // internal: encoder motion-search re-exports
pub use motion_estimation::{estimate_forward_mv, max_search_range, MotionSearchResult};
#[doc(hidden)] // internal: §6.2.5.2 motion-vector re-exports
pub use motion_vector::{
    encode_dmvector, encode_motion_component, encode_motion_vector, split_delta, MotionVector,
    MotionVectorEntry, MotionVectors, MotionVectorsContext, MotionVectorsKind,
};
#[doc(hidden)] // internal: §7.6.5/§7.6.6 selection re-exports
pub use motion_vector_selection::{
    select_predictions, MacroblockPrediction, PredictionOp, PredictionRegion, ReferenceTarget,
};
#[doc(hidden)] // internal: MPEG-1 motion-vector re-exports
pub use mpeg1_motion_vector::{Mpeg1MotionDirection, Mpeg1MotionVector};
#[doc(hidden)] // internal: MPEG-1 MV-reconstruction re-exports
pub use mpeg1_reconstruct::{
    reconstruct as mpeg1_reconstruct, reconstruct_absent as mpeg1_reconstruct_absent,
    reconstruct_zero as mpeg1_reconstruct_zero, Mpeg1FrameMvContext, Mpeg1Predictor,
    Mpeg1ReconstructedMv,
};
#[doc(hidden)] // internal: §7.2.1 DC-prelude re-exports
pub use mpeg2_block_dc::{
    dc_pred_reset_value as mpeg2_dc_pred_reset_value, decode_dc_block as mpeg2_decode_dc_block,
    encode_intra_dc as mpeg2_encode_intra_dc, qfs_zero_max as mpeg2_qfs_zero_max,
    ColourComponent as Mpeg2ColourComponent, DcCoefficient as Mpeg2DcCoefficient,
    DcComponent as Mpeg2DcComponent, DcPredictors as Mpeg2DcPredictors,
    MAX_DC_SIZE as MPEG2_MAX_DC_SIZE,
};
#[doc(hidden)] // internal: §6.2.6 block(i) re-exports
pub use mpeg2_block_decoder::{
    decode_block as mpeg2_decode_block, BlockContext as Mpeg2BlockContext,
    DecodedBlock as Mpeg2DecodedBlock,
};
#[doc(hidden)] // internal: §7.2.2 run-level VLC re-exports
pub use mpeg2_dct_coeff::{
    encode_dct_coeff as mpeg2_encode_dct_coeff, encode_end_of_block as mpeg2_encode_end_of_block,
    CoefficientPosition as Mpeg2CoefficientPosition, DctCoeff as Mpeg2DctCoeff,
    DctCoeffStep as Mpeg2DctCoeffStep, TableSelection as Mpeg2VlcTable,
    ESCAPE_SIGNED_LEVEL_MAX as MPEG2_ESCAPE_SIGNED_LEVEL_MAX,
    ESCAPE_SIGNED_LEVEL_MIN as MPEG2_ESCAPE_SIGNED_LEVEL_MIN, MAX_RUN as MPEG2_DCT_MAX_RUN,
};
#[doc(hidden)] // internal: §7.4 inverse-quantisation re-exports
pub use mpeg2_dequantize::{
    intra_dc_mult, intra_dc_mult_from_extension,
    inverse_quantise_block as mpeg2_inverse_quantise_block, quantiser_scale,
    saturate as mpeg2_saturate, select_weighting_matrix_index, sign as mpeg2_sign,
    BlockCoding as Mpeg2BlockCoding, Component as Mpeg2Component, DEFAULT_INTRA_WEIGHT,
    DEFAULT_NON_INTRA_WEIGHT, F_SATURATION_MAX, F_SATURATION_MIN, QUANTISER_SCALE_LINEAR,
    QUANTISER_SCALE_NONLINEAR,
};
#[doc(hidden)] // internal: §7.3 inverse-scan re-exports
pub use mpeg2_inverse_scan::{
    apply_inverse_scan as mpeg2_apply_inverse_scan, inverse_scan_table as mpeg2_inverse_scan_table,
    place_coefficient as mpeg2_place_coefficient, scan_table as mpeg2_scan_table,
    ALTERNATE_INVERSE_SCAN as MPEG2_ALTERNATE_INVERSE_SCAN, ALTERNATE_SCAN as MPEG2_ALTERNATE_SCAN,
    ZIGZAG_INVERSE_SCAN as MPEG2_ZIGZAG_INVERSE_SCAN,
};
#[doc(hidden)] // internal: §6.2.5 block-loop re-exports
pub use mpeg2_macroblock_blocks::{
    block_component as mpeg2_block_component, block_count as mpeg2_block_count,
    decode_macroblock_blocks as mpeg2_decode_macroblock_blocks,
    DecodedBlock as Mpeg2MacroblockDecodedBlock,
    MacroblockBlockContext as Mpeg2MacroblockBlockContext,
    DEFAULT_WEIGHT_MATRICES as MPEG2_DEFAULT_WEIGHT_MATRICES,
};
#[doc(hidden)] // internal: §6.2.3.3 extension re-exports
pub use picture_display_extension::{
    number_of_frame_centre_offsets, FieldUsage, FrameCentreOffset, FrameCentreOffsetDriver,
    FrameCentreOffsetState, PictureDisplayContext, PictureDisplayExtension,
    MAX_FRAME_CENTRE_OFFSETS, PICTURE_DISPLAY_EXTENSION_ID,
};
#[doc(hidden)] // internal: §6.2.3 picture-layer parser re-exports
pub use picture_header::{
    Mpeg2PictureHeader, PictureCodingExtension, PictureStructure, PICTURE_CODING_EXTENSION_ID,
    PICTURE_START_CODE,
};
#[doc(hidden)] // internal: picture-level reconstruction re-exports
pub use picture_reconstruction::{
    decode_field_picture, decode_field_picture_with_matrices, decode_inter_picture,
    decode_inter_picture_with_matrices, PicturePredictionParams,
};
#[doc(hidden)] // internal: §6.2.3.5 extension re-exports
pub use picture_spatial_scalable_extension::{
    PictureSpatialScalableExtension, PICTURE_SPATIAL_SCALABLE_EXTENSION_ID,
};
#[doc(hidden)] // internal: §6.2.3.4 extension re-exports
pub use picture_temporal_scalable_extension::{
    PictureReferences, PictureTemporalScalableExtension, ReferenceSource,
    PICTURE_TEMPORAL_SCALABLE_EXTENSION_ID,
};
#[doc(hidden)] // internal: §7.6.3 PMV-state re-exports
pub use pmv::{
    apply_spatial_temporal_reset, compute_delta, decode_motion_vector, reconstruct_component,
    reconstruct_motion_vector, scale_chroma, update_predictors, vector_range, Component, Direction,
    Pmv, PmvUpdateContext, PmvUpdateOutcome, ReconstructedComponent, ScaledMotionVector,
    VectorIndex,
};
#[doc(hidden)] // internal: quant-matrix state re-exports
pub use quant_matrix_extension::{
    QuantMatrixDriver, QuantMatrixExtension, QuantiserMatrixPayload, QuantiserMatrixState,
    QUANT_MATRIX_EXTENSION_ID,
};
#[doc(hidden)] // internal: MPEG-1 quantizer_scale re-exports
pub use quantizer_scale::{QuantizerScale, QUANTIZER_SCALE_MAX, QUANTIZER_SCALE_MIN};
#[doc(hidden)] // internal: §6.2.2.4 extension re-exports
pub use sequence_display_extension::{
    ColourDescription, SequenceDisplayExtension, VideoFormat, SEQUENCE_DISPLAY_EXTENSION_ID,
};
#[doc(hidden)] // internal: display-order driver re-exports
pub use sequence_display_order::{Requirement, SequenceDisplayOrderDriver};
#[doc(hidden)] // internal: §6.2.2.3 sequence-layer parser re-exports
pub use sequence_extension::{
    Mpeg2Sequence, Mpeg2SequenceExtension, EXTENSION_START_CODE, SEQUENCE_EXTENSION_ID,
};
#[doc(hidden)] // internal: §6.2.2.1 sequence-header re-exports
pub use sequence_header::{AspectRatio, Mpeg2SequenceHeader, SEQUENCE_HEADER_CODE};
#[doc(hidden)] // internal: §6.2.2.5 extension re-exports
pub use sequence_scalable_extension::{
    ScalableMode, SequenceScalableExtension, SpatialScalabilityParams, TemporalScalabilityParams,
    SEQUENCE_SCALABLE_EXTENSION_ID,
};
#[doc(hidden)] // internal: §7.6.6 skipped-macroblock re-exports
pub use skipped_macroblock::{
    apply_to_pmv as skipped_macroblock_apply_to_pmv, describe_skipped_macroblock,
    SkippedMacroblock, SkippedMacroblockContext, SkippedMotionVector,
};
#[doc(hidden)] // internal: §6.2.4 slice-header re-exports
pub use slice_header::{
    SliceContext, SliceHeader, SLICE_VERTICAL_POSITION_MAX, SLICE_VERTICAL_POSITION_MIN,
};
#[doc(hidden)] // internal: slice-walker re-exports
pub use slice_macroblock_walk::{
    reconstruct_record_motion_vectors, reconstruct_slice_motion_vectors, walk_slice, walk_slice_at,
    MacroblockRecord, ReconstructedMotionVectors, ReconstructedVector, SliceMotionRecord,
    SliceMotionWalk, SliceWalk, SliceWalkContext, PAST_INTRA_ADDRESS_RESET,
};
#[doc(hidden)] // internal: SNR-scalability re-exports
pub use snr_coefficient_addition::{
    add_layer_block, add_layer_chroma_simulcast, simulcast_dc_predictor_block, CoeffBlock,
};
#[doc(hidden)] // internal: spatial-scalability re-exports
pub use spatial_prediction_picture::{spatial_prediction_picture, SpatialPredictionPicture};
#[doc(hidden)] // internal: spatial-resampling re-exports
pub use spatial_resampling::{
    deinterlace, horizontal_resample, reinterlace, resample_progressive,
    upsample_spatial_prediction, vertical_resample, Field as ResampleField, Plane as ResamplePlane,
    ResampleParams, UpsampleCase,
};
#[doc(hidden)] // internal: spatial/temporal combining re-exports
pub use spatial_temporal_combine::{
    combine_field_interleaved, combine_macroblock_spatial_temporal, combine_spatial_temporal,
    combine_uniform, extract_colocated_spatial, SpatialWeight,
};
#[doc(hidden)] // internal: encoder bitstream-writer re-exports
pub use stream_writer::{
    write_picture_coding_extension, write_picture_header, write_sequence_extension,
    write_sequence_header, write_slice_header, PictureCodingExtensionParams, SequenceHeaderParams,
    SEQUENCE_END_CODE,
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
    /// yet implemented in this crate, so callers can tell "cannot
    /// decode yet" apart from "bitstream is broken". No §6.2.2.2
    /// `extension_and_user_data(i)` path returns it any longer — every
    /// Table 6-2 extension this crate's dispatcher reaches now has a
    /// parser — but the variant is retained for future syntax surfaces.
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

impl From<inter_reconstruction::InterError> for Error {
    /// Fold an [`inter_reconstruction::InterError`] into the crate-level
    /// [`Error`] so the picture-level driver can propagate it through a
    /// single [`Result`]. The "unsupported prediction mode" case maps to
    /// [`Error::NotImplemented`] (the field-based / 16×8-MC / dual-prime
    /// field-picture per-field reference assembly is a later milestone);
    /// every other case is a structural reconstruction failure
    /// ([`Error::InvalidBitstream`]).
    fn from(err: inter_reconstruction::InterError) -> Self {
        use inter_reconstruction::InterError;
        match err {
            InterError::UnsupportedPredictionMode => Error::NotImplemented,
            InterError::MissingForwardReference => {
                Error::InvalidBitstream("§7.6.4: forward prediction without a forward reference")
            }
            InterError::MissingBackwardReference => {
                Error::InvalidBitstream("§7.6.4: backward prediction without a backward reference")
            }
            InterError::ReferenceGeometryMismatch => {
                Error::InvalidBitstream("§7.6.4: reference frame geometry mismatch")
            }
            InterError::InvalidBlockIndex => {
                Error::InvalidBitstream("§6.1.1.8: residual block_index out of range")
            }
        }
    }
}

/// Crate-local `Result` alias.
pub type Result<T> = core::result::Result<T, Error>;

/// Install the MPEG-1 (`"mpeg1video"`) and MPEG-2 (`"mpeg2video"`)
/// video decoders into `reg`.
///
/// Both ids resolve to [`decoder::make_decoder`], which drives the
/// whole-elementary-stream reconstruction pipeline
/// ([`video_sequence::decode_video_sequence`]) behind the packet-based
/// [`oxideav_core::Decoder`] contract (see the [`decoder`] module docs
/// for the framing model). The tag sets are the FourCC / Matroska
/// codec ids the container crates map onto these two ids
/// (`V_MPEG1` / `V_MPEG2` in Matroska; `mp1v` / `mp2v` / `mpg1` /
/// `mpg2` / `hdv2` / `m2v1` FourCCs).
pub fn register_codecs(reg: &mut CodecRegistry) {
    let mpeg1_caps = CodecCapabilities::video("mpeg1video_sw")
        .with_lossy(true)
        .with_intra_only(false)
        .with_max_size(4096, 4096);
    reg.register(
        CodecInfo::new(CodecId::new(decoder::MPEG1_CODEC_ID_STR))
            .capabilities(mpeg1_caps)
            .decoder(decoder::make_decoder)
            .tags([
                CodecTag::fourcc(b"mp1v"),
                CodecTag::fourcc(b"mpg1"),
                CodecTag::fourcc(b"MPG1"),
                CodecTag::matroska("V_MPEG1"),
            ]),
    );

    let mpeg2_caps = CodecCapabilities::video("mpeg2video_sw")
        .with_lossy(true)
        .with_intra_only(false)
        .with_max_size(8192, 8192);
    reg.register(
        CodecInfo::new(CodecId::new(decoder::MPEG2_CODEC_ID_STR))
            .capabilities(mpeg2_caps)
            .decoder(decoder::make_decoder)
            .tags([
                CodecTag::fourcc(b"mp2v"),
                CodecTag::fourcc(b"mpg2"),
                CodecTag::fourcc(b"MPG2"),
                CodecTag::fourcc(b"hdv2"),
                CodecTag::fourcc(b"m2v1"),
                CodecTag::matroska("V_MPEG2"),
            ]),
    );
}

/// Unified registration entry point: install the MPEG-1 / MPEG-2 video
/// codec factories into the codec sub-registry of a [`RuntimeContext`].
///
/// This is the entry point every sibling crate follows and the one the
/// [`oxideav_core::register!`] macro below wires into
/// `oxideav_meta::register_all`. Direct callers that need only the codec
/// sub-registry can use [`register_codecs`] instead.
pub fn register(ctx: &mut RuntimeContext) {
    register_codecs(&mut ctx.codecs);
}

oxideav_core::register!("mpeg12video", register);
