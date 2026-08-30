# oxideav-mpeg12video

[![CI](https://github.com/OxideAV/oxideav-mpeg12video/actions/workflows/ci.yml/badge.svg)](https://github.com/OxideAV/oxideav-mpeg12video/actions/workflows/ci.yml) [![crates.io](https://img.shields.io/crates/v/oxideav-mpeg12video.svg)](https://crates.io/crates/oxideav-mpeg12video) [![docs.rs](https://docs.rs/oxideav-mpeg12video/badge.svg)](https://docs.rs/oxideav-mpeg12video) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Clean-room MPEG-1 Video (ISO/IEC 11172-2) and MPEG-2 Video
(ITU-T H.262 / ISO/IEC 13818-2) decode **and encode** building blocks
for the [oxideav](https://github.com/OxideAV/oxideav-workspace)
framework. Pure Rust, no C dependencies.

## Status

Clean-room rebuild. The crate implements the full MPEG-1 and MPEG-2
video decode pipeline as a set of composable, per-stage public modules
covering the bitstream-parsing surface and the pixel-reconstruction
math, topped by a `video_sequence()` driver
(`decode_video_sequence`) that decodes a whole elementary stream into
reconstructed frames in §6.1.1.11 display order (frame pictures and
field-picture pairs alike). It is now **wired into the runtime codec
registry**: `register` installs `oxideav_core::Decoder` factories under
both the `mpeg1video` and `mpeg2video` codec ids, so the codec is
consumed through `oxideav_core::make_decoder` (a `RuntimeContext` /
`register_all` lookup) as well as through the direct
`decoder::make_decoder` factory and the per-stage module APIs.
§7.10 **data-partitioned** streams decode through
`decode_data_partitioned` (partition pair → merged non-scalable
stream → the same driver), and a three-target cargo-fuzz harness
(`fuzz/`) runs daily against the decoder, the encode→decode round
trip and the partition merge.

## What works today

The decode pipeline is implemented end-to-end at the module level:

- **Sequence / GOP / picture / slice layers**: sequence header and the
  MPEG-2 extension family (sequence / sequence-display /
  sequence-scalable / quant-matrix / picture-coding / picture-display /
  picture-spatial-scalable / picture-temporal-scalable / copyright
  extensions), group-of-pictures header (time code, closed/broken
  flags), picture header, and slice header.
- **Macroblock-layer syntax**: `macroblock_address_increment`,
  `macroblock_type`, the macroblock-layer `quantizer_scale`,
  `coded_block_pattern`, and the `frame_motion_type` /
  `field_motion_type` / `dct_type` tail through `macroblock_modes()`.
- **Motion vectors**: the `motion_vectors()` / `motion_vector()` syntax
  with the Annex B motion-code VLCs, MPEG-1 reconstruction
  (§2.4.4.2 / §2.4.4.3) and MPEG-2 reconstruction
  (§7.6.3.1 / .3 / .4 / .7) including PMV state, wrap-around arithmetic,
  the vertical-half-prediction rule, inter-vector PMV copy/update, and
  MPEG-2 dual-prime (§7.6.3.6). The §7.6.3 predictor bank is now driven
  across a whole slice by `reconstruct_slice_motion_vectors`, which
  resets PMV at slice start (§7.6.3.4), reconstructs each coded
  macroblock's vectors (§7.6.3.1), applies the §7.6.3.3
  `update_predictors` table row, and runs the §7.6.6 skipped-macroblock
  PMV side-effect for the run of skipped slots preceding each coded
  macroblock.
- **Residual decode**: MPEG-1 intra DC prelude + zig-zag + run-level
  walker (§2.4.2.8 / §2.4.3.7 / §2.4.4.1, Annex B Tables B.5a–B.5f) and
  the MPEG-2 §7.2 residual VLC walker (Annex B Tables B-14 / B-15 /
  B-16 with the Table 7-3 table selector, the FIRST/NEXT alternates,
  and the escape encoding).
- **Inverse scan**: MPEG-2 §7.3 zig-zag and alternate-scan tables with
  the `alternate_scan`-flag dispatch.
- **Dequantisation**: the MPEG-1 intra/non-intra dequantiser (§2.4.4.1 /
  .2 with the `dct_dc_*_past` predictor chain, the even-mismatch fix and
  `[-2048, 2047]` saturation) and the MPEG-2 §7.4 inverse-quantisation
  pipeline (Tables 7-4 / 7-5 / 7-6, §7.4.2.3 reconstruction, §7.4.3
  saturation, §7.4.4 sum-parity mismatch control).
- **8×8 IDCT** (Annex A): validated against an IEEE Std 1180-1990
  conformance harness (the `pmse` / `omse` / `pme` / `ome` statistical
  metrics plus peak error).
- **Motion compensation**: the §7.6 pipeline — §7.6.4 forming
  predictions (pel reader), §7.6.7 combine predictions, §7.6.8 add
  coefficients with the `[0, 255]` clamp.
- **Picture-level P/B reconstruction**: `decode_inter_picture` is the
  top-level driver that reconstructs a whole P- or B-picture to real
  pixels. For each slice it walks the macroblock body with the §6.2.6
  block pipeline enabled, reconstructs the slice's motion vectors
  (§7.6.3), then per macroblock dispatches intra blocks to the §7.6.8
  `d = saturate(f)` intra placement and inter blocks to
  `reconstruct_inter_macroblock` — the §7.6 macroblock driver that
  forms the per-component prediction plane (16×16 luma + §7.6.3.7
  chroma-scaled prediction), combines forward/backward via the
  §7.6.7.1 `// 2` average, adds the §A IDCT residual per coded block
  (§6.3.17.4 `pattern_code[]`), and writes the result into the
  `FrameBuffer` honouring the §6.1.3 frame/field DCT line organisation.
  Skipped macroblocks (§7.6.6) reconstruct as a P-picture `(0,0)`
  forward copy or a B-picture prediction inheriting the previous
  macroblock's **direction** with the vectors taken directly from the
  §7.6.3 motion-vector predictors (§7.6.6.4). The
  MPEG-1 (ISO/IEC 11172-2) `recon_right`/`recon_down` half-sample
  vectors bridge into the same MC core via `MotionVectorPel::from_mpeg1`
  / `FrameMotion::from_mpeg1`. Frame-picture **frame-based** and
  **field-based** prediction are both driven end-to-end: the field-based
  path (Table 7-14 `Field-based` rows) predicts the macroblock's even
  (top-field) frame lines with the first vector and its odd lines with
  the second, each reading the reference **field its own §6.3.17.2
  `motion_vertical_field_select` flag names** (§7.6.4: `0` → top, `1` →
  bottom — the destination parity does not imply the source field), via
  the §7.6.4 `FieldReference` half-height field view
  (`predict_field_block`; field line `k` → frame row `2k + parity`,
  vertical pad-to-edge confined to the field), combining the directions
  per §7.6.7.2 and reusing the frame-based residual-add / §6.1.3
  write-out path (`reconstruct_field_based_macroblock`). These cover
  MPEG-1 entirely and the bulk of MPEG-2 frame-coded P/B (progressive
  and interlaced-in-frame). **Field-*picture* simple field prediction**
  (§7.6.1 *"within a field picture all predictions are field
  predictions"*, Table 7-13 `Field-based` rows) is now driven
  end-to-end too: `decode_field_picture` walks a field picture's slices
  with `PictureStructure::TopField` / `BottomField` (so the §6.2.5.1
  macroblock-modes parse selects `field_motion_type` and the §6.2.5.2
  `motion_vectors()` reads the `motion_vertical_field_select` bit),
  reconstructs the §7.6.3 vectors, and reconstructs each macroblock as a
  single 16×16 field block read from the reference frame's chosen field —
  Top when the §6.3.17.2 field-select flag is `0`, Bottom when `1`
  (§7.6.4) — via `reconstruct_field_picture_macroblock`
  (`FieldPictureMotion`: per-direction `(luma vector, FieldParity)`,
  §7.6.3.7 chroma scaling, §7.6.7.2 `// 2` bidirectional average, §6.1.3
  contiguous field-plane write-out since a field picture has no
  frame/field DCT distinction). The §7.6.6.2 P-field skip reconstructs as
  a `(0,0)` same-parity-field copy. **Field-picture 16×8 motion
  compensation** (§7.6.7.3, Table 7-13 `16x8 MC` rows) is now driven
  end-to-end too: `decode_field_picture` dispatches a `16x8 MC`
  macroblock to `reconstruct_field_picture_16x8_macroblock`
  (`FieldPicture16x8Motion`), which forms two separate predictions —
  `vector'[0]` for the upper 16×8 luminance region, `vector'[1]` for the
  lower — each carrying its own §6.3.17.2 `motion_vertical_field_select`
  flag so each region reads from its own chosen reference field (Top when
  `0`, Bottom when `1`, §7.6.4). The chroma regions follow §7.6.7.3 (full
  component width × half height per region: 4:2:0 → 8×4, 4:2:2 → 8×8,
  4:4:4 → 16×8) and reuse the §7.6.7.2 bidirectional average + §6.1.3
  field-plane write-out. **Dual-prime motion compensation** (§7.6.3.6 /
  §7.6.7.4, Table 7-13 / 7-14 `Dual prime` rows) is now driven end-to-end
  for **both** picture structures. The §7.6.3.6 opposite-parity vector
  derivation (`dual_prime::derive_all`, Tables 7-12 `m` / 7-13 `e` + the
  inline `dmvector[0..1]`) feeds two new MC drivers: the **field-picture**
  path (`reconstruct_field_picture_dual_prime_macroblock`,
  `FieldPictureDualPrimeMotion`) forms a same-parity prediction from the
  decoded `vector'[0][0]` and an opposite-parity prediction from the
  derived `vector'[2][0]`, averaging them per §7.6.7.4 `// 2`; the
  **frame-picture** path (`reconstruct_frame_dual_prime_macroblock`,
  `FrameDualPrimeMotion`) forms the four field predictions — top field from
  top reference (`vector'[0]`) + bottom reference (`vector'[2]`), bottom
  field from bottom reference (`vector'[0]`) + top reference (`vector'[3]`)
  — averages each field, and interleaves the two into the frame at stride 2
  (`top_field_first` selecting the Table 7-12 frame row). `decode_inter_picture`
  / `decode_field_picture` dispatch a `Dual prime` macroblock to these
  drivers (forward-only P-pictures, §7.6.3.6). The **field-picture B-field
  skip** (§7.6.6.3) now inherits the previous coded macroblock's
  forward/backward direction and motion vectors, forcing the same-parity
  reference field, in `reconstruct_skipped_field_macroblock`. 16×8-MC stays
  field-picture-only per the §7.6 constraint *"16x8 motion compensation
  shall only be used with field pictures"*, so there is no frame-picture
  16×8 path. The remaining motion-compensation surface is now driven
  end-to-end; the GOP-level reference-management / picture-reordering
  loop is driven by `decode_video_sequence`, which the runtime
  `Decoder` adapter now wraps (see **Runtime decoder** below).
- **Spatial-scalable lower-layer resampling**: the full §7.7.3 spatial-
  prediction pipeline — §7.7.3.4 deinterlace (`deinterlace`: the
  Table 7-19 vertical/temporal FIR, two-field aperture for Frame-Picture
  luma and one-field aperture for chroma / field-picture luma, with
  `sum // 16` saturation and the same-field nearest-neighbour border
  extension), §7.7.3.5 vertical and §7.7.3.6 horizontal linear-
  interpolation upsampling that resamples the progressive frame onto the
  enhancement-layer sample grid (`vertical_resample` carries the ×16
  scale, `horizontal_resample` folds both stages' scaling with one
  `// 256`, `resample_progressive` composes them), and §7.7.3.7
  reinterlace (`reinterlace`: progressive copy, or the top/bottom
  field-select line demultiplex) to form `pel_pred_spat` / `spat_pred_pic`,
  with the §4.1 `/` / `//` index-and-phase arithmetic, pad-to-edge border
  extension, and the Table 7-16/7-17/7-18 luma/chroma local-variable
  derivation. The §7.7.3.1 / Table 7-15 case dispatch
  (`UpsampleCase::select`) resolves the five upsampling rows from
  `(lower_layer_deinterlaced_field_select, lower_layer_progressive_frame,
  progressive_frame)` — including the two *"field_select shall be '1'"*
  constraints — and `upsample_spatial_prediction` composes the
  deinterlace → resample → reinterlace stages per the selected row to
  emit `spat_pred_pic` for one component.
- **Spatial-scalable prediction combination**: the §7.7.4 *"precise
  method for predictor calculation"* — combining the temporal
  enhancement-layer prediction with the spatial lower-layer prediction
  under the Table 7-21 `spatial_temporal_weight`, in both the single
  `(a)` whole-block form and the per-field `(a; b)` even/odd-row form
  (the `weight ∈ {0, 0.5, 1}` cases, with the `// 2` average for `0.5`).
  The **picture-level** spatial-prediction driver
  (`spatial_prediction_picture`) derives the Table 7-16 / 7-17 / 7-18
  `ResampleParams` from the parsed scalable-extension geometry and
  upsamples a whole lower-layer frame's Y/Cb/Cr planes to the
  enhancement-grid `SpatialPredictionPicture`; the **per-macroblock**
  combiner (`combine_macroblock_spatial_temporal` /
  `extract_colocated_spatial`) reads the co-located `spat_pred_pic` block
  at a macroblock's position (with §7.7.3 pad-to-edge border extension)
  and blends it with the temporal prediction under the resolved weight.
- **SNR-scalable coefficient addition (§7.8.3.4)**: `add_layer_block`
  forms `F'' = F''lower + F''enhance`; the `chroma_simulcast == 1` case
  (`add_layer_chroma_simulcast`) predicts the chroma DC from the lower
  layer and takes AC from the enhancement layer, with the Table 7-27
  `simulcast_dc_predictor_block` lookup selecting the coincident
  lower-layer chroma block per `(base, upper)` chroma pair.
- **Temporal-scalable reference selection (§7.9)**:
  `PictureTemporalScalableExtension::resolve_references` maps the
  `reference_select_code` into the named Table 7-28 (P-picture) /
  Table 7-29 (B-picture) prediction reference sources
  (`PictureReferences` / `ReferenceSource`).
- **Data partitioning (§7.10)**: `data_partitioning::split_data_partitions`
  divides a non-scalable ISO/IEC 13818-2 stream into its partition 0 /
  partition 1 pair at a Table 7-30 `priority_breakpoint` (`1..=3`, or
  `64..=127` for the per-`(run, level)`-pair breakpoints) — inserting
  the `sequence_scalable_extension()`, the slice-level
  `priority_breakpoint`, and the §7.10 redundant header copies (only
  `sequence_extension()` / `picture_coding_extension()` /
  `sequence_scalable_extension()` in partition 1) — and
  `merge_data_partitions` is the §7.10 *"current_partition"* switching
  procedure applied bitstream-to-bitstream, re-forming the non-scalable
  stream that `decode_video_sequence` reconstructs
  (`decode_data_partitioned` chains the two). Both directions are one
  engine over the crate's per-element syntax parsers (address
  increment, `macroblock_type` + modes tail, `quantiser_scale_code`,
  `motion_vectors(s)`, the concealment `marker_bit`,
  `coded_block_pattern()`, the §7.2.1 DC prelude, the §7.2.2 run-level
  walker); `merge(split(s, pb)) == s` **byte-exactly** for every `.m2v`
  stream of the conformance and self-encoded corpora at breakpoints
  1 / 2 / 3 / 64 / 65 / 72 / 127, §5.2.3 zero stuffing preserved, and
  hostile pairs (swapped, duplicated, truncated, non-scalable,
  ISO/IEC 11172-2) are rejected without panics.
- **Block / macroblock drivers**: `mpeg2_block_decoder::decode_block`
  chains DC prelude → residual VLC → inverse scan → inverse quant →
  IDCT into a single bitstream→plane entry point, and
  `mpeg2_macroblock_blocks::decode_macroblock_blocks` walks the
  `pattern_code[]` array per the 4:2:0 / 4:2:2 / 4:4:4 layouts.

Each stage is covered by synthetic unit tests plus integration fixtures
verifying the parsers against known-good encoded streams.

## Reference conformance

`tests/fixtures/conformance/` stages a **whole-sequence conformance
corpus**: twelve elementary streams — MPEG-1 IBBP GOPs, high-motion
wide-`f_code`, and VCD-rate CBR SIF; MPEG-2 IBBP with adaptive
quantisation, interlaced field prediction, `intra_vlc_format` +
non-linear quant + 10-bit DC, 4:2:2 profile, non-macroblock-multiple
100×62, and a hand-built field-picture stream (I/P/B field pairs with
both `motion_vertical_field_select` parities, §7.6.3.6 **dual prime**,
§7.6.7.3 **16×8 MC**) — each paired with a black-box reference decode.
`tests/reference_conformance.rs` decodes every stream end-to-end
through `decode_video_sequence` and holds it to: exact frame count and
dimensions, per-sample |Δ| ≤ 3 (the Annex A IDCT is only specified to
IEEE 1180 statistical accuracy, so conforming decoders may differ by
±1 per transform; empirically the corpus decodes at |Δ| ≤ 2 everywhere
but a single sample), and < 5 % differing samples per frame.
**MPEG-1 (ISO/IEC 11172-2) streams decode whole-sequence too**: the
driver classifies the sequence layer (no `sequence_extension()` →
11172-2) and routes pictures through the §2.4.4 MPEG-1 block/motion
pipeline (`mpeg1_block_decoder` + `mpeg1_picture`), including the
`dct_dc_*_past`/`past_intra_address` DC chain, the §2.4.4.2/.3
`recon_*_prev` predictor lifecycle, §2.4.4.4 skips, `full_pel_*_vector`
scaling and sequence-header quantiser matrices. **D-pictures**
(dc intra-coded, `picture_coding_type == 4`, §2.4.3.4) decode too:
the Table B.2d 1-bit `macroblock_type`, six DC-only blocks per
macroblock (no AC walk / `end_of_block`, §2.4.2.8), the
`end_of_macroblock` marker, and coded-order display (§2.4.1 D-only
sequences) — pinned by the hand-built `mpeg1-dpics-48x32.m1v` fixture
decoded sample-exactly against closed-form §2.4.4.1 arithmetic (no
black-box reference decoder in reach accepts type-4 pictures).

## Runtime decoder

`register` / `register_codecs` install `oxideav_core::Decoder`
factories under both the `mpeg1video` and `mpeg2video` codec ids
(claiming the `mp1v` / `mpg1` / `mp2v` / `mpg2` / `hdv2` / `m2v1`
FourCC and `V_MPEG1` / `V_MPEG2` Matroska tags the container crates map
onto them). The `decoder::Mpeg12Decoder` adapter bridges the
whole-elementary-stream driver `decode_video_sequence` to the
packet-oriented `Decoder` contract: it concatenates every packet's
payload into one contiguous elementary-stream buffer (the §6.1.1.11
display reorder spans the whole sequence, so a B-picture cannot commit
until its trailing coded-order anchor has been decoded), runs the driver
on `flush()`, and drains the reconstructed frames in display order —
returning `NeedMore` before the flush and `Eof` once drained. Each
reconstructed `FrameBuffer` converts to a tightly-packed planar Y/Cb/Cr
`VideoFrame` (`frame_buffer_to_video_frame`, `stride == plane width`)
stamped with a monotonic display-order presentation index, and `reset()`
returns the decoder to a fresh state. The direct
`decoder::make_decoder` factory and the `oxideav_core::register!`
registry path both reach it. `tests/runtime_decoder.rs` proves the
trait output is **sample-exact** with `decode_video_sequence` on the
real 352×240 4:2:0 fixture (and under a split-packet feed), that both
codec ids resolve through a `RuntimeContext`, and that `reset` makes the
decoder reusable.

## Encoder

An **MPEG-2 video encoder** is in hand for the baseline intra path
**and motion-compensated P + B pictures**, and a **conformant MPEG-1
(ISO/IEC 11172-2) encoder** now sits beside it — both built as the
bit-exact inverse of the decode pipeline so that everything they emit
round-trips back through `decode_video_sequence`. The MPEG-2 side:

- **§A forward DCT** (`forward_dct::fdct_8x8`, plus the `f64` reference
  / separable layers) — the transpose of the §A IDCT kernel.
- **Forward quantiser** (`forward_quant::forward_quantise_block`)
  inverting the §7.4.2.3 arithmetic: round-to-nearest for intra AC,
  dead-zone-toward-zero for non-intra, `Round(F/intra_dc_mult)` for the
  §7.4.1 intra DC.
- **Entropy encoders** against the same Annex B tables the decoder
  walks: `mpeg2_block_dc::encode_intra_dc` (§7.2.1 DC size VLC +
  differential), `mpeg2_dct_coeff::encode_dct_coeff` /
  `encode_end_of_block` (§7.2.2 run-level VLC + Table B-16 escape, with
  the §7.2.2.2 NOTE 2/3 FIRST/NEXT gating), `coded_block_pattern::encode_cbp420`
  (§6.2.5.3 Table B-9), and `motion_vector::encode_motion_vector` /
  `split_delta` (§6.2.5.2.1 Tables B-10/B-11, inverting §7.6.3.1).
- **Bitstream layer writers** (`stream_writer`) for the §6.2
  sequence / sequence-extension / picture / picture-coding-extension /
  slice headers + the sequence-end code.
- **`encode_intra_picture`** — a complete all-intra frame-picture
  encoder (sequence header → sequence-end code). The encode→decode
  round-trip in `tests/encode_intra_roundtrip.rs` proves: a flat frame
  round-trips exactly, a gradient round-trips with luma MAE < 4, and the
  encoder is reconstruction-idempotent (decode → re-encode → decode is a
  pixel-exact fixed point — the forward and inverse quantisers are exact
  inverses on the lattice).
- **`encode_nonintra_block`** + **`encode_p_copy_picture`** /
  **`encode_i_then_p_copy`** — the inter residual-block encoder
  (dead-zone non-intra quantise, no DC prelude, `dct_coeff_first`
  leading symbol) and a zero-MV P-picture assembler that reproduces the
  forward anchor exactly when decoded.
- **Motion estimation** (`motion_estimation::estimate_forward_mv`) — an
  integer-pel full search + half-pel refinement scoring each candidate by
  the SAD of the exact `forming_predictions::predict_block` prediction the
  decoder forms, with the window clamped to the §7.6.3.1 codable band.
- **`encode_p_picture`** — a full motion-compensated P-picture: per-MB
  search, §7.6.4 prediction, `current - prediction` residual, Table B-3
  `MC, Coded` / `MC, Not Coded` mode, §6.2.5.3 cbp, MVs differentially
  coded against the §7.6.3.4 PMV, **plus an intra-MB fallback** (Table
  B-3 `00011`) for content the prediction can't capture. The encoder
  reconstructs each MB the way the decoder does and returns that frame so
  it chains as the next reference.
- **`encode_b_picture`** — a bidirectional B-picture: forward / backward
  / interpolated (§7.6.7.1 `// 2` average) prediction chosen per MB by
  luma SAD, Table B-4 mode, with forward MVs before backward and a
  per-direction PMV slot.
- **Stream assemblers** — `encode_i_then_p` (I→P), `encode_i_p_chain`
  (I→P→P→… reference rotation), `encode_i_p_b` (I→P→B coded order,
  display I-B-P), and `encode_display_order_sequence` (a whole
  display-order frame list assembled as `I (B…) P (B…) P …` with
  §6.1.1.11 coded-order emission and per-display-index
  `temporal_reference`). Each decodes the intermediate anchors so the encoder
  predicts from the decoder's exact reconstruction, making the whole
  round-trip faithful. `tests/encode_inter_roundtrip.rs` proves a
  motion-compensated copy is a bit-exact fixed point, a clean translation
  reconstructs with luma MAE < 4, an unpredictable region triggers the
  intra fallback, and an I-B-P group decodes in display order.
- **GOP-structured assembly** — `encode_display_order_gop_sequence`
  extends the display-order assembler with §6.2.2.6
  `group_of_pictures_header()` emission (`gop_header::write_gop_header`
  + `TimeCode::from_display_index`, which counts at the Table 6-4
  nominal integral rate per the `drop_frame = 0` rule): one I-picture
  per GOP, per-GOP time codes, `closed_gop = 1` (the emitted structure
  never codes a B across a GOP boundary) with `broken_link = 0`, and
  the §6.3.9 per-GOP `temporal_reference` reset.

An **MPEG-1 (ISO/IEC 11172-2) encoder** (`mpeg1_encoder`) mirrors the
whole path against the crate's own §2.4 decode pipeline:

- **Sequence layer** — `mpeg1_stream_writer::write_mpeg1_sequence_header`
  (the §2.4.2.3 header: `pel_aspect_ratio` / `picture_rate` semantics,
  **no** `sequence_extension()` — its absence is the 11172-2
  classification) with `constrained_parameters_admissible` evaluating
  the §2.4.3.2 bounds (768×576 / 396-MB / MB×rate / rate ≤ 30 /
  f_code ≤ 4 / 1 856 000 bit/s / 327 680-bit VBV) so the assembler
  sets `constrained_parameters_flag` exactly when they hold; the
  **mandatory** §2.4.2.4 GOP layer is always emitted.
- **Entropy layer** — `block_dc::encode_dc_coefficient` (Tables
  B.5a/B.5b DC prelude, exhaustively round-tripped over `[-255, 255]`),
  `dct_coeff::encode_dct_coeff` (Tables B.5c/B.5d/B.5e + the Table
  B.5f short/long escape, exhaustively round-tripped over run 0..=63 ×
  |level| 1..=255), and the §2.4.4.1 round-to-nearest / §2.4.4.2
  dead-zone forward quantisers (`forward_quant::mpeg1_*`,
  `quantizer_scale` used directly).
- **Picture encoders** — `encode_mpeg1_intra_picture` (DC differentials
  against the exact `dct_dc_*_past` chain including the
  `past_intra_address` reset branch), `encode_mpeg1_p_picture`
  (Table B.2b modes + intra fallback, §2.4.4.2 `recon_*_prev`
  differential MVs, the picture header's real 3-bit f_code +
  `full_pel = 0`), and `encode_mpeg1_b_picture` (Table B.2c
  forward/backward/interpolated modes, per-direction predictors
  updated only on transmission). All reconstructions come from the
  decoder's own dequantiser + Annex A IDCT, so decodes are
  sample-exact against the encoder's returned frames.
- **`encode_mpeg1_display_order_sequence`** — the whole display-order
  assembler: `sequence_header` → one `group_of_pictures()` per GOP
  (time codes from each GOP's first display frame, `closed_gop = 1`,
  `broken_link = 0`, per-GOP `temporal_reference` reset per §2.4.3.4)
  → `I (B…) P (B…) P` coded-order groups that never predict across a
  GOP boundary → `sequence_end_code`. D-pictures are never emitted
  (§2.4.3.4 confines them to D-only sequences).
  `encode_mpeg1_intra_stream` is the single-frame degenerate.
  `tests/encode_mpeg1_roundtrip.rs` pins sample-exact I/P/B decodes,
  the two-GOP temporal-reference pattern, parse-back of the GOP
  headers/time codes, 100×62 geometry, wide motion at f_code 4, and
  the absence of any extension start code.

Both encoders now have **Annex C rate control**, and the MPEG-2
encoder codes **field pictures** end-to-end:

- **CBR rate control (`rate_control` / `vbv`)** — the `vbv` module is
  an *exact* implementation of the ISO/IEC 13818-2 Annex C (C.1–C.12)
  / ISO/IEC 11172-2 Annex C (C.1.1–C.1.4) Video Buffering Verifier:
  integer arithmetic in sub-units that keep every Table 6-4 rational
  frame rate, frame/field removal cadence, and 90 kHz `vbv_delay`
  tick exact (Annex C prescribes real-valued arithmetic), the
  C.3.1 / §6.3.9 / §2.4.3.4 `vbv_delay = 90 000 · B*(n) / R`
  computation, and the C.5 / C.6 occupancy bounds at every removal.
  On top of it sit three VBV-regulated assemblers —
  `encode_cbr_gop_sequence` (MPEG-2 frame pictures, C.9 cadence),
  `encode_field_cbr_gop_sequence` (MPEG-2 field pictures, C.11
  field-period cadence), and `encode_mpeg1_cbr_sequence` (11172-2,
  §2.4.3.2 picture rate, constrained-parameters flag evaluated as in
  the plain assembler) — which *satisfy* the `bit_rate` /
  `vbv_buffer_size` their sequence headers declare: a picture that
  exceeds the C.6 underflow bound is re-encoded at a coarser
  quantiser, one that undershoots the C.5 overflow bound draws
  zero-byte stuffing (§5.2.3 / §2.3 `next_start_code()` zero
  stuffing), a soft per-GOP I/P/B budget feedback steers the running
  per-type quantisers between the hard bounds, and every picture
  header carries the real `vbv_delay` (never the `0xFFFF`
  variable-rate sentinel). A whole-stream verifier
  (`vbv::verify_cbr_stream`) re-derives the model from the coded
  stream alone — declared parameters, per-picture removal cadence
  from `picture_structure`, picture-data spans per the Annex C
  definition — and holds it to the occupancy bounds plus
  C.3.1-consistent coded delays (±1 tick of quantisation); the CBR
  round-trip tests and the pinned CBR corpus streams all pass it.
- **Field-picture inter encode (`field_picture_encoder`)** — the
  encoder-side mirror of the §7.6.1 field-picture decode path:
  `encode_field_intra_picture` / `encode_field_p_picture` /
  `encode_field_b_picture` emit `picture_structure` = Top/Bottom
  field pictures (§6.3.10 constants: `top_field_first = 0`,
  `repeat_first_field = 0`, `frame_pred_frame_dct = 0`,
  `progressive_frame = 0`) with `field_motion_type = 01` (Table 6-18
  Field-based, no `dct_type` per §6.2.5.1), the §6.2.5.2
  `motion_vertical_field_select` flag, and field-unit motion vectors
  differentially coded against the §7.6.3.4 PMV (Table 7-9 both-slot
  update). The motion search (`estimate_field_mv`) scores every
  `(reference field parity, vector)` candidate by the SAD of the
  exact §7.6.4 prediction under §7.6.3.8-legal spans; P pictures
  carry the Table B-3 modes + intra fallback, B pictures the Table
  B-4 forward/backward/interpolated modes with per-direction PMV
  slots and field selects. `encode_field_display_order_gop_sequence`
  assembles whole interlaced sequences as §6.1.1.4.1 field pairs
  (top first, shared `temporal_reference`, closed GOPs) honouring the
  **§7.6.2.1 second-field-of-a-P-frame rule** via the same synthetic
  reference frame the decode loop builds. Reconstruction is
  decoder-exact: `tests/encode_field_pictures.rs` pins sample-exact
  I/P/B field-picture decodes against the encoder's reconstruction,
  a cross-parity prediction proof (a one-frame-line shift predicts
  through the opposite-parity field at a fraction of intra cost),
  and whole-sequence decode through `decode_video_sequence`.

The **4:2:2-profile encoder leg** (and a 4:4:4 leg) landed in round
447 — the frame-picture I/P/B encoders, the display-order / GOP
assemblers and the Annex C CBR controller are now **chroma-format
generic**:

- **Profile signalling**: `profile_and_level_indication` follows the
  chroma format (4:2:0 → Main@Main `0x48`; 4:2:2 / 4:4:4 → High@Main
  `0x18` — High is the one profile whose Table 8-5 row admits 4:2:2
  chroma in the 1995 text; 4:4:4 has no conformant label there and
  reuses the High superset, documented).
- **Eight/twelve-block macroblocks** on the encode path (Figures
  6-11 / 6-12 numbering, the interleaved Cb/Cr §7.2.1 DC chain, the
  §7.6.3.7 chroma vector scaling already shared with the decoder) and
  the §6.2.5.3 `coded_block_pattern()` extensions:
  `encode_coded_block_pattern` emits the Table B-9 VLC plus the 4:2:2
  two-bit `coded_block_pattern_1` / 4:4:4 six-bit
  `coded_block_pattern_2` per §6.3.17.4 (cbp 0 emitted exactly where
  Table B-9's NOTE permits — never at 4:2:0, legal at 4:2:2 when only
  extension blocks are coded). At 4:4:4 the **printed** §6.3.17.4
  derivation gives non-intra blocks 6/7 no wire slot
  (`coded_block_pattern_2` bits `11 - i` reach `pattern_code[8..12]`
  only), so the P/B encoders leave those residuals untransmitted with
  the reconstruction in agreement — emitted bits 5..4 stay zero, so
  the streams decode identically under a corrected six-block reading.
- **`intra_vlc_format` and `alternate_scan` are now honoured on the
  wire** (Table 7-3 → Table B-15 intra AC; §7.3 scan selection) beside
  the already-threaded non-linear `q_scale_type` and 8..=11-bit
  `intra_dc_precision`; the field / frame-field encoders explicitly
  reject the two flags they don't code.
- **§6.3.11 quantiser-matrix emission**: sequence-header
  `load_intra/non_intra_quantiser_matrix` payloads plus a
  `quant_matrix_extension()` writer for the chroma tables (w = 2 /
  w = 3, 4:2:2 / 4:4:4 only), with writer-side validation and the
  forward quantisers using exactly the Table 7-5 bank the decoder
  resolves (`encode_intra_picture_with_matrices`,
  `encode_p_picture_with_matrices`, `encode_b_picture_with_matrices`,
  `encode_display_order_gop_sequence_with_matrices`).

Both encoders are **externally conformance-validated**: a pinned
self-encoded corpus (`tests/fixtures/selfenc/` — twenty-two streams:
MPEG-2 all-intra 64×48 and non-macroblock-multiple 100×62, an I+3P
motion-compensated chain with intra fallback, an I/B/P group, a
7-frame IBBP display-order sequence, a two-GOP MPEG-2 stream with GOP
headers, an Annex C **CBR** MPEG-2 stream (240 kbit/s, 65 536-bit VBV,
real `vbv_delay` values, verifier-checked), a **field-coded** I B P B P
sequence (§6.1.1.4.1 field pairs, both `motion_vertical_field_select`
parities, §7.6.2.1 second-P-field reference), an **adaptive
field-mode** I P P mixing simple / 16×8 / dual-prime macroblocks in
the same slices, an interlaced **frame-picture field-based** I B P B P
(`frame_pred_frame_dct = 0`, field MC + field DCT), a frame-picture
**dual-prime** I P P, a **4:2:2-profile** I B P B P, a 4:2:2
I B P B P under the full flag set (`intra_vlc_format`,
`alternate_scan`, non-linear `q_scale_type`, 10-bit DC) with §6.3.11
luma-and-chroma matrix downloads, a **4:4:4** I B P, a
**skipped-macroblock + concealment-vector** I B P B P, a
**frame-field** I B P B P under the full entropy flag set, six MPEG-1
streams — all-intra, one-GOP
I P P P, a two-GOP I B B P | I B B P, an I B P with downloadable
§2.4.3.2 quantiser matrices, an 11172-2 Annex C CBR two-GOP stream
with the constrained-parameters flag set, and a **D-only** sequence
(the one stream with no black-box reference: the reference binary
emits zero frames for `picture_coding_type == 4`, so it is pinned
bit-exactly and decoded sample-exactly against the encoder's own
reconstruction)) decodes in a black-box reference decoder (strict
error-detection mode clean; for the field-pair streams the strict
mode flags packets while decoding all frames — the same documented
behaviour as the `fieldpics` conformance fixture) with its committed
reference decode agreeing with ours at max |Δ| 2 (pure Annex A IDCT
rounding).
`tests/selfenc_conformance.rs` pins every stream **bit-exactly**
(regenerate-and-compare against the committed bytes) so any
bit-moving encoder change must consciously refresh the corpus and
re-run the black-box validation. Getting there fixed a real encoder
bug: the motion search could pick §7.6.3.8-illegal vectors at
right/bottom edge macroblocks (scored through the padding predictor,
mirrored by our own padding decoder); it now visits only vectors
whose whole §7.6.4 read span stays inside the coded picture. The
MPEG-2 encoders declare `progressive_sequence = 1` (+ §6.3.10
`progressive_frame`), matching the `Ceil(h/16)` grid they code; the
MPEG-1 streams declare (and satisfy) the §2.4.3.2
constrained-parameters bounds.

The whole remaining §7.6 motion-compensation **encode** surface
landed in round 443:

- **Frame-picture field-based encode (`frame_field_encoder`)** — the
  `frame_pred_frame_dct = 0` frame-picture path:
  `encode_ff_intra_picture` / `encode_ff_p_picture` /
  `encode_ff_b_picture` and the interlaced display-order assembler
  `encode_ff_display_order_gop_sequence`
  (`progressive_sequence = 0`, §6.3.3 `2*Ceil(h/32)` grid). Per
  macroblock the encoder chooses the Table 6-17 `frame_motion_type`
  — **Frame-based** (one frame vector), **Field-based** (two field
  vectors, each with its own §6.2.5.2 `motion_vertical_field_select`
  parity, found by a both-parity 16×8 field-in-frame search over
  §7.6.3.8-legal spans), or **Dual-prime** (§7.6.3.6: one vector +
  Table B-11 `dmvector`, searched over the exact §7.6.7.4
  four-field-average prediction; P pictures with `b_between = 0`
  only) — plus the intra fallback, and picks the `dct_type` (field
  DCT, §6.1.3 stride-2 luma organisation) by **exact wire-bit cost**
  of both quantised organisations. Motion vectors mirror §7.6.3.1 /
  §7.6.3.3 exactly against the crate's own `Pmv` bank (field
  vectors in frame pictures code the vertical against `PMV DIV 2`
  and write back `2 * vector'`; the Table 7-10 update rows are
  applied per macroblock), and every macroblock is reconstructed by
  the decode-side §7.6 drivers, so decodes are sample-exact against
  the encoder's reconstruction (`tests/encode_frame_field.rs`).
  `FrameFieldStats` surfaces the per-macroblock decisions.
- **Adaptive field-picture modes** —
  `encode_field_p_picture_adaptive` /
  `encode_field_b_picture_adaptive` /
  `encode_field_adaptive_display_order_gop_sequence` emit the full
  Table 6-18 `field_motion_type` surface per macroblock: simple
  field prediction (`01`), **16×8 MC** (`10`, §7.6.7.3 — an
  independent `(vector, field-select)` per 16×8 region), and
  **dual-prime** (`11`, P-only, §7.6.3.6), with the Table 7-11
  predictor-update rows mirrored exactly and reconstruction through
  the decode-side drivers (`tests/encode_field_adaptive.rs`;
  `FieldModeStats`).
- **MPEG-1 D-picture encode** — `encode_mpeg1_d_picture` /
  `encode_mpeg1_d_sequence` (§2.4.3.4 `picture_coding_type = 4`,
  §2.4.1 D-only sequences): Table B.2d macroblock type, six DC-only
  blocks against the exact §2.4.4.1 `dct_dc_*_past` chain (no AC
  walk / `end_of_block`, §2.4.2.8), `end_of_macroblock` bits, GOP
  layers with per-GOP `temporal_reference` reset — round-tripped
  sample-exactly through `decode_mpeg1_d_picture` and
  `decode_video_sequence` (`tests/encode_d_pictures.rs`).

MPEG-1 `full_pel_*_vector = 1` emission is supported per direction
(`write_mpeg1_picture_header` + the P/B encoders' `full_pel_*` flags:
vectors confined to integer-pel positions, the wire carrying the
unshifted values the §2.4.4.2 / §2.4.4.3 final `recon <<= 1`
restores; pinned by a sample-exact round-trip and a strict black-box
decode).

Round 453 closed the frame-picture encode remainders behind
**`FrameEncodeOptions`** (`encode_options`; `_with_options` /
`_with_stats` variants of the I / P / B encoders and a per-display-
frame options closure on
`encode_display_order_gop_sequence_with_options`, which also returns
the whole-sequence `FrameEncodeStats` macroblock counts):

- **Skipped macroblocks (§7.6.6)** — the P encoder skips a
  macroblock whose zero-vector frame prediction quantises to an
  all-zero residual (the §7.6.6.2 reconstruction, predictors reset);
  the B encoder tries the previous macroblock's direction with the
  current `PMV` vectors first and skips when that residual quantises
  to zero (§7.6.6.4, predictors untouched, §7.6.3.8 span legality
  checked on the inherited vectors). Never the first / last
  macroblock of a slice, never in I-pictures; runs ride Table B-1
  `macroblock_escape` chains (`encode_mb_address_increment`).
- **Concealment motion vectors (§7.6.3.9)** — every intra macroblock
  (all of an I-picture's, the P encoder's intra fallbacks) carries a
  frame-format forward vector coded against `PMV[0][0]` plus the
  `marker_bit`, with the Table 7-9 both-slot predictor write-back;
  per the §7.6.3.9 recommendation the vector suits the macroblock
  *below* (searched against the previous anchor), `(0, 0)` on the
  bottom row. I-pictures write a real `f_code[0][*]` when (and only
  when) the flag is set, and the decode side
  (`decode_intra_picture_with_context`) walks such I-pictures.
- **§6.3.10 output-cadence signalling** — `top_field_first`,
  `repeat_first_field` and a `progressive_frame` override, validated
  against the §6.3.10 consistency rules
  (`PictureCodingExtensionParams::validate_frame_picture_flags`),
  with `FrameEncodeOptions::pulldown_32` for the classic 3:2
  pattern; `DecodedFrame` surfaces the three flags plus the
  `output_field_count` / `output_frame_count` helpers. The writer
  also derives `chroma_420_type` from `progressive_frame` and emits
  `top_field_first = 0` where a progressive sequence requires it.
- **Interlaced entropy flags** — the field-picture encoders (plain
  and adaptive) and the frame-field encoder honour `alternate_scan`
  and `intra_vlc_format` on the wire (through the block writers, the
  `dct_type` wire-bit cost comparison and the intra AC cost probe),
  so the whole picture-coding-extension flag set is encodable in
  every picture structure at 4:2:0.

## Runtime encoder

`register` installs `oxideav_core::Encoder` factories under both the
`mpeg1video` and `mpeg2video` codec ids beside the decoders (the
direct `encoder::make_encoder` factory is exported too). The
`encoder::Mpeg12Encoder` adapter drives the display-order GOP
assemblers behind the frame-to-packet `Encoder` contract, mirroring
the runtime decoder's whole-elementary-stream framing: display-order
frames buffered via `send_frame` are assembled at `flush()` into one
keyframe-flagged packet carrying the finished elementary stream
(`NeedMore` before the flush, `Eof` after the drain). Four optional
`CodecParameters` options — `quantiser_scale_code`, `b_between`,
`anchors_per_gop`, `f_code` — are range-validated at construction.
`tests/runtime_encoder.rs` proves registry resolution under both ids,
round-trips through both decode paths, and the 11172-2 classification
of the `mpeg1video` output (no extension start codes).

The frame-picture encode path covers 4:2:0 / 4:2:2 / 4:4:4 with
downloadable quantiser matrices, every picture-coding-extension
entropy flag, skipped macroblocks, concealment motion vectors and the
§6.3.10 output-cadence flags; the **field / frame-field** encoders
honour every entropy flag but remain 4:2:0-only. The runtime
`Encoder` adapter drives the baseline (no-options) assemblers.

## Not yet supported

- **Scalable multi-layer decode loops** for the spatial (§7.7), SNR
  (§7.8) and temporal (§7.9) profiles. Every per-stage building block
  is implemented and unit-tested — the §7.7.3 deinterlace / resample /
  reinterlace chain with the Table 7-15 case dispatch and the
  picture-level `spatial_prediction_picture` driver, the §7.7.4
  spatial/temporal combiner, the §7.8.3.4 coefficient addition (with
  the `chroma_simulcast` Table 7-27 case), the Table 7-28 / 7-29
  temporal reference selection, and all the extension parsers — but
  the top-level loop that demultiplexes an enhancement-layer bitstream
  and feeds those combiners picture by picture is not composed. No
  clean-room fixture source exists for these layers (the black-box
  reference binary neither produces nor decodes them), so a self-made
  layered encoder would be the only oracle. Data partitioning (§7.10)
  **is** supported (see above).
- **4:2:2 / 4:4:4 on the interlaced encode paths** — the field-picture
  and frame-field encoders are 4:2:0-only (the frame-picture encoders
  cover all three chroma formats).
- **Scalable-layer encoding** (spatial / SNR / temporal enhancement
  layers) and `vertical_size > 2800` (`slice_vertical_position_extension`)
  on the data-partitioning split / merge.
- The runtime `Encoder` adapter exposes the baseline assemblers only
  (`FrameEncodeOptions`, the interlaced encoders and CBR rate control
  are reached through the direct APIs).

## Clean-room provenance

Every line in `src/` traces to the ISO/IEC 13818-2:1995 (ITU-T H.262)
and ISO/IEC 11172-2:1993 specification PDFs staged under `docs/video/`,
plus `oxideav-core`'s `BitReader` API. An external CLI binary is used
**only** as an opaque encoder to produce integration-test fixtures; its
source code was not consulted. No external library source was read,
quoted, or paraphrased.

## License

MIT — see [LICENSE](LICENSE).
