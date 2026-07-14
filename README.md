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
- **Block / macroblock drivers**: `mpeg2_block_decoder::decode_block`
  chains DC prelude → residual VLC → inverse scan → inverse quant →
  IDCT into a single bitstream→plane entry point, and
  `mpeg2_macroblock_blocks::decode_macroblock_blocks` walks the
  `pattern_code[]` array per the 4:2:0 / 4:2:2 / 4:4:4 layouts.

Each stage is covered by synthetic unit tests plus integration fixtures
verifying the parsers against known-good encoded streams.

## Reference conformance

`tests/fixtures/conformance/` stages a **whole-sequence conformance
corpus**: nine elementary streams — MPEG-1 IBBP GOPs, high-motion
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

An **MPEG-2 video encoder** is now in hand for the baseline intra path
**and motion-compensated P + B pictures**, built as the bit-exact
inverse of the decode pipeline so that everything it emits round-trips
back through `decode_video_sequence`:

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

The encoder is now **externally conformance-validated**: a pinned
self-encoded corpus (`tests/fixtures/selfenc/` — all-intra 64×48 and
non-macroblock-multiple 100×62, an I+3P motion-compensated chain with
intra fallback, and an I/B/P group) decodes in a black-box reference
decoder (strict error-detection mode clean) with its committed
reference decode agreeing with ours at max |Δ| 2 (pure Annex A IDCT
rounding). `tests/selfenc_conformance.rs` pins the encoder
**bit-exactly** (regenerate-and-compare against the committed
streams) so any bit-moving encoder change must consciously refresh
the corpus and re-run the black-box validation. Getting there fixed a
real encoder bug: the motion search could pick §7.6.3.8-illegal
vectors at right/bottom edge macroblocks (scored through the padding
predictor, mirrored by our own padding decoder); it now visits only
vectors whose whole §7.6.4 read span stays inside the coded picture.
The encoders declare `progressive_sequence = 1` (+ §6.3.10
`progressive_frame`), matching the `Ceil(h/16)` grid they code.

Field-picture / field-based inter encoding (`frame_pred_frame_dct = 0`),
rate control, and encoder runtime-registry wiring remain future work
(the **decoder** is now registry-wired — see **Runtime decoder** above).

## Not yet supported

- A top-level **`video_sequence()` decode loop** now exists
  (`decode_video_sequence(stream) -> Vec<DecodedFrame>`): it parses the
  sequence layer once for the geometry, walks every `picture_start_code`,
  dispatches each frame picture to the matching per-picture driver with
  the running §7.6 anchor pair, and reorders the reconstructed frames
  into **display order** per §6.1.1.11 (B-frames pass through, I/P frames
  held back one). The structural reorder is independently cross-checkable
  against the `temporal_reference`-derived display order:
  `display_indices_from_temporal_references` accumulates a continuous
  display index per coded frame across §6.3.8/§6.3.9 GOP resets, and
  `verify_display_order` confirms a display-ordered sequence is strictly
  increasing in those indices (an integration test asserts the two agree
  on a decoded I-P-B run). It covers **frame pictures** (the MPEG-2 common case +
  MPEG-1 entirely) **and field-picture pairs** (§6.1.1.4.1): each field
  picture is reconstructed by `decode_field_picture`, the first field of
  a pair is held until its partner arrives, and the two are interleaved
  into one full-height frame by `assemble_frame_from_fields` (§3.131
  top→even lines / §3.13 bottom→odd lines), with the §7.6.2.1
  second-field-of-a-P-frame reference rule honoured via a synthetic
  reference frame pairing the current first field with the previous
  frame's opposite-parity field. **Downloadable quantiser matrices are
  threaded end-to-end** (§6.3.11): the sequence header's
  `load_*_quantiser_matrix` payloads and every
  `quant_matrix_extension()` update a running matrix state (reset to
  the §6.3.7 defaults at each `sequence_header_code`) that the slice
  walker's §7.4.2.3 reconstruction consumes — proven
  reference-conformant on a custom-matrix black-box fixture and
  exactly (splice/persist/reset) on self-encoded streams. The
  scalable layers are skipped by the start-code scan. The
  per-picture module APIs remain available directly
  (`decode_intra_picture` for I-pictures, `decode_inter_picture` for
  frame-picture P/B, `decode_field_picture` for field-picture P/B,
  each with a `_with_matrices` variant taking the §6.3.11 state),
  with the caller supplying the decoded reference frame(s). All §7.6
  motion-compensation prediction modes are now driven end-to-end: the
  frame-picture frame-based **and** field-based P/B reconstruction, the
  field-picture **simple field prediction**, the field-picture **16×8
  motion compensation**, the **dual-prime** four-/two-field reconstruction
  in both frame and field pictures (§7.6.3.6 / §7.6.7.4), and the
  field-picture **B-field skipped-macroblock** §7.6.6.3 direction
  inheritance. (Frame-picture 16×8-MC does not exist — §7.6 restricts 16×8
  MC to field pictures.)
- Scalability profiles and the spatial/temporal/SNR enhancement layers.
  Nearly all of the per-stage math is now implemented and composed:
  - **Spatial (§7.7)**: the §7.7.3.4 deinterlace, §7.7.3.5/.6 lower-layer
    resampling, §7.7.3.7 reinterlace, the §7.7.3.1 / Table 7-15
    upsampling-case dispatch + `upsample_spatial_prediction` driver, the
    §7.7.4 spatial/temporal prediction combination, and §7.7.5.1 PMV
    reset. The **picture-level spatial-prediction driver**
    (`spatial_prediction_picture`) now derives the Table 7-16 / 7-17 /
    7-18 `ResampleParams` from the parsed `sequence_scalable_extension()`
    + `picture_spatial_scalable_extension()` geometry and runs the
    upsample over a whole lower-layer frame to emit the enhancement-grid
    `SpatialPredictionPicture` (`spat_pred_pic` per component), and the
    **§7.7.4 per-macroblock combiner**
    (`combine_macroblock_spatial_temporal` /
    `extract_colocated_spatial`) extracts the co-located `spat_pred_pic`
    block at each macroblock position (with §7.7.3 border extension) and
    blends it with the temporal prediction under the Table 7-21 weight.
  - **SNR (§7.8)**: the §7.8.3.4 two-layer coefficient addition
    (`add_layer_block`, plus the `chroma_simulcast` DC-prediction case
    `add_layer_chroma_simulcast` with the Table 7-27
    `simulcast_dc_predictor_block` lookup).
  - **Temporal (§7.9)**: the Table 7-28 / 7-29 reference-frame selection
    (`PictureTemporalScalableExtension::resolve_references`).

  What remains uncomposed is the **top-level multi-layer decode loop**
  that demuxes the two layer bitstreams, decodes the lower layer, and
  walks the enhancement-layer macroblocks feeding the temporal
  predictions and lower-layer coefficients/frames into these combiners
  picture-by-picture.

## Clean-room provenance

Every line in `src/` traces to the ISO/IEC 13818-2:1995 (ITU-T H.262)
and ISO/IEC 11172-2:1993 specification PDFs staged under `docs/video/`,
plus `oxideav-core`'s `BitReader` API. An external CLI binary is used
**only** as an opaque encoder to produce integration-test fixtures; its
source code was not consulted. No external library source was read,
quoted, or paraphrased.

## License

MIT — see [LICENSE](LICENSE).
