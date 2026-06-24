# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- round 365: **top-level `video_sequence()` decode loop with §6.1.1.11
  display-order frame reordering** — the crate's #1 open gap, the driver
  *above* the per-picture reconstructors. New `video_sequence` module
  (`decode_video_sequence(stream) -> Vec<DecodedFrame>`, `DecodedFrame`,
  both re-exported at the crate root). It parses the §6.2.2.1
  `sequence_header()` + §6.2.2.3 `sequence_extension()` once for the
  geometry (`Mpeg2Sequence`), then walks every `picture_start_code`
  (`0x00000100`): for each it parses the §6.2.3 `picture_header()` +
  §6.2.3.1 `picture_coding_extension()` (`Mpeg2PictureHeader::`
  `parse_with_extension`), overlays the per-picture §6.2.3.1 DCT-context
  flags onto the sequence geometry, and dispatches the picture region to
  the matching per-picture driver — **I** → `decode_intra_picture`,
  frame-picture **P / B** → `decode_inter_picture` — supplying the
  reference frame(s) from the running §7.6 anchor pair (`forward_anchor`
  = previous I/P, `backward_anchor` = latest I/P; a P reads the latest, a
  B reads both). The reconstructed frames are reordered from coded order
  into **display order** per §6.1.1.11: a B-frame emits immediately; an
  I/P frame displays the previously held-back anchor (held one back) and
  becomes the new held anchor; the final anchor is flushed at end of
  stream — exactly the §6.1.1.11 worked-example mapping
  (`1I 4P 2B 3B 7P 5B 6B` → `1I 2B 3B 4P 5B 6B`). The picture-region
  scan includes the boundary start code's bytes so the per-picture slice
  walkers find their §5.2.3 23-zero terminator. Frame pictures only:
  a field-picture structure (`picture_structure != Frame`) surfaces
  `Error::NotImplemented`; a P/B before its anchor exists is rejected as
  `InvalidBitstream` (§6.1.1.11 *"the first coded frame after a sequence
  header shall not be a B-frame"*). 5 new `video_sequence` unit tests
  (the §6.1.1.11 reorder truth table incl. the worked example, the
  no-B-frame pass-through, the missing-sequence-header + boundary-scan
  guards) + 5 new `tests/video_sequence_decode.rs` integration tests that
  decode a full hand-built I/P/B elementary stream end-to-end (display
  order + reference-based prediction: the P copies the I anchor, the B
  averages I and P) plus the real 352×240 fixture's single-I-picture
  sequence. Spec: ISO/IEC 13818-2 §6.1.1.11 / §6.2.2 / §6.2.3 / §7.6.
  The loop is **sequence-aware** (§6.1.1.6): it re-reads the geometry at
  every repeat / new `sequence_header()` encountered before a picture
  (`find_picture_or_sequence_start_code` dispatches a sequence header to
  a geometry re-parse, a picture to reconstruction), so a multi-GOP /
  multi-sequence stream tracks geometry changes mid-stream while the
  §6.1.1.11 reorder + §7.6 anchors carry across the repeat header (the
  next coded I/P anchor flushes the held one normally). New
  `tests/video_sequence_decode.rs` test decoding two I-pictures separated
  by a repeat `sequence_header()` + `sequence_extension()`.

- round 359: **dual-prime motion compensation** (§7.6.3.6 / §7.6.7.4,
  Table 7-13 / 7-14 `Dual prime` rows) — driven end-to-end for **both**
  picture structures, plus the **field-picture B-field skipped-macroblock**
  §7.6.6.3 direction inheritance. The §7.6.3.6 opposite-parity vector
  derivation (`dual_prime::derive_all`, Tables 7-12 `m` / 7-13 `e` + the
  inline `dmvector[0..1]`) feeds two new
  `inter_reconstruction` MC drivers:
  - `reconstruct_field_picture_dual_prime_macroblock` (+
    `predict_field_picture_dual_prime_macroblock_planes`,
    `FieldPictureDualPrimeMotion`) — forms a same-parity prediction from
    the decoded `vector'[0][0]` and an opposite-parity prediction from the
    derived `vector'[2][0]`, averaged per §7.6.7.4 `// 2`; full-extent
    field-picture chroma.
  - `reconstruct_frame_dual_prime_macroblock` (+
    `predict_frame_dual_prime_macroblock_planes`, `FrameDualPrimeMotion`)
    — forms the four field predictions (top field from top ref `vector'[0]`
    + bottom ref `vector'[2]`; bottom field from bottom ref `vector'[0]` +
    top ref `vector'[3]`), averages each field, and interleaves the two
    into the frame at stride 2; `top_field_first` selects the Table 7-12
    frame row.

  `picture_reconstruction::PicturePredictionParams` gains a
  `top_field_first` field (+ `with_top_field_first`); `decode_inter_picture`
  / `decode_field_picture` now dispatch a `DualPrime` macroblock to the new
  drivers (forward-only P-pictures), with
  `frame_dual_prime_motion_from_reconstructed` /
  `field_picture_dual_prime_motion_from_reconstructed` building the motion
  from the reconstructed + wire records. `reconstruct_skipped_field_macroblock`
  now reconstructs a §7.6.6.3 B-field skip by inheriting the previous coded
  macroblock's direction + vectors and forcing the same-parity reference
  field (state threaded per slice). 16×8-MC stays field-picture-only per the
  §7.6 *"16x8 motion compensation shall only be used with field pictures"*
  constraint — there is no frame-picture 16×8 path. 8 new
  `inter_reconstruction` unit tests (field/frame same-/opposite-parity
  averaging, distinct-opposite-vector field divergence, geometry errors,
  residual add) + 3 new integration tests (`tests/field_picture_decode.rs`
  field-picture dual-prime + B-field skip inheritance;
  `tests/inter_picture_decode.rs` frame-picture dual-prime four-field
  interleave) decode hand-built `Dual prime` slices through the full §6.2.5
  parse + §7.6.3 + §7.6.4 pipeline. All §7.6 prediction modes are now driven
  end-to-end.

- round 358: **field-picture 16×8 motion compensation** (§7.6.7.3,
  Table 7-13 `16x8 MC` rows) — the macroblock-level prediction /
  reconstruction primitive. New
  `inter_reconstruction::reconstruct_field_picture_16x8_macroblock` (+
  `predict_field_picture_16x8_macroblock_planes`,
  `FieldPicture16x8Motion`): 16×8 MC forms two separate predictions per
  macroblock — `vector'[0]` predicts the upper 16×8 luminance region,
  `vector'[1]` the lower — each carrying its own §6.3.17.2
  `motion_vertical_field_select` flag so each region reads from its own
  chosen reference field (§7.6.4 NOTE). Chroma regions are the full
  component width × half its height per §7.6.7.3 (4:2:0 → 8×4, 4:2:2 →
  8×8, 4:4:4 → 16×8); §7.6.3.7 chroma scaling, §7.6.7.2 `// 2`
  bidirectional average, and §6.1.3 contiguous field-plane write-out
  (no frame/field DCT distinction inside a field picture). 9 new unit
  tests (region/field independence, distinct per-region vectors, chroma
  region split, bidirectional average, geometry/reference errors,
  residual add). **Driven end-to-end**: `picture_reconstruction::`
  `decode_field_picture` now dispatches a `SixteenByEight`
  field-picture macroblock to the new endpoint, building the
  `[upper, lower]` region pair from the two reconstructed §7.6.3 vectors
  and their per-entry `motion_vertical_field_select` flags
  (`field_picture_16x8_motion_from_reconstructed`); only field-picture
  dual-prime stays `UnsupportedPredictionMode`. 2 new integration tests
  in `tests/field_picture_decode.rs` (independent-field regions; a
  distinct half-pel lower-region vector) decode a hand-built `16x8 MC`
  field-picture slice through the full §6.2.5 parse + §7.6.3 + §7.6.4
  pipeline.

- round 351: **field-picture simple field prediction** (§7.6.1 *"within
  a field picture all predictions are field predictions"*, Table 7-13
  `Field-based` rows) — driven end-to-end. New
  `inter_reconstruction::reconstruct_field_picture_macroblock` (+
  `predict_field_picture_macroblock_planes`, `FieldPictureMotion`):
  a field-picture macroblock is a single 16×16 field block read from
  **one** reference field selected by the §6.3.17.2
  `motion_vertical_field_select` flag (Top when `0`, Bottom when `1`,
  §7.6.4) via the §7.6.4 `FieldReference` view; per-direction
  `(luma vector, FieldParity)`, §7.6.3.7 chroma scaling, §7.6.7.2 `// 2`
  bidirectional average, and §6.1.3 contiguous field-plane write-out
  (no frame/field DCT distinction inside a field picture). New top-level
  `picture_reconstruction::decode_field_picture` driver walks a field
  picture's slices with `PictureStructure::TopField` / `BottomField`
  (selecting `field_motion_type` + the field-select bit through the
  §6.2.5 parse), reconstructs the §7.6.3 vectors, pairs each with its
  field-select flag (`field_picture_motion_from_reconstructed`), and
  reconstructs Field-based macroblocks + §7.6.6.2 P-field skips (a
  `(0,0)` same-parity-field copy); 16×8-MC / dual-prime / B-field skip
  inheritance stay `UnsupportedPredictionMode`. 6 new unit tests +
  5 new `tests/field_picture_decode.rs` fixtures decoding synthetic
  field-picture slices end-to-end (parity selection, bottom-field
  select reading odd reference rows, bottom field picture, half-pel
  field-line average, 3-MB picture with skip). Spec: ISO/IEC 13818-2
  §7.6.1 / §7.6.3 / §7.6.4 / §7.6.5 (Table 7-13) / §7.6.6.2 / §6.1.3.
- round 346: **frame-picture field-based prediction** (Table 7-14
  `Field-based` rows) — the next interlaced-decode milestone after the
  frame-based P/B driver. New `forming_predictions::FieldReference`: a
  half-height field view over a frame-organised `ReferencePlane` (field
  line `k` → frame row `2k + parity`) with `predict_field_sample` /
  `predict_field_block` running the unmodified §7.6.4 half-pel
  interpolation against a single reference field — vertical pad-to-edge
  stays inside the field's own lines. New
  `inter_reconstruction::reconstruct_field_based_macroblock` (+
  `predict_field_based_macroblock_planes`, `FieldBasedMotion`): the
  top-field luminance vector predicts the macroblock's even (top-field)
  frame lines from the top reference field, the bottom-field vector its
  odd lines from the bottom field, the two directions combine per
  §7.6.7.2, chroma uses the per-field §7.6.3.7-scaled vectors, and the
  per-field prediction plane is returned in frame-row order so the
  residual-add / §6.1.3 block placement reuse the frame-based write-out
  path (4:2:0 chroma field-splits its 8 lines into 4+4). `decode_inter_picture`
  now drives field-based macroblocks (the §7.6.3 top/bottom field vector
  pair via `field_based_motion_from_reconstructed`) instead of rejecting
  them; 16×8-MC and dual-prime stay `UnsupportedPredictionMode`. 13 new
  unit tests + 2 end-to-end synthetic field-based P-picture integration
  tests.
- round 343: picture-level **P/B reconstruction driver** — the §7.6
  motion-compensated reconstruction wired end-to-end so a P- or
  B-picture reconstructs to real pixels. New `inter_reconstruction`
  module: `reconstruct_inter_macroblock` forms a frame-based
  macroblock's per-component prediction plane (§7.6.4 pel reader over
  the reference `FrameBuffer`, §7.6.3.7 chroma scaling, §7.6.7.1 `// 2`
  bidirectional average), adds the §A IDCT residual for each coded
  block (§6.3.17.4 `pattern_code[]`), and writes the §7.6.8
  `d = saturate(f + p)` result into the frame with the §6.1.3
  frame/field DCT line organisation. New `picture_reconstruction`
  module: `decode_inter_picture` scans each slice, walks it with block
  decoding enabled, reconstructs its motion vectors
  (`reconstruct_slice_motion_vectors`), and dispatches each macroblock
  to the intra placement or the inter MC driver, handling §7.6.6
  skipped macroblocks (P `(0,0)` forward, B inherited direction). The
  MPEG-1 (ISO/IEC 11172-2) reconstructed `recon_right`/`recon_down`
  vectors bridge into the same MC core (`MotionVectorPel::from_mpeg1`,
  `FrameMotion::from_mpeg1`). `slice_macroblock_walk` now resolves the
  §6.3.17.1 / Table 6-19 effective prediction type before the §7.6.3.3
  `update_predictors` call so an absent `frame_motion_type` tail
  (`frame_pred_frame_dct == 1`) no longer trips the update guard.
  `frame_assembly::Plane::put_sample` is now public. 20 new unit tests
  + 4 end-to-end synthetic P/B picture-decode integration tests.
- round 336: §7.7.3.1 / Table 7-15 upsampling-case dispatch in
  `spatial_resampling` — `UpsampleCase::select(field_select,
  lower_layer_progressive, progressive_frame)` resolves the five Table
  7-15 rows (interlaced→progressive top/bottom field, progressive frame
  ×2, interlaced→interlaced both fields) including the two *"shall have
  the value '1'"* `lower_layer_deinterlaced_field_select` constraints
  (rejected as `InvalidBitstream`), and `upsample_spatial_prediction`
  composes the existing §7.7.3.4 `deinterlace` → §7.7.3.5/.6
  `resample_progressive` → §7.7.3.7 `reinterlace` stages per the selected
  row to form `spat_pred_pic` for one component. This wires the
  previously-standalone spatial-resampling primitives into a single
  flag-driven entry point feeding the §7.7.4 spatial/temporal combiner.
  12 new unit tests (the full Table 7-15 truth table, both constraint
  rejections, and per-case driver-composition equivalence).
- round 331: §7.7.3.4 deinterlace + §7.7.3.7 reinterlace filters in
  `spatial_resampling`, completing the interlaced front/back of the
  §7.7.3 spatial-prediction pipeline that brackets the existing
  §7.7.3.5/.6 resampling. `deinterlace` builds the progressive `prog_pic`
  from an interlaced lower-layer reconstructed frame (top field = even
  rows, bottom field = odd rows) by zero-padding each field onto a
  field-rate progressive grid and applying the Table 7-19 vertical /
  temporal FIR: the two-field aperture (separate first-/second-field tap
  sets, temporal span `{-1, 0, +1}` reading the opposite field for the
  `±1` taps) for luminance in a Frame-Picture, and the one-field aperture
  (temporal `0` only) for chrominance and field-picture luminance. The
  `sum` is `sum // 16`-scaled — using a new signed `//` helper since the
  Table 7-19 `-1` / `-2` taps make `sum` possibly negative — and
  saturated to `[0, 255]`, with the §7.7.3.4 same-field
  nearest-neighbour border extension for taps outside `[0, ll_v_size)`.
  `reinterlace` forms `spat_pred_pic` from `hor_pic`: a straight copy for
  a progressive lower layer, or the field-select demultiplex (even lines
  for a top field, odd lines for a bottom field) for an interlaced one.
  New public surface: `deinterlace`, `reinterlace`, `Field`
  (re-exported as `ResampleField`).
- round 325: §7.7.3.5 / §7.7.3.6 spatial-scalable lower-layer resampling
  in a new `spatial_resampling` module — the linear-interpolation
  upsampling that takes a progressive lower-layer frame (`prog_pic`) and
  resamples it onto the enhancement-layer sample grid, producing the
  `pel_pred_spat` input the already-landed §7.7.4 combiner consumes.
  `vertical_resample` implements §7.7.3.5 (`vert_pic[yh+ll_v_offset][x] =
  (16-phase)*prog_pic[y1][x] + phase*prog_pic[y2][x]`, deferring its
  normalisation so it carries the ×16 scale); `horizontal_resample`
  implements §7.7.3.6 with the single `// 256` that folds both stages'
  ×16 scaling; `resample_progressive` composes the two for the
  progressive-to-progressive case (Table 7-15 row 3, no §7.7.3.4
  deinterlace / §7.7.3.7 reinterlace), where `hor_pic` is `spat_pred_pic`
  directly. The phase / `y1` / `y2` / `x1` / `x2` index math uses the
  §4.1 `/` (truncate toward zero) and `//` (round half away from zero,
  here non-negative so `(s + d/2) / d`) operators; out-of-frame reads use
  border extension (pad-to-edge clamp). `ResampleParams::luminance` /
  `ResampleParams::chrominance` derive the Table 7-16 local variables
  (`ll_*_size`, `ll_*_offset`, the four `*_subs_*` factors) from the raw
  `sequence_scalable_extension()` /
  `picture_spatial_scalable_extension()` fields, applying the Table 7-17
  `chroma_ratio` and Table 7-18 `format_ratio` adjustments for chroma. A
  `Plane` (`i32` row-major) carries the input / ×16 intermediate.
  Thirteen unit tests cover the 1:1 identity, vertical / horizontal /
  full 2× midpoint blends, border extension past the frame edge, the
  Table 7-16/7-17/7-18 chroma-ratio derivations (4:2:0→4:2:0 and
  4:2:0→4:4:4), the §4.1 `//` rounding examples, the unlisted-pair
  rejection, and the zero-size / zero-factor / mismatched-plane guards.
  All exported at the crate root (`ResamplePlane`, `ResampleParams`,
  `vertical_resample`, `horizontal_resample`, `resample_progressive`).
- round 320: §7.6.3 slice-level motion-vector reconstruction driver
  `reconstruct_slice_motion_vectors` in `slice_macroblock_walk` — the
  "walker → PMV state" wiring that composes the per-record
  `reconstruct_record_motion_vectors` endpoint into a full per-slice
  pass. It carries the §7.6.3 predictor bank (`PMV[r][s][t]`) across
  every macroblock of a parsed `SliceWalk`: §7.6.3.4 reset at slice
  start, §7.6.3.1 reconstruction per coded macroblock (accumulating the
  differential vectors across MBs), the §7.6.3.3 `update_predictors`
  table row after each one, and the §7.6.6 skipped-macroblock PMV
  side-effect (a P-picture reset, a B-picture no-op) for the
  `address_increment - 1` skipped slots that precede each coded MB. New
  `SliceMotionRecord` (per-coded-MB log: skipped-run count, whether the
  skip reset PMV, the reconstructed vectors, and the §7.6.3.3 update
  outcome) and `SliceMotionWalk` (the record list plus the final
  running `Pmv`), both re-exported at the crate root. Four unit tests
  cover the +1 → +2 cross-MB accumulation with the `NonIntraCopyForward`
  update row, the per-call §7.6.3.4 slice-start reset, the P-picture
  skipped-MB reset that breaks accumulation, and the §7.6.6 rejection of
  a skip in a non-scalable I-picture.
- round 315: §7.7.4 "Selection and combination of spatial and temporal
  predictions" — the *"precise method for predictor calculation"* in a
  new `spatial_temporal_combine` module that blends the temporal
  enhancement-layer prediction (`pel_pred_temp`) with the spatial
  lower-layer prediction (`pel_pred_spat`) under the Table 7-21
  `spatial_temporal_weight`. New `SpatialWeight` enum
  (`Temporal`/`Half`/`Spatial` for the only legal weights `{0, 0.5, 1}`)
  with `from_sixteenths()` mapping the `SpatialTemporalWeight`
  sixteenths (`0`/`8`/`16`) and `combine_sample()` implementing the
  page-115 per-sample formulae (`weight 0` → temporal,
  `weight 1` → spatial, `weight 0.5` → `(temp+spat)//2` with the §4.1
  away-from-zero rounding, identical to the §7.6.7 `avg2`). Block-level
  endpoints (all re-exported at the crate root): `combine_uniform` (the
  single `(a)` whole-block form, `table_index == '00'`),
  `combine_field_interleaved` (the per-field `(a; b)` form — `top_weight`
  to even rows / `bottom_weight` to odd rows, also the
  `progressive_frame == 0` interlaced-chroma case), and the
  `combine_spatial_temporal` driver keyed off
  `SpatialTemporalWeight::is_single`. Length / width / multiple-of-width
  geometry mismatches and out-of-table weights are rejected as
  `InvalidBitstream`. 21 unit tests incl. an exhaustive 256×256
  half-weight cross-check and Table-7-21-row reconstructions.
- round 310: §7.6.5 "Motion vector selection" (Tables 7-13 field
  pictures / 7-14 frame pictures) in a new `motion_vector_selection`
  module — the table driver that sits between §7.6.3 motion-vector
  reconstruction and the §7.6.4 pel reader, naming *which*
  reconstructed `vector'[r][s]` each prediction uses, *which reference*
  it is formed from, and *which region* of the 16×16 macroblock it
  covers. `select_predictions(&MacroblockPrediction) ->
  Result<Vec<PredictionOp>>` returns the ordered op list in bitstream
  order (the table NOTE) keyed off `picture_structure` +
  `prediction_type` (`field_motion_type`/`frame_motion_type`) + the
  three §6.3.17.1 flags. New types (all re-exported at the crate root):
  `PredictionOp` (`vector_index` r / `direction` s / `reference` /
  `region`), `ReferenceTarget` (`Frame` / `WholeField(parity)` /
  `DualPrimeSameParity(parity)` / `DualPrimeOppositeParity(parity)` —
  the latter two name the §7.6.3.6 derived vectors `vector'[2][0]` /
  `vector'[3][0]`), `PredictionRegion` (`Whole` 16×16 / `Upper16x8` /
  `Lower16x8`, with `luma_block_size()` → `BlockSize` and
  `luma_top_offset()` → 0/8), and `MacroblockPrediction` (the parsed
  inputs incl. the two `motion_vertical_field_select` parities §7.6.4
  needs in field pictures). Covers every Table 7-13 / 7-14 row:
  Frame-based (frame), Field-based (frame: first vector → top field,
  second → bottom; field: whole field from the selected parity), 16x8
  MC (field: upper/lower 16×8 with independent field selects),
  Dual-Prime (field: 2 same/opposite-parity ops; frame: 4 ops), and
  the §7.6.3.9 intra-concealment single-forward op. Rejects (via
  `Error::InvalidBitstream` with §7.6.5-named messages) the three
  malformed-descriptor cases: intra without
  `concealment_motion_vectors`, dual-prime with backward motion, and
  the no-motion-flags §7.6.3.5 implicit-zero (skipped) case that
  belongs to `skipped_macroblock`. Forms no samples, reconstructs no
  vectors, runs no §7.6.3.6 dual-prime derivation, and applies no
  §7.6.3.7 chroma scaling — it is the pure table glue between the
  already-landed endpoints. 18 new bit-exact unit tests (890 lib
  total, was 872): the region geometry pair, the Table 7-14 frame rows
  (Frame-based fwd/bwd/bidirectional, Field-based top-then-bottom and
  the 4-op bidirectional shape), the Table 7-13 field rows
  (Field-based field-select, 16x8 independent selects + 4-op
  bidirectional order), all three dual-prime shapes (field top/bottom,
  frame 4-op), the two intra-concealment arms (frame/field), and the
  three rejection paths.
- round 308: §7.7.5.1 "Resetting motion vector predictors" — the
  spatial-scalability extension to the §7.6.3.4 PMV reset rules, as a
  new `pmv::apply_spatial_temporal_reset(pmv, picture_coding_type,
  spatial_temporal_weight_class)` (re-exported at the crate root). The
  spec adds two reset cases on top of §7.6.3.4: a P-picture or
  B-picture macroblock that is purely spatially predicted
  (`spatial_temporal_weight_class == 4`, signalled by the scalable
  `macroblock_type` Tables B-5/B-6/B-7) carries no motion vector, so
  the running PMV state must be zeroed exactly as `Pmv::reset` does.
  The helper returns `true` when the reset fired (P/B + class 4),
  `false` otherwise (leaving the PMV untouched), so a macroblock-loop
  driver can label the side-effect. Intra pictures (not listed by
  §7.7.5.1) and classes `0..=3` (temporal-only or combined) take the
  no-reset path. 5 new bit-exact unit tests (872 lib total, was 867):
  the P/B class-4 reset, the intra-picture skip, the classes-0..3
  no-reset sweep across P and B, and an out-of-range-class guard.
- round 301: §6.2.5.1 `spatial_temporal_weight_code` read +
  Table 7-21 (§7.7.4) resolution wired into
  `macroblock_modes::MacroblockModesTail::parse`. The former
  rejection of `mb_type`s carrying
  `spatial_temporal_weight_code_flag == true` (set by the r294
  scalable B-5/B-6/B-7 tables) is replaced by the actual 2-bit code
  read, gated per §6.2.5.1 on `spatial_temporal_weight_code_flag == 1
  && spatial_temporal_weight_code_table_index != '00'`. New
  `SpatialTemporalWeight` type (re-exported at the crate root) carries
  the resolved `weight_class` (`1`/`2`/`3`), the per-field
  `spatial_temporal_weight(s)` pair (in sixteenths: `0`/`8`/`16`),
  the `is_single` `(a)`-vs-`(a; b)` shape, and the
  `spatial_temporal_integer_weight` flag — all transcribed from the
  Table 7-21 grid. The resolved class now drives the §6.3.17.2
  Field-based motion-vector-count split (Table 6-17) instead of the
  always-`0` context default. `MacroblockModesContext` grows
  `spatial_temporal_weight_code_table_index` (from
  `picture_spatial_scalable_extension()`, §6.3.14) with a new
  `MacroblockModesContext::scalable` constructor; the existing `new`
  seeds it to `0` so non-scalable callers are byte-identical.
  `MacroblockModesTail` grows `spatial_temporal_weight:
  Option<SpatialTemporalWeight>`. 8 new unit tests (867 lib total):
  the full Table 7-21 grid + out-of-range index, the code-read +
  class-resolution path (classes 1 and 3 driving the 2-vs-1 vector
  split), the `table_index == 00` no-code `00*` row, the flag-clear
  no-read path, truncation, and the scalable-context seeding.
  Empirical note: §6.3.17.1 cites "Table 7-20" for the class
  derivation, but Table 7-20 has no class column; §7.7.4
  authoritatively derives the class from Table 7-21 (used here).
- round 294: Annex B scalable `macroblock_type` tables — B-5
  (I-pictures, spatial scalability), B-6 (P-pictures, spatial
  scalability), B-7 (B-pictures, spatial scalability) and B-8 (I/P/B,
  SNR scalability) — added to the `macroblock_type` module alongside
  the existing non-scalable B-2/B-3/B-4 tables. The `Row` type and
  `MacroblockType` now carry the two extra §6.3.17.1 columns:
  `spatial_temporal_weight_code_flag` (now set per-row, no longer a
  hard `false`) and `spatial_temporal_weight_class`
  (`Option<u8>` — `Some(0)` / `Some(4)` for resolved classes,
  `None` when the flag is set and the class is one of `{1,2,3}` to be
  derived later from `spatial_temporal_weight_code` via Table 7-21).
  New `MacroblockTypeTable` enum + `MacroblockTypeTable::select`
  derive the table family from `scalable_mode` + picture type + the
  `picture_spatial_scalable_extension()`-present flag per Table 6-10
  (spatial scalability falls back to the non-scalable tables when the
  current picture lacks the spatial-scalable extension; data
  partitioning and temporal scalability always use B-2/B-3/B-4; SNR
  uses B-8). `MacroblockType::parse` keeps the non-scalable default;
  `MacroblockType::parse_with_table` takes an explicit family. The
  longest-first VLC walk now spans 1..=9 bits (B-7's deepest
  codewords). 11 new unit tests (861 lib total): every B-5/B-7 row,
  the B-6 compatible-vs-class-4 split, all three B-8 codewords across
  every picture type, the Table 6-10 selector, prefix-freeness of all
  four new tables, and parse/parse_with_table equivalence.

- round 291: §6.2.3.5 `picture_spatial_scalable_extension()` parser
  (field semantics §6.3.14) in a new
  `picture_spatial_scalable_extension` module and §6.2.3.4
  `picture_temporal_scalable_extension()` parser (field semantics
  §6.3.13) in a new `picture_temporal_scalable_extension` module —
  the last two of the four extensions the r279
  `extension_and_user_data(i)` dispatcher surfaced as
  `Error::NotImplemented`, closing that surface entirely.
  `PictureSpatialScalableExtension` carries
  `lower_layer_temporal_reference`, the two 15-bit `simsbf`
  `lower_layer_horizontal_offset` / `lower_layer_vertical_offset`
  (read via `read_i32`, full `[-16384, 16383]` range),
  `spatial_temporal_weight_code_table_index` (§7.7 Table 7-20/7-21),
  `lower_layer_progressive_frame`, and
  `lower_layer_deinterlaced_field_select`; its `validate(chroma_format)`
  helper enforces the §6.3.14 even-offset rules (horizontal even for
  4:2:0/4:2:2, vertical even for 4:2:0) the bare wire parse cannot
  see. `PictureTemporalScalableExtension` carries
  `reference_select_code`, `forward_temporal_reference`, and
  `backward_temporal_reference`; its `validate(picture_coding_type)`
  helper enforces the §7.9 / Table 7-28 / Table 7-29
  `reference_select_code` constraints (I-pictures shall be `'11'`,
  `'11'` forbidden in P-pictures, `'00'` forbidden in B-pictures).
  Both are wired into the r279 dispatcher's `i = 2` allowable set
  (new `ExtensionAndUserData` fields
  `picture_spatial_scalable_extension` /
  `picture_temporal_scalable_extension`), so no
  `extension_and_user_data(i)` path returns `Error::NotImplemented`
  any longer. Both `marker_bit` / start-code / identifier rejection
  sites are covered. 29 new bit-exact unit tests.
- round 283: §6.2.2.5 `sequence_scalable_extension()` parser (field
  semantics §6.3.7) in a new `sequence_scalable_extension` module and
  §6.2.3.6 `copyright_extension()` parser (field semantics §6.3.15)
  in a new `copyright_extension` module — the first two of the four
  extensions the r279 `extension_and_user_data(i)` dispatcher
  surfaced as `Error::NotImplemented`. New types:
  `SequenceScalableExtension` (`scalable_mode` + `layer_id`),
  `ScalableMode` (Table 6-10 — all four 2-bit codes are defined, no
  reserved row; each scalability type carries its §6.2.2.5
  mode-conditional parameter block in the variant),
  `SpatialScalabilityParams` (the two 14-bit lower-layer prediction
  sizes + the four 5-bit §7.7.2 subsampling factors, *"the value
  zero is forbidden"* enforced on each), `TemporalScalabilityParams`
  (`picture_mux_enable`, the conditionally-present
  `mux_to_progressive_sequence`, `picture_mux_order`,
  `picture_mux_factor` with the §6.3.7 reserved `'000'` rejected
  when `picture_mux_enable` is set — the only case the field is
  used — and preserved raw otherwise), and `CopyrightExtension`
  (`copyright_flag` / `copyright_identifier` / `original_or_copy` +
  the three number fields and the §6.3.15 64-bit
  `copyright_number()` derivation `(n1 << 44) + (n2 << 22) + n3`).
  Enforced rejection sites beyond the wire shape: the §6.1 / §6.3.7
  `layer_id` pins (data partitioning ⇒ partition 0 or 1; spatial /
  SNR / temporal ⇒ at least 1, because the base layer carries no
  `sequence_scalable_extension()` outside data partitioning), the
  spatial-block marker bit, the §6.3.15 *"shall have the value
  zero"* 7-bit `reserved` field, the three copyright marker bits,
  and the §6.3.15 clear-flag (`copyright_identifier` and
  `copyright_number` shall be 0) and zero-identifier
  (`copyright_number` shall be 0) constraints. Both parsers expose
  the crate-standard `parse` / `parse_with_reader` pair and are
  wired into the r279 dispatcher's `i = 0` / `i = 2` allowable sets
  via two new `ExtensionAndUserData` fields
  (`sequence_scalable_extension`, `copyright_extension`); the
  `Error::NotImplemented` surface shrinks to
  `picture_spatial_scalable_extension()` (§6.2.3.5) and
  `picture_temporal_scalable_extension()` (§6.2.3.4). The
  `SEQUENCE_SCALABLE_EXTENSION_ID` / `COPYRIGHT_EXTENSION_ID`
  constants moved into the new modules (still re-exported at the
  crate root, along with all new types). Follow-ups: the §6.1.1.6 /
  §6.3.7 cross-sequence occurrence rule (*"a bitstream is either
  scalable or it is not scalable"*; all data elements equal across
  repeat sequence headers) needs a
  `SequenceDisplayOrderDriver`-shaped sequence-layer driver, and the
  two picture scalable extension parsers plus the top-level §6.2.2
  `video_sequence()` walker remain future rounds. 34 new unit tests
  (936 total, was 902): the sequence-scalable wire surface across
  all four modes with per-mode encoded-length accounting and seven
  rejection sites (16), the copyright wire surface with the
  concatenation identity and seven rejection sites (13), and the
  dispatcher integration — positive `i = 0` / `i = 2` parses, a
  both-sequence-extensions window, the two wrong-location
  rejections, and the two remaining `NotImplemented` stubs (5 net).
- round 279: §6.2.2.2 `extension_and_user_data(i)` dispatcher plus
  §6.2.2.2.1 `extension_data(i)` and §6.2.2.2.2 `user_data()` in a new
  `extension_and_user_data` module — the §6.2.2 `video_sequence()`
  element invoked at `i = 0` (after `sequence_extension()`), `i = 1`
  (after `group_of_pictures_header()`), and `i = 2` (after
  `picture_coding_extension()`), tying the r261 / r241 / r244
  `sequence_display_extension()` / `quant_matrix_extension()` /
  `picture_display_extension()` parsers into one §5.2.3 start-code
  loop. New types: `ExtensionLocation` (the `i` argument; the `i = 2`
  arm carries `chroma_format` + `PictureDisplayContext`), `UserData`
  (the §6.3.4.1 byte series, terminated by *"receipt of another start
  code"* with the *"no string of 23 or more consecutive zero bits"*
  emulation guard enforced), and `ExtensionAndUserData` (the parsed
  optionals + `user_data` vec + `discarded_reserved_ids` +
  `byte_position_after`, the offset of the foreign start code that
  ended the loop). Enforced rejection sites: §6.2.2.2.1 NOTE
  (`extension_data()` never follows a `group_of_pictures_header()`),
  §6.3.1 at-most-once per extension type, §6.3.1 allowable-set
  (defined ID at the wrong invocation point), §5.2.3 non-zero
  stuffing bytes between elements and non-zero stuffing bits after an
  unaligned extension, and truncation at every lookahead. The §6.3.1
  reserved-identifier rule (*"discard all subsequent data until the
  next start code"*) is a recorded skip, not an error. The four
  spec-defined extensions without crate parsers yet
  (`sequence_scalable_extension()`, `copyright_extension()`,
  `picture_spatial_scalable_extension()`,
  `picture_temporal_scalable_extension()`) surface
  `Error::NotImplemented` — the variant's first user. New constants
  `USER_DATA_START_CODE`, `COPYRIGHT_EXTENSION_ID`,
  `SEQUENCE_SCALABLE_EXTENSION_ID`,
  `PICTURE_SPATIAL_SCALABLE_EXTENSION_ID`,
  `PICTURE_TEMPORAL_SCALABLE_EXTENSION_ID`; all new types re-exported
  at the crate root. The `i = 0` result's
  `sequence_display_extension` field feeds the r271
  `SequenceDisplayOrderDriver::on_sequence_header_window` directly
  (pinned end-to-end by a test). 31 new unit tests: the `user_data()`
  surface (7), positive dispatch shapes across all three locations
  including zero-stuffing and reserved-discard paths (11), every
  rejection site (12), and the §6.3.5 / §6.3.12 driver hand-off (1).
  Follow-up parsers for the four `NotImplemented` extensions
  (§6.2.2.5 / §6.2.3.6 / §6.2.3.5 / §6.2.3.4) and the top-level
  §6.2.2 `video_sequence()` walker remain future rounds.
- round 271: §6.3.5 / §6.3.12 `SequenceDisplayOrderDriver` sequence-layer
  ordering driver in a new `sequence_display_order` module — the two
  cross-element occurrence constraints r261's
  `sequence_display_extension()` parser deferred to "sequence-layer
  driver work". The driver owns the running
  `sequence_display_extension()` presence/value fact across one MPEG-2
  sequence and exposes the two checks as named methods, mirroring the
  `FrameCentreOffsetDriver` / `QuantMatrixDriver` shape.
  `on_sequence_header_window(Option<SequenceDisplayExtension>)` observes
  each `sequence_header()`-to-`picture_header()` window: the first call
  pins the §6.3.5 requirement (`Requirement::Forbidden` when absent,
  `Requirement::RequiredEqual(first)` when present), and every later
  call is checked against the pin per §6.3.5 *"all subsequent sequence
  headers shall be followed by `sequence_display_extension()` in which
  all data elements are the same as in the first … Conversely if no
  `sequence_display_extension()` occurs … then
  `sequence_display_extension()` shall not occur in the bitstream"* —
  rejecting a present-where-forbidden, absent-where-required, or
  differing-value repeat as `InvalidBitstream`.
  `check_picture_display_extension()` (and the bool accessor
  `picture_display_extension_permitted()`) answer the §6.3.12 gate
  *"a `picture_display_extension()` shall not occur unless a
  `sequence_display_extension()` followed the previous
  `sequence_header()`"* from the running presence fact. New types
  `SequenceDisplayOrderDriver` (`Copy + Default`; `Default` ↔ `new`,
  both at the pre-first-`sequence_header()` baseline) and `Requirement`
  (`Unpinned` / `Forbidden` / `RequiredEqual`) are re-exported at the
  crate root. The r261 doc-comment notes in `sequence_display_extension`
  and the `picture_display_extension` "Order constraint" note now point
  at the driver instead of flagging the rules as pending. 14 new unit
  tests cover the pre-window baseline, the two first-window pins, the
  Forbidden absent-ok / present-rejected pair, the RequiredEqual
  equal-ok / absent-rejected / differing-size-rejected /
  differing-colour-rejected quartet, the §6.3.12 gate before and after
  a present/absent window, and the two multi-window idempotency chains.
- round 261: §6.2.2.4 `sequence_display_extension()` parser plus the
  §6.3.6 field semantics in a new `sequence_display_extension` module
  — the first of the two sequence-layer elements the §6.3.12 ordering
  constraint (*"a `picture_display_extension()` shall not occur unless
  a `sequence_display_extension()` followed the previous
  `sequence_header()`"*) binds, flagged as unhandled by the r244
  order-constraint note the r260 `FrameCentreOffsetDriver` inherited.
  New types: `SequenceDisplayExtension` (the parsed element),
  `VideoFormat` (Table 6-6 — component / PAL / NTSC / SECAM / MAC /
  unspecified, with the reserved `110` / `111` codes preserved raw per
  the `AspectRatio::Reserved` policy since §6.3.6 says the field does
  not affect the decoding process; `Default` is pinned to
  `Unspecified` per the §6.3.6 absence rule), `ColourDescription`
  (the optional `colour_primaries` / `transfer_characteristics` /
  `matrix_coefficients` triple gated on the wire's
  `colour_description` flag, raw 8-bit components with the
  *"(forbidden)"* value `0` of each defining Table 6-7 / 6-8 / 6-9
  rejected at parse time and the reserved upper codes preserved), and
  the constant `SEQUENCE_DISPLAY_EXTENSION_ID = 0b0010` naming the
  Table 6-2 identifier. `ColourDescription::ASSUMED` exposes the
  §6.3.6 absence/flag-clear default (every component = the value-1
  row, Rec. ITU-R BT.709);
  `SequenceDisplayExtension::effective_colour_description()` applies
  that rule for parsed-but-flag-clear extensions. Parser enforces
  every §6.2.2.4 rejection site: 32-bit `extension_start_code`
  (`0x000001B5`), 4-bit `extension_start_code_identifier` (`0010`),
  the three forbidden-zero colour bytes, and the `marker_bit` between
  the two 14-bit `display_horizontal_size` / `display_vertical_size`
  fields (§6.3.6 units: samples / lines of the encoded frames). The
  §6.3.5 repeat-sequence-header occurrence constraint and the §6.3.12
  ordering gate remain sequence-layer driver work — the module
  doc-comment quotes both so the follow-up driver round can wire the
  presence fact into `FrameCentreOffsetDriver` without re-reading the
  spec. 18 new bit-exact unit tests cover the no-colour and
  full-colour positive parses, all six described Table 6-6 codes plus
  the two reserved codes, the 14-bit `0x3FFF` display-size
  round-trip, reserved colour codes above the described rows, the
  encoded bit-length accounting for both shapes (69 / 93 bits → 9 /
  12 padded bytes), the six rejection paths (wrong start code, wrong
  id, three forbidden-zero colour bytes, zero marker bit), two
  truncation points, and the three §6.3.6 absence-default rules.
- round 260: §6.3.12 `FrameCentreOffsetDriver` picture-level state
  machine that owns the running `FrameCentreOffsetState` across one
  MPEG-2 sequence and exposes the two §6.3.12 carry-over events as
  named methods, mirroring the round-254 `QuantMatrixDriver` shape so
  picture-driver authors stop spelling the
  `state.reset_to_zero()` / `state.apply(&ext)` dance themselves.
  `FrameCentreOffsetDriver::on_sequence_header()` invokes
  `FrameCentreOffsetState::reset_to_zero` per *"Following a
  `sequence_header()` the value zero shall be used for all frame
  centre offsets until a `picture_display_extension()` defines
  non-zero values"* (§6.3.12 rule 1).
  `FrameCentreOffsetDriver::on_picture_display_extension(ext)`
  composes a parsed `PictureDisplayExtension` onto the running state
  through the already-landed `FrameCentreOffsetState::apply`,
  adopting the first (transmission-order) `(horizontal, vertical)`
  pair as the new "most recently decoded frame centre offset" the
  §6.3.12 NOTE clarifies is sufficient even when two or three pairs
  are carried. `FrameCentreOffsetDriver::state()` returns a `Copy`
  snapshot a display-side caller can plumb downstream without
  borrowing the driver — pictures that omit the extension simply
  skip the second call and the carried snapshot threads forward per
  §6.3.12 rule 2 *"In the case that a given picture does not have a
  `picture_display_extension()` then the most recently decoded
  frame centre offset shall be used"*. Seven new unit tests cover
  the driver surface (`new` ↔ §6.3.12 zero baseline,
  `Default` ↔ `new`, sequence-header reset after an extension
  mutation, first-pair adoption matches the field-level
  `FrameCentreOffsetState::apply` byte-for-byte on a 3-pair payload,
  no-event picture carries the previous offset unchanged, two-cycle
  reset → apply → reset → apply idempotency, snapshot returned by
  value so local mutations leave the running state intact).
- round 254: §6.3.11 `QuantMatrixDriver` picture-level state machine
  that owns the running `QuantiserMatrixState` across one MPEG-2
  sequence and exposes the two §6.3.11 lifecycle events as named
  methods so picture-driver callers stop spelling the state-mutation
  dance themselves. `QuantMatrixDriver::on_sequence_header()` invokes
  `QuantiserMatrixState::reset_to_defaults` per *"When a
  `sequence_header_code` is decoded all matrices shall be reset to
  their default values"* (§6.3.11). `QuantMatrixDriver::on_quant_matrix_extension(
  ext, chroma_format)` composes a parsed `QuantMatrixExtension` onto
  the running state through the already-landed `QuantMatrixExtension::apply`,
  honouring the four-flag sequencing and 4:2:2 / 4:4:4
  chroma-follows-luma rule. `QuantMatrixDriver::state()` returns a
  `Copy` snapshot the slice-walker builder
  `SliceWalkContext::with_quantiser_matrices` consumes verbatim. Six
  new unit tests cover the driver surface (`new` ↔ §6.3.7 defaults,
  `Default` ↔ `new`, sequence-header reset after an extension
  mutation, extension shim matches the field-level API on a 4:4:4
  luma + chroma payload, two-cycle reset → apply → reset → apply
  idempotency, snapshot return-by-value). Two new integration tests
  prove the end-to-end driver → slice-walker arithmetic on the r251
  single-AC fixture: feeding a hand-built `quant_matrix_extension()`
  (intra-luma cells = 80) through the driver and dispatching the
  slice via `ctx.with_quantiser_matrices(driver.state())` yields
  `f_quant[0][1] == 140`, and a follow-up `driver.on_sequence_header()`
  brings the next slice's `f_quant[0][1]` back to the r251 baseline
  `28`.
- round 251: §6.3.11 `QuantiserMatrixState` wired through the §6.2.4
  slice walker into the §6.2.6 `block(i)` driver. New field
  `SliceWalkContext::quantiser_matrices: QuantiserMatrixState` carries
  the four §7.4.2.1 Table 7-5 `w`-indexed weighting matrices
  (`intra_luma`, `non_intra_luma`, `intra_chroma`, `non_intra_chroma`)
  per-slice; each existing constructor seeds the field to the §6.3.7
  defaults so prior callers see no behavioural change. New builder
  method `SliceWalkContext::with_quantiser_matrices` chains a parsed
  state onto any constructor so the picture-level driver pattern
  `state.reset_to_defaults(); ext.apply(&mut state, chroma);
  ctx.with_quantiser_matrices(state)` matches the §6.3.11 lifecycle
  spelling 1-to-1. Inside `walk_slice` the four matrices are unpacked
  into the `[[[u8; 8]; 8]; 4]` slot
  `MacroblockBlockContext::weight_matrices` expects, so the §7.4.2.3
  reconstruction step `F''[v][u] = (2*QF + k) * W * quantiser_scale /
  32` picks up the user-downloaded matrices verbatim rather than
  always reading the §6.3.7 defaults. The pre-r251 walker comment
  flagging this surfacing as a follow-up is resolved. Three new
  integration tests pin the wiring: a default-matrix baseline asserts
  `f_quant[0][1] == 28` against a single AC coefficient at zig-zag
  index 1 (`(run=0, level=+1)` Table B-14 NEXT-form), the same
  fixture with `intra_luma[0][1]` overridden to `80` asserts
  `f_quant[0][1] == 140` (a 5× change driven solely by the matrix
  override, confirming the new field reaches the §7.4.2.3 arithmetic),
  and a constructor sweep across `first_slice` / `first_slice_mpeg1`
  / `first_slice_with_block_decoding` confirms every entry-point
  defaults to the §6.3.7 matrices.
- round 244: §6.2.3.3 `picture_display_extension()` parser plus the
  §6.3.12 frame-centre offset state machine. New module
  `picture_display_extension` with `PictureDisplayExtension` (the
  fixed-capacity 1..=3 array of `(horizontal, vertical)` offset pairs),
  `FrameCentreOffset` (one signed-16 component pair),
  `PictureDisplayContext` (the four picture-layer flags §6.3.12 needs:
  `progressive_sequence`, `picture_structure`, `repeat_first_field`,
  `top_field_first`), the standalone
  `number_of_frame_centre_offsets(ctx)` helper transcribing the §6.3.12
  pseudocode 1-to-1, `FrameCentreOffsetState` carrying the §6.3.12
  "most recently decoded" pair with `reset_to_zero` (the
  post-`sequence_header()` reset) and `apply` (extension absorption,
  taking the first offset per the §6.3.12 NOTE), `FieldUsage` capturing
  the between-pictures vs. per-picture application context, and
  `PICTURE_DISPLAY_EXTENSION_ID = 0b0111` naming the Table 6-2
  identifier. Parser enforces every §6.2.3.3 rejection site: 32-bit
  `extension_start_code` (`0x000001B5`), 4-bit
  `extension_start_code_identifier` (`0111`), and the two `marker_bit`
  slots inside every loop iteration. The 16-bit signed (`simsbf`)
  offsets round-trip the full `[-32768, 32767]` range via
  `BitReader::read_i32(16)`. 22 new bit-exact unit tests cover the six
  §6.3.12 derivation arms, the 1 / 2 / 3 positive wire parses, the
  `i16::MIN` / `i16::MAX` boundary, the five rejection paths
  (wrong-start-code, wrong-id, horizontal-marker-zero,
  vertical-marker-zero, short-buffer), the encoded byte-length count
  for each loop arity (9 / 13 / 18 bytes — the count-1 / count-3 sizes
  are post-pad), and the three state-machine cases (zero baseline,
  extension absorption, reset).
- round 241: §6.2.3.2 `quant_matrix_extension()` parser plus the
  §6.3.11 per-sequence weighting-matrix state machine. New module
  `quant_matrix_extension` with `QuantMatrixExtension` (the four
  optional `intra` / `non_intra` / `chroma_intra` /
  `chroma_non_intra` payloads, each a `QuantiserMatrixPayload`),
  `QuantiserMatrixState` (the four §7.4.2.1 / Table 7-5
  `w`-indexed `[[u8; 8]; 8]` slots, initialised to the §6.3.7
  defaults — intra is the published Wf matrix shared with the
  MPEG-1 default, non-intra is the all-16 matrix), and
  `QUANT_MATRIX_EXTENSION_ID = 0b0011` (Table 6-2). Parser
  enforces every §6.3.11 rejection site: 32-bit
  `extension_start_code` (`0x000001B5`), 4-bit
  `extension_start_code_identifier` (`0011`),
  `chroma_format == 4:2:0 ⇒ load_chroma_*_quantiser_matrix ==
  '0'` (both flags), `intra_quantiser_matrix[0] == 8` (luma and
  chroma), and `value zero is forbidden` for every byte of every
  loaded payload. `QuantiserMatrixPayload::to_matrix` lifts the
  on-wire §7.3.1 default-zigzag-order bytes through
  `mpeg2_inverse_scan::inverse_scan_table(false)` into the
  row-major `W[v][u]` layout. `QuantMatrixExtension::apply`
  composes the four optionals against a
  `QuantiserMatrixState` per the §6.3.11 sequencing
  (luma-load first, optional chroma-load override second; the
  4:2:2 / 4:4:4 luma load copies into the chroma slot, while the
  4:2:0 luma load leaves chroma untouched).
  `QuantiserMatrixState::reset_to_defaults` performs the
  §6.3.11 `sequence_header_code` reset. 24 new bit-exact unit
  tests cover the positive parses (no-load / intra-only /
  non-intra-only / all-four), the 4:2:0 chroma-flag rejection
  pair, the byte-zero rejection pair, the intra first-byte-8
  rejection pair, the wrong-start-code / wrong-id / short-buffer
  rejection trio, the byte-alignment count for both empty and
  fully-loaded extensions, the §7.3.1 `to_matrix` round-trip, and
  the four `apply` sequencing cases (luma→chroma copy at 4:2:2 /
  4:4:4 vs. no copy at 4:2:0, chroma-load override on top of a
  same-extension luma-load, default reset). The walker's existing
  §6.3.7-default fallback in
  [`slice_macroblock_walk::walk_slice`] is now flagged with a
  forward pointer at the comment site so the next round can wire
  the state through `SliceWalkContext` without re-reading the
  spec.
- round 238: §7.6.3.1 PMV reconstruction wired into the §6.2.5
  `motion_vectors()` parser path. New
  `pmv::decode_motion_vector(pmv, r, s, t, motion_code,
  motion_residual, f_code)` — the brief-signature per-component
  entry point that wraps the round-12 `reconstruct_component`
  for the dominant non-vertical-half-pred case, threading the
  resulting `vector'[r][s][t]` back into `pmv[r][s][t]`. New
  `slice_macroblock_walk::reconstruct_record_motion_vectors(record,
  pmv, ctx)` — the wire-to-recon helper that reads a
  `MacroblockRecord`'s parsed `motion_vectors_forward` /
  `motion_vectors_backward`, runs `reconstruct_motion_vector`
  per `(r, s)` slot, and returns `ReconstructedMotionVectors`
  carrying per-`(r, s)` `ReconstructedVector` pairs (horizontal
  + vertical post-wrap `vector'[r][s][:]`). Honours the
  §6.3.17.1 / Table 6-19 absent-modes-tail default through the
  existing `effective_motion_type` derivation. Eleven new
  bit-exact tests cover the worked-example surface
  (zero-code, f_code-2 / f_code-3 residual paths with sign
  flip, wrap-around in both directions, residual presence
  gating, slot isolation) and the two-MB PMV accumulation
  across the slice walker.
- round 34: MPEG-2 §6.2.6 `block(i)` driver wired into the
  slice walker as an opt-in path. `slice_macroblock_walk::walk_slice`
  now calls [`crate::mpeg2_decode_macroblock_blocks`] for every
  coded block per the parsed `pattern_code[i]` whenever the new
  `SliceWalkContext::block_decoding_enabled` flag is `true`,
  chaining §7.2.1 DC prelude (intra blocks only), §7.2.2 residual
  VLC walker (B-14 / B-15 with §7.2.2.2 FIRST/NEXT alternation),
  §7.3 inverse scan, §7.4 inverse quantisation (Table 7-5
  weighting-matrix selection driven by `(BlockCoding, Component,
  ChromaFormat)`), and §A 8×8 IDCT into a single
  "bitstream → `f[y][x]`" pass per coded block. Maintains the
  §7.2.1 per-component DC predictor `dc_dct_pred[cc]` across
  macroblocks for the duration of the slice (allocated at slice
  start with the Table 7-2 reset value for the active
  `intra_dc_precision`; reset on every non-intra MB by the
  inner driver). `SliceWalkContext` grows five new fields —
  `intra_vlc_format` / `alternate_scan` / `intra_dc_precision`
  (with the Table 6-13 `0..=3` range enforced before the loop
  ever runs) / `q_scale_type` (driving the §7.4.2.2 Table 7-6
  `quantiser_scale_value` resolution from the carried-forward
  `quantiser_scale_code`) and `block_decoding_enabled`. All four
  existing constructors (`first_slice`,
  `first_slice_with_picture_extension`,
  `first_slice_with_picture_body`, `first_slice_mpeg1`) default
  the new fields to "block decoding off" (`false` /
  `intra_dc_precision = 0`) so the round-30..33 contract holds
  bit-identically — the walker stops at the
  `coded_block_pattern()` snapshot and emits no §6.2.6 reads. A
  new `first_slice_with_block_decoding` constructor surfaces the
  full §6.3.5 / §6.3.11 picture-extension state plus the four
  §6.2.6 fields and turns the gate on. `MacroblockRecord` gains
  `decoded_blocks: Option<Vec<DecodedBlock>>` — `None` in the
  wire-only path (round-30..33 contract), `Some(empty)` for a
  decoded MB with zero coded blocks (e.g. P-picture "MC, not
  coded"), or `Some([…])` with one entry per coded block
  (each carrying the full `QFS[]` / `QF[v][u]` / `F[v][u]` /
  `f[y][x]` reconstruction plus the post-EOB bit cursor). The
  §6.3.7 default weighting matrices are used —
  `quant_matrix_extension()` downloadable-matrix support is a
  separate follow-up clause. The §6.1.1.8 block ordering per
  chroma_format (4:2:0 = 6 blocks, 4:2:2 = 8, 4:4:4 = 12) is
  threaded straight through from
  `mpeg2_macroblock_blocks::block_count` /
  `mpeg2_macroblock_blocks::block_component`. The walker's
  `body_bit_position` snapshot keeps the round-30..33 meaning
  (cursor after `macroblock_modes()` + `quantiser_scale_code`,
  before any wire-body field) so external resume-from-cursor
  drivers stay unbroken; the new per-block `end_of_block_bit_position`
  exposed by [`crate::Mpeg2DecodedBlock::end_of_block_bit_position`]
  gives the post-§6.2.6 cursor on each emitted block. 7 new lib
  unit tests plus 3 new integration tests in
  `tests/slice_macroblock_walk_synthetic.rs` pin the wire-only
  vs block-decoding split, the 4:2:0 DC-only intra MB walk (6
  blocks, predictor reset to 128 with `intra_dc_precision == 0`),
  per-slice DC predictor advance across two intra MBs, the
  `intra_dc_precision == 4` pre-flight rejection, the
  linear-vs-non-linear `q_scale_type` lookup, and the
  `pattern_code` all-`false` empty-decoded-blocks path. The
  `DecodedBlock` types under `mpeg2_block_decoder` and
  `mpeg2_macroblock_blocks` now derive `PartialEq` / `Eq` so the
  enclosing `MacroblockRecord::decoded_blocks` keeps the
  walker's record-level `PartialEq` / `Eq` derive intact.
- round 33: MPEG-2 §6.2.5 macroblock body wire-parse wired into the
  slice walker. `slice_macroblock_walk::walk_slice` now follows
  `macroblock_modes()` + `quantiser_scale_code` with the four
  spec-deterministic body fields: `motion_vectors(0)` (gated on
  `macroblock_motion_forward == 1` OR `macroblock_intra &&
  concealment_motion_vectors == 1` per §6.2.5 / §6.3.11),
  `motion_vectors(1)` (`macroblock_motion_backward == 1`),
  `marker_bit` (the §6.3.17 concealment-MV `'1'` bit on intra
  macroblocks with `concealment_motion_vectors == 1` — a `'0'`
  here is rejected as `InvalidBitstream`), and
  `coded_block_pattern()` (`macroblock_pattern == 1`, with the
  §6.3.17.4 `pattern_code[12]` derivation pre-computed for the
  caller). Each is **wire-syntax only**: the §7.6.3.1 reconstruction
  of `vector'[r][s][t]` against the PMV state (and the §7.6.3.3 PMV
  update / §7.6.3.4 reset) stay deferred to the picture-level
  driver one layer up. `SliceWalkContext` grows six new fields —
  `f_code_fwd_horiz` / `f_code_fwd_vert` / `f_code_bwd_horiz` /
  `f_code_bwd_vert` (the four §6.3.11 `f_code[s][t]` widths driving
  `motion_residual`), `concealment_motion_vectors` (the §6.3.11
  intra-MB motion-vector / marker-bit gate), and `chroma_format`
  (the §6.3.5 `Yuv420` / `Yuv422` / `Yuv444` setting driving the
  §6.2.5.3 `coded_block_pattern_1` / `coded_block_pattern_2`
  extensions and the §6.3.17.4 `pattern_code` indexing). All three
  existing constructors keep their original signatures: `first_slice`,
  `first_slice_with_picture_extension`, and `first_slice_mpeg1`
  default the new fields to spec-legal "no body fields fire"
  placeholders (`f_code = 1`, `concealment_motion_vectors = false`,
  `chroma_format = Yuv420`) so the round-30..32 tests stay
  bit-identical. A new `first_slice_with_picture_body` surfaces the
  full pair so callers walking P/B slices, intra-concealment-MV
  slices, or 4:2:2 / 4:4:4 pictures can thread the real picture +
  sequence extension values through. `MacroblockRecord` gains five
  new fields — `motion_vectors_forward: Option<MotionVectors>`,
  `motion_vectors_backward: Option<MotionVectors>`,
  `concealment_marker_bit: Option<bool>`, `coded_block_pattern:
  Option<CodedBlockPattern>`, and `pattern_code: [bool; 12]` — and
  drops `Copy` (now `Clone` only) since two of those carry
  `Vec`-backed motion-vector entry lists. The `body_bit_position`
  cursor keeps its round-30..32 meaning: the offset right after
  `macroblock_modes()` + the §6.2.5 `quantiser_scale_code`, *before*
  any of the new wire-body fields, so any external driver that
  resumes parsing from `body_bit_position` is unaffected by this
  round's additions. The walker also synthesises a defaulted
  `MotionType` (Frame-based for frame pictures, Field-based for
  field pictures, `mv_count = 1`, `dmv = 0`) per §6.3.17.1 / Table
  6-19 in the absent-modes-tail case (frame picture with
  `frame_pred_frame_dct == 1` motion MB, or intra MB with
  concealment vectors) so `motion_vectors()` can still parse.
  10 new lib unit tests plus 4 new integration tests in
  `tests/slice_macroblock_walk_synthetic.rs` pin the new gates:
  the bare-intra full-`pattern_code` derivation, the
  concealment-MV `motion_vectors(0) + marker_bit` pair (including
  the `'0'`-marker rejection), the §6.3.17.4 CBP-driven 4:2:0 /
  4:2:2 / 4:4:4 `pattern_code` derivations, the B-picture
  `motion_vectors(0) + motion_vectors(1)` pair, the
  no-motion-no-pattern empty-pattern_code path, and the
  `body_bit_position` snapshot-before-body-fields contract. The
  integration tests also update the prior round-30 fixtures to
  carry valid `motion_vectors(0)` + CBP wire bits so the
  end-to-end `SliceHeader::parse → walk_slice` chain keeps green.
- round 32: MPEG-2 §6.2.5.1 `macroblock_modes()` tail wired into
  the slice walker. `slice_macroblock_walk::walk_slice` now reads
  `frame_motion_type` (Table 6-17, frame pictures with
  `frame_pred_frame_dct == 0` whose MB sets a motion flag),
  `field_motion_type` (Table 6-18, every motion-bearing MB in a
  field picture), and `dct_type` (frame pictures with
  `frame_pred_frame_dct == 0` whose MB is intra or has a coded
  pattern) between the existing `macroblock_type` parse and the
  §6.2.5 `if (macroblock_quant) quantiser_scale_code` read,
  matching the §6.2.5 syntax tree order. Two new
  `SliceWalkContext` fields `picture_structure` and
  `frame_pred_frame_dct` surface the §6.3.11 picture-extension
  state the tail is gated on; `SliceWalkContext::first_slice`
  keeps its existing 4-arg shorthand by defaulting to
  `picture_structure = Frame` / `frame_pred_frame_dct = true` (no
  tail reads — safe for I-pictures and the `frame_pred_frame_dct
  == 1` P/B case), while
  `first_slice_with_picture_extension` takes the full pair, and
  the new `first_slice_mpeg1` shorthand pins both fields so the
  §6.2.5.1 tail stays gated off on MPEG-1 streams (whose macroblock
  layer has its own §2.4.2.7 motion-vector parser). `MacroblockRecord`
  gains `motion_type: Option<MotionType>` and `dct_type: Option<bool>`
  alongside the existing `macroblock_type` / `quantiser_scale_code`
  fields. **Fixes a latent ordering bug** where the previous
  walker read the §6.2.5 `quantiser_scale_code` immediately after
  `macroblock_type`, misaligning the cursor on any MB whose
  `macroblock_modes()` tail consumed bits (e.g. P-picture
  `frame_pred_frame_dct == 0` motion MBs). 7 new lib tests plus 3
  new integration tests in `tests/slice_macroblock_walk_synthetic.rs`
  pin the four gate cases — frame_motion_type / field_motion_type /
  motion-omitted-by-frame_pred_frame_dct / dct_type emitted-with-intra
  — and a full P-picture MB chain that mixes type-tail-quant in one
  bit-exact 14-bit fixture.
- round 31: MPEG-2 §7.6.6 skipped-macroblock specification
  (`skipped_macroblock::describe_skipped_macroblock` +
  `skipped_macroblock_apply_to_pmv`) — the description module the
  round-30 slice walker flagged as a follow-up.
  `describe_skipped_macroblock` consumes a
  `SkippedMacroblockContext` (picture coding type / picture
  structure / previous-MB direction / PMV state /
  scalable_i_picture gate) and returns a `SkippedMacroblock` that
  pins the per-§7.6.6.1..4 deterministic prediction shape: the
  prediction type (Frame-based for §7.6.6.2 / §7.6.6.4,
  Field-based for §7.6.6.1 / §7.6.6.3), the derived `mv_format`,
  the same-parity field reference (§7.6.6.1 / §7.6.6.3), the
  direction (always `Forward` in P-pictures per §7.6.6.1 /
  §7.6.6.2; inherited from the previous MB in B-pictures per
  §7.6.6.3 / §7.6.6.4 "same as the previous macroblock"), the
  motion-vector source (`SkippedMotionVector::Zero` in
  P-pictures, `SkippedMotionVector::FromPmv { forward,
  backward }` in B-pictures with each slot present iff the
  inherited direction includes it), and the `reset_pmv` flag
  (true in P-pictures per §7.6.3.4 "In a P-picture when a
  macroblock is skipped" + §7.6.6.1 / §7.6.6.2 "Motion vector
  predictors shall be reset to zero"; false in B-pictures per
  §7.6.6.3 / §7.6.6.4 "Motion vector predictors are
  unaffected"). The §7.6.6 preamble I-picture rule rejects
  skipped MBs on non-scalable I-pictures; the
  `scalable_i_picture` gate exposes the spec exemption but
  surfaces "not yet supported" until the scalability extensions
  land. B-pictures with `previous_direction =
  PredictionDirection::Skipped` are also rejected per the
  "same as the previous macroblock" rule. The companion
  `skipped_macroblock_apply_to_pmv` hook fires the §7.6.3.4
  PMV reset (idempotent; no-op when `reset_pmv == false`). The
  module re-uses existing crate types (`Pmv` / `VectorIndex` /
  `Direction` / `Component` from `pmv`, `PictureCodingType` /
  `PictureStructure` from `picture_header`, `PredictionType` /
  `MvFormat` from `macroblock_modes`, `PredictionDirection`
  from `combine_predictions`, `FieldParity` from `dual_prime`)
  so the description plugs straight into the existing §7.6.4
  `predict_block` → §7.6.7 `combine_directional_predictions`
  prediction pipeline (sample-plane formation stays out of
  scope, since per-block residuals are conceptually zero for
  skipped MBs). New types
  `SkippedMacroblockContext` / `SkippedMacroblock` /
  `SkippedMotionVector` and entry points
  `describe_skipped_macroblock` /
  `skipped_macroblock_apply_to_pmv` are re-exported at the
  crate root. 15 new lib unit tests + 5 new integration tests
  under `tests/skipped_macroblock_synthetic.rs` pin the four
  §7.6.6 sub-clause derivations, the §7.6.6 preamble I-picture
  rejection (both non-scalable and scalable-but-unsupported),
  the B-picture previous-direction-Skipped rejection, the
  P-picture PMV reset (idempotent over a 10-MB run), and the
  B-picture PMV invariance over a 5-MB run.
- round 30: MPEG-2 §6.2.4 slice-level macroblock-header walker
  (`slice_macroblock_walk::walk_slice`) — the first §6.2.4 driver
  that picks up at the post-`slice_header()` cursor and walks the
  `do { macroblock() } while ( nextbits() != '0000 0000 0000 0000
  0000 0000' )` loop, parsing each macroblock's spec-deterministic
  header chain: §6.2.5 `macroblock_address_increment` (Table B-1
  with the `macroblock_escape` / MPEG-1 `macroblock_stuffing`
  chains), §6.2.5.1 `macroblock_modes()` opener (`macroblock_type`
  VLC against Tables B-2 / B-3 / B-4 keyed on
  `picture_coding_type`), and the conditional 5-bit
  macroblock-level `quantiser_scale_code` when `macroblock_quant
  == 1`. The driver tracks the §6.3.17.1 per-slice state across
  iterations — `previous_macroblock_address` seeded from
  `mb_row * mb_width - 1`, `macroblock_address` advancing through
  the increment chain, `past_intra_address` advancing to
  `macroblock_address` on every intra macroblock,
  `quantiser_scale_code` carried forward across macroblocks with
  intra-quant overrides applying to *this* MB and every
  subsequent MB in the slice — and rejects the first-MB
  increment-must-be-1 violation. Skipped-macroblock ranges
  (§6.3.17.4 / §7.6.6) are surfaced per record as
  `skipped_macroblock_count = increment - 1` so a future §7.6.6
  round can reconstruct them, without running the §7.6.6
  prediction itself here. The stop-condition peek is
  alignment-agnostic per §5.2.3 — the loop exits as soon as
  `nextbits()` shows 23 zero bits or the buffer runs out (the
  caller bounds the slice sub-buffer). New `SliceWalkContext` /
  `MacroblockRecord` / `SliceWalk` types are re-exported at the
  crate root alongside `walk_slice` and the
  `PAST_INTRA_ADDRESS_RESET = -2` sentinel from §6.3.17.1. 10 new
  lib unit tests + 5 new integration tests under
  `tests/slice_macroblock_walk_synthetic.rs` pin the empty-slice
  stop-pattern early-exit, the single-intra-MB I-picture walk,
  the §6.3.17.1 `macroblock_quant` override + carry-forward
  across subsequent MBs, the explicit override-then-reset
  three-MB walk, the first-MB-increment-rejection, the P-picture
  skipped-MB recording across a `mb_row=1` slice (starting at
  addr 22 with an increment-3 producing 2 skipped MBs), the
  intra `past_intra_address` advance across consecutive intra
  MBs, the rejection of zero `initial_quantiser_scale_code` /
  zero `mb_width`, the post-header `body_bit_position` accounting
  (entry point for the deferred `motion_vectors()` /
  `coded_block_pattern()` / `block(i)` driver rounds), and the
  end-to-end `SliceHeader::parse` + `walk_slice` chain on a
  hand-built slice-start-code-prefixed buffer. The
  `macroblock_modes()` tail (motion-type / dct_type),
  `motion_vectors(s)`, `coded_block_pattern()`, and the per-block
  walker stay out of scope this round — their PMV reset / f_code
  / per-block-context wiring intersects with cross-MB state that
  the picture-level driver above this slice walker needs to own;
  this driver exposes per-MB `body_bit_position` so each
  follow-on round can resume parsing at the post-header cursor.

- round 29: MPEG-2 §6.2.5 / §6.2.6 macroblock-block driver
  (`mpeg2_macroblock_blocks::decode_macroblock_blocks`) — the
  wrapper that walks a macroblock's `pattern_code[12]` array and
  dispatches the round-28 §6.2.6 `block(i)` driver
  (`mpeg2_block_decoder::decode_block`) once per coded slot,
  returning a `Vec<DecodedBlock>` paired with the §6.1.1.8
  block-index position. New `block_count(chroma_format)` (6 / 8 /
  12) and `block_component(i, chroma_format)` helpers encode the
  Figure 6-10 / 6-11 / 6-12 layout (4:2:0 = Y0..Y3 / Cb / Cr;
  4:2:2 = Y0..Y3 / Cb0..Cb1 / Cr0..Cr1; 4:4:4 = Y0..Y3 / Cb0..Cb3
  / Cr0..Cr3). The driver auto-derives the §7.4.2.1 Table 7-5
  weighting-matrix index `w` per coded block from `(coding,
  component, chroma_format)`, honours the §7.2.1 non-intra
  macroblock DC-predictor reset, validates the macroblock-level
  constants up-front (`intra_dc_precision ≤ 3`,
  `quantiser_scale_value ≠ 0`, predictor precision matches
  context precision), and propagates the first failing block's
  error without walking the rest. New `MacroblockBlockContext`
  groups the per-macroblock constants (`intra_vlc_format`,
  `alternate_scan`, `intra_dc_precision`, `quantiser_scale_value`,
  `chroma_format`, four weighting matrices); the per-block
  triplet (`component`, `macroblock_intra`, `weight`) is derived
  inside the driver from the parsed `MacroblockType` and the
  block index. `DEFAULT_WEIGHT_MATRICES` exposes the §6.3.7
  defaults indexed by Table 7-5 (intra luma, non-intra luma,
  intra chroma, non-intra chroma). Re-exported at the crate root
  as `mpeg2_decode_macroblock_blocks`,
  `Mpeg2MacroblockBlockContext`, `Mpeg2MacroblockDecodedBlock`,
  `mpeg2_block_component`, `mpeg2_block_count`,
  `MPEG2_DEFAULT_WEIGHT_MATRICES`. 15 new lib unit tests +
  6 new integration tests under
  `tests/mpeg2_macroblock_blocks_synthetic.rs` pin the §6.1.1.8
  block-index → component mapping across all three chroma
  formats, the six-block intra walk for 4:2:0, the eight-block
  4:2:2 walk (Cb-then-Cr ordering), the twelve-block 4:4:4 walk,
  the `pattern_code[]` gating (uncoded slots not consumed),
  Table 7-5 weighting-matrix dispatch for a 4:4:4 non-intra
  chroma block, the §7.2.1 non-intra-macroblock predictor reset,
  the intra-macroblock predictor-carry-over (no reset at MB
  entry), the cross-block DC predictor chain (128 → 129 → 130 →
  131 → 132 across four luma blocks with `dct_diff = +1` each),
  the bit-cursor accounting (28 bits for six size-0 intra blocks
  in 4:2:0), and the three argument-validation paths plus the
  first-failing-block propagation. This closes the round-28
  next-step candidate; the remaining MPEG-2 decode gap is the
  slice-layer driver that loops over macroblocks (parsing
  `macroblock_address_increment` / `macroblock_type` /
  `coded_block_pattern` / motion vectors per MB and dispatching
  to this macroblock-block driver per coded macroblock).

- round 28: MPEG-2 §6.2.6 `block(i)` driver
  (`mpeg2_block_decoder::decode_block`) — chains the §7.2.1 DC
  prelude (intra blocks only) → §7.2.2 residual VLC walker (with
  the §7.2.2.2 NOTE 2 / NOTE 3 FIRST / NEXT alternation honoured)
  → §7.3 inverse scan (Figure 7-2 / Figure 7-3 keyed off
  `alternate_scan`) → §7.4 inverse-quantisation pipeline
  (saturation + §7.4.4 mismatch control included) → §A 8×8 IDCT
  into a single "bitstream → `f[y][x]` plane" entry point. New
  `Mpeg2BlockContext` groups the per-macroblock constants
  (`intra_vlc_format`, `alternate_scan`, `intra_dc_precision`,
  `quantiser_scale_value`); per-block parameters
  (`component`, `macroblock_intra`, `weight`) move with each
  call. `Mpeg2DecodedBlock` carries the four intermediate planes
  (`QFS[]`, `QF[v][u]`, `F[v][u]`, `f[y][x]`) plus the post-EOB
  bit cursor. §7.2.2 wire-position constraint
  (`walker_index + run ≤ 63`) enforced as an `InvalidBitstream`
  rejection. Re-exported at the crate root as
  `mpeg2_decode_block`, `Mpeg2BlockContext`, and
  `Mpeg2DecodedBlock`. 16 new lib unit tests +
  7 new integration tests under
  `tests/mpeg2_block_decoder_synthetic.rs`.
- round 27: MPEG-2 §7.2.1 intra-block DC prelude — Annex B
  Tables B-12 (`dct_dc_size_luminance`) and B-13
  (`dct_dc_size_chrominance`) extended to `0..=11` with the
  long-prefix entries for `intra_dc_precision != 0`; §7.2.1
  `dc_dct_differential` → `dct_diff` `half_range`-threshold
  reconstruction (cross-checked against the §2.4.3.7 MPEG-1
  MSB-test form for every size 1..=8 input); per-component
  `dc_dct_pred[Y / Cb / Cr]` predictor state with Table 7-2 reset
  values `{128, 256, 512, 1024}` for `intra_dc_precision ∈
  {0, 1, 2, 3}`; the three-trigger reset contract (start of
  slice / non-intra macroblock / skipped macroblock); and the
  §7.2.1 `QFS[0] ∈ [0, 2^(8 + intra_dc_precision) - 1]`
  bitstream-constraint enforcement on the final predicted DC.
  Public driver `mpeg2_decode_dc_block(br, predictors, colour)`
  returns a typed `Mpeg2DcCoefficient` with the raw bits, signed
  `dct_diff`, final `QFS[0]`, and post-consume bit position.
  Re-exports at the crate root as `Mpeg2DcCoefficient`,
  `Mpeg2DcComponent`, `Mpeg2DcPredictors`, `Mpeg2ColourComponent`,
  `MPEG2_MAX_DC_SIZE`, `mpeg2_decode_dc_block`,
  `mpeg2_dc_pred_reset_value`, and `mpeg2_qfs_zero_max`. 29 new
  lib unit tests + 7 new integration tests under
  `tests/mpeg2_block_dc_synthetic.rs` pin Tables B-12 / B-13's
  cardinality + width invariants, the first-9-rows bit-exact
  match against MPEG-1 Tables B.5a / B.5b, the §7.2.1 ↔ §2.4.3.7
  reconstruction equivalence, the page-77 `dct_dc_size = 3`
  worked example, the size-11 corner values, the Table 7-2 reset
  lookup, the per-component routing (Y / Cb / Cr independence),
  the bitstream-constraint enforcement on both bounds, and the
  bit-position accounting for the shortest (size 0) and longest
  (size 11) B-12 codewords.

- round 26: MPEG-2 §7.3 inverse-scan — `ALTERNATE_SCAN`
  (Figure 7-3 / `scan[1][v][u]`), `ALTERNATE_INVERSE_SCAN`,
  `ZIGZAG_INVERSE_SCAN`, the `scan_table` / `inverse_scan_table`
  flag-driven selectors per `alternate_scan`, the
  `place_coefficient` per-sample writer that mates with the
  round-25 `Mpeg2DctCoeffStep` walker, and the `apply_inverse_scan`
  full §7.3 loop body for callers operating on a pre-flattened
  `QFS[0..64]` list. Figure 7-2 stays single-sourced from the
  MPEG-1 §2.4.4.1 `block_dc::SCAN` matrix; a unit test asserts the
  cell-for-cell equality so any future drift on either side trips
  immediately. Re-exports at the crate root as
  `MPEG2_ALTERNATE_SCAN`, `MPEG2_ALTERNATE_INVERSE_SCAN`,
  `MPEG2_ZIGZAG_INVERSE_SCAN`, `mpeg2_scan_table`,
  `mpeg2_inverse_scan_table`, `mpeg2_place_coefficient`,
  `mpeg2_apply_inverse_scan`. 21 new lib unit tests + 7 new
  integration tests under
  `tests/mpeg2_inverse_scan_synthetic.rs` pin Figures 7-2 / 7-3
  against the printed page 80, the permutation invariant, the
  forward · inverse round-trip in both scans, the §7.3.1
  matrix-download flag invariant, and a synthetic walker replay
  comparing per-coefficient placement against the full-loop body.

## [0.0.11](https://github.com/OxideAV/oxideav-mpeg12video/releases/tag/v0.0.11) - 2026-05-30

### Other

- round 25: MPEG-2 residual VLC walker (Tables B-14 / B-15 / B-16)
- round 24: §A 8×8 IDCT + IEEE 1180 / P1180/D2 conformance harness
- round 23: MPEG-2 §7.4 inverse-quantisation pipeline
- round 22: §7.6 per-macroblock pipeline driver
- round 21: §7.6.7 combine-predictions + §7.6.8 add-and-saturate
- round 20: §7.6.4 forming-predictions pel reader
- round 19: §7.6.3.6 MPEG-2 dual-prime additional arithmetic
- Round 18: MPEG-1 §2.4.4.1 / §2.4.4.2 dequantiser bodies
- Round 17: MPEG-1 dct_coeff_first / dct_coeff_next walker (Tables B.5c..B.5f)
- round 16: MPEG-1 intra-block DC prelude (§2.4.2.8 / §2.4.3.7) + zig-zag scan (§2.4.4.1)
- round 15: MPEG-1 §2.4.4.2 / §2.4.4.3 motion-vector reconstruction
- round 14: MPEG-1 motion_vector(s) per §2.4.2.7 + Annex B Table B.4
- round 13: §7.6.3.3 inter-vector PMV update (Tables 7-10 / 7-11)
- round 12: §7.6.3.1 motion-vector reconstruction + §7.6.3.4 reset + §7.6.3.7 chroma scaling
- round 11: motion_vectors() / motion_vector() + Tables B-10 / B-11 (§6.2.5.2)
- round 10: macroblock_modes() motion-type / dct_type tail (§6.2.5.1)
- round 9: parse macroblock-layer quantizer_scale (MPEG-1 §2.4.2.7/§2.4.3.6)
- refresh register() comment for the round 1–8 frontier
- round 8: coded_block_pattern() parser (§6.2.5.3, Table B-9)
- round 7: macroblock_type VLC (Annex B Tables B-2/B-3/B-4, §6.2.5.1)
- round 6: parse §6.2.5 macroblock_address_increment with Annex B Table B-1 VLC
- round 5: parse §6.2.4 slice() header bits (svp, q_scale, intra prelude)
- round 4: parse §6.2.3 picture_header() + §6.2.3.1 picture_coding_extension()
- round 3: parse §6.2.2.6 group_of_pictures_header() with 25-bit time_code
- round 2: parse §6.2.2.3 sequence_extension() and compose full 14-bit dimensions
- round 1: parse §6.2.2.1 sequence_header() for MPEG-2 / H.262
- orphan rebuild: clean-room scaffold post 2026-05-18 audit

### Added

- Clean-room rebuild round 25: MPEG-2 **residual VLC walker** for the
  §6.2.6 block-layer `dct_coeff_first` / `dct_coeff_next` body per
  ISO/IEC 13818-2 (ITU-T H.262) §7.2.2 (Annex B Tables **B-14** / **B-15**
  / **B-16**). The MPEG-1 §2.4.3.7 walker in
  [`dct_coeff::DctCoeffStep`] only handled the older Tables B.5c..B.5f
  shape; this round lands the MPEG-2 sibling that §7.2.2.3 explicitly
  notes is different — both in escape encoding and in the per-table
  `end_of_block` codeword.
  - `mpeg2_dct_coeff::TableSelection::from_context(intra_vlc_format,
    macroblock_intra)` — §7.2.2.1 Table 7-3 selector with the four-row
    resolution: `TableZero` (B-14) for `intra_vlc_format = 0` and for
    every non-intra block; `TableOne` (B-15) only when
    `intra_vlc_format = 1` and `macroblock_intra = 1`.
  - `mpeg2_dct_coeff::DctCoeffStep::parse(br, table, position)` — the
    actual walker. Implements §7.2.2.2 NOTE 2 / NOTE 3 — the FIRST-only
    `1s` (1-bit) and NEXT-only `11s` (2-bit) alternates for B-14's
    `(run = 0, level = ±1)`, both honouring the §7.2.2 sign-bit
    contract. Honours the table-dependent `end_of_block` codeword
    (`10` for B-14, `0110` for B-15) and the §7.2.2.3 escape prefix
    `000001` (6 bits).
  - `mpeg2_dct_coeff::CoefficientPosition` enum (`First` / `Next`)
    captures the §7.2.2.2 modification: §7.2.2.2's note clarifies that
    Table B-14 is only modified for the FIRST coefficient of a
    **non-intra** block, since the first coefficient of an intra block
    is the §7.2.1 DC value handled by [`block_dc`]. B-15 therefore
    always enters at `Next`.
  - Table B-16 escape payload: 6-bit `run` (`0..=63`) + 12-bit signed
    `signed_level` (`[-2047, +2047] \ {0}`, two's-complement wire word
    with `0x000` and `0x800` both rejected as the spec's forbidden
    values).
  - Coverage: 24 new unit tests against ISO/IEC 13818-2 Annex B and
    §7.2.2 — Table 7-3 selector matrix, table row counts (B-14 = 112,
    B-15 = 111), per-width codeword bit-width invariants, per-width
    uniqueness, prefix-freeness at FIRST and NEXT for both tables
    (including escape + EoB), round-trip of every B-14 row (224 cases
    counting both signs), round-trip of every B-15 row (222 cases),
    the FIRST-only and NEXT-only `(0, ±1)` disambiguation, both
    table-dependent EoB codewords, Table B-16 escape round-trip
    (positive + negative extremes including `±2047`), the forbidden
    `signed_level = 0` and `signed_level = -2048` wire words, the
    short-buffer error path, the unrecognised-prefix error path, and
    end-to-end block walks against both tables (B-14 non-intra:
    FIRST → NEXT → escape → EoB; B-15 intra: NEXT → NEXT → escape →
    EoB).

  Spec citations refer to ISO/IEC 13818-2 (ITU-T H.262) §§7.2.2,
  7.2.2.1 (Table 7-3), 7.2.2.2 (FIRST / NEXT modification), 7.2.2.3
  (escape, Table B-16), 7.2.2.4 (decoder pseudo-code), and Annex B
  Tables B-14 and B-15.

- Clean-room rebuild round 24: §A **8×8 inverse discrete cosine
  transform** (the IDCT stage of Figure 7-1 between §7.4
  inverse-quantisation and the §7.6 macroblock pipeline) with an IEEE
  Std 1180-1990 / P1180/D2 conformance harness against the four
  statistical metrics ISO/IEC 11172-2 Annex A and ISO/IEC 13818-2 §A
  require by reference.
  - `idct::idct_reference_f64` — direct 4-D summation of the §A
    trigonometric identity at `f64` precision; the "infinite
    precision" reference IEEE 1180 specifies as the gold standard.
  - `idct::idct_candidate_f64` — the fast separable 1-D-pass IDCT
    (eight row IDCTs followed by eight column IDCTs) used internally
    by the integer IDCT. Mathematically identical to the direct
    reference; differs only in `f64` rounding order.
  - `idct::idct_8x8` / `idct_8x8_from_i32` — the integer IDCT API the
    downstream §7.6 pipeline consumes: calls `idct_candidate_f64`,
    rounds with the spec's `Round(x)` operator, and saturates the
    9-bit signed pel range `[-256, +255]` per §7.5.
  - Module constants `F_INPUT_MIN` / `F_INPUT_MAX` (12-bit signed,
    `[-2048, 2047]`) and `F_OUTPUT_MIN` / `F_OUTPUT_MAX` (9-bit
    signed, `[-256, 255]`) expose the §7.4.3 input clamp and the §7.5
    output clamp respectively.
  - `tests/idct_p1180_conformance.rs` — IEEE 1180 / P1180/D2
    statistical accuracy test against the bounds staged at
    `docs/video/mpeg12video/idct-accuracy-spec.md` §4: peak error
    `pe ≤ 1`, peak per-position MSE `pmse ≤ 0.06`, overall MSE
    `omse ≤ 0.02`, peak per-position mean error `pme ≤ 0.015`, and
    overall absolute mean error `ome ≤ 0.0015`. Six pseudo-random
    parameter sets (`L ∈ {256, 5, 300}` × both signs, 1024 blocks
    each, deterministic LCG seeds) plus the spec's two deterministic
    edge cases (all-zero coefficient block → all-zero pel block;
    DC-only block → flat pel block).
- Clean-room rebuild round 23: MPEG-2 (ISO/IEC 13818-2 / Recommendation
  ITU-T H.262) §7.4 **inverse-quantisation pipeline** — the dequantiser
  stage of Figure 7-1 between §7.3 inverse-scan and the §A.1 IDCT. Lives
  in `src/mpeg2_dequantize.rs`; the MPEG-1 §2.4.4 dequantiser stays in
  `src/dequantize.rs` (the two formulations diverge by spec).
  - `mpeg2_dequantize::intra_dc_mult` — Table 7-4
    `intra_dc_precision → intra_dc_mult` (8 / 4 / 2 / 1), with
    `intra_dc_mult_from_extension` taking the parsed
    `PictureCodingExtension` directly.
  - `mpeg2_dequantize::DEFAULT_INTRA_WEIGHT` /
    `DEFAULT_NON_INTRA_WEIGHT` — the §6.3.7 default `W[0]` / `W[1]`
    matrices (the intra-default mirrors MPEG-1's `intra_quant`; the
    non-intra-default is all-16).
  - `mpeg2_dequantize::select_weighting_matrix_index(coding,
    component, chroma_format)` — Table 7-5 weighting-matrix index
    selection (`w ∈ {0, 1, 2, 3}`), folding the 4:2:0 chroma collapse
    into the luma slot and the 4:2:2 / 4:4:4 split out into `w == 2`
    (intra chroma) and `w == 3` (non-intra chroma).
  - `mpeg2_dequantize::QUANTISER_SCALE_LINEAR` /
    `QUANTISER_SCALE_NONLINEAR` — the Table 7-6 lookup arrays for
    `q_scale_type == 0` (linear, `2..=62`) and `q_scale_type == 1`
    (non-linear, `1..=112`); the safe accessor `quantiser_scale(code,
    q_scale_type)` rejects code `0` (forbidden per Table 7-6) and any
    value above the 5-bit range.
  - `mpeg2_dequantize::inverse_quantise_block(qf, coding, weight,
    quantiser_scale, intra_dc_mult)` composes §7.4.1 + §7.4.2.3 +
    §7.4.3 + §7.4.4 into one call: §7.4.1 intra-DC short-circuit at
    `(v, u) == (0, 0)` for `Intra`, §7.4.2.3 reconstruction `((2 *
    QF + k) * W * quantiser_scale) / 32` (`k = 0` for intra, `k =
    Sign(QF)` for non-intra) under the §4.1 round-toward-zero `/`
    operator, §7.4.3 saturation to `[-2048, 2047]`, and §7.4.4
    mismatch control (sum-parity LSB toggle on `F[7][7]`).
    `F_SATURATION_MIN` / `F_SATURATION_MAX` surface the §7.4.3
    clamp bounds; helper functions `sign` / `saturate` expose the
    §4.1 `Sign(...)` and §7.4.3 clamp for callers that want to
    replay the table outside `inverse_quantise_block`.
  - 21 unit tests (Table 7-4 / Table 7-5 / Table 7-6 coverage,
    `Sign`, `Saturate`, and synthetic intra + non-intra walks
    through §7.4.5) + a 7-test integration suite
    (`tests/mpeg2_dequantize_synthetic.rs`) that cross-checks the
    public surface against an independently-coded reference loop
    transcribed from the spec text.
  - Does *not* run the §A.1 IDCT itself (`#1110` IDCT-precision spec
    pending).
- Clean-room rebuild round 22: MPEG-2 (ISO/IEC 13818-2 / Recommendation
  ITU-T H.262) §7.6 **Per-macroblock pipeline driver** — the composition
  step that stitches the already-landed §7.6.5 / §7.6.6 case selection,
  §7.6.7 combine-predictions, and §7.6.8 add-and-saturate endpoints into
  a single "parsed-syntax + per-block predictions + per-block IDCT
  output in → final per-coded-block decoded samples out" driver, keyed
  off the parsed `MacroblockType` flags and the §6.3.17.4
  `pattern_code[12]` derivation of `CodedBlockPattern`.
  - `macroblock_pipeline::MacroblockKind { Intra, Inter(direction) }`
    classifies the macroblock per §7.6.5 / §7.6.6 (intra flag
    dominates motion flags; `(forward, backward) = (0, 0)` maps to
    the `Skipped` direction for the §7.6.3.5 implicit zero-MV case),
    and `MacroblockKind::from_macroblock_type` performs the
    classification from a parsed `MacroblockType`.
  - `macroblock_pipeline::BlockInputs` is the per-block payload —
    `transform: &[i16]` plus `prediction_forward: &[u8]` /
    `prediction_backward: &[u8]` — with `BlockInputs::intra` /
    `::forward` / `::backward` / `::bidirectional` constructors that
    leave the unused prediction side(s) empty.
  - `macroblock_pipeline::decode_block(kind, inputs)` is the inner
    driver: for `Intra` it calls `add_intra_block` (§7.6.8 `d =
    saturate(f)` shortcut); for every inter case it calls
    `combine_directional_predictions` then
    `add_prediction_and_coefficients` for the `d = saturate(f + p)`
    `[0, 255]` clamp.
  - `macroblock_pipeline::decode_macroblock(kind, cbp, mt, chroma,
    block_inputs)` is the outer driver: walks
    `pattern_code[0 .. blocks_per_macroblock(chroma)]` and invokes
    `decode_block` per coded slot, returning each `DecodedBlock` with
    its §6.3.17.4 `block_index`. Uncoded slots and out-of-format
    chroma slots are skipped.
  - `macroblock_pipeline::blocks_per_macroblock(chroma)` returns the
    §6.1.1.8 chroma-format block count per MB (6 for 4:2:0, 8 for
    4:2:2, 12 for 4:4:4).
  - `macroblock_pipeline::PipelineError { LengthMismatch,
    MissingForwardPrediction, MissingBackwardPrediction,
    MissingBidirectionalPrediction }` enumerates the four
    caller-bug paths; the driver does not parse bitstreams so an
    `InvalidBitstream` cannot originate here.
  - The driver intentionally does **not** run the §A.1 IDCT (still
    blocked by workspace issue #1110), the §7.6.4 pel reader, the
    `coded_block_pattern()` bitstream walk, or the §6.2.5 macroblock
    layer parsers; each of those is consumed as an input from the
    already-landed pieces in their own modules.
  - 22 new unit tests in `src/macroblock_pipeline.rs` cover the
    `MacroblockKind` classifier (intra overrides motion, four-way
    inter direction map), `decode_block`'s intra bit-equality with
    `add_intra_block` and its prediction-side-ignored property, the
    inter forward / backward / bidirectional / skipped combine-then-
    add arithmetic on 2×2 blocks, the four caller-bug errors,
    `blocks_per_macroblock` for all three chroma formats, and
    `decode_macroblock`'s intra-everywhere walk (6 / 12 blocks per
    MB), inter-only-cbp-bits-walked walk
    (`cbp = 0b101010` → blocks 0 / 2 / 4), skipped-zero-pattern walk
    (zero coded blocks), 4:2:2 walk (8 coded blocks driven by
    `coded_block_pattern_1`), and the error-propagation-on-first-
    failing-block path.
  - 8 new integration tests in `tests/macroblock_pipeline_synthetic.rs`
    drive the pipeline end-to-end on hand-built reference planes and
    fabricated `i16` IDCT outputs for: 4:2:0 intra-everywhere
    (6 blocks); P-forward-only zero-residual (prediction passes
    through unchanged); B-bidirectional with the §7.6.8 clamp engaging
    at both ends; B-backward-only on a single coded block; the
    all-zero `pattern_code[]` skipped MB; the inner `decode_block`
    on a canonical 8×8 intra block; `MissingForwardPrediction`
    propagation; and `blocks_per_macroblock` chroma-format mapping.

- Clean-room rebuild round 21: MPEG-2 (ISO/IEC 13818-2 / Recommendation
  ITU-T H.262) §7.6.7 **Combining predictions** + §7.6.8 **Adding
  prediction and coefficient data** — the bidirectional-average step
  that turns the up-to-two §7.6.4 forward / backward prediction blocks
  into the final per-component prediction sample plane, plus the
  prediction-plus-IDCT-plus-saturation reconstruction that produces the
  final decoded samples.
  - `combine_predictions::average_predictions(forward, backward)` and
    its `..._in_place` variant implement the §7.6.7.1 page-105
    formula `pel_pred[y][x] = (pel_pred_forward[y][x] +
    pel_pred_backward[y][x]) // 2` over equal-length `Vec<u8>`
    prediction blocks. The §4.1 `// 2` operator on the non-negative
    sum of two `u8` values is the canonical `(sum + 1) >> 1`
    rounded-up form.
  - `combine_predictions::PredictionDirection { Forward, Backward,
    Bidirectional, Skipped }` captures the four §7.6.5
    Tables 7-13 / 7-14 selection cases, and
    `combine_predictions::combine_directional_predictions(direction,
    forward, backward)` is the driver that returns the combined
    block (single-direction pass-through for `Forward` / `Backward`,
    `average_predictions` for `Bidirectional`, forward pass-through
    for the §7.6.3.5 implicit-zero-MV `Skipped` case).
  - `combine_predictions::average_dual_prime_predictions(same_parity,
    opposite_parity)` is the §7.6.7.4 alias of the same formula —
    arithmetic identical to the bidirectional average, separate name
    for caller readability when wiring §7.6.3.6 dual-prime vectors
    through the §7.6.4 reader.
  - `add_coefficients::saturate(value)` implements the two `if`
    clauses of §7.6.8 page 106 (`d < 0 -> 0`, `d > 255 -> 255`) as a
    single `i32::clamp` returning `u8`.
  - `add_coefficients::add_prediction_and_coefficients(transform,
    prediction)` and its `..._in_place` variant pointwise add the
    §A.1 IDCT output (`i16`) and the §7.6.7 prediction (`u8`) and
    saturate to `[0, 255]`. Geometry-agnostic — the spec writes the
    loop over 8×8 but the operation is intrinsically pointwise.
  - `add_coefficients::add_intra_block(transform)` is the
    intra-macroblock shortcut: no prediction step has run for
    `macroblock_intra == 1`, so the final samples are just
    `saturate(f)` across the IDCT output, equivalent to passing an
    all-zero prediction buffer.
  - 34 new unit tests cover the `// 2` rounding on every relevant
    case (no-tie, half-integer tie, u8 max), the four-way
    `combine_directional_predictions` switch (including length-
    mismatch rejection on the `Bidirectional` branch), the dual-prime
    alias's bit-equality with the bidirectional path, the saturation
    arithmetic at both clamps (`-1 -> 0`, `256 -> 255`, `i32::MIN /
    MAX` extremes), the in-place add and the intra shortcut's
    equivalence to the zero-prediction path. A new
    `tests/combine_add_synthetic.rs` integration test (7 cases)
    drives the full §7.6.4 → §7.6.7 → §7.6.8 chain on hand-crafted
    references for the intra / P-forward / B-bidirectional /
    B-backward / skipped / 8×8 paths, with hand-computed expected
    samples (no external decoder oracle).

- Clean-room rebuild round 20: MPEG-2 (ISO/IEC 13818-2 / Recommendation
  ITU-T H.262) §7.6.4 **Forming predictions** — the integer-and-half-pel
  sample reader that turns a fully-reconstructed `vector'[r][s][1:0]`
  into a `width × height` pel-prediction block.
  - `forming_predictions::split_component(vector)` implements the
    per-axis split: `int_vec = vector DIV 2`, `half_flag = (vector -
    2 * int_vec) != 0`. `DIV` is the §4.1 floor-toward-minus-infinity
    operator, so `(-3) DIV 2 = -2` (not `-1` as truncate-toward-zero
    would give); the half-flag is set whenever the original component
    is odd, including for negative odd vectors.
  - `forming_predictions::HalfPattern` enumerates the four
    `(half_flag[0], half_flag[1])` outcomes (`Integer`,
    `HalfHorizontal`, `HalfVertical`, `HalfBoth`) that drive the
    §7.6.4 page-88 four-arm sample-reading switch.
  - `forming_predictions::predict_sample` / `predict_block` apply the
    switch — single sample for the integer case, the `// 2` two-
    sample average for horizontal-or-vertical half-pel, and the
    `// 4` four-sample bilinear average for the diagonal half-pel
    case. The §4.1 round-half-away-from-zero `//` operator on the
    non-negative reference sums coincides with `(sum + d/2) / d`
    integer division.
  - `forming_predictions::ReferencePlane` is a borrowed view
    `(data, width, height)` with a `BoundaryMode::PadEdge` clip-to-
    nearest-in-bounds-sample rule for motion vectors that reach past
    the picture edge; the H.262 base text leaves out-of-picture
    behaviour undefined.
  - `forming_predictions::BlockSize { width, height }` keeps the
    block geometry dimensionless so the §7.6.5 prediction-mode table
    (16×16 frame, 16×8 MC, 16×16 field, 8×8 / 8×16 / 16×16 chroma)
    can drive this loop without per-mode duplication.
  - 38 new unit tests cover the §4.1 `DIV` semantics across the
    sign / parity / magnitude grid, the four `HalfPattern` outcomes
    with their flag round-trip, `ReferencePlane` boundary clamping
    in all four directions plus the corner case, the four-arm
    sample switch with `// 2` and `// 4` rounding (including the
    `(0,1,0,1) // 4 → 1` half-integer-tie case), negative-odd
    vectors that exercise the `DIV`-vs-truncate difference, and
    `predict_block` end-to-end on 2×2 / 4×2 / 3×1 / 1×3 / 4×1
    geometries.

- Clean-room rebuild round 19: MPEG-2 (ISO/IEC 13818-2 / Recommendation
  ITU-T H.262) §7.6.3.6 **dual-prime additional arithmetic** — derive
  the opposite-parity motion vector(s) `vector'[r][0][1:0]` from the
  same-parity vector decoded by §7.6.3.1 and the inline `dmvector[0..1]`.
  - `dual_prime::derive_opposite_parity_vector(picture, parity_ref,
    parity_pred, vector_index, decoded_horiz, decoded_vert,
    dmvector_horiz, dmvector_vert)` is the single-row entry point;
    `dual_prime::derive_all(picture, decoded_horiz, decoded_vert,
    dmvector_horiz, dmvector_vert)` is the picture-level driver that
    yields one derived vector for a field picture (`r = 2`) or two
    derived vectors for a frame picture (`r = 2` top, `r = 3` bottom)
    per the §7.6.3.6 page-87 sentence "The top field shall use
    `vector'[2][0][1:0]` for opposite parity prediction and the
    bottom field shall use `vector'[3][0][1:0]`".
  - `dual_prime::m_factor` encodes Table 7-12 (the
    `picture_structure` / `top_field_first`-keyed field-distance
    factor). Frame `tff=1` → `(m[1][0], m[0][1]) = (1, 3)`; `tff=0`
    → `(3, 1)`. Top-field picture → only `m[1][0] = 1`; bottom-field
    picture → only `m[0][1] = 1`. Diagonal cells `m[0][0]` /
    `m[1][1]` are not on Table 7-12 (the same-parity vector is the
    input, not derived) and the function errors when asked for them.
  - `dual_prime::e_offset` encodes Table 7-13 (the unconditional
    vertical-line adjustment between top / bottom fields, picture-
    structure-independent): `e[0][0] = 0`, `e[0][1] = +1`,
    `e[1][0] = -1`, `e[1][1] = 0`.
  - The `//` operator (§4.1 page 9 "integer division with rounding
    to the nearest integer; half-integer values rounded away from
    zero") is honoured for the `(decoded * m) // 2` halving via a
    private helper distinct from `i32::div` (the spec's `/`) and
    `i32::div_euclid` (the spec's `DIV`).
  - `dual_prime::DualPrimePicture` / `dual_prime::dual_prime_picture`
    lower the parser-level `(PictureStructure, top_field_first)` pair
    into a typed context the call site doesn't have to branch on.
  - `dual_prime::FieldParity` (`Top` = 0, `Bottom` = 1) and
    `dual_prime::DerivedDualPrimeVector` (`{parity_ref, parity_pred,
    vector_index, horiz, vert}`) round out the surface.
  - Rejection sites: `dmvector` component outside `{-1, 0, +1}`
    (defensive guard around upstream Table B-11 parsing); any
    `(parity_ref, parity_pred)` pair that isn't on Table 7-12 for the
    active picture type.
  - 19 new unit tests cover the §4.1 `//` examples (`3//2 = 2`,
    `-3//2 = -2`, exact-divisible, `1//2`, `-1//2`, `5//2`, `-5//2`);
    Table 7-12 all four rows; the off-row error paths; Table 7-13 all
    four entries; §7.6.3.6 worked examples for both field-picture
    parities, both frame-picture `tff` values, both `r = 2` and `r =
    3` derivations, the `m = 3` triple-scaling, the swap on `tff =
    0`, the rounding-away-from-zero `decoded = ±3` case under `m =
    1`; out-of-range `dmvector` rejection; the `derive_all` driver's
    one-vs-two output shape; the `dual_prime_picture` lowering for
    all three `PictureStructure` values; and `FieldParity::index` /
    `FieldParity::opposite`.

- Clean-room rebuild round 18: MPEG-1 (ISO/IEC 11172-2:1993)
  §2.4.4.1 / §2.4.4.2 dequantiser bodies — the pure-math stage that
  consumes the round-17 walker's `dct_zz[]` array and emits the
  `dct_recon[m][n]` matrix the §A.1 IDCT operates on.
  - `dequantize::dequantize_intra_block(dct_zz, quantizer_scale,
    intra_quant, kind, predictors, macroblock_address)` folds the
    four §2.4.4.1 (page 32) block-loops (`LuminanceFirst`,
    `LuminanceSubsequent`, `ChrominanceCb`, `ChrominanceCr` via
    `dequantize::IntraBlockKind`) into a single entry-point. The
    shared body applies the `2 * dct_zz[scan[m][n]] *
    quantizer_scale * intra_quant[m][n] / 16` numerator, the `if
    (recon & 1) == 0 -> recon -= Sign(recon)` even-mismatch rule,
    and the `[-2048, 2047]` saturating clip. The DC element
    `dct_recon[0][0]` is then overwritten per the block-kind
    branch: `LuminanceFirst` / `ChrominanceCb` / `ChrominanceCr`
    pick between `128*8 + dct_zz[0]*8` (when `macroblock_address -
    past_intra_address > 1`) and `dct_dc_<comp>_past +
    dct_zz[0]*8`; `LuminanceSubsequent` is unconditional
    `dct_dc_y_past + dct_zz[0]*8`. The matching `dct_dc_<comp>_past`
    field of `IntraDcPredictors` is updated in place.
  - `dequantize::IntraDcPredictors` holds the three per-component
    `dct_dc_*_past` chains and `past_intra_address`.
    `at_slice_start()` returns the §2.4.4.1 slice-start state (all
    1024, `past_intra_address = -2`). `reset_dc_to_default()`
    zeros the three predictors back to 1024 without touching
    `past_intra_address` — the spec's per-non-intra-macroblock
    reset (including skipped macroblocks).
    `dequantize::finalise_intra_macroblock(predictors,
    macroblock_address)` performs the per-macroblock
    `past_intra_address = macroblock_address` close-out.
  - `dequantize::dequantize_non_intra_block(dct_zz,
    quantizer_scale, non_intra_quant)` implements the §2.4.4.2
    page-35 body: numerator is `(2*dct_zz[i] + Sign(dct_zz[i])) *
    quantizer_scale * non_intra_quant[m][n]`, then the same
    even-mismatch + saturation pipeline, then a final `if
    (dct_zz[i] == 0) dct_recon[m][n] = 0;` zeroing pass. There is
    no DC predictor chain for non-intra blocks.
  - `dequantize::DEFAULT_INTRA_QUANT` and
    `dequantize::DEFAULT_NON_INTRA_QUANT` are the §2.4.3.2 page-25
    default matrices used when the sequence header sets the
    matching `load_*_quantizer_matrix == 0`.
    `DEFAULT_INTRA_QUANT[0][0] == 8` matches the spec's
    `intra_quant[0][0] = 8` requirement; every entry of
    `DEFAULT_NON_INTRA_QUANT` is 16.
  - Rejection sites: `quantizer_scale == 0` and `> 31` are
    rejected with `Error::InvalidBitstream` (§2.4.3.6, defensive),
    and any zero entry in the active `intra_quant` /
    `non_intra_quant` matrix is rejected (§2.4.3.2 "The value zero
    is forbidden.").
  - Public re-exports from the crate root:
    `dequantize_intra_block`, `dequantize_non_intra_block`,
    `finalise_intra_macroblock`, `IntraBlockKind`,
    `IntraDcPredictors`, `DCT_RECON_MAX`, `DCT_RECON_MIN`,
    `DC_PREDICTOR_RESET`, `DEFAULT_INTRA_QUANT`,
    `DEFAULT_NON_INTRA_QUANT`.
- 35 new unit tests in `src/dequantize.rs::tests`: default
  matrices (corners, `intra_quant[0][0] = 8`, all-16 non-intra),
  predictor reset (slice-start, per-non-intra-macroblock),
  `Sign(...)` primitive, even-mismatch rule (no-op on odd, ±1
  correction on even of both signs, zero left alone), saturation
  at both bounds, intra rejection of `quantizer_scale = 0` / `>
  31` / `intra_quant[i][j] = 0`, the slice-start zero-`dct_zz`
  walkthrough that fires the reset branch, the adjacent-vs-gap
  `past_intra_address` branches for `LuminanceFirst`,
  `LuminanceSubsequent` ignoring `past_intra_address`, Cb / Cr
  predictor isolation, the `finalise_intra_macroblock` close-out,
  intra AC worked examples (positive-even subtract-sign,
  negative-even add-sign, saturation at both bounds), non-intra
  rejection sites, all-zero `dct_zz` → all-zero recon, positive
  and negative non-intra worked examples (`+3` → 55, `-3` → -55),
  non-intra saturation, the zeroing-pass override at zero
  neighbours, the full four-luma + Cb + Cr intra-macroblock walk
  with isolated per-component predictor advance, and a
  second-macroblock walk that confirms the address-gap branch
  switches correctly between reset and chain.
- 2 new black-box integration tests at
  `tests/dequantize_synthetic.rs` chain the round-17
  `DctCoeffStep` walker directly into the round-18 dequantiser:
  one non-intra (FIRST `(0, +3)` + NEXT `(2, -1)` + EoB → spec
  closed form `+55` / `-23` at the matching `INVERSE_SCAN` cells)
  and one intra (synthetic `dct_zz[0] = +5` DC prelude + NEXT
  `(0, +3)` + EoB → `1064` reset-branch DC and `+47` AC).
- Crate-level docstring + `register()` docstring updated to
  mention the round-18 dequantiser landing; the function is still
  a no-op because the §A.1 IDCT and motion-compensation
  pel-prediction loops are still ahead.

- Clean-room rebuild round 17: MPEG-1 (ISO/IEC 11172-2:1993) residual
  block `dct_coeff_first` / `dct_coeff_next` walker — the
  zig-zag-coded run-level body that follows the round 16 DC prelude
  (intra blocks) or replaces it (non-intra blocks).
  - `dct_coeff::DctCoeffStep::parse(br, position)` walks Annex B
    Tables B.5c / B.5d / B.5e (the run-level codebook) longest-first
    across all code widths from 1 to 16 bits, matches the
    `(run, level)` pair, then reads the trailing 1-bit sign `s` and
    applies it to produce the signed `dct_zz[i]` coefficient to write
    at the §2.4.3.7 zig-zag position (`i = run` for FIRST,
    `i += run + 1` for NEXT).
  - `dct_coeff::CoefficientPosition { First, Next }` disambiguates
    the spec's two `(run = 0, level = 1)` codes: `dct_coeff_first`
    uses the 2-bit `1s` form (legal only as the first coefficient of
    a non-intra block); `dct_coeff_next` uses the 3-bit `11s` form.
    The `end_of_block` codeword `10` is accepted only at `Next` per
    Table B.5c note 2 — FIRST decoding `10` returns the `1s` form
    instead.
  - Table B.5f escape coverage: the 6-bit `000001` prefix is followed
    by a 6-bit `run` (1..=63) and an 8-bit signed-level word; the
    short form covers `level ∈ [-127, +127] \ {0}` directly, and the
    long form (8-bit prefix `0x80` for negative or `0x00` for
    positive plus a second 8-bit magnitude byte) extends the range to
    `[-255, -128]` and `[+128, +255]`. The forbidden `-256` and the
    forbidden long-form positive `< 128` (which the short form
    already covers) are rejected. The escape encoding is
    intentionally **not** the same as MPEG-2 Table B-16 — the spec
    text in ISO/IEC 13818-2 §7.2.2.3 explicitly notes the change.
  - `dct_coeff::DctCoeff` is the decoded symbol: either
    `RunLevel { run, signed_level, escape }` (with the `escape`
    field recording whether the symbol came through the Table B.5f
    path) or `EndOfBlock`.
  - `dct_coeff::MAX_RUN = 63` and `dct_coeff::MAX_LEVEL_MAG = 255`
    document the spec bounds for both VLC and escape forms.
  - 31 unit tests cover: every Table B.5c / B.5d / B.5e row parsed
    and round-tripped with both signs (across all 112 ordinary
    rows × 2 signs); per-width prefix-freeness and code-fit-width
    invariants; FIRST-vs-NEXT `(0, 1)` disambiguation; EoB
    recognition only at NEXT; Table B.5f short form including the
    `±127` corner; Table B.5f long form for both signs (`-128`,
    `-255`, `+128`, `+200`, `+255`); rejection of the forbidden
    `-256` and forbidden long-form-positive-below-128 encodings;
    truncated and empty buffer rejection; and bit-position
    accounting across every code width from the 2-bit FIRST form to
    the 17-bit B.5e maximum.
  - 2 new black-box integration tests synthesise complete MPEG-1
    residual block runs (FIRST + several NEXT including a B.5f
    escape, then `end_of_block`) and confirm the §2.4.3.7
    zig-zag-position update never exceeds 63 plus the running bit
    cursor lines up exactly with the encoded bit lengths.

- Clean-room rebuild round 16: MPEG-1 (ISO/IEC 11172-2:1993) intra-block
  DC prelude — the entry point of the residual block layer.
  - `block_dc::DcCoefficient::parse(br, component)` walks Annex B
    Table B.5a (`dct_dc_size_luminance`) or Table B.5b
    (`dct_dc_size_chrominance`) for the size VLC, then reads the
    `dct_dc_size`-wide `dct_dc_differential` field MSB-first per
    §2.4.2.8 and applies the §2.4.3.7 sign-extension formula
    (`if (raw & (1 << (size-1))) zz0 = raw; else zz0 = ((-1) << size)
    | (raw + 1)`) to produce the signed `dct_zz[0]` in the range
    `[-(2^size - 1), +(2^size - 1)]`.
  - `block_dc::DcComponent { Luminance, Chrominance }` selects the
    matching VLC table.
  - `block_dc::SCAN: [[u8; 8]; 8]` encodes the §2.4.4.1 page-32 8x8
    zig-zag `scan[m][n]` matrix that maps a (zig-zag-ordered) cell
    to its raster index `i` for the §2.4.4.1 dequantiser loop.
    `block_dc::INVERSE_SCAN: [(u8, u8); 64]` is the compile-time
    inverse for encoders / trace tools.
  - `block_dc::MAX_DC_SIZE = 8` documents the spec upper bound
    enforced by both tables.
  - 23 unit tests cover: every B.5a / B.5b row, code-width
    uniqueness, codes-fit-their-width invariants, the §2.4.3.7
    page-30 worked example (`dc_size = 3`: `000 → -7, 001 → -6, ...
    111 → +7`), the `dc_size = 1 / 2 / 8` corner values
    (`reconstruct(8, 0x00) == -255`, `reconstruct(8, 0xFF) == +255`),
    truncated-buffer / garbage-prefix detection, luminance-vs-
    chrominance table disambiguation on identical wire bits
    (`'00'` decodes as size 1 in B.5a but size 0 in B.5b), full
    bit-position tracking on size 0 (3 bits) vs size 8 (15 bits),
    and the §2.4.4.1 `SCAN` matrix's spec corners
    (`scan[0][0] = 0`, `scan[7][7] = 63`), its zig-zag diagonal
    opening (`0, 1, 2, 3, 4, 5, 6, 7, 8, 9` mapping), and the
    `SCAN` / `INVERSE_SCAN` round-trip.

- Clean-room rebuild round 15: MPEG-1 (ISO/IEC 11172-2:1993) §2.4.4.2
  / §2.4.4.3 motion-vector reconstruction — the bridge from the round
  14 parser's `(code, r)` pairs to the integer `right_for / down_for`
  offsets the §2.4.4.2 pel-prediction equations consume.
  - `mpeg1_reconstruct::reconstruct(mv, ctx, &mut predictor,
    direction)` runs the §2.4.4.2 four-step formula (`r_size` / `f`
    derivation, `complement_*_r` derivation, `*_little` / `*_big`
    arithmetic, PMV update with wrap-around to `[-16*f, 16*f-1]`,
    then the optional `full_pel_*_vector << 1` shift on the recon
    output).
  - `Mpeg1Predictor { recon_right_prev, recon_down_prev }` carries
    the half-sample-unit PMV across consecutive predictive
    macroblocks. `Mpeg1Predictor::reset()` zeroes it at the
    start-of-slice and at the §2.4.4.2 ¶3 P-picture "no MV" case.
  - `mpeg1_reconstruct::reconstruct_zero(&mut predictor)` — the
    §2.4.4.2 ¶3 P-picture "no forward MV data" path: zeroes both
    the returned recon and the predictor.
  - `mpeg1_reconstruct::reconstruct_absent(ctx, &predictor)` — the
    §2.4.4.3 B-picture "no MV data" carry-over: recon = predictor
    unchanged (in contrast to the P-picture zero-reset).
  - `Mpeg1FrameMvContext { f_code, full_pel }` packages the picture
    header's `<dir>_f_code` and `full_pel_<dir>_vector` fields the
    reconstruction needs in addition to the parsed element.
  - `Mpeg1ReconstructedMv` exposes the §2.4.4.2 closing table's
    split: `(recon_right, recon_down)` plus the luminance
    (`right_for_luma = recon_right >> 1`, `right_half_for_luma`)
    and chrominance (`right_for_chroma = (recon_right / 2) >> 1`,
    `right_half_for_chroma = recon_right / 2 - 2 *
    right_for_chroma`) whole / half-pel pairs. Luma uses arithmetic
    `>>` (floored); chroma uses C-style `/` (truncated toward
    zero) — the spec's bit-exact divergence on negative values is
    preserved.
  - §2.4.4.2 conformance guards on `*_little != ±forward_f * 16`
    are enforced (both seam values rejected as
    `Error::InvalidBitstream`).
  - 23 new unit tests pinning every documented branch (`f_code`
    1/2 paths, complement zero / nonzero, positive / negative
    codes, PMV accumulation, wrap in both directions, `full_pel`
    shift, both seam guards, all input-validation sites, the
    P-picture zero-reset, the B-picture carry-over, and the luma
    / chroma half-pel split for both positive and negative
    `recon_*` values).
  - 2 new black-box integration tests
    (`tests/mpeg1_reconstruct_synthetic.rs`) against a
    hand-assembled two-macroblock bitstream — parse via
    `Mpeg1MotionVector::parse` then reconstruct end-to-end,
    asserting PMV propagation.
  - Re-exports `Mpeg1FrameMvContext`, `Mpeg1Predictor`,
    `Mpeg1ReconstructedMv`, `mpeg1_reconstruct`,
    `mpeg1_reconstruct_zero`, and `mpeg1_reconstruct_absent` at the
    crate root.
  - The §2.4.4.2 pel-prediction loop itself (the bilinear half-pel
    filter against `pel_past[][]` / `pel_future[][]`) is the
    next-round concern — it needs a reference-picture buffer the
    decoder doesn't yet allocate.
- Clean-room rebuild round 14: MPEG-1 (ISO/IEC 11172-2:1993)
  `motion_vector(s)` parser per §2.4.2.7 with the §2.4.3.6 field
  semantics, driven by Annex B Table B.4 (`motion_*_code` VLC). The
  parser lands the four-field wire shape (`motion_horizontal_*_code`,
  `motion_horizontal_*_r`, `motion_vertical_*_code`,
  `motion_vertical_*_r`) for the forward and backward directions
  selected by `Mpeg1MotionDirection`, with the residual gate
  `<dir>_f != 1 && motion_*_code != 0` and the
  `<dir>_r_size = <dir>_f_code - 1` width rule.
  - `mpeg1_motion_vector::Mpeg1MotionVector::parse(br, direction,
    f_code)` consumes the spec-mandated bits and returns a typed
    record with the bit cursor position immediately after the
    element.
  - `mpeg1_motion_vector::Mpeg1MotionDirection { Forward, Backward }`
    parameterises the parse on which `<dir>_f_code` /
    `full_pel_<dir>_vector` family applies.
  - Table B.4's 33 codeword → signed value rows are decoded via a
    new `pub(crate) motion_vector::match_motion_code` accessor that
    reuses the existing longest-first walker (MPEG-1 Table B.4 and
    MPEG-2 Annex B Table B-10 share the same numerical mapping
    row-for-row).
  - `f_code` range guard: `1..=7` per §2.4.3.4; zero is rejected as
    "forbidden", values `≥ 8` as outside the spec's range.
  - 20 new unit tests pinning every Table B.4 row to its tabulated
    bit width + signed value, plus the §2.4.3.6 presence matrix
    (residual gated on `f_code != 1 && code != 0`), the widest
    `r_size = 6` case for `f_code = 7`, mixed/zero/non-zero
    component combos, truncated-residual short-buffer detection,
    invalid-prefix detection, and the `Backward` direction tag.
  - Re-exports `Mpeg1MotionDirection`, `Mpeg1MotionVector` at the
    crate root.
  - §2.4.4.2 / §2.4.4.3 reconstruction (`recon_right_for_*` /
    `recon_down_for_*` with the `full_pel_*_vector` shift and the
    "right_little / right_big" wrap-around) is deferred to the next
    round, mirroring the MPEG-2 split between §6.2.5.2 (parser) and
    §7.6.3.1 (reconstruction).
- Clean-room rebuild round 13: §7.6.3.3 inter-vector PMV update
  (Tables 7-10 / 7-11), the once-per-macroblock pass that follows
  motion-vector reconstruction and propagates the `[r = 0]` PMV slot
  into the `[r = 1]` slot (or zeroes every slot) so prediction modes
  with fewer-than-maximum vectors still leave a sensible `PMV[1]`
  behind.
  - `pmv::update_predictors(&mut Pmv, PmvUpdateContext)` driving
    Tables 7-10 (frame pictures) and 7-11 (field pictures). The two
    tables share the right-hand "Predictors to Update" column
    row-for-row; the implementation branches on `picture_structure`
    + `prediction_type` to pick the right row family then applies
    the (fwd, bwd, intra) sub-cell.
  - Intra path covers both the `‡` row (`Frame-based`/`Field-based`
    assumed when `frame_motion_type` / `field_motion_type` is absent
    from the bitstream) and the `◊` footnote (when
    `concealment_motion_vectors == 0` the entire PMV is reset per
    §7.6.3.4 instead of copying `[0][0][1:0]` into `[1][0][1:0]`).
  - Non-intra Frame-based / Field-based / 16x8 MC / Dual-Prime row
    coverage including the `§` zero-motion footnote (PMV reset, only
    reachable in a P-picture) and the Dual-Prime forward-copy row.
  - Cells the spec leaves unreachable are rejected:
    `InvalidBitstream` for intra-with-motion-flag, Frame-based in a
    field picture, 16x8 MC in a frame picture, Dual-Prime with
    backward motion, Field-based / 16x8 with both motion flags zero,
    and non-intra without motion-type code.
  - `PmvUpdateOutcome` enum labels which row fired
    (`IntraConcealmentCopyForwardFirst`, `IntraResetAll`,
    `NonIntraCopyForward`, `NonIntraCopyBackward`,
    `NonIntraCopyBoth`, `NonIntraZeroMotionReset`, `NoUpdate`,
    `DualPrimeCopyForward`).
  - 18 new unit tests pinning every row of Tables 7-10 / 7-11 to a
    bit-exact PMV outcome, plus an end-to-end
    `reconstruct_motion_vector → update_predictors` chain
    confirming the reconstructed (3, -2) vector propagates from
    `PMV[0][0][:]` into `PMV[1][0][:]`.
  - Re-exports `update_predictors`, `PmvUpdateContext`,
    `PmvUpdateOutcome` at the crate root alongside the existing
    `Pmv` family.
- Clean-room rebuild round 1: parser for the MPEG-2 (ITU-T H.262 /
  ISO/IEC 13818-2) `sequence_header()` syntax element (§6.2.2.1)
  with field semantics from §6.3.3:
  - Start-code `0x000001B3` validation.
  - 12-bit horizontal/vertical size values (forbidden zero rejected).
  - 4-bit `aspect_ratio_information` decoded against Table 6-3.
  - 4-bit `frame_rate_code` (Table 6-4; forbidden zero rejected).
  - 18-bit `bit_rate_value` (forbidden zero rejected),
    `marker_bit` enforcement, 10-bit `vbv_buffer_size_value`,
    `constrained_parameters_flag`.
  - Optional `intra_quantiser_matrix[64]` and
    `non_intra_quantiser_matrix[64]` loads.
- Typed `Mpeg2SequenceHeader` and `AspectRatio` enums (re-exported
  at the crate root).
- 12 unit tests + 1 black-box integration test against an
  `ffmpeg`-produced 352×240 MPEG-2 elementary stream.
- Clean-room rebuild round 2: parser for `sequence_extension()`
  (§6.2.2.3) with field semantics from §6.3.5.
  - `extension_start_code` `0x000001B5` + 4-bit
    `extension_start_code_identifier == '0001'` Sequence Extension
    ID (Table 6-2) validation.
  - 8-bit `profile_and_level_indication` (raw byte),
    `progressive_sequence` flag, 2-bit `chroma_format`
    decoded against Table 6-5 (reserved `00` rejected).
  - 2-bit `horizontal_size_extension`, 2-bit
    `vertical_size_extension`, 12-bit `bit_rate_extension`,
    `marker_bit` enforcement, 8-bit `vbv_buffer_size_extension`,
    `low_delay`, 2-bit `frame_rate_extension_n`, 5-bit
    `frame_rate_extension_d`.
- Composed-view helper `Mpeg2Sequence::from_buf` that parses a
  `sequence_header()` + `next_start_code()` zero-byte stuffing
  (§5.2.3) + `sequence_extension()` pair and synthesises the
  14-bit `horizontal_size`, 14-bit `vertical_size`, 30-bit
  `bit_rate`, and 18-bit `vbv_buffer_size`. Composite
  `bit_rate == 0` rejected per §6.3.3.
- Typed `Mpeg2SequenceExtension`, `ChromaFormat`, and
  `Mpeg2Sequence` (re-exported at the crate root).
- 12 new unit tests + 2 new black-box integration tests against
  the existing 352×240 fixture (extension decode + composed
  `Mpeg2Sequence` round-trip).
- Clean-room rebuild round 3: parser for
  `group_of_pictures_header()` (§6.2.2.6) with field semantics
  from §6.3.8.
  - `group_start_code` = `0x000001B8` validation.
  - 25-bit `time_code` decomposition per Table 6-11:
    1-bit `drop_frame_flag`, 5-bit `time_code_hours` (0..=23),
    6-bit `time_code_minutes` (0..=59), 1-bit `marker_bit`
    enforcement, 6-bit `time_code_seconds` (0..=59), 6-bit
    `time_code_pictures` (0..=59).
  - 1-bit `closed_gop` and 1-bit `broken_link` flags.
- Typed `Mpeg2Gop` + `TimeCode` (re-exported at the crate root).
- 11 new unit tests + 1 new black-box integration test against
  the existing 352×240 fixture (locates the GOP start code,
  decodes the time-code, asserts `closed_gop = 1`,
  `broken_link = 0`).
- Clean-room rebuild round 4: parser for `picture_header()`
  (§6.2.3) with field semantics from §6.3.10.
  - `picture_start_code` = `0x00000100` validation.
  - 10-bit `temporal_reference`, 3-bit `picture_coding_type`
    decoded against Table 6-12 (forbidden / D-picture /
    reserved codes rejected), 16-bit `vbv_delay`.
  - Conditional 1-bit `full_pel_forward_vector` + 3-bit
    `forward_f_code` for P / B pictures.
  - Conditional 1-bit `full_pel_backward_vector` + 3-bit
    `backward_f_code` for B pictures.
  - `extra_information_picture` byte loop captured as raw
    `Vec<u8>` (empty for every conforming MPEG-2 stream).
- Parser for `picture_coding_extension()` (§6.2.3.1) with field
  semantics from §6.3.11.
  - `extension_start_code` `0x000001B5` + 4-bit Picture Coding
    Extension ID `1000` (Table 6-2) validation.
  - Four 4-bit `f_code[s][t]` sub-fields with the §6.3.11
    forbidden-zero guard.
  - 2-bit `intra_dc_precision` (Table 6-13) and 2-bit
    `picture_structure` (Table 6-14; reserved `00` rejected).
  - 10 trailing single-bit flags from `top_field_first` through
    `composite_display_flag`.
- Composed-view helper `Mpeg2PictureHeader::parse_with_extension`
  that parses `picture_header()` + `next_start_code()`
  zero-byte stuffing (§5.2.3) + `picture_coding_extension()`.
- Typed `Mpeg2PictureHeader`, `PictureCodingType`,
  `PictureCodingExtension`, `PictureStructure` (re-exported at
  the crate root); `PICTURE_START_CODE` /
  `PICTURE_CODING_EXTENSION_ID` constants.
- 18 new unit tests + 2 new black-box integration tests against
  the existing 352×240 fixture (picture-header parse +
  picture_header + picture_coding_extension composition).
- Clean-room rebuild round 5: parser for the `slice()` header bits
  (§6.2.4) with field semantics from §6.3.16.
  - 32-bit `slice_start_code`: 24-bit prefix `0x000001` + 8-bit
    `slice_vertical_position` validated against the Table 6-1
    range `0x01..=0xAF`.
  - Optional 3-bit `slice_vertical_position_extension`, present
    iff the caller's `SliceContext::vertical_size` is `> 2800`
    (§6.2.4); §6.3.16's stricter `svp ∈ [1:128]` constraint
    enforced when the extension is present.
  - Optional 7-bit `priority_breakpoint`, gated on the caller's
    `SliceContext::priority_breakpoint_present` (caller derives
    from `sequence_scalable_extension()`, not yet parsed by this
    crate).
  - 5-bit `quantiser_scale_code` with the spec-defined
    forbidden-zero check (§6.3.16).
  - Optional intra-slice prelude: `intra_slice_flag` /
    `intra_slice` / 7-bit `reserved_bits` (enforced `== 0`) plus
    the `extra_information_slice` byte loop driven by the same
    `nextbits() == '1'` mechanism used for
    `extra_information_picture`. Final `extra_bit_slice` bit
    enforced `== '0'`.
  - `SliceHeader::mb_row()` helper computes `mb_row` per
    §6.3.16: `(extension << 7) + svp - 1` when the extension is
    present, `svp - 1` otherwise.
  - `body_bit_position` field gives the bit offset (from the start
    of the slice buffer) at which the first `macroblock()` begins
    — the macroblock body itself is **not** in scope for round 5.
- Typed `SliceHeader` + `SliceContext` (re-exported at the crate
  root) plus `SLICE_VERTICAL_POSITION_MIN` /
  `SLICE_VERTICAL_POSITION_MAX` constants.
- 14 new unit tests covering every spec-defined rejection site
  (wrong prefix, svp = 0, svp = 0xB0, svp > 128 with extension,
  zero `quantiser_scale_code`, non-zero `reserved_bits`,
  truncated buffer) plus the intra-slice prelude /
  `extra_information_slice` round-trip and bit-position
  accounting.
- 2 new black-box integration tests against the existing 352×240
  fixture (first slice header at offset `0x2e`,
  slice-start-code multiplicity sanity check).
- Clean-room rebuild round 6: parser for
  `macroblock_address_increment` (§6.2.5) with field semantics
  from §6.3.17.1 and the Annex B Table B-1 VLC.
  - 33 increment-value codewords (`1..=33`) walked MSB-first
    against the spec's tabulated bit-strings (1-, 3-, 4-, 5-, 7-,
    8-, 10-, and 11-bit code groups).
  - `macroblock_escape` (`0000 0001 000`, 11 bits) consumption
    with the §6.3.17.1 "add 33 per escape" chain rule (caller
    receives the consumed escape count for spec-bound validation
    against `mb_width × (mb_height − mb_row)`).
  - Optional MPEG-1 `macroblock_stuffing` (`0000 0001 111`)
    recognition, gated on `MbAddressIncrementContext::mpeg1` per
    ISO/IEC 11172-2:1993 §D.5.5.1. MPEG-2 streams (the default
    context) reject the stuffing code as a §6.3.17.1 violation;
    stuffing after an escape is rejected in both contexts.
  - Typed `MbAddressIncrement { value, escape_count,
    stuffing_count, bit_position_after }` (re-exported at the
    crate root) plus the `MbAddressIncrementContext` helper
    (`mpeg1()` / `mpeg2()`).
- 16 new unit tests covering every Table B-1 entry (parsed
  individually), the escape chain (`33 + N`, `66`, `70` worked
  example from §D.5.5.2), spec-mandated stuffing rejection in
  MPEG-2, MPEG-1 stuffing acceptance, stuffing-after-escape
  rejection, garbage-prefix rejection, truncated-buffer
  handling, bit-position accounting, and Table B-1 internal
  invariants (33 values + escape + stuffing, no width-collisions,
  every code fits its declared bit width).
- 2 new black-box integration tests against the existing 352×240
  fixture: parses the first `macroblock_address_increment`
  immediately after the first slice header (expected `value = 1`,
  the single bit `'1'`) and confirms the fixture does not emit
  the MPEG-1-only stuffing code.
- Clean-room rebuild round 7: parser for `macroblock_type` — the
  leading VLC of `macroblock_modes()` (§6.2.5.1) with field
  semantics from §6.3.17.1 and the non-scalable Annex B
  Tables B-2 / B-3 / B-4.
  - Table selection by `picture_coding_type` per Table 6-10
    (no `sequence_scalable_extension()`): B-2 for I-pictures,
    B-3 for P-pictures, B-4 for B-pictures.
  - All 2 + 7 + 11 codewords walked MSB-first longest-first,
    decoding the six §6.3.17.1 derived flags `macroblock_quant`,
    `macroblock_motion_forward`, `macroblock_motion_backward`,
    `macroblock_pattern`, `macroblock_intra`, and
    `spatial_temporal_weight_code_flag` (always `0` for the
    non-scalable tables).
  - The rest of `macroblock_modes()`
    (`spatial_temporal_weight_code`, `frame_motion_type` /
    `field_motion_type`, `dct_type`) and the macroblock body
    remain out of scope.
- Typed `MacroblockType { macroblock_quant,
  macroblock_motion_forward, macroblock_motion_backward,
  macroblock_pattern, macroblock_intra,
  spatial_temporal_weight_code_flag, bit_position_after }`
  (re-exported at the crate root).
- 13 new unit tests covering every Table B-2 / B-3 / B-4 row
  (parsed individually), longest-first prefix disambiguation,
  unknown-codeword and truncated-buffer rejection, and per-table
  invariants (codes fit their declared width, every table is
  prefix-free, row counts match Annex B).
- 2 new black-box integration tests against the existing 352×240
  fixture: the first I-picture macroblock's `macroblock_type`
  decodes to plain `Intra` (single-bit Table B-2 code `'1'`) and
  advances the cursor by exactly one bit past the increment.
- Clean-room rebuild round 8: parser for `coded_block_pattern()`
  (§6.2.5.3) with field semantics from §6.3.17.4 and the Annex B
  Table B-9 variable-length codes.
  - All 64 `coded_block_pattern_420` codewords (3- to 9-bit)
    walked MSB-first longest-first against the spec's tabulated
    bit-strings, decoding to the 6-bit `cbp` (0..=63).
  - 4:2:2 / 4:4:4 chroma extensions: 2-bit `coded_block_pattern_1`
    and 6-bit `coded_block_pattern_2` fixed-length codes read when
    the caller-supplied `chroma_format` selects them.
  - `CodedBlockPattern::pattern_code(macroblock_intra,
    macroblock_pattern)` derives the 12-entry `pattern_code[i]`
    array per §6.3.17.4 (intra-default all-ones, then `cbp` /
    `coded_block_pattern_1` / `coded_block_pattern_2` masking).
  - Typed `CodedBlockPattern { cbp, coded_block_pattern_1,
    coded_block_pattern_2, bit_position_after }` (re-exported at
    the crate root).
- 19 new unit tests covering every Table B-9 row (parsed
  individually), all-64-cbp coverage, longest-first prefix
  disambiguation, the 4:2:2 / 4:4:4 extensions, the §6.3.17.4
  `pattern_code` derivation, unknown-codeword / truncated-buffer
  rejection, and table invariants (prefix-free, widths fit,
  64 rows).
- 2 new black-box integration tests against the existing 352×240
  fixture: the first I-picture macroblock is plain `Intra`
  (`macroblock_pattern = 0`, so no `coded_block_pattern()` per
  §6.2.5.3), and the fixture's chroma is pinned to 4:2:0 before
  decoding a Table B-9 codeword against it.
- Clean-room rebuild round 9: parser for the macroblock-layer
  `quantizer_scale` per ISO/IEC 11172-2:1993 (MPEG-1 Video)
  §2.4.2.7 (syntax) with field semantics from §2.4.3.6. Fills the
  bitstream gap between round 7's `macroblock_type` and round 8's
  `coded_block_pattern()` — `quantizer_scale` is read immediately
  after `macroblock_type`, conditional on the `macroblock_quant`
  flag the type carries.
  - When `macroblock_quant` is set, a 5-bit `quantizer_scale` is
    read and validated against the §2.4.3.6 range `1..=31` (the
    value `0` is forbidden).
  - When `macroblock_quant` is clear the field is absent: zero bits
    are read and `quantizer_scale = None` is returned, matching the
    §2.4.3.6 persistence rule (the decoder keeps the value
    established at the slice layer or a prior macroblock).
  - `QuantizerScale::parse_after_type(br, &MacroblockType)`
    convenience threads the flag straight from a decoded
    `macroblock_type`.
  - Typed `QuantizerScale { quantizer_scale, bit_position_after }`
    (re-exported at the crate root) plus `QUANTIZER_SCALE_MIN` /
    `QUANTIZER_SCALE_MAX` constants.
- 12 new unit tests covering the present / absent branches, every
  legal value `1..=31`, forbidden-zero rejection, truncated- and
  empty-buffer handling on both branches, `parse_after_type`
  flag-threading for both flag states, the bound constants, and
  bit-position accounting.
- 2 new black-box integration tests against the existing 352×240
  fixture: the first I-picture macroblock is plain `Intra`
  (`macroblock_quant = 0`), so per §2.4.2.7 it carries no
  `quantizer_scale` and the parser consumes zero bits; a spliced
  `macroblock_quant`-set `macroblock_type` then decodes a synthetic
  5-bit `quantizer_scale` with correct value and bit accounting.
- Clean-room rebuild round 10: parser for the remainder of
  `macroblock_modes()` after `macroblock_type` per ISO/IEC 13818-2
  (ITU-T H.262) §6.2.5.1 with field semantics from §6.3.17.1 /
  §6.3.17.2 and the meaning Tables 6-17, 6-18 and 6-19. Closes
  `macroblock_modes()`; `motion_vectors()` and the block loop stay
  out of scope.
  - 2-bit `frame_motion_type` (Table 6-17) / `field_motion_type`
    (Table 6-18) decoding, surfacing the derived `prediction_type`,
    `motion_vector_count`, `mv_format`, and `dmv`; reserved code
    `00` rejected; the two `Field-based` rows of Table 6-17 split by
    a caller-supplied `spatial_temporal_weight_class`.
  - §6.2.5.1 presence gates honoured: the motion-type code is read
    only when a motion flag is set (omitted in frame pictures when
    `frame_pred_frame_dct == 1`), and `dct_type` only when
    `picture_structure == frame`, `frame_pred_frame_dct == 0`, and
    the macroblock is intra or has a coded pattern. Absent fields
    read zero bits.
  - `spatial_temporal_weight_code` is not read (scalable-only,
    `spatial_temporal_weight_code_flag` is always `0` for the
    non-scalable tables); a `mb_type` claiming the flag is rejected.
  - Typed `MacroblockModesTail { motion_type, dct_type,
    bit_position_after }`, `MotionType`, `PredictionType`,
    `MvFormat`, and `MacroblockModesContext` (re-exported at the
    crate root).
- 21 new unit tests covering every Table 6-17 / 6-18 row, the
  per-class `Field-based` vector-count split, reserved-code
  rejection, the motion-type / `dct_type` presence matrix, the
  scalable-flag rejection, zero-bit absent paths, and truncated-
  buffer handling.
- 2 new black-box integration tests against the existing 352×240
  fixture: a full slice → increment → type → quantizer_scale →
  `macroblock_modes()` tail chain on the first I-picture macroblock
  (motion-type absent; `dct_type` presence keyed to the fixture's
  own `frame_pred_frame_dct`), plus a spliced P-picture frame
  macroblock decoding `frame_motion_type` + `dct_type` with exact
  bit accounting.

- Clean-room rebuild round 11: parsers for the `motion_vectors(s)`
  wrapper per ISO/IEC 13818-2 (ITU-T H.262) §6.2.5.2 and the inner
  `motion_vector(r, s)` per §6.2.5.2.1, with field semantics from
  §6.3.17.2 / §6.3.17.3 and the Annex B Tables B-10 (`motion_code`)
  and B-11 (`dmvector`). The numerical reconstruction of
  `vector'[r][s][t]` (§7.6.3.1, PMV state machine, wrap-around) stays
  out of scope.
  - Table B-10 `motion_code`: all 33 codewords (`-16..=+16`) walked
    MSB-first longest-first, 1-/3-/4-/5-/7-/8-/10-/11-bit groups,
    prefix-free.
  - Table B-11 `dmvector[t]`: the 1-/2-bit `{0, +1, -1}` VLC.
  - Fixed-length `motion_residual[r][s][t]` read iff `f_code != 1 &&
    motion_code != 0`, width `r_size = f_code - 1` (1..=8 bits);
    `f_code` outside §6.3.11's `1..=9` range rejected when a residual
    would otherwise drive the cursor.
  - `motion_vertical_field_select[r][s]` flag honoured per §6.2.5.2 —
    suppressed when `motion_vector_count == 1 && (mv_format == frame
    || dmv == 1)`, present otherwise (both rows when count == 2).
  - Typed `MotionVector`, `MotionVectorEntry`, `MotionVectors`,
    `MotionVectorsContext`, `MotionVectorsKind` (re-exported at the
    crate root). `MotionVectors::parse(br, kind, &MotionType, &ctx)`
    threads a parsed `frame_motion_type` / `field_motion_type` (round
    10) and the `f_code[s][t]` matrix straight through.
- 29 new unit tests covering every Table B-10 row (parsed
  individually), the +16 / -16 extremes, the 33-unique-values
  invariant, prefix-freeness and width-fitting, unknown-prefix /
  truncated-buffer rejection on both VLC tables, Table B-11's three
  values plus its truncated-second-bit short case, the
  `motion_vector(r, s)` presence-matrix (no residual on f_code = 1 or
  motion_code = 0, residual width = `f_code - 1`, dmvector suppressed
  when `dmv = 0`), out-of-range `f_code` rejection, all four
  `motion_vectors(s)` shapes (frame count-1 / field count-1 /
  dual-prime count-1 / count-2), the Forward / Backward `f_code` pair
  selection, `motion_vector_count` validation, and truncated-VFS-/
  truncated-motion-code short paths.
- 2 new black-box integration tests against the existing 352×240
  fixture: the first I-picture is plain `Intra` so per §6.2.5.2 no
  `motion_vectors()` element exists (the fixture's f_codes are pinned
  to the §6.3.11 "unused" sentinel `15`), and a spliced P-picture
  frame macroblock prefix that drives the full `macroblock_type` →
  `frame_motion_type` → `dct_type` → `motion_vectors(0)` chain
  (`motion_code = -1`, `motion_residual = 1` with `f_code = 2`,
  `motion_code_vert = 0`) and asserts the 9-bit total cursor
  accounting.

- Clean-room rebuild round 12: motion-vector reconstruction per
  ISO/IEC 13818-2 (Recommendation ITU-T H.262) §7.6.3.1 plus
  §7.6.3.4 reset rules and §7.6.3.7 chroma scaling. The bridge from
  round 11's parsed `motion_code` / `motion_residual` / `dmvector`
  fields to the spec's `vector'[r][s][t]` reconstructed luminance
  motion vector.
  - `compute_delta(motion_code, motion_residual, f_code)` derives
    `delta` per the spec formula (`f = 1 << (f_code - 1)`, shortcut
    `delta = motion_code` when `f == 1 || motion_code == 0`,
    otherwise `sign(motion_code) * ((|motion_code| - 1) * f +
    motion_residual + 1)`).
  - `vector_range(f_code)` returns `(low, high, range) =
    (-16*f, 16*f - 1, 32*f)`. Doubles per Table 7-8 entry across
    `f_code ∈ {1..=9}`; out-of-range `f_code` rejected.
  - `reconstruct_component(motion_code, motion_residual, f_code,
    prior_pmv, mv_format, picture_structure, t)` runs the §7.6.3.1
    procedure for one component: half-pred for the
    `(mv_format == field && t == vertical && picture_structure ==
    frame)` case using §4.3 floor-division (`div_euclid`), `vector' =
    prediction + delta`, wrap into `[low, high]` via `± range`, and
    `new_pmv = vector' * 2` for the half-pred case else `vector'`.
    Bitstream conformance per §7.6.3.2 (`delta`, `vector'`, new PMV
    all in range) enforced as a parse-time invariant.
  - `reconstruct_motion_vector(pmv, &MotionVector, r, s, f_code_h,
    f_code_v, mv_format, picture_structure)` chains the two
    components and writes the new PMVs back into the supplied
    `Pmv` slot.
  - `Pmv { values: [[[i32; 2]; 2]; 2] }` carries the four
    `PMV[r][s][t]` predictors in half-sample units (Table 7-7) with
    `Pmv::get` / `set` typed by `VectorIndex` / `Direction` /
    `Component`. `Pmv::reset()` for §7.6.3.4 (slice start /
    non-concealment intra / P-picture non-intra without forward /
    P-skipped). `Pmv::default()` and `Pmv::new()` zero every slot.
  - `scale_chroma(luma_horiz, luma_vert, ChromaFormat) ->
    ScaledMotionVector` per §7.6.3.7: 4:2:0 halves both components,
    4:2:2 halves only horizontal, 4:4:4 is identity. Toward-zero
    integer division per §4.3.
  - Typed `Pmv`, `ReconstructedComponent { vector_prime, new_pmv,
    delta, range }`, `ScaledMotionVector { luma_horiz, luma_vert,
    chroma_horiz, chroma_vert }`, `Component { Horizontal,
    Vertical }`, `Direction { Forward, Backward }`,
    `VectorIndex { First, Second }` (re-exported at the crate root).
- 29 new unit tests covering: `compute_delta`'s shortcut and
  full-formula branches across `f_code ∈ {1..=9}` and motion-code
  signs, `motion_residual` presence-required / presence-forbidden
  rejection, out-of-range `f_code` rejection (0, 10, 15),
  `vector_range` invariants (f_code=1 ⇒ ±16, f_code=9 ⇒ ±4096,
  range doubles per step), end-to-end `reconstruct_motion_vector`
  with no-wrap / wrap-low / wrap-high paths, vertical-half-pred
  matrix (frame vs field picture, horizontal vs vertical component),
  floor-division for negative PMV under half-pred, PMV slot
  independence for `(r = 0 vs r = 1)` and `(forward vs backward)`,
  delta-outside-range rejection, `Pmv::default` / `Pmv::reset` zero
  every slot, chroma scaling for all three `ChromaFormat` values
  plus toward-zero rounding on negative odd inputs, and Table 7-7
  index enums.
- 2 new black-box integration tests against the existing 352×240
  fixture: confirms the fixture's I-picture is the §7.6.3 "PMV
  unused" case (every f_code is the `15` sentinel; PMV stays zero
  after the §7.6.3.4 reset), and a spliced two-macroblock
  P-picture chain (`motion_code = +2, residual = 0` then
  `motion_code = -1, residual = 0`, both with `f_code = 2`)
  decodes through `MotionVector → reconstruct_motion_vector`
  with PMV state evolving from `0 → 3 → 2` (second `delta = -1`
  added on top of the first vector's predictor `PMV = 3`), and the
  4:2:0 chroma scaling halves both components.

### Erased

- Prior master history was force-erased on **2026-05-18** under
  Hat-3 cold enforcement of the workspace clean-room policy
  (`docs/IMPLEMENTOR_ROUND.md`).

### Reset

- Crate reduced to a minimal `oxideav_core::register!` stub. Every
  public API returns `Error::NotImplemented`. The crates.io version
  (`0.0.11`) is preserved on the new master to avoid breaking
  downstream version pins; the published versions on crates.io will
  be yanked by the maintainer.

### Next

- `quant_matrix_extension()` (§6.2.3.2),
  `picture_display_extension()` (§6.2.3.3),
  `sequence_display_extension()` (§6.2.2.4),
  `sequence_scalable_extension()` (§6.2.2.5).
- Composite-display sub-fields (`v_axis` / `field_sequence` /
  `sub_carrier` / `burst_amplitude` / `sub_carrier_phase`) inside
  `picture_coding_extension()` when `composite_display_flag` is 1.
- §7.6.3.3 inter-vector PMV-copy table (Tables 7-9 / 7-10) — the
  macroblock-loop driver's responsibility once that lands.
- §7.6.3.6 dual-prime additional arithmetic (deriving the
  opposite-parity vector from the decoded forward vector).
- §7.6.3.9 concealment motion vectors.
- Residual block VLC tables (B-12 .. B-16) plus IDCT and motion
  compensation.
- The scalable `macroblock_type` Tables B-5 .. B-8 once
  `sequence_scalable_extension()` parsing lands.
- `oxideav_core::Decoder` wiring once a complete picture round-trips.
