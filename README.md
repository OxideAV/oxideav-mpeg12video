# oxideav-mpeg12video

Clean-room MPEG-1 Video (ISO/IEC 11172-2) and MPEG-2 Video
(ITU-T H.262 / ISO/IEC 13818-2) decode building blocks for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.
Pure Rust, no C dependencies.

## Status

Clean-room rebuild. The crate implements the full MPEG-1 and MPEG-2
video decode pipeline as a set of composable, per-stage public modules
covering the bitstream-parsing surface and the pixel-reconstruction
math, topped by a `video_sequence()` driver
(`decode_video_sequence`) that decodes a whole elementary stream of
**frame** pictures into reconstructed frames in §6.1.1.11 display order.
It is not yet wired into the runtime codec registry — `register`
is a no-op placeholder, so the codec is consumed today through its
direct module APIs rather than `oxideav_core::make_decoder`.

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
  forward copy or a B-picture inherited-direction prediction. The
  MPEG-1 (ISO/IEC 11172-2) `recon_right`/`recon_down` half-sample
  vectors bridge into the same MC core via `MotionVectorPel::from_mpeg1`
  / `FrameMotion::from_mpeg1`. Frame-picture **frame-based** and
  **field-based** prediction are both driven end-to-end: the field-based
  path (Table 7-14 `Field-based` rows) predicts the macroblock's even
  (top-field) frame lines from the top reference field with the
  top-field vector and its odd lines from the bottom field with the
  bottom-field vector, via the §7.6.4 `FieldReference` half-height field
  view (`predict_field_block`; field line `k` → frame row `2k + parity`,
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
  end-to-end; the open work is the GOP-level reference-management /
  picture-reordering loop and runtime registration.
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
- **Block / macroblock drivers**: `mpeg2_block_decoder::decode_block`
  chains DC prelude → residual VLC → inverse scan → inverse quant →
  IDCT into a single bitstream→plane entry point, and
  `mpeg2_macroblock_blocks::decode_macroblock_blocks` walks the
  `pattern_code[]` array per the 4:2:0 / 4:2:2 / 4:4:4 layouts.

Each stage is covered by synthetic unit tests plus integration fixtures
verifying the parsers against known-good encoded streams.

## Not yet supported

- Runtime registration (`register` is a no-op).
- A top-level **`video_sequence()` decode loop** now exists
  (`decode_video_sequence(stream) -> Vec<DecodedFrame>`): it parses the
  sequence layer once for the geometry, walks every `picture_start_code`,
  dispatches each frame picture to the matching per-picture driver with
  the running §7.6 anchor pair, and reorders the reconstructed frames
  into **display order** per §6.1.1.11 (B-frames pass through, I/P frames
  held back one). It covers **frame pictures** (the MPEG-2 common case +
  MPEG-1 entirely) **and field-picture pairs** (§6.1.1.4.1): each field
  picture is reconstructed by `decode_field_picture`, the first field of
  a pair is held until its partner arrives, and the two are interleaved
  into one full-height frame by `assemble_frame_from_fields` (§3.131
  top→even lines / §3.13 bottom→odd lines), with the §7.6.2.1
  second-field-of-a-P-frame reference rule honoured via a synthetic
  reference frame pairing the current first field with the previous
  frame's opposite-parity field. Downloadable `quant_matrix_extension()`
  matrices / the scalable layers
  are skipped by the start-code scan (threaded by a later round). The
  per-picture module APIs remain available directly
  (`decode_intra_picture` for I-pictures, `decode_inter_picture` for
  frame-picture P/B, `decode_field_picture` for field-picture P/B),
  with the caller supplying the decoded reference frame(s). All §7.6
  motion-compensation prediction modes are now driven end-to-end: the
  frame-picture frame-based **and** field-based P/B reconstruction, the
  field-picture **simple field prediction**, the field-picture **16×8
  motion compensation**, the **dual-prime** four-/two-field reconstruction
  in both frame and field pictures (§7.6.3.6 / §7.6.7.4), and the
  field-picture **B-field skipped-macroblock** §7.6.6.3 direction
  inheritance. (Frame-picture 16×8-MC does not exist — §7.6 restricts 16×8
  MC to field pictures.)
- Scalability profiles and the spatial/temporal/SNR enhancement layers
  (parsed structurally; the §7.7.3.4 deinterlace, §7.7.3.5/.6 lower-layer
  resampling, §7.7.3.7 reinterlace, the §7.7.3.1 / Table 7-15 upsampling-
  case dispatch + `upsample_spatial_prediction` driver, the §7.7.4
  spatial/temporal prediction combination, and §7.7.5.1 PMV reset are all
  implemented, but the full enhancement-layer decode loop that drives
  them per macroblock — deriving the Table 7-16 `ResampleParams` from the
  sequence/picture geometry and threading the per-macroblock
  `spat_pred_pic` into the §7.7.4 combiner across a whole picture — is not
  yet composed).

## Clean-room provenance

Every line in `src/` traces to the ISO/IEC 13818-2:1995 (ITU-T H.262)
and ISO/IEC 11172-2:1993 specification PDFs staged under `docs/video/`,
plus `oxideav-core`'s `BitReader` API. An external CLI binary is used
**only** as an opaque encoder to produce integration-test fixtures; its
source code was not consulted. No external library source was read,
quoted, or paraphrased.

## License

MIT — see [LICENSE](LICENSE).
