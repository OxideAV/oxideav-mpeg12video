# oxideav-mpeg12video

Clean-room MPEG-1 Video (ISO/IEC 11172-2) and MPEG-2 Video
(ITU-T H.262 / ISO/IEC 13818-2) decode building blocks for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.
Pure Rust, no C dependencies.

## Status

Clean-room rebuild. The crate implements the full MPEG-1 and MPEG-2
video decode pipeline as a set of composable, per-stage public modules
covering the bitstream-parsing surface and the pixel-reconstruction
math. It is not yet wired into the runtime codec registry — `register`
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
  MPEG-2 dual-prime (§7.6.3.6).
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
- **Block / macroblock drivers**: `mpeg2_block_decoder::decode_block`
  chains DC prelude → residual VLC → inverse scan → inverse quant →
  IDCT into a single bitstream→plane entry point, and
  `mpeg2_macroblock_blocks::decode_macroblock_blocks` walks the
  `pattern_code[]` array per the 4:2:0 / 4:2:2 / 4:4:4 layouts.

Each stage is covered by synthetic unit tests plus integration fixtures
verifying the parsers against known-good encoded streams.

## Not yet supported

- Runtime registration (`register` is a no-op).
- A single top-level frame-decode / encode entry point; the pipeline is
  driven through the per-stage module APIs.
- Scalability profiles and the spatial/temporal/SNR enhancement layers
  (parsed structurally, decode not composed).

## Clean-room provenance

Every line in `src/` traces to the ISO/IEC 13818-2:1995 (ITU-T H.262)
and ISO/IEC 11172-2:1993 specification PDFs staged under `docs/video/`,
plus `oxideav-core`'s `BitReader` API. An external CLI binary is used
**only** as an opaque encoder to produce integration-test fixtures; its
source code was not consulted. No external library source was read,
quoted, or paraphrased.

## License

MIT — see [LICENSE](LICENSE).
