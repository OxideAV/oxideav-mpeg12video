# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- MPEG-2 encoder: 4:2:2 and 4:4:4 chroma format support — `Yuv422P` /
  `Yuv444P` input maps to the corresponding MPEG-2 `chroma_format` code in
  `sequence_extension`. Block counts are 8 / 12 per MB for 4:2:2 / 4:4:4;
  extended CBP is emitted per H.262 §6.3.17.4 (2 raw bits for 4:2:2,
  2 + 6 raw bits for 4:4:4); chroma MC is MV-scaled correctly via
  `scale_mv_h_to_chroma` / `scale_mv_v_to_chroma`. `make_encoder_mpeg2`
  selects the chroma format automatically from `params.pixel_format`.
- MPEG-2 encoder: interlaced frame encoding — `make_encoder_mpeg2_interlaced`
  emits `progressive_frame = 0` and `frame_pred_frame_dct = 0` in every
  `picture_coding_extension`. Each intra MB writes a `dct_type = 1` bit and
  uses field-DCT: luma rows are split into top-field (even rows 0,2,…,14)
  and bottom-field (odd rows 1,3,…,15); each field half feeds a separate
  8×8 DCT block. Reconstruction interleaves the IDCT output back into the
  correct frame rows. Chroma blocks always use frame-DCT regardless of
  `dct_type` (H.262 §6.3.17.1).
- MPEG-2 encoder: I+P GOP support — `make_encoder_mpeg2_with_gop` now accepts
  `gop_size > 1` with `num_b_frames = 0`, enabling true I+P bitstreams at
  any GOP length. Previously only I-only (`gop_size = 1`) was permitted.
- New encoder factory `make_encoder_mpeg2_interlaced(params, gop_size)` for
  interlaced content.
- Tests: `mpeg2_422_iframe_round_trip`, `ffmpeg_decodes_mpeg2_422_output`,
  `mpeg2_444_iframe_round_trip`, `ffmpeg_decodes_mpeg2_444_output`,
  `mpeg2_interlaced_iframe_round_trip`, `ffmpeg_decodes_mpeg2_interlaced_output`,
  `mpeg2_ip_long_gop_round_trip` — all using self-roundtrip + optional
  ffmpeg cross-validation (skips when ffmpeg unavailable).

### Changed

- `make_encoder_mpeg2_with_gop` no longer rejects `gop_size > 1`; only
  `num_b_frames != 0` is rejected.
- Test `mpeg2_encoder_rejects_b_frames_and_long_gop` renamed to
  `mpeg2_encoder_rejects_b_frames` and the long-GOP assertion removed.
- `encode_frames` helper in the MPEG-2 test file now uses `gop_size = 1`
  explicitly (via `make_encoder_mpeg2_with_gop`) to preserve the I-only
  start-code census assertions.

### Added (decoder — see previous milestone notes)

- MPEG-2 decoder: 4:2:2 and 4:4:4 chroma format support — `chroma_format`
  in `sequence_extension` selects between 6 / 8 / 12 blocks per
  macroblock, the chroma planes are sized accordingly (full vertical for
  4:2:2 / 4:4:4, full horizontal for 4:4:4), and chroma motion vectors
  are scaled per-format. Output `VideoFrame` chroma plane stride matches
  the format (was hard-coded to width / 2 before).
- MPEG-2 decoder: interlaced frame pictures
  (`progressive_frame = 0` and / or `progressive_sequence = 0` with
  `picture_structure = frame`). Includes `frame_pred_frame_dct = 0` mode
  with per-MB `dct_type` field-DCT row permutation (luma rows split into
  top-field rows 0,2,…,14 and bottom-field rows 1,3,…,15; chroma rows
  split for 4:2:2 / 4:4:4).
- MPEG-2 decoder: field motion vectors in frame pictures
  (`frame_motion_type = Field`). Decoder reads the
  `motion_vertical_field_select` parity bit and the two field MVs per
  direction, then composes the 16×16 luma + matching chroma prediction
  by interleaving two 16×8 patches fetched from the reference field of
  the chosen parity.
- MPEG-2 decoder: dual-prime motion vectors
  (`frame_motion_type = DualPrime`, H.262 §6.3.17.2 / §7.6.3.6). Decoder
  consumes the single transmitted MV + 2-component `dmvector[]` per axis
  and forms the averaged parity-pair prediction.
- MPEG-2 decoder: concealment motion vectors are now consumed and
  discarded (intra MBs in I/P pictures when
  `concealment_motion_vectors = 1`). Previously such streams were
  rejected with `Error::Unsupported`.
- MPEG-2 motion-vector decoder (`motion::decode_motion_component_mpeg2`)
  per H.262 §7.6.3.1 — modulo-wrap range `[-16f, 16f-1]`, no `full_pel`
  flag, `f_code = 15` is the "axis unused" sentinel for I-pictures.
- New chroma-format helpers in `picture::ChromaFormat` (`from_code`,
  `to_code`, `blocks_per_mb`, `chroma_h_shift`, `chroma_v_shift`,
  `output_pixel_format`).
- Tests: `tests/mpeg2_extended.rs` cross-validates 4:2:2 I/P-frame
  decode, interlaced frame-picture decode (frame-DCT and field-DCT) and
  concealment-MV parser sync against ffmpeg-generated fixtures (skipped
  silently when ffmpeg is unavailable).

### Changed

- The MPEG-2 decoder's picture-buffer allocation moved from
  picture-header parse time to first-slice time so the parsed
  `picture_coding_extension` can drive the `chroma_format` selection.

### Deferred

- Field pictures (`picture_structure ∈ {TopField, BottomField}`) are
  still rejected — they need a different reference-buffer model that
  pairs two coded pictures into a single output frame.
- `intra_vlc_format = 1` (Table B-15 alternate intra AC VLC) is still
  rejected.
- The MPEG-2 encoder remains I-only progressive 4:2:0; the new decoder
  features are not mirrored on the encoder side in this milestone.


## [0.0.10](https://github.com/OxideAV/oxideav-mpeg12video/compare/v0.0.9...v0.0.10) - 2026-05-04

### Other

- cargo fmt: split long ffmpeg-status assert
- cross-validate MPEG-2 I-only output via ffmpeg
- honour q_scale_type per picture (H.262 Table 7-6)
- honour alternate_scan per picture (H.262 Figure 7-3)

### Added

- MPEG-2 decoder: honour `alternate_scan` per picture (H.262 §7.3 Figure 7-3
  / `scan[1][]`). Streams with `picture_coding_extension.alternate_scan = 1`
  are now decoded instead of returning `Error::Unsupported`.
- MPEG-2 decoder: honour `q_scale_type = 1` per picture, mapping the 5-bit
  `quantiser_scale_code` through H.262 §7.4.2.2 Table 7-6 (range up to 112).
  Both the slice-header code and per-MB `quantiser_scale_code` overrides are
  routed through the new lookup. Previously rejected with
  `Error::Unsupported`.
- Test: `ffmpeg_decodes_our_mpeg2_output` cross-validates our MPEG-2 I-only
  encoder against ffmpeg as a black-box decoder (skips silently when ffmpeg
  is unavailable).

## [0.0.9](https://github.com/OxideAV/oxideav-mpeg12video/compare/v0.0.8...v0.0.9) - 2026-05-03

### Other

- cargo fmt: pending rustfmt cleanup
- replace never-match regex with semver_check = false
- migrate to centralized OxideAV/.github reusable workflows
- adopt slim VideoFrame shape
- pin release-plz to patch-only bumps

## [0.0.8](https://github.com/OxideAV/oxideav-mpeg12video/compare/v0.0.7...v0.0.8) - 2026-04-25

### Other

- drop oxideav-codec/oxideav-container shims, import from oxideav-core

## [0.0.7](https://github.com/OxideAV/oxideav-mpeg12video/compare/v0.0.6...v0.0.7) - 2026-04-24

### Other

- bump criterion 0.5 → 0.8

## [0.0.6](https://github.com/OxideAV/oxideav-mpeg12video/compare/v0.0.5...v0.0.6) - 2026-04-19

### Other

- drop Cargo.lock — this crate is a library
- bump oxideav-core / oxideav-codec dep examples to "0.1"
- bump to oxideav-core 0.1.1 + codec 0.1.1
- migrate register() to CodecInfo builder
- bump oxideav-core + oxideav-codec deps to "0.1"
- claim AVI FourCCs via oxideav-codec CodecTag registry
- bump oxideav-core to 0.0.5
- migrate to oxideav_core::bits shared BitReader / BitWriter

## [0.0.5](https://github.com/OxideAV/oxideav-mpeg12video/compare/v0.0.4...v0.0.5) - 2026-04-18

### Other

- document decoder-path optimizations + bench numbers
- add MC unclipped fast path (~-6% MPEG-1 inter decode)
- release v0.0.3

## [0.0.4](https://github.com/OxideAV/oxideav-mpeg12video/releases/tag/v0.0.4) - 2026-04-18

### Other

- bump version to 0.0.4
- satisfy cargo fmt + clippy
- add MPEG-2 (H.262) support, rename crate, optimize VLC
- update README + crate description to reflect I/P/B encoder
- add B-frame encoder (FWD/BWD/BI + reorder buffer)
- make crate standalone (pin deps, add CI + release-plz + LICENSE)
- add Decoder::reset overrides for video decoders
- move repo to OxideAV/oxideav-workspace
- add publish metadata (readme/homepage/keywords/categories)
- complete P-frame encode with half-pel ME refinement
- add P-frame encoder (forward MC + residual)
- add I-frame encoder
- full I+P+B frame decode with display-order reordering
- fix I-frame decode — always read EOB after intra AC loop
- scaffold MPEG-1 video decoder (ISO/IEC 11172-2) — headers + VLC tables
