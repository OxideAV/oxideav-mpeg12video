# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
- Macroblock-loop continuation past `macroblock_type`: the rest of
  `macroblock_modes()` (`spatial_temporal_weight_code`,
  `frame_motion_type` / `field_motion_type` per Tables 6-17 / 6-18,
  `dct_type`), then `motion_vectors()` (Tables B-10 / B-11),
  `coded_block_pattern` (Table B-9), and the residual block VLC
  tables (B-12 .. B-16) plus IDCT.
- The scalable `macroblock_type` Tables B-5 .. B-8 once
  `sequence_scalable_extension()` parsing lands.
- `oxideav_core::Decoder` wiring once a complete picture round-trips.
