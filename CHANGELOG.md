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
- Slice/macroblock decoding, VLC tables, motion compensation, IDCT.
- `oxideav_core::Decoder` wiring once a complete picture round-trips.
