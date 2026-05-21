# oxideav-mpeg12video

A pure-Rust MPEG-1 Video / MPEG-2 Video codec for the
[oxideav](https://github.com/OxideAV/oxideav) framework.

## Status

**Clean-room rebuild — rounds 1–4 (sequence layer + GOP header + picture header).**

Master was orphan-rebuilt on **2026-05-18** under the workspace
[clean-room policy](https://github.com/OxideAV/oxideav/blob/master/docs/IMPLEMENTOR_ROUND.md);
the prior implementation had VLC table modules whose data could not
be defended as clean-room. The rebuild starts here.

### What round 1 landed

* Parser for `sequence_header()` per **ISO/IEC 13818-2 (ITU-T H.262)
  §6.2.2.1** with field semantics from §6.3.3:
  * `sequence_header_code` = `0x000001B3` start-code check
  * 12-bit `horizontal_size_value`, 12-bit `vertical_size_value`
    (forbidden zero rejected per §6.3.3)
  * 4-bit `aspect_ratio_information` decoded against Table 6-3
    (`forbidden`, `Square`, DAR `3:4`, `9:16`, `1:2,21`, plus
    `Reserved` capture for codes 0101..1111)
  * 4-bit `frame_rate_code` (Table 6-4; forbidden zero rejected)
  * 18-bit `bit_rate_value` (forbidden zero rejected per §6.3.3)
  * `marker_bit` enforcement
  * 10-bit `vbv_buffer_size_value`
  * `constrained_parameters_flag` (semantic note: §6.3.3 says this
    has no meaning in 13818-2 and shall be `'0'`; we preserve the
    bit rather than coerce)
  * Optional 64-byte `intra_quantiser_matrix` and
    `non_intra_quantiser_matrix` loads (default-matrix `None` /
    explicit `Some([u8; 64])`)
* Typed `Mpeg2SequenceHeader { width, height, aspect_ratio,
  frame_rate_code, bit_rate, vbv_buffer_size,
  constrained_parameters, intra_quant, non_intra_quant }` return.

### What round 2 lands

* Parser for `sequence_extension()` per **ISO/IEC 13818-2 §6.2.2.3**
  with field semantics from §6.3.5:
  * `extension_start_code` = `0x000001B5` + 4-bit
    `extension_start_code_identifier == '0001'` Sequence Extension
    ID (Table 6-2).
  * 8-bit `profile_and_level_indication` (raw byte — clause 8
    interpretation deferred).
  * `progressive_sequence` flag, 2-bit `chroma_format` decoded
    against Table 6-5 (`reserved` 00 rejected; 4:2:0 / 4:2:2 / 4:4:4).
  * 2-bit `horizontal_size_extension`, 2-bit
    `vertical_size_extension`, 12-bit `bit_rate_extension`,
    `marker_bit` enforcement, 8-bit `vbv_buffer_size_extension`.
  * `low_delay` flag, 2-bit `frame_rate_extension_n`, 5-bit
    `frame_rate_extension_d`.
* Composed view `Mpeg2Sequence::from_buf(buf)` that parses the
  header + `next_start_code()` zero-byte stuffing (§5.2.3) +
  extension and synthesises the full 14-bit `horizontal_size` /
  `vertical_size`, 30-bit `bit_rate`, and 18-bit `vbv_buffer_size`.
* Combined-`bit_rate == 0` guard at the composer level (§6.3.3
  forbids the composite zero).
* 12 new unit tests (every spec-defined rejection site exercised:
  wrong start code, wrong identifier, reserved chroma_format,
  zeroed marker_bit, truncated buffer, missing extension after
  header, full-shape composition with stuffing).
* 2 new black-box integration tests against the same
  `ffmpeg`-produced 352×240 fixture (extension decode +
  end-to-end composed `Mpeg2Sequence` round-trip).

### What round 3 lands

* Parser for `group_of_pictures_header()` per **ISO/IEC 13818-2
  §6.2.2.6** with field semantics from §6.3.8:
  * `group_start_code` = `0x000001B8` validation.
  * 25-bit `time_code` decomposition (Table 6-11):
    * 1-bit `drop_frame_flag`
    * 5-bit `time_code_hours` (range 0..=23 enforced)
    * 6-bit `time_code_minutes` (range 0..=59 enforced)
    * 1-bit `marker_bit` (enforced `'1'`)
    * 6-bit `time_code_seconds` (range 0..=59 enforced)
    * 6-bit `time_code_pictures` (range 0..=59 enforced)
  * 1-bit `closed_gop` flag.
  * 1-bit `broken_link` flag.
* Typed `Mpeg2Gop { time_code, closed_gop, broken_link }` plus
  typed `TimeCode { drop_frame, hours, minutes, seconds, pictures }`
  (both re-exported at the crate root).
* 11 new unit tests (every spec-defined rejection site exercised:
  wrong start code, hours/minutes/seconds/pictures out of range,
  zeroed marker_bit, truncated buffer, full flag-matrix capture).
* 1 new black-box integration test against the existing 352×240
  fixture — locates the `0x000001B8` GOP start code, decodes the
  time-code, and asserts `closed_gop = 1`, `broken_link = 0` (the
  defaults that ffmpeg's mpeg2video encoder writes).

### What round 4 lands

* Parser for `picture_header()` per **ISO/IEC 13818-2 §6.2.3**
  with field semantics from §6.3.10:
  * `picture_start_code` = `0x00000100` validation.
  * 10-bit `temporal_reference`.
  * 3-bit `picture_coding_type` decoded against Table 6-12 (`I` /
    `P` / `B`; `forbidden` `000`, MPEG-1 `D` `100`, and
    `reserved` `101`/`110`/`111` codes rejected).
  * 16-bit `vbv_delay` (raw — the `0xFFFF` VBR sentinel is
    surfaced without reinterpretation).
  * Conditional 1-bit `full_pel_forward_vector` + 3-bit
    `forward_f_code` for `picture_coding_type ∈ {P, B}`.
  * Conditional 1-bit `full_pel_backward_vector` + 3-bit
    `backward_f_code` for `picture_coding_type == B`.
  * `extra_information_picture` byte loop driven by
    `extra_bit_picture` (collected as a raw `Vec<u8>`).
* Parser for `picture_coding_extension()` per **§6.2.3.1** with
  field semantics from §6.3.11:
  * `extension_start_code` + Picture Coding Extension ID `1000`
    (Table 6-2) validation.
  * Four 4-bit `f_code[s][t]` sub-fields with the forbidden-zero
    guard (§6.3.11; `15` = unused).
  * 2-bit `intra_dc_precision` (Table 6-13) and 2-bit
    `picture_structure` (Table 6-14; reserved `00` rejected).
  * Ten trailing single-bit flags from `top_field_first` through
    `composite_display_flag`.
* Composed-view helper `Mpeg2PictureHeader::parse_with_extension`
  that handles the `next_start_code()` zero-byte stuffing
  between the two layers (§5.2.3).
* Typed `Mpeg2PictureHeader`, `PictureCodingType`,
  `PictureCodingExtension`, `PictureStructure` plus
  `PICTURE_START_CODE` / `PICTURE_CODING_EXTENSION_ID` constants
  (re-exported at the crate root).
* 18 new unit tests (every spec-defined rejection site exercised:
  wrong start code, forbidden / D-picture / reserved
  `picture_coding_type`, zero `f_code[s][t]`, reserved
  `picture_structure`, wrong PCE extension identifier, truncated
  buffers) plus the round-trip extra-info loop and the
  helper-flag matrix.
* 2 new black-box integration tests against the existing 352×240
  fixture (picture-header decode at offset `0x1e` and
  picture_header + picture_coding_extension composition).

### What's NOT in rounds 1–4

* `quant_matrix_extension()` (§6.2.3.2),
  `picture_display_extension()` (§6.2.3.3),
  `sequence_display_extension()` (§6.2.2.4),
  `sequence_scalable_extension()` (§6.2.2.5)
* Composite-display sub-fields inside
  `picture_coding_extension()` when `composite_display_flag = 1`
* Slice / macroblock decoding, VLC tables, motion vectors, IDCT
* Encoder
* `oxideav_core::Decoder` / `Encoder` trait wiring — `register()`
  is still a no-op so the registry does not yet route to this crate

## Clean-room provenance

Every line in this crate's `src/` traces to:

* `docs/video/h262/is138182-1995.pdf` — ISO/IEC 13818-2:1995 base
  text (Recommendation ITU-T H.262 (1995 E)) §§5.2.3, 6.2.2.1,
  6.2.2.3, 6.2.2.6, 6.2.3, 6.2.3.1, 6.3.3, 6.3.4, 6.3.5, 6.3.8,
  6.3.10, 6.3.11, Tables 6-2 / 6-3 / 6-4 / 6-5 / 6-11 / 6-12 / 6-13
  / 6-14.
* `docs/video/h262/IEC-13818-2_Specs.pdf` — second copy of the
  same spec, cross-referenced for typography.
* `oxideav-core`'s published `BitReader` MSB-first API.
* The `ffmpeg` CLI binary, used **only** as an opaque encoder for
  the integration-test fixture. Its source code was not consulted.

No external library source was read, quoted, or paraphrased.

## License

MIT — see [LICENSE](./LICENSE).
