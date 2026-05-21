# oxideav-mpeg12video

A pure-Rust MPEG-1 Video / MPEG-2 Video codec for the
[oxideav](https://github.com/OxideAV/oxideav) framework.

## Status

**Clean-room rebuild — rounds 1–3 (sequence layer + GOP header).**

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

### What's NOT in rounds 1–3

* `picture_header()` / `picture_coding_extension()` parsers,
  `quant_matrix_extension()`, `sequence_display_extension()`,
  `sequence_scalable_extension()`
* Slice / macroblock decoding, VLC tables, motion vectors, IDCT
* Encoder
* `oxideav_core::Decoder` / `Encoder` trait wiring — `register()`
  is still a no-op so the registry does not yet route to this crate

## Clean-room provenance

Every line in this crate's `src/` traces to:

* `docs/video/h262/is138182-1995.pdf` — ISO/IEC 13818-2:1995 base
  text (Recommendation ITU-T H.262 (1995 E)) §§5.2.3, 6.2.2.1,
  6.2.2.3, 6.2.2.6, 6.3.3, 6.3.4, 6.3.5, 6.3.8, Tables 6-2 / 6-3 /
  6-4 / 6-5 / 6-11.
* `docs/video/h262/IEC-13818-2_Specs.pdf` — second copy of the
  same spec, cross-referenced for typography.
* `oxideav-core`'s published `BitReader` MSB-first API.
* The `ffmpeg` CLI binary, used **only** as an opaque encoder for
  the integration-test fixture. Its source code was not consulted.

No external library source was read, quoted, or paraphrased.

## License

MIT — see [LICENSE](./LICENSE).
