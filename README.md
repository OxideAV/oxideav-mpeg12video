# oxideav-mpeg12video

A pure-Rust MPEG-1 Video / MPEG-2 Video codec for the
[oxideav](https://github.com/OxideAV/oxideav) framework.

## Status

**Clean-room rebuild — rounds 1–9 (sequence layer + GOP header + picture header + slice header + macroblock_address_increment + macroblock_type + macroblock-layer quantizer_scale + coded_block_pattern).**

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

### What round 5 lands

* Parser for the `slice()` header bits per **ISO/IEC 13818-2 §6.2.4**
  with field semantics from §6.3.16. Macroblock decoding is **not**
  in scope; the parser stops as soon as the terminating
  `extra_bit_slice` is consumed and reports `body_bit_position` so a
  later round can seed a macroblock parser at the right place.
  * 32-bit `slice_start_code`: 24-bit prefix `0x000001` + 8-bit
    `slice_vertical_position` validated against the Table 6-1 range
    `0x01..=0xAF`.
  * Optional 3-bit `slice_vertical_position_extension`, present iff
    the caller's [`SliceContext::vertical_size`] is `> 2800`
    (§6.2.4). When present, §6.3.16's stricter `svp ∈ [1:128]`
    constraint is enforced.
  * Optional 7-bit `priority_breakpoint`, gated on the caller's
    [`SliceContext::priority_breakpoint_present`] flag, which the
    higher layer derives from `sequence_scalable_extension()` (not
    yet parsed by this crate).
  * 5-bit `quantiser_scale_code` with the spec-defined
    forbidden-zero check (§6.3.16).
  * Optional intra-slice prelude: `intra_slice_flag` /
    `intra_slice` / 7-bit `reserved_bits` (enforced `== 0`) + the
    `extra_information_slice` byte loop driven by the same
    `nextbits() == '1'` mechanism used for `extra_information_picture`.
  * Terminating `extra_bit_slice` (enforced `== '0'`).
  * `SliceHeader::mb_row()` computes `mb_row` per §6.3.16:
    `(extension << 7) + svp - 1` when the extension is present,
    `svp - 1` otherwise.
* Typed `SliceHeader { slice_vertical_position,
  slice_vertical_position_extension, priority_breakpoint,
  quantiser_scale_code, intra_slice_flag, intra_slice,
  extra_information_slice, body_bit_position }` plus the
  `SliceContext` caller-supplied state container (both re-exported
  at the crate root).
* `SLICE_VERTICAL_POSITION_MIN` / `SLICE_VERTICAL_POSITION_MAX`
  constants taken from Table 6-1.
* 14 new unit tests covering every spec-defined rejection site
  (wrong prefix, svp = 0, svp = 0xB0, svp > 128 with extension,
  zero `quantiser_scale_code`, non-zero `reserved_bits`,
  truncated buffer) plus the intra-slice prelude /
  `extra_information_slice` round-trip and the `mb_row()` /
  bit-position-tracking accounting.
* 2 new black-box integration tests against the existing 352×240
  fixture (first slice header at offset `0x2e`, slice-start-code
  multiplicity sanity check).

### What round 6 lands

* Parser for `macroblock_address_increment` per **ISO/IEC 13818-2
  §6.2.5** with field semantics from §6.3.17.1 and the Annex B
  Table B-1 VLC walker. The macroblock body (type, motion vectors,
  coded block pattern, transform coefficients) remains out of scope.
  * All 33 increment-value VLC codes (`1..=33`) walked MSB-first
    against the spec's tabulated bit-strings — 1-, 3-, 4-, 5-, 7-,
    8-, 10-, and 11-bit code groups (Table B-1).
  * `macroblock_escape` (`0000 0001 000`, 11 bits) consumed with
    the §6.3.17.1 "add 33 per escape" chain rule. The decoded
    `MbAddressIncrement` surfaces `escape_count` separately so the
    higher layer can validate against the spec-imposed bound
    `mb_width × (mb_height − mb_row)`.
  * MPEG-1-only `macroblock_stuffing` (`0000 0001 111`, 11 bits)
    recognition, gated on `MbAddressIncrementContext::mpeg1` per
    ISO/IEC 11172-2:1993 §D.5.5.1. MPEG-2 streams (the default)
    reject the stuffing code as a §6.3.17.1 violation; stuffing
    *after* an escape is rejected in both MPEG-1 and MPEG-2
    contexts.
  * Returns `MbAddressIncrement { value, escape_count,
    stuffing_count, bit_position_after }` so callers can chain
    the parser into the next macroblock field without losing the
    partial-byte cursor.
* Typed `MbAddressIncrement` + `MbAddressIncrementContext`
  (re-exported at the crate root).
* 16 new unit tests covering every Table B-1 entry parsed
  individually, the escape chain (`34`, `66`, `70` — the §D.5.5.2
  worked example), spec-mandated stuffing rejection in MPEG-2,
  MPEG-1 stuffing acceptance, stuffing-after-escape rejection,
  garbage-prefix rejection, truncated-buffer handling,
  bit-position accounting, and Table B-1 internal invariants
  (33 values + escape + stuffing, no width-collisions, every
  code fits its declared bit width).
* 2 new black-box integration tests against the existing 352×240
  fixture: parses the first `macroblock_address_increment`
  immediately after the first slice header (expected `value = 1`,
  the single bit `'1'`) and confirms the fixture does not emit
  the MPEG-1-only stuffing code.

### What round 7 lands

* Parser for `macroblock_type` — the leading VLC of
  `macroblock_modes()` per **ISO/IEC 13818-2 §6.2.5.1** with field
  semantics from §6.3.17.1 and the non-scalable Annex B
  Table B-2 / B-3 / B-4 codeword sets. The rest of
  `macroblock_modes()` (motion-type / dct_type) and the macroblock
  body remain out of scope.
  * Table selection by `picture_coding_type` per Table 6-10 (no
    `sequence_scalable_extension()` present): Table B-2 for
    I-pictures, B-3 for P-pictures, B-4 for B-pictures.
  * All 2 + 7 + 11 codewords walked MSB-first longest-first against
    the spec's tabulated bit-strings, decoding the six §6.3.17.1
    derived flags `macroblock_quant`, `macroblock_motion_forward`,
    `macroblock_motion_backward`, `macroblock_pattern`,
    `macroblock_intra`, and `spatial_temporal_weight_code_flag`
    (always `0` for the non-scalable tables).
  * Returns `MacroblockType { macroblock_quant,
    macroblock_motion_forward, macroblock_motion_backward,
    macroblock_pattern, macroblock_intra,
    spatial_temporal_weight_code_flag, bit_position_after }` so
    callers chain into the next `macroblock_modes()` field without
    losing the partial-byte cursor.
* Typed `MacroblockType` (re-exported at the crate root).
* 13 new unit tests covering every Table B-2 / B-3 / B-4 row
  (parsed individually), the longest-first prefix-disambiguation,
  unknown-codeword and truncated-buffer rejection, and per-table
  internal invariants (codes fit their width, every table is
  prefix-free, row counts match Annex B).
* 2 new black-box integration tests against the existing 352×240
  fixture: the first I-picture macroblock's `macroblock_type`
  decodes to plain `Intra` (single-bit Table B-2 code `'1'`,
  `macroblock_intra = 1` and every other flag `0`) and advances
  the cursor by exactly one bit past the increment.

### What round 8 lands

* Parser for `coded_block_pattern()` per **ISO/IEC 13818-2
  §6.2.5.3** with field semantics from §6.3.17.4 and the Annex B
  Table B-9 variable-length codes. `coded_block_pattern()` appears
  in the bitstream exactly when `macroblock_pattern` (from
  `macroblock_type`) is set; the block loop (`block()`, the DCT
  coefficient VLCs, the IDCT) remains out of scope.
  * All 64 Table B-9 `coded_block_pattern_420` codewords (3- to
    9-bit) walked MSB-first longest-first, decoding to the 6-bit
    `cbp` (0..=63).
  * 4:2:2 / 4:4:4 chroma extensions: the 2-bit
    `coded_block_pattern_1` (4:2:2) and 6-bit
    `coded_block_pattern_2` (4:4:4) fixed-length codes are read
    when the caller-supplied `chroma_format` selects them.
  * `CodedBlockPattern::pattern_code(macroblock_intra,
    macroblock_pattern)` derives the 12-entry `pattern_code[i]`
    array per §6.3.17.4 (intra-default all-ones, then `cbp` /
    `coded_block_pattern_1` / `coded_block_pattern_2` masking).
  * Returns `CodedBlockPattern { cbp, coded_block_pattern_1,
    coded_block_pattern_2, bit_position_after }` so callers chain
    into the block loop without losing the partial-byte cursor.
* Typed `CodedBlockPattern` (re-exported at the crate root).
* 19 new unit tests covering every Table B-9 row (parsed
  individually), all-64-cbp coverage, longest-first prefix
  disambiguation, the 4:2:2 / 4:4:4 extensions, the §6.3.17.4
  `pattern_code` derivation across intra / non-intra and the
  chroma extensions, unknown-codeword / truncated-buffer
  rejection, and table invariants (prefix-free, widths fit,
  64 rows).
* 2 new black-box integration tests against the existing 352×240
  fixture: confirms the first I-picture macroblock is plain
  `Intra` (`macroblock_pattern = 0`) so per §6.2.5.3 it carries
  no `coded_block_pattern()`, and pins the fixture's chroma to
  4:2:0 then decodes a Table B-9 codeword against that format.

### What round 9 lands

* Parser for the macroblock-layer `quantizer_scale` per **ISO/IEC
  11172-2:1993 (MPEG-1 Video) §2.4.2.7** (syntax) with field
  semantics from **§2.4.3.6**. Within `macroblock()` the spec reads
  this field immediately after `macroblock_type`, conditional on the
  `macroblock_quant` flag the type carries. Round 9 fills exactly that
  bitstream gap between round 7's `macroblock_type` and round 8's
  `coded_block_pattern()`; the motion-vector fields and the residual
  block loop remain out of scope.
  * When `macroblock_quant` is set, a 5-bit `quantizer_scale` is read
    as an unsigned integer and validated against the §2.4.3.6 range
    `1..=31` (the value `0` is forbidden).
  * When `macroblock_quant` is clear the field is absent: the parser
    reads **zero** bits and returns `quantizer_scale = None`, so the
    decoder keeps the value established at the slice layer (§2.4.2.6)
    or a previous macroblock — the §2.4.3.6 persistence rule.
  * Returns `QuantizerScale { quantizer_scale, bit_position_after }`
    so callers chain into the motion-vector / `coded_block_pattern()`
    fields without losing the partial-byte cursor.
  * Convenience `QuantizerScale::parse_after_type(br, &MacroblockType)`
    threads the flag straight from a decoded `macroblock_type` — the
    two fields the spec reads back to back.
* Typed `QuantizerScale` plus `QUANTIZER_SCALE_MIN` /
  `QUANTIZER_SCALE_MAX` constants (re-exported at the crate root).
* 12 new unit tests covering the present / absent branches, every
  legal value `1..=31`, the forbidden-zero rejection, truncated- and
  empty-buffer handling on both branches, the `parse_after_type`
  flag-threading for both flag states, the bound constants, and
  bit-position accounting.
* 2 new black-box integration tests against the existing 352×240
  fixture: the first I-picture macroblock is plain `Intra`
  (`macroblock_quant = 0`), so per §2.4.2.7 it carries no
  `quantizer_scale` and the parser consumes zero bits; a spliced
  `macroblock_quant`-set `macroblock_type` then decodes a synthetic
  5-bit `quantizer_scale` with correct value and bit accounting.

### What's NOT in rounds 1–9

* `quant_matrix_extension()` (§6.2.3.2),
  `picture_display_extension()` (§6.2.3.3),
  `sequence_display_extension()` (§6.2.2.4),
  `sequence_scalable_extension()` (§6.2.2.5)
* Composite-display sub-fields inside
  `picture_coding_extension()` when `composite_display_flag = 1`
* The remainder of `macroblock_modes()` after `macroblock_type`:
  `spatial_temporal_weight_code`, `frame_motion_type` /
  `field_motion_type` (Tables 6-17 / 6-18), and `dct_type`
  (§6.2.5.1)
* The scalable `macroblock_type` Tables B-5 .. B-8 (spatial / SNR
  scalability), which require `sequence_scalable_extension()`
  parsing
* `motion_vectors()` (Tables B-10 / B-11), block-residual VLC
  Tables B-12 .. B-16, and IDCT
* Encoder
* `oxideav_core::Decoder` / `Encoder` trait wiring — `register()`
  is still a no-op so the registry does not yet route to this crate

## Clean-room provenance

Every line in this crate's `src/` traces to:

* `docs/video/h262/is138182-1995.pdf` — ISO/IEC 13818-2:1995 base
  text (Recommendation ITU-T H.262 (1995 E)) §§5.2.3, 6.2.2.1,
  6.2.2.3, 6.2.2.6, 6.2.3, 6.2.3.1, 6.2.4, 6.2.5, 6.2.5.1, 6.2.5.3,
  6.3.3, 6.3.4, 6.3.5, 6.3.8, 6.3.10, 6.3.11, 6.3.16, 6.3.17.1,
  6.3.17.4, Tables 6-1 / 6-2 / 6-3 / 6-4 / 6-5 / 6-10 / 6-11 /
  6-12 / 6-13 / 6-14, and Annex B Tables B-1 / B-2 / B-3 / B-4 /
  B-9.
* `docs/video/h262/IEC-13818-2_Specs.pdf` — second copy of the
  same spec, cross-referenced for typography.
* `docs/video/mpeg1/ISO_IEC_11172-2-MPEG1-Video-1993.pdf` —
  ISO/IEC 11172-2:1993 (MPEG-1 Video) §2.4.2.6, §2.4.2.7, §2.4.3.5,
  §2.4.3.6, §D.5.5.1, §D.5.5.2, and Annex B Table B.1. Referenced
  for the `macroblock_stuffing` semantics (a code MPEG-2 drops) and,
  in round 9, for the macroblock-layer `quantizer_scale` field
  (syntax §2.4.2.7, semantics §2.4.3.6 — the `1..=31` range and the
  slice/macroblock persistence rule). The MPEG-2 Table B-1 entries
  themselves trace to 13818-2.
* `oxideav-core`'s published `BitReader` MSB-first API.
* The `ffmpeg` CLI binary, used **only** as an opaque encoder for
  the integration-test fixture. Its source code was not consulted.

No external library source was read, quoted, or paraphrased.

## License

MIT — see [LICENSE](./LICENSE).
