# oxideav-mpeg12video

A pure-Rust MPEG-1 Video / MPEG-2 Video codec for the
[oxideav](https://github.com/OxideAV/oxideav) framework.

## Status

**Clean-room rebuild — rounds 1–17 (sequence layer + GOP header + picture header + slice header + macroblock_address_increment + macroblock_type + macroblock-layer quantizer_scale + coded_block_pattern + macroblock_modes() motion-type / dct_type tail + MPEG-2 motion_vectors() / motion_vector() + Tables B-10 / B-11 + §7.6.3.1 PMV reconstruction with wrap-around + §7.6.3.3 inter-vector PMV update (Tables 7-10 / 7-11) + §7.6.3.4 reset + §7.6.3.7 chroma scaling + MPEG-1 motion_vector(s) per §2.4.2.7 driven by Annex B Table B.4 + MPEG-1 §2.4.4.2 / §2.4.4.3 motion-vector reconstruction with `right_little` / `right_big` wrap-around, `full_pel_*_vector` shift, and the luma / chroma whole/half-pel split + MPEG-1 §2.4.2.8 / §2.4.3.7 intra-block DC prelude with Annex B Tables B.5a / B.5b VLCs and the differential→`dct_zz[0]` reconstruction, plus the §2.4.4.1 8x8 zig-zag `scan[m][n]` + MPEG-1 §2.4.3.7 `dct_coeff_first` / `dct_coeff_next` run-level walker driven by Annex B Tables B.5c / B.5d / B.5e VLCs with the §2.4.3.7 `dct_coeff_first` vs `dct_coeff_next` `(0, 1)` disambiguation, `end_of_block` recognition, and Table B.5f escape encoding for the short 14-bit `[-127, +127] \ {0}` form and the long 22-bit `[-255, -128] ∪ [+128, +255]` form).**

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

### What round 10 lands

* Parser for the remainder of `macroblock_modes()` after
  `macroblock_type` per **ISO/IEC 13818-2 §6.2.5.1** with field
  semantics from §6.3.17.1 and the meaning Tables 6-17, 6-18 and 6-19.
  Round 10 closes `macroblock_modes()` itself; `motion_vectors()`
  (Tables B-10 / B-11) and the residual block loop remain out of scope.
  * The 2-bit `frame_motion_type` (frame pictures) decoded against
    **Table 6-17** and the 2-bit `field_motion_type` (field pictures)
    decoded against **Table 6-18**, each surfacing the derived
    `prediction_type` (`Field-based` / `Frame-based` / `16x8 MC` /
    `Dual-Prime`), `motion_vector_count`, `mv_format`
    (`field` / `frame`), and `dmv` (§6.3.17.2). The reserved code `00`
    is rejected in both tables. The two `Field-based` rows of Table
    6-17 are disambiguated by the caller-supplied
    `spatial_temporal_weight_class` (always `0` for the non-scalable
    streams this crate can reach).
  * The motion-type code's §6.2.5.1 presence gate
    (`macroblock_motion_forward || macroblock_motion_backward`, omitted
    in a frame picture when `frame_pred_frame_dct == 1`) is honoured —
    when absent the parser reads zero bits and the decoder applies the
    §6.3.17.1 "as if Frame-/Field-based" default (deferred to the
    motion-vector round).
  * The 1-bit `dct_type` flag (`1` = field DCT coded) read only when
    `picture_structure == frame`, `frame_pred_frame_dct == 0`, and the
    macroblock is intra or has a coded pattern (§6.2.5.1). When absent
    the **Table 6-19** effective-value derivation is left to the
    block-organisation round; the field is surfaced as `Option`.
  * `spatial_temporal_weight_code` is not read: it is gated on
    `spatial_temporal_weight_code_flag`, always `0` for the
    non-scalable Tables B-2 / B-3 / B-4; a `mb_type` claiming the flag
    is rejected so the cursor is never silently misaligned.
  * Returns `MacroblockModesTail { motion_type, dct_type,
    bit_position_after }` driven by a `MacroblockModesContext`
    (picture_structure + frame_pred_frame_dct + weight class) so
    callers chain into `coded_block_pattern()` / `motion_vectors()`
    without losing the partial-byte cursor.
* Typed `MacroblockModesTail`, `MotionType`, `PredictionType`,
  `MvFormat`, and `MacroblockModesContext` (re-exported at the crate
  root).
* 21 new unit tests covering every Table 6-17 / 6-18 row, the
  per-class `Field-based` vector-count split, reserved-code rejection
  in both tables, the motion-type and `dct_type` presence matrix
  (frame vs field picture, `frame_pred_frame_dct`, intra / pattern),
  the scalable-flag rejection, zero-bit absent paths, and truncated-
  buffer handling on both fields.
* 2 new black-box integration tests against the existing 352×240
  fixture: chains slice → `macroblock_address_increment` →
  `macroblock_type` → `quantizer_scale` → `macroblock_modes()` tail on
  the first I-picture macroblock and asserts the motion-type code is
  absent (plain Intra) with `dct_type` presence keyed to the fixture's
  own `frame_pred_frame_dct`; plus a spliced P-picture frame macroblock
  that decodes `frame_motion_type` + `dct_type` with exact bit
  accounting.

### What round 11 lands

* Parsers for `motion_vectors(s)` per **ISO/IEC 13818-2 §6.2.5.2** and
  the inner `motion_vector(r, s)` per **§6.2.5.2.1** with field semantics
  from §6.3.17.2 / §6.3.17.3 and the Annex B Tables B-10
  (`motion_code`) and B-11 (`dmvector`). The numerical reconstruction
  of `vector'[r][s][t]` from the parsed pieces (§7.6.3.1) remains out of
  scope — that needs the PMV state machine the next round will land.
  * **Table B-10 `motion_code`**: all 33 variable-length codewords
    (`-16..=+16`) walked MSB-first longest-first against the spec's
    tabulated bit-strings — the 1-bit zero entry, 3-/4-/5-/7-/8-/10-/
    11-bit codes on each sign, prefix-free and width-fitting.
  * **Table B-11 `dmvector[t]`**: the 1-/2-bit `{0, +1, -1}` VLC.
  * Fixed-length `motion_residual[r][s][t]` consumed when `f_code != 1
    && motion_code != 0`, with `r_size = f_code - 1` driving the width
    (1..=8 bits). `f_code ∉ {2..=9}` rejected as a §6.3.11 violation
    when a residual would otherwise be read.
  * `motion_vertical_field_select[r][s]` flag honoured per §6.2.5.2 —
    suppressed when `motion_vector_count == 1 && (mv_format == frame ||
    dmv == 1)`, present otherwise.
  * `MotionVectors::parse(br, kind, &MotionType, &MotionVectorsContext)`
    drives the wrapper from a parsed `frame_motion_type` /
    `field_motion_type` (round 10) and the `f_code[s][t]` matrix the
    caller carries from `picture_coding_extension()`. Returns
    `MotionVectors { kind, entries, bit_position_after }` with one or
    two `MotionVectorEntry { vertical_field_select, motion_vector }`
    rows.
* Typed `MotionVector`, `MotionVectorEntry`, `MotionVectors`,
  `MotionVectorsContext`, `MotionVectorsKind` (re-exported at the crate
  root).
* 29 new unit tests covering every Table B-10 row (parsed individually),
  the +16 / -16 extremes, the table's 33 unique values, prefix-freeness
  and width-fitting invariants, unknown-prefix and truncated-buffer
  rejection on both VLC tables, Table B-11's three values plus its
  truncated-second-bit short case, the `motion_vector(r, s)`
  presence-matrix (no residual on f_code = 1 or motion_code = 0,
  residual width = `f_code - 1`, dmvector suppressed when `dmv = 0`),
  out-of-range `f_code` rejection, all four `motion_vectors(s)` shapes
  (frame count-1 / field count-1 / dual-prime count-1 / count-2), the
  Forward / Backward `f_code` pair selection, `motion_vector_count`
  validation, and truncated-VFS-/truncated-code short paths.
* 2 new black-box integration tests against the existing 352×240
  fixture: the first I-picture is plain `Intra` so per §6.2.5.2 no
  `motion_vectors()` element exists — the test confirms the fixture's
  f_codes are the §6.3.11 unused sentinel `15`; and a spliced
  P-picture frame macroblock prefix that drives the full
  `macroblock_type` → `frame_motion_type` → `dct_type` →
  `motion_vectors(0)` chain (`motion_code = -1`, `motion_residual = 1`
  with f_code = 2, `motion_code_vert = 0`) and asserts the 9-bit total
  cursor accounting.

### What round 12 lands

* `vector'[r][s][t]` reconstruction per **ISO/IEC 13818-2 §7.6.3.1** —
  the bridge from round 11's parsed motion-vector syntax to actual
  luminance motion-compensation vectors. The Annex's worked formula is
  implemented verbatim:
  * `compute_delta(motion_code, motion_residual, f_code)` derives the
    spec's `delta` (`f = 1 << (f_code - 1)`, `delta = motion_code` when
    `f == 1 || motion_code == 0`, otherwise
    `delta = sign(motion_code) * ((|motion_code| - 1) * f +
    motion_residual + 1)`).
  * `vector_range(f_code)` produces the `[low, high]` half-range
    (`[-16*f, 16*f - 1]`) and `range = 32*f`. Verified against Table
    7-8 across `f_code ∈ {1..=9}` (range doubles per step).
  * `reconstruct_component(...)` combines `delta` with the prior
    `PMV[r][s][t]`, applies the §7.6.3.1 vertical-half-pred rule
    (`mv_format == field && t == 1 && picture_structure == frame ⇒
    prediction = PMV / 2` using §4.3 floor-division, PMV-writeback =
    `vector' * 2`), wraps the result into `[low, high]`, and returns
    the new PMV value. §7.6.3.2 range-conformance is enforced as a
    parse-time invariant.
  * `reconstruct_motion_vector(pmv, &MotionVector, r, s, f_code_h,
    f_code_v, mv_format, picture_structure)` runs the §7.6.3.1
    procedure for both `t = 0` and `t = 1`, threading round 11's
    parsed `MotionVector` straight in.
* `Pmv` state container per **§7.6.3** (Table 7-7): the four PMV slots
  indexed by `r ∈ {0, 1}`, `s ∈ {0, 1}`, `t ∈ {0, 1}`, with
  `Pmv::new()` (zero-initialised) and `Pmv::reset()` for the §7.6.3.4
  slice-start / non-concealment-intra / P-picture-non-intra-without-
  forward / P-skipped reset rules.
* `scale_chroma(luma_horiz, luma_vert, ChromaFormat)` per **§7.6.3.7**:
  4:2:0 halves both, 4:2:2 halves only horizontal, 4:4:4 is identity.
  Uses spec §4.3 toward-zero division.
* Typed `Pmv`, `ReconstructedComponent`, `ScaledMotionVector`,
  `Component`, `Direction`, `VectorIndex` (re-exported at the crate
  root) plus the `compute_delta` / `vector_range` /
  `reconstruct_component` / `reconstruct_motion_vector` /
  `scale_chroma` free functions.
* 29 new unit tests covering: `compute_delta`'s shortcut +
  full-formula branches across `f_code ∈ {1..=9}` and motion_code
  signs, `motion_residual` presence-required / presence-forbidden
  rejection, out-of-range `f_code` rejection, `vector_range` doubling
  per `f_code` step, end-to-end `reconstruct_motion_vector` with no
  wrap / wrap-low / wrap-high paths, vertical-half-pred firing /
  not-firing matrix (frame vs field picture, horizontal vs vertical
  component), floor-division for negative PMV under half-pred, PMV
  slot independence for `(r = 0 vs r = 1)` and `(forward vs
  backward)`, delta-outside-range rejection, `Pmv::reset` clears every
  slot, chroma scaling for all three `ChromaFormat` values plus
  toward-zero rounding on negative odd inputs, and Table 7-7 index
  enums.
* 2 new black-box integration tests against the existing 352×240
  fixture: confirms the fixture's I-picture is the §7.6.3 "PMV unused"
  case (every f_code = 15 sentinel, PMV stays zero after the §7.6.3.4
  reset), and a spliced two-macroblock P-picture chain
  (`motion_code = +2, residual = 0` then `motion_code = -1,
  residual = 0`, both with `f_code = 2`) decodes through `MotionVector
  → reconstruct_motion_vector` with PMV state evolving from 0 → 3 → 2
  (the second `delta = -1` added on top of the first vector's
  predictor), plus a §7.6.3.7 chroma scaling check on the 4:2:0 case.

### What round 13 lands

* `update_predictors(&mut Pmv, PmvUpdateContext)` per **ISO/IEC
  13818-2 §7.6.3.3** — the once-per-macroblock "Predictors to Update"
  pass that propagates the `[r = 0]` slot into the `[r = 1]` slot (or
  resets every slot) so that prediction modes which decoded fewer
  motion vectors than the maximum still leave a sensible `PMV[1]`
  behind for downstream macroblocks. Tables 7-10 (frame pictures) and
  7-11 (field pictures) are both implemented — the two tables share
  the same right-hand "Predictors to Update" column row-for-row.
* Typed `PmvUpdateContext` carries the macroblock-level inputs the
  table consumes (`picture_structure`, `frame_motion_type` /
  `field_motion_type` as `Option<PredictionType>`,
  `macroblock_motion_forward` / `_backward` / `_intra`, and
  `concealment_motion_vectors`). The intra path handles both the
  `‡` row (`Frame-based`/`Field-based` assumed when motion-type is
  absent) and the `◊` footnote (zero every slot when
  `concealment_motion_vectors == 0`).
* Typed `PmvUpdateOutcome` labels which Tables 7-10/7-11 row fired
  (`IntraConcealmentCopyForwardFirst`, `IntraResetAll`,
  `NonIntraCopyBoth`, `NonIntraCopyForward`, `NonIntraCopyBackward`,
  `NonIntraZeroMotionReset` (§ footnote), `NoUpdate` (Field-based
  in frame picture / 16x8 MC in field picture), `DualPrimeCopyForward`).
  The label lets tests assert the right branch was taken without
  poking at the post-update PMV state.
* Spec-conformance checks reject the cells the spec leaves
  unreachable: intra macroblocks with a motion flag set, Frame-based
  in a field picture (Table 7-11 has no such row), 16x8 MC in a
  frame picture, Dual-Prime with backward motion, Field-based or
  16x8 MC rows with both motion flags zero, and non-intra
  macroblocks with absent motion-type code.
* 18 new unit tests covering: the intra concealment-MV copy +
  no-concealment reset branches, intra-with-motion-flag rejection,
  Frame-based fwd-only / bwd-only / both / zero-motion (§ footnote
  reset) in frame pictures, Field-based row coverage in both frame
  pictures (NoUpdate) and field pictures (matching Frame-based
  shape), 16x8 MC in field pictures (NoUpdate), Dual-Prime forward
  copy in both picture types, Dual-Prime backward-flag rejection,
  cross-picture-type table rejection (Frame-based in field, 16x8 in
  frame), absent-motion-type rejection for non-intra, and an
  end-to-end chain that runs `reconstruct_motion_vector` then
  `update_predictors` and verifies the reconstructed (3, -2) vector
  propagates from `[0][0][:]` into `[1][0][:]`.

### What's NOT in rounds 1–13

* `quant_matrix_extension()` (§6.2.3.2),
  `picture_display_extension()` (§6.2.3.3),
  `sequence_display_extension()` (§6.2.2.4),
  `sequence_scalable_extension()` (§6.2.2.5)
* Composite-display sub-fields inside
  `picture_coding_extension()` when `composite_display_flag = 1`
* `spatial_temporal_weight_code` (§6.2.5.1) — the scalable-only
  field of `macroblock_modes()` gated on
  `spatial_temporal_weight_code_flag`, plus the §6.3.17.1 /
  Table 6-19 effective-value derivations for the absent
  motion-type and `dct_type` cases (deferred to the motion-vector /
  block rounds)
* The scalable `macroblock_type` Tables B-5 .. B-8 (spatial / SNR
  scalability), which require `sequence_scalable_extension()`
  parsing
* §7.6.3.6 dual-prime additional arithmetic (deriving the
  opposite-parity vector from the decoded forward vector). Dual-prime
  `motion_code` / `motion_residual` / `dmvector` parsing landed in
  round 11; round 12 reconstructs the parsed vector but does not yet
  derive the opposite-parity vector. Round 13 added the §7.6.3.3
  inter-vector PMV update so the dual-prime row of Tables 7-10 / 7-11
  fires correctly even though the §7.6.3.6 vector derivation itself
  is still ahead.
* §7.6.3.9 concealment motion vectors (intra macroblocks with the
  `concealment_motion_vectors` flag set).
* Residual `dct_coeff_first` / `dct_coeff_next` walker (MPEG-1
  Annex B Tables B.5c..B.5e plus the B.5f escape; the equivalent
  MPEG-2 Tables B-12..B-16). Round 16 lands the MPEG-1 DC prelude
  (`dct_dc_size_*` Tables B.5a / B.5b + the §2.4.3.7 differential
  formula + the §2.4.4.1 8x8 zig-zag `scan[m][n]`); the wider
  run-length VLC and the IDCT itself remain ahead.
* Encoder
* `oxideav_core::Decoder` / `Encoder` trait wiring — `register()`
  is still a no-op so the registry does not yet route to this crate

### What round 14 lands

* Parser for `motion_vector(s)` per **ISO/IEC 11172-2:1993 (MPEG-1
  Video) §2.4.2.7** with field semantics from §2.4.3.6, driven by
  the Annex B **Table B.4** `motion_*_code` VLC. Unlike the MPEG-2
  layer (which folds in `motion_vertical_field_select`, `mv_format`,
  `dmv`, and `dmvector`), the MPEG-1 element is just the four-field
  sequence `(horizontal_code, horizontal_r, vertical_code,
  vertical_r)` parameterised on the matching `<dir>_f_code` from
  the picture header.
  * `Mpeg1MotionVector::parse(br, direction, f_code)` reads the
    four fields and returns a typed record with the post-parse bit
    position.
  * `Mpeg1MotionDirection { Forward, Backward }` selects which of
    `forward_f_code` / `backward_f_code` (and the matching
    `full_pel_*_vector` flag, applied during §2.4.4.2 / §2.4.4.3
    reconstruction in a later round) the parse is associated with.
  * Residual presence rule from §2.4.3.6: `motion_*_r` is in the
    bitstream iff `<dir>_f != 1 && motion_*_code != 0`. With
    `<dir>_f = 1 << (<dir>_f_code - 1)`, this collapses to "skip the
    residual iff `f_code == 1` or `code == 0`".
  * Residual width: `<dir>_r_size = <dir>_f_code - 1` bits
    (`1..=6`) per §2.4.4.2.
  * `f_code` range guard: §2.4.3.4 constrains `forward_f_code` /
    `backward_f_code` to `1..=7`; zero is rejected as "forbidden",
    values `≥ 8` are rejected as outside the spec's range.
  * Shared longest-first walker: MPEG-1 Table B.4 lists the same
    33 codeword → signed-value rows as MPEG-2 Annex B Table B-10,
    so a new `pub(crate) motion_vector::match_motion_code` accessor
    re-uses the existing per-row constants instead of retyping them.
    The per-row test in this module transcribes Table B.4
    independently from page 43 of the 11172-2 spec to confirm the
    mapping.
  * 20 new unit tests covering every Table B.4 row (1- to 11-bit
    codes, the `-16..=+16` signed value range), the four-corner
    presence matrix (`f_code = 1` always-no-residual, `f_code = 7`
    widest residual, mixed zero/non-zero components, both-zero
    bare codes), the §2.4.3.4 `f_code` range guard, truncated
    short-buffer and invalid-prefix detection, and the `Backward`
    direction tag.
* Reconstruction (§2.4.4.2 forward, §2.4.4.3 backward — the
  `right_little` / `right_big` wrap-around with `recon_right_for_prev`
  predictor state plus the `full_pel_forward_vector` shift) is the
  next-round concern, mirroring the round 11 → 12 split on the
  MPEG-2 side (parser → reconstruction).

### What round 15 lands

* MPEG-1 motion-vector reconstruction per **ISO/IEC 11172-2:1993
  §2.4.4.2** (P-picture forward) and §2.4.4.3 (B-picture forward +
  backward). This is the second domino of the MPEG-1 motion-vector
  pipeline — the round 14 parser delivers `(code, r)` pairs; this
  round folds them through the §2.4.4.2 predictor-update arithmetic
  into the integer `right_for / down_for` offsets the §2.4.4.2
  luminance / chrominance pel-prediction equations consume.
  * `mpeg1_reconstruct::reconstruct(mv, ctx, &mut predictor, dir)`
    runs the §2.4.4.2 four-step formula end-to-end:
    1. `r_size = f_code - 1`, `f = 1 << r_size`.
    2. `complement = (f == 1 || code == 0) ? 0 : f - 1 - r`.
    3. `little = code * f`, then `little ∓= complement` (sign
       toward zero); `big = little ∓ 32*f`.
    4. `new_vector = prev + little`; pick `little` if it stays in
       `[-16*f, 16*f-1]`, else use `prev + big`; write back to PMV;
       apply the `full_pel_*_vector << 1` shift to the recon
       output.
  * `Mpeg1Predictor { recon_right_prev, recon_down_prev }` carries
    the half-sample-unit PMV across macroblocks. `Mpeg1Predictor::
    reset()` zeroes it for the start-of-slice / P-picture "no MV"
    case.
  * `mpeg1_reconstruct::reconstruct_zero(&mut predictor)` — the
    §2.4.4.2 ¶3 P-picture "no forward MV data" path: zeroes both
    the returned recon and the predictor.
  * `mpeg1_reconstruct::reconstruct_absent(ctx, &predictor)` — the
    §2.4.4.3 B-picture "no MV data" carry-over: recon =
    predictor unchanged.
  * `Mpeg1FrameMvContext { f_code, full_pel }` packages the two
    picture-header fields (§2.4.2.3 / §2.4.3.4 / §2.4.3.5) the
    reconstruction needs in addition to the parsed element.
  * `Mpeg1ReconstructedMv` carries `recon_right` / `recon_down`
    plus the §2.4.4.2 closing table's luminance (`right_for_luma =
    recon_right >> 1`, `right_half_for_luma`) and chrominance
    (`right_for_chroma = (recon_right / 2) >> 1`,
    `right_half_for_chroma = recon_right / 2 - 2 * right_for_chroma`)
    whole / half-pel splits. The spec deliberately uses arithmetic
    `>>` for luma vs C-style `/` for chroma — the divergence on
    negative `recon_*` values is preserved bit-exact.
  * §2.4.4.2 conformance guards on `*_little != ±forward_f * 16`
    are enforced (both seam values flagged as
    `Error::InvalidBitstream`).
  * 23 new unit tests covering: `f_code = 1` zero / non-zero
    codes, `f_code = 2` complement-zero / complement-nonzero
    paths, positive / negative codes, PMV accumulation across
    consecutive macroblocks, wrap-around in both directions,
    `full_pel` post-PMV shift, both seam guards, every input
    validation site (direction mismatch, `f_code = 0` / `≥ 8`,
    residual present-when-forbidden / absent-when-required), the
    P-picture zero-reset and B-picture carry-over paths, and the
    luma / chroma half-pel split for both positive and negative
    `recon_*` values.
  * 2 new black-box integration tests against a hand-assembled
    two-macroblock bitstream — parse via
    `Mpeg1MotionVector::parse` then reconstruct end-to-end,
    asserting the PMV propagates from MB1 into MB2.
* The §2.4.4.2 pel-prediction loop itself (the
  `pel[i][j] = pel_past[i+down_for][j+right_for]` bilinear
  half-pel filter at the top of page 35) is the next-round
  concern — it consumes a reference-picture buffer, which the
  decoder doesn't yet allocate.

### What round 16 lands

* The MPEG-1 intra-block **DC prelude** per **ISO/IEC 11172-2:1993
  §2.4.2.8 / §2.4.3.7** — the per-block entry point of the residual
  block layer.
  * `block_dc::DcCoefficient::parse(br, component)` walks Annex B
    **Table B.5a** (`dct_dc_size_luminance`, 9 codes 2..=7 bits
    wide) when `component == Luminance`, else Annex B **Table B.5b**
    (`dct_dc_size_chrominance`, 9 codes 2..=8 bits wide) for the
    size VLC; then reads the `dct_dc_size`-wide
    `dct_dc_differential` field MSB-first per §2.4.2.8 and applies
    the §2.4.3.7 sign-extension formula:

    ```text
    if (raw & (1 << (size - 1)))
        zz0 = raw ;
    else
        zz0 = ((-1) << size) | (raw + 1) ;
    ```

    to produce the signed `dct_zz[0]` in the range
    `[-(2^size - 1), +(2^size - 1)]`. `size = 0` is the absent-
    differential case (`zz0 = 0`).
  * `block_dc::DcComponent { Luminance, Chrominance }` selects the
    matching table.
  * `block_dc::SCAN: [[u8; 8]; 8]` encodes the §2.4.4.1 page-32 8x8
    `scan[m][n]` zig-zag matrix (`scan[0][0] = 0`,
    `scan[0][7] = 28`, `scan[7][0] = 35`, `scan[7][7] = 63`) used
    by every block-layer dequantiser as `i = SCAN[m][n]`.
    `block_dc::INVERSE_SCAN: [(u8, u8); 64]` is the compile-time
    inverse for encoders / trace tools.
  * `block_dc::MAX_DC_SIZE = 8` documents the spec upper bound on
    both tables.
  * 23 new unit tests cover every B.5a / B.5b row, code-width
    uniqueness, codes-fit-their-width invariants, the §2.4.3.7
    page-30 worked example for `dc_size = 3` (`000 → -7, 001 → -6,
    ... 111 → +7`), corner values for `dc_size = 1 / 2 / 8`
    (`reconstruct(8, 0x00) == -255`, `reconstruct(8, 0xFF) == +255`,
    `reconstruct(8, 0x80) == +128`, `reconstruct(8, 0x7F) == -128`),
    truncated-buffer / garbage-prefix detection, luminance-vs-
    chrominance table disambiguation on identical wire bits
    (the bit-string `'00'` decodes as `dc_size = 1` against B.5a
    but `dc_size = 0` against B.5b), full bit-position tracking
    across size 0 (3 bits) and size 8 (7 code bits + 8 differential
    bits), the §2.4.4.1 `SCAN` matrix's spec corners and the
    classic zig-zag diagonal opening (`0, 1, 2, 3, 4, 5, 6, 7, 8,
    9` mapping), and the `SCAN` / `INVERSE_SCAN` round-trip.
* The remaining `dct_coeff_first` / `dct_coeff_next` walker
  (Annex B Tables B.5c..B.5e plus the B.5f escape) is the
  next-round concern — that's the wider run-length VLC the
  block-layer dequantiser consumes after the DC field.

### What round 17 lands

* The MPEG-1 residual `dct_coeff_first` / `dct_coeff_next` walker
  per **ISO/IEC 11172-2:1993 §2.4.2.8 / §2.4.3.7** — the
  zig-zag-coded body of every block, fed by Annex B **Tables B.5c
  / B.5d / B.5e** (the run-level codebook) and **Table B.5f** (the
  escape encoding). This is the second half of the residual block
  layer; round 16 landed the intra DC prelude and this round fills
  the rest of the per-block syntax up to the §2.4.4 dequantiser.
  * `dct_coeff::DctCoeffStep::parse(br, position)` walks the
    longest-first codeword tree across all three sub-tables,
    matches the `(run, level)` pair, then reads the trailing 1-bit
    sign `s` and applies it to produce the signed `dct_zz[i]`
    coefficient to write.
  * `dct_coeff::CoefficientPosition { First, Next }`
    disambiguates the spec's two `(run = 0, level = 1)` forms:
    `dct_coeff_first` uses the 2-bit `1s` code (legal only as the
    first coefficient of a non-intra block), `dct_coeff_next` uses
    the 3-bit `11s` code. The `end_of_block` codeword `10` is
    accepted only at `Next` per Table B.5c note 2.
  * Table B.5f escape coverage: the 6-bit `000001` prefix is
    followed by a 6-bit `run` and an 8-bit signed level word; the
    short form covers `level ∈ [-127, +127] \ {0}` directly, and
    the long form (8-bit prefix `0x80` for negative or `0x00` for
    positive plus an 8-bit magnitude) extends the range to
    `[-255, -128]` and `[+128, +255]`. The forbidden `-256`
    (prefix `0x80` + magnitude `0x00`) and the forbidden long-form
    positive `< 128` (prefix `0x00` + magnitude `< 0x80`) are
    rejected — the spec explicitly notes the MPEG-1 escape
    encoding differs from the later ISO/IEC 13818-2 §7.2.2.3
    fixed-length scheme.
  * `dct_coeff::DctCoeff` is the decoded symbol: either
    `RunLevel { run, signed_level, escape }` or `EndOfBlock`. The
    `escape` flag records whether the symbol came through the
    Table B.5f path so trace tools can reconstruct the wire form.
  * `dct_coeff::MAX_RUN = 63` and `dct_coeff::MAX_LEVEL_MAG = 255`
    document the spec bounds for both VLC and escape forms.
* 31 new unit tests cover: every Table B.5c / B.5d / B.5e row
  parsed and round-tripped via the walker with both signs;
  per-width prefix-freeness and code-width-fit invariants;
  FIRST-vs-NEXT disambiguation of the `(0, 1)` two-code form;
  `end_of_block` recognition only at NEXT; Table B.5f short form
  for positive / negative levels and the `±127` corner; Table
  B.5f long form for `-128` / `-255` and `+128` / `+200` / `+255`;
  rejection of the forbidden `-256` long-form encoding and the
  forbidden long-form positive `< 128` encoding; truncated /
  empty buffer rejection; and full bit-position accounting across
  every codeword width from the 2-bit FIRST form to the 17-bit
  B.5e maximum.
* 2 new black-box integration tests synthesise complete MPEG-1
  residual block runs (FIRST + several NEXT including an escape,
  then `end_of_block`) and confirm the §2.4.3.7 `i = run` /
  `i += run + 1` zig-zag-position update never exceeds 63 plus
  the running bit cursor lines up exactly with the encoded
  bit lengths. The existing 352×240 fixture is an MPEG-2 stream
  (it uses the differently-encoded MPEG-2 Table B-16 escape) and
  cannot exercise the MPEG-1 escape path — these tests assemble
  spec-defined bit-strings in-process instead.

## Clean-room provenance

Every line in this crate's `src/` traces to:

* `docs/video/h262/is138182-1995.pdf` — ISO/IEC 13818-2:1995 base
  text (Recommendation ITU-T H.262 (1995 E)) §§4.3, 5.2.3, 6.2.2.1,
  6.2.2.3, 6.2.2.6, 6.2.3, 6.2.3.1, 6.2.4, 6.2.5, 6.2.5.1, 6.2.5.2,
  6.2.5.2.1, 6.2.5.3, 6.3.3, 6.3.4, 6.3.5, 6.3.8, 6.3.10, 6.3.11,
  6.3.16, 6.3.17.1, 6.3.17.2, 6.3.17.3, 6.3.17.4, 7.6.3, 7.6.3.1,
  7.6.3.2, 7.6.3.3, 7.6.3.4, 7.6.3.7, Tables 6-1 / 6-2 / 6-3 / 6-4 /
  6-5 / 6-10 / 6-11 / 6-12 / 6-13 / 6-14 / 6-17 / 6-18 / 6-19 / 7-7 /
  7-8 / 7-10 / 7-11, and Annex B Tables B-1 / B-2 / B-3 / B-4 / B-9 /
  B-10 / B-11.
* `docs/video/h262/IEC-13818-2_Specs.pdf` — second copy of the
  same spec, cross-referenced for typography.
* `docs/video/mpeg1/ISO_IEC_11172-2-MPEG1-Video-1993.pdf` —
  ISO/IEC 11172-2:1993 (MPEG-1 Video) §2.4.2.6, §2.4.2.7, §2.4.2.8,
  §2.4.3.4, §2.4.3.5, §2.4.3.6, §2.4.3.7, §2.4.4.1, §2.4.4.2,
  §2.4.4.3, §D.5.5.1, §D.5.5.2, and Annex B Table B.1 + Table B.4
  + Table B.5a + Table B.5b + Table B.5c + Table B.5d + Table B.5e
  + Table B.5f. Referenced for the
  `macroblock_stuffing` semantics (a code MPEG-2 drops); in round 9
  for the macroblock-layer `quantizer_scale` field (syntax §2.4.2.7,
  semantics §2.4.3.6 — the `1..=31` range and the slice/macroblock
  persistence rule); in round 14 for the MPEG-1 `motion_vector(s)`
  element itself (§2.4.2.7 wire shape, §2.4.3.6 residual gate,
  §2.4.3.4 `forward_f_code` / `backward_f_code` `1..=7` range, and
  Annex B Table B.4 motion-code VLC); and in round 16 for the
  intra-block DC prelude (Annex B Tables B.5a / B.5b
  `dct_dc_size_*` VLCs, §2.4.2.8 / §2.4.3.7 `dct_dc_differential`
  → `dct_zz[0]` sign-extension formula, and the §2.4.4.1 page-32
  8x8 `scan[m][n]` zig-zag matrix); and in round 17 for the
  residual-block `dct_coeff_first` / `dct_coeff_next` walker
  (Annex B Tables B.5c / B.5d / B.5e run-level VLCs spanning code
  widths 1..=16, Table B.5f escape encoding with its short 14-bit
  and long 22-bit forms, the §2.4.3.7 `(0, 1)` FIRST-vs-NEXT
  disambiguation, the Table B.5c note-2 `end_of_block`
  restriction, and the spec note that the MPEG-1 escape form is
  intentionally different from the ISO/IEC 13818-2 §7.2.2.3
  fixed-length 12-bit scheme). The MPEG-2 Table B-1 entries
  themselves trace to 13818-2.
* `oxideav-core`'s published `BitReader` MSB-first API.
* The `ffmpeg` CLI binary, used **only** as an opaque encoder for
  the integration-test fixture. Its source code was not consulted.

No external library source was read, quoted, or paraphrased.

## License

MIT — see [LICENSE](./LICENSE).
