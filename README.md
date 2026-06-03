# oxideav-mpeg12video

A pure-Rust MPEG-1 Video / MPEG-2 Video codec for the
[oxideav](https://github.com/OxideAV/oxideav) framework.

## Status

**Clean-room rebuild — rounds 1–33.** Round 1–23 cover the bitstream
parsing surface (sequence / GOP / picture / slice headers + the
macroblock-layer syntax through `motion_vectors()` and
`coded_block_pattern`), motion-vector reconstruction for both stream
types (MPEG-1 §2.4.4.2/§2.4.4.3 and MPEG-2 §7.6.3.1/.3/.4/.7), MPEG-2
dual-prime §7.6.3.6, the MPEG-1 intra DC prelude + zig-zag + run-level
walker (§2.4.2.8 / §2.4.3.7 / §2.4.4.1, Annex B Tables B.5a–B.5f), the
MPEG-1 intra/non-intra dequantiser (§2.4.4.1/.2 with the
`dct_dc_*_past` predictor chain, even-mismatch fix and `[-2048, 2047]`
saturation), the §7.6 motion-compensation pipeline (§7.6.4
forming-predictions pel reader, §7.6.7 combine-predictions, §7.6.8
add-coefficients with the `[0, 255]` clamp), and the MPEG-2 §7.4
inverse-quantisation pipeline (Tables 7-4 / 7-5 / 7-6, §7.4.2.3
reconstruction, §7.4.3 saturation, §7.4.4 sum-parity mismatch control
on `F[7][7]`). Round 24 lands the **§A 8×8 inverse discrete cosine
transform** with an IEEE Std 1180-1990 / P1180/D2 conformance harness
exercising the four statistical metrics (`pmse`, `omse`, `pme`, `ome`)
plus peak error against the bounds transcribed in
`docs/video/mpeg12video/idct-accuracy-spec.md` §4. Round 25 lands the
**MPEG-2 residual VLC walker** per §7.2.2 with Annex B Tables B-14 /
B-15 / B-16 — the §7.2.2.1 Table 7-3 `(intra_vlc_format,
macroblock_intra)` table selector, the §7.2.2.2 NOTE 2 / NOTE 3 FIRST
/ NEXT alternates for B-14's `(0, ±1)`, the table-dependent
`end_of_block` codeword (B-14 `10`, B-15 `0110`), and the Table B-16
escape encoding (`000001` prefix + 6-bit run + 12-bit signed_level
with both `0x000` and `0x800` rejected). Round 26 lands the **MPEG-2
§7.3 inverse-scan** — `ALTERNATE_SCAN` (Figure 7-3) plus the
`alternate_scan` flag-driven `scan_table` / `inverse_scan_table`
selectors, the `place_coefficient` per-sample writer that mates with
the round-25 walker, and the `apply_inverse_scan` full §7.3 loop body
for callers operating on a pre-flattened `QFS[0..64]` list; Figure 7-2
stays single-sourced from [`block_dc::SCAN`] with a permutation +
equality test pinning the relationship. Round 27 lands the **MPEG-2
§7.2.1 intra-block DC prelude** — Annex B Tables B-12 / B-13
(`dct_dc_size_luminance` / `dct_dc_size_chrominance` sized to
`0..=11` to accommodate `intra_dc_precision = 3`), the §7.2.1
`dc_dct_differential` → `dct_diff` `half_range` reconstruction, the
three-cell per-component DC predictor `dc_dct_pred[cc]` with
Table 7-2 reset values (`{128, 256, 512, 1024}` for
`intra_dc_precision ∈ {0,1,2,3}`), and the §7.2.1 `QFS[0] ∈ [0,
2^(8 + intra_dc_precision) - 1]` bitstream constraint enforcement.
Round 28 lands the **MPEG-2 §6.2.6 `block(i)` driver** —
`mpeg2_block_decoder::decode_block` chains the §7.2.1 DC prelude
(intra blocks only) → §7.2.2 residual VLC walker (with §7.2.2.2
NOTE 2 / NOTE 3 FIRST / NEXT alternation) → §7.3 inverse scan
(Figure 7-2 / Figure 7-3 keyed off `alternate_scan`) → §7.4
inverse-quantisation pipeline → §A 8×8 IDCT into a single
"bitstream → `f[y][x]` plane ready for §7.6.8 add-and-saturate"
entry point, with the §7.2.2 wire-position `walker_index + run ≤
63` constraint enforced as an `InvalidBitstream` rejection.
Round 29 lands the **§6.2.5 / §6.2.6 macroblock-block driver** —
`mpeg2_macroblock_blocks::decode_macroblock_blocks` walks a
macroblock's `pattern_code[12]` array and dispatches the round-28
`mpeg2_decode_block` once per coded slot, auto-deriving the
§6.1.1.8 block-index → component mapping (Figures 6-10 / 6-11 /
6-12), the §7.4.2.1 Table 7-5 weighting-matrix `w` per `(coding,
component, chroma_format)`, and the §7.2.1 non-intra-macroblock
DC-predictor reset, returning a `Vec<DecodedBlock>` paired with
the §6.1.1.8 block-index position.
Round 30 lands the **§6.2.4 slice-level macroblock-header walker**
— `slice_macroblock_walk::walk_slice` picks up at the post-
`slice_header()` cursor and walks the `do { macroblock() } while
( nextbits() != '0000 0000 0000 0000 0000 0000' )` loop,
parsing each macroblock's spec-deterministic header chain
(§6.2.5 `macroblock_address_increment` with Table B-1 / escape /
MPEG-1 stuffing, §6.2.5.1 `macroblock_modes()` opener via
`macroblock_type` against Tables B-2 / B-3 / B-4 keyed on
`picture_coding_type`, and the conditional 5-bit
macroblock-level `quantiser_scale_code` when `macroblock_quant
== 1`), tracking the §6.3.17.1 per-slice state across iterations
(`previous_macroblock_address` seeded from
`mb_row * mb_width - 1`, `macroblock_address` advancing through
the increment chain, `past_intra_address` advancing to
`macroblock_address` on every intra macroblock, and the
intra-quant override applying to *this* MB and every subsequent
MB), surfacing skipped-MB ranges as
`skipped_macroblock_count = increment - 1` for a future §7.6.6
round, rejecting the first-MB-increment-must-be-1 violation,
and exposing per-MB `body_bit_position` as the entry point for
the deferred `motion_vectors()` / `coded_block_pattern()` /
`block(i)` driver rounds. The `macroblock_modes()` tail
(motion-type / dct_type), motion vectors, CBP, and per-block
walker stay out of scope this round — their PMV reset / f_code
/ per-block-context wiring intersects with cross-MB state the
picture-level driver above this slice walker will own.
Round 31 lands the **§7.6.6 skipped-macroblock specification** —
`skipped_macroblock::describe_skipped_macroblock` consumes the
round-30 slice walker's `skipped_macroblock_count` ranges and
returns the per-§7.6.6.1..4 deterministic prediction shape
(prediction type / `mv_format` / same-parity field reference /
MV source / PMV side-effect) for one skipped slot at a time
across all four picture-coding-type × picture-structure
combinations, plus the §7.6.6 preamble I-picture rejection
(non-scalable case) and the §7.6.3.4 PMV-reset hook
`skipped_macroblock_apply_to_pmv` that fires the
P-picture-only "Motion vector predictors shall be reset to
zero" rule.
Round 32 wires the **§6.2.5.1 `macroblock_modes()` tail**
into `slice_macroblock_walk::walk_slice`: `frame_motion_type`
(Table 6-17) on frame pictures with `frame_pred_frame_dct == 0`
whose MB sets a motion flag, `field_motion_type` (Table 6-18)
on every motion-bearing MB in a field picture, and `dct_type`
on frame pictures with `frame_pred_frame_dct == 0` whose MB
is intra or has a coded pattern — read between the existing
`macroblock_type` parse and the §6.2.5
`if (macroblock_quant) quantiser_scale_code` read so the
walker now follows the §6.2.5 syntax-tree order. Two new
`SliceWalkContext` fields (`picture_structure`,
`frame_pred_frame_dct`) carry the §6.3.11 gates; the
`first_slice` shorthand defaults both to a tail-gated-off
shape (preserving the round-30 I-picture / `frame_pred_frame_dct
== 1` semantics verbatim) while
`first_slice_with_picture_extension` accepts the full pair
and `first_slice_mpeg1` pins both for MPEG-1 streams.
`MacroblockRecord` gains `motion_type: Option<MotionType>`
and `dct_type: Option<bool>` alongside the existing
`macroblock_type` / `quantiser_scale_code` fields. This also
fixes a latent ordering bug where the walker had been reading
`quantiser_scale_code` immediately after `macroblock_type`,
misaligning the cursor on any MB whose `macroblock_modes()`
tail consumed bits.
Round 33 wires the **§6.2.5 macroblock body wire-parse** into
`slice_macroblock_walk::walk_slice`: `motion_vectors(0)` (gated
on `macroblock_motion_forward == 1` or `macroblock_intra &&
concealment_motion_vectors == 1`), `motion_vectors(1)` (gated
on `macroblock_motion_backward == 1`), the concealment-MV
`marker_bit` (the §6.3.17 `'1'` bit on intra macroblocks with
`concealment_motion_vectors == 1` — a `'0'` here is
`InvalidBitstream`), and `coded_block_pattern()`
(`macroblock_pattern == 1`, with the §6.3.17.4
`pattern_code[12]` derivation pre-computed for the caller).
**Wire-syntax only**: the §7.6.3.1 reconstruction of
`vector'[r][s][t]` against the PMV state (and the §7.6.3.3 PMV
update / §7.6.3.4 reset) stay deferred to the picture-level
driver. `SliceWalkContext` grows six new fields —
`f_code_fwd_horiz` / `f_code_fwd_vert` / `f_code_bwd_horiz` /
`f_code_bwd_vert` (the four §6.3.11 `f_code[s][t]` widths
driving the `motion_residual` bit-width),
`concealment_motion_vectors` (the §6.3.11 intra-MB MV /
marker-bit gate), and `chroma_format` (the §6.3.5
`Yuv420` / `Yuv422` / `Yuv444` setting driving the §6.2.5.3
`coded_block_pattern_1` / `coded_block_pattern_2` extensions
and the §6.3.17.4 indexing). Existing `first_slice` /
`first_slice_with_picture_extension` / `first_slice_mpeg1`
shorthand constructors default the new fields to placeholders
that fire none of the new gates, so the round-30..32 fixtures
stay bit-identical. New `first_slice_with_picture_body`
surfaces the full pair for callers walking P/B slices,
intra-concealment-MV slices, or 4:2:2 / 4:4:4 pictures.
`MacroblockRecord` gains `motion_vectors_forward:
Option<MotionVectors>`, `motion_vectors_backward:
Option<MotionVectors>`, `concealment_marker_bit: Option<bool>`,
`coded_block_pattern: Option<CodedBlockPattern>`, and
`pattern_code: [bool; 12]`. The §6.3.17.1 / Table 6-19 absent-
modes-tail default (Frame-based for frame pictures, Field-based
for field pictures, `mv_count = 1`, `dmv = 0`) is synthesised
internally so `motion_vectors()` can still parse for the
`frame_pred_frame_dct == 1` motion-MB and intra-concealment-MV
paths.

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

### What round 18 lands

* The MPEG-1 §2.4.4.1 (intra) / §2.4.4.2 (non-intra) **dequantiser
  bodies** per **ISO/IEC 11172-2:1993** (pages 32 / 35) — the pure
  arithmetic stage between round 17's `dct_coeff_first` /
  `dct_coeff_next` walker (which fills `dct_zz[]`) and the §A.1
  IDCT (which the README "lacks" tail still names as the gating
  blocker on IEEE P1180/D2).
  * [`dequantize::dequantize_intra_block`] folds the four §2.4.4.1
    block-loops (first luminance / subsequent luminance / Cb / Cr)
    into a single entry point gated by an
    [`dequantize::IntraBlockKind`] selector. The shared body is
    `dct_recon[m][n] = (2 * dct_zz[scan[m][n]] * quantizer_scale *
    intra_quant[m][n]) / 16` for every `(m, n)`, the `if (recon &
    1) == 0 -> recon -= Sign(recon)` even-mismatch rule (spec
    footnote: *"This has been found to prevent accumulation of
    mismatch errors."*), and the `[-2048, 2047]` saturating clip.
  * The DC element `dct_recon[0][0]` is then overwritten per the
    spec's block-kind branch: `LuminanceFirst` / `ChrominanceCb` /
    `ChrominanceCr` choose between `128*8 + dct_zz[0]*8` (when
    `macroblock_address - past_intra_address > 1`) and
    `dct_dc_<comp>_past + dct_zz[0]*8`; `LuminanceSubsequent` is
    unconditional `dct_dc_y_past + dct_zz[0]*8`. The matching
    `dct_dc_<comp>_past` field of
    [`dequantize::IntraDcPredictors`] is then updated to the new
    DC value.
  * [`dequantize::IntraDcPredictors::at_slice_start`] returns the
    slice-start state per §2.4.4.1: all three `dct_dc_*_past =
    128*8 = 1024`, `past_intra_address = -2`.
    [`dequantize::IntraDcPredictors::reset_dc_to_default`] zeros
    the three `dct_dc_*_past` fields back to 1024 without touching
    `past_intra_address` — the spec's per-non-intra-macroblock
    reset (including skipped macroblocks).
    [`dequantize::finalise_intra_macroblock`] performs the
    macroblock close-out (`past_intra_address =
    macroblock_address`).
  * [`dequantize::dequantize_non_intra_block`] implements the
    §2.4.4.2 page-35 body: the numerator becomes `(2*dct_zz[i] +
    Sign(dct_zz[i])) * quantizer_scale * non_intra_quant[m][n]`,
    the same even-mismatch + saturation pipeline follows, and a
    final `if (dct_zz[i] == 0) dct_recon[m][n] = 0;` zeroing pass
    forces zero coefficients all the way through. There is no DC
    predictor for non-intra blocks.
  * [`dequantize::DEFAULT_INTRA_QUANT`] and
    [`dequantize::DEFAULT_NON_INTRA_QUANT`] expose the §2.4.3.2
    page-25 default matrices used when the sequence header sets
    the matching `load_*_quantizer_matrix == 0`.
    `DEFAULT_INTRA_QUANT[0][0] == 8` matches the spec's
    `intra_quant[0][0] = 8` requirement; every entry of
    `DEFAULT_NON_INTRA_QUANT` is 16.
  * Rejection sites: `quantizer_scale == 0` and
    `quantizer_scale > 31` (§2.4.3.6, contractually impossible
    given the upstream 5-bit field but enforced defensively), and
    any zero entry in the active `intra_quant` /
    `non_intra_quant` matrix (§2.4.3.2 *"The value zero is
    forbidden."*).
* 35 new unit tests cover: default-matrix corners
  (`intra_quant[0][0] = 8`, all-16 non-intra), predictor reset
  (slice-start, per-non-intra-macroblock); `Sign(...)` returning
  -1 / 0 / +1; the mismatch-prevention rule no-op on odd values
  and `+/- 1` correction on even values (positive and negative);
  saturation at both bounds; intra rejection sites
  (`quantizer_scale = 0`, `> 31`, zero quant entry); the slice-
  start all-zero `dct_zz` walkthrough that fires the
  `past_intra_address > 1` reset; the adjacent-macroblock branch
  (`gap == 1`) using `dct_dc_y_past`; the gap branch using `1024`;
  `LuminanceSubsequent` ignoring `past_intra_address`; Cb and Cr
  using their own predictor chains without touching the others;
  the per-macroblock `finalise_intra_macroblock` close-out; an AC
  worked example (uniform quant + qs = 4, `dct_zz = +5`, `(0, 1)`
  → 39); the negative-even subtract-sign case (`dct_zz = -3` →
  -23); saturation to `+2047` / `-2048` via large `qs * q`
  products; non-intra rejection of `quantizer_scale = 0` and zero
  quant entries; non-intra all-zero `dct_zz` → all-zero recon; the
  positive (`+3` → 55) and negative (`-3` → -55) non-intra worked
  examples; non-intra saturation to both bounds; the zeroing-pass
  override at zero coefficients neighbouring non-zero ones; the
  full four-luma + Cb + Cr intra-macroblock walk-through that
  advances each predictor in isolation; and a second-macroblock
  walk that confirms the address-gap branch flips correctly.
* 2 new black-box integration tests at
  `tests/dequantize_synthetic.rs` chain the round-17
  [`DctCoeffStep`] walker directly into the round-18 dequantiser:
  one assembles a 13-bit non-intra block (FIRST + NEXT + EoB),
  walks it into `dct_zz[]`, runs `dequantize_non_intra_block`, and
  verifies both non-zero `recon` cells against the spec's closed
  form and the §2.4.4.2 zeroing pass at every other cell; the
  other assembles an intra AC body, sets `dct_zz[0]` from a
  synthetic §2.4.3.7 DC prelude, runs
  `dequantize_intra_block::LuminanceFirst` at slice start, and
  pins both the `1024 + 40` reset-branch DC and the AC `47`
  produced by the spec's body. The existing 352×240 fixture is an
  MPEG-2 stream — its §7.4.2 dequantiser is differently shaped
  (the `(2 * dct_zz[i] + k) * Wq / 32` MPEG-2 form rather than
  the `2 * dct_zz[i] * Wq / 16` MPEG-1 form) and cannot exercise
  this round's MPEG-1-only arithmetic.

### What round 19 lands

* §7.6.3.6 MPEG-2 **dual-prime additional arithmetic** per
  **ISO/IEC 13818-2 (Recommendation ITU-T H.262)** — derives the
  opposite-parity motion vector(s) `vector'[r][0][1:0]` (`r = 2` for a
  field picture; `r ∈ {2, 3}` for a frame picture) from the
  same-parity vector decoded by round 12's
  [`pmv::reconstruct_motion_vector`] and the inline `dmvector[0..1]`
  decoded by round 11's [`motion_vector::MotionVector`].
  * The two spec formulae (page 87, lines 5-7) are:
    ```text
    vector'[r][0][0] = ((vector'[0][0][0] * m[parity_ref][parity_pred]) // 2) + dmvector[0]
    vector'[r][0][1] = ((vector'[0][0][1] * m[parity_ref][parity_pred]) // 2) + e[parity_ref][parity_pred] + dmvector[1]
    ```
  * [`dual_prime::m_factor`] encodes **Table 7-12** — the
    `picture_structure` / `top_field_first`-keyed field-distance
    factor. Frame pictures with `tff = 1` use `(m[1][0], m[0][1]) =
    (1, 3)`; `tff = 0` swaps to `(3, 1)`. Field pictures only ever
    consult the matching cross-parity entry (top-field row picks
    `m[1][0] = 1`, bottom-field row picks `m[0][1] = 1`). Diagonal
    cells `m[0][0]` / `m[1][1]` are not on Table 7-12 (the
    same-parity vector is the input, not derived) and the function
    errors when asked for them.
  * [`dual_prime::e_offset`] encodes **Table 7-13** — the
    unconditional vertical-line adjustment between fields:
    `e[0][0] = 0`, `e[0][1] = +1`, `e[1][0] = -1`, `e[1][1] = 0`.
    Independent of picture structure.
  * The `//` halving in the formulae is **§4.1 page 9** "integer
    division with rounding to the nearest integer; half-integer
    values rounded away from zero" — distinct from `DIV` (toward
    minus infinity) and `/` (toward zero). A private helper
    `div_round_away_from_zero(a, 2) = (a + sign(a)) / 2` honours
    `3//2 = 2` and `-3//2 = -2` per the spec examples; the same path
    handles `5//2 = 3` / `-5//2 = -3` (the only other half-integer
    cases reachable for `divisor = 2`).
  * [`dual_prime::derive_opposite_parity_vector`] is the single-row
    entry point; [`dual_prime::derive_all`] is the picture-level
    driver that returns the `r = 2` derivation for a field picture
    (its predicted parity drives `parity_ref = opposite`) or the
    `[r = 2, r = 3]` pair for a frame picture (top-field prediction
    in slot 0, bottom-field prediction in slot 1, per the §7.6.3.6
    page-87 lines-13-14 sentence "The top field shall use
    `vector'[2][0][1:0]` for opposite parity prediction and the
    bottom field shall use `vector'[3][0][1:0]`").
  * The derived `r ∈ {2, 3}` vectors do **not** flow back into the
    [`pmv::Pmv`] slots — Table 7-7's note explicitly says only `r ∈
    {0, 1}` have PMV storage; `r = 2` / `r = 3` are recomputed
    per-macroblock by §7.6.3.6.
  * Rejection sites: `dmvector` component outside `{-1, 0, +1}`
    (defensive guard for upstream Table B-11 parsing); any
    `(parity_ref, parity_pred)` pair that isn't on Table 7-12 for
    the active picture type (errors `InvalidBitstream`).
* [`dual_prime::dual_prime_picture`] lowers the parser-level
  `(PictureStructure, top_field_first)` pair into a typed
  [`dual_prime::DualPrimePicture`] so the call site in the
  macroblock-loop driver doesn't reason about field-vs-frame branching
  inline.
* 19 new unit tests cover: §4.1 `//` examples (`3//2 = 2`, `-3//2 =
  -2`, exact divisible, `1//2 = 1`, `-1//2 = -1`, `5//2 = 3`, `-5//2
  = -3`); Table 7-12 all four `(picture_structure, tff)` rows; the
  off-row error path for top / bottom field pictures and for the
  diagonal-cells case; Table 7-13 all four entries; §7.6.3.6
  worked-example arithmetic for a field-top derivation
  `(0, 0)/(0, 0) → (0, -1)`, a field-bottom derivation
  `(2, 2)/(0, 0) → (1, 2)`, a frame-tff=1 `r = 2` derivation
  `(4, 6)/(1, -1) → (3, 1)`, a frame-tff=1 `r = 3` derivation
  `(4, 6)/(0, 0) → (6, 10)` with the `m = 3` triple-scaling, and a
  frame-tff=0 swap test that confirms `m` halves on the `r = 2` row
  and triples on the `r = 3` row; the rounding-away-from-zero case
  `decoded = ±3` honoured under `m = 1`; out-of-range `dmvector`
  rejection (`-2`, `+2`, `+3`, `-5` on each axis); the
  `derive_all` driver returning one vector for field pictures and
  two vectors for frame pictures with the spec's r-index ordering;
  the `dual_prime_picture` lowering for all three
  `PictureStructure` values; and the [`FieldParity`] `index` /
  `opposite` helpers.

### What round 20 lands

* §7.6.4 **Forming predictions** per **ISO/IEC 13818-2 (Recommendation
  ITU-T H.262) page 88-89** — the integer-and-half-pel sample reader
  that turns a fully-reconstructed `vector'[r][s][1:0]` (from round 12)
  into a `width × height` pel-prediction block.
  * [`forming_predictions::split_component`] /
    [`forming_predictions::split_vector`] implement the per-axis
    split:
    ```text
    int_vec[t]  = vector[r][s][t] DIV 2
    half_flag[t] = (vector[r][s][t] - 2 * int_vec[t]) != 0
    ```
    `DIV` is the **§4.1 page 9** floor-toward-minus-infinity operator
    (`3 DIV 2 = 1`, `-3 DIV 2 = -2`), so `(-3) DIV 2 = -2` (not `-1`
    as Rust's `/` truncate-toward-zero would give); the half-flag is
    set iff the original component is odd, including for negative
    odd vectors.
  * [`forming_predictions::HalfPattern`] enumerates the four
    `(half_flag[0], half_flag[1])` outcomes — `Integer`,
    `HalfHorizontal`, `HalfVertical`, `HalfBoth` — that drive the
    page-88 four-arm `if` switch.
  * [`forming_predictions::predict_sample`] / [`forming_predictions::predict_block`]
    apply the §7.6.4 page-88 four-arm switch:
    ```text
    Integer       : pel_pred = pel_ref[y + iy][x + ix]
    HalfHoriz     : pel_pred = (pel_ref[y+iy][x+ix] + pel_ref[y+iy][x+ix+1]) // 2
    HalfVert      : pel_pred = (pel_ref[y+iy][x+ix] + pel_ref[y+iy+1][x+ix]) // 2
    HalfBoth      : pel_pred = (sum of the 4 corners of the 2x2 neighbourhood) // 4
    ```
    The `// 2` / `// 4` averaging is the **§4.1 page 9** round-to-
    nearest-with-half-integer-away-from-zero operator. On a non-
    negative sum it reduces to `(sum + d/2) / d` integer division,
    which the helpers express as `(a + b).div_ceil(2)` and `(sum +
    2) >> 2`. Indexing follows the spec: `t = 0` is horizontal (x),
    `t = 1` is vertical (y); positive vector components mean the
    prediction sample lies right of / below the current sample.
  * [`forming_predictions::ReferencePlane`] is a borrowed view
    `(data, width, height)` over a single sample plane with a
    [`forming_predictions::BoundaryMode::PadEdge`] clip-to-nearest-
    in-bounds-sample rule for reads that fall past the picture edge
    (the universally-implemented MPEG convention; the §7.6.4 base
    text leaves out-of-picture behaviour undefined).
  * [`forming_predictions::BlockSize`] keeps the block geometry
    dimensionless — `(width, height)`. The §7.6.5 prediction-mode
    table (16×16 frame, 16×8 MC, 16×16 field, 8×8 4:2:0 chroma,
    8×16 4:2:2 chroma, …) drives this loop without per-mode
    duplication.
* 38 new unit tests cover: `DIV` floor on positives / negatives /
  zero / odd / even / large magnitudes (`-1`, `-2`, `-3`, `1`, `2`,
  `3`, `1023`, `-1023`); the four `HalfPattern` outcomes with their
  flag round-trip; `ReferencePlane` in-bounds reads, four-direction
  pad-edge clamps, corner clamps, and the dimensions-mismatch
  rejection; `predict_sample` for the four patterns including
  `// 2` rounding-up (`avg(10, 11) = 11`), even-sum no-tie
  (`avg(10, 12) = 11`), `// 4` ties (`(0,1,0,1) -> 1`), negative
  integer vector pad-edge fallback, and negative-odd vectors that
  exercise the `DIV`-vs-truncate difference; and `predict_block`
  end-to-end on 2×2, 4×2, 3×1, 1×3, 4×1 geometries (copy, integer
  translation, half-horizontal, half-vertical, half-both, and
  right-edge padding).

### What round 21 lands

* §7.6.7 **Combining predictions** + §7.6.8 **Adding prediction and
  coefficient data** per **ISO/IEC 13818-2 (Recommendation ITU-T
  H.262) pages 104–106** — the two pointwise stages that turn the
  up-to-two §7.6.4 prediction blocks of a macroblock into the final
  decoded sample plane.
  * [`combine_predictions::average_predictions`] /
    [`combine_predictions::average_predictions_in_place`] implement
    the §7.6.7.1 page-105 formula:
    ```text
    pel_pred[y][x] = (pel_pred_forward[y][x] + pel_pred_backward[y][x]) // 2
    ```
    The `//` operator is the **§4.1 page 9** round-to-nearest /
    half-integer-away-from-zero operator; on a non-negative
    `u16` sum of two `u8` values it collapses to the canonical
    `(sum + 1) >> 1` rounded-up form.
  * [`combine_predictions::PredictionDirection`] enumerates the four
    §7.6.5 Tables 7-13 / 7-14 selection cases — `Forward`,
    `Backward`, `Bidirectional`, `Skipped` —
    keyed on the §6.3.17.1 `macroblock_motion_forward` /
    `macroblock_motion_backward` flags plus the §7.6.3.5
    implicit-zero-MV `(0, 0)` case.
    [`combine_predictions::combine_directional_predictions`] is the
    driver: forward-only / backward-only branches pass-through, the
    bidirectional branch calls `average_predictions`, and the
    `Skipped` branch returns the caller-supplied implicit-zero-MV
    forward block unchanged.
  * [`combine_predictions::average_dual_prime_predictions`] is the
    §7.6.7.4 alias of the same formula —
    `(pel_pred_same_parity + pel_pred_opposite_parity) // 2`.
    Arithmetic identical to the bidirectional average; the alias
    exists for caller readability when wiring §7.6.3.6 dual-prime
    vectors through the §7.6.4 reader.
  * [`add_coefficients::saturate`] implements the two `if` clauses
    of §7.6.8 page 106 (`d < 0 -> 0`, `d > 255 -> 255`) as a single
    `i32::clamp` returning `u8`. Bit-equivalent to the spec's
    two-branch form for any integer input.
  * [`add_coefficients::add_prediction_and_coefficients`] and its
    `..._in_place` variant pointwise add the §A.1 IDCT output
    (`i16`) and the §7.6.7 prediction (`u8`) and saturate to
    `[0, 255]`. The spec writes the loop over an 8×8 transform
    block; the operation is intrinsically pointwise so the
    signatures take `&[i16]` / `&[u8]` and work for any matching
    block geometry the §7.6.5 / §A.1 chain produces.
  * [`add_coefficients::add_intra_block`] is the intra shortcut:
    `macroblock_intra == 1` has no prediction step, so the final
    samples are `saturate(f)` across the IDCT output. Equivalent to
    passing an all-zero prediction to
    `add_prediction_and_coefficients`.
* 34 new unit tests cover: the `// 2` averaging — no-tie (`(10,12)
  → 11`), half-integer tie rounded up (`(10,11) → 11`, `(254,255) →
  255`), u8 max (`(255,255) → 255`), and the symmetry across the
  full `(x, x+1)` band; the four-way
  `combine_directional_predictions` switch including the
  length-mismatch rejection on the `Bidirectional` branch and the
  argument-ignored behaviour of the single-direction branches; the
  dual-prime alias's bit-equality with the bidirectional path; the
  saturation arithmetic at both clamps (`-1 → 0`, `-256 → 0`,
  `i32::MIN → 0`, `256 → 255`, `1000 → 255`, `i32::MAX → 255`); the
  pointwise add on a 64-sample 8×8-shaped block plus both saturation
  endpoints; the in-place add's exact match with the allocating
  variant; the intra shortcut's exact match with the zero-prediction
  path; and the empty-input degenerate case.
* New `tests/combine_add_synthetic.rs` integration test (7 cases)
  drives the full §7.6.4 → §7.6.7 → §7.6.8 chain on hand-crafted
  reference planes and IDCT-stand-in `i16` values for the intra /
  P-forward-only / B-bidirectional (with and without §7.6.8 clamp
  engagement) / B-backward-only / skipped-macroblock / 8×8 paths.
  Expected samples are hand-computed from the spec formulas, no
  external decoder is consulted.

### What round 22 lands

* §7.6 **Per-macroblock pipeline driver** per **ISO/IEC 13818-2
  (Recommendation ITU-T H.262) page 102** — the composition step that
  stitches the already-landed §7.6.5 / §7.6.6 case selection, the
  §7.6.7 combining endpoints, and the §7.6.8 add-and-saturate step
  into a single "block in → decoded samples out" driver, keyed off
  the parsed [`MacroblockType`] flags and the §6.3.17.4
  `pattern_code[12]` derivation of [`CodedBlockPattern`].
  * [`macroblock_pipeline::MacroblockKind`] is the §7.6.5 / §7.6.6
    case (`Intra` vs `Inter(PredictionDirection)`) classified by
    `MacroblockKind::from_macroblock_type`: intra flag dominates,
    `(forward, backward)` map to `Forward` / `Backward` /
    `Bidirectional` / `Skipped` (the last is the §7.6.3.5 implicit
    zero-MV case).
  * [`macroblock_pipeline::BlockInputs`] is the per-block payload —
    post-IDCT transform plane plus the §7.6.4 prediction sides
    (forward / backward) for the slot. Constructor helpers
    `BlockInputs::intra` / `::forward` / `::backward` /
    `::bidirectional` reflect the prediction subset each case needs.
  * [`macroblock_pipeline::decode_block`] is the inner driver: for
    the intra case it calls [`add_intra_block`] (the §7.6.8
    `d = saturate(f)` shortcut, prediction conceptually zero); for
    every inter case it calls [`combine_directional_predictions`]
    then [`add_prediction_and_coefficients`]. Returns the
    `[0, 255]`-clamped sample plane of the same length as the
    transform.
  * [`macroblock_pipeline::decode_macroblock`] is the outer driver:
    walks `pattern_code[0 .. blocks_per_macroblock(chroma)]` and
    invokes [`decode_block`] per coded slot, returning each
    [`DecodedBlock`] with its §6.3.17.4 `block_index`. Uncoded
    slots are skipped — the caller handles their `d = p` short-
    circuit if it wants their samples too.
  * [`macroblock_pipeline::blocks_per_macroblock`] returns the
    §6.1.1.8 chroma-format block count per MB: 6 for 4:2:0,
    8 for 4:2:2, 12 for 4:4:4. The walker uses this to bound
    `pattern_code[]` iteration.
  * [`macroblock_pipeline::PipelineError`] enumerates the four
    caller-bug paths: `LengthMismatch` (transform vs prediction
    slice length differs), `MissingForwardPrediction` /
    `MissingBackwardPrediction` / `MissingBidirectionalPrediction`
    (the inter direction needs a prediction side the caller didn't
    supply). The driver itself doesn't parse bitstreams, so an
    `InvalidBitstream` cannot originate here.
* The driver explicitly does **not** run the §A.1 IDCT — the
  transform plane enters pre-IDCT'd (the §A.1 implementation is
  still blocked by workspace issue #1110). It also does not parse
  the bitstream or form predictions — `BlockInputs` carry the
  outputs of the §7.6.4 [`predict_block`] and the (caller-supplied)
  IDCT. The driver's contract is intentionally narrow: it is the
  per-coded-block dispatch loop that was missing between
  "parsed syntax + per-block predictions and transforms in hand"
  and "final per-block decoded samples out."
* 22 new unit tests in `src/macroblock_pipeline.rs` cover:
  the four-way `MacroblockKind::from_macroblock_type` classification
  including the intra-overrides-motion case; `decode_block`'s intra
  shortcut bit-equality with `add_intra_block` and its
  prediction-side-ignored property; the inter forward / backward /
  bidirectional / skipped paths' combine-then-add arithmetic on
  hand-built 2×2 blocks; the four caller-bug errors
  (`MissingForward`, `MissingBackward`, `MissingBidirectional`,
  `LengthMismatch` on each single-side and the bidirectional path);
  the `blocks_per_macroblock` map for all three chroma formats; and
  the `decode_macroblock` walker's behaviour for the intra-everywhere
  case (6 / 12 blocks per MB), the inter-only-cbp-bits-walked case
  (`cbp = 0b101010` → blocks 0, 2, 4), the skipped-zero-pattern case
  (zero coded blocks), the 4:2:2 walk (8 coded blocks with
  `coded_block_pattern_1` feeding entries 6..7), and the
  error-propagation-on-first-failing-block path.
* New `tests/macroblock_pipeline_synthetic.rs` integration test
  (8 cases) drives the full pipeline end-to-end on hand-crafted
  reference planes and fabricated `i16` IDCT outputs for: 4:2:0
  intra-everywhere (6 blocks); P-forward-only zero-residual (the
  prediction passes through unchanged); B-bidirectional with the
  §7.6.8 clamp engaging at both ends (`255` / `0`); B-backward-only
  on a single coded block; the all-zero `pattern_code[]` skipped MB
  (zero decoded blocks); the inner `decode_block` on a canonical 8×8
  intra block matching the pointwise `saturate(f)` formula; the
  caller-bug `MissingForwardPrediction` propagation; and the
  `blocks_per_macroblock` chroma-format map.

### What round 23 lands

* **MPEG-2 §7.4 inverse-quantisation pipeline** per **ISO/IEC 13818-2
  / Recommendation ITU-T H.262 pages 73–76**, in a fresh
  `src/mpeg2_dequantize.rs` module. This is the dequantiser stage
  between §7.3 inverse-scan (already in hand via the §6.3.17.4
  `pattern_code[]` walker) and the §A.1 IDCT (still blocked by
  workspace issue #1110). The MPEG-1 §2.4.4 dequantiser in
  `src/dequantize.rs` is left untouched — the two formulations
  diverge on `k`, the saturation placement, and the mismatch
  control, so they live in separate modules.
  * §7.4.1 intra-DC: [`mpeg2_dequantize::intra_dc_mult`] encodes
    Table 7-4 (`intra_dc_precision ∈ {0,1,2,3} → intra_dc_mult ∈
    {8,4,2,1}`); the inverse-quantise pipeline short-circuits at
    `(v, u) == (0, 0)` for `Intra` blocks and emits `intra_dc_mult
    * QF[0][0]`.
  * §7.4.2.1 weighting matrices:
    [`mpeg2_dequantize::DEFAULT_INTRA_WEIGHT`] and
    [`mpeg2_dequantize::DEFAULT_NON_INTRA_WEIGHT`] expose the §6.3.7
    defaults (intra-default matches MPEG-1's `intra_quant`; non-
    intra-default is all-16).
    [`mpeg2_dequantize::select_weighting_matrix_index(coding,
    component, chroma_format)`] encodes Table 7-5 — the 4:2:0
    chroma collapse into the luma slot, and the 4:2:2 / 4:4:4 split
    into `w == 2` (intra chroma) and `w == 3` (non-intra chroma).
  * §7.4.2.2 quantiser_scale:
    [`mpeg2_dequantize::QUANTISER_SCALE_LINEAR`] and
    [`mpeg2_dequantize::QUANTISER_SCALE_NONLINEAR`] are the Table 7-6
    lookup arrays (`q_scale_type == 0` linear `2..=62`,
    `q_scale_type == 1` non-linear `1..=112`). The accessor
    [`mpeg2_dequantize::quantiser_scale(code, q_scale_type)`] rejects
    code `0` (forbidden per Table 7-6) and any value above the 5-bit
    range.
  * §7.4.2.3 reconstruction:
    [`mpeg2_dequantize::inverse_quantise_block`] applies `F''[v][u]
    = ((2 * QF[v][u] + k) * W[v][u] * quantiser_scale) / 32` with
    `k = 0` for intra and `k = Sign(QF[v][u])` for non-intra, under
    the §4.1 round-toward-zero `/` operator (Rust's signed-`/`
    matches).
  * §7.4.3 saturation: same `[-2048, 2047]` band as MPEG-1, but
    applied after §7.4.2.3 on `F''` to yield `F'`.
    [`mpeg2_dequantize::F_SATURATION_MIN`] /
    [`mpeg2_dequantize::F_SATURATION_MAX`] expose the constants.
  * §7.4.4 mismatch control: sums `F'[v][u]` over the block; if the
    sum is even, toggles the LSB of `F'[7][7]` to flip parity. The
    spec's Note 1 (XOR-of-LSBs equivalence) is mentioned in the
    module doc as a future optimisation; we use the literal sum form
    so the implementation tracks the printed pseudocode.
  * §7.4.5 summary: `inverse_quantise_block` is the single
    entrypoint composing §7.4.1 + §7.4.2.3 + §7.4.3 + §7.4.4,
    returning `F[v][u]` directly. The §A.1 IDCT is the next stage
    and is still blocked.
* 21 new unit tests in `src/mpeg2_dequantize.rs` cover: every cell
  of Table 7-4 plus its out-of-range rejection; Table 7-5 across
  every `(coding, component, chroma_format)` triple including the
  4:2:0 chroma-collapse and the 4:2:2 / 4:4:4 split; Table 7-6's
  linear column (every code) and the spot-checked non-linear column
  plus the full-table equivalence test (every legal code); the
  forbidden `code == 0` and the out-of-range rejection; `Sign`
  matching §4.1 across negative / zero / positive; `Saturate`
  clamping at both ends; the §7.4.1 short-circuit on an all-zero
  AC plane; the §7.4.4 mismatch flip on even sums; the §7.4.4
  no-op on odd sums; intra AC arithmetic with default
  `intra_quant`; non-intra arithmetic with positive and negative
  `QF[v][u]` (driving both `k = +1` and `k = -1` branches);
  saturation engagement at both `+2047` and `-2048`; and the
  mismatch isolation that proves only `F[7][7]` is ever touched.
* New `tests/mpeg2_dequantize_synthetic.rs` integration test
  (7 cases) drives the public surface against an independently-
  coded reference loop that transcribes §7.4.5 from the spec text:
  intra-block end-to-end equivalence; non-intra-block end-to-end
  equivalence; sweep of every legal `quantiser_scale_code` across
  both `q_scale_type` columns; the full Table 7-4 walk; the
  Table 7-5 walk; the Table 7-6 boundary slots; and the §7.4.3
  clamp constants. No external decoder is consulted — the
  reference loop is built from the §7.4 printed pseudocode in
  ISO/IEC 13818-2.

### What round 24 lands

* **§A 8×8 inverse discrete cosine transform** per **ISO/IEC 11172-2
  Annex A** ("8 by 8 Inverse discrete cosine transform", page 39,
  invoking **IEEE Draft Standard P1180/D2, July 18, 1990**) and
  **ISO/IEC 13818-2 Annex A** (same role, invoking **IEEE Std
  1180-1990, December 6, 1990**), in a fresh `src/idct.rs` module.
  This is the IDCT stage of Figure 7-1 between the round-23 §7.4
  inverse-quantisation pipeline and the round-22 §7.6 macroblock
  pipeline.
  * Three-layer API matching the spec's clean separation of
    reference / candidate / integer-output:
    * `idct::idct_reference_f64` — the **double-precision direct
      4-D reference**: evaluates `f[y][x] = (1/4) Σ_v Σ_u
      C(u)·C(v)·F[v][u]·cos((2x+1)uπ/16)·cos((2y+1)vπ/16)` literally
      using a cached cosine kernel. This is the closest practical
      analogue to the "infinite-precision" reference IEEE 1180 / P1180
      compare candidates against.
    * `idct::idct_candidate_f64` — the **separable 1-D-pass
      candidate**: eight 8-point row IDCTs followed by eight 8-point
      column IDCTs (the `O(N³)` decomposition). Mathematically
      identical to the direct reference; differs only in `f64`
      rounding order.
    * `idct::idct_8x8` / `idct_8x8_from_i32` — the **integer IDCT**
      the downstream §7.6 pipeline consumes: calls
      `idct_candidate_f64`, rounds with the §4.1 `Round(x)` operator
      (ties away from zero), and saturates the 9-bit signed pel
      range `[-256, +255]` per §7.5.
  * Module constants surface the spec's input/output ranges:
    `F_INPUT_MIN` / `F_INPUT_MAX` = `[-2048, 2047]` (the §7.4.3
    12-bit signed input clamp the upstream dequantiser produces) and
    `F_OUTPUT_MIN` / `F_OUTPUT_MAX` = `[-256, 255]` (the §7.5 9-bit
    signed pel-grid output clamp). Helper functions
    `saturate_idct_input` / `saturate_idct_output` apply each clamp.
  * 11 unit tests in `src/idct.rs` cover: all-zero input → all-zero
    output (the IEEE 1180 deterministic edge case); DC-only input
    produces a flat block (positive and negative); the §7.5 output
    saturation reaches `[-256, +255]` even for extremal coefficient
    inputs; `saturate_output` / `saturate_input` clamp behaviour;
    `idct_8x8_from_i32` saturates out-of-range coefficients; the
    `idct_reference_f64` and `idct_candidate_f64` IDCT outputs agree
    to within `1e-10` per pixel on an arbitrary input block.
* New `tests/idct_p1180_conformance.rs` integration test
  (8 cases) is the IEEE Std 1180-1990 / P1180/D2 **statistical
  conformance harness** against the bounds transcribed in
  `docs/video/mpeg12video/idct-accuracy-spec.md` §4. It exercises
  the **four statistical metrics** the spec defines plus peak
  error and the two mandatory deterministic edge cases:
  | Metric | Bound | Role |
  |--------|-------|------|
  | Peak error `pe`        | `≤ 1`      | Max absolute integer-domain candidate ↔ reference error, any pixel, any block. |
  | Peak per-position MSE `pmse` | `≤ 0.06`   | Worst per-position mean-square error across the 64 grid positions. |
  | Overall MSE `omse`     | `≤ 0.02`   | Mean-square error averaged across all 64 positions. |
  | Peak per-position mean error `pme` | `≤ 0.015`  | Worst absolute per-position mean error across the 64 positions. |
  | Overall absolute mean error `ome`  | `≤ 0.0015` | Mean per-position bias averaged across all 64 positions. |
  Six pseudo-random input conditions are exercised — three input-range
  parameters `L ∈ {256, 5, 300}` and both signs per parameter set —
  with `BLOCKS_PER_CONDITION = 1024` blocks each. Inputs are generated
  from a deterministic 64-bit linear congruential generator seeded per
  parameter set, so the harness is reproducible on every run. The
  candidate-vs-reference comparison runs at `f64` precision (the
  separable kernel vs. the direct 4-D summation, both clamped to the
  §7.5 pel range before differencing) so the test isolates the
  numerical precision of the separable kernel from the unavoidable
  `± 0.5` LSB rounding noise of the final integer cast. The two
  deterministic checks — all-zero input → all-zero output, DC-only
  input → flat output — cover the spec-mandated exact cases.

### What round 25 lands

* The **MPEG-2 residual VLC walker** — the §6.2.6 block-layer
  `dct_coeff_first` / `dct_coeff_next` body — per **ISO/IEC 13818-2
  (ITU-T H.262) §7.2.2** with field semantics from §6.2.6 / §7.2.2.4
  and Annex B **Tables B-14** / **B-15** / **B-16**. The MPEG-1
  walker in [`dct_coeff::DctCoeffStep`] from round 17 covers only
  Tables B.5c..B.5f; §7.2.2.3 explicitly notes that the MPEG-2 escape
  encoding is different and §7.2.2.1 introduces a second VLC table
  selected by the `intra_vlc_format` picture-coding-extension flag.
  Both gaps close in this round.
  * `mpeg2_dct_coeff::TableSelection::from_context(intra_vlc_format,
    macroblock_intra)` — the **§7.2.2.1 Table 7-3** selector with the
    four-row truth table:
    | `intra_vlc_format` | intra | non-intra |
    |---|---|---|
    | 0 | B-14 | B-14 |
    | 1 | B-15 | B-14 |
    So Table B-15 is reached **only** when `intra_vlc_format = 1`
    **and** the macroblock is intra; every other row stays on
    Table B-14.
  * `mpeg2_dct_coeff::DctCoeffStep::parse(br, table, position)` —
    the actual walker. Implements **§7.2.2.2** NOTE 2 / NOTE 3 — the
    FIRST-only `1s` (1-bit) and the NEXT-only `11s` (2-bit)
    alternates for B-14's `(run = 0, level = ±1)`. The §7.2.2.2 note
    clarifies that this modification is only meaningful when B-14
    decodes a **non-intra** block, since the first coefficient of
    an intra block is the §7.2.1 DC value handled by
    [`block_dc::DcCoefficient`] — so the walker always starts at the
    second coefficient for intra blocks. B-15 is therefore always
    entered at NEXT and has no NOTE 2 / NOTE 3 split (its `(0, 1)`
    row is the unambiguous 2-bit `10s`).
  * The table-dependent `end_of_block` codeword:
    * Table B-14: 2-bit `10` (same encoding as MPEG-1 Table B.5c,
      modelled separately because it has no sign bit).
    * Table B-15: 4-bit `0110`. The wider EoB is one of the key
      shape differences between the two MPEG-2 tables.
  * The §7.2.2.3 **Table B-16 escape** payload:
    * 6-bit `escape_prefix` = `000001` (shared with both VLC tables).
    * 6-bit `run` (`0..=63`).
    * 12-bit `signed_level` in two's complement. The spec range is
      `[-2047, +2047] \ {0}` — both the all-zeros wire word
      (`signed_level = 0`) and the `0x800` wire word (which would
      represent `-2048`) are explicitly forbidden per the listed
      `1000 0000 0001` (-2047) lower bound. The walker rejects both.
  * `mpeg2_dct_coeff::CoefficientPosition` (`First` / `Next`),
    `mpeg2_dct_coeff::DctCoeff::{RunLevel, EndOfBlock}`, and
    `mpeg2_dct_coeff::DctCoeffStep` mirror the MPEG-1 walker's
    shape so the downstream slice-decoder driver can dispatch on
    the picture-extension flag without two parallel decoded-symbol
    types. Re-exported at the crate root as `Mpeg2VlcTable`,
    `Mpeg2CoefficientPosition`, `Mpeg2DctCoeff`, and
    `Mpeg2DctCoeffStep` so downstream callers spell out the
    MPEG-2-vs-MPEG-1 distinction at the use site.
* **24 new unit tests** in `src/mpeg2_dct_coeff.rs` pin Tables B-14
  / B-15 / B-16 and the §7.2.2 walker semantics:
  * Selector — every row of §7.2.2.1 Table 7-3.
  * Table shape — B-14 has exactly 112 codeword rows (32 + 32 + 32 +
    16, both NOTE 2 / NOTE 3 alternates for `(0, 1)` counted), B-15
    has exactly 111 (31 + 32 + 32 + 16, no alternate). Every code
    fits its declared width; codes within each width are unique;
    the full per-table codebook (codewords + EoB + escape) is
    prefix-free at FIRST and NEXT.
  * Round-trips — every B-14 row at NEXT (with both signs, ≈224
    cases) and every B-15 row at NEXT (≈222 cases) emit-then-parse
    back to the same `(run, signed_level)`. The B-14 FIRST-only
    `1s` form and the NEXT-only `11s` form are exercised separately
    against `Position::First` / `Position::Next`.
  * EoB — both `10` (B-14) and `0110` (B-15) decode to
    `DctCoeff::EndOfBlock`.
  * Escape — Table B-16 round-trip across positive and negative
    extremes (including `±2047`), the forbidden `signed_level = 0`
    wire word, and the forbidden wire word `0x800` (= -2048). Run
    field exercises `0..=63`.
  * Block walks — B-14 non-intra: `FIRST (0, +3)` → `NEXT (2, -1)` →
    `NEXT escape (4, +1500)` → `NEXT EoB`; B-15 intra:
    `NEXT (0, +1)` → `NEXT (0, +2)` → `NEXT escape (20, -1234)` →
    `NEXT EoB`. Both walks confirm the per-step bit-position
    accounting and the cross-step state for the §7.2.2.4
    pseudo-code.
  * Error paths — empty buffer returns `Error::ShortHeader`; a 24-zero
    prefix returns `Error::InvalidBitstream` (no Table B-14 codeword,
    no escape, no EoB).

  Round 25 closes the headline residual-VLC gap. The downstream
  pieces still missing from a complete §6.2.6 block iterator are
  (a) the §7.2.1 intra-DC prelude for MPEG-2 (Tables B-12 / B-13 — the
  MPEG-1 analogue B.5a / B.5b is in [`block_dc`]) and (b) the §7.3
  inverse-scan (`alternate_scan` 0 / 1 from Figure 7-2 / Figure 7-3).
  Both are noted as next-round work.

### What round 26 lands

* The **MPEG-2 §7.3 inverse-scan** body per **ISO/IEC 13818-2
  (ITU-T H.262) §7.3** — the second of the two next-round
  candidates flagged by round 25. The walker output from round 25
  (`mpeg2_dct_coeff::DctCoeffStep`) emits `(run, signed_level)` pairs
  along a `QFS[n]` cursor in `0..=63`; §7.3 maps that
  one-dimensional list back into the two-dimensional `QF[v][u]` block
  consumed by the §7.4 inverse-quantisation pipeline. The matrix
  selection is driven by the `alternate_scan` flag carried in the
  picture coding extension (already parsed by
  [`picture_header::PictureCodingExtension::alternate_scan`]).
  * `mpeg2_inverse_scan::ALTERNATE_SCAN` — the **Figure 7-3**
    `scan[1][v][u]` matrix (alternate scan, `alternate_scan = 1`).
    The matrix's distinguishing feature is that it walks down
    column 0 first
    (`scan[1][0..=3][0] = {0, 1, 2, 3}`,
    `scan[1][4..=7][0] = {10, 11, 12, 13}`) — the contrasting
    "across-then-down" first column of Figure 7-2
    (`{0, 2, 3, 9, 10, 20, 21, 35}`) makes the two scans visually
    obvious. The Figure 7-2 (`scan[0][v][u]`, zig-zag) matrix is
    not re-encoded — it is identical cell-for-cell to the MPEG-1
    §2.4.4.1 `scan[m][n]` matrix already in
    [`block_dc::SCAN`], and a unit test
    (`figure_7_2_equals_block_dc_scan_cell_for_cell`) asserts the
    match so any future drift on either side trips a regression
    immediately.
  * `mpeg2_inverse_scan::ALTERNATE_INVERSE_SCAN` and
    `mpeg2_inverse_scan::ZIGZAG_INVERSE_SCAN` — the inverse maps,
    each `(u8, u8); 64` indexed as
    `INVERSE[n] = (v, u)`. Both are derived at compile time from
    their forward partners (and `ZIGZAG_INVERSE_SCAN` is asserted
    against `block_dc::INVERSE_SCAN` in
    `zigzag_inverse_scan_matches_block_dc_inverse_scan`).
  * `mpeg2_inverse_scan::scan_table(alternate_scan: bool)` and
    `mpeg2_inverse_scan::inverse_scan_table(alternate_scan: bool)`
    — flag-driven selectors that return `&'static` refs to the
    chosen matrix / inverse map, matching the §7.3 spelling
    `scan[alternate_scan][v][u]`.
  * `mpeg2_inverse_scan::place_coefficient(qf, index, value,
    alternate_scan)` — the per-coefficient writer that mates with
    round 25's `Mpeg2DctCoeffStep`: each `RunLevel` symbol
    advances the cursor `n += 1 + run`, and `place_coefficient`
    writes the level into `qf[v][u]` at the `(v, u)` named by the
    selected inverse-scan map. Bounds-checked at `index < 64`.
  * `mpeg2_inverse_scan::apply_inverse_scan(qfs, alternate_scan)`
    — the direct transliteration of the §7.3 pseudo-code
    `for (v) for (u) QF[v][u] = QFS[scan[alternate_scan][v][u]]`
    for callers that have already accumulated a 64-entry flat list
    (encoder forward pass, trace tools).
  * §7.3.1 (matrix-download flag invariant) — the docstring on
    `scan_table` reminds callers that quantisation-matrix downloads
    always use `scan[0]` regardless of the picture-coding-extension
    flag, so the caller passes `false` rather than the live bit.
  * Re-exported at the crate root with explicit `MPEG2_…`
    spellings (`MPEG2_ALTERNATE_SCAN`,
    `MPEG2_ALTERNATE_INVERSE_SCAN`, `MPEG2_ZIGZAG_INVERSE_SCAN`,
    `mpeg2_scan_table`, `mpeg2_inverse_scan_table`,
    `mpeg2_place_coefficient`, `mpeg2_apply_inverse_scan`) so
    downstream call sites spell out the MPEG-2-vs-MPEG-1
    distinction at the use site.
* **21 new lib unit tests** in `src/mpeg2_inverse_scan.rs` pin
  the §7.3 invariants: both scans are permutations of `0..=63`;
  Figure 7-2 == `block_dc::SCAN` cell-for-cell; the
  `ZIGZAG_INVERSE_SCAN` constant equals `block_dc::INVERSE_SCAN`
  entry-for-entry; Figure 7-3 corners (0,0)=0 / (0,7)=52 /
  (7,0)=13 / (7,7)=63 and rows 0 / 7 match the printed page 80;
  Figure 7-3 column 0 walks down `{0,1,2,3,10,11,12,13}`;
  cells where scan[0] and scan[1] differ ((0,1) → 1 vs 4,
  (1,0) → 2 vs 1) are pinned; the forward · inverse round-trip
  closes for all 64 cells in both scans; the flag selectors
  return value-equal tables for each branch; `place_coefficient`
  writes only one cell with all 63 others untouched, agrees with
  a hand-traced sample under both scan flags, and the index-0 /
  index-63 corners match for both scans (DC at (0,0); the last
  coefficient at (7,7)); a bad index panics with a clean
  spec-named message; `apply_inverse_scan` round-trips a
  synthetic QF block through QFS and back under both scan flags;
  and `apply_inverse_scan` agrees with a loop of
  `place_coefficient` calls for every index in
  `-30..=33`.
* **7 new integration tests** in
  `tests/mpeg2_inverse_scan_synthetic.rs` re-pin Figures 7-2 / 7-3
  cell-for-cell against an independent verbatim copy of the
  spec page 80, confirm MPEG-1 `SCAN` and MPEG-2 `scan[0]` agree,
  exercise the inverse-table round-trip across the public
  re-exports, check the constant / function partners produce
  identical data, and replay a synthetic §7.2.2 walker emission
  stream through `place_coefficient` and through the full §7.3
  loop body for both scan flags — confirming the two equivalent
  expressions of the §7.3 pseudo-code agree at every step.

  Round 26 closes the §7.3 inverse-scan gap noted at the end of
  round 25. The §6.2.6 block-iterator skeleton can now combine
  the §7.2.2 walker (round 25) and the §7.3 placement (this round)
  to materialise a `QF[v][u]` block ready for the round-23
  `mpeg2_inverse_quantise_block`. The remaining round-25 next-step
  candidate (a) — the §7.2.1 MPEG-2 intra-DC prelude (Tables B-12
  / B-13) — is now the obvious next-round work.

### What round 27 lands

* The **MPEG-2 §7.2.1 intra-block DC prelude** per **ISO/IEC
  13818-2 (ITU-T H.262) §7.2.1** — the very gap flagged as the
  remaining round-25 next-step candidate at the end of the round-26
  notes. With this in hand the §6.2.6 block-iterator skeleton can
  consume the DC-coefficient prelude (this round) and chain it
  into the §7.2.2 residual walker (round 25) + §7.3 inverse-scan
  placement (round 26) + §7.4 inverse-quantisation pipeline
  (round 23) — i.e. the full intra-block path from bitstream to a
  ready-to-IDCT `F[v][u]` matrix is now spec-complete on the
  MPEG-2 side.
  * `mpeg2_block_dc::TABLE_B12` — **Annex B Table B-12**
    (`dct_dc_size_luminance`) sized to `0..=11`. The first 9 rows
    (sizes `0..=8`) are bit-exact MPEG-1's Table B.5a; sizes 9, 10,
    and 11 extend the prefix with `1111 1110`, `1111 1111 0`, and
    `1111 1111 1` to accommodate the wider DC differentials needed
    when `intra_dc_precision != 0`. A unit test
    (`b12_first_9_rows_match_b5a`) drives every size-0..=8 codeword
    through both the MPEG-1 `block_dc::DcCoefficient::parse` and
    the new MPEG-2 walker to pin the equivalence.
  * `mpeg2_block_dc::TABLE_B13` — **Annex B Table B-13**
    (`dct_dc_size_chrominance`), similarly sized to `0..=11` with
    the new long-prefix entries `1111 1111 0` / `1111 1111 10` /
    `1111 1111 11`. The first 9 rows match Table B.5b bit-for-bit
    and a parallel test (`b13_first_9_rows_match_b5b`) pins the
    equivalence.
  * `mpeg2_block_dc::DcComponent` — table-selector enum (`Luminance`
    for `cc == 0` / `Chrominance` for `cc != 0`), distinct from the
    per-component predictor routing because Cb and Cr share Table
    B-13 but each have their own predictor cell.
  * `mpeg2_block_dc::ColourComponent` — `Y` / `Cb` / `Cr` per
    Table 7-1, projects to `DcComponent` via
    `colour.dc_component()` for table selection.
  * `mpeg2_block_dc::DcPredictors` — the per-component
    `dc_dct_pred[Y / Cb / Cr]` predictor state, primed via
    `DcPredictors::new(intra_dc_precision)` to the §7.2.1 reset
    value and resettable via `DcPredictors::reset()` per the §7.2.1
    three-trigger contract (start of slice, non-intra macroblock,
    skipped macroblock).
  * `mpeg2_block_dc::dc_pred_reset_value(intra_dc_precision)` —
    direct **Table 7-2** lookup returning `{128, 256, 512, 1024}`
    for `intra_dc_precision ∈ {0, 1, 2, 3}`; rejects values
    outside `0..=3` (Table 6-13 only defines those four).
  * `mpeg2_block_dc::qfs_zero_max(intra_dc_precision)` —
    `2^(8 + intra_dc_precision) - 1` per the §7.2.1 bitstream
    constraint *"QFS[0] shall lie in the range 0 to ((2^(8 +
    intra_dc_precision)) - 1)"*. Returns `{255, 511, 1023, 2047}`.
  * `mpeg2_block_dc::decode_dc_block(br, predictors, colour)` —
    end-to-end driver: pulls the `dct_dc_size` VLC (Table B-12 /
    Table B-13 selected by `colour.dc_component()`), reads the
    `dct_dc_size`-bit `dc_dct_differential`, reconstructs
    `dct_diff` per §7.2.1 (the `half_range = 2^(dct_dc_size - 1)`
    threshold-test form, which is mathematically equivalent to
    MPEG-1's §2.4.3.7 MSB-test form — a unit test
    `mpeg2_recon_matches_mpeg1_for_sizes_1_through_8` walks every
    `(size, raw)` pair within the MPEG-1 range to pin it), adds
    the §7.2.1 predictor `dc_dct_pred[cc]`, asserts the §7.2.1
    `[0, qfs_zero_max(precision)]` bitstream constraint on the
    resulting `QFS[0]`, then updates the predictor cell for the
    component. Returns a typed `DcCoefficient` record carrying the
    raw bits, the signed `dct_diff`, the final `QFS[0]`, and the
    post-consume bit position.
  * Re-exported at the crate root with explicit `Mpeg2…` /
    `MPEG2_…` spellings (`Mpeg2DcComponent`, `Mpeg2DcPredictors`,
    `Mpeg2DcCoefficient`, `Mpeg2ColourComponent`,
    `MPEG2_MAX_DC_SIZE`, `mpeg2_decode_dc_block`,
    `mpeg2_dc_pred_reset_value`, `mpeg2_qfs_zero_max`) so the
    MPEG-2-vs-MPEG-1 distinction stays explicit at call sites.
* **29 new lib unit tests** in `src/mpeg2_block_dc.rs` pin
  Tables B-12 / B-13's `0..=11` cardinality + uniqueness per
  width + width-correctness, the bit-exact match with MPEG-1's
  B.5a / B.5b on the first 9 rows, the §7.2.1 reconstruction
  formula's equivalence to MPEG-1 §2.4.3.7 across every
  `(size ≤ 8, raw)` pair, the page-77 spec-example
  reconstruction table at `dct_dc_size = 3` (raw `000..=111` →
  dct_diff `-7..=+7`), the size-11 corner values (-2047, -2046,
  -1024, 1024, 2047), Table 7-2 reset values (128 / 256 / 512 /
  1024), the `qfs_zero_max` matching `2^(8 + precision) - 1`
  (255 / 511 / 1023 / 2047), the predictor lifecycle (`new`
  primes all three to reset; `reset` returns all three to
  Table 7-2 value), the Y / Cb / Cr per-component routing
  (independent predictor cells, Y-Y chain preserves Cb / Cr at
  reset), the §7.2.1 bitstream constraint enforcement (negative
  QFS[0] rejected, above-max QFS[0] rejected,
  precision = 3 widens the window), the bit-position accounting
  for size 0 (3 code bits, 0 differential) and size 11 (9 code
  bits, 11 differential), and the `ColourComponent` → `DcComponent`
  projection.
* **7 new integration tests** in
  `tests/mpeg2_block_dc_synthetic.rs` re-pin Table 7-2 across all
  four `intra_dc_precision` values through the public re-exports,
  exercise a Y-Y-Cb-Cr four-block predictor chain confirming the
  per-component independence at the public-API level, walk a
  reset cycle, drive a size-9 underflow rejection at
  `intra_dc_precision = 1`, and round-trip the long-prefix
  size-10 (B-13) and size-11 (B-12) codewords through
  `mpeg2_decode_dc_block`.

  Round 27 closes the round-25 / round-26 next-step candidate
  (a). With the §7.2.1 prelude + §7.2.2 residual walker + §7.3
  inverse scan all spec-complete on the MPEG-2 side, the
  §6.2.6 block-iterator skeleton now has all three of its
  intra-block building blocks ready, and the remaining gap on
  the MPEG-2 path is the §6.2.6 driver itself (which chains
  prelude → walker → scan → §7.4 inverse-quantise → §A IDCT into
  a per-block decode entry-point).

### What round 28 lands

* The **§6.2.6 `block(i)` driver** per **ISO/IEC 13818-2 (ITU-T
  H.262) §6.2.6** — the missing block-level composition step the
  round-27 notes flagged as the remaining MPEG-2 gap. With this
  in hand the full intra-block path from the bitstream cursor to
  a `f[y][x]` plane ready for §7.6.8 add-and-saturate runs in a
  single call: §7.2.1 DC prelude (intra blocks only) → §7.2.2
  residual VLC walker (with the §7.2.2.2 NOTE 2 / NOTE 3 FIRST
  / NEXT alternation honoured by the walker) → §7.3 inverse
  scan (Figure 7-2 / Figure 7-3 keyed off `alternate_scan`) →
  §7.4 inverse-quantisation pipeline (saturation + §7.4.4
  mismatch control included) → §A 8×8 IDCT, all chained off the
  already-landed sibling endpoints with no duplication of their
  spec-pinned arithmetic.
  * `mpeg2_block_decoder::BlockContext` — groups the
    per-macroblock constants the §6.2.6 driver needs:
    `intra_vlc_format` (Table 7-3 selector), `alternate_scan`
    (§7.3 Figure 7-2 / Figure 7-3 selector), `intra_dc_precision`
    (§7.2.1 reset value + Table 7-4 `intra_dc_mult`), and
    `quantiser_scale_value` (post-Table 7-6 resolved scale).
    Per-block parameters (`component`, `macroblock_intra`,
    `weight`) move with each call so a macroblock-level driver
    can dispatch to the same `BlockContext` for all 6 / 8 / 12
    blocks of a macroblock.
  * `mpeg2_block_decoder::DecodedBlock` — captures every
    intermediate plane: the 64-entry `QFS[]` out of the walker,
    the §7.3 inverse-scan output `QF[v][u]`, the §7.4
    inverse-quantisation output `F[v][u]`, and the §A IDCT
    output `f[y][x]` (already saturated to `[-256, +255]` per
    §7.5). Carries the post-EOB bit cursor for round-trip
    accounting against the caller's `BitReader`.
  * `mpeg2_block_decoder::decode_block(br, ctx, dc_predictors,
    component, macroblock_intra, weight)` — the §6.2.6 driver
    entry point. For an intra block the §7.2.1 path advances
    the walker cursor to zig-zag index 1 and switches the §7.2.2
    walker to `Position::Next` (because the DC slot has already
    been consumed); for a non-intra block the walker starts at
    zig-zag index 0 with `Position::First`. The §7.2.2 spec
    constraint *"the position of the coefficient ... shall not
    exceed 63"* is enforced as an `InvalidBitstream` rejection
    on any `walker_index + run ≥ 64`. Per-call inputs are
    validated up-front (`intra_dc_precision ≤ 3`,
    `quantiser_scale_value ≠ 0`, predictor precision matches
    context precision) so a downstream driver doesn't have to
    re-prove the §6.3.7 / §7.4.2.2 invariants.
  * Re-exported at the crate root as `mpeg2_decode_block`,
    `Mpeg2BlockContext`, `Mpeg2DecodedBlock` so a caller picking
    between the MPEG-1 and MPEG-2 paths keeps the
    stream-type-distinct spelling at every call site (matches
    the existing `mpeg2_decode_dc_block` /
    `mpeg2_inverse_quantise_block` convention).
* **16 new lib unit tests** in `src/mpeg2_block_decoder.rs`:
  size-0 DC + immediate EOB intra block (predictor reset value
  flows through to `QF[0][0]` → `F[0][0]`), size-1 positive DC
  (predictor walk 128 → 129), Cb chroma routing (Table B-13 +
  Cb predictor cell, leaving Y / Cr undisturbed), non-intra
  block rejection on FIRST-position EOB (which is NEXT-only per
  §7.2.2.2), non-intra FIRST `(0, +1)` → `QFS[0]`, §7.4 non-intra
  arithmetic `(2*QF + Sign(QF))*W*Qs/32 = 12` for `QF = +1`,
  intra block with NEXT `(0, +1)` placing at zig-zag index 1
  (the §7.2.2.2 alternation in action), intra block with run = 3
  placing at zig-zag index 4 (cursor accounting), `alternate_scan`
  remapping `QFS[1]` away from the zig-zag `(0, 1)` cell,
  end-of-block bit-position reporting matches the reader's
  cursor, `quantiser_scale_value == 0` rejection (Table 7-6),
  `intra_dc_precision > 3` rejection (Table 6-13),
  predictor-vs-context precision-mismatch rejection,
  `intra_dc_precision = 1` Table 7-2 reset-value chain (predictor
  = 256, `F[0][0]` = `4 * 256 = 1024` per Table 7-4 `intra_dc_mult`),
  `intra_dc_mult_local` matches Table 7-4 across `0..=3`, and
  the §7.2.2 `run + position ≤ 63` constraint enforcement
  (Table B-16 escape with run = 63 at walker_index = 1 → reject).
* **7 new integration tests** in
  `tests/mpeg2_block_decoder_synthetic.rs` exercise the public
  re-exports end-to-end: Y / Cb / Cr per-component predictor
  independence, three-block Y chain accumulating positive
  `dct_diff` (128 → 129 → 130 → 131), cursor accounting across
  two concatenated blocks (5 + 5 = 10 bits), non-intra round-trip
  through `DEFAULT_NON_INTRA_WEIGHT`, two-non-intra-block chain
  confirming §7.2.1 predictor updates skip non-intra blocks,
  `Mpeg2BlockCoding` enum convention pin, and a 4:2:0 six-block
  macroblock skeleton (Y0..Y3 + Cb + Cr) walking 28 bits of
  bitstream.

  Round 28 closes the round-27 next-step candidate. The §6.2.6
  block-level entry point is now wire-complete on the MPEG-2
  side; the remaining gap on the MPEG-2 decode path is the
  macroblock-level driver that walks `pattern_code[12]` to
  dispatch `decode_block` once per coded block (the existing
  [`macroblock_pipeline::decode_macroblock`] is shaped for this
  but currently takes pre-decoded `BlockInputs` rather than a
  raw `BitReader`; the natural round-29 work is a wrapper that
  drives the existing macroblock_pipeline by calling
  `mpeg2_decode_block` per coded slot).

### What round 29 lands

* The **§6.2.5 / §6.2.6 macroblock-block driver** per **ISO/IEC
  13818-2 (ITU-T H.262)** — the wrapper round 28 flagged as the
  natural follow-up. `mpeg2_macroblock_blocks::decode_macroblock_blocks`
  consumes a `BitReader` positioned at the first block's syntax
  start, the parsed [`MacroblockType`], the parsed
  [`CodedBlockPattern`], a [`mpeg2_block_dc::DcPredictors`]
  reference, and a [`mpeg2_macroblock_blocks::MacroblockBlockContext`]
  carrying the per-macroblock constants (`intra_vlc_format`,
  `alternate_scan`, `intra_dc_precision`, `quantiser_scale_value`,
  [`ChromaFormat`], and the four §6.3.7 / §6.3.11.1 weighting
  matrices). It then walks `pattern_code[12]` and dispatches the
  round-28 §6.2.6 driver
  ([`mpeg2_block_decoder::decode_block`]) once per coded slot,
  returning a `Vec<DecodedBlock>` paired with the §6.1.1.8
  block-index position.
  * `mpeg2_macroblock_blocks::block_count(chroma_format)` —
    Figures 6-10 / 6-11 / 6-12: `Yuv420 → 6`, `Yuv422 → 8`,
    `Yuv444 → 12`. Matches
    [`macroblock_pipeline::blocks_per_macroblock`] (re-pinned
    side-by-side so the macroblock-block driver is self-contained
    for audit).
  * `mpeg2_macroblock_blocks::block_component(i, chroma_format)`
    — Figures 6-10 / 6-11 / 6-12: indices `0..=3` → Y for every
    chroma format; for 4:2:0 index 4 → Cb, 5 → Cr; for 4:2:2
    indices 4..=5 → Cb, 6..=7 → Cr; for 4:4:4 indices 4..=7 → Cb,
    8..=11 → Cr. Returns `None` past `block_count(chroma_format)`.
  * `mpeg2_macroblock_blocks::MacroblockBlockContext` /
    `DEFAULT_WEIGHT_MATRICES` — the four §6.3.7 default weighting
    matrices indexed by §7.4.2.1 Table 7-5 `w ∈ {0, 1, 2, 3}`
    (intra luma / non-intra luma / intra chroma / non-intra
    chroma). The convenience constructor
    `MacroblockBlockContext::with_default_weight_matrices` returns
    a context bound to the static defaults.
  * §7.4.2.1 Table 7-5 weighting-matrix dispatch — the driver
    auto-derives `w` per coded block from `(coding, component,
    chroma_format)` via
    [`mpeg2_dequantize::select_weighting_matrix_index`] and
    looks up the matching matrix in
    `context.weight_matrices[w]` before passing it down to the
    round-28 inner driver. For 4:2:0 the chroma matrix selector
    collapses to the luma one (Table 7-5 row); for 4:2:2 / 4:4:4
    the chroma indices `w ∈ {2, 3}` route to the chroma matrices.
  * §7.2.1 non-intra-macroblock DC-predictor reset — the driver
    calls `DcPredictors::reset()` at every non-intra macroblock
    before walking blocks, per the §7.2.1 three-trigger reset
    contract (slice-start and skipped-macroblock resets remain a
    slice-layer concern). For intra macroblocks the predictors
    are *preserved* at MB entry and update per block per
    Table 7-1, as the spec requires.
  * Up-front argument validation — the driver rejects
    `intra_dc_precision > 3` (Table 6-13), `quantiser_scale_value
    == 0` (Table 7-6 forbidden), and the
    `DcPredictors.intra_dc_precision != context.intra_dc_precision`
    mismatch as `InvalidBitstream`, so a misconfigured call site
    surfaces at the macroblock-block driver entry rather than
    deep in the round-28 stack frame.
  * Re-exported at the crate root as `mpeg2_decode_macroblock_blocks`,
    `Mpeg2MacroblockBlockContext`, `Mpeg2MacroblockDecodedBlock`,
    `mpeg2_block_component`, `mpeg2_block_count`,
    `MPEG2_DEFAULT_WEIGHT_MATRICES` — keeping the
    stream-type-distinct spelling at every call site (matches the
    existing `mpeg2_decode_block` / `mpeg2_decode_dc_block`
    convention).
* **15 new lib unit tests** in `src/mpeg2_macroblock_blocks.rs`:
  the §6.1.1.8 block-index → component mapping for all three
  chroma formats (4:2:0 Y-vs-Cb-vs-Cr at indices 0..=5 and `None`
  past 5; 4:2:2 Cb at 4..=5 / Cr at 6..=7; 4:4:4 Cb at 4..=7 /
  Cr at 8..=11), the `block_count(chroma_format)` table, a
  six-block intra walk for 4:2:0 (all blocks decode with QFS[0]
  = 128 and a constant `f[y][x]` plane), the `pattern_code[]`
  gating (uncoded slots not consumed from the bitstream — verified
  by emitting only two block bodies and confirming `cbp =
  0b010100` walks blocks 1 and 3), the §7.2.1 non-intra-MB
  predictor reset (seeded predictors snap back to 128 with no
  blocks walked), the intra-MB predictor preservation at MB
  entry (a seeded Y predictor of 200 carries through), Table 7-5
  weighting-matrix dispatch (a 4:4:4 non-intra Cb block routes to
  `w = 3` and surfaces F[0][0] = 24 vs the luma-matrix would-be
  value 12), and the three argument-validation paths plus the
  first-failing-block propagation (a truncated chroma block on
  block 4 surfaces as Short or InvalidBitstream).
* **6 new integration tests** in
  `tests/mpeg2_macroblock_blocks_synthetic.rs` exercise the public
  re-exports end-to-end: a six-block 4:2:0 intra walk in
  Figure 6-10 order (Y-Y-Y-Y-Cb-Cr), an eight-block 4:2:2 intra
  walk in Figure 6-11 order (4 Y then 2 Cb then 2 Cr), a
  twelve-block 4:4:4 intra walk in Figure 6-12 order (4 Y then
  4 Cb then 4 Cr), the per-component predictor chain across four
  luma blocks (128 → 129 → 130 → 131 → 132 with `dct_diff = +1`
  each, Cb / Cr cells staying at the reset value 128), the
  non-intra-MB-with-no-coded-blocks predictor reset (Y/Cb/Cr
  snap back to 128 from seeded `500 / 600 / 700`), and the
  bit-cursor accounting across the six-block 4:2:0 walk (4 × 5
  luma + 2 × 4 chroma = 28 bits, matching the last block's
  reported post-EOB bit position).

  Round 29 closes the round-28 next-step candidate. The §6.2.5 /
  §6.2.6 macroblock-block driver is now wire-complete on the
  MPEG-2 side, sitting one layer above the round-28 §6.2.6
  `block(i)` driver and one layer below the still-pending
  slice-layer driver. The remaining gap on the MPEG-2 decode
  path is now the slice-layer driver itself — the loop that
  parses `macroblock_address_increment` /
  `macroblock_type` / `coded_block_pattern` / motion vectors per
  MB and dispatches to `mpeg2_decode_macroblock_blocks` once per
  coded macroblock — which is the natural round-30 work.

### What round 30 lands

* The **§6.2.4 slice-level macroblock-header walker** per
  **ISO/IEC 13818-2 (ITU-T H.262)** — the loop round 29 flagged
  as the natural follow-up. `slice_macroblock_walk::walk_slice`
  picks up at the post-`slice_header()` cursor and walks the
  `do { macroblock() } while ( nextbits() != '0000 0000 0000
  0000 0000 0000' )` loop from page 51 of 13818-2:1995, parsing
  each macroblock's spec-deterministic header chain into a
  `MacroblockRecord` and accumulating the §6.3.17.1 per-slice
  state.
  * Per-MB header chain: §6.2.5
    `macroblock_address_increment` (Table B-1 VLC with the
    `macroblock_escape` chain and the MPEG-1
    `macroblock_stuffing` no-op, via
    [`crate::MbAddressIncrement::parse`]), §6.2.5.1
    `macroblock_modes()` opener (`macroblock_type` against
    Tables B-2 / B-3 / B-4 keyed on the picture's
    `picture_coding_type`, via
    [`crate::MacroblockType::parse`]), and the conditional
    5-bit macroblock-level `quantiser_scale_code` when
    `macroblock_quant == 1` (range `1..=31`, §6.3.16
    forbidden-zero check enforced).
  * §6.3.17.1 per-slice state: `previous_macroblock_address`
    seeded from `mb_row * mb_width - 1`, `macroblock_address =
    previous_macroblock_address + macroblock_address_increment`
    per MB, `past_intra_address` initialised to
    [`crate::PAST_INTRA_ADDRESS_RESET`] (`-2`) at picture
    start and advanced to `macroblock_address` after every
    intra MB, `quantiser_scale_code` carried forward from the
    slice header and overridden by any MB with
    `macroblock_quant == 1` (override applies to *this* MB
    and every subsequent MB in the slice). First-MB
    `macroblock_address_increment != 1` violation rejected
    per §6.3.17.1.
  * Skipped-MB accounting: each `MacroblockRecord` carries
    `skipped_macroblock_count = address_increment - 1`,
    surfacing the §6.3.17.4 / §7.6.6 skipped-MB ranges so a
    future §7.6.6 round can reconstruct them. The driver
    does not run the §7.6.6 skipped-MB reconstruction itself.
  * §6.2.4 stop-condition: alignment-agnostic per §5.2.3, the
    loop exits as soon as `nextbits()` peeks 23 zero bits at
    the current cursor or the buffer runs out (the caller
    bounds the slice sub-buffer; the trailing
    `next_start_code()` invocation is the caller's
    responsibility once the do-while exits).
  * Per-MB `body_bit_position` surfaces the cursor right after
    the macroblock-header chain — i.e. the entry point for the
    deferred `macroblock_modes()` tail (motion-type / dct_type),
    `motion_vectors(s)`, `coded_block_pattern()`, and the
    per-block walker rounds. Those parsers intersect with
    cross-MB state (PMV reset / f_code / per-block
    `BlockContext`) that the picture-level driver above this
    walker will own, so they remain explicitly out of scope
    this round.
  * Up-front argument validation rejects
    `initial_quantiser_scale_code == 0` (forbidden per §6.3.16)
    and `mb_width == 0` (no legal sequence) so a misconfigured
    call site surfaces at driver entry rather than mid-loop.
  * Re-exported at the crate root as `walk_slice`,
    `SliceWalk`, `SliceWalkContext`, `MacroblockRecord`,
    `PAST_INTRA_ADDRESS_RESET` — keeping the surface flat and
    consistent with the existing `Mpeg2*` naming on the
    decode-pipeline modules.
* **10 new lib unit tests** in `src/slice_macroblock_walk.rs`:
  the immediate-stop-pattern empty-slice case, the single
  intra macroblock in an I-picture (Table B-2 `1`), the
  `Intra-Quant` override + carry-forward to a following
  no-quant intra MB (verifies the §6.3.17.1
  "applies to subsequent MBs" semantics), the first-MB
  `address_increment != 1` rejection, the P-picture
  skipped-MB recording with `mb_row = 1` (start addr 22,
  increment 3 → 2 skipped MBs at addr 22..=24), the
  three-MB intra walk where `past_intra_address` advances
  to each MB's address (0 → 1 → 2), zero
  `initial_quantiser_scale_code` rejection, zero
  `mb_width` rejection, the `body_bit_position` accounting
  (the post-header cursor lands at bit 2 after a 1-bit
  increment and a 1-bit `macroblock_type`), and a
  three-MB explicit override-then-reset walk (q=7 → carry
  7 → q=15) that pins the final
  [`SliceWalk::quantiser_scale_code`] at 15.
* **5 new integration tests** in
  `tests/slice_macroblock_walk_synthetic.rs` exercise the
  public re-exports end-to-end: a hand-built
  slice-start-code-prefixed buffer chained through
  `SliceHeader::parse` + `walk_slice` for a two-intra-MB
  I-picture slice (verifies the post-`slice_header()`
  cursor reaches the body), the P-picture skipped-MB
  recording across a `mb_row=1` slice, an override + reset
  three-MB walk pinning the cross-MB carry-forward
  semantics, the empty-slice-body early-exit on the
  23-zero-bit stop pattern, and the first-MB
  increment-2 rejection.

  Round 30 closes the round-29 next-step candidate on the
  slice-driver side: the §6.2.4 do-while loop now sequences
  the macroblock-header chain across an entire slice with the
  §6.3.17.1 state mechanics in place. The remaining gap on
  the MPEG-2 decode path is the per-MB **body** — `macroblock_modes()`
  tail / `motion_vectors(s)` / `coded_block_pattern()` / the
  per-block walker — and the §7.6.6 skipped-MB reconstruction
  flagged by round 30 as a follow-up. Round 31 picks up the
  latter.

### What round 31 lands

* The **§7.6.6 skipped-macroblock specification** per
  **ISO/IEC 13818-2 (ITU-T H.262)** — the description module
  the round-30 slice walker flagged as a follow-up.
  `skipped_macroblock::describe_skipped_macroblock` consumes a
  `SkippedMacroblockContext` (the picture's coding type and
  structure from `picture_header()` / `picture_coding_extension()`,
  the previous MB's prediction direction, the PMV state, and
  the I-picture scalability gate) and returns a
  `SkippedMacroblock` description that pins the per-§7.6.6.1..4
  deterministic prediction shape: the prediction type
  (Frame-based for §7.6.6.2 / §7.6.6.4, Field-based for
  §7.6.6.1 / §7.6.6.3), the derived `mv_format`, the
  same-parity field reference (§7.6.6.1 / §7.6.6.3 only), the
  prediction direction (always `Forward` in P-pictures —
  §7.6.6.1 / §7.6.6.2 implicit zero-MV forward prediction;
  inherited from the previous MB in B-pictures per §7.6.6.3 /
  §7.6.6.4 "same as the previous macroblock"), the motion-vector
  source (`SkippedMotionVector::Zero` for P-pictures —
  §7.6.6.1 / §7.6.6.2 "the motion vector shall be zero";
  `SkippedMotionVector::FromPmv { forward, backward }` for
  B-pictures — §7.6.6.3 / §7.6.6.4 "motion vectors are taken
  from the appropriate motion-vector predictors", with each
  slot present iff the inherited previous direction includes
  it), and the `reset_pmv` flag (true for P-pictures per
  §7.6.3.4 bullet "In a P-picture when a macroblock is
  skipped" and §7.6.6.1 / §7.6.6.2 "Motion vector predictors
  shall be reset to zero"; false for B-pictures per §7.6.6.3 /
  §7.6.6.4 "Motion vector predictors are unaffected"). The
  companion `skipped_macroblock_apply_to_pmv` hook fires the
  §7.6.3.4 PMV reset on the caller's `Pmv` state when
  `reset_pmv == true` (idempotent; no-op otherwise).
* §7.6.6 preamble rejections: I-picture + `scalable_i_picture
  == false` raises `InvalidBitstream` per "There shall be no
  skipped macroblocks in I-pictures except when…"; B-picture
  + `previous_direction == PredictionDirection::Skipped`
  raises `InvalidBitstream` per "the same as the previous
  macroblock" — the previous MB must have an encoded
  direction. The I-picture scalable-exemption gate is
  exposed for a future scalability round but currently
  rejects with an explicit "not yet supported" message; the
  scalability extensions (`picture_spatial_scalable_extension()` /
  `sequence_scalable_extension()` with `scalable_mode =
  "SNR scalability"`) own the prediction formation in that
  case and are not yet parsed by this crate.
* The module re-uses the existing crate types — `Pmv` /
  `VectorIndex` / `Direction` / `Component` from
  [`pmv`], `PictureCodingType` / `PictureStructure` from
  [`picture_header`], `PredictionType` / `MvFormat` from
  [`macroblock_modes`], `PredictionDirection` from
  [`combine_predictions`], and `FieldParity` from
  [`dual_prime`] — so callers can take a description straight
  into the existing §7.6.4 / §7.6.7 prediction pipeline
  (`predict_block` → `combine_directional_predictions`).
* Re-exported at the crate root as `describe_skipped_macroblock`,
  `SkippedMacroblock`, `SkippedMacroblockContext`,
  `SkippedMotionVector`, and `skipped_macroblock_apply_to_pmv`.
* **15 new lib unit tests** in `src/skipped_macroblock.rs`:
  the §7.6.6.1 P field (top + bottom parity) Field-based /
  zero-MV / PMV-reset descriptions, the §7.6.6.2 P frame
  Frame-based / zero-MV / PMV-reset description, the
  §7.6.6.4 B frame inheriting each of the three previous-MB
  directions (Forward only / Backward only / Bidirectional)
  with the matching PMV slot subset surfaced, the §7.6.6.3
  B field same-parity rule for both top and bottom
  structures, the §7.6.6 preamble rejection on a non-scalable
  I-picture, the "not yet supported" stub on a scalable
  I-picture, the rejection of `previous_direction ==
  Skipped` in a B-picture, the `apply_to_pmv` zeroing in
  P-pictures, the `apply_to_pmv` no-op in B-pictures, the
  idempotence of repeated `apply_to_pmv` calls, and the
  `FieldParity::{Top, Bottom}.index() == {0, 1}` spec
  numbering reaffirmation.
* **5 new integration tests** in
  `tests/skipped_macroblock_synthetic.rs` exercise the
  public re-exports end-to-end: a §7.6.6.2 run where the
  round-30 slice walker's `skipped_macroblock_count = 3` is
  iterated through `describe_skipped_macroblock` /
  `skipped_macroblock_apply_to_pmv` and the PMV converges
  on zero (matches §7.6.3.4 / §7.6.6.2), a §7.6.6.3 B field
  top-parity 5-MB run whose `apply_to_pmv` calls leave the
  full `Pmv` byte-equal (matches "Motion vector predictors
  are unaffected"), a §7.6.6.4 B frame forward-only
  inheritance verifying the backward PMV slot is *not*
  surfaced when the previous MB had no backward direction,
  the §7.6.6-preamble I-picture-non-scalable rejection
  through the public surface, and a 10-MB §7.6.6.1 P field
  (bottom parity) run pinning the convergence-on-zero
  invariant over a long run.

  Round 31 closes the round-30 next-step candidate on the
  §7.6.6 skipped-MB side. The actual sample-plane formation
  for skipped MBs is then a thin glue layer
  (`SkippedMacroblock` → §7.6.4 `predict_block` → §7.6.7
  `combine_directional_predictions`); per-block residuals
  are conceptually zero for skipped MBs so no §7.6.8
  add-coefficients dispatch is needed. The remaining gap on
  the MPEG-2 decode path is still the per-MB **body** —
  `macroblock_modes()` tail / `motion_vectors(s)` /
  `coded_block_pattern()` / the per-block walker the round-30
  walker's `body_bit_position` cursor points at — which is
  the natural round-32 work.

### What round 32 lands

* The §6.2.5.1 **`macroblock_modes()` tail** wired into
  `slice_macroblock_walk::walk_slice`. The slice driver now
  parses, in §6.2.5 syntax-tree order:
    1. `macroblock_address_increment` (Table B-1),
    2. `macroblock_modes()` — `macroblock_type` (Tables
       B-2 / B-3 / B-4) then the new tail bits:
       `frame_motion_type` (Table 6-17) on frame pictures
       with `frame_pred_frame_dct == 0` and any motion flag
       set; `field_motion_type` (Table 6-18) on every
       motion-bearing MB in a field picture; `dct_type` on
       frame pictures with `frame_pred_frame_dct == 0`
       whose MB is intra or has a coded pattern,
    3. `quantiser_scale_code` (5 bits) when
       `macroblock_quant == 1`.
* This **fixes a latent ordering bug** in the round-30
  walker, which read `quantiser_scale_code` immediately
  after `macroblock_type` and so misaligned the cursor on
  any P/B-picture MB whose `macroblock_modes()` tail
  consumed bits (every motion MB on a frame picture with
  `frame_pred_frame_dct == 0`, every motion MB on a field
  picture, and every coded-pattern MB on a frame picture
  with `frame_pred_frame_dct == 0`).
* `SliceWalkContext` gains two new fields —
  `picture_structure` (§6.3.11 Table 6-14) and
  `frame_pred_frame_dct` (§6.3.11) — that the §6.2.5.1
  gates need. The existing
  `SliceWalkContext::first_slice(mb_width, mb_row,
  picture_coding_type, initial_quantiser_scale_code)`
  shorthand keeps its 4-argument signature and defaults
  both new fields to a tail-gated-off shape
  (`PictureStructure::Frame` + `frame_pred_frame_dct =
  true`), preserving every round-30 test verbatim — that
  defaulting is safe for I-pictures (no motion possible)
  and for the `frame_pred_frame_dct == 1` P/B case. Full-
  fidelity callers use the new
  `first_slice_with_picture_extension` constructor, and
  MPEG-1 callers use `first_slice_mpeg1` which pins both
  fields so the §6.2.5.1 tail is always gated off (MPEG-1's
  macroblock layer carries its own §2.4.2.7 motion-vector
  syntax outside this driver).
* `MacroblockRecord` gains `motion_type: Option<MotionType>`
  (the parsed `frame_motion_type` / `field_motion_type` with
  the §6.3.17.2 `motion_vector_count` / `mv_format` / `dmv`
  derivation) and `dct_type: Option<bool>` alongside the
  existing fields. The `Option`-typed surface preserves the
  spec-level distinction between "field is absent" (where the
  Table 6-17 / Table 6-19 defaults apply, but at the motion-
  vector or block-organisation site rather than here) and
  "field is present with value X".
* **7 new lib unit tests** in `src/slice_macroblock_walk.rs`
  pin the four §6.2.5.1 gate cases: frame_motion_type read
  when the motion flag fires and `frame_pred_frame_dct ==
  0` (`10` Frame-based), frame_motion_type read with `11`
  Dual-Prime, field_motion_type read in a top-field picture
  (`10` 16×8 MC with mv_count = 2), motion_type omitted in
  a frame picture with `frame_pred_frame_dct == 1`,
  dct_type emitted on an intra MB in a frame picture with
  `frame_pred_frame_dct == 0`, dct_type omitted in a field
  picture, the full "type-tail-quant" 14-bit P-picture
  fixture (`00010 10 0 19_5b`), and the MPEG-1 shorthand
  asserting both Options are `None`.
* **3 new integration tests** in
  `tests/slice_macroblock_walk_synthetic.rs` exercise the
  same gates end-to-end through the public re-exports: a
  two-MB P-frame chain with `frame_motion_type` between
  each `macroblock_type` and the next increment (`10`
  Frame-based then `11` Dual-Prime), a single-MB top-field
  P-picture with "MC, Coded, Quant" type + `field_motion_type
  = 01` Field-based mv_count=1 + 5-bit quantiser_scale_code
  = 23, and a two-MB intra-frame I-picture with dct_type
  alternating between field-DCT and frame-DCT.

  The remaining gap on the MPEG-2 decode path is now the
  three post-`macroblock_modes()` fields: `motion_vectors(s)`
  (with its §7.6.3.4 PMV reset and `f_code[][]` matrix from
  `picture_coding_extension()`), the concealment-MV
  `marker_bit`, and `coded_block_pattern()` + the per-block
  walker (which need the §7.4.2.1 weighting matrices and the
  §6.3.17.4 `pattern_code[12]` derivation already in the
  `mpeg2_macroblock_blocks` module). The natural round-33
  work is the `motion_vectors(s)` wiring — its f_code matrix
  comes from the picture coding extension `SliceWalkContext`
  already implicitly references via `frame_pred_frame_dct`.

## Clean-room provenance

Every line in this crate's `src/` traces to:

* `docs/video/h262/is138182-1995.pdf` — ISO/IEC 13818-2:1995 base
  text (Recommendation ITU-T H.262 (1995 E)) §§4.1, 4.3, 5.2.3,
  6.1.1.8, 6.2.2.1, 6.2.2.3, 6.2.2.6, 6.2.3, 6.2.3.1, 6.2.4, 6.2.5,
  6.2.5.1, 6.2.5.2, 6.2.5.2.1, 6.2.5.3, 6.3.3, 6.3.4, 6.3.5, 6.3.8,
  6.3.10, 6.3.11, 6.3.16, 6.3.17.1, 6.3.17.2, 6.3.17.3, 6.3.17.4,
  7.6, 7.6.3, 7.6.3.1, 7.6.3.2, 7.6.3.3, 7.6.3.4, 7.6.3.5, 7.6.3.6,
  7.6.3.7, 7.6.4, 7.6.5, 7.6.6, 7.6.7.1, 7.6.7.2, 7.6.7.4, 7.6.8,
  Tables 6-1 / 6-2 / 6-3 / 6-4 / 6-5 / 6-10 / 6-11 / 6-12 / 6-13 /
  6-14 / 6-17 / 6-18 / 6-19 / 7-7 / 7-8 / 7-10 / 7-11 / 7-12 / 7-13 /
  7-14, and Annex B Tables B-1 / B-2 / B-3 / B-4 / B-9 / B-10 / B-11 /
  B-14 / B-15 / B-16. Round 25 also cites §7.2.2, §7.2.2.1 (Table 7-3),
  §7.2.2.2 (NOTE 2 / NOTE 3 FIRST / NEXT modification), §7.2.2.3
  (Table B-16 escape encoding distinct from MPEG-1's Table B.5f), and
  §7.2.2.4 (decoder pseudo-code). Round 26 adds §7.3 (inverse-scan
  loop body and the `alternate_scan` flag dispatch), §7.3.1
  (matrix-download fixed-flag invariant), and Figure 7-2 / Figure 7-3
  (`scan[0][v][u]` / `scan[1][v][u]`) per the page-80 printout.
  Round 27 adds §7.2.1 (DC coefficients in intra blocks: the
  `dct_dc_size` VLC dispatch, the `dc_dct_differential` →
  `dct_diff` `half_range` reconstruction, the per-component
  `dc_dct_pred[cc]` predictor add + Table 7-1 routing, the
  three-trigger reset contract, and the `QFS[0]` bitstream
  constraint), Table 7-2 (`intra_dc_precision` → reset value
  `{128, 256, 512, 1024}`), and Annex B Tables B-12 / B-13
  (`dct_dc_size_luminance` / `dct_dc_size_chrominance` extended to
  `0..=11`). Round 28 adds §6.2.6 (`block(i)` syntax — the
  `pattern_code[i]` gate, the `macroblock_intra` split between
  the §7.2.1 DC prelude and `dct_coeff_first`, the
  `while (nextbits() != End-of-block)` loop, and the
  table-dependent `end_of_block` terminator) — no new clauses
  beyond those already cited; the driver composes the existing
  §7.2.1 / §7.2.2 / §7.3 / §7.4 / §A endpoints behind one entry
  point. Round 29 adds §6.1.1.8 Figures 6-10 / 6-11 / 6-12 (the
  4:2:0 / 4:2:2 / 4:4:4 macroblock-block layout that maps
  `pattern_code[]` index → colour component) and §6.2.5
  (`macroblock()` syntax — the `for (i = 0; i < block_count; i++)
  block(i)` loop body the macroblock-block driver dispatches),
  and re-uses §7.4.2.1 Table 7-5 (already cited by round 23) for
  the per-block weighting-matrix dispatch. Round 31 adds the
  §7.6.6 sub-clauses (§7.6.6.1 P field picture, §7.6.6.2 P frame
  picture, §7.6.6.3 B field picture, §7.6.6.4 B frame picture,
  plus the §7.6.6 preamble's I-picture-no-skipped-MBs rule and
  scalability exemption), and re-cites §7.6.3.4 (PMV reset
  bullet "In a P-picture when a macroblock is skipped", already
  cited by round 23) and §7.6.3.6 (the field-parity numbering
  "top field has parity zero, the bottom field has parity one",
  already cited by round 22 for dual-prime) for the
  same-parity field-reference rule.
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
  fixed-length 12-bit scheme); and in round 18 for the §2.4.4.1
  page-32 intra-block dequantiser (the four-block-loop arithmetic
  body with the `dct_dc_y_past` / `dct_dc_cb_past` /
  `dct_dc_cr_past` predictor chain, the `past_intra_address > 1`
  reset branch, the `(128 * 8)` slice-start reset, the `Sign(...)`
  even-mismatch-prevention footnote, and the `[-2048, 2047]`
  saturation), the §2.4.4.2 page-35 non-intra dequantiser (the
  `(2*dct_zz[i] + Sign(dct_zz[i]))` dead-zone numerator and the
  `dct_zz[i] == 0 -> 0` zeroing pass), and the §2.4.3.2 page-25
  default `intra_quant[m][n]` (`intra_quant[0][0] = 8`) and
  default `non_intra_quant[m][n]` (uniform 16) matrices used when
  `load_intra_quantizer_matrix` / `load_non_intra_quantizer_matrix`
  is `0`. The MPEG-2 Table B-1 entries themselves trace to 13818-2.
* `oxideav-core`'s published `BitReader` MSB-first API.
* The `ffmpeg` CLI binary, used **only** as an opaque encoder for
  the integration-test fixture. Its source code was not consulted.

No external library source was read, quoted, or paraphrased.

## License

MIT — see [LICENSE](./LICENSE).
