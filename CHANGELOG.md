# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Clean-room rebuild round 13: §7.6.3.3 inter-vector PMV update
  (Tables 7-10 / 7-11), the once-per-macroblock pass that follows
  motion-vector reconstruction and propagates the `[r = 0]` PMV slot
  into the `[r = 1]` slot (or zeroes every slot) so prediction modes
  with fewer-than-maximum vectors still leave a sensible `PMV[1]`
  behind.
  - `pmv::update_predictors(&mut Pmv, PmvUpdateContext)` driving
    Tables 7-10 (frame pictures) and 7-11 (field pictures). The two
    tables share the right-hand "Predictors to Update" column
    row-for-row; the implementation branches on `picture_structure`
    + `prediction_type` to pick the right row family then applies
    the (fwd, bwd, intra) sub-cell.
  - Intra path covers both the `‡` row (`Frame-based`/`Field-based`
    assumed when `frame_motion_type` / `field_motion_type` is absent
    from the bitstream) and the `◊` footnote (when
    `concealment_motion_vectors == 0` the entire PMV is reset per
    §7.6.3.4 instead of copying `[0][0][1:0]` into `[1][0][1:0]`).
  - Non-intra Frame-based / Field-based / 16x8 MC / Dual-Prime row
    coverage including the `§` zero-motion footnote (PMV reset, only
    reachable in a P-picture) and the Dual-Prime forward-copy row.
  - Cells the spec leaves unreachable are rejected:
    `InvalidBitstream` for intra-with-motion-flag, Frame-based in a
    field picture, 16x8 MC in a frame picture, Dual-Prime with
    backward motion, Field-based / 16x8 with both motion flags zero,
    and non-intra without motion-type code.
  - `PmvUpdateOutcome` enum labels which row fired
    (`IntraConcealmentCopyForwardFirst`, `IntraResetAll`,
    `NonIntraCopyForward`, `NonIntraCopyBackward`,
    `NonIntraCopyBoth`, `NonIntraZeroMotionReset`, `NoUpdate`,
    `DualPrimeCopyForward`).
  - 18 new unit tests pinning every row of Tables 7-10 / 7-11 to a
    bit-exact PMV outcome, plus an end-to-end
    `reconstruct_motion_vector → update_predictors` chain
    confirming the reconstructed (3, -2) vector propagates from
    `PMV[0][0][:]` into `PMV[1][0][:]`.
  - Re-exports `update_predictors`, `PmvUpdateContext`,
    `PmvUpdateOutcome` at the crate root alongside the existing
    `Pmv` family.
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
- Clean-room rebuild round 8: parser for `coded_block_pattern()`
  (§6.2.5.3) with field semantics from §6.3.17.4 and the Annex B
  Table B-9 variable-length codes.
  - All 64 `coded_block_pattern_420` codewords (3- to 9-bit)
    walked MSB-first longest-first against the spec's tabulated
    bit-strings, decoding to the 6-bit `cbp` (0..=63).
  - 4:2:2 / 4:4:4 chroma extensions: 2-bit `coded_block_pattern_1`
    and 6-bit `coded_block_pattern_2` fixed-length codes read when
    the caller-supplied `chroma_format` selects them.
  - `CodedBlockPattern::pattern_code(macroblock_intra,
    macroblock_pattern)` derives the 12-entry `pattern_code[i]`
    array per §6.3.17.4 (intra-default all-ones, then `cbp` /
    `coded_block_pattern_1` / `coded_block_pattern_2` masking).
  - Typed `CodedBlockPattern { cbp, coded_block_pattern_1,
    coded_block_pattern_2, bit_position_after }` (re-exported at
    the crate root).
- 19 new unit tests covering every Table B-9 row (parsed
  individually), all-64-cbp coverage, longest-first prefix
  disambiguation, the 4:2:2 / 4:4:4 extensions, the §6.3.17.4
  `pattern_code` derivation, unknown-codeword / truncated-buffer
  rejection, and table invariants (prefix-free, widths fit,
  64 rows).
- 2 new black-box integration tests against the existing 352×240
  fixture: the first I-picture macroblock is plain `Intra`
  (`macroblock_pattern = 0`, so no `coded_block_pattern()` per
  §6.2.5.3), and the fixture's chroma is pinned to 4:2:0 before
  decoding a Table B-9 codeword against it.
- Clean-room rebuild round 9: parser for the macroblock-layer
  `quantizer_scale` per ISO/IEC 11172-2:1993 (MPEG-1 Video)
  §2.4.2.7 (syntax) with field semantics from §2.4.3.6. Fills the
  bitstream gap between round 7's `macroblock_type` and round 8's
  `coded_block_pattern()` — `quantizer_scale` is read immediately
  after `macroblock_type`, conditional on the `macroblock_quant`
  flag the type carries.
  - When `macroblock_quant` is set, a 5-bit `quantizer_scale` is
    read and validated against the §2.4.3.6 range `1..=31` (the
    value `0` is forbidden).
  - When `macroblock_quant` is clear the field is absent: zero bits
    are read and `quantizer_scale = None` is returned, matching the
    §2.4.3.6 persistence rule (the decoder keeps the value
    established at the slice layer or a prior macroblock).
  - `QuantizerScale::parse_after_type(br, &MacroblockType)`
    convenience threads the flag straight from a decoded
    `macroblock_type`.
  - Typed `QuantizerScale { quantizer_scale, bit_position_after }`
    (re-exported at the crate root) plus `QUANTIZER_SCALE_MIN` /
    `QUANTIZER_SCALE_MAX` constants.
- 12 new unit tests covering the present / absent branches, every
  legal value `1..=31`, forbidden-zero rejection, truncated- and
  empty-buffer handling on both branches, `parse_after_type`
  flag-threading for both flag states, the bound constants, and
  bit-position accounting.
- 2 new black-box integration tests against the existing 352×240
  fixture: the first I-picture macroblock is plain `Intra`
  (`macroblock_quant = 0`), so per §2.4.2.7 it carries no
  `quantizer_scale` and the parser consumes zero bits; a spliced
  `macroblock_quant`-set `macroblock_type` then decodes a synthetic
  5-bit `quantizer_scale` with correct value and bit accounting.
- Clean-room rebuild round 10: parser for the remainder of
  `macroblock_modes()` after `macroblock_type` per ISO/IEC 13818-2
  (ITU-T H.262) §6.2.5.1 with field semantics from §6.3.17.1 /
  §6.3.17.2 and the meaning Tables 6-17, 6-18 and 6-19. Closes
  `macroblock_modes()`; `motion_vectors()` and the block loop stay
  out of scope.
  - 2-bit `frame_motion_type` (Table 6-17) / `field_motion_type`
    (Table 6-18) decoding, surfacing the derived `prediction_type`,
    `motion_vector_count`, `mv_format`, and `dmv`; reserved code
    `00` rejected; the two `Field-based` rows of Table 6-17 split by
    a caller-supplied `spatial_temporal_weight_class`.
  - §6.2.5.1 presence gates honoured: the motion-type code is read
    only when a motion flag is set (omitted in frame pictures when
    `frame_pred_frame_dct == 1`), and `dct_type` only when
    `picture_structure == frame`, `frame_pred_frame_dct == 0`, and
    the macroblock is intra or has a coded pattern. Absent fields
    read zero bits.
  - `spatial_temporal_weight_code` is not read (scalable-only,
    `spatial_temporal_weight_code_flag` is always `0` for the
    non-scalable tables); a `mb_type` claiming the flag is rejected.
  - Typed `MacroblockModesTail { motion_type, dct_type,
    bit_position_after }`, `MotionType`, `PredictionType`,
    `MvFormat`, and `MacroblockModesContext` (re-exported at the
    crate root).
- 21 new unit tests covering every Table 6-17 / 6-18 row, the
  per-class `Field-based` vector-count split, reserved-code
  rejection, the motion-type / `dct_type` presence matrix, the
  scalable-flag rejection, zero-bit absent paths, and truncated-
  buffer handling.
- 2 new black-box integration tests against the existing 352×240
  fixture: a full slice → increment → type → quantizer_scale →
  `macroblock_modes()` tail chain on the first I-picture macroblock
  (motion-type absent; `dct_type` presence keyed to the fixture's
  own `frame_pred_frame_dct`), plus a spliced P-picture frame
  macroblock decoding `frame_motion_type` + `dct_type` with exact
  bit accounting.

- Clean-room rebuild round 11: parsers for the `motion_vectors(s)`
  wrapper per ISO/IEC 13818-2 (ITU-T H.262) §6.2.5.2 and the inner
  `motion_vector(r, s)` per §6.2.5.2.1, with field semantics from
  §6.3.17.2 / §6.3.17.3 and the Annex B Tables B-10 (`motion_code`)
  and B-11 (`dmvector`). The numerical reconstruction of
  `vector'[r][s][t]` (§7.6.3.1, PMV state machine, wrap-around) stays
  out of scope.
  - Table B-10 `motion_code`: all 33 codewords (`-16..=+16`) walked
    MSB-first longest-first, 1-/3-/4-/5-/7-/8-/10-/11-bit groups,
    prefix-free.
  - Table B-11 `dmvector[t]`: the 1-/2-bit `{0, +1, -1}` VLC.
  - Fixed-length `motion_residual[r][s][t]` read iff `f_code != 1 &&
    motion_code != 0`, width `r_size = f_code - 1` (1..=8 bits);
    `f_code` outside §6.3.11's `1..=9` range rejected when a residual
    would otherwise drive the cursor.
  - `motion_vertical_field_select[r][s]` flag honoured per §6.2.5.2 —
    suppressed when `motion_vector_count == 1 && (mv_format == frame
    || dmv == 1)`, present otherwise (both rows when count == 2).
  - Typed `MotionVector`, `MotionVectorEntry`, `MotionVectors`,
    `MotionVectorsContext`, `MotionVectorsKind` (re-exported at the
    crate root). `MotionVectors::parse(br, kind, &MotionType, &ctx)`
    threads a parsed `frame_motion_type` / `field_motion_type` (round
    10) and the `f_code[s][t]` matrix straight through.
- 29 new unit tests covering every Table B-10 row (parsed
  individually), the +16 / -16 extremes, the 33-unique-values
  invariant, prefix-freeness and width-fitting, unknown-prefix /
  truncated-buffer rejection on both VLC tables, Table B-11's three
  values plus its truncated-second-bit short case, the
  `motion_vector(r, s)` presence-matrix (no residual on f_code = 1 or
  motion_code = 0, residual width = `f_code - 1`, dmvector suppressed
  when `dmv = 0`), out-of-range `f_code` rejection, all four
  `motion_vectors(s)` shapes (frame count-1 / field count-1 /
  dual-prime count-1 / count-2), the Forward / Backward `f_code` pair
  selection, `motion_vector_count` validation, and truncated-VFS-/
  truncated-motion-code short paths.
- 2 new black-box integration tests against the existing 352×240
  fixture: the first I-picture is plain `Intra` so per §6.2.5.2 no
  `motion_vectors()` element exists (the fixture's f_codes are pinned
  to the §6.3.11 "unused" sentinel `15`), and a spliced P-picture
  frame macroblock prefix that drives the full `macroblock_type` →
  `frame_motion_type` → `dct_type` → `motion_vectors(0)` chain
  (`motion_code = -1`, `motion_residual = 1` with `f_code = 2`,
  `motion_code_vert = 0`) and asserts the 9-bit total cursor
  accounting.

- Clean-room rebuild round 12: motion-vector reconstruction per
  ISO/IEC 13818-2 (Recommendation ITU-T H.262) §7.6.3.1 plus
  §7.6.3.4 reset rules and §7.6.3.7 chroma scaling. The bridge from
  round 11's parsed `motion_code` / `motion_residual` / `dmvector`
  fields to the spec's `vector'[r][s][t]` reconstructed luminance
  motion vector.
  - `compute_delta(motion_code, motion_residual, f_code)` derives
    `delta` per the spec formula (`f = 1 << (f_code - 1)`, shortcut
    `delta = motion_code` when `f == 1 || motion_code == 0`,
    otherwise `sign(motion_code) * ((|motion_code| - 1) * f +
    motion_residual + 1)`).
  - `vector_range(f_code)` returns `(low, high, range) =
    (-16*f, 16*f - 1, 32*f)`. Doubles per Table 7-8 entry across
    `f_code ∈ {1..=9}`; out-of-range `f_code` rejected.
  - `reconstruct_component(motion_code, motion_residual, f_code,
    prior_pmv, mv_format, picture_structure, t)` runs the §7.6.3.1
    procedure for one component: half-pred for the
    `(mv_format == field && t == vertical && picture_structure ==
    frame)` case using §4.3 floor-division (`div_euclid`), `vector' =
    prediction + delta`, wrap into `[low, high]` via `± range`, and
    `new_pmv = vector' * 2` for the half-pred case else `vector'`.
    Bitstream conformance per §7.6.3.2 (`delta`, `vector'`, new PMV
    all in range) enforced as a parse-time invariant.
  - `reconstruct_motion_vector(pmv, &MotionVector, r, s, f_code_h,
    f_code_v, mv_format, picture_structure)` chains the two
    components and writes the new PMVs back into the supplied
    `Pmv` slot.
  - `Pmv { values: [[[i32; 2]; 2]; 2] }` carries the four
    `PMV[r][s][t]` predictors in half-sample units (Table 7-7) with
    `Pmv::get` / `set` typed by `VectorIndex` / `Direction` /
    `Component`. `Pmv::reset()` for §7.6.3.4 (slice start /
    non-concealment intra / P-picture non-intra without forward /
    P-skipped). `Pmv::default()` and `Pmv::new()` zero every slot.
  - `scale_chroma(luma_horiz, luma_vert, ChromaFormat) ->
    ScaledMotionVector` per §7.6.3.7: 4:2:0 halves both components,
    4:2:2 halves only horizontal, 4:4:4 is identity. Toward-zero
    integer division per §4.3.
  - Typed `Pmv`, `ReconstructedComponent { vector_prime, new_pmv,
    delta, range }`, `ScaledMotionVector { luma_horiz, luma_vert,
    chroma_horiz, chroma_vert }`, `Component { Horizontal,
    Vertical }`, `Direction { Forward, Backward }`,
    `VectorIndex { First, Second }` (re-exported at the crate root).
- 29 new unit tests covering: `compute_delta`'s shortcut and
  full-formula branches across `f_code ∈ {1..=9}` and motion-code
  signs, `motion_residual` presence-required / presence-forbidden
  rejection, out-of-range `f_code` rejection (0, 10, 15),
  `vector_range` invariants (f_code=1 ⇒ ±16, f_code=9 ⇒ ±4096,
  range doubles per step), end-to-end `reconstruct_motion_vector`
  with no-wrap / wrap-low / wrap-high paths, vertical-half-pred
  matrix (frame vs field picture, horizontal vs vertical component),
  floor-division for negative PMV under half-pred, PMV slot
  independence for `(r = 0 vs r = 1)` and `(forward vs backward)`,
  delta-outside-range rejection, `Pmv::default` / `Pmv::reset` zero
  every slot, chroma scaling for all three `ChromaFormat` values
  plus toward-zero rounding on negative odd inputs, and Table 7-7
  index enums.
- 2 new black-box integration tests against the existing 352×240
  fixture: confirms the fixture's I-picture is the §7.6.3 "PMV
  unused" case (every f_code is the `15` sentinel; PMV stays zero
  after the §7.6.3.4 reset), and a spliced two-macroblock
  P-picture chain (`motion_code = +2, residual = 0` then
  `motion_code = -1, residual = 0`, both with `f_code = 2`)
  decodes through `MotionVector → reconstruct_motion_vector`
  with PMV state evolving from `0 → 3 → 2` (second `delta = -1`
  added on top of the first vector's predictor `PMV = 3`), and the
  4:2:0 chroma scaling halves both components.

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
- §7.6.3.3 inter-vector PMV-copy table (Tables 7-9 / 7-10) — the
  macroblock-loop driver's responsibility once that lands.
- §7.6.3.6 dual-prime additional arithmetic (deriving the
  opposite-parity vector from the decoded forward vector).
- §7.6.3.9 concealment motion vectors.
- Residual block VLC tables (B-12 .. B-16) plus IDCT and motion
  compensation.
- The scalable `macroblock_type` Tables B-5 .. B-8 once
  `sequence_scalable_extension()` parsing lands.
- `oxideav_core::Decoder` wiring once a complete picture round-trips.
