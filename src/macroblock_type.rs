//! Parser for `macroblock_type` — the leading VLC of
//! `macroblock_modes()` per ISO/IEC 13818-2 (Recommendation ITU-T
//! H.262) §6.2.5.1, with the field semantics from §6.3.17.1 and the
//! Annex B Table B-2 / B-3 / B-4 codeword sets.
//!
//! Round 7 advances the macroblock loop one field past round 6's
//! `macroblock_address_increment`. Given a
//! [`oxideav_core::bits::BitReader`] sitting at the first bit of
//! `macroblock_modes()` (i.e. right after a parsed
//! [`crate::MbAddressIncrement`]), the parser walks the
//! `macroblock_type` VLC for the current `picture_coding_type` and
//! returns the six derived flags the spec lists in §6.3.17.1:
//!
//! * `macroblock_quant` — when set, a `quantiser_scale_code` follows
//!   in the bitstream (§6.2.5 macroblock()).
//! * `macroblock_motion_forward` / `macroblock_motion_backward` —
//!   control whether `motion_vectors(0)` / `motion_vectors(1)` are
//!   present and which prediction is formed.
//! * `macroblock_pattern` — when set, `coded_block_pattern()` follows.
//! * `macroblock_intra` — selects intra coding for the macroblock.
//! * `spatial_temporal_weight_code_flag` — `0` for the non-scalable
//!   B-2 / B-3 / B-4 tables; set per-row by the scalable Tables
//!   B-5 / B-6 / B-7. It indicates whether a `spatial_temporal_weight_code`
//!   follows in `macroblock_modes()` (§6.3.17.1).
//!
//! Round 7 covered the non-scalable tables B-2 (I-pictures), B-3
//! (P-pictures) and B-4 (B-pictures). Per Table 6-10 those are the
//! tables a decoder selects when no `sequence_scalable_extension()` is
//! present (or for data-partitioning / temporal scalability, or for a
//! spatial-scalable sequence whose current picture lacks a
//! `picture_spatial_scalable_extension()`). Round 294 adds the
//! **scalable Tables B-5 (I, spatial), B-6 (P, spatial), B-7
//! (B, spatial) and B-8 (I/P/B, SNR scalability)** now that the
//! `sequence_scalable_extension()` (r283) and
//! `picture_spatial_scalable_extension()` (r291) parsers make scalable
//! streams reachable. [`MacroblockType::parse`] keeps its non-scalable
//! Table 6-10 default; [`MacroblockType::parse_with_table`] takes an
//! explicit [`MacroblockTypeTable`] (which [`MacroblockTypeTable::select`]
//! derives from `scalable_mode` + picture type + the
//! spatial-scalable-extension-present flag per Table 6-10).
//!
//! The fields *after* `macroblock_type` inside `macroblock_modes()`
//! (`spatial_temporal_weight_code`, `frame_motion_type`,
//! `field_motion_type`, `dct_type`) are likewise deferred — they
//! depend on `picture_coding_extension()` state (`frame_pred_frame_dct`,
//! `picture_structure`) that is best threaded through in a later round.
//!
//! D-pictures (ISO/IEC 11172-2 `picture_coding_type == 4`) carry the
//! single-row Table B.2d of 11172-2: the one codeword `'1'` selecting
//! a plain intra macroblock (no quant, no motion, no pattern). The
//! non-scalable family serves it for
//! [`crate::PictureCodingType::DcIntra`]; the 13818-2 scalable
//! families reject the pairing (D-pictures do not occur in MPEG-2
//! streams, Table 6-12).
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)) §6.2.5.1, §6.3.17.1, Table
//! 6-10, and Annex B Tables B-2, B-3, B-4.

// Bit-group widths follow the spec's MSB-first visual layout of the
// Annex B tables (e.g. `0b0000_01` for the 6-bit P-picture
// "Intra, Quant" code) so an audit can read each constant against the
// printed table at a glance. clippy's `unusual_byte_groupings` lint
// prefers equal-size 4-bit groups, which would obscure the spec
// mapping.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitReader;

use crate::picture_header::PictureCodingType;
use crate::{Error, Result};

/// The flags derived from a `macroblock_type` VLC per §6.3.17.1.
///
/// For the non-scalable Tables B-2 / B-3 / B-4 the
/// `spatial_temporal_weight_code_flag` is always `false` and
/// `spatial_temporal_weight_class` is `Some(0)`. The scalable Tables
/// B-5 / B-6 / B-7 set both columns per-row (and may leave the class
/// unresolved — `None` — when `spatial_temporal_weight_code_flag` is
/// `true`, because the class is then derived from the
/// `spatial_temporal_weight_code` via Table 7-21). Table B-8 (SNR
/// scalability) always reports class `Some(0)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MacroblockType {
    /// `macroblock_quant`: a `quantiser_scale_code` follows in the
    /// bitstream (§6.2.5).
    pub macroblock_quant: bool,
    /// `macroblock_motion_forward`: forward motion vectors are present
    /// / forward prediction is formed.
    pub macroblock_motion_forward: bool,
    /// `macroblock_motion_backward`: backward motion vectors are
    /// present / backward prediction is formed.
    pub macroblock_motion_backward: bool,
    /// `macroblock_pattern`: a `coded_block_pattern()` follows in the
    /// bitstream (§6.2.5).
    pub macroblock_pattern: bool,
    /// `macroblock_intra`: the macroblock is intra-coded.
    pub macroblock_intra: bool,
    /// `spatial_temporal_weight_code_flag`: `false` for the
    /// non-scalable Tables B-2 / B-3 / B-4 and for the Table B-8
    /// SNR-scalability rows; set per-row by the spatial-scalable
    /// Tables B-5 / B-6 / B-7. When `true` a `spatial_temporal_weight_code`
    /// follows in `macroblock_modes()` (§6.3.17.1).
    pub spatial_temporal_weight_code_flag: bool,
    /// `spatial_temporal_weight_class` derived directly from the
    /// macroblock_type table (§6.3.17.1, "permitted
    /// spatial_temporal_weight_classes" column):
    ///
    /// * `Some(0)` — non-scalable, SNR-scalable, or a spatial-scalable
    ///   row with no compatible prediction (the macroblock is
    ///   temporal-only / intra).
    /// * `Some(4)` — a spatial-scalable "Compatible" / "Coded,
    ///   Compatible" row whose prediction is spatial-only.
    /// * `None` — a spatial-scalable row with
    ///   `spatial_temporal_weight_code_flag == true`; the class is one
    ///   of `{1, 2, 3}` and is resolved later from the
    ///   `spatial_temporal_weight_code` via Table 7-21.
    pub spatial_temporal_weight_class: Option<u8>,
    /// Bit position (relative to the start of the buffer the
    /// [`BitReader`] was created from) right after the consumed
    /// `macroblock_type` VLC. Lets callers chain into the next
    /// `macroblock_modes()` field without losing the partial-byte
    /// cursor.
    pub bit_position_after: u64,
}

/// One Annex B table row: a right-justified MSB-first VLC code, its
/// bit length, and the spec flag columns.
///
/// The two trailing columns (`stwcf` / `weight_class`) only vary on
/// the scalable Tables B-5 .. B-8. For the non-scalable Tables
/// B-2 / B-3 / B-4 they are constructed with [`Row::plain`], which
/// fixes `stwcf == false` and `weight_class == Some(0)` (the §6.3.17.1
/// non-scalable defaults). The longer scalable tables list the column
/// explicitly.
#[derive(Clone, Copy)]
struct Row {
    /// VLC code right-justified into a `u16` — e.g. the bit string
    /// `0000 01` becomes `0b00_0001` (decimal 1) with `bits == 6`.
    /// Table B-7 needs up to 9 bits, so a `u16` is used.
    code: u16,
    /// Length of `code` in bits (`1..=9` across B-2 .. B-8).
    bits: u8,
    /// `macroblock_quant` column.
    quant: bool,
    /// `macroblock_motion_forward` column.
    fwd: bool,
    /// `macroblock_motion_backward` column.
    bwd: bool,
    /// `macroblock_pattern` column.
    pattern: bool,
    /// `macroblock_intra` column.
    intra: bool,
    /// `spatial_temporal_weight_code_flag` column (`false` for
    /// B-2 / B-3 / B-4 / B-8; per-row for B-5 / B-6 / B-7).
    stwcf: bool,
    /// `spatial_temporal_weight_class` column. `Some(c)` when the
    /// table pins a single class (`0` or `4`); `None` when
    /// `stwcf == true` and the class is one of `{1, 2, 3}` to be
    /// resolved from `spatial_temporal_weight_code` (Table 7-21).
    weight_class: Option<u8>,
}

impl Row {
    /// Construct a non-scalable (B-2 / B-3 / B-4) row: the two
    /// spatial-temporal columns take their §6.3.17.1 non-scalable
    /// defaults (`stwcf == false`, `weight_class == Some(0)`).
    const fn plain(
        code: u16,
        bits: u8,
        quant: bool,
        fwd: bool,
        bwd: bool,
        pattern: bool,
        intra: bool,
    ) -> Self {
        Self {
            code,
            bits,
            quant,
            fwd,
            bwd,
            pattern,
            intra,
            stwcf: false,
            weight_class: Some(0),
        }
    }

    /// Construct a scalable (B-5 / B-6 / B-7 / B-8) row with the two
    /// extra columns spelled out.
    #[allow(clippy::too_many_arguments)]
    const fn scalable(
        code: u16,
        bits: u8,
        quant: bool,
        fwd: bool,
        bwd: bool,
        pattern: bool,
        intra: bool,
        stwcf: bool,
        weight_class: Option<u8>,
    ) -> Self {
        Self {
            code,
            bits,
            quant,
            fwd,
            bwd,
            pattern,
            intra,
            stwcf,
            weight_class,
        }
    }
}

/// ISO/IEC 11172-2 Table B.2d — `macroblock_type` in dc intra-coded
/// pictures (D-pictures).
///
/// ```text
/// VLC  quant fwd bwd pat intra  Description
/// 1     0     0   0   0   1     Intra
/// ```
const TABLE_B2D_D: &[Row] = &[
    //          code bits quant  fwd    bwd    pat    intra
    Row::plain(0b1, 1, false, false, false, false, true),
];

/// Table B-2 — `macroblock_type` in I-pictures.
///
/// ```text
/// VLC  quant fwd bwd pat intra  Description
/// 1     0     0   0   0   1     Intra
/// 01    1     0   0   0   1     Intra, Quant
/// ```
const TABLE_B2_I: &[Row] = &[
    //          code      bits quant fwd    bwd    pat    intra
    Row::plain(0b1, 1, false, false, false, false, true),
    Row::plain(0b01, 2, true, false, false, false, true),
];

/// Table B-3 — `macroblock_type` in P-pictures.
///
/// ```text
/// VLC      quant fwd bwd pat intra  Description
/// 1         0     1   0   1   0     MC, Coded
/// 01        0     0   0   1   0     No MC, Coded
/// 001       0     1   0   0   0     MC, Not Coded
/// 0001 1    0     0   0   0   1     Intra
/// 0001 0    1     1   0   1   0     MC, Coded, Quant
/// 0000 1    1     0   0   1   0     No MC, Coded, Quant
/// 0000 01   1     0   0   0   1     Intra, Quant
/// ```
const TABLE_B3_P: &[Row] = &[
    //          code        bits quant fwd    bwd    pat    intra
    Row::plain(0b1, 1, false, true, false, true, false), // MC, Coded
    Row::plain(0b01, 2, false, false, false, true, false), // No MC, Coded
    Row::plain(0b001, 3, false, true, false, false, false), // MC, Not Coded
    Row::plain(0b0001_1, 5, false, false, false, false, true), // Intra
    Row::plain(0b0001_0, 5, true, true, false, true, false), // MC, Coded, Quant
    Row::plain(0b0000_1, 5, true, false, false, true, false), // No MC, Coded, Quant
    Row::plain(0b0000_01, 6, true, false, false, false, true), // Intra, Quant
];

/// Table B-4 — `macroblock_type` in B-pictures.
///
/// ```text
/// VLC      quant fwd bwd pat intra  Description
/// 10        0     1   1   0   0     Interp, Not Coded
/// 11        0     1   1   1   0     Interp, Coded
/// 010       0     0   1   0   0     Bwd, Not Coded
/// 011       0     0   1   1   0     Bwd, Coded
/// 0010      0     1   0   0   0     Fwd, Not Coded
/// 0011      0     1   0   1   0     Fwd, Coded
/// 0001 1    0     0   0   0   1     Intra
/// 0001 0    1     1   1   1   0     Interp, Coded, Quant
/// 0000 11   1     1   0   1   0     Fwd, Coded, Quant
/// 0000 10   1     0   1   1   0     Bwd, Coded, Quant
/// 0000 01   1     0   0   0   1     Intra, Quant
/// ```
const TABLE_B4_B: &[Row] = &[
    //          code        bits quant fwd    bwd    pat    intra
    Row::plain(0b10, 2, false, true, true, false, false), // Interp, Not Coded
    Row::plain(0b11, 2, false, true, true, true, false),  // Interp, Coded
    Row::plain(0b010, 3, false, false, true, false, false), // Bwd, Not Coded
    Row::plain(0b011, 3, false, false, true, true, false), // Bwd, Coded
    Row::plain(0b0010, 4, false, true, false, false, false), // Fwd, Not Coded
    Row::plain(0b0011, 4, false, true, false, true, false), // Fwd, Coded
    Row::plain(0b0001_1, 5, false, false, false, false, true), // Intra
    Row::plain(0b0001_0, 5, true, true, true, true, false), // Interp, Coded, Quant
    Row::plain(0b0000_11, 6, true, true, false, true, false), // Fwd, Coded, Quant
    Row::plain(0b0000_10, 6, true, false, true, true, false), // Bwd, Coded, Quant
    Row::plain(0b0000_01, 6, true, false, false, false, true), // Intra, Quant
];

/// Table B-5 — `macroblock_type` in I-pictures with spatial
/// scalability.
///
/// Columns after `intra` are `spatial_temporal_weight_code_flag`
/// (`stwcf`) and the permitted `spatial_temporal_weight_class`.
///
/// ```text
/// VLC    quant fwd bwd pat intra stwcf  class  Description
/// 1       0     0   0   1   0     0     4      Coded, Compatible
/// 01      1     0   0   1   0     0     4      Coded, Compatible, Quant
/// 0011    0     0   0   0   1     0     0      Intra
/// 0010    1     0   0   0   1     0     0      Intra, Quant
/// 0001    0     0   0   0   0     0     4      Not Coded, Compatible
/// ```
const TABLE_B5_I_SPATIAL: &[Row] = &[
    //             code      bits quant fwd    bwd    pat    intra  stwcf  class
    Row::scalable(0b1, 1, false, false, false, true, false, false, Some(4)),
    Row::scalable(0b01, 2, true, false, false, true, false, false, Some(4)),
    Row::scalable(0b0011, 4, false, false, false, false, true, false, Some(0)),
    Row::scalable(0b0010, 4, true, false, false, false, true, false, Some(0)),
    Row::scalable(0b0001, 4, false, false, false, false, false, false, Some(4)),
];

/// Table B-6 — `macroblock_type` in P-pictures with spatial
/// scalability.
///
/// ```text
/// VLC       quant fwd bwd pat intra stwcf  class    Description
/// 10         0     1   0   1   0     0     0        MC, Coded
/// 011        0     1   0   1   0     1     1,2,3    MC, Coded, Compatible
/// 0000 100   0     0   0   1   0     0     0        No MC, Coded
/// 0001 11    0     0   0   1   0     1     1,2,3    No MC, Coded, Compatible
/// 0010       0     1   0   0   0     0     0        MC, Not Coded
/// 0000 111   0     0   0   0   1     0     0        Intra
/// 0011       0     1   0   0   0     1     1,2,3    MC, Not coded, Compatible
/// 010        1     1   0   1   0     0     0        MC, Coded, Quant
/// 0001 00    1     0   0   1   0     0     0        No MC, Coded, Quant
/// 0000 110   1     0   0   0   1     0     0        Intra, Quant
/// 11         1     1   0   1   0     1     1,2,3    MC, Coded, Compatible, Quant
/// 0001 01    1     0   0   1   0     1     1,2,3    No MC, Coded, Compatible, Quant
/// 0001 10    0     0   0   0   0     1     1,2,3    No MC, Not Coded, Compatible
/// 0000 101   0     0   0   1   0     0     4        Coded, Compatible
/// 0000 010   1     0   0   1   0     0     4        Coded, Compatible, Quant
/// 0000 011   0     0   0   0   0     0     4        Not Coded, Compatible
/// ```
const TABLE_B6_P_SPATIAL: &[Row] = &[
    //             code         bits quant fwd    bwd    pat    intra  stwcf  class
    Row::scalable(0b10, 2, false, true, false, true, false, false, Some(0)),
    Row::scalable(0b011, 3, false, true, false, true, false, true, None),
    Row::scalable(
        0b0000_100,
        7,
        false,
        false,
        false,
        true,
        false,
        false,
        Some(0),
    ),
    Row::scalable(0b0001_11, 6, false, false, false, true, false, true, None),
    Row::scalable(0b0010, 4, false, true, false, false, false, false, Some(0)),
    Row::scalable(
        0b0000_111,
        7,
        false,
        false,
        false,
        false,
        true,
        false,
        Some(0),
    ),
    Row::scalable(0b0011, 4, false, true, false, false, false, true, None),
    Row::scalable(0b010, 3, true, true, false, true, false, false, Some(0)),
    Row::scalable(
        0b0001_00,
        6,
        true,
        false,
        false,
        true,
        false,
        false,
        Some(0),
    ),
    Row::scalable(
        0b0000_110,
        7,
        true,
        false,
        false,
        false,
        true,
        false,
        Some(0),
    ),
    Row::scalable(0b11, 2, true, true, false, true, false, true, None),
    Row::scalable(0b0001_01, 6, true, false, false, true, false, true, None),
    Row::scalable(0b0001_10, 6, false, false, false, false, false, true, None),
    Row::scalable(
        0b0000_101,
        7,
        false,
        false,
        false,
        true,
        false,
        false,
        Some(4),
    ),
    Row::scalable(
        0b0000_010,
        7,
        true,
        false,
        false,
        true,
        false,
        false,
        Some(4),
    ),
    Row::scalable(
        0b0000_011,
        7,
        false,
        false,
        false,
        false,
        false,
        false,
        Some(4),
    ),
];

/// Table B-7 — `macroblock_type` in B-pictures with spatial
/// scalability.
///
/// ```text
/// VLC         quant fwd bwd pat intra stwcf class   Description
/// 10           0     1   1   0   0     0    0       Interp, Not coded
/// 11           0     1   1   1   0     0    0       Interp, Coded
/// 010          0     0   1   0   0     0    0       Back, Not coded
/// 011          0     0   1   1   0     0    0       Back, Coded
/// 0010         0     1   0   0   0     0    0       For, Not coded
/// 0011         0     1   0   1   0     0    0       For, Coded
/// 0001 10      0     0   1   0   0     1    1,2,3   Back, Not Coded, Compatible
/// 0001 11      0     0   1   1   0     1    1,2,3   Back, Coded, Compatible
/// 0001 00      0     1   0   0   0     1    1,2,3   For, Not Coded, Compatible
/// 0001 01      0     1   0   1   0     1    1,2,3   For, Coded, Compatible
/// 0000 110     0     0   0   0   1     0    0       Intra
/// 0000 111     1     1   1   1   0     0    0       Interp, Coded, Quant
/// 0000 100     1     1   0   1   0     0    0       For, Coded, Quant
/// 0000 101     1     0   1   1   0     0    0       Back, Coded, Quant
/// 0000 0100    1     0   0   0   1     0    0       Intra, Quant
/// 0000 0101    1     1   0   1   0     1    1,2,3   For, Coded, Compatible, Quant
/// 0000 0110 0  1     0   1   1   0     1    1,2,3   Back, Coded, Compatible, Quant
/// 0000 0111 0  0     0   0   0   0     0    4       Not Coded, Compatible
/// 0000 0110 1  1     0   0   1   0     0    4       Coded, Compatible, Quant
/// 0000 0111 1  0     0   0   1   0     0    4       Coded, Compatible
/// ```
const TABLE_B7_B_SPATIAL: &[Row] = &[
    //             code          bits quant fwd    bwd    pat    intra  stwcf  class
    Row::scalable(0b10, 2, false, true, true, false, false, false, Some(0)),
    Row::scalable(0b11, 2, false, true, true, true, false, false, Some(0)),
    Row::scalable(0b010, 3, false, false, true, false, false, false, Some(0)),
    Row::scalable(0b011, 3, false, false, true, true, false, false, Some(0)),
    Row::scalable(0b0010, 4, false, true, false, false, false, false, Some(0)),
    Row::scalable(0b0011, 4, false, true, false, true, false, false, Some(0)),
    Row::scalable(0b0001_10, 6, false, false, true, false, false, true, None),
    Row::scalable(0b0001_11, 6, false, false, true, true, false, true, None),
    Row::scalable(0b0001_00, 6, false, true, false, false, false, true, None),
    Row::scalable(0b0001_01, 6, false, true, false, true, false, true, None),
    Row::scalable(
        0b0000_110,
        7,
        false,
        false,
        false,
        false,
        true,
        false,
        Some(0),
    ),
    Row::scalable(0b0000_111, 7, true, true, true, true, false, false, Some(0)),
    Row::scalable(
        0b0000_100,
        7,
        true,
        true,
        false,
        true,
        false,
        false,
        Some(0),
    ),
    Row::scalable(
        0b0000_101,
        7,
        true,
        false,
        true,
        true,
        false,
        false,
        Some(0),
    ),
    Row::scalable(
        0b0000_0100,
        8,
        true,
        false,
        false,
        false,
        true,
        false,
        Some(0),
    ),
    Row::scalable(0b0000_0101, 8, true, true, false, true, false, true, None),
    Row::scalable(0b0000_0110_0, 9, true, false, true, true, false, true, None),
    Row::scalable(
        0b0000_0111_0,
        9,
        false,
        false,
        false,
        false,
        false,
        false,
        Some(4),
    ),
    Row::scalable(
        0b0000_0110_1,
        9,
        true,
        false,
        false,
        true,
        false,
        false,
        Some(4),
    ),
    Row::scalable(
        0b0000_0111_1,
        9,
        false,
        false,
        false,
        true,
        false,
        false,
        Some(4),
    ),
];

/// Table B-8 — `macroblock_type` in I-, P- and B-pictures with SNR
/// scalability. The same three codewords serve every picture type
/// (the NOTE in Annex B: "There is no differentiation between picture
/// types, since macroblocks are processed identically"). The
/// `spatial_temporal_weight_code_flag` is always `0`; the NOTE under
/// the table records that this table never sets it.
///
/// ```text
/// VLC   quant fwd bwd pat intra stwcf class  Description
/// 1      0     0   0   1   0     0     0      Coded
/// 01     1     0   0   1   0     0     0      Coded, Quant
/// 001    0     0   0   0   0     0     0      Not Coded
/// ```
const TABLE_B8_SNR: &[Row] = &[
    //          code   bits quant fwd    bwd    pat    intra
    Row::plain(0b1, 1, false, false, false, true, false), // Coded
    Row::plain(0b01, 2, true, false, false, true, false), // Coded, Quant
    Row::plain(0b001, 3, false, false, false, false, false), // Not Coded
];

/// Which Annex B `macroblock_type` table family applies, per Table
/// 6-10. The picture-type variants of each family are resolved inside
/// [`table_for`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MacroblockTypeTable {
    /// Tables B-2 / B-3 / B-4 — the non-scalable tables. Selected when
    /// no `sequence_scalable_extension()` is present, for
    /// `data partitioning` and `temporal scalability`, and for a
    /// spatial-scalable sequence whose current picture carries no
    /// `picture_spatial_scalable_extension()` (Table 6-10).
    NonScalable,
    /// Tables B-5 / B-6 / B-7 — spatial scalability, selected when
    /// `scalable_mode == spatial scalability` and the current picture
    /// carries a `picture_spatial_scalable_extension()` (Table 6-10).
    SpatialScalable,
    /// Table B-8 — SNR scalability (Table 6-10). One codeword set for
    /// every picture type.
    SnrScalable,
}

impl MacroblockTypeTable {
    /// Resolve the §6.2.5.1 / Table 6-10 table family from the three
    /// inputs that drive the selection:
    ///
    /// * `scalable_mode` — `None` when no `sequence_scalable_extension()`
    ///   is present; otherwise the 2-bit `scalable_mode` code (`00` =
    ///   data partitioning, `01` = spatial, `10` = SNR, `11` =
    ///   temporal) from §6.3.5 Table 6-10.
    /// * `picture_spatial_scalable_extension_present` — whether the
    ///   current picture carries a `picture_spatial_scalable_extension()`.
    ///   Only consulted for the spatial-scalability mode.
    ///
    /// Per Table 6-10 a spatial-scalable sequence whose current picture
    /// has no `picture_spatial_scalable_extension()` is decoded with
    /// the non-scalable tables ("that picture shall be decoded in a
    /// non-scalable manner").
    pub fn select(
        scalable_mode: Option<u8>,
        picture_spatial_scalable_extension_present: bool,
    ) -> Self {
        match scalable_mode {
            // No sequence_scalable_extension(): always non-scalable.
            None => Self::NonScalable,
            // 00 data partitioning, 11 temporal scalability → B-2/B-3/B-4.
            Some(0b00) | Some(0b11) => Self::NonScalable,
            // 01 spatial scalability: B-5/B-6/B-7 only when the picture
            // carries the spatial-scalable extension; otherwise
            // non-scalable.
            Some(0b01) => {
                if picture_spatial_scalable_extension_present {
                    Self::SpatialScalable
                } else {
                    Self::NonScalable
                }
            }
            // 10 SNR scalability → B-8.
            Some(0b10) => Self::SnrScalable,
            // scalable_mode is a 2-bit field; no other value is reachable.
            Some(_) => Self::NonScalable,
        }
    }
}

/// Select the `macroblock_type` row table for a `(table family,
/// picture coding type)` pair per Table 6-10 (plus the ISO/IEC
/// 11172-2 Table B.2d single-row set for dc intra-coded pictures).
fn table_for(
    table: MacroblockTypeTable,
    picture_coding_type: PictureCodingType,
) -> Result<&'static [Row]> {
    Ok(match table {
        MacroblockTypeTable::NonScalable => match picture_coding_type {
            PictureCodingType::Intra => TABLE_B2_I,
            PictureCodingType::Predictive => TABLE_B3_P,
            PictureCodingType::Bidirectional => TABLE_B4_B,
            PictureCodingType::DcIntra => TABLE_B2D_D,
        },
        MacroblockTypeTable::SpatialScalable => match picture_coding_type {
            PictureCodingType::Intra => TABLE_B5_I_SPATIAL,
            PictureCodingType::Predictive => TABLE_B6_P_SPATIAL,
            PictureCodingType::Bidirectional => TABLE_B7_B_SPATIAL,
            // D-pictures exist only in ISO/IEC 11172-2, which has no
            // scalability — no Table 6-10 row selects this pairing.
            PictureCodingType::DcIntra => {
                return Err(Error::InvalidBitstream(
                    "macroblock_type: D-pictures do not occur in scalable ISO/IEC 13818-2 streams (Table 6-12)",
                ))
            }
        },
        // Table B-8 is picture-type-independent.
        MacroblockTypeTable::SnrScalable => TABLE_B8_SNR,
    })
}

/// Widths to probe for a given table family, longest-first, so a
/// shorter codeword can never be matched on the high bits of a longer
/// one. The non-scalable / SNR tables top out at 6 bits; the spatial
/// tables (B-7) reach 9 bits.
fn widths_for(table: MacroblockTypeTable) -> &'static [u8] {
    match table {
        MacroblockTypeTable::NonScalable | MacroblockTypeTable::SnrScalable => &[6, 5, 4, 3, 2, 1],
        MacroblockTypeTable::SpatialScalable => &[9, 8, 7, 6, 5, 4, 3, 2, 1],
    }
}

/// Walk the selected table longest-first so a shorter codeword can
/// never be matched on the high bits of a longer one.
fn match_row(br: &mut BitReader<'_>, table: &'static [Row], widths: &[u8]) -> Result<Row> {
    for &width in widths {
        if br.bits_remaining() < u64::from(width) {
            continue;
        }
        let peeked = br
            .peek_u32(u32::from(width))
            .map_err(|_| Error::ShortHeader)? as u16;
        for &row in table.iter().filter(|r| r.bits == width) {
            if row.code == peeked {
                br.consume(u32::from(width))
                    .map_err(|_| Error::ShortHeader)?;
                return Ok(row);
            }
        }
    }
    Err(Error::InvalidBitstream(
        "macroblock_type: no Annex B (B-2 .. B-8) codeword matches the bit prefix (§6.2.5.1)",
    ))
}

impl MacroblockType {
    /// Parse one `macroblock_type` VLC starting at the current
    /// position of `br`, selecting the table from
    /// `picture_coding_type` per Table 6-10 (non-scalable streams).
    /// Consumes from `br` on success.
    ///
    /// Errors:
    /// * [`Error::InvalidBitstream`] if no codeword in the selected
    ///   table matches the upcoming bits.
    /// * [`Error::ShortHeader`] if the bitstream ends before a full
    ///   codeword could be read.
    pub fn parse(br: &mut BitReader<'_>, picture_coding_type: PictureCodingType) -> Result<Self> {
        Self::parse_with_table(br, picture_coding_type, MacroblockTypeTable::NonScalable)
    }

    /// Parse one `macroblock_type` VLC starting at the current position
    /// of `br`, using an explicit [`MacroblockTypeTable`] family
    /// (Table 6-10). Use [`MacroblockTypeTable::select`] to derive the
    /// family from `scalable_mode` and the
    /// `picture_spatial_scalable_extension()`-present flag, then pass it
    /// here. [`MacroblockType::parse`] is the
    /// [`MacroblockTypeTable::NonScalable`] shorthand.
    ///
    /// Consumes from `br` on success.
    ///
    /// Errors:
    /// * [`Error::InvalidBitstream`] if no codeword in the selected
    ///   table matches the upcoming bits.
    /// * [`Error::ShortHeader`] if the bitstream ends before a full
    ///   codeword could be read.
    pub fn parse_with_table(
        br: &mut BitReader<'_>,
        picture_coding_type: PictureCodingType,
        table: MacroblockTypeTable,
    ) -> Result<Self> {
        let rows = table_for(table, picture_coding_type)?;
        let row = match_row(br, rows, widths_for(table))?;
        Ok(Self {
            macroblock_quant: row.quant,
            macroblock_motion_forward: row.fwd,
            macroblock_motion_backward: row.bwd,
            macroblock_pattern: row.pattern,
            macroblock_intra: row.intra,
            spatial_temporal_weight_code_flag: row.stwcf,
            spatial_temporal_weight_class: row.weight_class,
            bit_position_after: br.bit_position(),
        })
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact round-trips covering every row of Tables
    //! B-2, B-3 and B-4 plus the rejection / truncation paths.
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// One scalable-table expectation row used by the B-5 / B-7
    /// per-row assertions: `(code, bits, quant, fwd, bwd, pattern,
    /// intra, stwcf, spatial_temporal_weight_class)`.
    type ScalableCase = (u32, u32, bool, bool, bool, bool, bool, bool, Option<u8>);

    /// Emit a code into a fresh buffer, padded with a trailing `'1'`
    /// (so the reader has a valid trailing byte the parser will never
    /// confuse with the codeword under test — every table here is
    /// prefix-free).
    fn buf_for(code: u32, bits: u32) -> Vec<u8> {
        let mut bw = BitWriter::new();
        bw.write_u32(code, bits);
        // Pad to a byte boundary with 1-bits; the parser only reads
        // `bits` of them.
        bw.write_bit(true);
        bw.align_to_byte();
        bw.finish()
    }

    fn parse(code: u32, bits: u32, pct: PictureCodingType) -> MacroblockType {
        let buf = buf_for(code, bits);
        let mut br = BitReader::new(&buf);
        MacroblockType::parse(&mut br, pct).expect("codeword should parse")
    }

    #[test]
    fn i_picture_intra() {
        // Table B-2 row '1': Intra (intra only).
        let mt = parse(0b1, 1, PictureCodingType::Intra);
        assert!(mt.macroblock_intra);
        assert!(!mt.macroblock_quant);
        assert!(!mt.macroblock_motion_forward);
        assert!(!mt.macroblock_motion_backward);
        assert!(!mt.macroblock_pattern);
        assert!(!mt.spatial_temporal_weight_code_flag);
        assert_eq!(mt.bit_position_after, 1);
    }

    #[test]
    fn i_picture_intra_quant() {
        // Table B-2 row '01': Intra, Quant.
        let mt = parse(0b01, 2, PictureCodingType::Intra);
        assert!(mt.macroblock_intra);
        assert!(mt.macroblock_quant);
        assert!(!mt.macroblock_motion_forward);
        assert!(!mt.macroblock_motion_backward);
        assert!(!mt.macroblock_pattern);
        assert_eq!(mt.bit_position_after, 2);
    }

    #[test]
    fn p_picture_all_rows() {
        // (code, bits, quant, fwd, bwd, pattern, intra)
        let cases: &[(u32, u32, bool, bool, bool, bool, bool)] = &[
            (0b1, 1, false, true, false, true, false),    // MC, Coded
            (0b01, 2, false, false, false, true, false),  // No MC, Coded
            (0b001, 3, false, true, false, false, false), // MC, Not Coded
            (0b0001_1, 5, false, false, false, false, true), // Intra
            (0b0001_0, 5, true, true, false, true, false), // MC, Coded, Quant
            (0b0000_1, 5, true, false, false, true, false), // No MC, Coded, Quant
            (0b0000_01, 6, true, false, false, false, true), // Intra, Quant
        ];
        for &(code, bits, quant, fwd, bwd, pattern, intra) in cases {
            let mt = parse(code, bits, PictureCodingType::Predictive);
            assert_eq!(mt.macroblock_quant, quant, "quant for code {code:b}");
            assert_eq!(mt.macroblock_motion_forward, fwd, "fwd for code {code:b}");
            assert_eq!(mt.macroblock_motion_backward, bwd, "bwd for code {code:b}");
            assert_eq!(mt.macroblock_pattern, pattern, "pattern for code {code:b}");
            assert_eq!(mt.macroblock_intra, intra, "intra for code {code:b}");
            assert!(!mt.spatial_temporal_weight_code_flag);
            assert_eq!(mt.bit_position_after, u64::from(bits));
        }
    }

    #[test]
    fn b_picture_all_rows() {
        let cases: &[(u32, u32, bool, bool, bool, bool, bool)] = &[
            (0b10, 2, false, true, true, false, false), // Interp, Not Coded
            (0b11, 2, false, true, true, true, false),  // Interp, Coded
            (0b010, 3, false, false, true, false, false), // Bwd, Not Coded
            (0b011, 3, false, false, true, true, false), // Bwd, Coded
            (0b0010, 4, false, true, false, false, false), // Fwd, Not Coded
            (0b0011, 4, false, true, false, true, false), // Fwd, Coded
            (0b0001_1, 5, false, false, false, false, true), // Intra
            (0b0001_0, 5, true, true, true, true, false), // Interp, Coded, Quant
            (0b0000_11, 6, true, true, false, true, false), // Fwd, Coded, Quant
            (0b0000_10, 6, true, false, true, true, false), // Bwd, Coded, Quant
            (0b0000_01, 6, true, false, false, false, true), // Intra, Quant
        ];
        for &(code, bits, quant, fwd, bwd, pattern, intra) in cases {
            let mt = parse(code, bits, PictureCodingType::Bidirectional);
            assert_eq!(mt.macroblock_quant, quant, "quant for code {code:b}");
            assert_eq!(mt.macroblock_motion_forward, fwd, "fwd for code {code:b}");
            assert_eq!(mt.macroblock_motion_backward, bwd, "bwd for code {code:b}");
            assert_eq!(mt.macroblock_pattern, pattern, "pattern for code {code:b}");
            assert_eq!(mt.macroblock_intra, intra, "intra for code {code:b}");
            assert!(!mt.spatial_temporal_weight_code_flag);
            assert_eq!(mt.bit_position_after, u64::from(bits));
        }
    }

    #[test]
    fn longest_first_match_does_not_misread_prefixes() {
        // In a P-picture, '1' is a 1-bit code (MC, Coded). The 5-bit
        // '0001 1' (Intra) and the 6-bit '0000 01' (Intra, Quant)
        // both *start* with prefixes of shorter codes. Decoding each
        // full codeword must not be hijacked by a shorter prefix.
        let intra = parse(0b0001_1, 5, PictureCodingType::Predictive);
        assert!(intra.macroblock_intra);
        assert_eq!(intra.bit_position_after, 5);

        let intra_q = parse(0b0000_01, 6, PictureCodingType::Predictive);
        assert!(intra_q.macroblock_intra && intra_q.macroblock_quant);
        assert_eq!(intra_q.bit_position_after, 6);
    }

    #[test]
    fn rejects_unknown_codeword() {
        // In an I-picture, the only valid prefixes are '1' and '01'.
        // A leading '00...' matches neither.
        let buf = [0u8; 2];
        let mut br = BitReader::new(&buf);
        let err = MacroblockType::parse(&mut br, PictureCodingType::Intra).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_truncated_buffer() {
        // An empty buffer offers no bits; the parser cannot read even
        // the shortest 1-bit I-picture codeword.
        let buf: [u8; 0] = [];
        let mut br = BitReader::new(&buf);
        let err = MacroblockType::parse(&mut br, PictureCodingType::Intra).unwrap_err();
        assert!(matches!(
            err,
            Error::ShortHeader | Error::InvalidBitstream(_)
        ));
    }

    #[test]
    fn rejects_all_zero_prefix_in_p_picture() {
        // '000000' (6 bits) matches no Table B-3 codeword — every
        // valid 5- and 6-bit P-table code begins '0001' or '0000 1'.
        let buf = [0u8; 2];
        let mut br = BitReader::new(&buf);
        let err = MacroblockType::parse(&mut br, PictureCodingType::Predictive).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn table_b2_is_prefix_free() {
        assert_prefix_free(TABLE_B2_I);
    }

    #[test]
    fn table_b3_is_prefix_free() {
        assert_prefix_free(TABLE_B3_P);
    }

    #[test]
    fn table_b4_is_prefix_free() {
        assert_prefix_free(TABLE_B4_B);
    }

    /// Every code must (a) fit in its declared bit width and (b) be
    /// prefix-free: no codeword is a bit-prefix of another. A
    /// non-prefix-free table would make `match_row`'s longest-first
    /// walk ambiguous.
    fn assert_prefix_free(table: &[Row]) {
        for r in table {
            let max = 1u32 << u32::from(r.bits);
            assert!(
                u32::from(r.code) < max,
                "code {:b} does not fit in {} bits",
                r.code,
                r.bits
            );
        }
        for (i, a) in table.iter().enumerate() {
            for b in &table[i + 1..] {
                // Compare the shorter against the high bits of the
                // longer.
                let (short, long) = if a.bits <= b.bits { (a, b) } else { (b, a) };
                let shift = long.bits - short.bits;
                let long_prefix = u32::from(long.code) >> u32::from(shift);
                assert_ne!(
                    long_prefix,
                    u32::from(short.code),
                    "code {:b} ({}b) is a prefix of {:b} ({}b)",
                    short.code,
                    short.bits,
                    long.code,
                    long.bits
                );
            }
        }
    }

    #[test]
    fn table_b5_is_prefix_free() {
        assert_prefix_free(TABLE_B5_I_SPATIAL);
    }

    #[test]
    fn table_b6_is_prefix_free() {
        assert_prefix_free(TABLE_B6_P_SPATIAL);
    }

    #[test]
    fn table_b7_is_prefix_free() {
        assert_prefix_free(TABLE_B7_B_SPATIAL);
    }

    #[test]
    fn table_b8_is_prefix_free() {
        assert_prefix_free(TABLE_B8_SNR);
    }

    #[test]
    fn table_sizes_match_spec() {
        // Annex B row counts.
        assert_eq!(TABLE_B2_I.len(), 2);
        assert_eq!(TABLE_B3_P.len(), 7);
        assert_eq!(TABLE_B4_B.len(), 11);
        assert_eq!(TABLE_B5_I_SPATIAL.len(), 5);
        assert_eq!(TABLE_B6_P_SPATIAL.len(), 16);
        assert_eq!(TABLE_B7_B_SPATIAL.len(), 20);
        assert_eq!(TABLE_B8_SNR.len(), 3);
    }

    /// Parse a codeword against an explicit table family.
    fn parse_table(
        code: u32,
        bits: u32,
        pct: PictureCodingType,
        table: MacroblockTypeTable,
    ) -> MacroblockType {
        let buf = buf_for(code, bits);
        let mut br = BitReader::new(&buf);
        MacroblockType::parse_with_table(&mut br, pct, table)
            .expect("scalable codeword should parse")
    }

    #[test]
    fn b5_spatial_all_rows() {
        // (code, bits, quant, fwd, bwd, pat, intra, stwcf, class)
        let cases: &[ScalableCase] = &[
            (0b1, 1, false, false, false, true, false, false, Some(4)), // Coded, Compatible
            (0b01, 2, true, false, false, true, false, false, Some(4)), // Coded, Compatible, Quant
            (0b0011, 4, false, false, false, false, true, false, Some(0)), // Intra
            (0b0010, 4, true, false, false, false, true, false, Some(0)), // Intra, Quant
            (0b0001, 4, false, false, false, false, false, false, Some(4)), // Not Coded, Compatible
        ];
        for &(code, bits, quant, fwd, bwd, pat, intra, stwcf, class) in cases {
            let mt = parse_table(
                code,
                bits,
                PictureCodingType::Intra,
                MacroblockTypeTable::SpatialScalable,
            );
            assert_eq!(mt.macroblock_quant, quant, "quant code {code:b}");
            assert_eq!(mt.macroblock_motion_forward, fwd, "fwd code {code:b}");
            assert_eq!(mt.macroblock_motion_backward, bwd, "bwd code {code:b}");
            assert_eq!(mt.macroblock_pattern, pat, "pattern code {code:b}");
            assert_eq!(mt.macroblock_intra, intra, "intra code {code:b}");
            assert_eq!(
                mt.spatial_temporal_weight_code_flag, stwcf,
                "stwcf code {code:b}"
            );
            assert_eq!(
                mt.spatial_temporal_weight_class, class,
                "class code {code:b}"
            );
            assert_eq!(mt.bit_position_after, u64::from(bits));
        }
    }

    #[test]
    fn b6_spatial_compatible_rows_set_flag_and_unresolved_class() {
        // The "Compatible" rows of Table B-6 set
        // spatial_temporal_weight_code_flag and leave the class
        // unresolved (one of {1,2,3} from Table 7-21).
        let mt = parse_table(
            0b011,
            3,
            PictureCodingType::Predictive,
            MacroblockTypeTable::SpatialScalable,
        );
        // 011 → MC, Coded, Compatible.
        assert!(mt.macroblock_motion_forward);
        assert!(mt.macroblock_pattern);
        assert!(mt.spatial_temporal_weight_code_flag);
        assert_eq!(mt.spatial_temporal_weight_class, None);

        // 0000 101 → Coded, Compatible (class 4, flag clear).
        let mt = parse_table(
            0b0000_101,
            7,
            PictureCodingType::Predictive,
            MacroblockTypeTable::SpatialScalable,
        );
        assert!(!mt.spatial_temporal_weight_code_flag);
        assert_eq!(mt.spatial_temporal_weight_class, Some(4));
        assert!(mt.macroblock_pattern);
    }

    #[test]
    fn b7_spatial_nine_bit_rows_parse() {
        // The longest Table B-7 codewords are 9 bits; verify the
        // longest-first walk resolves them without being hijacked by a
        // shorter prefix.
        let mt = parse_table(
            0b0000_0110_0,
            9,
            PictureCodingType::Bidirectional,
            MacroblockTypeTable::SpatialScalable,
        );
        // Back, Coded, Compatible, Quant.
        assert!(mt.macroblock_quant);
        assert!(mt.macroblock_motion_backward);
        assert!(mt.macroblock_pattern);
        assert!(mt.spatial_temporal_weight_code_flag);
        assert_eq!(mt.spatial_temporal_weight_class, None);
        assert_eq!(mt.bit_position_after, 9);

        let mt = parse_table(
            0b0000_0111_0,
            9,
            PictureCodingType::Bidirectional,
            MacroblockTypeTable::SpatialScalable,
        );
        // Not Coded, Compatible (class 4).
        assert!(!mt.macroblock_pattern);
        assert!(!mt.spatial_temporal_weight_code_flag);
        assert_eq!(mt.spatial_temporal_weight_class, Some(4));
    }

    #[test]
    fn b8_snr_all_rows() {
        // Table B-8 is picture-type-independent; the same three
        // codewords resolve for every picture type and never set the
        // weight-code flag.
        let cases: &[(u32, u32, bool, bool)] = &[
            (0b1, 1, false, true),    // Coded
            (0b01, 2, true, true),    // Coded, Quant
            (0b001, 3, false, false), // Not Coded
        ];
        for pct in [
            PictureCodingType::Intra,
            PictureCodingType::Predictive,
            PictureCodingType::Bidirectional,
        ] {
            for &(code, bits, quant, pattern) in cases {
                let mt = parse_table(code, bits, pct, MacroblockTypeTable::SnrScalable);
                assert_eq!(mt.macroblock_quant, quant, "quant code {code:b}");
                assert_eq!(mt.macroblock_pattern, pattern, "pattern code {code:b}");
                assert!(!mt.macroblock_intra);
                assert!(!mt.macroblock_motion_forward);
                assert!(!mt.macroblock_motion_backward);
                assert!(!mt.spatial_temporal_weight_code_flag);
                assert_eq!(mt.spatial_temporal_weight_class, Some(0));
            }
        }
    }

    #[test]
    fn table_select_follows_table_6_10() {
        use MacroblockTypeTable::*;
        // No sequence_scalable_extension() → non-scalable.
        assert_eq!(MacroblockTypeTable::select(None, false), NonScalable);
        assert_eq!(MacroblockTypeTable::select(None, true), NonScalable);
        // 00 data partitioning, 11 temporal → non-scalable.
        assert_eq!(MacroblockTypeTable::select(Some(0b00), true), NonScalable);
        assert_eq!(MacroblockTypeTable::select(Some(0b11), true), NonScalable);
        // 01 spatial: B-5/B-6/B-7 only when the picture carries the
        // spatial-scalable extension.
        assert_eq!(
            MacroblockTypeTable::select(Some(0b01), true),
            SpatialScalable
        );
        assert_eq!(MacroblockTypeTable::select(Some(0b01), false), NonScalable);
        // 10 SNR → B-8.
        assert_eq!(MacroblockTypeTable::select(Some(0b10), false), SnrScalable);
        assert_eq!(MacroblockTypeTable::select(Some(0b10), true), SnrScalable);
    }

    #[test]
    fn spatial_table_rejects_unknown_codeword() {
        // '0000 0000 0' (9 bits) matches no Table B-7 codeword.
        let buf = [0u8; 2];
        let mut br = BitReader::new(&buf);
        let err = MacroblockType::parse_with_table(
            &mut br,
            PictureCodingType::Bidirectional,
            MacroblockTypeTable::SpatialScalable,
        )
        .unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn non_scalable_parse_matches_default() {
        // parse() and parse_with_table(.., NonScalable) must agree.
        let buf = buf_for(0b0001_1, 5);
        let mut a = BitReader::new(&buf);
        let mut b = BitReader::new(&buf);
        let via_default = MacroblockType::parse(&mut a, PictureCodingType::Predictive).unwrap();
        let via_table = MacroblockType::parse_with_table(
            &mut b,
            PictureCodingType::Predictive,
            MacroblockTypeTable::NonScalable,
        )
        .unwrap();
        assert_eq!(via_default, via_table);
        assert_eq!(via_default.spatial_temporal_weight_class, Some(0));
    }

    #[test]
    fn debug_impl_smoke() {
        let mt = MacroblockType {
            macroblock_quant: true,
            macroblock_motion_forward: false,
            macroblock_motion_backward: false,
            macroblock_pattern: false,
            macroblock_intra: true,
            spatial_temporal_weight_code_flag: false,
            spatial_temporal_weight_class: Some(0),
            bit_position_after: 2,
        };
        let s = format!("{mt:?}");
        assert!(s.contains("MacroblockType"));
        assert!(s.contains("macroblock_intra"));
    }
}
