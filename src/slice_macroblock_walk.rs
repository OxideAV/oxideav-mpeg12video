//! MPEG-2 §6.2.4 slice-level macroblock-header walker per
//! **ISO/IEC 13818-2 (ITU-T H.262)**.
//!
//! The §6.2.4 slice body is a `do { macroblock() } while
//! ( nextbits() != '0000 0000 0000 0000 0000 0000' )` loop that the
//! crate already parses *piecewise* through the header bits
//! ([`crate::SliceHeader`]) and the per-macroblock parsers
//! ([`crate::MbAddressIncrement`], [`crate::MacroblockType`],
//! [`crate::QuantizerScale`], [`crate::MacroblockModesTail`], …).
//! This module composes those parsers into the §6.2.4 loop itself:
//! the per-slice driver that picks up at
//! [`crate::SliceHeader::body_bit_position`] and walks macroblock
//! after macroblock until the §5.2.3 / §6.2.4 stop condition is met.
//!
//! ## What this driver delivers
//!
//! The slice driver folds the §6.2.5 / §6.2.5.1 macroblock-header
//! **and `macroblock_modes()` tail** into a single per-slice walk
//! over the parsers already in this crate:
//!
//! 1. **§6.2.5** `macroblock_address_increment` (Table B-1 VLC with
//!    `macroblock_escape` chains).
//! 2. **§6.2.5.1** `macroblock_modes()`:
//!     * opener — `macroblock_type` VLC (Tables B-2 / B-3 / B-4)
//!       decoded against the picture coding type;
//!     * tail — `frame_motion_type` (Table 6-17) in frame pictures
//!       with `frame_pred_frame_dct == 0`, `field_motion_type`
//!       (Table 6-18) in field pictures, both gated on
//!       `macroblock_motion_forward || macroblock_motion_backward`;
//!     * tail — `dct_type` in frame pictures with
//!       `frame_pred_frame_dct == 0` whose MB is intra or has a
//!       coded pattern.
//! 3. **§6.2.5** macroblock-level `quantiser_scale_code` (5 bits, in
//!    `1..=31`) when `macroblock_quant == 1`, read *after* the
//!    `macroblock_modes()` block per the §6.2.5 syntax tree.
//!
//! The remainder of `macroblock()` — `motion_vectors(s)`, the
//! `marker_bit` when intra concealment vectors are present,
//! `coded_block_pattern()`, and the per-block walker — is still
//! out of scope at the slice-walker layer: those parsers need the
//! per-picture PMV state, the `f_code[][]` matrix from
//! `picture_coding_extension()`, and the §7.6.3.4 reset semantics
//! that bind to the picture-level driver above this one. The
//! [`MacroblockRecord::body_bit_position`] cursor is the entry
//! point those rounds will resume from.
//!
//! ## What §6.2.4 / §6.3.17.1 say
//!
//! Page 51 of ISO/IEC 13818-2:1995 gives the slice body:
//!
//! ```text
//! slice() {
//!     slice_start_code
//!     ... slice-header bits ...
//!     do {
//!         macroblock()
//!     } while ( nextbits() != '0000 0000 0000 0000 0000 0000' )
//!     next_start_code()
//! }
//! ```
//!
//! Per §5.2.3 / §6.2.4 the stop condition is "the next 23 bits, if
//! they were read, would all be zero" — i.e. the byte-aligned
//! `next_start_code()` prefix `0x000001` is one bit-shift away. In
//! practice every legal MPEG-2 slice ends on a byte boundary with
//! the bytes `0x00 0x00 0x01 <next start code>` immediately
//! following the last bit of the last macroblock, with optional
//! zero-byte stuffing in between (§5.2.3).
//!
//! §6.3.17.1 gives the per-slice state the driver maintains:
//!
//! * `previous_macroblock_address` is `mb_row * mb_width - 1` at the
//!   start of the slice (the macroblock immediately before the first
//!   macroblock of the slice). In this driver the caller passes
//!   `mb_row * mb_width - 1` in through [`SliceWalkContext`].
//! * `macroblock_address = previous_macroblock_address +
//!   macroblock_address_increment` per macroblock.
//! * Any macroblocks at addresses `previous_macroblock_address + 1
//!   .. macroblock_address - 1` are **skipped** macroblocks (§6.3.17.4
//!   §7.6.6). The §7.6.6 skipped-MB reconstruction is not run here;
//!   the driver merely records the skipped-MB index ranges so the
//!   higher-layer (§7.6.6) round can dispatch.
//! * `past_intra_address` is `-2` at the start of the picture and set
//!   to `macroblock_address` after every intra macroblock. The driver
//!   tracks it across macroblocks within a single slice; carrying it
//!   *across* slices is the picture-level driver's job.
//! * `quantiser_scale_code` is set from `slice_header()` initially
//!   and overwritten in any macroblock that has `macroblock_quant == 1`.
//!
//! Macroblocks at the **start of a slice** shall have
//! `macroblock_address_increment == 1` (§6.3.17.1). The driver
//! enforces this on the first macroblock and rejects any other
//! value.
//!
//! ## Why the remaining body fields are deferred
//!
//! Each remaining post-`macroblock_modes()` field comes with
//! cross-macroblock state that this driver alone cannot satisfy:
//!
//! * `motion_vectors(s)` needs the per-slice §7.6.3.4 PMV reset on
//!   intra macroblocks, the `f_code` array from
//!   `picture_coding_extension()`, and the §7.6.3.1 reconstruction
//!   call to actually use the parsed vectors. The reconstruction
//!   itself is in [`crate::pmv`]; bolting it onto this driver before
//!   we have a picture-level driver above would force a circular
//!   "slice driver knows picture state" coupling.
//! * The `marker_bit` after `motion_vectors(1)` is gated on
//!   `macroblock_intra && concealment_motion_vectors`; the
//!   concealment-MV path *itself* reads `motion_vectors(0)` even
//!   for an intra macroblock, so the marker_bit read has to follow
//!   that path landing.
//! * `coded_block_pattern()` is a small parser but the §6.3.17.4
//!   `pattern_code[12]` derivation it feeds is consumed by the
//!   §6.2.6 `block(i)` driver — already landed in
//!   [`crate::mpeg2_block_decoder`] — and the per-block walker
//!   [`crate::mpeg2_macroblock_blocks`]. Wiring those together
//!   requires the per-block `BlockContext` plus the §7.4.2.1
//!   weighting matrices, which today come from the sequence
//!   extension rather than the slice driver.
//!
//! These will land progressively in follow-on rounds; this driver
//! exposes per-macroblock [`MacroblockRecord::body_bit_position`] so
//! each upcoming round can resume parsing at the post-`macroblock_modes()`
//! cursor.
//!
//! ## What this module provides
//!
//! * [`SliceWalkContext`] — the per-picture / per-slice constants
//!   the driver needs (mb_width, picture_coding_type, mpeg1-vs-mpeg2
//!   address-increment context, initial quantiser_scale_code,
//!   plus the §6.3.11 `picture_structure` and `frame_pred_frame_dct`
//!   that gate the §6.2.5.1 `macroblock_modes()` tail).
//! * [`MacroblockRecord`] — the per-macroblock summary the walker
//!   emits, including the parsed `motion_type` and `dct_type` from
//!   `macroblock_modes()`.
//! * [`SliceWalk`] — the per-slice summary: the [`MacroblockRecord`]
//!   list, the final `previous_macroblock_address` /
//!   `past_intra_address`, and the bit position right after the last
//!   macroblock-header field (the entry point for the deferred body
//!   parsers above).
//! * [`walk_slice`] — the driver entry point.
//!
//! Spec citations refer to **ISO/IEC 13818-2:1995** (Recommendation
//! ITU-T H.262 (1995 E)) §5.2.3 (`next_start_code`), §6.2.4
//! (`slice()`), §6.2.5 / §6.2.5.1 (`macroblock()` /
//! `macroblock_modes()` — Tables 6-17 / 6-18 / 6-19 for the
//! motion-type / dct_type defaults), §6.3.11 (Table 6-14
//! `picture_structure` and `frame_pred_frame_dct`), §6.3.17.1
//! (slice-state semantics).

use oxideav_core::bits::BitReader;

use crate::coded_block_pattern::CodedBlockPattern;
use crate::combine_predictions::PredictionDirection;
use crate::macroblock_modes::{MacroblockModesContext, MacroblockModesTail, MotionType, MvFormat};
use crate::macroblock_type::MacroblockType;
use crate::mb_address_increment::{MbAddressIncrement, MbAddressIncrementContext};
use crate::motion_vector::{MotionVectors, MotionVectorsContext, MotionVectorsKind};
use crate::mpeg2_block_dc::DcPredictors;
use crate::mpeg2_dequantize::quantiser_scale;
use crate::mpeg2_macroblock_blocks::{
    decode_macroblock_blocks, DecodedBlock, MacroblockBlockContext,
};
use crate::picture_header::{PictureCodingType, PictureStructure};
use crate::pmv::{
    reconstruct_motion_vector, update_predictors, Direction, Pmv, PmvUpdateContext,
    PmvUpdateOutcome, ReconstructedComponent, VectorIndex,
};
use crate::quant_matrix_extension::QuantiserMatrixState;
use crate::quantizer_scale::{QUANTIZER_SCALE_MAX, QUANTIZER_SCALE_MIN};
use crate::sequence_extension::ChromaFormat;
use crate::skipped_macroblock::{
    apply_to_pmv as skipped_apply_to_pmv, describe_skipped_macroblock, SkippedMacroblockContext,
};
use crate::{Error, Result};

/// `past_intra_address` sentinel for "no intra macroblock has been
/// seen in the current picture yet" per §6.3.17.1: the value `-2`.
///
/// The spec uses `-2` (not `-1`) because the §2.4.4.1 / §7.4.1 DC
/// predictor reset gate is `(macroblock_address - past_intra_address) >
/// 1`; with `past_intra_address = -1` the first macroblock of the
/// picture would *not* trigger a reset (its difference of 1 is not
/// strictly greater than 1), but the spec requires every picture to
/// start with a fresh predictor state. `-2` makes the gate trigger
/// uniformly on the picture's first intra macroblock.
pub const PAST_INTRA_ADDRESS_RESET: i32 = -2;

/// Caller-supplied per-slice / per-picture context for [`walk_slice`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SliceWalkContext {
    /// Width of the picture in macroblocks per §6.3.17.1. Used to
    /// derive `previous_macroblock_address = mb_row * mb_width - 1`
    /// for the start of the slice and to bound the `macroblock_address`
    /// against the picture extent.
    pub mb_width: u32,
    /// `mb_row` of the slice as derived from
    /// [`crate::SliceHeader::mb_row`]. Combined with `mb_width` to
    /// pre-seed `previous_macroblock_address`.
    pub mb_row: u32,
    /// Picture coding type from `picture_header()`. Drives Table B-2
    /// / B-3 / B-4 selection for `macroblock_type`.
    pub picture_coding_type: PictureCodingType,
    /// Whether the slice belongs to a MPEG-1 (ISO/IEC 11172-2) stream
    /// (vs. MPEG-2 / 13818-2). Forwarded to
    /// [`MbAddressIncrementContext`] so the MPEG-1
    /// `macroblock_stuffing` code is recognised when applicable.
    pub mpeg1: bool,
    /// `quantiser_scale_code` from the parsed
    /// [`crate::SliceHeader::quantiser_scale_code`]. The driver
    /// carries this forward across macroblocks; any macroblock with
    /// `macroblock_quant == 1` overrides it for itself **and** all
    /// subsequent macroblocks in the slice (§6.3.17.1 / §6.2.5).
    pub initial_quantiser_scale_code: u8,
    /// `past_intra_address` carried over from the previous slice of
    /// the same picture. Callers parsing the picture from its first
    /// slice supply [`PAST_INTRA_ADDRESS_RESET`].
    pub past_intra_address: i32,
    /// `picture_structure` from `picture_coding_extension()`
    /// (§6.3.11, Table 6-14). Drives the §6.2.5.1 gates that decide
    /// whether `frame_motion_type` (`Frame`) or `field_motion_type`
    /// (`TopField` / `BottomField`) appears, and whether `dct_type`
    /// can appear at all (frame pictures only). MPEG-1 sequences
    /// always run with `Frame` per ISO/IEC 11172-2.
    pub picture_structure: PictureStructure,
    /// `frame_pred_frame_dct` from `picture_coding_extension()`
    /// (§6.3.11). When `true` and the picture is a frame, the
    /// `frame_motion_type` field is omitted (defaults to
    /// `Frame-based`) and `dct_type` is omitted (defaults to
    /// `frame` per Table 6-19). MPEG-1 sequences always run with
    /// `true` (no field-picture or field-DCT support per ISO/IEC
    /// 11172-2).
    pub frame_pred_frame_dct: bool,
    /// `f_code[0][0]` — forward horizontal `f_code` from
    /// `picture_coding_extension()` per §6.3.11 (range `1..=9`;
    /// `15` is the §6.3.11 "unused" marker). Drives the
    /// `motion_residual` bit-width when `s == 0`.
    ///
    /// Only consumed by the §6.2.5 `motion_vectors(0)` read, which
    /// fires when either `macroblock_motion_forward == 1` or
    /// `macroblock_intra && concealment_motion_vectors == 1`. For
    /// I-pictures with `concealment_motion_vectors == 0` and for
    /// every macroblock that triggers no `motion_vectors()` read
    /// the value is unused — placeholder `1` is safe.
    pub f_code_fwd_horiz: u8,
    /// `f_code[0][1]` — forward vertical `f_code` from
    /// `picture_coding_extension()` per §6.3.11.
    pub f_code_fwd_vert: u8,
    /// `f_code[1][0]` — backward horizontal `f_code` from
    /// `picture_coding_extension()` per §6.3.11. Only used when
    /// the picture coding type is `B` and `macroblock_motion_backward
    /// == 1`.
    pub f_code_bwd_horiz: u8,
    /// `f_code[1][1]` — backward vertical `f_code` from
    /// `picture_coding_extension()` per §6.3.11.
    pub f_code_bwd_vert: u8,
    /// `concealment_motion_vectors` from `picture_coding_extension()`
    /// per §6.3.11. When `true`, intra macroblocks carry a
    /// `motion_vectors(0)` block followed by a `marker_bit ==
    /// '1'`; when `false`, intra macroblocks have no
    /// motion-vector payload. Always `false` for MPEG-1 streams.
    pub concealment_motion_vectors: bool,
    /// `chroma_format` from `sequence_extension()` per §6.3.5.
    /// Drives the §6.2.5.3 `coded_block_pattern()` 4:2:2 / 4:4:4
    /// fixed-length extensions and the §6.3.17.4 `pattern_code[12]`
    /// derivation. MPEG-1 streams are always `Yuv420` (the
    /// chroma_format field of `sequence_extension()` doesn't exist
    /// in ISO/IEC 11172-2).
    pub chroma_format: ChromaFormat,
    /// `intra_vlc_format` from `picture_coding_extension()` per
    /// §6.3.11. Drives the §7.2.2.1 Table 7-3
    /// `(intra_vlc_format, macroblock_intra)` table selector
    /// (B-14 vs B-15) when the §6.2.6 `block(i)` driver runs.
    /// Only consumed when [`Self::block_decoding_enabled`] is
    /// `true`; otherwise the walker stops at the
    /// `coded_block_pattern()` cursor and this field is unused
    /// (default `false` is safe).
    pub intra_vlc_format: bool,
    /// `alternate_scan` from `picture_coding_extension()` per
    /// §6.3.11. Drives the §7.3 inverse-scan dispatch (Figure 7-2
    /// vs Figure 7-3) when the §6.2.6 `block(i)` driver runs.
    /// Only consumed when [`Self::block_decoding_enabled`] is
    /// `true`; default `false` is safe otherwise.
    pub alternate_scan: bool,
    /// `intra_dc_precision` from `picture_coding_extension()` per
    /// §6.3.11 (Table 6-13, `0..=3`). Drives the §7.2.1 DC
    /// predictor reset value (Table 7-2) and the Table 7-4
    /// `intra_dc_mult` per-block weighting when the §6.2.6
    /// `block(i)` driver runs. Only consumed when
    /// [`Self::block_decoding_enabled`] is `true`; default `0`
    /// (8-bit precision) is safe otherwise.
    pub intra_dc_precision: u8,
    /// `q_scale_type` from `picture_coding_extension()` per
    /// §6.3.11 (Table 7-6). Drives the §7.4.2.2 resolution from
    /// `quantiser_scale_code` (1..=31) to `quantiser_scale_value`
    /// (1..=112) when the §6.2.6 `block(i)` driver runs. Only
    /// consumed when [`Self::block_decoding_enabled`] is `true`;
    /// default `false` (linear) is safe otherwise.
    pub q_scale_type: bool,
    /// When `true`, the walker runs the §6.2.6 `block(i)` driver
    /// for every coded block per the parsed `pattern_code[i]`
    /// (advancing the cursor across `dct_coeff_*` + EOB and
    /// emitting the full per-block §A IDCT plane on each
    /// [`MacroblockRecord::decoded_blocks`] entry). When `false`
    /// (the default for every existing constructor), the walker
    /// stops at the `coded_block_pattern()` cursor as in rounds
    /// 30..33 — `decoded_blocks` is `None` on every record and
    /// the four §6.2.6 fields above are not consulted.
    pub block_decoding_enabled: bool,
    /// §7.4.2.1 Table 7-5 weighting matrices carried across the
    /// sequence per §6.3.11. Defaults to
    /// [`QuantiserMatrixState::defaults`] (the §6.3.7 default
    /// matrices) so existing callers that never decoded a
    /// `quant_matrix_extension()` keep the prior behaviour.
    ///
    /// Callers that *have* parsed a
    /// [`crate::quant_matrix_extension::QuantMatrixExtension`]
    /// thread the resulting [`QuantiserMatrixState`] through here
    /// (typically by chaining
    /// [`SliceWalkContext::with_quantiser_matrices`] off one of
    /// the existing constructors), and the walker forwards the
    /// matrices verbatim to [`crate::mpeg2_decode_macroblock_blocks`]
    /// so the §7.4.2.3 reconstruction step uses the
    /// user-downloaded matrices instead of the defaults. Only
    /// consumed when [`Self::block_decoding_enabled`] is `true`.
    ///
    /// Per §6.3.11 a `sequence_header()` resets every matrix back
    /// to its §6.3.7 default; the picture-level driver that owns
    /// the sequence-header parsing event invokes
    /// [`QuantiserMatrixState::reset_to_defaults`] at that point
    /// and then passes the (possibly default) state into this
    /// walker on its next call.
    pub quantiser_matrices: QuantiserMatrixState,
}

impl SliceWalkContext {
    /// Convenience constructor for the dominant non-scalable
    /// frame-picture case with `frame_pred_frame_dct == true`:
    /// every §6.2.5.1 motion-type / `dct_type` read is omitted per
    /// the Table 6-19 / Tables 6-17 defaults, so this context is
    /// safe for I-pictures (no motion possible) and for
    /// `frame_pred_frame_dct == true` P/B pictures where the
    /// caller knows the tail is suppressed. `past_intra_address`
    /// is seeded to [`PAST_INTRA_ADDRESS_RESET`].
    ///
    /// For field pictures or `frame_pred_frame_dct == false`
    /// frame pictures, use
    /// [`SliceWalkContext::first_slice_with_picture_extension`]
    /// so the walker reads the §6.2.5.1 tail fields.
    pub const fn first_slice(
        mb_width: u32,
        mb_row: u32,
        picture_coding_type: PictureCodingType,
        initial_quantiser_scale_code: u8,
    ) -> Self {
        Self {
            mb_width,
            mb_row,
            picture_coding_type,
            mpeg1: false,
            initial_quantiser_scale_code,
            past_intra_address: PAST_INTRA_ADDRESS_RESET,
            picture_structure: PictureStructure::Frame,
            frame_pred_frame_dct: true,
            f_code_fwd_horiz: 1,
            f_code_fwd_vert: 1,
            f_code_bwd_horiz: 1,
            f_code_bwd_vert: 1,
            concealment_motion_vectors: false,
            chroma_format: ChromaFormat::Yuv420,
            intra_vlc_format: false,
            alternate_scan: false,
            intra_dc_precision: 0,
            q_scale_type: false,
            block_decoding_enabled: false,
            quantiser_matrices: QuantiserMatrixState::defaults(),
        }
    }

    /// Full-fidelity constructor that surfaces the §6.3.11
    /// `picture_structure` and `frame_pred_frame_dct` fields the
    /// §6.2.5.1 `macroblock_modes()` tail (motion_type, dct_type)
    /// is gated on.
    pub const fn first_slice_with_picture_extension(
        mb_width: u32,
        mb_row: u32,
        picture_coding_type: PictureCodingType,
        initial_quantiser_scale_code: u8,
        picture_structure: PictureStructure,
        frame_pred_frame_dct: bool,
    ) -> Self {
        Self {
            mb_width,
            mb_row,
            picture_coding_type,
            mpeg1: false,
            initial_quantiser_scale_code,
            past_intra_address: PAST_INTRA_ADDRESS_RESET,
            picture_structure,
            frame_pred_frame_dct,
            f_code_fwd_horiz: 1,
            f_code_fwd_vert: 1,
            f_code_bwd_horiz: 1,
            f_code_bwd_vert: 1,
            concealment_motion_vectors: false,
            chroma_format: ChromaFormat::Yuv420,
            intra_vlc_format: false,
            alternate_scan: false,
            intra_dc_precision: 0,
            q_scale_type: false,
            block_decoding_enabled: false,
            quantiser_matrices: QuantiserMatrixState::defaults(),
        }
    }

    /// Full-fidelity constructor exposing every §6.3.5 / §6.3.11
    /// picture / sequence extension field the §6.2.5 macroblock body
    /// is gated on: `picture_structure`, `frame_pred_frame_dct`, the
    /// four `f_code[s][t]` entries that drive `motion_vector(r, s)`
    /// residual widths, the picture-level
    /// `concealment_motion_vectors` flag that gates
    /// `motion_vectors(0)` on intra macroblocks, and the
    /// sequence-level `chroma_format` that drives the §6.2.5.3
    /// `coded_block_pattern()` extensions and the §6.3.17.4
    /// `pattern_code[12]` derivation.
    ///
    /// Use this constructor when the slice body contains motion
    /// vectors and/or a `coded_block_pattern()` — i.e. any P- or
    /// B-picture macroblock that has `macroblock_motion_*` or
    /// `macroblock_pattern` set, or any intra macroblock in a
    /// picture with `concealment_motion_vectors == 1`. For purely
    /// intra slices with no concealment vectors,
    /// [`SliceWalkContext::first_slice`] or
    /// [`SliceWalkContext::first_slice_with_picture_extension`]
    /// suffice — their f_code / `chroma_format` placeholders are
    /// never read.
    #[allow(clippy::too_many_arguments)]
    pub const fn first_slice_with_picture_body(
        mb_width: u32,
        mb_row: u32,
        picture_coding_type: PictureCodingType,
        initial_quantiser_scale_code: u8,
        picture_structure: PictureStructure,
        frame_pred_frame_dct: bool,
        f_code_fwd_horiz: u8,
        f_code_fwd_vert: u8,
        f_code_bwd_horiz: u8,
        f_code_bwd_vert: u8,
        concealment_motion_vectors: bool,
        chroma_format: ChromaFormat,
    ) -> Self {
        Self {
            mb_width,
            mb_row,
            picture_coding_type,
            mpeg1: false,
            initial_quantiser_scale_code,
            past_intra_address: PAST_INTRA_ADDRESS_RESET,
            picture_structure,
            frame_pred_frame_dct,
            f_code_fwd_horiz,
            f_code_fwd_vert,
            f_code_bwd_horiz,
            f_code_bwd_vert,
            concealment_motion_vectors,
            chroma_format,
            intra_vlc_format: false,
            alternate_scan: false,
            intra_dc_precision: 0,
            q_scale_type: false,
            block_decoding_enabled: false,
            quantiser_matrices: QuantiserMatrixState::defaults(),
        }
    }

    /// Convenience constructor for the MPEG-1 (ISO/IEC 11172-2)
    /// case: every picture is a `Frame` and `frame_pred_frame_dct`
    /// is implicitly `true` (no field-organised tail fields exist
    /// in MPEG-1), so the §6.2.5.1 motion-type and `dct_type`
    /// reads are always omitted. MPEG-1's §2.4.2.7 macroblock layer
    /// keeps its own motion-vector parsers outside this driver.
    pub const fn first_slice_mpeg1(
        mb_width: u32,
        mb_row: u32,
        picture_coding_type: PictureCodingType,
        initial_quantiser_scale_code: u8,
    ) -> Self {
        Self {
            mb_width,
            mb_row,
            picture_coding_type,
            mpeg1: true,
            initial_quantiser_scale_code,
            past_intra_address: PAST_INTRA_ADDRESS_RESET,
            picture_structure: PictureStructure::Frame,
            frame_pred_frame_dct: true,
            f_code_fwd_horiz: 1,
            f_code_fwd_vert: 1,
            f_code_bwd_horiz: 1,
            f_code_bwd_vert: 1,
            concealment_motion_vectors: false,
            chroma_format: ChromaFormat::Yuv420,
            intra_vlc_format: false,
            alternate_scan: false,
            intra_dc_precision: 0,
            q_scale_type: false,
            block_decoding_enabled: false,
            quantiser_matrices: QuantiserMatrixState::defaults(),
        }
    }

    /// Full-fidelity constructor that surfaces every field the
    /// §6.2.5 macroblock header **and** the §6.2.6 `block(i)`
    /// driver consult — i.e. the `first_slice_with_picture_body`
    /// surface plus the four §6.3.11
    /// `picture_coding_extension()` fields the per-block
    /// reconstruction pipeline reads (`intra_vlc_format`,
    /// `alternate_scan`, `intra_dc_precision`, `q_scale_type`) —
    /// and toggles [`Self::block_decoding_enabled`] to `true` so
    /// the walker calls
    /// [`crate::mpeg2_decode_macroblock_blocks`] for every
    /// coded macroblock per the parsed `pattern_code[i]`.
    ///
    /// Use this constructor when the slice is to be fully decoded
    /// at the bitstream layer — every macroblock that has any
    /// coded block emits a [`MacroblockRecord::decoded_blocks`]
    /// `Some(Vec<DecodedBlock>)` payload carrying the §7.2 / §7.3
    /// / §7.4 / §A pipeline output. For wire-only walks (header
    /// fields + motion vectors + CBP but no per-block VLC walk),
    /// use [`SliceWalkContext::first_slice_with_picture_body`]
    /// instead — that path skips the §6.2.6 driver entirely.
    ///
    /// `intra_dc_precision` must be in `0..=3` per Table 6-13;
    /// values outside that range surface as
    /// [`Error::InvalidBitstream`] when the walker runs.
    /// `q_scale_type` selects between Table 7-6's linear and
    /// non-linear columns.
    #[allow(clippy::too_many_arguments)]
    pub const fn first_slice_with_block_decoding(
        mb_width: u32,
        mb_row: u32,
        picture_coding_type: PictureCodingType,
        initial_quantiser_scale_code: u8,
        picture_structure: PictureStructure,
        frame_pred_frame_dct: bool,
        f_code_fwd_horiz: u8,
        f_code_fwd_vert: u8,
        f_code_bwd_horiz: u8,
        f_code_bwd_vert: u8,
        concealment_motion_vectors: bool,
        chroma_format: ChromaFormat,
        intra_vlc_format: bool,
        alternate_scan: bool,
        intra_dc_precision: u8,
        q_scale_type: bool,
    ) -> Self {
        Self {
            mb_width,
            mb_row,
            picture_coding_type,
            mpeg1: false,
            initial_quantiser_scale_code,
            past_intra_address: PAST_INTRA_ADDRESS_RESET,
            picture_structure,
            frame_pred_frame_dct,
            f_code_fwd_horiz,
            f_code_fwd_vert,
            f_code_bwd_horiz,
            f_code_bwd_vert,
            concealment_motion_vectors,
            chroma_format,
            intra_vlc_format,
            alternate_scan,
            intra_dc_precision,
            q_scale_type,
            block_decoding_enabled: true,
            quantiser_matrices: QuantiserMatrixState::defaults(),
        }
    }

    /// Chain a parsed [`QuantiserMatrixState`] onto a context built
    /// from any of the other constructors. The state replaces the
    /// §6.3.7 default matrices for the §7.4.2.3 reconstruction step
    /// that the §6.2.6 `block(i)` driver runs on every coded block —
    /// the four `w`-indexed matrices in
    /// [`QuantiserMatrixState`] are forwarded verbatim through
    /// [`crate::mpeg2_macroblock_blocks::MacroblockBlockContext::weight_matrices`].
    ///
    /// Per §6.3.11 the picture-level driver above this walker
    /// invokes [`QuantiserMatrixState::reset_to_defaults`] whenever
    /// a fresh `sequence_header_code` is decoded; after that reset
    /// the resulting state is once again equivalent to the
    /// constructor's default and chaining
    /// [`Self::with_quantiser_matrices`] becomes a no-op.
    ///
    /// Used in tandem with
    /// [`crate::quant_matrix_extension::QuantMatrixExtension::apply`]
    /// which applies a parsed extension's optional payload onto a
    /// running [`QuantiserMatrixState`], so the picture-level
    /// driver's flow per picture is:
    ///
    /// ```text
    /// // once per sequence_header_code (§6.3.11):
    /// state.reset_to_defaults();
    /// // for every quant_matrix_extension() between pictures:
    /// extension.apply(&mut state, chroma_format);
    /// // when dispatching each slice:
    /// let ctx = SliceWalkContext::first_slice_with_block_decoding(...)
    ///     .with_quantiser_matrices(state);
    /// ```
    pub const fn with_quantiser_matrices(
        mut self,
        quantiser_matrices: QuantiserMatrixState,
    ) -> Self {
        self.quantiser_matrices = quantiser_matrices;
        self
    }
}

/// Per-macroblock summary the walker emits for one iteration of the
/// §6.2.4 do-while loop.
///
/// Note: this used to derive `Copy` (rounds 30–32) but now holds two
/// `MotionVectors` payloads (each carrying a small heap-allocated
/// `Vec<MotionVectorEntry>` for the `r`-loop entries) and a
/// `CodedBlockPattern` (a plain POD value but the surrounding type
/// is uniformly `Clone` only for parallelism with the motion-vector
/// payloads).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MacroblockRecord {
    /// `macroblock_address` per §6.3.17.1, i.e. the picture-relative
    /// raster index of this macroblock.
    pub macroblock_address: u32,
    /// `macroblock_address_increment` consumed for this macroblock
    /// (always `>= 1`; `== 1` on the slice's first macroblock per
    /// §6.3.17.1, may be `> 1` thereafter to indicate skipped
    /// macroblocks).
    pub address_increment: u16,
    /// Number of `macroblock_escape` codewords consumed in the
    /// preceding `macroblock_address_increment`. Surfaced for
    /// audit / round-trip purposes.
    pub address_escape_count: u8,
    /// Number of MPEG-1 `macroblock_stuffing` codewords consumed
    /// before the increment proper. Always `0` on MPEG-2 streams.
    pub address_stuffing_count: u8,
    /// Parsed `macroblock_type`. The six flag columns come straight
    /// from Tables B-2 / B-3 / B-4 against
    /// [`SliceWalkContext::picture_coding_type`].
    pub macroblock_type: MacroblockType,
    /// `frame_motion_type` (frame pictures) or `field_motion_type`
    /// (field pictures) decoded against Tables 6-17 / 6-18; `None`
    /// when the field is absent (no motion flag set, or
    /// `frame_pred_frame_dct == 1` in a frame picture). Per
    /// §6.3.17.1 / Table 6-19 the defaulted "as-if Frame-based"
    /// value when the field is absent is **not** synthesised here
    /// — callers consume `motion_type.unwrap_or(default)` at the
    /// motion-vector site.
    pub motion_type: Option<MotionType>,
    /// `dct_type` when present (frame picture, `frame_pred_frame_dct
    /// == 0`, and the macroblock is intra or has a coded pattern);
    /// `Some(true)` = field DCT coded, `Some(false)` = frame DCT
    /// coded. `None` when the field is absent (Table 6-19 supplies
    /// the effective value at the block-organisation site).
    pub dct_type: Option<bool>,
    /// The active `quantiser_scale_code` **after** this macroblock —
    /// equal to the macroblock-level override when `macroblock_quant
    /// == 1`, otherwise the value carried forward from the previous
    /// macroblock / slice header. Always in `1..=31` (§6.3.16 /
    /// §6.2.5 enforce non-zero).
    pub quantiser_scale_code: u8,
    /// `true` when this macroblock supplied its own `quantiser_scale_code`
    /// (`macroblock_quant == 1`); `false` when it inherited the slice
    /// / previous-MB value.
    pub macroblock_quant_present: bool,
    /// `past_intra_address` after this macroblock (set to
    /// `macroblock_address` when `macroblock_intra == 1`, carried
    /// forward otherwise).
    pub past_intra_address: i32,
    /// Bit position (relative to the start of the buffer the
    /// [`BitReader`] was created from) right after the
    /// macroblock-header chain. That is right after `macroblock_modes()`
    /// plus the §6.2.5 `if (macroblock_quant) quantiser_scale_code`
    /// read, and crucially still *before* any `motion_vectors()` /
    /// `coded_block_pattern()` / `block(i)` field. Downstream
    /// round-by-round drivers can pick up at this cursor when they
    /// want to re-parse the wire fields the walker captured above.
    pub body_bit_position: u64,
    /// Skipped-macroblock count derived from
    /// `address_increment - 1`: the number of macroblocks at addresses
    /// `previous_macroblock_address + 1 .. macroblock_address - 1`
    /// that the §7.6.6 round must reconstruct from the previous
    /// macroblock's state.
    pub skipped_macroblock_count: u32,
    /// `motion_vectors(0)` payload when the §6.2.5 syntax has the
    /// driver consume it — i.e. when either
    /// `macroblock_motion_forward == 1` (any picture coding type) or
    /// `macroblock_intra == 1 && concealment_motion_vectors == 1`
    /// (the §6.3.11 concealment path). `None` otherwise.
    ///
    /// This is the **wire-syntax** parse only: the §7.6.3.1
    /// reconstruction of `vector'[r][s][t]` against the PMV state
    /// (and the §7.6.3.3 update of PMV slots and §7.6.3.4 reset) is
    /// not run here — that needs cross-macroblock PMV state the
    /// slice walker doesn't own.
    pub motion_vectors_forward: Option<MotionVectors>,
    /// `motion_vectors(1)` payload when the §6.2.5 syntax has the
    /// driver consume it — i.e. when `macroblock_motion_backward
    /// == 1` (only legal in B-pictures). `None` otherwise. Same
    /// wire-only caveat as [`Self::motion_vectors_forward`].
    pub motion_vectors_backward: Option<MotionVectors>,
    /// The `marker_bit == '1'` consumed after `motion_vectors(0)` on
    /// intra macroblocks with `concealment_motion_vectors == 1`
    /// (§6.2.5). `Some(true)` when the marker bit was read (and
    /// matched); `None` when the gate is off and the marker bit is
    /// absent. The §6.3.17 marker-bit rule is enforced — a `'0'`
    /// in this slot rejects the slice as
    /// [`Error::InvalidBitstream`].
    pub concealment_marker_bit: Option<bool>,
    /// `coded_block_pattern()` payload when `macroblock_pattern ==
    /// 1` (§6.2.5). `None` otherwise — meaning either the
    /// macroblock is intra without `macroblock_pattern == 1`
    /// (full-pattern by default per §6.3.17.4) or the macroblock
    /// carries no coded residuals at all.
    pub coded_block_pattern: Option<CodedBlockPattern>,
    /// `pattern_code[12]` derived from the macroblock per §6.3.17.4:
    ///
    /// * intra macroblock without `macroblock_pattern` → every entry
    ///   `true` (every block is coded; DC-only intra blocks are
    ///   still "coded" in the §6.3.17.4 sense).
    /// * `macroblock_pattern == 1` → driven by
    ///   `coded_block_pattern.pattern_code(macroblock_intra,
    ///   macroblock_pattern)` per §6.3.17.4.
    /// * neither — every entry `false` (the macroblock is purely
    ///   prediction with no residuals at all; the
    ///   [`crate::macroblock_pipeline`] reconstruction passes the
    ///   prediction through unchanged).
    ///
    /// Entries `0..6` cover the §6.1.1.8 Y/Cb/Cr block ordering for
    /// 4:2:0; entries `6..8` extend Cb/Cr for 4:2:2; entries
    /// `8..12` extend Cb/Cr for 4:4:4. Entries past
    /// [`crate::mpeg2_block_count`] for the active chroma_format
    /// are always `false`.
    pub pattern_code: [bool; 12],
    /// §6.2.6 `block(i)` payloads, one entry per **coded** block
    /// (i.e. each `i` with `pattern_code[i] == true`), in §6.1.1.8
    /// raster order.
    ///
    /// * `None` — the walker was running in wire-only mode
    ///   ([`SliceWalkContext::block_decoding_enabled == false`]),
    ///   so the per-block §7.2 / §7.3 / §7.4 / §A pipeline never
    ///   ran and the cursor stopped at the
    ///   `coded_block_pattern()` snapshot. This is the round-30..33
    ///   contract.
    /// * `Some(Vec::new())` — block decoding was enabled but the
    ///   macroblock has zero coded blocks (every `pattern_code[i]`
    ///   is `false`; e.g. a non-intra MB with no `macroblock_pattern`
    ///   and no coded residuals at all).
    /// * `Some(blocks)` — `blocks.len() ==` number of `true` entries
    ///   in [`Self::pattern_code`], up to the §6.1.1.8
    ///   `block_count(chroma_format)` slot count. Each entry carries
    ///   the full `QFS[] → QF[v][u] → F[v][u] → f[y][x]` reconstruction
    ///   plus the post-EOB bit cursor.
    ///
    /// Per §7.2.1 the per-component DC predictor state is reset at
    /// every non-intra macroblock; the per-slice [`walk_slice`]
    /// driver carries the predictor across macroblocks and applies
    /// that reset via the inner driver before each coded block runs.
    pub decoded_blocks: Option<Vec<DecodedBlock>>,
}

/// Per-slice summary the walker emits when the §6.2.4 do-while loop
/// terminates.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SliceWalk {
    /// All parsed macroblock-header records, in bitstream order.
    pub macroblocks: Vec<MacroblockRecord>,
    /// `previous_macroblock_address` per §6.3.17.1 after the last
    /// macroblock of the slice. Equal to the last
    /// `MacroblockRecord::macroblock_address` when the slice was
    /// non-empty; equal to the seeded value (`mb_row * mb_width - 1`)
    /// when the slice contained zero macroblocks (which is a spec
    /// violation, but the driver surfaces the state for inspection
    /// rather than asserting).
    pub previous_macroblock_address: i32,
    /// Final `past_intra_address`, carrying forward to the next slice
    /// of the same picture per §6.3.17.1.
    pub past_intra_address: i32,
    /// Final `quantiser_scale_code` after the last macroblock —
    /// equal to the last record's value, or the
    /// [`SliceWalkContext::initial_quantiser_scale_code`] when the
    /// slice contained zero macroblocks.
    pub quantiser_scale_code: u8,
    /// Bit position right after the last macroblock's header chain
    /// — i.e. the position at which the §6.2.4 stop-condition check
    /// passed.
    pub end_bit_position: u64,
}

/// Resolve the §6.2.5.2 / §6.3.17.1 effective `MotionType` to thread
/// into [`MotionVectors::parse`].
///
/// * When the `macroblock_modes()` tail parsed an explicit
///   `frame_motion_type` / `field_motion_type` code, that is the
///   answer.
/// * When the field is **absent**, §6.3.17.1 + Table 6-19 supply the
///   default: `Frame-based` for frame pictures, `Field-based` for
///   field pictures, with `motion_vector_count = 1`, `dmv = 0`, and
///   `mv_format` matching the prediction type (frame for frame-based,
///   field for field-based). This is the path concealment-MV intra
///   macroblocks (§6.3.11) and `frame_pred_frame_dct == 1` motion
///   MBs follow.
///
/// The function is deterministic on `(macroblock_type, modes_tail,
/// picture_structure)`; it never reads bits and so cannot fail on
/// bitstream exhaustion. The MPEG-2 spec also reserves Table 6-17
/// code `00` — that is rejected upstream by
/// [`MacroblockModesTail::parse`] when the code is read explicitly,
/// so we never see it here.
fn effective_motion_type(
    macroblock_type: &MacroblockType,
    modes_tail: &MacroblockModesTail,
    ctx: &SliceWalkContext,
) -> Result<MotionType> {
    use crate::macroblock_modes::{MvFormat, PredictionType};
    if let Some(mt) = modes_tail.motion_type {
        return Ok(mt);
    }
    // §6.3.17.1 default for the absent-tail case.
    let _ = macroblock_type;
    let (prediction_type, mv_format) = match ctx.picture_structure {
        PictureStructure::Frame => (PredictionType::FrameBased, MvFormat::Frame),
        PictureStructure::TopField | PictureStructure::BottomField => {
            (PredictionType::FieldBased, MvFormat::Field)
        }
    };
    // The raw `code` field of `MotionType` is the wire-level 2-bit
    // value. For the absent-tail case there is no wire code; we pick
    // the Table 6-17 / 6-18 row that *would* have produced the same
    // prediction type so callers can read it without surprises.
    let code = match prediction_type {
        PredictionType::FrameBased => 0b10,
        PredictionType::FieldBased => 0b01,
        _ => unreachable!("absent-tail default is Frame-based or Field-based per §6.3.17.1"),
    };
    Ok(MotionType {
        code,
        prediction_type,
        motion_vector_count: 1,
        mv_format,
        dmv: false,
    })
}

/// Walk the §6.2.4 macroblock loop, parsing the macroblock-header
/// chain for each iteration and accumulating the per-slice summary.
///
/// `buf` is expected to start at
/// [`crate::SliceHeader::body_bit_position`] **mapped to a byte-
/// aligned cursor**: callers chain a fresh [`BitReader`] off the
/// same buffer the slice header was parsed from and seek to that
/// bit position (see test fixtures for the idiomatic shape).
///
/// The driver stops as soon as the §6.2.4 / §5.2.3 stop condition
/// fires:
/// * `nextbits()` shows 23 zero bits when peeked at the current
///   byte-aligned position — i.e. the next byte-aligned word is
///   `0x00 0x00 0x00..0x01` consistent with a `next_start_code()`
///   prefix.
/// * Or the buffer ends without enough remaining bits to peek the
///   23-bit stop pattern; the driver reports a successful walk
///   anyway since `next_start_code()` is allowed to *be* the end of
///   the buffer when the caller passed a slice-bounded sub-buffer.
///
/// Errors:
/// * [`Error::InvalidBitstream`] if `macroblock_address_increment !=
///   1` on the first macroblock (§6.3.17.1), if `macroblock_address`
///   exceeds the `u32::MAX` representable range, or if the
///   macroblock-level `quantiser_scale_code` is `0` (forbidden per
///   §6.3.16) — plus whatever [`MbAddressIncrement::parse`] /
///   [`MacroblockType::parse`] reject. Strict `mb_height` bounding
///   is deferred to the picture-level driver.
/// * [`Error::ShortHeader`] if any required field runs past the end
///   of `buf`.
pub fn walk_slice(buf: &[u8], ctx: SliceWalkContext) -> Result<SliceWalk> {
    walk_slice_at(buf, 0, ctx)
}

/// Walk the §6.2.4 macroblock body starting `body_bit_position` bits
/// into `buf`, rather than from the very start.
///
/// This is the picture-level entry point: a real elementary stream's
/// slice body begins at the (generally **not** byte-aligned)
/// [`crate::SliceHeader::body_bit_position`] inside the slice's
/// byte-aligned start-code-relative buffer. The plain [`walk_slice`]
/// requires a body-only buffer (it reads from bit 0); this variant
/// seeks past the slice header in-place so callers can pass the whole
/// slice buffer the [`crate::SliceHeader`] was parsed from together
/// with the `body_bit_position` the header reported.
///
/// `buf` must start at the slice's `0x000001` start-code byte (the
/// same buffer [`crate::SliceHeader::parse`] consumed);
/// `body_bit_position` is the value [`crate::SliceHeader::body_bit_position`]
/// returned. Behaviour is otherwise identical to [`walk_slice`].
///
/// # Errors
///
/// Same as [`walk_slice`], plus [`Error::ShortHeader`] if the seek to
/// `body_bit_position` runs past the end of `buf`.
pub fn walk_slice_at(
    buf: &[u8],
    body_bit_position: u64,
    ctx: SliceWalkContext,
) -> Result<SliceWalk> {
    let mut br = BitReader::new(buf);
    if body_bit_position > 0 {
        let to_skip = u32::try_from(body_bit_position).map_err(|_| Error::ShortHeader)?;
        br.skip(to_skip).map_err(|_| Error::ShortHeader)?;
    }
    let mut records: Vec<MacroblockRecord> = Vec::new();

    let mb_width_i64 = i64::from(ctx.mb_width);
    let mb_row_i64 = i64::from(ctx.mb_row);
    let mut previous_macroblock_address: i64 = mb_row_i64 * mb_width_i64 - 1;
    let mut past_intra_address: i32 = ctx.past_intra_address;
    let mut quantiser_scale_code: u8 = ctx.initial_quantiser_scale_code;

    // §6.3.16 forbids 0, but the caller derived this from
    // SliceHeader::quantiser_scale_code which already rejected 0. We
    // re-assert at the entry point so the slice-walk surface stays
    // self-consistent against hand-built contexts.
    if !(QUANTIZER_SCALE_MIN..=QUANTIZER_SCALE_MAX).contains(&quantiser_scale_code) {
        return Err(Error::InvalidBitstream(
            "initial_quantiser_scale_code: must be in 1..=31 (§6.3.16)",
        ));
    }
    if ctx.mb_width == 0 {
        return Err(Error::InvalidBitstream(
            "mb_width: zero macroblocks per row is not a legal sequence (§6.3.3)",
        ));
    }

    let increment_ctx = if ctx.mpeg1 {
        MbAddressIncrementContext::mpeg1()
    } else {
        MbAddressIncrementContext::mpeg2()
    };

    // §7.2.1: per-slice DC-predictor state. Allocated only when
    // the §6.2.6 driver is gated on — the wire-only path keeps
    // the round-30..33 contract of "no DC predictor state needed,
    // walker stops at CBP". The §7.2.1 "reset at start of slice"
    // rule is satisfied by [`DcPredictors::new`] which seeds every
    // component to the Table 7-2 reset value selected by
    // `intra_dc_precision`. Block decoding requires
    // `intra_dc_precision` to be in `0..=3` (Table 6-13); the
    // validation surfaces as [`Error::InvalidBitstream`] up-front.
    let mut dc_predictors: Option<DcPredictors> = if ctx.block_decoding_enabled && !ctx.mpeg1 {
        Some(DcPredictors::new(ctx.intra_dc_precision)?)
    } else {
        None
    };

    // MPEG-1 (ISO/IEC 11172-2) block decoding carries the §2.4.4.1
    // `dct_dc_*_past` + `past_intra_address` chain instead of the
    // §7.2.1 predictor bank. The chain starts in its §2.4.4.1
    // slice-start state (`128 * 8` per component, address `-2`); the
    // per-non-intra / per-skipped resets are realised through the
    // `(macroblock_address - past_intra_address) > 1` test the
    // dequantiser applies, driven by `finalise_intra_macroblock`.
    let mut mpeg1_dc_predictors: Option<crate::dequantize::IntraDcPredictors> =
        if ctx.block_decoding_enabled && ctx.mpeg1 {
            Some(crate::dequantize::IntraDcPredictors::at_slice_start())
        } else {
            None
        };

    let end_bit_position: u64;

    loop {
        // §6.2.4 stop-condition: `nextbits() != '0000 0000 0000 0000
        // 0000 0000'`. Per §5.2.3 `nextbits()` peeks the next bits
        // **without** advancing the cursor and **without** requiring
        // byte alignment — the alignment happens inside
        // `next_start_code()` after the do-while exits. We peek 23
        // bits because the minimal prefix of a start code is
        // `0x000001` (24 bits), one of whose 23 leading bits is the
        // last `0` of the all-zero zero-byte stuffing the spec
        // permits; the 24-bit `'0000 0000 0000 0000 0000 0001'`
        // would prematurely match a malformed bitstream that
        // happens to put a `1` bit at the right offset, so we use
        // the conservative 23-bit pattern.
        //
        // If the buffer is too short to peek 23 full bits the
        // slice runs to the end of the stream (a legal stream may
        // end right after its last slice with no sequence_end_code
        // appended by the transport). §5.2.3 zero-stuffs up to the
        // next start code, so the truncated peek is evaluated with
        // zero extension: an all-zero (or empty) tail is stuffing
        // — a successful slice end — while any `1` bit in the tail
        // is the next macroblock's data and the walk continues.
        match br.peek_u32(23) {
            Ok(0) => {
                end_bit_position = br.bit_position();
                break;
            }
            Ok(_) => {
                // Not a stop pattern — fall through and parse the
                // next macroblock.
            }
            Err(_) => {
                let remaining = br.bits_remaining().min(22) as u32;
                if remaining == 0 || matches!(br.peek_u32(remaining), Ok(0)) {
                    end_bit_position = br.bit_position();
                    break;
                }
                // A `1` bit inside the short tail: more macroblock
                // data — fall through and parse the next macroblock.
            }
        }

        // §6.2.5: macroblock_address_increment (with optional
        // macroblock_escape / macroblock_stuffing chains).
        let increment = MbAddressIncrement::parse(&mut br, increment_ctx)?;

        // §6.3.17.1: the first macroblock of every slice shall have
        // macroblock_address_increment == 1.
        let is_first = records.is_empty();
        if is_first && increment.value != 1 {
            return Err(Error::InvalidBitstream(
                "macroblock_address_increment: first macroblock of slice must be 1 (§6.3.17.1)",
            ));
        }

        let macroblock_address = previous_macroblock_address
            .checked_add(i64::from(increment.value))
            .ok_or(Error::InvalidBitstream(
                "macroblock_address: i64 overflow (§6.3.17.1)",
            ))?;

        // §6.3.17.1: macroblock_address must stay within
        // mb_row * mb_width <= addr < mb_width * (mb_row + 1) +
        // mb_width * remaining_rows — i.e. within the picture extent.
        // We don't know mb_height here (the caller's concern), so we
        // bound only against "still on the same row" optimistically;
        // strict mb_height bounding is deferred to the picture-level
        // driver.
        if macroblock_address < 0 {
            return Err(Error::InvalidBitstream(
                "macroblock_address: went negative — increment skipped past start of slice (§6.3.17.1)",
            ));
        }
        // u32 upper-bound check — slice walks cannot run beyond u32
        // worth of macroblocks. Real pictures cap at <2^20.
        if macroblock_address > i64::from(u32::MAX) {
            return Err(Error::InvalidBitstream(
                "macroblock_address: exceeded u32 range (§6.3.17.1)",
            ));
        }
        let macroblock_address_u32 = macroblock_address as u32;

        // §6.3.17.1: any macroblocks at addresses
        // previous_macroblock_address + 1 .. macroblock_address - 1
        // are skipped. Count is `increment - 1` modulo the first-MB
        // rule above (which has increment == 1, so 0 skipped).
        let skipped_macroblock_count = u32::from(increment.value) - 1;

        // §7.2.1 (page 71): the per-component DC predictors are reset
        // "whenever a macroblock is skipped", in addition to the
        // slice-start and non-intra-macroblock resets. The skip run
        // precedes *this* coded macroblock, so the reset must land
        // before this macroblock's blocks decode — an intra macroblock
        // that follows a skip run codes its `dct_dc_differential`
        // against the Table 7-2 reset value, not against the previous
        // intra macroblock's final predictor.
        if skipped_macroblock_count > 0 {
            if let Some(ref mut predictors) = dc_predictors {
                predictors.reset();
            }
        }

        // §6.2.5.1: macroblock_modes() opens with macroblock_type
        // (Tables B-2 / B-3 / B-4 keyed on picture_coding_type).
        let macroblock_type = MacroblockType::parse(&mut br, ctx.picture_coding_type)?;

        // §6.2.5.1: the remainder of macroblock_modes() —
        // `spatial_temporal_weight_code` (gated on
        // `spatial_temporal_weight_code_flag == 1`, never set by
        // the non-scalable Tables B-2 / B-3 / B-4 this walker
        // reaches), `frame_motion_type` / `field_motion_type`
        // (gated on either motion flag and on
        // `frame_pred_frame_dct` for frame pictures), and
        // `dct_type` (gated on frame picture && !frame_pred_frame_dct
        // && (intra || pattern)). The walker reads these now so
        // the cursor advances past `macroblock_modes()` before
        // the §6.2.5 `if (macroblock_quant) quantiser_scale_code`
        // check fires. MPEG-1 streams reach this branch with
        // `frame_pred_frame_dct == true` and `picture_structure ==
        // Frame` (forced by `first_slice_mpeg1`), so every
        // motion-type / dct_type read is gated off — MPEG-1's
        // macroblock layer keeps its own §2.4.2.7 fields out of
        // this driver.
        let modes_ctx =
            MacroblockModesContext::new(ctx.picture_structure, ctx.frame_pred_frame_dct);
        let modes_tail = MacroblockModesTail::parse(&mut br, &macroblock_type, &modes_ctx)?;

        // §6.2.5: if (macroblock_quant) read 5-bit
        // quantiser_scale_code in 1..=31. This follows the full
        // `macroblock_modes()` block above per the §6.2.5 syntax
        // tree, not directly after `macroblock_type`.
        let macroblock_quant_present = macroblock_type.macroblock_quant;
        if macroblock_quant_present {
            let raw = br.read_u32(5).map_err(|_| Error::ShortHeader)? as u8;
            if !(QUANTIZER_SCALE_MIN..=QUANTIZER_SCALE_MAX).contains(&raw) {
                return Err(Error::InvalidBitstream(
                    "macroblock-level quantiser_scale_code: must be in 1..=31 (§6.3.16 / §6.2.5)",
                ));
            }
            // §6.3.17.1: a macroblock-level override applies to this
            // macroblock and every subsequent macroblock in the slice.
            quantiser_scale_code = raw;
        }

        // §6.3.17.1: past_intra_address advances on intra MBs.
        if macroblock_type.macroblock_intra {
            past_intra_address = macroblock_address_u32 as i32;
        }

        // Snapshot the cursor right after the macroblock-header chain
        // (mb_addr_inc + macroblock_modes() + quantiser_scale_code).
        // The wire-position body fields below live further into the
        // bitstream and have their own `bit_position_after` cursors,
        // so we preserve the historical [`MacroblockRecord::body_bit_position`]
        // value for callers that resume parsing the body themselves.
        let body_bit_position = br.bit_position();

        // §6.2.5 macroblock body wire-parse: motion_vectors(0),
        // optional motion_vectors(1), optional marker_bit
        // (concealment), and optional coded_block_pattern().
        // **Wire-syntax only** — the §7.6.3.1 reconstruction of
        // `vector'[r][s][t]` and the §7.6.3.3 PMV-slot update are
        // driven from the per-picture / per-slice PMV state held by
        // the picture-level driver one layer up and are intentionally
        // not run here.

        // §6.2.5: motion_vectors(0) is read iff
        // `macroblock_motion_forward == 1` (any picture coding type) or
        // `macroblock_intra && concealment_motion_vectors == 1`
        // (§6.3.11 picture-extension gate).
        let needs_forward = macroblock_type.macroblock_motion_forward
            || (macroblock_type.macroblock_intra && ctx.concealment_motion_vectors);
        let motion_vectors_forward = if needs_forward {
            let motion_type = effective_motion_type(&macroblock_type, &modes_tail, &ctx)?;
            let mv_ctx = MotionVectorsContext {
                f_code_fwd_horiz: ctx.f_code_fwd_horiz,
                f_code_fwd_vert: ctx.f_code_fwd_vert,
                f_code_bwd_horiz: ctx.f_code_bwd_horiz,
                f_code_bwd_vert: ctx.f_code_bwd_vert,
            };
            Some(MotionVectors::parse(
                &mut br,
                MotionVectorsKind::Forward,
                &motion_type,
                &mv_ctx,
            )?)
        } else {
            None
        };

        // §6.2.5: motion_vectors(1) iff macroblock_motion_backward.
        // (The picture_coding_type==B constraint is enforced upstream
        // by the Tables B-3 / B-4 macroblock_type VLCs: B-3 has no
        // row that sets `bwd`, so the only legal stream path reaching
        // a `true` here is a B-picture macroblock_type from B-4.)
        let motion_vectors_backward = if macroblock_type.macroblock_motion_backward {
            let motion_type = effective_motion_type(&macroblock_type, &modes_tail, &ctx)?;
            let mv_ctx = MotionVectorsContext {
                f_code_fwd_horiz: ctx.f_code_fwd_horiz,
                f_code_fwd_vert: ctx.f_code_fwd_vert,
                f_code_bwd_horiz: ctx.f_code_bwd_horiz,
                f_code_bwd_vert: ctx.f_code_bwd_vert,
            };
            Some(MotionVectors::parse(
                &mut br,
                MotionVectorsKind::Backward,
                &motion_type,
                &mv_ctx,
            )?)
        } else {
            None
        };

        // §6.2.5: if (macroblock_intra && concealment_motion_vectors)
        // marker_bit. The §6.3.17 marker_bit rule requires the bit be
        // `'1'`; we reject `'0'` as a §6.3.17 violation.
        let concealment_marker_bit =
            if macroblock_type.macroblock_intra && ctx.concealment_motion_vectors {
                let bit = br.read_bit().map_err(|_| Error::ShortHeader)?;
                if !bit {
                    return Err(Error::InvalidBitstream(
                        "concealment marker_bit: must be '1' (§6.3.17 / §6.2.5)",
                    ));
                }
                Some(bit)
            } else {
                None
            };

        // §6.2.5: if (macroblock_pattern) coded_block_pattern().
        // The §6.3.17.4 `pattern_code[12]` derivation is then driven
        // from the parsed CBP plus the `macroblock_intra` /
        // `macroblock_pattern` flags.
        let coded_block_pattern = if macroblock_type.macroblock_pattern {
            Some(CodedBlockPattern::parse(&mut br, ctx.chroma_format)?)
        } else {
            None
        };

        // §6.3.17.4 pattern_code[12] derivation:
        // * macroblock_pattern == 1 → driven by the parsed CBP.
        // * macroblock_intra == 1 && !macroblock_pattern → every
        //   block coded (CBP slot is *not* in the bitstream; every
        //   intra block carries at least a DC coefficient per
        //   §7.2.1 / §2.4.2.8).
        // * else → no coded blocks (a pure-prediction MB with no
        //   residuals at all).
        let pattern_code = if let Some(ref cbp) = coded_block_pattern {
            cbp.pattern_code(
                macroblock_type.macroblock_intra,
                macroblock_type.macroblock_pattern,
            )
        } else if macroblock_type.macroblock_intra {
            [true; 12]
        } else {
            [false; 12]
        };

        // §6.2.6 `block(i)` driver: when the surrounding context
        // signals `block_decoding_enabled`, run the per-block
        // §7.2.1 / §7.2.2 / §7.3 / §7.4 / §A pipeline for every
        // coded block via [`decode_macroblock_blocks`]. Skipped
        // when block decoding is gated off — the cursor stops at
        // the post-`coded_block_pattern()` snapshot per the
        // round-30..33 wire-only contract.
        //
        // The §6.2.6 syntax is gated on `pattern_code[i]`; the
        // helper iterates the §6.1.1.8 block ordering internally
        // and skips uncoded slots so a zero-CBP non-intra MB
        // emits an empty `Vec<DecodedBlock>` with no bitstream
        // reads. Intra MBs still emit one §7.2.1 DC prelude per
        // coded block per §6.2.6.
        let decoded_blocks = if let Some(ref mut mpeg1_predictors) = mpeg1_dc_predictors {
            // MPEG-1 §2.4.3.7 block loop: iterate the six 4:2:0
            // block slots gated by `pattern_code[i]` and decode each
            // through the §2.4.4.1 / §2.4.4.2 pipeline. MPEG-1's
            // `quantizer_scale` is the 5-bit linear value directly
            // (there is no Table 7-6 mapping) and the chrominance
            // blocks share the luminance matrices (§2.4.2.3 defines
            // one intra and one non-intra matrix for the whole
            // picture).
            let mut blocks = Vec::new();
            for i in 0..6u8 {
                if !pattern_code[i as usize] {
                    continue;
                }
                let decoded = if macroblock_type.macroblock_intra {
                    crate::mpeg1_block_decoder::decode_intra_block(
                        &mut br,
                        i,
                        quantiser_scale_code,
                        &ctx.quantiser_matrices.intra_luma,
                        mpeg1_predictors,
                        macroblock_address_u32 as i32,
                    )?
                } else {
                    crate::mpeg1_block_decoder::decode_non_intra_block(
                        &mut br,
                        quantiser_scale_code,
                        &ctx.quantiser_matrices.non_intra_luma,
                    )?
                };
                let component = crate::mpeg2_macroblock_blocks::block_component(
                    i as usize,
                    ChromaFormat::Yuv420,
                )
                .ok_or(Error::InvalidBitstream(
                    "mpeg1 block index out of the 4:2:0 range (§2.4.3.7)",
                ))?;
                blocks.push(crate::mpeg2_macroblock_blocks::DecodedBlock {
                    block_index: i,
                    component,
                    decoded,
                });
            }
            if macroblock_type.macroblock_intra {
                // §2.4.4.1: `past_intra_address = macroblock_address`
                // after all the blocks in the macroblock are
                // processed.
                crate::dequantize::finalise_intra_macroblock(
                    mpeg1_predictors,
                    macroblock_address_u32 as i32,
                );
            }
            Some(blocks)
        } else if let Some(ref mut predictors) = dc_predictors {
            // §7.4.2.2 Table 7-6: resolve `quantiser_scale_code`
            // through the `q_scale_type` column to the final
            // `quantiser_scale_value` in `1..=112`. The `quantiser_scale_code`
            // here is the post-override value carried by the
            // walker (i.e. either the slice-header value or the
            // most-recent `macroblock_quant == 1` override),
            // which is the spec-correct input to Table 7-6 per
            // §7.4.2.2 (the override applies to *this* MB).
            let quantiser_scale_value = quantiser_scale(quantiser_scale_code, ctx.q_scale_type)?;
            // §6.3.7 / §6.3.11 weighting matrices. The
            // [`SliceWalkContext::quantiser_matrices`] field now
            // carries a [`QuantiserMatrixState`] that the
            // picture-level driver maintains across each
            // `sequence_header_code` reset (§6.3.11 first sentence)
            // and `quant_matrix_extension()` apply call
            // ([`crate::quant_matrix_extension::QuantMatrixExtension::apply`]),
            // so each of the four Table 7-5 `w`-indexed matrices
            // [`MacroblockBlockContext::weight_matrices`] reads is
            // the user-downloaded one when an extension overrode
            // it and the §6.3.7 default otherwise. The
            // post-reset / no-extension case is byte-identical to
            // the prior `with_default_weight_matrices` path since
            // [`QuantiserMatrixState::defaults`] returns exactly
            // the §6.3.7 defaults.
            let weight_matrices = [
                ctx.quantiser_matrices.intra_luma,
                ctx.quantiser_matrices.non_intra_luma,
                ctx.quantiser_matrices.intra_chroma,
                ctx.quantiser_matrices.non_intra_chroma,
            ];
            let mb_block_ctx = MacroblockBlockContext {
                intra_vlc_format: ctx.intra_vlc_format,
                alternate_scan: ctx.alternate_scan,
                intra_dc_precision: ctx.intra_dc_precision,
                quantiser_scale_value,
                chroma_format: ctx.chroma_format,
                weight_matrices: &weight_matrices,
            };
            // [`decode_macroblock_blocks`] internally derives
            // `pattern_code` from the same CBP / macroblock_type
            // we already computed above, so when the macroblock
            // has no `coded_block_pattern` payload but is
            // non-intra (every entry `false`) it returns an
            // empty `Vec` without reading any bits. The walker's
            // own per-MB `coded_block_pattern.is_none() &&
            // !macroblock_intra` case is the same path.
            //
            // §7.2.1 also says the DC predictor is reset on
            // every non-intra MB; the helper applies that
            // before its per-block loop runs.
            //
            // Macroblocks whose `macroblock_pattern == 0` and
            // `macroblock_intra == 1` (pattern_code == [true;
            // 12]) reach this helper without a parsed CBP —
            // the helper handles that by treating "no CBP" as
            // "every block coded" so we synthesise a
            // pattern-all-coded shim CBP for the call.
            let cbp_for_decode = coded_block_pattern.unwrap_or(CodedBlockPattern {
                // §6.3.17.4: intra MB without `macroblock_pattern`
                // has every block coded. The §6.2.5.3 CBP wire
                // encoding for "every block coded" is `cbp ==
                // 63` (with no 4:2:2 / 4:4:4 extension), but
                // [`CodedBlockPattern::pattern_code(true, false)`]
                // ignores the `cbp` payload entirely and returns
                // `[true; 12]` straight away. Any CBP is
                // therefore safe here; we use the "every block
                // coded" row + no extensions so a hypothetical
                // future `pattern_code(macroblock_intra,
                // macroblock_pattern)` mismatch would still be
                // self-consistent. `bit_position_after` is
                // synthesised at the current cursor — no CBP
                // bits were actually read.
                cbp: 63,
                coded_block_pattern_1: None,
                coded_block_pattern_2: None,
                bit_position_after: br.bit_position(),
            });
            let blocks = decode_macroblock_blocks(
                &mut br,
                &mb_block_ctx,
                predictors,
                &macroblock_type,
                &cbp_for_decode,
            )?;
            Some(blocks)
        } else {
            None
        };

        records.push(MacroblockRecord {
            macroblock_address: macroblock_address_u32,
            address_increment: increment.value,
            address_escape_count: increment.escape_count,
            address_stuffing_count: increment.stuffing_count,
            macroblock_type,
            motion_type: modes_tail.motion_type,
            dct_type: modes_tail.dct_type,
            quantiser_scale_code,
            macroblock_quant_present,
            past_intra_address,
            body_bit_position,
            skipped_macroblock_count,
            motion_vectors_forward,
            motion_vectors_backward,
            concealment_marker_bit,
            coded_block_pattern,
            pattern_code,
            decoded_blocks,
        });

        previous_macroblock_address = macroblock_address;
    }

    let previous_macroblock_address_i32 = if previous_macroblock_address < 0 {
        // Empty slice — record the seeded "before-first-MB" value as
        // i32 so callers know nothing landed.
        if previous_macroblock_address < i64::from(i32::MIN) {
            i32::MIN
        } else {
            previous_macroblock_address as i32
        }
    } else if previous_macroblock_address > i64::from(i32::MAX) {
        i32::MAX
    } else {
        previous_macroblock_address as i32
    };

    Ok(SliceWalk {
        macroblocks: records,
        previous_macroblock_address: previous_macroblock_address_i32,
        past_intra_address,
        quantiser_scale_code,
        end_bit_position,
    })
}

/// One reconstructed `vector'[r][s][:]` pair (`t = 0` horizontal,
/// `t = 1` vertical) per §7.6.3.1, surfaced after running
/// [`reconstruct_record_motion_vectors`] across a [`MacroblockRecord`].
///
/// The horizontal / vertical components are paired so the picture-
/// level driver can index `[r]` to read the post-reconstruction
/// motion vector for slot `r` and feed it into the §7.6.4 forming-
/// predictions pel reader.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReconstructedVector {
    /// Horizontal component (`vector'[r][s][0]`) and the updated
    /// `PMV[r][s][0]` value the spec wrote back.
    pub horizontal: ReconstructedComponent,
    /// Vertical component (`vector'[r][s][1]`) and the updated
    /// `PMV[r][s][1]` value.
    pub vertical: ReconstructedComponent,
}

/// All reconstructed motion vectors for one macroblock — the forward
/// (`s = 0`) and backward (`s = 1`) entries, each with up to two
/// `(horizontal, vertical)` components.
///
/// `forward` / `backward` mirror
/// [`MacroblockRecord::motion_vectors_forward`] /
/// [`MacroblockRecord::motion_vectors_backward`] presence: `None`
/// when the wire-syntax parse skipped the field, `Some(vec)` when it
/// was consumed (with `vec.len() ∈ {1, 2}` matching the parsed
/// `motion_vector_count`).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ReconstructedMotionVectors {
    /// `vector'[r][0][t]` per parsed forward `motion_vectors(0)`
    /// entry, one row per `motion_vector_count`.
    pub forward: Option<Vec<ReconstructedVector>>,
    /// `vector'[r][1][t]` per parsed backward `motion_vectors(1)`
    /// entry.
    pub backward: Option<Vec<ReconstructedVector>>,
}

/// Run the §7.6.3.1 PMV reconstruction for every parsed motion
/// vector on a single [`MacroblockRecord`] using the round-238
/// [`crate::pmv::reconstruct_motion_vector`] entry point. Updates
/// `pmv` in place per Table 7-7 and returns the reconstructed
/// `vector'[r][s][:]` pairs the §7.6.4 forming-predictions stage
/// reads.
///
/// `mv_format_override`, when `Some`, forces the `mv_format` field
/// of the [`MotionType`] used for §7.6.3.1 (the §6.3.17.1 /
/// Table 6-19 default for concealment-MV intra macroblocks where
/// no `frame_motion_type` was present in the bitstream). When
/// `None`, the parsed `MotionVectors`' embedded entries' bit-shape
/// already determines the `mv_format`; this helper picks the
/// dominant case where every entry's `mv_format` matches the
/// surrounding macroblock's `effective_motion_type`.
///
/// `picture_structure` is the §6.3.11 `picture_structure` (from
/// the surrounding [`SliceWalkContext::picture_structure`]). It
/// drives the §7.6.3.1 vertical-half-pred gate.
///
/// The §7.6.3.4 reset of `pmv` at slice boundaries is the
/// caller's responsibility (the picture-level driver does this
/// before each new slice); this helper assumes the predictor bank
/// already holds the post-previous-macroblock values per §7.6.3.3.
///
/// Errors:
/// * [`Error::InvalidBitstream`] when a parsed `motion_code` +
///   `f_code` combination violates §7.6.3.1's `[low, high]` range
///   even after wrap (see [`crate::pmv::reconstruct_component`]
///   for the full error surface).
pub fn reconstruct_record_motion_vectors(
    record: &MacroblockRecord,
    pmv: &mut Pmv,
    ctx: &SliceWalkContext,
) -> Result<ReconstructedMotionVectors> {
    let mut out = ReconstructedMotionVectors::default();

    // The motion_type the record carries is the parsed wire value
    // (`None` if the §6.2.5.1 tail was omitted). The PMV reconstruction
    // path needs the *effective* motion_type per §6.3.17.1 /
    // Table 6-19; reuse the same `effective_motion_type` helper the
    // wire-parse path uses so the mv_format here matches what
    // [`MotionVectors::parse`] expanded against above.
    let effective_mt = effective_motion_type(&record.macroblock_type, &derive_tail(record), ctx)?;
    let mv_format = effective_mt.mv_format;

    if let Some(ref mvs) = record.motion_vectors_forward {
        let recons = reconstruct_mvs(
            mvs,
            Direction::Forward,
            mv_format,
            ctx,
            ctx.f_code_fwd_horiz,
            ctx.f_code_fwd_vert,
            pmv,
        )?;
        out.forward = Some(recons);
    }
    if let Some(ref mvs) = record.motion_vectors_backward {
        let recons = reconstruct_mvs(
            mvs,
            Direction::Backward,
            mv_format,
            ctx,
            ctx.f_code_bwd_horiz,
            ctx.f_code_bwd_vert,
            pmv,
        )?;
        out.backward = Some(recons);
    }
    Ok(out)
}

/// The §7.6.3 PMV side-effects applied for one macroblock by the
/// [`reconstruct_slice_motion_vectors`] driver, surfaced so tests and
/// callers can confirm the right §7.6.3.3 / §7.6.3.4 row fired without
/// re-reading the running [`Pmv`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SliceMotionRecord {
    /// `macroblock_address` of the **coded** macroblock this entry
    /// describes (mirrors [`MacroblockRecord::macroblock_address`]).
    pub macroblock_address: u32,
    /// Number of §7.6.6 skipped macroblocks immediately preceding this
    /// coded macroblock (`address_increment - 1`). Each one applied its
    /// §7.6.3.4 PMV side-effect via [`skipped_apply_to_pmv`] before the
    /// coded macroblock's own vectors were reconstructed.
    pub skipped_before: u32,
    /// `true` when at least one of the §7.6.6 skipped macroblocks in
    /// `skipped_before` reset the running PMV (P-picture rule). `false`
    /// when there were no skipped macroblocks, or the picture is a
    /// B-picture (§7.6.6.3 / §7.6.6.4 "predictors unaffected").
    pub skipped_reset_pmv: bool,
    /// The reconstructed `vector'[r][s][:]` pairs for this coded
    /// macroblock per §7.6.3.1 (empty `forward` / `backward` mirror the
    /// wire-parse presence on the [`MacroblockRecord`]).
    pub reconstructed: ReconstructedMotionVectors,
    /// The §7.6.3.3 update-row label that fired for this coded
    /// macroblock (Tables 7-10 / 7-11), or `None` for the MPEG-1
    /// (ISO/IEC 11172-2) path where §7.6.3.3 does not apply.
    pub update_outcome: Option<PmvUpdateOutcome>,
    /// The running predictor bank **as it stood when this macroblock's
    /// processing began** — after the previous coded macroblock's
    /// §7.6.3.3 update, before this entry's §7.6.6 skip run applied its
    /// side-effect and before this macroblock's own reconstruction.
    /// This is the predictor state a §7.6.6.4 B-picture skipped
    /// macroblock in the preceding run reads its motion vectors from
    /// (*"the motion vectors are taken directly from the appropriate
    /// motion vector predictors"*).
    pub pmv_before: Pmv,
}

/// Per-slice result of [`reconstruct_slice_motion_vectors`]: the
/// per-coded-macroblock §7.6.3 side-effect log plus the final running
/// PMV state (to seed the next slice's §7.6.3.4 reset boundary, or to
/// read the last predictor values from).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SliceMotionWalk {
    /// One entry per coded macroblock in the slice, in bitstream order.
    pub records: Vec<SliceMotionRecord>,
    /// The running [`Pmv`] after the last coded macroblock's §7.6.3.3
    /// update — i.e. the predictor bank as it stands at the §6.2.4
    /// stop condition.
    pub pmv: Pmv,
}

/// Run the full §7.6.3 motion-vector reconstruction lifecycle across an
/// already-parsed [`SliceWalk`], carrying the §7.6.3 predictor bank
/// (`PMV[r][s][t]`) across every macroblock of the slice.
///
/// The per-record [`reconstruct_record_motion_vectors`] entry point
/// reconstructs one macroblock's vectors against a caller-owned [`Pmv`]
/// but leaves the surrounding lifecycle — the §7.6.3.4 reset at slice
/// start, the §7.6.3.3 [`update_predictors`] table after each coded
/// macroblock, and the §7.6.6 skipped-macroblock PMV side-effects — to
/// the caller. This driver composes those steps into a single pass so a
/// higher-level picture driver can hand it a `SliceWalk` and read back
/// the reconstructed vectors plus the running predictor state.
///
/// The lifecycle per the spec is:
///
/// 1. **§7.6.3.4 slice-start reset.** The predictor bank is zeroed
///    before the first macroblock — *"At the start of each slice the
///    motion vector predictors are reset to zero."*
/// 2. For each coded [`MacroblockRecord`] (in bitstream order):
///     * **§7.6.6 skipped-macroblock run.** The
///       `address_increment - 1` macroblocks between this one and the
///       previous coded one are skipped (§6.3.17.1). For each, the
///       §7.6.6 description is derived ([`describe_skipped_macroblock`])
///       and its §7.6.3.4 PMV side-effect applied
///       ([`skipped_apply_to_pmv`] — a P-picture reset, a B-picture
///       no-op). The skip run is processed **before** the coded
///       macroblock's vectors so the coded macroblock differentially
///       decodes against the post-skip predictor state.
///     * **§7.6.3.1 reconstruction.** The coded macroblock's parsed
///       vectors are reconstructed via
///       [`reconstruct_record_motion_vectors`], writing the
///       `vector'[r][s][:]` values into the predictor slots per
///       Table 7-7.
///     * **§7.6.3.3 update.** [`update_predictors`] applies the
///       Tables 7-10 / 7-11 "Predictors to Update" column (copy the
///       first-vector predictor into the second slot, reset on a
///       zero-motion non-intra macroblock, etc.). Skipped for the
///       MPEG-1 path (`ctx.mpeg1`), whose §2.4.4.2 / §2.4.4.3
///       reconstruction owns its own predictor update and is decoded
///       through [`crate::mpeg1_reconstruct`].
///
/// The §7.6.3.4 reset *between* slices is the caller's responsibility:
/// each call resets at its own start, so a picture-level driver simply
/// calls this once per slice. The returned [`SliceMotionWalk::pmv`] is
/// the post-slice predictor state for inspection; it is **not** carried
/// into the next slice (a fresh slice resets per §7.6.3.4).
///
/// # Errors
///
/// * Propagates [`Error::InvalidBitstream`] from
///   [`reconstruct_record_motion_vectors`] (an out-of-range
///   `motion_code` + `f_code`), from [`update_predictors`] (a
///   `(prediction_type, fwd, bwd, intra)` combination absent from
///   Tables 7-10 / 7-11), and from [`describe_skipped_macroblock`] (a
///   skipped macroblock in a non-scalable I-picture, or a B-picture
///   skip whose previous macroblock had no encoded direction).
pub fn reconstruct_slice_motion_vectors(
    walk: &SliceWalk,
    ctx: &SliceWalkContext,
) -> Result<SliceMotionWalk> {
    // §7.6.3.4: the predictor bank starts the slice zeroed.
    let mut pmv = Pmv::new();
    let mut records: Vec<SliceMotionRecord> = Vec::with_capacity(walk.macroblocks.len());

    // §7.6.6.3 / §7.6.6.4: a B-picture skipped macroblock copies the
    // direction of the *previous coded* macroblock. We carry that
    // direction across the slice so each skip run can name its source.
    let mut previous_direction: Option<PredictionDirection> = None;

    for record in &walk.macroblocks {
        let skipped_before = record.skipped_macroblock_count;
        let mut skipped_reset_pmv = false;
        // Snapshot the predictor bank before any per-MB side-effect —
        // the state a §7.6.6.4 skip run preceding this coded MB reads.
        let pmv_before = pmv;

        // §7.6.6: process the run of skipped macroblocks that precede
        // this coded one. Their §7.6.3.4 PMV side-effect is applied
        // before the coded macroblock reconstructs its own vectors.
        if skipped_before > 0 {
            let previous = previous_direction.unwrap_or(PredictionDirection::Forward);
            let skip_ctx = SkippedMacroblockContext {
                picture_coding_type: ctx.picture_coding_type,
                picture_structure: ctx.picture_structure,
                previous_direction: previous,
                // The §7.6.6 preamble's scalable-I-picture exemption is
                // not surfaced through the slice walker yet; a
                // non-scalable I-picture skip is a bitstream error,
                // which `describe_skipped_macroblock` rejects.
                scalable_i_picture: false,
                pmv,
            };
            let description = describe_skipped_macroblock(skip_ctx)?;
            // The §7.6.3.4 side-effect is identical for every skipped MB
            // in the run (a P-picture reset is idempotent; a B-picture
            // run is a no-op), so applying it once per run reaches the
            // same predictor state as applying it per macroblock.
            skipped_apply_to_pmv(&description, &mut pmv);
            skipped_reset_pmv = description.reset_pmv;
        }

        // §7.6.3.1: reconstruct this coded macroblock's vectors against
        // the running predictor bank. The MPEG-1 path keeps its own
        // §2.4.4.2 / §2.4.4.3 reconstruction (out of scope for the
        // MPEG-2 §7.6.3 PMV bank), so the wire-only records carry no
        // MPEG-2 motion-vector payloads in that mode and this is a no-op
        // pass-through that records the empty reconstruction.
        let reconstructed = reconstruct_record_motion_vectors(record, &mut pmv, ctx)?;

        // §7.6.3.3: apply the Tables 7-10 / 7-11 predictor-update row.
        // MPEG-1 does not have the §7.6.3.3 table; skip it there.
        let update_outcome = if ctx.mpeg1 {
            None
        } else {
            let mt = &record.macroblock_type;
            // §6.3.17.1 / Table 6-19: a non-intra macroblock whose
            // `frame_motion_type` / `field_motion_type` field was absent
            // from the bitstream (the `frame_pred_frame_dct == 1` frame
            // picture, or any field-picture default) still has an
            // *effective* prediction type — Frame-based in a frame
            // picture, Field-based in a field picture. The §7.6.3.3
            // update table is keyed on that effective type, so resolve
            // it the same way the §7.6.3.1 reconstruction path does
            // rather than passing the raw (possibly `None`) wire value.
            let prediction_type = if mt.macroblock_intra {
                record.motion_type.map(|m| m.prediction_type)
            } else {
                Some(effective_motion_type(mt, &derive_tail(record), ctx)?.prediction_type)
            };
            let update_ctx = PmvUpdateContext {
                picture_structure: ctx.picture_structure,
                prediction_type,
                macroblock_motion_forward: mt.macroblock_motion_forward,
                macroblock_motion_backward: mt.macroblock_motion_backward,
                macroblock_intra: mt.macroblock_intra,
                concealment_motion_vectors: ctx.concealment_motion_vectors,
            };
            Some(update_predictors(&mut pmv, update_ctx)?)
        };

        previous_direction = Some(record_direction(&record.macroblock_type));

        records.push(SliceMotionRecord {
            macroblock_address: record.macroblock_address,
            skipped_before,
            skipped_reset_pmv,
            reconstructed,
            update_outcome,
            pmv_before,
        });
    }

    Ok(SliceMotionWalk { records, pmv })
}

/// Map a coded macroblock's §6.3.17.1 motion flags to the
/// [`PredictionDirection`] a following §7.6.6 B-picture skip run copies.
/// An intra macroblock has no prediction direction; §7.6.6.3 /
/// §7.6.6.4 forbid a B-picture skip immediately after an intra
/// macroblock, so [`describe_skipped_macroblock`] rejects the
/// [`PredictionDirection::Skipped`] sentinel we map intra to.
fn record_direction(macroblock_type: &MacroblockType) -> PredictionDirection {
    match (
        macroblock_type.macroblock_motion_forward,
        macroblock_type.macroblock_motion_backward,
    ) {
        (true, true) => PredictionDirection::Bidirectional,
        (true, false) => PredictionDirection::Forward,
        (false, true) => PredictionDirection::Backward,
        // Intra (or a non-intra MB with no motion, which a B-picture
        // skip cannot follow): the §7.6.6 driver rejects this sentinel.
        (false, false) => PredictionDirection::Skipped,
    }
}

/// Rebuild the [`MacroblockModesTail`] payload the §6.2.5.1 parser
/// stored on the record. The record only carries `motion_type` and
/// `dct_type`; a re-synthesis matching the wire-time leaf fields is
/// enough for [`effective_motion_type`] — the `bit_position_after`
/// is replayed from the record's `body_bit_position` snapshot.
fn derive_tail(record: &MacroblockRecord) -> MacroblockModesTail {
    MacroblockModesTail {
        spatial_temporal_weight: None,
        motion_type: record.motion_type,
        dct_type: record.dct_type,
        bit_position_after: record.body_bit_position,
    }
}

#[allow(clippy::too_many_arguments)]
fn reconstruct_mvs(
    mvs: &MotionVectors,
    s: Direction,
    mv_format: MvFormat,
    ctx: &SliceWalkContext,
    f_code_horiz: u8,
    f_code_vert: u8,
    pmv: &mut Pmv,
) -> Result<Vec<ReconstructedVector>> {
    let mut out: Vec<ReconstructedVector> = Vec::with_capacity(mvs.entries.len());
    for (idx, entry) in mvs.entries.iter().enumerate() {
        // Table 7-7: `r ∈ {0, 1}`. The §6.2.5.2 `r`-loop iterates in
        // bitstream order, so index 0 → First, index 1 → Second.
        let r = match idx {
            0 => VectorIndex::First,
            1 => VectorIndex::Second,
            // The §6.2.5.2 parser caps `motion_vector_count` at 2.
            _ => {
                return Err(Error::InvalidBitstream(
                    "reconstruct_record_motion_vectors: motion_vector_count > 2 (Tables 6-17 / 6-18)",
                ));
            }
        };
        let [h, v] = reconstruct_motion_vector(
            pmv,
            &entry.motion_vector,
            r,
            s,
            f_code_horiz,
            f_code_vert,
            mv_format,
            ctx.picture_structure,
        )?;
        out.push(ReconstructedVector {
            horizontal: h,
            vertical: v,
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact `slice()`-body fixtures for every
    //! spec-defined entry point this driver exposes.

    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Emit the Table B-1 codeword for `macroblock_address_increment`
    /// values `1..=33`. We re-create the table inline so this test
    /// stays self-contained — the canonical table is in
    /// [`crate::mb_address_increment`].
    fn write_address_increment(bw: &mut BitWriter, value: u16) {
        // Subset of Table B-1 used by these tests (1, 2, 3, 4, 5).
        // The full 33-row table is exercised in mb_address_increment's
        // own test module.
        match value {
            1 => bw.write_u32(0b1, 1),
            2 => bw.write_u32(0b011, 3),
            3 => bw.write_u32(0b010, 3),
            4 => bw.write_u32(0b0011, 4),
            5 => bw.write_u32(0b0010, 4),
            other => panic!("test fixture only supports increment in 1..=5, got {other}"),
        }
    }

    /// Table B-2 (I-pictures): `macroblock_type` codewords.
    /// Row "Intra" = `1`; row "Intra, Quant" = `01`.
    fn write_mb_type_i_intra(bw: &mut BitWriter) {
        bw.write_u32(0b1, 1);
    }
    fn write_mb_type_i_intra_quant(bw: &mut BitWriter) {
        bw.write_u32(0b01, 2);
    }

    /// Table B-3 (P-pictures): subset used by these tests.
    /// "Pattern, motion forward" = `1`.
    fn write_mb_type_p_pattern_fwd(bw: &mut BitWriter) {
        bw.write_u32(0b1, 1);
    }

    /// Emit a 5-bit `quantiser_scale_code`.
    fn write_q_scale(bw: &mut BitWriter, value: u8) {
        bw.write_u32(u32::from(value), 5);
    }

    /// Emit the Table B-10 `motion_code = 0` codeword (the 1-bit `1`).
    /// When the surrounding context has `f_code == 1`, no residual
    /// follows; combined with `dmv == 0`, this is the shortest legal
    /// `motion_vector(r, s)` wire form — 2 bits total for the
    /// horizontal + vertical components.
    fn write_zero_motion_vector(bw: &mut BitWriter) {
        // horizontal motion_code = 0 → 1-bit `1`.
        bw.write_u32(0b1, 1);
        // vertical motion_code = 0 → 1-bit `1`.
        bw.write_u32(0b1, 1);
    }

    /// Emit the smallest legal `motion_vectors(s)` wire form for a
    /// frame-based MB whose `mv_format == Frame` and
    /// `motion_vector_count == 1` and surrounding f_code == 1 — i.e.
    /// just one zero-vector with no vertical_field_select bit.
    fn write_zero_motion_vectors_frame_one(bw: &mut BitWriter) {
        write_zero_motion_vector(bw);
    }

    /// Emit the smallest legal `motion_vectors(s)` wire form for a
    /// field-picture `16x8 MC` MB whose `mv_format == Field` and
    /// `motion_vector_count == 2`: two `(vfs, zero_motion_vector)`
    /// pairs.
    fn write_zero_motion_vectors_field_two(bw: &mut BitWriter) {
        for _ in 0..2 {
            bw.write_u32(0, 1);
            write_zero_motion_vector(bw);
        }
    }

    /// Emit the Table B-9 codeword for `cbp = 60` (`0b111`, 3 bits)
    /// — the densest row of the table. We pick `60` for tests that
    /// just need any valid CBP to align the cursor.
    fn write_cbp_60(bw: &mut BitWriter) {
        bw.write_u32(0b111, 3);
    }

    /// Emit the Table B-12 `dct_dc_size_luminance = 0` codeword
    /// (`100`, 3 bits) — the shortest legal DC prelude on a
    /// luminance intra block.
    fn write_dc_size_zero_luma(bw: &mut BitWriter) {
        bw.write_u32(0b100, 3);
    }

    /// Emit the Table B-13 `dct_dc_size_chrominance = 0` codeword
    /// (`00`, 2 bits) — the shortest legal DC prelude on a
    /// chrominance intra block.
    fn write_dc_size_zero_chroma(bw: &mut BitWriter) {
        bw.write_u32(0b00, 2);
    }

    /// Emit the Table B-14 `end_of_block` codeword (`10`, 2 bits).
    /// This is the EOB used by the FIRST/NEXT walker once
    /// `dct_dc_size == 0` has already absorbed the only coefficient
    /// the block carries.
    fn write_eob_b14(bw: &mut BitWriter) {
        bw.write_u32(0b10, 2);
    }

    /// Emit the wire form of one §6.2.6 intra `block(i)` whose DC
    /// size is 0 and whose residual is an immediate EOB — i.e.
    /// `QFS = [0, 0, ..., 0]` riding on the DC predictor.
    ///
    /// * Luma: 3 + 2 = 5 bits.
    /// * Chroma: 2 + 2 = 4 bits.
    fn write_dc_zero_intra_block(bw: &mut BitWriter, is_luma: bool) {
        if is_luma {
            write_dc_size_zero_luma(bw);
        } else {
            write_dc_size_zero_chroma(bw);
        }
        write_eob_b14(bw);
    }

    /// Emit the wire form of one 4:2:0 intra macroblock whose 6
    /// blocks (4 luma + 1 Cb + 1 Cr) all use `dct_dc_size == 0` +
    /// immediate EOB. Total length is 4 * 5 + 2 * 4 = 28 bits.
    fn write_dc_zero_intra_macroblock_420(bw: &mut BitWriter) {
        for _ in 0..4 {
            write_dc_zero_intra_block(bw, true);
        }
        write_dc_zero_intra_block(bw, false);
        write_dc_zero_intra_block(bw, false);
    }

    /// Pad with zero bits up to the next byte boundary and append at
    /// least 3 zero bytes so the stop-pattern peek finds 23 zero
    /// bits.
    fn end_with_stop(mut bw: BitWriter) -> Vec<u8> {
        bw.align_to_byte_zero();
        let mut bytes = bw.finish();
        bytes.extend_from_slice(&[0x00, 0x00, 0x00, 0x01, 0xB7]);
        bytes
    }

    #[test]
    fn empty_slice_with_immediate_stop_pattern() {
        // The slice body starts on a byte-aligned position and the
        // first 23 bits are zero. The driver returns zero
        // macroblocks (which is a spec violation but this layer is
        // not the enforcement point).
        let buf = vec![0x00, 0x00, 0x00, 0x01, 0xB7];
        let walk = walk_slice(
            &buf,
            SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 1),
        )
        .unwrap();
        assert!(walk.macroblocks.is_empty());
        assert_eq!(walk.quantiser_scale_code, 1);
        assert_eq!(walk.past_intra_address, PAST_INTRA_ADDRESS_RESET);
    }

    #[test]
    fn single_intra_macroblock_i_picture() {
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        let buf = end_with_stop(bw);

        let walk = walk_slice(
            &buf,
            SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 14),
        )
        .unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        let mb0 = &walk.macroblocks[0];
        assert_eq!(mb0.macroblock_address, 0);
        assert_eq!(mb0.address_increment, 1);
        assert_eq!(mb0.address_escape_count, 0);
        assert_eq!(mb0.address_stuffing_count, 0);
        assert!(mb0.macroblock_type.macroblock_intra);
        assert!(!mb0.macroblock_type.macroblock_quant);
        assert_eq!(mb0.quantiser_scale_code, 14);
        assert!(!mb0.macroblock_quant_present);
        assert_eq!(mb0.past_intra_address, 0);
        assert_eq!(mb0.skipped_macroblock_count, 0);

        assert_eq!(walk.previous_macroblock_address, 0);
        assert_eq!(walk.past_intra_address, 0);
        assert_eq!(walk.quantiser_scale_code, 14);
    }

    #[test]
    fn intra_quant_overrides_slice_quantiser() {
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra_quant(&mut bw);
        write_q_scale(&mut bw, 7);
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        let buf = end_with_stop(bw);

        let walk = walk_slice(
            &buf,
            SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 31),
        )
        .unwrap();
        assert_eq!(walk.macroblocks.len(), 2);
        assert!(walk.macroblocks[0].macroblock_quant_present);
        assert_eq!(walk.macroblocks[0].quantiser_scale_code, 7);
        assert_eq!(walk.macroblocks[0].past_intra_address, 0);
        // Carry-forward: second MB inherits the overridden q-scale.
        assert!(!walk.macroblocks[1].macroblock_quant_present);
        assert_eq!(walk.macroblocks[1].quantiser_scale_code, 7);
        assert_eq!(walk.macroblocks[1].macroblock_address, 1);
        assert_eq!(walk.macroblocks[1].past_intra_address, 1);

        assert_eq!(walk.previous_macroblock_address, 1);
        assert_eq!(walk.past_intra_address, 1);
        assert_eq!(walk.quantiser_scale_code, 7);
    }

    #[test]
    fn first_macroblock_rejects_increment_above_one() {
        // increment == 2 on the first MB is a §6.3.17.1 violation.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 2);
        write_mb_type_i_intra(&mut bw);
        let buf = end_with_stop(bw);

        let err = walk_slice(
            &buf,
            SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 1),
        )
        .unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn p_picture_skipped_macroblocks_recorded() {
        // P-picture with one fwd-pattern MB, then increment=3 to skip
        // 2 MBs, then another fwd-pattern MB. Table B-3 "Pattern,
        // motion forward" sets `fwd=true, pattern=true`, so each MB
        // carries `motion_vectors(0)` and `coded_block_pattern()`.
        // We use the `first_slice(...)` default with
        // `frame_pred_frame_dct == true` so no `frame_motion_type`
        // is emitted and the absent-tail default Frame-based
        // (mv_count == 1, mv_format == Frame, dmv == 0) applies.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_p_pattern_fwd(&mut bw);
        write_zero_motion_vectors_frame_one(&mut bw);
        write_cbp_60(&mut bw);
        write_address_increment(&mut bw, 3);
        write_mb_type_p_pattern_fwd(&mut bw);
        write_zero_motion_vectors_frame_one(&mut bw);
        write_cbp_60(&mut bw);
        let buf = end_with_stop(bw);

        let walk = walk_slice(
            &buf,
            SliceWalkContext::first_slice(22, 1, PictureCodingType::Predictive, 8),
        )
        .unwrap();
        // mb_row=1 → previous_macroblock_address starts at 22*1-1 = 21.
        assert_eq!(walk.macroblocks.len(), 2);
        assert_eq!(walk.macroblocks[0].macroblock_address, 22);
        assert_eq!(walk.macroblocks[0].skipped_macroblock_count, 0);
        assert_eq!(walk.macroblocks[1].macroblock_address, 25);
        assert_eq!(walk.macroblocks[1].skipped_macroblock_count, 2);
        assert_eq!(walk.previous_macroblock_address, 25);
        // No intra MBs encountered.
        assert_eq!(walk.past_intra_address, PAST_INTRA_ADDRESS_RESET);
    }

    #[test]
    fn past_intra_address_carries_over_within_slice() {
        // Two intra MBs in an I-picture — past_intra_address must
        // advance to each MB's address as it's parsed.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        let buf = end_with_stop(bw);

        let walk = walk_slice(
            &buf,
            SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 1),
        )
        .unwrap();
        assert_eq!(walk.macroblocks.len(), 3);
        assert_eq!(walk.macroblocks[0].past_intra_address, 0);
        assert_eq!(walk.macroblocks[1].past_intra_address, 1);
        assert_eq!(walk.macroblocks[2].past_intra_address, 2);
        assert_eq!(walk.past_intra_address, 2);
    }

    #[test]
    fn rejects_zero_initial_quantiser_scale_code() {
        let buf = vec![0x00, 0x00, 0x00, 0x01, 0xB7];
        let err = walk_slice(
            &buf,
            SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 0),
        )
        .unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_zero_mb_width() {
        let buf = vec![0x00, 0x00, 0x00, 0x01, 0xB7];
        let err = walk_slice(
            &buf,
            SliceWalkContext::first_slice(0, 0, PictureCodingType::Intra, 1),
        )
        .unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn body_bit_position_advances_past_header_chain() {
        let mut bw = BitWriter::new();
        // increment=1 (1 bit) + Table B-2 "Intra" macroblock_type (1
        // bit) = 2 bits before the post-header cursor.
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        let buf = end_with_stop(bw);

        let walk = walk_slice(
            &buf,
            SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 1),
        )
        .unwrap();
        assert_eq!(walk.macroblocks[0].body_bit_position, 2);
    }

    #[test]
    fn quantiser_scale_carries_forward_across_macroblocks() {
        // MB0 = Intra-Quant, q=7. MB1 = Intra (no quant). MB2 =
        // Intra-Quant, q=15. Expected final = 15.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra_quant(&mut bw);
        write_q_scale(&mut bw, 7);
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra_quant(&mut bw);
        write_q_scale(&mut bw, 15);
        let buf = end_with_stop(bw);

        let walk = walk_slice(
            &buf,
            SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 31),
        )
        .unwrap();
        assert_eq!(walk.macroblocks.len(), 3);
        assert_eq!(walk.macroblocks[0].quantiser_scale_code, 7);
        assert_eq!(walk.macroblocks[1].quantiser_scale_code, 7);
        assert_eq!(walk.macroblocks[2].quantiser_scale_code, 15);
        assert_eq!(walk.quantiser_scale_code, 15);
    }

    /// Table B-3 P-picture row "MC, Not Coded" = `001` (3 bits) —
    /// `fwd = true`, `bwd = false`, `pattern = false`, `intra = false`.
    /// Combined with `picture_structure = Frame` and
    /// `frame_pred_frame_dct = false`, the §6.2.5.1 syntax demands
    /// a 2-bit `frame_motion_type` between the macroblock_type and
    /// the (absent) `quantiser_scale_code`.
    fn write_mb_type_p_mc_not_coded(bw: &mut BitWriter) {
        bw.write_u32(0b001, 3);
    }

    #[test]
    fn frame_motion_type_read_when_motion_flag_and_not_frame_pred_frame_dct() {
        // Single P-picture MB carrying forward motion, in a frame
        // picture with `frame_pred_frame_dct == 0`. §6.2.5.1 then
        // emits a 2-bit `frame_motion_type` after `macroblock_type`.
        //
        // Table 6-17 `frame_motion_type` codes:
        // * `01` → Field-based, mv_count = 2 (class 0/1)
        // * `10` → Frame-based, mv_count = 1
        // * `11` → Dual-Prime, mv_count = 1
        //
        // We pick `10` (Frame-based) — the dominant
        // `frame_pred_frame_dct == 0` case.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_p_mc_not_coded(&mut bw);
        // frame_motion_type = `10` → Frame-based.
        bw.write_u32(0b10, 2);
        // motion_vectors(0): Frame-based, mv_count == 1, dmv == 0 →
        // 2 zero-bits (horiz `motion_code=0` + vert `motion_code=0`).
        write_zero_motion_vectors_frame_one(&mut bw);
        let buf = end_with_stop(bw);

        let ctx = SliceWalkContext::first_slice_with_picture_extension(
            22,
            0,
            PictureCodingType::Predictive,
            8,
            PictureStructure::Frame,
            false,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        let mb0 = &walk.macroblocks[0];
        assert!(mb0.macroblock_type.macroblock_motion_forward);
        assert!(!mb0.macroblock_type.macroblock_motion_backward);
        assert!(!mb0.macroblock_type.macroblock_pattern);
        assert!(!mb0.macroblock_type.macroblock_intra);
        let mt = mb0.motion_type.expect("frame_motion_type present");
        assert_eq!(mt.code, 0b10);
        assert_eq!(mt.motion_vector_count, 1);
        // dct_type stays None — `macroblock_pattern == 0 &&
        // macroblock_intra == 0` gates dct_type off regardless of
        // frame_pred_frame_dct.
        assert!(mb0.dct_type.is_none());
        // body_bit_position after 1 (increment) + 3 (mb_type) + 2
        // (motion_type) = 6 bits (the wire-position body fields
        // sit further along).
        assert_eq!(mb0.body_bit_position, 6);
        // motion_vectors(0) is captured; the wire-position cursor
        // moves 2 bits beyond body_bit_position.
        let mv = mb0
            .motion_vectors_forward
            .as_ref()
            .expect("motion_vectors(0) present");
        assert_eq!(mv.entries.len(), 1);
        assert_eq!(mv.bit_position_after, 8);
        assert!(mb0.motion_vectors_backward.is_none());
        assert!(mb0.concealment_marker_bit.is_none());
        assert!(mb0.coded_block_pattern.is_none());
        assert_eq!(mb0.pattern_code, [false; 12]);
    }

    #[test]
    fn field_motion_type_read_in_field_picture() {
        // Field picture (top field) carrying forward motion. Per
        // §6.2.5.1 a 2-bit `field_motion_type` is emitted whenever
        // a motion flag is set, regardless of `frame_pred_frame_dct`.
        // Table 6-18:
        // * `01` → Field-based, mv_count = 1
        // * `10` → 16x8 MC, mv_count = 2
        // * `11` → Dual-Prime, mv_count = 1
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_p_mc_not_coded(&mut bw);
        // field_motion_type = `10` → 16x8 MC.
        bw.write_u32(0b10, 2);
        // motion_vectors(0): 16x8 MC, mv_count == 2 → two `(vfs,
        // zero motion_vector)` pairs (1 + 2 + 1 + 2 = 6 bits).
        write_zero_motion_vectors_field_two(&mut bw);
        let buf = end_with_stop(bw);

        let ctx = SliceWalkContext::first_slice_with_picture_extension(
            22,
            0,
            PictureCodingType::Predictive,
            8,
            PictureStructure::TopField,
            true,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        let mt = walk.macroblocks[0]
            .motion_type
            .expect("field_motion_type present");
        assert_eq!(mt.code, 0b10);
        // 16x8 MC has two motion vectors.
        assert_eq!(mt.motion_vector_count, 2);
        assert!(walk.macroblocks[0].dct_type.is_none());
        let mv = walk.macroblocks[0]
            .motion_vectors_forward
            .as_ref()
            .expect("motion_vectors(0) present");
        assert_eq!(mv.entries.len(), 2);
        for entry in &mv.entries {
            assert_eq!(entry.vertical_field_select, Some(false));
        }
    }

    #[test]
    fn motion_type_omitted_when_frame_pred_frame_dct_in_frame_picture() {
        // Same P-picture MC MB but with `frame_pred_frame_dct == 1`.
        // §6.2.5.1 gate: in a frame picture with
        // `frame_pred_frame_dct == 1`, `frame_motion_type` is
        // omitted (defaulted to Frame-based at the motion-vector
        // decode site). The §6.2.5 `motion_vectors(0)` block still
        // fires because `macroblock_motion_forward == 1`; the
        // walker uses [`effective_motion_type`] to thread the
        // defaulted Frame-based / mv_count=1 / dmv=0 row in.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_p_mc_not_coded(&mut bw);
        // No frame_motion_type emitted. motion_vectors(0) with the
        // defaulted Frame-based row consumes 2 bits.
        write_zero_motion_vectors_frame_one(&mut bw);
        let buf = end_with_stop(bw);

        let ctx = SliceWalkContext::first_slice_with_picture_extension(
            22,
            0,
            PictureCodingType::Predictive,
            8,
            PictureStructure::Frame,
            true,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        assert!(walk.macroblocks[0].motion_type.is_none());
        assert!(walk.macroblocks[0].dct_type.is_none());
        // body_bit_position after 1 (increment) + 3 (mb_type) = 4
        // bits, no motion_type consumed.
        assert_eq!(walk.macroblocks[0].body_bit_position, 4);
        // motion_vectors(0) still captured via the defaulted row.
        let mv = walk.macroblocks[0]
            .motion_vectors_forward
            .as_ref()
            .expect("motion_vectors(0) emitted despite frame_pred_frame_dct == 1");
        assert_eq!(mv.entries.len(), 1);
    }

    #[test]
    fn dct_type_read_for_intra_macroblock_in_frame_picture_no_fpfd() {
        // I-picture MB ("Intra" = Table B-2 `1`) in a frame picture
        // with `frame_pred_frame_dct == 0`. §6.2.5.1: dct_type
        // present because picture_structure == Frame &&
        // !frame_pred_frame_dct && macroblock_intra. No motion
        // flag is set so motion_type stays absent.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        // dct_type = 1 → field DCT coded.
        bw.write_u32(0b1, 1);
        let buf = end_with_stop(bw);

        let ctx = SliceWalkContext::first_slice_with_picture_extension(
            22,
            0,
            PictureCodingType::Intra,
            1,
            PictureStructure::Frame,
            false,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        assert!(walk.macroblocks[0].motion_type.is_none());
        assert_eq!(walk.macroblocks[0].dct_type, Some(true));
        // body_bit_position after 1 (inc) + 1 (mb_type) + 1
        // (dct_type) = 3 bits.
        assert_eq!(walk.macroblocks[0].body_bit_position, 3);
    }

    #[test]
    fn dct_type_omitted_in_field_picture() {
        // Same I-Intra MB but in a top-field picture: §6.2.5.1
        // emits no `dct_type` because the gate requires
        // `picture_structure == Frame`. The field-picture
        // `frame_pred_frame_dct == 0` value here is also
        // ignored for this gate.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        let buf = end_with_stop(bw);

        let ctx = SliceWalkContext::first_slice_with_picture_extension(
            22,
            0,
            PictureCodingType::Intra,
            1,
            PictureStructure::BottomField,
            false,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        assert!(walk.macroblocks[0].motion_type.is_none());
        assert!(walk.macroblocks[0].dct_type.is_none());
    }

    #[test]
    fn motion_type_and_dct_type_both_read_then_quant_in_same_mb() {
        // Table B-3 "MC + Coded, Quant" = `0001 0` (5 bits), with
        // fwd=true, pattern=true, quant=true. Frame picture with
        // `frame_pred_frame_dct == 0`:
        //   bits: increment(1) + mb_type(5) + frame_motion_type(2)
        //         + dct_type(1) + q_scale(5) = 14 bits.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        // P-picture "MC, Coded, Quant" = `00010` (5 bits).
        bw.write_u32(0b00010, 5);
        // frame_motion_type = `10` (Frame-based).
        bw.write_u32(0b10, 2);
        // dct_type = 0 (frame DCT coded).
        bw.write_u32(0b0, 1);
        // quantiser_scale_code = 19.
        bw.write_u32(19, 5);
        // motion_vectors(0): Frame-based, mv_count == 1, dmv == 0 →
        // 2 bits.
        write_zero_motion_vectors_frame_one(&mut bw);
        // coded_block_pattern(): cbp = 60 (3-bit `111`).
        write_cbp_60(&mut bw);
        let buf = end_with_stop(bw);

        let ctx = SliceWalkContext::first_slice_with_picture_extension(
            22,
            0,
            PictureCodingType::Predictive,
            8,
            PictureStructure::Frame,
            false,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        let mb0 = &walk.macroblocks[0];
        assert!(mb0.macroblock_type.macroblock_quant);
        assert!(mb0.macroblock_type.macroblock_motion_forward);
        assert!(mb0.macroblock_type.macroblock_pattern);
        let mt = mb0.motion_type.expect("frame_motion_type present");
        assert_eq!(mt.code, 0b10);
        assert_eq!(mb0.dct_type, Some(false));
        // The quantiser_scale_code override applies to this MB and
        // every subsequent MB.
        assert!(mb0.macroblock_quant_present);
        assert_eq!(mb0.quantiser_scale_code, 19);
        assert_eq!(mb0.body_bit_position, 14);
        assert_eq!(walk.quantiser_scale_code, 19);
        // motion_vectors(0) (2 bits) + cbp=60 (3 bits) sit beyond
        // the body cursor.
        let mv = mb0
            .motion_vectors_forward
            .as_ref()
            .expect("motion_vectors(0) present");
        assert_eq!(mv.entries.len(), 1);
        let cbp = mb0
            .coded_block_pattern
            .as_ref()
            .expect("coded_block_pattern present");
        assert_eq!(cbp.cbp, 60);
        // Table 6-19 cbp=60 → bits set for blocks 0..4 (Y0..Y3); the
        // §6.3.17.4 derivation mirrors that for non-intra MBs.
        let mut expected = [false; 12];
        expected[0] = true;
        expected[1] = true;
        expected[2] = true;
        expected[3] = true;
        assert_eq!(mb0.pattern_code, expected);
    }

    #[test]
    fn mpeg1_shorthand_omits_macroblock_modes_tail() {
        // MPEG-1 (ISO/IEC 11172-2) has no `macroblock_modes()` tail
        // — the §2.4.2.7 macroblock layer carries its own
        // `motion_horizontal_forward_code` etc. straight after the
        // `macroblock_type`. The `first_slice_mpeg1` shorthand sets
        // `frame_pred_frame_dct == true` and Frame structure so the
        // §6.2.5.1 tail reads are gated off. (MPEG-1 motion is then
        // parsed by the §2.4.2.7 round outside this driver.)
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        let buf = end_with_stop(bw);

        let walk = walk_slice(
            &buf,
            SliceWalkContext::first_slice_mpeg1(22, 0, PictureCodingType::Intra, 1),
        )
        .unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        assert!(walk.macroblocks[0].motion_type.is_none());
        assert!(walk.macroblocks[0].dct_type.is_none());
        // body_bit_position = increment(1) + mb_type(1) = 2.
        assert_eq!(walk.macroblocks[0].body_bit_position, 2);
    }

    // ---- §6.2.5 body wire-parse coverage ----

    #[test]
    fn intra_macroblock_full_pattern_code_without_cbp() {
        // I-picture "Intra" MB — `macroblock_intra == 1`,
        // `macroblock_pattern == 0`. Per §6.3.17.4 the whole 12-entry
        // `pattern_code` is `true` (every intra block carries at
        // least a DC coefficient), with no `coded_block_pattern()`
        // emitted. No motion vectors either since there's no
        // concealment.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        let buf = end_with_stop(bw);

        let walk = walk_slice(
            &buf,
            SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 1),
        )
        .unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        let mb0 = &walk.macroblocks[0];
        assert!(mb0.motion_vectors_forward.is_none());
        assert!(mb0.motion_vectors_backward.is_none());
        assert!(mb0.concealment_marker_bit.is_none());
        assert!(mb0.coded_block_pattern.is_none());
        assert_eq!(mb0.pattern_code, [true; 12]);
    }

    #[test]
    fn intra_macroblock_with_concealment_motion_vectors_and_marker_bit() {
        // I-picture "Intra" MB in a picture with
        // `concealment_motion_vectors == 1`. §6.2.5 then requires a
        // `motion_vectors(0)` block + a `marker_bit == '1'` after
        // it, even though `macroblock_intra == 1` and
        // `macroblock_motion_forward == 0`.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        // motion_vectors(0): Frame-based default (mv_count == 1) → 2
        // zero bits.
        write_zero_motion_vectors_frame_one(&mut bw);
        // marker_bit = 1.
        bw.write_u32(0b1, 1);
        let buf = end_with_stop(bw);

        let ctx = SliceWalkContext::first_slice_with_picture_body(
            22,
            0,
            PictureCodingType::Intra,
            1,
            PictureStructure::Frame,
            true,
            1,
            1,
            1,
            1,
            true,
            ChromaFormat::Yuv420,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        let mb0 = &walk.macroblocks[0];
        assert!(mb0.motion_vectors_forward.is_some());
        assert!(mb0.motion_vectors_backward.is_none());
        assert_eq!(mb0.concealment_marker_bit, Some(true));
        // Pattern is still fully coded for intra without CBP.
        assert_eq!(mb0.pattern_code, [true; 12]);
    }

    #[test]
    fn rejects_zero_concealment_marker_bit() {
        // Same shape but the marker_bit is `'0'` — §6.3.17 violation,
        // walker must reject.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        write_zero_motion_vectors_frame_one(&mut bw);
        // marker_bit = 0 → rejected.
        bw.write_u32(0b0, 1);
        let buf = end_with_stop(bw);

        let ctx = SliceWalkContext::first_slice_with_picture_body(
            22,
            0,
            PictureCodingType::Intra,
            1,
            PictureStructure::Frame,
            true,
            1,
            1,
            1,
            1,
            true,
            ChromaFormat::Yuv420,
        );
        let err = walk_slice(&buf, ctx).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn coded_block_pattern_drives_pattern_code_derivation() {
        // P-picture "Pattern, motion forward" MB carrying motion +
        // CBP. cbp=60 → blocks 0..4 (Y0..Y3) coded.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_p_pattern_fwd(&mut bw);
        write_zero_motion_vectors_frame_one(&mut bw);
        write_cbp_60(&mut bw);
        let buf = end_with_stop(bw);

        let walk = walk_slice(
            &buf,
            SliceWalkContext::first_slice(22, 0, PictureCodingType::Predictive, 8),
        )
        .unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        let mb0 = &walk.macroblocks[0];
        assert_eq!(
            mb0.coded_block_pattern.as_ref().expect("cbp emitted").cbp,
            60
        );
        let mut expected = [false; 12];
        expected[0] = true;
        expected[1] = true;
        expected[2] = true;
        expected[3] = true;
        assert_eq!(mb0.pattern_code, expected);
    }

    #[test]
    fn coded_block_pattern_drives_pattern_code_422_extension() {
        // P-picture pattern MB with `chroma_format = Yuv422`. The
        // 2-bit `coded_block_pattern_1` extension drives blocks 6..8.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_p_pattern_fwd(&mut bw);
        write_zero_motion_vectors_frame_one(&mut bw);
        // cbp = 0 → blocks 0..6 cleared. Table B-9 row code is the
        // 9-bit `0b000000001`.
        bw.write_u32(0b0_0000_0001, 9);
        // coded_block_pattern_1 = `11` → blocks 6,7 set.
        bw.write_u32(0b11, 2);
        let buf = end_with_stop(bw);

        let ctx = SliceWalkContext::first_slice_with_picture_body(
            22,
            0,
            PictureCodingType::Predictive,
            8,
            PictureStructure::Frame,
            true,
            1,
            1,
            1,
            1,
            false,
            ChromaFormat::Yuv422,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        let mb0 = &walk.macroblocks[0];
        let cbp = mb0.coded_block_pattern.as_ref().expect("cbp emitted");
        assert_eq!(cbp.cbp, 0);
        assert_eq!(cbp.coded_block_pattern_1, Some(0b11));
        let mut expected = [false; 12];
        expected[6] = true;
        expected[7] = true;
        assert_eq!(mb0.pattern_code, expected);
    }

    #[test]
    fn b_picture_macroblock_emits_both_motion_vectors() {
        // Table B-4 row "interpolated, not coded" = `10` (2 bits) —
        // `fwd == 1, bwd == 1, pattern == 0, intra == 0`. Both
        // `motion_vectors(0)` and `motion_vectors(1)` fire.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        // B-4 "interpolated, not coded" = `10`.
        bw.write_u32(0b10, 2);
        // motion_vectors(0): Frame-based, mv_count == 1 → 2 bits.
        write_zero_motion_vectors_frame_one(&mut bw);
        // motion_vectors(1): Frame-based, mv_count == 1 → 2 bits.
        write_zero_motion_vectors_frame_one(&mut bw);
        let buf = end_with_stop(bw);

        let walk = walk_slice(
            &buf,
            SliceWalkContext::first_slice(22, 0, PictureCodingType::Bidirectional, 8),
        )
        .unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        let mb0 = &walk.macroblocks[0];
        assert!(mb0.macroblock_type.macroblock_motion_forward);
        assert!(mb0.macroblock_type.macroblock_motion_backward);
        let mv_fwd = mb0
            .motion_vectors_forward
            .as_ref()
            .expect("motion_vectors(0) emitted");
        let mv_bwd = mb0
            .motion_vectors_backward
            .as_ref()
            .expect("motion_vectors(1) emitted");
        assert_eq!(mv_fwd.kind, MotionVectorsKind::Forward);
        assert_eq!(mv_bwd.kind, MotionVectorsKind::Backward);
        assert_eq!(mv_fwd.entries.len(), 1);
        assert_eq!(mv_bwd.entries.len(), 1);
    }

    #[test]
    fn macroblock_with_no_motion_no_pattern_has_empty_pattern_code() {
        // Reach the `else` branch of the §6.3.17.4 derivation: a
        // macroblock with neither `macroblock_intra` nor
        // `macroblock_pattern`. In B-pictures Table B-4 "No MC, not
        // coded" reaches this state — but it is also the §7.6.6
        // skipped-MB case. We instead use the simpler "skip via
        // skipped-MB-as-described-in-r31" path: a B-picture MB with
        // motion (fwd-not-coded = `01`) drives the empty pattern_code.
        //
        // Table B-4 "Fwd, not coded" = `0010` → fwd=true,
        // bwd=false, pattern=false, intra=false. Pattern_code stays
        // all-false in this row.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        bw.write_u32(0b0010, 4);
        write_zero_motion_vectors_frame_one(&mut bw);
        let buf = end_with_stop(bw);

        let walk = walk_slice(
            &buf,
            SliceWalkContext::first_slice(22, 0, PictureCodingType::Bidirectional, 8),
        )
        .unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        let mb0 = &walk.macroblocks[0];
        assert!(!mb0.macroblock_type.macroblock_intra);
        assert!(!mb0.macroblock_type.macroblock_pattern);
        assert_eq!(mb0.pattern_code, [false; 12]);
        assert!(mb0.coded_block_pattern.is_none());
    }

    #[test]
    fn body_bit_position_records_post_macroblock_modes_cursor() {
        // Confirm the historical [`MacroblockRecord::body_bit_position`]
        // contract: it snapshots the cursor right after
        // `quantiser_scale_code` (or, when absent, right after
        // `macroblock_modes()`), **before** any motion_vectors() /
        // CBP wire. A round-29-era caller depending on this offset
        // to resume parsing the body itself stays unbroken.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_p_pattern_fwd(&mut bw);
        // motion_vectors(0) and CBP wire bits follow but do NOT
        // move body_bit_position.
        write_zero_motion_vectors_frame_one(&mut bw);
        write_cbp_60(&mut bw);
        let buf = end_with_stop(bw);

        let walk = walk_slice(
            &buf,
            SliceWalkContext::first_slice(22, 0, PictureCodingType::Predictive, 8),
        )
        .unwrap();
        let mb0 = &walk.macroblocks[0];
        // increment(1) + mb_type(1) = 2 bits, no quant code → body
        // cursor at bit 2.
        assert_eq!(mb0.body_bit_position, 2);
    }

    // -----------------------------------------------------------
    // §6.2.6 `block(i)` wiring (round 232 / changelog "round 34")
    // -----------------------------------------------------------

    /// Construct the §6.2.6 §6.2.5-body context for the dominant
    /// 4:2:0 / `intra_dc_precision = 0` / linear-q-scale case.
    fn block_ctx_420_default_iframe(q: u8) -> SliceWalkContext {
        SliceWalkContext::first_slice_with_block_decoding(
            22,
            0,
            PictureCodingType::Intra,
            q,
            PictureStructure::Frame,
            true,
            1,
            1,
            1,
            1,
            false,
            ChromaFormat::Yuv420,
            false, // intra_vlc_format
            false, // alternate_scan
            0,     // intra_dc_precision
            false, // q_scale_type
        )
    }

    #[test]
    fn block_decoding_off_keeps_round_33_contract_decoded_blocks_none() {
        // The round-30..33 contract: when `block_decoding_enabled
        // == false` (every existing constructor), the walker does
        // NOT advance past `coded_block_pattern()` into §6.2.6 and
        // `decoded_blocks` is always `None`.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        let buf = end_with_stop(bw);

        let walk = walk_slice(
            &buf,
            SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 14),
        )
        .unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        assert!(walk.macroblocks[0].decoded_blocks.is_none());
    }

    #[test]
    fn block_decoding_on_dc_only_intra_macroblock_emits_six_decoded_blocks() {
        // §6.2.6 `block(i)` driver wired into the walker. The MB is
        // a 4:2:0 bare-intra (no `macroblock_pattern`); §6.3.17.4
        // says every block is coded. Each of the six §6.1.1.8
        // blocks (4 Y, 1 Cb, 1 Cr) carries `dct_dc_size == 0` +
        // immediate EOB — the smallest legal intra block.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        write_dc_zero_intra_macroblock_420(&mut bw);
        let buf = end_with_stop(bw);

        let walk = walk_slice(&buf, block_ctx_420_default_iframe(14)).unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        let mb0 = &walk.macroblocks[0];
        let blocks = mb0.decoded_blocks.as_ref().expect("§6.2.6 ran");
        assert_eq!(blocks.len(), 6);
        // §6.1.1.8: block 0..=3 are Y, 4 is Cb, 5 is Cr.
        use crate::mpeg2_block_dc::ColourComponent as CC;
        assert_eq!(blocks[0].component, CC::Y);
        assert_eq!(blocks[1].component, CC::Y);
        assert_eq!(blocks[2].component, CC::Y);
        assert_eq!(blocks[3].component, CC::Y);
        assert_eq!(blocks[4].component, CC::Cb);
        assert_eq!(blocks[5].component, CC::Cr);
        // Every QFS slot above [0] is zero (DC-only block).
        for b in blocks {
            for i in 1..b.decoded.qfs.len() {
                assert_eq!(b.decoded.qfs[i], 0);
            }
        }
        // The cursor advanced past the §6.2.6 wire bits: increment
        // (1) + mb_type (1) + 4 luma blocks (4*5=20) + 2 chroma
        // blocks (2*4=8) = 30 bits into the buffer.
        assert_eq!(blocks[5].decoded.end_of_block_bit_position, 30);
    }

    #[test]
    fn block_decoding_on_dc_predictor_advances_per_intra_block() {
        // Two intra MBs in a row, both DC-only with size 0 → both
        // produce QFS[0] = predictor (no differential). Across two
        // intra MBs the predictor for Y is fed forward, so MB1's Y
        // DC equals MB0's Y DC. Both equal the Table 7-2 reset
        // value 128 because the first intra MB's predictor is the
        // slice-start reset (no preceding `dct_diff != 0`).
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        write_dc_zero_intra_macroblock_420(&mut bw);
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        write_dc_zero_intra_macroblock_420(&mut bw);
        let buf = end_with_stop(bw);

        let walk = walk_slice(&buf, block_ctx_420_default_iframe(14)).unwrap();
        assert_eq!(walk.macroblocks.len(), 2);
        let mb0 = walk.macroblocks[0].decoded_blocks.as_ref().unwrap();
        let mb1 = walk.macroblocks[1].decoded_blocks.as_ref().unwrap();
        // Y DC for MB0 block 0 — should be the predictor reset
        // value: with `intra_dc_precision == 0` Table 7-2 gives
        // 128.
        assert_eq!(mb0[0].decoded.qfs[0], 128);
        // Y DC for MB1 block 0 — predictor carried forward
        // unchanged because every MB0 block had `dct_diff == 0`.
        assert_eq!(mb1[0].decoded.qfs[0], 128);
    }

    #[test]
    fn block_decoding_rejects_intra_dc_precision_out_of_range() {
        // Pre-flight validation: `intra_dc_precision = 4` is
        // outside Table 6-13's `0..=3` and the §7.2.1 DC
        // predictor allocation must reject it before the loop
        // ever runs (i.e. the error must surface even when the
        // bitstream contains zero macroblocks).
        let buf = vec![0x00, 0x00, 0x00, 0x01, 0xB7];
        let ctx = SliceWalkContext::first_slice_with_block_decoding(
            22,
            0,
            PictureCodingType::Intra,
            14,
            PictureStructure::Frame,
            true,
            1,
            1,
            1,
            1,
            false,
            ChromaFormat::Yuv420,
            false,
            false,
            4, // intra_dc_precision out of range
            false,
        );
        let err = walk_slice(&buf, ctx).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn block_decoding_constructor_signals_enabled_flag() {
        // Sanity check on the constructor itself: every other
        // constructor leaves `block_decoding_enabled == false`,
        // and the §6.2.6 constructor flips it.
        let off = SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 14);
        assert!(!off.block_decoding_enabled);
        let on = block_ctx_420_default_iframe(14);
        assert!(on.block_decoding_enabled);
    }

    #[test]
    fn block_decoding_q_scale_type_drives_table_7_6_lookup() {
        // §7.4.2.2 Table 7-6 mapping: `quantiser_scale_code = 1`
        // is `2` on the linear column (`q_scale_type == 0`) and
        // `1` on the non-linear column (`q_scale_type == 1`). The
        // walker's resolved `quantiser_scale_value` flows into the
        // per-block context. We sanity-check the linear case via a
        // walk-then-inspect: the value isn't surfaced directly on
        // `MacroblockRecord`, but the walk succeeding (with the
        // DC-only block above) confirms the lookup yielded a legal
        // value. The non-linear column is also walkable.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        write_dc_zero_intra_macroblock_420(&mut bw);
        let buf = end_with_stop(bw);

        // Linear column (q_scale_type = false), code = 1.
        let linear = SliceWalkContext::first_slice_with_block_decoding(
            22,
            0,
            PictureCodingType::Intra,
            1,
            PictureStructure::Frame,
            true,
            1,
            1,
            1,
            1,
            false,
            ChromaFormat::Yuv420,
            false,
            false,
            0,
            false,
        );
        walk_slice(&buf, linear).unwrap();
        // Non-linear column (q_scale_type = true), code = 1.
        let nonlinear = SliceWalkContext::first_slice_with_block_decoding(
            22,
            0,
            PictureCodingType::Intra,
            1,
            PictureStructure::Frame,
            true,
            1,
            1,
            1,
            1,
            false,
            ChromaFormat::Yuv420,
            false,
            false,
            0,
            true,
        );
        walk_slice(&buf, nonlinear).unwrap();
    }

    // ---- §7.6.3.1 wire-to-reconstruction wiring ----

    #[test]
    fn reconstruct_record_zero_vector_leaves_pmv_at_zero() {
        // P-picture MB with a zero forward motion vector (motion_code
        // h/v = 0, f_code = 1). §7.6.3.1 delta = 0 in both components,
        // prior PMV = 0, vector' = 0, PMV stays 0.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_p_mc_not_coded(&mut bw);
        // frame_motion_type = `10` (Frame-based), mv_count = 1.
        bw.write_u32(0b10, 2);
        write_zero_motion_vectors_frame_one(&mut bw);
        let buf = end_with_stop(bw);

        let ctx = SliceWalkContext::first_slice_with_picture_extension(
            22,
            0,
            PictureCodingType::Predictive,
            8,
            PictureStructure::Frame,
            false,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 1);

        let mut pmv = Pmv::new();
        let recon = reconstruct_record_motion_vectors(&walk.macroblocks[0], &mut pmv, &ctx)
            .expect("§7.6.3.1");
        let fwd = recon.forward.expect("forward present");
        assert_eq!(fwd.len(), 1);
        assert_eq!(fwd[0].horizontal.vector_prime, 0);
        assert_eq!(fwd[0].horizontal.new_pmv, 0);
        assert_eq!(fwd[0].vertical.vector_prime, 0);
        assert_eq!(fwd[0].vertical.new_pmv, 0);
        assert!(recon.backward.is_none());
        // PMV state — every slot still zero.
        for r in [VectorIndex::First, VectorIndex::Second] {
            for s in [Direction::Forward, Direction::Backward] {
                for t in [
                    crate::pmv::Component::Horizontal,
                    crate::pmv::Component::Vertical,
                ] {
                    assert_eq!(pmv.get(r, s, t), 0);
                }
            }
        }
    }

    #[test]
    fn reconstruct_record_threads_pmv_across_two_macroblocks() {
        // Two-MB slice: each MB carries a forward motion vector with
        // motion_code horiz = +1 (3-bit Table B-10 code `010`),
        // motion_code vert = 0 (1-bit `1`). f_code = 1 throughout, so
        // each call leaves delta = +1 in the horizontal. The first
        // MB's PMV starts at 0 → vector' = 1 → PMV becomes 1. The
        // second MB picks up PMV = 1 → vector' = 1 + 1 = 2 → PMV
        // becomes 2. Confirms §7.6.3.1 PMV accumulation across MBs.
        let mut bw = BitWriter::new();
        // MB 0: increment 1, "MC, not coded", frame_motion_type=10,
        // mv h=+1 (3 bits `010`), mv v=0 (1 bit `1`).
        write_address_increment(&mut bw, 1);
        write_mb_type_p_mc_not_coded(&mut bw);
        bw.write_u32(0b10, 2);
        bw.write_u32(0b010, 3);
        bw.write_u32(0b1, 1);
        // MB 1: same shape.
        write_address_increment(&mut bw, 1);
        write_mb_type_p_mc_not_coded(&mut bw);
        bw.write_u32(0b10, 2);
        bw.write_u32(0b010, 3);
        bw.write_u32(0b1, 1);
        let buf = end_with_stop(bw);

        let ctx = SliceWalkContext::first_slice_with_picture_extension(
            22,
            0,
            PictureCodingType::Predictive,
            8,
            PictureStructure::Frame,
            false,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 2);

        let mut pmv = Pmv::new();
        let recon0 = reconstruct_record_motion_vectors(&walk.macroblocks[0], &mut pmv, &ctx)
            .expect("§7.6.3.1 MB0");
        let f0 = recon0.forward.unwrap();
        assert_eq!(f0[0].horizontal.vector_prime, 1);
        assert_eq!(f0[0].horizontal.new_pmv, 1);
        assert_eq!(f0[0].vertical.vector_prime, 0);

        let recon1 = reconstruct_record_motion_vectors(&walk.macroblocks[1], &mut pmv, &ctx)
            .expect("§7.6.3.1 MB1");
        let f1 = recon1.forward.unwrap();
        assert_eq!(f1[0].horizontal.vector_prime, 2);
        assert_eq!(f1[0].horizontal.new_pmv, 2);
        // Vertical PMV stays 0; vert motion_code was 0 every MB.
        assert_eq!(f1[0].vertical.vector_prime, 0);

        // Final PMV state: [First][Forward][Horizontal] = 2.
        assert_eq!(
            pmv.get(
                VectorIndex::First,
                Direction::Forward,
                crate::pmv::Component::Horizontal,
            ),
            2
        );
    }

    #[test]
    fn reconstruct_record_handles_absent_motion_vectors() {
        // Intra MB without concealment_motion_vectors: no
        // motion_vectors() consumed, so both forward / backward come
        // back `None` from the reconstruction helper. PMV unchanged.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        let buf = end_with_stop(bw);

        let ctx = SliceWalkContext::first_slice_with_picture_extension(
            22,
            0,
            PictureCodingType::Intra,
            8,
            PictureStructure::Frame,
            true,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 1);

        let mut pmv = Pmv::new();
        // Seed a non-zero PMV value so we can assert the helper
        // didn't touch it.
        pmv.set(
            VectorIndex::First,
            Direction::Forward,
            crate::pmv::Component::Horizontal,
            42,
        );
        let recon = reconstruct_record_motion_vectors(&walk.macroblocks[0], &mut pmv, &ctx)
            .expect("§7.6.3.1 no-op");
        assert!(recon.forward.is_none());
        assert!(recon.backward.is_none());
        assert_eq!(
            pmv.get(
                VectorIndex::First,
                Direction::Forward,
                crate::pmv::Component::Horizontal,
            ),
            42
        );
    }

    #[test]
    fn reconstruct_record_modulo_wraps_when_delta_pushes_past_range() {
        // P-picture MB with motion_code = -3 in the horizontal. With
        // f_code = 1 (range [-16, 15]) and a prior PMV of -15, the
        // §7.6.3.1 raw sum is -18, which wraps +32 to +14.
        //
        // motion_code = -3 → Table B-10 row `0001 1` (5 bits).
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_p_mc_not_coded(&mut bw);
        bw.write_u32(0b10, 2); // frame_motion_type = Frame-based
                               // motion_code = -3 → Table B-10 row `0001 1` (5 bits, value
                               // 0x03). Spelt as a plain integer to dodge the 4-bit clippy
                               // byte-grouping lint without an allow blanket.
        bw.write_u32(0x03, 5);
        bw.write_u32(0b1, 1); // motion_code vert = 0
        let buf = end_with_stop(bw);

        let ctx = SliceWalkContext::first_slice_with_picture_extension(
            22,
            0,
            PictureCodingType::Predictive,
            8,
            PictureStructure::Frame,
            false,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 1);

        let mut pmv = Pmv::new();
        pmv.set(
            VectorIndex::First,
            Direction::Forward,
            crate::pmv::Component::Horizontal,
            -15,
        );
        let recon = reconstruct_record_motion_vectors(&walk.macroblocks[0], &mut pmv, &ctx)
            .expect("§7.6.3.1");
        let f = recon.forward.unwrap();
        assert_eq!(f[0].horizontal.delta, -3);
        assert_eq!(f[0].horizontal.vector_prime, 14);
        assert_eq!(f[0].horizontal.new_pmv, 14);
    }

    #[test]
    fn block_decoding_decoded_blocks_omitted_for_records_with_no_coded_blocks() {
        // Wire-only round-30 contract: a non-intra MB whose
        // `macroblock_pattern == 0` carries no coded blocks at
        // all. With block decoding ON the walker still emits a
        // record but `decoded_blocks` is `Some(empty)` because
        // every `pattern_code[i]` is `false`.
        //
        // Build a P-picture MB with Table B-3 row "MC, not
        // coded" (`001`, 3 bits): macroblock_motion_forward == 1,
        // macroblock_pattern == 0, macroblock_intra == 0.
        // motion_vectors(0) follows (we use the zero-vector form
        // with f_code == 1). No CBP, no blocks.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        // Table B-3 "MC, not coded" = `001` (3 bits).
        bw.write_u32(0b001, 3);
        write_zero_motion_vectors_frame_one(&mut bw);
        let buf = end_with_stop(bw);

        let ctx = SliceWalkContext::first_slice_with_block_decoding(
            22,
            0,
            PictureCodingType::Predictive,
            8,
            PictureStructure::Frame,
            true,
            1,
            1,
            1,
            1,
            false,
            ChromaFormat::Yuv420,
            false,
            false,
            0,
            false,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 1);
        let blocks = walk.macroblocks[0]
            .decoded_blocks
            .as_ref()
            .expect("§6.2.6 ran");
        assert!(blocks.is_empty());
    }

    /// Build the two-MB "+1 horizontal forward MV per MB" P-picture
    /// slice used by the slice-level reconstruction tests, with the
    /// first macroblock's `macroblock_address_increment` set to
    /// `first_increment` (so a caller can inject a skipped-MB run before
    /// the second coded macroblock by passing `2`).
    fn build_two_mb_forward_slice(second_increment: u16) -> Vec<u8> {
        let mut bw = BitWriter::new();
        // MB 0: increment 1, "MC, not coded", frame_motion_type=10,
        // mv h=+1 (3 bits `010`), mv v=0 (1 bit `1`).
        write_address_increment(&mut bw, 1);
        write_mb_type_p_mc_not_coded(&mut bw);
        bw.write_u32(0b10, 2);
        bw.write_u32(0b010, 3);
        bw.write_u32(0b1, 1);
        // MB 1: same shape, but its address increment may skip MBs.
        write_address_increment(&mut bw, second_increment);
        write_mb_type_p_mc_not_coded(&mut bw);
        bw.write_u32(0b10, 2);
        bw.write_u32(0b010, 3);
        bw.write_u32(0b1, 1);
        end_with_stop(bw)
    }

    #[test]
    fn slice_driver_threads_pmv_and_runs_update_across_macroblocks() {
        // The slice-level driver should (a) reset PMV at slice start
        // per §7.6.3.4, (b) reconstruct each MB's vectors against the
        // running predictor bank per §7.6.3.1 — accumulating +1 → +2
        // across the two MBs — and (c) apply the §7.6.3.3 update row
        // after each MB. Both MBs are forward-only frame-based
        // non-intra, so the Tables 7-10 row is "copy forward".
        let buf = build_two_mb_forward_slice(1);
        let ctx = SliceWalkContext::first_slice_with_picture_extension(
            22,
            0,
            PictureCodingType::Predictive,
            8,
            PictureStructure::Frame,
            false,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 2);

        let motion = reconstruct_slice_motion_vectors(&walk, &ctx).expect("§7.6.3 slice driver");
        assert_eq!(motion.records.len(), 2);

        // MB0: PMV started at 0 → vector' = 1.
        let r0 = &motion.records[0];
        assert_eq!(r0.skipped_before, 0);
        assert!(!r0.skipped_reset_pmv);
        let f0 = r0.reconstructed.forward.as_ref().expect("MB0 forward");
        assert_eq!(f0[0].horizontal.vector_prime, 1);
        assert_eq!(
            r0.update_outcome,
            Some(PmvUpdateOutcome::NonIntraCopyForward)
        );

        // MB1: PMV carried 1 → vector' = 2 (accumulation across MBs).
        let r1 = &motion.records[1];
        assert_eq!(r1.skipped_before, 0);
        let f1 = r1.reconstructed.forward.as_ref().expect("MB1 forward");
        assert_eq!(f1[0].horizontal.vector_prime, 2);
        assert_eq!(
            r1.update_outcome,
            Some(PmvUpdateOutcome::NonIntraCopyForward)
        );

        // The running PMV ends at the last reconstructed horizontal.
        assert_eq!(
            motion.pmv.get(
                VectorIndex::First,
                Direction::Forward,
                crate::pmv::Component::Horizontal,
            ),
            2
        );
    }

    #[test]
    fn slice_driver_resets_pmv_at_slice_start() {
        // §7.6.3.4: each call starts with a zeroed predictor bank, so
        // MB0 of the slice always reconstructs against PMV = 0
        // regardless of any state a prior slice left behind. We confirm
        // by decoding the same slice twice and getting identical MB0
        // vectors.
        let buf = build_two_mb_forward_slice(1);
        let ctx = SliceWalkContext::first_slice_with_picture_extension(
            22,
            0,
            PictureCodingType::Predictive,
            8,
            PictureStructure::Frame,
            false,
        );
        let walk = walk_slice(&buf, ctx).unwrap();

        let first = reconstruct_slice_motion_vectors(&walk, &ctx).expect("first pass");
        let second = reconstruct_slice_motion_vectors(&walk, &ctx).expect("second pass");
        assert_eq!(first.records, second.records);
        assert_eq!(
            first.records[0].reconstructed.forward.as_ref().unwrap()[0]
                .horizontal
                .vector_prime,
            1
        );
    }

    #[test]
    fn slice_driver_skipped_macroblock_resets_pmv_in_p_picture() {
        // MB1 carries `macroblock_address_increment = 2`, so one
        // macroblock is skipped between MB0 and MB1. In a P-picture the
        // §7.6.6 skipped macroblock resets the predictor bank
        // (§7.6.3.4), so MB1 reconstructs against PMV = 0 → vector' = 1
        // again, NOT the +2 accumulation seen without a skip.
        let buf = build_two_mb_forward_slice(2);
        let ctx = SliceWalkContext::first_slice_with_picture_extension(
            22,
            0,
            PictureCodingType::Predictive,
            8,
            PictureStructure::Frame,
            false,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 2);
        assert_eq!(walk.macroblocks[1].skipped_macroblock_count, 1);

        let motion = reconstruct_slice_motion_vectors(&walk, &ctx).expect("§7.6.3 slice driver");
        let r1 = &motion.records[1];
        assert_eq!(r1.skipped_before, 1);
        assert!(r1.skipped_reset_pmv, "P-picture skip resets PMV");
        let f1 = r1.reconstructed.forward.as_ref().expect("MB1 forward");
        assert_eq!(
            f1[0].horizontal.vector_prime, 1,
            "skip-reset PMV → no accumulation"
        );
    }

    #[test]
    fn slice_driver_rejects_skip_in_non_scalable_i_picture() {
        // §7.6.6 preamble: skipped macroblocks are forbidden in a
        // non-scalable I-picture. A second MB with increment 2 in an
        // I-picture must surface an InvalidBitstream from the driver.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_i_intra(&mut bw);
        write_address_increment(&mut bw, 2);
        write_mb_type_i_intra(&mut bw);
        let buf = end_with_stop(bw);

        let ctx = SliceWalkContext::first_slice_with_picture_extension(
            22,
            0,
            PictureCodingType::Intra,
            8,
            PictureStructure::Frame,
            true,
        );
        let walk = walk_slice(&buf, ctx).unwrap();
        assert_eq!(walk.macroblocks.len(), 2);
        let err = reconstruct_slice_motion_vectors(&walk, &ctx).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }
}
