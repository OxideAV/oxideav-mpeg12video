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

use crate::macroblock_modes::{MacroblockModesContext, MacroblockModesTail, MotionType};
use crate::macroblock_type::MacroblockType;
use crate::mb_address_increment::{MbAddressIncrement, MbAddressIncrementContext};
use crate::picture_header::{PictureCodingType, PictureStructure};
use crate::quantizer_scale::{QUANTIZER_SCALE_MAX, QUANTIZER_SCALE_MIN};
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
        }
    }
}

/// Per-macroblock summary the walker emits for one iteration of the
/// §6.2.4 do-while loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
    /// macroblock-header chain — the entry point for the
    /// macroblock_modes-tail / motion_vectors() / coded_block_pattern()
    /// / block(i) parsers that future rounds will plug in.
    pub body_bit_position: u64,
    /// Skipped-macroblock count derived from
    /// `address_increment - 1`: the number of macroblocks at addresses
    /// `previous_macroblock_address + 1 .. macroblock_address - 1`
    /// that the §7.6.6 round must reconstruct from the previous
    /// macroblock's state.
    pub skipped_macroblock_count: u32,
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
    let mut br = BitReader::new(buf);
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
        // If the buffer is too short to peek 23 bits we treat that
        // as a successful slice end (the caller bounded the
        // buffer — the next-start-code itself may live in a parent
        // buffer).
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
                end_bit_position = br.bit_position();
                break;
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
            body_bit_position: br.bit_position(),
            skipped_macroblock_count,
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
        // 2 MBs, then another fwd-pattern MB.
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_p_pattern_fwd(&mut bw);
        write_address_increment(&mut bw, 3);
        write_mb_type_p_pattern_fwd(&mut bw);
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
        // (motion_type) = 6 bits.
        assert_eq!(mb0.body_bit_position, 6);
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
    }

    #[test]
    fn motion_type_omitted_when_frame_pred_frame_dct_in_frame_picture() {
        // Same P-picture MC MB but with `frame_pred_frame_dct == 1`.
        // §6.2.5.1 gate: in a frame picture with
        // `frame_pred_frame_dct == 1`, `frame_motion_type` is
        // omitted (defaulted to Frame-based at the motion-vector
        // decode site).
        let mut bw = BitWriter::new();
        write_address_increment(&mut bw, 1);
        write_mb_type_p_mc_not_coded(&mut bw);
        // No frame_motion_type emitted — the cursor should sit at
        // bit 4 after the 3-bit mb_type.
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
}
