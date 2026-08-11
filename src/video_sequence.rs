//! §6.2.2 `video_sequence()` top-level decode loop with §6.1.1.11
//! display-order frame reordering, per ISO/IEC 13818-2 (ITU-T H.262) /
//! ISO/IEC 11172-2 (MPEG-1).
//!
//! ## Where this sits
//!
//! Every per-picture reconstruction driver already exists:
//! [`crate::decode_intra_picture`] reconstructs an **I** picture,
//! [`crate::decode_inter_picture`] a frame-picture **P / B**, and
//! [`crate::decode_field_picture`] a field picture. Each takes a slice
//! region plus the already-decoded reference frame(s) and returns a
//! [`FrameBuffer`]. What was missing — and is the crate's top open gap —
//! is the loop **above** them: the §6.2.2 `video_sequence()` walker that
//!
//! 1. parses the §6.2.2.1 `sequence_header()` + §6.2.2.3
//!    `sequence_extension()` for the frame geometry
//!    ([`crate::Mpeg2Sequence`]),
//! 2. scans each §6.2.3 `picture_header()` + §6.2.3.1
//!    `picture_coding_extension()` for the `picture_coding_type`,
//!    `temporal_reference` and the f_codes / DCT-context flags,
//! 3. dispatches the picture's slice region to the matching per-picture
//!    driver — **I** → [`crate::decode_intra_picture`], **P / B** →
//!    [`crate::decode_inter_picture`] — supplying the reference frame(s)
//!    from the running anchor pair, and
//! 4. **reorders** the reconstructed frames from coded order into
//!    display order per §6.1.1.11, so the caller receives them in the
//!    order they are shown, not the order they are coded.
//!
//! ## §6.1.1.11 frame reordering
//!
//! The coded order (the order pictures appear in the bitstream, which is
//! the order the decoder reconstructs them) is **not** the display
//! order whenever B-frames are present. The spec rule, transcribed:
//!
//! * If the current coded frame is a **B**-frame, the output frame is
//!   the frame just reconstructed from that B-frame — B-frames are never
//!   delayed, they pass straight through.
//! * If the current coded frame is an **I**- or **P**-frame, the output
//!   frame is the *previously* reconstructed I- or P-frame (the held-back
//!   anchor) if one exists; if none exists (start of sequence) no frame
//!   is output yet. The newly reconstructed I/P frame becomes the new
//!   held-back anchor.
//! * At the end of the sequence the final held-back I/P anchor is
//!   flushed (output last).
//!
//! Worked example from §6.1.1.11 — coded order
//! `1I 4P 2B 3B 7P 5B 6B …` reorders to display order
//! `1I 2B 3B 4P 5B 6B …`. The driver implements exactly that mapping by
//! holding one I/P anchor back by one and emitting it when the next
//! anchor arrives (or at end of stream).
//!
//! ## Reference management (§7.6)
//!
//! A **P**-frame predicts from the most recently decoded I/P frame (the
//! forward anchor). A **B**-frame predicts from the two most recently
//! decoded I/P frames — `forward` is the older anchor (past), `backward`
//! the newer (future). Because a B-frame is coded *after* both of its
//! anchors, both are already reconstructed when the B-frame is decoded,
//! so no look-ahead is needed: the driver keeps the two latest anchors
//! (`forward_anchor`, the previous I/P; `backward_anchor`, the latest
//! I/P) and feeds them to [`crate::decode_inter_picture`].
//!
//! ## Field pictures (§6.1.1.4.1)
//!
//! When an interlaced sequence is coded as **field pictures** the loop
//! handles them too: each field picture (`picture_structure ==
//! TopField` / `BottomField`) is reconstructed into a field-height
//! buffer by [`crate::decode_field_picture`], the first field of a pair
//! is held until its partner arrives, and the two are interleaved into
//! one full-height reconstructed frame
//! ([`crate::assemble_frame_from_fields`], §3.131 top→even lines / §3.13
//! bottom→odd lines). The assembled frame then flows through the same
//! §6.1.1.11 reorder + §7.6 anchor rotation as a frame picture. The
//! §7.6.2.1 second-field-of-a-P-frame reference rule (the most-recent
//! reference field is this frame's just-decoded first field) is honoured
//! by building a synthetic reference frame that pairs the current first
//! field with the previous frame's opposite-parity field.
//!
//! ## Scope
//!
//! The §6.2.3 extension family beyond `sequence_extension()` /
//! `picture_coding_extension()` (`quant_matrix_extension()`, the display
//! / scalable extensions) is skipped over by the start-code scan;
//! downloadable quantiser matrices and the scalable layers are threaded
//! by a later round. MPEG-1 streams (no `sequence_extension()` /
//! `picture_coding_extension()`) are a later milestone too — this driver
//! requires the MPEG-2 extensions for the geometry and f_codes.

use crate::frame_assembly::{
    assemble_frame_from_fields, decode_intra_picture_with_matrices, FrameBuffer, IntraPictureParams,
};
use crate::inter_reconstruction::ReferenceFrames;
use crate::mpeg1_picture::{
    decode_mpeg1_d_picture, decode_mpeg1_inter_picture, decode_mpeg1_intra_picture,
    Mpeg1InterParams, Mpeg1PictureParams,
};
use crate::picture_header::{
    Mpeg2PictureHeader, PictureCodingExtension, PictureCodingType, PictureStructure,
    PICTURE_START_CODE,
};
use crate::picture_reconstruction::{
    decode_field_picture_with_matrices, decode_inter_picture_with_matrices, PicturePredictionParams,
};
use crate::quant_matrix_extension::{QuantMatrixExtension, QuantiserMatrixState};
use crate::sequence_extension::Mpeg2Sequence;
use crate::sequence_header::SEQUENCE_HEADER_CODE;
use crate::{Error, Result};

/// A reconstructed frame paired with its §6.3.10 `temporal_reference` —
/// the display index (modulo 1024) the §6.1.1.11 reorder is defined
/// against.
#[derive(Debug, Clone)]
pub struct DecodedFrame {
    /// The reconstructed picture in display form.
    pub frame: FrameBuffer,
    /// The §6.3.10 `temporal_reference` (0..=1023) of the coded picture.
    /// In display order these increment by one (modulo 1024) within a
    /// GOP; the field is surfaced so a caller can verify the ordering or
    /// map to a presentation timestamp.
    pub temporal_reference: u16,
    /// The §6.3.10 `picture_coding_type` of the coded picture (I / P / B).
    pub picture_coding_type: PictureCodingType,
}

/// Decode a whole MPEG-2 elementary stream into a `Vec` of reconstructed
/// frames **in display order** (§6.1.1.11).
///
/// `stream` is the elementary stream from the leading
/// `sequence_header_code` (`0x000001B3`) onward. The driver parses the
/// sequence layer once for the geometry, then walks every
/// `picture_start_code` (`0x00000100`), reconstructs each picture with
/// the matching per-picture driver, and reorders the results into
/// display order.
///
/// Returns the frames in the order they are displayed: a leading I/P
/// frame, any B-frames coded between it and the next anchor (emitted
/// before the anchor), then the anchor, and so on, with the final anchor
/// flushed at end of stream.
///
/// # Errors
///
/// * [`Error::InvalidBitstream`] if the stream does not begin with a
///   `sequence_header()` / `sequence_extension()` pair, if a P/B picture
///   appears before any anchor exists (no forward reference, §6.1.1.11
///   *"the first coded frame after a sequence header shall not be a
///   B-frame"* / a P needs a forward anchor), or from any lower-layer
///   parse.
/// * [`Error::ShortHeader`] on truncation.
pub fn decode_video_sequence(stream: &[u8]) -> Result<Vec<DecodedFrame>> {
    // The geometry is established by the leading sequence layer and
    // re-established at every repeat / new `sequence_header()` encountered
    // before a picture, so a multi-sequence stream whose geometry changes
    // mid-stream tracks the new sizes (§6.1.1.6: a repeat sequence header
    // may legally restate the parameters).
    let mut geometry = parse_leading_sequence(stream)?;

    // §6.3.11 weighting matrices: reset to the §6.3.7 defaults at
    // every sequence_header_code, then overwritten by the header's
    // own load flags and by any `quant_matrix_extension()` between
    // pictures. MPEG-1 carries its matrices inside
    // `Mpeg1PictureParams` instead (§2.4.2.3, no extension exists).
    let mut matrices = geometry.initial_matrices();

    let mut reorder = ReorderBuffer::new();
    let mut output: Vec<DecodedFrame> = Vec::new();

    // The two running I/P anchors, newest-last. `forward_anchor` is the
    // older of the pair (the past reference); `backward_anchor` is the
    // newer (the future reference for B-frames, and the forward
    // reference for the next P-frame).
    let mut forward_anchor: Option<FrameBuffer> = None;
    let mut backward_anchor: Option<FrameBuffer> = None;

    // §6.1.1.4.1: field pictures occur in pairs (one top + one bottom)
    // that together constitute a coded frame, encoded in output order.
    // The first field of a pair is held here until its partner arrives,
    // when the two are interleaved into one reconstructed frame
    // ([`assemble_frame_from_fields`]).
    let mut pending_field: Option<PendingField> = None;

    let mut offset = 0usize;
    while let Some(rel) = find_picture_or_sequence_start_code(&stream[offset..]) {
        let code_start = offset + rel;

        // A `sequence_header()` before the next picture re-establishes the
        // geometry (§6.1.1.6). The reorder buffer and anchors carry across
        // — §6.1.1.11 reordering is defined over the whole reconstructed
        // sequence, and a repeat sequence header does not by itself force a
        // flush (the next coded frame, which §6.1.1.11 requires not to be a
        // B-frame, is an I/P anchor that flushes the held one in the normal
        // way).
        if is_start_code(&stream[code_start..], SEQUENCE_HEADER_CODE) {
            geometry = sequence_geometry_at(&stream[code_start..])?;
            // §6.3.11: "When a sequence_header_code is decoded all
            // matrices shall be reset to their default values" — then
            // the header's own load flags apply.
            matrices = geometry.initial_matrices();
            // Advance past this sequence header's start code; the next scan
            // finds the sequence_extension / picture that follows.
            offset = code_start + 4;
            continue;
        }

        let pic_start = code_start;

        // The picture spans from its picture_start_code up to the next
        // picture / GOP / sequence / sequence-end start code. The
        // boundary start code's own bytes are **included** in the region:
        // the per-picture slice walkers peek for the §5.2.3 23-zero
        // start-code prefix to detect end-of-slice, so the buffer must
        // carry that prefix or the last slice is truncated one macroblock
        // early. The next iteration still re-anchors on the boundary
        // start code (`next_offset`), so including its bytes here only
        // lends the walker its terminator — the drivers stop their own
        // slice scan at the first non-slice start code regardless.
        let boundary = find_next_picture_boundary(&stream[pic_start + 4..]).map(|p| p + 4);
        let region_end = match boundary {
            // Include the 4-byte boundary start code in the slice buffer.
            Some(b) => (b + 4).min(stream.len() - pic_start),
            None => stream.len() - pic_start,
        };
        let next_offset = match boundary {
            Some(b) => pic_start + b,
            None => stream.len(),
        };
        let picture_region = &stream[pic_start..pic_start + region_end];

        let coded = match geometry {
            SequenceGeometry::Mpeg1(mpeg1_params) => {
                // ISO/IEC 11172-2: frame pictures only, no
                // picture_coding_extension — the motion-vector context
                // (f_codes + full_pel flags) lives in the picture
                // header itself (§2.4.3.4).
                let header = Mpeg2PictureHeader::parse(picture_region)?;
                reconstruct_mpeg1_picture(
                    picture_region,
                    &header,
                    &mpeg1_params,
                    forward_anchor.as_ref(),
                    backward_anchor.as_ref(),
                )?
            }
            SequenceGeometry::Mpeg2(mpeg2_geometry, _) => {
                let (header, ext) = Mpeg2PictureHeader::parse_with_extension(picture_region)?;

                // §6.2.3.7 extension_and_user_data(2): any
                // `quant_matrix_extension()` between the
                // picture_coding_extension and the first slice updates
                // the running §6.3.11 matrices (which then persist for
                // the following pictures until the next
                // sequence_header_code reset).
                apply_quant_matrix_extensions(
                    picture_region,
                    mpeg2_geometry.chroma_format,
                    &mut matrices,
                )?;

                // §6.1.1.4.1 field-picture pair: decode the field, hold the
                // first of a pair until its partner arrives, then interleave
                // the pair into one reconstructed frame and route that frame
                // exactly as a frame picture would be.
                if ext.picture_structure != PictureStructure::Frame {
                    match reconstruct_field_pair(
                        picture_region,
                        &header,
                        &ext,
                        mpeg2_geometry,
                        forward_anchor.as_ref(),
                        backward_anchor.as_ref(),
                        &mut pending_field,
                        &matrices,
                    )? {
                        // First field of a pair: held back, no frame yet.
                        None => {
                            offset = next_offset;
                            continue;
                        }
                        // Second field completed the pair into a frame.
                        Some(frame) => frame,
                    }
                } else {
                    // The slice region begins after the picture header +
                    // extensions: the per-picture drivers find the first
                    // slice_start_code themselves, so we hand them the whole
                    // picture region.
                    reconstruct_picture(
                        picture_region,
                        &header,
                        &ext,
                        mpeg2_geometry,
                        forward_anchor.as_ref(),
                        backward_anchor.as_ref(),
                        &matrices,
                    )?
                }
            }
        };

        // §6.1.1.11 reorder + §7.6 reference rotation.
        match coded.picture_coding_type {
            PictureCodingType::Bidirectional => {
                // B-frames are displayed immediately, in coded order, and
                // never become a reference. They emit before the held-back
                // anchor.
                reorder.push_b(coded.clone(), &mut output);
            }
            PictureCodingType::DcIntra => {
                // 11172-2 §2.4.1: a D-picture sequence contains no
                // other picture types, so there is no reorder and no
                // reference rotation — D-pictures display in coded
                // order and never serve as a prediction reference.
                output.push(coded);
            }
            PictureCodingType::Intra | PictureCodingType::Predictive => {
                // An I/P frame displaces the previously held-back anchor
                // (which now displays) and becomes the new held-back
                // anchor. Rotate the §7.6 reference pair: the old backward
                // anchor becomes the new forward anchor.
                reorder.push_anchor(coded.clone(), &mut output);
                forward_anchor = backward_anchor.take();
                backward_anchor = Some(coded.frame);
            }
        }

        offset = next_offset;
    }

    // §6.1.1.11: flush the final held-back I/P anchor.
    reorder.flush(&mut output);

    Ok(output)
}

/// Compute the **continuous display index** of each frame in a
/// **coded-order** sequence from its §6.3.10 `temporal_reference`,
/// tracking the §6.3.8 / §6.1.1.11 GOP reset.
///
/// `temporal_reference` is a 10-bit field that, in display order,
/// increments by one (modulo 1024) within a group of pictures and is
/// *"set to zero"* for the first frame (in display order) after each
/// `group_of_pictures_header()` (§6.3.9). On its own a `temporal_reference`
/// is therefore only meaningful relative to its GOP base; to compare
/// frames across GOPs (or to detect the modulo-1024 wrap inside a long
/// GOP) the per-GOP base must be accumulated.
///
/// This function walks the **coded-order** frames and assigns each a
/// strictly-GOP-relative-base + `temporal_reference` index that is
/// globally monotonic in display order:
///
/// * A GOP boundary is detected when a frame's `temporal_reference`
///   returns to `0` after a non-zero value has been seen in the current
///   GOP (the §6.3.9 *"set to zero"* reset), at which point the running
///   base advances past the previous GOP's highest display index.
/// * Within a GOP the index is `gop_base + temporal_reference`.
///
/// The result is in **coded order** (same order as the input); each
/// entry is the display index of that coded frame. Sorting the frames by
/// this index yields display order — equivalent to the §6.1.1.11
/// structural reorder, but derived from `temporal_reference` rather than
/// from the I/P/B hold-back, so the two can be cross-checked
/// ([`verify_display_order`]).
///
/// `temporal_references` is the coded-order list of each frame's
/// `temporal_reference`. For a coded frame sent as two field pictures
/// the two share one `temporal_reference` (§6.3.9); pass it once per
/// **frame**.
pub fn display_indices_from_temporal_references(temporal_references: &[u16]) -> Vec<u64> {
    let mut indices = Vec::with_capacity(temporal_references.len());
    let mut gop_base: u64 = 0;
    // Highest display index seen in the current GOP (so the next GOP's
    // base starts just past it). `None` until the first frame.
    let mut gop_max: Option<u64> = None;
    // Whether a non-zero temporal_reference has been seen since the last
    // GOP reset — needed so a leading `tr == 0` (the first GOP) is not
    // mistaken for a reset, and so a GOP whose first coded frame has
    // tr != 0 (coded order: the I-frame's tr is its display position,
    // which can be > 0 when B-frames precede it in display order) still
    // resets correctly on the next `tr == 0`.
    let mut seen_nonzero = false;

    for (i, &tr) in temporal_references.iter().enumerate() {
        if tr == 0 && i != 0 && seen_nonzero {
            // §6.3.9 reset: a new GOP begins. Advance the base past the
            // previous GOP's highest display index.
            gop_base = gop_max.map_or(0, |m| m + 1);
            seen_nonzero = false;
        }
        if tr != 0 {
            seen_nonzero = true;
        }
        let index = gop_base + u64::from(tr);
        gop_max = Some(gop_max.map_or(index, |m| m.max(index)));
        indices.push(index);
    }
    indices
}

/// Compute the continuous display index of each frame in a
/// **coded-order** sequence from its `temporal_reference` **and**
/// `picture_coding_type`, tracking GOP boundaries by the anchor
/// pattern rather than the bare `tr == 0` reset heuristic of
/// [`display_indices_from_temporal_references`].
///
/// Within one GOP the anchors (I / P) appear in coded order with
/// **strictly increasing** `temporal_reference` (each anchor is coded
/// before the B-frames that precede it in display order, and after
/// every earlier anchor of the GOP). A new GOP therefore begins
/// exactly at an anchor whose `temporal_reference` is **not greater**
/// than the previous anchor's — the §6.3.9 / §2.4.3.3 per-GOP reset
/// makes the new GOP's first coded frame (its I, whose `tr` counts
/// the leading B-frames) land at or below the old GOP's running
/// anchor position. This classifies the GOP's leading I correctly
/// where the trefs-only heuristic cannot: coded order
/// `… P(tr 11) | I(tr 2) B(tr 0) B(tr 1) …` — the I belongs to the
/// **new** GOP even though its `tr` is neither `0` nor smaller than
/// every earlier index.
///
/// Returns the display index of each coded frame, in coded order.
pub fn display_indices_from_coded_pictures(pictures: &[(u16, PictureCodingType)]) -> Vec<u64> {
    let mut indices = Vec::with_capacity(pictures.len());
    let mut gop_base: u64 = 0;
    let mut gop_max: Option<u64> = None;
    // The previous anchor's temporal_reference within the current GOP.
    let mut prev_anchor_tr: Option<u16> = None;

    for &(tr, kind) in pictures {
        let is_anchor = matches!(
            kind,
            PictureCodingType::Intra | PictureCodingType::Predictive
        );
        if is_anchor {
            if let Some(prev) = prev_anchor_tr {
                if tr <= prev {
                    // §6.3.9 GOP reset: advance the base past the
                    // previous GOP's highest display index.
                    gop_base = gop_max.map_or(0, |m| m + 1);
                }
            }
            prev_anchor_tr = Some(tr);
        }
        let index = gop_base + u64::from(tr);
        gop_max = Some(gop_max.map_or(index, |m| m.max(index)));
        indices.push(index);
    }
    indices
}

/// [`verify_display_order`] with the coded pictures' coding types —
/// uses the anchor-pattern GOP detection of
/// [`display_indices_from_coded_pictures`], which classifies a GOP
/// whose leading I-frame has `temporal_reference > 0` correctly.
///
/// `coded_order` lists each coded frame's `(temporal_reference,
/// picture_coding_type)` in coded order; `display_order_trefs` the
/// `temporal_reference` of each frame in the display order under
/// test.
///
/// # Errors
/// * [`Error::InvalidBitstream`] under the same conditions as
///   [`verify_display_order`].
pub fn verify_display_order_with_types(
    coded_order: &[(u16, PictureCodingType)],
    display_order_trefs: &[u16],
) -> Result<()> {
    if coded_order.len() != display_order_trefs.len() {
        return Err(Error::InvalidBitstream(
            "verify_display_order_with_types: coded / display frame counts differ (§6.1.1.11)",
        ));
    }
    let coded_indices = display_indices_from_coded_pictures(coded_order);
    verify_display_order_against_indices(
        coded_order.iter().map(|&(tr, _)| tr),
        &coded_indices,
        display_order_trefs,
    )
}

/// Verify that a **display-order** sequence of frames is consistent with
/// the §6.1.1.11 reorder: the continuous display indices derived from
/// each frame's `temporal_reference`
/// ([`display_indices_from_temporal_references`], computed over the
/// *coded* order) must be strictly increasing when the frames are read in
/// display order.
///
/// `coded_order_trefs` is the `temporal_reference` of each frame in
/// **coded** order; `display_order_trefs` is the `temporal_reference` of
/// each frame in **display** order (e.g. the order
/// [`decode_video_sequence`] returns). Both list one entry per frame and
/// must be permutations of each other.
///
/// Returns `Ok(())` when the display order is a valid §6.1.1.11
/// presentation order; otherwise [`Error::InvalidBitstream`].
///
/// # Errors
/// * [`Error::InvalidBitstream`] if the two lists differ in length, are
///   not permutations of one another, or the display order is not
///   strictly increasing in display index (a reorder inconsistency).
pub fn verify_display_order(coded_order_trefs: &[u16], display_order_trefs: &[u16]) -> Result<()> {
    if coded_order_trefs.len() != display_order_trefs.len() {
        return Err(Error::InvalidBitstream(
            "verify_display_order: coded / display frame counts differ (§6.1.1.11)",
        ));
    }
    let coded_indices = display_indices_from_temporal_references(coded_order_trefs);
    verify_display_order_against_indices(
        coded_order_trefs.iter().copied(),
        &coded_indices,
        display_order_trefs,
    )
}

/// Shared body of [`verify_display_order`] /
/// [`verify_display_order_with_types`]: match each display frame to
/// the smallest unconsumed coded frame carrying the same
/// `temporal_reference` (a `temporal_reference` value can repeat
/// across GOPs but never within one, and the derived display indices
/// are globally unique), then require the matched indices to be
/// strictly increasing.
fn verify_display_order_against_indices(
    coded_order_trefs: impl Iterator<Item = u16>,
    coded_indices: &[u64],
    display_order_trefs: &[u16],
) -> Result<()> {
    let mut available: Vec<(u16, u64)> = coded_order_trefs
        .zip(coded_indices.iter().copied())
        .collect();

    let mut prev: Option<u64> = None;
    for &tr in display_order_trefs {
        // Find the smallest unconsumed display index whose frame carries
        // this temporal_reference.
        let pick = available
            .iter()
            .enumerate()
            .filter(|(_, (t, _))| *t == tr)
            .min_by_key(|(_, (_, idx))| *idx)
            .map(|(pos, (_, idx))| (pos, *idx));
        let Some((pos, idx)) = pick else {
            return Err(Error::InvalidBitstream(
                "verify_display_order: display frame has no matching coded frame (not a permutation)",
            ));
        };
        if let Some(p) = prev {
            if idx <= p {
                return Err(Error::InvalidBitstream(
                    "verify_display_order: display order not strictly increasing in display index (§6.1.1.11)",
                ));
            }
        }
        prev = Some(idx);
        available.swap_remove(pos);
    }
    debug_assert!(available.is_empty());
    Ok(())
}

/// Holds back one I/P anchor by one picture so it displays *after* the
/// B-frames that follow it in coded order but precede it in display
/// order (§6.1.1.11).
#[derive(Debug, Default)]
struct ReorderBuffer {
    /// The most recently decoded I/P frame, waiting to be displayed once
    /// the next anchor arrives (or at end of stream).
    held_anchor: Option<DecodedFrame>,
}

impl ReorderBuffer {
    fn new() -> Self {
        Self::default()
    }

    /// A B-frame: emit it immediately (coded order == display order for a
    /// B-frame, which is always displayed before the next anchor).
    fn push_b(&mut self, frame: DecodedFrame, output: &mut Vec<DecodedFrame>) {
        output.push(frame);
    }

    /// An I/P frame: the previously held anchor now displays, and this
    /// frame becomes the new held anchor.
    fn push_anchor(&mut self, frame: DecodedFrame, output: &mut Vec<DecodedFrame>) {
        if let Some(prev) = self.held_anchor.take() {
            output.push(prev);
        }
        self.held_anchor = Some(frame);
    }

    /// End of sequence: flush the final held anchor.
    fn flush(&mut self, output: &mut Vec<DecodedFrame>) {
        if let Some(prev) = self.held_anchor.take() {
            output.push(prev);
        }
    }
}

/// Parse the leading `sequence_header()` + `sequence_extension()` pair at
/// the start of `stream`.
fn parse_leading_sequence(stream: &[u8]) -> Result<SequenceGeometry> {
    let Some(rel) = find_start_code(stream, |code| code == SEQUENCE_HEADER_CODE) else {
        return Err(Error::InvalidBitstream(
            "video_sequence(): missing leading sequence_header_code 0x000001B3 (§6.2.2)",
        ));
    };
    sequence_geometry_at(&stream[rel..])
}

/// The parsed sequence layer, discriminating the two standards this
/// crate decodes: an ISO/IEC 13818-2 stream (sequence_header +
/// mandatory sequence_extension, §6.1.1.6) or an ISO/IEC 11172-2
/// stream (sequence header only — the extension start code does not
/// follow).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SequenceGeometry {
    /// ISO/IEC 13818-2 geometry + per-picture DCT context seed, plus
    /// the §6.3.11 weighting-matrix state as established by this
    /// sequence header (defaults reset + the header's own
    /// `load_*_quantiser_matrix` payloads).
    Mpeg2(IntraPictureParams, QuantiserMatrixState),
    /// ISO/IEC 11172-2 geometry + sequence-header quantiser matrices.
    Mpeg1(Mpeg1PictureParams),
}

impl SequenceGeometry {
    /// The §6.3.11 matrix state right after this sequence header:
    /// defaults, overwritten by the header's own load flags. MPEG-1
    /// carries its matrices inside [`Mpeg1PictureParams`] instead,
    /// so its slot here stays at the (unused) defaults.
    fn initial_matrices(&self) -> QuantiserMatrixState {
        match self {
            SequenceGeometry::Mpeg2(_, matrices) => *matrices,
            SequenceGeometry::Mpeg1(_) => QuantiserMatrixState::default(),
        }
    }
}

/// Parse the sequence layer at `buf` (which begins with the
/// `sequence_header_code`) and classify the stream: a
/// `sequence_extension()` immediately after the header makes it an
/// ISO/IEC 13818-2 sequence (§6.1.1.6 *"sequence_header() shall be
/// followed by sequence_extension()"*); its absence makes it an
/// ISO/IEC 11172-2 sequence, whose geometry and quantiser matrices
/// come from the header alone (§2.4.2.3).
fn sequence_geometry_at(buf: &[u8]) -> Result<SequenceGeometry> {
    match Mpeg2Sequence::from_buf(buf) {
        Ok(seq) => {
            // §6.3.11: the sequence header resets every matrix to its
            // §6.3.7 default, then its own load flags download
            // replacements. A header-loaded matrix applies to both the
            // luminance and (at 4:2:2 / 4:4:4) chrominance slots —
            // exactly the composition `QuantMatrixExtension::apply`
            // implements for the same two payload kinds, so the header
            // loads are routed through it.
            let mut matrices = QuantiserMatrixState::default();
            let header_loads =
                crate::quant_matrix_extension::QuantMatrixExtension {
                    intra: seq.header.intra_quant.map(|zz| {
                        crate::quant_matrix_extension::QuantiserMatrixPayload { bytes: zz }
                    }),
                    non_intra: seq.header.non_intra_quant.map(|zz| {
                        crate::quant_matrix_extension::QuantiserMatrixPayload { bytes: zz }
                    }),
                    chroma_intra: None,
                    chroma_non_intra: None,
                };
            header_loads.apply(&mut matrices, seq.extension.chroma_format);
            Ok(SequenceGeometry::Mpeg2(sequence_geometry(&seq), matrices))
        }
        Err(_) => {
            // No sequence_extension: ISO/IEC 11172-2. Parse the bare
            // header (geometry + optional downloadable matrices, both
            // transmitted in zigzag order per §2.4.2.3 / §2.4.3.2).
            let header = crate::sequence_header::Mpeg2SequenceHeader::parse(buf)?;
            let to_raster = |zz: [u8; 64]| {
                crate::quant_matrix_extension::QuantiserMatrixPayload { bytes: zz }.to_matrix()
            };
            let intra_quant = header
                .intra_quant
                .map(to_raster)
                .unwrap_or(crate::dequantize::DEFAULT_INTRA_QUANT);
            let non_intra_quant = header
                .non_intra_quant
                .map(to_raster)
                .unwrap_or([[16u8; 8]; 8]);
            Ok(SequenceGeometry::Mpeg1(Mpeg1PictureParams {
                width: header.width as usize,
                height: header.height as usize,
                intra_quant,
                non_intra_quant,
            }))
        }
    }
}

/// Build the per-picture geometry the I/P/B drivers consume from the
/// parsed sequence layer. The DCT-context flags live in
/// `picture_coding_extension()` (per-picture), so they are seeded to a
/// neutral default here and overwritten per picture by
/// [`reconstruct_picture`].
fn sequence_geometry(sequence: &Mpeg2Sequence) -> IntraPictureParams {
    IntraPictureParams {
        width: sequence.horizontal_size as usize,
        height: sequence.vertical_size as usize,
        chroma_format: sequence.extension.chroma_format,
        frame_pred_frame_dct: true,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        // §6.3.3: drives the frame-picture macroblock-grid height
        // (Ceil(h/16) progressive vs 2*Ceil(h/32) interlaced).
        progressive_sequence: sequence.extension.progressive_sequence,
    }
}

/// Scan the picture region (picture header → first slice) for
/// `quant_matrix_extension()`s and apply each to the running §6.3.11
/// matrix state.
///
/// Per §6.2.3.7 `extension_and_user_data(2)` any number of extensions
/// may sit between the `picture_coding_extension()` and the first
/// `slice_start_code`; only those whose 4-bit
/// `extension_start_code_identifier` is the Table 6-2 Quant Matrix
/// Extension ID (`0011`) are parsed — others (display, copyright,
/// scalable…) are left to their own parsers.
fn apply_quant_matrix_extensions(
    picture_region: &[u8],
    chroma_format: crate::sequence_extension::ChromaFormat,
    matrices: &mut QuantiserMatrixState,
) -> Result<()> {
    let mut pos = 0usize;
    while pos + 4 < picture_region.len() {
        let w = &picture_region[pos..];
        if w[0] == 0x00 && w[1] == 0x00 && w[2] == 0x01 {
            let code = w[3];
            // First slice start code ends the extension region
            // (§6.2.3.6 picture_data()).
            if (0x01..=0xAF).contains(&code) {
                break;
            }
            if code == 0xB5 && pos + 4 < picture_region.len() {
                // 4-bit extension_start_code_identifier in the high
                // nibble of the byte after the start code (Table 6-2;
                // Quant Matrix Extension ID = 0011).
                if picture_region[pos + 4] >> 4 == 0b0011 {
                    let ext = QuantMatrixExtension::parse(&picture_region[pos..], chroma_format)?;
                    ext.apply(matrices, chroma_format);
                }
            }
            pos += 4;
        } else {
            pos += 1;
        }
    }
    Ok(())
}

/// Reconstruct one picture, dispatching on `picture_coding_type` and
/// applying the per-picture DCT-context flags from
/// `picture_coding_extension()`.
fn reconstruct_picture(
    picture_region: &[u8],
    header: &Mpeg2PictureHeader,
    ext: &PictureCodingExtension,
    base_geometry: IntraPictureParams,
    forward_anchor: Option<&FrameBuffer>,
    backward_anchor: Option<&FrameBuffer>,
    matrices: &QuantiserMatrixState,
) -> Result<DecodedFrame> {
    // Overlay the per-picture §6.2.3.1 DCT-context flags onto the
    // sequence geometry.
    let geometry = IntraPictureParams {
        frame_pred_frame_dct: ext.frame_pred_frame_dct,
        intra_dc_precision: ext.intra_dc_precision,
        intra_vlc_format: ext.intra_vlc_format,
        alternate_scan: ext.alternate_scan,
        q_scale_type: ext.q_scale_type,
        ..base_geometry
    };

    let frame = match header.picture_coding_type {
        PictureCodingType::Intra => {
            let (frame, _placed) =
                decode_intra_picture_with_matrices(picture_region, geometry, matrices)?;
            frame
        }
        // Unreachable: `parse_with_extension` rejects the Table 6-12
        // '100' code before this dispatcher runs.
        PictureCodingType::DcIntra => {
            return Err(Error::InvalidBitstream(
                "picture_coding_type: 100 (D-picture) shall not be used in MPEG-2 (Table 6-12)",
            ))
        }
        PictureCodingType::Predictive => {
            // §7.6: a P-frame predicts from the latest decoded I/P anchor.
            let forward = backward_anchor.ok_or(Error::InvalidBitstream(
                "§6.1.1.11: P-picture before any I/P anchor exists (no forward reference)",
            ))?;
            let params = inter_params(header, ext, geometry);
            let (frame, _placed) = decode_inter_picture_with_matrices(
                picture_region,
                params,
                ReferenceFrames::forward_only(forward),
                matrices,
            )?;
            frame
        }
        PictureCodingType::Bidirectional => {
            // §7.6: a B-frame predicts from the two latest I/P anchors —
            // forward (older) and backward (newer).
            let forward = forward_anchor.ok_or(Error::InvalidBitstream(
                "§6.1.1.11: B-picture before two I/P anchors exist (no forward reference)",
            ))?;
            let backward = backward_anchor.ok_or(Error::InvalidBitstream(
                "§6.1.1.11: B-picture before two I/P anchors exist (no backward reference)",
            ))?;
            let params = inter_params(header, ext, geometry);
            let (frame, _placed) = decode_inter_picture_with_matrices(
                picture_region,
                params,
                ReferenceFrames::bidirectional(forward, backward),
                matrices,
            )?;
            frame
        }
    };

    Ok(DecodedFrame {
        frame,
        temporal_reference: header.temporal_reference,
        picture_coding_type: header.picture_coding_type,
    })
}

/// Reconstruct one ISO/IEC 11172-2 picture, dispatching on
/// `picture_coding_type`. MPEG-1 pictures are always frame pictures;
/// the motion-vector context (forward/backward f_code + full_pel
/// flags) comes straight from the picture header (§2.4.3.4).
fn reconstruct_mpeg1_picture(
    picture_region: &[u8],
    header: &Mpeg2PictureHeader,
    params: &Mpeg1PictureParams,
    forward_anchor: Option<&FrameBuffer>,
    backward_anchor: Option<&FrameBuffer>,
) -> Result<DecodedFrame> {
    let frame = match header.picture_coding_type {
        PictureCodingType::Intra => {
            let (frame, _placed) = decode_mpeg1_intra_picture(picture_region, params)?;
            frame
        }
        PictureCodingType::DcIntra => {
            // §2.4.3.4 dc intra-coded picture: DC-only intra blocks,
            // Table B.2d macroblock type, end_of_macroblock markers.
            let (frame, _placed) = decode_mpeg1_d_picture(picture_region, params)?;
            frame
        }
        PictureCodingType::Predictive => {
            let forward = backward_anchor.ok_or(Error::InvalidBitstream(
                "§2.4.1: P-picture before any I/P anchor exists (no forward reference)",
            ))?;
            let inter = mpeg1_inter_params(header, params)?;
            let (frame, _placed) = decode_mpeg1_inter_picture(
                picture_region,
                &inter,
                ReferenceFrames::forward_only(forward),
            )?;
            frame
        }
        PictureCodingType::Bidirectional => {
            let forward = forward_anchor.ok_or(Error::InvalidBitstream(
                "§2.4.1: B-picture before two I/P anchors exist (no forward reference)",
            ))?;
            let backward = backward_anchor.ok_or(Error::InvalidBitstream(
                "§2.4.1: B-picture before two I/P anchors exist (no backward reference)",
            ))?;
            let inter = mpeg1_inter_params(header, params)?;
            let (frame, _placed) = decode_mpeg1_inter_picture(
                picture_region,
                &inter,
                ReferenceFrames::bidirectional(forward, backward),
            )?;
            frame
        }
    };

    Ok(DecodedFrame {
        frame,
        temporal_reference: header.temporal_reference,
        picture_coding_type: header.picture_coding_type,
    })
}

/// Build the §2.4.4.2 / §2.4.4.3 motion context for an MPEG-1 P/B
/// picture from its picture header.
fn mpeg1_inter_params(
    header: &Mpeg2PictureHeader,
    params: &Mpeg1PictureParams,
) -> Result<Mpeg1InterParams> {
    let forward_f_code = header.fwd_f_code.ok_or(Error::InvalidBitstream(
        "mpeg1 P/B picture header missing forward_f_code (§2.4.3.4)",
    ))?;
    let full_pel_forward_vector = header.full_pel_forward_vector.unwrap_or(false);
    let (backward_f_code, full_pel_backward_vector) =
        if header.picture_coding_type == PictureCodingType::Bidirectional {
            (
                header.bwd_f_code.ok_or(Error::InvalidBitstream(
                    "mpeg1 B picture header missing backward_f_code (§2.4.3.4)",
                ))?,
                header.full_pel_backward_vector.unwrap_or(false),
            )
        } else {
            (1, false)
        };
    Ok(Mpeg1InterParams {
        base: *params,
        picture_coding_type: header.picture_coding_type,
        forward_f_code,
        full_pel_forward_vector,
        backward_f_code,
        full_pel_backward_vector,
    })
}

/// The first field of a §6.1.1.4.1 coded-frame pair, held until its
/// partner field arrives so the two can be interleaved into one
/// reconstructed frame.
struct PendingField {
    /// The reconstructed first field (field-height [`FrameBuffer`]).
    field: FrameBuffer,
    /// Which field of the frame this is (`TopField` / `BottomField`).
    structure: PictureStructure,
    /// §6.3.10 `picture_coding_type` — both fields of a coded frame share
    /// the I/P/B classification for reordering purposes (an I+P pair is
    /// still an anchor frame; §6.1.1.4.1 also constrains the second field
    /// type by the first).
    coding_type: PictureCodingType,
    /// §6.3.10 `temporal_reference` — identical for both field pictures of
    /// a coded frame (§6.3.10 *"When a frame is coded as two field
    /// pictures, the temporal_reference associated with each coded picture
    /// shall be the same"*).
    temporal_reference: u16,
}

/// Decode one **field picture** and, when it completes a §6.1.1.4.1 pair,
/// interleave the two fields into one reconstructed [`DecodedFrame`].
///
/// Returns `Ok(None)` when this is the **first** field of a pair (held in
/// `pending_field` for its partner), and `Ok(Some(frame))` when this is
/// the **second** field, which interleaves the held first field with this
/// one into a frame (§3.131 / §3.13 top→even / bottom→odd lines).
///
/// ## Reference fields (§7.6.2.1)
///
/// * **First field** of a coded frame, or any **B**-field: the two
///   reference fields are the two fields of the previously reconstructed
///   reference frame(s) — `backward_anchor` for the forward/P reference
///   and `forward_anchor` for the B backward reference, exactly as a
///   frame picture uses them.
/// * **Second field** of a **P** coded frame: §7.6.2.1 / Figures 7-7,
///   7-8 — the most-recent reference field is the *first field of this
///   same coded frame* (just reconstructed); the other reference field is
///   the opposite-parity field of the previous reconstructed frame. The
///   synthetic reference frame handed to the driver therefore carries the
///   current first field in its own parity slot and the previous frame's
///   field in the opposite slot ([`reference_frame_for_second_p_field`]).
///   A second I-field needs no reference; a second B-field uses the two
///   anchor frames like the first.
// Every argument is §7.6.2 state the field-pair reconstruction needs
// (the two anchors, the held first field, and the §6.3.11 matrices).
#[allow(clippy::too_many_arguments)]
fn reconstruct_field_pair(
    picture_region: &[u8],
    header: &Mpeg2PictureHeader,
    ext: &PictureCodingExtension,
    base_geometry: IntraPictureParams,
    forward_anchor: Option<&FrameBuffer>,
    backward_anchor: Option<&FrameBuffer>,
    pending_field: &mut Option<PendingField>,
    matrices: &QuantiserMatrixState,
) -> Result<Option<DecodedFrame>> {
    let structure = ext.picture_structure;
    let is_second_field = pending_field
        .as_ref()
        .is_some_and(|p| p.structure != structure);

    // The field geometry: half the frame height, frame_pred_frame_dct
    // forced off (§6.3.10 forbids it in a field picture). The DCT-context
    // flags come from the picture coding extension.
    let geometry = field_geometry(base_geometry, ext);

    // Build, when this is the P second field, the synthetic reference
    // frame that pairs the just-decoded first field with the previous
    // frame's opposite-parity field. Materialised here so it outlives the
    // `ReferenceFrames` borrow below.
    let second_field_reference =
        if is_second_field && header.picture_coding_type == PictureCodingType::Predictive {
            let pending = pending_field
                .as_ref()
                .expect("second field implies pending");
            let prev = backward_anchor.ok_or(Error::InvalidBitstream(
                "§7.6.2.1: P second field before any reference frame exists",
            ))?;
            Some(reference_frame_for_second_p_field(
                &pending.field,
                pending.structure,
                prev,
            )?)
        } else {
            None
        };

    let field = match header.picture_coding_type {
        // Field pictures are 13818-2-only; D-pictures are 11172-2-only
        // (Table 6-12) — `parse_with_extension` rejects the pairing
        // before this dispatcher runs.
        PictureCodingType::DcIntra => {
            return Err(Error::InvalidBitstream(
                "picture_coding_type: 100 (D-picture) shall not be used in MPEG-2 (Table 6-12)",
            ))
        }
        PictureCodingType::Intra => {
            // An I **field** picture must be walked with the field
            // picture_structure: §6.2.5.1 gates `dct_type` on
            // `picture_structure == "Frame picture"`, so a field
            // picture's intra macroblocks carry no dct_type bit —
            // routing through the frame-structure intra driver would
            // consume one spurious bit per macroblock and shear the
            // whole slice parse. `decode_field_picture` places intra
            // macroblocks itself; an I field forms no predictions, so
            // no references are supplied.
            let params = inter_params(header, ext, geometry);
            let (field, _placed) = decode_field_picture_with_matrices(
                picture_region,
                params,
                structure,
                ReferenceFrames {
                    forward: None,
                    backward: None,
                },
                matrices,
            )?;
            field
        }
        PictureCodingType::Predictive => {
            let params = inter_params(header, ext, geometry);
            let reference = if let Some(synthetic) = second_field_reference.as_ref() {
                synthetic
            } else {
                backward_anchor.ok_or(Error::InvalidBitstream(
                    "§7.6.2.1: P field before any reference frame exists",
                ))?
            };
            let (field, _placed) = decode_field_picture_with_matrices(
                picture_region,
                params,
                structure,
                ReferenceFrames::forward_only(reference),
                matrices,
            )?;
            field
        }
        PictureCodingType::Bidirectional => {
            let forward = forward_anchor.ok_or(Error::InvalidBitstream(
                "§7.6.2.1: B field before two reference frames exist (no forward reference)",
            ))?;
            let backward = backward_anchor.ok_or(Error::InvalidBitstream(
                "§7.6.2.1: B field before two reference frames exist (no backward reference)",
            ))?;
            let params = inter_params(header, ext, geometry);
            let (field, _placed) = decode_field_picture_with_matrices(
                picture_region,
                params,
                structure,
                ReferenceFrames::bidirectional(forward, backward),
                matrices,
            )?;
            field
        }
    };

    match pending_field.take() {
        // First field of a pair: hold it back for its partner.
        None => {
            *pending_field = Some(PendingField {
                field,
                structure,
                coding_type: header.picture_coding_type,
                temporal_reference: header.temporal_reference,
            });
            Ok(None)
        }
        // Second field: interleave the pair into one frame (§3.131 /
        // §3.13). The top field always supplies the even lines regardless
        // of which field was coded first.
        Some(first) => {
            let (top, bottom) = match first.structure {
                PictureStructure::TopField => (&first.field, &field),
                _ => (&field, &first.field),
            };
            let frame = assemble_frame_from_fields(top, bottom)?;
            Ok(Some(DecodedFrame {
                frame,
                // §6.3.10: both fields share the temporal_reference; the
                // frame's I/P/B class for reordering follows the first
                // field (an I+P field pair is an anchor frame).
                temporal_reference: first.temporal_reference,
                picture_coding_type: first.coding_type,
            }))
        }
    }
}

/// Derive the **field** geometry of a field picture from the frame
/// geometry: the field height is half the frame height (§6.1.1.4.1), and
/// `frame_pred_frame_dct` is forced off (a field picture forbids it,
/// §6.3.10). The §6.2.3.1 DCT-context flags are taken from the picture
/// coding extension.
fn field_geometry(
    base_geometry: IntraPictureParams,
    ext: &PictureCodingExtension,
) -> IntraPictureParams {
    IntraPictureParams {
        // §6.3.3: a field is Ceil(h/2) lines tall (the top field of
        // an odd-height frame carries the extra line), and its own
        // macroblock grid is Ceil(vertical_size / 32) rows =
        // Ceil(field_height / 16) — 16-aligned in field coordinates,
        // which is the progressive grid rule, so the field-level
        // params set `progressive_sequence` accordingly.
        height: base_geometry.height.div_ceil(2),
        frame_pred_frame_dct: false,
        intra_dc_precision: ext.intra_dc_precision,
        intra_vlc_format: ext.intra_vlc_format,
        alternate_scan: ext.alternate_scan,
        q_scale_type: ext.q_scale_type,
        progressive_sequence: true,
        ..base_geometry
    }
}

/// Build the synthetic reference **frame** a §7.6.2.1 P second field
/// predicts from (Figures 7-7 / 7-8): the just-decoded first field sits
/// in its own parity slot, and the opposite-parity reference field is
/// taken from `prev_frame` (the previously reconstructed reference
/// frame). The driver's §6.3.17.2 `motion_vertical_field_select` then
/// picks between the two as usual.
///
/// `first_field` is the field-height first field; `first_structure` is
/// its parity; `prev_frame` is the full-height previous reference frame.
/// The returned frame interleaves the two fields by parity (§3.131 /
/// §3.13).
pub(crate) fn reference_frame_for_second_p_field(
    first_field: &FrameBuffer,
    first_structure: PictureStructure,
    prev_frame: &FrameBuffer,
) -> Result<FrameBuffer> {
    // Extract the previous frame's two fields, then replace the
    // first-field parity slot with this frame's first field.
    let prev_top = extract_field(prev_frame, PictureStructure::TopField);
    let prev_bottom = extract_field(prev_frame, PictureStructure::BottomField);
    let (top, bottom) = match first_structure {
        PictureStructure::TopField => (first_field, &prev_bottom),
        _ => (&prev_top, first_field),
    };
    assemble_frame_from_fields(top, bottom)
}

/// Extract one field (even / odd lines) of a full-height frame into a
/// field-height [`FrameBuffer`], the inverse of
/// [`assemble_frame_from_fields`]: top field = even frame lines (§3.131),
/// bottom field = odd frame lines (§3.13). Used to recover the previous
/// frame's individual reference fields for §7.6.2.1 second-field
/// prediction.
pub(crate) fn extract_field(frame: &FrameBuffer, structure: PictureStructure) -> FrameBuffer {
    let parity = match structure {
        PictureStructure::BottomField => 1usize,
        _ => 0usize,
    };
    let field_height = frame.height.div_ceil(2);
    let mut field = FrameBuffer::new(frame.width, field_height, frame.chroma_format);
    copy_field_plane(&mut field.y, &frame.y, parity);
    copy_field_plane(&mut field.cb, &frame.cb, parity);
    copy_field_plane(&mut field.cr, &frame.cr, parity);
    field
}

/// Copy every `2·r + parity` row of `src` into row `r` of `dst`.
fn copy_field_plane(
    dst: &mut crate::frame_assembly::Plane,
    src: &crate::frame_assembly::Plane,
    parity: usize,
) {
    for r in 0..dst.height() {
        let src_y = 2 * r + parity;
        for x in 0..dst.width() {
            if let Some(v) = src.get(x, src_y) {
                dst.put_sample(x, r, v);
            }
        }
    }
}

/// Build the §7.6.3 motion-vector parameters for a P/B picture from the
/// picture header + coding extension.
fn inter_params(
    header: &Mpeg2PictureHeader,
    ext: &PictureCodingExtension,
    geometry: IntraPictureParams,
) -> PicturePredictionParams {
    PicturePredictionParams {
        geometry,
        picture_coding_type: header.picture_coding_type,
        f_code_fwd_horiz: ext.f_code_fwd_horiz,
        f_code_fwd_vert: ext.f_code_fwd_vert,
        f_code_bwd_horiz: ext.f_code_bwd_horiz,
        f_code_bwd_vert: ext.f_code_bwd_vert,
        concealment_motion_vectors: ext.concealment_motion_vectors,
        top_field_first: ext.top_field_first,
    }
}

/// Find the byte offset of the next `picture_start_code` (`0x00000100`)
/// or `sequence_header_code` (`0x000001B3`) in `buf`, or `None`. These
/// are the two start codes the top-level loop dispatches on: a picture to
/// reconstruct, or a sequence header to re-read the geometry from.
fn find_picture_or_sequence_start_code(buf: &[u8]) -> Option<usize> {
    find_start_code(buf, |code| {
        code == PICTURE_START_CODE || code == SEQUENCE_HEADER_CODE
    })
}

/// Whether `buf` begins with the start code `code`.
fn is_start_code(buf: &[u8], code: u32) -> bool {
    buf.len() >= 4
        && (u32::from(buf[0]) << 24
            | u32::from(buf[1]) << 16
            | u32::from(buf[2]) << 8
            | u32::from(buf[3]))
            == code
}

/// Find the byte offset of the next picture-region boundary in `buf` —
/// the next `picture_start_code`, `group_start_code` (`0x000001B8`),
/// `sequence_header_code` (`0x000001B3`), or `sequence_end_code`
/// (`0x000001B7`) — or `None` if the region runs to the end of the
/// buffer.
fn find_next_picture_boundary(buf: &[u8]) -> Option<usize> {
    buf.windows(4).position(|w| {
        w[0] == 0x00 && w[1] == 0x00 && w[2] == 0x01 && matches!(w[3], 0x00 | 0xB8 | 0xB3 | 0xB7)
    })
}

/// Find the byte offset of the first start code whose 32-bit value
/// satisfies `pred`, or `None`.
fn find_start_code(buf: &[u8], pred: impl Fn(u32) -> bool) -> Option<usize> {
    buf.windows(4).position(|w| {
        if w[0] == 0x00 && w[1] == 0x00 && w[2] == 0x01 {
            let code = (u32::from(w[0]) << 24)
                | (u32::from(w[1]) << 16)
                | (u32::from(w[2]) << 8)
                | u32::from(w[3]);
            pred(code)
        } else {
            false
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reorder_buffer_passes_b_frames_through_immediately() {
        let mut output = Vec::new();
        let mut buf = ReorderBuffer::new();

        // Coded order I B B P → display I B B P (anchor held one back).
        buf.push_anchor(frame_with(0, PictureCodingType::Intra), &mut output);
        // I is held; nothing displayed yet.
        assert!(output.is_empty());

        buf.push_b(frame_with(1, PictureCodingType::Bidirectional), &mut output);
        buf.push_b(frame_with(2, PictureCodingType::Bidirectional), &mut output);
        // The two B-frames display immediately, before the held I.
        assert_eq!(trefs(&output), vec![1, 2]);

        buf.push_anchor(frame_with(3, PictureCodingType::Predictive), &mut output);
        // The held I now displays as the new anchor arrives.
        assert_eq!(trefs(&output), vec![1, 2, 0]);

        buf.flush(&mut output);
        // The final P flushes.
        assert_eq!(trefs(&output), vec![1, 2, 0, 3]);
    }

    #[test]
    fn reorder_matches_spec_6_1_1_11_example() {
        // §6.1.1.11 worked example, coded order:
        //   1I 4P 2B 3B 7P 5B 6B
        // (temporal_reference shown; display order 1 2 3 4 5 6 7).
        let mut output = Vec::new();
        let mut buf = ReorderBuffer::new();

        let coded: &[(u16, PictureCodingType)] = &[
            (0, PictureCodingType::Intra),         // 1I
            (3, PictureCodingType::Predictive),    // 4P
            (1, PictureCodingType::Bidirectional), // 2B
            (2, PictureCodingType::Bidirectional), // 3B
            (6, PictureCodingType::Predictive),    // 7P
            (4, PictureCodingType::Bidirectional), // 5B
            (5, PictureCodingType::Bidirectional), // 6B
        ];
        for &(tref, kind) in coded {
            match kind {
                PictureCodingType::Bidirectional => buf.push_b(frame_with(tref, kind), &mut output),
                _ => buf.push_anchor(frame_with(tref, kind), &mut output),
            }
        }
        buf.flush(&mut output);

        // Display order temporal_references: 0 1 2 3 4 5 6.
        assert_eq!(trefs(&output), vec![0, 1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn no_b_frames_keeps_coded_order() {
        // I P P → no reordering (low_delay-style, §6.1.1.11).
        let mut output = Vec::new();
        let mut buf = ReorderBuffer::new();
        buf.push_anchor(frame_with(0, PictureCodingType::Intra), &mut output);
        buf.push_anchor(frame_with(1, PictureCodingType::Predictive), &mut output);
        buf.push_anchor(frame_with(2, PictureCodingType::Predictive), &mut output);
        buf.flush(&mut output);
        assert_eq!(trefs(&output), vec![0, 1, 2]);
    }

    #[test]
    fn missing_sequence_header_rejected() {
        // A stream that starts with a picture_start_code, no sequence.
        let stream = [0x00, 0x00, 0x01, 0x00, 0xFF, 0xFF];
        let err = decode_video_sequence(&stream).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn boundary_scan_stops_at_each_start_code() {
        // picture data … then a group_start_code.
        let buf = [
            0xAA, 0xBB, 0x00, 0x00, 0x01, 0xB8, 0xCC, // group_start_code at 2
        ];
        assert_eq!(find_next_picture_boundary(&buf), Some(2));

        // sequence_end_code.
        let buf2 = [0x11, 0x00, 0x00, 0x01, 0xB7];
        assert_eq!(find_next_picture_boundary(&buf2), Some(1));

        // No boundary.
        let buf3 = [0x11, 0x22, 0x33, 0x44];
        assert_eq!(find_next_picture_boundary(&buf3), None);
    }

    fn frame_with(tref: u16, kind: PictureCodingType) -> DecodedFrame {
        DecodedFrame {
            frame: FrameBuffer::new(16, 16, crate::sequence_extension::ChromaFormat::Yuv420),
            temporal_reference: tref,
            picture_coding_type: kind,
        }
    }

    fn trefs(output: &[DecodedFrame]) -> Vec<u16> {
        output.iter().map(|f| f.temporal_reference).collect()
    }

    use crate::sequence_extension::ChromaFormat;

    /// A field-height frame whose every luma sample equals `value`.
    fn flat_field(width: usize, height: usize, value: u8) -> FrameBuffer {
        let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
        for y in 0..height {
            for x in 0..width {
                f.y.put_sample(x, y, value);
            }
        }
        f
    }

    /// A full-height frame whose luma sample equals `2·row + parity-ish`
    /// encoding so even / odd lines are distinguishable.
    fn full_frame_row_coded(width: usize, height: usize) -> FrameBuffer {
        let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
        for y in 0..height {
            for x in 0..width {
                f.y.put_sample(x, y, y as u8);
            }
        }
        f
    }

    #[test]
    fn extract_field_recovers_even_and_odd_lines() {
        // A 4×8 frame whose luma row r == r. Top field (even lines) must
        // be 0,2,4,6; bottom field (odd lines) 1,3,5,7 (§3.131 / §3.13).
        let frame = full_frame_row_coded(4, 8);
        let top = extract_field(&frame, PictureStructure::TopField);
        let bottom = extract_field(&frame, PictureStructure::BottomField);
        // Visible field extent is 4×4 (the plane storage is
        // macroblock-aligned and larger).
        assert_eq!((top.width, top.height), (4, 4));
        for r in 0..4u8 {
            assert_eq!(top.y.get(0, r as usize), Some(2 * r));
            assert_eq!(bottom.y.get(0, r as usize), Some(2 * r + 1));
        }
    }

    #[test]
    fn extract_field_round_trips_overhang_rows_of_a_field_assembled_frame() {
        // The r413 non-multiple-of-32 case end-to-end: 24-line fields
        // (storage 32 = the field's own §6.3.3 macroblock grid)
        // assemble into a 48-line frame with 64 storage rows, and
        // extract_field must recover each field's **entire storage**,
        // overhang rows included, so a §7.6.2.1 second-field synthetic
        // reference carries the same reference material a conforming
        // decoder holds.
        let cf = crate::sequence_extension::ChromaFormat::Yuv420;
        let mut top = FrameBuffer::new(16, 24, cf);
        let mut bottom = FrameBuffer::new(16, 24, cf);
        for y in 0..32 {
            for x in 0..16 {
                top.y.put_sample(x, y, 60 + y as u8);
                bottom.y.put_sample(x, y, 160 + y as u8);
            }
        }
        let frame = crate::frame_assembly::assemble_frame_from_fields(&top, &bottom).unwrap();

        let top_back = extract_field(&frame, PictureStructure::TopField);
        let bottom_back = extract_field(&frame, PictureStructure::BottomField);
        assert_eq!((top_back.width, top_back.height), (16, 24));
        assert_eq!(top_back.y.height(), 32, "extracted field keeps its grid");
        for r in 0..32 {
            assert_eq!(
                top_back.y.get(0, r),
                Some(60 + r as u8),
                "top field storage row {r}"
            );
            assert_eq!(
                bottom_back.y.get(0, r),
                Some(160 + r as u8),
                "bottom field storage row {r}"
            );
        }
    }

    #[test]
    fn second_p_field_reference_pairs_current_first_with_prev_opposite() {
        // §7.6.2.1 / Figures 7-7, 7-8: when the second P field is the
        // BOTTOM field, the synthetic reference frame must carry the
        // just-decoded TOP first field (value 99) on its even lines and
        // the previous frame's BOTTOM field on its odd lines. The
        // previous frame's row r == r, so its bottom field lines are
        // 1,3,5,7.
        let first_field = flat_field(4, 4, 99); // current frame's top field
        let prev = full_frame_row_coded(4, 8);
        let synthetic =
            reference_frame_for_second_p_field(&first_field, PictureStructure::TopField, &prev)
                .unwrap();
        // Visible frame extent is 4×8 (the plane storage is
        // macroblock-aligned and larger).
        assert_eq!((synthetic.width, synthetic.height), (4, 8));
        for r in 0..4u8 {
            // Even (top reference field) = current first field (99).
            assert_eq!(synthetic.y.get(0, (2 * r) as usize), Some(99));
            // Odd (bottom reference field) = previous frame's odd rows.
            assert_eq!(synthetic.y.get(0, (2 * r + 1) as usize), Some(2 * r + 1));
        }
    }

    #[test]
    fn second_p_field_reference_bottom_first_field() {
        // When the first field is the BOTTOM field, it sits on the odd
        // lines and the previous frame's TOP field (even rows 0,2,4,6) on
        // the even lines.
        let first_field = flat_field(4, 4, 77); // current frame's bottom field
        let prev = full_frame_row_coded(4, 8);
        let synthetic =
            reference_frame_for_second_p_field(&first_field, PictureStructure::BottomField, &prev)
                .unwrap();
        for r in 0..4u8 {
            assert_eq!(synthetic.y.get(0, (2 * r) as usize), Some(2 * r)); // prev top
            assert_eq!(synthetic.y.get(0, (2 * r + 1) as usize), Some(77)); // current bottom
        }
    }

    #[test]
    fn field_geometry_halves_height_and_forces_field_dct() {
        let base = IntraPictureParams {
            width: 16,
            height: 32,
            chroma_format: ChromaFormat::Yuv420,
            frame_pred_frame_dct: true,
            intra_dc_precision: 0,
            intra_vlc_format: false,
            alternate_scan: false,
            progressive_sequence: false, // interlaced: field pictures exist
            q_scale_type: false,
        };
        let ext = PictureCodingExtension {
            f_code_fwd_horiz: 15,
            f_code_fwd_vert: 15,
            f_code_bwd_horiz: 15,
            f_code_bwd_vert: 15,
            intra_dc_precision: 2,
            picture_structure: PictureStructure::TopField,
            top_field_first: true,
            frame_pred_frame_dct: false,
            concealment_motion_vectors: false,
            q_scale_type: true,
            intra_vlc_format: true,
            alternate_scan: true,
            repeat_first_field: false,
            chroma_420_type: false,
            progressive_frame: false,
            composite_display_flag: false,
        };
        let geom = field_geometry(base, &ext);
        assert_eq!(geom.height, 16, "field height is half the frame height");
        assert!(!geom.frame_pred_frame_dct, "field picture forbids it");
        // The §6.2.3.1 DCT-context flags come from the picture coding ext.
        assert_eq!(geom.intra_dc_precision, 2);
        assert!(geom.intra_vlc_format);
        assert!(geom.alternate_scan);
        assert!(geom.q_scale_type);
        assert_eq!(geom.width, 16, "width is unchanged");
    }

    // ---- display_indices_from_temporal_references / verify_display_order ----

    #[test]
    fn display_indices_single_gop_passes_through_temporal_reference() {
        // Coded order I P B B P B B with temporal_references
        // 0 3 1 2 6 4 5 (the §6.1.1.11 worked example, GOP-relative).
        let coded = [0u16, 3, 1, 2, 6, 4, 5];
        let idx = display_indices_from_temporal_references(&coded);
        // Single GOP: index == temporal_reference.
        assert_eq!(idx, vec![0, 3, 1, 2, 6, 4, 5]);
    }

    #[test]
    fn display_indices_accumulate_across_a_gop_reset() {
        // Two GOPs: first GOP trefs 0 2 1 (max display index 2), second
        // GOP resets to 0: trefs 0 2 1. The second GOP's base is 3.
        let coded = [0u16, 2, 1, 0, 2, 1];
        let idx = display_indices_from_temporal_references(&coded);
        assert_eq!(idx, vec![0, 2, 1, 3, 5, 4]);
    }

    #[test]
    fn display_indices_leading_zero_is_not_a_reset() {
        // The very first frame's tref 0 must not be treated as a GOP
        // reset; the base stays 0.
        let coded = [0u16, 1, 2];
        let idx = display_indices_from_temporal_references(&coded);
        assert_eq!(idx, vec![0, 1, 2]);
    }

    #[test]
    fn display_indices_consecutive_zero_anchors_each_start_a_gop() {
        // I-only stream, one I per GOP, each tref 0. Every 0 after the
        // first that follows a non-zero is a reset; but here there is no
        // non-zero between them, so they share a GOP base (degenerate but
        // consistent — an all-I, all-tref-0 stream maps to base 0 for
        // every frame). The structural and tref reorders agree (identity).
        let coded = [0u16, 0, 0];
        let idx = display_indices_from_temporal_references(&coded);
        assert_eq!(idx, vec![0, 0, 0]);
    }

    #[test]
    fn verify_display_order_accepts_the_6_1_1_11_example() {
        // Coded order: I(0) P(3) B(1) B(2) P(6) B(4) B(5).
        let coded = [0u16, 3, 1, 2, 6, 4, 5];
        // Display order: I(0) B(1) B(2) P(3) B(4) B(5) P(6).
        let display = [0u16, 1, 2, 3, 4, 5, 6];
        assert!(verify_display_order(&coded, &display).is_ok());
    }

    #[test]
    fn verify_display_order_accepts_two_gops() {
        let coded = [0u16, 2, 1, 0, 2, 1];
        // GOP1 display 0,1,2 then GOP2 display 0,1,2.
        let display = [0u16, 1, 2, 0, 1, 2];
        assert!(verify_display_order(&coded, &display).is_ok());
    }

    #[test]
    fn verify_display_order_rejects_a_wrong_permutation() {
        let coded = [0u16, 3, 1, 2, 6, 4, 5];
        // Swap the two anchors out of order in display.
        let bad = [0u16, 1, 2, 6, 4, 5, 3];
        assert!(matches!(
            verify_display_order(&coded, &bad),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn verify_display_order_rejects_length_mismatch() {
        assert!(matches!(
            verify_display_order(&[0u16, 1], &[0u16]),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn verify_display_order_rejects_non_permutation() {
        // display has a tref the coded order never produced.
        assert!(matches!(
            verify_display_order(&[0u16, 1, 2], &[0u16, 1, 9]),
            Err(Error::InvalidBitstream(_))
        ));
    }
}
