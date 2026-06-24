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
//! ## Scope
//!
//! Frame pictures only (`picture_structure == Frame`). Field-picture
//! pairs (`TopField` + `BottomField` reconstructing into one frame) are
//! a later milestone — a field picture is reported through
//! [`crate::Error::NotImplemented`]. The §6.2.3 extension family beyond
//! `sequence_extension()` / `picture_coding_extension()`
//! (`quant_matrix_extension()`, the display / scalable extensions) is
//! skipped over by the start-code scan; downloadable quantiser matrices
//! and the scalable layers are threaded by a later round. MPEG-1
//! streams (no `sequence_extension()` / `picture_coding_extension()`)
//! are a later milestone too — this driver requires the MPEG-2
//! extensions for the geometry and f_codes.

use crate::frame_assembly::{decode_intra_picture, FrameBuffer, IntraPictureParams};
use crate::inter_reconstruction::ReferenceFrames;
use crate::picture_header::{
    Mpeg2PictureHeader, PictureCodingExtension, PictureCodingType, PictureStructure,
    PICTURE_START_CODE,
};
use crate::picture_reconstruction::{decode_inter_picture, PicturePredictionParams};
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
/// * [`Error::NotImplemented`] for a field picture
///   (`picture_structure != Frame`).
/// * [`Error::ShortHeader`] on truncation.
pub fn decode_video_sequence(stream: &[u8]) -> Result<Vec<DecodedFrame>> {
    // The geometry is established by the leading sequence layer and
    // re-established at every repeat / new `sequence_header()` encountered
    // before a picture, so a multi-sequence stream whose geometry changes
    // mid-stream tracks the new sizes (§6.1.1.6: a repeat sequence header
    // may legally restate the parameters).
    let mut geometry = sequence_geometry(&parse_leading_sequence(stream)?);

    let mut reorder = ReorderBuffer::new();
    let mut output: Vec<DecodedFrame> = Vec::new();

    // The two running I/P anchors, newest-last. `forward_anchor` is the
    // older of the pair (the past reference); `backward_anchor` is the
    // newer (the future reference for B-frames, and the forward
    // reference for the next P-frame).
    let mut forward_anchor: Option<FrameBuffer> = None;
    let mut backward_anchor: Option<FrameBuffer> = None;

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
            geometry = sequence_geometry(&Mpeg2Sequence::from_buf(&stream[code_start..])?);
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

        let (header, ext) = Mpeg2PictureHeader::parse_with_extension(picture_region)?;

        if ext.picture_structure != PictureStructure::Frame {
            return Err(Error::NotImplemented);
        }

        // The slice region begins after the picture header + extensions:
        // the per-picture drivers find the first slice_start_code
        // themselves, so we hand them the whole picture region.
        let coded = reconstruct_picture(
            picture_region,
            &header,
            &ext,
            geometry,
            forward_anchor.as_ref(),
            backward_anchor.as_ref(),
        )?;

        // §6.1.1.11 reorder + §7.6 reference rotation.
        match header.picture_coding_type {
            PictureCodingType::Bidirectional => {
                // B-frames are displayed immediately, in coded order, and
                // never become a reference. They emit before the held-back
                // anchor.
                reorder.push_b(coded.clone(), &mut output);
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
fn parse_leading_sequence(stream: &[u8]) -> Result<Mpeg2Sequence> {
    let Some(rel) = find_start_code(stream, |code| code == SEQUENCE_HEADER_CODE) else {
        return Err(Error::InvalidBitstream(
            "video_sequence(): missing leading sequence_header_code 0x000001B3 (§6.2.2)",
        ));
    };
    Mpeg2Sequence::from_buf(&stream[rel..])
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
    }
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
            let (frame, _placed) = decode_intra_picture(picture_region, geometry)?;
            frame
        }
        PictureCodingType::Predictive => {
            // §7.6: a P-frame predicts from the latest decoded I/P anchor.
            let forward = backward_anchor.ok_or(Error::InvalidBitstream(
                "§6.1.1.11: P-picture before any I/P anchor exists (no forward reference)",
            ))?;
            let params = inter_params(header, ext, geometry);
            let (frame, _placed) = decode_inter_picture(
                picture_region,
                params,
                ReferenceFrames::forward_only(forward),
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
            let (frame, _placed) = decode_inter_picture(
                picture_region,
                params,
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
}
