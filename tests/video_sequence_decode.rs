//! End-to-end §6.2.2 `video_sequence()` decode with §6.1.1.11
//! display-order reordering against a hand-built MPEG-2 elementary
//! stream.
//!
//! These chain the whole top-level pipeline the per-stage modules
//! previously only exposed piecemeal: a full elementary stream
//! (sequence_header + sequence_extension + an I / P / B picture run,
//! each picture_header + picture_coding_extension + a single-macroblock
//! slice) is decoded by [`decode_video_sequence`], which must reconstruct
//! every picture against the running reference anchors (§7.6) and emit
//! the frames in **display** order, not coded order (§6.1.1.11).
//!
//! Clean-room: only the ISO/IEC 13818-2 syntax (start codes, the §6.2.2
//! layer order, the VLC tables) is used; no external library source is
//! read. The encoded pictures are built bit-by-bit through the public
//! [`BitWriter`].

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::picture_header::PICTURE_START_CODE;
use oxideav_mpeg12video::sequence_extension::EXTENSION_START_CODE;
use oxideav_mpeg12video::sequence_header::SEQUENCE_HEADER_CODE;
use oxideav_mpeg12video::{decode_video_sequence, PictureCodingType};

/// A real 352×240 4:2:0 MPEG-2 elementary stream from an opaque
/// black-box encoder (its source is not read). Its single coded picture
/// is an I-picture.
const FIXTURE: &[u8] = include_bytes!("fixtures/ffmpeg-352x240-25fps.m2v");

const SEQUENCE_END_CODE: u32 = 0x0000_01B7;
/// Table B-14 EOB code (`10`, 2 bits) — the spec end-of-block marker.
const EOB: u32 = 0b10;

/// Build a 16×16, 4:2:0 `sequence_header()` (no quantiser matrices).
fn write_sequence_header(bw: &mut BitWriter) {
    bw.write_u32(SEQUENCE_HEADER_CODE, 32);
    bw.write_u32(16, 12); // horizontal_size_value
    bw.write_u32(16, 12); // vertical_size_value
    bw.write_u32(0b0001, 4); // aspect_ratio (square)
    bw.write_u32(0b0011, 4); // frame_rate_code (25 fps)
    bw.write_u32(2500, 18); // bit_rate_value (non-zero)
    bw.write_bit(true); // marker_bit
    bw.write_u32(112, 10); // vbv_buffer_size_value
    bw.write_bit(false); // constrained_parameters_flag
    bw.write_bit(false); // load_intra_quantiser_matrix
    bw.write_bit(false); // load_non_intra_quantiser_matrix
    bw.align_to_byte();
}

/// Build the §6.2.2.3 `sequence_extension()` for a 4:2:0 stream.
fn write_sequence_extension(bw: &mut BitWriter) {
    bw.write_u32(EXTENSION_START_CODE, 32);
    bw.write_u32(0b0001, 4); // Sequence Extension ID
    bw.write_u32(0x48, 8); // profile_and_level (Main@Main, any byte legal)
    bw.write_bit(false); // progressive_sequence
    bw.write_u32(0b01, 2); // chroma_format = 4:2:0
    bw.write_u32(0, 2); // horizontal_size_extension
    bw.write_u32(0, 2); // vertical_size_extension
    bw.write_u32(0, 12); // bit_rate_extension
    bw.write_bit(true); // marker_bit
    bw.write_u32(0, 8); // vbv_buffer_size_extension
    bw.write_bit(false); // low_delay
    bw.write_u32(0, 2); // frame_rate_extension_n
    bw.write_u32(0, 5); // frame_rate_extension_d
    bw.align_to_byte();
}

/// Build a `picture_header()` for the given coding type + temporal
/// reference. f_code arms are written for P/B per §6.2.3.
fn write_picture_header(
    bw: &mut BitWriter,
    temporal_reference: u32,
    coding_type: PictureCodingType,
) {
    bw.write_u32(PICTURE_START_CODE, 32);
    bw.write_u32(temporal_reference, 10);
    let ct_code = match coding_type {
        PictureCodingType::Intra => 0b001,
        PictureCodingType::Predictive => 0b010,
        PictureCodingType::Bidirectional => 0b011,
    };
    bw.write_u32(ct_code, 3);
    bw.write_u32(0xFFFF, 16); // vbv_delay
                              // full_pel_forward_vector + forward_f_code (P/B), backward (B).
    if matches!(
        coding_type,
        PictureCodingType::Predictive | PictureCodingType::Bidirectional
    ) {
        bw.write_bit(false); // full_pel_forward_vector
        bw.write_u32(0b111, 3); // forward_f_code (MPEG-1-style placeholder)
    }
    if coding_type == PictureCodingType::Bidirectional {
        bw.write_bit(false); // full_pel_backward_vector
        bw.write_u32(0b111, 3); // backward_f_code
    }
    bw.write_bit(false); // extra_bit_picture = 0
    bw.align_to_byte();
}

/// Build the §6.2.3.1 `picture_coding_extension()` with frame structure,
/// `frame_pred_frame_dct = 1`, and the given f_codes (15 = unused).
fn write_picture_coding_extension(bw: &mut BitWriter, f_fwd: u8, f_bwd: u8) {
    bw.write_u32(EXTENSION_START_CODE, 32);
    bw.write_u32(0b1000, 4); // Picture Coding Extension ID
    bw.write_u32(u32::from(f_fwd), 4); // f_code[0][0]
    bw.write_u32(u32::from(f_fwd), 4); // f_code[0][1]
    bw.write_u32(u32::from(f_bwd), 4); // f_code[1][0]
    bw.write_u32(u32::from(f_bwd), 4); // f_code[1][1]
    bw.write_u32(0, 2); // intra_dc_precision = 0
    bw.write_u32(0b11, 2); // picture_structure = Frame
    bw.write_bit(true); // top_field_first
    bw.write_bit(true); // frame_pred_frame_dct = 1
    bw.write_bit(false); // concealment_motion_vectors
    bw.write_bit(false); // q_scale_type
    bw.write_bit(false); // intra_vlc_format
    bw.write_bit(false); // alternate_scan
    bw.write_bit(false); // repeat_first_field
    bw.write_bit(false); // chroma_420_type
    bw.write_bit(false); // progressive_frame
    bw.write_bit(false); // composite_display_flag
    bw.align_to_byte();
}

/// Write the §6.3.16 slice header for `mb_row = 0`.
fn write_slice_header(bw: &mut BitWriter, q_scale: u8) {
    bw.write_u32(0x00_00_01, 24); // slice_start_code prefix
    bw.write_u32(1, 8); // slice_vertical_position = mb_row + 1
    bw.write_u32(u32::from(q_scale), 5);
    bw.write_bit(false); // extra_bit_slice = 0
}

/// Write a single intra macroblock whose six blocks are all `dct_dc_size
/// = 0` + immediate EOB. With a size-0 DC differential the §7.2.1 DC
/// predictor stays at its Table 7-2 reset value (128 for
/// intra_dc_precision = 0), so every reconstructed sample is 128 — a
/// flat 16×16 frame.
fn write_intra_macroblock(bw: &mut BitWriter) {
    bw.write_bit(true); // macroblock_address_increment = 1
    bw.write_bit(true); // macroblock_type "Intra" (Table B-2 `1`)
                        // 4 luma blocks: dct_dc_size_luminance = 0 → `100` (3 bits) + EOB.
    for _ in 0..4 {
        bw.write_u32(0b100, 3);
        bw.write_u32(EOB, 2);
    }
    // 2 chroma blocks: dct_dc_size_chrominance = 0 → `00` (2 bits) + EOB.
    for _ in 0..2 {
        bw.write_u32(0b00, 2);
        bw.write_u32(EOB, 2);
    }
}

/// Write a P macroblock "MC, Not Coded" (Table B-3 `001`) with a zero
/// forward motion vector and no coded blocks → a verbatim copy of the
/// forward reference.
fn write_p_copy_macroblock(bw: &mut BitWriter) {
    bw.write_bit(true); // macroblock_address_increment = 1
    bw.write_u32(0b001, 3); // macroblock_type "MC, Not Coded"
    bw.write_bit(true); // forward motion_code horiz = 0 (Table B-10 `1`)
    bw.write_bit(true); // forward motion_code vert = 0
}

/// Write a B macroblock "Interp, Not Coded" (Table B-4 `10`) with zero
/// forward + backward motion vectors → the `// 2` average of the two
/// references.
fn write_b_interp_macroblock(bw: &mut BitWriter) {
    bw.write_bit(true); // macroblock_address_increment = 1
    bw.write_u32(0b10, 2); // macroblock_type "Interp, Not Coded"
    bw.write_bit(true); // fwd motion_code horiz = 0
    bw.write_bit(true); // fwd motion_code vert = 0
    bw.write_bit(true); // bwd motion_code horiz = 0
    bw.write_bit(true); // bwd motion_code vert = 0
}

/// Append a picture's slice body (after the picture header + extension)
/// followed by byte alignment.
fn write_picture(
    out: &mut BitWriter,
    temporal_reference: u32,
    coding_type: PictureCodingType,
    f_fwd: u8,
    f_bwd: u8,
    body: impl Fn(&mut BitWriter),
) {
    write_picture_header(out, temporal_reference, coding_type);
    write_picture_coding_extension(out, f_fwd, f_bwd);
    write_slice_header(out, 8);
    body(out);
    out.align_to_byte_zero();
}

#[test]
fn decodes_i_p_b_run_in_display_order() {
    // Coded order: I(tr=0) P(tr=2) B(tr=1).
    // Display order: I(0) B(1) P(2).
    let mut bw = BitWriter::new();
    write_sequence_header(&mut bw);
    write_sequence_extension(&mut bw);
    // I-picture, tr = 0 (flat 128 frame).
    write_picture(&mut bw, 0, PictureCodingType::Intra, 15, 15, |b| {
        write_intra_macroblock(b)
    });
    // P-picture, tr = 2: zero-MV copy of the I anchor → flat 128.
    write_picture(&mut bw, 2, PictureCodingType::Predictive, 1, 15, |b| {
        write_p_copy_macroblock(b)
    });
    // B-picture, tr = 1: // 2 average of I (128) and P (128) → 128.
    write_picture(&mut bw, 1, PictureCodingType::Bidirectional, 1, 1, |b| {
        write_b_interp_macroblock(b)
    });
    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());

    let frames = decode_video_sequence(&stream).expect("video sequence decode");

    // Three frames, reordered into display order: I(0) B(1) P(2).
    assert_eq!(frames.len(), 3, "I + P + B decoded");
    assert_eq!(
        frames
            .iter()
            .map(|f| f.temporal_reference)
            .collect::<Vec<_>>(),
        vec![0, 1, 2],
        "display order temporal_references"
    );
    assert_eq!(frames[0].picture_coding_type, PictureCodingType::Intra);
    assert_eq!(
        frames[1].picture_coding_type,
        PictureCodingType::Bidirectional
    );
    assert_eq!(frames[2].picture_coding_type, PictureCodingType::Predictive);

    // Every frame is the flat-128 reconstruction (the I anchor copied
    // forward through the P, averaged through the B).
    for f in &frames {
        assert_eq!((f.frame.y.width(), f.frame.y.height()), (16, 16));
        for y in 0..16 {
            for x in 0..16 {
                assert_eq!(f.frame.y.get(x, y), Some(128), "luma flat 128");
            }
        }
        for y in 0..8 {
            for x in 0..8 {
                assert_eq!(f.frame.cb.get(x, y), Some(128));
                assert_eq!(f.frame.cr.get(x, y), Some(128));
            }
        }
    }
}

#[test]
fn p_picture_copies_distinct_intra_anchor() {
    // A two-picture stream: I then P. The I is a flat 128 frame; the P
    // copies it with a zero MV. We confirm the P frame is reconstructed
    // from the *decoded* I anchor (not a zero buffer), and that with no
    // B-frames the display order equals the coded order.
    let mut bw = BitWriter::new();
    write_sequence_header(&mut bw);
    write_sequence_extension(&mut bw);
    write_picture(&mut bw, 0, PictureCodingType::Intra, 15, 15, |b| {
        write_intra_macroblock(b)
    });
    write_picture(&mut bw, 1, PictureCodingType::Predictive, 1, 15, |b| {
        write_p_copy_macroblock(b)
    });
    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());

    let frames = decode_video_sequence(&stream).expect("video sequence decode");
    assert_eq!(frames.len(), 2);
    // No B-frames → coded order == display order.
    assert_eq!(
        frames
            .iter()
            .map(|f| f.temporal_reference)
            .collect::<Vec<_>>(),
        vec![0, 1]
    );
    // The P frame must equal the I anchor sample-for-sample.
    for y in 0..16 {
        for x in 0..16 {
            assert_eq!(frames[1].frame.y.get(x, y), frames[0].frame.y.get(x, y));
        }
    }
}

#[test]
fn decodes_real_fixture_single_i_picture_sequence() {
    // The driver must parse a real encoder's sequence layer and decode
    // its single I-picture to a full 352×240 frame in display order.
    let frames = decode_video_sequence(FIXTURE).expect("real fixture decode");
    assert_eq!(frames.len(), 1, "fixture has one coded picture");
    let f = &frames[0];
    assert_eq!(f.picture_coding_type, PictureCodingType::Intra);
    assert_eq!((f.frame.y.width(), f.frame.y.height()), (352, 240));
    assert_eq!((f.frame.cb.width(), f.frame.cb.height()), (176, 120));

    // The reconstructed luma must not be flat — a real I-picture carries
    // structured content, so at least two distinct sample values appear.
    let first = f.frame.y.get(0, 0).unwrap();
    let varied = (0..240)
        .flat_map(|y| (0..352).map(move |x| (x, y)))
        .any(|(x, y)| f.frame.y.get(x, y) != Some(first));
    assert!(varied, "real I-picture luma must not be flat");
}

#[test]
fn rejects_p_picture_before_any_anchor() {
    // A P-picture as the first coded picture has no forward reference.
    let mut bw = BitWriter::new();
    write_sequence_header(&mut bw);
    write_sequence_extension(&mut bw);
    write_picture(&mut bw, 0, PictureCodingType::Predictive, 1, 15, |b| {
        write_p_copy_macroblock(b)
    });
    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());

    let err = decode_video_sequence(&stream).unwrap_err();
    assert!(
        matches!(err, oxideav_mpeg12video::Error::InvalidBitstream(_)),
        "P before anchor must be rejected, got {err:?}"
    );
}

#[test]
fn rejects_field_picture_structure() {
    // Build an I-picture whose coding extension declares a field
    // structure (TopField); the frame-only driver rejects it.
    let mut bw = BitWriter::new();
    write_sequence_header(&mut bw);
    write_sequence_extension(&mut bw);
    write_picture_header(&mut bw, 0, PictureCodingType::Intra);
    // Custom coding extension with picture_structure = TopField (0b01).
    bw.write_u32(EXTENSION_START_CODE, 32);
    bw.write_u32(0b1000, 4);
    bw.write_u32(15, 4);
    bw.write_u32(15, 4);
    bw.write_u32(15, 4);
    bw.write_u32(15, 4);
    bw.write_u32(0, 2); // intra_dc_precision
    bw.write_u32(0b01, 2); // picture_structure = TopField
    bw.write_bit(true);
    bw.write_bit(false); // frame_pred_frame_dct = 0 (field structure)
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.align_to_byte();
    write_slice_header(&mut bw, 8);
    write_intra_macroblock(&mut bw);
    bw.align_to_byte_zero();
    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());

    let err = decode_video_sequence(&stream).unwrap_err();
    assert!(
        matches!(err, oxideav_mpeg12video::Error::NotImplemented),
        "field-picture structure must surface NotImplemented, got {err:?}"
    );
}
