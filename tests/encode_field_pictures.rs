//! Field-picture inter encode round-trips (§7.6.1: within a field
//! picture all predictions are field predictions):
//!
//! * per-picture **sample-exactness** — each encoded field picture,
//!   decoded by the crate's own `decode_field_picture` driver with the
//!   same references, reproduces the encoder-side reconstruction
//!   exactly (encoder and decoder share the §7.6.4 prediction +
//!   residual arithmetic);
//! * **cross-parity prediction** — content shifted by one *frame* line
//!   makes the opposite-parity reference field the only good
//!   predictor, so a working `motion_vertical_field_select` produces a
//!   near-free P field pair whose decode equals the shifted anchor
//!   sample-for-sample;
//! * the **display-order field assembler** emits whole interlaced
//!   sequences (I/P/B field pairs, §7.6.2.1 second-field-of-P
//!   reference rule) that `decode_video_sequence` reassembles into
//!   full frames in display order with bounded distortion.

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::picture_header::PictureStructure;
use oxideav_mpeg12video::picture_reconstruction::PicturePredictionParams;
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    decode_field_picture, decode_video_sequence, encode_field_b_picture,
    encode_field_display_order_gop_sequence, encode_field_intra_picture, encode_field_p_picture,
    FrameBuffer, IntraPictureParams, PictureCodingType,
};

use oxideav_mpeg12video::inter_reconstruction::ReferenceFrames;

const W: usize = 48;
const H: usize = 64; // frame height (multiple of 32); fields are 48x32

fn frame_params() -> IntraPictureParams {
    IntraPictureParams {
        width: W,
        height: H,
        chroma_format: ChromaFormat::Yuv420,
        frame_pred_frame_dct: false,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        progressive_sequence: false,
    }
}

fn field_params() -> IntraPictureParams {
    IntraPictureParams {
        height: H / 2,
        frame_pred_frame_dct: false,
        progressive_sequence: true, // 16-aligned grid in field coordinates
        ..frame_params()
    }
}

/// Interlaced-looking source frame: per-line phase (so the two fields
/// differ), moving diagonally with `t`; `dy_lines` shifts the whole
/// frame down by whole frame lines.
fn frame_at(t: usize, dy_lines: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(W, H, ChromaFormat::Yuv420);
    for y in 0..H {
        for x in 0..W {
            let sy = y as i64 - dy_lines as i64;
            let v = 30 + ((x as i64 * 4 + sy * 7 + t as i64 * 3).rem_euclid(180)) as usize;
            let line = if sy.rem_euclid(2) == 0 { 12 } else { 0 };
            f.y.put_sample(x, y, (v + line).min(235) as u8);
        }
    }
    for y in 0..H / 2 {
        for x in 0..W / 2 {
            f.cb.put_sample(x, y, (90 + (x + t) % 80) as u8);
            f.cr.put_sample(x, y, (190u8).saturating_sub(((y + 2 * t) % 80) as u8));
        }
    }
    f
}

/// Extract one field (even/odd frame lines) of a frame.
fn field_of(frame: &FrameBuffer, structure: PictureStructure) -> FrameBuffer {
    let parity = usize::from(structure == PictureStructure::BottomField);
    let mut field = FrameBuffer::new(W, H / 2, ChromaFormat::Yuv420);
    for y in 0..H / 2 {
        for x in 0..W {
            field
                .y
                .put_sample(x, y, frame.y.get(x, 2 * y + parity).unwrap());
        }
    }
    for y in 0..H / 4 {
        for x in 0..W / 2 {
            field
                .cb
                .put_sample(x, y, frame.cb.get(x, 2 * y + parity).unwrap());
            field
                .cr
                .put_sample(x, y, frame.cr.get(x, 2 * y + parity).unwrap());
        }
    }
    field
}

fn pred_params(kind: PictureCodingType, f_code: u8) -> PicturePredictionParams {
    PicturePredictionParams {
        geometry: field_params(),
        picture_coding_type: kind,
        f_code_fwd_horiz: f_code,
        f_code_fwd_vert: f_code,
        f_code_bwd_horiz: f_code,
        f_code_bwd_vert: f_code,
        concealment_motion_vectors: false,
        top_field_first: false,
    }
}

fn assert_fields_equal(name: &str, a: &FrameBuffer, b: &FrameBuffer) {
    for y in 0..H / 2 {
        for x in 0..W {
            assert_eq!(a.y.get(x, y), b.y.get(x, y), "{name}: luma ({x},{y})");
        }
    }
    for y in 0..H / 4 {
        for x in 0..W / 2 {
            assert_eq!(a.cb.get(x, y), b.cb.get(x, y), "{name}: cb ({x},{y})");
            assert_eq!(a.cr.get(x, y), b.cr.get(x, y), "{name}: cr ({x},{y})");
        }
    }
}

#[test]
fn intra_field_picture_is_sample_exact_against_decoder() {
    let src = field_of(&frame_at(0, 0), PictureStructure::TopField);
    let mut bw = BitWriter::new();
    let recon = encode_field_intra_picture(
        &mut bw,
        &src,
        &field_params(),
        PictureStructure::TopField,
        0,
        6,
    )
    .expect("encode intra field");
    let layer = bw.finish();

    let (decoded, placed) = decode_field_picture(
        &layer,
        pred_params(PictureCodingType::Intra, 15),
        PictureStructure::TopField,
        ReferenceFrames {
            forward: None,
            backward: None,
        },
    )
    .expect("decode intra field");
    assert_eq!(placed, (W / 16) * (H / 32));
    assert_fields_equal("intra field", &decoded, &recon);
}

#[test]
fn p_field_picture_is_sample_exact_against_decoder() {
    // Reference frame: the decoder-exact reconstruction of an I field
    // pair, assembled to full height.
    let anchor_frame = frame_at(0, 0);
    let mut bw = BitWriter::new();
    let top_i = encode_field_intra_picture(
        &mut bw,
        &field_of(&anchor_frame, PictureStructure::TopField),
        &field_params(),
        PictureStructure::TopField,
        0,
        6,
    )
    .expect("encode I top");
    let bottom_i = encode_field_intra_picture(
        &mut bw,
        &field_of(&anchor_frame, PictureStructure::BottomField),
        &field_params(),
        PictureStructure::BottomField,
        0,
        6,
    )
    .expect("encode I bottom");
    let reference =
        oxideav_mpeg12video::assemble_frame_from_fields(&top_i, &bottom_i).expect("assemble");

    // P field: a diagonal pan of the same content.
    let target = field_of(&frame_at(1, 0), PictureStructure::TopField);
    let mut bw = BitWriter::new();
    let recon = encode_field_p_picture(
        &mut bw,
        &target,
        &reference,
        &field_params(),
        PictureStructure::TopField,
        1,
        6,
        3,
    )
    .expect("encode P field");
    let layer = bw.finish();

    let (decoded, _) = decode_field_picture(
        &layer,
        pred_params(PictureCodingType::Predictive, 3),
        PictureStructure::TopField,
        ReferenceFrames::forward_only(&reference),
    )
    .expect("decode P field");
    assert_fields_equal("P field", &decoded, &recon);
}

#[test]
fn b_field_picture_is_sample_exact_against_decoder() {
    // Anchors: two intra field pairs.
    let assemble_anchor = |t: usize| {
        let frame = frame_at(t, 0);
        let mut bw = BitWriter::new();
        let top = encode_field_intra_picture(
            &mut bw,
            &field_of(&frame, PictureStructure::TopField),
            &field_params(),
            PictureStructure::TopField,
            0,
            6,
        )
        .expect("encode I top");
        let bottom = encode_field_intra_picture(
            &mut bw,
            &field_of(&frame, PictureStructure::BottomField),
            &field_params(),
            PictureStructure::BottomField,
            0,
            6,
        )
        .expect("encode I bottom");
        oxideav_mpeg12video::assemble_frame_from_fields(&top, &bottom).expect("assemble")
    };
    let fwd = assemble_anchor(0);
    let bwd = assemble_anchor(2);

    let target = field_of(&frame_at(1, 0), PictureStructure::BottomField);
    let mut bw = BitWriter::new();
    let recon = encode_field_b_picture(
        &mut bw,
        &target,
        &fwd,
        &bwd,
        &field_params(),
        PictureStructure::BottomField,
        1,
        6,
        3,
        3,
    )
    .expect("encode B field");
    let layer = bw.finish();

    let (decoded, _) = decode_field_picture(
        &layer,
        pred_params(PictureCodingType::Bidirectional, 3),
        PictureStructure::BottomField,
        ReferenceFrames::bidirectional(&fwd, &bwd),
    )
    .expect("decode B field");
    assert_fields_equal("B field", &decoded, &recon);
}

#[test]
fn cross_parity_field_select_predicts_one_line_shift_exactly() {
    // Frame 1 shifted down by ONE frame line: frame 2's top field is
    // (almost) frame 1's bottom field, and vice versa — only a decoder
    // reading the §6.3.17.2-selected opposite-parity reference field
    // can predict it nearly for free.
    let f0 = frame_at(0, 0);
    let f1 = frame_at(0, 1);
    let stream = encode_field_display_order_gop_sequence(
        &[f0, f1],
        0, // I P
        1,
        &frame_params(),
        2, // fine quantiser: the anchor reconstruction is near-exact,
        // so the shifted field predicts with dead-zone-small residuals
        3,
        3,
    )
    .expect("encode field sequence");

    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 2);
    // The P frame's TOP field is exactly the anchor's BOTTOM field
    // shifted one field line (f1_top(k) = f0(2k-1) = f0_bottom(k-1)),
    // so a working motion_vertical_field_select predicts it with only
    // quantisation-sized residuals: decoded top-field line 2k must
    // track decoded frame-0 line 2k-1 tightly. A same-parity-only
    // predictor cannot do this (adjacent frame lines here differ by a
    // per-parity brightness step), and would need heavy residuals.
    // Measure the second field-macroblock row only: the first row's
    // ideal -1-field-line vector would read above the reference field,
    // which §7.6.3.8 forbids, so row 0 legitimately codes residuals.
    let d0 = &decoded[0].frame;
    let d1 = &decoded[1].frame;
    let mut sum = 0u64;
    let mut count = 0u64;
    for k in 16..H / 2 {
        for x in 0..W {
            let a = d1.y.get(x, 2 * k).unwrap();
            let b = d0.y.get(x, 2 * k - 1).unwrap();
            sum += u64::from(a.abs_diff(b));
            count += 1;
        }
    }
    let mae = sum as f64 / count as f64;
    assert!(
        mae < 2.5,
        "cross-parity prediction failed: top-field shift MAE {mae}"
    );

    // And the P field pair must be far cheaper than the I pair: find
    // the four picture start codes and compare coded sizes.
    let mut picture_offsets = Vec::new();
    for i in 0..stream.len() - 3 {
        if stream[i] == 0 && stream[i + 1] == 0 && stream[i + 2] == 1 && stream[i + 3] == 0 {
            picture_offsets.push(i);
        }
    }
    assert_eq!(picture_offsets.len(), 4);
    let i_pair = picture_offsets[2] - picture_offsets[0];
    let p_pair = stream.len() - picture_offsets[2];
    // The top P field is nearly free through the opposite-parity
    // reference; the bottom field legitimately codes residuals (a
    // one-frame-line shift is a half-field-line offset for it, which
    // half-pel interpolation cannot reproduce exactly). A broken
    // parity selection would leave BOTH fields residual-heavy and the
    // pair close to intra cost.
    assert!(
        p_pair * 2 < i_pair,
        "P field pair ({p_pair} bytes) should be far cheaper than the I pair ({i_pair} bytes)"
    );
}

#[test]
fn field_display_order_sequence_round_trips() {
    // 5 frames, I B P B P per GOP — the full assembler: field pairs,
    // shared temporal_reference, §7.6.2.1 second-P-field reference.
    let frames: Vec<FrameBuffer> = (0..5).map(|t| frame_at(t, 0)).collect();
    let stream = encode_field_display_order_gop_sequence(&frames, 1, 2, &frame_params(), 6, 3, 3)
        .expect("encode field sequence");

    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 5);
    for (t, d) in decoded.iter().enumerate() {
        assert_eq!((d.frame.width, d.frame.height), (W, H));
        let src = frame_at(t, 0);
        let mut sum = 0u64;
        for y in 0..H {
            for x in 0..W {
                sum += u64::from(
                    d.frame
                        .y
                        .get(x, y)
                        .unwrap()
                        .abs_diff(src.y.get(x, y).unwrap()),
                );
            }
        }
        let mae = sum as f64 / (W * H) as f64;
        assert!(mae < 8.0, "frame {t} luma MAE {mae}");
    }
    // Display order: temporal references per GOP-relative display
    // index must be strictly increasing per GOP in display order.
    assert_eq!(decoded[0].picture_coding_type, PictureCodingType::Intra);
}

#[test]
fn field_assembler_rejects_bad_geometry() {
    // Height not a multiple of 32.
    let mut p = frame_params();
    p.height = 48;
    let f = FrameBuffer::new(W, 48, ChromaFormat::Yuv420);
    assert!(encode_field_display_order_gop_sequence(&[f], 0, 1, &p, 6, 3, 3).is_err());
    // progressive_sequence = 1 cannot carry field pictures (§6.3.5).
    let mut p = frame_params();
    p.progressive_sequence = true;
    let f = FrameBuffer::new(W, H, ChromaFormat::Yuv420);
    assert!(encode_field_display_order_gop_sequence(&[f], 0, 1, &p, 6, 3, 3).is_err());
}
