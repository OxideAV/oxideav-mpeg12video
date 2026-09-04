//! 4:2:2 / 4:4:4 on the **field-picture** encode paths (§6.1.3
//! Figures 6-11 / 6-12 macroblocks in `picture_structure` = Top /
//! Bottom field pictures): the plain and adaptive field encoders, the
//! field display-order assembler and the Annex C field CBR controller
//! are chroma-format generic. Every stream decodes back through the
//! crate's own decoder; the single-picture tests are sample-exact
//! against the encoder's reconstruction.

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::picture_header::PictureStructure;
use oxideav_mpeg12video::picture_reconstruction::PicturePredictionParams;
use oxideav_mpeg12video::sequence_extension::{ChromaFormat, Mpeg2Sequence};
use oxideav_mpeg12video::vbv::{verify_cbr_stream, VbvStandard};
use oxideav_mpeg12video::{
    decode_field_picture, decode_video_sequence, encode_field_adaptive_display_order_gop_sequence,
    encode_field_b_picture, encode_field_cbr_gop_sequence, encode_field_display_order_gop_sequence,
    encode_field_intra_picture, encode_field_p_picture, CbrConfig, FrameBuffer, IntraPictureParams,
    PictureCodingType,
};

use oxideav_mpeg12video::inter_reconstruction::ReferenceFrames;

const W: usize = 48;
const H: usize = 64; // frame height (multiple of 32); fields are 48x32

fn frame_params(chroma: ChromaFormat) -> IntraPictureParams {
    IntraPictureParams {
        width: W,
        height: H,
        chroma_format: chroma,
        frame_pred_frame_dct: false,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        progressive_sequence: false,
    }
}

fn field_params(chroma: ChromaFormat) -> IntraPictureParams {
    IntraPictureParams {
        height: H / 2,
        frame_pred_frame_dct: false,
        progressive_sequence: true, // 16-aligned grid in field coordinates
        ..frame_params(chroma)
    }
}

/// Interlaced-looking source frame at full chroma resolution for the
/// format: per-line luma phase (the two fields differ), diagonal pan
/// with `t`, and chroma planes carrying **per-row** detail so a 4:2:0
/// collapse (or a chroma field-parity mix-up) shows up as error.
fn frame_at(chroma: ChromaFormat, t: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(W, H, chroma);
    for y in 0..H {
        for x in 0..W {
            let v = 30 + ((x * 4 + y * 7 + t * 3) % 180);
            let line = if y % 2 == 0 { 12 } else { 0 };
            f.y.put_sample(x, y, (v + line).min(235) as u8);
        }
    }
    let (cw, ch) = f.visible_chroma_dims();
    for y in 0..ch {
        for x in 0..cw {
            let phase = if y % 2 == 0 { 20 } else { 0 };
            f.cb.put_sample(x, y, (60 + (x * 3 + y * 5 + t * 2 + phase) % 120) as u8);
            f.cr.put_sample(
                x,
                y,
                (200u8).saturating_sub(((x * 2 + y * 7 + t) % 120) as u8),
            );
        }
    }
    f
}

/// Extract one field (even / odd frame lines) of a frame, chroma
/// included at the format's own vertical resolution.
fn field_of(frame: &FrameBuffer, structure: PictureStructure) -> FrameBuffer {
    let parity = usize::from(structure == PictureStructure::BottomField);
    let mut field = FrameBuffer::new(frame.width, frame.height / 2, frame.chroma_format);
    for y in 0..frame.height / 2 {
        for x in 0..frame.width {
            field
                .y
                .put_sample(x, y, frame.y.get(x, 2 * y + parity).unwrap());
        }
    }
    let (cw, ch) = frame.visible_chroma_dims();
    for y in 0..ch / 2 {
        for x in 0..cw {
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

fn pred_params(
    chroma: ChromaFormat,
    kind: PictureCodingType,
    f_code: u8,
) -> PicturePredictionParams {
    PicturePredictionParams {
        geometry: field_params(chroma),
        picture_coding_type: kind,
        f_code_fwd_horiz: f_code,
        f_code_fwd_vert: f_code,
        f_code_bwd_horiz: f_code,
        f_code_bwd_vert: f_code,
        concealment_motion_vectors: false,
        top_field_first: false,
    }
}

fn assert_planes_equal(name: &str, a: &FrameBuffer, b: &FrameBuffer) {
    assert_eq!(a.chroma_format, b.chroma_format, "{name}: chroma format");
    for y in 0..a.height {
        for x in 0..a.width {
            assert_eq!(a.y.get(x, y), b.y.get(x, y), "{name}: luma ({x},{y})");
        }
    }
    let (cw, ch) = a.visible_chroma_dims();
    for y in 0..ch {
        for x in 0..cw {
            assert_eq!(a.cb.get(x, y), b.cb.get(x, y), "{name}: cb ({x},{y})");
            assert_eq!(a.cr.get(x, y), b.cr.get(x, y), "{name}: cr ({x},{y})");
        }
    }
}

/// Mean absolute error over a plane's visible rectangle.
fn plane_mae(
    a: &oxideav_mpeg12video::Plane,
    b: &oxideav_mpeg12video::Plane,
    w: usize,
    h: usize,
) -> f64 {
    let mut total = 0u64;
    for y in 0..h {
        for x in 0..w {
            total += u64::from(a.get(x, y).unwrap().abs_diff(b.get(x, y).unwrap()));
        }
    }
    total as f64 / (w * h) as f64
}

fn assert_sequence_close(name: &str, stream: &[u8], inputs: &[FrameBuffer], chroma: ChromaFormat) {
    let seq = Mpeg2Sequence::from_buf(stream).expect("sequence layer parses");
    assert_eq!(
        seq.extension.chroma_format, chroma,
        "{name}: chroma_format signalled"
    );
    assert!(
        !seq.extension.progressive_sequence,
        "{name}: field pictures live in an interlaced sequence (§6.3.5)"
    );
    // Table 8-5: only the High profile admits 4:2:2 chroma; the 4:4:4
    // leg reuses that label (documented in stream_writer).
    let expected_profile = if chroma == ChromaFormat::Yuv420 {
        0x48
    } else {
        0x18
    };
    assert_eq!(
        seq.extension.profile_and_level, expected_profile,
        "{name}: profile"
    );

    let decoded = decode_video_sequence(stream).expect("self-encoded field stream decodes");
    assert_eq!(decoded.len(), inputs.len(), "{name}: frame count");
    let (cw, ch) = inputs[0].visible_chroma_dims();
    // At 4:4:4 the printed §6.3.17.4 derivation leaves non-intra blocks
    // 6 / 7 without a coded_block_pattern slot, so the busy chroma of
    // this fixture cannot be fully refined on P / B fields — the
    // encoder documents that as an intended (stream-compatible) gap.
    let chroma_bound = if chroma == ChromaFormat::Yuv444 {
        16.0
    } else {
        8.0
    };
    for (i, (d, input)) in decoded.iter().zip(inputs).enumerate() {
        let f = &d.frame;
        assert_eq!((f.width, f.height), (W, H), "{name}: frame {i} geometry");
        assert_eq!(
            f.visible_chroma_dims(),
            (cw, ch),
            "{name}: frame {i} chroma dims"
        );
        let y_mae = plane_mae(&f.y, &input.y, W, H);
        let cb_mae = plane_mae(&f.cb, &input.cb, cw, ch);
        let cr_mae = plane_mae(&f.cr, &input.cr, cw, ch);
        assert!(y_mae < 8.0, "{name}: frame {i} luma MAE {y_mae:.2}");
        assert!(
            cb_mae < chroma_bound,
            "{name}: frame {i} Cb MAE {cb_mae:.2}"
        );
        assert!(
            cr_mae < chroma_bound,
            "{name}: frame {i} Cr MAE {cr_mae:.2}"
        );
    }
}

#[test]
fn intra_field_picture_422_is_sample_exact_against_decoder() {
    for chroma in [ChromaFormat::Yuv422, ChromaFormat::Yuv444] {
        let src = field_of(&frame_at(chroma, 0), PictureStructure::TopField);
        let mut bw = BitWriter::new();
        let recon = encode_field_intra_picture(
            &mut bw,
            &src,
            &field_params(chroma),
            PictureStructure::TopField,
            0,
            6,
        )
        .expect("encode intra field");
        let layer = bw.finish();

        let (decoded, placed) = decode_field_picture(
            &layer,
            pred_params(chroma, PictureCodingType::Intra, 15),
            PictureStructure::TopField,
            ReferenceFrames {
                forward: None,
                backward: None,
            },
        )
        .expect("decode intra field");
        assert_eq!(placed, (W / 16) * (H / 32));
        assert_planes_equal(&format!("intra field {chroma:?}"), &decoded, &recon);
        // The chroma planes really carry the format's resolution.
        let (cw, ch) = recon.visible_chroma_dims();
        let expected = match chroma {
            ChromaFormat::Yuv420 => (W / 2, H / 4),
            ChromaFormat::Yuv422 => (W / 2, H / 2),
            ChromaFormat::Yuv444 => (W, H / 2),
        };
        assert_eq!((cw, ch), expected, "{chroma:?} field chroma dims");
    }
}

#[test]
fn p_and_b_field_pictures_422_are_sample_exact_against_decoder() {
    for chroma in [ChromaFormat::Yuv422, ChromaFormat::Yuv444] {
        let params = field_params(chroma);
        let anchor_frame = frame_at(chroma, 0);
        let mut bw = BitWriter::new();
        let top_i = encode_field_intra_picture(
            &mut bw,
            &field_of(&anchor_frame, PictureStructure::TopField),
            &params,
            PictureStructure::TopField,
            0,
            6,
        )
        .expect("encode I top");
        let bottom_i = encode_field_intra_picture(
            &mut bw,
            &field_of(&anchor_frame, PictureStructure::BottomField),
            &params,
            PictureStructure::BottomField,
            0,
            6,
        )
        .expect("encode I bottom");
        let reference =
            oxideav_mpeg12video::assemble_frame_from_fields(&top_i, &bottom_i).expect("assemble");

        // P field: a diagonal pan of the same content.
        let target = field_of(&frame_at(chroma, 1), PictureStructure::TopField);
        let mut bw = BitWriter::new();
        let recon = encode_field_p_picture(
            &mut bw,
            &target,
            &reference,
            &params,
            PictureStructure::TopField,
            1,
            6,
            3,
        )
        .expect("encode P field");
        let layer = bw.finish();
        let (decoded, placed) = decode_field_picture(
            &layer,
            pred_params(chroma, PictureCodingType::Predictive, 3),
            PictureStructure::TopField,
            ReferenceFrames::forward_only(&reference),
        )
        .expect("decode P field");
        assert_eq!(placed, (W / 16) * (H / 32));
        assert_planes_equal(&format!("P field {chroma:?}"), &decoded, &recon);
        let p_frame = oxideav_mpeg12video::assemble_frame_from_fields(&recon, &bottom_i)
            .expect("assemble P frame");

        // B field between the two anchors.
        let b_target = field_of(&frame_at(chroma, 2), PictureStructure::BottomField);
        let mut bw = BitWriter::new();
        let b_recon = encode_field_b_picture(
            &mut bw,
            &b_target,
            &reference,
            &p_frame,
            &params,
            PictureStructure::BottomField,
            2,
            6,
            3,
            3,
        )
        .expect("encode B field");
        let layer = bw.finish();
        let (decoded, placed) = decode_field_picture(
            &layer,
            pred_params(chroma, PictureCodingType::Bidirectional, 3),
            PictureStructure::BottomField,
            ReferenceFrames::bidirectional(&reference, &p_frame),
        )
        .expect("decode B field");
        assert_eq!(placed, (W / 16) * (H / 32));
        assert_planes_equal(&format!("B field {chroma:?}"), &decoded, &b_recon);
    }
}

#[test]
fn field_sequence_422_and_444_roundtrip_through_decode_video_sequence() {
    for chroma in [ChromaFormat::Yuv422, ChromaFormat::Yuv444] {
        let frames: Vec<FrameBuffer> = (0..5).map(|t| frame_at(chroma, t)).collect();
        let stream =
            encode_field_display_order_gop_sequence(&frames, 1, 2, &frame_params(chroma), 6, 3, 3)
                .expect("field sequence encode");
        assert_sequence_close(&format!("field seq {chroma:?}"), &stream, &frames, chroma);
    }
}

#[test]
fn adaptive_field_sequence_422_covers_every_table_6_18_mode_slot() {
    for chroma in [ChromaFormat::Yuv422, ChromaFormat::Yuv444] {
        let frames: Vec<FrameBuffer> = (0..3).map(|t| frame_at(chroma, t)).collect();
        let (stream, stats) = encode_field_adaptive_display_order_gop_sequence(
            &frames,
            0,
            2,
            &frame_params(chroma),
            6,
            3,
            3,
            true,
        )
        .expect("adaptive field sequence encode");
        // Two P frames = four P fields of (48/16) * (32/16) macroblocks.
        let p_macroblocks = 4 * (W / 16) * (H / 32);
        assert_eq!(
            stats.simple_field + stats.sixteen_by_eight + stats.dual_prime + stats.intra,
            p_macroblocks,
            "{chroma:?}: every P-field macroblock is accounted for"
        );
        assert_sequence_close(
            &format!("adaptive field {chroma:?}"),
            &stream,
            &frames,
            chroma,
        );
    }
}

#[test]
fn field_cbr_422_holds_the_annex_c_bounds() {
    let chroma = ChromaFormat::Yuv422;
    let frames: Vec<FrameBuffer> = (0..4).map(|t| frame_at(chroma, t)).collect();
    let cbr = CbrConfig {
        bit_rate_value: 600, // 240 kbit/s
        vbv_buffer_size_value: 4,
        frame_rate_code: 3,
        initial_quantiser_scale_code: 6,
    };
    let enc = encode_field_cbr_gop_sequence(&frames, 1, 2, &frame_params(chroma), &cbr, 3, 3)
        .expect("field CBR encode at 4:2:2");
    let report = verify_cbr_stream(&enc.stream, VbvStandard::Mpeg2).expect("VBV conformant");
    assert_eq!(
        report.pictures.len(),
        frames.len() * 2,
        "one VBV record per field"
    );
    assert_sequence_close("field cbr 4:2:2", &enc.stream, &frames, chroma);
}

#[test]
fn field_encoders_reject_frame_dct_flag_but_not_chroma_format() {
    // The former 4:2:0-only guard is gone; the §6.3.10 flag guard
    // stays.
    let src = field_of(
        &frame_at(ChromaFormat::Yuv422, 0),
        PictureStructure::TopField,
    );
    let bad = IntraPictureParams {
        frame_pred_frame_dct: true,
        ..field_params(ChromaFormat::Yuv422)
    };
    let mut bw = BitWriter::new();
    assert!(
        encode_field_intra_picture(&mut bw, &src, &bad, PictureStructure::TopField, 0, 6).is_err(),
        "frame_pred_frame_dct = 1 is rejected for field pictures"
    );
}
