//! Adaptive field-picture encode round-trips: per-macroblock Table
//! 6-18 mode selection between **simple field prediction**, **16×8
//! MC** (§7.6.7.3) and **dual-prime** (§7.6.3.6) — every emitted
//! stream decodes **sample-exactly** against the encoder's
//! decoder-driver reconstruction through `decode_video_sequence`.

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::frame_assembly::{
    assemble_frame_from_fields, FrameBuffer, IntraPictureParams,
};
use oxideav_mpeg12video::picture_header::PictureStructure;
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::stream_writer::{
    write_sequence_extension, write_sequence_header, SequenceHeaderParams, SEQUENCE_END_CODE,
};
use oxideav_mpeg12video::{
    decode_video_sequence, encode_field_adaptive_display_order_gop_sequence,
    encode_field_b_picture_adaptive, encode_field_intra_picture, encode_field_p_picture_adaptive,
    second_p_field_reference,
};

/// Field-geometry parameters (`height` = field height).
fn field_params(width: usize, field_height: usize) -> IntraPictureParams {
    IntraPictureParams {
        width,
        height: field_height,
        chroma_format: ChromaFormat::Yuv420,
        frame_pred_frame_dct: false,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        progressive_sequence: true, // field grid arithmetic (Ceil(h/16))
    }
}

/// A textured field-height buffer.
fn textured_field(width: usize, height: usize, seed: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            let v = 60 + ((x * 5 + y * 9 + seed * 3) % 150);
            f.y.put_sample(x, y, v as u8);
        }
    }
    for y in 0..height / 2 {
        for x in 0..width / 2 {
            f.cb.put_sample(x, y, 120);
            f.cr.put_sample(x, y, 136);
        }
    }
    f
}

/// Shift a field buffer horizontally with a **different shift for the
/// upper and lower half of every 16-line macroblock row** — motion one
/// field vector per macroblock cannot capture but two 16×8 vectors
/// can.
fn shift_split_regions(src: &FrameBuffer, upper_dx: i32, lower_dx: i32) -> FrameBuffer {
    let mut out = FrameBuffer::new(src.width, src.height, ChromaFormat::Yuv420);
    for y in 0..src.height {
        let dx = if (y % 16) < 8 { upper_dx } else { lower_dx };
        for x in 0..src.width {
            let sx = (x as i32 - dx).clamp(0, src.width as i32 - 1) as usize;
            out.y.put_sample(x, y, src.y.get(sx, y).unwrap());
        }
    }
    for y in 0..src.height / 2 {
        for x in 0..src.width / 2 {
            out.cb.put_sample(x, y, src.cb.get(x, y).unwrap());
            out.cr.put_sample(x, y, src.cr.get(x, y).unwrap());
        }
    }
    out
}

fn assert_visible_equal(
    decoded: &FrameBuffer,
    recon: &FrameBuffer,
    w: usize,
    h: usize,
    what: &str,
) {
    for y in 0..h {
        for x in 0..w {
            assert_eq!(
                decoded.y.get(x, y),
                recon.y.get(x, y),
                "{what}: luma ({x}, {y})"
            );
        }
    }
    for y in 0..h / 2 {
        for x in 0..w / 2 {
            assert_eq!(decoded.cb.get(x, y), recon.cb.get(x, y), "{what}: cb");
            assert_eq!(decoded.cr.get(x, y), recon.cr.get(x, y), "{what}: cr");
        }
    }
}

/// Compose an interlaced sequence layer around manually encoded
/// field-picture layers.
fn wrap_sequence<F: FnOnce(&mut BitWriter) -> Vec<FrameBuffer>>(
    width: usize,
    frame_height: usize,
    encode: F,
) -> (Vec<u8>, Vec<FrameBuffer>) {
    let mut bw = BitWriter::new();
    write_sequence_header(
        &mut bw,
        &SequenceHeaderParams {
            horizontal_size: width as u16,
            vertical_size: frame_height as u16,
            ..Default::default()
        },
    );
    write_sequence_extension(&mut bw, ChromaFormat::Yuv420, false);
    let recons = encode(&mut bw);
    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
    (stream, recons)
}

#[test]
fn sixteen_by_eight_wins_on_split_region_motion_and_roundtrips_exactly() {
    let width = 64;
    let field_h = 32; // frame 64x64
    let params = field_params(width, field_h);

    // Reference frame from two intra fields; the P fields' 16x8
    // regions move in opposite directions.
    let top0 = textured_field(width, field_h, 0);
    let bottom0 = textured_field(width, field_h, 1);
    let top1 = shift_split_regions(&top0, 4, -4);
    let bottom1 = shift_split_regions(&bottom0, 4, -4);

    let mut stats_sum = None;
    let (stream, recons) = wrap_sequence(width, field_h * 2, |bw| {
        let i_top =
            encode_field_intra_picture(bw, &top0, &params, PictureStructure::TopField, 0, 6)
                .expect("I top");
        let i_bottom =
            encode_field_intra_picture(bw, &bottom0, &params, PictureStructure::BottomField, 0, 6)
                .expect("I bottom");
        let i_frame = assemble_frame_from_fields(&i_top, &i_bottom).expect("assemble I");

        let (p_top, s1) = encode_field_p_picture_adaptive(
            bw,
            &top1,
            &i_frame,
            &params,
            PictureStructure::TopField,
            1,
            6,
            3,
            false,
        )
        .expect("P top");
        let second_ref = second_p_field_reference(&p_top, PictureStructure::TopField, &i_frame)
            .expect("second ref");
        let (p_bottom, s2) = encode_field_p_picture_adaptive(
            bw,
            &bottom1,
            &second_ref,
            &params,
            PictureStructure::BottomField,
            1,
            6,
            3,
            false,
        )
        .expect("P bottom");
        let mut total = s1;
        total.add(&s2);
        stats_sum = Some(total);
        let p_frame = assemble_frame_from_fields(&p_top, &p_bottom).expect("assemble P");
        vec![i_frame, p_frame]
    });
    let stats = stats_sum.unwrap();
    assert!(
        stats.sixteen_by_eight > 0,
        "split-region motion must code 16x8 macroblocks: {stats:?}"
    );

    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 2);
    assert_visible_equal(&decoded[0].frame, &recons[0], width, field_h * 2, "I");
    assert_visible_equal(&decoded[1].frame, &recons[1], width, field_h * 2, "16x8 P");
}

#[test]
fn field_dual_prime_wins_on_noisy_reference_and_roundtrips_exactly() {
    let width = 64;
    let field_h = 32;
    let params = field_params(width, field_h);

    // Column-constant base with per-sample noise on the reference
    // fields only: the §7.6.7.4 two-field average halves the noise.
    let noise = |x: usize, y: usize, seed: usize| -> i32 {
        let h = x
            .wrapping_mul(31)
            .wrapping_add(y.wrapping_mul(97))
            .wrapping_add(seed.wrapping_mul(131));
        ((h % 17) as i32) - 8
    };
    let base_field = |noisy: Option<usize>| -> FrameBuffer {
        let mut f = FrameBuffer::new(width, field_h, ChromaFormat::Yuv420);
        for y in 0..field_h {
            for x in 0..width {
                let base = 90 + ((x * 7) % 100) as i32;
                let v = match noisy {
                    Some(seed) => base + noise(x, y, seed),
                    None => base,
                };
                f.y.put_sample(x, y, v.clamp(0, 255) as u8);
            }
        }
        for y in 0..field_h / 2 {
            for x in 0..width / 2 {
                f.cb.put_sample(x, y, 128);
                f.cr.put_sample(x, y, 128);
            }
        }
        f
    };
    let top0 = base_field(Some(1));
    let bottom0 = base_field(Some(2));
    let top1 = base_field(None);
    let bottom1 = base_field(None);

    let mut stats_sum = None;
    let (stream, recons) = wrap_sequence(width, field_h * 2, |bw| {
        let i_top =
            encode_field_intra_picture(bw, &top0, &params, PictureStructure::TopField, 0, 6)
                .expect("I top");
        let i_bottom =
            encode_field_intra_picture(bw, &bottom0, &params, PictureStructure::BottomField, 0, 6)
                .expect("I bottom");
        let i_frame = assemble_frame_from_fields(&i_top, &i_bottom).expect("assemble I");

        let (p_top, s1) = encode_field_p_picture_adaptive(
            bw,
            &top1,
            &i_frame,
            &params,
            PictureStructure::TopField,
            1,
            6,
            3,
            true,
        )
        .expect("P top");
        let second_ref = second_p_field_reference(&p_top, PictureStructure::TopField, &i_frame)
            .expect("second ref");
        let (p_bottom, s2) = encode_field_p_picture_adaptive(
            bw,
            &bottom1,
            &second_ref,
            &params,
            PictureStructure::BottomField,
            1,
            6,
            3,
            true,
        )
        .expect("P bottom");
        let mut total = s1;
        total.add(&s2);
        stats_sum = Some(total);
        let p_frame = assemble_frame_from_fields(&p_top, &p_bottom).expect("assemble P");
        vec![i_frame, p_frame]
    });
    let stats = stats_sum.unwrap();
    assert!(
        stats.dual_prime > 0,
        "noise-averaging content must select dual-prime somewhere: {stats:?}"
    );

    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 2);
    assert_visible_equal(&decoded[1].frame, &recons[1], width, field_h * 2, "DP P");
}

#[test]
fn b_field_16x8_modes_roundtrip_exactly() {
    let width = 64;
    let field_h = 32;
    let params = field_params(width, field_h);

    let top0 = textured_field(width, field_h, 0);
    let bottom0 = textured_field(width, field_h, 1);
    // Anchors and midpoint with split-region motion.
    let mk = |d: i32| {
        (
            shift_split_regions(&top0, d, -d),
            shift_split_regions(&bottom0, d, -d),
        )
    };
    let (top2, bottom2) = mk(6);
    let (top1, bottom1) = mk(3);

    let mut stats_sum = None;
    let (stream, recons) = wrap_sequence(width, field_h * 2, |bw| {
        let i_top =
            encode_field_intra_picture(bw, &top0, &params, PictureStructure::TopField, 0, 6)
                .expect("I top");
        let i_bottom =
            encode_field_intra_picture(bw, &bottom0, &params, PictureStructure::BottomField, 0, 6)
                .expect("I bottom");
        let i_frame = assemble_frame_from_fields(&i_top, &i_bottom).expect("assemble I");

        let (p_top, _) = encode_field_p_picture_adaptive(
            bw,
            &top2,
            &i_frame,
            &params,
            PictureStructure::TopField,
            2,
            6,
            3,
            false,
        )
        .expect("P top");
        let second_ref = second_p_field_reference(&p_top, PictureStructure::TopField, &i_frame)
            .expect("second ref");
        let (p_bottom, _) = encode_field_p_picture_adaptive(
            bw,
            &bottom2,
            &second_ref,
            &params,
            PictureStructure::BottomField,
            2,
            6,
            3,
            false,
        )
        .expect("P bottom");
        let p_frame = assemble_frame_from_fields(&p_top, &p_bottom).expect("assemble P");

        let (b_top, s1) = encode_field_b_picture_adaptive(
            bw,
            &top1,
            &i_frame,
            &p_frame,
            &params,
            PictureStructure::TopField,
            1,
            6,
            3,
            3,
        )
        .expect("B top");
        let (b_bottom, s2) = encode_field_b_picture_adaptive(
            bw,
            &bottom1,
            &i_frame,
            &p_frame,
            &params,
            PictureStructure::BottomField,
            1,
            6,
            3,
            3,
        )
        .expect("B bottom");
        let mut total = s1;
        total.add(&s2);
        stats_sum = Some(total);
        let b_frame = assemble_frame_from_fields(&b_top, &b_bottom).expect("assemble B");
        vec![i_frame, b_frame, p_frame]
    });
    let stats = stats_sum.unwrap();
    assert!(
        stats.sixteen_by_eight > 0,
        "split-region motion must code 16x8 B macroblocks: {stats:?}"
    );

    // Coded order I P B -> display order I B P.
    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 3);
    assert_visible_equal(&decoded[0].frame, &recons[0], width, field_h * 2, "I");
    assert_visible_equal(&decoded[1].frame, &recons[1], width, field_h * 2, "B");
    assert_visible_equal(&decoded[2].frame, &recons[2], width, field_h * 2, "P");
}

#[test]
fn adaptive_assembler_emits_whole_sequence_with_bounded_error() {
    let width = 64;
    let frame_h = 64;
    let frame_params = IntraPictureParams {
        width,
        height: frame_h,
        chroma_format: ChromaFormat::Yuv420,
        frame_pred_frame_dct: false,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        progressive_sequence: false,
    };
    // Frames whose 16x8 regions (in field coordinates: 16-frame-line
    // bands) drift apart.
    let base = {
        let mut f = FrameBuffer::new(width, frame_h, ChromaFormat::Yuv420);
        for y in 0..frame_h {
            for x in 0..width {
                f.y.put_sample(x, y, (50 + (x * 5 + y * 3) % 160) as u8);
            }
        }
        for y in 0..frame_h / 2 {
            for x in 0..width / 2 {
                f.cb.put_sample(x, y, 120);
                f.cr.put_sample(x, y, 140);
            }
        }
        f
    };
    let frames: Vec<FrameBuffer> = (0i32..4)
        .map(|t| {
            let mut out = FrameBuffer::new(width, frame_h, ChromaFormat::Yuv420);
            for y in 0..frame_h {
                // Bands of 16 frame lines (8 field lines) alternate
                // direction.
                let dx = if (y / 16) % 2 == 0 { 2 * t } else { -2 * t };
                for x in 0..width {
                    let sx = (x as i32 - dx).clamp(0, width as i32 - 1) as usize;
                    out.y.put_sample(x, y, base.y.get(sx, y).unwrap());
                }
            }
            for y in 0..frame_h / 2 {
                for x in 0..width / 2 {
                    out.cb.put_sample(x, y, base.cb.get(x, y).unwrap());
                    out.cr.put_sample(x, y, base.cr.get(x, y).unwrap());
                }
            }
            out
        })
        .collect();

    let (stream, stats) = encode_field_adaptive_display_order_gop_sequence(
        &frames,
        1,
        2,
        &frame_params,
        6,
        3,
        3,
        false,
    )
    .expect("adaptive assemble");
    assert!(
        stats.sixteen_by_eight > 0,
        "banded motion must code 16x8 somewhere: {stats:?}"
    );

    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 4);
    for (i, frame) in frames.iter().enumerate() {
        let mut err_sum = 0u64;
        let mut n = 0u64;
        for y in 4..frame_h - 4 {
            for x in 12..width - 12 {
                let d = i64::from(decoded[i].frame.y.get(x, y).unwrap())
                    - i64::from(frame.y.get(x, y).unwrap());
                err_sum += d.unsigned_abs();
                n += 1;
            }
        }
        let mae = err_sum as f64 / n as f64;
        assert!(mae < 6.0, "frame {i} interior luma MAE {mae}");
    }
}

#[test]
fn adaptive_assembler_rejects_dual_prime_with_b_pictures() {
    let frame_params = IntraPictureParams {
        width: 64,
        height: 64,
        chroma_format: ChromaFormat::Yuv420,
        frame_pred_frame_dct: false,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        progressive_sequence: false,
    };
    let frames = vec![FrameBuffer::new(64, 64, ChromaFormat::Yuv420)];
    assert!(encode_field_adaptive_display_order_gop_sequence(
        &frames,
        1,
        1,
        &frame_params,
        6,
        3,
        3,
        true
    )
    .is_err());
}
