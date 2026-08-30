//! Frame-picture **field-based encode** round-trips
//! (`frame_pred_frame_dct = 0`, §6.3.10 / §6.2.5.1): per-macroblock
//! Table 6-17 frame/field prediction selection, per-macroblock
//! `dct_type` (field DCT) selection, dual-prime P-macroblocks, and the
//! interlaced display-order assembler — every emitted stream decodes
//! **sample-exactly** against the encoder's decoder-driver
//! reconstruction through `decode_video_sequence`.

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::frame_assembly::{FrameBuffer, IntraPictureParams};
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::stream_writer::{
    write_sequence_extension, write_sequence_header, SequenceHeaderParams, SEQUENCE_END_CODE,
};
use oxideav_mpeg12video::{
    decode_video_sequence, encode_ff_b_picture, encode_ff_display_order_gop_sequence,
    encode_ff_intra_picture, encode_ff_p_picture, DecodedFrame,
};

/// Interlaced-coding parameters: `progressive_sequence = 0`,
/// `frame_pred_frame_dct = 0` (the §6.3.3 grid is
/// `2 * Ceil(height / 32)` macroblock rows).
fn ff_params(width: usize, height: usize) -> IntraPictureParams {
    IntraPictureParams {
        progressive_sequence: false,
        width,
        height,
        chroma_format: ChromaFormat::Yuv420,
        frame_pred_frame_dct: false,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
    }
}

/// Deterministic pseudo-noise in `[-amp, amp]`.
fn noise(x: usize, y: usize, seed: usize, amp: i32) -> i32 {
    let h = x
        .wrapping_mul(31)
        .wrapping_add(y.wrapping_mul(97))
        .wrapping_add(seed.wrapping_mul(131));
    ((h % (2 * amp as usize + 1)) as i32) - amp
}

/// A textured base frame (both fields carry the same pattern).
fn textured_frame(width: usize, height: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            let v = 100 + ((x * 5 + (y / 2) * 7) % 80) as i32;
            f.y.put_sample(x, y, v as u8);
        }
    }
    for y in 0..height / 2 {
        for x in 0..width / 2 {
            f.cb.put_sample(x, y, (110 + (x % 20)) as u8);
            f.cr.put_sample(x, y, (140 + (y % 20)) as u8);
        }
    }
    f
}

/// Shift each **field** of `src` horizontally by its own amount
/// (`top_dx` for even lines, `bottom_dx` for odd lines), clamping at
/// the edges — motion a single frame vector cannot capture but two
/// field vectors can.
fn shift_fields(src: &FrameBuffer, top_dx: i32, bottom_dx: i32) -> FrameBuffer {
    let mut out = FrameBuffer::new(src.width, src.height, ChromaFormat::Yuv420);
    for y in 0..src.height {
        let dx = if y % 2 == 0 { top_dx } else { bottom_dx };
        for x in 0..src.width {
            let sx = (x as i32 - dx).clamp(0, src.width as i32 - 1) as usize;
            out.y.put_sample(x, y, src.y.get(sx, y).unwrap());
        }
    }
    // 4:2:0 chroma: one chroma line covers both fields; copy unshifted
    // (the test measures luma).
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
            assert_eq!(
                decoded.cb.get(x, y),
                recon.cb.get(x, y),
                "{what}: cb ({x}, {y})"
            );
            assert_eq!(
                decoded.cr.get(x, y),
                recon.cr.get(x, y),
                "{what}: cr ({x}, {y})"
            );
        }
    }
}

/// Compose an interlaced sequence layer around manually encoded
/// picture layers.
fn wrap_sequence<F: FnOnce(&mut BitWriter) -> Vec<FrameBuffer>>(
    width: usize,
    height: usize,
    encode: F,
) -> (Vec<u8>, Vec<FrameBuffer>) {
    let mut bw = BitWriter::new();
    write_sequence_header(
        &mut bw,
        &SequenceHeaderParams {
            horizontal_size: width as u16,
            vertical_size: height as u16,
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
fn field_prediction_wins_on_per_field_motion_and_roundtrips_exactly() {
    let width = 64;
    let height = 64;
    let params = ff_params(width, height);
    let reference_src = textured_frame(width, height);
    // Top field moves +4 px, bottom field -4 px: a single frame vector
    // cannot capture both, two field vectors can.
    let current_src = shift_fields(&reference_src, 4, -4);

    let mut p_stats = None;
    let (stream, recons) = wrap_sequence(width, height, |bw| {
        let (i_recon, _) =
            encode_ff_intra_picture(bw, &reference_src, &params, 0, 6).expect("encode ff intra");
        let (p_recon, stats) =
            encode_ff_p_picture(bw, &current_src, &i_recon, &params, 1, 6, 3, false)
                .expect("encode ff P");
        p_stats = Some(stats);
        vec![i_recon, p_recon]
    });
    let stats = p_stats.unwrap();
    assert!(
        stats.field_mc > stats.frame_mc,
        "per-field motion must favour Field-based macroblocks: {stats:?}"
    );

    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 2);
    assert_visible_equal(&decoded[0].frame, &recons[0], width, height, "I");
    assert_visible_equal(&decoded[1].frame, &recons[1], width, height, "P");

    // Field prediction captures the per-field shift: interior luma MAE
    // stays small.
    let mut err_sum = 0u64;
    let mut n = 0u64;
    for y in 4..height - 4 {
        for x in 8..width - 8 {
            let d = i64::from(decoded[1].frame.y.get(x, y).unwrap())
                - i64::from(current_src.y.get(x, y).unwrap());
            err_sum += d.unsigned_abs();
            n += 1;
        }
    }
    let mae = err_sum as f64 / n as f64;
    assert!(mae < 4.0, "P interior luma MAE {mae}");
}

#[test]
fn mixed_frame_and_field_macroblocks_share_one_pmv_bank_exactly() {
    // Left half: per-field motion (field MBs); right half: uniform
    // frame motion (frame MBs). Sample-exact decode proves the encoder
    // mirrors the §7.6.3.1 / §7.6.3.3 predictor arithmetic across
    // mode changes inside one slice.
    let width = 96;
    let height = 64;
    let params = ff_params(width, height);
    let reference_src = textured_frame(width, height);
    let mut current_src = shift_fields(&reference_src, 3, -3);
    // Right half: uniform +2 shift of both fields.
    for y in 0..height {
        for x in width / 2..width {
            let sx = (x as i32 - 2).clamp(0, width as i32 - 1) as usize;
            let v = reference_src.y.get(sx, y).unwrap();
            current_src.y.put_sample(x, y, v);
        }
    }

    let mut p_stats = None;
    let (stream, recons) = wrap_sequence(width, height, |bw| {
        let (i_recon, _) =
            encode_ff_intra_picture(bw, &reference_src, &params, 0, 6).expect("encode ff intra");
        let (p_recon, stats) =
            encode_ff_p_picture(bw, &current_src, &i_recon, &params, 1, 6, 3, false)
                .expect("encode ff P");
        p_stats = Some(stats);
        vec![i_recon, p_recon]
    });
    let stats = p_stats.unwrap();
    assert!(
        stats.field_mc > 0 && stats.frame_mc > 0,
        "the mixed picture must code both modes: {stats:?}"
    );

    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 2);
    assert_visible_equal(&decoded[1].frame, &recons[1], width, height, "mixed P");
}

#[test]
fn field_dct_selected_on_interlaced_detail_and_roundtrips_exactly() {
    // Fields with sharply different content: field DCT concentrates
    // the energy each field carries, so the exact bit-cost decision
    // must pick it on at least some macroblocks — and the stream must
    // still decode sample-exactly.
    let width = 64;
    let height = 64;
    let params = ff_params(width, height);
    let mut frame = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            // Top field: horizontal ramp; bottom field: inverted ramp
            // with an offset — strong line-to-line alternation.
            let v = if y % 2 == 0 {
                60 + ((x * 3) % 120) as i32
            } else {
                200 - ((x * 3) % 120) as i32
            };
            frame.y.put_sample(x, y, v as u8);
        }
    }
    for y in 0..height / 2 {
        for x in 0..width / 2 {
            frame.cb.put_sample(x, y, 128);
            frame.cr.put_sample(x, y, 128);
        }
    }

    let mut i_stats = None;
    let (stream, recons) = wrap_sequence(width, height, |bw| {
        let (i_recon, stats) =
            encode_ff_intra_picture(bw, &frame, &params, 0, 6).expect("encode ff intra");
        i_stats = Some(stats);
        vec![i_recon]
    });
    let stats = i_stats.unwrap();
    assert!(
        stats.field_dct > 0,
        "alternating fields must select field DCT somewhere: {stats:?}"
    );

    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 1);
    assert_visible_equal(&decoded[0].frame, &recons[0], width, height, "field-DCT I");
}

#[test]
fn dual_prime_wins_on_noisy_reference_and_roundtrips_exactly() {
    // Base content constant down each column (both fields identical),
    // plus per-sample noise in the *reference* only: the §7.6.7.4
    // dual-prime average of two field predictions halves the noise, so
    // dual-prime beats the single-field/frame copy.
    let width = 64;
    let height = 64;
    let params = ff_params(width, height);
    let mut reference_src = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    let mut current_src = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            let base = 90 + ((x * 7) % 100) as i32;
            reference_src
                .y
                .put_sample(x, y, (base + noise(x, y, 1, 8)).clamp(0, 255) as u8);
            current_src.y.put_sample(x, y, base as u8);
        }
    }
    for y in 0..height / 2 {
        for x in 0..width / 2 {
            reference_src.cb.put_sample(x, y, 128);
            reference_src.cr.put_sample(x, y, 128);
            current_src.cb.put_sample(x, y, 128);
            current_src.cr.put_sample(x, y, 128);
        }
    }

    let mut p_stats = None;
    let (stream, recons) = wrap_sequence(width, height, |bw| {
        let (i_recon, _) =
            encode_ff_intra_picture(bw, &reference_src, &params, 0, 6).expect("encode ff intra");
        let (p_recon, stats) =
            encode_ff_p_picture(bw, &current_src, &i_recon, &params, 1, 6, 3, true)
                .expect("encode ff P with dual prime");
        p_stats = Some(stats);
        vec![i_recon, p_recon]
    });
    let stats = p_stats.unwrap();
    assert!(
        stats.dual_prime > 0,
        "noise-averaging content must select dual-prime somewhere: {stats:?}"
    );

    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 2);
    assert_visible_equal(&decoded[1].frame, &recons[1], width, height, "dual-prime P");
}

#[test]
fn b_picture_field_modes_roundtrip_exactly() {
    let width = 64;
    let height = 64;
    let params = ff_params(width, height);
    let frame0 = textured_frame(width, height);
    let frame2 = shift_fields(&frame0, 6, -6);
    // The B midpoint: fields halfway along their own trajectories.
    let frame1 = shift_fields(&frame0, 3, -3);

    let mut b_stats = None;
    let (stream, recons) = wrap_sequence(width, height, |bw| {
        let (i_recon, _) =
            encode_ff_intra_picture(bw, &frame0, &params, 0, 6).expect("encode ff intra");
        let (p_recon, _) = encode_ff_p_picture(bw, &frame2, &i_recon, &params, 2, 6, 3, false)
            .expect("encode ff P");
        let (b_recon, stats) =
            encode_ff_b_picture(bw, &frame1, &i_recon, &p_recon, &params, 1, 6, 3, 3)
                .expect("encode ff B");
        b_stats = Some(stats);
        vec![i_recon, b_recon, p_recon]
    });
    let stats = b_stats.unwrap();
    assert!(
        stats.field_mc > 0,
        "per-field motion must code field-based B macroblocks: {stats:?}"
    );

    // Coded order I P B -> display order I B P (§6.1.1.11).
    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 3);
    let trefs: Vec<u16> = decoded.iter().map(|d| d.temporal_reference).collect();
    assert_eq!(trefs, vec![0, 1, 2]);
    assert_visible_equal(&decoded[0].frame, &recons[0], width, height, "I");
    assert_visible_equal(&decoded[1].frame, &recons[1], width, height, "B");
    assert_visible_equal(&decoded[2].frame, &recons[2], width, height, "P");
}

#[test]
fn assembler_emits_whole_interlaced_sequence_with_gops() {
    let width = 64;
    let height = 64;
    let params = ff_params(width, height);
    let base = textured_frame(width, height);
    let frames: Vec<FrameBuffer> = (0i32..5)
        .map(|i| shift_fields(&base, 2 * i, -2 * i))
        .collect();

    let (stream, stats) =
        encode_ff_display_order_gop_sequence(&frames, 1, 2, &params, 6, 3, 3, false)
            .expect("assemble ff sequence");
    assert!(stats.field_mc > 0, "per-field motion sequence: {stats:?}");

    let decoded: Vec<DecodedFrame> = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 5);
    // Display order is strictly the input order; bounded error per
    // frame against the source.
    for (i, frame) in frames.iter().enumerate() {
        let mut err_sum = 0u64;
        let mut n = 0u64;
        for y in 4..height - 4 {
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
fn non_multiple_of_32_height_uses_interlaced_grid() {
    // 48-line interlaced frame pictures code 2 * Ceil(48/32) = 4
    // macroblock rows (§6.3.3); the stream must round-trip exactly.
    let width = 48;
    let height = 48;
    let params = ff_params(width, height);
    assert_eq!(params.mb_height(), 4, "interlaced §6.3.3 grid");
    let frame = textured_frame(width, height);

    let (stream, recons) = wrap_sequence(width, height, |bw| {
        let (i_recon, _) =
            encode_ff_intra_picture(bw, &frame, &params, 0, 6).expect("encode ff intra");
        vec![i_recon]
    });
    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 1);
    assert_visible_equal(&decoded[0].frame, &recons[0], width, height, "48-line I");
}

#[test]
fn ff_encoders_reject_bad_configurations() {
    let frame = textured_frame(64, 64);
    let frames = vec![frame.clone()];

    // Progressive params are rejected (§6.3.10).
    let progressive = IntraPictureParams {
        progressive_sequence: true,
        ..ff_params(64, 64)
    };
    assert!(
        encode_ff_display_order_gop_sequence(&frames, 0, 1, &progressive, 6, 3, 3, false).is_err()
    );

    // frame_pred_frame_dct = 1 params are rejected.
    let fpfd = IntraPictureParams {
        frame_pred_frame_dct: true,
        ..ff_params(64, 64)
    };
    assert!(encode_ff_display_order_gop_sequence(&frames, 0, 1, &fpfd, 6, 3, 3, false).is_err());

    // Dual-prime with B-pictures in between is rejected (§7.6.3.6).
    assert!(
        encode_ff_display_order_gop_sequence(&frames, 2, 1, &ff_params(64, 64), 6, 3, 3, true)
            .is_err()
    );

    // Empty frames / zero anchors.
    assert!(
        encode_ff_display_order_gop_sequence(&[], 0, 1, &ff_params(64, 64), 6, 3, 3, false)
            .is_err()
    );
    assert!(encode_ff_display_order_gop_sequence(
        &frames,
        0,
        0,
        &ff_params(64, 64),
        6,
        3,
        3,
        false
    )
    .is_err());
}

#[test]
fn ff_assembler_honours_alternate_scan_and_intra_vlc_format() {
    // Round 453: the frame-field encoder codes the full entropy flag
    // set on the wire — `alternate_scan` (§7.3), `intra_vlc_format`
    // (Table 7-3 → Table B-15), non-linear `q_scale_type` and 10-bit
    // `intra_dc_precision` — and the stream still decodes with
    // bounded distortion through `decode_video_sequence`.
    let width = 64;
    let height = 64;
    let params = IntraPictureParams {
        alternate_scan: true,
        intra_vlc_format: true,
        q_scale_type: true,
        intra_dc_precision: 2,
        ..ff_params(width, height)
    };
    let base = textured_frame(width, height);
    let frames: Vec<FrameBuffer> = (0i32..5)
        .map(|i| shift_fields(&base, 2 * i, -2 * i))
        .collect();

    let (stream, stats) =
        encode_ff_display_order_gop_sequence(&frames, 1, 2, &params, 6, 3, 3, false)
            .expect("assemble full-flag ff sequence");
    assert!(stats.field_mc > 0, "per-field motion sequence: {stats:?}");

    // The flags are on the wire.
    let pic = stream
        .windows(4)
        .position(|w| w == [0x00, 0x00, 0x01, 0x00])
        .expect("picture start code");
    let (_hdr, ext) =
        oxideav_mpeg12video::picture_header::Mpeg2PictureHeader::parse_with_extension(
            &stream[pic..],
        )
        .expect("parse picture headers");
    assert!(ext.alternate_scan);
    assert!(ext.intra_vlc_format);
    assert!(ext.q_scale_type);
    assert_eq!(ext.intra_dc_precision, 2);

    let decoded: Vec<DecodedFrame> = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 5);
    for (i, frame) in frames.iter().enumerate() {
        let mut err_sum = 0u64;
        let mut n = 0u64;
        for y in 4..height - 4 {
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
