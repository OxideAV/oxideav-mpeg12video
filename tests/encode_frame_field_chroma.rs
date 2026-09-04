//! 4:2:2 / 4:4:4 on the **frame-picture field-based** encode path
//! (`frame_pred_frame_dct = 0`): §6.1.3 says the 4:2:2 / 4:4:4 chroma
//! blocks follow the luma field / frame organisation under `dct_type`
//! (only 4:2:0 chroma stays frame-organised), so field DCT now covers
//! the chroma too, and the per-macroblock `dct_type` decision weighs
//! every block. Streams decode sample-exactly against the encoder's
//! reconstruction and the whole assembler round-trips.

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::frame_assembly::{FrameBuffer, IntraPictureParams};
use oxideav_mpeg12video::sequence_extension::{ChromaFormat, Mpeg2Sequence};
use oxideav_mpeg12video::stream_writer::{
    write_sequence_extension, write_sequence_header, SequenceHeaderParams, SEQUENCE_END_CODE,
};
use oxideav_mpeg12video::{
    decode_video_sequence, encode_ff_b_picture, encode_ff_display_order_gop_sequence,
    encode_ff_intra_picture, encode_ff_p_picture,
};

fn ff_params(width: usize, height: usize, chroma: ChromaFormat) -> IntraPictureParams {
    IntraPictureParams {
        progressive_sequence: false,
        width,
        height,
        chroma_format: chroma,
        frame_pred_frame_dct: false,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
    }
}

/// A textured base frame whose chroma planes carry per-row detail at
/// the format's full chroma resolution (so field-organised chroma
/// blocks differ from frame-organised ones).
fn textured_frame(width: usize, height: usize, chroma: ChromaFormat) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, chroma);
    for y in 0..height {
        for x in 0..width {
            let v = 100 + ((x * 5 + (y / 2) * 7) % 80) as i32;
            f.y.put_sample(x, y, v as u8);
        }
    }
    let (cw, ch) = f.visible_chroma_dims();
    for y in 0..ch {
        for x in 0..cw {
            let phase = if y % 2 == 0 { 24 } else { 0 };
            f.cb.put_sample(x, y, (70 + (x * 3 + y * 5 + phase) % 110) as u8);
            f.cr.put_sample(x, y, (190u8).saturating_sub(((x * 2 + y * 9) % 110) as u8));
        }
    }
    f
}

/// Shift each **field** of `src` horizontally by its own amount —
/// luma and (at 4:2:2 / 4:4:4) the full-height chroma alike, so the
/// per-field motion is visible in every component.
fn shift_fields(src: &FrameBuffer, top_dx: i32, bottom_dx: i32) -> FrameBuffer {
    let mut out = FrameBuffer::new(src.width, src.height, src.chroma_format);
    for y in 0..src.height {
        let dx = if y % 2 == 0 { top_dx } else { bottom_dx };
        for x in 0..src.width {
            let sx = (x as i32 - dx).clamp(0, src.width as i32 - 1) as usize;
            out.y.put_sample(x, y, src.y.get(sx, y).unwrap());
        }
    }
    let (cw, ch) = src.visible_chroma_dims();
    let (sx_shift, _) = oxideav_mpeg12video::frame_assembly::chroma_shift(src.chroma_format);
    for y in 0..ch {
        // Full-height chroma keeps the field structure; 4:2:0 chroma
        // covers both fields with one line and is copied unshifted.
        let dx = if ch == src.height {
            (if y % 2 == 0 { top_dx } else { bottom_dx }) >> sx_shift
        } else {
            0
        };
        for x in 0..cw {
            let sx = (x as i32 - dx).clamp(0, cw as i32 - 1) as usize;
            out.cb.put_sample(x, y, src.cb.get(sx, y).unwrap());
            out.cr.put_sample(x, y, src.cr.get(sx, y).unwrap());
        }
    }
    out
}

fn assert_visible_equal(decoded: &FrameBuffer, recon: &FrameBuffer, what: &str) {
    for y in 0..recon.height {
        for x in 0..recon.width {
            assert_eq!(
                decoded.y.get(x, y),
                recon.y.get(x, y),
                "{what}: luma ({x}, {y})"
            );
        }
    }
    let (cw, ch) = recon.visible_chroma_dims();
    assert_eq!(
        decoded.visible_chroma_dims(),
        (cw, ch),
        "{what}: chroma dims"
    );
    for y in 0..ch {
        for x in 0..cw {
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
    chroma: ChromaFormat,
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
    write_sequence_extension(&mut bw, chroma, false);
    let recons = encode(&mut bw);
    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
    (stream, recons)
}

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

#[test]
fn frame_field_i_p_b_422_and_444_decode_sample_exactly() {
    for chroma in [ChromaFormat::Yuv422, ChromaFormat::Yuv444] {
        let width = 64;
        let height = 64;
        let params = ff_params(width, height, chroma);
        let reference_src = textured_frame(width, height, chroma);
        let p_src = shift_fields(&reference_src, 4, -4);
        let b_src = shift_fields(&reference_src, 2, -2);

        let mut p_stats = None;
        let (stream, recons) = wrap_sequence(width, height, chroma, |bw| {
            let (i_recon, i_stats) = encode_ff_intra_picture(bw, &reference_src, &params, 0, 6)
                .expect("encode ff intra");
            assert_eq!(i_stats.intra, (width / 16) * (height / 16));
            let (p_recon, stats) =
                encode_ff_p_picture(bw, &p_src, &i_recon, &params, 2, 6, 3, false)
                    .expect("encode ff P");
            p_stats = Some(stats);
            let (b_recon, _) =
                encode_ff_b_picture(bw, &b_src, &i_recon, &p_recon, &params, 1, 6, 3, 3)
                    .expect("encode ff B");
            vec![i_recon, b_recon, p_recon]
        });
        let stats = p_stats.unwrap();
        assert!(
            stats.field_mc > stats.frame_mc,
            "{chroma:?}: per-field motion must favour Field-based macroblocks: {stats:?}"
        );

        let decoded = decode_video_sequence(&stream).expect("decode");
        assert_eq!(decoded.len(), 3, "{chroma:?}: display-order frame count");
        assert_visible_equal(&decoded[0].frame, &recons[0], "I");
        assert_visible_equal(&decoded[1].frame, &recons[1], "B");
        assert_visible_equal(&decoded[2].frame, &recons[2], "P");

        // The chroma really is refined at full height: interior Cb MAE
        // against the source stays small on the P frame.
        // (4:4:4 non-intra blocks 6 / 7 carry no residual — printed
        // §6.3.17.4 slot gap — so its bound is looser.)
        let (cw, ch) = p_src.visible_chroma_dims();
        let cb_mae = plane_mae(&decoded[2].frame.cb, &p_src.cb, cw, ch);
        let bound = if chroma == ChromaFormat::Yuv444 {
            16.0
        } else {
            6.0
        };
        assert!(cb_mae < bound, "{chroma:?}: P Cb MAE {cb_mae:.2}");
    }
}

#[test]
fn frame_field_field_dct_fires_on_full_height_chroma_422() {
    // A frame whose fields differ strongly in *chroma* only: the luma
    // is field-uniform, so the §6.1.3 chroma organisation decides the
    // per-macroblock dct_type — field DCT must win somewhere.
    let chroma = ChromaFormat::Yuv422;
    let width = 64;
    let height = 64;
    let params = ff_params(width, height, chroma);
    let mut src = FrameBuffer::new(width, height, chroma);
    for y in 0..height {
        for x in 0..width {
            src.y.put_sample(x, y, (90 + (x * 3) % 60) as u8);
        }
    }
    let (cw, ch) = src.visible_chroma_dims();
    for y in 0..ch {
        for x in 0..cw {
            let v = if y % 2 == 0 {
                40 + (x * 7) % 90
            } else {
                200 - (x * 5) % 90
            };
            src.cb.put_sample(x, y, v as u8);
            src.cr.put_sample(x, y, (255 - v) as u8);
        }
    }
    let mut bw = BitWriter::new();
    let (_, stats) = encode_ff_intra_picture(&mut bw, &src, &params, 0, 4).expect("encode");
    assert!(
        stats.field_dct > 0,
        "chroma-only field structure must select field DCT at 4:2:2: {stats:?}"
    );
}

#[test]
fn frame_field_assembler_422_and_444_roundtrip() {
    for chroma in [ChromaFormat::Yuv422, ChromaFormat::Yuv444] {
        let width = 64;
        let height = 64;
        let params = ff_params(width, height, chroma);
        let base = textured_frame(width, height, chroma);
        let frames: Vec<FrameBuffer> = (0..5).map(|t| shift_fields(&base, 2 * t, -2 * t)).collect();
        let (stream, stats) =
            encode_ff_display_order_gop_sequence(&frames, 1, 2, &params, 6, 3, 3, false)
                .expect("frame-field assembler");
        assert!(stats.field_mc > 0, "{chroma:?}: field MC fires: {stats:?}");

        let seq = Mpeg2Sequence::from_buf(&stream).expect("sequence layer");
        assert_eq!(seq.extension.chroma_format, chroma);
        assert_eq!(
            seq.extension.profile_and_level, 0x18,
            "High profile for 4:2:2 / 4:4:4"
        );
        assert!(!seq.extension.progressive_sequence);

        let decoded = decode_video_sequence(&stream).expect("decode");
        assert_eq!(decoded.len(), frames.len());
        let (cw, ch) = base.visible_chroma_dims();
        // 4:4:4 non-intra blocks 6 / 7 carry no residual (printed
        // §6.3.17.4 slot gap), so its chroma bound is looser.
        let chroma_bound = if chroma == ChromaFormat::Yuv444 {
            14.0
        } else {
            8.0
        };
        for (i, (d, want)) in decoded.iter().zip(&frames).enumerate() {
            let y_mae = plane_mae(&d.frame.y, &want.y, width, height);
            let cb_mae = plane_mae(&d.frame.cb, &want.cb, cw, ch);
            let cr_mae = plane_mae(&d.frame.cr, &want.cr, cw, ch);
            assert!(y_mae < 8.0, "{chroma:?}: frame {i} luma MAE {y_mae:.2}");
            assert!(
                cb_mae < chroma_bound,
                "{chroma:?}: frame {i} Cb MAE {cb_mae:.2}"
            );
            assert!(
                cr_mae < chroma_bound,
                "{chroma:?}: frame {i} Cr MAE {cr_mae:.2}"
            );
        }
    }
}

#[test]
fn frame_field_dual_prime_422_roundtrips() {
    let chroma = ChromaFormat::Yuv422;
    let params = ff_params(64, 64, chroma);
    let base = textured_frame(64, 64, chroma);
    let frames: Vec<FrameBuffer> = (0..3).map(|t| shift_fields(&base, t, t)).collect();
    let (stream, stats) =
        encode_ff_display_order_gop_sequence(&frames, 0, 2, &params, 6, 3, 3, true)
            .expect("dual-prime assembler at 4:2:2");
    let p_macroblocks = 2 * (64 / 16) * (64 / 16);
    assert_eq!(
        stats.frame_mc + stats.field_mc + stats.dual_prime + stats.intra,
        p_macroblocks + (64 / 16) * (64 / 16),
        "every macroblock accounted for: {stats:?}"
    );
    let decoded = decode_video_sequence(&stream).expect("decode");
    assert_eq!(decoded.len(), 3);
}
