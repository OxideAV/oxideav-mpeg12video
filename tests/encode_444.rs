//! The 4:4:4 **encoder** leg: Figure 6-12 twelve-block macroblocks
//! (Cb 4/6/8/10, Cr 5/7/9/11, column-major), full-resolution chroma,
//! §7.6.3.7 unscaled chroma motion vectors, and the §6.2.5.3 six-bit
//! `coded_block_pattern_2` extension.
//!
//! One deliberate restriction is pinned here: the **printed** §6.3.17.4
//! derivation drives `pattern_code[8..12]` from `coded_block_pattern_2`
//! bits 3..0 and gives non-intra blocks 6 / 7 no wire representation,
//! so the encoder leaves those two residuals untransmitted (blocks
//! stay uncoded; the reconstruction accounts for it, keeping the
//! decode decoder-exact). Bits 5..4 of the six-bit field are always
//! emitted as zero, so a decoder applying a six-block `i = 6..12`
//! reading of the same field reconstructs identically.

use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    decode_video_sequence, encode_display_order_gop_sequence, encode_i_then_p,
    encode_intra_picture, encode_p_picture, FrameBuffer, IntraPictureParams,
};

fn params_444(width: usize, height: usize) -> IntraPictureParams {
    IntraPictureParams {
        progressive_sequence: true,
        width,
        height,
        chroma_format: ChromaFormat::Yuv444,
        frame_pred_frame_dct: true,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
    }
}

/// Deterministic 4:4:4 content: full-resolution chroma with detail in
/// both dimensions.
fn frame_444(width: usize, height: usize, shift: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv444);
    for y in 0..height {
        for x in 0..width {
            let sx = x + shift;
            let g = 24 + ((sx * 3 + y * 5) % 192);
            let c = if (sx / 4 + y / 4) % 2 == 0 { 12 } else { 0 };
            f.y.put_sample(x, y, (g + c).min(235) as u8);
            f.cb.put_sample(x, y, (64 + (sx * 2 + y * 7) % 128) as u8);
            f.cr.put_sample(x, y, (192u8).saturating_sub(((sx * 3 + y * 5) % 128) as u8));
        }
    }
    f
}

fn mae(a: &oxideav_mpeg12video::Plane, b: &oxideav_mpeg12video::Plane, w: usize, h: usize) -> f64 {
    let mut sum = 0u64;
    for y in 0..h {
        for x in 0..w {
            sum += u64::from(
                (i32::from(a.get(x, y).unwrap()) - i32::from(b.get(x, y).unwrap())).unsigned_abs(),
            );
        }
    }
    sum as f64 / (w * h) as f64
}

fn assert_planes_equal(a: &FrameBuffer, b: &FrameBuffer, w: usize, h: usize) {
    for y in 0..h {
        for x in 0..w {
            assert_eq!(a.y.get(x, y), b.y.get(x, y), "luma ({x},{y})");
            assert_eq!(a.cb.get(x, y), b.cb.get(x, y), "cb ({x},{y})");
            assert_eq!(a.cr.get(x, y), b.cr.get(x, y), "cr ({x},{y})");
        }
    }
}

#[test]
fn intra_444_flat_frame_roundtrips_exactly() {
    let mut f = FrameBuffer::new(32, 32, ChromaFormat::Yuv444);
    for y in 0..32 {
        for x in 0..32 {
            f.y.put_sample(x, y, 100);
            f.cb.put_sample(x, y, 90);
            f.cr.put_sample(x, y, 170);
        }
    }
    let stream = encode_intra_picture(&f, params_444(32, 32), 0, 8).expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 1);
    let out = &frames[0].frame;
    assert_eq!(out.chroma_format, ChromaFormat::Yuv444);
    assert_eq!(
        out.visible_chroma_dims(),
        (32, 32),
        "4:4:4 chroma is full-resolution"
    );
    for y in 0..32 {
        for x in 0..32 {
            assert_eq!(out.y.get(x, y), Some(100), "luma ({x},{y})");
            assert_eq!(out.cb.get(x, y), Some(90), "cb ({x},{y})");
            assert_eq!(out.cr.get(x, y), Some(170), "cr ({x},{y})");
        }
    }
}

#[test]
fn intra_444_structured_frame_roundtrips_faithfully() {
    // Intra macroblocks code all twelve blocks (pattern_code[i] = 1
    // for intra, §6.3.17.4) — no cbp restriction applies, so every
    // chroma quadrant must reconstruct faithfully.
    let f = frame_444(64, 48, 0);
    let stream = encode_intra_picture(&f, params_444(64, 48), 0, 4).expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 1);
    let out = &frames[0].frame;
    assert!(mae(&f.y, &out.y, 64, 48) < 4.0, "luma MAE");
    assert!(mae(&f.cb, &out.cb, 64, 48) < 4.0, "cb MAE");
    assert!(mae(&f.cr, &out.cr, 64, 48) < 4.0, "cr MAE");
    // Per-quadrant check: the bottom-left chroma quadrant (blocks
    // 6 / 7 in Figure 6-12) must be as faithful as the rest in an
    // all-intra picture.
    let mut sum = 0u64;
    for y in 8..16 {
        for x in 0..8 {
            sum += u64::from(
                (i32::from(f.cb.get(x, y).unwrap()) - i32::from(out.cb.get(x, y).unwrap()))
                    .unsigned_abs(),
            );
        }
    }
    assert!(
        (sum as f64) / 64.0 < 6.0,
        "intra bottom-left chroma quadrant MAE"
    );
}

#[test]
fn p_444_mc_copy_is_a_fixed_point() {
    let f = frame_444(64, 48, 0);
    let p = params_444(64, 48);
    let i_stream = encode_intra_picture(&f, p, 0, 6).expect("encode I");
    let anchor = decode_video_sequence(&i_stream).expect("decode I")[0]
        .frame
        .clone();
    let stream = encode_i_then_p(&f, &anchor, p, 6, 2).expect("encode I+P");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 2);
    assert_planes_equal(&frames[0].frame, &frames[1].frame, 64, 48);
}

#[test]
fn p_444_translation_is_decoder_exact_against_encoder_recon() {
    // Decoder-exactness under the blocks-6/7-uncoded restriction: the
    // encoder's returned reconstruction (which drops those residuals
    // exactly as the wire does) must equal the decode.
    let f0 = frame_444(64, 48, 0);
    let f1 = frame_444(64, 48, 4);
    let p = params_444(64, 48);
    let i_stream = encode_intra_picture(&f0, p, 0, 6).expect("encode I");
    let anchor = decode_video_sequence(&i_stream).expect("decode I")[0]
        .frame
        .clone();
    let mut scratch = oxideav_core::bits::BitWriter::new();
    let recon = encode_p_picture(&mut scratch, &f1, &anchor, p, 1, 6, 2).expect("encode P");
    let stream = encode_i_then_p(&f0, &f1, p, 6, 2).expect("encode I+P");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 2);
    assert_planes_equal(&frames[1].frame, &recon, 64, 48);
    assert!(mae(&f1.y, &frames[1].frame.y, 64, 48) < 4.0, "luma MAE");
}

#[test]
fn p_444_right_half_chroma_blocks_are_transmitted() {
    // Perturb only the right-half chroma of a decoded anchor: blocks
    // 8..11 (Figure 6-12 right column) carry the residual through
    // coded_block_pattern_2 bits 3..0 while luma stays uncoded.
    let f = frame_444(64, 48, 0);
    let p = params_444(64, 48);
    let i_stream = encode_intra_picture(&f, p, 0, 6).expect("encode I");
    let anchor = decode_video_sequence(&i_stream).expect("decode I")[0]
        .frame
        .clone();
    let mut target = anchor.clone();
    for y in 0..48 {
        for x in (0..64).filter(|x| x % 16 >= 8) {
            let v = target.cb.get(x, y).unwrap();
            target.cb.put_sample(x, y, v.saturating_add(24));
        }
    }
    let mut scratch = oxideav_core::bits::BitWriter::new();
    let recon = encode_p_picture(&mut scratch, &target, &anchor, p, 1, 6, 2).expect("encode P");
    let stream = encode_i_then_p(&f, &target, p, 6, 2).expect("encode I+P");
    let frames = decode_video_sequence(&stream).expect("decode");
    let out = &frames[1].frame;
    assert_planes_equal(out, &recon, 64, 48);
    let mut moved = 0usize;
    for y in 0..48 {
        for x in (0..64).filter(|x| x % 16 >= 8) {
            let dec = i32::from(out.cb.get(x, y).unwrap());
            let anc = i32::from(anchor.cb.get(x, y).unwrap());
            if (dec - anc) > 8 {
                moved += 1;
            }
        }
    }
    assert!(
        moved > 800,
        "right-column chroma residuals must be transmitted (moved {moved})"
    );
}

#[test]
fn p_444_bottom_left_chroma_residual_is_dropped_but_decoder_exact() {
    // Perturb only the bottom-left chroma quadrant (blocks 6 / 7):
    // the printed §6.3.17.4 gives those blocks no non-intra wire
    // slot, so the P picture cannot transmit the residual — the
    // decode must equal the encoder's reconstruction (which also
    // dropped it) and stay at the anchor's values.
    let f = frame_444(64, 48, 0);
    let p = params_444(64, 48);
    let i_stream = encode_intra_picture(&f, p, 0, 6).expect("encode I");
    let anchor = decode_video_sequence(&i_stream).expect("decode I")[0]
        .frame
        .clone();
    let mut target = anchor.clone();
    for y in (0..48).filter(|y| y % 16 >= 8) {
        for x in (0..64).filter(|x| x % 16 < 8) {
            let v = target.cb.get(x, y).unwrap();
            target.cb.put_sample(x, y, v.saturating_add(20));
        }
    }
    let mut scratch = oxideav_core::bits::BitWriter::new();
    let recon = encode_p_picture(&mut scratch, &target, &anchor, p, 1, 6, 2).expect("encode P");
    let stream = encode_i_then_p(&f, &target, p, 6, 2).expect("encode I+P");
    let frames = decode_video_sequence(&stream).expect("decode");
    let out = &frames[1].frame;
    assert_planes_equal(out, &recon, 64, 48);
    // The residual could not travel: bottom-left chroma equals the
    // anchor (motion is (0,0) since luma is identical).
    for y in (0..48).filter(|y| y % 16 >= 8) {
        for x in (0..64).filter(|x| x % 16 < 8) {
            assert_eq!(
                out.cb.get(x, y),
                anchor.cb.get(x, y),
                "bottom-left cb ({x},{y})"
            );
        }
    }
}

#[test]
fn gop_444_display_order_sequence_roundtrips() {
    // A translating GOP at 4:4:4: prediction covers the motion, so
    // the blocks-6/7 restriction costs little and every frame stays
    // faithful.
    let frames_in: Vec<FrameBuffer> = (0..5).map(|k| frame_444(64, 48, 2 * k)).collect();
    let p = params_444(64, 48);
    let stream =
        encode_display_order_gop_sequence(&frames_in, 1, 2, p, 6, 3, 3).expect("encode GOPs");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 5);
    for (i, want) in frames_in.iter().enumerate() {
        let out = &frames[i].frame;
        assert_eq!(out.chroma_format, ChromaFormat::Yuv444);
        assert!(mae(&want.y, &out.y, 64, 48) < 5.0, "frame {i} luma MAE");
        // The chroma bounds are looser than luma: the printed
        // §6.3.17.4 leaves non-intra blocks 6 / 7 without a wire
        // slot, so bottom-left chroma residual drifts across a P
        // chain until the next I picture (measured <= 7.4 by frame 4).
        assert!(mae(&want.cb, &out.cb, 64, 48) < 9.0, "frame {i} cb MAE");
        assert!(mae(&want.cr, &out.cr, 64, 48) < 9.0, "frame {i} cr MAE");
    }
}
