//! The 4:2:2-profile **encoder** leg: end-to-end encode → decode
//! round-trips for `chroma_format == 4:2:2` streams (ISO/IEC 13818-2
//! §6.3.5, Figure 6-11 eight-block macroblock structure, §6.2.5.3
//! `coded_block_pattern_1`).
//!
//! The decoder side has handled 4:2:2 since the Figure 6-11 block
//! numbering fix; these tests hold the encoder to it:
//!
//! * the emitted `sequence_extension()` declares `chroma_format = 10`
//!   and the High@Main `profile_and_level_indication` (Table 8-5 —
//!   the one 1995-text profile row admitting 4:2:2 chroma);
//! * intra macroblocks carry eight blocks in Figure 6-11 order (the
//!   §7.2.1 DC predictor chain interleaves Cb/Cr as 4,5,6,7);
//! * full-height chroma detail survives the round trip (a 4:2:0
//!   regression would halve the chroma rows);
//! * encode ∘ decode is a fixed point on its own reconstruction.

use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    decode_video_sequence, encode_display_order_gop_sequence, encode_i_p_b, encode_i_then_p,
    encode_intra_picture, encode_p_picture, FrameBuffer, IntraPictureParams,
};

fn params_422(width: usize, height: usize) -> IntraPictureParams {
    IntraPictureParams {
        progressive_sequence: true,
        width,
        height,
        chroma_format: ChromaFormat::Yuv422,
        frame_pred_frame_dct: true,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
    }
}

/// Deterministic 4:2:2 content: diagonal luma gradient + checker, and
/// chroma with genuine **vertical** detail (full-height 4:2:2 chroma
/// planes; every chroma row differs from its neighbour).
fn frame_422(width: usize, height: usize, shift: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv422);
    for y in 0..height {
        for x in 0..width {
            let sx = x + shift;
            let g = 24 + ((sx * 3 + y * 5) % 192);
            let c = if (sx / 4 + y / 4) % 2 == 0 { 12 } else { 0 };
            f.y.put_sample(x, y, (g + c).min(235) as u8);
        }
    }
    // 4:2:2 chroma: half width, FULL height.
    for y in 0..height {
        for x in 0..width / 2 {
            f.cb.put_sample(x, y, (64 + (x * 2 + y * 7 + shift) % 128) as u8);
            f.cr.put_sample(x, y, (192u8).saturating_sub(((x * 3 + y * 5) % 128) as u8));
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

#[test]
fn intra_422_flat_frame_roundtrips_exactly() {
    let mut f = FrameBuffer::new(32, 32, ChromaFormat::Yuv422);
    for y in 0..32 {
        for x in 0..32 {
            f.y.put_sample(x, y, 100);
        }
    }
    for y in 0..32 {
        for x in 0..16 {
            f.cb.put_sample(x, y, 90);
            f.cr.put_sample(x, y, 170);
        }
    }
    let stream = encode_intra_picture(&f, params_422(32, 32), 0, 8).expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 1);
    let out = &frames[0].frame;
    assert_eq!(out.chroma_format, ChromaFormat::Yuv422);
    for y in 0..32 {
        for x in 0..32 {
            assert_eq!(out.y.get(x, y), Some(100), "luma ({x},{y})");
        }
        for x in 0..16 {
            assert_eq!(out.cb.get(x, y), Some(90), "cb ({x},{y})");
            assert_eq!(out.cr.get(x, y), Some(170), "cr ({x},{y})");
        }
    }
}

#[test]
fn intra_422_structured_frame_roundtrips_faithfully() {
    let f = frame_422(64, 48, 0);
    let stream = encode_intra_picture(&f, params_422(64, 48), 0, 4).expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 1);
    let out = &frames[0].frame;
    assert_eq!((out.width, out.height), (64, 48));
    assert_eq!(out.chroma_format, ChromaFormat::Yuv422);
    let (cw, ch) = out.visible_chroma_dims();
    assert_eq!(
        (cw, ch),
        (32, 48),
        "4:2:2 chroma is half-width, full-height"
    );
    assert!(mae(&f.y, &out.y, 64, 48) < 4.0, "luma MAE");
    assert!(mae(&f.cb, &out.cb, 32, 48) < 4.0, "cb MAE");
    assert!(mae(&f.cr, &out.cr, 32, 48) < 4.0, "cr MAE");
    // Vertical chroma detail must survive: the source's per-row chroma
    // ramp means adjacent chroma rows differ; a 4:2:0 collapse (or a
    // block-numbering swap) would flatten or shear this. Compare the
    // reconstruction against the source row-by-row rather than against
    // an averaged pair.
    let mut worst_row_mae = 0.0f64;
    for y in 0..48 {
        let mut sum = 0u64;
        for x in 0..32 {
            sum += u64::from(
                (i32::from(f.cb.get(x, y).unwrap()) - i32::from(out.cb.get(x, y).unwrap()))
                    .unsigned_abs(),
            );
        }
        worst_row_mae = worst_row_mae.max(sum as f64 / 32.0);
    }
    assert!(worst_row_mae < 8.0, "worst chroma-row MAE {worst_row_mae}");
}

#[test]
fn intra_422_encode_is_reconstruction_idempotent() {
    let f = frame_422(48, 32, 3);
    let p = params_422(48, 32);
    let stream1 = encode_intra_picture(&f, p, 0, 6).expect("encode 1");
    let dec1 = decode_video_sequence(&stream1).expect("decode 1");
    assert_eq!(dec1.len(), 1);
    let stream2 = encode_intra_picture(&dec1[0].frame, p, 0, 6).expect("encode 2");
    let dec2 = decode_video_sequence(&stream2).expect("decode 2");
    assert_eq!(dec2.len(), 1);
    let (a, b) = (&dec1[0].frame, &dec2[0].frame);
    for y in 0..32 {
        for x in 0..48 {
            assert_eq!(a.y.get(x, y), b.y.get(x, y), "luma ({x},{y})");
        }
        for x in 0..24 {
            assert_eq!(a.cb.get(x, y), b.cb.get(x, y), "cb ({x},{y})");
            assert_eq!(a.cr.get(x, y), b.cr.get(x, y), "cr ({x},{y})");
        }
    }
}

/// Decode the I anchor the inter assemblers will predict from.
fn decoded_anchor(f: &FrameBuffer, p: IntraPictureParams, q: u8) -> FrameBuffer {
    let i_stream = encode_intra_picture(f, p, 0, q).expect("encode I");
    decode_video_sequence(&i_stream).expect("decode I")[0]
        .frame
        .clone()
}

fn assert_planes_equal(a: &FrameBuffer, b: &FrameBuffer, w: usize, h: usize) {
    for y in 0..h {
        for x in 0..w {
            assert_eq!(a.y.get(x, y), b.y.get(x, y), "luma ({x},{y})");
        }
        for x in 0..w / 2 {
            assert_eq!(a.cb.get(x, y), b.cb.get(x, y), "cb ({x},{y})");
            assert_eq!(a.cr.get(x, y), b.cr.get(x, y), "cr ({x},{y})");
        }
    }
}

#[test]
fn p_422_mc_copy_is_a_fixed_point() {
    // A P target equal to the decoded 4:2:2 anchor must reproduce it
    // sample-for-sample: the (0,0) prediction is exact so every block
    // (including chroma blocks 6/7) quantises to zero.
    let f = frame_422(64, 48, 0);
    let p = params_422(64, 48);
    let anchor = decoded_anchor(&f, p, 6);
    let stream = encode_i_then_p(&f, &anchor, p, 6, 2).expect("encode I+P");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 2);
    assert_planes_equal(&frames[0].frame, &frames[1].frame, 64, 48);
}

#[test]
fn p_422_translation_is_decoder_exact_against_encoder_recon() {
    // A pure translation target: the decoded P frame must equal the
    // reconstruction encode_p_picture returned (decoder-exactness of
    // the eight-block residual path + §7.6.3.7 4:2:2 chroma scaling).
    let f0 = frame_422(64, 48, 0);
    let f1 = frame_422(64, 48, 4);
    let p = params_422(64, 48);
    let anchor = decoded_anchor(&f0, p, 6);
    // Parallel encode into a scratch writer to recover the encoder's
    // reconstruction (encode_i_then_p drives the same call).
    let mut scratch = oxideav_core::bits::BitWriter::new();
    let recon = encode_p_picture(&mut scratch, &f1, &anchor, p, 1, 6, 2).expect("encode P");
    let stream = encode_i_then_p(&f0, &f1, p, 6, 2).expect("encode I+P");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 2);
    assert_planes_equal(&frames[1].frame, &recon, 64, 48);
    assert!(mae(&f1.y, &frames[1].frame.y, 64, 48) < 4.0, "luma MAE");
    assert!(mae(&f1.cb, &frames[1].frame.cb, 32, 48) < 5.0, "cb MAE");
}

#[test]
fn p_422_chroma_extension_only_blocks_are_transmitted() {
    // Perturb ONLY the bottom-half chroma rows of each macroblock
    // (Figure 6-11 blocks 6/7 territory) of a decoded anchor. Luma and
    // the top chroma blocks predict exactly at (0,0), so any coded
    // macroblock carries cbp420 == 0 with a non-zero
    // coded_block_pattern_1 — the §6.2.5.3 4:2:2 extension path.
    let f = frame_422(64, 48, 0);
    let p = params_422(64, 48);
    let anchor = decoded_anchor(&f, p, 6);
    let mut target = anchor.clone();
    for y in 0..48 {
        if y % 16 < 8 {
            continue; // top chroma block rows stay bit-identical
        }
        for x in 0..32 {
            let v = target.cb.get(x, y).unwrap();
            target.cb.put_sample(x, y, v.saturating_add(24));
        }
    }
    let mut scratch = oxideav_core::bits::BitWriter::new();
    let recon = encode_p_picture(&mut scratch, &target, &anchor, p, 1, 6, 2).expect("encode P");
    let stream = encode_i_then_p(&f, &target, p, 6, 2).expect("encode I+P");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 2);
    let out = &frames[1].frame;
    // Decoder-exact against the encoder's reconstruction.
    assert_planes_equal(out, &recon, 64, 48);
    // The perturbation must actually survive the wire: the decoded
    // bottom chroma rows moved toward the target, away from the anchor.
    let mut moved = 0usize;
    for y in (8..48).filter(|y| y % 16 >= 8) {
        for x in 0..32 {
            let dec = i32::from(out.cb.get(x, y).unwrap());
            let anc = i32::from(anchor.cb.get(x, y).unwrap());
            if (dec - anc) > 8 {
                moved += 1;
            }
        }
    }
    assert!(
        moved > 300,
        "bottom-chroma-block residuals must be transmitted (moved {moved})"
    );
    // Top chroma rows stayed put (their blocks were uncoded).
    for y in (0..48).filter(|y| y % 16 < 8) {
        for x in 0..32 {
            assert_eq!(out.cb.get(x, y), anchor.cb.get(x, y), "top cb ({x},{y})");
        }
    }
}

#[test]
fn ipb_422_group_decodes_in_display_order() {
    let f0 = frame_422(48, 32, 0);
    let f1 = frame_422(48, 32, 2);
    let f2 = frame_422(48, 32, 4);
    let p = params_422(48, 32);
    let stream = encode_i_p_b(&f0, &f1, &f2, p, 6, 2, 2).expect("encode IPB");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 3);
    for (i, want) in [&f0, &f1, &f2].into_iter().enumerate() {
        let out = &frames[i].frame;
        assert_eq!(out.chroma_format, ChromaFormat::Yuv422);
        assert!(mae(&want.y, &out.y, 48, 32) < 5.0, "frame {i} luma MAE");
        assert!(mae(&want.cb, &out.cb, 24, 32) < 6.0, "frame {i} cb MAE");
    }
}

#[test]
fn gop_422_display_order_sequence_roundtrips() {
    // Two GOPs of I B P B P structure at 4:2:2 through the full
    // display-order assembler (GOP headers, temporal_reference reset).
    let frames_in: Vec<FrameBuffer> = (0..7).map(|k| frame_422(64, 48, k)).collect();
    let p = params_422(64, 48);
    let stream =
        encode_display_order_gop_sequence(&frames_in, 1, 2, p, 6, 3, 3).expect("encode GOPs");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 7);
    for (i, want) in frames_in.iter().enumerate() {
        let out = &frames[i].frame;
        assert_eq!(out.chroma_format, ChromaFormat::Yuv422);
        assert!(mae(&want.y, &out.y, 64, 48) < 5.0, "frame {i} luma MAE");
        assert!(mae(&want.cb, &out.cb, 32, 48) < 6.0, "frame {i} cb MAE");
        assert!(mae(&want.cr, &out.cr, 32, 48) < 6.0, "frame {i} cr MAE");
    }
}

/// The r447 feature-flag set: Table B-15 intra AC (`intra_vlc_format`),
/// Table 7-6 non-linear quantiser scale (`q_scale_type`), 10-bit intra
/// DC (`intra_dc_precision = 2`), and the §7.3 alternate scan.
fn params_422_full_flags(width: usize, height: usize) -> IntraPictureParams {
    IntraPictureParams {
        intra_dc_precision: 2,
        intra_vlc_format: true,
        alternate_scan: true,
        q_scale_type: true,
        ..params_422(width, height)
    }
}

#[test]
fn intra_422_full_feature_flags_flat_is_exact() {
    // Flat content quantises to DC-only regardless of the entropy
    // table / scan, so a wrong Table B-15 or alternate-scan emission
    // shows up as a decode failure or wrong samples immediately.
    let mut f = FrameBuffer::new(32, 32, ChromaFormat::Yuv422);
    for y in 0..32 {
        for x in 0..32 {
            f.y.put_sample(x, y, 137);
        }
        for x in 0..16 {
            f.cb.put_sample(x, y, 77);
            f.cr.put_sample(x, y, 201);
        }
    }
    let stream = encode_intra_picture(&f, params_422_full_flags(32, 32), 0, 1).expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 1);
    let out = &frames[0].frame;
    for y in 0..32 {
        for x in 0..32 {
            assert_eq!(out.y.get(x, y), Some(137), "luma ({x},{y})");
        }
        for x in 0..16 {
            assert_eq!(out.cb.get(x, y), Some(77), "cb ({x},{y})");
            assert_eq!(out.cr.get(x, y), Some(201), "cr ({x},{y})");
        }
    }
}

#[test]
fn intra_422_full_feature_flags_structured_roundtrips() {
    // Structured content walks real Table B-15 codewords in
    // alternate-scan order with the non-linear quantiser at a low
    // code (fine quantisation): the round trip must be faithful and
    // idempotent.
    let f = frame_422(64, 48, 1);
    let p = params_422_full_flags(64, 48);
    let stream = encode_intra_picture(&f, p, 0, 8).expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 1);
    let out = &frames[0].frame;
    assert!(mae(&f.y, &out.y, 64, 48) < 4.0, "luma MAE");
    assert!(mae(&f.cb, &out.cb, 32, 48) < 4.0, "cb MAE");
    assert!(mae(&f.cr, &out.cr, 32, 48) < 4.0, "cr MAE");
    // Second-generation convergence. Strict idempotence cannot be
    // demanded at intra_dc_precision = 2: the DC step is
    // intra_dc_mult = 4, while FDCT-ing the u8-rounded reconstruction
    // perturbs the DC by up to 64 · 0.5 / 8 = 4 — more than half a
    // step — so a boundary DC can legitimately move one level. The
    // second generation must still stay within ±1 of the first
    // everywhere (no drift, no shear).
    let stream2 = encode_intra_picture(out, p, 0, 8).expect("encode 2");
    let dec2 = decode_video_sequence(&stream2).expect("decode 2");
    let g2 = &dec2[0].frame;
    for y in 0..48 {
        for x in 0..64 {
            let d =
                (i32::from(out.y.get(x, y).unwrap()) - i32::from(g2.y.get(x, y).unwrap())).abs();
            assert!(d <= 1, "gen-2 luma drift {d} at ({x},{y})");
        }
        for x in 0..32 {
            let dcb =
                (i32::from(out.cb.get(x, y).unwrap()) - i32::from(g2.cb.get(x, y).unwrap())).abs();
            let dcr =
                (i32::from(out.cr.get(x, y).unwrap()) - i32::from(g2.cr.get(x, y).unwrap())).abs();
            assert!(dcb <= 1 && dcr <= 1, "gen-2 chroma drift at ({x},{y})");
        }
    }
}

#[test]
fn intra_422_ten_bit_dc_beats_eight_bit_dc_on_smooth_ramps() {
    // A shallow diagonal ramp is dominated by per-block DC steps; the
    // finer intra_dc_mult of intra_dc_precision = 2 must never lose to
    // precision 0 at the same quantiser (Table 6-13 / §7.4.1).
    let mut f = FrameBuffer::new(64, 48, ChromaFormat::Yuv422);
    for y in 0..48 {
        for x in 0..64 {
            f.y.put_sample(x, y, (60 + (x + y) / 8) as u8);
        }
        for x in 0..32 {
            f.cb.put_sample(x, y, (110 + (x + y) / 12) as u8);
            f.cr.put_sample(x, y, (140 - (x + y) / 12) as u8);
        }
    }
    let p8 = params_422(64, 48);
    let p10 = IntraPictureParams {
        intra_dc_precision: 2,
        ..p8
    };
    let mae8 = {
        let s = encode_intra_picture(&f, p8, 0, 2).expect("encode p8");
        let d = decode_video_sequence(&s).expect("decode p8");
        mae(&f.y, &d[0].frame.y, 64, 48)
    };
    let mae10 = {
        let s = encode_intra_picture(&f, p10, 0, 2).expect("encode p10");
        let d = decode_video_sequence(&s).expect("decode p10");
        mae(&f.y, &d[0].frame.y, 64, 48)
    };
    assert!(
        mae10 <= mae8 + 1e-9,
        "10-bit DC (MAE {mae10}) must not lose to 8-bit DC (MAE {mae8})"
    );
}

#[test]
fn gop_422_full_feature_flags_roundtrips_with_intra_fallback() {
    // A moving sequence whose later frames carry a high-contrast
    // stamp the prediction cannot capture: the P intra fallback fires
    // with Table B-15 + alternate scan + non-linear quant + 10-bit DC
    // active, exercising every flag inside inter pictures too.
    let frames_in: Vec<FrameBuffer> = (0..5)
        .map(|k| {
            let mut f = frame_422(64, 48, k);
            if k >= 2 {
                for y in 8..20 {
                    for x in 8..20 {
                        f.y.put_sample(x, y, if (x + y) % 2 == 0 { 16 } else { 235 });
                    }
                }
            }
            f
        })
        .collect();
    let p = params_422_full_flags(64, 48);
    let stream =
        encode_display_order_gop_sequence(&frames_in, 1, 2, p, 8, 3, 3).expect("encode GOPs");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 5);
    for (i, want) in frames_in.iter().enumerate() {
        let out = &frames[i].frame;
        assert!(mae(&want.y, &out.y, 64, 48) < 6.0, "frame {i} luma MAE");
        assert!(mae(&want.cb, &out.cb, 32, 48) < 6.0, "frame {i} cb MAE");
    }
}

#[test]
fn intra_422_non_multiple_of_16_dimensions_roundtrip() {
    // 100×62: right/bottom edge macroblocks overhang the visible
    // picture; 4:2:2 chroma is 50×62 visible.
    let f = frame_422(100, 62, 0);
    let stream = encode_intra_picture(&f, params_422(100, 62), 0, 6).expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 1);
    let out = &frames[0].frame;
    assert_eq!((out.width, out.height), (100, 62));
    assert_eq!(out.visible_chroma_dims(), (50, 62));
    assert!(mae(&f.y, &out.y, 100, 62) < 4.0, "luma MAE");
    assert!(mae(&f.cb, &out.cb, 50, 62) < 4.0, "cb MAE");
}

#[test]
fn cbr_422_gop_sequence_is_vbv_conformant_and_decodes() {
    // Annex C CBR regulation over the (now chroma-generic) frame
    // encoders at 4:2:2: the emitted stream must satisfy its declared
    // bit_rate / vbv_buffer_size under the whole-stream verifier and
    // decode faithfully.
    use oxideav_mpeg12video::vbv::{verify_cbr_stream, VbvStandard};
    use oxideav_mpeg12video::CbrConfig;

    let frames_in: Vec<FrameBuffer> = (0..5).map(|k| frame_422(64, 48, 2 * k)).collect();
    let cbr = CbrConfig {
        bit_rate_value: 800, // 320 kbit/s (4:2:2 carries more chroma)
        vbv_buffer_size_value: 4,
        frame_rate_code: 3,
        initial_quantiser_scale_code: 6,
    };
    let encoded = oxideav_mpeg12video::encode_cbr_gop_sequence(
        &frames_in,
        1,
        2,
        params_422(64, 48),
        &cbr,
        3,
        3,
    )
    .expect("4:2:2 CBR encode");
    verify_cbr_stream(&encoded.stream, VbvStandard::Mpeg2).expect("Annex C verification");
    let frames = decode_video_sequence(&encoded.stream).expect("decode");
    assert_eq!(frames.len(), 5);
    for (i, want) in frames_in.iter().enumerate() {
        let out = &frames[i].frame;
        assert_eq!(out.chroma_format, ChromaFormat::Yuv422);
        assert!(mae(&want.y, &out.y, 64, 48) < 10.0, "frame {i} luma MAE");
    }
}
