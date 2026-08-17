//! Generate the **self-encoded conformance corpus**
//! (`tests/fixtures/selfenc/`): deterministic synthetic frames pushed
//! through this crate's own MPEG-2 encoder
//! (`encode_intra_picture` / `encode_i_p_chain` / `encode_i_p_b` /
//! `encode_display_order_gop_sequence`) and MPEG-1 encoder
//! (`encode_mpeg1_intra_stream` /
//! `encode_mpeg1_display_order_sequence`), whose output streams are
//! then decoded by a black-box reference decoder to produce the
//! committed `.ref.yuv` files. The paired test
//! `tests/selfenc_conformance.rs` regenerates the streams (pinning
//! the encoder bit-exactly) and holds our own decode against the
//! committed reference decode.
//!
//! Usage: `gen_selfenc_corpus <out-dir>`

use oxideav_mpeg12video::quant_matrix_extension::{QuantMatrixExtension, QuantiserMatrixPayload};
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    encode_cbr_gop_sequence, encode_display_order_gop_sequence,
    encode_display_order_gop_sequence_with_matrices, encode_display_order_sequence,
    encode_ff_display_order_gop_sequence, encode_field_display_order_gop_sequence, encode_i_p_b,
    encode_i_p_chain, encode_intra_picture, encode_mpeg1_cbr_sequence, encode_mpeg1_d_sequence,
    encode_mpeg1_display_order_sequence, encode_mpeg1_intra_stream, CbrConfig, FrameBuffer,
    IntraPictureParams, Mpeg1SequenceParams,
};

/// Deterministic busy frame: diagonal luma gradient + 4×4 checker,
/// plaid chroma, all shifted by `(dx, dy)` so successive frames are a
/// clean translation (predictable by motion compensation) with a
/// fixed high-contrast square that appears at frame 2 (exercising the
/// P intra fallback).
fn frame_at(width: usize, height: usize, dx: usize, dy: usize, stamp: bool) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            let sx = x + dx;
            let sy = y + dy;
            let g = 24 + ((sx * 3 + sy * 5) % 192);
            let c = if (sx / 4 + sy / 4) % 2 == 0 { 16 } else { 0 };
            f.y.put_sample(x, y, (g + c).min(235) as u8);
        }
    }
    if stamp {
        // A high-contrast 12×12 block the translated anchor cannot
        // predict — drives the Table B-3 intra-macroblock fallback.
        for y in 8..20.min(height) {
            for x in 8..20.min(width) {
                f.y.put_sample(x, y, if (x + y) % 2 == 0 { 16 } else { 235 });
            }
        }
    }
    for y in 0..height.div_ceil(2) {
        for x in 0..width.div_ceil(2) {
            f.cb.put_sample(x, y, (96 + (x + dx / 2 + y) % 64) as u8);
            f.cr.put_sample(x, y, (160u8).saturating_sub(((x + y + dy / 2) % 64) as u8));
        }
    }
    f
}

fn params(width: usize, height: usize) -> IntraPictureParams {
    IntraPictureParams {
        width,
        height,
        chroma_format: ChromaFormat::Yuv420,
        frame_pred_frame_dct: true,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        progressive_sequence: true,
    }
}

fn main() {
    let dir = std::env::args()
        .nth(1)
        .expect("usage: gen_selfenc_corpus <out-dir>");
    let write = |name: &str, bytes: &[u8]| {
        let path = format!("{dir}/{name}");
        std::fs::write(&path, bytes).expect("write stream");
        eprintln!("wrote {path}: {} bytes", bytes.len());
    };

    // 1. All-intra picture, 64x48.
    let intra = encode_intra_picture(&frame_at(64, 48, 0, 0, false), params(64, 48), 0, 6)
        .expect("intra encode");
    write("selfenc-intra-64x48.m2v", &intra);

    // 2. All-intra at non-macroblock-multiple 100x62.
    let intra_odd = encode_intra_picture(&frame_at(100, 62, 0, 0, false), params(100, 62), 0, 5)
        .expect("intra 100x62 encode");
    write("selfenc-intra-100x62.m2v", &intra_odd);

    // 3. I + P + P + P chain, translating content plus an unpredictable
    //    stamp on the second P (intra-fallback macroblocks).
    let anchor = frame_at(64, 48, 0, 0, false);
    let targets = [
        frame_at(64, 48, 2, 1, false),
        frame_at(64, 48, 4, 2, true),
        frame_at(64, 48, 6, 3, true),
    ];
    let chain =
        encode_i_p_chain(&anchor, &targets, params(64, 48), 6, 3).expect("i-p-chain encode");
    write("selfenc-ipchain-64x48.m2v", &chain);

    // 5. Whole display-order I B B P B B P sequence (7 frames,
    //    2 B-pictures between anchors), diagonal pan with the stamp
    //    appearing on the middle anchor.
    let display: Vec<FrameBuffer> = (0..7).map(|k| frame_at(64, 48, 2 * k, k, k == 3)).collect();
    let ibbp =
        encode_display_order_sequence(&display, 2, params(64, 48), 6, 3, 3).expect("ibbp encode");
    write("selfenc-ibbp-64x48.m2v", &ibbp);

    // 4. I / B / P group (coded order I, P, B; display I, B, P).
    let ipb = encode_i_p_b(
        &frame_at(64, 48, 0, 0, false),
        &frame_at(64, 48, 2, 1, false),
        &frame_at(64, 48, 4, 2, false),
        params(64, 48),
        6,
        3,
        3,
    )
    .expect("i-p-b encode");
    write("selfenc-ipb-64x48.m2v", &ipb);

    // 6. MPEG-2 GOP-structured sequence: 8 frames, 1 B between
    //    anchors, 2 anchor periods per GOP → GOP headers at display
    //    0 and 5 (I B P B P | I B P), closed GOPs, per-GOP
    //    temporal_reference reset.
    let display: Vec<FrameBuffer> = (0..8).map(|k| frame_at(48, 32, 2 * k, k, false)).collect();
    let gops = encode_display_order_gop_sequence(&display, 1, 2, params(48, 32), 6, 3, 3)
        .expect("mpeg2 gop encode");
    write("selfenc-gops-48x32.m2v", &gops);

    // ---- ISO/IEC 11172-2 (MPEG-1) streams -------------------------

    let mpeg1_seq = |w: u16, h: u16| Mpeg1SequenceParams {
        horizontal_size: w,
        vertical_size: h,
        ..Default::default()
    };

    // 7. MPEG-1 all-intra (one closed GOP, one I picture).
    let m1_intra = encode_mpeg1_intra_stream(&frame_at(64, 48, 0, 0, false), &mpeg1_seq(64, 48), 6)
        .expect("mpeg1 intra encode");
    write("selfenc-mpeg1-intra-64x48.m1v", &m1_intra);

    // 8. MPEG-1 I P P P chain (one GOP, 3 anchor periods, no Bs),
    //    with the intra-fallback stamp from the second P on.
    let display: Vec<FrameBuffer> = vec![
        frame_at(64, 48, 0, 0, false),
        frame_at(64, 48, 2, 1, false),
        frame_at(64, 48, 4, 2, true),
        frame_at(64, 48, 6, 3, true),
    ];
    let m1_ipp = encode_mpeg1_display_order_sequence(&display, 0, 3, &mpeg1_seq(64, 48), 6, 3, 3)
        .expect("mpeg1 ipp encode");
    write("selfenc-mpeg1-ippp-64x48.m1v", &m1_ipp);

    // 9. MPEG-1 two-GOP I B B P | I B B P (8 frames, 2 Bs between
    //    anchors, 1 anchor period per GOP): GOP headers with advancing
    //    time codes, closed GOPs, per-GOP temporal_reference reset.
    let display: Vec<FrameBuffer> = (0..8).map(|k| frame_at(64, 48, 2 * k, k, k == 5)).collect();
    let m1_ibbp = encode_mpeg1_display_order_sequence(&display, 2, 1, &mpeg1_seq(64, 48), 6, 3, 3)
        .expect("mpeg1 ibbp encode");
    write("selfenc-mpeg1-ibbp2gop-64x48.m1v", &m1_ibbp);

    // 10. MPEG-1 I B P group with downloadable §2.4.3.2 quantiser
    //     matrices (flat-ish intra ramp + all-20 non-intra) loaded by
    //     the sequence header.
    let mut intra = [8u8; 64];
    for (i, v) in intra.iter_mut().enumerate().skip(1) {
        *v = 12 + (i as u8 % 8);
    }
    let seq_qmat = Mpeg1SequenceParams {
        intra_quant_matrix: Some(intra),
        non_intra_quant_matrix: Some([20u8; 64]),
        ..mpeg1_seq(48, 32)
    };
    let display: Vec<FrameBuffer> = (0..3).map(|k| frame_at(48, 32, 2 * k, k, false)).collect();
    let m1_qmat = encode_mpeg1_display_order_sequence(&display, 1, 1, &seq_qmat, 6, 3, 3)
        .expect("mpeg1 qmat encode");
    write("selfenc-mpeg1-qmat-48x32.m1v", &m1_qmat);

    // ---- Annex C CBR (rate-controlled) streams --------------------

    // 11. MPEG-2 CBR: 8 frames, I B P B P | I B P GOP structure at
    //     240 kbit/s with a 65 536-bit VBV buffer — every picture
    //     header carries the real §6.3.9 vbv_delay and the stream
    //     satisfies the Annex C occupancy bounds (quantiser
    //     adaptation + zero stuffing).
    let display: Vec<FrameBuffer> = (0..8).map(|k| frame_at(64, 48, 2 * k, k, k == 4)).collect();
    let cbr = CbrConfig {
        bit_rate_value: 600, // 240 kbit/s
        vbv_buffer_size_value: 4,
        frame_rate_code: 3,
        initial_quantiser_scale_code: 6,
    };
    let m2_cbr =
        encode_cbr_gop_sequence(&display, 1, 2, params(64, 48), &cbr, 3, 3).expect("cbr encode");
    write("selfenc-cbr-64x48.m2v", &m2_cbr.stream);

    // 12. MPEG-1 CBR: two-GOP I B B P | I B B P at 240 kbit/s with a
    //     65 536-bit VBV buffer — real §2.4.3.4 vbv_delay values under
    //     the 11172-2 Annex C model (inside the §2.4.3.2 constrained
    //     bounds, so the flag is set).
    let seq_cbr = Mpeg1SequenceParams {
        bit_rate_value: 600,
        vbv_buffer_size_value: 4,
        ..mpeg1_seq(64, 48)
    };
    let display: Vec<FrameBuffer> = (0..8).map(|k| frame_at(64, 48, 2 * k, k, k == 5)).collect();
    let m1_cbr =
        encode_mpeg1_cbr_sequence(&display, 2, 1, &seq_cbr, 6, 3, 3).expect("mpeg1 cbr encode");
    write("selfenc-mpeg1-cbr-64x48.m1v", &m1_cbr.stream);

    // ---- Field-picture (interlaced) stream ------------------------

    // 13. MPEG-2 field-coded I B P B P sequence, 48x64 (fields 48x32):
    //     §6.1.1.4.1 field pairs (top first), field_motion_type = 01
    //     with motion_vertical_field_select over both parities, the
    //     §7.6.2.1 second-P-field synthetic reference, Table B-4
    //     B-field modes. Interlaced-phased content so the two fields
    //     of a frame genuinely differ.
    let field_frames: Vec<FrameBuffer> = (0..5)
        .map(|t| {
            let (w, h) = (48usize, 64usize);
            let mut f = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
            for y in 0..h {
                for x in 0..w {
                    let v = 30 + ((x * 4 + y * 7 + t * 3) % 180);
                    let line = if y % 2 == 0 { 12 } else { 0 };
                    f.y.put_sample(x, y, (v + line).min(235) as u8);
                }
            }
            for y in 0..h / 2 {
                for x in 0..w / 2 {
                    f.cb.put_sample(x, y, (90 + (x + t) % 80) as u8);
                    f.cr.put_sample(x, y, (190u8).saturating_sub(((y + 2 * t) % 80) as u8));
                }
            }
            f
        })
        .collect();
    let field_params = IntraPictureParams {
        width: 48,
        height: 64,
        chroma_format: ChromaFormat::Yuv420,
        frame_pred_frame_dct: false,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        progressive_sequence: false,
    };
    let fieldseq =
        encode_field_display_order_gop_sequence(&field_frames, 1, 2, &field_params, 6, 3, 3)
            .expect("field sequence encode");
    write("selfenc-fieldseq-48x64.m2v", &fieldseq);

    // ---- Frame-picture field-based (frame_pred_frame_dct = 0) ----

    let ff_params = IntraPictureParams {
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

    // 14. Interlaced frame-picture sequence I B P B P (64x64): each
    //     frame's two fields translate in opposite directions, so the
    //     per-macroblock Table 6-17 decision codes Field-based
    //     macroblocks (two field vectors with their own
    //     motion_vertical_field_select) beside Frame-based ones, and
    //     the alternating-field detail exercises the per-macroblock
    //     dct_type (field DCT) decision.
    let ff_frames: Vec<FrameBuffer> = (0..5)
        .map(|t| {
            let (w, h) = (64usize, 64usize);
            let mut f = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
            for y in 0..h {
                // Top field pans right, bottom field pans left.
                let dx = if y % 2 == 0 {
                    2 * t as i32
                } else {
                    -2 * (t as i32)
                };
                for x in 0..w {
                    let sx = (x as i32 - dx).rem_euclid(w as i32) as usize;
                    let v = 40 + ((sx * 5 + (y / 2) * 9) % 160);
                    let line = if y % 2 == 0 { 10 } else { 0 };
                    f.y.put_sample(x, y, (v + line).min(235) as u8);
                }
            }
            for y in 0..h / 2 {
                for x in 0..w / 2 {
                    f.cb.put_sample(x, y, (100 + (x + 2 * t) % 72) as u8);
                    f.cr.put_sample(x, y, (180u8).saturating_sub(((y + 3 * t) % 72) as u8));
                }
            }
            f
        })
        .collect();
    let (ff_stream, ff_stats) =
        encode_ff_display_order_gop_sequence(&ff_frames, 1, 2, &ff_params, 6, 3, 3, false)
            .expect("frame-field sequence encode");
    eprintln!("frame-field stats: {ff_stats:?}");
    assert!(ff_stats.field_mc > 0, "field MC must fire in stream 14");
    assert!(ff_stats.field_dct > 0, "field DCT must fire in stream 14");
    write("selfenc-framefield-64x64.m2v", &ff_stream);

    // 15. Dual-prime I P P (64x64, no B-pictures per §7.6.3.6): the I
    //     reference carries deterministic per-sample noise over a
    //     column-constant base, the P targets are the clean base — the
    //     §7.6.7.4 two-field average halves the reference noise, so
    //     Table 6-17 Dual-prime wins on most macroblocks.
    let noise = |x: usize, y: usize, seed: usize| -> i32 {
        let h = x
            .wrapping_mul(31)
            .wrapping_add(y.wrapping_mul(97))
            .wrapping_add(seed.wrapping_mul(131));
        ((h % 17) as i32) - 8
    };
    let dp_frames: Vec<FrameBuffer> = (0..3)
        .map(|t| {
            let (w, h) = (64usize, 64usize);
            let mut f = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
            for y in 0..h {
                for x in 0..w {
                    let base = 90 + ((x * 7) % 100) as i32;
                    let v = if t == 0 { base + noise(x, y, 1) } else { base };
                    f.y.put_sample(x, y, v.clamp(0, 255) as u8);
                }
            }
            for y in 0..h / 2 {
                for x in 0..w / 2 {
                    f.cb.put_sample(x, y, 128);
                    f.cr.put_sample(x, y, 128);
                }
            }
            f
        })
        .collect();
    let (dp_stream, dp_stats) =
        encode_ff_display_order_gop_sequence(&dp_frames, 0, 2, &ff_params, 6, 3, 3, true)
            .expect("dual-prime sequence encode");
    eprintln!("dual-prime stats: {dp_stats:?}");
    assert!(dp_stats.dual_prime > 0, "dual-prime must fire in stream 15");
    write("selfenc-dualprime-64x64.m2v", &dp_stream);

    // ---- ISO/IEC 11172-2 D-picture stream -------------------------

    // 16. MPEG-1 D-only sequence (§2.4.1): four dc intra-coded
    //     pictures, two per GOP. No black-box reference decode exists
    //     — the reference binary refuses picture_coding_type 4 (the
    //     same limitation as the hand-built mpeg1-dpics conformance
    //     fixture) — so this stream is pinned bit-exactly and decoded
    //     sample-exactly against the encoder's own §2.4.4.1
    //     reconstruction instead.
    let d_frames: Vec<FrameBuffer> = (0..4)
        .map(|t| {
            let (w, h) = (48usize, 32usize);
            let mut f = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
            for y in 0..h {
                for x in 0..w {
                    let mb = (y / 16) * w.div_ceil(16) + x / 16;
                    let v = 40 + 23 * (mb % 8) + 7 * t + (x + y) % 5;
                    f.y.put_sample(x, y, v.min(235) as u8);
                }
            }
            for y in 0..h / 2 {
                for x in 0..w / 2 {
                    f.cb.put_sample(x, y, (90 + x + 3 * t).min(240) as u8);
                    f.cr.put_sample(x, y, (170usize.saturating_sub(y + 2 * t)).max(16) as u8);
                }
            }
            f
        })
        .collect();
    let d_stream = encode_mpeg1_d_sequence(&d_frames, &mpeg1_seq(48, 32), 8, 2)
        .expect("mpeg1 d sequence encode");
    write("selfenc-mpeg1-dpics-48x32.m1v", &d_stream);

    // ---- Adaptive field-picture stream (16x8 MC + dual-prime) -----

    // 17. MPEG-2 field-coded I P P (64x64, fields 64x32, b_between = 0
    //     so §7.6.3.6 admits dual-prime): the I fields carry
    //     deterministic per-sample noise over a column-constant base,
    //     P1 is the clean base (the §7.6.7.4 dual-prime average halves
    //     the reference noise), and P2 shifts alternating 16-frame-line
    //     bands in opposite directions (motion two §7.6.7.3 16x8
    //     region vectors capture but one field vector cannot).
    let fa_noise = |x: usize, y: usize, seed: usize| -> i32 {
        let h = x
            .wrapping_mul(31)
            .wrapping_add(y.wrapping_mul(97))
            .wrapping_add(seed.wrapping_mul(131));
        ((h % 17) as i32) - 8
    };
    let fa_frames: Vec<FrameBuffer> = (0..3)
        .map(|t| {
            let (w, h) = (64usize, 64usize);
            let mut f = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
            for y in 0..h {
                for x in 0..w {
                    let dx: i32 = if t == 2 {
                        if (y / 16) % 2 == 0 {
                            4
                        } else {
                            -4
                        }
                    } else {
                        0
                    };
                    let sx = (x as i32 - dx).clamp(0, w as i32 - 1) as usize;
                    let base = 90 + ((sx * 7) % 100) as i32;
                    // Per-field decorrelated noise: field-line coordinate
                    // + per-parity seed (frame-coordinate noise from this
                    // linear hash correlates the two fields, defeating the
                    // dual-prime average).
                    let v = if t == 0 {
                        base + fa_noise(x, y / 2, 1 + (y % 2))
                    } else {
                        base
                    };
                    f.y.put_sample(x, y, v.clamp(0, 255) as u8);
                }
            }
            for y in 0..h / 2 {
                for x in 0..w / 2 {
                    f.cb.put_sample(x, y, 128);
                    f.cr.put_sample(x, y, 128);
                }
            }
            f
        })
        .collect();
    let fa_params = IntraPictureParams {
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
    let (fa_stream, fa_stats) =
        oxideav_mpeg12video::encode_field_adaptive_display_order_gop_sequence(
            &fa_frames, 0, 2, &fa_params, 6, 3, 3, true,
        )
        .expect("adaptive field sequence encode");
    eprintln!("adaptive field stats: {fa_stats:?}");
    assert!(fa_stats.sixteen_by_eight > 0, "16x8 must fire in stream 17");
    assert!(fa_stats.dual_prime > 0, "dual-prime must fire in stream 17");
    write("selfenc-fieldmodes-64x64.m2v", &fa_stream);

    // ---- 4:2:2 profile streams (round 447) ------------------------

    // 18. 4:2:2 I B P B P (single GOP): Figure 6-11 eight-block
    //     macroblocks, §6.2.5.3 coded_block_pattern_1, §7.6.3.7
    //     horizontal-only chroma MV scaling, High@Main
    //     profile_and_level_indication. Vertical chroma detail so a
    //     4:2:0 collapse would show; the stamp from display frame 3
    //     fires the P intra fallback.
    let f422: Vec<FrameBuffer> = (0..5)
        .map(|t| frame_422_at(64, 48, 2 * t, t >= 3))
        .collect();
    let m422 = encode_display_order_gop_sequence(&f422, 1, 4, params_422(64, 48), 6, 3, 3)
        .expect("4:2:2 gop encode");
    write("selfenc-422-ibbp-64x48.m2v", &m422);

    // 19. 4:2:2 I B P B P with the full r447 flag set — Table B-15
    //     intra AC (intra_vlc_format), §7.3 alternate scan, Table 7-6
    //     non-linear quantiser scale, 10-bit intra DC — plus §6.3.11
    //     downloadable matrices: luminance intra/non-intra loads in
    //     the sequence header and chroma intra/non-intra tables in a
    //     quant_matrix_extension() inside the I picture.
    let full_params = IntraPictureParams {
        intra_dc_precision: 2,
        intra_vlc_format: true,
        alternate_scan: true,
        q_scale_type: true,
        ..params_422(64, 48)
    };
    let mut intra_zz = [0u8; 64];
    intra_zz[0] = 8;
    for (i, v) in intra_zz.iter_mut().enumerate().skip(1) {
        *v = 14 + (i as u8 % 10);
    }
    let matrices = QuantMatrixExtension {
        intra: Some(QuantiserMatrixPayload { bytes: intra_zz }),
        non_intra: Some(QuantiserMatrixPayload { bytes: [18u8; 64] }),
        chroma_intra: Some({
            let mut zz = [24u8; 64];
            zz[0] = 8;
            QuantiserMatrixPayload { bytes: zz }
        }),
        chroma_non_intra: Some(QuantiserMatrixPayload { bytes: [22u8; 64] }),
    };
    let m422_full = encode_display_order_gop_sequence_with_matrices(
        &f422,
        1,
        4,
        full_params,
        8,
        3,
        3,
        &matrices,
    )
    .expect("4:2:2 full-flag encode");
    write("selfenc-422-full-64x48.m2v", &m422_full);
}

/// 4:2:2 params: progressive frame pictures, Figure 6-11 macroblocks.
fn params_422(width: usize, height: usize) -> IntraPictureParams {
    IntraPictureParams {
        chroma_format: ChromaFormat::Yuv422,
        ..params(width, height)
    }
}

/// Deterministic 4:2:2 frame: diagonal luma gradient + checker shifted
/// by `dx`, full-height chroma with per-row detail, optional
/// high-contrast stamp (P intra fallback).
fn frame_422_at(width: usize, height: usize, dx: usize, stamp: bool) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv422);
    for y in 0..height {
        for x in 0..width {
            let sx = x + dx;
            let g = 24 + ((sx * 3 + y * 5) % 192);
            let c = if (sx / 4 + y / 4) % 2 == 0 { 16 } else { 0 };
            f.y.put_sample(x, y, (g + c).min(235) as u8);
        }
    }
    if stamp {
        for y in 8..20.min(height) {
            for x in 8..20.min(width) {
                f.y.put_sample(x, y, if (x + y) % 2 == 0 { 16 } else { 235 });
            }
        }
    }
    for y in 0..height {
        for x in 0..width / 2 {
            f.cb.put_sample(x, y, (64 + (x * 2 + y * 7 + dx / 2) % 128) as u8);
            f.cr.put_sample(
                x,
                y,
                (192u8).saturating_sub(((x * 3 + y * 5 + dx / 2) % 128) as u8),
            );
        }
    }
    f
}
