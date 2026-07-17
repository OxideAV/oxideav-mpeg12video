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

use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    encode_display_order_gop_sequence, encode_display_order_sequence, encode_i_p_b,
    encode_i_p_chain, encode_intra_picture, encode_mpeg1_display_order_sequence,
    encode_mpeg1_intra_stream, FrameBuffer, IntraPictureParams, Mpeg1SequenceParams,
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
}
