//! Coverage-guided target for the §7.7 spatial two-layer decode loop:
//! a fixed, valid 2:1 lower layer (built once by the crate's own
//! encoder) is paired with attacker bytes as the enhancement layer —
//! raw, or a valid self-encoded enhancement layer with fuzzer-chosen
//! byte corruptions / truncation — so the Table B-5 / B-6 / B-7
//! walker, the weight-class dispatch and the resampling paths are
//! reached. Panic-freedom is the contract.
#![no_main]

use std::sync::OnceLock;

use libfuzzer_sys::fuzz_target;
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    decode_spatial_scalable_sequence, encode_display_order_gop_sequence,
    encode_spatial_enhancement_layer, FrameBuffer, IntraPictureParams, SpatialLayerConfig,
};

fn full_frame(t: usize) -> FrameBuffer {
    let (w, h) = (48usize, 32usize);
    let mut f = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
    for y in 0..h {
        for x in 0..w {
            let sx = x + 2 * t;
            f.y.put_sample(
                x,
                y,
                (40 + ((sx * 2 + y * 3) % 160) + (sx * 7 + y * 11) % 13).min(235) as u8,
            );
        }
    }
    for y in 0..h / 2 {
        for x in 0..w / 2 {
            f.cb.put_sample(x, y, (96 + (x + y + t) % 64) as u8);
            f.cr.put_sample(x, y, (160 - ((x + t) % 64)) as u8);
        }
    }
    f
}

fn downsample(full: &FrameBuffer) -> FrameBuffer {
    let (w, h) = (full.width / 2, full.height / 2);
    let mut f = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
    for y in 0..h {
        for x in 0..w {
            f.y.put_sample(x, y, full.y.get(2 * x, 2 * y).unwrap());
        }
    }
    for y in 0..h / 2 {
        for x in 0..w / 2 {
            f.cb.put_sample(x, y, full.cb.get(2 * x, 2 * y).unwrap());
            f.cr.put_sample(x, y, full.cr.get(2 * x, 2 * y).unwrap());
        }
    }
    f
}

fn layers() -> &'static (Vec<u8>, Vec<u8>) {
    static LAYERS: OnceLock<(Vec<u8>, Vec<u8>)> = OnceLock::new();
    LAYERS.get_or_init(|| {
        let sources: Vec<FrameBuffer> = (0..3).map(full_frame).collect();
        let lower: Vec<FrameBuffer> = sources.iter().map(downsample).collect();
        let params = IntraPictureParams {
            width: 24,
            height: 16,
            chroma_format: ChromaFormat::Yuv420,
            frame_pred_frame_dct: true,
            intra_dc_precision: 0,
            intra_vlc_format: false,
            alternate_scan: false,
            q_scale_type: false,
            progressive_sequence: true,
        };
        let base =
            encode_display_order_gop_sequence(&lower, 1, 2, params, 8, 3, 3).expect("lower layer");
        let enh = encode_spatial_enhancement_layer(&base, &sources, &SpatialLayerConfig::default())
            .expect("enhancement layer")
            .stream;
        (base, enh)
    })
}

fuzz_target!(|data: &[u8]| {
    if data.is_empty() {
        return;
    }
    let (base, enh) = layers();
    if data[0] & 1 == 0 {
        let _ = decode_spatial_scalable_sequence(base, &data[1..]);
    } else {
        let mut corrupt = enh.clone();
        let len = corrupt.len();
        for (i, &b) in data.iter().enumerate().skip(1) {
            if let Some(slot) = corrupt.get_mut((i * 7 + usize::from(b)) % len) {
                *slot ^= b;
            }
        }
        let cut = if data[0] & 2 != 0 {
            (usize::from(data[0]) * 13) % len
        } else {
            len
        };
        let _ = decode_spatial_scalable_sequence(base, &corrupt[..cut]);
    }
});
