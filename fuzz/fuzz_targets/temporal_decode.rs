//! Coverage-guided target for the §7.9 temporal two-layer decode loop:
//! a fixed, valid lower layer (built once by the crate's own encoder)
//! is paired with attacker bytes as the enhancement layer — raw, or a
//! valid self-encoded enhancement layer with fuzzer-chosen byte
//! corruptions / truncation — so the reference-selection, extension
//! and picture paths are reached. Panic-freedom is the contract.
#![no_main]

use std::sync::OnceLock;

use libfuzzer_sys::fuzz_target;
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    decode_temporal_scalable_sequence, encode_display_order_gop_sequence,
    encode_temporal_enhancement_layer, FrameBuffer, IntraPictureParams, TemporalLayerConfig,
};

fn frame_at(t: usize) -> FrameBuffer {
    let (w, h) = (48usize, 32usize);
    let mut f = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
    for y in 0..h {
        for x in 0..w {
            let sx = x + t;
            f.y.put_sample(x, y, (24 + ((sx * 3 + y * 5) % 192)).min(235) as u8);
        }
    }
    for y in 0..h / 2 {
        for x in 0..w / 2 {
            f.cb.put_sample(x, y, (96 + (x + t) % 64) as u8);
            f.cr.put_sample(x, y, (160 - (y % 64)) as u8);
        }
    }
    f
}

fn layers() -> &'static (Vec<u8>, Vec<u8>) {
    static LAYERS: OnceLock<(Vec<u8>, Vec<u8>)> = OnceLock::new();
    LAYERS.get_or_init(|| {
        let lower: Vec<FrameBuffer> = (0..3).map(|j| frame_at(2 * j)).collect();
        let params = IntraPictureParams {
            width: 48,
            height: 32,
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
        let sources: Vec<FrameBuffer> = (0..2).map(|j| frame_at(2 * j + 1)).collect();
        let enh =
            encode_temporal_enhancement_layer(&base, &sources, &TemporalLayerConfig::default())
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
        let _ = decode_temporal_scalable_sequence(base, &data[1..]);
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
        let _ = decode_temporal_scalable_sequence(base, &corrupt[..cut]);
    }
});
