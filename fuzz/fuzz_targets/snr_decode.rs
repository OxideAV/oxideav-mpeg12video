//! Coverage-guided target for the §7.8 SNR two-layer decode loop:
//! a fixed, valid lower layer (built once from deterministic frames
//! by the crate's own encoder) is paired with attacker bytes as the
//! enhancement layer — either raw, or a valid self-encoded
//! enhancement layer with fuzzer-chosen byte corruptions so the deep
//! Table B-8 / block / coefficient-addition paths are reached. The
//! contract is panic-freedom: any error is fine, any panic is a bug.
#![no_main]

use std::sync::OnceLock;

use libfuzzer_sys::fuzz_target;
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    decode_snr_scalable_sequence, encode_display_order_gop_sequence, encode_snr_enhancement_layer,
    FrameBuffer, IntraPictureParams,
};

fn frame_at(t: usize) -> FrameBuffer {
    let (w, h) = (48usize, 32usize);
    let mut f = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
    for y in 0..h {
        for x in 0..w {
            let sx = x + 2 * t;
            let v = 24 + ((sx * 3 + y * 5) % 192) + (sx * 7 + y * 13) % 9;
            f.y.put_sample(x, y, v.min(235) as u8);
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

/// `(base, enhancement)` built once.
fn layers() -> &'static (Vec<u8>, Vec<u8>) {
    static LAYERS: OnceLock<(Vec<u8>, Vec<u8>)> = OnceLock::new();
    LAYERS.get_or_init(|| {
        let sources: Vec<FrameBuffer> = (0..3).map(frame_at).collect();
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
        let base = encode_display_order_gop_sequence(&sources, 1, 2, params, 12, 3, 3)
            .expect("lower layer");
        let enh = encode_snr_enhancement_layer(&base, &sources, 4)
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
        // Raw attacker bytes as the enhancement layer.
        let _ = decode_snr_scalable_sequence(base, &data[1..]);
    } else {
        // The valid enhancement layer with fuzzer-chosen corruptions.
        let mut corrupt = enh.clone();
        let len = corrupt.len();
        for (i, &b) in data.iter().enumerate().skip(1) {
            if let Some(slot) = corrupt.get_mut((i * 7 + usize::from(b)) % len) {
                *slot ^= b;
            }
        }
        // Truncate too, sometimes.
        let cut = if data[0] & 2 != 0 {
            (usize::from(data[0]) * 13) % len
        } else {
            len
        };
        let _ = decode_snr_scalable_sequence(base, &corrupt[..cut]);
    }
});
