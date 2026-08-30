//! Coverage-guided §7.10 data-partitioning target: attacker-shaped
//! partition pairs through `merge_data_partitions` (panic-freedom),
//! plus a structure-aware feed that splits a valid self-encoded
//! stream at a fuzzer-chosen breakpoint, corrupts one partition with
//! the attacker bytes, and merges — the split/merge engine must
//! reject or accept, never crash.
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    encode_display_order_gop_sequence, merge_data_partitions, split_data_partitions, FrameBuffer,
    IntraPictureParams,
};
use std::sync::OnceLock;

fn partitions() -> &'static Vec<(Vec<u8>, Vec<u8>)> {
    static P: OnceLock<Vec<(Vec<u8>, Vec<u8>)>> = OnceLock::new();
    P.get_or_init(|| {
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
        let frames: Vec<FrameBuffer> = (0..3)
            .map(|t| {
                let mut f = FrameBuffer::new(48, 32, ChromaFormat::Yuv420);
                for y in 0..32 {
                    for x in 0..48 {
                        f.y.put_sample(x, y, ((x * 5 + y * 3 + t * 7) % 200) as u8);
                    }
                }
                for y in 0..16 {
                    for x in 0..24 {
                        f.cb.put_sample(x, y, 128);
                        f.cr.put_sample(x, y, 120);
                    }
                }
                f
            })
            .collect();
        let stream = encode_display_order_gop_sequence(&frames, 1, 2, params, 6, 3, 3)
            .expect("valid stream");
        [1u8, 3, 64, 127]
            .iter()
            .map(|&pb| split_data_partitions(&stream, pb).expect("split"))
            .collect()
    })
}

fuzz_target!(|data: &[u8]| {
    if data.is_empty() {
        return;
    }
    // 1. Raw attacker bytes as both partitions (and split halves).
    let _ = merge_data_partitions(data, data);
    let mid = data.len() / 2;
    let _ = merge_data_partitions(&data[..mid], &data[mid..]);
    let _ = split_data_partitions(data, 64);

    // 2. A valid pair with one partition corrupted by the attacker.
    let pairs = partitions();
    let (p0, p1) = &pairs[usize::from(data[0]) % pairs.len()];
    let mut corrupt = if data[0] & 1 == 0 { p0.clone() } else { p1.clone() };
    let len = corrupt.len();
    for (i, &b) in data.iter().enumerate().skip(1) {
        if let Some(slot) = corrupt.get_mut((i * 7 + usize::from(b)) % len) {
            *slot ^= b;
        }
    }
    let _ = if data[0] & 1 == 0 {
        merge_data_partitions(&corrupt, p1)
    } else {
        merge_data_partitions(p0, &corrupt)
    };
});
