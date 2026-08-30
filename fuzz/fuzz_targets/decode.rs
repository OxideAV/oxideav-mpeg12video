//! Coverage-guided decode target: attacker-controlled bytes through
//! the whole-elementary-stream driver `decode_video_sequence`.
//!
//! Three feeds per iteration so the deep layers are reached often:
//!
//! 1. the raw attacker bytes (start-code scan, sequence-layer
//!    classification, truncation handling);
//! 2. a valid MPEG-2 sequence-header + extension + picture-header
//!    skeleton (borrowed from the crate's own intra encoder output)
//!    with the attacker bytes spliced in as slice/macroblock payload —
//!    the slice walk, VLC decoders, inverse quant, IDCT and motion
//!    compensation see structured garbage;
//! 3. the same splice over an ISO/IEC 11172-2 skeleton so the MPEG-1
//!    block/motion pipeline is exercised too.
//!
//! The contract is panic-freedom: every outcome (Ok or Err) is
//! acceptable, crashing is not.
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    decode_video_sequence, encode_intra_picture, encode_mpeg1_intra_stream, FrameBuffer,
    IntraPictureParams, Mpeg1SequenceParams,
};
use std::sync::OnceLock;

/// A valid stream prefix ending right where slice data begins (the
/// first slice start code is kept, the slice body is the fuzzer's).
fn prefixes() -> &'static (Vec<u8>, Vec<u8>) {
    static PREFIXES: OnceLock<(Vec<u8>, Vec<u8>)> = OnceLock::new();
    PREFIXES.get_or_init(|| {
        let mut frame = FrameBuffer::new(48, 32, ChromaFormat::Yuv420);
        for y in 0..32 {
            for x in 0..48 {
                frame.y.put_sample(x, y, ((x * 5 + y * 3) % 200) as u8);
            }
        }
        for y in 0..16 {
            for x in 0..24 {
                frame.cb.put_sample(x, y, 128);
                frame.cr.put_sample(x, y, 128);
            }
        }
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
        let m2v = encode_intra_picture(&frame, params, 0, 6).expect("valid intra stream");
        let m1v = encode_mpeg1_intra_stream(
            &frame,
            &Mpeg1SequenceParams {
                horizontal_size: 48,
                vertical_size: 32,
                ..Default::default()
            },
            6,
        )
        .expect("valid mpeg1 stream");
        let cut = |s: &[u8]| -> Vec<u8> {
            // Keep everything through the first slice start code
            // (0x000001 [0x01..=0xAF]) plus one byte of header.
            for i in 0..s.len().saturating_sub(4) {
                if s[i] == 0 && s[i + 1] == 0 && s[i + 2] == 1 && (1..=0xAF).contains(&s[i + 3]) {
                    return s[..(i + 5).min(s.len())].to_vec();
                }
            }
            s.to_vec()
        };
        (cut(&m2v), cut(&m1v))
    })
}

fuzz_target!(|data: &[u8]| {
    // 1. Raw bytes.
    let _ = decode_video_sequence(data);

    // 2. MPEG-2 skeleton + attacker slice payload.
    let (m2v, m1v) = prefixes();
    let mut spliced = m2v.clone();
    spliced.extend_from_slice(data);
    spliced.extend_from_slice(&[0x00, 0x00, 0x01, 0xB7]); // sequence_end_code
    let _ = decode_video_sequence(&spliced);

    // 3. MPEG-1 skeleton + attacker slice payload.
    let mut spliced1 = m1v.clone();
    spliced1.extend_from_slice(data);
    spliced1.extend_from_slice(&[0x00, 0x00, 0x01, 0xB7]);
    let _ = decode_video_sequence(&spliced1);
});
