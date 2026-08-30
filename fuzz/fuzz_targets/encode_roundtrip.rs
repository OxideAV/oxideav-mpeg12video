//! Coverage-guided encode→decode round-trip target: the fuzzer picks
//! the geometry, chroma format, entropy flags, GOP shape,
//! `FrameEncodeOptions` and every pixel; the crate's display-order
//! GOP assembler encodes the frames and `decode_video_sequence` must
//! accept the result and return the right frame count. The
//! encoder/decoder contract is the oracle — a decode error or a
//! frame-count mismatch on a self-encoded stream is a bug, as is any
//! panic on the way.
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_mpeg12video::quant_matrix_extension::QuantMatrixExtension;
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    decode_video_sequence, encode_display_order_gop_sequence_with_options, FrameBuffer,
    FrameEncodeOptions, IntraPictureParams,
};

fuzz_target!(|data: &[u8]| {
    if data.len() < 12 {
        return;
    }
    let b = |i: usize| -> usize { usize::from(data[i]) };

    // Geometry: 16..=96 in each axis, non-multiples of 16 included.
    let width = 16 + b(0) % 81;
    let height = 16 + b(1) % 81;
    let chroma_format = match b(2) % 3 {
        0 => ChromaFormat::Yuv420,
        1 => ChromaFormat::Yuv422,
        _ => ChromaFormat::Yuv444,
    };
    let params = IntraPictureParams {
        width,
        height,
        chroma_format,
        frame_pred_frame_dct: true,
        intra_dc_precision: (b(3) % 4) as u8,
        intra_vlc_format: b(4) & 1 != 0,
        alternate_scan: b(4) & 2 != 0,
        q_scale_type: b(4) & 4 != 0,
        progressive_sequence: true,
    };
    let quantiser_scale_code = 1 + (b(5) % 31) as u8;
    let f_code = 1 + (b(6) % 4) as u8;
    let b_between = b(7) % 3;
    let anchors_per_gop = 1 + b(7) / 64;
    let n_frames = 1 + b(8) % 3;
    let options = FrameEncodeOptions {
        skipped_macroblocks: b(9) & 1 != 0,
        concealment_motion_vectors: b(9) & 2 != 0,
        // §6.3.10: in a progressive sequence top_field_first may be 1
        // only with repeat_first_field 1.
        repeat_first_field: b(9) & 4 != 0,
        top_field_first: b(9) & 4 != 0 && b(9) & 8 != 0,
        progressive_frame: None,
    };

    // Frames: every pixel attacker-chosen (cycled), successive frames
    // shifted so motion search has something to chase.
    let pix = &data[10..];
    let frames: Vec<FrameBuffer> = (0..n_frames)
        .map(|t| {
            let mut f = FrameBuffer::new(width, height, chroma_format);
            let (cw, ch) = (f.cb.width(), f.cb.height());
            let mut k = t * 7;
            for y in 0..height {
                for x in 0..width {
                    f.y.put_sample(x, y, pix[k % pix.len()]);
                    k += 1;
                }
            }
            for y in 0..ch {
                for x in 0..cw {
                    f.cb.put_sample(x, y, pix[k % pix.len()]);
                    k += 1;
                    f.cr.put_sample(x, y, pix[k % pix.len()]);
                    k += 1;
                }
            }
            f
        })
        .collect();

    let stream = encode_display_order_gop_sequence_with_options(
        &frames,
        b_between,
        anchors_per_gop,
        params,
        quantiser_scale_code,
        f_code,
        f_code,
        &QuantMatrixExtension::default(),
        &|_| options,
    )
    .expect("assembler accepts every constructed configuration")
    .0;

    let decoded = decode_video_sequence(&stream).expect("self-encoded stream must decode");
    assert_eq!(decoded.len(), n_frames, "display-order frame count");
});
