//! Coverage-guided encode→decode round-trip target: the fuzzer picks
//! the picture structure (progressive frame pictures, field-picture
//! pairs, or `frame_pred_frame_dct = 0` frame pictures), the geometry
//! (including `vertical_size > 2800` tall pictures that exercise the
//! §6.3.16 `slice_vertical_position_extension`), the chroma format on
//! every path, the entropy flags, the GOP shape, `FrameEncodeOptions`
//! and every pixel; the crate's assemblers encode the frames and
//! `decode_video_sequence` must accept the result and return the
//! right frame count. The encoder/decoder contract is the oracle — a
//! decode error or a frame-count mismatch on a self-encoded stream is
//! a bug, as is any panic on the way.
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_mpeg12video::quant_matrix_extension::QuantMatrixExtension;
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    decode_video_sequence, encode_display_order_gop_sequence_with_options,
    encode_ff_display_order_gop_sequence, encode_field_display_order_gop_sequence, FrameBuffer,
    FrameEncodeOptions, IntraPictureParams,
};

fuzz_target!(|data: &[u8]| {
    if data.len() < 12 {
        return;
    }
    let b = |i: usize| -> usize { usize::from(data[i]) };

    // Picture structure: 0 = progressive frame pictures (the widest
    // option surface), 1 = field-picture pairs, 2 = frame pictures
    // with per-macroblock field prediction / field DCT.
    let structure = b(11) % 3;

    // Geometry: 16..=96 in each axis, non-multiples of 16 included;
    // one value in eight selects a tall 16-wide picture whose
    // vertical_size exceeds 2800 (§6.2.4 extension on every slice).
    let tall = b(0) >= 224;
    let (width, height) = if tall {
        (16, 2801 + b(1) % 48)
    } else {
        (16 + b(0) % 81, 16 + b(1) % 81)
    };
    // The field assembler needs a 32-line-multiple frame height.
    let height = if structure == 1 {
        (height.div_ceil(32) * 32).max(32)
    } else {
        height
    };
    let chroma_format = match b(2) % 3 {
        0 => ChromaFormat::Yuv420,
        1 => ChromaFormat::Yuv422,
        _ => ChromaFormat::Yuv444,
    };
    let params = IntraPictureParams {
        width,
        height,
        chroma_format,
        frame_pred_frame_dct: structure == 0,
        intra_dc_precision: (b(3) % 4) as u8,
        intra_vlc_format: b(4) & 1 != 0,
        alternate_scan: b(4) & 2 != 0,
        q_scale_type: b(4) & 4 != 0,
        progressive_sequence: structure == 0,
    };
    let quantiser_scale_code = 1 + (b(5) % 31) as u8;
    let f_code = 1 + (b(6) % 4) as u8;
    let b_between = b(7) % 3;
    let anchors_per_gop = 1 + b(7) / 64;
    // Tall pictures are ~176 macroblock rows each: keep them to one
    // or two frames so an iteration stays cheap.
    let n_frames = if tall { 1 + b(8) % 2 } else { 1 + b(8) % 3 };
    let options = FrameEncodeOptions {
        skipped_macroblocks: b(9) & 1 != 0,
        concealment_motion_vectors: b(9) & 2 != 0,
        // §6.3.10: in a progressive sequence top_field_first may be 1
        // only with repeat_first_field 1.
        repeat_first_field: b(9) & 4 != 0,
        top_field_first: b(9) & 4 != 0 && b(9) & 8 != 0,
        progressive_frame: None,
    };
    // Dual-prime needs b_between == 0 (§7.6.3.6).
    let allow_dual_prime = b(10) & 1 != 0 && b_between == 0;

    // Frames: every pixel attacker-chosen (cycled), successive frames
    // shifted so motion search has something to chase.
    let pix = &data[12..];
    if pix.is_empty() {
        return;
    }
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

    let stream = match structure {
        0 => {
            encode_display_order_gop_sequence_with_options(
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
            .expect("frame assembler accepts every constructed configuration")
            .0
        }
        1 => encode_field_display_order_gop_sequence(
            &frames,
            b_between,
            anchors_per_gop,
            &params,
            quantiser_scale_code,
            f_code,
            f_code,
        )
        .expect("field assembler accepts every constructed configuration"),
        _ => {
            encode_ff_display_order_gop_sequence(
                &frames,
                b_between,
                anchors_per_gop,
                &params,
                quantiser_scale_code,
                f_code,
                f_code,
                allow_dual_prime,
            )
            .expect("frame-field assembler accepts every constructed configuration")
            .0
        }
    };

    let decoded = decode_video_sequence(&stream).expect("self-encoded stream must decode");
    assert_eq!(decoded.len(), n_frames, "display-order frame count");
});
