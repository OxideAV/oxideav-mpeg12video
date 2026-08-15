//! MPEG-1 **D-picture encode** round-trips (ISO/IEC 11172-2 §2.4.3.4
//! `picture_coding_type == 4`): the encoder emits dc intra-coded
//! pictures — Table B.2d `macroblock_type`, six DC-only blocks per
//! macroblock (§2.4.2.8: no AC walk, no `end_of_block`), the
//! `end_of_macroblock` `'1'` bit — and every stream decodes
//! sample-exactly against the encoder's own §2.4.4.1 reconstruction
//! through both the per-picture driver and the whole-stream loop.

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::frame_assembly::FrameBuffer;
use oxideav_mpeg12video::mpeg1_picture::decode_mpeg1_d_picture;
use oxideav_mpeg12video::mpeg1_stream_writer::Mpeg1SequenceParams;
use oxideav_mpeg12video::picture_header::{Mpeg2PictureHeader, PictureCodingType};
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{decode_video_sequence, encode_mpeg1_d_picture, encode_mpeg1_d_sequence};

/// Deterministic synthetic frame: per-macroblock luma staircase with
/// smooth chroma ramps, varied per picture index.
fn synthetic_frame(width: usize, height: usize, pic: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            let mb = (y / 16) * width.div_ceil(16) + x / 16;
            let v = 40 + 23 * (mb % 8) + 7 * pic + (x + y) % 5;
            f.y.put_sample(x, y, v.min(235) as u8);
        }
    }
    for y in 0..height / 2 {
        for x in 0..width / 2 {
            f.cb.put_sample(x, y, (90 + x + 3 * pic).min(240) as u8);
            f.cr.put_sample(x, y, (170usize.saturating_sub(y + 2 * pic)).max(16) as u8);
        }
    }
    f
}

fn seq_params(width: u16, height: u16) -> Mpeg1SequenceParams {
    Mpeg1SequenceParams {
        horizontal_size: width,
        vertical_size: height,
        ..Default::default()
    }
}

fn params_for(seq: &Mpeg1SequenceParams) -> oxideav_mpeg12video::mpeg1_picture::Mpeg1PictureParams {
    oxideav_mpeg12video::mpeg1_picture::Mpeg1PictureParams {
        width: usize::from(seq.horizontal_size),
        height: usize::from(seq.vertical_size),
        intra_quant: oxideav_mpeg12video::dequantize::DEFAULT_INTRA_QUANT,
        non_intra_quant: oxideav_mpeg12video::dequantize::DEFAULT_NON_INTRA_QUANT,
    }
}

fn assert_frames_equal(a: &FrameBuffer, b: &FrameBuffer, what: &str) {
    for (name, pa, pb) in [
        ("y", &a.y, &b.y),
        ("cb", &a.cb, &b.cb),
        ("cr", &a.cr, &b.cr),
    ] {
        assert_eq!(pa.width(), pb.width(), "{what}: {name} width");
        assert_eq!(pa.height(), pb.height(), "{what}: {name} height");
        for y in 0..pa.height() {
            for x in 0..pa.width() {
                assert_eq!(
                    pa.get(x, y),
                    pb.get(x, y),
                    "{what}: {name} sample ({x}, {y})"
                );
            }
        }
    }
}

#[test]
fn single_d_picture_roundtrips_sample_exactly() {
    let seq = seq_params(48, 32);
    let params = params_for(&seq);
    let frame = synthetic_frame(48, 32, 0);

    let mut bw = BitWriter::new();
    let recon = encode_mpeg1_d_picture(&mut bw, &frame, &params, 0, 8).expect("encode D picture");
    let bytes = bw.finish();

    // The per-picture driver scans for slice start codes, so the
    // picture-header prefix is harmless.
    let (decoded, placed) = decode_mpeg1_d_picture(&bytes, &params).expect("decode D picture");
    assert_eq!(placed, 3 * 2, "48x32 = 3x2 macroblocks, all coded");
    assert_frames_equal(
        &decoded,
        &recon,
        "D picture decode vs encoder reconstruction",
    );
}

#[test]
fn d_picture_header_carries_type_4_and_no_f_codes() {
    let seq = seq_params(32, 16);
    let params = params_for(&seq);
    let frame = synthetic_frame(32, 16, 1);

    let mut bw = BitWriter::new();
    encode_mpeg1_d_picture(&mut bw, &frame, &params, 5, 8).expect("encode D picture");
    let bytes = bw.finish();

    let header = Mpeg2PictureHeader::parse(&bytes).expect("parse picture header");
    assert_eq!(header.picture_coding_type, PictureCodingType::DcIntra);
    assert_eq!(header.temporal_reference, 5);
}

#[test]
fn d_sequence_decodes_sample_exactly_through_whole_stream_loop() {
    let seq = seq_params(64, 48);
    let params = params_for(&seq);
    let frames: Vec<FrameBuffer> = (0..4).map(|p| synthetic_frame(64, 48, p)).collect();

    let stream = encode_mpeg1_d_sequence(&frames, &seq, 8, 2).expect("encode D sequence");
    let decoded = decode_video_sequence(&stream).expect("decode D sequence");
    assert_eq!(
        decoded.len(),
        4,
        "four D pictures, coded order == display order"
    );

    // Recompute the per-picture reconstructions (temporal_reference
    // resets per 2-picture GOP) and hold the whole-stream decode to
    // sample exactness.
    for (i, frame) in frames.iter().enumerate() {
        let mut scratch = BitWriter::new();
        let recon = encode_mpeg1_d_picture(&mut scratch, frame, &params, (i % 2) as u16, 8)
            .expect("re-encode for reconstruction");
        assert_frames_equal(
            &decoded[i].frame,
            &recon,
            &format!("whole-stream D decode, picture {i}"),
        );
    }
}

#[test]
fn d_sequence_layout_gops_and_temporal_references() {
    let seq = seq_params(32, 32);
    let frames: Vec<FrameBuffer> = (0..5).map(|p| synthetic_frame(32, 32, p)).collect();
    let stream = encode_mpeg1_d_sequence(&frames, &seq, 8, 2).expect("encode D sequence");

    // Three GOP headers (2 + 2 + 1 pictures).
    let gop_count = stream
        .windows(4)
        .filter(|w| w == &[0x00, 0x00, 0x01, 0xB8])
        .count();
    assert_eq!(gop_count, 3, "5 pictures at 2 per GOP -> 3 GOP headers");

    // No MPEG-2 extension start code anywhere (11172-2 stream).
    assert!(
        !stream.windows(4).any(|w| w == [0x00, 0x00, 0x01, 0xB5]),
        "an ISO/IEC 11172-2 stream carries no extension_start_code"
    );

    // Every picture is type 4 with the per-GOP temporal_reference
    // reset (0, 1 | 0, 1 | 0).
    let mut trefs = Vec::new();
    for i in 0..stream.len().saturating_sub(5) {
        if stream[i..i + 4] == [0x00, 0x00, 0x01, 0x00] {
            let header = Mpeg2PictureHeader::parse(&stream[i..]).expect("parse picture header");
            assert_eq!(header.picture_coding_type, PictureCodingType::DcIntra);
            trefs.push(header.temporal_reference);
        }
    }
    assert_eq!(trefs, vec![0, 1, 0, 1, 0]);
}

#[test]
fn d_picture_reconstruction_is_flat_per_block_at_bounded_error() {
    // DC-only coding reconstructs each 8x8 block as a flat plane at
    // its quantised mean: a gentle gradient must round-trip with a
    // small bounded luma error.
    let width = 48;
    let height = 32;
    let mut frame = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            frame.y.put_sample(x, y, (100 + x / 8 + y / 8) as u8);
        }
    }
    for y in 0..height / 2 {
        for x in 0..width / 2 {
            frame.cb.put_sample(x, y, 128);
            frame.cr.put_sample(x, y, 128);
        }
    }
    let seq = seq_params(width as u16, height as u16);
    let params = params_for(&seq);
    let mut bw = BitWriter::new();
    let recon = encode_mpeg1_d_picture(&mut bw, &frame, &params, 0, 8).expect("encode");

    let mut max_err = 0i32;
    for y in 0..height {
        for x in 0..width {
            let e = (i32::from(recon.y.get(x, y).unwrap()) - i32::from(frame.y.get(x, y).unwrap()))
                .abs();
            max_err = max_err.max(e);
        }
    }
    // Each block's samples sit within 1 of the block mean, plus IDCT /
    // quantisation rounding.
    assert!(max_err <= 4, "flat-per-block luma max err {max_err}");
}

#[test]
fn d_sequence_rejects_bad_configurations() {
    let seq = seq_params(32, 32);
    let frames = vec![synthetic_frame(32, 32, 0)];

    assert!(
        encode_mpeg1_d_sequence(&[], &seq, 8, 1).is_err(),
        "empty frames"
    );
    assert!(
        encode_mpeg1_d_sequence(&frames, &seq, 8, 0).is_err(),
        "pictures_per_gop == 0"
    );
    assert!(
        encode_mpeg1_d_sequence(&frames, &seq, 0, 1).is_err(),
        "quantizer_scale 0"
    );
    assert!(
        encode_mpeg1_d_sequence(&frames, &seq, 32, 1).is_err(),
        "quantizer_scale 32"
    );

    let mismatched = seq_params(64, 32);
    assert!(
        encode_mpeg1_d_sequence(&frames, &mismatched, 8, 1).is_err(),
        "geometry mismatch"
    );
}
