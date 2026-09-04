//! Arbitrary slice structures: ISO/IEC 13818-2 §6.1.2 lets a slice
//! start anywhere in a macroblock row (several slices per row) and
//! ISO/IEC 11172-2 §2.4.1 lets slices start and finish anywhere,
//! spanning rows. The first macroblock of a slice positions itself
//! through its `macroblock_address_increment` against the §6.3.17.1 /
//! §2.4.3.6 reset `previous_macroblock_address = mb_row * mb_width -
//! 1` — it is **not** a run of skipped macroblocks. The slice-length
//! intra encoders emit both shapes; the decoded pictures equal the
//! one-slice-per-row encodes sample for sample (the DC-predictor
//! resets change the coded bits, never the reconstruction).

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::slice_header::{SliceContext, SliceHeader};
use oxideav_mpeg12video::{
    decode_video_sequence, encode_intra_picture, encode_intra_picture_with_slice_length,
    encode_mpeg1_intra_picture, encode_mpeg1_intra_picture_with_slice_length, FrameBuffer,
    IntraPictureParams, Mpeg1PictureParams, SliceWalkContext,
};

fn frame_at(width: usize, height: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            let g = 24 + ((x * 3 + y * 5) % 192);
            let c = if (x / 4 + y / 4) % 2 == 0 { 16 } else { 0 };
            f.y.put_sample(x, y, (g + c).min(235) as u8);
        }
    }
    for y in 0..height.div_ceil(2) {
        for x in 0..width.div_ceil(2) {
            f.cb.put_sample(x, y, (96 + (x + y) % 64) as u8);
            f.cr.put_sample(x, y, (160u8).saturating_sub(((x * 2 + y) % 64) as u8));
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

/// `(offset, slice_vertical_position)` of every slice start code.
fn slices(stream: &[u8]) -> Vec<(usize, u8)> {
    stream
        .windows(4)
        .enumerate()
        .filter(|(_, w)| w[0] == 0 && w[1] == 0 && w[2] == 1 && (0x01..=0xAF).contains(&w[3]))
        .map(|(i, w)| (i, w[3]))
        .collect()
}

fn assert_same_frame(a: &FrameBuffer, b: &FrameBuffer) {
    assert_eq!(a.y.samples(), b.y.samples(), "luma");
    assert_eq!(a.cb.samples(), b.cb.samples(), "cb");
    assert_eq!(a.cr.samples(), b.cr.samples(), "cr");
}

#[test]
fn mpeg2_several_slices_per_row_decode_like_one_slice_per_row() {
    // 64x48: four macroblocks per row; three per slice gives slices of
    // 3 + 1 in every row, the second starting mid-row.
    let f = frame_at(64, 48);
    let reference =
        decode_video_sequence(&encode_intra_picture(&f, params(64, 48), 0, 6).unwrap()).unwrap();
    for per_slice in [1usize, 2, 3, 4, 7] {
        let stream = encode_intra_picture_with_slice_length(&f, params(64, 48), 0, 6, per_slice)
            .expect("slice-length encode");
        let expected_slices: usize = 3 * (4usize).div_ceil(per_slice.min(4));
        let found = slices(&stream);
        assert_eq!(found.len(), expected_slices, "{per_slice} per slice");
        // §6.1.2: every slice stays within one row — the vertical
        // positions repeat within a row and never exceed the picture.
        for (_, svp) in &found {
            assert!((1..=3).contains(svp));
        }
        let decoded = decode_video_sequence(&stream).expect("multi-slice picture decodes");
        assert_eq!(decoded.len(), 1);
        assert_same_frame(&decoded[0].frame, &reference[0].frame);
    }
}

#[test]
fn mpeg1_slices_span_rows_and_start_mid_row() {
    // 64x48 = 4 x 3 macroblocks; five per slice → slices of 5, 5, 2
    // crossing row boundaries and starting mid-row (§2.4.1).
    let f = frame_at(64, 48);
    let p = Mpeg1PictureParams {
        width: 64,
        height: 48,
        intra_quant: oxideav_mpeg12video::DEFAULT_INTRA_QUANT,
        non_intra_quant: [[16u8; 8]; 8],
    };
    let seq = oxideav_mpeg12video::Mpeg1SequenceParams {
        horizontal_size: 64,
        vertical_size: 48,
        ..Default::default()
    };
    let wrap = |picture: Vec<u8>| -> Vec<u8> {
        let mut bw = BitWriter::new();
        oxideav_mpeg12video::write_mpeg1_sequence_header(&mut bw, &seq).unwrap();
        oxideav_mpeg12video::write_gop_header(
            &mut bw,
            &oxideav_mpeg12video::Mpeg2Gop {
                time_code: oxideav_mpeg12video::TimeCode::from_display_index(0, 3).unwrap(),
                closed_gop: true,
                broken_link: false,
            },
        );
        let mut s = bw.finish();
        s.extend_from_slice(&picture);
        s.extend_from_slice(&0x0000_01B7u32.to_be_bytes());
        s
    };
    let mut bw = BitWriter::new();
    let recon_rows = encode_mpeg1_intra_picture(&mut bw, &f, &p, 0, 6).unwrap();
    let rows = wrap(bw.finish());
    let reference = decode_video_sequence(&rows).unwrap();
    assert_same_frame(&reference[0].frame, &recon_rows);

    for per_slice in [1usize, 5, 7, 12, 20] {
        let mut bw = BitWriter::new();
        let recon = encode_mpeg1_intra_picture_with_slice_length(&mut bw, &f, &p, 0, 6, per_slice)
            .expect("slice-length encode");
        assert_same_frame(&recon, &recon_rows);
        let stream = wrap(bw.finish());
        let found = slices(&stream);
        assert_eq!(
            found.len(),
            (12usize).div_ceil(per_slice),
            "{per_slice} per slice"
        );
        if per_slice == 5 {
            // Slices start at macroblocks 0, 5, 10 → rows 1, 2, 3.
            let rows: Vec<u8> = found.iter().map(|(_, s)| *s).collect();
            assert_eq!(rows, vec![1, 2, 3]);
        }
        let decoded = decode_video_sequence(&stream).expect("row-spanning slices decode");
        assert_eq!(decoded.len(), 1);
        assert_same_frame(&decoded[0].frame, &recon_rows);
    }
}

#[test]
fn walker_positions_a_mid_row_first_macroblock_without_skips() {
    // A slice starting at column 2 of row 1 in the 4-wide MPEG-2
    // picture: first increment 3, no skipped macroblocks, address 6.
    let f = frame_at(64, 48);
    let stream = encode_intra_picture_with_slice_length(&f, params(64, 48), 0, 6, 2).unwrap();
    let found = slices(&stream);
    let (off, svp) = found[3]; // row 1, second slice (cols 2..4)
    assert_eq!(svp, 2);
    let end = found.get(4).map(|(o, _)| *o).unwrap_or(stream.len());
    let slice_buf = &stream[off..(end + 4).min(stream.len())];
    let header = SliceHeader::parse(slice_buf, SliceContext::non_scalable(48)).unwrap();
    let ctx = SliceWalkContext::first_slice_with_block_decoding(
        4,
        header.mb_row(),
        oxideav_mpeg12video::PictureCodingType::Intra,
        header.quantiser_scale_code,
        oxideav_mpeg12video::PictureStructure::Frame,
        true,
        15,
        15,
        15,
        15,
        false,
        ChromaFormat::Yuv420,
        false,
        false,
        0,
        false,
    );
    let walk =
        oxideav_mpeg12video::walk_slice_at(slice_buf, header.body_bit_position, ctx).unwrap();
    assert_eq!(walk.macroblocks.len(), 2);
    assert_eq!(walk.macroblocks[0].address_increment, 3);
    assert_eq!(walk.macroblocks[0].macroblock_address, 6);
    assert_eq!(walk.macroblocks[0].skipped_macroblock_count, 0);
    assert_eq!(walk.macroblocks[1].macroblock_address, 7);
}

#[test]
fn zero_macroblocks_per_slice_is_rejected() {
    let f = frame_at(32, 32);
    assert!(encode_intra_picture_with_slice_length(&f, params(32, 32), 0, 6, 0).is_err());
    let p = Mpeg1PictureParams {
        width: 32,
        height: 32,
        intra_quant: oxideav_mpeg12video::DEFAULT_INTRA_QUANT,
        non_intra_quant: [[16u8; 8]; 8],
    };
    let mut bw = BitWriter::new();
    assert!(encode_mpeg1_intra_picture_with_slice_length(&mut bw, &f, &p, 0, 6, 0).is_err());
}
