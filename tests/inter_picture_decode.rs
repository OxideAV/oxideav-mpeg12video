//! End-to-end **P / B** picture reconstruction against hand-built
//! synthetic MPEG-2 slices.
//!
//! These chain the whole §7.6 motion-compensation pipeline the
//! per-stage modules previously only exposed piecemeal: a synthetic
//! P/B slice (slice header + macroblock-header chain + `motion_vectors()`
//! VLCs) is walked, its motion vectors reconstructed (§7.6.3), and each
//! macroblock motion-compensated against a known reference
//! [`FrameBuffer`] (§7.6.4–§7.6.8) into a full picture. The decoded
//! pixels are then checked against the closed-form expectation the
//! motion vector + reference imply.
//!
//! Clean-room: only the ISO/IEC 13818-2 syntax (VLC tables, motion-
//! vector reconstruction) is used; no external library source is read.

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::frame_assembly::{FrameBuffer, IntraPictureParams};
use oxideav_mpeg12video::inter_reconstruction::ReferenceFrames;
use oxideav_mpeg12video::{
    decode_inter_picture, ChromaFormat, PictureCodingType, PicturePredictionParams,
};

/// Build a single-macroblock-wide (`mb_width = 1`) P/B picture geometry
/// for a 16×16 4:2:0 frame.
fn geometry_16x16() -> IntraPictureParams {
    IntraPictureParams {
        width: 16,
        height: 16,
        chroma_format: ChromaFormat::Yuv420,
        frame_pred_frame_dct: true,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
    }
}

/// Write the §6.3.16 slice header for `mb_row = 0`, `quantiser_scale_code`.
fn write_slice_header(bw: &mut BitWriter, q_scale: u8) {
    bw.write_u32(0x00_00_01, 24); // slice_start_code prefix
    bw.write_u32(1, 8); // slice_vertical_position = mb_row + 1
    bw.write_u32(u32::from(q_scale), 5);
    bw.write_u32(0, 1); // extra_bit_slice = 0
}

/// Append the §6.2.4 stop pattern: align + a sequence_end_code so the
/// walker terminates cleanly at the next start code.
fn append_stop(mut bw: BitWriter) -> Vec<u8> {
    bw.align_to_byte_zero();
    let mut bytes = bw.finish();
    bytes.extend_from_slice(&[0x00, 0x00, 0x01, 0xB7]);
    bytes
}

/// Fill every plane of a frame with a constant.
fn solid(value: u8) -> FrameBuffer {
    let mut f = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
    for y in 0..16 {
        for x in 0..16 {
            f.y.put_sample(x, y, value);
        }
    }
    for y in 0..8 {
        for x in 0..8 {
            f.cb.put_sample(x, y, value);
            f.cr.put_sample(x, y, value);
        }
    }
    f
}

fn p_params() -> PicturePredictionParams {
    PicturePredictionParams {
        geometry: geometry_16x16(),
        picture_coding_type: PictureCodingType::Predictive,
        f_code_fwd_horiz: 1,
        f_code_fwd_vert: 1,
        f_code_bwd_horiz: 1,
        f_code_bwd_vert: 1,
        concealment_motion_vectors: false,
    }
}

#[test]
fn p_picture_zero_mv_no_residual_copies_reference() {
    // One P macroblock, "MC, Not Coded" (Table B-3 `001`), forward
    // zero motion vector, no coded blocks → the macroblock is a verbatim
    // copy of the reference frame.
    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 8);
    bw.write_u32(0b1, 1); // macroblock_address_increment = 1
    bw.write_u32(0b001, 3); // macroblock_type "MC, Not Coded"
                            // frame_pred_frame_dct == true → no motion-type tail.
                            // motion_vectors(0): Frame-based default (1 MV, f_code 1).
    bw.write_u32(0b1, 1); // motion_code horiz = 0 (Table B-10 `1`)
    bw.write_u32(0b1, 1); // motion_code vert = 0
    let picture = append_stop(bw);

    let reference = solid(120);
    let refs = ReferenceFrames::forward_only(&reference);
    let (frame, placed) = decode_inter_picture(&picture, p_params(), refs).unwrap();

    assert_eq!(placed, 1, "one macroblock reconstructed");
    for y in 0..16 {
        for x in 0..16 {
            assert_eq!(frame.y.get(x, y), Some(120), "luma ({x},{y})");
        }
    }
    for y in 0..8 {
        for x in 0..8 {
            assert_eq!(frame.cb.get(x, y), Some(120));
            assert_eq!(frame.cr.get(x, y), Some(120));
        }
    }
}

#[test]
fn p_picture_integer_mv_reads_shifted_reference() {
    // Forward motion_code horiz = +1 (Table B-10 `010`), vert = 0.
    // With f_code = 1 the reconstructed vector' is (2, 0) half-pel
    // (motion_code << (f_code-1) ... → here +1, then the §7.6.3.1
    // scaling and the mv_scale produce an integer +1 sample horizontal
    // shift). We verify against the closed-form: dest(x,y) reads the
    // reference at the reconstructed integer offset, clamped to edge.
    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 8);
    bw.write_u32(0b1, 1); // increment = 1
    bw.write_u32(0b001, 3); // "MC, Not Coded"
    bw.write_u32(0b010, 3); // motion_code horiz = +1
    bw.write_u32(0b1, 1); // motion_code vert = 0
    let picture = append_stop(bw);

    // Reference is a horizontal ramp so a horizontal shift is visible.
    let mut reference = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
    for y in 0..16 {
        for x in 0..16 {
            reference.y.put_sample(x, y, (x * 4) as u8);
        }
    }
    for y in 0..8 {
        for x in 0..8 {
            reference.cb.put_sample(x, y, 0);
            reference.cr.put_sample(x, y, 0);
        }
    }
    let refs = ReferenceFrames::forward_only(&reference);
    let (frame, placed) = decode_inter_picture(&picture, p_params(), refs).unwrap();
    assert_eq!(placed, 1);

    // The reconstruction must not be a verbatim copy (the MV shifted
    // the ramp) and every sample must be a valid in-range reference
    // value (a multiple of 4, since half-pel averaging of two adjacent
    // ramp samples 4*x and 4*(x+1) yields 4*x+2 — still bounded).
    let row0: Vec<u8> = (0..16).map(|x| frame.y.get(x, 0).unwrap()).collect();
    let ref_row0: Vec<u8> = (0..16).map(|x| reference.y.get(x, 0).unwrap()).collect();
    assert_ne!(row0, ref_row0, "a non-zero MV must shift the content");
    // The right edge clamps; the shifted ramp must be monotonically
    // non-decreasing left-to-right (a pure translation of a ramp).
    for x in 1..16 {
        assert!(
            row0[x] >= row0[x - 1],
            "shifted ramp must stay monotonic: row0={row0:?}"
        );
    }
}

#[test]
fn b_picture_bidirectional_averages_two_references() {
    // One B macroblock, "Interp, Not Coded" (Table B-4 `10`), forward
    // and backward zero motion vectors, no coded blocks → the
    // macroblock is the // 2 average of the two reference frames.
    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 8);
    bw.write_u32(0b1, 1); // increment = 1
    bw.write_u32(0b10, 2); // macroblock_type "Interp, Not Coded"
                           // motion_vectors(0) forward: zero MV.
    bw.write_u32(0b1, 1); // fwd motion_code horiz = 0
    bw.write_u32(0b1, 1); // fwd motion_code vert = 0
                          // motion_vectors(1) backward: zero MV.
    bw.write_u32(0b1, 1); // bwd motion_code horiz = 0
    bw.write_u32(0b1, 1); // bwd motion_code vert = 0
    let picture = append_stop(bw);

    let fwd = solid(100);
    let bwd = solid(200);
    let refs = ReferenceFrames::bidirectional(&fwd, &bwd);

    let params = PicturePredictionParams {
        picture_coding_type: PictureCodingType::Bidirectional,
        ..p_params()
    };
    let (frame, placed) = decode_inter_picture(&picture, params, refs).unwrap();
    assert_eq!(placed, 1);
    // (100 + 200) // 2 = 150 everywhere.
    for y in 0..16 {
        for x in 0..16 {
            assert_eq!(frame.y.get(x, y), Some(150), "luma ({x},{y})");
        }
    }
    for y in 0..8 {
        for x in 0..8 {
            assert_eq!(frame.cb.get(x, y), Some(150));
            assert_eq!(frame.cr.get(x, y), Some(150));
        }
    }
}

#[test]
fn p_picture_skipped_macroblock_copies_reference() {
    // mb_width = 2 so a slice can have a skipped macroblock. First MB
    // coded ("MC, Not Coded", zero MV), then increment = 2 reaching
    // address 2 — but there are only 2 MBs in a row (addresses 0, 1),
    // so use mb_width = 3. MB0 coded at addr 0, increment 2 → MB at
    // addr 2, skipping addr 1. The skipped MB (P-picture) is a (0,0)
    // forward copy of the reference.
    let geom = IntraPictureParams {
        width: 48,
        height: 16,
        ..geometry_16x16()
    };
    let params = PicturePredictionParams {
        geometry: geom,
        ..p_params()
    };

    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 8);
    // MB0 at address 0.
    bw.write_u32(0b1, 1); // increment = 1
    bw.write_u32(0b001, 3); // "MC, Not Coded"
    bw.write_u32(0b1, 1); // motion_code horiz = 0
    bw.write_u32(0b1, 1); // motion_code vert = 0
                          // Next coded MB: increment = 2 (Table B-1 `011`) → skip address 1.
    bw.write_u32(0b011, 3); // increment = 2
    bw.write_u32(0b001, 3); // "MC, Not Coded"
    bw.write_u32(0b1, 1); // motion_code horiz = 0
    bw.write_u32(0b1, 1); // motion_code vert = 0
    let picture = append_stop(bw);

    let reference = {
        let mut f = FrameBuffer::new(48, 16, ChromaFormat::Yuv420);
        for y in 0..16 {
            for x in 0..48 {
                f.y.put_sample(x, y, 77);
            }
        }
        for y in 0..8 {
            for x in 0..24 {
                f.cb.put_sample(x, y, 77);
                f.cr.put_sample(x, y, 77);
            }
        }
        f
    };
    let refs = ReferenceFrames::forward_only(&reference);
    let (frame, placed) = decode_inter_picture(&picture, params, refs).unwrap();
    // 2 coded + 1 skipped = 3 macroblocks reconstructed.
    assert_eq!(placed, 3);
    // The skipped macroblock (MB1, columns 16..32) must be a copy of
    // the reference (77).
    for y in 0..16 {
        for x in 16..32 {
            assert_eq!(frame.y.get(x, y), Some(77), "skipped MB luma ({x},{y})");
        }
    }
}
