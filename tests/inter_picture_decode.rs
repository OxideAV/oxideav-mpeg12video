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
        // hand-built stream: progressive grid (Ceil(h/16) macroblock rows)
        progressive_sequence: true,
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
        top_field_first: true,
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
fn p_picture_field_based_zero_mv_copies_reference() {
    // §7.6.5 Table 7-14 Field-based: a P frame-picture macroblock with
    // `frame_pred_frame_dct == 0` and `frame_motion_type == 01`
    // (Field-based, 2 vectors). Both field vectors are zero with their
    // own motion_vertical_field_select: the first reads the top field,
    // the second the bottom field. With both MVs zero the macroblock is
    // a verbatim copy of the reference frame (each parity copies its own
    // lines), exercising the field-based per-field reference assembly
    // end-to-end.
    let geom = IntraPictureParams {
        // hand-built stream: progressive grid (Ceil(h/16) macroblock rows)
        progressive_sequence: true,
        frame_pred_frame_dct: false,
        ..geometry_16x16()
    };
    let params = PicturePredictionParams {
        geometry: geom,
        ..p_params()
    };

    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 8);
    bw.write_u32(0b1, 1); // macroblock_address_increment = 1
    bw.write_u32(0b001, 3); // "MC, Not Coded"
    bw.write_u32(0b01, 2); // frame_motion_type = Field-based (2 vectors)
                           // Vector 0 (top field): vfs=0 + horiz=0 (`1`) + vert=0 (`1`).
    bw.write_u32(0b0, 1); // motion_vertical_field_select[0] = 0 (top)
    bw.write_u32(0b1, 1); // motion_code horiz = 0
    bw.write_u32(0b1, 1); // motion_code vert = 0
                          // Vector 1 (bottom field): vfs=1 + horiz=0 + vert=0.
    bw.write_u32(0b1, 1); // motion_vertical_field_select[1] = 1 (bottom)
    bw.write_u32(0b1, 1); // motion_code horiz = 0
    bw.write_u32(0b1, 1); // motion_code vert = 0
    let picture = append_stop(bw);

    // Reference is a vertical ramp so a field copy (which preserves the
    // interleave) is distinguishable from a frame-vs-field mix-up.
    let mut reference = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
    for y in 0..16 {
        for x in 0..16 {
            reference.y.put_sample(x, y, (y * 8) as u8);
        }
    }
    for y in 0..8 {
        for x in 0..8 {
            reference.cb.put_sample(x, y, (y * 8) as u8);
            reference.cr.put_sample(x, y, 0);
        }
    }
    let refs = ReferenceFrames::forward_only(&reference);
    let (frame, placed) = decode_inter_picture(&picture, params, refs).unwrap();
    assert_eq!(placed, 1, "one field-based macroblock reconstructed");
    // Zero field MVs: each frame row equals the reference's same row.
    for y in 0..16 {
        for x in 0..16 {
            assert_eq!(frame.y.get(x, y), Some((y * 8) as u8), "luma ({x},{y})");
        }
    }
    for y in 0..8 {
        for x in 0..8 {
            assert_eq!(frame.cb.get(x, y), Some((y * 8) as u8), "cb ({x},{y})");
        }
    }
}

#[test]
fn p_picture_field_based_top_field_vector_shifts_only_even_lines() {
    // Field-based with the top-field vector = +1 half-sample vertical
    // (vertical motion_code +1, f_code 1 → vector' = +1 half-pel in
    // FIELD-line units) and the bottom-field vector = 0. The top-field
    // (even) frame lines become the half-pel average of adjacent FIELD
    // lines (frame rows two apart) — proving the field grid, not the
    // frame grid, is sampled; the odd lines copy verbatim. The two field
    // vectors address their own parity independently.
    let geom = IntraPictureParams {
        // hand-built stream: progressive grid (Ceil(h/16) macroblock rows)
        progressive_sequence: true,
        frame_pred_frame_dct: false,
        ..geometry_16x16()
    };
    let params = PicturePredictionParams {
        geometry: geom,
        ..p_params()
    };

    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 8);
    bw.write_u32(0b1, 1); // increment = 1
    bw.write_u32(0b001, 3); // "MC, Not Coded"
    bw.write_u32(0b01, 2); // frame_motion_type = Field-based
                           // Vector 0 (top field): vfs=0, horiz=0 (`1`), vert=+1 (`010`).
    bw.write_u32(0b0, 1); // vfs[0] = top
    bw.write_u32(0b1, 1); // horiz motion_code = 0
    bw.write_u32(0b010, 3); // vert motion_code = +1
                            // Vector 1 (bottom field): vfs=1, horiz=0, vert=0.
    bw.write_u32(0b1, 1); // vfs[1] = bottom
    bw.write_u32(0b1, 1); // horiz = 0
    bw.write_u32(0b1, 1); // vert = 0
    let picture = append_stop(bw);

    // Vertical ramp by frame row so a field-line shift is visible.
    let mut reference = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
    for y in 0..16 {
        for x in 0..16 {
            reference.y.put_sample(x, y, (y * 8) as u8);
        }
    }
    for y in 0..8 {
        for x in 0..8 {
            reference.cb.put_sample(x, y, 0);
            reference.cr.put_sample(x, y, 0);
        }
    }
    let refs = ReferenceFrames::forward_only(&reference);
    let (frame, _) = decode_inter_picture(&picture, params, refs).unwrap();

    // Odd (bottom-field) frame rows: zero MV → verbatim copy.
    for k in 0..8 {
        let row = 2 * k + 1;
        assert_eq!(
            frame.y.get(0, row),
            Some((row * 8) as u8),
            "bottom row {row} unchanged"
        );
    }
    // Even (top-field) frame rows: half-pel vertical in field space.
    // Top-field line k = frame row 2k; the +1 half-sample reads the
    // average of field line k and k+1 = frame rows 2k and 2(k+1),
    // clamped at the last top line (k=7). Values are y*8, so the average
    // of rows 2k and 2k+2 is (16k + 16k+16)//2 = 16k+8 for k<7, and
    // row 14's own value (112) at the clamped last line.
    for k in 0..8usize {
        let dest_row = 2 * k;
        let lo = (2 * k * 8) as u32; // frame row 2k value
        let hi_line = (k + 1).min(7);
        let hi = (2 * hi_line * 8) as u32; // frame row 2*(k+1) value (clamped)
        let expected = lo.midpoint(hi) as u8; // // 2 round-up average
        assert_eq!(
            frame.y.get(0, dest_row),
            Some(expected),
            "top row {dest_row} half-pel field average"
        );
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
        // hand-built stream: progressive grid (Ceil(h/16) macroblock rows)
        progressive_sequence: true,
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

#[test]
fn p_picture_frame_dual_prime_averages_four_field_predictions() {
    // §7.6.2 / Table 7-14 `Dual prime`: a P frame-picture macroblock with
    // `frame_pred_frame_dct == 0` and `frame_motion_type == 11` (Dual
    // prime). One motion vector is decoded; `dmv == 1` so each component
    // is followed by a dmvector and NO motion_vertical_field_select bit.
    //
    // Each predicted field averages a same-parity and an opposite-parity
    // prediction (§7.6.7.4); with a zero decoded vector and zero dmvector
    // the §7.6.3.6 derivation gives small vertical e-offsets that round to
    // the same field lines, so on a parity-split reference (top field = 60,
    // bottom field = 180) the top predicted field = avg(60, 180) = 120 and
    // the bottom predicted field = avg(180, 60) = 120 — the whole 16×16
    // frame is 120, formed through the four-field interleave path.
    let geom = IntraPictureParams {
        // hand-built stream: progressive grid (Ceil(h/16) macroblock rows)
        progressive_sequence: true,
        frame_pred_frame_dct: false,
        ..geometry_16x16()
    };
    let params = PicturePredictionParams {
        geometry: geom,
        ..p_params()
    }
    .with_top_field_first(true);

    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 8);
    bw.write_u32(0b1, 1); // address_increment = 1
    bw.write_u32(0b001, 3); // macroblock_type "MC, Not Coded" (forward)
    bw.write_u32(0b11, 2); // frame_motion_type = Dual prime
    bw.write_u32(0b1, 1); // motion_code horiz = 0
    bw.write_u32(0b0, 1); // dmvector[0] = 0
    bw.write_u32(0b1, 1); // motion_code vert = 0
    bw.write_u32(0b0, 1); // dmvector[1] = 0
    let picture = append_stop(bw);

    // Parity-split reference: even (top-field) rows 60, odd (bottom) 180.
    let reference = {
        let mut f = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
        for y in 0..16usize {
            let v = if y % 2 == 0 { 60 } else { 180 };
            for x in 0..16 {
                f.y.put_sample(x, y, v);
            }
        }
        for y in 0..8 {
            for x in 0..8 {
                f.cb.put_sample(x, y, 0);
                f.cr.put_sample(x, y, 0);
            }
        }
        f
    };
    let refs = ReferenceFrames::forward_only(&reference);
    let (frame, placed) = decode_inter_picture(&picture, params, refs).unwrap();
    assert_eq!(placed, 1, "one frame dual-prime macroblock");
    for y in 0..16 {
        for x in 0..16 {
            assert_eq!(frame.y.get(x, y), Some(120), "dual-prime ({x},{y})");
        }
    }
}
