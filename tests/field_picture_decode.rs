//! End-to-end **field-picture** simple-field-prediction reconstruction
//! against hand-built synthetic MPEG-2 field-picture slices.
//!
//! In a field picture every prediction is a field prediction (§7.6.1):
//! the whole picture is one field (half the frame height in frame terms)
//! and each macroblock is a single 16×16 field block read from **one**
//! reference field of the reference frame, selected by the §6.3.17.2
//! `motion_vertical_field_select` flag. These fixtures drive the whole
//! §7.6 pipeline — the §6.2.5.1 `field_motion_type` macroblock-modes
//! parse, the §6.2.5.2 `motion_vectors()` with the field-select bit, the
//! §7.6.3 reconstruction, and the §7.6.4 field pel read — through
//! [`decode_field_picture`].
//!
//! Clean-room: only the ISO/IEC 13818-2 syntax (VLC tables, motion-
//! vector reconstruction) is used; no external library source is read.

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::frame_assembly::{FrameBuffer, IntraPictureParams};
use oxideav_mpeg12video::inter_reconstruction::ReferenceFrames;
use oxideav_mpeg12video::picture_header::PictureStructure;
use oxideav_mpeg12video::{
    decode_field_picture, ChromaFormat, PictureCodingType, PicturePredictionParams,
};

/// Field geometry for a 16×16-luma **field** (the field of a 16×32
/// reference frame), 4:2:0, single-macroblock-wide.
fn field_geometry_16x16() -> IntraPictureParams {
    IntraPictureParams {
        // hand-built stream: progressive grid (Ceil(h/16) macroblock rows)
        progressive_sequence: true,
        width: 16,
        height: 16, // FIELD height (the frame is 32 rows)
        chroma_format: ChromaFormat::Yuv420,
        // A field picture forbids frame_pred_frame_dct.
        frame_pred_frame_dct: false,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
    }
}

fn write_slice_header(bw: &mut BitWriter, q_scale: u8) {
    bw.write_u32(0x00_00_01, 24); // slice_start_code prefix
    bw.write_u32(1, 8); // slice_vertical_position = mb_row + 1
    bw.write_u32(u32::from(q_scale), 5);
    bw.write_u32(0, 1); // extra_bit_slice = 0
}

fn append_stop(mut bw: BitWriter) -> Vec<u8> {
    bw.align_to_byte_zero();
    let mut bytes = bw.finish();
    bytes.extend_from_slice(&[0x00, 0x00, 0x01, 0xB7]); // sequence_end_code
    bytes
}

fn p_params() -> PicturePredictionParams {
    PicturePredictionParams {
        geometry: field_geometry_16x16(),
        picture_coding_type: PictureCodingType::Predictive,
        f_code_fwd_horiz: 1,
        f_code_fwd_vert: 1,
        f_code_bwd_horiz: 1,
        f_code_bwd_vert: 1,
        concealment_motion_vectors: false,
        top_field_first: true,
    }
}

/// A 16×32 reference frame whose luma encodes the *frame row* (`y * 4`)
/// so a parity selection is visible: top field = even frame rows, bottom
/// field = odd frame rows. Chroma is zeroed.
fn vertical_ramp_frame() -> FrameBuffer {
    let mut f = FrameBuffer::new(16, 32, ChromaFormat::Yuv420);
    for y in 0..32 {
        for x in 0..16 {
            f.y.put_sample(x, y, (y * 4) as u8);
        }
    }
    for y in 0..16 {
        for x in 0..8 {
            f.cb.put_sample(x, y, 0);
            f.cr.put_sample(x, y, 0);
        }
    }
    f
}

#[test]
fn top_field_picture_zero_mv_reads_even_frame_rows() {
    // One field-picture P macroblock, "MC, Not Coded" (Table B-3 `001`),
    // field_motion_type = Field-based (`01`), motion_vertical_field_select
    // = 0 (top reference field), zero motion vector. The decoded top
    // field's line k must equal the reference's frame row 2k.
    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 8);
    bw.write_u32(0b1, 1); // macroblock_address_increment = 1
    bw.write_u32(0b001, 3); // macroblock_type "MC, Not Coded"
    bw.write_u32(0b01, 2); // field_motion_type = Field-based (1 vector)
    bw.write_u32(0b0, 1); // motion_vertical_field_select = 0 (top)
    bw.write_u32(0b1, 1); // motion_code horiz = 0
    bw.write_u32(0b1, 1); // motion_code vert = 0
    let picture = append_stop(bw);

    let reference = vertical_ramp_frame();
    let refs = ReferenceFrames::forward_only(&reference);
    let (field, placed) =
        decode_field_picture(&picture, p_params(), PictureStructure::TopField, refs).unwrap();

    assert_eq!(placed, 1, "one field-picture macroblock reconstructed");
    // Field line k → reference frame row 2k (top parity), value 2k*4.
    for k in 0..16usize {
        let expected = (2 * k * 4) as u8;
        for x in 0..16 {
            assert_eq!(field.y.get(x, k), Some(expected), "field line {k} col {x}");
        }
    }
}

#[test]
fn top_field_picture_bottom_select_reads_odd_frame_rows() {
    // Same macroblock but motion_vertical_field_select = 1 selects the
    // BOTTOM reference field: even though we are decoding the top field,
    // the prediction is read from the bottom reference field (odd frame
    // rows). This proves the field-select flag — not the picture's own
    // parity — chooses the reference field.
    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 8);
    bw.write_u32(0b1, 1); // increment = 1
    bw.write_u32(0b001, 3); // "MC, Not Coded"
    bw.write_u32(0b01, 2); // field_motion_type = Field-based
    bw.write_u32(0b1, 1); // motion_vertical_field_select = 1 (bottom)
    bw.write_u32(0b1, 1); // motion_code horiz = 0
    bw.write_u32(0b1, 1); // motion_code vert = 0
    let picture = append_stop(bw);

    let reference = vertical_ramp_frame();
    let refs = ReferenceFrames::forward_only(&reference);
    let (field, _) =
        decode_field_picture(&picture, p_params(), PictureStructure::TopField, refs).unwrap();

    // Field line k → reference frame row 2k+1 (bottom parity), value
    // (2k+1)*4.
    for k in 0..16usize {
        let expected = ((2 * k + 1) * 4) as u8;
        assert_eq!(field.y.get(0, k), Some(expected), "field line {k}");
    }
}

#[test]
fn bottom_field_picture_zero_mv_reads_selected_field() {
    // Decode the BOTTOM field picture with a top-field select + zero MV:
    // the destination is the bottom field of the frame, predicted from
    // the top reference field. The bottom field picture is just another
    // field-height buffer; the field-select flag still drives the read.
    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 8);
    bw.write_u32(0b1, 1); // increment = 1
    bw.write_u32(0b001, 3); // "MC, Not Coded"
    bw.write_u32(0b01, 2); // field_motion_type = Field-based
    bw.write_u32(0b0, 1); // field-select = 0 (top reference field)
    bw.write_u32(0b1, 1); // horiz = 0
    bw.write_u32(0b1, 1); // vert = 0
    let picture = append_stop(bw);

    let reference = vertical_ramp_frame();
    let refs = ReferenceFrames::forward_only(&reference);
    let (field, placed) =
        decode_field_picture(&picture, p_params(), PictureStructure::BottomField, refs).unwrap();

    assert_eq!(placed, 1);
    for k in 0..16usize {
        let expected = (2 * k * 4) as u8; // top reference field rows
        assert_eq!(field.y.get(0, k), Some(expected), "field line {k}");
    }
}

#[test]
fn field_picture_half_pel_vertical_averages_field_lines() {
    // motion_code vert = +1 (Table B-10 `010`) → reconstructed vertical
    // = +1 half-sample in FIELD units. The top-field prediction reads the
    // // 2 average of adjacent field lines (reference frame rows two
    // apart), proving the field grid is sampled.
    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 8);
    bw.write_u32(0b1, 1); // increment = 1
    bw.write_u32(0b001, 3); // "MC, Not Coded"
    bw.write_u32(0b01, 2); // field_motion_type = Field-based
    bw.write_u32(0b0, 1); // field-select = 0 (top)
    bw.write_u32(0b1, 1); // horiz motion_code = 0
    bw.write_u32(0b010, 3); // vert motion_code = +1
    let picture = append_stop(bw);

    let reference = vertical_ramp_frame();
    let refs = ReferenceFrames::forward_only(&reference);
    let (field, _) =
        decode_field_picture(&picture, p_params(), PictureStructure::TopField, refs).unwrap();

    for k in 0..16usize {
        let lo = (2 * k * 4) as u32; // field line k = frame row 2k
        let hi_line = (k + 1).min(15); // top field's last line is k=15
        let hi = (2 * hi_line * 4) as u32;
        let expected = lo.midpoint(hi) as u8;
        assert_eq!(field.y.get(0, k), Some(expected), "half-pel field line {k}");
    }
}

#[test]
fn field_picture_two_macroblocks_with_skip() {
    // A 48-wide (3-MB) field picture: MB0 coded ("MC, Not Coded", zero
    // MV, top-field select), then increment = 2 reaching address 2, which
    // skips address 1. The §7.6.6.2 P-picture skip is a (0,0) forward
    // copy reading the same-parity (top) reference field.
    let geom = IntraPictureParams {
        // hand-built stream: progressive grid (Ceil(h/16) macroblock rows)
        progressive_sequence: true,
        width: 48,
        ..field_geometry_16x16()
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
    bw.write_u32(0b01, 2); // field_motion_type = Field-based
    bw.write_u32(0b0, 1); // field-select = top
    bw.write_u32(0b1, 1); // horiz = 0
    bw.write_u32(0b1, 1); // vert = 0
                          // Next coded MB at address 2: increment = 2 (Table B-1 `011`).
    bw.write_u32(0b011, 3); // increment = 2 → skip address 1
    bw.write_u32(0b001, 3); // "MC, Not Coded"
    bw.write_u32(0b01, 2); // field_motion_type = Field-based
    bw.write_u32(0b0, 1); // field-select = top
    bw.write_u32(0b1, 1); // horiz = 0
    bw.write_u32(0b1, 1); // vert = 0
    let picture = append_stop(bw);

    // 48×32 reference, top field = even rows ramp.
    let mut reference = FrameBuffer::new(48, 32, ChromaFormat::Yuv420);
    for y in 0..32 {
        for x in 0..48 {
            reference.y.put_sample(x, y, (y * 4) as u8);
        }
    }
    for y in 0..16 {
        for x in 0..24 {
            reference.cb.put_sample(x, y, 0);
            reference.cr.put_sample(x, y, 0);
        }
    }
    let refs = ReferenceFrames::forward_only(&reference);
    let (field, placed) =
        decode_field_picture(&picture, params, PictureStructure::TopField, refs).unwrap();

    // 2 coded + 1 skipped = 3 macroblocks.
    assert_eq!(placed, 3);
    // Every macroblock (including the skipped MB1, columns 16..32) is a
    // zero-MV top-field copy: field line k = top reference field row 2k.
    for k in 0..16usize {
        let expected = (2 * k * 4) as u8;
        for x in 0..48 {
            assert_eq!(field.y.get(x, k), Some(expected), "x={x} field line {k}");
        }
    }
}

#[test]
fn field_picture_16x8_regions_select_independent_fields_end_to_end() {
    // One field-picture P macroblock with field_motion_type = 16x8 MC
    // (`10`, two vectors). The upper 16×8 region selects the TOP reference
    // field (field-select 0), the lower region the BOTTOM (field-select
    // 1); both zero MV. The decoded field's upper eight lines must equal
    // top-reference-field rows (even frame rows) and the lower eight equal
    // bottom-reference-field rows shifted by the lower region's field-line
    // origin (field line 8 of the bottom field → frame row 2*8+1 = 17).
    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 8);
    bw.write_u32(0b1, 1); // macroblock_address_increment = 1
    bw.write_u32(0b001, 3); // macroblock_type "MC, Not Coded"
    bw.write_u32(0b10, 2); // field_motion_type = 16x8 MC (2 vectors)
                           // motion_vectors(0): upper region (vfs, h, v) then lower (vfs, h, v).
    bw.write_u32(0b0, 1); // upper field-select = 0 (top)
    bw.write_u32(0b1, 1); // upper horiz motion_code = 0
    bw.write_u32(0b1, 1); // upper vert  motion_code = 0
    bw.write_u32(0b1, 1); // lower field-select = 1 (bottom)
    bw.write_u32(0b1, 1); // lower horiz motion_code = 0
    bw.write_u32(0b1, 1); // lower vert  motion_code = 0
    let picture = append_stop(bw);

    let reference = vertical_ramp_frame();
    let refs = ReferenceFrames::forward_only(&reference);
    let (field, placed) =
        decode_field_picture(&picture, p_params(), PictureStructure::TopField, refs).unwrap();

    assert_eq!(placed, 1, "one 16x8 field-picture macroblock reconstructed");
    // Upper region (lines 0..8): top reference field line k → frame row 2k.
    for k in 0..8usize {
        let expected = (2 * k * 4) as u8;
        for x in 0..16 {
            assert_eq!(field.y.get(x, k), Some(expected), "upper line {k} col {x}");
        }
    }
    // Lower region (lines 8..16): bottom reference field line k → frame
    // row 2k+1.
    for k in 8..16usize {
        let expected = ((2 * k + 1) * 4) as u8;
        for x in 0..16 {
            assert_eq!(field.y.get(x, k), Some(expected), "lower line {k} col {x}");
        }
    }
}

#[test]
fn field_picture_16x8_lower_region_uses_distinct_vector_end_to_end() {
    // 16x8 MC with both regions on the TOP field but the lower region a
    // +1 half-sample vertical vector (motion_code +1 = Table B-10 `010`),
    // i.e. a half-pel between adjacent field lines. The lower region's
    // first line (field line 8) reads the // 2 average of top-field line 8
    // (frame row 16, value 64) and line 9 (frame row 18, value 72) = 68 —
    // proving each region uses its own (here half-pel) vector while the
    // upper region stays a full-pel copy.
    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 8);
    bw.write_u32(0b1, 1); // increment = 1
    bw.write_u32(0b001, 3); // "MC, Not Coded"
    bw.write_u32(0b10, 2); // field_motion_type = 16x8 MC
    bw.write_u32(0b0, 1); // upper field-select = top
    bw.write_u32(0b1, 1); // upper horiz = 0
    bw.write_u32(0b1, 1); // upper vert  = 0
    bw.write_u32(0b0, 1); // lower field-select = top
    bw.write_u32(0b1, 1); // lower horiz = 0
    bw.write_u32(0b010, 3); // lower vert motion_code = +1 (one field line)
    let picture = append_stop(bw);

    let reference = vertical_ramp_frame();
    let refs = ReferenceFrames::forward_only(&reference);
    let (field, _) =
        decode_field_picture(&picture, p_params(), PictureStructure::TopField, refs).unwrap();

    // Upper line 0 unchanged: top field line 0 → frame row 0.
    assert_eq!(field.y.get(0, 0), Some(0));
    // Lower line 8: // 2 average of top field lines 8 (64) and 9 (72) = 68.
    assert_eq!(field.y.get(0, 8), Some(68));
}

/// A 16×32 reference frame whose top field (even frame rows) is a solid
/// `top` and bottom field (odd frame rows) a solid `bottom`, so a
/// dual-prime same-/opposite-parity average is directly observable.
fn parity_split_frame(top: u8, bottom: u8) -> FrameBuffer {
    let mut f = FrameBuffer::new(16, 32, ChromaFormat::Yuv420);
    for y in 0..32usize {
        let v = if y % 2 == 0 { top } else { bottom };
        for x in 0..16 {
            f.y.put_sample(x, y, v);
        }
    }
    for y in 0..16 {
        for x in 0..8 {
            f.cb.put_sample(x, y, 0);
            f.cr.put_sample(x, y, 0);
        }
    }
    f
}

#[test]
fn p_field_picture_dual_prime_averages_same_and_opposite_fields() {
    // §7.6.2 / Table 7-13 `Dual prime`: a P-field-picture macroblock with
    // field_motion_type = `11` (Dual prime). Only ONE motion vector is in
    // the bitstream (the same-parity vector); `dmv == 1` so each component
    // is followed by a dmvector (Table B-11) and the
    // motion_vertical_field_select bit is ABSENT. With a zero same-parity
    // vector and zero dmvector the §7.6.3.6 opposite-parity vector is
    // (0, -1) for a top field (e[1][0] = -1) — but reading the bottom
    // reference field at field line 0 with a -1 half-sample (rounds to the
    // same line) still yields the solid bottom constant. §7.6.7.4 then
    // averages the same-parity (top = 80) and opposite-parity (bottom =
    // 160) predictions: (80 + 160) // 2 = 120.
    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 8);
    bw.write_u32(0b1, 1); // macroblock_address_increment = 1
    bw.write_u32(0b001, 3); // macroblock_type "MC, Not Coded" (fwd)
    bw.write_u32(0b11, 2); // field_motion_type = Dual prime
                           // motion_vector(0,0): dmv == 1, no field-select bit.
    bw.write_u32(0b1, 1); // motion_code horiz = 0
    bw.write_u32(0b0, 1); // dmvector[0] = 0 (Table B-11 `0`)
    bw.write_u32(0b1, 1); // motion_code vert = 0
    bw.write_u32(0b0, 1); // dmvector[1] = 0
    let picture = append_stop(bw);

    let reference = parity_split_frame(80, 160);
    let refs = ReferenceFrames::forward_only(&reference);
    let (field, placed) =
        decode_field_picture(&picture, p_params(), PictureStructure::TopField, refs).unwrap();
    assert_eq!(placed, 1, "one dual-prime macroblock reconstructed");
    for k in 0..16usize {
        for x in 0..16 {
            assert_eq!(field.y.get(x, k), Some(120), "dual-prime line {k} col {x}");
        }
    }
}

#[test]
fn b_field_picture_skip_inherits_previous_direction_same_parity() {
    // §7.6.6.3 B-field skip: a coded forward B macroblock at address 0,
    // then a skipped macroblock at address 1 (address_increment = 2 on the
    // *next* coded macroblock would skip it; here we end the slice after a
    // coded MB whose increment leaves a gap). We use two coded MBs with an
    // address gap of one between them so the middle macroblock is skipped
    // and inherits the first's forward, same-parity (top) direction.
    //
    // Reference top field = 90, bottom field = 200. The coded forward MB
    // reads the top field (90); the skipped MB inherits forward + top
    // parity, so it too reads the top field (90).
    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 8);
    // MB0 (address 0): coded forward "MC, Not Coded" B macroblock.
    // Table B-4: macroblock_type with motion_forward only = `010`.
    bw.write_u32(0b1, 1); // address_increment = 1 → address 0
    bw.write_u32(0b010, 3); // macroblock_type forward-only (B)
    bw.write_u32(0b01, 2); // field_motion_type = Field-based (1 vector)
    bw.write_u32(0b0, 1); // motion_vertical_field_select = 0 (top)
    bw.write_u32(0b1, 1); // fwd motion_code horiz = 0
    bw.write_u32(0b1, 1); // fwd motion_code vert = 0
                          // MB2 (address 2): address_increment = 2 skips address 1.
    bw.write_u32(0b011, 3); // address_increment = 2 (Table B-1 `011`)
    bw.write_u32(0b010, 3); // forward-only B macroblock
    bw.write_u32(0b01, 2); // Field-based
    bw.write_u32(0b0, 1); // field-select = top
    bw.write_u32(0b1, 1); // horiz = 0
    bw.write_u32(0b1, 1); // vert = 0
    let picture = append_stop(bw);

    // Use a 3-macroblock-wide field so addresses 0,1,2 are on one row.
    // Reference top field = 90, bottom field = 200.
    let big_ref = {
        let mut f = FrameBuffer::new(48, 32, ChromaFormat::Yuv420);
        for y in 0..32usize {
            let v = if y % 2 == 0 { 90 } else { 200 };
            for x in 0..48 {
                f.y.put_sample(x, y, v);
            }
        }
        for y in 0..16 {
            for x in 0..24 {
                f.cb.put_sample(x, y, 0);
                f.cr.put_sample(x, y, 0);
            }
        }
        f
    };
    let geom = IntraPictureParams {
        // hand-built stream: progressive grid (Ceil(h/16) macroblock rows)
        progressive_sequence: true,
        width: 48,
        ..field_geometry_16x16()
    };
    let params = PicturePredictionParams {
        geometry: geom,
        picture_coding_type: PictureCodingType::Bidirectional,
        ..p_params()
    };
    // A B picture needs both references; use the same frame for both so the
    // forward inheritance reads a known constant either way.
    let refs = ReferenceFrames::bidirectional(&big_ref, &big_ref);
    let (field, placed) =
        decode_field_picture(&picture, params, PictureStructure::TopField, refs).unwrap();
    // 3 macroblocks reconstructed (2 coded + 1 skipped).
    assert_eq!(placed, 3, "two coded + one skipped");
    // All three macroblocks read the top field (90): the coded ones by
    // their field-select bit, the skipped one by §7.6.6.3 same-parity
    // inheritance of the forward direction.
    for mb in 0..3usize {
        let col = mb * 16;
        for k in 0..16usize {
            assert_eq!(field.y.get(col, k), Some(90), "mb {mb} line {k}");
        }
    }
}
