//! Black-box integration tests for the §6.2.4 slice-level
//! macroblock-header walker per **ISO/IEC 13818-2 (ITU-T H.262)**.
//!
//! These chain the existing [`SliceHeader::parse`] + the new
//! [`walk_slice`] driver end-to-end on hand-built synthetic
//! bitstreams: a slice-start-code-prefixed buffer with the §6.3.16
//! header bits followed by 1..N macroblock-header chains and the
//! §6.2.4 stop pattern.

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::{
    walk_slice, ChromaFormat, MotionVectorsKind, PictureCodingType, PictureStructure, SliceContext,
    SliceHeader, SliceWalkContext, PAST_INTRA_ADDRESS_RESET,
};

/// Slice-header builder for a non-scalable 352×240 picture
/// (`mb_width = 22`, `vertical_size = 240`, no priority breakpoint),
/// slice on row 0, given `quantiser_scale_code`. No intra-slice
/// prelude is written (the `nextbits() == '1'` gate is satisfied by
/// the very first macroblock_type bit if it happens to be `'1'`,
/// but the spec says the prelude `intra_slice_flag` is the same
/// `'1'` bit — we choose to not emit a prelude here so the slice
/// header ends cleanly with `extra_bit_slice = '0'`).
fn write_slice_header(bw: &mut BitWriter, q_scale: u8, mb_row: u8) {
    // 32-bit slice_start_code: 24-bit 0x000001 + slice_vertical_position.
    bw.write_u32(0x00_00_01, 24);
    // svp = mb_row + 1. No slice_vertical_position_extension
    // (vertical_size <= 2800). No priority_breakpoint (no data
    // partitioning).
    bw.write_u32(u32::from(mb_row + 1), 8);
    bw.write_u32(u32::from(q_scale), 5);
    // intra_slice prelude absent: the conditional gate is satisfied
    // only when nextbits() == '1'. We write '0' as the
    // extra_bit_slice terminator.
    bw.write_u32(0, 1);
}

/// Table B-1 increment 1: `1`.
fn write_increment_1(bw: &mut BitWriter) {
    bw.write_u32(0b1, 1);
}

/// Table B-2 (I-pictures): "Intra" = `1`.
fn write_i_intra(bw: &mut BitWriter) {
    bw.write_u32(0b1, 1);
}

/// Table B-2: "Intra, Quant" = `01`.
fn write_i_intra_quant(bw: &mut BitWriter) {
    bw.write_u32(0b01, 2);
}

fn write_q_scale(bw: &mut BitWriter, value: u8) {
    bw.write_u32(u32::from(value), 5);
}

/// Append the §5.2.3 / §6.2.4 stop pattern: pad with zero bits to a
/// byte boundary, then append `0x00 0x00 0x01 0xB7` (the
/// `sequence_end_code` from §6.3.4) so a downstream parser can
/// confirm the bitstream is well-formed even after the do-while
/// exits.
fn append_stop(mut bw: BitWriter) -> Vec<u8> {
    bw.align_to_byte_zero();
    let mut bytes = bw.finish();
    bytes.extend_from_slice(&[0x00, 0x00, 0x00, 0x01, 0xB7]);
    bytes
}

#[test]
fn parse_slice_header_then_walk_macroblocks_in_i_picture() {
    let mut bw = BitWriter::new();
    write_slice_header(&mut bw, 14, 0);
    // Two intra macroblocks.
    write_increment_1(&mut bw);
    write_i_intra(&mut bw);
    write_increment_1(&mut bw);
    write_i_intra(&mut bw);
    let buf = append_stop(bw);

    // Step 1 — parse the slice header.
    let header = SliceHeader::parse(&buf, SliceContext::non_scalable(240)).unwrap();
    assert_eq!(header.quantiser_scale_code, 14);
    assert_eq!(header.mb_row(), 0);
    assert!(header.intra_slice_flag.is_none());

    // Step 2 — the body of the slice starts at
    // header.body_bit_position. We can't currently feed a bit-aligned
    // BitReader to walk_slice, so synthesise a body-only buffer by
    // chopping the input at the byte boundary nearest the
    // body_bit_position. Here the slice header writes exactly
    // 24 + 8 + 5 + 1 = 38 bits — i.e. the body starts at bit 38,
    // not on a byte boundary. The walker is body-buffer based so we
    // re-emit a body-only buffer from the BitWriter helpers.
    let mut body_bw = BitWriter::new();
    write_increment_1(&mut body_bw);
    write_i_intra(&mut body_bw);
    write_increment_1(&mut body_bw);
    write_i_intra(&mut body_bw);
    let body_buf = append_stop(body_bw);

    let walk = walk_slice(
        &body_buf,
        SliceWalkContext::first_slice(
            22,
            header.mb_row(),
            PictureCodingType::Intra,
            header.quantiser_scale_code,
        ),
    )
    .unwrap();

    assert_eq!(walk.macroblocks.len(), 2);
    assert_eq!(walk.macroblocks[0].macroblock_address, 0);
    assert_eq!(walk.macroblocks[1].macroblock_address, 1);
    assert_eq!(walk.past_intra_address, 1);
    assert_eq!(walk.quantiser_scale_code, 14);
}

#[test]
fn walk_records_skipped_macroblocks_in_p_picture() {
    let mut bw = BitWriter::new();
    write_increment_1(&mut bw);
    // Table B-3 P-picture row "Pattern, motion forward" = `1`.
    bw.write_u32(0b1, 1);
    // motion_vectors(0) for the Frame-based default
    // (mv_count == 1, dmv == 0, f_code == 1): two zero motion_codes.
    bw.write_u32(0b11, 2);
    // coded_block_pattern(): cbp = 60 (Table B-9 `111`).
    bw.write_u32(0b111, 3);
    // increment = 3 → skip 2 MBs.
    bw.write_u32(0b010, 3);
    bw.write_u32(0b1, 1);
    bw.write_u32(0b11, 2);
    bw.write_u32(0b111, 3);
    let buf = append_stop(bw);

    let walk = walk_slice(
        &buf,
        SliceWalkContext::first_slice(22, 1, PictureCodingType::Predictive, 8),
    )
    .unwrap();
    assert_eq!(walk.macroblocks.len(), 2);
    // mb_row=1 → starts at addr 22.
    assert_eq!(walk.macroblocks[0].macroblock_address, 22);
    assert_eq!(walk.macroblocks[0].skipped_macroblock_count, 0);
    assert_eq!(walk.macroblocks[1].macroblock_address, 25);
    assert_eq!(walk.macroblocks[1].skipped_macroblock_count, 2);
    assert_eq!(walk.past_intra_address, PAST_INTRA_ADDRESS_RESET);
}

#[test]
fn walk_intra_quant_carry_forward_then_explicit_reset() {
    let mut bw = BitWriter::new();
    // MB0 — Intra-Quant, q=7
    write_increment_1(&mut bw);
    write_i_intra_quant(&mut bw);
    write_q_scale(&mut bw, 7);
    // MB1 — Intra (no quant)
    write_increment_1(&mut bw);
    write_i_intra(&mut bw);
    // MB2 — Intra-Quant, q=15
    write_increment_1(&mut bw);
    write_i_intra_quant(&mut bw);
    write_q_scale(&mut bw, 15);
    let buf = append_stop(bw);

    let walk = walk_slice(
        &buf,
        SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 31),
    )
    .unwrap();
    assert_eq!(walk.macroblocks[0].quantiser_scale_code, 7);
    assert!(walk.macroblocks[0].macroblock_quant_present);
    assert_eq!(walk.macroblocks[1].quantiser_scale_code, 7);
    assert!(!walk.macroblocks[1].macroblock_quant_present);
    assert_eq!(walk.macroblocks[2].quantiser_scale_code, 15);
    assert!(walk.macroblocks[2].macroblock_quant_present);
    assert_eq!(walk.quantiser_scale_code, 15);
    assert_eq!(walk.past_intra_address, 2);
}

#[test]
fn walk_handles_empty_slice_body() {
    // A buffer whose first 23 bits are zero terminates without
    // walking any macroblocks. The §6.2.4 stop-condition matches
    // immediately. The driver returns an empty walk — the higher
    // layer is what rejects empty slices per §6.3.17.1.
    let buf = vec![0x00, 0x00, 0x00, 0x01, 0xB7];
    let walk = walk_slice(
        &buf,
        SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 1),
    )
    .unwrap();
    assert!(walk.macroblocks.is_empty());
    assert_eq!(walk.past_intra_address, PAST_INTRA_ADDRESS_RESET);
    assert_eq!(walk.previous_macroblock_address, -1);
    assert_eq!(walk.quantiser_scale_code, 1);
}

#[test]
fn walk_rejects_first_mb_increment_above_one() {
    let mut bw = BitWriter::new();
    // Table B-1 increment 2 = `011`.
    bw.write_u32(0b011, 3);
    write_i_intra(&mut bw);
    let buf = append_stop(bw);

    let err = walk_slice(
        &buf,
        SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 1),
    )
    .unwrap_err();
    let msg = format!("{err}");
    assert!(msg.contains("first macroblock"));
}

#[test]
fn walk_p_picture_frame_motion_type_advances_past_macroblock_modes_tail() {
    // P-picture frame with `frame_pred_frame_dct == 0`: every
    // motion-bearing MB carries a 2-bit `frame_motion_type` per
    // §6.2.5.1, which the round-32 walker consumes between the
    // 3-bit Table B-3 "MC, Not Coded" macroblock_type and the
    // (absent here) quantiser_scale_code. Without the round-32
    // wiring the walker would mis-advance into the next
    // increment field by 2 bits.
    let mut bw = BitWriter::new();
    write_increment_1(&mut bw);
    // Table B-3 "MC, Not Coded" = `001` (3 bits) — fwd=true,
    // bwd=false, pattern=false, intra=false.
    bw.write_u32(0b001, 3);
    // frame_motion_type = `10` (Frame-based, Table 6-17).
    bw.write_u32(0b10, 2);
    // motion_vectors(0): Frame-based, mv_count=1, dmv=0 →
    // horiz motion_code=0 + vert motion_code=0 = 2 bits `11`.
    bw.write_u32(0b11, 2);
    // Second MB — increment=1 then the same MC pattern again.
    write_increment_1(&mut bw);
    bw.write_u32(0b001, 3);
    // frame_motion_type = `11` (Dual-Prime, mv_count=1, dmv=1).
    bw.write_u32(0b11, 2);
    // motion_vectors(0): Dual-Prime mv_format=Field, mv_count=1,
    // dmv=1 → vfs absent (because dmv==1), so the body is just
    // motion_code+dmvector pairs: horiz (`1`) + dmvector_horiz
    // (`0`) + vert (`1`) + dmvector_vert (`0`) = 4 bits.
    bw.write_u32(0b1010, 4);
    let buf = append_stop(bw);

    let ctx = SliceWalkContext::first_slice_with_picture_extension(
        22,
        0,
        PictureCodingType::Predictive,
        8,
        PictureStructure::Frame,
        false,
    );
    let walk = walk_slice(&buf, ctx).unwrap();
    assert_eq!(walk.macroblocks.len(), 2);
    let mb0_mt = walk.macroblocks[0].motion_type.expect("present");
    assert_eq!(mb0_mt.code, 0b10);
    let mb1_mt = walk.macroblocks[1].motion_type.expect("present");
    assert_eq!(mb1_mt.code, 0b11);
    // body_bit_position records the *post-quant* cursor for each MB
    // — i.e. right after macroblock_modes() since no quant_code is
    // emitted. MB1's body cursor = 1+3+2 (MB0 hdr) + 2 (MB0 MV) +
    // 1+3+2 (MB1 hdr) = 14.
    assert_eq!(walk.macroblocks[1].body_bit_position, 14);
}

#[test]
fn walk_field_picture_field_motion_type_then_quant_in_same_mb() {
    // Top-field P-picture with one MB carrying "MC, Coded, Quant"
    // = Table B-3 row `0001 0` (5 bits). §6.2.5.1 in a field
    // picture: `field_motion_type` is unconditionally read on
    // motion, `dct_type` is gated off (Frame-only). The 5-bit
    // `quantiser_scale_code` follows `macroblock_modes()` per
    // §6.2.5.
    let mut bw = BitWriter::new();
    write_increment_1(&mut bw);
    // P-picture "MC, Coded, Quant" = `00010` (5 bits).
    bw.write_u32(0b00010, 5);
    // field_motion_type = `01` → Field-based, mv_count=1.
    bw.write_u32(0b01, 2);
    // quantiser_scale_code = 23.
    bw.write_u32(23, 5);
    // motion_vectors(0): Field-based mv_count=1, dmv=0 →
    // vertical_field_select (1 bit `0`) + horiz mv_code (1 bit `1`)
    // + vert mv_code (1 bit `1`) = 3 bits `011`.
    bw.write_u32(0b011, 3);
    // coded_block_pattern(): cbp = 60 (Table B-9 `111`).
    bw.write_u32(0b111, 3);
    let buf = append_stop(bw);

    let ctx = SliceWalkContext::first_slice_with_picture_extension(
        22,
        0,
        PictureCodingType::Predictive,
        8,
        PictureStructure::TopField,
        true,
    );
    let walk = walk_slice(&buf, ctx).unwrap();
    assert_eq!(walk.macroblocks.len(), 1);
    let mb0 = &walk.macroblocks[0];
    assert!(mb0.macroblock_type.macroblock_quant);
    let mt = mb0.motion_type.expect("field_motion_type present");
    assert_eq!(mt.code, 0b01);
    assert_eq!(mt.motion_vector_count, 1);
    // No dct_type in a field picture.
    assert!(mb0.dct_type.is_none());
    assert!(mb0.macroblock_quant_present);
    assert_eq!(mb0.quantiser_scale_code, 23);
    // body_bit_position = 1 (inc) + 5 (mb_type) + 2 (field_mt) +
    // 5 (q_scale) = 13 bits.
    assert_eq!(mb0.body_bit_position, 13);
    assert_eq!(walk.quantiser_scale_code, 23);
}

#[test]
fn walk_b_picture_emits_both_motion_vectors_and_no_pattern_code() {
    // B-picture frame, Table B-4 "Interpolated, Not Coded" = `10`
    // (2 bits) — fwd=true, bwd=true, pattern=false, intra=false.
    // The walker reads both `motion_vectors(0)` and
    // `motion_vectors(1)`; there is no CBP and pattern_code is
    // all-zero.
    let mut bw = BitWriter::new();
    write_increment_1(&mut bw);
    bw.write_u32(0b10, 2);
    // motion_vectors(0): Frame-based default → 2 bits.
    bw.write_u32(0b11, 2);
    // motion_vectors(1): Frame-based default → 2 bits.
    bw.write_u32(0b11, 2);
    let buf = append_stop(bw);

    let walk = walk_slice(
        &buf,
        SliceWalkContext::first_slice(22, 0, PictureCodingType::Bidirectional, 8),
    )
    .unwrap();
    assert_eq!(walk.macroblocks.len(), 1);
    let mb0 = &walk.macroblocks[0];
    let mv_fwd = mb0
        .motion_vectors_forward
        .as_ref()
        .expect("motion_vectors(0) emitted");
    let mv_bwd = mb0
        .motion_vectors_backward
        .as_ref()
        .expect("motion_vectors(1) emitted");
    assert_eq!(mv_fwd.kind, MotionVectorsKind::Forward);
    assert_eq!(mv_bwd.kind, MotionVectorsKind::Backward);
    assert_eq!(mb0.pattern_code, [false; 12]);
    assert!(mb0.coded_block_pattern.is_none());
}

#[test]
fn walk_intra_macroblock_with_concealment_motion_vectors_reads_marker_bit() {
    // §6.3.11 concealment_motion_vectors == 1 in an I-picture: every
    // intra MB carries a `motion_vectors(0)` block followed by a
    // single `marker_bit == '1'` per §6.2.5.
    use oxideav_mpeg12video::SliceWalkContext as Ctx;
    let mut bw = BitWriter::new();
    write_increment_1(&mut bw);
    write_i_intra(&mut bw);
    // motion_vectors(0): Frame-based default → 2 bits.
    bw.write_u32(0b11, 2);
    // marker_bit = 1.
    bw.write_u32(0b1, 1);
    let buf = append_stop(bw);

    let walk = walk_slice(
        &buf,
        Ctx::first_slice_with_picture_body(
            22,
            0,
            PictureCodingType::Intra,
            1,
            PictureStructure::Frame,
            true,
            1,
            1,
            1,
            1,
            true,
            ChromaFormat::Yuv420,
        ),
    )
    .unwrap();
    assert_eq!(walk.macroblocks.len(), 1);
    let mb0 = &walk.macroblocks[0];
    assert!(mb0.motion_vectors_forward.is_some());
    assert_eq!(mb0.concealment_marker_bit, Some(true));
    assert_eq!(mb0.pattern_code, [true; 12]);
}

#[test]
fn walk_pattern_code_drives_444_extension() {
    // P-picture frame with `chroma_format = Yuv444` — the
    // `coded_block_pattern_2` 6-bit extension drives blocks 8..12.
    use oxideav_mpeg12video::SliceWalkContext as Ctx;
    let mut bw = BitWriter::new();
    write_increment_1(&mut bw);
    // Table B-3 "Pattern, motion forward" = `1` (1 bit).
    bw.write_u32(0b1, 1);
    // motion_vectors(0): Frame-based default → 2 bits.
    bw.write_u32(0b11, 2);
    // cbp = 63 (Table B-9 6-bit `001100`).
    bw.write_u32(0b001100, 6);
    // coded_block_pattern_2 = `1111` -> blocks 8,9,10,11 set
    // (the `bits` are `coded_block_pattern_2 & (1 << (11 - i))`
    // for i in 8..12 → mask bits 3,2,1,0).
    bw.write_u32(0b1111, 6);
    let buf = append_stop(bw);

    let walk = walk_slice(
        &buf,
        Ctx::first_slice_with_picture_body(
            22,
            0,
            PictureCodingType::Predictive,
            8,
            PictureStructure::Frame,
            true,
            1,
            1,
            1,
            1,
            false,
            ChromaFormat::Yuv444,
        ),
    )
    .unwrap();
    assert_eq!(walk.macroblocks.len(), 1);
    let mb0 = &walk.macroblocks[0];
    let cbp = mb0.coded_block_pattern.as_ref().expect("cbp present");
    assert_eq!(cbp.cbp, 63);
    assert_eq!(cbp.coded_block_pattern_2, Some(0b001111));
    let mut expected = [false; 12];
    // cbp = 63 → blocks 0..6 all set.
    for slot in expected.iter_mut().take(6) {
        *slot = true;
    }
    // cbp2 0b001111 → blocks 8..12 set.
    expected[8] = true;
    expected[9] = true;
    expected[10] = true;
    expected[11] = true;
    assert_eq!(mb0.pattern_code, expected);
}

#[test]
fn walk_intra_frame_picture_emits_dct_type_when_not_frame_pred_frame_dct() {
    // I-picture frame with `frame_pred_frame_dct == 0`: §6.2.5.1
    // dct_type fires on every intra MB. Two MBs alternating
    // dct_type values verifies the walker doesn't mis-align the
    // second MB's increment after consuming the first MB's
    // dct_type bit.
    let mut bw = BitWriter::new();
    write_increment_1(&mut bw);
    write_i_intra(&mut bw);
    // dct_type = 1 (field DCT coded).
    bw.write_u32(0b1, 1);
    write_increment_1(&mut bw);
    write_i_intra(&mut bw);
    // dct_type = 0 (frame DCT coded).
    bw.write_u32(0b0, 1);
    let buf = append_stop(bw);

    let ctx = SliceWalkContext::first_slice_with_picture_extension(
        22,
        0,
        PictureCodingType::Intra,
        1,
        PictureStructure::Frame,
        false,
    );
    let walk = walk_slice(&buf, ctx).unwrap();
    assert_eq!(walk.macroblocks.len(), 2);
    assert_eq!(walk.macroblocks[0].dct_type, Some(true));
    assert_eq!(walk.macroblocks[1].dct_type, Some(false));
    assert_eq!(walk.past_intra_address, 1);
}
