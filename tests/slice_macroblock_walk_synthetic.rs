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
    walk_slice, ChromaFormat, MotionVectorsKind, Mpeg2ColourComponent, PictureCodingType,
    PictureStructure, QuantMatrixDriver, QuantMatrixExtension, QuantiserMatrixState, SliceContext,
    SliceHeader, SliceWalkContext, DEFAULT_INTRA_WEIGHT, EXTENSION_START_CODE,
    PAST_INTRA_ADDRESS_RESET, QUANT_MATRIX_EXTENSION_ID,
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

// ----- §6.2.6 `block(i)` driver wiring (round 232) ---------------

/// Table B-12 size 0 = `100` (3 bits).
fn write_dc_size_zero_luma(bw: &mut BitWriter) {
    bw.write_u32(0b100, 3);
}
/// Table B-13 size 0 = `00` (2 bits).
fn write_dc_size_zero_chroma(bw: &mut BitWriter) {
    bw.write_u32(0b00, 2);
}
/// Table B-14 EOB = `10` (2 bits).
fn write_eob_b14(bw: &mut BitWriter) {
    bw.write_u32(0b10, 2);
}

/// One §6.2.6 intra block whose DC size is 0 + immediate EOB.
fn write_dc_zero_intra_block(bw: &mut BitWriter, is_luma: bool) {
    if is_luma {
        write_dc_size_zero_luma(bw);
    } else {
        write_dc_size_zero_chroma(bw);
    }
    write_eob_b14(bw);
}

/// Six §6.2.6 intra blocks (4 luma + 1 Cb + 1 Cr) for a 4:2:0 MB.
fn write_dc_zero_intra_macroblock_420(bw: &mut BitWriter) {
    for _ in 0..4 {
        write_dc_zero_intra_block(bw, true);
    }
    write_dc_zero_intra_block(bw, false);
    write_dc_zero_intra_block(bw, false);
}

#[test]
fn walk_slice_with_block_decoding_emits_six_blocks_per_intra_macroblock() {
    // §6.2.4 → §6.2.5 → §6.2.6 from a body-only buffer. The
    // walker's contract is "feed me a buffer starting at the
    // post-slice-header cursor"; the existing integration tests
    // ([`parse_slice_header_then_walk_macroblocks_in_i_picture`])
    // explain the body-only shape — `walk_slice` doesn't accept
    // a bit-aligned cursor, so the body is built independently
    // from the slice header.
    let mut body_bw = BitWriter::new();
    write_increment_1(&mut body_bw);
    write_i_intra(&mut body_bw);
    write_dc_zero_intra_macroblock_420(&mut body_bw);
    let body_buf = append_stop(body_bw);

    let ctx = SliceWalkContext::first_slice_with_block_decoding(
        22,
        0,
        PictureCodingType::Intra,
        14,
        PictureStructure::Frame,
        true,
        1,
        1,
        1,
        1,
        false,
        ChromaFormat::Yuv420,
        false, // intra_vlc_format
        false, // alternate_scan
        0,     // intra_dc_precision
        false, // q_scale_type
    );
    let walk = walk_slice(&body_buf, ctx).unwrap();
    assert_eq!(walk.macroblocks.len(), 1);
    let mb0 = &walk.macroblocks[0];
    let blocks = mb0.decoded_blocks.as_ref().expect("§6.2.6 ran");
    assert_eq!(blocks.len(), 6);
    assert_eq!(blocks[0].component, Mpeg2ColourComponent::Y);
    assert_eq!(blocks[4].component, Mpeg2ColourComponent::Cb);
    assert_eq!(blocks[5].component, Mpeg2ColourComponent::Cr);
    // §7.2.1: with `intra_dc_precision == 0` the DC predictor
    // reset value is 128 (Table 7-2). With every block having
    // `dct_diff == 0` every QFS[0] equals 128.
    for b in blocks {
        assert_eq!(b.decoded.qfs[0], 128);
    }
}

#[test]
fn walk_slice_with_block_decoding_off_keeps_decoded_blocks_none() {
    // Confirm the round-30..33 contract: when the caller uses the
    // existing `first_slice` constructor (block decoding off) the
    // §6.2.6 driver never runs and `decoded_blocks` is `None` on
    // every record.
    let mut body_bw = BitWriter::new();
    write_increment_1(&mut body_bw);
    write_i_intra(&mut body_bw);
    let body_buf = append_stop(body_bw);

    let walk = walk_slice(
        &body_buf,
        SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 14),
    )
    .unwrap();
    assert_eq!(walk.macroblocks.len(), 1);
    assert!(walk.macroblocks[0].decoded_blocks.is_none());
}

#[test]
fn walk_slice_with_block_decoding_chains_two_intra_macroblocks_dc_predictor() {
    // Two §6.2.6-decoded intra MBs in the same slice — the per-slice
    // DC predictor is allocated once and fed across MB boundaries
    // per §7.2.1. Both MBs are DC-only (size 0 → dct_diff = 0), so
    // both Y DC predictors land on the §7.2.1 reset value 128.
    let mut body_bw = BitWriter::new();
    write_increment_1(&mut body_bw);
    write_i_intra(&mut body_bw);
    write_dc_zero_intra_macroblock_420(&mut body_bw);
    write_increment_1(&mut body_bw);
    write_i_intra(&mut body_bw);
    write_dc_zero_intra_macroblock_420(&mut body_bw);
    let body_buf = append_stop(body_bw);

    let ctx = SliceWalkContext::first_slice_with_block_decoding(
        22,
        0,
        PictureCodingType::Intra,
        14,
        PictureStructure::Frame,
        true,
        1,
        1,
        1,
        1,
        false,
        ChromaFormat::Yuv420,
        false,
        false,
        0,
        false,
    );
    let walk = walk_slice(&body_buf, ctx).unwrap();
    assert_eq!(walk.macroblocks.len(), 2);
    let mb0 = walk.macroblocks[0].decoded_blocks.as_ref().unwrap();
    let mb1 = walk.macroblocks[1].decoded_blocks.as_ref().unwrap();
    assert_eq!(mb0.len(), 6);
    assert_eq!(mb1.len(), 6);
    assert_eq!(mb0[0].decoded.qfs[0], 128);
    assert_eq!(mb1[0].decoded.qfs[0], 128);
    // §6.3.17.1: past_intra_address advances to the last intra
    // MB's address.
    assert_eq!(walk.past_intra_address, 1);
}

// ----- §6.3.11 quant_matrix_extension state wiring (round 251) ---

/// Write a single intra Y block carrying one AC coefficient at
/// zig-zag index 1 (`(v, u) = (0, 1)`) with `level == +1`:
///
/// * `dct_dc_size_luminance = 0` (Table B-12 code `100`, 3 bits) —
///   no `dct_dc_differential`, so `QF[0][0]` resolves to the §7.2.1
///   reset value when the predictor is fresh.
/// * `dct_coeff_next` = `(run = 0, level = 1)` (Table B-14 NEXT-form
///   code `11`, 2 bits) followed by the positive-sign bit `0`. The
///   FIRST-form `1s` (1-bit) is rejected by the walker on intra
///   blocks per §7.2.2.2 NOTE 2, so this is the only `(0, +1)` shape
///   the parser will accept here.
/// * `end_of_block` = Table B-14 `10` (2 bits).
///
/// After the §7.2.1 DC consumes index 0 the AC walk advances by
/// `1 + run = 1` per symbol, so `(run=0, level=1)` lands at
/// zig-zag index 1 — which §7.3 maps to `(v, u) = (0, 1)`.
///
/// The companion 5 blocks of a 4:2:0 MB are DC-only, so the matrix
/// element this asserts on is the `[0][1]` entry of the **luma intra**
/// matrix (Table 7-5 `w == 0`).
fn write_intra_y_block_one_ac(bw: &mut BitWriter) {
    write_dc_size_zero_luma(bw);
    // (run=0, level=1): Table B-14 NEXT-form code `11` (2 bits) + sign `0`.
    bw.write_u32(0b11, 2);
    bw.write_bit(false);
    write_eob_b14(bw);
}

/// A 4:2:0 macroblock whose block 0 (luma) carries the AC coefficient
/// from [`write_intra_y_block_one_ac`] and whose remaining 5 blocks are
/// DC-only.
fn write_intra_420_one_ac_then_dc_only(bw: &mut BitWriter) {
    write_intra_y_block_one_ac(bw);
    // Y blocks 1..=3 — DC-only.
    for _ in 0..3 {
        write_dc_zero_intra_block(bw, true);
    }
    // Cb, Cr — DC-only.
    write_dc_zero_intra_block(bw, false);
    write_dc_zero_intra_block(bw, false);
}

#[test]
fn walk_slice_threads_default_quantiser_matrices_through_block_driver() {
    // Baseline: `first_slice_with_block_decoding` carries the
    // §6.3.7 default `intra_luma` matrix (W[0][1] = 16) since no
    // `quant_matrix_extension()` was applied.
    //
    // F''[0][1] = (2 * QF[0][1] + 0) * W[0][1] * quantiser_scale / 32
    //           = (2 * 1) * 16 * 28 / 32 = 28
    // with `quantiser_scale_code = 14, q_scale_type = 0` → Table 7-6
    // linear column gives `quantiser_scale = 28`. The §7.4.4
    // mismatch-control LSB toggle only ever touches `F[7][7]`, so
    // `f_quant[0][1]` is the post-pipeline value verbatim.
    let mut body_bw = BitWriter::new();
    write_increment_1(&mut body_bw);
    write_i_intra(&mut body_bw);
    write_intra_420_one_ac_then_dc_only(&mut body_bw);
    let body_buf = append_stop(body_bw);

    let ctx = SliceWalkContext::first_slice_with_block_decoding(
        22,
        0,
        PictureCodingType::Intra,
        14,
        PictureStructure::Frame,
        true,
        1,
        1,
        1,
        1,
        false,
        ChromaFormat::Yuv420,
        false, // intra_vlc_format = 0 → Table B-14 path
        false,
        0,
        false,
    );
    assert_eq!(ctx.quantiser_matrices, QuantiserMatrixState::defaults());
    let walk = walk_slice(&body_buf, ctx).unwrap();
    let blocks = walk.macroblocks[0]
        .decoded_blocks
        .as_ref()
        .expect("§6.2.6 ran");
    assert_eq!(blocks.len(), 6);
    let y0 = &blocks[0];
    assert_eq!(y0.component, Mpeg2ColourComponent::Y);
    // §7.3 inverse scan puts `(run=1, level=1)` at QF[0][1] = +1.
    assert_eq!(y0.decoded.qf[0][1], 1);
    // §7.4.2.3 reconstruction with the §6.3.7 default W[0][1] = 16.
    assert_eq!(y0.decoded.f_quant[0][1], 28);
}

#[test]
fn walk_slice_threads_custom_quantiser_matrices_through_block_driver() {
    // Same bitstream as the baseline above, but the SliceWalkContext
    // is chained with a custom QuantiserMatrixState whose
    // `intra_luma[0][1]` cell is overridden to 80 (vs. the §6.3.7
    // default 16). The walker must forward the override into the
    // §6.2.6 driver's `MacroblockBlockContext::weight_matrices`
    // verbatim so the §7.4.2.3 reconstruction step picks up the
    // changed entry.
    //
    // F''[0][1] = (2 * 1) * 80 * 28 / 32 = 140
    // — i.e. five times the baseline value. The other matrix cells
    // are left at their defaults so the QF / f_pel of the
    // surrounding DC-only blocks does not change (DC bypasses W via
    // the §7.4.1 intra_dc_mult path).
    let mut body_bw = BitWriter::new();
    write_increment_1(&mut body_bw);
    write_i_intra(&mut body_bw);
    write_intra_420_one_ac_then_dc_only(&mut body_bw);
    let body_buf = append_stop(body_bw);

    let mut matrices = QuantiserMatrixState::defaults();
    matrices.intra_luma[0][1] = 80;

    let ctx = SliceWalkContext::first_slice_with_block_decoding(
        22,
        0,
        PictureCodingType::Intra,
        14,
        PictureStructure::Frame,
        true,
        1,
        1,
        1,
        1,
        false,
        ChromaFormat::Yuv420,
        false,
        false,
        0,
        false,
    )
    .with_quantiser_matrices(matrices);
    assert_eq!(ctx.quantiser_matrices.intra_luma[0][1], 80);
    // §6.3.11: other defaults survive the override.
    assert_eq!(
        ctx.quantiser_matrices.non_intra_luma,
        QuantiserMatrixState::defaults().non_intra_luma
    );

    let walk = walk_slice(&body_buf, ctx).unwrap();
    let blocks = walk.macroblocks[0]
        .decoded_blocks
        .as_ref()
        .expect("§6.2.6 ran");
    let y0 = &blocks[0];
    assert_eq!(y0.decoded.qf[0][1], 1);
    // §7.4.2.3 with the overridden W[0][1] = 80. The default of 16
    // would give 28 (asserted in the baseline test above), so a
    // mismatch here would mean the matrix is **not** being
    // forwarded through the SliceWalkContext → MacroblockBlockContext
    // boundary the new wiring just introduced.
    assert_eq!(y0.decoded.f_quant[0][1], 140);
}

#[test]
fn slice_walk_context_quantiser_matrices_default_matches_table_7_5_defaults() {
    // Sanity: every constructor seeds `quantiser_matrices` to the
    // §6.3.7 defaults (the same four matrices `DEFAULT_INTRA_WEIGHT`
    // / `DEFAULT_NON_INTRA_WEIGHT` exposed at the crate root).
    let ctx_first_slice = SliceWalkContext::first_slice(22, 0, PictureCodingType::Intra, 1);
    assert_eq!(
        ctx_first_slice.quantiser_matrices.intra_luma,
        DEFAULT_INTRA_WEIGHT
    );
    let ctx_mpeg1 = SliceWalkContext::first_slice_mpeg1(22, 0, PictureCodingType::Intra, 1);
    assert_eq!(
        ctx_mpeg1.quantiser_matrices,
        QuantiserMatrixState::defaults()
    );
    let ctx_block = SliceWalkContext::first_slice_with_block_decoding(
        22,
        0,
        PictureCodingType::Intra,
        14,
        PictureStructure::Frame,
        true,
        1,
        1,
        1,
        1,
        false,
        ChromaFormat::Yuv420,
        false,
        false,
        0,
        false,
    );
    assert_eq!(
        ctx_block.quantiser_matrices,
        QuantiserMatrixState::defaults()
    );
}

/// Build a synthetic `quant_matrix_extension()` that loads a single
/// luminance intra matrix where every cell except `[0][0]` equals
/// `target_value`; the `[0][0]` cell remains `8` per the §6.3.11
/// "first value shall always be 8" rule. The §7.3.1 inverse zigzag
/// puts the first zigzag byte (`bytes[0]`) at `[0][0]` and the
/// second zigzag byte (`bytes[1]`) at `[0][1]`, so a uniform
/// non-`[0][0]` payload makes the post-decode `intra_luma[0][1]` an
/// independently-checkable footprint of the extension.
fn write_quant_matrix_extension_intra_only(bw: &mut BitWriter, target_value: u8) {
    bw.write_u32(EXTENSION_START_CODE, 32);
    bw.write_u32(QUANT_MATRIX_EXTENSION_ID, 4);
    bw.write_bit(true); // load_intra_quantiser_matrix
    bw.write_u32(8, 8); // bytes[0] — first value shall be 8 (§6.3.11)
    for _ in 1..64 {
        bw.write_u32(u32::from(target_value), 8);
    }
    bw.write_bit(false); // load_non_intra_quantiser_matrix
    bw.write_bit(false); // load_chroma_intra_quantiser_matrix
    bw.write_bit(false); // load_chroma_non_intra_quantiser_matrix
}

#[test]
fn quant_matrix_driver_feeds_slice_walker_user_matrices() {
    // Round-254 picture-level driver end-to-end: the §6.3.11 lifecycle
    // `driver.on_sequence_header(); driver.on_quant_matrix_extension(...)`
    // must produce the same `quantiser_matrices` snapshot the slice
    // walker would otherwise have to build by hand, and the §7.4.2.3
    // reconstruction step must read the user-downloaded entries
    // through the new driver → builder path.
    let mut ext_bw = BitWriter::new();
    // target = 80 → intra_luma[0][1] = 80 after §7.3.1 inverse scan.
    write_quant_matrix_extension_intra_only(&mut ext_bw, 80);
    let ext_bytes = ext_bw.finish();
    let ext = QuantMatrixExtension::parse(&ext_bytes, ChromaFormat::Yuv420).expect("parse");

    let mut driver = QuantMatrixDriver::new();
    driver.on_sequence_header();
    driver.on_quant_matrix_extension(ext, ChromaFormat::Yuv420);
    // Sanity: the per-zigzag-cell footprint of the synthetic extension
    // — `intra_luma[0][1] == 80` while `intra_luma[0][0]` keeps the
    // first-value-shall-be-8 byte at the §7.3.1 zigzag origin.
    let state = driver.state();
    assert_eq!(state.intra_luma[0][0], 8);
    assert_eq!(state.intra_luma[0][1], 80);
    // Non-intra slot was never loaded so it stays at the §6.3.7
    // default — the driver does not mutate slots an extension did not
    // touch.
    assert_eq!(
        state.non_intra_luma,
        QuantiserMatrixState::defaults().non_intra_luma
    );

    // Same wire bitstream as the r251 baseline / custom tests so the
    // §7.4.2.3 arithmetic comparison is bit-identical with the
    // overridden `W[0][1] = 80` path.
    let mut body_bw = BitWriter::new();
    write_increment_1(&mut body_bw);
    write_i_intra(&mut body_bw);
    write_intra_420_one_ac_then_dc_only(&mut body_bw);
    let body_buf = append_stop(body_bw);

    let ctx = SliceWalkContext::first_slice_with_block_decoding(
        22,
        0,
        PictureCodingType::Intra,
        14,
        PictureStructure::Frame,
        true,
        1,
        1,
        1,
        1,
        false,
        ChromaFormat::Yuv420,
        false,
        false,
        0,
        false,
    )
    .with_quantiser_matrices(driver.state());

    // The builder snapshot matches the driver's running state.
    assert_eq!(ctx.quantiser_matrices, state);

    let walk = walk_slice(&body_buf, ctx).unwrap();
    let blocks = walk.macroblocks[0]
        .decoded_blocks
        .as_ref()
        .expect("§6.2.6 ran");
    let y0 = &blocks[0];
    assert_eq!(y0.component, Mpeg2ColourComponent::Y);
    assert_eq!(y0.decoded.qf[0][1], 1);
    // F''[0][1] = (2 * 1) * 80 * 28 / 32 = 140 — the same value the
    // r251 hand-built-matrix test asserts. Reaching it via the
    // r254 driver → builder pipeline proves the §6.3.11 lifecycle
    // plumbing matches the in-place-matrix path byte-for-byte.
    assert_eq!(y0.decoded.f_quant[0][1], 140);
}

#[test]
fn quant_matrix_driver_sequence_header_reset_restores_default_arithmetic() {
    // The complementary lifecycle event: after the driver has
    // applied a user matrix, a subsequent `sequence_header_code`
    // must replay the §6.3.7 defaults so the next slice's §7.4.2.3
    // arithmetic matches the r251 baseline value (`28`).
    let mut ext_bw = BitWriter::new();
    write_quant_matrix_extension_intra_only(&mut ext_bw, 80);
    let ext_bytes = ext_bw.finish();
    let ext = QuantMatrixExtension::parse(&ext_bytes, ChromaFormat::Yuv420).expect("parse");

    let mut driver = QuantMatrixDriver::new();
    driver.on_quant_matrix_extension(ext, ChromaFormat::Yuv420);
    // The custom matrix is now installed.
    assert_eq!(driver.state().intra_luma[0][1], 80);

    // §6.3.11 sequence-header reset wipes the customisation.
    driver.on_sequence_header();
    assert_eq!(driver.state(), QuantiserMatrixState::defaults());

    let mut body_bw = BitWriter::new();
    write_increment_1(&mut body_bw);
    write_i_intra(&mut body_bw);
    write_intra_420_one_ac_then_dc_only(&mut body_bw);
    let body_buf = append_stop(body_bw);

    let ctx = SliceWalkContext::first_slice_with_block_decoding(
        22,
        0,
        PictureCodingType::Intra,
        14,
        PictureStructure::Frame,
        true,
        1,
        1,
        1,
        1,
        false,
        ChromaFormat::Yuv420,
        false,
        false,
        0,
        false,
    )
    .with_quantiser_matrices(driver.state());

    let walk = walk_slice(&body_buf, ctx).unwrap();
    let blocks = walk.macroblocks[0]
        .decoded_blocks
        .as_ref()
        .expect("§6.2.6 ran");
    let y0 = &blocks[0];
    // Back to the r251 baseline: F''[0][1] = (2 * 1) * 16 * 28 / 32 = 28.
    assert_eq!(y0.decoded.f_quant[0][1], 28);
}
