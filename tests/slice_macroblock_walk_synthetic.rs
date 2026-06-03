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
    walk_slice, PictureCodingType, SliceContext, SliceHeader, SliceWalkContext,
    PAST_INTRA_ADDRESS_RESET,
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
    // increment = 3 → skip 2 MBs.
    bw.write_u32(0b010, 3);
    bw.write_u32(0b1, 1);
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
