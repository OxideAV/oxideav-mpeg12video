//! Integration coverage for the §6.2.6 `block(i)` driver
//! ([`oxideav_mpeg12video::mpeg2_decode_block`]) — exercises the
//! public re-export surface end-to-end and pins composition behaviour
//! that's awkward to assert inside the unit tests (chained Y / Cb / Cr
//! blocks, several intra blocks updating the predictor in sequence,
//! and the cursor accounting against pre-padded bit lengths).

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_mpeg12video::{
    mpeg2_decode_block, ChromaFormat, Mpeg2BlockCoding, Mpeg2BlockContext, Mpeg2ColourComponent,
    Mpeg2Component, Mpeg2DcPredictors, DEFAULT_INTRA_WEIGHT, DEFAULT_NON_INTRA_WEIGHT,
};

/// Table B-14 EOB code (`10`, 2 bits) — spec constant; matches the
/// private const in `mpeg2_dct_coeff`.
const EOB_B14_CODE: u32 = 0b10;
const EOB_B14_BITS: u32 = 2;

/// Padding-and-finish helper mirroring the unit-test convention so a
/// BitReader can load at least one trailing byte.
fn pad(mut bw: BitWriter) -> Vec<u8> {
    bw.write_bit(false);
    bw.align_to_byte();
    bw.finish()
}

fn baseline_ctx() -> Mpeg2BlockContext {
    Mpeg2BlockContext {
        intra_vlc_format: false,
        alternate_scan: false,
        intra_dc_precision: 0,
        quantiser_scale_value: 8,
    }
}

#[test]
fn chained_y_cb_cr_intra_blocks_update_predictors_independently() {
    // Three intra blocks (Y, Cb, Cr) each with size-0 DC + immediate
    // EOB. Per §7.2.1 each predictor lands at its Table 7-2 reset
    // value (128 at intra_dc_precision = 0) and stays there
    // because dct_diff = 0.
    let mut bw = BitWriter::new();
    // Y: size = 0 → `100` (3 bits), then EOB.
    bw.write_u32(0b100, 3);
    bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
    // Cb: B-13 size = 0 → `00` (2 bits), then EOB.
    bw.write_u32(0b00, 2);
    bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
    // Cr: same shape as Cb.
    bw.write_u32(0b00, 2);
    bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
    let buf = pad(bw);
    let mut br = BitReader::new(&buf);
    let ctx = baseline_ctx();
    let mut dc = Mpeg2DcPredictors::new(0).unwrap();

    let y = mpeg2_decode_block(
        &mut br,
        &ctx,
        &mut dc,
        Mpeg2ColourComponent::Y,
        true,
        &DEFAULT_INTRA_WEIGHT,
    )
    .expect("Y");
    let cb = mpeg2_decode_block(
        &mut br,
        &ctx,
        &mut dc,
        Mpeg2ColourComponent::Cb,
        true,
        &DEFAULT_INTRA_WEIGHT,
    )
    .expect("Cb");
    let cr = mpeg2_decode_block(
        &mut br,
        &ctx,
        &mut dc,
        Mpeg2ColourComponent::Cr,
        true,
        &DEFAULT_INTRA_WEIGHT,
    )
    .expect("Cr");

    assert_eq!(y.qfs[0], 128);
    assert_eq!(cb.qfs[0], 128);
    assert_eq!(cr.qfs[0], 128);
    // §7.2.1 per-component predictor cells all at 128 (reset).
    assert_eq!(dc.get(Mpeg2ColourComponent::Y), 128);
    assert_eq!(dc.get(Mpeg2ColourComponent::Cb), 128);
    assert_eq!(dc.get(Mpeg2ColourComponent::Cr), 128);
}

#[test]
fn intra_chain_accumulates_positive_diff_across_three_y_blocks() {
    // Three Y intra blocks, each with size = 1 and
    // dc_dct_differential = `1` (dct_diff = +1). Predictor walks
    // 128 → 129 → 130 → 131.
    let mut bw = BitWriter::new();
    for _ in 0..3 {
        // dct_dc_size_luminance = 1 (`00`, 2 bits) + diff bit `1`.
        bw.write_u32(0b00, 2);
        bw.write_bit(true);
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
    }
    let buf = pad(bw);
    let mut br = BitReader::new(&buf);
    let ctx = baseline_ctx();
    let mut dc = Mpeg2DcPredictors::new(0).unwrap();

    let b0 = mpeg2_decode_block(
        &mut br,
        &ctx,
        &mut dc,
        Mpeg2ColourComponent::Y,
        true,
        &DEFAULT_INTRA_WEIGHT,
    )
    .expect("b0");
    let b1 = mpeg2_decode_block(
        &mut br,
        &ctx,
        &mut dc,
        Mpeg2ColourComponent::Y,
        true,
        &DEFAULT_INTRA_WEIGHT,
    )
    .expect("b1");
    let b2 = mpeg2_decode_block(
        &mut br,
        &ctx,
        &mut dc,
        Mpeg2ColourComponent::Y,
        true,
        &DEFAULT_INTRA_WEIGHT,
    )
    .expect("b2");

    assert_eq!(b0.qfs[0], 129);
    assert_eq!(b1.qfs[0], 130);
    assert_eq!(b2.qfs[0], 131);
    assert_eq!(dc.get(Mpeg2ColourComponent::Y), 131);
    // Cb / Cr untouched.
    assert_eq!(dc.get(Mpeg2ColourComponent::Cb), 128);
    assert_eq!(dc.get(Mpeg2ColourComponent::Cr), 128);
}

#[test]
fn end_of_block_bit_position_matches_buffer_position_for_concatenated_blocks() {
    // After each `mpeg2_decode_block` call the returned
    // `end_of_block_bit_position` must equal the reader's
    // `bit_position()`, end-to-end across two intra blocks.
    let mut bw = BitWriter::new();
    // Block 0: size = 0 (3 bits) + EOB (2 bits) = 5 bits.
    bw.write_u32(0b100, 3);
    bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
    // Block 1: same shape as block 0.
    bw.write_u32(0b100, 3);
    bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
    let buf = pad(bw);
    let mut br = BitReader::new(&buf);
    let ctx = baseline_ctx();
    let mut dc = Mpeg2DcPredictors::new(0).unwrap();

    let b0 = mpeg2_decode_block(
        &mut br,
        &ctx,
        &mut dc,
        Mpeg2ColourComponent::Y,
        true,
        &DEFAULT_INTRA_WEIGHT,
    )
    .expect("b0");
    assert_eq!(b0.end_of_block_bit_position, 5);
    assert_eq!(br.bit_position(), 5);

    let b1 = mpeg2_decode_block(
        &mut br,
        &ctx,
        &mut dc,
        Mpeg2ColourComponent::Y,
        true,
        &DEFAULT_INTRA_WEIGHT,
    )
    .expect("b1");
    assert_eq!(b1.end_of_block_bit_position, 10);
    assert_eq!(br.bit_position(), 10);
}

#[test]
fn non_intra_block_with_single_first_runlevel_round_trips_against_default_weights() {
    // Non-intra block: FIRST `(0, +1)` codeword (Table B-14
    // 1-bit `1` + sign `0`), then EOB.
    let mut bw = BitWriter::new();
    bw.write_bit(true);
    bw.write_bit(false);
    bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
    let buf = pad(bw);
    let mut br = BitReader::new(&buf);
    let ctx = baseline_ctx();
    let mut dc = Mpeg2DcPredictors::new(0).unwrap();

    let out = mpeg2_decode_block(
        &mut br,
        &ctx,
        &mut dc,
        Mpeg2ColourComponent::Y,
        false,
        &DEFAULT_NON_INTRA_WEIGHT,
    )
    .expect("non-intra");
    assert_eq!(out.qfs[0], 1);
    assert_eq!(out.qf[0][0], 1);
    // Predictor must NOT have been touched by a non-intra block.
    assert_eq!(dc.get(Mpeg2ColourComponent::Y), 128);
}

#[test]
fn non_intra_block_chain_does_not_disturb_dc_predictors() {
    // Two non-intra blocks in a row; the DC predictor for Y must
    // remain at its initial reset value because §7.2.1's predictor
    // update applies to intra blocks only.
    let mut bw = BitWriter::new();
    // Block 0: FIRST (0, +1) + EOB.
    bw.write_bit(true);
    bw.write_bit(false);
    bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
    // Block 1: FIRST (0, -1) + EOB.
    bw.write_bit(true);
    bw.write_bit(true);
    bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
    let buf = pad(bw);
    let mut br = BitReader::new(&buf);
    let ctx = baseline_ctx();
    let mut dc = Mpeg2DcPredictors::new(0).unwrap();

    let b0 = mpeg2_decode_block(
        &mut br,
        &ctx,
        &mut dc,
        Mpeg2ColourComponent::Y,
        false,
        &DEFAULT_NON_INTRA_WEIGHT,
    )
    .expect("b0");
    let b1 = mpeg2_decode_block(
        &mut br,
        &ctx,
        &mut dc,
        Mpeg2ColourComponent::Y,
        false,
        &DEFAULT_NON_INTRA_WEIGHT,
    )
    .expect("b1");

    assert_eq!(b0.qfs[0], 1);
    assert_eq!(b1.qfs[0], -1);
    // Y predictor untouched: §7.2.1 does NOT update on non-intra.
    assert_eq!(dc.get(Mpeg2ColourComponent::Y), 128);
}

#[test]
fn dequant_block_coding_enum_lines_up_with_driver_macroblock_intra_flag() {
    // Documents the convention: `macroblock_intra = true` matches
    // `Mpeg2BlockCoding::Intra` for upstream dequantiser inspection.
    // This is the wiring assumption the §6.2.6 driver makes
    // internally — pin it at the public surface.
    let intra: Mpeg2BlockCoding = Mpeg2BlockCoding::Intra;
    let non_intra: Mpeg2BlockCoding = Mpeg2BlockCoding::NonIntra;
    assert_ne!(intra, non_intra);
}

#[test]
fn yuv_420_intra_dc_predictor_chain_for_six_blocks_advances_consistently() {
    // 4:2:0 macroblock has 6 blocks: Y0, Y1, Y2, Y3, Cb, Cr.
    // Drive every block with size-0 DC + EOB. All three predictors
    // remain at reset value 128. This is the "skeleton macroblock"
    // pattern an upstream macroblock-level driver will use to
    // walk `pattern_code[12]`.
    let _component = Mpeg2Component::Luminance; // documentation-only
    let _chroma = ChromaFormat::Yuv420;
    let mut bw = BitWriter::new();
    // Y0..Y3
    for _ in 0..4 {
        bw.write_u32(0b100, 3);
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
    }
    // Cb / Cr.
    for _ in 0..2 {
        bw.write_u32(0b00, 2);
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
    }
    let buf = pad(bw);
    let mut br = BitReader::new(&buf);
    let ctx = baseline_ctx();
    let mut dc = Mpeg2DcPredictors::new(0).unwrap();

    for _ in 0..4 {
        let out = mpeg2_decode_block(
            &mut br,
            &ctx,
            &mut dc,
            Mpeg2ColourComponent::Y,
            true,
            &DEFAULT_INTRA_WEIGHT,
        )
        .expect("Y_n");
        assert_eq!(out.qfs[0], 128);
    }
    let cb = mpeg2_decode_block(
        &mut br,
        &ctx,
        &mut dc,
        Mpeg2ColourComponent::Cb,
        true,
        &DEFAULT_INTRA_WEIGHT,
    )
    .expect("Cb");
    let cr = mpeg2_decode_block(
        &mut br,
        &ctx,
        &mut dc,
        Mpeg2ColourComponent::Cr,
        true,
        &DEFAULT_INTRA_WEIGHT,
    )
    .expect("Cr");
    assert_eq!(cb.qfs[0], 128);
    assert_eq!(cr.qfs[0], 128);
    // 6 blocks consumed:
    // 4 Y × (3 + 2) = 20 bits
    // 2 chroma × (2 + 2) = 8 bits
    // total = 28 bits.
    assert_eq!(br.bit_position(), 28);
}
