//! Black-box integration tests for the §6.2.5 / §6.2.6
//! macroblock-block driver per **ISO/IEC 13818-2 (ITU-T H.262)**.
//!
//! These exercise the public re-exports of
//! [`oxideav_mpeg12video::mpeg2_decode_macroblock_blocks`]
//! end-to-end: a synthetic bitstream made out of the already-pinned
//! Tables B-12 / B-13 / B-14 codewords, decoded into a
//! `Vec<Mpeg2MacroblockDecodedBlock>` whose `block_index` /
//! `component` columns are asserted against the §6.1.1.8
//! Figure 6-10 / 6-11 / 6-12 layout.

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_mpeg12video::{
    mpeg2_decode_macroblock_blocks, ChromaFormat, CodedBlockPattern, MacroblockType,
    Mpeg2ColourComponent, Mpeg2DcPredictors, Mpeg2MacroblockBlockContext,
};

/// Table B-14 EOB = `10` (2 bits).
const EOB_B14_CODE: u32 = 0b10;
const EOB_B14_BITS: u32 = 2;

/// Emit a size-0 intra block: `dct_dc_size_*` codeword for
/// `size = 0` followed by an immediate Table B-14 EOB.
fn write_size_zero_intra_block(bw: &mut BitWriter, component: Mpeg2ColourComponent) {
    match component {
        Mpeg2ColourComponent::Y => bw.write_u32(0b100, 3), // B-12 size 0
        Mpeg2ColourComponent::Cb | Mpeg2ColourComponent::Cr => bw.write_u32(0b00, 2), // B-13 size 0
    }
    bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
}

/// Tail-pad with a `0` bit and align to a byte so the BitReader
/// has a trailing byte to load past the payload.
fn pad(mut bw: BitWriter) -> Vec<u8> {
    bw.write_bit(false);
    bw.align_to_byte();
    bw.finish()
}

fn mt_intra() -> MacroblockType {
    MacroblockType {
        macroblock_quant: false,
        macroblock_motion_forward: false,
        macroblock_motion_backward: false,
        macroblock_pattern: false,
        macroblock_intra: true,
        spatial_temporal_weight_code_flag: false,
        bit_position_after: 0,
    }
}

fn cbp_all_intra() -> CodedBlockPattern {
    CodedBlockPattern {
        cbp: 0,
        coded_block_pattern_1: None,
        coded_block_pattern_2: None,
        bit_position_after: 0,
    }
}

#[test]
fn integration_intra_macroblock_420_walks_six_blocks_in_figure_6_10_order() {
    // Build six §6.2.6 intra blocks: four Y + Cb + Cr, each
    // size-zero plus EOB.
    let mut bw = BitWriter::new();
    for _ in 0..4 {
        write_size_zero_intra_block(&mut bw, Mpeg2ColourComponent::Y);
    }
    write_size_zero_intra_block(&mut bw, Mpeg2ColourComponent::Cb);
    write_size_zero_intra_block(&mut bw, Mpeg2ColourComponent::Cr);
    let buf = pad(bw);
    let mut br = BitReader::new(&buf);

    let ctx = Mpeg2MacroblockBlockContext::with_default_weight_matrices(
        false,
        false,
        0,
        8,
        ChromaFormat::Yuv420,
    );
    let mut dc = Mpeg2DcPredictors::new(0).unwrap();
    let mt = mt_intra();
    let cbp = cbp_all_intra();

    let out = mpeg2_decode_macroblock_blocks(&mut br, &ctx, &mut dc, &mt, &cbp)
        .expect("six-block intra walk decodes");
    assert_eq!(out.len(), 6, "4:2:0 intra MB has six coded blocks");
    let indices: Vec<u8> = out.iter().map(|b| b.block_index).collect();
    assert_eq!(indices, vec![0, 1, 2, 3, 4, 5]);
    let comps: Vec<Mpeg2ColourComponent> = out.iter().map(|b| b.component).collect();
    assert_eq!(
        comps,
        vec![
            Mpeg2ColourComponent::Y,
            Mpeg2ColourComponent::Y,
            Mpeg2ColourComponent::Y,
            Mpeg2ColourComponent::Y,
            Mpeg2ColourComponent::Cb,
            Mpeg2ColourComponent::Cr,
        ]
    );
    // Each block carries the §7.2.1 reset predictor value 128 in
    // QFS[0] (since dct_diff = 0 everywhere). The §A IDCT of a
    // constant DC and zero AC is a flat plane.
    for db in &out {
        assert_eq!(db.decoded.qfs[0], 128);
        // f[y][x] is constant across the block.
        let expected = db.decoded.f_pel[0][0];
        for v in 0..8 {
            for u in 0..8 {
                assert_eq!(db.decoded.f_pel[v][u], expected, "{v},{u}");
            }
        }
    }
}

#[test]
fn integration_intra_macroblock_422_walks_eight_blocks_in_figure_6_11_order() {
    // 4:2:2: 4 Y + 2 Cb + 2 Cr per Figure 6-11.
    let mut bw = BitWriter::new();
    for _ in 0..4 {
        write_size_zero_intra_block(&mut bw, Mpeg2ColourComponent::Y);
    }
    for _ in 0..2 {
        write_size_zero_intra_block(&mut bw, Mpeg2ColourComponent::Cb);
    }
    for _ in 0..2 {
        write_size_zero_intra_block(&mut bw, Mpeg2ColourComponent::Cr);
    }
    let buf = pad(bw);
    let mut br = BitReader::new(&buf);

    let ctx = Mpeg2MacroblockBlockContext::with_default_weight_matrices(
        false,
        false,
        0,
        8,
        ChromaFormat::Yuv422,
    );
    let mut dc = Mpeg2DcPredictors::new(0).unwrap();
    let mt = mt_intra();
    let cbp = cbp_all_intra();

    let out = mpeg2_decode_macroblock_blocks(&mut br, &ctx, &mut dc, &mt, &cbp)
        .expect("eight-block 4:2:2 intra walk decodes");
    assert_eq!(out.len(), 8, "4:2:2 intra MB has eight coded blocks");
    let comps: Vec<Mpeg2ColourComponent> = out.iter().map(|b| b.component).collect();
    assert_eq!(
        comps,
        vec![
            Mpeg2ColourComponent::Y,
            Mpeg2ColourComponent::Y,
            Mpeg2ColourComponent::Y,
            Mpeg2ColourComponent::Y,
            Mpeg2ColourComponent::Cb,
            Mpeg2ColourComponent::Cb,
            Mpeg2ColourComponent::Cr,
            Mpeg2ColourComponent::Cr,
        ],
        "Figure 6-11 puts both Cb blocks before both Cr blocks",
    );
}

#[test]
fn integration_intra_macroblock_444_walks_twelve_blocks_in_figure_6_12_order() {
    // 4:4:4: 4 Y + 4 Cb + 4 Cr per Figure 6-12.
    let mut bw = BitWriter::new();
    for _ in 0..4 {
        write_size_zero_intra_block(&mut bw, Mpeg2ColourComponent::Y);
    }
    for _ in 0..4 {
        write_size_zero_intra_block(&mut bw, Mpeg2ColourComponent::Cb);
    }
    for _ in 0..4 {
        write_size_zero_intra_block(&mut bw, Mpeg2ColourComponent::Cr);
    }
    let buf = pad(bw);
    let mut br = BitReader::new(&buf);

    let ctx = Mpeg2MacroblockBlockContext::with_default_weight_matrices(
        false,
        false,
        0,
        8,
        ChromaFormat::Yuv444,
    );
    let mut dc = Mpeg2DcPredictors::new(0).unwrap();
    let mt = mt_intra();
    let cbp = cbp_all_intra();

    let out = mpeg2_decode_macroblock_blocks(&mut br, &ctx, &mut dc, &mt, &cbp)
        .expect("twelve-block 4:4:4 intra walk decodes");
    assert_eq!(out.len(), 12);
    let indices: Vec<u8> = out.iter().map(|b| b.block_index).collect();
    assert_eq!(indices, (0u8..12).collect::<Vec<_>>());
    let comps: Vec<Mpeg2ColourComponent> = out.iter().map(|b| b.component).collect();
    assert_eq!(
        comps,
        vec![
            Mpeg2ColourComponent::Y,
            Mpeg2ColourComponent::Y,
            Mpeg2ColourComponent::Y,
            Mpeg2ColourComponent::Y,
            Mpeg2ColourComponent::Cb,
            Mpeg2ColourComponent::Cb,
            Mpeg2ColourComponent::Cb,
            Mpeg2ColourComponent::Cb,
            Mpeg2ColourComponent::Cr,
            Mpeg2ColourComponent::Cr,
            Mpeg2ColourComponent::Cr,
            Mpeg2ColourComponent::Cr,
        ],
    );
}

#[test]
fn integration_predictor_chain_carries_across_blocks_within_a_macroblock() {
    // §7.2.1: the predictor cell for each component carries its
    // value across blocks within the same macroblock — only the
    // matching cell updates. A four-Y chain should walk the Y
    // predictor 128 → 129 → 130 → 131 if every block emits
    // dct_diff = +1.
    let mut bw = BitWriter::new();
    for _ in 0..4 {
        // dct_dc_size_luminance = 1 → B-12 code `00` (2 bits)
        bw.write_u32(0b00, 2);
        // dc_dct_differential bit = 1 → dct_diff = +1
        bw.write_bit(true);
        bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
    }
    // Cb and Cr at size 0 to terminate the MB.
    bw.write_u32(0b00, 2); // B-13 size 0
    bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
    bw.write_u32(0b00, 2); // B-13 size 0
    bw.write_u32(EOB_B14_CODE, EOB_B14_BITS);
    let buf = pad(bw);
    let mut br = BitReader::new(&buf);

    let ctx = Mpeg2MacroblockBlockContext::with_default_weight_matrices(
        false,
        false,
        0,
        8,
        ChromaFormat::Yuv420,
    );
    let mut dc = Mpeg2DcPredictors::new(0).unwrap();
    let mt = mt_intra();
    let cbp = cbp_all_intra();

    let out = mpeg2_decode_macroblock_blocks(&mut br, &ctx, &mut dc, &mt, &cbp).expect("walk");
    assert_eq!(out.len(), 6);
    // Four Y blocks accumulate +1 per call: 129 → 130 → 131 → 132.
    assert_eq!(out[0].decoded.qfs[0], 129);
    assert_eq!(out[1].decoded.qfs[0], 130);
    assert_eq!(out[2].decoded.qfs[0], 131);
    assert_eq!(out[3].decoded.qfs[0], 132);
    // Cb / Cr predictors stay at their independent reset value 128.
    assert_eq!(out[4].decoded.qfs[0], 128);
    assert_eq!(out[5].decoded.qfs[0], 128);
    // Final predictor cells: Y at 132, Cb / Cr at 128.
    assert_eq!(dc.get(Mpeg2ColourComponent::Y), 132);
    assert_eq!(dc.get(Mpeg2ColourComponent::Cb), 128);
    assert_eq!(dc.get(Mpeg2ColourComponent::Cr), 128);
}

#[test]
fn integration_non_intra_macroblock_with_no_coded_blocks_resets_predictors() {
    // §7.2.1: every non-intra macroblock resets the DC predictors
    // even if pattern_code is all-false.
    let ctx = Mpeg2MacroblockBlockContext::with_default_weight_matrices(
        false,
        false,
        0,
        8,
        ChromaFormat::Yuv420,
    );
    let mut dc = Mpeg2DcPredictors::new(0).unwrap();
    // Seed predictors to non-reset values.
    dc.luma = 500;
    dc.cb = 600;
    dc.cr = 700;
    let mt = MacroblockType {
        macroblock_quant: false,
        macroblock_motion_forward: true,
        macroblock_motion_backward: false,
        macroblock_pattern: false,
        macroblock_intra: false,
        spatial_temporal_weight_code_flag: false,
        bit_position_after: 0,
    };
    let cbp = CodedBlockPattern {
        cbp: 0,
        coded_block_pattern_1: None,
        coded_block_pattern_2: None,
        bit_position_after: 0,
    };
    let buf = [0u8; 4];
    let mut br = BitReader::new(&buf);
    let out = mpeg2_decode_macroblock_blocks(&mut br, &ctx, &mut dc, &mt, &cbp).unwrap();
    assert!(out.is_empty());
    assert_eq!(dc.luma, 128);
    assert_eq!(dc.cb, 128);
    assert_eq!(dc.cr, 128);
}

#[test]
fn integration_bit_cursor_advances_past_the_decoded_blocks_only() {
    // After a six-block intra walk, the BitReader cursor matches
    // the post-EOB bit position reported by the LAST block's
    // inner DecodedBlock. Each size-0 + EOB block contributes
    // 3 (B-12) + 2 (B-14 EOB) = 5 bits for luma; 2 (B-13) + 2 = 4
    // bits for chroma. Total: 4 * 5 + 2 * 4 = 28 bits.
    let mut bw = BitWriter::new();
    for _ in 0..4 {
        write_size_zero_intra_block(&mut bw, Mpeg2ColourComponent::Y);
    }
    write_size_zero_intra_block(&mut bw, Mpeg2ColourComponent::Cb);
    write_size_zero_intra_block(&mut bw, Mpeg2ColourComponent::Cr);
    let buf = pad(bw);
    let mut br = BitReader::new(&buf);

    let ctx = Mpeg2MacroblockBlockContext::with_default_weight_matrices(
        false,
        false,
        0,
        8,
        ChromaFormat::Yuv420,
    );
    let mut dc = Mpeg2DcPredictors::new(0).unwrap();
    let mt = mt_intra();
    let cbp = cbp_all_intra();
    let out = mpeg2_decode_macroblock_blocks(&mut br, &ctx, &mut dc, &mt, &cbp).unwrap();
    // Last block's EOB position is the post-walk position.
    let last_eob = out
        .last()
        .map(|b| b.decoded.end_of_block_bit_position)
        .unwrap();
    assert_eq!(last_eob, 28, "4*5 (luma) + 2*4 (chroma) = 28 bits");
}
