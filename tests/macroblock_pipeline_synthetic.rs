//! End-to-end synthetic integration of the §7.6 macroblock pipeline
//! driver per ISO/IEC 13818-2.
//!
//! These tests drive the §7.6.4 pel reader + the §7.6.5 / §7.6.6 case
//! selection + §7.6.7 combine + §7.6.8 add through the
//! [`pipeline_decode_macroblock`] driver, on hand-crafted reference
//! planes and fabricated (i.e. stub) IDCT outputs. The IDCT itself
//! is still ahead — these tests use fabricated `i16` transform values
//! to stand in for the §A.1 output (no external IDCT is invoked).
//!
//! Spec citations refer to ISO/IEC 13818-2 §7.6.5 / §7.6.7 / §7.6.8 +
//! §6.3.17.4 (`pattern_code[12]`).

use oxideav_mpeg12video::{
    blocks_per_macroblock, pipeline_decode_block, pipeline_decode_macroblock, predict_block,
    BlockInputs, BlockSize, BoundaryMode, ChromaFormat, CodedBlockPattern, MacroblockKind,
    MacroblockType, PipelineError, PredictionDirection, ReferencePlane,
};

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

fn mt_inter(forward: bool, backward: bool, pattern: bool) -> MacroblockType {
    MacroblockType {
        macroblock_quant: false,
        macroblock_motion_forward: forward,
        macroblock_motion_backward: backward,
        macroblock_pattern: pattern,
        macroblock_intra: false,
        spatial_temporal_weight_code_flag: false,
        bit_position_after: 0,
    }
}

fn cbp(value: u8) -> CodedBlockPattern {
    CodedBlockPattern {
        cbp: value,
        coded_block_pattern_1: None,
        coded_block_pattern_2: None,
        bit_position_after: 0,
    }
}

#[test]
fn intra_macroblock_4_2_0_six_blocks_via_driver_match_add_intra_block() {
    // The intra path collapses to add_intra_block per block. Verify
    // the pipeline driver routes each per-block transform correctly
    // and emits exactly the six blocks of a 4:2:0 MB in §6.3.17.4
    // order.
    let mt = mt_intra();
    let cbp_v = cbp(0);
    let transforms: Vec<Vec<i16>> = (0..6)
        .map(|i| {
            vec![
                (i * 10 + 1) as i16,
                (i * 10 + 2) as i16,
                (i * 10 + 3) as i16,
            ]
        })
        .collect();
    let mut inputs = [BlockInputs::intra(&[]); 12];
    for (i, t) in transforms.iter().enumerate() {
        inputs[i] = BlockInputs::intra(t);
    }
    let out = pipeline_decode_macroblock(
        MacroblockKind::Intra,
        &cbp_v,
        &mt,
        ChromaFormat::Yuv420,
        &inputs,
    )
    .expect("intra always succeeds");
    assert_eq!(out.len(), 6);
    for (i, db) in out.iter().enumerate() {
        assert_eq!(db.block_index as usize, i);
        // saturate(f) — none of the test values exceed 255 / dip below 0.
        let expected: Vec<u8> = transforms[i]
            .iter()
            .map(|f| (*f).clamp(0, 255) as u8)
            .collect();
        assert_eq!(db.samples, expected);
    }
}

#[test]
fn p_picture_forward_only_macroblock_pipeline() {
    // P-picture, forward-only MB in 4:2:0 with cbp = 0b111111 (all 6
    // blocks coded). Predictions come from the §7.6.4 pel reader on a
    // hand-built reference plane; transform residual is zero so the
    // §7.6.8 step passes the prediction through.
    let mt = mt_inter(true, false, true);
    let cbp_v = cbp(0b111111);
    let kind = MacroblockKind::Inter(PredictionDirection::Forward);

    // Per-component reference plane (12×12, simple ramp). Use the
    // upper-left 4×4 region as the per-block prediction so we don't
    // overlap MV reads.
    let plane_buf: Vec<u8> = (0..144u32).map(|v| (v % 256) as u8).collect();
    let plane = ReferencePlane::with_boundary(&plane_buf, 12, 12, BoundaryMode::PadEdge)
        .expect("plane fits");
    let predictions: Vec<Vec<u8>> = (0..6)
        .map(|i| {
            // Different (x, y) per block so the prediction values are
            // distinct.
            predict_block(plane, i, 0, BlockSize::new(2, 2).expect("non-zero"), 0, 0)
        })
        .collect();
    let zero_transform: Vec<i16> = vec![0; 4];
    let mut inputs = [BlockInputs::intra(&[]); 12];
    for (i, p) in predictions.iter().enumerate() {
        inputs[i] = BlockInputs::forward(&zero_transform, p);
    }

    let out = pipeline_decode_macroblock(kind, &cbp_v, &mt, ChromaFormat::Yuv420, &inputs).unwrap();
    assert_eq!(out.len(), 6);
    for (db, expected_prediction) in out.iter().zip(predictions.iter()) {
        assert_eq!(&db.samples, expected_prediction);
    }
}

#[test]
fn b_picture_bidirectional_macroblock_pipeline_averages_then_saturates() {
    // B-picture, bidirectional MB in 4:2:0 with only block 0 coded.
    // The driver averages forward + backward, then adds the IDCT
    // residual with [0, 255] clamp.
    let mt = mt_inter(true, true, true);
    let cbp_v = cbp(0b100000); // only block 0 coded (bit 5)
    let kind = MacroblockKind::Inter(PredictionDirection::Bidirectional);

    let forward = vec![250u8; 4]; // near upper clamp
    let backward = vec![254u8; 4];
    // average: (250+254)//2 = 252
    let transform: Vec<i16> = vec![10, 100, -300, -253];
    // 252+10=262→255, 252+100=352→255, 252-300=-48→0, 252-253=-1→0
    let expected = vec![255u8, 255, 0, 0];

    let mut inputs = [BlockInputs::intra(&[]); 12];
    inputs[0] = BlockInputs::bidirectional(&transform, &forward, &backward);
    let out = pipeline_decode_macroblock(kind, &cbp_v, &mt, ChromaFormat::Yuv420, &inputs).unwrap();
    assert_eq!(out.len(), 1);
    assert_eq!(out[0].block_index, 0);
    assert_eq!(out[0].samples, expected);
}

#[test]
fn b_picture_backward_only_macroblock_pipeline() {
    // B-picture, backward-only on a single coded block.
    let mt = mt_inter(false, true, true);
    let cbp_v = cbp(0b000001); // only block 5 coded (bit 0)
    let kind = MacroblockKind::Inter(PredictionDirection::Backward);

    let backward = vec![100u8, 110, 120, 130];
    let transform: Vec<i16> = vec![1, 2, 3, 4];
    // 101, 112, 123, 134.
    let expected = vec![101u8, 112, 123, 134];

    let mut inputs = [BlockInputs::intra(&[]); 12];
    inputs[5] = BlockInputs::backward(&transform, &backward);
    let out = pipeline_decode_macroblock(kind, &cbp_v, &mt, ChromaFormat::Yuv420, &inputs).unwrap();
    assert_eq!(out.len(), 1);
    assert_eq!(out[0].block_index, 5);
    assert_eq!(out[0].samples, expected);
}

#[test]
fn skipped_macroblock_with_pattern_zero_emits_no_blocks() {
    // §7.6.6 skipped MB in a P-picture: macroblock_pattern = false,
    // intra = false → pattern_code is all-false → the driver walks
    // the MB and emits zero decoded blocks. (The caller is
    // responsible for the §7.6.8 d = p short-circuit on uncoded
    // blocks — this driver is about coded-block reconstruction.)
    let mt = mt_inter(false, false, false);
    let cbp_v = cbp(0);
    let inputs = [BlockInputs::intra(&[]); 12];
    let out = pipeline_decode_macroblock(
        MacroblockKind::Inter(PredictionDirection::Skipped),
        &cbp_v,
        &mt,
        ChromaFormat::Yuv420,
        &inputs,
    )
    .unwrap();
    assert!(out.is_empty());
}

#[test]
fn decode_block_intra_8x8_matches_pointwise_saturate() {
    // Drive the inner per-block call on the spec's canonical 8×8
    // shape to verify the §7.6.8 d = saturate(f) loop end-to-end.
    let transform: Vec<i16> = (0..64).map(|i| (i as i16) - 30).collect();
    let out = pipeline_decode_block(MacroblockKind::Intra, BlockInputs::intra(&transform))
        .expect("intra never errors");
    assert_eq!(out.len(), 64);
    for (i, sample) in out.iter().enumerate() {
        let expected = ((i as i32) - 30).clamp(0, 255) as u8;
        assert_eq!(*sample, expected, "sample {i}");
    }
}

#[test]
fn pipeline_propagates_caller_bug_errors() {
    // Caller-bug: inter forward MB but the prediction slice is empty
    // while transform isn't — driver must return MissingForwardPrediction
    // and walk no further than the first coded block.
    let mt = mt_inter(true, false, true);
    let cbp_v = cbp(0b111111);
    let kind = MacroblockKind::Inter(PredictionDirection::Forward);
    let transform: Vec<i16> = vec![0; 4];
    let mut inputs = [BlockInputs::intra(&[]); 12];
    for slot in inputs.iter_mut() {
        *slot = BlockInputs {
            transform: &transform,
            prediction_forward: &[],
            prediction_backward: &[],
        };
    }
    let err =
        pipeline_decode_macroblock(kind, &cbp_v, &mt, ChromaFormat::Yuv420, &inputs).unwrap_err();
    assert_eq!(err, PipelineError::MissingForwardPrediction);
}

#[test]
fn blocks_per_macroblock_matches_chroma_geometry() {
    assert_eq!(blocks_per_macroblock(ChromaFormat::Yuv420), 6);
    assert_eq!(blocks_per_macroblock(ChromaFormat::Yuv422), 8);
    assert_eq!(blocks_per_macroblock(ChromaFormat::Yuv444), 12);
}
