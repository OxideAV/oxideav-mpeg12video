//! End-to-end synthetic integration of the §7.6.4 / §7.6.7 / §7.6.8
//! prediction chain per ISO/IEC 13818-2.
//!
//! These tests drive the three landed pieces of the inter-prediction
//! pipeline together (`predict_block` → `combine_directional_predictions`
//! → `add_prediction_and_coefficients`) on hand-crafted reference
//! planes and IDCT outputs, verifying that the per-stage arithmetic
//! composes into a sensible final-decoded-sample block. The IDCT
//! itself is still ahead — this exercise uses fabricated `i16`
//! transform values to stand in for the §A.1 output.
//!
//! Each test is short and spec-traceable: it cites the subclause and
//! sequence of operations it exercises, and the expected values are
//! computed by hand from the spec's formulas (not from any external
//! decoder).

use oxideav_mpeg12video::{
    add_intra_block, add_prediction_and_coefficients, combine_directional_predictions,
    predict_block, BlockSize, BoundaryMode, PredictionDirection, ReferencePlane,
};

#[test]
fn intra_block_pipeline_via_saturated_idct_only() {
    // Intra macroblock: no prediction is formed; §7.6.8 with p=0
    // collapses to `d = saturate(f)`.
    //
    // 4×4 transform with negative + positive + overflow values, to
    // exercise the clamps at both ends. Hand-computed expected:
    let transform: Vec<i16> = vec![
        -10, 0, 50, 255, // saturate -> 0, 0, 50, 255
        -200, 100, 256, 300, // -> 0, 100, 255, 255
        128, 64, 32, 16, // -> 128, 64, 32, 16
        -1, 1, -255, 511, // -> 0, 1, 0, 255
    ];
    let out = add_intra_block(&transform);
    assert_eq!(
        out,
        vec![0, 0, 50, 255, 0, 100, 255, 255, 128, 64, 32, 16, 0, 1, 0, 255]
    );
}

#[test]
fn p_picture_forward_only_pipeline_zero_residual() {
    // P-picture, forward-only: prediction comes from forward
    // reference plane; transform residual is zero; output equals the
    // prediction unchanged after §7.6.8 (since 0 + p with p in
    // [0, 255] is already in range).
    let reference: Vec<u8> = (0..16).collect(); // 4×4 plane
    let plane =
        ReferencePlane::with_boundary(&reference, 4, 4, BoundaryMode::PadEdge).expect("plane fits");
    let forward = predict_block(plane, 0, 0, BlockSize::new(2, 2).expect("non-zero"), 2, 0);
    // Vector (2, 0) -> integer horizontal +1 -> reads (1, 0), (2, 0), (1, 1), (2, 1)
    // i.e. samples 1, 2, 5, 6.
    assert_eq!(forward, vec![1, 2, 5, 6]);
    // §7.6.7.1: forward-only branch -> output is the forward block.
    let combined = combine_directional_predictions(PredictionDirection::Forward, &forward, &[])
        .expect("forward branch never None");
    assert_eq!(combined, forward);
    // §7.6.8: add zero residual; saturate -> same values.
    let transform = vec![0i16; combined.len()];
    let decoded = add_prediction_and_coefficients(&transform, &combined).expect("equal len");
    assert_eq!(decoded, vec![1, 2, 5, 6]);
}

#[test]
fn b_picture_bidirectional_average_then_add() {
    // B-picture, bidirectional: average forward + backward, then add
    // a small residual.
    //
    // Use two 2×2 prediction blocks whose pointwise average lies
    // mid-range so the §7.6.8 clamps do not engage.
    let forward = vec![10u8, 20, 30, 40];
    let backward = vec![20u8, 30, 40, 50];
    // (10+20)//2 = 15; (20+30)//2 = 25; (30+40)//2 = 35; (40+50)//2 = 45.
    let combined =
        combine_directional_predictions(PredictionDirection::Bidirectional, &forward, &backward)
            .expect("equal length");
    assert_eq!(combined, vec![15, 25, 35, 45]);
    // Add a residual within range.
    let transform: Vec<i16> = vec![5, -5, 10, -10];
    let decoded = add_prediction_and_coefficients(&transform, &combined).unwrap();
    // 15+5=20, 25-5=20, 35+10=45, 45-10=35
    assert_eq!(decoded, vec![20, 20, 45, 35]);
}

#[test]
fn b_picture_bidirectional_average_saturates_at_clamp() {
    // The §7.6.8 saturation engages when prediction + transform falls
    // outside [0, 255].
    let forward = vec![250u8; 4];
    let backward = vec![254u8; 4];
    // (250+254)//2 = 252.
    let combined =
        combine_directional_predictions(PredictionDirection::Bidirectional, &forward, &backward)
            .expect("equal length");
    assert_eq!(combined, vec![252; 4]);
    // Push beyond 255 on the first two, drop below 0 on the last two.
    let transform = vec![10i16, 100, -300, -253];
    let decoded = add_prediction_and_coefficients(&transform, &combined).unwrap();
    // 252+10=262 -> 255; 252+100=352 -> 255; 252-300=-48 -> 0; 252-253=-1 -> 0.
    assert_eq!(decoded, vec![255, 255, 0, 0]);
}

#[test]
fn b_picture_backward_only_pipeline() {
    // Tables 7-13 / 7-14: `(forward, backward) = (0, 1)` -> output
    // is the backward prediction unchanged.
    let backward = vec![100u8, 110, 120, 130];
    let forward: Vec<u8> = Vec::new(); // ignored
    let combined =
        combine_directional_predictions(PredictionDirection::Backward, &forward, &backward)
            .expect("backward branch never None");
    assert_eq!(combined, backward);
}

#[test]
fn skipped_macroblock_uses_implicit_zero_mv_prediction() {
    // §7.6.6 / §7.6.3.5 skipped non-intra macroblock: prediction is
    // formed from `(0, 0)` motion vector; §7.6.7 passes it through
    // unchanged.
    let reference: Vec<u8> = (0..16).collect();
    let plane = ReferencePlane::new(&reference, 4, 4).expect("plane fits");
    let forward = predict_block(plane, 0, 0, BlockSize::new(2, 2).expect("non-zero"), 0, 0);
    assert_eq!(forward, vec![0, 1, 4, 5]); // zero-MV identity copy
    let combined = combine_directional_predictions(PredictionDirection::Skipped, &forward, &[])
        .expect("skipped branch never None");
    assert_eq!(combined, forward);
    // Skipped macroblocks have no transform data; spec says the
    // prediction *is* the final decoded sample. We exercise this by
    // adding a zero residual.
    let transform = vec![0i16; combined.len()];
    let decoded = add_prediction_and_coefficients(&transform, &combined).expect("equal len");
    assert_eq!(decoded, vec![0, 1, 4, 5]);
}

#[test]
fn full_pipeline_8x8_block() {
    // The §7.6.8 spec text writes the loop over an 8×8 block (the
    // IDCT transform block size). Drive a full-shape exercise.
    //
    // Reference plane: 12×12, increasing values.
    let reference: Vec<u8> = (0..144u32).map(|v| (v % 256) as u8).collect();
    let plane = ReferencePlane::new(&reference, 12, 12).expect("plane fits");
    // Zero motion vector: prediction is just the upper-left 8×8.
    let prediction = predict_block(plane, 0, 0, BlockSize::new(8, 8).expect("non-zero"), 0, 0);
    assert_eq!(prediction.len(), 64);
    // Hand-check first row of prediction: should be 0..8.
    assert_eq!(&prediction[0..8], &[0u8, 1, 2, 3, 4, 5, 6, 7]);
    // Apply §7.6.8 with all-zero residual: output equals prediction.
    let transform = vec![0i16; 64];
    let decoded = add_prediction_and_coefficients(&transform, &prediction).expect("equal len");
    assert_eq!(decoded, prediction);
}
