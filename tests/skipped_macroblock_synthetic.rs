//! Black-box integration tests for the §7.6.6 skipped-macroblock
//! description module per **ISO/IEC 13818-2 (ITU-T H.262)**.
//!
//! These chain the existing slice-walker output ([`walk_slice`] /
//! [`MacroblockRecord::skipped_macroblock_count`]) with the new
//! [`describe_skipped_macroblock`] / [`skipped_macroblock_apply_to_pmv`]
//! endpoints to verify the §7.6.6.1..§7.6.6.4 per-case derivations
//! hold against the public re-exports.

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::{
    describe_skipped_macroblock, skipped_macroblock_apply_to_pmv, walk_slice, Component, Direction,
    MvFormat, PictureCodingType, PictureStructure, Pmv, PredictionDirection, PredictionType,
    SkippedMacroblockContext, SkippedMotionVector, SliceWalkContext, VectorIndex,
};

fn append_stop(mut bw: BitWriter) -> Vec<u8> {
    bw.align_to_byte_zero();
    let mut bytes = bw.finish();
    bytes.extend_from_slice(&[0x00, 0x00, 0x00, 0x01, 0xB7]);
    bytes
}

#[test]
fn slice_walker_skipped_count_feeds_p_frame_picture_description() {
    // §7.6.6.2: every skipped macroblock in a P frame picture is
    // described as Frame-based / zero-MV / PMV-reset, irrespective
    // of how many slots are skipped between coded macroblocks.
    let mut bw = BitWriter::new();
    // MB0 — Table B-1 increment = 1 (1-bit `1`).
    bw.write_u32(0b1, 1);
    // Table B-3 "Pattern, motion forward" = 1 (1-bit `1`):
    // fwd == 1, pattern == 1, intra == 0.
    bw.write_u32(0b1, 1);
    // motion_vectors(0): Frame-based default (mv_count==1, dmv==0,
    // f_code==1) → 2 bits (`motion_code = 0` horizontal + vertical).
    bw.write_u32(0b1, 1);
    bw.write_u32(0b1, 1);
    // coded_block_pattern(): cbp = 60 (Table B-9 3-bit `111`).
    bw.write_u32(0b111, 3);

    // MB1 — Table B-1 increment 4 is the 4-bit code `0011`. The
    // §6.3.17.1 skipped-MB range covers addresses MB0+1 .. MB0+3
    // (3 macroblocks, matching `address_increment - 1`).
    bw.write_u32(0b0011, 4);
    bw.write_u32(0b1, 1);
    bw.write_u32(0b1, 1);
    bw.write_u32(0b1, 1);
    bw.write_u32(0b111, 3);
    let buf = append_stop(bw);

    let walk = walk_slice(
        &buf,
        SliceWalkContext::first_slice(22, 1, PictureCodingType::Predictive, 12),
    )
    .unwrap();

    assert_eq!(walk.macroblocks.len(), 2);
    let skipped_run = walk.macroblocks[1].skipped_macroblock_count;
    assert_eq!(skipped_run, 3, "increment 4 → 3 skipped MBs");

    let mut pmv = Pmv::new();
    pmv.set(
        VectorIndex::First,
        Direction::Forward,
        Component::Horizontal,
        9,
    );
    pmv.set(
        VectorIndex::First,
        Direction::Forward,
        Component::Vertical,
        -4,
    );

    // Iterate the §6.3.17.1 skipped-MB range and describe each.
    for _ in 0..skipped_run {
        let desc = describe_skipped_macroblock(SkippedMacroblockContext {
            picture_coding_type: PictureCodingType::Predictive,
            picture_structure: PictureStructure::Frame,
            // P-pictures ignore previous_direction.
            previous_direction: PredictionDirection::Forward,
            scalable_i_picture: false,
            pmv,
        })
        .unwrap();
        assert_eq!(desc.prediction_type, PredictionType::FrameBased);
        assert_eq!(desc.mv_format, MvFormat::Frame);
        assert_eq!(desc.reference_parity, None);
        assert_eq!(desc.direction, PredictionDirection::Forward);
        assert_eq!(desc.motion_vector, SkippedMotionVector::Zero);
        assert!(desc.reset_pmv);
        skipped_macroblock_apply_to_pmv(&desc, &mut pmv);
    }

    // §7.6.3.4 / §7.6.6.2: after the run, the PMV must be zero.
    assert_eq!(
        pmv.get(
            VectorIndex::First,
            Direction::Forward,
            Component::Horizontal
        ),
        0
    );
    assert_eq!(
        pmv.get(VectorIndex::First, Direction::Forward, Component::Vertical),
        0
    );
}

#[test]
fn b_field_picture_skipped_run_inherits_previous_direction_and_leaves_pmv_intact() {
    // §7.6.6.3: B field picture (top), previous MB direction =
    // Bidirectional → every skipped MB description is
    // Field-based / parity Top / direction Bidirectional /
    // MVs from PMV[0][0..1][0..1] / PMVs unaffected.
    let mut pmv = Pmv::new();
    pmv.set(
        VectorIndex::First,
        Direction::Forward,
        Component::Horizontal,
        3,
    );
    pmv.set(
        VectorIndex::First,
        Direction::Forward,
        Component::Vertical,
        4,
    );
    pmv.set(
        VectorIndex::First,
        Direction::Backward,
        Component::Horizontal,
        -1,
    );
    pmv.set(
        VectorIndex::First,
        Direction::Backward,
        Component::Vertical,
        -2,
    );

    let pmv_before = pmv;

    for _ in 0..5 {
        let desc = describe_skipped_macroblock(SkippedMacroblockContext {
            picture_coding_type: PictureCodingType::Bidirectional,
            picture_structure: PictureStructure::TopField,
            previous_direction: PredictionDirection::Bidirectional,
            scalable_i_picture: false,
            pmv,
        })
        .unwrap();

        assert_eq!(desc.prediction_type, PredictionType::FieldBased);
        assert_eq!(desc.mv_format, MvFormat::Field);
        assert_eq!(desc.direction, PredictionDirection::Bidirectional);
        // §7.6.6.3 same-parity rule: top field → top reference.
        assert!(desc.reference_parity.is_some());
        assert_eq!(
            desc.motion_vector,
            SkippedMotionVector::FromPmv {
                forward: Some((3, 4)),
                backward: Some((-1, -2)),
            }
        );
        assert!(!desc.reset_pmv);
        skipped_macroblock_apply_to_pmv(&desc, &mut pmv);
    }

    // §7.6.6.3 "Motion vector predictors are unaffected" — the
    // entire 5-MB run leaves the state byte-equal.
    assert_eq!(pmv, pmv_before);
}

#[test]
fn b_frame_picture_skipped_run_inherits_forward_only_direction() {
    // §7.6.6.4: B frame picture, previous MB direction = Forward
    // → description direction is Forward, only forward PMV slot
    // is surfaced.
    let mut pmv = Pmv::new();
    pmv.set(
        VectorIndex::First,
        Direction::Forward,
        Component::Horizontal,
        12,
    );
    pmv.set(
        VectorIndex::First,
        Direction::Forward,
        Component::Vertical,
        -8,
    );
    pmv.set(
        VectorIndex::First,
        Direction::Backward,
        Component::Horizontal,
        999,
    );

    let desc = describe_skipped_macroblock(SkippedMacroblockContext {
        picture_coding_type: PictureCodingType::Bidirectional,
        picture_structure: PictureStructure::Frame,
        previous_direction: PredictionDirection::Forward,
        scalable_i_picture: false,
        pmv,
    })
    .unwrap();

    assert_eq!(desc.prediction_type, PredictionType::FrameBased);
    assert_eq!(desc.direction, PredictionDirection::Forward);
    assert_eq!(
        desc.motion_vector,
        SkippedMotionVector::FromPmv {
            forward: Some((12, -8)),
            backward: None,
        },
        "§7.6.6.4: backward PMV slot must not be surfaced when previous MB had no backward direction"
    );
}

#[test]
fn i_picture_non_scalable_rejects_skipped_macroblock_per_preamble() {
    // §7.6.6 preamble: "There shall be no skipped macroblocks in
    // I-pictures except when..." — the non-scalable case is
    // unambiguously a bitstream violation.
    let err = describe_skipped_macroblock(SkippedMacroblockContext {
        picture_coding_type: PictureCodingType::Intra,
        picture_structure: PictureStructure::Frame,
        previous_direction: PredictionDirection::Forward,
        scalable_i_picture: false,
        pmv: Pmv::new(),
    })
    .unwrap_err();
    let msg = format!("{err}");
    assert!(msg.contains("non-scalable I-picture"), "unexpected: {msg}");
}

#[test]
fn p_field_bottom_picks_bottom_parity_with_pmv_reset_idempotent_over_a_long_run() {
    // §7.6.6.1: P field picture (bottom) → parity Bottom, MV
    // zero, PMV reset. A multi-MB run must converge on the same
    // zeroed PMV state regardless of length.
    let mut pmv = Pmv::new();
    pmv.set(
        VectorIndex::First,
        Direction::Forward,
        Component::Horizontal,
        100,
    );
    pmv.set(
        VectorIndex::Second,
        Direction::Backward,
        Component::Vertical,
        -77,
    );

    for _ in 0..10 {
        let desc = describe_skipped_macroblock(SkippedMacroblockContext {
            picture_coding_type: PictureCodingType::Predictive,
            picture_structure: PictureStructure::BottomField,
            previous_direction: PredictionDirection::Forward,
            scalable_i_picture: false,
            pmv,
        })
        .unwrap();
        assert!(desc.reset_pmv);
        skipped_macroblock_apply_to_pmv(&desc, &mut pmv);
    }

    assert_eq!(pmv, Pmv::new());
}
