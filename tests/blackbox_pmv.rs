//! Black-box validation of the §7.6.3.1 PMV reconstruction against the
//! same MPEG-2 elementary stream produced by an opaque encoder that
//! the other integration tests use, plus a hand-spliced macroblock that
//! drives a multi-vector PMV chain.
//!
//! The fixture under `tests/fixtures/` was produced via:
//!
//! ```text
//! ffmpeg -y -f lavfi -i testsrc=size=352x240:rate=25:duration=0.04 \
//!        -c:v mpeg2video -b:v 800k -an -f mpeg2video out.m2v
//! ```
//!
//! Only the file's *bytes* are consumed here; the encoder's source code
//! is not.
//!
//! The fixture's first picture is an I-picture whose first macroblock is
//! plain `Intra` (no motion vectors). The integration tests in this file
//! confirm:
//!
//! 1. At slice start (and at every fresh I-picture macroblock per
//!    §7.6.3.4) the PMV slots are zero, and the fixture's f_codes are
//!    the "unused" sentinel `15` — so §7.6.3.1 would not run on the
//!    I-picture path at all.
//! 2. A hand-spliced sequence of two P-picture macroblocks (forward
//!    motion, frame-based) decodes through `motion_vectors() →
//!    reconstruct_motion_vector()` and the PMV state evolves the way
//!    §7.6.3.1 says it should: the second vector's `delta = motion_code`
//!    is added on top of the first vector's PMV.

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_mpeg12video::macroblock_modes::{
    MacroblockModesContext, MacroblockModesTail, MvFormat, PredictionType,
};
use oxideav_mpeg12video::motion_vector::{MotionVectors, MotionVectorsContext, MotionVectorsKind};
use oxideav_mpeg12video::picture_header::{
    Mpeg2PictureHeader, PictureCodingType, PictureStructure,
};
use oxideav_mpeg12video::{
    reconstruct_motion_vector, scale_chroma, ChromaFormat, Component, Direction, MacroblockType,
    Pmv, VectorIndex, PICTURE_START_CODE,
};

const FIXTURE: &[u8] = include_bytes!("fixtures/ffmpeg-352x240-25fps.m2v");

fn find_start_code(haystack: &[u8], code: u32) -> Option<usize> {
    haystack.windows(4).position(|w| {
        (u32::from(w[0]) << 24 | u32::from(w[1]) << 16 | u32::from(w[2]) << 8 | u32::from(w[3]))
            == code
    })
}

#[test]
fn fixture_i_picture_pmv_stays_zero_no_reconstruction_runs() {
    // Confirm the fixture's first picture is the spec's "PMV unused"
    // case: I-picture with the §6.3.11 `15` sentinel in every f_code.
    // Per §7.6.3.4 the PMV is reset at slice start; per §6.2.5.2 no
    // motion_vectors() element appears for an Intra macroblock; so the
    // PMV state never moves off zero.
    let pic_pos = find_start_code(FIXTURE, PICTURE_START_CODE).expect("picture start code");
    let (pic, ext) = Mpeg2PictureHeader::parse_with_extension(&FIXTURE[pic_pos..])
        .expect("picture_header + picture_coding_extension");
    assert_eq!(pic.picture_coding_type, PictureCodingType::Intra);
    assert_eq!(ext.picture_structure, PictureStructure::Frame);
    assert_eq!(ext.f_code_fwd_horiz, 15);
    assert_eq!(ext.f_code_fwd_vert, 15);
    assert_eq!(ext.f_code_bwd_horiz, 15);
    assert_eq!(ext.f_code_bwd_vert, 15);

    // Simulate the §7.6.3.4 slice-start reset and confirm every PMV slot
    // is zero — the state §7.6.3.1 would consume if a motion vector
    // arrived.
    let mut pmv = Pmv::new();
    pmv.reset();
    for r in [VectorIndex::First, VectorIndex::Second] {
        for s in [Direction::Forward, Direction::Backward] {
            for t in [Component::Horizontal, Component::Vertical] {
                assert_eq!(pmv.get(r, s, t), 0);
            }
        }
    }
}

#[test]
fn spliced_p_picture_two_macroblock_pmv_chain_predicts_correctly() {
    // Two P-picture frame macroblocks back-to-back; both "MC, Not Coded"
    // (Table B-3 code '001' = motion_forward only, no pattern). Frame
    // motion_type '10'. f_code = 2 (r_size = 1).
    //
    // First macroblock: motion_code = 2, motion_residual = 0 (1 bit).
    //   Per §7.6.3.1: f = 2, delta = (2-1)*2 + 0 + 1 = 3.
    //   PMV starts at 0 ⇒ vector' = 0 + 3 = 3. New PMV = 3.
    //
    // Second macroblock: motion_code = -1, no residual (f_code != 1 but
    //   motion_code != 0 ⇒ residual present; build with residual = 0).
    //   Per §7.6.3.1: delta = -((1-1)*2 + 0 + 1) = -1.
    //   PMV = 3 ⇒ vector' = 3 + (-1) = 2. New PMV = 2.
    //
    // We hand-build only the bits the parsers need.

    let context = MacroblockModesContext {
        picture_structure: PictureStructure::Frame,
        frame_pred_frame_dct: false,
        spatial_temporal_weight_class: 0,
        spatial_temporal_weight_code_table_index: 0,
    };
    let mv_ctx = MotionVectorsContext {
        f_code_fwd_horiz: 2,
        f_code_fwd_vert: 2,
        f_code_bwd_horiz: 15,
        f_code_bwd_vert: 15,
    };

    // ---------- first macroblock ----------
    // macroblock_type = '001' (3 bits, MC + Not Coded, Table B-3) — sets
    // macroblock_motion_forward and clears macroblock_pattern. No quant
    // bit, so the type is followed by frame_motion_type immediately.
    //
    // motion_vectors(0): motion_vector_count = 1, mv_format = frame,
    // dmv = 0 ⇒ no VFS; just the motion_vector(0, 0).
    //
    // motion_code horiz = +2 (Table B-10 code '0010', 4 bits), residual
    // 0 (1 bit, since r_size = 1), motion_code vert = 0 (1 bit '1'), no
    // residual.
    //
    // Layout: [B-3 '001'][frame_motion_type '10'][motion_code+2 '0010']
    //         [residual '0'][motion_code 0 '1']

    let mut bw = BitWriter::new();
    bw.write_u32(0b001, 3); // macroblock_type "MC, Not Coded"
    bw.write_u32(0b10, 2); // frame_motion_type Frame-based
    bw.write_u32(0b0010, 4); // motion_code +2 (Table B-10)
    bw.write_u32(0b0, 1); // motion_residual horiz
    bw.write_u32(0b1, 1); // motion_code 0 (vert)
    bw.write_bit(true); // padding so the slice doesn't run dry
    bw.align_to_byte();
    let mb1 = bw.finish();

    let mut br = BitReader::new(&mb1);
    let mb_type = MacroblockType::parse(&mut br, PictureCodingType::Predictive).expect("mb_type");
    assert!(mb_type.macroblock_motion_forward);
    assert!(!mb_type.macroblock_pattern);

    let tail = MacroblockModesTail::parse(&mut br, &mb_type, &context).expect("tail");
    assert_eq!(
        tail.motion_type.as_ref().unwrap().prediction_type,
        PredictionType::FrameBased
    );
    assert_eq!(
        tail.motion_type.as_ref().unwrap().mv_format,
        MvFormat::Frame
    );

    let mvs = MotionVectors::parse(
        &mut br,
        MotionVectorsKind::Forward,
        tail.motion_type.as_ref().unwrap(),
        &mv_ctx,
    )
    .expect("motion_vectors");
    assert_eq!(mvs.entries.len(), 1);
    assert_eq!(mvs.entries[0].motion_vector.motion_code_horiz, 2);
    assert_eq!(mvs.entries[0].motion_vector.motion_residual_horiz, Some(0));
    assert_eq!(mvs.entries[0].motion_vector.motion_code_vert, 0);

    // §7.6.3.4 slice-start reset.
    let mut pmv = Pmv::new();
    let [h1, v1] = reconstruct_motion_vector(
        &mut pmv,
        &mvs.entries[0].motion_vector,
        VectorIndex::First,
        Direction::Forward,
        mv_ctx.f_code_fwd_horiz,
        mv_ctx.f_code_fwd_vert,
        MvFormat::Frame,
        PictureStructure::Frame,
    )
    .expect("reconstruct mb1");
    assert_eq!(h1.delta, 3);
    assert_eq!(h1.vector_prime, 3);
    assert_eq!(h1.new_pmv, 3);
    assert_eq!(v1.delta, 0);
    assert_eq!(v1.vector_prime, 0);
    assert_eq!(v1.new_pmv, 0);
    assert_eq!(
        pmv.get(
            VectorIndex::First,
            Direction::Forward,
            Component::Horizontal
        ),
        3
    );

    // §7.6.3.7 chroma scaling for 4:2:0 — horiz 3, vert 0 ⇒ chroma 1, 0.
    let scaled = scale_chroma(h1.vector_prime, v1.vector_prime, ChromaFormat::Yuv420);
    assert_eq!(scaled.chroma_horiz, 1);
    assert_eq!(scaled.chroma_vert, 0);

    // ---------- second macroblock ----------
    // Same shape, motion_code horiz = -1 (Table B-10 code '011', 3 bits)
    // + residual 0 (1 bit) + motion_code vert = 0.
    let mut bw = BitWriter::new();
    bw.write_u32(0b001, 3);
    bw.write_u32(0b10, 2);
    bw.write_u32(0b011, 3); // motion_code -1
    bw.write_u32(0b0, 1); // residual
    bw.write_u32(0b1, 1); // vert code 0
    bw.write_bit(true);
    bw.align_to_byte();
    let mb2 = bw.finish();

    let mut br = BitReader::new(&mb2);
    let mb_type = MacroblockType::parse(&mut br, PictureCodingType::Predictive).expect("mb_type");
    let tail = MacroblockModesTail::parse(&mut br, &mb_type, &context).expect("tail");
    let mvs = MotionVectors::parse(
        &mut br,
        MotionVectorsKind::Forward,
        tail.motion_type.as_ref().unwrap(),
        &mv_ctx,
    )
    .expect("motion_vectors mb2");
    assert_eq!(mvs.entries[0].motion_vector.motion_code_horiz, -1);
    assert_eq!(mvs.entries[0].motion_vector.motion_residual_horiz, Some(0));

    let [h2, _v2] = reconstruct_motion_vector(
        &mut pmv,
        &mvs.entries[0].motion_vector,
        VectorIndex::First,
        Direction::Forward,
        mv_ctx.f_code_fwd_horiz,
        mv_ctx.f_code_fwd_vert,
        MvFormat::Frame,
        PictureStructure::Frame,
    )
    .expect("reconstruct mb2");
    assert_eq!(h2.delta, -1);
    assert_eq!(h2.vector_prime, 2); // prior PMV 3 + delta -1 = 2
    assert_eq!(h2.new_pmv, 2);
    assert_eq!(
        pmv.get(
            VectorIndex::First,
            Direction::Forward,
            Component::Horizontal
        ),
        2
    );
}
