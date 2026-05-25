//! End-to-end synthetic-fixture test for MPEG-1 (ISO/IEC 11172-2:1993)
//! §2.4.4.2 / §2.4.4.3 motion-vector reconstruction.
//!
//! The fixture is a hand-assembled bitstream of two consecutive
//! `motion_vector(s)` elements per §2.4.2.7, parsed via
//! [`oxideav_mpeg12video::Mpeg1MotionVector::parse`] and then fed
//! through [`oxideav_mpeg12video::mpeg1_reconstruct`] with `f_code = 1`
//! and `full_pel = false`. We verify (a) the parser consumes the right
//! number of bits and (b) the PMV state propagates from the first
//! macroblock into the second, matching the §2.4.4.2 closed-form
//! arithmetic.
//!
//! Spec basis: ISO/IEC 11172-2:1993 §2.4.2.7, §2.4.3.6, §2.4.4.2,
//! §2.4.4.3, Annex B Table B.4.

use oxideav_core::bits::{BitReader, BitWriter};
use oxideav_mpeg12video::{
    mpeg1_reconstruct, Mpeg1FrameMvContext, Mpeg1MotionDirection, Mpeg1MotionVector,
    Mpeg1Predictor, Mpeg1ReconstructedMv,
};

/// Helper: build a bitstream containing two `motion_vector(s)`
/// elements at `f_code = 1`. With `f_code = 1`, every component is
/// just a Table B.4 codeword (no residual). We use the spec's Table
/// B.4 row `code = +1 → 0b010` (3 bits) and `code = -1 → 0b011`
/// (3 bits) plus the `code = 0 → 0b1` row (1 bit) to assemble two
/// distinct macroblocks.
fn fixture() -> Vec<u8> {
    let mut bw = BitWriter::new();

    // MB1: horizontal = +1 (0b010, 3b), vertical = -1 (0b011, 3b).
    bw.write_u32(0b010, 3);
    bw.write_u32(0b011, 3);
    // MB2: horizontal = +1 (0b010, 3b), vertical = 0 (0b1, 1b).
    bw.write_u32(0b010, 3);
    bw.write_u32(0b1, 1);

    bw.align_to_byte();
    bw.finish()
}

#[test]
fn two_macroblocks_propagate_pmv() {
    let data = fixture();
    let mut br = BitReader::new(&data);
    let mut pred = Mpeg1Predictor::new();
    let ctx = Mpeg1FrameMvContext {
        f_code: 1,
        full_pel: false,
    };

    // First macroblock.
    let mv1 = Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Forward, 1).unwrap();
    assert_eq!(mv1.horizontal_code, 1);
    assert_eq!(mv1.vertical_code, -1);
    assert_eq!(mv1.bit_position_after, 6);

    let rc1: Mpeg1ReconstructedMv =
        mpeg1_reconstruct(&mv1, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap();
    assert_eq!(rc1.recon_right, 1);
    assert_eq!(rc1.recon_down, -1);
    // PMV holds the recon (in half-sample units pre-shift).
    assert_eq!(pred.recon_right_prev, 1);
    assert_eq!(pred.recon_down_prev, -1);

    // Second macroblock — PMV must carry over.
    let mv2 = Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Forward, 1).unwrap();
    assert_eq!(mv2.horizontal_code, 1);
    assert_eq!(mv2.vertical_code, 0);
    assert_eq!(mv2.bit_position_after, 10);

    let rc2 = mpeg1_reconstruct(&mv2, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap();
    // horizontal: prev (1) + little (1*1=1) = 2.
    assert_eq!(rc2.recon_right, 2);
    // vertical: code = 0 → little = 0 → recon = prev (-1).
    assert_eq!(rc2.recon_down, -1);
    assert_eq!(pred.recon_right_prev, 2);
    assert_eq!(pred.recon_down_prev, -1);
}

#[test]
fn full_pel_doubles_recon_consistently_through_parser() {
    let data = fixture();
    let mut br = BitReader::new(&data);
    let mut pred = Mpeg1Predictor::new();
    let ctx = Mpeg1FrameMvContext {
        f_code: 1,
        full_pel: true,
    };

    let mv1 = Mpeg1MotionVector::parse(&mut br, Mpeg1MotionDirection::Forward, 1).unwrap();
    let rc = mpeg1_reconstruct(&mv1, ctx, &mut pred, Mpeg1MotionDirection::Forward).unwrap();
    // full_pel doubles the recon output but stores predictor pre-shift.
    assert_eq!(rc.recon_right, 2);
    assert_eq!(rc.recon_down, -2);
    assert_eq!(pred.recon_right_prev, 1);
    assert_eq!(pred.recon_down_prev, -1);
}
