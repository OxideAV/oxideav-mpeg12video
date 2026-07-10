//! Generate the hand-built **field-picture** conformance fixture
//! (`tests/fixtures/conformance/fieldpics-48x64.m2v`): an ISO/IEC
//! 13818-2 elementary stream coded entirely as field-picture pairs,
//! exercising simple field prediction (`motion_vertical_field_select`
//! both ways), **dual-prime** (§7.6.3.6), **16x8 MC** (§7.6.7.3) and
//! B-field pairs — the prediction modes no black-box encoder in reach
//! emits. The reference decode is produced by a black-box decoder
//! binary; see the fixture notes for the commands.
//!
//! Geometry: 48x64 frame -> two 48x32 fields, 3x2 macroblocks per
//! field, two slices per field picture. Motion vectors are only ever
//! given to interior positions/magnitudes that keep every §7.6.4 read
//! inside the coded picture (§7.6.3.8). All inter macroblocks are
//! "Not Coded" (prediction only); the I fields carry per-macroblock
//! DC steps plus one vertical AC coefficient so wrong field/parity
//! selection or vector reconstruction shows as a pixel difference.
//!
//! Usage: `gen_field_conformance <out.m2v>`

use oxideav_core::bits::BitWriter;
use oxideav_mpeg12video::picture_header::PICTURE_START_CODE;
use oxideav_mpeg12video::sequence_extension::EXTENSION_START_CODE;
use oxideav_mpeg12video::sequence_header::SEQUENCE_HEADER_CODE;

const EOB: u32 = 0b10;

fn write_seq(bw: &mut BitWriter) {
    bw.write_u32(SEQUENCE_HEADER_CODE, 32);
    bw.write_u32(48, 12); // horizontal_size
    bw.write_u32(64, 12); // vertical_size
    bw.write_u32(0b0001, 4); // square aspect
    bw.write_u32(0b0011, 4); // 25 fps
    bw.write_u32(2500, 18);
    bw.write_bit(true);
    bw.write_u32(112, 10);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.align_to_byte();
    bw.write_u32(EXTENSION_START_CODE, 32);
    bw.write_u32(0b0001, 4); // sequence extension id
    bw.write_u32(0x48, 8); // Main@Main
    bw.write_bit(false); // progressive_sequence = 0
    bw.write_u32(0b01, 2); // 4:2:0
    bw.write_u32(0, 2);
    bw.write_u32(0, 2);
    bw.write_u32(0, 12);
    bw.write_bit(true);
    bw.write_u32(0, 8);
    bw.write_bit(false);
    bw.write_u32(0, 2);
    bw.write_u32(0, 5);
    bw.align_to_byte();
}

/// Picture header + field picture_coding_extension.
fn write_pic_headers(bw: &mut BitWriter, tr: u32, ct: u32, structure: u32, f_fwd: u32, f_bwd: u32) {
    bw.write_u32(PICTURE_START_CODE, 32);
    bw.write_u32(tr, 10);
    bw.write_u32(ct, 3);
    bw.write_u32(0xFFFF, 16);
    if ct >= 2 {
        bw.write_bit(false); // full_pel_forward_vector ('0' in 13818-2)
        bw.write_u32(7, 3); // forward_f_code placeholder '111'
    }
    if ct == 3 {
        bw.write_bit(false);
        bw.write_u32(7, 3);
    }
    bw.write_bit(false); // extra_bit_picture
    bw.align_to_byte();
    bw.write_u32(EXTENSION_START_CODE, 32);
    bw.write_u32(0b1000, 4); // picture coding extension id
    bw.write_u32(f_fwd, 4);
    bw.write_u32(f_fwd, 4);
    bw.write_u32(f_bwd, 4);
    bw.write_u32(f_bwd, 4);
    bw.write_u32(0, 2); // intra_dc_precision = 8-bit
    bw.write_u32(structure, 2); // 01 top / 10 bottom
    bw.write_bit(true); // top_field_first
    bw.write_bit(false); // frame_pred_frame_dct (forbidden in field pics)
    bw.write_bit(false); // concealment_motion_vectors
    bw.write_bit(false); // q_scale_type
    bw.write_bit(false); // intra_vlc_format
    bw.write_bit(false); // alternate_scan
    bw.write_bit(false); // repeat_first_field
    bw.write_bit(false); // chroma_420_type
    bw.write_bit(false); // progressive_frame
    bw.write_bit(false); // composite_display_flag
    bw.align_to_byte();
}

fn write_slice_open(bw: &mut BitWriter, vertical_position: u32) {
    bw.write_u32(0x0000_0100 | vertical_position, 32);
    bw.write_u32(8, 5); // quantiser_scale_code
    bw.write_bit(false); // extra_bit_slice
}

/// One intra macroblock: first luma block carries `dc_diff` (size-2
/// differential, value in -3..=3 excluding -1..=1), the other luma
/// blocks repeat the predictor, every luma block carries a single AC
/// coefficient at zig-zag index 2 (vertical structure), chroma flat.
fn write_intra_mb(bw: &mut BitWriter, dc_diff: i32, ac_neg: bool) {
    bw.write_bit(true); // macroblock_address_increment = 1
    bw.write_bit(true); // macroblock_type Intra (Table B-2) -- I pictures
                        // DC size 2 ('01') + differential bits.
    let bits = match dc_diff {
        3 => 0b11,
        2 => 0b10,
        -2 => 0b01,
        -3 => 0b00,
        _ => panic!("dc_diff must be +-2/3"),
    };
    bw.write_u32(0b01, 2);
    bw.write_u32(bits, 2);
    bw.write_u32(0b011, 3); // run 1 / level 1 (Table B-14)
    bw.write_bit(ac_neg);
    bw.write_u32(EOB, 2);
    for _ in 0..3 {
        bw.write_u32(0b100, 3); // DC size 0: repeat predictor
        bw.write_u32(0b011, 3);
        bw.write_bit(ac_neg);
        bw.write_u32(EOB, 2);
    }
    for _ in 0..2 {
        bw.write_u32(0b00, 2); // chroma DC size 0
        bw.write_u32(EOB, 2);
    }
}

/// motion_code delta: 0 -> '1', +1 -> '010', -1 -> '011' (Table B-10,
/// f_code 1: no residual bits).
fn write_mv_code(bw: &mut BitWriter, delta: i32) {
    match delta {
        0 => bw.write_bit(true),
        1 => bw.write_u32(0b010, 3),
        -1 => bw.write_u32(0b011, 3),
        _ => panic!("delta out of the f_code-1 single-code range"),
    }
}

/// dmvector: 0 -> '0', +1 -> '10', -1 -> '11' (Table B-11).
fn write_dmv(bw: &mut BitWriter, v: i32) {
    match v {
        0 => bw.write_bit(false),
        1 => bw.write_u32(0b10, 2),
        -1 => bw.write_u32(0b11, 2),
        _ => panic!("dmvector in -1..=1"),
    }
}

/// Tracks the horizontal/vertical forward (and backward) PMV so the
/// per-macroblock `motion_code` deltas land on absolute targets.
#[derive(Default)]
struct Pmv {
    fwd: (i32, i32),
    bwd: (i32, i32),
}

/// P field macroblock, simple field prediction, MC Not Coded.
fn write_p_field_mb(bw: &mut BitWriter, pmv: &mut Pmv, select: u32, target: (i32, i32)) {
    bw.write_bit(true);
    bw.write_u32(0b001, 3); // Table B-3: MC, Not Coded
    bw.write_u32(0b01, 2); // field_motion_type = Field-based
    bw.write_u32(select, 1);
    write_mv_code(bw, target.0 - pmv.fwd.0);
    write_mv_code(bw, target.1 - pmv.fwd.1);
    pmv.fwd = target;
}

/// P field macroblock, dual prime, MC Not Coded. `target` is the
/// same-parity vector (updates the PMV); `dmv` the differential.
fn write_p_dualprime_mb(bw: &mut BitWriter, pmv: &mut Pmv, target: (i32, i32), dmv: (i32, i32)) {
    bw.write_bit(true);
    bw.write_u32(0b001, 3);
    bw.write_u32(0b11, 2); // field_motion_type = Dual prime
                           // dmv == 1: no motion_vertical_field_select (§6.2.5.2).
    write_mv_code(bw, target.0 - pmv.fwd.0);
    write_dmv(bw, dmv.0);
    write_mv_code(bw, target.1 - pmv.fwd.1);
    write_dmv(bw, dmv.1);
    pmv.fwd = target;
}

/// P field macroblock, 16x8 MC, MC Not Coded: two vector sets (upper
/// then lower region), each with its own field select. The first set
/// codes against PMV[0], the second against PMV[1]; with one 16x8 MB
/// per slice-run here both predictors track `pmv.fwd`/`pmv.bwd`.
fn write_p_16x8_mb(
    bw: &mut BitWriter,
    pmv: &mut Pmv,
    upper: (u32, (i32, i32)),
    lower: (u32, (i32, i32)),
) {
    bw.write_bit(true);
    bw.write_u32(0b001, 3);
    bw.write_u32(0b10, 2); // field_motion_type = 16x8 MC
    bw.write_u32(upper.0, 1);
    write_mv_code(bw, upper.1 .0 - pmv.fwd.0);
    write_mv_code(bw, upper.1 .1 - pmv.fwd.1);
    // PMV[1] starts from PMV[0]'s post-update value per §7.6.3.1
    // (vector'[0] predicts from PMV[0], vector'[1] from PMV[1]; both
    // slots were equal at slice start and §7.6.3.3 16x8 rows update
    // both slots pairwise).
    bw.write_u32(lower.0, 1);
    write_mv_code(bw, lower.1 .0 - pmv.bwd.0);
    write_mv_code(bw, lower.1 .1 - pmv.bwd.1);
    pmv.fwd = upper.1;
    pmv.bwd = lower.1;
}

/// B field macroblock, interpolated, Not Coded.
fn write_b_interp_mb(
    bw: &mut BitWriter,
    pmv: &mut Pmv,
    fsel: u32,
    ftarget: (i32, i32),
    bsel: u32,
    btarget: (i32, i32),
) {
    bw.write_bit(true);
    bw.write_u32(0b10, 2); // Table B-4: Interp, Not Coded
    bw.write_u32(0b01, 2);
    bw.write_u32(fsel, 1);
    write_mv_code(bw, ftarget.0 - pmv.fwd.0);
    write_mv_code(bw, ftarget.1 - pmv.fwd.1);
    bw.write_u32(bsel, 1);
    write_mv_code(bw, btarget.0 - pmv.bwd.0);
    write_mv_code(bw, btarget.1 - pmv.bwd.1);
    pmv.fwd = ftarget;
    pmv.bwd = btarget;
}

fn main() {
    let out = std::env::args()
        .nth(1)
        .expect("usage: gen_field_conformance <out.m2v>");
    let mut bw = BitWriter::new();
    write_seq(&mut bw);

    // ---- I field pair, tr 0. Per-MB DC steps + vertical AC.
    for (structure, start_diff, ac_neg) in [(0b01u32, 3i32, false), (0b10, -2, true)] {
        write_pic_headers(&mut bw, 0, 1, structure, 15, 15);
        for row in 1..=2u32 {
            write_slice_open(&mut bw, row);
            // First MB of the slice sets the DC step; the §7.2.1
            // predictor then carries it across the row.
            write_intra_mb(&mut bw, start_diff, ac_neg);
            write_intra_mb(&mut bw, 2, ac_neg);
            write_intra_mb(&mut bw, -2, ac_neg);
            bw.align_to_byte_zero();
        }
    }

    // ---- P field pair, tr 1: top field = dual prime on the centre
    // macroblock of the SECOND slice (the §7.6.3.6 opposite-parity
    // derivation carries e = -1 vertically, so the read must sit
    // below the field's first line); bottom field = opposite-parity
    // select + non-zero MV on the centre macroblocks (downward
    // half-pel in the first slice, upward in the second, keeping the
    // §7.6.4 reads inside the field per §7.6.3.8). Edge macroblocks
    // are same-parity zero-MV copies.
    write_pic_headers(&mut bw, 1, 2, 0b01, 1, 15);
    for row in 1..=2u32 {
        write_slice_open(&mut bw, row);
        let mut pmv = Pmv::default();
        write_p_field_mb(&mut bw, &mut pmv, 0, (0, 0));
        if row == 2 {
            write_p_dualprime_mb(&mut bw, &mut pmv, (1, 0), (1, 0));
        } else {
            write_p_field_mb(&mut bw, &mut pmv, 0, (0, 0));
        }
        write_p_field_mb(&mut bw, &mut pmv, 0, (0, 0));
        bw.align_to_byte_zero();
    }
    write_pic_headers(&mut bw, 1, 2, 0b10, 1, 15);
    for row in 1..=2u32 {
        write_slice_open(&mut bw, row);
        let mut pmv = Pmv::default();
        let v = if row == 1 { 1 } else { -1 };
        write_p_field_mb(&mut bw, &mut pmv, 1, (0, 0));
        write_p_field_mb(&mut bw, &mut pmv, 0, (1, v)); // opposite parity + MV
        write_p_field_mb(&mut bw, &mut pmv, 1, (0, 0));
        bw.align_to_byte_zero();
    }

    // ---- P field pair, tr 2: 16x8 MC on the centre macroblock of
    // each field, distinct field selects / vectors per 16x8 region.
    for (structure, own) in [(0b01u32, 0u32), (0b10, 1)] {
        write_pic_headers(&mut bw, 2, 2, structure, 1, 15);
        for row in 1..=2u32 {
            write_slice_open(&mut bw, row);
            let mut pmv = Pmv::default();
            // Lower-region vertical component points up (-1) so the
            // bottom 16x8 region's reads stay inside the field.
            write_p_field_mb(&mut bw, &mut pmv, own, (0, 0));
            write_p_16x8_mb(&mut bw, &mut pmv, (own, (1, 0)), (1 - own, (-1, -1)));
            write_p_field_mb(&mut bw, &mut pmv, own, (0, 0));
            bw.align_to_byte_zero();
        }
    }

    // ---- P field pair, tr 4 (the backward anchor for the B pair):
    // plain same-parity copies.
    for (structure, own) in [(0b01u32, 0u32), (0b10, 1)] {
        write_pic_headers(&mut bw, 4, 2, structure, 1, 15);
        for row in 1..=2u32 {
            write_slice_open(&mut bw, row);
            let mut pmv = Pmv::default();
            for _ in 0..3 {
                write_p_field_mb(&mut bw, &mut pmv, own, (0, 0));
            }
            bw.align_to_byte_zero();
        }
    }

    // ---- B field pair, tr 3: interpolated centre macroblock with
    // opposite-parity selects; edges forward-only copies.
    for (structure, own) in [(0b01u32, 0u32), (0b10, 1)] {
        write_pic_headers(&mut bw, 3, 3, structure, 1, 1);
        for row in 1..=2u32 {
            write_slice_open(&mut bw, row);
            let mut pmv = Pmv::default();
            // Fwd, Not Coded (Table B-4 '0010'), field-based.
            bw.write_bit(true);
            bw.write_u32(0b0010, 4);
            bw.write_u32(0b01, 2);
            bw.write_u32(own, 1);
            write_mv_code(&mut bw, 0);
            write_mv_code(&mut bw, 0);
            let v = if row == 1 { 1 } else { -1 };
            write_b_interp_mb(&mut bw, &mut pmv, 1 - own, (1, 0), own, (0, v));
            bw.write_bit(true);
            bw.write_u32(0b0010, 4);
            bw.write_u32(0b01, 2);
            bw.write_u32(own, 1);
            write_mv_code(&mut bw, 0 - pmv.fwd.0);
            write_mv_code(&mut bw, 0 - pmv.fwd.1);
            bw.align_to_byte_zero();
        }
    }

    let mut stream = bw.finish();
    stream.extend_from_slice(&0x0000_01B7u32.to_be_bytes());
    std::fs::write(&out, &stream).unwrap();
    eprintln!("wrote {} bytes to {out}", stream.len());
}
