//! Frame-picture **field-based** encoders — the MPEG-2
//! `frame_pred_frame_dct = 0` frame-picture encode path (§6.3.10 /
//! §6.2.5.1): per-macroblock **frame vs field prediction** selection
//! (Table 6-17 `frame_motion_type`), per-macroblock **frame vs field
//! DCT** selection (`dct_type`, §6.1.3 / Figure 6-14), and
//! **dual-prime** motion compensation (§7.6.3.6 / §7.6.7.4) inside P
//! frame pictures — the encoder-side mirror of the decode paths
//! [`crate::reconstruct_field_based_macroblock`] /
//! [`crate::reconstruct_frame_dual_prime_macroblock`] drive.
//!
//! ## Syntax emitted (§6.2.5)
//!
//! A frame picture written with `frame_pred_frame_dct = 0` carries, per
//! motion-compensated macroblock, after its Table B-3 / B-4
//! `macroblock_type`:
//!
//! * `frame_motion_type` (2 bits, Table 6-17): `10` Frame-based
//!   (`motion_vector_count = 1`, `mv_format = frame`), `01` Field-based
//!   (`motion_vector_count = 2`, `mv_format = field`), `11` Dual-prime
//!   (`motion_vector_count = 1`, `mv_format = field`, `dmv = 1`);
//! * `dct_type` (1 bit, present when `macroblock_intra ||
//!   macroblock_pattern`): `1` = field DCT — the four luminance blocks
//!   are field-organised at frame-row stride 2 (§6.1.3; 4:2:0 chroma
//!   stays frame-organised);
//! * the motion vectors: a Field-based macroblock carries **two**
//!   vectors per present direction (first predicts the macroblock's
//!   even / top-field frame lines, second its odd / bottom-field lines,
//!   §7.6.5), each with its §6.2.5.2 `motion_vertical_field_select`
//!   flag; a Dual-prime macroblock carries one vector with the Table
//!   B-11 `dmvector[t]` after each component and **no** field-select
//!   flag.
//!
//! ## Motion-vector coding (§7.6.3.1 / §7.6.3.3)
//!
//! Field vectors inside a frame picture code their **vertical**
//! component in field-sample units against the *halved* predictor: the
//! encoder computes `delta = vector' - (PMV DIV 2)` and writes
//! `PMV = 2 * vector'` back, exactly inverting the §7.6.3.1
//! vertical-half-pred branch the decoder runs. The §7.6.3.3 Table 7-10
//! update rows are mirrored with the crate's own [`crate::pmv::Pmv`]
//! bank: Frame-based copies `PMV[0][s]` into `PMV[1][s]` per present
//! direction, Field-based updates both `r` slots in place ("(none)"
//! row), Dual-prime copies the forward pair, and an intra macroblock
//! (no concealment vectors) resets the bank.
//!
//! ## Reconstruction
//!
//! Every macroblock is reconstructed by the **decode-side §7.6
//! drivers** ([`crate::reconstruct_inter_macroblock`] /
//! [`crate::reconstruct_field_based_macroblock`] /
//! [`crate::reconstruct_frame_dual_prime_macroblock`]) fed with the
//! encoder's quantised-residual `f_pel` blocks, so the reference frames
//! the encoder carries forward are the decoder's exact
//! reconstructions.
//!
//! ## Sequence-level constraints
//!
//! `frame_pred_frame_dct = 0` frame pictures are emitted with
//! `progressive_frame = 0` in an interlaced sequence
//! (`progressive_sequence = 0`, §6.3.10: *"If progressive_frame is set
//! to 1 ... frame_pred_frame_dct shall be 1"*), so the macroblock grid
//! is the §6.3.3 interlaced `2 * Ceil(height / 32)` rows.
//! Dual-prime is only searched when the caller asserts the §7.6.3.6
//! constraint (*"there shall be no B-pictures between the predicted and
//! reference frame"*) holds — the assembler enables it only for
//! `b_between == 0` sequences.

#![allow(clippy::too_many_arguments)]
#![allow(clippy::needless_range_loop)]
// Codewords are grouped to mirror the printed Annex B / Table 6-17
// layouts for audit against the spec pages.
#![allow(clippy::unusual_byte_groupings)]

use oxideav_core::bits::BitWriter;

use crate::b_picture_encoder::{write_b_macroblock_type, BDirection};
use crate::coded_block_pattern::encode_cbp420;
use crate::dual_prime::{derive_all, DualPrimePicture, FieldParity};
use crate::field_picture_encoder::FieldSearchResult;
use crate::forming_predictions::{predict_field_block, BlockSize, FieldReference, ReferencePlane};
use crate::forward_dct::fdct_8x8;
use crate::forward_quant::forward_quantise_block;
use crate::frame_assembly::{block_placement, place_intra_block, FrameBuffer, IntraPictureParams};
use crate::gop_header::{write_gop_header, Mpeg2Gop, TimeCode};
use crate::idct::idct_8x8_from_i32;
use crate::inter_reconstruction::{
    predict_field_based_macroblock_planes, predict_frame_dual_prime_macroblock_planes,
    predict_frame_macroblock_planes, reconstruct_field_based_macroblock,
    reconstruct_frame_dual_prime_macroblock, reconstruct_inter_macroblock, FieldBasedMotion,
    FieldVector, FrameDualPrimeMotion, FrameMotion, MotionVectorPel, ReferenceFrames,
    ResidualBlock,
};
use crate::motion_estimation::{estimate_forward_mv, max_search_range};
use crate::motion_vector::{encode_dmvector, encode_motion_component};
use crate::mpeg2_block_dc::{encode_intra_dc, ColourComponent, DcComponent};
use crate::mpeg2_dct_coeff::{
    encode_dct_coeff, encode_end_of_block, CoefficientPosition, TableSelection,
};
use crate::mpeg2_dequantize::{
    intra_dc_mult, inverse_quantise_block, quantiser_scale, BlockCoding, DEFAULT_INTRA_WEIGHT,
};
use crate::mpeg2_inverse_scan::inverse_scan_table;
use crate::mpeg2_macroblock_blocks::{block_component, block_count};
use crate::p_picture_encoder::{
    intra_activity, quantise_inter_block, wrap_delta, write_inter_block_coeffs, InterBlock,
    IntraDcPred,
};
use crate::picture_header::PictureCodingType;
use crate::sequence_extension::ChromaFormat;
use crate::stream_writer::{
    write_picture_coding_extension, write_picture_header, write_sequence_extension,
    write_sequence_header, write_slice_header, PictureCodingExtensionParams, SequenceHeaderParams,
    SEQUENCE_END_CODE,
};
use crate::{Error, Result};

/// A macroblock's chroma extent for the 4:2:0 layout this encoder
/// supports (8×8 per chroma block).
const CHROMA_MB: usize = 8;

/// Flat bit-cost bias (in SAD units) charged to a Field-based
/// macroblock when competing with a Frame-based one: two field-select
/// flags plus a second vector pair cost wire bits a frame vector does
/// not, so field prediction must win by a margin to be worth it.
const FIELD_MC_BIAS: u32 = 64;

/// Flat bias charged to a Dual-prime macroblock: one vector plus two
/// 1–2-bit `dmvector` codes — cheaper than Field-based, dearer than
/// Frame-based.
const DUAL_PRIME_BIAS: u32 = 32;

/// Per-macroblock statistics one frame-picture encode reports, so
/// callers and tests can confirm the per-macroblock adaptive decisions
/// (frame/field prediction, dual-prime, field DCT) actually fire.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FrameFieldStats {
    /// Macroblocks coded with Table 6-17 `Frame-based` prediction.
    pub frame_mc: usize,
    /// Macroblocks coded with Table 6-17 `Field-based` prediction.
    pub field_mc: usize,
    /// Macroblocks coded with Table 6-17 `Dual-prime` prediction.
    pub dual_prime: usize,
    /// Intra-coded macroblocks (I-picture macroblocks or P/B intra
    /// fallbacks).
    pub intra: usize,
    /// Macroblocks whose transmitted `dct_type` selected **field DCT**.
    pub field_dct: usize,
}

impl FrameFieldStats {
    /// Accumulate another picture's counters into this one.
    pub fn add(&mut self, other: &FrameFieldStats) {
        self.frame_mc += other.frame_mc;
        self.field_mc += other.field_mc;
        self.dual_prime += other.dual_prime;
        self.intra += other.intra;
        self.field_dct += other.field_dct;
    }
}

/// Validate the geometry / format constraints shared by the
/// frame-picture field-based encoders.
fn check_ff_params(params: &IntraPictureParams) -> Result<()> {
    if params.chroma_format != ChromaFormat::Yuv420 {
        return Err(Error::InvalidBitstream(
            "frame-field encoder: only 4:2:0 is supported",
        ));
    }
    if params.frame_pred_frame_dct {
        return Err(Error::InvalidBitstream(
            "frame-field encoder: frame_pred_frame_dct must be 0 (use the \
             frame_pred_frame_dct = 1 encoders otherwise)",
        ));
    }
    if params.progressive_sequence {
        return Err(Error::InvalidBitstream(
            "frame-field encoder: frame_pred_frame_dct = 0 pictures are emitted with \
             progressive_frame = 0, which requires an interlaced sequence (§6.3.10)",
        ));
    }
    if params.alternate_scan || params.intra_vlc_format {
        return Err(Error::InvalidBitstream(
            "frame-field encoder: alternate_scan / intra_vlc_format are not supported",
        ));
    }
    Ok(())
}

/// Write the picture header + frame-picture `picture_coding_extension()`
/// with `frame_pred_frame_dct = 0` / `progressive_frame = 0`.
fn write_ff_picture_headers(
    bw: &mut BitWriter,
    params: &IntraPictureParams,
    coding_type: PictureCodingType,
    temporal_reference: u16,
    forward_f_code: u8,
    backward_f_code: u8,
) {
    // §6.3.10: the MPEG-1 legacy f_code fields in the picture header
    // are unused ('111') in an ISO/IEC 13818-2 stream.
    write_picture_header(bw, temporal_reference, coding_type, 0b111, 0b111);
    write_picture_coding_extension(
        bw,
        &PictureCodingExtensionParams {
            forward_f_code,
            backward_f_code,
            intra_dc_precision: params.intra_dc_precision,
            q_scale_type: params.q_scale_type,
            intra_vlc_format: params.intra_vlc_format,
            alternate_scan: params.alternate_scan,
            frame_pred_frame_dct: false,
            progressive_frame: false,
        },
    );
}

// =============================================================
// Field-in-frame motion search
// =============================================================

/// Luma SAD between the current frame macroblock's `dest_parity` lines
/// (its even frame rows for `Top`, odd for `Bottom`) and the §7.6.4
/// field prediction `(ref_parity, (hx, hy))` forms from `reference`.
/// The block is 16 wide × 8 field lines; edge reads of the current
/// frame clamp (the macroblock grid pads the storage).
fn field_in_frame_luma_sad(
    current: &FrameBuffer,
    reference: &FrameBuffer,
    ref_parity: FieldParity,
    dest_parity: FieldParity,
    mb_col: usize,
    mb_row: usize,
    hx: i32,
    hy: i32,
) -> u32 {
    let plane = ReferencePlane::new(
        reference.y.samples(),
        reference.y.width(),
        reference.y.height(),
    )
    .expect("reference luma plane is width*height");
    let Some(field) = FieldReference::new(plane, ref_parity.index()) else {
        return u32::MAX;
    };
    let size = BlockSize::new(16, 8).expect("16x8 is a valid block size");
    // The macroblock's first top-field frame row is mb_row*16; its
    // co-located field line is mb_row*8 in either reference field.
    let pred = predict_field_block(
        field,
        (mb_col * 16) as i32,
        (mb_row * 8) as i32,
        size,
        hx,
        hy,
    );
    let w = current.y.width();
    let h = current.y.height();
    let base_x = mb_col * 16;
    let base_y = mb_row * 16 + dest_parity.index();
    let mut sad = 0u32;
    for r in 0..8 {
        let sy = (base_y + 2 * r).min(h.saturating_sub(1));
        for c in 0..16 {
            let sx = (base_x + c).min(w.saturating_sub(1));
            let cur = i32::from(current.y.get(sx, sy).unwrap_or(0));
            let prd = i32::from(pred[r * 16 + c]);
            sad += (cur - prd).unsigned_abs();
        }
    }
    sad
}

/// Whether the 16×8 §7.6.4 read span of `(hx, hy)` at the macroblock's
/// field origin stays inside the `ref_parity` field view of a
/// `fw × frame_h` luma plane (§7.6.3.8).
fn field_span_legal(
    fw: i32,
    frame_h: i32,
    ref_parity: FieldParity,
    mb_col: usize,
    mb_row: usize,
    hx: i32,
    hy: i32,
) -> bool {
    let fh = (frame_h - ref_parity.index() as i32 + 1) / 2;
    let base_x = (mb_col * 16) as i32;
    let base_y = (mb_row * 8) as i32;
    let ix = hx.div_euclid(2);
    let iy = hy.div_euclid(2);
    let ex = i32::from(hx.rem_euclid(2) != 0);
    let ey = i32::from(hy.rem_euclid(2) != 0);
    base_x + ix >= 0 && base_y + iy >= 0 && base_x + ix + 15 + ex < fw && base_y + iy + 7 + ey < fh
}

/// Estimate the field motion vector predicting the macroblock's
/// `dest_parity` frame lines from `reference`, searching **both
/// reference field parities** (integer full search + half-pel
/// refinement), §7.6.3.8-legal spans only. The vertical component is in
/// field-sample half units.
pub fn estimate_field_in_frame_mv(
    current: &FrameBuffer,
    reference: &FrameBuffer,
    dest_parity: FieldParity,
    mb_col: usize,
    mb_row: usize,
    search_range: i32,
) -> FieldSearchResult {
    let fw = reference.y.width() as i32;
    let frame_h = reference.y.height() as i32;
    let vec_cost = |hx: i32, hy: i32| -> u32 { hx.unsigned_abs() + hy.unsigned_abs() };

    let mut best: Option<FieldSearchResult> = None;
    let mut best_score = u32::MAX;
    for parity in [FieldParity::Top, FieldParity::Bottom] {
        let legal =
            |hx: i32, hy: i32| field_span_legal(fw, frame_h, parity, mb_col, mb_row, hx, hy);
        let mut p_best: Option<(i32, i32, u32, u32)> = None; // (hx, hy, sad, score)
        let consider =
            |hx: i32, hy: i32, p_best: &mut Option<(i32, i32, u32, u32)>, current: &FrameBuffer| {
                if !legal(hx, hy) {
                    return;
                }
                let sad = field_in_frame_luma_sad(
                    current,
                    reference,
                    parity,
                    dest_parity,
                    mb_col,
                    mb_row,
                    hx,
                    hy,
                );
                let score = sad.saturating_add(vec_cost(hx, hy));
                if p_best.map(|b| score < b.3).unwrap_or(true) {
                    *p_best = Some((hx, hy, sad, score));
                }
            };
        for dy in -search_range..=search_range {
            for dx in -search_range..=search_range {
                consider(dx * 2, dy * 2, &mut p_best, current);
            }
        }
        if let Some((int_hx, int_hy, _, _)) = p_best {
            for &(ox, oy) in &[
                (-1i32, -1i32),
                (0, -1),
                (1, -1),
                (-1, 0),
                (1, 0),
                (-1, 1),
                (0, 1),
                (1, 1),
            ] {
                consider(int_hx + ox, int_hy + oy, &mut p_best, current);
            }
        }
        if let Some((hx, hy, sad, score)) = p_best {
            if score < best_score {
                best_score = score;
                best = Some(FieldSearchResult {
                    vector: MotionVectorPel::new(hx, hy),
                    parity,
                    sad,
                });
            }
        }
    }
    // The zero vector into the same-parity field is always legal for
    // any in-grid macroblock, so a winner exists.
    best.expect("at least the (0,0) same-parity candidate is legal")
}

/// Search the shared **dual-prime base vector** (§7.6.3.6: one vector
/// applied same-parity to both fields — top from top reference, bottom
/// from bottom), returning `(vector, summed same-parity SAD)`.
fn estimate_dual_prime_base(
    current: &FrameBuffer,
    reference: &FrameBuffer,
    mb_col: usize,
    mb_row: usize,
    search_range: i32,
) -> (MotionVectorPel, u32) {
    let fw = reference.y.width() as i32;
    let frame_h = reference.y.height() as i32;
    let legal = |hx: i32, hy: i32| {
        field_span_legal(fw, frame_h, FieldParity::Top, mb_col, mb_row, hx, hy)
            && field_span_legal(fw, frame_h, FieldParity::Bottom, mb_col, mb_row, hx, hy)
    };
    let sad_of = |hx: i32, hy: i32| -> u32 {
        field_in_frame_luma_sad(
            current,
            reference,
            FieldParity::Top,
            FieldParity::Top,
            mb_col,
            mb_row,
            hx,
            hy,
        )
        .saturating_add(field_in_frame_luma_sad(
            current,
            reference,
            FieldParity::Bottom,
            FieldParity::Bottom,
            mb_col,
            mb_row,
            hx,
            hy,
        ))
    };
    let mut best = (0i32, 0i32, sad_of(0, 0));
    for dy in -search_range..=search_range {
        for dx in -search_range..=search_range {
            if dx == 0 && dy == 0 {
                continue;
            }
            let (hx, hy) = (dx * 2, dy * 2);
            if !legal(hx, hy) {
                continue;
            }
            let sad = sad_of(hx, hy);
            if sad < best.2 {
                best = (hx, hy, sad);
            }
        }
    }
    let (int_hx, int_hy, _) = best;
    for &(ox, oy) in &[
        (-1i32, -1i32),
        (0, -1),
        (1, -1),
        (-1, 0),
        (1, 0),
        (-1, 1),
        (0, 1),
        (1, 1),
    ] {
        let (hx, hy) = (int_hx + ox, int_hy + oy);
        if !legal(hx, hy) {
            continue;
        }
        let sad = sad_of(hx, hy);
        if sad < best.2 {
            best = (hx, hy, sad);
        }
    }
    (MotionVectorPel::new(best.0, best.1), best.2)
}

/// Whether all four §7.6.3.6 / §7.6.7.4 field read spans of a
/// frame-picture dual-prime motion stay inside the reference fields:
/// the same-parity vector on both parities and each derived
/// opposite-parity vector on its opposite field.
fn dual_prime_spans_legal(
    reference: &FrameBuffer,
    mb_col: usize,
    mb_row: usize,
    motion: &FrameDualPrimeMotion,
) -> bool {
    let fw = reference.y.width() as i32;
    let frame_h = reference.y.height() as i32;
    let m = motion;
    // Top predicted field: same from Top ref, opposite from Bottom ref.
    // Bottom predicted field: same from Bottom ref, opposite from Top.
    field_span_legal(
        fw,
        frame_h,
        FieldParity::Top,
        mb_col,
        mb_row,
        m.same_parity_vector.horizontal,
        m.same_parity_vector.vertical,
    ) && field_span_legal(
        fw,
        frame_h,
        FieldParity::Bottom,
        mb_col,
        mb_row,
        m.same_parity_vector.horizontal,
        m.same_parity_vector.vertical,
    ) && field_span_legal(
        fw,
        frame_h,
        FieldParity::Bottom,
        mb_col,
        mb_row,
        m.top_field_opposite_vector.horizontal,
        m.top_field_opposite_vector.vertical,
    ) && field_span_legal(
        fw,
        frame_h,
        FieldParity::Top,
        mb_col,
        mb_row,
        m.bottom_field_opposite_vector.horizontal,
        m.bottom_field_opposite_vector.vertical,
    )
}

// =============================================================
// Residual gather / cost under frame vs field DCT
// =============================================================

/// Gather one 8×8 `current - prediction` residual block at
/// `placement`, honouring the placement's §6.1.3 frame-row stride
/// (`1` frame DCT, `2` field DCT). `pred` is the macroblock-local
/// prediction plane in **frame-row** order (`pred_w` wide);
/// `(origin_x, origin_y)` the macroblock's top-left in the component
/// plane.
fn gather_residual_placed(
    cur_plane: &crate::frame_assembly::Plane,
    pred: &[u8],
    pred_w: usize,
    placement: crate::frame_assembly::BlockPlacement,
    origin_x: usize,
    origin_y: usize,
) -> [[i16; 8]; 8] {
    let stride = placement.row_stride();
    let w = cur_plane.width();
    let h = cur_plane.height();
    let local_x0 = placement.base_x - origin_x;
    let local_y0 = placement.base_y - origin_y;
    let mut out = [[0i16; 8]; 8];
    for r in 0..8 {
        let sy = (placement.base_y + r * stride).min(h.saturating_sub(1));
        let py = local_y0 + r * stride;
        for c in 0..8 {
            let sx = (placement.base_x + c).min(w.saturating_sub(1));
            let cur = i32::from(cur_plane.get(sx, sy).unwrap_or(0));
            let prd = i32::from(pred[py * pred_w + (local_x0 + c)]);
            out[r][c] = (cur - prd) as i16;
        }
    }
    out
}

/// Quantise all blocks of one inter macroblock against its prediction
/// planes under the given `dct_type`, returning `(blocks, cbp)`.
fn quantise_mb_residuals(
    current: &FrameBuffer,
    luma_pred: &[u8],
    cb_pred: &[u8],
    cr_pred: &[u8],
    mb_col: usize,
    mb_row: usize,
    qscale: u8,
    field_dct: bool,
    chroma_format: ChromaFormat,
) -> (Vec<InterBlock>, u8) {
    let nblocks = block_count(chroma_format);
    let mut blocks = Vec::with_capacity(nblocks);
    let mut cbp = 0u8;
    for i in 0..nblocks {
        let placement =
            block_placement(i, chroma_format, mb_col, mb_row, field_dct).expect("valid index");
        let component = block_component(i, chroma_format).expect("valid component");
        let (cur_plane, pred, pred_w, origin_x, origin_y) = match component {
            ColourComponent::Y => (&current.y, luma_pred, 16usize, mb_col * 16, mb_row * 16),
            ColourComponent::Cb => (
                &current.cb,
                cb_pred,
                CHROMA_MB,
                mb_col * CHROMA_MB,
                mb_row * CHROMA_MB,
            ),
            ColourComponent::Cr => (
                &current.cr,
                cr_pred,
                CHROMA_MB,
                mb_col * CHROMA_MB,
                mb_row * CHROMA_MB,
            ),
        };
        let residual =
            gather_residual_placed(cur_plane, pred, pred_w, placement, origin_x, origin_y);
        let block = quantise_inter_block(&residual, qscale);
        if block.is_coded() {
            cbp |= 1 << (5 - i);
        }
        blocks.push(block);
    }
    (blocks, cbp)
}

/// Exact §7.2.2 wire-bit cost of a quantised luma block set: the sum of
/// the run-level VLC + `end_of_block` lengths of every coded block.
fn luma_coeff_bits(blocks: &[InterBlock]) -> u64 {
    let mut scratch = BitWriter::new();
    for b in blocks.iter().take(4) {
        if let Some(qf) = b.qf_ref() {
            write_inter_block_coeffs(&mut scratch, qf);
        }
    }
    scratch.bit_position()
}

/// Choose the macroblock's `dct_type` for an inter macroblock: quantise
/// the luminance residual in both §6.1.3 organisations and keep the one
/// whose coded luma costs fewer exact wire bits (ties → frame DCT).
/// Returns `(field_dct, blocks, cbp)` with the **full** block set (luma
/// + chroma) quantised under the winner.
fn choose_inter_dct(
    current: &FrameBuffer,
    luma_pred: &[u8],
    cb_pred: &[u8],
    cr_pred: &[u8],
    mb_col: usize,
    mb_row: usize,
    qscale: u8,
    chroma_format: ChromaFormat,
) -> (bool, Vec<InterBlock>, u8) {
    let (frame_blocks, frame_cbp) = quantise_mb_residuals(
        current,
        luma_pred,
        cb_pred,
        cr_pred,
        mb_col,
        mb_row,
        qscale,
        false,
        chroma_format,
    );
    let (field_blocks, field_cbp) = quantise_mb_residuals(
        current,
        luma_pred,
        cb_pred,
        cr_pred,
        mb_col,
        mb_row,
        qscale,
        true,
        chroma_format,
    );
    if luma_coeff_bits(&field_blocks) < luma_coeff_bits(&frame_blocks) {
        (true, field_blocks, field_cbp)
    } else {
        (false, frame_blocks, frame_cbp)
    }
}

// =============================================================
// Intra macroblock with dct_type
// =============================================================

/// Gather one raw 8×8 intra block at `placement`, honouring its
/// frame-row stride.
fn gather_intra_placed(
    frame: &FrameBuffer,
    component: ColourComponent,
    placement: crate::frame_assembly::BlockPlacement,
) -> [[i16; 8]; 8] {
    let plane = match component {
        ColourComponent::Y => &frame.y,
        ColourComponent::Cb => &frame.cb,
        ColourComponent::Cr => &frame.cr,
    };
    let stride = placement.row_stride();
    let w = plane.width();
    let h = plane.height();
    let mut out = [[0i16; 8]; 8];
    for r in 0..8 {
        let sy = (placement.base_y + r * stride).min(h.saturating_sub(1));
        for c in 0..8 {
            let sx = (placement.base_x + c).min(w.saturating_sub(1));
            out[r][c] = i16::from(plane.get(sx, sy).unwrap_or(128));
        }
    }
    out
}

fn dc_table_component(c: ColourComponent) -> DcComponent {
    match c {
        ColourComponent::Y => DcComponent::Luminance,
        ColourComponent::Cb | ColourComponent::Cr => DcComponent::Chrominance,
    }
}

/// Exact §7.2.1/§7.2.2 luma AC bit cost of intra-coding this
/// macroblock's four luminance blocks under one DCT organisation
/// (DC differentials excluded — they depend on the predictor chain,
/// which both organisations share block-for-block).
fn intra_luma_ac_bits(
    frame: &FrameBuffer,
    mb_col: usize,
    mb_row: usize,
    qscale: u8,
    dc_mult: i32,
    field_dct: bool,
    chroma_format: ChromaFormat,
) -> u64 {
    let scan = inverse_scan_table(false);
    let mut scratch = BitWriter::new();
    for i in 0..4usize {
        let placement =
            block_placement(i, chroma_format, mb_col, mb_row, field_dct).expect("valid index");
        let raw = gather_intra_placed(frame, ColourComponent::Y, placement);
        let f = fdct_8x8(&raw);
        let qf = forward_quantise_block(
            &f,
            BlockCoding::Intra,
            &DEFAULT_INTRA_WEIGHT,
            qscale,
            dc_mult,
        );
        let mut run = 0u8;
        for &(v, u) in scan.iter().skip(1) {
            let level = qf[v as usize][u as usize];
            if level == 0 {
                run += 1;
                continue;
            }
            encode_dct_coeff(
                &mut scratch,
                TableSelection::TableZero,
                CoefficientPosition::Next,
                run,
                level.clamp(-2047, 2047) as i16,
            );
            run = 0;
        }
        encode_end_of_block(&mut scratch, TableSelection::TableZero);
    }
    scratch.bit_position()
}

/// Encode + reconstruct one intra macroblock of a
/// `frame_pred_frame_dct = 0` frame picture, honouring `field_dct`
/// (§6.1.3 block organisation) for the luminance blocks (4:2:0 chroma
/// stays frame-organised). The caller has already written the
/// macroblock address / type / `dct_type` bits; this writes the block
/// layer and places the reconstruction.
fn encode_ff_intra_mb(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    recon: &mut FrameBuffer,
    mb_col: usize,
    mb_row: usize,
    qscale: u8,
    dc_mult: i32,
    pred: &mut IntraDcPred,
    field_dct: bool,
    chroma_format: ChromaFormat,
) {
    let scan = inverse_scan_table(false);
    let table = TableSelection::TableZero;
    let nblocks = block_count(chroma_format);
    for i in 0..nblocks {
        let placement =
            block_placement(i, chroma_format, mb_col, mb_row, field_dct).expect("valid index");
        let component = block_component(i, chroma_format).expect("valid component");
        let raw = gather_intra_placed(current, component, placement);
        let f = fdct_8x8(&raw);
        let qf = forward_quantise_block(
            &f,
            BlockCoding::Intra,
            &DEFAULT_INTRA_WEIGHT,
            qscale,
            dc_mult,
        );

        // §7.2.1 DC differential.
        let qfs0 = qf[0][0];
        let diff = qfs0 - pred.get(component);
        encode_intra_dc(bw, dc_table_component(component), diff);
        pred.set(component, qfs0);

        // §7.2.2 AC run-level.
        let mut run = 0u8;
        for &(v, u) in scan.iter().skip(1) {
            let level = qf[v as usize][u as usize];
            if level == 0 {
                run += 1;
                continue;
            }
            encode_dct_coeff(
                bw,
                table,
                CoefficientPosition::Next,
                run,
                level.clamp(-2047, 2047) as i16,
            );
            run = 0;
        }
        encode_end_of_block(bw, table);

        // Decoder-exact reconstruction into `recon` at the placement's
        // stride.
        let dequant = inverse_quantise_block(
            &qf,
            BlockCoding::Intra,
            &DEFAULT_INTRA_WEIGHT,
            qscale,
            dc_mult,
        );
        let f_pel = idct_8x8_from_i32(&dequant);
        place_intra_block(recon, placement, &f_pel);
    }
}

// =============================================================
// PMV mirror + motion-vector emission
// =============================================================

/// The encoder-side mirror of the decoder's §7.6.3 predictor bank —
/// [`crate::pmv::Pmv`] driven with exactly the §7.6.3.1 / §7.6.3.3
/// arithmetic the slice walker runs.
struct PmvMirror {
    pmv: crate::pmv::Pmv,
}

impl PmvMirror {
    fn new() -> Self {
        Self {
            pmv: crate::pmv::Pmv::new(),
        }
    }

    /// §7.6.3.4 reset (slice start, intra macroblock).
    fn reset(&mut self) {
        self.pmv.reset();
    }

    fn get(&self, r: usize, s: usize, t: usize) -> i32 {
        self.pmv.values[r][s][t]
    }

    fn set(&mut self, r: usize, s: usize, t: usize, v: i32) {
        self.pmv.values[r][s][t] = v;
    }

    /// Table 7-10 Frame-based / Dual-prime row: copy `PMV[0][s]` into
    /// `PMV[1][s]`.
    fn copy_r0_to_r1(&mut self, s: usize) {
        self.pmv.values[1][s] = self.pmv.values[0][s];
    }
}

/// Emit one **frame-format** motion vector (`mv_format = frame`)
/// differentially against `PMV[0][s]`, updating the slot; the caller
/// applies the Table 7-10 copy row afterwards.
fn emit_frame_vector(
    bw: &mut BitWriter,
    pmv: &mut PmvMirror,
    s: usize,
    mv: MotionVectorPel,
    f_code: u8,
) -> Result<()> {
    let dx = wrap_delta(mv.horizontal - pmv.get(0, s, 0), f_code)?;
    let dy = wrap_delta(mv.vertical - pmv.get(0, s, 1), f_code)?;
    encode_motion_component(bw, dx, f_code);
    encode_motion_component(bw, dy, f_code);
    pmv.set(0, s, 0, mv.horizontal);
    pmv.set(0, s, 1, mv.vertical);
    Ok(())
}

/// Emit one **field-format** vector `r` of a frame picture
/// (`mv_format = field`): the §6.2.5.2 `motion_vertical_field_select`
/// flag, then the components — the vertical coded against the
/// §7.6.3.1-halved `PMV[r][s][1]` with `2 * vector'` written back.
fn emit_field_vector(
    bw: &mut BitWriter,
    pmv: &mut PmvMirror,
    r: usize,
    s: usize,
    fv: FieldVector,
    f_code: u8,
) -> Result<()> {
    bw.write_bit(fv.reference_field == FieldParity::Bottom);
    let dx = wrap_delta(fv.vector.horizontal - pmv.get(r, s, 0), f_code)?;
    let dy = wrap_delta(fv.vector.vertical - pmv.get(r, s, 1).div_euclid(2), f_code)?;
    encode_motion_component(bw, dx, f_code);
    encode_motion_component(bw, dy, f_code);
    pmv.set(r, s, 0, fv.vector.horizontal);
    pmv.set(r, s, 1, fv.vector.vertical * 2);
    Ok(())
}

/// Emit a **dual-prime** vector (`mv_format = field`, `dmv = 1`, no
/// field-select flag): each component followed by its Table B-11
/// `dmvector[t]`; PMV handling as [`emit_field_vector`] on the `r = 0`
/// forward slot, with the Table 7-10 Dual-prime copy applied by the
/// caller.
fn emit_dual_prime_vector(
    bw: &mut BitWriter,
    pmv: &mut PmvMirror,
    mv: MotionVectorPel,
    dmv: (i8, i8),
    f_code: u8,
) -> Result<()> {
    let dx = wrap_delta(mv.horizontal - pmv.get(0, 0, 0), f_code)?;
    encode_motion_component(bw, dx, f_code);
    encode_dmvector(bw, dmv.0);
    let dy = wrap_delta(mv.vertical - pmv.get(0, 0, 1).div_euclid(2), f_code)?;
    encode_motion_component(bw, dy, f_code);
    encode_dmvector(bw, dmv.1);
    pmv.set(0, 0, 0, mv.horizontal);
    pmv.set(0, 0, 1, mv.vertical * 2);
    pmv.copy_r0_to_r1(0);
    Ok(())
}

// =============================================================
// Picture encoders
// =============================================================

/// The chosen prediction mode of one P-macroblock.
enum PMode {
    Frame(MotionVectorPel),
    Field(FieldVector, FieldVector),
    DualPrime(MotionVectorPel, (i8, i8), FrameDualPrimeMotion),
}

/// Encode one **intra frame picture** with `frame_pred_frame_dct = 0`:
/// every macroblock is Table B-2 Intra with a transmitted `dct_type`
/// chosen per macroblock by exact luma AC bit cost.
///
/// Returns `(reconstruction, stats)`; the reconstruction storage covers
/// the §6.3.3 interlaced macroblock grid.
///
/// # Errors
/// [`Error::InvalidBitstream`] for geometry / format violations.
pub fn encode_ff_intra_picture(
    bw: &mut BitWriter,
    frame: &FrameBuffer,
    params: &IntraPictureParams,
    temporal_reference: u16,
    quantiser_scale_code: u8,
) -> Result<(FrameBuffer, FrameFieldStats)> {
    check_ff_params(params)?;
    if frame.width != params.width || frame.height != params.height {
        return Err(Error::InvalidBitstream(
            "encode_ff_intra_picture: frame dimensions do not match params",
        ));
    }
    let qscale = quantiser_scale(quantiser_scale_code, params.q_scale_type)?;
    let dc_mult = intra_dc_mult(params.intra_dc_precision)?;

    write_ff_picture_headers(
        bw,
        params,
        PictureCodingType::Intra,
        temporal_reference,
        15,
        15,
    );

    let mut recon = params.new_frame_buffer();
    let mut stats = FrameFieldStats::default();
    let mb_width = params.mb_width();
    let mb_height = params.mb_height();
    for mb_row in 0..mb_height {
        write_slice_header(bw, mb_row as u32, quantiser_scale_code);
        let mut pred = IntraDcPred::reset(params.intra_dc_precision);
        for mb_col in 0..mb_width {
            // macroblock_address_increment = 1; macroblock_type = Intra
            // (Table B-2 `1`).
            bw.write_bit(true);
            bw.write_bit(true);
            // §6.2.5.1: dct_type present (frame picture,
            // frame_pred_frame_dct == 0, macroblock_intra).
            let field_dct = intra_luma_ac_bits(
                frame,
                mb_col,
                mb_row,
                qscale,
                dc_mult,
                true,
                params.chroma_format,
            ) < intra_luma_ac_bits(
                frame,
                mb_col,
                mb_row,
                qscale,
                dc_mult,
                false,
                params.chroma_format,
            );
            bw.write_bit(field_dct);
            encode_ff_intra_mb(
                bw,
                frame,
                &mut recon,
                mb_col,
                mb_row,
                qscale,
                dc_mult,
                &mut pred,
                field_dct,
                params.chroma_format,
            );
            stats.intra += 1;
            if field_dct {
                stats.field_dct += 1;
            }
        }
        bw.align_to_byte_zero();
    }
    Ok((recon, stats))
}

/// Encode one **P frame picture** with `frame_pred_frame_dct = 0`:
/// per macroblock the encoder scores Table 6-17 **Frame-based**
/// prediction (one frame vector), **Field-based** prediction (two
/// field vectors, each with its own reference-field parity), and —
/// when `allow_dual_prime` asserts the §7.6.3.6 no-B constraint —
/// **Dual-prime** (one vector + `dmvector`, four averaged field
/// predictions), keeps the cheapest (bit-cost-biased luma SAD), picks
/// `dct_type` by exact luma bit cost, and falls back to intra for
/// content no prediction captures.
///
/// Reconstruction runs through the decode-side §7.6 drivers, so the
/// returned frame is the decoder's exact reconstruction.
///
/// # Errors
/// [`Error::InvalidBitstream`] for geometry / format violations;
/// propagates prediction errors.
pub fn encode_ff_p_picture(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    reference: &FrameBuffer,
    params: &IntraPictureParams,
    temporal_reference: u16,
    quantiser_scale_code: u8,
    forward_f_code: u8,
    allow_dual_prime: bool,
) -> Result<(FrameBuffer, FrameFieldStats)> {
    check_ff_params(params)?;
    if current.width != params.width || current.height != params.height {
        return Err(Error::InvalidBitstream(
            "encode_ff_p_picture: frame dimensions do not match params",
        ));
    }
    let qscale = quantiser_scale(quantiser_scale_code, params.q_scale_type)?;
    let dc_mult = intra_dc_mult(params.intra_dc_precision)?;
    let search_range = max_search_range(forward_f_code).min(16);

    write_ff_picture_headers(
        bw,
        params,
        PictureCodingType::Predictive,
        temporal_reference,
        forward_f_code,
        forward_f_code,
    );

    let mut recon = params.new_frame_buffer();
    let mut stats = FrameFieldStats::default();
    let mb_width = params.mb_width();
    let mb_height = params.mb_height();

    for mb_row in 0..mb_height {
        write_slice_header(bw, mb_row as u32, quantiser_scale_code);
        // §7.6.3.4: the predictor bank resets at slice start.
        let mut pmv = PmvMirror::new();
        let mut intra_pred = IntraDcPred::reset(params.intra_dc_precision);
        let slice_first_addr = (mb_row * mb_width) as i32;
        let mut past_intra_address = slice_first_addr - 2;

        for mb_col in 0..mb_width {
            let mb_address = slice_first_addr + mb_col as i32;

            // ---- Mode search ----
            let frame_search =
                estimate_forward_mv(current, reference, mb_col, mb_row, search_range);
            let top_search = estimate_field_in_frame_mv(
                current,
                reference,
                FieldParity::Top,
                mb_col,
                mb_row,
                search_range,
            );
            let bottom_search = estimate_field_in_frame_mv(
                current,
                reference,
                FieldParity::Bottom,
                mb_col,
                mb_row,
                search_range,
            );
            let field_sad = top_search.sad.saturating_add(bottom_search.sad);

            let mut best_mode = PMode::Frame(frame_search.vector);
            let mut best_sad = frame_search.sad;
            let mut best_score = frame_search.sad;
            if field_sad.saturating_add(FIELD_MC_BIAS) < best_score {
                best_mode = PMode::Field(
                    FieldVector::new(top_search.vector, top_search.parity),
                    FieldVector::new(bottom_search.vector, bottom_search.parity),
                );
                best_sad = field_sad;
                best_score = field_sad.saturating_add(FIELD_MC_BIAS);
            }
            if allow_dual_prime {
                let (base_mv, _) =
                    estimate_dual_prime_base(current, reference, mb_col, mb_row, search_range);
                let mut dp_best: Option<(u32, (i8, i8), FrameDualPrimeMotion)> = None;
                for dmv_v in [-1i8, 0, 1] {
                    for dmv_h in [-1i8, 0, 1] {
                        let Ok(derived) = derive_all(
                            DualPrimePicture::Frame {
                                top_field_first: true,
                            },
                            base_mv.horizontal,
                            base_mv.vertical,
                            i32::from(dmv_h),
                            i32::from(dmv_v),
                        ) else {
                            continue;
                        };
                        let (Some(top_opp), Some(bottom_opp)) = (derived.first(), derived.get(1))
                        else {
                            continue;
                        };
                        let motion = FrameDualPrimeMotion::new(
                            base_mv,
                            MotionVectorPel::new(top_opp.horiz, top_opp.vert),
                            MotionVectorPel::new(bottom_opp.horiz, bottom_opp.vert),
                        );
                        if !dual_prime_spans_legal(reference, mb_col, mb_row, &motion) {
                            continue;
                        }
                        let Ok((luma, _, _)) = predict_frame_dual_prime_macroblock_planes(
                            &recon, reference, mb_col, mb_row, motion,
                        ) else {
                            continue;
                        };
                        let mut sad = 0u32;
                        let w = current.y.width();
                        let h = current.y.height();
                        for r in 0..16 {
                            let sy = (mb_row * 16 + r).min(h.saturating_sub(1));
                            for c in 0..16 {
                                let sx = (mb_col * 16 + c).min(w.saturating_sub(1));
                                let cur = i32::from(current.y.get(sx, sy).unwrap_or(0));
                                sad += (cur - i32::from(luma[r * 16 + c])).unsigned_abs();
                            }
                        }
                        if dp_best.map(|b| sad < b.0).unwrap_or(true) {
                            dp_best = Some((sad, (dmv_h, dmv_v), motion));
                        }
                    }
                }
                if let Some((sad, dmv, motion)) = dp_best {
                    if sad.saturating_add(DUAL_PRIME_BIAS) < best_score {
                        best_mode = PMode::DualPrime(base_mv, dmv, motion);
                        best_sad = sad;
                    }
                }
            }

            // ---- Intra fallback ----
            let intra_cost = intra_activity(current, mb_col, mb_row);
            if best_sad > intra_cost.saturating_mul(2).saturating_add(512) {
                if mb_address - past_intra_address > 1 {
                    intra_pred = IntraDcPred::reset(params.intra_dc_precision);
                }
                // macroblock_address_increment = 1; Intra (Table B-3
                // `00011`); dct_type present (intra).
                bw.write_bit(true);
                bw.write_u32(0b0001_1, 5);
                let field_dct = intra_luma_ac_bits(
                    current,
                    mb_col,
                    mb_row,
                    qscale,
                    dc_mult,
                    true,
                    params.chroma_format,
                ) < intra_luma_ac_bits(
                    current,
                    mb_col,
                    mb_row,
                    qscale,
                    dc_mult,
                    false,
                    params.chroma_format,
                );
                bw.write_bit(field_dct);
                encode_ff_intra_mb(
                    bw,
                    current,
                    &mut recon,
                    mb_col,
                    mb_row,
                    qscale,
                    dc_mult,
                    &mut intra_pred,
                    field_dct,
                    params.chroma_format,
                );
                // §7.6.3.4 / Table 7-10 ◊ row: intra without concealment
                // vectors resets the predictor bank.
                pmv.reset();
                past_intra_address = mb_address;
                stats.intra += 1;
                if field_dct {
                    stats.field_dct += 1;
                }
                continue;
            }

            // ---- Prediction planes (the exact §7.6 arithmetic) ----
            let (luma_pred, cb_pred, cr_pred) = match &best_mode {
                PMode::Frame(mv) => predict_frame_macroblock_planes(
                    &recon,
                    ReferenceFrames::forward_only(reference),
                    mb_col,
                    mb_row,
                    FrameMotion::forward(*mv),
                )
                .map_err(crate::Error::from)?,
                PMode::Field(top, bottom) => predict_field_based_macroblock_planes(
                    &recon,
                    ReferenceFrames::forward_only(reference),
                    mb_col,
                    mb_row,
                    FieldBasedMotion::forward(*top, *bottom),
                )
                .map_err(crate::Error::from)?,
                PMode::DualPrime(_, _, motion) => predict_frame_dual_prime_macroblock_planes(
                    &recon, reference, mb_col, mb_row, *motion,
                )
                .map_err(crate::Error::from)?,
            };

            // ---- dct_type + residual quantisation ----
            let (field_dct, blocks, cbp) = choose_inter_dct(
                current,
                &luma_pred,
                &cb_pred,
                &cr_pred,
                mb_col,
                mb_row,
                qscale,
                params.chroma_format,
            );
            // §6.2.5.1: dct_type is only transmitted when the macroblock
            // is coded; an uncoded macroblock defaults to frame DCT.
            let effective_field_dct = field_dct && cbp != 0;

            // ---- Macroblock layer ----
            bw.write_bit(true); // macroblock_address_increment = 1
            if cbp != 0 {
                bw.write_bit(true); // Table B-3 "MC, Coded" `1`
            } else {
                bw.write_u32(0b001, 3); // "MC, Not Coded"
            }
            // frame_motion_type (Table 6-17).
            match &best_mode {
                PMode::Frame(_) => bw.write_u32(0b10, 2),
                PMode::Field(_, _) => bw.write_u32(0b01, 2),
                PMode::DualPrime(_, _, _) => bw.write_u32(0b11, 2),
            }
            if cbp != 0 {
                bw.write_bit(effective_field_dct);
            }
            match &best_mode {
                PMode::Frame(mv) => {
                    emit_frame_vector(bw, &mut pmv, 0, *mv, forward_f_code)?;
                    // Table 7-10 Frame-based fwd row: PMV[1][0] = PMV[0][0].
                    pmv.copy_r0_to_r1(0);
                    stats.frame_mc += 1;
                }
                PMode::Field(top, bottom) => {
                    emit_field_vector(bw, &mut pmv, 0, 0, *top, forward_f_code)?;
                    emit_field_vector(bw, &mut pmv, 1, 0, *bottom, forward_f_code)?;
                    // Table 7-10 Field-based row: "(none)".
                    stats.field_mc += 1;
                }
                PMode::DualPrime(mv, dmv, _) => {
                    emit_dual_prime_vector(bw, &mut pmv, *mv, *dmv, forward_f_code)?;
                    stats.dual_prime += 1;
                }
            }
            if cbp != 0 {
                encode_cbp420(bw, cbp);
                for b in &blocks {
                    if let Some(qf) = b.qf_ref() {
                        write_inter_block_coeffs(bw, qf);
                    }
                }
            }
            if effective_field_dct {
                stats.field_dct += 1;
            }

            // ---- Decoder-exact reconstruction ----
            let residuals: Vec<ResidualBlock<'_>> = blocks
                .iter()
                .enumerate()
                .filter(|(_, b)| b.is_coded())
                .map(|(i, b)| ResidualBlock {
                    block_index: i as u8,
                    f_pel: b.f_pel_ref(),
                })
                .collect();
            match &best_mode {
                PMode::Frame(mv) => {
                    reconstruct_inter_macroblock(
                        &mut recon,
                        ReferenceFrames::forward_only(reference),
                        mb_col,
                        mb_row,
                        effective_field_dct,
                        FrameMotion::forward(*mv),
                        &residuals,
                    )
                    .map_err(crate::Error::from)?;
                }
                PMode::Field(top, bottom) => {
                    reconstruct_field_based_macroblock(
                        &mut recon,
                        ReferenceFrames::forward_only(reference),
                        mb_col,
                        mb_row,
                        effective_field_dct,
                        FieldBasedMotion::forward(*top, *bottom),
                        &residuals,
                    )
                    .map_err(crate::Error::from)?;
                }
                PMode::DualPrime(_, _, motion) => {
                    reconstruct_frame_dual_prime_macroblock(
                        &mut recon,
                        reference,
                        mb_col,
                        mb_row,
                        effective_field_dct,
                        *motion,
                        &residuals,
                    )
                    .map_err(crate::Error::from)?;
                }
            }
        }
        bw.align_to_byte_zero();
    }
    Ok((recon, stats))
}

/// One candidate B-macroblock prediction: its Table 6-17 motion type,
/// Table B-4 direction, and the per-direction vectors.
struct BCandidate {
    dir: BDirection,
    /// `None` = Frame-based (one frame vector per direction);
    /// `Some` = Field-based (a `(top, bottom)` pair per direction).
    frame_fwd: Option<MotionVectorPel>,
    frame_bwd: Option<MotionVectorPel>,
    field_fwd: Option<(FieldVector, FieldVector)>,
    field_bwd: Option<(FieldVector, FieldVector)>,
    is_field: bool,
}

/// Encode one **B frame picture** with `frame_pred_frame_dct = 0`:
/// per macroblock the encoder scores forward / backward / interpolated
/// prediction in both the Table 6-17 **Frame-based** and
/// **Field-based** forms (six candidates, bit-cost-biased luma SAD),
/// emits the Table B-4 mode + `frame_motion_type` + `dct_type` (exact
/// luma bit cost) + vectors (forward before backward; two per
/// direction when field-based), and reconstructs through the
/// decode-side §7.6 drivers.
///
/// # Errors
/// [`Error::InvalidBitstream`] for geometry / format violations;
/// propagates prediction errors.
pub fn encode_ff_b_picture(
    bw: &mut BitWriter,
    current: &FrameBuffer,
    forward: &FrameBuffer,
    backward: &FrameBuffer,
    params: &IntraPictureParams,
    temporal_reference: u16,
    quantiser_scale_code: u8,
    forward_f_code: u8,
    backward_f_code: u8,
) -> Result<(FrameBuffer, FrameFieldStats)> {
    check_ff_params(params)?;
    if current.width != params.width || current.height != params.height {
        return Err(Error::InvalidBitstream(
            "encode_ff_b_picture: frame dimensions do not match params",
        ));
    }
    let qscale = quantiser_scale(quantiser_scale_code, params.q_scale_type)?;
    let fwd_range = max_search_range(forward_f_code).min(16);
    let bwd_range = max_search_range(backward_f_code).min(16);

    write_ff_picture_headers(
        bw,
        params,
        PictureCodingType::Bidirectional,
        temporal_reference,
        forward_f_code,
        backward_f_code,
    );

    let mut recon = params.new_frame_buffer();
    let mut stats = FrameFieldStats::default();
    let mb_width = params.mb_width();
    let mb_height = params.mb_height();

    for mb_row in 0..mb_height {
        write_slice_header(bw, mb_row as u32, quantiser_scale_code);
        let mut pmv = PmvMirror::new();

        for mb_col in 0..mb_width {
            // ---- Per-direction searches (frame + field forms) ----
            let f_frame = estimate_forward_mv(current, forward, mb_col, mb_row, fwd_range);
            let b_frame = estimate_forward_mv(current, backward, mb_col, mb_row, bwd_range);
            let f_top = estimate_field_in_frame_mv(
                current,
                forward,
                FieldParity::Top,
                mb_col,
                mb_row,
                fwd_range,
            );
            let f_bottom = estimate_field_in_frame_mv(
                current,
                forward,
                FieldParity::Bottom,
                mb_col,
                mb_row,
                fwd_range,
            );
            let b_top = estimate_field_in_frame_mv(
                current,
                backward,
                FieldParity::Top,
                mb_col,
                mb_row,
                bwd_range,
            );
            let b_bottom = estimate_field_in_frame_mv(
                current,
                backward,
                FieldParity::Bottom,
                mb_col,
                mb_row,
                bwd_range,
            );
            let field_fwd_pair = (
                FieldVector::new(f_top.vector, f_top.parity),
                FieldVector::new(f_bottom.vector, f_bottom.parity),
            );
            let field_bwd_pair = (
                FieldVector::new(b_top.vector, b_top.parity),
                FieldVector::new(b_bottom.vector, b_bottom.parity),
            );

            let refs_fwd = ReferenceFrames::forward_only(forward);
            let refs_bwd = ReferenceFrames {
                forward: None,
                backward: Some(backward),
            };
            let refs_both = ReferenceFrames::bidirectional(forward, backward);

            // ---- Candidate predictions + SAD scoring ----
            let candidates = [
                BCandidate {
                    dir: BDirection::Forward,
                    frame_fwd: Some(f_frame.vector),
                    frame_bwd: None,
                    field_fwd: None,
                    field_bwd: None,
                    is_field: false,
                },
                BCandidate {
                    dir: BDirection::Backward,
                    frame_fwd: None,
                    frame_bwd: Some(b_frame.vector),
                    field_fwd: None,
                    field_bwd: None,
                    is_field: false,
                },
                BCandidate {
                    dir: BDirection::Interpolated,
                    frame_fwd: Some(f_frame.vector),
                    frame_bwd: Some(b_frame.vector),
                    field_fwd: None,
                    field_bwd: None,
                    is_field: false,
                },
                BCandidate {
                    dir: BDirection::Forward,
                    frame_fwd: None,
                    frame_bwd: None,
                    field_fwd: Some(field_fwd_pair),
                    field_bwd: None,
                    is_field: true,
                },
                BCandidate {
                    dir: BDirection::Backward,
                    frame_fwd: None,
                    frame_bwd: None,
                    field_fwd: None,
                    field_bwd: Some(field_bwd_pair),
                    is_field: true,
                },
                BCandidate {
                    dir: BDirection::Interpolated,
                    frame_fwd: None,
                    frame_bwd: None,
                    field_fwd: Some(field_fwd_pair),
                    field_bwd: Some(field_bwd_pair),
                    is_field: true,
                },
            ];

            let predict = |cand: &BCandidate| -> Result<(Vec<u8>, Vec<u8>, Vec<u8>)> {
                let refs = match cand.dir {
                    BDirection::Forward => refs_fwd,
                    BDirection::Backward => refs_bwd,
                    BDirection::Interpolated => refs_both,
                };
                if cand.is_field {
                    let motion = FieldBasedMotion {
                        forward: cand.field_fwd,
                        backward: cand.field_bwd,
                    };
                    predict_field_based_macroblock_planes(&recon, refs, mb_col, mb_row, motion)
                        .map_err(crate::Error::from)
                } else {
                    let motion = FrameMotion {
                        forward: cand.frame_fwd,
                        backward: cand.frame_bwd,
                    };
                    predict_frame_macroblock_planes(&recon, refs, mb_col, mb_row, motion)
                        .map_err(crate::Error::from)
                }
            };
            let luma_sad = |pred: &[u8]| -> u32 {
                let w = current.y.width();
                let h = current.y.height();
                let mut sad = 0u32;
                for r in 0..16 {
                    let sy = (mb_row * 16 + r).min(h.saturating_sub(1));
                    for c in 0..16 {
                        let sx = (mb_col * 16 + c).min(w.saturating_sub(1));
                        let cur = i32::from(current.y.get(sx, sy).unwrap_or(0));
                        sad += (cur - i32::from(pred[r * 16 + c])).unsigned_abs();
                    }
                }
                sad
            };

            let mut best_idx = 0usize;
            let mut best_planes: Option<(Vec<u8>, Vec<u8>, Vec<u8>)> = None;
            let mut best_score = u32::MAX;
            for (idx, cand) in candidates.iter().enumerate() {
                let planes = predict(cand)?;
                let sad = luma_sad(&planes.0);
                let score = if cand.is_field {
                    sad.saturating_add(FIELD_MC_BIAS)
                } else {
                    sad
                };
                if score < best_score {
                    best_score = score;
                    best_idx = idx;
                    best_planes = Some(planes);
                }
            }
            let chosen = &candidates[best_idx];
            let (luma_pred, cb_pred, cr_pred) = best_planes.expect("six candidates were scored");

            // ---- dct_type + residual quantisation ----
            let (field_dct, blocks, cbp) = choose_inter_dct(
                current,
                &luma_pred,
                &cb_pred,
                &cr_pred,
                mb_col,
                mb_row,
                qscale,
                params.chroma_format,
            );
            let effective_field_dct = field_dct && cbp != 0;

            // ---- Macroblock layer ----
            bw.write_bit(true); // macroblock_address_increment = 1
            write_b_macroblock_type(bw, chosen.dir, cbp != 0);
            // frame_motion_type (Table 6-17).
            bw.write_u32(if chosen.is_field { 0b01 } else { 0b10 }, 2);
            if cbp != 0 {
                bw.write_bit(effective_field_dct);
            }
            // Forward motion_vectors(0) precede backward
            // motion_vectors(1) (§6.2.5.2).
            if chosen.is_field {
                if let Some((top, bottom)) = &chosen.field_fwd {
                    emit_field_vector(bw, &mut pmv, 0, 0, *top, forward_f_code)?;
                    emit_field_vector(bw, &mut pmv, 1, 0, *bottom, forward_f_code)?;
                }
                if let Some((top, bottom)) = &chosen.field_bwd {
                    emit_field_vector(bw, &mut pmv, 0, 1, *top, backward_f_code)?;
                    emit_field_vector(bw, &mut pmv, 1, 1, *bottom, backward_f_code)?;
                }
                // Table 7-10 Field-based row: "(none)".
                stats.field_mc += 1;
            } else {
                if let Some(mv) = &chosen.frame_fwd {
                    emit_frame_vector(bw, &mut pmv, 0, *mv, forward_f_code)?;
                    pmv.copy_r0_to_r1(0);
                }
                if let Some(mv) = &chosen.frame_bwd {
                    emit_frame_vector(bw, &mut pmv, 1, *mv, backward_f_code)?;
                    pmv.copy_r0_to_r1(1);
                }
                stats.frame_mc += 1;
            }
            if cbp != 0 {
                encode_cbp420(bw, cbp);
                for b in &blocks {
                    if let Some(qf) = b.qf_ref() {
                        write_inter_block_coeffs(bw, qf);
                    }
                }
            }
            if effective_field_dct {
                stats.field_dct += 1;
            }

            // ---- Decoder-exact reconstruction ----
            let residuals: Vec<ResidualBlock<'_>> = blocks
                .iter()
                .enumerate()
                .filter(|(_, b)| b.is_coded())
                .map(|(i, b)| ResidualBlock {
                    block_index: i as u8,
                    f_pel: b.f_pel_ref(),
                })
                .collect();
            let refs = match chosen.dir {
                BDirection::Forward => refs_fwd,
                BDirection::Backward => refs_bwd,
                BDirection::Interpolated => refs_both,
            };
            if chosen.is_field {
                reconstruct_field_based_macroblock(
                    &mut recon,
                    refs,
                    mb_col,
                    mb_row,
                    effective_field_dct,
                    FieldBasedMotion {
                        forward: chosen.field_fwd,
                        backward: chosen.field_bwd,
                    },
                    &residuals,
                )
                .map_err(crate::Error::from)?;
            } else {
                reconstruct_inter_macroblock(
                    &mut recon,
                    refs,
                    mb_col,
                    mb_row,
                    effective_field_dct,
                    FrameMotion {
                        forward: chosen.frame_fwd,
                        backward: chosen.frame_bwd,
                    },
                    &residuals,
                )
                .map_err(crate::Error::from)?;
            }
        }
        bw.align_to_byte_zero();
    }
    Ok((recon, stats))
}

// =============================================================
// Whole-sequence assembler
// =============================================================

/// Encode a whole **display-order** frame sequence as an interlaced
/// MPEG-2 elementary stream of `frame_pred_frame_dct = 0` **frame
/// pictures** with per-macroblock frame/field prediction and DCT
/// selection, mirroring the GOP structure of
/// [`crate::encode_display_order_gop_sequence`] (one I per GOP, closed
/// GOPs, per-GOP `temporal_reference` reset, anchors every
/// `b_between + 1` display frames).
///
/// The sequence declares `progressive_sequence = 0` and every picture
/// `progressive_frame = 0` (§6.3.10), so the coded grid is the §6.3.3
/// interlaced `2 * Ceil(height / 32)` macroblock rows.
///
/// `allow_dual_prime` enables the Table 6-17 `Dual-prime` mode in P
/// pictures; §7.6.3.6 forbids it when B-pictures separate the
/// predicted and reference frames, so it is rejected unless
/// `b_between == 0`.
///
/// Returns `(stream, stats)` — the accumulated per-macroblock mode
/// counters over every coded picture.
///
/// # Errors
/// [`Error::InvalidBitstream`] for an empty `frames`, geometry /
/// format violations, `anchors_per_gop == 0`, or `allow_dual_prime`
/// with `b_between > 0`; propagates encode errors.
pub fn encode_ff_display_order_gop_sequence(
    frames: &[FrameBuffer],
    b_between: usize,
    anchors_per_gop: usize,
    params: &IntraPictureParams,
    quantiser_scale_code: u8,
    forward_f_code: u8,
    backward_f_code: u8,
    allow_dual_prime: bool,
) -> Result<(Vec<u8>, FrameFieldStats)> {
    check_ff_params(params)?;
    if frames.is_empty() {
        return Err(Error::InvalidBitstream(
            "encode_ff_display_order_gop_sequence: no frames to encode",
        ));
    }
    if anchors_per_gop == 0 {
        return Err(Error::InvalidBitstream(
            "encode_ff_display_order_gop_sequence: anchors_per_gop must be >= 1",
        ));
    }
    if allow_dual_prime && b_between > 0 {
        return Err(Error::InvalidBitstream(
            "encode_ff_display_order_gop_sequence: dual-prime requires no B-pictures \
             between the predicted and reference frames (§7.6.3.6)",
        ));
    }
    for f in frames {
        if f.width != params.width || f.height != params.height {
            return Err(Error::InvalidBitstream(
                "encode_ff_display_order_gop_sequence: frame geometry mismatch",
            ));
        }
    }

    let sequence_params = SequenceHeaderParams {
        horizontal_size: params.width as u16,
        vertical_size: params.height as u16,
        ..Default::default()
    };
    let mut bw = BitWriter::new();
    write_sequence_header(&mut bw, &sequence_params);
    // §6.3.5: interlaced sequence (progressive_sequence = 0).
    write_sequence_extension(&mut bw, params.chroma_format, false);

    let mut stats = FrameFieldStats::default();
    let step = b_between + 1;
    let mut gop_start = 0usize;
    while gop_start < frames.len() {
        let gop_end = (gop_start + anchors_per_gop * step).min(frames.len() - 1);

        write_gop_header(
            &mut bw,
            &Mpeg2Gop {
                time_code: TimeCode::from_display_index(
                    gop_start as u64,
                    sequence_params.frame_rate_code,
                )?,
                closed_gop: true,
                broken_link: false,
            },
        );

        let (mut forward_ref, i_stats) =
            encode_ff_intra_picture(&mut bw, &frames[gop_start], params, 0, quantiser_scale_code)?;
        stats.add(&i_stats);

        let mut prev_anchor = gop_start;
        while prev_anchor < gop_end {
            let next_anchor = (prev_anchor + step).min(gop_end);
            let (backward_ref, p_stats) = encode_ff_p_picture(
                &mut bw,
                &frames[next_anchor],
                &forward_ref,
                params,
                (next_anchor - gop_start) as u16,
                quantiser_scale_code,
                forward_f_code,
                allow_dual_prime,
            )?;
            stats.add(&p_stats);
            for b in (prev_anchor + 1)..next_anchor {
                let (_, b_stats) = encode_ff_b_picture(
                    &mut bw,
                    &frames[b],
                    &forward_ref,
                    &backward_ref,
                    params,
                    (b - gop_start) as u16,
                    quantiser_scale_code,
                    forward_f_code,
                    backward_f_code,
                )?;
                stats.add(&b_stats);
            }
            forward_ref = backward_ref;
            prev_anchor = next_anchor;
        }

        gop_start = gop_end + 1;
    }

    let mut stream = bw.finish();
    stream.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
    Ok((stream, stats))
}
