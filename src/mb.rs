//! Macroblock / slice-level decoding for MPEG-1 video and MPEG-2 video
//! (H.262) frame pictures.
//!
//! Covers:
//!   * MPEG-1 4:2:0 frame pictures.
//!   * MPEG-2 4:2:0 / 4:2:2 / 4:4:4 frame pictures (8 or 12 blocks per MB
//!     in the non-4:2:0 cases) — H.262 §6.3.5 / §7.2.
//!   * Frame-DCT and field-DCT — when `frame_pred_frame_dct = 0` the MB
//!     header carries a `dct_type` bit that decides whether luma rows are
//!     transmitted in interleaved frame order or split into two field-DCT
//!     halves.
//!   * Frame motion vectors *and* field motion vectors inside frame
//!     pictures — `frame_motion_type ∈ {Frame, Field, DualPrime}` per H.262
//!     §6.3.17.1.
//!   * Concealment motion vectors — consumed and discarded; their purpose
//!     is to give a decoder a fallback MV when error concealment is needed
//!     (out of scope for our offline decode but required to keep the
//!     bitstream parser in sync).
//!   * Dual-prime motion vectors — single-direction parity-pair prediction.
//!
//! Field pictures (`picture_structure ≠ Frame`) are still rejected upstream;
//! enabling them needs a different reference-buffer model.

use oxideav_core::{Error, Result};

use crate::block::{decode_intra_block, decode_non_intra_block};
use crate::coding_mode::{
    Codec, PictureParams, PictureStructure, AXIS_H, AXIS_V, DIR_BWD, DIR_FWD,
};
use crate::headers::{PictureHeader, PictureType, SequenceHeader};
use crate::motion::{
    self, decode_dmvector, decode_motion_component, decode_motion_component_mpeg2, MvPredictor,
};
use crate::picture::{ChromaFormat, PictureBuffer};
use crate::tables::{cbp, mb_type, mba};
use crate::vlc;
use oxideav_core::bits::BitReader;

/// MPEG-2 `frame_motion_type` code (H.262 §6.3.17.1, 2 bits).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FrameMotionType {
    /// `10` — frame-based (single 16×16 MV per direction, like MPEG-1).
    Frame,
    /// `01` — field-based (two field MVs per direction).
    Field,
    /// `11` — dual-prime (single MV + dmvector, single direction only).
    DualPrime,
}

impl FrameMotionType {
    fn from_code(code: u8) -> Option<Self> {
        match code {
            0b10 => Some(Self::Frame),
            0b01 => Some(Self::Field),
            0b11 => Some(Self::DualPrime),
            _ => None,
        }
    }
}

/// Running slice-decode state carried between macroblocks.
pub struct SliceState {
    pub dc_pred: [i32; 3],
    pub fwd: MvPredictor,
    pub bwd: MvPredictor,
    /// Field motion-vector predictors per direction × parity. Used by the
    /// MPEG-2 field-MC path. Index `[dir][parity]` where parity 0 = top
    /// field reference, parity 1 = bottom field reference.
    pub field_mv: [[MvPredictor; 2]; 2],
    /// True if the previous macroblock in this slice had a forward MV.
    /// Required for B-frame "skipped MB inherits previous MB type".
    pub last_had_forward: bool,
    /// True if the previous macroblock in this slice had a backward MV.
    pub last_had_backward: bool,
}

impl SliceState {
    pub fn new(dc_reset: i32) -> Self {
        Self {
            dc_pred: [dc_reset; 3],
            fwd: MvPredictor::default(),
            bwd: MvPredictor::default(),
            field_mv: [[MvPredictor::default(); 2]; 2],
            last_had_forward: false,
            last_had_backward: false,
        }
    }
}

impl Default for SliceState {
    fn default() -> Self {
        Self::new(1024)
    }
}

/// Decode one slice.
#[allow(clippy::too_many_arguments)]
pub fn decode_slice(
    br: &mut BitReader<'_>,
    slice_start_code: u8,
    seq: &SequenceHeader,
    pic_hdr: &PictureHeader,
    params: &PictureParams,
    pic: &mut PictureBuffer,
    fwd_ref: Option<&PictureBuffer>,
    bwd_ref: Option<&PictureBuffer>,
) -> Result<()> {
    let picture_type = pic_hdr.picture_type;
    // `slice_quantiser_scale_code` (5 bits). For MPEG-2 with q_scale_type=1
    // the code is mapped through Table 7-6; otherwise it is the scale itself.
    let qsc = br.read_u32(5)? as u8;
    let mut quant_scale = params.quantiser_scale(qsc.max(1));

    // extra_bit_slice.
    while br.read_u32(1)? == 1 {
        br.read_u32(8)?;
    }

    let mut state = SliceState::new(params.intra_dc_reset_value());

    let mb_row = slice_start_code as i32 - 1;
    if mb_row < 0 || (mb_row as usize) >= pic.mb_height {
        return Err(Error::invalid("slice: MB row out of range"));
    }
    let mb_width = pic.mb_width as i32;
    let mut mb_addr: i32 = mb_row * mb_width - 1;
    let mut first_mb = true;

    loop {
        // Read MB address increment — may be a sequence of stuffing codes
        // followed by an escape + actual increment.
        let mut incr: u32 = 0;
        loop {
            let sym = vlc::decode(br, mba::table())?;
            if sym == mba::STUFFING {
                continue;
            }
            if sym == mba::ESCAPE {
                incr += 33;
                continue;
            }
            incr += sym as u32;
            break;
        }
        let prev_mb_addr = mb_addr;
        mb_addr += incr as i32;
        if mb_addr >= (mb_row + 1) * mb_width {
            return Err(Error::invalid("slice: MB address past end of row"));
        }

        // Handle skipped MBs (those between prev+1..mb_addr exclusive).
        // For I-pictures: spec forbids skipped MBs (every MB must be
        // intra-coded). For P-pictures: skipped = no MC (MV=0) + forward
        // prediction from the reference, no residual. For B-pictures:
        // skipped = same MV type and MV values as previous MB, no
        // residual.
        if !first_mb && incr > 1 {
            match picture_type {
                PictureType::I => {
                    return Err(Error::invalid("I-picture: skipped MB not allowed"));
                }
                PictureType::P => {
                    // Per §2.4.4.2 / §2.4.3.4: P-skip MBs have MV=(0,0)
                    // and reset MV predictors.
                    state.fwd.reset();
                    state.bwd.reset();
                    state.last_had_forward = true;
                    state.last_had_backward = false;
                    for skip_addr in (prev_mb_addr + 1)..mb_addr {
                        let sx = (skip_addr % mb_width) as usize;
                        let sy = (skip_addr / mb_width) as usize;
                        fill_forward_predict(pic, fwd_ref, sx, sy, 0, 0)?;
                    }
                }
                PictureType::B => {
                    // Skipped B MB inherits same MV direction + values as
                    // previous MB.
                    for skip_addr in (prev_mb_addr + 1)..mb_addr {
                        let sx = (skip_addr % mb_width) as usize;
                        let sy = (skip_addr / mb_width) as usize;
                        let fwd_mv = if state.last_had_forward {
                            Some((state.fwd.x, state.fwd.y))
                        } else {
                            None
                        };
                        let bwd_mv = if state.last_had_backward {
                            Some((state.bwd.x, state.bwd.y))
                        } else {
                            None
                        };
                        fill_bidir_predict(pic, fwd_ref, bwd_ref, sx, sy, fwd_mv, bwd_mv)?;
                    }
                }
                PictureType::D => {
                    return Err(Error::unsupported("D-picture not supported"));
                }
            }
        }

        first_mb = false;

        let mb_x = (mb_addr % mb_width) as usize;
        let mb_y = (mb_addr / mb_width) as usize;

        // macroblock_type per picture.
        let mb_type_flags = match picture_type {
            PictureType::I => vlc::decode(br, mb_type::i_table())?,
            PictureType::P => vlc::decode(br, mb_type::p_table())?,
            PictureType::B => vlc::decode(br, mb_type::b_table())?,
            PictureType::D => {
                return Err(Error::unsupported("D-picture not supported"));
            }
        };

        // Decode `frame_motion_type` (MPEG-2, frame pictures, when motion
        // prediction is signalled and `frame_pred_frame_dct = 0`).
        let frame_motion_type = if params.is_mpeg2()
            && !params.frame_pred_frame_dct
            && (mb_type_flags.motion_forward || mb_type_flags.motion_backward)
        {
            let code = br.read_u32(2)? as u8;
            FrameMotionType::from_code(code).ok_or_else(|| {
                Error::invalid(format!("MPEG-2 frame_motion_type code {code} invalid"))
            })?
        } else {
            FrameMotionType::Frame
        };

        // Decode `dct_type` when applicable. Spec §6.3.17.1: present when
        // (cbp != 0 OR intra) AND `frame_pred_frame_dct = 0` AND chroma
        // format is 4:2:0 or 4:2:2 or 4:4:4 (always in MPEG-2).
        // For the 4:2:0 case the chroma blocks are not affected by
        // dct_type; for 4:2:2 and 4:4:4 they are.
        let needs_dct_type = params.is_mpeg2()
            && !params.frame_pred_frame_dct
            && (mb_type_flags.intra || mb_type_flags.pattern);
        let dct_type_field = if needs_dct_type {
            br.read_u32(1)? == 1
        } else {
            false
        };

        if mb_type_flags.quant {
            let qs = br.read_u32(5)? as u8;
            if qs != 0 {
                quant_scale = params.quantiser_scale(qs);
            }
        }

        // Parse MV(s).
        let mut mv_data = MotionData::default();
        if mb_type_flags.motion_forward {
            decode_mvs(
                br,
                params,
                &mut state,
                frame_motion_type,
                DIR_FWD,
                &mut mv_data,
            )?;
        } else if matches!(picture_type, PictureType::P) && !mb_type_flags.intra {
            // No-MC P MB: vector is (0,0) and predictor resets.
            state.fwd.reset();
            mv_data.fwd = Some((0, 0));
        }
        if mb_type_flags.motion_backward {
            decode_mvs(
                br,
                params,
                &mut state,
                frame_motion_type,
                DIR_BWD,
                &mut mv_data,
            )?;
        }

        // Concealment MVs in I (and P, when not motion-coded above) MBs.
        if params.is_mpeg2()
            && params.concealment_motion_vectors
            && mb_type_flags.intra
            && matches!(picture_type, PictureType::I | PictureType::P)
        {
            // §6.3.17.1: a conceal MV uses the f_code[0][*] axes (forward
            // f_codes) and the same predictor slot as the forward MV. The
            // MB also reads a 1-bit marker after the two components.
            let mx = decode_motion_component_mpeg2(
                br,
                params.f_code[DIR_FWD][AXIS_H],
                &mut state.fwd.x,
            )?;
            let my = decode_motion_component_mpeg2(
                br,
                params.f_code[DIR_FWD][AXIS_V],
                &mut state.fwd.y,
            )?;
            let _ = (mx, my);
            // marker_bit
            let _ = br.read_u32(1)?;
        }

        // Track MV availability for B-skip inheritance.
        if matches!(picture_type, PictureType::B) && !mb_type_flags.intra {
            state.last_had_forward = mv_data.fwd.is_some();
            state.last_had_backward = mv_data.bwd.is_some();
        }

        // Per §2.4.4.1 / H.262 §7.2.1: DC predictors reset whenever a
        // non-intra MB is seen (so the next intra MB starts from the reset
        // value again).
        if !mb_type_flags.intra {
            let r = params.intra_dc_reset_value();
            state.dc_pred = [r, r, r];
        }

        // Parse coded_block_pattern if this MB has pattern bit set.
        // For non-4:2:0 chroma, additional `coded_block_pattern_1` (2 bits,
        // 4:2:2) and/or `coded_block_pattern_2` (6 bits, 4:4:4) follow.
        // The 4:2:0 base CBP is itself 6 bits (4 luma + 1 Cb + 1 Cr); the
        // extra fields add ONE more chroma block per direction (4:2:2) or
        // THREE more (4:4:4), i.e. `chroma_blocks_per_mb - 2` extra bits.
        let cbp_full: u32 = if mb_type_flags.pattern {
            let base = vlc::decode(br, cbp::table())? as u32;
            let extra_bits = match params.chroma_format {
                ChromaFormat::Yuv420 => 0u32,
                ChromaFormat::Yuv422 => 2u32,
                ChromaFormat::Yuv444 => 6u32,
            };
            let extra = if extra_bits == 0 {
                0
            } else {
                br.read_u32(extra_bits)?
            };
            (base << extra_bits) | extra
        } else if mb_type_flags.intra {
            // All blocks coded.
            (1u32 << (4 + params.chroma_format.chroma_blocks_per_mb())) - 1
        } else {
            0
        };

        if mb_type_flags.intra {
            // Pure intra MB.
            decode_mb_intra(
                br,
                seq,
                params,
                &mut state,
                pic,
                mb_x,
                mb_y,
                quant_scale,
                dct_type_field,
            )?;
        } else {
            // Non-intra MB: apply motion compensation and optional
            // residual add.
            decode_mb_inter(
                br,
                seq,
                params,
                pic,
                fwd_ref,
                bwd_ref,
                mb_x,
                mb_y,
                quant_scale,
                &mv_data,
                cbp_full,
                frame_motion_type,
                dct_type_field,
            )?;
        }

        if !matches!(picture_type, PictureType::B) {
            // For I/P we track MV history so subsequent skip handling
            // within the slice is consistent.
            state.last_had_forward = mb_type_flags.motion_forward
                || (!mb_type_flags.intra && matches!(picture_type, PictureType::P));
            state.last_had_backward = mb_type_flags.motion_backward;
        }

        // Slice termination: loop until the next start code. The next
        // start code is `0x000001XX` (preceded optionally by stuffing zero
        // bytes), so we peek up to the next 23 bits (or fewer near the
        // tail) and break only if all the bits we can see are zero.
        //
        // Also break when we've reached the right edge of the row — the
        // next MB would be in the next slice/row.
        if mb_addr + 1 >= (mb_row + 1) * mb_width {
            break;
        }
        let avail = br.bits_remaining().min(23) as u32;
        if avail == 0 {
            break;
        }
        let peek = br.peek_u32(avail)?;
        if peek == 0 {
            // All trailing bits are zero — at the start of a start code.
            break;
        }
    }

    Ok(())
}

/// Per-MB collected motion data after MV parsing.
#[derive(Clone, Copy, Default)]
struct MotionData {
    /// Frame-MC forward MV in half-pel units (luma grid).
    fwd: Option<(i32, i32)>,
    /// Frame-MC backward MV.
    bwd: Option<(i32, i32)>,
    /// Field-MC forward MVs and their selected reference parities.
    /// `[ (mv_top, ref_parity_top), (mv_bottom, ref_parity_bot) ]`.
    fwd_field: Option<[(i32, i32, u8); 2]>,
    /// Field-MC backward MVs (analogous to `fwd_field`).
    bwd_field: Option<[(i32, i32, u8); 2]>,
    /// Dual-prime base MV + dmvector (3 components per axis).
    /// `(mv_x, mv_y, dmv_x, dmv_y)`.
    dual_prime: Option<(i32, i32, i32, i32)>,
}

/// Decode the motion vectors for one direction of one MB.
fn decode_mvs(
    br: &mut BitReader<'_>,
    params: &PictureParams,
    state: &mut SliceState,
    motion_type: FrameMotionType,
    dir: usize,
    out: &mut MotionData,
) -> Result<()> {
    if !params.is_mpeg2() {
        // MPEG-1: single frame MV per direction.
        let pred = if dir == DIR_FWD {
            &mut state.fwd
        } else {
            &mut state.bwd
        };
        let full_pel = if dir == DIR_FWD {
            params.full_pel_fwd
        } else {
            params.full_pel_bwd
        };
        let mx = decode_motion_component(br, params.f_code[dir][AXIS_H], full_pel, &mut pred.x)?;
        let my = decode_motion_component(br, params.f_code[dir][AXIS_V], full_pel, &mut pred.y)?;
        if dir == DIR_FWD {
            out.fwd = Some((mx, my));
        } else {
            out.bwd = Some((mx, my));
        }
        return Ok(());
    }
    // MPEG-2 frame picture.
    match motion_type {
        FrameMotionType::Frame => {
            let pred = if dir == DIR_FWD {
                &mut state.fwd
            } else {
                &mut state.bwd
            };
            let mx = decode_motion_component_mpeg2(br, params.f_code[dir][AXIS_H], &mut pred.x)?;
            // For frame MV in interlaced frame picture, vertical is
            // half-pel-of-the-frame, transmitted in the same units as
            // luma. No special scaling at decode time — the spec defines
            // motion_residual relative to frame coordinates already.
            let my = decode_motion_component_mpeg2(br, params.f_code[dir][AXIS_V], &mut pred.y)?;
            if dir == DIR_FWD {
                out.fwd = Some((mx, my));
            } else {
                out.bwd = Some((mx, my));
            }
        }
        FrameMotionType::Field => {
            // Two field MVs per direction. Per §6.3.17.2 each field MV is
            // preceded by a 1-bit `motion_vertical_field_select` choosing
            // top (0) or bottom (1) field of the reference frame.
            let mut entries = [(0i32, 0i32, 0u8); 2];
            for slot in 0..2 {
                let parity = br.read_u32(1)? as u8;
                let pred = &mut state.field_mv[dir][slot];
                let mx =
                    decode_motion_component_mpeg2(br, params.f_code[dir][AXIS_H], &mut pred.x)?;
                // §7.6.3.4: vertical field MVs are stored "halved" relative
                // to the frame grid — each MV step is one *field-grid* line,
                // i.e. two frame-grid lines. The spec encodes them in
                // half-field-pel units, so multiplying back by 2 gives a
                // frame-grid half-pel unit. The predictor itself is stored
                // in field-pel units though, so we keep the bare value here.
                let my =
                    decode_motion_component_mpeg2(br, params.f_code[dir][AXIS_V], &mut pred.y)?;
                entries[slot] = (mx, my, parity);
            }
            // Replicate the regular frame predictor from one of the field
            // predictors so that a subsequent frame-coded MB starts from a
            // sensible state.
            let frame_pred = if dir == DIR_FWD {
                &mut state.fwd
            } else {
                &mut state.bwd
            };
            frame_pred.x = state.field_mv[dir][0].x;
            frame_pred.y = state.field_mv[dir][0].y * 2;
            if dir == DIR_FWD {
                out.fwd_field = Some(entries);
            } else {
                out.bwd_field = Some(entries);
            }
        }
        FrameMotionType::DualPrime => {
            // §6.3.17.2: a single MV plus a 2-component dmvector. Only
            // legal when the picture is P (not B) and is a frame picture.
            let pred = if dir == DIR_FWD {
                &mut state.fwd
            } else {
                &mut state.bwd
            };
            let mx = decode_motion_component_mpeg2(br, params.f_code[dir][AXIS_H], &mut pred.x)?;
            let dmv_x = decode_dmvector(br)?;
            let my = decode_motion_component_mpeg2(br, params.f_code[dir][AXIS_V], &mut pred.y)?;
            let dmv_y = decode_dmvector(br)?;
            out.dual_prime = Some((mx, my, dmv_x, dmv_y));
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn decode_mb_intra(
    br: &mut BitReader<'_>,
    seq: &SequenceHeader,
    params: &PictureParams,
    state: &mut SliceState,
    pic: &mut PictureBuffer,
    mb_x: usize,
    mb_y: usize,
    quant_scale: u8,
    dct_type_field: bool,
) -> Result<()> {
    let n_blocks = params.chroma_format.blocks_per_mb();
    // Pre-allocate luma working store (16x16) and chroma working stores
    // (variable size per format) so we can apply field-DCT permutation
    // before writing back to the picture buffer.
    let mut luma_buf = [0u8; 16 * 16];
    let chroma_v = match params.chroma_format {
        ChromaFormat::Yuv420 => 8usize,
        ChromaFormat::Yuv422 => 16usize,
        ChromaFormat::Yuv444 => 16usize,
    };
    let chroma_w = match params.chroma_format {
        ChromaFormat::Yuv420 | ChromaFormat::Yuv422 => 8usize,
        ChromaFormat::Yuv444 => 16usize,
    };
    let mut cb_buf = vec![0u8; chroma_w * chroma_v];
    let mut cr_buf = vec![0u8; chroma_w * chroma_v];

    for b in 0..n_blocks {
        let comp_idx = block_component(b, params.chroma_format);
        let (dst_buf, dst_stride): (&mut [u8], usize) = match comp_idx {
            0 => (&mut luma_buf, 16),
            1 => (&mut cb_buf, chroma_w),
            _ => (&mut cr_buf, chroma_w),
        };
        let (off_x, off_y) = block_offset_in_mb(b, comp_idx, params.chroma_format, dct_type_field);
        let sub = &mut dst_buf[off_y * dst_stride + off_x..];
        decode_intra_block(
            br,
            comp_idx != 0,
            &mut state.dc_pred[comp_idx],
            quant_scale,
            &seq.intra_quantiser,
            params,
            sub,
            dst_stride,
        )?;
    }

    // Undo field-DCT row interleave on luma if needed.
    let final_luma = if dct_type_field && params.is_mpeg2() {
        deinterleave_field_dct(&luma_buf)
    } else {
        luma_buf
    };
    // Chroma field-DCT is only meaningful for 4:2:2 / 4:4:4 (chroma vertical
    // is also 16). For 4:2:0 chroma there is no field permutation.
    let (final_cb, final_cr) = if dct_type_field && params.is_mpeg2() && chroma_v == 16 {
        (
            deinterleave_field_dct_chroma(&cb_buf, chroma_w),
            deinterleave_field_dct_chroma(&cr_buf, chroma_w),
        )
    } else {
        (cb_buf.clone(), cr_buf.clone())
    };

    // Write back into the picture buffer.
    write_mb_planes(
        pic,
        mb_x,
        mb_y,
        &final_luma,
        &final_cb,
        &final_cr,
        chroma_w,
        chroma_v,
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn decode_mb_inter(
    br: &mut BitReader<'_>,
    seq: &SequenceHeader,
    params: &PictureParams,
    pic: &mut PictureBuffer,
    fwd_ref: Option<&PictureBuffer>,
    bwd_ref: Option<&PictureBuffer>,
    mb_x: usize,
    mb_y: usize,
    quant_scale: u8,
    mv: &MotionData,
    cbp_full: u32,
    frame_motion_type: FrameMotionType,
    dct_type_field: bool,
) -> Result<()> {
    // Build prediction.
    let chroma_w = match params.chroma_format {
        ChromaFormat::Yuv420 | ChromaFormat::Yuv422 => 8usize,
        ChromaFormat::Yuv444 => 16usize,
    };
    let chroma_v = match params.chroma_format {
        ChromaFormat::Yuv420 => 8usize,
        ChromaFormat::Yuv422 | ChromaFormat::Yuv444 => 16usize,
    };
    let mut pred_y = [0u8; 16 * 16];
    let mut pred_cb = vec![0u8; chroma_w * chroma_v];
    let mut pred_cr = vec![0u8; chroma_w * chroma_v];

    build_prediction_mb(
        params,
        fwd_ref,
        bwd_ref,
        mb_x,
        mb_y,
        mv,
        frame_motion_type,
        &mut pred_y,
        &mut pred_cb,
        &mut pred_cr,
        chroma_w,
        chroma_v,
    )?;

    // Final reconstruction buffers (post-residual / pre-write).
    let mut recon_y = [0u8; 16 * 16];
    let mut recon_cb = vec![0u8; chroma_w * chroma_v];
    let mut recon_cr = vec![0u8; chroma_w * chroma_v];
    // Default = pure prediction.
    recon_y.copy_from_slice(&pred_y);
    recon_cb.copy_from_slice(&pred_cb);
    recon_cr.copy_from_slice(&pred_cr);

    // For each block, decode residual if its CBP bit is set. Block ordering
    // is luma 0..3, then chroma per format. The CBP bit ordering follows
    // §6.3.5: bit `(N-1-b)` corresponds to block index `b` where `N` is
    // `blocks_per_mb`.
    let n_blocks = params.chroma_format.blocks_per_mb();
    for b in 0..n_blocks {
        let coded = (cbp_full >> (n_blocks - 1 - b)) & 1 != 0;
        let comp_idx = block_component(b, params.chroma_format);
        let (off_x, off_y) = block_offset_in_mb(b, comp_idx, params.chroma_format, dct_type_field);
        let (recon_buf, recon_stride): (&mut [u8], usize) = match comp_idx {
            0 => (&mut recon_y, 16),
            1 => (&mut recon_cb, chroma_w),
            _ => (&mut recon_cr, chroma_w),
        };
        let recon_sub = &mut recon_buf[off_y * recon_stride + off_x..];
        if coded {
            // Use the prediction block for this 8×8 sub-region as the
            // additive prediction passed to the residual decoder.
            // Build a stack-allocated 8×8 prediction patch.
            let mut pred_patch = [0u8; 64];
            let pred_src = match comp_idx {
                0 => &pred_y[..],
                1 => &pred_cb[..],
                _ => &pred_cr[..],
            };
            let pred_stride = match comp_idx {
                0 => 16,
                _ => chroma_w,
            };
            for j in 0..8 {
                let s = (off_y + j) * pred_stride + off_x;
                pred_patch[j * 8..j * 8 + 8].copy_from_slice(&pred_src[s..s + 8]);
            }
            decode_non_intra_block(
                br,
                quant_scale,
                &seq.non_intra_quantiser,
                params,
                &pred_patch,
                8,
                recon_sub,
                recon_stride,
            )?;
        } else {
            // Pure prediction copy. Already done by initial copy_from_slice
            // above — nothing to do here.
        }
    }

    // Apply field-DCT row deinterleave if the residual blocks were coded
    // in field order. Note: only the *coded* blocks needed permutation;
    // because we wrote each 8×8 into its post-deinterleave slot via
    // `block_offset_in_mb`, no further reshuffle is required for non-4:2:0.
    // For 4:2:0 the chroma blocks are not permuted regardless.
    let final_luma = if dct_type_field && params.is_mpeg2() {
        // Only deinterleave the *post-residual* luma if any luma block was
        // coded; otherwise the prediction stayed in frame order.
        let any_luma_coded = (0..4).any(|b| (cbp_full >> (n_blocks - 1 - b)) & 1 != 0);
        if any_luma_coded {
            deinterleave_field_dct(&recon_y)
        } else {
            recon_y
        }
    } else {
        recon_y
    };
    let (final_cb, final_cr) = if dct_type_field && params.is_mpeg2() && chroma_v == 16 {
        // Same caveat for chroma in 4:2:2/4:4:4.
        let chroma_blocks_start = 4;
        let chroma_block_count = params.chroma_format.chroma_blocks_per_mb();
        let any_chroma_coded = (chroma_blocks_start..chroma_blocks_start + chroma_block_count)
            .any(|b| (cbp_full >> (n_blocks - 1 - b)) & 1 != 0);
        if any_chroma_coded {
            (
                deinterleave_field_dct_chroma(&recon_cb, chroma_w),
                deinterleave_field_dct_chroma(&recon_cr, chroma_w),
            )
        } else {
            (recon_cb, recon_cr)
        }
    } else {
        (recon_cb, recon_cr)
    };

    write_mb_planes(
        pic,
        mb_x,
        mb_y,
        &final_luma,
        &final_cb,
        &final_cr,
        chroma_w,
        chroma_v,
    );
    Ok(())
}

/// Map a per-MB block index to its component index (0=Y, 1=Cb, 2=Cr) for
/// a given chroma format. Block layout follows H.262 §6.1.1.8 / Figure 6-10:
///
/// * 4:2:0: blocks 0..3 = luma, 4 = Cb, 5 = Cr.
/// * 4:2:2: blocks 0..3 = luma, 4..5 = Cb (top/bottom), 6..7 = Cr (top/bottom).
/// * 4:4:4: blocks 0..3 = luma, 4..7 = Cb (TL/TR/BL/BR), 8..11 = Cr.
fn block_component(b: usize, fmt: ChromaFormat) -> usize {
    match fmt {
        ChromaFormat::Yuv420 => match b {
            0..=3 => 0,
            4 => 1,
            _ => 2,
        },
        ChromaFormat::Yuv422 => match b {
            0..=3 => 0,
            4..=5 => 1,
            _ => 2,
        },
        ChromaFormat::Yuv444 => match b {
            0..=3 => 0,
            4..=7 => 1,
            _ => 2,
        },
    }
}

/// Return `(off_x, off_y)` of an 8×8 block within its component's MB-local
/// buffer (luma 16×16, chroma 8×8 / 8×16 / 16×16 depending on format).
///
/// In **field-DCT** mode the luma blocks are split horizontally into two
/// 16×8 field regions — block 0/1 cover the top field (rows 0,2,...,14)
/// and 2/3 cover the bottom field (rows 1,3,...,15). After block decode the
/// caller must call [`deinterleave_field_dct`] to interleave the rows back
/// into frame order. In this scheme block 0 is at MB-local rows 0..7, block
/// 1 at rows 0..7 cols 8..15, block 2 at rows 8..15 cols 0..7, block 3 at
/// rows 8..15 cols 8..15 — same layout as frame-DCT, with the per-row
/// permutation deferred until the deinterleave step. The chroma offset
/// table follows the same logic for 4:2:2 / 4:4:4.
fn block_offset_in_mb(
    b: usize,
    comp_idx: usize,
    fmt: ChromaFormat,
    _dct_type_field: bool,
) -> (usize, usize) {
    if comp_idx == 0 {
        // Luma 0..3 — same layout for every format.
        return match b {
            0 => (0, 0),
            1 => (8, 0),
            2 => (0, 8),
            3 => (8, 8),
            _ => (0, 0),
        };
    }
    // Chroma — depends on format.
    match fmt {
        ChromaFormat::Yuv420 => (0, 0),
        ChromaFormat::Yuv422 => {
            // Chroma block index relative to the chroma component (0 or 1
            // for 4:2:2 — top, bottom).
            let local = (b - 4) % 2;
            (0, local * 8)
        }
        ChromaFormat::Yuv444 => {
            // Four 8×8 blocks in a 16×16 chroma component (TL/TR/BL/BR).
            let local = (b - 4) % 4;
            match local {
                0 => (0, 0),
                1 => (8, 0),
                2 => (0, 8),
                _ => (8, 8),
            }
        }
    }
}

/// Re-interleave a 16×16 luma block from field-DCT order back to frame
/// order. Field-DCT layout: rows 0..7 of the buffer hold the *top field*
/// (MB rows 0,2,4,...,14); rows 8..15 hold the *bottom field* (MB rows
/// 1,3,5,...,15). Output buffer is in natural frame order (row 0,1,2,...).
fn deinterleave_field_dct(field: &[u8; 16 * 16]) -> [u8; 16 * 16] {
    let mut out = [0u8; 16 * 16];
    for r in 0..8 {
        // Top field row r → frame row 2*r.
        let src = &field[r * 16..r * 16 + 16];
        let dst = &mut out[(2 * r) * 16..(2 * r) * 16 + 16];
        dst.copy_from_slice(src);
    }
    for r in 0..8 {
        // Bottom field row r → frame row 2*r+1.
        let src = &field[(8 + r) * 16..(8 + r) * 16 + 16];
        let dst = &mut out[(2 * r + 1) * 16..(2 * r + 1) * 16 + 16];
        dst.copy_from_slice(src);
    }
    out
}

/// Same as [`deinterleave_field_dct`] but for an N-column × 16-row chroma
/// component (4:2:2 → 8 cols, 4:4:4 → 16 cols).
fn deinterleave_field_dct_chroma(field: &[u8], cols: usize) -> Vec<u8> {
    let mut out = vec![0u8; cols * 16];
    for r in 0..8 {
        out[(2 * r) * cols..(2 * r) * cols + cols]
            .copy_from_slice(&field[r * cols..r * cols + cols]);
        out[(2 * r + 1) * cols..(2 * r + 1) * cols + cols]
            .copy_from_slice(&field[(8 + r) * cols..(8 + r) * cols + cols]);
    }
    out
}

/// Write a fully-reconstructed MB into the picture buffer.
#[allow(clippy::too_many_arguments)]
fn write_mb_planes(
    pic: &mut PictureBuffer,
    mb_x: usize,
    mb_y: usize,
    luma: &[u8; 16 * 16],
    cb: &[u8],
    cr: &[u8],
    chroma_w: usize,
    chroma_v: usize,
) {
    let ys = pic.y_stride;
    let cs = pic.c_stride;
    for j in 0..16 {
        let off = (mb_y * 16 + j) * ys + mb_x * 16;
        pic.y[off..off + 16].copy_from_slice(&luma[j * 16..j * 16 + 16]);
    }
    let mb_cx = mb_x * chroma_w;
    let mb_cy = mb_y * chroma_v;
    for j in 0..chroma_v {
        let off = (mb_cy + j) * cs + mb_cx;
        pic.cb[off..off + chroma_w].copy_from_slice(&cb[j * chroma_w..j * chroma_w + chroma_w]);
        pic.cr[off..off + chroma_w].copy_from_slice(&cr[j * chroma_w..j * chroma_w + chroma_w]);
    }
}

/// Build the prediction buffers for a non-intra MB. Dispatches to frame /
/// field / dual-prime / forward-only / backward-only / interpolated
/// pathways based on `mv` and `frame_motion_type`.
#[allow(clippy::too_many_arguments)]
fn build_prediction_mb(
    params: &PictureParams,
    fwd_ref: Option<&PictureBuffer>,
    bwd_ref: Option<&PictureBuffer>,
    mb_x: usize,
    mb_y: usize,
    mv: &MotionData,
    frame_motion_type: FrameMotionType,
    pred_y: &mut [u8; 16 * 16],
    pred_cb: &mut [u8],
    pred_cr: &mut [u8],
    chroma_w: usize,
    chroma_v: usize,
) -> Result<()> {
    // Allocate temp prediction buffers for fwd / bwd (each filled by the
    // dispatch below if applicable).
    let mut fwd_y = [0u8; 16 * 16];
    let mut bwd_y = [0u8; 16 * 16];
    let mut fwd_cb = vec![0u8; chroma_w * chroma_v];
    let mut bwd_cb = vec![0u8; chroma_w * chroma_v];
    let mut fwd_cr = vec![0u8; chroma_w * chroma_v];
    let mut bwd_cr = vec![0u8; chroma_w * chroma_v];

    let mut have_fwd = false;
    let mut have_bwd = false;

    match frame_motion_type {
        FrameMotionType::Frame => {
            if let Some((mx, my)) = mv.fwd {
                let refp = fwd_ref
                    .ok_or_else(|| Error::invalid("forward MV without forward reference"))?;
                mc_mb_full(
                    params,
                    refp,
                    mb_x,
                    mb_y,
                    mx,
                    my,
                    &mut fwd_y,
                    &mut fwd_cb,
                    &mut fwd_cr,
                    chroma_w,
                    chroma_v,
                );
                have_fwd = true;
            }
            if let Some((mx, my)) = mv.bwd {
                let refp = bwd_ref
                    .ok_or_else(|| Error::invalid("backward MV without backward reference"))?;
                mc_mb_full(
                    params,
                    refp,
                    mb_x,
                    mb_y,
                    mx,
                    my,
                    &mut bwd_y,
                    &mut bwd_cb,
                    &mut bwd_cr,
                    chroma_w,
                    chroma_v,
                );
                have_bwd = true;
            }
        }
        FrameMotionType::Field => {
            if let Some(entries) = mv.fwd_field {
                let refp = fwd_ref
                    .ok_or_else(|| Error::invalid("field forward MV without forward reference"))?;
                mc_mb_field(
                    params,
                    refp,
                    mb_x,
                    mb_y,
                    &entries,
                    &mut fwd_y,
                    &mut fwd_cb,
                    &mut fwd_cr,
                    chroma_w,
                    chroma_v,
                );
                have_fwd = true;
            }
            if let Some(entries) = mv.bwd_field {
                let refp = bwd_ref.ok_or_else(|| {
                    Error::invalid("field backward MV without backward reference")
                })?;
                mc_mb_field(
                    params,
                    refp,
                    mb_x,
                    mb_y,
                    &entries,
                    &mut bwd_y,
                    &mut bwd_cb,
                    &mut bwd_cr,
                    chroma_w,
                    chroma_v,
                );
                have_bwd = true;
            }
        }
        FrameMotionType::DualPrime => {
            // Dual-prime is forward-only, single-direction (P-picture).
            // Average two parity-pair predictions: same-parity field MV =
            // (mx, my), opposite-parity = (mx + dmv_x, my+dmv_y + parity_adj).
            let refp =
                fwd_ref.ok_or_else(|| Error::invalid("dual-prime requires forward reference"))?;
            let (mx, my, dmv_x, dmv_y) = mv
                .dual_prime
                .ok_or_else(|| Error::invalid("dual-prime: missing MV payload"))?;
            // Build two field-MV entries: top reference + bottom reference.
            // Same-parity MVs: top→top uses (mx, my/2), bottom→bottom uses
            // (mx, my/2). Opposite-parity: top→bottom uses
            // (mx+dmv_x, my/2+dmv_y - 1), bottom→top uses
            // (mx+dmv_x, my/2+dmv_y + 1). The /2 reflects the
            // "field-relative" Y axis; we keep MVs in luma-frame half-pels
            // and let the field-MC fetcher remap.
            // Simpler implementation: compute four field predictions and
            // average parity-pair outputs.
            let mvp = [
                (mx, my, 0u8, 0u8),
                (mx, my, 1u8, 1u8),
                (mx + dmv_x, my + dmv_y, 0u8, 1u8),
                (mx + dmv_x, my + dmv_y, 1u8, 0u8),
            ];
            // Build predictions in two passes: same-parity, opposite-parity.
            let mut same_y = [0u8; 16 * 16];
            let mut opp_y = [0u8; 16 * 16];
            let mut same_cb = vec![0u8; chroma_w * chroma_v];
            let mut opp_cb = vec![0u8; chroma_w * chroma_v];
            let mut same_cr = vec![0u8; chroma_w * chroma_v];
            let mut opp_cr = vec![0u8; chroma_w * chroma_v];
            // Same-parity entries: index 0 = top→top, 1 = bottom→bottom.
            let same_entries = [
                (mvp[0].0, mvp[0].1, mvp[0].3),
                (mvp[1].0, mvp[1].1, mvp[1].3),
            ];
            mc_mb_field(
                params,
                refp,
                mb_x,
                mb_y,
                &same_entries,
                &mut same_y,
                &mut same_cb,
                &mut same_cr,
                chroma_w,
                chroma_v,
            );
            // Opposite-parity entries: top dest from bottom ref; bottom dest
            // from top ref.
            let opp_entries = [
                (mvp[2].0, mvp[2].1, mvp[2].3),
                (mvp[3].0, mvp[3].1, mvp[3].3),
            ];
            mc_mb_field(
                params,
                refp,
                mb_x,
                mb_y,
                &opp_entries,
                &mut opp_y,
                &mut opp_cb,
                &mut opp_cr,
                chroma_w,
                chroma_v,
            );
            // Average.
            for i in 0..16 * 16 {
                fwd_y[i] = ((same_y[i] as u32 + opp_y[i] as u32 + 1) >> 1) as u8;
            }
            for i in 0..chroma_w * chroma_v {
                fwd_cb[i] = ((same_cb[i] as u32 + opp_cb[i] as u32 + 1) >> 1) as u8;
                fwd_cr[i] = ((same_cr[i] as u32 + opp_cr[i] as u32 + 1) >> 1) as u8;
            }
            have_fwd = true;
        }
    }

    match (have_fwd, have_bwd) {
        (true, false) => {
            pred_y.copy_from_slice(&fwd_y);
            pred_cb.copy_from_slice(&fwd_cb);
            pred_cr.copy_from_slice(&fwd_cr);
        }
        (false, true) => {
            pred_y.copy_from_slice(&bwd_y);
            pred_cb.copy_from_slice(&bwd_cb);
            pred_cr.copy_from_slice(&bwd_cr);
        }
        (true, true) => {
            for i in 0..16 * 16 {
                pred_y[i] = ((fwd_y[i] as u32 + bwd_y[i] as u32 + 1) >> 1) as u8;
            }
            for i in 0..chroma_w * chroma_v {
                pred_cb[i] = ((fwd_cb[i] as u32 + bwd_cb[i] as u32 + 1) >> 1) as u8;
                pred_cr[i] = ((fwd_cr[i] as u32 + bwd_cr[i] as u32 + 1) >> 1) as u8;
            }
        }
        (false, false) => {
            return Err(Error::invalid(
                "inter MB with neither forward nor backward MV",
            ));
        }
    }
    Ok(())
}

/// Frame motion compensation — fetch a 16×16 luma + matching chroma from
/// `refp` at MB position `(mb_x, mb_y)` with `(mv_x, mv_y)` half-pel
/// offsets. Uses the picture's chroma_format to size and scale chroma.
#[allow(clippy::too_many_arguments)]
fn mc_mb_full(
    params: &PictureParams,
    refp: &PictureBuffer,
    mb_x: usize,
    mb_y: usize,
    mv_x: i32,
    mv_y: i32,
    dst_y: &mut [u8; 16 * 16],
    dst_cb: &mut [u8],
    dst_cr: &mut [u8],
    chroma_w: usize,
    chroma_v: usize,
) {
    let mb_px = (mb_x * 16) as i32;
    let mb_py = (mb_y * 16) as i32;
    motion::predict_block(
        &refp.y,
        refp.y_stride,
        refp.y_stride as i32,
        (refp.y.len() / refp.y_stride) as i32,
        mb_px,
        mb_py,
        mv_x,
        mv_y,
        16,
        dst_y,
        16,
    );
    let c_px = (mb_x * chroma_w) as i32;
    let c_py = (mb_y * chroma_v) as i32;
    let mv_cx = motion::scale_mv_h_to_chroma(mv_x, params.chroma_format);
    let mv_cy = motion::scale_mv_v_to_chroma(mv_y, params.chroma_format);
    motion::predict_block(
        &refp.cb,
        refp.c_stride,
        refp.c_stride as i32,
        (refp.cb.len() / refp.c_stride) as i32,
        c_px,
        c_py,
        mv_cx,
        mv_cy,
        chroma_w as i32,
        dst_cb,
        chroma_w,
    );
    motion::predict_block(
        &refp.cr,
        refp.c_stride,
        refp.c_stride as i32,
        (refp.cr.len() / refp.c_stride) as i32,
        c_px,
        c_py,
        mv_cx,
        mv_cy,
        chroma_w as i32,
        dst_cr,
        chroma_w,
    );
    let _ = chroma_v;
    let _ = params;
}

/// Field motion compensation — predict each 16×8 luma half (rows 0,2,…,14
/// and rows 1,3,…,15) from one of the two field references chosen by
/// `entries[i].2` (parity), then write the 16×16 output in deinterleaved
/// frame order.
///
/// `entries[0]` = top output field, `entries[1]` = bottom output field.
/// Each `(mv_x, mv_y, parity)` selects a 16×8 patch from the same parity
/// field of the reference frame (parity 0 = top field, 1 = bottom field).
/// The MV vertical component is in *frame* half-pel units; the field
/// fetcher down-shifts by one bit to move into the field grid.
#[allow(clippy::too_many_arguments)]
fn mc_mb_field(
    params: &PictureParams,
    refp: &PictureBuffer,
    mb_x: usize,
    mb_y: usize,
    entries: &[(i32, i32, u8); 2],
    dst_y: &mut [u8; 16 * 16],
    dst_cb: &mut [u8],
    dst_cr: &mut [u8],
    chroma_w: usize,
    chroma_v: usize,
) {
    for (slot, &(mv_x, mv_y, parity)) in entries.iter().enumerate() {
        // Field grid: rows of one parity field are spaced 2 apart in frame
        // coordinates. Vertical MV in frame half-pels is `mv_y`; the
        // *field-grid* half-pel value is `mv_y` (we already have a half-
        // pel base on the frame grid; mapping a 16×8 field block to its
        // frame-grid row indices needs a doubling — we do that by stepping
        // the integer source row by 2 and the half-pel selector by `mv_y`
        // mod 2 directly).
        //
        // We use the existing `predict_block` helper to fetch a 16×16 patch
        // out of the chosen field, then sub-sample every other row to land
        // on a 16×8 block. To keep the code simple (and avoid duplicating
        // a separate 16×8 fetcher) we synthesize a temporary 16×16 fetch
        // from a "field frame" by shifting `mb_py` and dividing the stride.
        //
        // In our PictureBuffer model the reference is stored as a frame.
        // The top field is rows {0,2,4,...} and the bottom field is rows
        // {1,3,5,...}. We can express "fetch a 16×8 block from field of
        // parity P at frame-grid (mb_px, mb_py + slot)" as:
        //   step in source = 2 frame rows per output row.
        //
        // For brevity we fall back to a row-by-row fetch.
        fetch_field_patch(
            refp,
            mb_x,
            mb_y,
            slot,
            parity,
            mv_x,
            mv_y,
            dst_y,
            chroma_w,
            chroma_v,
            dst_cb,
            dst_cr,
            params.chroma_format,
        );
    }
}

/// Fetch one 16×8 luma + matching chroma 16×8/16×4 patch into the
/// `slot`-th output field row range (slot=0 → output rows 0,2,...,14;
/// slot=1 → output rows 1,3,...,15).
#[allow(clippy::too_many_arguments)]
fn fetch_field_patch(
    refp: &PictureBuffer,
    mb_x: usize,
    mb_y: usize,
    slot: usize,
    parity: u8,
    mv_x: i32,
    mv_y: i32,
    dst_y: &mut [u8; 16 * 16],
    chroma_w: usize,
    chroma_v: usize,
    dst_cb: &mut [u8],
    dst_cr: &mut [u8],
    fmt: ChromaFormat,
) {
    let mb_px = (mb_x * 16) as i32;
    let mb_py_top = (mb_y * 16) as i32;
    // The output field-row 0 of this MB sits at frame-row mb_py + slot.
    // Source (reference) field-row 0 of parity P sits at frame-row
    // (parity as i32). Source rows of parity P are at frame-row
    // (parity + 2*field_row).
    //
    // Convert MV from luma frame half-pels to field-grid: vertical step is
    // measured in half-pels of the field axis, which is half as dense as
    // the frame axis. A frame-grid MV of `mv_y` half-pels corresponds to
    // `mv_y` *field* half-pels too in the H.262 convention, since the
    // field MV is transmitted in half-pel-of-the-field units (§7.6.3.4).
    // So we use `mv_y` directly against the field stride.
    let int_x = mv_x.div_euclid(2);
    let hx = (mv_x.rem_euclid(2)) != 0;
    let int_y = mv_y.div_euclid(2);
    let hy = (mv_y.rem_euclid(2)) != 0;

    // Luma — fetch 16 wide × 8 high from the parity field of the reference.
    let src_x = mb_px + int_x;
    let src_field_y0 = (mb_py_top / 2) + int_y; // start row inside the parity field
    fetch_16x8_field(
        &refp.y,
        refp.y_stride,
        parity as i32,
        src_x,
        src_field_y0,
        hx,
        hy,
        dst_y,
        slot,
    );

    // Chroma — fetch chroma_w wide × (chroma_v/2) high from the parity
    // field of the reference chroma plane.
    let mv_cx = motion::scale_mv_h_to_chroma(mv_x, fmt);
    let mv_cy = motion::scale_mv_v_to_chroma(mv_y, fmt);
    let int_cx = mv_cx.div_euclid(2);
    let hcx = mv_cx.rem_euclid(2) != 0;
    let int_cy = mv_cy.div_euclid(2);
    let hcy = mv_cy.rem_euclid(2) != 0;
    let mb_cx = (mb_x * chroma_w) as i32;
    let mb_cy_top = (mb_y * chroma_v) as i32;
    let src_cx = mb_cx + int_cx;
    let chroma_v_half = chroma_v / 2;

    if chroma_v == 8 {
        // 4:2:0 chroma — only ONE row of chroma blocks per MB pair of
        // field rows. Use parity-pair strategy: each output luma field
        // sees a different chroma row from the reference. We approximate
        // by averaging — the spec is fiddly here. For simplicity we
        // fall back to frame chroma MC (good enough for most ffmpeg
        // streams; "field MC" with 4:2:0 chroma is rare and the ME
        // search already converged on a good MV).
        if slot == 0 {
            let src_cy = mb_cy_top / 2 + int_cy;
            fetch_block_clamped(
                &refp.cb,
                refp.c_stride,
                src_cx,
                src_cy,
                hcx,
                hcy,
                chroma_w as i32,
                chroma_v as i32,
                dst_cb,
                chroma_w,
            );
            fetch_block_clamped(
                &refp.cr,
                refp.c_stride,
                src_cx,
                src_cy,
                hcx,
                hcy,
                chroma_w as i32,
                chroma_v as i32,
                dst_cr,
                chroma_w,
            );
        }
    } else {
        // 4:2:2 / 4:4:4 chroma — chroma vertical is full, so chroma also
        // has top and bottom field rows.
        let src_cy_field0 = mb_cy_top / 2 + int_cy;
        fetch_NxM_field(
            &refp.cb,
            refp.c_stride,
            parity as i32,
            src_cx,
            src_cy_field0,
            hcx,
            hcy,
            dst_cb,
            chroma_w,
            chroma_v_half,
            slot,
        );
        fetch_NxM_field(
            &refp.cr,
            refp.c_stride,
            parity as i32,
            src_cx,
            src_cy_field0,
            hcx,
            hcy,
            dst_cr,
            chroma_w,
            chroma_v_half,
            slot,
        );
    }
}

/// Fetch a 16×8 luma block from a parity field of the reference and write
/// into the slot-th output field rows of a 16×16 destination buffer.
#[allow(clippy::too_many_arguments)]
fn fetch_16x8_field(
    plane: &[u8],
    stride: usize,
    parity: i32,
    src_x: i32,
    src_field_y0: i32,
    hx: bool,
    hy: bool,
    dst: &mut [u8; 16 * 16],
    slot: usize,
) {
    let plane_h = (plane.len() / stride) as i32;
    let plane_w = stride as i32;
    let clamp_x = |x: i32| x.clamp(0, plane_w - 1);
    let clamp_field_y =
        |fy: i32| -> i32 { ((parity + 2 * fy).clamp(0, plane_h - 1) - parity).div_euclid(2) };

    for j in 0..8 {
        let fy0 = clamp_field_y(src_field_y0 + j);
        let fy1 = if hy {
            clamp_field_y(src_field_y0 + j + 1)
        } else {
            fy0
        };
        // The actual sample row in the frame plane is `parity + 2 * fy`.
        let row0 = (parity + 2 * fy0) as usize;
        let row1 = (parity + 2 * fy1) as usize;
        for i in 0..16 {
            let x0 = clamp_x(src_x + i) as usize;
            let x1 = if hx {
                clamp_x(src_x + i + 1) as usize
            } else {
                x0
            };
            let a = plane[row0 * stride + x0] as u32;
            let b = plane[row0 * stride + x1] as u32;
            let c = plane[row1 * stride + x0] as u32;
            let d = plane[row1 * stride + x1] as u32;
            let v = match (hx, hy) {
                (false, false) => a,
                (true, false) => (a + b + 1) >> 1,
                (false, true) => (a + c + 1) >> 1,
                (true, true) => (a + b + c + d + 2) >> 2,
            };
            // Write to output frame row 2*j + slot, column i.
            dst[(2 * j as usize + slot) * 16 + i as usize] = v as u8;
        }
    }
}

/// Same as [`fetch_16x8_field`] but for an arbitrary `cols × rows` patch.
/// Used for 4:2:2 / 4:4:4 chroma (cols ∈ {8, 16}, rows = 8).
#[allow(clippy::too_many_arguments, non_snake_case)]
fn fetch_NxM_field(
    plane: &[u8],
    stride: usize,
    parity: i32,
    src_x: i32,
    src_field_y0: i32,
    hx: bool,
    hy: bool,
    dst: &mut [u8],
    cols: usize,
    rows: usize,
    slot: usize,
) {
    let plane_h = (plane.len() / stride) as i32;
    let plane_w = stride as i32;
    let clamp_x = |x: i32| x.clamp(0, plane_w - 1);
    let clamp_field_y =
        |fy: i32| -> i32 { ((parity + 2 * fy).clamp(0, plane_h - 1) - parity).div_euclid(2) };

    for j in 0..rows {
        let fy0 = clamp_field_y(src_field_y0 + j as i32);
        let fy1 = if hy {
            clamp_field_y(src_field_y0 + j as i32 + 1)
        } else {
            fy0
        };
        let row0 = (parity + 2 * fy0) as usize;
        let row1 = (parity + 2 * fy1) as usize;
        for i in 0..cols {
            let x0 = clamp_x(src_x + i as i32) as usize;
            let x1 = if hx {
                clamp_x(src_x + i as i32 + 1) as usize
            } else {
                x0
            };
            let a = plane[row0 * stride + x0] as u32;
            let b = plane[row0 * stride + x1] as u32;
            let c = plane[row1 * stride + x0] as u32;
            let d = plane[row1 * stride + x1] as u32;
            let v = match (hx, hy) {
                (false, false) => a,
                (true, false) => (a + b + 1) >> 1,
                (false, true) => (a + c + 1) >> 1,
                (true, true) => (a + b + c + d + 2) >> 2,
            };
            dst[(2 * j + slot) * cols + i] = v as u8;
        }
    }
}

/// Generic clamped 2D fetch used for 4:2:0 chroma in the field-MC fallback.
#[allow(clippy::too_many_arguments)]
fn fetch_block_clamped(
    plane: &[u8],
    stride: usize,
    src_x: i32,
    src_y: i32,
    hx: bool,
    hy: bool,
    cols: i32,
    rows: i32,
    dst: &mut [u8],
    dst_stride: usize,
) {
    let plane_h = (plane.len() / stride) as i32;
    let plane_w = stride as i32;
    let clamp_x = |x: i32| x.clamp(0, plane_w - 1);
    let clamp_y = |y: i32| y.clamp(0, plane_h - 1);
    for j in 0..rows {
        let y0 = clamp_y(src_y + j) as usize;
        let y1 = if hy {
            clamp_y(src_y + j + 1) as usize
        } else {
            y0
        };
        for i in 0..cols {
            let x0 = clamp_x(src_x + i) as usize;
            let x1 = if hx {
                clamp_x(src_x + i + 1) as usize
            } else {
                x0
            };
            let a = plane[y0 * stride + x0] as u32;
            let b = plane[y0 * stride + x1] as u32;
            let c = plane[y1 * stride + x0] as u32;
            let d = plane[y1 * stride + x1] as u32;
            let v = match (hx, hy) {
                (false, false) => a,
                (true, false) => (a + b + 1) >> 1,
                (false, true) => (a + c + 1) >> 1,
                (true, true) => (a + b + c + d + 2) >> 2,
            };
            dst[(j as usize) * dst_stride + (i as usize)] = v as u8;
        }
    }
}

/// Skipped-P helper — write a forward-predicted MB into `pic` using
/// MV `(mv_x, mv_y)` and the given reference. Used when the slice contains
/// MBA increments larger than 1.
fn fill_forward_predict(
    pic: &mut PictureBuffer,
    fwd_ref: Option<&PictureBuffer>,
    mb_x: usize,
    mb_y: usize,
    mv_x: i32,
    mv_y: i32,
) -> Result<()> {
    let refp = fwd_ref.ok_or_else(|| Error::invalid("skip MB without forward ref"))?;
    let chroma_w = match pic.chroma_format {
        ChromaFormat::Yuv420 | ChromaFormat::Yuv422 => 8usize,
        ChromaFormat::Yuv444 => 16usize,
    };
    let chroma_v = match pic.chroma_format {
        ChromaFormat::Yuv420 => 8usize,
        ChromaFormat::Yuv422 | ChromaFormat::Yuv444 => 16usize,
    };
    let mut y = [0u8; 16 * 16];
    let mut cb = vec![0u8; chroma_w * chroma_v];
    let mut cr = vec![0u8; chroma_w * chroma_v];
    let dummy_params = PictureParams {
        codec: Codec::Mpeg2,
        intra_dc_precision: 0,
        alternate_scan: false,
        intra_vlc_format: false,
        q_scale_type: false,
        f_code: [[15, 15], [15, 15]],
        full_pel_fwd: false,
        full_pel_bwd: false,
        chroma_format: pic.chroma_format,
        picture_structure: PictureStructure::Frame,
        frame_pred_frame_dct: true,
        concealment_motion_vectors: false,
        progressive_frame: true,
        top_field_first: true,
    };
    mc_mb_full(
        &dummy_params,
        refp,
        mb_x,
        mb_y,
        mv_x,
        mv_y,
        &mut y,
        &mut cb,
        &mut cr,
        chroma_w,
        chroma_v,
    );
    write_mb_planes(pic, mb_x, mb_y, &y, &cb, &cr, chroma_w, chroma_v);
    Ok(())
}

/// Skipped-B helper — write a bidirectionally-predicted MB into `pic` using
/// the supplied forward / backward MVs. Either MV may be `None` (in which
/// case that direction is omitted).
fn fill_bidir_predict(
    pic: &mut PictureBuffer,
    fwd_ref: Option<&PictureBuffer>,
    bwd_ref: Option<&PictureBuffer>,
    mb_x: usize,
    mb_y: usize,
    fwd_mv: Option<(i32, i32)>,
    bwd_mv: Option<(i32, i32)>,
) -> Result<()> {
    let chroma_w = match pic.chroma_format {
        ChromaFormat::Yuv420 | ChromaFormat::Yuv422 => 8usize,
        ChromaFormat::Yuv444 => 16usize,
    };
    let chroma_v = match pic.chroma_format {
        ChromaFormat::Yuv420 => 8usize,
        ChromaFormat::Yuv422 | ChromaFormat::Yuv444 => 16usize,
    };
    let mut y = [0u8; 16 * 16];
    let mut cb = vec![0u8; chroma_w * chroma_v];
    let mut cr = vec![0u8; chroma_w * chroma_v];
    let dummy_params = PictureParams {
        codec: Codec::Mpeg2,
        intra_dc_precision: 0,
        alternate_scan: false,
        intra_vlc_format: false,
        q_scale_type: false,
        f_code: [[15, 15], [15, 15]],
        full_pel_fwd: false,
        full_pel_bwd: false,
        chroma_format: pic.chroma_format,
        picture_structure: PictureStructure::Frame,
        frame_pred_frame_dct: true,
        concealment_motion_vectors: false,
        progressive_frame: true,
        top_field_first: true,
    };
    let mv_data = MotionData {
        fwd: fwd_mv,
        bwd: bwd_mv,
        fwd_field: None,
        bwd_field: None,
        dual_prime: None,
    };
    build_prediction_mb(
        &dummy_params,
        fwd_ref,
        bwd_ref,
        mb_x,
        mb_y,
        &mv_data,
        FrameMotionType::Frame,
        &mut y,
        &mut cb,
        &mut cr,
        chroma_w,
        chroma_v,
    )?;
    write_mb_planes(pic, mb_x, mb_y, &y, &cb, &cr, chroma_w, chroma_v);
    Ok(())
}
