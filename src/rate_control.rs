//! Constant-bit-rate rate control for the MPEG-2 encoder — a
//! VBV-regulated (ISO/IEC 13818-2 Annex C) display-order GOP assembler
//! that adapts the per-picture quantiser and stamps real §6.3.9
//! `vbv_delay` values.
//!
//! ## How the Annex C constraints drive the controller
//!
//! The standard does not prescribe a rate-control algorithm; it
//! prescribes the **buffer model** the finished stream must satisfy
//! (Annex C): at every examination the VBV occupancy must lie in
//! `[0, B]` (C.5 / C.6). For the constant-rate operation this module
//! emits (`vbv_delay != 0xFFFF`, C.3.1), that turns into two hard
//! per-picture bounds the exact [`crate::vbv::VbvCbrModel`] supplies:
//!
//! * **underflow bound** ([`crate::vbv::VbvCbrModel::max_end_bits`]) —
//!   all of a picture's data must have arrived by its removal time
//!   (C.6), capping the picture's size; the controller *re-encodes the
//!   picture at a coarser `quantiser_scale_code`* until it fits;
//! * **overflow bound** ([`crate::vbv::VbvCbrModel::min_end_bits`]) —
//!   by the next examination the buffer must not exceed `B` (C.5); a
//!   picture that undershoots is padded with **zero-byte stuffing**
//!   before the next start code (legal per the §5.2.3
//!   `next_start_code()` zero stuffing; Annex C counts stuffing
//!   following a picture as that picture's data).
//!
//! Between the hard bounds a soft feedback loop steers quality: each
//! picture type (I / P / B) carries a running `quantiser_scale_code`
//! that steps up when the type's last picture overshot its nominal
//! bit budget and down when it undershot — the budget being the
//! per-GOP bit pool `R * gop_pictures / frame_rate` split across the
//! GOP's picture types by fixed I : P : B weights.
//!
//! Every picture header carries the real C.3.1
//! `vbv_delay = 90 000 * B*(n) / R` (stamped by
//! [`crate::vbv::patch_vbv_delay`]), so the stream verifies end-to-end
//! against [`crate::vbv::verify_cbr_stream`].

// The coded-order GOP walk indexes `frames` by display position (the
// B-run between two anchors), mirroring `inter_encoder`'s assemblers.
#![allow(clippy::needless_range_loop)]

use oxideav_core::bits::BitWriter;

use crate::frame_assembly::{FrameBuffer, IntraPictureParams};
use crate::gop_header::{write_gop_header, Mpeg2Gop, TimeCode};
use crate::stream_writer::{
    write_sequence_extension, write_sequence_header, SequenceHeaderParams, SEQUENCE_END_CODE,
};
use crate::vbv::{frame_rate_value, patch_vbv_delay, RemovalInterval, VbvCbrModel, VbvParams};
use crate::{Error, Result};

/// Static CBR configuration for [`encode_cbr_gop_sequence`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CbrConfig {
    /// §6.2.2.1 `bit_rate_value` (18 bits, units of 400 bit/s). The
    /// actual rate `R` the VBV model runs at is exactly
    /// `bit_rate_value * 400` — declared and actual rate coincide, as
    /// §6.3.3 recommends for constant-rate operation.
    pub bit_rate_value: u32,
    /// §6.2.2.1 `vbv_buffer_size_value` (10 bits);
    /// `B = vbv_buffer_size_value * 16 * 1024` bits.
    pub vbv_buffer_size_value: u16,
    /// Table 6-4 `frame_rate_code` for the sequence header (also the
    /// VBV removal cadence, C.9).
    pub frame_rate_code: u8,
    /// The `quantiser_scale_code` every picture type starts from
    /// (`1..=31`); the controller adapts from here.
    pub initial_quantiser_scale_code: u8,
}

impl Default for CbrConfig {
    fn default() -> Self {
        Self {
            bit_rate_value: 2500, // 1 Mbit/s
            vbv_buffer_size_value: 20,
            frame_rate_code: 3, // 25 frames/s
            initial_quantiser_scale_code: 6,
        }
    }
}

/// The result of a CBR encode: the elementary stream plus the
/// controller's per-picture decisions for inspection / tests.
#[derive(Debug, Clone)]
pub struct CbrEncoded {
    /// The finished elementary stream.
    pub stream: Vec<u8>,
    /// The `quantiser_scale_code` each coded picture was finally
    /// written with, in coded order.
    pub quantiser_scale_codes: Vec<u8>,
    /// Zero stuffing bytes appended across the stream to hold the C.5
    /// overflow bound.
    pub stuffing_bytes: u64,
}

/// Per-type index into the running quantiser table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PictureKind {
    I,
    P,
    B,
}

impl PictureKind {
    fn index(self) -> usize {
        match self {
            Self::I => 0,
            Self::P => 1,
            Self::B => 2,
        }
    }
}

/// Bit-budget weights per picture type, ×10 (I : P : B = 4 : 1.6 : 0.9)
/// — a conventional anchor-heavy split; only the *ratios* matter, the
/// hard Annex C bounds keep any mis-estimate legal.
const TYPE_WEIGHT_X10: [u64; 3] = [40, 16, 9];

/// The CBR writer state threaded through the coded-order emission.
struct CbrWriter {
    out: Vec<u8>,
    model: VbvCbrModel,
    /// Frame-rate denominator (for the removal-interval arithmetic).
    frd: u32,
    /// Nominal per-type picture bit budgets (soft targets).
    target_bits: [u64; 3],
    /// Running per-type quantiser_scale_code.
    q: [u8; 3],
    /// Report fields.
    quantisers: Vec<u8>,
    stuffing_bytes: u64,
    started: bool,
    /// `B` in bits (for the initial-delay choice).
    buffer_bits: u64,
    /// `R` in bits/s.
    bit_rate: u64,
}

impl CbrWriter {
    /// Emit one coded picture: `headers` are the bytes that belong to
    /// this picture's data but precede its `picture_start_code`
    /// (GOP header; empty otherwise), `enc(q)` produces the picture
    /// layer (starting with the `picture_start_code`, byte-aligned) and
    /// the encoder-side reconstruction for anchor pictures.
    ///
    /// Applies the C.6 re-encode loop, stamps the C.3.1 `vbv_delay`,
    /// appends C.5 zero stuffing, and advances the model by one frame
    /// period (C.9 — the emitted sequence is progressive,
    /// `repeat_first_field = 0`).
    fn emit<F>(
        &mut self,
        headers: &[u8],
        kind: PictureKind,
        is_last: bool,
        mut enc: F,
    ) -> Result<Option<FrameBuffer>>
    where
        F: FnMut(u8) -> Result<(Vec<u8>, Option<FrameBuffer>)>,
    {
        self.out.extend_from_slice(headers);
        let pos_bits = (self.out.len() as u64 + 4) * 8;

        if !self.started {
            // Seed the model: target ~3/4 of the buffer as the initial
            // occupancy (comfortably inside both Annex C bounds for any
            // sane configuration; the model checks regardless).
            let tau0_ticks = (90_000u128 * u128::from(self.buffer_bits) * 3
                / (4 * u128::from(self.bit_rate)))
            .min(0xFFFE) as u16;
            self.model.start(tau0_ticks, pos_bits);
            self.started = true;
        }
        let delay = self.model.delay_for_picture(pos_bits)?;
        let max_end = self.model.max_end_bits();

        let tail_bits = if is_last { 32u64 } else { 0 }; // sequence_end_code
        let end_bits_with =
            |out_len: usize, layer_len: usize| ((out_len + layer_len) as u64) * 8 + tail_bits;

        let mut q = self.q[kind.index()];
        let (mut layer, mut recon) = enc(q)?;
        // C.6 hard bound: coarsen until the picture fits the buffer's
        // available data at its removal time.
        let mut guard = 0u32;
        while end_bits_with(self.out.len(), layer.len()) > max_end && q < 31 && guard < 16 {
            q = (q + 1 + q / 4).min(31);
            let (l, r) = enc(q)?;
            layer = l;
            recon = r;
            guard += 1;
        }
        if end_bits_with(self.out.len(), layer.len()) > max_end {
            return Err(Error::InvalidBitstream(
                "CBR: picture exceeds the C.6 underflow bound even at quantiser_scale_code 31 \
                 (bit_rate / vbv_buffer_size too small for this content)",
            ));
        }

        patch_vbv_delay(&mut layer, delay)?;
        let actual_bits = (layer.len() as u64) * 8;
        self.out.extend_from_slice(&layer);
        if is_last {
            self.out.extend_from_slice(&SEQUENCE_END_CODE.to_be_bytes());
        }

        let mut end_bits = (self.out.len() as u64) * 8;
        if !is_last {
            // C.5: stuff up to the overflow bound for the next
            // examination (no examination follows the last picture).
            let min_end = self.model.min_end_bits(RemovalInterval::Frame, self.frd);
            if end_bits < min_end {
                let stuff = min_end.saturating_sub(end_bits).div_ceil(8);
                self.out.resize(self.out.len() + stuff as usize, 0u8);
                self.stuffing_bytes += stuff;
                end_bits += stuff * 8;
            }
        }

        self.model
            .remove_picture(end_bits, RemovalInterval::Frame, self.frd)
            .map_err(|v| {
                // The controller holds both bounds before removal, so a
                // violation here is an internal invariant break.
                let _ = v;
                Error::InvalidBitstream("CBR: internal VBV accounting violation")
            })?;

        // Soft feedback: steer the next picture of this type toward its
        // nominal budget.
        let target = self.target_bits[kind.index()].max(1);
        self.q[kind.index()] = if actual_bits * 5 > target * 6 {
            (q + 1).min(31)
        } else if actual_bits * 10 < target * 7 {
            q.saturating_sub(1).max(1)
        } else {
            q
        };
        self.quantisers.push(q);
        Ok(recon)
    }
}

/// Encode a whole **display-order** frame sequence as a CBR MPEG-2
/// elementary stream with the same GOP structure as
/// [`crate::encode_display_order_gop_sequence`] (one I per GOP,
/// `anchors_per_gop` predictive periods of `b_between` B-pictures,
/// closed GOPs, per-GOP `temporal_reference` reset), under Annex C
/// VBV regulation:
///
/// * the sequence header declares `cbr.bit_rate_value` /
///   `cbr.vbv_buffer_size_value` and the stream **satisfies** them —
///   every picture holds the C.5 / C.6 occupancy bounds of the exact
///   buffer model, by per-picture quantiser adaptation (coarsening
///   re-encode against the underflow bound, soft budget feedback
///   between pictures) and zero-byte stuffing against the overflow
///   bound;
/// * every `picture_header()` carries the real C.3.1 / §6.3.9
///   `vbv_delay` (never the `0xFFFF` variable-rate sentinel), so
///   [`crate::vbv::verify_cbr_stream`] accepts the result.
///
/// As with the other assemblers, every anchor the encoder predicts
/// from is the decoder's exact reconstruction, so the stream decodes
/// faithfully through [`crate::decode_video_sequence`].
///
/// # Errors
/// [`Error::InvalidBitstream`] for an empty `frames`,
/// `anchors_per_gop == 0`, out-of-range config fields, a
/// `vbv_buffer_size` too large for the 16-bit `vbv_delay` at this rate
/// (§6.3.9), or content that cannot fit the declared rate even at
/// `quantiser_scale_code` 31; propagates encode / decode errors.
pub fn encode_cbr_gop_sequence(
    frames: &[FrameBuffer],
    b_between: usize,
    anchors_per_gop: usize,
    params: IntraPictureParams,
    cbr: &CbrConfig,
    forward_f_code: u8,
    backward_f_code: u8,
) -> Result<CbrEncoded> {
    if frames.is_empty() {
        return Err(Error::InvalidBitstream(
            "encode_cbr_gop_sequence: no frames to encode",
        ));
    }
    if anchors_per_gop == 0 {
        return Err(Error::InvalidBitstream(
            "encode_cbr_gop_sequence: anchors_per_gop must be >= 1",
        ));
    }
    if cbr.bit_rate_value == 0 || cbr.bit_rate_value > 0x3FFFF {
        return Err(Error::InvalidBitstream(
            "encode_cbr_gop_sequence: bit_rate_value out of the 18-bit §6.2.2.1 range",
        ));
    }
    if cbr.vbv_buffer_size_value == 0 || cbr.vbv_buffer_size_value > 0x3FF {
        return Err(Error::InvalidBitstream(
            "encode_cbr_gop_sequence: vbv_buffer_size_value out of the 10-bit §6.2.2.1 range",
        ));
    }
    if !(1..=31).contains(&cbr.initial_quantiser_scale_code) {
        return Err(Error::InvalidBitstream(
            "encode_cbr_gop_sequence: initial_quantiser_scale_code out of range",
        ));
    }

    let frame_rate = frame_rate_value(cbr.frame_rate_code)?;
    let bit_rate = u64::from(cbr.bit_rate_value) * 400;
    let buffer_bits = u64::from(cbr.vbv_buffer_size_value) * 16 * 1024;
    let model = VbvCbrModel::new(&VbvParams {
        bit_rate,
        buffer_size_bits: buffer_bits,
        frame_rate,
    })?;

    // Nominal per-type budgets from the GOP composition: a full GOP
    // holds 1 I + anchors P + anchors * b_between B pictures over
    // (1 + anchors * (b_between + 1)) frame periods of rate.
    let step = b_between + 1;
    let gop_pictures = 1 + anchors_per_gop * step;
    let per_pic_bits = bit_rate * u64::from(frame_rate.1) / u64::from(frame_rate.0);
    let gop_bits = per_pic_bits * gop_pictures as u64;
    let weight_sum = TYPE_WEIGHT_X10[0]
        + TYPE_WEIGHT_X10[1] * anchors_per_gop as u64
        + TYPE_WEIGHT_X10[2] * (anchors_per_gop * b_between) as u64;
    let target_bits = [
        gop_bits * TYPE_WEIGHT_X10[0] / weight_sum,
        gop_bits * TYPE_WEIGHT_X10[1] / weight_sum,
        gop_bits * TYPE_WEIGHT_X10[2] / weight_sum,
    ];

    let sequence_params = SequenceHeaderParams {
        horizontal_size: params.width as u16,
        vertical_size: params.height as u16,
        frame_rate_code: cbr.frame_rate_code,
        bit_rate_value: cbr.bit_rate_value,
        vbv_buffer_size_value: cbr.vbv_buffer_size_value,
        ..Default::default()
    };
    let mut head = BitWriter::new();
    write_sequence_header(&mut head, &sequence_params);
    // §6.3.5/§6.3.3: the frame-picture encoders code the Ceil(h/16)
    // progressive grid, so the sequence declares progressive_sequence.
    write_sequence_extension(&mut head, params.chroma_format, params.progressive_sequence);

    let mut w = CbrWriter {
        out: head.finish(),
        model,
        frd: frame_rate.1,
        target_bits,
        q: [cbr.initial_quantiser_scale_code; 3],
        quantisers: Vec::new(),
        stuffing_bytes: 0,
        started: false,
        buffer_bits,
        bit_rate,
    };

    // The coded-order GOP walk mirrors
    // `encode_display_order_gop_sequence`.
    let total = frames.len();
    let mut coded = 0usize;
    let mut gop_start = 0usize;
    while gop_start < total {
        let gop_end = (gop_start + anchors_per_gop * step).min(total - 1);

        let mut gop_bw = BitWriter::new();
        write_gop_header(
            &mut gop_bw,
            &Mpeg2Gop {
                time_code: TimeCode::from_display_index(
                    gop_start as u64,
                    sequence_params.frame_rate_code,
                )?,
                closed_gop: true,
                broken_link: false,
            },
        );
        let gop_header = gop_bw.finish();

        // The GOP's I anchor.
        coded += 1;
        let is_last = coded == total;
        let i_frame = &frames[gop_start];
        let mut forward_ref = w
            .emit(&gop_header, PictureKind::I, is_last, |q| {
                let stream = crate::intra_encoder::encode_intra_picture(i_frame, params, 0, q)?;
                let pic_start = find_start(&stream, 0x0000_0100).ok_or(Error::InvalidBitstream(
                    "encode_cbr_gop_sequence: I picture start code missing",
                ))?;
                let layer = stream[pic_start..stream.len() - 4].to_vec();
                let recon = crate::decode_video_sequence(&stream)?
                    .first()
                    .map(|d| d.frame.clone())
                    .ok_or(Error::InvalidBitstream(
                        "encode_cbr_gop_sequence: I anchor decode produced no frame",
                    ))?;
                Ok((layer, Some(recon)))
            })?
            .expect("I emission returns a reconstruction");

        let mut prev_anchor = gop_start;
        while prev_anchor < gop_end {
            let next_anchor = (prev_anchor + step).min(gop_end);

            coded += 1;
            let is_last = coded == total;
            let p_frame = &frames[next_anchor];
            let fwd = forward_ref.clone();
            let backward_ref = w
                .emit(&[], PictureKind::P, is_last, |q| {
                    let mut bw = BitWriter::new();
                    let recon = crate::p_picture_encoder::encode_p_picture(
                        &mut bw,
                        p_frame,
                        &fwd,
                        params,
                        (next_anchor - gop_start) as u16,
                        q,
                        forward_f_code,
                    )?;
                    Ok((bw.finish(), Some(recon)))
                })?
                .expect("P emission returns a reconstruction");

            for b in (prev_anchor + 1)..next_anchor {
                coded += 1;
                let is_last = coded == total;
                let b_frame = &frames[b];
                let fwd = &forward_ref;
                let bwd = &backward_ref;
                w.emit(&[], PictureKind::B, is_last, |q| {
                    let mut bw = BitWriter::new();
                    crate::b_picture_encoder::encode_b_picture(
                        &mut bw,
                        b_frame,
                        fwd,
                        bwd,
                        params,
                        (b - gop_start) as u16,
                        q,
                        forward_f_code,
                        backward_f_code,
                    )?;
                    Ok((bw.finish(), None))
                })?;
            }

            forward_ref = backward_ref;
            prev_anchor = next_anchor;
        }

        gop_start = gop_end + 1;
    }

    Ok(CbrEncoded {
        stream: w.out,
        quantiser_scale_codes: w.quantisers,
        stuffing_bytes: w.stuffing_bytes,
    })
}

/// Find the first 4-byte big-endian `code` start-code in `buf`.
fn find_start(buf: &[u8], code: u32) -> Option<usize> {
    buf.windows(4).position(|w| {
        (u32::from(w[0]) << 24 | u32::from(w[1]) << 16 | u32::from(w[2]) << 8 | u32::from(w[3]))
            == code
    })
}
