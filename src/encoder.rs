//! Runtime [`oxideav_core::Encoder`] wiring for MPEG-1 Video
//! (ISO/IEC 11172-2) and MPEG-2 Video (ITU-T H.262 / ISO/IEC 13818-2).
//!
//! The crate's encode pipeline is driven at the whole-sequence level by
//! the display-order assemblers
//! ([`crate::encode_display_order_gop_sequence`] for MPEG-2 frame
//! pictures, [`crate::encode_mpeg1_display_order_sequence`] for
//! ISO/IEC 11172-2): the classic `I (B…) P (B…) P …` GOP structure in
//! which every anchor the encoder predicts from is the decoder's exact
//! reconstruction. This module adapts those assemblers to the
//! frame-to-packet [`oxideav_core::Encoder`] contract.
//!
//! # Framing model
//!
//! The §6.1.1.11 display-order reorder is defined over the whole coded
//! sequence — a display-order frame list cannot be emitted
//! incrementally without fixing the GOP structure around a lookahead
//! window. This adapter mirrors the runtime **decoder**'s
//! whole-elementary-stream framing ([`crate::decoder`]): it buffers
//! every [`Frame`] fed through [`Encoder::send_frame`] (display
//! order), runs the assembler once at [`Encoder::flush`], and emits
//! the finished elementary stream as **one keyframe-flagged
//! [`Packet`]**. Before the flush, [`Encoder::receive_packet`] returns
//! [`Error::NeedMore`]; after the drain it returns [`Error::Eof`].
//!
//! # Options
//!
//! The factory reads four optional integer knobs from
//! [`CodecParameters::options`]:
//!
//! | key | meaning | default | range |
//! |-----|---------|---------|-------|
//! | `quantiser_scale_code` | the per-slice §6.3.10 / §2.4.3.6 quantiser scale code | `6` | `1..=31` |
//! | `b_between` | B-pictures between anchors (display order) | `2` | `0..=` |
//! | `anchors_per_gop` | predictive periods per GOP | `4` | `1..=` |
//! | `f_code` | motion-vector `f_code` for both directions | `3` | `1..=7` |
//!
//! The emitted MPEG-2 sequences declare the assembler's defaults
//! (progressive, 25 frames/s nominal); the MPEG-1 sequences use the
//! §2.4.3.2 defaults with the constrained-parameters flag evaluated by
//! the assembler.

use std::collections::VecDeque;

use oxideav_core::{
    CodecId, CodecParameters, Encoder, Error, Frame, Packet, PixelFormat, Result, TimeBase,
    VideoFrame,
};

use crate::frame_assembly::{FrameBuffer, IntraPictureParams};
use crate::mpeg1_stream_writer::Mpeg1SequenceParams;
use crate::sequence_extension::ChromaFormat;
use crate::Error as Mpeg12Error;

/// Fold a crate-local [`Mpeg12Error`] into the framework
/// [`oxideav_core::Error`] the [`Encoder`] contract speaks (same
/// mapping as the runtime decoder's).
fn map_err(err: Mpeg12Error) -> Error {
    match err {
        Mpeg12Error::InvalidBitstream(detail) => {
            Error::invalid(format!("mpeg12video encoder: {detail}"))
        }
        Mpeg12Error::ShortHeader => Error::invalid("mpeg12video encoder: short header"),
        Mpeg12Error::NotImplemented => {
            Error::unsupported("mpeg12video encoder: not-yet-implemented syntax path")
        }
    }
}

/// Encoder tuning knobs parsed from [`CodecParameters::options`].
#[derive(Debug, Clone, Copy)]
struct EncoderConfig {
    quantiser_scale_code: u8,
    b_between: usize,
    anchors_per_gop: usize,
    f_code: u8,
}

/// Parse one integer option, surfacing a clean error on garbage.
fn parse_option<T: std::str::FromStr>(params: &CodecParameters, key: &str) -> Result<Option<T>> {
    match params.options.get(key) {
        None => Ok(None),
        Some(raw) => raw.parse::<T>().map(Some).map_err(|_| {
            Error::invalid(format!(
                "mpeg12video encoder: option '{key}' is not a valid integer: '{raw}'"
            ))
        }),
    }
}

impl EncoderConfig {
    fn from_params(params: &CodecParameters) -> Result<Self> {
        let quantiser_scale_code: u8 = parse_option(params, "quantiser_scale_code")?.unwrap_or(6);
        if !(1..=31).contains(&quantiser_scale_code) {
            return Err(Error::invalid(
                "mpeg12video encoder: quantiser_scale_code outside 1..=31",
            ));
        }
        let b_between: usize = parse_option(params, "b_between")?.unwrap_or(2);
        let anchors_per_gop: usize = parse_option(params, "anchors_per_gop")?.unwrap_or(4);
        if anchors_per_gop == 0 {
            return Err(Error::invalid(
                "mpeg12video encoder: anchors_per_gop must be >= 1",
            ));
        }
        let f_code: u8 = parse_option(params, "f_code")?.unwrap_or(3);
        if !(1..=7).contains(&f_code) {
            return Err(Error::invalid("mpeg12video encoder: f_code outside 1..=7"));
        }
        Ok(Self {
            quantiser_scale_code,
            b_between,
            anchors_per_gop,
            f_code,
        })
    }
}

/// Build a boxed [`Encoder`] for the codec id named by `params`.
///
/// Registered under both `"mpeg1video"` and `"mpeg2video"`; the id
/// selects the assembler (11172-2 vs 13818-2 syntax). `params` must be
/// a video parameter set with `width` / `height` present and a pixel
/// format of 4:2:0 planar (or unset — 4:2:0 is assumed).
///
/// # Errors
///
/// [`Error::invalid`] for a non-video parameter set, missing geometry,
/// a non-4:2:0 pixel format, or an out-of-range option value.
pub fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    let width = params
        .width
        .ok_or_else(|| Error::invalid("mpeg12video encoder: width missing"))?;
    let height = params
        .height
        .ok_or_else(|| Error::invalid("mpeg12video encoder: height missing"))?;
    if width == 0 || height == 0 || width > 4095 || height > 4095 {
        return Err(Error::invalid(
            "mpeg12video encoder: width/height outside the 12-bit sequence-header range",
        ));
    }
    match params.pixel_format {
        None | Some(PixelFormat::Yuv420P) => {}
        Some(other) => {
            return Err(Error::invalid(format!(
                "mpeg12video encoder: unsupported pixel format {other:?} (4:2:0 planar only)"
            )));
        }
    }
    let config = EncoderConfig::from_params(params)?;
    let is_mpeg1 = params.codec_id.as_str() == crate::decoder::MPEG1_CODEC_ID_STR;

    let mut output_params = CodecParameters::video(params.codec_id.clone());
    output_params.width = Some(width);
    output_params.height = Some(height);
    output_params.pixel_format = Some(PixelFormat::Yuv420P);
    output_params.frame_rate = params.frame_rate;

    Ok(Box::new(Mpeg12Encoder {
        codec_id: params.codec_id.clone(),
        output_params,
        width: width as usize,
        height: height as usize,
        is_mpeg1,
        config,
        frames: Vec::new(),
        ready: VecDeque::new(),
        flushed: false,
    }))
}

/// A frame-to-packet MPEG-1 / MPEG-2 video encoder.
///
/// See the [module docs](self) for the whole-stream framing model.
#[derive(Debug)]
pub struct Mpeg12Encoder {
    codec_id: CodecId,
    output_params: CodecParameters,
    width: usize,
    height: usize,
    is_mpeg1: bool,
    config: EncoderConfig,
    /// Buffered display-order input frames.
    frames: Vec<FrameBuffer>,
    /// The finished elementary stream, ready to drain.
    ready: VecDeque<Packet>,
    flushed: bool,
}

/// Convert a planar 4:2:0 [`VideoFrame`] into a [`FrameBuffer`],
/// honouring each plane's stride.
fn video_frame_to_frame_buffer(v: &VideoFrame, width: usize, height: usize) -> Result<FrameBuffer> {
    if v.planes.len() < 3 {
        return Err(Error::invalid(
            "mpeg12video encoder: expected 3 planar 4:2:0 planes",
        ));
    }
    let cw = width.div_ceil(2);
    let ch = height.div_ceil(2);
    let mut out = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    let copy = |plane: &oxideav_core::VideoPlane,
                dst: &mut crate::frame_assembly::Plane,
                w: usize,
                h: usize|
     -> Result<()> {
        if plane.stride < w || plane.data.len() < plane.stride * h {
            return Err(Error::invalid(
                "mpeg12video encoder: video plane smaller than the declared geometry",
            ));
        }
        for y in 0..h {
            let row = &plane.data[y * plane.stride..y * plane.stride + w];
            for (x, &sample) in row.iter().enumerate() {
                dst.put_sample(x, y, sample);
            }
        }
        Ok(())
    };
    copy(&v.planes[0], &mut out.y, width, height)?;
    copy(&v.planes[1], &mut out.cb, cw, ch)?;
    copy(&v.planes[2], &mut out.cr, cw, ch)?;
    Ok(out)
}

impl Mpeg12Encoder {
    /// Run the display-order assembler over the buffered frames and
    /// queue the finished elementary stream as one keyframe packet.
    fn encode_buffered(&mut self) -> Result<()> {
        if self.flushed {
            return Ok(());
        }
        self.flushed = true;
        if self.frames.is_empty() {
            return Ok(());
        }
        let stream = if self.is_mpeg1 {
            let seq = Mpeg1SequenceParams {
                horizontal_size: self.width as u16,
                vertical_size: self.height as u16,
                ..Default::default()
            };
            crate::mpeg1_encoder::encode_mpeg1_display_order_sequence(
                &self.frames,
                self.config.b_between,
                self.config.anchors_per_gop,
                &seq,
                self.config.quantiser_scale_code,
                self.config.f_code,
                self.config.f_code,
            )
            .map_err(map_err)?
        } else {
            let params = IntraPictureParams {
                width: self.width,
                height: self.height,
                chroma_format: ChromaFormat::Yuv420,
                frame_pred_frame_dct: true,
                intra_dc_precision: 0,
                intra_vlc_format: false,
                alternate_scan: false,
                q_scale_type: false,
                progressive_sequence: true,
            };
            crate::inter_encoder::encode_display_order_gop_sequence(
                &self.frames,
                self.config.b_between,
                self.config.anchors_per_gop,
                params,
                self.config.quantiser_scale_code,
                self.config.f_code,
                self.config.f_code,
            )
            .map_err(map_err)?
        };
        let frame_count = self.frames.len() as i64;
        self.frames.clear();
        let mut packet = Packet::new(0, TimeBase::new(1, 25), stream);
        packet.pts = Some(0);
        packet.dts = Some(0);
        packet.duration = Some(frame_count);
        packet.flags.keyframe = true;
        self.ready.push_back(packet);
        Ok(())
    }
}

impl Encoder for Mpeg12Encoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn output_params(&self) -> &CodecParameters {
        &self.output_params
    }

    fn send_frame(&mut self, frame: &Frame) -> Result<()> {
        if self.flushed {
            return Err(Error::invalid(
                "mpeg12video encoder: send_frame after flush",
            ));
        }
        let Frame::Video(v) = frame else {
            return Err(Error::invalid(
                "mpeg12video encoder: expected a video frame",
            ));
        };
        let fb = video_frame_to_frame_buffer(v, self.width, self.height)?;
        self.frames.push(fb);
        Ok(())
    }

    fn receive_packet(&mut self) -> Result<Packet> {
        if let Some(packet) = self.ready.pop_front() {
            return Ok(packet);
        }
        if self.flushed {
            return Err(Error::Eof);
        }
        Err(Error::NeedMore)
    }

    fn flush(&mut self) -> Result<()> {
        self.encode_buffered()
    }
}
