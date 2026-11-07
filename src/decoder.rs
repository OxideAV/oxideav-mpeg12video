//! Runtime [`oxideav_core::Decoder`] wiring for MPEG-1 Video
//! (ISO/IEC 11172-2) and MPEG-2 Video (ITU-T H.262 / ISO/IEC 13818-2).
//!
//! The pixel-reconstruction pipeline is driven, at the elementary-stream
//! level, by [`crate::video_sequence::SequenceDecoder`]: it parses the
//! sequence layer for geometry, walks every `picture_start_code`,
//! reconstructs each picture with the matching per-picture driver, and
//! reorders the results into §6.1.1.11 display order. This module adapts
//! that incremental driver to the packet-oriented
//! [`oxideav_core::Decoder`] contract.
//!
//! # Framing model
//!
//! An MPEG-1/2 video elementary stream is a single concatenated
//! bitstream: the §6.1.1.11 display reorder is defined over the *whole*
//! reconstructed sequence (a B-picture cannot be emitted until the
//! anchor that follows it in coded order has been decoded), and a
//! `sequence_header()` establishes geometry that every following picture
//! inherits. There is no self-delimiting per-frame unit at the codec
//! layer — the container above is responsible for any packetisation.
//!
//! This decoder therefore feeds the payload of every [`Packet`] into one
//! [`SequenceDecoder`], which treats the packets as one contiguous
//! elementary stream. A picture is reconstructed once the start code that
//! ends it has arrived, and [`Decoder::receive_frame`] returns each frame
//! as soon as the reorder commits it; until then it returns
//! [`Error::NeedMore`]. Memory stays bounded by one coded picture plus the
//! reference anchors, however long the stream. [`Decoder::flush`] marks
//! the end of the stream, releasing the last picture and the held-back
//! anchor; once those drain the decoder returns [`Error::Eof`].
//!
//! # Codec-id coverage
//!
//! Both `"mpeg1video"` and `"mpeg2video"` are registered and route
//! through the same [`SequenceDecoder`] driver. That driver
//! resolves the picture geometry and `f_code`s from the MPEG-2 extension
//! family (`sequence_extension()` / `picture_coding_extension()`), so it
//! decodes the MPEG-2 bitstream (and the MPEG-1-compatible subset a
//! §13818-2 encoder emits) end-to-end. A *pure* ISO/IEC 11172-2 (MPEG-1)
//! elementary stream, which carries no extension start codes, is not yet
//! driven at the whole-stream level — the MPEG-1 macroblock-layer
//! machinery exists per-stage but the picture driver is a later
//! milestone — so such a stream currently surfaces a clean parse error
//! rather than frames. The registration reserves the `mpeg1video` id for
//! that path.
//!
//! Each reconstructed [`crate::frame_assembly::FrameBuffer`] is converted
//! to a planar Y/Cb/Cr [`VideoFrame`] (three [`VideoPlane`]s, each
//! tightly packed at `stride == plane width`); the stream's pixel format
//! (4:2:0 / 4:2:2 / 4:4:4) lives on the caller's
//! [`oxideav_core::CodecParameters`], not per-frame. Frames are stamped
//! with a monotonically increasing display-order presentation index so a
//! caller that muxes the output keeps the display ordering the driver
//! already resolved.

use std::collections::VecDeque;

use oxideav_core::{
    CodecId, CodecParameters, Decoder, Error, Frame, Packet, Result, VideoFrame, VideoPlane,
};

use crate::Error as Mpeg12Error;
use crate::frame_assembly::FrameBuffer;
use crate::video_sequence::SequenceDecoder;

/// Fold a crate-local [`Mpeg12Error`] into the framework
/// [`oxideav_core::Error`] the [`Decoder`] contract speaks. A bitstream
/// constraint violation or a truncated stream is an
/// [`Error::invalid`](oxideav_core::Error::invalid); a spec-defined but
/// not-yet-implemented syntax path is an
/// [`Error::unsupported`](oxideav_core::Error::unsupported).
fn map_err(err: Mpeg12Error) -> Error {
    match err {
        Mpeg12Error::InvalidBitstream(detail) => {
            Error::invalid(format!("mpeg12video: invalid bitstream: {detail}"))
        }
        Mpeg12Error::ShortHeader => {
            Error::invalid("mpeg12video: short header (unexpected end of elementary stream)")
        }
        Mpeg12Error::NotImplemented => {
            Error::unsupported("mpeg12video: bitstream uses a not-yet-implemented syntax path")
        }
    }
}

/// Codec id string for MPEG-1 video (ISO/IEC 11172-2).
pub const MPEG1_CODEC_ID_STR: &str = "mpeg1video";

/// Codec id string for MPEG-2 video (ITU-T H.262 / ISO/IEC 13818-2).
pub const MPEG2_CODEC_ID_STR: &str = "mpeg2video";

/// Build a boxed [`Decoder`] for the codec id named by `params`.
///
/// Registered under both `"mpeg1video"` and `"mpeg2video"`; the same
/// driver decodes either — MPEG-1 is the subset the §2 syntax picks out
/// of the shared bitstream, and the per-picture reconstruction dispatch
/// keys off the parsed sequence layer, not off the codec id. The codec
/// id is preserved so [`Decoder::codec_id`] reports it back verbatim.
///
/// # Errors
///
/// Never fails at construction — the decoder validates the bitstream
/// lazily at [`Decoder::flush`] time.
pub fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    Ok(Box::new(Mpeg12Decoder::new(params.codec_id.clone())))
}

/// A packet-to-frame MPEG-1 / MPEG-2 video decoder.
///
/// See the [module docs](self) for the streaming framing model.
#[derive(Debug)]
pub struct Mpeg12Decoder {
    codec_id: CodecId,
    /// The incremental `video_sequence()` driver fed by every packet.
    sequence: SequenceDecoder,
    /// Whether any elementary-stream bytes have arrived; an empty stream
    /// flushes to [`Error::Eof`] rather than a missing-header error.
    received: bool,
    /// Reconstructed frames in §6.1.1.11 display order, ready to drain.
    ready: VecDeque<VideoFrame>,
    /// Set by [`Decoder::flush`]: no more input will arrive.
    flushed: bool,
    /// Monotonic display-order presentation index stamped onto frames.
    next_pts: i64,
}

impl Mpeg12Decoder {
    /// Create an empty decoder that reports `codec_id`.
    pub fn new(codec_id: CodecId) -> Self {
        Self {
            codec_id,
            sequence: SequenceDecoder::new(),
            received: false,
            ready: VecDeque::new(),
            flushed: false,
            next_pts: 0,
        }
    }

    /// Move every frame the driver can commit so far into `ready`,
    /// stamping display-order presentation indices.
    fn drain_sequence(&mut self) -> Result<()> {
        if !self.received {
            return Ok(());
        }
        while let Some(decoded) = self.sequence.next_frame().map_err(map_err)? {
            let mut vf = frame_buffer_to_video_frame(&decoded.frame);
            vf.pts = Some(self.next_pts);
            self.next_pts += 1;
            self.ready.push_back(vf);
        }
        Ok(())
    }
}

impl Decoder for Mpeg12Decoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> Result<()> {
        self.received |= !packet.data.is_empty();
        self.sequence.push(&packet.data);
        self.drain_sequence()
    }

    fn receive_frame(&mut self) -> Result<Frame> {
        match self.ready.pop_front() {
            Some(vf) => Ok(Frame::Video(vf)),
            None if self.flushed => Err(Error::Eof),
            None => Err(Error::NeedMore),
        }
    }

    fn flush(&mut self) -> Result<()> {
        if self.flushed {
            return Ok(());
        }
        self.flushed = true;
        self.sequence.finish();
        self.drain_sequence()
    }

    fn reset(&mut self) -> Result<()> {
        *self = Self::new(self.codec_id.clone());
        Ok(())
    }
}

/// Convert a reconstructed [`FrameBuffer`] into a planar Y/Cb/Cr
/// [`VideoFrame`].
///
/// Each component becomes one tightly-packed [`VideoPlane`]
/// (`stride == visible plane width`). Plane order is Y, Cb, Cr — the
/// natural order for every YUV [`oxideav_core::PixelFormat`]. The
/// reconstruction storage covers the full macroblock grid (the §7.6
/// reference area); the display output crops each plane to the visible
/// `width × height` (luma) / [`FrameBuffer::visible_chroma_dims`]
/// (chroma) rectangle.
pub fn frame_buffer_to_video_frame(frame: &FrameBuffer) -> VideoFrame {
    let (cw, ch) = frame.visible_chroma_dims();
    let plane = |plane: &crate::frame_assembly::Plane, w: usize, h: usize| VideoPlane {
        stride: w,
        data: plane.packed_rect(w, h),
    };
    VideoFrame {
        pts: None,
        planes: vec![
            plane(&frame.y, frame.width, frame.height),
            plane(&frame.cb, cw, ch),
            plane(&frame.cr, cw, ch),
        ],
    }
}
