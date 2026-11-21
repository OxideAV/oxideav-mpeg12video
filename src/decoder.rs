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
//! elementary stream. A picture can be reconstructed once the start code
//! that ends it has arrived; [`Decoder::receive_frame`] decodes on demand
//! and returns each frame as soon as the reorder commits it (bitstream
//! errors surface there too), [`Error::NeedMore`] when the input so far
//! commits none. Memory stays bounded by one coded picture plus the
//! reference anchors, however long the stream. [`Decoder::flush`] marks
//! the end of the stream, releasing the last picture and the held-back
//! anchor; once those drain the decoder returns [`Error::Eof`].
//!
//! # Codec-id coverage
//!
//! Both `"mpeg1video"` and `"mpeg2video"` are registered and route
//! through the same [`SequenceDecoder`] driver, which classifies each
//! sequence by its syntax (a `sequence_extension()` makes it ISO/IEC
//! 13818-2, its absence ISO/IEC 11172-2) — not by the codec id — so a
//! stream may even switch between the two at a sequence boundary.
//!
//! # Output shape, timing and damage
//!
//! Each reconstructed [`crate::frame_assembly::FrameBuffer`] is
//! converted to a planar Y/Cb/Cr [`VideoFrame`] (three
//! [`VideoPlane`]s, each tightly packed at `stride == plane width`).
//! The picture size, pixel format (4:2:0 / 4:2:2 / 4:4:4) and frame
//! rate are stream parameters, not per-frame fields: once the first
//! frame has been reconstructed [`Decoder::output_params`] states
//! them, and when a later `sequence_header()` changes the shape the
//! decoder returns a [`Frame::StreamParams`] event just before the
//! first frame of the new shape and advances
//! [`Decoder::output_params_generation`].
//!
//! A frame carries the presentation time of the packet that held its
//! `picture_start_code` (the first field's for a field pair), so the
//! container's times survive the §6.1.1.11 reorder; when no packet
//! carries a time (a raw elementary stream fed in arbitrary pieces),
//! frames are numbered in display order instead.
//!
//! By default the decoder conceals damage the way a player needs: a
//! picture that fails to decode is replaced by a copy of its nearest
//! reference, and B-pictures without a forward reference (the leading
//! pictures of an open GOP when decoding starts mid-stream) are
//! dropped. The option `strict = true`
//! ([`Mpeg12DecoderOptions::strict`]) makes every such picture an
//! error instead.

use std::collections::VecDeque;

use oxideav_core::{
    CodecId, CodecOptionsStruct, CodecParameters, Decoder, Error, Frame, OptionField, OptionKind,
    OptionValue, Packet, PixelFormat, Rational, Result, VideoFrame, VideoPlane, parse_options,
};

use crate::Error as Mpeg12Error;
use crate::frame_assembly::FrameBuffer;
use crate::sequence_extension::ChromaFormat;
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

/// Options of the runtime decoder, parsed from
/// [`CodecParameters::options`] by [`make_decoder`].
///
/// | key | kind | default | meaning |
/// |-----|------|---------|---------|
/// | `strict` | bool | `false` | fail on a damaged or unreferenced picture instead of concealing it |
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub struct Mpeg12DecoderOptions {
    /// Return an error for a picture that cannot be decoded instead of
    /// concealing it (see the [module docs](self)).
    pub strict: bool,
}

impl Mpeg12DecoderOptions {
    /// Set `strict`.
    #[must_use]
    pub fn with_strict(mut self, v: bool) -> Self {
        self.strict = v;
        self
    }
}

impl CodecOptionsStruct for Mpeg12DecoderOptions {
    const SCHEMA: &'static [OptionField] = &[OptionField {
        name: "strict",
        kind: OptionKind::Bool,
        default: OptionValue::Bool(false),
        help: "fail on a damaged or unreferenced picture instead of concealing it",
    }];

    fn apply(&mut self, key: &str, value: &OptionValue) -> Result<()> {
        match key {
            "strict" => self.strict = value.as_bool()?,
            other => {
                return Err(Error::invalid(format!(
                    "mpeg12video decoder: unknown option '{other}'"
                )));
            }
        }
        Ok(())
    }
}

/// Build a boxed [`Decoder`] for the codec id named by `params`.
///
/// Registered under both `"mpeg1video"` and `"mpeg2video"`; the same
/// driver decodes either, keyed off the parsed sequence layer rather
/// than the codec id, which is preserved so [`Decoder::codec_id`]
/// reports it back verbatim. `params.options` is parsed against
/// [`Mpeg12DecoderOptions::SCHEMA`].
///
/// # Errors
///
/// [`Error::invalid`] for a malformed value of one of this decoder's
/// options (other keys are ignored). The bitstream is validated as it
/// is decoded: its errors come from [`Decoder::receive_frame`].
pub fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    // The parameter set of a stream often carries its encoder's
    // options too: only this decoder's own keys are read.
    let mut own = oxideav_core::CodecOptions::default();
    for (key, value) in params.options.iter() {
        if Mpeg12DecoderOptions::SCHEMA.iter().any(|f| f.name == key) {
            own.insert(key, value);
        }
    }
    let options: Mpeg12DecoderOptions = parse_options(&own)?;
    Ok(Box::new(Mpeg12Decoder::with_options(
        params.codec_id.clone(),
        options,
    )))
}

/// The output shape the stream parameters describe.
type Shape = (usize, usize, ChromaFormat);

/// How many packet times are remembered for frames still inside the
/// decoder (a GOP's reorder depth is a handful of pictures).
const PACKET_TIME_HISTORY: usize = 512;

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
    /// Frames in §6.1.1.11 display order (and the stream-parameter
    /// events between them), ready to drain.
    ready: VecDeque<Frame>,
    /// Set by [`Decoder::flush`]: no more input will arrive.
    flushed: bool,
    /// Display-order index stamped onto frames when no packet carries
    /// a presentation time.
    next_pts: i64,
    /// Elementary-stream bytes pushed so far (the stream offset of the
    /// next packet's first byte).
    pushed: u64,
    /// `(stream offset of the packet's first byte, its presentation
    /// time, already given to a frame)`, oldest first.
    packet_times: VecDeque<(u64, i64, bool)>,
    /// Whether any packet carried a presentation time.
    timed: bool,
    /// Shape of the frames returned so far.
    shape: Option<Shape>,
    /// The parameters [`Decoder::output_params`] reports.
    params: Option<CodecParameters>,
    generation: u64,
}

impl Mpeg12Decoder {
    /// Create an empty decoder that reports `codec_id`, with the
    /// default options.
    pub fn new(codec_id: CodecId) -> Self {
        Self::with_options(codec_id, Mpeg12DecoderOptions::default())
    }

    /// Create an empty decoder that reports `codec_id`.
    pub fn with_options(codec_id: CodecId, options: Mpeg12DecoderOptions) -> Self {
        Self {
            codec_id,
            sequence: Self::driver(options),
            received: false,
            ready: VecDeque::new(),
            flushed: false,
            next_pts: 0,
            pushed: 0,
            packet_times: VecDeque::new(),
            timed: false,
            shape: None,
            params: None,
            generation: 0,
        }
    }

    fn driver(options: Mpeg12DecoderOptions) -> SequenceDecoder {
        if options.strict {
            SequenceDecoder::new()
        } else {
            SequenceDecoder::with_concealment()
        }
    }

    /// The presentation time for a frame whose picture start code sits
    /// at `offset`: that of the packet holding it, given out once.
    fn time_for(&mut self, offset: Option<u64>) -> Option<i64> {
        if !self.timed {
            let pts = self.next_pts;
            self.next_pts += 1;
            return Some(pts);
        }
        let offset = offset?;
        let after = self
            .packet_times
            .partition_point(|&(start, _, _)| start <= offset);
        let entry = self.packet_times.get_mut(after.checked_sub(1)?)?;
        if entry.2 {
            return None;
        }
        entry.2 = true;
        Some(entry.1)
    }

    /// The stream parameters for frames of `shape`.
    fn params_for(&self, shape: Shape) -> CodecParameters {
        let mut params = CodecParameters::video(self.codec_id.clone());
        params.width = Some(shape.0 as u32);
        params.height = Some(shape.1 as u32);
        params.pixel_format = Some(match shape.2 {
            ChromaFormat::Yuv420 => PixelFormat::Yuv420P,
            ChromaFormat::Yuv422 => PixelFormat::Yuv422P,
            ChromaFormat::Yuv444 => PixelFormat::Yuv444P,
        });
        params.frame_rate = self
            .sequence
            .frame_rate()
            .map(|(n, d)| Rational::new(i64::from(n), i64::from(d)));
        params
    }

    /// Decode until the driver commits a frame (or runs out of input)
    /// and queue it. One frame at a time, on demand: the input may
    /// hold any number of pictures — a damaged or hostile stream can
    /// spend four bytes on each — and none is reconstructed before the
    /// caller asks for it.
    fn pull_frame(&mut self) -> Result<()> {
        if !self.received {
            return Ok(());
        }
        if let Some(decoded) = self.sequence.next_frame().map_err(map_err)? {
            let shape = (
                decoded.frame.width,
                decoded.frame.height,
                decoded.frame.chroma_format,
            );
            if self.shape != Some(shape) {
                let params = self.params_for(shape);
                if self.shape.is_some() {
                    self.generation += 1;
                    self.ready
                        .push_back(Frame::StreamParams(Box::new(params.clone())));
                }
                self.shape = Some(shape);
                self.params = Some(params);
            }
            let mut vf = frame_buffer_to_video_frame(&decoded.frame);
            vf.pts = self.time_for(decoded.stream_offset);
            self.ready.push_back(Frame::Video(vf));
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
        if let Some(pts) = packet.pts {
            self.timed = true;
            if self.packet_times.len() == PACKET_TIME_HISTORY {
                self.packet_times.pop_front();
            }
            self.packet_times.push_back((self.pushed, pts, false));
        }
        self.pushed += packet.data.len() as u64;
        self.sequence.push(&packet.data);
        Ok(())
    }

    fn receive_frame(&mut self) -> Result<Frame> {
        if self.ready.is_empty() {
            self.pull_frame()?;
        }
        match self.ready.pop_front() {
            Some(frame) => Ok(frame),
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
        Ok(())
    }

    fn reset(&mut self) -> Result<()> {
        // A seek: forget the buffered bytes, the reference pictures
        // and the pending output, keep the sequence layer (the stream
        // may resume at a GOP without repeating its sequence header)
        // and the stream parameters already reported.
        self.sequence.reset();
        self.ready.clear();
        self.packet_times.clear();
        self.flushed = false;
        self.next_pts = 0;
        Ok(())
    }

    fn set_execution_context(&mut self, ctx: &oxideav_core::ExecutionContext) {
        self.sequence.set_threads(ctx.threads);
    }

    fn output_pixel_format(&self) -> Option<PixelFormat> {
        self.params.as_ref().and_then(|p| p.pixel_format)
    }

    fn output_params(&self) -> Option<CodecParameters> {
        self.params.clone()
    }

    fn output_params_generation(&self) -> u64 {
        self.generation
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
