//! Runtime [`oxideav_core::Encoder`] wiring for MPEG-1 Video
//! (ISO/IEC 11172-2) and MPEG-2 Video (ITU-T H.262 / ISO/IEC 13818-2).
//!
//! The crate's encode pipeline is driven at the whole-sequence level by
//! the display-order assemblers — the frame-picture GOP assembler
//! ([`crate::encode_display_order_gop_sequence_with_options`]), the
//! field-picture assemblers
//! ([`crate::encode_field_display_order_gop_sequence`] /
//! [`crate::encode_field_adaptive_display_order_gop_sequence`]), the
//! frame-picture field-based assembler
//! ([`crate::encode_ff_display_order_gop_sequence`]), the Annex C CBR
//! controllers ([`crate::encode_cbr_gop_sequence`] /
//! [`crate::encode_field_cbr_gop_sequence`] /
//! [`crate::encode_mpeg1_cbr_sequence`]) and the ISO/IEC 11172-2
//! assemblers ([`crate::encode_mpeg1_display_order_sequence`] /
//! [`crate::encode_mpeg1_d_sequence`]): the classic `I (B…) P (B…) P …`
//! GOP structure in which every anchor the encoder predicts from is
//! the decoder's exact reconstruction. This module adapts those
//! assemblers to the frame-to-packet [`oxideav_core::Encoder`]
//! contract and exposes their whole surface through the **typed
//! option schema** [`Mpeg12EncoderOptions`].
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
//! [`Packet`]** (two packets — partition 0 then partition 1 — under
//! `data_partitioning`). Before the flush, [`Encoder::receive_packet`]
//! returns [`Error::NeedMore`]; after the drain it returns
//! [`Error::Eof`].
//!
//! # Options
//!
//! [`CodecParameters::options`] is parsed against
//! [`Mpeg12EncoderOptions::SCHEMA`] with
//! [`oxideav_core::parse_options`] — unknown keys and malformed
//! values are rejected at construction, every value is range-checked,
//! and combinations the assemblers cannot honour surface as
//! [`Error::Unsupported`]. The pixel format of the parameter set
//! selects the chroma format (`Yuv420P` / `Yuv422P` / `Yuv444P`; unset
//! means 4:2:0); [`CodecParameters::frame_rate`] selects the Table 6-4
//! `frame_rate_code` (25 frames/s when unset) and
//! [`CodecParameters::bit_rate`] the CBR `bit_rate_value` when the
//! option leaves it at `0`.
//!
//! | key | kind | default | meaning |
//! |-----|------|---------|---------|
//! | `quantiser_scale_code` | u32 `1..=31` | `6` | the per-slice §6.3.16 / §2.4.3.6 quantiser scale code (the starting value under `cbr`) |
//! | `b_between` | u32 | `2` | B-pictures between anchors (display order) |
//! | `anchors_per_gop` | u32 `>= 1` | `4` | predictive periods per GOP |
//! | `f_code` | u32 `1..=7` (MPEG-2: `1..=9`) | `3` | forward motion-vector `f_code` |
//! | `backward_f_code` | u32 | `0` | backward `f_code`; `0` = same as `f_code` |
//! | `picture_structure` | `frame` / `field` / `frame_field` / `field_adaptive` | `frame` | frame pictures (`frame_pred_frame_dct = 1`), §6.1.1.4.1 field-picture pairs, frame pictures with per-macroblock field prediction / field DCT (`frame_pred_frame_dct = 0`), or field pictures with the full Table 6-18 mode set (simple / 16×8 / dual-prime) |
//! | `interlaced` | bool | `false` | `progressive_sequence = 0` for `frame` pictures (implied by the other structures) |
//! | `intra_vlc_format` | bool | `false` | Table B-15 intra AC coding (MPEG-2) |
//! | `alternate_scan` | bool | `false` | §7.3 alternate scan (MPEG-2) |
//! | `q_scale_type` | bool | `false` | Table 7-6 non-linear quantiser scale (MPEG-2) |
//! | `intra_dc_precision` | u32 `0..=3` | `0` | 8..=11-bit intra DC (MPEG-2) |
//! | `skipped_macroblocks` | bool | `false` | §7.6.6 skipped-macroblock emission (`frame` pictures) |
//! | `concealment_motion_vectors` | bool | `false` | §7.6.3.9 concealment vectors on intra macroblocks (`frame` pictures) |
//! | `top_field_first` | bool | `false` | §6.3.10 `top_field_first` on every frame picture |
//! | `repeat_first_field` | bool | `false` | §6.3.10 `repeat_first_field` on every frame picture |
//! | `pulldown` | `none` / `3:2` | `none` | the classic 3:2 pulldown cadence ([`FrameEncodeOptions::pulldown_32`]) over an `interlaced` `frame` sequence |
//! | `dual_prime` | bool | `false` | allow §7.6.3.6 dual-prime macroblocks (`frame_field` / `field_adaptive`, needs `b_between = 0`) |
//! | `rate_control` | `constant_quantiser` / `cbr` | `constant_quantiser` | Annex C VBV-regulated constant bit rate (`frame` / `field` structures and MPEG-1) |
//! | `bit_rate_value` | u32 `1..=0x3FFFE` | `0` | §6.2.2.1 `bit_rate_value` (units of 400 bit/s) under `cbr`; `0` = from [`CodecParameters::bit_rate`], else 1 Mbit/s |
//! | `vbv_buffer_size_value` | u32 `1..=0x3FF` | `0` | §6.2.2.1 `vbv_buffer_size_value` (units of 16 kbit) under `cbr`; `0` = 20, shrunk at low rates so a full buffer's `vbv_delay` fits 16 bits |
//! | `data_partitioning` | u32 `0`, `1..=3`, `64..=127` | `0` | §7.10 `priority_breakpoint`: split the MPEG-2 stream into two partition packets |
//! | `mpeg1_d_pictures` | bool | `false` | ISO/IEC 11172-2 §2.4.3.4 D-only sequence (`mpeg1video`) |
//!
//! Options that only exist on one syntax are rejected on the other
//! (`intra_vlc_format` and friends on `mpeg1video`, `mpeg1_d_pictures`
//! on `mpeg2video`), as are structure / option combinations no
//! assembler implements (`cbr` with `frame_field` / `field_adaptive`,
//! `skipped_macroblocks` / `concealment_motion_vectors` / `pulldown`
//! outside `frame` pictures, `dual_prime` with `frame` / `field`).

use std::collections::VecDeque;

use oxideav_core::{
    parse_options, CodecId, CodecOptionsStruct, CodecParameters, Encoder, Error, Frame,
    OptionField, OptionKind, OptionValue, Packet, PixelFormat, Result, TimeBase, VideoFrame,
};

use crate::encode_options::FrameEncodeOptions;
use crate::frame_assembly::{FrameBuffer, IntraPictureParams};
use crate::mpeg1_stream_writer::Mpeg1SequenceParams;
use crate::quant_matrix_extension::QuantMatrixExtension;
use crate::rate_control::CbrConfig;
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

/// `picture_structure` option values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PictureStructureOption {
    /// Frame pictures with `frame_pred_frame_dct = 1`.
    #[default]
    Frame,
    /// §6.1.1.4.1 field-picture pairs (simple field prediction).
    Field,
    /// Frame pictures with `frame_pred_frame_dct = 0` (per-macroblock
    /// frame / field prediction, field DCT, optional dual-prime).
    FrameField,
    /// Field-picture pairs with the full Table 6-18 mode set (simple
    /// / 16×8 / dual-prime).
    FieldAdaptive,
}

/// `rate_control` option values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RateControlOption {
    /// One `quantiser_scale_code` for every slice.
    #[default]
    ConstantQuantiser,
    /// Annex C VBV-regulated constant bit rate.
    Cbr,
}

/// `pulldown` option values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PulldownOption {
    /// Constant `top_field_first` / `repeat_first_field`.
    #[default]
    None,
    /// The 3:2 cadence of [`FrameEncodeOptions::pulldown_32`].
    ThreeTwo,
}

/// The typed option schema of the runtime encoder — see the
/// [module docs](self) for every key. Build it directly for a typed
/// caller, or let [`make_encoder`] parse it from
/// [`CodecParameters::options`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Mpeg12EncoderOptions {
    /// `quantiser_scale_code` (`1..=31`).
    pub quantiser_scale_code: u32,
    /// `b_between`.
    pub b_between: u32,
    /// `anchors_per_gop` (`>= 1`).
    pub anchors_per_gop: u32,
    /// `f_code` — forward.
    pub f_code: u32,
    /// `backward_f_code`; `0` = same as `f_code`.
    pub backward_f_code: u32,
    /// `picture_structure`.
    pub picture_structure: PictureStructureOption,
    /// `interlaced`.
    pub interlaced: bool,
    /// `intra_vlc_format`.
    pub intra_vlc_format: bool,
    /// `alternate_scan`.
    pub alternate_scan: bool,
    /// `q_scale_type`.
    pub q_scale_type: bool,
    /// `intra_dc_precision` (`0..=3`).
    pub intra_dc_precision: u32,
    /// `skipped_macroblocks`.
    pub skipped_macroblocks: bool,
    /// `concealment_motion_vectors`.
    pub concealment_motion_vectors: bool,
    /// `top_field_first`.
    pub top_field_first: bool,
    /// `repeat_first_field`.
    pub repeat_first_field: bool,
    /// `pulldown`.
    pub pulldown: PulldownOption,
    /// `dual_prime`.
    pub dual_prime: bool,
    /// `rate_control`.
    pub rate_control: RateControlOption,
    /// `bit_rate_value`; `0` = derive.
    pub bit_rate_value: u32,
    /// `vbv_buffer_size_value`; `0` = default.
    pub vbv_buffer_size_value: u32,
    /// `data_partitioning` (`priority_breakpoint`; `0` = off).
    pub data_partitioning: u32,
    /// `mpeg1_d_pictures`.
    pub mpeg1_d_pictures: bool,
}

impl Default for Mpeg12EncoderOptions {
    fn default() -> Self {
        Self {
            quantiser_scale_code: 6,
            b_between: 2,
            anchors_per_gop: 4,
            f_code: 3,
            backward_f_code: 0,
            picture_structure: PictureStructureOption::Frame,
            interlaced: false,
            intra_vlc_format: false,
            alternate_scan: false,
            q_scale_type: false,
            intra_dc_precision: 0,
            skipped_macroblocks: false,
            concealment_motion_vectors: false,
            top_field_first: false,
            repeat_first_field: false,
            pulldown: PulldownOption::None,
            dual_prime: false,
            rate_control: RateControlOption::ConstantQuantiser,
            bit_rate_value: 0,
            vbv_buffer_size_value: 0,
            data_partitioning: 0,
            mpeg1_d_pictures: false,
        }
    }
}

const PICTURE_STRUCTURE_VALUES: &[&str] = &["frame", "field", "frame_field", "field_adaptive"];
const RATE_CONTROL_VALUES: &[&str] = &["constant_quantiser", "cbr"];
const PULLDOWN_VALUES: &[&str] = &["none", "3:2"];

impl CodecOptionsStruct for Mpeg12EncoderOptions {
    const SCHEMA: &'static [OptionField] = &[
        OptionField {
            name: "quantiser_scale_code",
            kind: OptionKind::U32,
            default: OptionValue::U32(6),
            help: "per-slice quantiser_scale_code (1..=31); the starting value under cbr",
        },
        OptionField {
            name: "b_between",
            kind: OptionKind::U32,
            default: OptionValue::U32(2),
            help: "B-pictures between anchors in display order",
        },
        OptionField {
            name: "anchors_per_gop",
            kind: OptionKind::U32,
            default: OptionValue::U32(4),
            help: "predictive periods per GOP (>= 1)",
        },
        OptionField {
            name: "f_code",
            kind: OptionKind::U32,
            default: OptionValue::U32(3),
            help: "forward motion-vector f_code (1..=7; MPEG-2 1..=9)",
        },
        OptionField {
            name: "backward_f_code",
            kind: OptionKind::U32,
            default: OptionValue::U32(0),
            help: "backward motion-vector f_code; 0 = same as f_code",
        },
        OptionField {
            name: "picture_structure",
            kind: OptionKind::Enum(PICTURE_STRUCTURE_VALUES),
            default: OptionValue::String(String::new()),
            help: "frame | field | frame_field | field_adaptive",
        },
        OptionField {
            name: "interlaced",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "progressive_sequence = 0 for frame pictures",
        },
        OptionField {
            name: "intra_vlc_format",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "Table B-15 intra AC coding (MPEG-2)",
        },
        OptionField {
            name: "alternate_scan",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "alternate coefficient scan (MPEG-2)",
        },
        OptionField {
            name: "q_scale_type",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "non-linear quantiser scale (MPEG-2)",
        },
        OptionField {
            name: "intra_dc_precision",
            kind: OptionKind::U32,
            default: OptionValue::U32(0),
            help: "intra DC precision 0..=3 = 8..=11 bits (MPEG-2)",
        },
        OptionField {
            name: "skipped_macroblocks",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "emit skipped macroblocks (frame pictures)",
        },
        OptionField {
            name: "concealment_motion_vectors",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "code concealment motion vectors on intra macroblocks (frame pictures)",
        },
        OptionField {
            name: "top_field_first",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "top_field_first on every frame picture",
        },
        OptionField {
            name: "repeat_first_field",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "repeat_first_field on every frame picture",
        },
        OptionField {
            name: "pulldown",
            kind: OptionKind::Enum(PULLDOWN_VALUES),
            default: OptionValue::String(String::new()),
            help: "none | 3:2 (interlaced frame pictures)",
        },
        OptionField {
            name: "dual_prime",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "allow dual-prime macroblocks (frame_field / field_adaptive, b_between = 0)",
        },
        OptionField {
            name: "rate_control",
            kind: OptionKind::Enum(RATE_CONTROL_VALUES),
            default: OptionValue::String(String::new()),
            help: "constant_quantiser | cbr",
        },
        OptionField {
            name: "bit_rate_value",
            kind: OptionKind::U32,
            default: OptionValue::U32(0),
            help: "bit_rate_value in units of 400 bit/s under cbr; 0 = from bit_rate",
        },
        OptionField {
            name: "vbv_buffer_size_value",
            kind: OptionKind::U32,
            default: OptionValue::U32(0),
            help: "vbv_buffer_size_value in units of 16 kbit under cbr; 0 = 20 (less at low rates)",
        },
        OptionField {
            name: "data_partitioning",
            kind: OptionKind::U32,
            default: OptionValue::U32(0),
            help: "priority_breakpoint (1..=3, 64..=127) to emit two partition packets; 0 = off",
        },
        OptionField {
            name: "mpeg1_d_pictures",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "ISO/IEC 11172-2 D-only sequence (mpeg1video)",
        },
    ];

    fn apply(&mut self, key: &str, value: &OptionValue) -> Result<()> {
        match key {
            "quantiser_scale_code" => self.quantiser_scale_code = value.as_u32()?,
            "b_between" => self.b_between = value.as_u32()?,
            "anchors_per_gop" => self.anchors_per_gop = value.as_u32()?,
            "f_code" => self.f_code = value.as_u32()?,
            "backward_f_code" => self.backward_f_code = value.as_u32()?,
            "picture_structure" => {
                self.picture_structure = match value.as_str()? {
                    "frame" => PictureStructureOption::Frame,
                    "field" => PictureStructureOption::Field,
                    "frame_field" => PictureStructureOption::FrameField,
                    "field_adaptive" => PictureStructureOption::FieldAdaptive,
                    other => {
                        return Err(Error::invalid(format!(
                            "mpeg12video encoder: picture_structure '{other}'"
                        )))
                    }
                }
            }
            "interlaced" => self.interlaced = value.as_bool()?,
            "intra_vlc_format" => self.intra_vlc_format = value.as_bool()?,
            "alternate_scan" => self.alternate_scan = value.as_bool()?,
            "q_scale_type" => self.q_scale_type = value.as_bool()?,
            "intra_dc_precision" => self.intra_dc_precision = value.as_u32()?,
            "skipped_macroblocks" => self.skipped_macroblocks = value.as_bool()?,
            "concealment_motion_vectors" => self.concealment_motion_vectors = value.as_bool()?,
            "top_field_first" => self.top_field_first = value.as_bool()?,
            "repeat_first_field" => self.repeat_first_field = value.as_bool()?,
            "pulldown" => {
                self.pulldown = match value.as_str()? {
                    "none" => PulldownOption::None,
                    "3:2" => PulldownOption::ThreeTwo,
                    other => {
                        return Err(Error::invalid(format!(
                            "mpeg12video encoder: pulldown '{other}'"
                        )))
                    }
                }
            }
            "dual_prime" => self.dual_prime = value.as_bool()?,
            "rate_control" => {
                self.rate_control = match value.as_str()? {
                    "constant_quantiser" => RateControlOption::ConstantQuantiser,
                    "cbr" => RateControlOption::Cbr,
                    other => {
                        return Err(Error::invalid(format!(
                            "mpeg12video encoder: rate_control '{other}'"
                        )))
                    }
                }
            }
            "bit_rate_value" => self.bit_rate_value = value.as_u32()?,
            "vbv_buffer_size_value" => self.vbv_buffer_size_value = value.as_u32()?,
            "data_partitioning" => self.data_partitioning = value.as_u32()?,
            "mpeg1_d_pictures" => self.mpeg1_d_pictures = value.as_bool()?,
            other => {
                return Err(Error::invalid(format!(
                    "mpeg12video encoder: unknown option '{other}'"
                )))
            }
        }
        Ok(())
    }
}

/// The resolved, validated configuration the adapter runs with.
#[derive(Debug, Clone)]
struct EncoderConfig {
    options: Mpeg12EncoderOptions,
    chroma_format: ChromaFormat,
    /// Table 6-4 / §2.4.3.2 `frame_rate_code` (`picture_rate` for
    /// 11172-2).
    frame_rate_code: u8,
    /// The CBR configuration (only meaningful under `cbr`).
    cbr: CbrConfig,
}

fn invalid(msg: impl Into<String>) -> Error {
    Error::invalid(format!("mpeg12video encoder: {}", msg.into()))
}

fn unsupported(msg: impl Into<String>) -> Error {
    Error::unsupported(format!("mpeg12video encoder: {}", msg.into()))
}

/// Map a [`CodecParameters::frame_rate`] onto the Table 6-4 /
/// §2.4.3.2 `frame_rate_code` (25 frames/s when unset).
fn frame_rate_code_for(params: &CodecParameters) -> Result<u8> {
    let Some(rate) = params.frame_rate else {
        return Ok(3);
    };
    if rate.num <= 0 || rate.den <= 0 {
        return Err(invalid("frame_rate must be positive"));
    }
    for code in 1u8..=8 {
        let (n, d) = crate::vbv::frame_rate_value(code).map_err(map_err)?;
        // rate == n/d  <=>  rate.num * d == n * rate.den
        if rate.num as i128 * i128::from(d) == i128::from(n) * rate.den as i128 {
            return Ok(code);
        }
    }
    Err(unsupported(format!(
        "frame_rate {}/{} is not a Table 6-4 frame_rate_code",
        rate.num, rate.den
    )))
}

impl EncoderConfig {
    fn resolve(params: &CodecParameters, is_mpeg1: bool) -> Result<Self> {
        let o: Mpeg12EncoderOptions = parse_options(&params.options)?;

        if !(1..=31).contains(&o.quantiser_scale_code) {
            return Err(invalid("quantiser_scale_code outside 1..=31"));
        }
        if o.anchors_per_gop == 0 {
            return Err(invalid("anchors_per_gop must be >= 1"));
        }
        let f_code_max = if is_mpeg1 { 7 } else { 9 };
        if !(1..=f_code_max).contains(&o.f_code) {
            return Err(invalid(format!("f_code outside 1..={f_code_max}")));
        }
        if o.backward_f_code != 0 && !(1..=f_code_max).contains(&o.backward_f_code) {
            return Err(invalid(format!(
                "backward_f_code outside 1..={f_code_max} (0 = same as f_code)"
            )));
        }
        if o.intra_dc_precision > 3 {
            return Err(invalid("intra_dc_precision outside 0..=3"));
        }
        if o.data_partitioning != 0
            && !crate::data_partitioning::is_supported_breakpoint(
                u8::try_from(o.data_partitioning).unwrap_or(u8::MAX),
            )
        {
            return Err(invalid(
                "data_partitioning must be a Table 7-30 priority_breakpoint (1..=3, 64..=127)",
            ));
        }
        if o.bit_rate_value > 0x3FFFE {
            return Err(invalid("bit_rate_value outside 1..=0x3FFFE"));
        }
        if o.vbv_buffer_size_value > 0x3FF {
            return Err(invalid("vbv_buffer_size_value outside 1..=0x3FF"));
        }

        let chroma_format = match params.pixel_format {
            None | Some(PixelFormat::Yuv420P) => ChromaFormat::Yuv420,
            Some(PixelFormat::Yuv422P) => ChromaFormat::Yuv422,
            Some(PixelFormat::Yuv444P) => ChromaFormat::Yuv444,
            Some(other) => {
                return Err(invalid(format!(
                    "unsupported pixel format {other:?} (planar 4:2:0 / 4:2:2 / 4:4:4)"
                )))
            }
        };

        // Syntax-specific options.
        if is_mpeg1 {
            if chroma_format != ChromaFormat::Yuv420 {
                return Err(unsupported("ISO/IEC 11172-2 is 4:2:0 only"));
            }
            if o.picture_structure != PictureStructureOption::Frame || o.interlaced {
                return Err(unsupported(
                    "ISO/IEC 11172-2 has no interlace: picture_structure must be frame, interlaced false",
                ));
            }
            if o.intra_vlc_format || o.alternate_scan || o.q_scale_type || o.intra_dc_precision != 0
            {
                return Err(unsupported(
                    "intra_vlc_format / alternate_scan / q_scale_type / intra_dc_precision are MPEG-2 only",
                ));
            }
            if o.skipped_macroblocks
                || o.concealment_motion_vectors
                || o.top_field_first
                || o.repeat_first_field
                || o.pulldown != PulldownOption::None
                || o.dual_prime
            {
                return Err(unsupported(
                    "skipped_macroblocks / concealment_motion_vectors / top_field_first / repeat_first_field / pulldown / dual_prime are MPEG-2 only",
                ));
            }
            if o.data_partitioning != 0 {
                return Err(unsupported(
                    "data partitioning is §7.10 of ISO/IEC 13818-2 only",
                ));
            }
            if o.mpeg1_d_pictures && o.rate_control == RateControlOption::Cbr {
                return Err(unsupported("mpeg1_d_pictures has no CBR controller"));
            }
        } else {
            if o.mpeg1_d_pictures {
                return Err(unsupported("mpeg1_d_pictures is ISO/IEC 11172-2 only"));
            }
            let frame_only = o.skipped_macroblocks
                || o.concealment_motion_vectors
                || o.top_field_first
                || o.repeat_first_field
                || o.pulldown != PulldownOption::None;
            if frame_only && o.picture_structure != PictureStructureOption::Frame {
                return Err(unsupported(
                    "skipped_macroblocks / concealment_motion_vectors / top_field_first / repeat_first_field / pulldown apply to picture_structure = frame",
                ));
            }
            if frame_only && o.rate_control == RateControlOption::Cbr {
                return Err(unsupported(
                    "the CBR controller drives the baseline frame-picture encoders (no FrameEncodeOptions)",
                ));
            }
            if o.pulldown != PulldownOption::None && !o.interlaced {
                return Err(unsupported(
                    "pulldown 3:2 alternates top_field_first, which a progressive sequence forbids (§6.3.10): set interlaced",
                ));
            }
            if o.dual_prime
                && !matches!(
                    o.picture_structure,
                    PictureStructureOption::FrameField | PictureStructureOption::FieldAdaptive
                )
            {
                return Err(unsupported(
                    "dual_prime needs picture_structure = frame_field or field_adaptive",
                ));
            }
            if o.dual_prime && o.b_between != 0 {
                return Err(unsupported("dual_prime needs b_between = 0 (§7.6.3.6)"));
            }
            if o.rate_control == RateControlOption::Cbr
                && matches!(
                    o.picture_structure,
                    PictureStructureOption::FrameField | PictureStructureOption::FieldAdaptive
                )
            {
                return Err(unsupported(
                    "cbr drives the frame and field assemblers only (not frame_field / field_adaptive)",
                ));
            }
        }

        let frame_rate_code = frame_rate_code_for(params)?;
        let bit_rate_value = if o.bit_rate_value != 0 {
            o.bit_rate_value
        } else {
            match params.bit_rate {
                Some(bps) if bps > 0 => {
                    let v = bps.div_ceil(400);
                    u32::try_from(v)
                        .ok()
                        .filter(|v| *v <= 0x3FFFE)
                        .ok_or_else(|| invalid("bit_rate too large for bit_rate_value"))?
                }
                _ => CbrConfig::default().bit_rate_value,
            }
        };
        // Default VBV size: the CbrConfig default (20 × 16 kbit), shrunk
        // at low rates so a full buffer's §6.3.9 vbv_delay =
        // 90 000 · B / R still fits the 16-bit field.
        let default_vbv = {
            let rate = u64::from(bit_rate_value) * 400;
            let fits = rate * 65_535 / (90_000 * 16 * 1024);
            fits.clamp(1, u64::from(CbrConfig::default().vbv_buffer_size_value)) as u16
        };
        let cbr = CbrConfig {
            bit_rate_value,
            vbv_buffer_size_value: if o.vbv_buffer_size_value != 0 {
                o.vbv_buffer_size_value as u16
            } else {
                default_vbv
            },
            frame_rate_code,
            initial_quantiser_scale_code: o.quantiser_scale_code as u8,
        };

        Ok(Self {
            options: o,
            chroma_format,
            frame_rate_code,
            cbr,
        })
    }

    fn backward_f_code(&self) -> u8 {
        if self.options.backward_f_code == 0 {
            self.options.f_code as u8
        } else {
            self.options.backward_f_code as u8
        }
    }

    /// The MPEG-2 picture parameters for the selected structure.
    fn picture_params(&self, width: usize, height: usize) -> IntraPictureParams {
        let o = &self.options;
        let (frame_pred_frame_dct, progressive_sequence) = match o.picture_structure {
            PictureStructureOption::Frame => (true, !o.interlaced),
            PictureStructureOption::Field
            | PictureStructureOption::FrameField
            | PictureStructureOption::FieldAdaptive => (false, false),
        };
        IntraPictureParams {
            width,
            height,
            chroma_format: self.chroma_format,
            frame_pred_frame_dct,
            intra_dc_precision: o.intra_dc_precision as u8,
            intra_vlc_format: o.intra_vlc_format,
            alternate_scan: o.alternate_scan,
            q_scale_type: o.q_scale_type,
            progressive_sequence,
        }
    }

    /// The per-display-frame [`FrameEncodeOptions`].
    fn frame_options(&self, display_index: usize) -> FrameEncodeOptions {
        let o = &self.options;
        match o.pulldown {
            PulldownOption::ThreeTwo => FrameEncodeOptions {
                skipped_macroblocks: o.skipped_macroblocks,
                concealment_motion_vectors: o.concealment_motion_vectors,
                ..FrameEncodeOptions::pulldown_32(display_index)
            },
            PulldownOption::None => FrameEncodeOptions {
                skipped_macroblocks: o.skipped_macroblocks,
                concealment_motion_vectors: o.concealment_motion_vectors,
                top_field_first: o.top_field_first,
                repeat_first_field: o.repeat_first_field,
                progressive_frame: None,
            },
        }
    }

    fn mpeg1_sequence(&self, width: usize, height: usize) -> Mpeg1SequenceParams {
        let base = Mpeg1SequenceParams::default();
        let cbr = self.options.rate_control == RateControlOption::Cbr;
        Mpeg1SequenceParams {
            horizontal_size: width as u16,
            vertical_size: height as u16,
            picture_rate_code: self.frame_rate_code,
            bit_rate_value: if cbr {
                self.cbr.bit_rate_value
            } else {
                base.bit_rate_value
            },
            vbv_buffer_size_value: if cbr {
                self.cbr.vbv_buffer_size_value
            } else {
                base.vbv_buffer_size_value
            },
            ..base
        }
    }
}

/// Build a boxed [`Encoder`] for the codec id named by `params`.
///
/// Registered under both `"mpeg1video"` and `"mpeg2video"`; the id
/// selects the syntax (11172-2 vs 13818-2). `params` must be a video
/// parameter set with `width` / `height` present, a planar 4:2:0 /
/// 4:2:2 / 4:4:4 pixel format (or unset — 4:2:0 is assumed), and
/// options conforming to [`Mpeg12EncoderOptions::SCHEMA`] (see the
/// [module docs](self)).
///
/// # Errors
///
/// [`Error::invalid`] for a non-video parameter set, missing geometry,
/// an unsupported pixel format, an unknown option, a malformed or
/// out-of-range option value; [`Error::unsupported`] for a structure /
/// option combination no assembler implements.
pub fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    let width = params.width.ok_or_else(|| invalid("width missing"))?;
    let height = params.height.ok_or_else(|| invalid("height missing"))?;
    if width == 0 || height == 0 || width > 4095 || height > 4095 {
        return Err(invalid(
            "width/height outside the 12-bit sequence-header range",
        ));
    }
    let is_mpeg1 = params.codec_id.as_str() == crate::decoder::MPEG1_CODEC_ID_STR;
    let config = EncoderConfig::resolve(params, is_mpeg1)?;
    if matches!(
        config.options.picture_structure,
        PictureStructureOption::Field | PictureStructureOption::FieldAdaptive
    ) && height % 32 != 0
    {
        return Err(unsupported(
            "field pictures need a frame height that is a multiple of 32 (exact per-field §6.3.3 grid)",
        ));
    }

    let mut output_params = CodecParameters::video(params.codec_id.clone());
    output_params.width = Some(width);
    output_params.height = Some(height);
    output_params.pixel_format = Some(match config.chroma_format {
        ChromaFormat::Yuv420 => PixelFormat::Yuv420P,
        ChromaFormat::Yuv422 => PixelFormat::Yuv422P,
        ChromaFormat::Yuv444 => PixelFormat::Yuv444P,
    });
    output_params.frame_rate = params.frame_rate;
    if config.options.rate_control == RateControlOption::Cbr {
        output_params.bit_rate = Some(u64::from(config.cbr.bit_rate_value) * 400);
    }
    output_params.options = params.options.clone();

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
/// See the [module docs](self) for the whole-stream framing model and
/// the option schema.
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

/// Convert a planar [`VideoFrame`] into a [`FrameBuffer`] of
/// `chroma_format`, honouring each plane's stride.
fn video_frame_to_frame_buffer(
    v: &VideoFrame,
    width: usize,
    height: usize,
    chroma_format: ChromaFormat,
) -> Result<FrameBuffer> {
    if v.planes.len() < 3 {
        return Err(invalid("expected 3 planar Y/Cb/Cr planes"));
    }
    let mut out = FrameBuffer::new(width, height, chroma_format);
    let (cw, ch) = out.visible_chroma_dims();
    let copy = |plane: &oxideav_core::VideoPlane,
                dst: &mut crate::frame_assembly::Plane,
                w: usize,
                h: usize|
     -> Result<()> {
        if plane.stride < w || plane.data.len() < plane.stride * (h - 1) + w {
            return Err(invalid("video plane smaller than the declared geometry"));
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
    /// The elementary stream for the buffered frames.
    fn assemble(&self) -> Result<Vec<u8>> {
        let c = &self.config;
        let o = &c.options;
        let frames = &self.frames;
        let b_between = o.b_between as usize;
        let anchors = o.anchors_per_gop as usize;
        let q = o.quantiser_scale_code as u8;
        let fwd = o.f_code as u8;
        let bwd = c.backward_f_code();

        if self.is_mpeg1 {
            let seq = c.mpeg1_sequence(self.width, self.height);
            let stream = if o.mpeg1_d_pictures {
                crate::mpeg1_encoder::encode_mpeg1_d_sequence(frames, &seq, q, anchors)
            } else if o.rate_control == RateControlOption::Cbr {
                crate::rate_control::encode_mpeg1_cbr_sequence(
                    frames, b_between, anchors, &seq, q, fwd, bwd,
                )
                .map(|e| e.stream)
            } else {
                crate::mpeg1_encoder::encode_mpeg1_display_order_sequence(
                    frames, b_between, anchors, &seq, q, fwd, bwd,
                )
            };
            return stream.map_err(map_err);
        }

        let params = c.picture_params(self.width, self.height);
        let stream = match (o.picture_structure, o.rate_control) {
            (PictureStructureOption::Frame, RateControlOption::ConstantQuantiser) => {
                crate::inter_encoder::encode_display_order_gop_sequence_with_options(
                    frames,
                    b_between,
                    anchors,
                    params,
                    q,
                    fwd,
                    bwd,
                    &QuantMatrixExtension::default(),
                    &|display_index| c.frame_options(display_index),
                )
                .map(|(stream, _stats)| stream)
            }
            (PictureStructureOption::Frame, RateControlOption::Cbr) => {
                crate::rate_control::encode_cbr_gop_sequence(
                    frames, b_between, anchors, params, &c.cbr, fwd, bwd,
                )
                .map(|e| e.stream)
            }
            (PictureStructureOption::Field, RateControlOption::ConstantQuantiser) => {
                crate::field_picture_encoder::encode_field_display_order_gop_sequence(
                    frames, b_between, anchors, &params, q, fwd, bwd,
                )
            }
            (PictureStructureOption::Field, RateControlOption::Cbr) => {
                crate::rate_control::encode_field_cbr_gop_sequence(
                    frames, b_between, anchors, &params, &c.cbr, fwd, bwd,
                )
                .map(|e| e.stream)
            }
            (PictureStructureOption::FrameField, _) => {
                crate::frame_field_encoder::encode_ff_display_order_gop_sequence(
                    frames,
                    b_between,
                    anchors,
                    &params,
                    q,
                    fwd,
                    bwd,
                    o.dual_prime,
                )
                .map(|(stream, _stats)| stream)
            }
            (PictureStructureOption::FieldAdaptive, _) => {
                crate::field_picture_encoder::encode_field_adaptive_display_order_gop_sequence(
                    frames,
                    b_between,
                    anchors,
                    &params,
                    q,
                    fwd,
                    bwd,
                    o.dual_prime,
                )
                .map(|(stream, _stats)| stream)
            }
        };
        stream.map_err(map_err)
    }

    /// Run the display-order assembler over the buffered frames and
    /// queue the finished elementary stream as one keyframe packet
    /// (two under data partitioning).
    fn encode_buffered(&mut self) -> Result<()> {
        if self.flushed {
            return Ok(());
        }
        self.flushed = true;
        if self.frames.is_empty() {
            return Ok(());
        }
        let stream = self.assemble()?;
        let frame_count = self.frames.len() as i64;
        self.frames.clear();

        let (n, d) = crate::vbv::frame_rate_value(self.config.frame_rate_code).map_err(map_err)?;
        let time_base = TimeBase::new(i64::from(d), i64::from(n));

        let breakpoint = self.config.options.data_partitioning;
        let payloads = if breakpoint != 0 {
            let (p0, p1) =
                crate::data_partitioning::split_data_partitions(&stream, breakpoint as u8)
                    .map_err(map_err)?;
            vec![p0, p1]
        } else {
            vec![stream]
        };
        for payload in payloads {
            let mut packet = Packet::new(0, time_base, payload);
            packet.pts = Some(0);
            packet.dts = Some(0);
            packet.duration = Some(frame_count);
            packet.flags.keyframe = true;
            self.ready.push_back(packet);
        }
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
            return Err(invalid("send_frame after flush"));
        }
        let Frame::Video(v) = frame else {
            return Err(invalid("expected a video frame"));
        };
        let fb =
            video_frame_to_frame_buffer(v, self.width, self.height, self.config.chroma_format)?;
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
