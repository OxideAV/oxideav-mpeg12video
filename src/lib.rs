//! # oxideav-mpeg12video
//!
//! Clean-room MPEG-1 Video (ISO/IEC 11172-2) / MPEG-2 Video
//! (ITU-T H.262 / ISO/IEC 13818-2) decoder and encoder for the
//! [oxideav](https://github.com/OxideAV/oxideav) framework.
//!
//! **Status:** rebuild rounds 1–8 — structural sequence-layer
//! parsers, the `group_of_pictures_header()` layer, the
//! `picture_header()` (+ `picture_coding_extension()`) layer, the
//! `slice()` header bits, and the leading macroblock-loop syntax
//! (`macroblock_address_increment`, `macroblock_type`,
//! `coded_block_pattern()`). The block loop — DCT-coefficient VLCs,
//! IDCT, and motion compensation — is not wired up yet; the public
//! `register` symbol is still a no-op so that downstream consumers
//! can depend on the crate without the decoder being inadvertently
//! selected by the registry.
//!
//! The landed pieces so far are:
//!
//! * [`sequence_header::Mpeg2SequenceHeader`] — `sequence_header()`
//!   from ISO/IEC 13818-2 §6.2.2.1 (field semantics §6.3.3).
//! * [`sequence_extension::Mpeg2SequenceExtension`] —
//!   `sequence_extension()` from §6.2.2.3 (field semantics §6.3.5).
//! * [`sequence_extension::Mpeg2Sequence`] — composed view that
//!   pairs the two and synthesises the full 14-bit width/height,
//!   30-bit bit_rate, and 18-bit vbv_buffer_size.
//! * [`gop_header::Mpeg2Gop`] — `group_of_pictures_header()` from
//!   §6.2.2.6 (field semantics §6.3.8), including the 25-bit
//!   `time_code` decomposition and the `closed_gop` / `broken_link`
//!   editing flags.
//! * [`picture_header::Mpeg2PictureHeader`] — `picture_header()`
//!   from §6.2.3 (field semantics §6.3.10) plus the companion
//!   [`picture_header::PictureCodingExtension`] for §6.2.3.1 /
//!   §6.3.11.
//! * [`slice_header::SliceHeader`] — the start-code-aligned header
//!   bits of `slice()` from §6.2.4 (field semantics §6.3.16):
//!   `slice_vertical_position` (from the start code), optional
//!   `slice_vertical_position_extension` (when `vertical_size >
//!   2800`), optional `priority_breakpoint` (when the surrounding
//!   sequence is data-partitioned), `quantiser_scale_code`, the
//!   optional `intra_slice_flag` / `intra_slice` / `reserved_bits`
//!   prelude, and the `extra_information_slice` byte loop. The
//!   macroblock body is **not** yet decoded.
//! * [`mb_address_increment::MbAddressIncrement`] — the leading
//!   `macroblock_address_increment` of `macroblock()` per §6.2.5
//!   (field semantics §6.3.17.1), with the Annex B Table B-1 VLC
//!   walker plus the `macroblock_escape` chain and (when
//!   [`mb_address_increment::MbAddressIncrementContext::mpeg1`] is
//!   set) the MPEG-1 `macroblock_stuffing` no-op.
//! * [`macroblock_type::MacroblockType`] — the `macroblock_type` VLC
//!   that opens `macroblock_modes()` per §6.2.5.1 (field semantics
//!   §6.3.17.1), decoding the six derived flags from the
//!   non-scalable Annex B Tables B-2 (I), B-3 (P), and B-4 (B).
//! * [`coded_block_pattern::CodedBlockPattern`] — the
//!   `coded_block_pattern()` syntax per §6.2.5.3 (field semantics
//!   §6.3.17.4): the Annex B Table B-9 `coded_block_pattern_420` VLC
//!   plus the 4:2:2 / 4:4:4 fixed-length extensions, and the
//!   §6.3.17.4 `pattern_code[12]` derivation.

#![warn(missing_debug_implementations)]

use oxideav_core::RuntimeContext;

pub mod coded_block_pattern;
pub mod gop_header;
pub mod macroblock_type;
pub mod mb_address_increment;
pub mod picture_header;
pub mod sequence_extension;
pub mod sequence_header;
pub mod slice_header;

pub use coded_block_pattern::CodedBlockPattern;
pub use gop_header::{Mpeg2Gop, TimeCode, GROUP_START_CODE};
pub use macroblock_type::MacroblockType;
pub use mb_address_increment::{MbAddressIncrement, MbAddressIncrementContext};
pub use picture_header::{
    Mpeg2PictureHeader, PictureCodingExtension, PictureCodingType, PictureStructure,
    PICTURE_CODING_EXTENSION_ID, PICTURE_START_CODE,
};
pub use sequence_extension::{
    ChromaFormat, Mpeg2Sequence, Mpeg2SequenceExtension, EXTENSION_START_CODE,
    SEQUENCE_EXTENSION_ID,
};
pub use sequence_header::{AspectRatio, Mpeg2SequenceHeader, SEQUENCE_HEADER_CODE};
pub use slice_header::{
    SliceContext, SliceHeader, SLICE_VERTICAL_POSITION_MAX, SLICE_VERTICAL_POSITION_MIN,
};

/// Crate-local error type. Each variant is raised at most by the
/// specific decoder stage named in its docstring; sites may grow as
/// future rounds add slice/macroblock layers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// A bitstream constraint defined by ISO/IEC 13818-2 was
    /// violated (forbidden value, marker_bit zero, wrong start code,
    /// reserved entry where reserved values are not allowed, etc.).
    /// The static message names the spec subclause.
    InvalidBitstream(&'static str),
    /// The input buffer ended before the parser had read every bit
    /// the syntax element demanded.
    ShortHeader,
    /// Placeholder for syntax paths that are spec-defined but not
    /// yet implemented in this crate (motion vectors, IDCT, slice
    /// decoding, …). No code path currently returns this — it is
    /// kept as the contract for the encoder/decoder traits that
    /// later rounds will wire up.
    NotImplemented,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidBitstream(detail) => {
                write!(f, "mpeg12video: invalid bitstream: {detail}")
            }
            Self::ShortHeader => {
                write!(f, "mpeg12video: short header (unexpected end of input)")
            }
            Self::NotImplemented => {
                write!(f, "mpeg12video: feature not implemented yet")
            }
        }
    }
}

impl std::error::Error for Error {}

/// Crate-local `Result` alias.
pub type Result<T> = core::result::Result<T, Error>;

/// No-op codec registration. Rounds 1–4 only parse the sequence,
/// group-of-pictures, and picture headers — they do not yet provide
/// a [`oxideav_core::Decoder`] or [`oxideav_core::Encoder`], so there
/// is nothing to install in the registry.
pub fn register(_ctx: &mut RuntimeContext) {}

oxideav_core::register!("mpeg12video", register);
