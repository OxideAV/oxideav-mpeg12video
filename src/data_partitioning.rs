//! ISO/IEC 13818-2 §7.10 **data partitioning** — the one scalable
//! mode that is purely syntactic: a single video bitstream is split
//! into two *partitions* at a per-slice `priority_breakpoint`
//! (Table 7-30), and the decoding process alternates between the two
//! partitions element by element. Sequence, GOP and picture headers
//! are copied redundantly into partition 1 (whose only extensions
//! are `sequence_extension()`, `picture_coding_extension()` and
//! `sequence_scalable_extension()`), every slice appears in both
//! partitions down to `extra_bit_slice` (partition 1's carrying
//! `priority_breakpoint = 0`), and the `sequence_end_code` is
//! duplicated.
//!
//! This module implements the §7.10 partition switching as a
//! bitstream-to-bitstream engine over the crate's per-element syntax
//! parsers (`macroblock_address_increment`, `macroblock_type` +
//! `macroblock_modes()` tail, `quantiser_scale_code`,
//! `motion_vectors(s)`, the concealment `marker_bit`,
//! `coded_block_pattern()`, the §7.2.1 DC prelude and the §7.2.2
//! run-level walker), driven in two configurations:
//!
//! * [`split_data_partitions`] — the encoder side: a non-scalable
//!   ISO/IEC 13818-2 stream is divided into partition 0 / partition 1
//!   at a chosen breakpoint (the `sequence_scalable_extension()` and
//!   the slice-level `priority_breakpoint` are inserted, the
//!   partition-1 header copies filtered per §7.10);
//! * [`merge_data_partitions`] — the decoder side: the §7.10
//!   *"set current_partition …"* procedure, re-forming the
//!   non-scalable stream (`priority_breakpoint` and the scalable
//!   extension removed) that [`crate::decode_video_sequence`] then
//!   reconstructs — [`decode_data_partitioned`] chains the two.
//!
//! `merge(split(stream, pb))` reproduces `stream` **byte-exactly** for
//! every supported breakpoint, which is the round-trip pinned by the
//! integration tests over the whole conformance corpus.
//!
//! ## Element classes (Table 7-30)
//!
//! Every syntax element below `extra_bit_slice` is assigned the
//! smallest `priority_breakpoint` value that places it in partition
//! 0:
//!
//! | element | class |
//! |---|---|
//! | `macroblock_escape*` + `macroblock_address_increment` | 2 |
//! | `macroblock_type` … `marker_bit` (modes, `quantiser_scale_code`, motion vectors) | 3 |
//! | `coded_block_pattern()` | 64 |
//! | intra `dct_dc_size` + `dct_dc_differential` | 64 |
//! | the *j*-th `(run, level)` pair — or the `end_of_block` arriving in its place | 63 + *j* |
//!
//! An element goes to partition 0 iff `class <= priority_breakpoint`.
//! Breakpoint `1` places nothing but the slice header in partition 0;
//! `4..=63` are reserved and `0` is partition 1's own marker
//! (§6.3.16), so both are rejected.

use oxideav_core::bits::{BitReader, BitWriter};

use crate::coded_block_pattern::CodedBlockPattern;
use crate::macroblock_modes::{
    MacroblockModesContext, MacroblockModesTail, MotionType, MvFormat, PredictionType,
};
use crate::macroblock_type::MacroblockType;
use crate::mb_address_increment::{MbAddressIncrement, MbAddressIncrementContext};
use crate::motion_vector::{MotionVectors, MotionVectorsContext, MotionVectorsKind};
use crate::mpeg2_block_dc::{decode_dc_block, DcPredictors};
use crate::mpeg2_dct_coeff::{CoefficientPosition, DctCoeff, DctCoeffStep, TableSelection};
use crate::mpeg2_macroblock_blocks::{block_component, block_count};
use crate::picture_header::{
    Mpeg2PictureHeader, PictureCodingExtension, PictureCodingType, PictureStructure,
};
use crate::quantizer_scale::QuantizerScale;
use crate::sequence_extension::ChromaFormat;
use crate::sequence_scalable_extension::{
    ScalableMode, SequenceScalableExtension, SEQUENCE_SCALABLE_EXTENSION_ID,
};
use crate::slice_header::{SliceContext, SliceHeader};
use crate::{Error, Result};

/// Table 7-30 class of `macroblock_address_increment` (with its
/// escapes).
const CLASS_MBAI: u8 = 2;
/// Table 7-30 class of everything from `macroblock_type` up to but
/// not including `coded_block_pattern()`.
const CLASS_MODES: u8 = 3;
/// Table 7-30 class of `coded_block_pattern()` and of the intra DC
/// coefficient.
const CLASS_CBP_DC: u8 = 64;

/// Table 7-30 class of the `j`-th (1-based) `(run, level)` pair of a
/// block, or of the `end_of_block` that arrives in its place.
fn pair_class(j: u8) -> u8 {
    63u8.saturating_add(j)
}

/// Whether `priority_breakpoint` is a value Table 7-30 defines for
/// partition 0: `1..=3` or `64..=127` (`0` is reserved for partition
/// 1, `4..=63` are reserved).
pub fn is_supported_breakpoint(priority_breakpoint: u8) -> bool {
    matches!(priority_breakpoint, 1..=3 | 64..=127)
}

fn check_breakpoint(priority_breakpoint: u8) -> Result<()> {
    if is_supported_breakpoint(priority_breakpoint) {
        Ok(())
    } else {
        Err(Error::InvalidBitstream(
            "priority_breakpoint: partition 0 admits 1..=3 or 64..=127 only (Table 7-30)",
        ))
    }
}

/// Copy the bit range `[from, to)` of `src` into `bw`.
pub fn copy_bits(src: &[u8], from: u64, to: u64, bw: &mut BitWriter) -> Result<()> {
    if to < from || to > (src.len() as u64) * 8 {
        return Err(Error::ShortHeader);
    }
    let mut br = BitReader::new(src);
    let skip = u32::try_from(from).map_err(|_| Error::ShortHeader)?;
    br.skip(skip).map_err(|_| Error::ShortHeader)?;
    let mut remaining = to - from;
    while remaining > 0 {
        let take = remaining.min(32) as u32;
        let v = br.read_u32(take).map_err(|_| Error::ShortHeader)?;
        bw.write_u32(v, take);
        remaining -= u64::from(take);
    }
    Ok(())
}

/// The picture-level syntax context the slice walk needs.
#[derive(Debug, Clone, Copy)]
struct PictureSyntax {
    coding_type: PictureCodingType,
    structure: PictureStructure,
    frame_pred_frame_dct: bool,
    concealment_motion_vectors: bool,
    intra_vlc_format: bool,
    intra_dc_precision: u8,
    mv_ctx: MotionVectorsContext,
}

/// Sequence-level context.
#[derive(Debug, Clone, Copy)]
struct SequenceSyntax {
    chroma_format: ChromaFormat,
    vertical_size: u32,
}

/// The default `motion_type` when none is coded (§6.3.17.1): Frame-based
/// in frame pictures, Field-based in field pictures.
fn default_motion_type(structure: PictureStructure) -> MotionType {
    match structure {
        PictureStructure::Frame => MotionType {
            code: 0b10,
            prediction_type: PredictionType::FrameBased,
            motion_vector_count: 1,
            mv_format: MvFormat::Frame,
            dmv: false,
        },
        PictureStructure::TopField | PictureStructure::BottomField => MotionType {
            code: 0b01,
            prediction_type: PredictionType::FieldBased,
            motion_vector_count: 1,
            mv_format: MvFormat::Field,
            dmv: false,
        },
    }
}

/// The partition-switching bit source/sink the slice walk drives.
///
/// [`Self::element`] parses one syntax element of Table 7-30 class
/// `class` from the partition that holds it and copies its bits to
/// the destination that receives it; [`Self::slice_body_done`] tests
/// the §6.2.4 slice-termination condition (`nextbits()` all zero /
/// exhausted) on the partition that holds the next
/// `macroblock_address_increment`.
trait PartitionIo {
    fn element<T>(
        &mut self,
        class: u8,
        parse: impl FnOnce(&mut BitReader<'_>) -> Result<T>,
    ) -> Result<T>;
    fn slice_body_done(&mut self) -> Result<bool>;
}

/// §6.2.4: the macroblock loop ends when the next 23 bits are zero
/// (a start code follows). A slice buffer cut at the next start code
/// may hold fewer than 23 bits of byte-alignment / stuffing zeros at
/// its end, so with fewer than 23 bits left the loop ends iff every
/// remaining bit is zero (no macroblock begins with an all-zero
/// tail: Table B-1 codewords carry a `1` within 11 bits).
fn body_done(br: &mut BitReader<'_>) -> Result<bool> {
    let remaining = br.bits_remaining();
    if remaining == 0 {
        return Ok(true);
    }
    let probe = remaining.min(23) as u32;
    Ok(br.peek_u32(probe).map_err(|_| Error::ShortHeader)? == 0)
}

/// The bytes of a slice buffer beyond the byte holding bit position
/// `pos` — the §5.2.3 zero stuffing a stream may carry before the next
/// start code. Preserved verbatim through partition 0 so that
/// `merge(split(s)) == s` byte-for-byte.
fn slice_tail(data: &[u8], pos: u64) -> &[u8] {
    let used = pos.div_ceil(8) as usize;
    &data[used.min(data.len())..]
}

/// Encoder-side configuration: one source cursor, two partition
/// writers.
struct SplitIo<'a, 'w> {
    buf: &'a [u8],
    br: BitReader<'a>,
    priority_breakpoint: u8,
    w0: &'w mut BitWriter,
    w1: &'w mut BitWriter,
}

impl PartitionIo for SplitIo<'_, '_> {
    fn element<T>(
        &mut self,
        class: u8,
        parse: impl FnOnce(&mut BitReader<'_>) -> Result<T>,
    ) -> Result<T> {
        let start = self.br.bit_position();
        let value = parse(&mut self.br)?;
        let end = self.br.bit_position();
        let sink: &mut BitWriter = if class <= self.priority_breakpoint {
            self.w0
        } else {
            self.w1
        };
        copy_bits(self.buf, start, end, sink)?;
        Ok(value)
    }

    fn slice_body_done(&mut self) -> Result<bool> {
        body_done(&mut self.br)
    }
}

/// Decoder-side configuration: two partition cursors, one output.
struct MergeIo<'a, 'w> {
    buf0: &'a [u8],
    r0: BitReader<'a>,
    buf1: &'a [u8],
    r1: BitReader<'a>,
    priority_breakpoint: u8,
    out: &'w mut BitWriter,
}

impl PartitionIo for MergeIo<'_, '_> {
    fn element<T>(
        &mut self,
        class: u8,
        parse: impl FnOnce(&mut BitReader<'_>) -> Result<T>,
    ) -> Result<T> {
        let (buf, br) = if class <= self.priority_breakpoint {
            (self.buf0, &mut self.r0)
        } else {
            (self.buf1, &mut self.r1)
        };
        let start = br.bit_position();
        let value = parse(br)?;
        let end = br.bit_position();
        copy_bits(buf, start, end, self.out)?;
        Ok(value)
    }

    fn slice_body_done(&mut self) -> Result<bool> {
        if CLASS_MBAI <= self.priority_breakpoint {
            body_done(&mut self.r0)
        } else {
            body_done(&mut self.r1)
        }
    }
}

/// Walk one slice's macroblock loop (§6.2.4 / §6.2.5 / §6.2.6),
/// routing every element through `io` by its Table 7-30 class.
fn walk_slice_body<IO: PartitionIo>(
    io: &mut IO,
    seq: &SequenceSyntax,
    pic: &PictureSyntax,
) -> Result<()> {
    let modes_ctx = MacroblockModesContext::new(pic.structure, pic.frame_pred_frame_dct);
    let mut dc_pred = DcPredictors::new(pic.intra_dc_precision)?;
    let nblocks = block_count(seq.chroma_format);
    let mut macroblocks = 0usize;

    while !io.slice_body_done()? {
        // Class 2: macroblock_escape* + macroblock_address_increment.
        let mbai = io.element(CLASS_MBAI, |br| {
            MbAddressIncrement::parse(br, MbAddressIncrementContext::mpeg2())
        })?;
        // §7.2.1: the DC predictors reset at every skipped macroblock
        // (and, below, at every non-intra one) — the walk tracks them
        // only so the DC prelude's range check sees the values the
        // decoder would.
        if mbai.value > 1 {
            dc_pred = DcPredictors::new(pic.intra_dc_precision)?;
        }

        // Class 3: macroblock_type, macroblock_modes() tail,
        // quantiser_scale_code, motion_vectors(0/1), marker_bit.
        let mb_type = io.element(CLASS_MODES, |br| {
            let mb_type = MacroblockType::parse(br, pic.coding_type)?;
            let tail = MacroblockModesTail::parse(br, &mb_type, &modes_ctx)?;
            QuantizerScale::parse_after_type(br, &mb_type)?;
            let motion_type = tail
                .motion_type
                .unwrap_or_else(|| default_motion_type(pic.structure));
            if mb_type.macroblock_motion_forward
                || (mb_type.macroblock_intra && pic.concealment_motion_vectors)
            {
                MotionVectors::parse(br, MotionVectorsKind::Forward, &motion_type, &pic.mv_ctx)?;
            }
            if mb_type.macroblock_motion_backward {
                MotionVectors::parse(br, MotionVectorsKind::Backward, &motion_type, &pic.mv_ctx)?;
            }
            if mb_type.macroblock_intra && pic.concealment_motion_vectors {
                let marker = br.read_bit().map_err(|_| Error::ShortHeader)?;
                if !marker {
                    return Err(Error::InvalidBitstream(
                        "macroblock(): concealment marker_bit shall be '1' (§6.2.5)",
                    ));
                }
            }
            Ok(mb_type)
        })?;

        if !mb_type.macroblock_intra {
            dc_pred = DcPredictors::new(pic.intra_dc_precision)?;
        }

        // Class 64: coded_block_pattern(); §6.3.17.4 pattern_code[].
        let pattern_code: [bool; 12] = if mb_type.macroblock_pattern {
            let cbp = io.element(CLASS_CBP_DC, |br| {
                CodedBlockPattern::parse(br, seq.chroma_format)
            })?;
            cbp.pattern_code(mb_type.macroblock_intra, true)
        } else if mb_type.macroblock_intra {
            [true; 12]
        } else {
            [false; 12]
        };

        // Blocks: DC (intra, class 64) then (run, level) pairs at
        // class 63 + j, end_of_block taking the class of the pair
        // position it occupies.
        let table = TableSelection::from_context(pic.intra_vlc_format, mb_type.macroblock_intra);
        for (i, &coded) in pattern_code.iter().enumerate().take(nblocks) {
            if !coded {
                continue;
            }
            let mut position = CoefficientPosition::First;
            if mb_type.macroblock_intra {
                let component = block_component(i, seq.chroma_format).ok_or(
                    Error::InvalidBitstream("block(): invalid block index for chroma_format"),
                )?;
                io.element(CLASS_CBP_DC, |br| {
                    decode_dc_block(br, &mut dc_pred, component).map(|_| ())
                })?;
                position = CoefficientPosition::Next;
            }
            let mut pair = 0u8;
            loop {
                pair = pair.checked_add(1).ok_or(Error::InvalidBitstream(
                    "block(): more than 64 (run, level) pairs (§7.2.2)",
                ))?;
                if pair > 65 {
                    return Err(Error::InvalidBitstream(
                        "block(): more than 64 (run, level) pairs (§7.2.2)",
                    ));
                }
                let step = io.element(pair_class(pair), |br| {
                    DctCoeffStep::parse(br, table, position)
                })?;
                position = CoefficientPosition::Next;
                if step.symbol == DctCoeff::EndOfBlock {
                    break;
                }
            }
        }
        macroblocks += 1;
    }

    if macroblocks == 0 {
        return Err(Error::InvalidBitstream(
            "slice(): every slice shall contain at least one macroblock (§6.3.16)",
        ));
    }
    Ok(())
}

// -------------------------------------------------------------------
// Stream structure
// -------------------------------------------------------------------

/// One start-code delimited block of an elementary stream.
#[derive(Debug, Clone, Copy)]
struct StreamBlock {
    /// The start-code value byte (`0x00` picture, `0x01..=0xAF`
    /// slice, `0xB2` user data, `0xB3` sequence header, `0xB5`
    /// extension, `0xB7` sequence end, `0xB8` GOP).
    code: u8,
    start: usize,
    end: usize,
}

impl StreamBlock {
    fn is_slice(&self) -> bool {
        (0x01..=0xAF).contains(&self.code)
    }
}

/// Split `stream` into its start-code delimited blocks.
fn scan_blocks(stream: &[u8]) -> Result<Vec<StreamBlock>> {
    let mut starts: Vec<usize> = Vec::new();
    let mut i = 0usize;
    while i + 3 < stream.len() {
        if stream[i] == 0 && stream[i + 1] == 0 && stream[i + 2] == 1 {
            starts.push(i);
            i += 4;
        } else {
            i += 1;
        }
    }
    if starts.is_empty() {
        return Err(Error::InvalidBitstream(
            "data partitioning: no start codes in the stream",
        ));
    }
    if starts[0] != 0 {
        return Err(Error::InvalidBitstream(
            "data partitioning: the stream shall begin with a start code",
        ));
    }
    Ok(starts
        .iter()
        .enumerate()
        .map(|(k, &s)| StreamBlock {
            code: stream[s + 3],
            start: s,
            end: starts.get(k + 1).copied().unwrap_or(stream.len()),
        })
        .collect())
}

/// The 4-bit `extension_start_code_identifier` of an extension block.
fn extension_id(block: &[u8]) -> Option<u32> {
    block.get(4).map(|b| u32::from(b >> 4))
}

const SEQUENCE_EXTENSION_ID: u32 = 0b0001;
const PICTURE_CODING_EXTENSION_ID: u32 = 0b1000;

/// Parse the §6.2.2.3 `sequence_extension()` fields the walk needs
/// (`chroma_format`, `vertical_size_extension`).
fn parse_sequence_extension_fields(block: &[u8]) -> Result<(ChromaFormat, u32)> {
    let mut br = BitReader::new(block);
    // start code (32) + id (4) + profile_and_level_indication (8) +
    // progressive_sequence (1)
    br.skip(32 + 4 + 8 + 1).map_err(|_| Error::ShortHeader)?;
    let chroma = match br.read_u32(2).map_err(|_| Error::ShortHeader)? {
        0b01 => ChromaFormat::Yuv420,
        0b10 => ChromaFormat::Yuv422,
        0b11 => ChromaFormat::Yuv444,
        _ => {
            return Err(Error::InvalidBitstream(
                "sequence_extension: chroma_format 00 is reserved (§6.3.5)",
            ))
        }
    };
    let _horizontal_size_extension = br.read_u32(2).map_err(|_| Error::ShortHeader)?;
    let vertical_size_extension = br.read_u32(2).map_err(|_| Error::ShortHeader)?;
    Ok((chroma, vertical_size_extension))
}

/// Parse the §6.2.2.1 `vertical_size_value` of a sequence header.
fn parse_vertical_size_value(block: &[u8]) -> Result<u32> {
    let mut br = BitReader::new(block);
    br.skip(32 + 12).map_err(|_| Error::ShortHeader)?;
    br.read_u32(12).map_err(|_| Error::ShortHeader)
}

fn picture_syntax(header: &Mpeg2PictureHeader, ext: &PictureCodingExtension) -> PictureSyntax {
    PictureSyntax {
        coding_type: header.picture_coding_type,
        structure: ext.picture_structure,
        frame_pred_frame_dct: ext.frame_pred_frame_dct,
        concealment_motion_vectors: ext.concealment_motion_vectors,
        intra_vlc_format: ext.intra_vlc_format,
        intra_dc_precision: ext.intra_dc_precision,
        mv_ctx: MotionVectorsContext {
            f_code_fwd_horiz: ext.f_code_fwd_horiz,
            f_code_fwd_vert: ext.f_code_fwd_vert,
            f_code_bwd_horiz: ext.f_code_bwd_horiz,
            f_code_bwd_vert: ext.f_code_bwd_vert,
        },
    }
}

/// Emit a §6.2.2.5 `sequence_scalable_extension()` declaring data
/// partitioning (`scalable_mode = 00`) for `layer_id`, byte-aligned
/// with zero stuffing (§5.2.3 `next_start_code()`).
pub fn write_data_partitioning_scalable_extension(bw: &mut BitWriter, layer_id: u8) {
    bw.write_u32(0x0000_01B5, 32);
    bw.write_u32(SEQUENCE_SCALABLE_EXTENSION_ID, 4);
    bw.write_u32(0b00, 2); // scalable_mode = data partitioning
    bw.write_u32(u32::from(layer_id & 0xF), 4);
    bw.align_to_byte_zero();
}

/// Bit offset of the bits following `quantiser_scale_code` in a
/// non-scalable slice header with `vertical_size <= 2800`:
/// start code (32) + quantiser_scale_code (5).
const SLICE_TAIL_BIT_NON_SCALABLE: u64 = 32 + 5;
/// The same offset in a data-partitioned slice header: start code
/// (32) + priority_breakpoint (7) + quantiser_scale_code (5).
const SLICE_TAIL_BIT_PARTITIONED: u64 = 32 + 7 + 5;

// -------------------------------------------------------------------
// Split
// -------------------------------------------------------------------

/// Split a **non-scalable** ISO/IEC 13818-2 elementary stream into
/// its §7.10 partition 0 / partition 1 pair at `priority_breakpoint`
/// (Table 7-30: `1..=3` or `64..=127`).
///
/// * A `sequence_scalable_extension()` (`scalable_mode` = data
///   partitioning, `layer_id` 0 / 1) is inserted after every
///   `sequence_extension()` in both partitions.
/// * Sequence, GOP and picture headers, `sequence_extension()` and
///   `picture_coding_extension()` and the `sequence_end_code` are
///   copied into both partitions; every other extension and user
///   data stays in partition 0 only (§7.10).
/// * Every slice header is emitted in both partitions with
///   `priority_breakpoint` (partition 0) / `0` (partition 1), and the
///   macroblock data is routed element by element.
///
/// # Errors
/// [`Error::InvalidBitstream`] for an unsupported breakpoint, an
/// ISO/IEC 11172-2 stream (no `sequence_extension()`), a stream that
/// is already scalable, `vertical_size > 2800`, or any syntax error
/// in the macroblock layer; [`Error::ShortHeader`] on truncation.
pub fn split_data_partitions(stream: &[u8], priority_breakpoint: u8) -> Result<(Vec<u8>, Vec<u8>)> {
    check_breakpoint(priority_breakpoint)?;
    let blocks = scan_blocks(stream)?;
    let mut w0 = BitWriter::new();
    let mut w1 = BitWriter::new();
    let mut seq: Option<SequenceSyntax> = None;
    let mut vertical_size_value = 0u32;
    let mut pending_header: Option<Mpeg2PictureHeader> = None;
    let mut pic: Option<PictureSyntax> = None;

    for block in &blocks {
        let data = &stream[block.start..block.end];
        match block.code {
            0xB3 => {
                vertical_size_value = parse_vertical_size_value(data)?;
                // A new sequence header resets the extension state.
                seq = None;
                w0.write_bytes(data);
                w1.write_bytes(data);
            }
            0xB5 => match extension_id(data) {
                Some(SEQUENCE_EXTENSION_ID) => {
                    let (chroma_format, vext) = parse_sequence_extension_fields(data)?;
                    let vertical_size = (vext << 12) | vertical_size_value;
                    if vertical_size > 2800 {
                        return Err(Error::InvalidBitstream(
                            "data partitioning: vertical_size > 2800 (slice_vertical_position_extension) is not supported",
                        ));
                    }
                    seq = Some(SequenceSyntax {
                        chroma_format,
                        vertical_size,
                    });
                    w0.write_bytes(data);
                    w1.write_bytes(data);
                    write_data_partitioning_scalable_extension(&mut w0, 0);
                    write_data_partitioning_scalable_extension(&mut w1, 1);
                }
                Some(SEQUENCE_SCALABLE_EXTENSION_ID) => {
                    return Err(Error::InvalidBitstream(
                        "data partitioning: the source stream is already scalable",
                    ));
                }
                Some(PICTURE_CODING_EXTENSION_ID) => {
                    let ext = PictureCodingExtension::parse(data)?;
                    let header = pending_header.take().ok_or(Error::InvalidBitstream(
                        "picture_coding_extension() without a preceding picture_header()",
                    ))?;
                    pic = Some(picture_syntax(&header, &ext));
                    w0.write_bytes(data);
                    w1.write_bytes(data);
                }
                _ => {
                    // §7.10: no other extension is allowed in partition 1.
                    w0.write_bytes(data);
                }
            },
            0x00 => {
                let header = Mpeg2PictureHeader::parse(data)?;
                pending_header = Some(header);
                pic = None;
                w0.write_bytes(data);
                w1.write_bytes(data);
            }
            0xB8 | 0xB7 => {
                w0.write_bytes(data);
                w1.write_bytes(data);
            }
            0xB2 => {
                w0.write_bytes(data);
            }
            code if block.is_slice() => {
                let _ = code;
                let seq = seq.ok_or(Error::InvalidBitstream(
                    "data partitioning: slice before sequence_extension() — only ISO/IEC 13818-2 streams can be partitioned",
                ))?;
                let pic = pic.ok_or(Error::InvalidBitstream(
                    "data partitioning: slice before picture_coding_extension()",
                ))?;
                let header =
                    SliceHeader::parse(data, SliceContext::non_scalable(seq.vertical_size))?;
                // Both partitions carry slice() down to extra_bit_slice.
                for (w, pb) in [(&mut w0, priority_breakpoint), (&mut w1, 0u8)] {
                    w.write_u32(0x0000_0001, 24);
                    w.write_u32(u32::from(header.slice_vertical_position), 8);
                    w.write_u32(u32::from(pb), 7);
                    w.write_u32(u32::from(header.quantiser_scale_code), 5);
                    copy_bits(
                        data,
                        SLICE_TAIL_BIT_NON_SCALABLE,
                        header.body_bit_position,
                        w,
                    )?;
                }
                let mut br = BitReader::new(data);
                br.skip(header.body_bit_position as u32)
                    .map_err(|_| Error::ShortHeader)?;
                let mut io = SplitIo {
                    buf: data,
                    br,
                    priority_breakpoint,
                    w0: &mut w0,
                    w1: &mut w1,
                };
                walk_slice_body(&mut io, &seq, &pic)?;
                let end_pos = io.br.bit_position();
                w0.align_to_byte_zero();
                w1.align_to_byte_zero();
                // Trailing zero stuffing rides partition 0.
                w0.write_bytes(slice_tail(data, end_pos));
            }
            _ => {
                return Err(Error::InvalidBitstream(
                    "data partitioning: unsupported start code in the source stream",
                ));
            }
        }
    }
    if seq.is_none() && pic.is_none() {
        return Err(Error::InvalidBitstream(
            "data partitioning: no sequence_extension() — not an ISO/IEC 13818-2 stream",
        ));
    }
    Ok((w0.finish(), w1.finish()))
}

// -------------------------------------------------------------------
// Merge
// -------------------------------------------------------------------

/// Re-form the non-scalable elementary stream from a §7.10 partition
/// pair — the decoding-process partition switching of §7.10 applied
/// bitstream-to-bitstream. The headers come from partition 0 (the
/// `sequence_scalable_extension()` is dropped, the slice
/// `priority_breakpoint` removed); partition 1's redundant headers
/// are consumed structurally and its slices are paired with partition
/// 0's in order.
///
/// # Errors
/// [`Error::InvalidBitstream`] when partition 0 carries no
/// data-partitioning `sequence_scalable_extension()` (or the wrong
/// `layer_id`), a slice's `priority_breakpoint` is unsupported,
/// partition 1's slice does not carry `priority_breakpoint = 0` or
/// disagrees on `slice_vertical_position` / `quantiser_scale_code`,
/// the slice counts differ, or the macroblock syntax is malformed;
/// [`Error::ShortHeader`] on truncation.
pub fn merge_data_partitions(partition0: &[u8], partition1: &[u8]) -> Result<Vec<u8>> {
    let blocks0 = scan_blocks(partition0)?;
    let blocks1 = scan_blocks(partition1)?;
    let slices1: Vec<StreamBlock> = blocks1.iter().copied().filter(|b| b.is_slice()).collect();
    let mut slice_index = 0usize;

    // Partition 1 must declare itself (layer_id = 1).
    let mut p1_declared = false;
    for b in &blocks1 {
        let data = &partition1[b.start..b.end];
        if b.code == 0xB5 && extension_id(data) == Some(SEQUENCE_SCALABLE_EXTENSION_ID) {
            let sse = SequenceScalableExtension::parse(data)?;
            if sse.scalable_mode != ScalableMode::DataPartitioning || sse.layer_id != 1 {
                return Err(Error::InvalidBitstream(
                    "data partitioning: partition 1 shall declare scalable_mode = data partitioning, layer_id = 1 (§6.3.7)",
                ));
            }
            p1_declared = true;
        }
    }
    if !p1_declared {
        return Err(Error::InvalidBitstream(
            "data partitioning: partition 1 carries no sequence_scalable_extension()",
        ));
    }

    let mut out = BitWriter::new();
    let mut seq: Option<SequenceSyntax> = None;
    let mut vertical_size_value = 0u32;
    let mut pending_header: Option<Mpeg2PictureHeader> = None;
    let mut pic: Option<PictureSyntax> = None;
    let mut declared = false;

    for block in &blocks0 {
        let data = &partition0[block.start..block.end];
        match block.code {
            0xB3 => {
                vertical_size_value = parse_vertical_size_value(data)?;
                seq = None;
                out.write_bytes(data);
            }
            0xB5 => match extension_id(data) {
                Some(SEQUENCE_EXTENSION_ID) => {
                    let (chroma_format, vext) = parse_sequence_extension_fields(data)?;
                    let vertical_size = (vext << 12) | vertical_size_value;
                    if vertical_size > 2800 {
                        return Err(Error::InvalidBitstream(
                            "data partitioning: vertical_size > 2800 is not supported",
                        ));
                    }
                    seq = Some(SequenceSyntax {
                        chroma_format,
                        vertical_size,
                    });
                    out.write_bytes(data);
                }
                Some(SEQUENCE_SCALABLE_EXTENSION_ID) => {
                    let sse = SequenceScalableExtension::parse(data)?;
                    if sse.scalable_mode != ScalableMode::DataPartitioning || sse.layer_id != 0 {
                        return Err(Error::InvalidBitstream(
                            "data partitioning: partition 0 shall declare scalable_mode = data partitioning, layer_id = 0 (§6.3.7)",
                        ));
                    }
                    declared = true;
                    // Dropped: the merged stream is non-scalable.
                }
                Some(PICTURE_CODING_EXTENSION_ID) => {
                    let ext = PictureCodingExtension::parse(data)?;
                    let header = pending_header.take().ok_or(Error::InvalidBitstream(
                        "picture_coding_extension() without a preceding picture_header()",
                    ))?;
                    pic = Some(picture_syntax(&header, &ext));
                    out.write_bytes(data);
                }
                _ => out.write_bytes(data),
            },
            0x00 => {
                pending_header = Some(Mpeg2PictureHeader::parse(data)?);
                pic = None;
                out.write_bytes(data);
            }
            code if block.is_slice() => {
                let _ = code;
                if !declared {
                    return Err(Error::InvalidBitstream(
                        "data partitioning: partition 0 carries no sequence_scalable_extension() before its first slice",
                    ));
                }
                let seq = seq.ok_or(Error::InvalidBitstream(
                    "data partitioning: slice before sequence_extension()",
                ))?;
                let pic = pic.ok_or(Error::InvalidBitstream(
                    "data partitioning: slice before picture_coding_extension()",
                ))?;
                let ctx = SliceContext {
                    vertical_size: seq.vertical_size,
                    priority_breakpoint_present: true,
                };
                let header0 = SliceHeader::parse(data, ctx)?;
                let priority_breakpoint =
                    header0.priority_breakpoint.ok_or(Error::InvalidBitstream(
                        "data partitioning: partition 0 slice without priority_breakpoint",
                    ))?;
                check_breakpoint(priority_breakpoint)?;
                let slice1 = slices1.get(slice_index).ok_or(Error::InvalidBitstream(
                    "data partitioning: partition 1 has fewer slices than partition 0",
                ))?;
                slice_index += 1;
                let data1 = &partition1[slice1.start..slice1.end];
                let header1 = SliceHeader::parse(data1, ctx)?;
                if header1.priority_breakpoint != Some(0) {
                    return Err(Error::InvalidBitstream(
                        "data partitioning: partition 1 slices shall carry priority_breakpoint = 0 (§6.3.16)",
                    ));
                }
                if header1.slice_vertical_position != header0.slice_vertical_position
                    || header1.quantiser_scale_code != header0.quantiser_scale_code
                {
                    return Err(Error::InvalidBitstream(
                        "data partitioning: partition 1 slice header disagrees with partition 0",
                    ));
                }

                out.write_u32(0x0000_0001, 24);
                out.write_u32(u32::from(header0.slice_vertical_position), 8);
                out.write_u32(u32::from(header0.quantiser_scale_code), 5);
                copy_bits(
                    data,
                    SLICE_TAIL_BIT_PARTITIONED,
                    header0.body_bit_position,
                    &mut out,
                )?;

                let mut r0 = BitReader::new(data);
                r0.skip(header0.body_bit_position as u32)
                    .map_err(|_| Error::ShortHeader)?;
                let mut r1 = BitReader::new(data1);
                r1.skip(header1.body_bit_position as u32)
                    .map_err(|_| Error::ShortHeader)?;
                let mut io = MergeIo {
                    buf0: data,
                    r0,
                    buf1: data1,
                    r1,
                    priority_breakpoint,
                    out: &mut out,
                };
                walk_slice_body(&mut io, &seq, &pic)?;
                let end_pos0 = io.r0.bit_position();
                out.align_to_byte_zero();
                out.write_bytes(slice_tail(data, end_pos0));
            }
            _ => out.write_bytes(data),
        }
    }
    if slice_index != slices1.len() {
        return Err(Error::InvalidBitstream(
            "data partitioning: partition 1 has more slices than partition 0",
        ));
    }
    if !declared {
        return Err(Error::InvalidBitstream(
            "data partitioning: partition 0 carries no sequence_scalable_extension()",
        ));
    }
    Ok(out.finish())
}

/// Decode a §7.10 data-partitioned pair: [`merge_data_partitions`]
/// then [`crate::decode_video_sequence`].
///
/// # Errors
/// As the two steps.
pub fn decode_data_partitioned(
    partition0: &[u8],
    partition1: &[u8],
) -> Result<Vec<crate::video_sequence::DecodedFrame>> {
    let merged = merge_data_partitions(partition0, partition1)?;
    crate::video_sequence::decode_video_sequence(&merged)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn copy_bits_moves_arbitrary_spans() {
        let src = [0b1010_1100u8, 0b0011_1111, 0b1000_0001];
        let mut bw = BitWriter::new();
        copy_bits(&src, 3, 3 + 13, &mut bw).unwrap();
        // bits 3..16: 0 1100 0011 1111 → as a 13-bit value.
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        assert_eq!(br.read_u32(13).unwrap(), 0b0_1100_0011_1111);
        assert!(copy_bits(&src, 5, 30, &mut BitWriter::new()).is_err());
        assert!(copy_bits(&src, 9, 8, &mut BitWriter::new()).is_err());
    }

    #[test]
    fn breakpoint_validation_follows_table_7_30() {
        for pb in [1u8, 2, 3, 64, 65, 100, 127] {
            assert!(is_supported_breakpoint(pb), "{pb}");
        }
        for pb in [0u8, 4, 5, 63, 128, 200, 255] {
            assert!(!is_supported_breakpoint(pb), "{pb}");
        }
        assert_eq!(pair_class(1), 64);
        assert_eq!(pair_class(64), 127);
    }

    #[test]
    fn scalable_extension_writer_round_trips() {
        let mut bw = BitWriter::new();
        write_data_partitioning_scalable_extension(&mut bw, 1);
        let bytes = bw.finish();
        assert_eq!(bytes.len(), 6);
        let sse = SequenceScalableExtension::parse(&bytes).unwrap();
        assert_eq!(sse.scalable_mode, ScalableMode::DataPartitioning);
        assert_eq!(sse.layer_id, 1);
    }

    #[test]
    fn split_rejects_bad_inputs() {
        assert!(split_data_partitions(&[], 64).is_err());
        assert!(split_data_partitions(&[0, 0, 1, 0xB3, 0, 0, 0, 0], 0).is_err());
        assert!(split_data_partitions(&[0, 0, 1, 0xB3, 0, 0, 0, 0], 10).is_err());
        assert!(split_data_partitions(&[0x12, 0x34], 64).is_err());
    }

    #[test]
    fn merge_rejects_undeclared_partitions() {
        let hdr = [0u8, 0, 1, 0xB3, 0, 0, 0, 0, 0, 0, 0, 0];
        assert!(merge_data_partitions(&hdr, &hdr).is_err());
        assert!(merge_data_partitions(&[], &hdr).is_err());
    }
}
