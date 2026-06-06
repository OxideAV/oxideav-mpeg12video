//! Parser for the MPEG-2 video `quant_matrix_extension()` element.
//!
//! Implements the bitstream syntax in ISO/IEC 13818-2 (Recommendation
//! ITU-T H.262) §6.2.3.2 and the field semantics in §6.3.11. The
//! `quant_matrix_extension()` is the second of two §6.3.11 download
//! sites at which a sequence may install user-defined weighting
//! matrices (the first being the optional `intra_quantiser_matrix` /
//! `non_intra_quantiser_matrix` trailers of `sequence_header()`
//! itself, §6.2.2.1).
//!
//! Per §6.3.11 a fresh `sequence_header_code` resets every matrix to
//! its default values from §6.3.7 (default intra matrix; default
//! non-intra matrix is all-16). Subsequent `quant_matrix_extension()`
//! occurrences carry up to four independent `load_*` flags, each
//! gating a 64-byte `*_quantiser_matrix` payload that the decoder
//! must inverse-zig-zag (per §7.3.1) into the row-major `W[v][u]`
//! weighting matrix used by §7.4.2.3 inverse quantisation.
//!
//! Four `load_*` slots are defined:
//!
//! 1. `load_intra_quantiser_matrix` — luminance intra matrix
//!    (`w == 0` in Table 7-5).
//! 2. `load_non_intra_quantiser_matrix` — luminance non-intra matrix
//!    (`w == 1`).
//! 3. `load_chroma_intra_quantiser_matrix` — chrominance intra
//!    matrix (`w == 2`, only used at 4:2:2 / 4:4:4).
//! 4. `load_chroma_non_intra_quantiser_matrix` — chrominance
//!    non-intra matrix (`w == 3`, only used at 4:2:2 / 4:4:4).
//!
//! Two `chroma_format`-driven constraints in §6.3.11 are enforced by
//! this parser:
//!
//! * When `chroma_format == 4:2:0`, both
//!   `load_chroma_*_quantiser_matrix` flags shall take the value
//!   `'0'`. A `'1'` is rejected as a bitstream error.
//! * When the luminance matrix is loaded at 4:2:2 / 4:4:4, the new
//!   values "shall be used for both the luminance ... matrix and the
//!   chrominance ... matrix" (the chroma matrix may be subsequently
//!   loaded with a different matrix later in the same extension —
//!   the §6.3.11 ordering puts the chroma-load flags *after* the
//!   luminance-load flags so the spec wording is naturally
//!   compositional). [`QuantiserMatrixState::apply_extension`]
//!   implements that ordering: luminance assignment first, then
//!   optional chrominance override.
//!
//! Two §6.3.11 byte-value rules are also enforced:
//!
//! * Every byte of every loaded matrix must be non-zero (`'value
//!   zero is forbidden'`).
//! * The first byte of an intra matrix (either luminance or
//!   chrominance) must be `8` (`'The first value shall always be
//!   8'`). The §6.3.11 wording attaches this constraint to the
//!   intra slots specifically, so this parser does not impose a
//!   first-byte requirement on the non-intra payloads.
//!
//! The encoded `*_quantiser_matrix[64]` payload sits on the bit
//! stream as 64 contiguous 8-bit unsigned values in default
//! zigzag order (§7.3.1). Because the four pre-payload flags
//! land on the same byte (`4 + 1 + 1 + 1 + 1 = 8` bits when none
//! are loaded; loaded payloads are byte-sized at `8 * 64 = 512`
//! bits each), the parser exits naturally byte-aligned and the
//! caller can drive `next_start_code()` (§5.2.3) without further
//! alignment work.
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)).

use oxideav_core::bits::BitReader;

use crate::mpeg2_dequantize::{DEFAULT_INTRA_WEIGHT, DEFAULT_NON_INTRA_WEIGHT};
use crate::mpeg2_inverse_scan::inverse_scan_table;
use crate::sequence_extension::{ChromaFormat, EXTENSION_START_CODE};
use crate::{Error, Result};

/// `extension_start_code_identifier` value for
/// `quant_matrix_extension()` (Table 6-2 entry `0011`).
pub const QUANT_MATRIX_EXTENSION_ID: u32 = 0b0011;

/// One §7.3.1 default-zigzag-ordered 64-byte payload, as it sits on
/// the wire.
///
/// Values are 8-bit unsigned per §6.3.11; an intra payload's first
/// byte (`zz[0]`, which the §7.3.1 inverse zigzag places at
/// `[v=0][u=0]`) must be `8`, and every byte must be non-zero.
/// [`QuantiserMatrixPayload::to_matrix`] applies the §7.3.1 inverse
/// scan and emits the row-major `W[v][u]` weighting matrix consumed
/// by §7.4.2.3 inverse quantisation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuantiserMatrixPayload {
    /// 64-byte payload in §7.3.1 default-zigzag order. `bytes[i]`
    /// belongs at `[v][u]` where `(v, u) = scan_table(false)[i]`
    /// (i.e. the default scan table from
    /// [`crate::mpeg2_inverse_scan`]).
    pub bytes: [u8; 64],
}

impl QuantiserMatrixPayload {
    /// Materialise the row-major `W[v][u]` matrix by applying the
    /// §7.3.1 default inverse zigzag scan to [`Self::bytes`].
    ///
    /// The scan used is unconditionally the zigzag scan
    /// (`alternate_scan == 0` in §7.3.1 Figure 7-3); §6.3.11
    /// explicitly says "encoded in the default zigzag scanning
    /// order as described in 7.3.1, replace the previous values".
    pub fn to_matrix(self) -> [[u8; 8]; 8] {
        let inv = inverse_scan_table(false);
        let mut matrix = [[0u8; 8]; 8];
        for (i, &byte) in self.bytes.iter().enumerate() {
            let (v, u) = inv[i];
            matrix[v as usize][u as usize] = byte;
        }
        matrix
    }
}

/// Parsed `quant_matrix_extension()` (§6.2.3.2).
///
/// The four optional payloads mirror the §6.3.11 syntax slot-for-
/// slot. A `None` indicates the matching `load_*_quantiser_matrix`
/// flag was `'0'` — i.e. "there is no change in the values that
/// shall be used" (§6.3.11). [`Self::apply`] composes the four
/// optionals over a current [`QuantiserMatrixState`] under the
/// chroma_format-dependent rules of §6.3.11.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct QuantMatrixExtension {
    /// `intra_quantiser_matrix[64]` when
    /// `load_intra_quantiser_matrix == 1`, else `None`.
    pub intra: Option<QuantiserMatrixPayload>,
    /// `non_intra_quantiser_matrix[64]` when
    /// `load_non_intra_quantiser_matrix == 1`, else `None`.
    pub non_intra: Option<QuantiserMatrixPayload>,
    /// `chroma_intra_quantiser_matrix[64]` when
    /// `load_chroma_intra_quantiser_matrix == 1`, else `None`. Per
    /// §6.3.11 this slot shall not be loaded when
    /// `chroma_format == 4:2:0`; [`Self::parse`] enforces that
    /// constraint.
    pub chroma_intra: Option<QuantiserMatrixPayload>,
    /// `chroma_non_intra_quantiser_matrix[64]` when
    /// `load_chroma_non_intra_quantiser_matrix == 1`, else
    /// `None`. Same `chroma_format == 4:2:0` constraint as
    /// `chroma_intra`.
    pub chroma_non_intra: Option<QuantiserMatrixPayload>,
}

impl QuantMatrixExtension {
    /// Parse `quant_matrix_extension()` from a slice that starts
    /// with the four start-code bytes `00 00 01 B5` (§6.3.4). The
    /// trailing `next_start_code()` (§5.2.3) byte-align + zero
    /// stuffing is not consumed.
    ///
    /// `chroma_format` carries the §6.3.5 value parsed earlier from
    /// the active `sequence_extension()` and is needed to apply the
    /// §6.3.11 `4:2:0 ⇒ load_chroma_* == '0'` rejection rule.
    pub fn parse(buf: &[u8], chroma_format: ChromaFormat) -> Result<Self> {
        let mut br = BitReader::new(buf);
        Self::parse_with_reader(&mut br, chroma_format)
    }

    /// Parse from an existing [`BitReader`] positioned at the
    /// start of `quant_matrix_extension()` (i.e. its 32-bit
    /// `extension_start_code`).
    pub fn parse_with_reader(br: &mut BitReader<'_>, chroma_format: ChromaFormat) -> Result<Self> {
        // §6.2.3.2: extension_start_code, value 0x000001B5
        // (§6.3.4 names the constant).
        let code = br.read_u32(32).map_err(|_| Error::ShortHeader)?;
        if code != EXTENSION_START_CODE {
            return Err(Error::InvalidBitstream(
                "extension_start_code: expected 0x000001B5 (§6.3.4)",
            ));
        }

        // 4-bit extension_start_code_identifier; Quant Matrix
        // Extension ID is `0011` per Table 6-2.
        let id = br.read_u32(4).map_err(|_| Error::ShortHeader)?;
        if id != QUANT_MATRIX_EXTENSION_ID {
            return Err(Error::InvalidBitstream(
                "extension_start_code_identifier: expected '0011' Quant Matrix Extension ID (Table 6-2)",
            ));
        }

        let intra = read_matrix_payload_if(br, MatrixKind::Intra)?;
        let non_intra = read_matrix_payload_if(br, MatrixKind::NonIntra)?;

        // §6.3.11: if chroma_format == 4:2:0, both
        // load_chroma_*_quantiser_matrix flags shall take the
        // value '0'.
        let chroma_intra = read_chroma_matrix_payload(br, MatrixKind::Intra, chroma_format)?;
        let chroma_non_intra = read_chroma_matrix_payload(br, MatrixKind::NonIntra, chroma_format)?;

        // §6.3.11 + §6.2.3.2: the four pre-payload flags total 4
        // bits, every loaded payload is 64 * 8 = 512 bits, so we
        // exit byte-aligned regardless of which flags fired.
        debug_assert!(br.is_byte_aligned());

        Ok(Self {
            intra,
            non_intra,
            chroma_intra,
            chroma_non_intra,
        })
    }

    /// Apply this parsed extension to a [`QuantiserMatrixState`]
    /// per §6.3.11.
    ///
    /// Sequencing:
    ///
    /// 1. If `intra` is `Some`, it replaces the luminance intra
    ///    matrix. At 4:2:2 / 4:4:4 the new value also overwrites
    ///    the chrominance intra matrix per §6.3.11.
    /// 2. If `non_intra` is `Some`, same pattern: luminance
    ///    non-intra always, plus chrominance non-intra at 4:2:2 /
    ///    4:4:4.
    /// 3. If `chroma_intra` is `Some`, it overrides the
    ///    chrominance intra matrix only (parsing already rejected
    ///    this slot at 4:2:0).
    /// 4. If `chroma_non_intra` is `Some`, same — chrominance
    ///    non-intra only.
    ///
    /// The order (1, 2, 3, 4) mirrors the §6.3.11 syntax order, so
    /// a stream that loads both `intra_quantiser_matrix` *and*
    /// `chroma_intra_quantiser_matrix` in the same extension ends
    /// with the chroma-specific value installed on the chroma slot,
    /// matching the spec's "may subsequently be loaded with a
    /// different matrix" note.
    pub fn apply(self, state: &mut QuantiserMatrixState, chroma_format: ChromaFormat) {
        let chroma_follows_luma = !matches!(chroma_format, ChromaFormat::Yuv420);

        if let Some(payload) = self.intra {
            let m = payload.to_matrix();
            state.intra_luma = m;
            if chroma_follows_luma {
                state.intra_chroma = m;
            }
        }
        if let Some(payload) = self.non_intra {
            let m = payload.to_matrix();
            state.non_intra_luma = m;
            if chroma_follows_luma {
                state.non_intra_chroma = m;
            }
        }
        if let Some(payload) = self.chroma_intra {
            state.intra_chroma = payload.to_matrix();
        }
        if let Some(payload) = self.chroma_non_intra {
            state.non_intra_chroma = payload.to_matrix();
        }
    }
}

/// Internal selector for the two byte-value rules of §6.3.11.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MatrixKind {
    /// An intra-coded matrix (luminance or chrominance): first
    /// byte must equal `8`, every byte must be non-zero.
    Intra,
    /// A non-intra matrix: every byte must be non-zero, no
    /// first-byte constraint.
    NonIntra,
}

fn read_matrix_payload_if(
    br: &mut BitReader<'_>,
    kind: MatrixKind,
) -> Result<Option<QuantiserMatrixPayload>> {
    let load = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
    if !load {
        return Ok(None);
    }
    Ok(Some(read_matrix_payload(br, kind)?))
}

fn read_chroma_matrix_payload(
    br: &mut BitReader<'_>,
    kind: MatrixKind,
    chroma_format: ChromaFormat,
) -> Result<Option<QuantiserMatrixPayload>> {
    let load = br.read_u32(1).map_err(|_| Error::ShortHeader)? == 1;
    if !load {
        return Ok(None);
    }
    // §6.3.11: "If chroma_format is "4:2:0" this flag shall take
    // the value '0'." A '1' is a bitstream error and is rejected
    // before reading the payload (the payload would not be
    // present in a conformant stream).
    if matches!(chroma_format, ChromaFormat::Yuv420) {
        let msg = match kind {
            MatrixKind::Intra => {
                "load_chroma_intra_quantiser_matrix: shall be '0' when chroma_format == 4:2:0 (§6.3.11)"
            }
            MatrixKind::NonIntra => {
                "load_chroma_non_intra_quantiser_matrix: shall be '0' when chroma_format == 4:2:0 (§6.3.11)"
            }
        };
        return Err(Error::InvalidBitstream(msg));
    }
    Ok(Some(read_matrix_payload(br, kind)?))
}

fn read_matrix_payload(br: &mut BitReader<'_>, kind: MatrixKind) -> Result<QuantiserMatrixPayload> {
    let mut bytes = [0u8; 64];
    for slot in bytes.iter_mut() {
        *slot = br.read_u32(8).map_err(|_| Error::ShortHeader)? as u8;
    }
    // §6.3.11: "For all of the 8-bit unsigned integers, the value
    // zero is forbidden." Applies to every slot in both intra and
    // non-intra payloads.
    if bytes.contains(&0) {
        return Err(Error::InvalidBitstream(
            "quantiser_matrix[64]: value zero is forbidden (§6.3.11)",
        ));
    }
    // §6.3.11: "The first value shall always be 8." Applies to
    // intra_quantiser_matrix and chroma_intra_quantiser_matrix
    // only — the spec attaches this constraint specifically to
    // the intra-matrix paragraphs.
    if matches!(kind, MatrixKind::Intra) && bytes[0] != 8 {
        return Err(Error::InvalidBitstream(
            "intra_quantiser_matrix[0]: shall always be 8 (§6.3.11)",
        ));
    }
    Ok(QuantiserMatrixPayload { bytes })
}

/// The four §7.4.2.1 / Table 7-5 weighting matrices a decoder must
/// carry across a sequence.
///
/// §6.3.11: "Each quantisation matrix has a default set of values.
/// When a sequence_header_code is decoded all matrices shall be
/// reset to their default values." [`Self::reset_to_defaults`]
/// performs that reset; [`QuantMatrixExtension::apply`] applies the
/// per-extension overrides.
///
/// The fields map to the Table 7-5 `w` index:
///
/// * `intra_luma` → `w == 0`
/// * `non_intra_luma` → `w == 1`
/// * `intra_chroma` → `w == 2` (4:2:2 / 4:4:4 only; `w == 0`
///   shares the luma matrix at 4:2:0 per Table 7-5).
/// * `non_intra_chroma` → `w == 3` (same chroma-format gate).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuantiserMatrixState {
    /// `w == 0` — luminance intra matrix.
    pub intra_luma: [[u8; 8]; 8],
    /// `w == 1` — luminance non-intra matrix.
    pub non_intra_luma: [[u8; 8]; 8],
    /// `w == 2` — chrominance intra matrix.
    pub intra_chroma: [[u8; 8]; 8],
    /// `w == 3` — chrominance non-intra matrix.
    pub non_intra_chroma: [[u8; 8]; 8],
}

impl QuantiserMatrixState {
    /// Construct a fresh state with every matrix at its §6.3.7
    /// default. Equivalent to the post-`sequence_header_code`
    /// reset described in §6.3.11.
    pub const fn defaults() -> Self {
        Self {
            intra_luma: DEFAULT_INTRA_WEIGHT,
            non_intra_luma: DEFAULT_NON_INTRA_WEIGHT,
            intra_chroma: DEFAULT_INTRA_WEIGHT,
            non_intra_chroma: DEFAULT_NON_INTRA_WEIGHT,
        }
    }

    /// Reset every matrix back to its §6.3.7 default. Called from
    /// the sequence-layer driver whenever a fresh
    /// `sequence_header_code` is decoded (§6.3.11).
    pub fn reset_to_defaults(&mut self) {
        *self = Self::defaults();
    }
}

impl Default for QuantiserMatrixState {
    fn default() -> Self {
        Self::defaults()
    }
}

#[cfg(test)]
mod tests {
    //! Hand-built bit-exact `quant_matrix_extension()` fixtures plus
    //! negative cases for every §6.3.11 rejection site this parser
    //! introduces.
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// §6.3.7 default intra matrix encoded back into §7.3.1
    /// zigzag-scan order. Used both as a positive-load fixture and
    /// to test the inverse-scan helper.
    fn default_intra_in_zigzag_order() -> [u8; 64] {
        let inv = inverse_scan_table(false);
        let mut bytes = [0u8; 64];
        for (i, &(v, u)) in inv.iter().enumerate() {
            bytes[i] = DEFAULT_INTRA_WEIGHT[v as usize][u as usize];
        }
        bytes
    }

    /// §6.3.7 default non-intra matrix (every entry is 16) is
    /// scan-invariant — useful as a positive-load fixture for the
    /// non-intra path.
    fn default_non_intra_in_zigzag_order() -> [u8; 64] {
        [16u8; 64]
    }

    /// Emit `quant_matrix_extension()` with caller-controlled
    /// `load_*` flags + payloads. Pads to byte boundary so the
    /// caller can splice in `next_start_code()` zero stuffing.
    fn write_quant_matrix_extension(
        bw: &mut BitWriter,
        intra: Option<&[u8; 64]>,
        non_intra: Option<&[u8; 64]>,
        chroma_intra: Option<&[u8; 64]>,
        chroma_non_intra: Option<&[u8; 64]>,
    ) {
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(QUANT_MATRIX_EXTENSION_ID, 4);
        bw.write_bit(intra.is_some());
        if let Some(payload) = intra {
            for &b in payload.iter() {
                bw.write_u32(u32::from(b), 8);
            }
        }
        bw.write_bit(non_intra.is_some());
        if let Some(payload) = non_intra {
            for &b in payload.iter() {
                bw.write_u32(u32::from(b), 8);
            }
        }
        bw.write_bit(chroma_intra.is_some());
        if let Some(payload) = chroma_intra {
            for &b in payload.iter() {
                bw.write_u32(u32::from(b), 8);
            }
        }
        bw.write_bit(chroma_non_intra.is_some());
        if let Some(payload) = chroma_non_intra {
            for &b in payload.iter() {
                bw.write_u32(u32::from(b), 8);
            }
        }
        bw.align_to_byte();
    }

    #[test]
    fn parses_extension_with_no_loads() {
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, None, None, None, None);
        let bytes = bw.finish();
        let ext = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv420).expect("parse");
        assert_eq!(ext, QuantMatrixExtension::default());
    }

    #[test]
    fn parses_extension_with_intra_load() {
        let payload = default_intra_in_zigzag_order();
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, Some(&payload), None, None, None);
        let bytes = bw.finish();
        let ext = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv420).expect("parse");
        let parsed = ext.intra.expect("intra payload present");
        assert_eq!(parsed.bytes, payload);
        assert_eq!(parsed.to_matrix(), DEFAULT_INTRA_WEIGHT);
        assert!(ext.non_intra.is_none());
        assert!(ext.chroma_intra.is_none());
        assert!(ext.chroma_non_intra.is_none());
    }

    #[test]
    fn parses_extension_with_non_intra_load() {
        let payload = default_non_intra_in_zigzag_order();
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, None, Some(&payload), None, None);
        let bytes = bw.finish();
        let ext = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv420).expect("parse");
        let parsed = ext.non_intra.expect("non-intra payload present");
        assert_eq!(parsed.bytes, payload);
        assert_eq!(parsed.to_matrix(), DEFAULT_NON_INTRA_WEIGHT);
    }

    #[test]
    fn parses_chroma_loads_at_422() {
        let payload_i = default_intra_in_zigzag_order();
        let payload_ni = default_non_intra_in_zigzag_order();
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, None, None, Some(&payload_i), Some(&payload_ni));
        let bytes = bw.finish();
        let ext = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv422).expect("parse");
        assert!(ext.chroma_intra.is_some());
        assert!(ext.chroma_non_intra.is_some());
    }

    #[test]
    fn parses_chroma_loads_at_444() {
        let payload_i = default_intra_in_zigzag_order();
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, None, None, Some(&payload_i), None);
        let bytes = bw.finish();
        let ext = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv444).expect("parse");
        assert!(ext.chroma_intra.is_some());
    }

    #[test]
    fn parses_all_four_loads_simultaneously() {
        let intra = default_intra_in_zigzag_order();
        let non_intra = default_non_intra_in_zigzag_order();
        // Distinct chroma intra payload: still starts with 8 and
        // has no zero bytes, but uses a non-default tail so we can
        // distinguish a copy-from-luma override mistake.
        let mut chroma_intra = default_intra_in_zigzag_order();
        chroma_intra[63] = chroma_intra[63].saturating_add(1);
        let chroma_non_intra = default_non_intra_in_zigzag_order();
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(
            &mut bw,
            Some(&intra),
            Some(&non_intra),
            Some(&chroma_intra),
            Some(&chroma_non_intra),
        );
        let bytes = bw.finish();
        let ext = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv444).expect("parse");
        assert_eq!(ext.intra.unwrap().bytes, intra);
        assert_eq!(ext.non_intra.unwrap().bytes, non_intra);
        assert_eq!(ext.chroma_intra.unwrap().bytes, chroma_intra);
        assert_eq!(ext.chroma_non_intra.unwrap().bytes, chroma_non_intra);
    }

    #[test]
    fn rejects_chroma_intra_load_at_420() {
        let payload = default_intra_in_zigzag_order();
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, None, None, Some(&payload), None);
        let bytes = bw.finish();
        let err = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv420).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_chroma_non_intra_load_at_420() {
        let payload = default_non_intra_in_zigzag_order();
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, None, None, None, Some(&payload));
        let bytes = bw.finish();
        let err = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv420).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_zero_byte_in_intra_payload() {
        let mut payload = default_intra_in_zigzag_order();
        // Plant a forbidden zero somewhere past the first byte;
        // the first-byte-must-be-8 rule wouldn't fire here.
        payload[7] = 0;
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, Some(&payload), None, None, None);
        let bytes = bw.finish();
        let err = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv420).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_zero_byte_in_non_intra_payload() {
        let mut payload = default_non_intra_in_zigzag_order();
        payload[42] = 0;
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, None, Some(&payload), None, None);
        let bytes = bw.finish();
        let err = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv420).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_intra_first_byte_other_than_8() {
        let mut payload = default_intra_in_zigzag_order();
        payload[0] = 16;
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, Some(&payload), None, None, None);
        let bytes = bw.finish();
        let err = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv420).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_chroma_intra_first_byte_other_than_8() {
        let mut payload = default_intra_in_zigzag_order();
        payload[0] = 4;
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, None, None, Some(&payload), None);
        let bytes = bw.finish();
        let err = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv422).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_wrong_extension_start_code() {
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, None, None, None, None);
        let mut bytes = bw.finish();
        bytes[3] = 0xB3; // sequence_header_code instead
        let err = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv420).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn rejects_wrong_extension_identifier() {
        // Build manually with the identifier set to '0001'
        // (Sequence Extension ID) rather than '0011'.
        let mut bw = BitWriter::new();
        bw.write_u32(EXTENSION_START_CODE, 32);
        bw.write_u32(0b0001, 4);
        bw.align_to_byte();
        let bytes = bw.finish();
        let err = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv420).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn short_buffer_returns_short_header() {
        // Two bytes of start code only — no identifier yet.
        let bytes = [0u8, 0u8];
        let err = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv420).unwrap_err();
        assert!(matches!(err, Error::ShortHeader));
    }

    #[test]
    fn default_state_matches_section_6_3_7_defaults() {
        let state = QuantiserMatrixState::defaults();
        assert_eq!(state.intra_luma, DEFAULT_INTRA_WEIGHT);
        assert_eq!(state.non_intra_luma, DEFAULT_NON_INTRA_WEIGHT);
        assert_eq!(state.intra_chroma, DEFAULT_INTRA_WEIGHT);
        assert_eq!(state.non_intra_chroma, DEFAULT_NON_INTRA_WEIGHT);
    }

    #[test]
    fn reset_returns_state_to_defaults() {
        let mut state = QuantiserMatrixState::defaults();
        // Mutate so we can confirm the reset actually overwrites.
        state.intra_luma[0][0] = 99;
        state.non_intra_chroma[7][7] = 99;
        state.reset_to_defaults();
        assert_eq!(state, QuantiserMatrixState::defaults());
    }

    #[test]
    fn apply_intra_load_copies_to_chroma_at_422() {
        // §6.3.11: when chroma_format is 4:2:2 / 4:4:4, the new
        // intra value "shall be used for both the luminance intra
        // matrix and the chrominance intra matrix".
        let intra = default_intra_in_zigzag_order();
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, Some(&intra), None, None, None);
        let bytes = bw.finish();
        let ext = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv422).expect("parse");

        // Start from a state whose chroma slot has been mutated
        // away from the default so we can confirm the luma load
        // actually overwrites it.
        let mut state = QuantiserMatrixState::defaults();
        state.intra_chroma[3][4] = 99;
        ext.apply(&mut state, ChromaFormat::Yuv422);
        assert_eq!(state.intra_luma, DEFAULT_INTRA_WEIGHT);
        assert_eq!(state.intra_chroma, DEFAULT_INTRA_WEIGHT);
    }

    #[test]
    fn apply_intra_load_skips_chroma_at_420() {
        // §6.3.11: with 4:2:0 data only two matrices are used; the
        // chroma slot is irrelevant. We still keep the field in
        // [`QuantiserMatrixState`] but the §6.3.11 paragraph that
        // says the luma load "shall be used for both" matrices
        // applies only at 4:2:2 / 4:4:4, so the chroma slot at
        // 4:2:0 stays untouched.
        let intra = default_intra_in_zigzag_order();
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, Some(&intra), None, None, None);
        let bytes = bw.finish();
        let ext = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv420).expect("parse");

        let mut state = QuantiserMatrixState::defaults();
        state.intra_chroma[3][4] = 99;
        ext.apply(&mut state, ChromaFormat::Yuv420);
        // Luma was overwritten, chroma kept the prior mutated
        // value (i.e. was NOT replaced by the luma load).
        assert_eq!(state.intra_luma, DEFAULT_INTRA_WEIGHT);
        assert_eq!(state.intra_chroma[3][4], 99);
    }

    #[test]
    fn apply_chroma_intra_load_overrides_luma_copy() {
        // Order is luma-load first, chroma-load second per the
        // §6.3.11 syntax. So a same-extension intra + chroma_intra
        // pair ends with the chroma payload installed on the
        // chroma slot, not the luma payload.
        let luma_payload = default_intra_in_zigzag_order();
        let mut chroma_payload = default_intra_in_zigzag_order();
        chroma_payload[63] = chroma_payload[63].saturating_add(1);

        let mut bw = BitWriter::new();
        write_quant_matrix_extension(
            &mut bw,
            Some(&luma_payload),
            None,
            Some(&chroma_payload),
            None,
        );
        let bytes = bw.finish();
        let ext = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv444).expect("parse");

        let mut state = QuantiserMatrixState::defaults();
        ext.apply(&mut state, ChromaFormat::Yuv444);

        let expected_chroma = ext.chroma_intra.unwrap().to_matrix();
        assert_eq!(state.intra_luma, DEFAULT_INTRA_WEIGHT);
        assert_eq!(state.intra_chroma, expected_chroma);
        assert_ne!(state.intra_chroma, state.intra_luma);
    }

    #[test]
    fn apply_non_intra_load_copies_to_chroma_at_444() {
        let non_intra = default_non_intra_in_zigzag_order();
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, None, Some(&non_intra), None, None);
        let bytes = bw.finish();
        let ext = QuantMatrixExtension::parse(&bytes, ChromaFormat::Yuv444).expect("parse");
        let mut state = QuantiserMatrixState::defaults();
        state.non_intra_chroma[0][0] = 99;
        ext.apply(&mut state, ChromaFormat::Yuv444);
        assert_eq!(state.non_intra_luma, DEFAULT_NON_INTRA_WEIGHT);
        assert_eq!(state.non_intra_chroma, DEFAULT_NON_INTRA_WEIGHT);
    }

    #[test]
    fn payload_to_matrix_round_trips_default_intra() {
        let payload = QuantiserMatrixPayload {
            bytes: default_intra_in_zigzag_order(),
        };
        assert_eq!(payload.to_matrix(), DEFAULT_INTRA_WEIGHT);
    }

    #[test]
    fn parser_exits_byte_aligned_for_no_loads() {
        // 32 (start code) + 4 (id) + 4 (load flags) = 40 bits → 5
        // bytes. With no loads the total extension is exactly 5
        // bytes.
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(&mut bw, None, None, None, None);
        let bytes = bw.finish();
        assert_eq!(bytes.len(), 5);
    }

    #[test]
    fn parser_exits_byte_aligned_for_full_loads() {
        // 5 bytes pre/per-flag bookkeeping + 4 * 64 = 261 bytes.
        let intra = default_intra_in_zigzag_order();
        let non_intra = default_non_intra_in_zigzag_order();
        let mut chroma_intra = default_intra_in_zigzag_order();
        chroma_intra[63] = chroma_intra[63].saturating_add(1);
        let chroma_non_intra = default_non_intra_in_zigzag_order();
        let mut bw = BitWriter::new();
        write_quant_matrix_extension(
            &mut bw,
            Some(&intra),
            Some(&non_intra),
            Some(&chroma_intra),
            Some(&chroma_non_intra),
        );
        let bytes = bw.finish();
        assert_eq!(bytes.len(), 5 + 4 * 64);
    }
}
