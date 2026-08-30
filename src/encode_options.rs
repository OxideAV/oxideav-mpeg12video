//! Optional frame-picture **encode** behaviours layered over the
//! baseline I / P / B frame-picture encoders: skipped-macroblock
//! emission (§7.6.6), concealment motion vectors (§7.6.3.9), and the
//! §6.3.10 `top_field_first` / `repeat_first_field` /
//! `progressive_frame` output-cadence signalling.
//!
//! The baseline encoders keep their historical signatures (every
//! macroblock coded, no concealment vectors, `top_field_first = 0`,
//! `repeat_first_field = 0`, `progressive_frame = progressive_sequence`);
//! the `_with_options` variants take a [`FrameEncodeOptions`].
//!
//! ## Skipped macroblocks (§7.6.6)
//!
//! A skipped macroblock carries no data at all — the next coded
//! macroblock's `macroblock_address_increment` (Table B-1, with the
//! `macroblock_escape` chain for runs longer than 33) counts it. The
//! encoder may skip a macroblock exactly when the decoder's mandated
//! reconstruction for a skip equals the reconstruction it would have
//! coded:
//!
//! * **P frame picture** (§7.6.6.2): the prediction is `Frame-based`
//!   from the forward reference with a **zero** vector, and the
//!   predictors reset. A `MC, Not Coded` macroblock whose vector is
//!   `(0, 0)` reconstructs identically (its PMV write-back is `(0, 0)`
//!   too), so it is skipped instead.
//! * **B frame picture** (§7.6.6.4): the prediction direction is the
//!   **previous macroblock's**, the vectors are the current
//!   predictors (`PMV`), and the predictors are unaffected. The encoder
//!   therefore tests "previous direction with the PMV vectors" first:
//!   if that prediction quantises to an all-zero residual the
//!   macroblock is skipped.
//!
//! The §6.3.17 restrictions are honoured: the first and last macroblock
//! of a slice are never skipped, a B-picture never skips immediately
//! after an intra macroblock, and I-pictures never skip.
//!
//! ## Concealment motion vectors (§7.6.3.9)
//!
//! With `concealment_motion_vectors = 1` every intra macroblock carries
//! a `motion_vectors(0)` block (frame-format, coded against
//! `PMV[0][0]` exactly like a P macroblock's forward vector, Table
//! 7-9 `Frame-based‡ intra` row: `PMV[1][0] = PMV[0][0]`) followed by
//! a `marker_bit`. Per the §7.6.3.9 recommendation the vector is chosen
//! to suit the macroblock **below** (a decoder concealing a lost slice
//! predicts it from the slice above), and the bottom row carries
//! `(0, 0)`. The vector is searched against the reference the decoder
//! would conceal from (the previous anchor); with no reference in reach
//! the vector is `(0, 0)`.
//!
//! ## Output cadence flags (§6.3.10)
//!
//! `top_field_first` / `repeat_first_field` / `progressive_frame` do
//! not affect reconstruction; they tell the display process how many
//! fields / frames to output. The options are validated against the
//! §6.3.10 rules by
//! [`crate::stream_writer::PictureCodingExtensionParams::validate_frame_picture_flags`].

use crate::frame_assembly::IntraPictureParams;
use crate::stream_writer::PictureCodingExtensionParams;
use crate::Result;

/// Per-picture macroblock decision counts reported by the
/// `_with_stats` frame-picture encoders.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FrameEncodeStats {
    /// Macroblocks coded with a residual (`macroblock_pattern = 1`).
    pub coded: usize,
    /// Motion-compensated macroblocks coded without a residual.
    pub not_coded: usize,
    /// Intra-coded macroblocks.
    pub intra: usize,
    /// §7.6.6 skipped macroblocks (no data on the wire).
    pub skipped: usize,
}

impl FrameEncodeStats {
    /// Accumulate another picture's counts.
    pub fn add(&mut self, other: &FrameEncodeStats) {
        self.coded += other.coded;
        self.not_coded += other.not_coded;
        self.intra += other.intra;
        self.skipped += other.skipped;
    }

    /// Total macroblocks counted.
    pub fn total(&self) -> usize {
        self.coded + self.not_coded + self.intra + self.skipped
    }
}

/// Optional behaviours for the frame-picture encoders.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FrameEncodeOptions {
    /// Emit §7.6.6 skipped macroblocks where the skip reconstruction
    /// equals the coded one.
    pub skipped_macroblocks: bool,
    /// Set `concealment_motion_vectors` and code a §7.6.3.9 vector +
    /// `marker_bit` on every intra macroblock.
    pub concealment_motion_vectors: bool,
    /// `top_field_first` (§6.3.10).
    pub top_field_first: bool,
    /// `repeat_first_field` (§6.3.10).
    pub repeat_first_field: bool,
    /// `progressive_frame` override; `None` inherits
    /// `progressive_sequence` (the baseline behaviour).
    pub progressive_frame: Option<bool>,
}

impl FrameEncodeOptions {
    /// The `progressive_frame` value to signal for a picture of
    /// `params` (§6.3.10: a progressive sequence's frames are always
    /// progressive; the override only matters in interlaced sequences).
    pub fn resolved_progressive_frame(&self, params: &IntraPictureParams) -> bool {
        self.progressive_frame
            .unwrap_or(params.progressive_sequence)
    }

    /// Fill a [`PictureCodingExtensionParams`] with the flags these
    /// options imply for a frame picture of `params` and check the
    /// §6.3.10 consistency rules.
    ///
    /// # Errors
    /// [`crate::Error::InvalidBitstream`] on a §6.3.10 violation.
    pub fn picture_coding_extension(
        &self,
        params: &IntraPictureParams,
        forward_f_code: u8,
        backward_f_code: u8,
    ) -> Result<PictureCodingExtensionParams> {
        let p = PictureCodingExtensionParams {
            forward_f_code,
            backward_f_code,
            intra_dc_precision: params.intra_dc_precision,
            frame_pred_frame_dct: params.frame_pred_frame_dct,
            q_scale_type: params.q_scale_type,
            intra_vlc_format: params.intra_vlc_format,
            alternate_scan: params.alternate_scan,
            progressive_frame: self.resolved_progressive_frame(params),
            top_field_first: self.top_field_first,
            repeat_first_field: self.repeat_first_field,
            concealment_motion_vectors: self.concealment_motion_vectors,
            chroma_format: params.chroma_format,
        };
        p.validate_frame_picture_flags(params.progressive_sequence)?;
        Ok(p)
    }

    /// A 3:2-pulldown style pattern helper: the flags for display frame
    /// `index` of a film-rate source signalled in an interlaced
    /// sequence at the higher field rate — alternating
    /// `repeat_first_field` with `top_field_first` toggling every
    /// repeated frame (`progressive_frame = 1` throughout, as §6.3.10
    /// requires for a repeated field).
    pub fn pulldown_32(index: usize) -> Self {
        // Frames A B C D → A: TFF=1 RFF=1 (3 fields), B: TFF=0 RFF=0
        // (2 fields), C: TFF=0 RFF=1 (3 fields), D: TFF=1 RFF=0 (2
        // fields); the field parity alternates so the field sequence
        // stays continuous (T B T | B T | B T B | T B).
        let (tff, rff) = match index % 4 {
            0 => (true, true),
            1 => (false, false),
            2 => (false, true),
            _ => (true, false),
        };
        Self {
            top_field_first: tff,
            repeat_first_field: rff,
            progressive_frame: Some(true),
            ..Self::default()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sequence_extension::ChromaFormat;

    fn params(progressive_sequence: bool) -> IntraPictureParams {
        IntraPictureParams {
            width: 32,
            height: 32,
            chroma_format: ChromaFormat::Yuv420,
            frame_pred_frame_dct: true,
            intra_dc_precision: 0,
            intra_vlc_format: false,
            alternate_scan: false,
            q_scale_type: false,
            progressive_sequence,
        }
    }

    #[test]
    fn defaults_are_the_baseline_flags() {
        let p = FrameEncodeOptions::default()
            .picture_coding_extension(&params(true), 3, 15)
            .unwrap();
        assert!(p.progressive_frame);
        assert!(!p.top_field_first);
        assert!(!p.repeat_first_field);
        assert!(!p.concealment_motion_vectors);
        assert!(p.chroma_420_type());
    }

    #[test]
    fn progressive_sequence_rejects_tff_without_rff() {
        let o = FrameEncodeOptions {
            top_field_first: true,
            ..Default::default()
        };
        assert!(o.picture_coding_extension(&params(true), 3, 15).is_err());
        let o = FrameEncodeOptions {
            top_field_first: true,
            repeat_first_field: true,
            ..Default::default()
        };
        assert!(o.picture_coding_extension(&params(true), 3, 15).is_ok());
    }

    #[test]
    fn interlaced_frame_rejects_rff_unless_progressive_frame() {
        let o = FrameEncodeOptions {
            repeat_first_field: true,
            ..Default::default()
        };
        assert!(o.picture_coding_extension(&params(false), 3, 15).is_err());
        let o = FrameEncodeOptions {
            repeat_first_field: true,
            progressive_frame: Some(true),
            ..Default::default()
        };
        assert!(o.picture_coding_extension(&params(false), 3, 15).is_ok());
    }

    #[test]
    fn progressive_sequence_rejects_progressive_frame_zero() {
        let o = FrameEncodeOptions {
            progressive_frame: Some(false),
            ..Default::default()
        };
        assert!(o.picture_coding_extension(&params(true), 3, 15).is_err());
    }

    #[test]
    fn pulldown_pattern_is_consistent_in_an_interlaced_sequence() {
        for i in 0..8 {
            let o = FrameEncodeOptions::pulldown_32(i);
            o.picture_coding_extension(&params(false), 3, 15)
                .expect("pulldown flags are §6.3.10-consistent");
        }
        // Field counts over one 4-frame period: 3 + 2 + 3 + 2 = 10.
        let fields: usize = (0..4)
            .map(|i| {
                let o = FrameEncodeOptions::pulldown_32(i);
                if o.repeat_first_field {
                    3
                } else {
                    2
                }
            })
            .sum();
        assert_eq!(fields, 10);
    }
}
