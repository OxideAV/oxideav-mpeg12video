//! Sequence-layer ordering driver for the MPEG-2 video
//! `sequence_display_extension()` / `picture_display_extension()` pair.
//!
//! Implements the two cross-element occurrence constraints ISO/IEC
//! 13818-2 (Recommendation ITU-T H.262) attaches to
//! `sequence_display_extension()` — the §6.3.5 repeat-sequence-header
//! consistency rule and the §6.3.12 `picture_display_extension()`
//! ordering gate. Both rules bind the *presence* (and, for §6.3.5, the
//! *value*) of `sequence_display_extension()` across the bitstream, so
//! they live one layer above the per-element
//! [`crate::sequence_display_extension`] parser. This module owns the
//! running presence fact and exposes the two checks as named methods,
//! mirroring the [`crate::FrameCentreOffsetDriver`] /
//! [`crate::QuantMatrixDriver`] driver shape so a picture-level caller
//! threading the spec flow never has to re-derive the constraints.
//!
//! ## §6.3.5 repeat-sequence-header consistency
//!
//! §6.3.5 binds the extension's appearance across repeat sequence
//! headers: *"If a `sequence_display_extension()` occurs after the
//! first `sequence_header()` all subsequent sequence headers shall be
//! followed by `sequence_display_extension()` in which all data
//! elements are the same as in the first
//! `sequence_display_extension()`. Conversely if no
//! `sequence_display_extension()` occurs between the first
//! `sequence_header()` and the first `picture_header()` then
//! `sequence_display_extension()` shall not occur in the bitstream."*
//!
//! The driver models that as: the **first** observed
//! `sequence_display_extension()` (between the first `sequence_header()`
//! and the first `picture_header()`) pins the required presence and the
//! required value for every subsequent sequence header. After the
//! presence is pinned, [`Self::on_sequence_header_window`] checks the
//! next window's `Option<SequenceDisplayExtension>` against the pinned
//! requirement: a missing extension where one is required is a §6.3.5
//! violation; a present extension where none is permitted is a §6.3.5
//! violation; a present extension whose data elements differ from the
//! first is a §6.3.5 violation.
//!
//! ## §6.3.12 `picture_display_extension()` ordering gate
//!
//! §6.3.12 requires that *"a `picture_display_extension()` shall not
//! occur unless a `sequence_display_extension()` followed the previous
//! `sequence_header()`"*. The driver tracks whether the most recent
//! `sequence_header()` was followed by a `sequence_display_extension()`
//! and answers [`Self::picture_display_extension_permitted`] for the
//! current sequence. A caller that decodes a
//! `picture_display_extension()` while this returns `false` has a
//! §6.3.12 ordering violation; [`Self::check_picture_display_extension`]
//! folds the answer into a `Result` for callers that want the
//! [`crate::Error::InvalidBitstream`] directly.
//!
//! Spec citations refer to the 1995 base text of ISO/IEC 13818-2
//! (Recommendation ITU-T H.262 (1995 E)).

use crate::sequence_display_extension::SequenceDisplayExtension;
use crate::{Error, Result};

/// What the §6.3.5 rule has pinned for `sequence_display_extension()`
/// across the sequence's repeat sequence headers.
///
/// Until the **first** sequence-header window completes (between the
/// first `sequence_header()` and the first `picture_header()`), the
/// requirement is [`Requirement::Unpinned`]. The first window's
/// observation then pins one of the two terminal requirements per
/// §6.3.5.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Requirement {
    /// No sequence-header window has completed yet; the §6.3.5 rule has
    /// not been pinned. This is the post-construction / pre-first-window
    /// state.
    #[default]
    Unpinned,
    /// The first window carried **no** `sequence_display_extension()`,
    /// so §6.3.5's *"`sequence_display_extension()` shall not occur in
    /// the bitstream"* clause forbids it for every subsequent window.
    Forbidden,
    /// The first window carried a `sequence_display_extension()`, so
    /// §6.3.5 requires every subsequent sequence header be followed by
    /// a `sequence_display_extension()` whose data elements equal the
    /// pinned first value.
    RequiredEqual(SequenceDisplayExtension),
}

/// Sequence-layer ordering driver owning the §6.3.5 / §6.3.12
/// occurrence state across one MPEG-2 video sequence.
///
/// The driver is `Copy + Default`; a freshly-`Default`-ed driver is
/// byte-identical to [`Self::new`] and sits at the
/// pre-first-sequence-header baseline (requirement [`Requirement::Unpinned`],
/// `picture_display_extension()` not yet permitted).
///
/// ## Lifecycle
///
/// ```text
/// // First sequence-header window (sequence_header() up to the first
/// // picture_header()): supply the Option<SequenceDisplayExtension>
/// // that followed it. This pins the §6.3.5 requirement and records
/// // the §6.3.12 presence fact for this sequence.
/// driver.on_sequence_header_window(maybe_first_ext)?;
///
/// // Each later sequence_header() repeat is checked against the pin:
/// driver.on_sequence_header_window(maybe_repeat_ext)?;
///
/// // Before decoding a picture_display_extension(), confirm §6.3.12:
/// driver.check_picture_display_extension()?;
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SequenceDisplayOrderDriver {
    /// The §6.3.5 requirement pinned by the first sequence-header
    /// window (or [`Requirement::Unpinned`] before that window).
    requirement: Requirement,
    /// §6.3.12: did the most-recently-observed `sequence_header()`
    /// window carry a `sequence_display_extension()`? Drives
    /// [`Self::picture_display_extension_permitted`]. `false` before
    /// any window has been observed.
    sequence_display_present: bool,
}

impl SequenceDisplayOrderDriver {
    /// Construct a fresh driver at the pre-first-`sequence_header()`
    /// baseline: requirement [`Requirement::Unpinned`] and no
    /// `sequence_display_extension()` observed yet (so a
    /// `picture_display_extension()` is not yet permitted per §6.3.12).
    pub const fn new() -> Self {
        Self {
            requirement: Requirement::Unpinned,
            sequence_display_present: false,
        }
    }

    /// The §6.3.5 requirement pinned so far.
    pub const fn requirement(&self) -> Requirement {
        self.requirement
    }

    /// §6.3.12: whether a `picture_display_extension()` is permitted in
    /// the current sequence — i.e. whether the most recent
    /// `sequence_header()` was followed by a
    /// `sequence_display_extension()`.
    pub const fn picture_display_extension_permitted(&self) -> bool {
        self.sequence_display_present
    }

    /// Observe one sequence-header window: the
    /// `Option<SequenceDisplayExtension>` that followed a
    /// `sequence_header()` (up to the next `picture_header()`).
    ///
    /// The first call pins the §6.3.5 requirement
    /// ([`Requirement::Forbidden`] when `ext` is `None`,
    /// [`Requirement::RequiredEqual`] when `ext` is `Some`). Every
    /// subsequent call is checked against the pin:
    ///
    /// * Pinned [`Requirement::Forbidden`] but `ext` is `Some` →
    ///   §6.3.5 *"`sequence_display_extension()` shall not occur"*
    ///   violation.
    /// * Pinned [`Requirement::RequiredEqual`] but `ext` is `None` →
    ///   §6.3.5 *"all subsequent sequence headers shall be followed by
    ///   `sequence_display_extension()`"* violation.
    /// * Pinned [`Requirement::RequiredEqual(first)`] and `ext` is
    ///   `Some(other)` with `other != first` → §6.3.5 *"all data
    ///   elements are the same as in the first
    ///   `sequence_display_extension()`"* violation.
    ///
    /// On success the §6.3.12 presence fact is updated for the current
    /// sequence so [`Self::picture_display_extension_permitted`]
    /// reflects this window.
    pub fn on_sequence_header_window(
        &mut self,
        ext: Option<SequenceDisplayExtension>,
    ) -> Result<()> {
        match self.requirement {
            Requirement::Unpinned => {
                // First window pins the §6.3.5 requirement.
                self.requirement = match ext {
                    Some(first) => Requirement::RequiredEqual(first),
                    None => Requirement::Forbidden,
                };
            }
            Requirement::Forbidden => {
                if ext.is_some() {
                    return Err(Error::InvalidBitstream(
                        "sequence_display_extension(): shall not occur when the first \
                         sequence_header() had none (§6.3.5)",
                    ));
                }
            }
            Requirement::RequiredEqual(first) => match ext {
                None => {
                    return Err(Error::InvalidBitstream(
                        "sequence_display_extension(): every repeat sequence_header() shall be \
                         followed by it once the first one carried it (§6.3.5)",
                    ));
                }
                Some(other) if other != first => {
                    return Err(Error::InvalidBitstream(
                        "sequence_display_extension(): all data elements shall equal the first \
                         occurrence's (§6.3.5)",
                    ));
                }
                Some(_) => {}
            },
        }
        // §6.3.12: record whether this window carried the extension.
        self.sequence_display_present = ext.is_some();
        Ok(())
    }

    /// Fold [`Self::picture_display_extension_permitted`] into a
    /// `Result` for the §6.3.12 ordering gate: returns
    /// [`Error::InvalidBitstream`] when a `picture_display_extension()`
    /// would occur without a preceding `sequence_display_extension()`
    /// after the current `sequence_header()`.
    pub fn check_picture_display_extension(&self) -> Result<()> {
        if self.picture_display_extension_permitted() {
            Ok(())
        } else {
            Err(Error::InvalidBitstream(
                "picture_display_extension(): shall not occur unless a \
                 sequence_display_extension() followed the previous sequence_header() (§6.3.12)",
            ))
        }
    }
}

impl Default for SequenceDisplayOrderDriver {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    //! Cover the §6.3.5 pin matrix and the §6.3.12 ordering gate.
    use super::*;
    use crate::sequence_display_extension::{ColourDescription, VideoFormat};

    fn ext(video_format: VideoFormat, dhs: u16, dvs: u16) -> SequenceDisplayExtension {
        SequenceDisplayExtension {
            video_format,
            colour_description: None,
            display_horizontal_size: dhs,
            display_vertical_size: dvs,
        }
    }

    fn ext_with_colour(colour: ColourDescription) -> SequenceDisplayExtension {
        SequenceDisplayExtension {
            video_format: VideoFormat::Ntsc,
            colour_description: Some(colour),
            display_horizontal_size: 352,
            display_vertical_size: 240,
        }
    }

    // ---- Baseline -------------------------------------------------

    #[test]
    fn new_is_unpinned_and_forbids_picture_display() {
        let d = SequenceDisplayOrderDriver::new();
        assert_eq!(d.requirement(), Requirement::Unpinned);
        assert!(!d.picture_display_extension_permitted());
        assert!(d.check_picture_display_extension().is_err());
    }

    #[test]
    fn default_matches_new() {
        assert_eq!(
            SequenceDisplayOrderDriver::default(),
            SequenceDisplayOrderDriver::new(),
        );
    }

    // ---- §6.3.5 first-window pin ----------------------------------

    #[test]
    fn first_window_none_pins_forbidden() {
        let mut d = SequenceDisplayOrderDriver::new();
        d.on_sequence_header_window(None).expect("first window");
        assert_eq!(d.requirement(), Requirement::Forbidden);
        // §6.3.12: no extension this window -> picture_display forbidden.
        assert!(!d.picture_display_extension_permitted());
    }

    #[test]
    fn first_window_some_pins_required_equal() {
        let e = ext(VideoFormat::Pal, 720, 576);
        let mut d = SequenceDisplayOrderDriver::new();
        d.on_sequence_header_window(Some(e)).expect("first window");
        assert_eq!(d.requirement(), Requirement::RequiredEqual(e));
        assert!(d.picture_display_extension_permitted());
    }

    // ---- §6.3.5 Forbidden enforcement -----------------------------

    #[test]
    fn forbidden_then_absent_ok() {
        let mut d = SequenceDisplayOrderDriver::new();
        d.on_sequence_header_window(None).expect("first window");
        d.on_sequence_header_window(None).expect("repeat window");
        assert!(!d.picture_display_extension_permitted());
    }

    #[test]
    fn forbidden_then_present_rejected() {
        let mut d = SequenceDisplayOrderDriver::new();
        d.on_sequence_header_window(None).expect("first window");
        let err = d
            .on_sequence_header_window(Some(ext(VideoFormat::Ntsc, 1, 1)))
            .unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    // ---- §6.3.5 RequiredEqual enforcement -------------------------

    #[test]
    fn required_then_equal_ok() {
        let e = ext(VideoFormat::Pal, 720, 576);
        let mut d = SequenceDisplayOrderDriver::new();
        d.on_sequence_header_window(Some(e)).expect("first window");
        d.on_sequence_header_window(Some(e)).expect("repeat window");
        assert!(d.picture_display_extension_permitted());
    }

    #[test]
    fn required_then_absent_rejected() {
        let e = ext(VideoFormat::Pal, 720, 576);
        let mut d = SequenceDisplayOrderDriver::new();
        d.on_sequence_header_window(Some(e)).expect("first window");
        let err = d.on_sequence_header_window(None).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn required_then_different_value_rejected() {
        let first = ext(VideoFormat::Pal, 720, 576);
        let other = ext(VideoFormat::Pal, 720, 480); // differs in dvs
        let mut d = SequenceDisplayOrderDriver::new();
        d.on_sequence_header_window(Some(first))
            .expect("first window");
        let err = d.on_sequence_header_window(Some(other)).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    #[test]
    fn required_then_different_colour_rejected() {
        // Two extensions that match in every field but the colour
        // triple must still trip the §6.3.5 "all data elements are the
        // same" rule.
        let first = ext_with_colour(ColourDescription {
            colour_primaries: 1,
            transfer_characteristics: 1,
            matrix_coefficients: 1,
        });
        let other = ext_with_colour(ColourDescription {
            colour_primaries: 5,
            transfer_characteristics: 1,
            matrix_coefficients: 1,
        });
        let mut d = SequenceDisplayOrderDriver::new();
        d.on_sequence_header_window(Some(first))
            .expect("first window");
        let err = d.on_sequence_header_window(Some(other)).unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    // ---- §6.3.12 ordering gate ------------------------------------

    #[test]
    fn picture_display_permitted_only_after_present_window() {
        let e = ext(VideoFormat::Ntsc, 352, 240);
        let mut d = SequenceDisplayOrderDriver::new();
        // Before any window: forbidden.
        assert!(d.check_picture_display_extension().is_err());
        // After a present window: permitted.
        d.on_sequence_header_window(Some(e)).expect("first window");
        assert!(d.check_picture_display_extension().is_ok());
    }

    #[test]
    fn picture_display_forbidden_after_absent_window() {
        let mut d = SequenceDisplayOrderDriver::new();
        d.on_sequence_header_window(None).expect("first window");
        let err = d.check_picture_display_extension().unwrap_err();
        assert!(matches!(err, Error::InvalidBitstream(_)));
    }

    // ---- Multi-cycle ----------------------------------------------

    #[test]
    fn multi_window_required_chain_idempotent() {
        // §6.3.5 RequiredEqual holds across many repeats; §6.3.12 stays
        // permitted throughout.
        let e = ext(VideoFormat::Component, 1920, 1080);
        let mut d = SequenceDisplayOrderDriver::new();
        d.on_sequence_header_window(Some(e)).expect("first");
        for _ in 0..5 {
            d.on_sequence_header_window(Some(e)).expect("repeat");
            assert!(d.check_picture_display_extension().is_ok());
        }
        assert_eq!(d.requirement(), Requirement::RequiredEqual(e));
    }

    #[test]
    fn multi_window_forbidden_chain_idempotent() {
        let mut d = SequenceDisplayOrderDriver::new();
        d.on_sequence_header_window(None).expect("first");
        for _ in 0..5 {
            d.on_sequence_header_window(None).expect("repeat");
            assert!(d.check_picture_display_extension().is_err());
        }
        assert_eq!(d.requirement(), Requirement::Forbidden);
    }
}
