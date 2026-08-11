//! Video Buffering Verifier (VBV) — the exact constant-bit-rate buffer
//! model that ISO/IEC 13818-2 Annex C (C.1–C.12) and ISO/IEC 11172-2
//! Annex C (C.1.1–C.1.4) impose on coded video bitstreams, plus a
//! whole-stream conformance verifier and the `vbv_delay` byte-patcher
//! the encoders use to stamp real delays into finished picture layers.
//!
//! ## The model (13818-2 C.3.1 / 11172-2 C.1.3–C.1.4)
//!
//! The VBV is a hypothetical decoder with an input buffer of size `B`
//! (`vbv_buffer_size * 16 * 1024` bits, §6.3.3 / §2.4.3.2). For
//! constant-bit-rate operation (`vbv_delay != 0xFFFF`), coded data
//! enters the buffer at the rate `R` and whole coded pictures are
//! removed **instantaneously** at examination times `t(n)`:
//!
//! * after filling the buffer with everything up to and including the
//!   first `picture_start_code`, the buffer fills for the time coded in
//!   the first picture's `vbv_delay`, and decoding begins (C.3.1 /
//!   C.1.3);
//! * successive examinations follow at the Annex C cadence — for the
//!   streams this crate emits (`repeat_first_field = 0`,
//!   `low_delay = 0`) that is one **frame period** per frame picture
//!   (13818-2 C.9 / C.11, 11172-2 C.1.4) and one **field period** per
//!   field picture (13818-2 C.11);
//! * at every examination, occupancy must lie in `[0, B]` (13818-2
//!   C.5–C.6, 11172-2 C.1.4): the buffer must neither overflow before a
//!   removal nor underflow (all of the picture's data must have
//!   arrived by its removal time).
//!
//! "Picture data" spans everything from the end of the previous
//! picture's data to the start code following this picture's slices —
//! including the headers immediately preceding the picture and any
//! trailing zero stuffing, and including a terminating
//! `sequence_end_code` (13818-2 C.5, 11172-2 C.1.4).
//!
//! The coded `vbv_delay` is (§6.3.9 / §2.4.3.4):
//!
//! ```text
//! vbv_delay(n) = 90 000 * B*(n) / R
//! ```
//!
//! where `B*(n)` is the occupancy immediately before removing picture
//! `n` **excluding** the headers / stuffing / start code that precede
//! picture `n`'s data elements — equivalently `R * t(n)` minus the
//! stream position of the end of picture `n`'s start code.
//!
//! ## Exactness
//!
//! Annex C prescribes real-valued arithmetic ("no rounding errors can
//! propagate"). [`VbvCbrModel`] therefore tracks `R * t` in integer
//! sub-units of `1 / (180 000 * frame_rate_numerator)` seconds — every
//! Table 6-4 / §2.4.3.2 rate (including the 1001-denominator rates),
//! every frame/field period, and every 90 kHz `vbv_delay` tick is exact
//! in those units, so the model is bit-precise with `i128` arithmetic.
//! Only the final `vbv_delay` value is rounded (to the nearest 90 kHz
//! tick — the quantisation the C.3.1 NOTE acknowledges).

use crate::picture_header::PictureStructure;
use crate::{Error, Result};

/// The Table 6-4 (13818-2) / §2.4.3.2 (11172-2) frame-rate table:
/// `frame_rate_code` → `(numerator, denominator)` frames per second.
///
/// # Errors
/// [`Error::InvalidBitstream`] for the forbidden / reserved codes.
pub fn frame_rate_value(frame_rate_code: u8) -> Result<(u32, u32)> {
    match frame_rate_code {
        1 => Ok((24_000, 1001)),
        2 => Ok((24, 1)),
        3 => Ok((25, 1)),
        4 => Ok((30_000, 1001)),
        5 => Ok((30, 1)),
        6 => Ok((50, 1)),
        7 => Ok((60_000, 1001)),
        8 => Ok((60, 1)),
        _ => Err(Error::InvalidBitstream(
            "frame_rate_code: forbidden or reserved value (Table 6-4 / §2.4.3.2)",
        )),
    }
}

/// The interval to the **next** VBV examination after removing a
/// picture, per the 13818-2 C.9 / C.11 cadence (with
/// `repeat_first_field = 0`) and the 11172-2 C.1.4 picture interval.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RemovalInterval {
    /// One frame period (`1 / frame_rate`): every 11172-2 picture
    /// (C.1.4) and every 13818-2 frame picture (C.9 / C.11 with
    /// `repeat_first_field = 0`).
    Frame,
    /// One field period (`1 / (2 * frame_rate)`): a 13818-2 field
    /// picture (C.11).
    Field,
}

/// Static parameters of a CBR VBV run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VbvParams {
    /// `R` — the actual bitrate in bits/second. For the streams this
    /// crate emits this equals the declared `bit_rate * 400` exactly
    /// (§6.3.3 recommends the declared value be the actual CBR rate).
    pub bit_rate: u64,
    /// `B` — the VBV buffer size in bits
    /// (`vbv_buffer_size * 16 * 1024`, §6.3.3 / §2.4.3.2).
    pub buffer_size_bits: u64,
    /// The frame rate as `(numerator, denominator)` frames/second
    /// ([`frame_rate_value`]).
    pub frame_rate: (u32, u32),
}

/// A VBV constraint violation found while running the model.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VbvViolation {
    /// 13818-2 C.6 / 11172-2 C.1.4: not all of picture `picture`'s data
    /// had entered the buffer at its removal time.
    Underflow { picture: usize },
    /// 13818-2 C.5 / 11172-2 C.1.4: occupancy exceeded `B` immediately
    /// before removing picture `picture`.
    Overflow { picture: usize },
    /// The coded `vbv_delay` of picture `picture` differs from the
    /// model's C.3.1 value by more than one 90 kHz tick.
    DelayMismatch {
        picture: usize,
        coded: u16,
        expected: u16,
    },
}

impl std::fmt::Display for VbvViolation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Underflow { picture } => {
                write!(f, "VBV underflow removing picture {picture} (C.6)")
            }
            Self::Overflow { picture } => {
                write!(f, "VBV overflow before removing picture {picture} (C.5)")
            }
            Self::DelayMismatch {
                picture,
                coded,
                expected,
            } => write!(
                f,
                "vbv_delay mismatch on picture {picture}: coded {coded}, model {expected} (C.3.1)"
            ),
        }
    }
}

/// The exact CBR VBV state machine (13818-2 C.3.1 / 11172-2 C.1.3).
///
/// Stream positions are bit offsets from the first bit of the
/// elementary stream. The caller feeds, per picture `n`:
///
/// 1. [`VbvCbrModel::delay_for_picture`] with `pos(n)` — the offset of
///    the **end** of picture `n`'s `picture_start_code` — to obtain the
///    `vbv_delay` to code (picture 0's coded delay instead *seeds* the
///    model via [`VbvCbrModel::start`]);
/// 2. [`VbvCbrModel::remove_picture`] with `end(n)` — the offset where
///    the **next** picture's preceding headers begin (or the stream
///    ends) — plus the [`RemovalInterval`] that follows picture `n`.
#[derive(Debug, Clone)]
pub struct VbvCbrModel {
    /// `R` in bits/second.
    r: i128,
    /// Time sub-units per second: `180_000 * frame_rate.0`.
    u_per_s: i128,
    /// `B * u_per_s` — the buffer bound in scaled occupancy units.
    b_scaled: i128,
    /// `R * t(n) * u_per_s / R`... precisely: `R * t(n)` expressed in
    /// `1 / u_per_s` bits — the total bits offered to the buffer by the
    /// next removal time, scaled so every quantity is an integer.
    rt_scaled: i128,
    /// `end(n-1)` — the bit offset where the previous picture's data
    /// ended (0 before the first picture).
    prev_end_bits: u64,
    /// Number of pictures removed so far.
    removed: usize,
    /// Whether [`VbvCbrModel::start`] has run.
    started: bool,
}

impl VbvCbrModel {
    /// Build a model for `params`.
    ///
    /// # Errors
    /// [`Error::InvalidBitstream`] when a parameter is zero, or when
    /// `90_000 * B / R` exceeds the 16-bit `vbv_delay` range (a full
    /// buffer could then not code its delay; §6.3.9 / §2.4.3.4 make
    /// `0xFFFF` a reserved sentinel, so the ceiling is `0xFFFE`).
    pub fn new(params: &VbvParams) -> Result<Self> {
        let (frn, frd) = params.frame_rate;
        if params.bit_rate == 0 || params.buffer_size_bits == 0 || frn == 0 || frd == 0 {
            return Err(Error::InvalidBitstream(
                "VBV params: bit_rate, buffer size and frame rate must be non-zero",
            ));
        }
        // Delay representability: occupancy <= B must always yield
        // vbv_delay <= 0xFFFE.
        if (params.buffer_size_bits as i128) * 90_000 > (params.bit_rate as i128) * 0xFFFE {
            return Err(Error::InvalidBitstream(
                "VBV params: 90000 * B / R exceeds the 16-bit vbv_delay range (§6.3.9)",
            ));
        }
        let u_per_s = 180_000i128 * i128::from(frn);
        Ok(Self {
            r: i128::from(params.bit_rate),
            u_per_s,
            b_scaled: i128::from(params.buffer_size_bits) * u_per_s,
            rt_scaled: 0,
            prev_end_bits: 0,
            removed: 0,
            started: false,
        })
    }

    /// The frame-rate denominator scaled interval in time sub-units:
    /// one frame period is `frd / frn` s = `180_000 * frd` units, one
    /// field period half that.
    fn interval_units(&self, interval: RemovalInterval, frd: u32) -> i128 {
        match interval {
            RemovalInterval::Frame => 180_000i128 * i128::from(frd),
            RemovalInterval::Field => 90_000i128 * i128::from(frd),
        }
    }

    /// Seed the model with picture 0's coded `vbv_delay` and `pos(0)`
    /// (bit offset of the end of the first `picture_start_code`):
    /// `t(0)` is the arrival time of that offset plus the delay
    /// (13818-2 C.3.1 / 11172-2 C.1.3).
    pub fn start(&mut self, initial_vbv_delay: u16, pos0_bits: u64) {
        // R * t(0) scaled: pos0 arrives at pos0 / R seconds, so the
        // scaled product is pos0 * u_per_s; the delay adds
        // tau0 / 90_000 s * R, scaled: tau0 * R * u_per_s / 90_000.
        let tau_scale = self.u_per_s / 90_000; // = 2 * frn, exact
        self.rt_scaled = i128::from(pos0_bits) * self.u_per_s
            + i128::from(initial_vbv_delay) * self.r * tau_scale;
        self.started = true;
    }

    /// The C.3.1 / §6.3.9 `vbv_delay` for the picture about to be
    /// removed, from `pos(n)` (bit offset of the end of its
    /// `picture_start_code`): `90_000 * (R * t(n) - pos(n)) / R`,
    /// rounded to the nearest tick.
    ///
    /// # Errors
    /// [`Error::InvalidBitstream`] if the model was not started, the
    /// occupancy is negative (the caller's picture overran its budget),
    /// or the delay exceeds `0xFFFE`.
    pub fn delay_for_picture(&self, pos_bits: u64) -> Result<u16> {
        if !self.started {
            return Err(Error::InvalidBitstream("VBV model not started"));
        }
        let occ_scaled = self.rt_scaled - i128::from(pos_bits) * self.u_per_s;
        if occ_scaled < 0 {
            return Err(Error::InvalidBitstream(
                "VBV: negative occupancy at picture start code (C.3.1)",
            ));
        }
        // delay = occ_scaled * 90_000 / (u_per_s * R), round nearest.
        let denom = (self.u_per_s / 90_000) * self.r; // 2 * frn * R
        let delay = (occ_scaled + denom / 2) / denom;
        if delay > 0xFFFE {
            return Err(Error::InvalidBitstream(
                "VBV: vbv_delay exceeds 0xFFFE (§6.3.9 sentinel)",
            ));
        }
        Ok(delay as u16)
    }

    /// The largest legal `end(n)` (bit offset where this picture's data
    /// may end) that avoids underflow at the pending removal: all of
    /// the picture's data must have arrived by `t(n)` (C.6 / C.1.4).
    pub fn max_end_bits(&self) -> u64 {
        let bits = self.rt_scaled / self.u_per_s;
        if bits < 0 {
            0
        } else {
            bits as u64
        }
    }

    /// The smallest legal `end(n)` that avoids overflow immediately
    /// before the **next** removal (C.5 / C.1.4): by `t(n+1)` the
    /// buffer will hold `R * t(n+1) - end(n)` bits, which must not
    /// exceed `B`. `frd` is the frame-rate denominator; `interval` the
    /// cadence step following this picture.
    pub fn min_end_bits(&self, interval: RemovalInterval, frd: u32) -> u64 {
        let rt_next = self.rt_scaled + self.r * self.interval_units(interval, frd);
        let min_scaled = rt_next - self.b_scaled;
        if min_scaled <= 0 {
            return 0;
        }
        // ceil(min_scaled / u_per_s)
        ((min_scaled + self.u_per_s - 1) / self.u_per_s) as u64
    }

    /// Remove the pending picture whose data ends at `end_bits`,
    /// checking the C.5 overflow bound (occupancy before this removal)
    /// and the C.6 underflow bound (all data present), then advance the
    /// examination clock by `interval`.
    ///
    /// # Errors
    /// The violated constraint, as [`VbvViolation`].
    pub fn remove_picture(
        &mut self,
        end_bits: u64,
        interval: RemovalInterval,
        frd: u32,
    ) -> std::result::Result<(), VbvViolation> {
        let n = self.removed;
        // C.5: occupancy immediately before removal (everything that
        // has arrived minus everything previously removed) <= B.
        let occ_before = self.rt_scaled - i128::from(self.prev_end_bits) * self.u_per_s;
        if occ_before > self.b_scaled {
            return Err(VbvViolation::Overflow { picture: n });
        }
        // C.6: all of this picture's data must have arrived by t(n).
        if i128::from(end_bits) * self.u_per_s > self.rt_scaled {
            return Err(VbvViolation::Underflow { picture: n });
        }
        self.prev_end_bits = end_bits;
        self.rt_scaled += self.r * self.interval_units(interval, frd);
        self.removed += 1;
        Ok(())
    }

    /// Occupancy in whole bits immediately before the pending removal
    /// (for diagnostics; truncates the exact scaled value).
    pub fn occupancy_before_removal_bits(&self) -> i64 {
        ((self.rt_scaled - i128::from(self.prev_end_bits) * self.u_per_s) / self.u_per_s) as i64
    }
}

/// Overwrite the 16-bit `vbv_delay` field of a picture layer in place.
///
/// `picture` must begin with the `picture_start_code` (`00 00 01 00`).
/// Per the §6.2.3 / §2.4.3.4 syntax the field occupies bits 45..61 of
/// the layer (after the 32-bit start code, the 10-bit
/// `temporal_reference` and the 3-bit `picture_coding_type`), i.e. the
/// low 3 bits of byte 5, all of byte 6, and the high 5 bits of byte 7.
///
/// # Errors
/// [`Error::InvalidBitstream`] when the buffer is too short or does not
/// start with a `picture_start_code`.
pub fn patch_vbv_delay(picture: &mut [u8], vbv_delay: u16) -> Result<()> {
    if picture.len() < 8 || picture[0] != 0 || picture[1] != 0 || picture[2] != 1 || picture[3] != 0
    {
        return Err(Error::InvalidBitstream(
            "patch_vbv_delay: buffer does not start with a picture_start_code",
        ));
    }
    picture[5] = (picture[5] & 0b1111_1000) | ((vbv_delay >> 13) as u8);
    picture[6] = ((vbv_delay >> 5) & 0xFF) as u8;
    picture[7] = (picture[7] & 0b0000_0111) | (((vbv_delay & 0x1F) as u8) << 3);
    Ok(())
}

/// Which standard's sequence layer a stream under
/// [`verify_cbr_stream`] declares.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VbvStandard {
    /// ISO/IEC 11172-2: no `sequence_extension()`, every picture is a
    /// frame picture removed at the §2.4.3.2 nominal picture rate
    /// (Annex C C.1.4).
    Mpeg1,
    /// ISO/IEC 13818-2: `sequence_extension()` carries the
    /// `bit_rate` / `vbv_buffer_size` upper bits; field pictures follow
    /// the C.11 field-period cadence.
    Mpeg2,
}

/// One picture's bookkeeping inside a [`VbvStreamReport`].
#[derive(Debug, Clone, Copy)]
pub struct VbvPictureRecord {
    /// Coded `vbv_delay`.
    pub vbv_delay: u16,
    /// The model's expected delay (C.3.1).
    pub expected_delay: u16,
    /// Picture data size in bits (headers + picture + trailing
    /// stuffing, per the C.5 / C.1.4 "picture data" definition).
    pub data_bits: u64,
    /// Occupancy in bits immediately before this picture's removal.
    pub occupancy_before_bits: i64,
}

/// The result of a successful [`verify_cbr_stream`] run.
#[derive(Debug, Clone)]
pub struct VbvStreamReport {
    /// The declared `R` in bits/second (`bit_rate * 400`).
    pub bit_rate: u64,
    /// The declared `B` in bits (`vbv_buffer_size * 16 * 1024`).
    pub buffer_size_bits: u64,
    /// Per-picture records in coded order.
    pub pictures: Vec<VbvPictureRecord>,
    /// The minimum occupancy (bits) observed immediately after any
    /// removal — headroom above the C.6 underflow bound.
    pub min_occupancy_after_bits: i64,
    /// The maximum occupancy (bits) observed immediately before any
    /// removal — headroom below the C.5 overflow bound `B`.
    pub max_occupancy_before_bits: i64,
}

/// A [`verify_cbr_stream`] failure: either the stream could not be
/// parsed at all, or it parsed but violates an Annex C constraint.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum VbvVerifyError {
    /// A syntax-level failure while locating / parsing headers.
    Parse(Error),
    /// A structural VBV violation (C.5 / C.6 / C.3.1).
    Violation(VbvViolation),
}

impl std::fmt::Display for VbvVerifyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Parse(e) => write!(f, "VBV verify parse failure: {e}"),
            Self::Violation(v) => write!(f, "VBV violation: {v}"),
        }
    }
}

impl std::error::Error for VbvVerifyError {}

impl From<Error> for VbvVerifyError {
    fn from(e: Error) -> Self {
        Self::Parse(e)
    }
}

impl From<VbvViolation> for VbvVerifyError {
    fn from(v: VbvViolation) -> Self {
        Self::Violation(v)
    }
}

/// Every start code in `stream` as `(byte_offset, code_byte)`.
fn scan_start_codes(stream: &[u8]) -> Vec<(usize, u8)> {
    let mut out = Vec::new();
    let mut i = 0usize;
    while i + 3 < stream.len() {
        if stream[i] == 0 && stream[i + 1] == 0 && stream[i + 2] == 1 {
            out.push((i, stream[i + 3]));
            i += 4;
        } else {
            i += 1;
        }
    }
    out
}

/// Run the full Annex C CBR verification over a whole elementary
/// stream: parse the declared `bit_rate` / `vbv_buffer_size` / frame
/// rate, seed the model from picture 0's coded `vbv_delay`, and check
/// every picture for the C.5 overflow bound, the C.6 underflow bound,
/// and a C.3.1-consistent coded `vbv_delay` (±1 tick of quantisation).
///
/// Scope: `low_delay = 0`, `repeat_first_field = 0` streams — the only
/// kind this crate's encoders emit. A stream whose first picture codes
/// `vbv_delay == 0xFFFF` is variable-rate (C.3.2) and is rejected here.
///
/// # Errors
/// [`VbvVerifyError::Parse`] for syntax failures or a VBR sentinel
/// delay; [`VbvVerifyError::Violation`] for a C.5 / C.6 / C.3.1
/// violation.
pub fn verify_cbr_stream(
    stream: &[u8],
    standard: VbvStandard,
) -> std::result::Result<VbvStreamReport, VbvVerifyError> {
    let codes = scan_start_codes(stream);
    let seq_off = codes
        .iter()
        .find(|&&(_, c)| c == 0xB3)
        .map(|&(o, _)| o)
        .ok_or(VbvVerifyError::Parse(Error::InvalidBitstream(
            "VBV verify: no sequence header",
        )))?;
    let header = crate::sequence_header::Mpeg2SequenceHeader::parse(&stream[seq_off..])?;

    let (bit_rate, buffer_size_bits, frame_rate) = match standard {
        VbvStandard::Mpeg1 => (
            u64::from(header.bit_rate) * 400,
            u64::from(header.vbv_buffer_size) * 16 * 1024,
            frame_rate_value(header.frame_rate_code)?,
        ),
        VbvStandard::Mpeg2 => {
            let seq = crate::sequence_extension::Mpeg2Sequence::from_buf(&stream[seq_off..])?;
            let ext = &seq.extension;
            let full_rate = (u64::from(ext.bit_rate_extension) << 18) | u64::from(header.bit_rate);
            let full_vbv = (u64::from(ext.vbv_buffer_size_extension) << 10)
                | u64::from(header.vbv_buffer_size);
            // frame_rate_extension_n / _d are zero in every stream this
            // crate emits; Table 6-4 applies directly.
            (
                full_rate * 400,
                full_vbv * 16 * 1024,
                frame_rate_value(header.frame_rate_code)?,
            )
        }
    };

    // Group the start codes into pictures: a picture unit runs from its
    // picture_start_code through its slices; `end(n)` is the offset of
    // the first non-slice start code after its first slice (the next
    // picture's preceding headers begin there). A terminating
    // sequence_end_code is included in the final picture's data (C.5).
    struct Pic {
        pos_bits: u64,
        end_bits: u64,
        vbv_delay: u16,
        interval: RemovalInterval,
    }
    let mut pics: Vec<Pic> = Vec::new();
    let mut idx = 0usize;
    while idx < codes.len() {
        let (off, code) = codes[idx];
        if code != 0x00 {
            idx += 1;
            continue;
        }
        // Parse the picture header (and extension for MPEG-2) for the
        // coded vbv_delay and the picture structure.
        let (ph, interval) = match standard {
            VbvStandard::Mpeg1 => (
                crate::picture_header::Mpeg2PictureHeader::parse(&stream[off..])?,
                RemovalInterval::Frame,
            ),
            VbvStandard::Mpeg2 => {
                let (ph, ext) = crate::picture_header::Mpeg2PictureHeader::parse_with_extension(
                    &stream[off..],
                )?;
                let interval = if ext.picture_structure == PictureStructure::Frame {
                    RemovalInterval::Frame
                } else {
                    RemovalInterval::Field
                };
                (ph, interval)
            }
        };
        // Find the first slice code after this picture, then the first
        // non-slice code after that.
        let mut j = idx + 1;
        while j < codes.len() && !(0x01..=0xAF).contains(&codes[j].1) {
            j += 1;
        }
        while j < codes.len() && (0x01..=0xAF).contains(&codes[j].1) {
            j += 1;
        }
        let end_bits = if j < codes.len() {
            if codes[j].1 == 0xB7 {
                // sequence_end_code: included in this picture's data.
                (codes[j].0 as u64 + 4) * 8
            } else {
                codes[j].0 as u64 * 8
            }
        } else {
            stream.len() as u64 * 8
        };
        pics.push(Pic {
            pos_bits: (off as u64 + 4) * 8,
            end_bits,
            vbv_delay: ph.vbv_delay,
            interval,
        });
        idx = j;
    }

    if pics.is_empty() {
        return Err(VbvVerifyError::Parse(Error::InvalidBitstream(
            "VBV verify: no pictures",
        )));
    }
    if pics[0].vbv_delay == 0xFFFF {
        return Err(VbvVerifyError::Parse(Error::InvalidBitstream(
            "VBV verify: vbv_delay == 0xFFFF is variable-rate operation (C.3.2)",
        )));
    }

    let mut model = VbvCbrModel::new(&VbvParams {
        bit_rate,
        buffer_size_bits,
        frame_rate,
    })?;
    model.start(pics[0].vbv_delay, pics[0].pos_bits);
    let frd = frame_rate.1;

    let mut records = Vec::with_capacity(pics.len());
    let mut min_after = i64::MAX;
    let mut max_before = i64::MIN;
    let mut prev_end = 0u64;
    for (n, pic) in pics.iter().enumerate() {
        let expected = model.delay_for_picture(pic.pos_bits)?;
        // ±1 tick: the encoder rounds the exact real value to the
        // nearest 90 kHz tick (C.3.1 NOTE).
        if u32::from(expected).abs_diff(u32::from(pic.vbv_delay)) > 1 {
            return Err(VbvVerifyError::Violation(VbvViolation::DelayMismatch {
                picture: n,
                coded: pic.vbv_delay,
                expected,
            }));
        }
        let occ_before = model.occupancy_before_removal_bits();
        max_before = max_before.max(occ_before);
        model.remove_picture(pic.end_bits, pic.interval, frd)?;
        let occ_after = occ_before - (pic.end_bits - prev_end) as i64;
        min_after = min_after.min(occ_after);
        records.push(VbvPictureRecord {
            vbv_delay: pic.vbv_delay,
            expected_delay: expected,
            data_bits: (pic.end_bits - prev_end),
            occupancy_before_bits: occ_before,
        });
        prev_end = pic.end_bits;
    }

    Ok(VbvStreamReport {
        bit_rate,
        buffer_size_bits,
        pictures: records,
        min_occupancy_after_bits: min_after,
        max_occupancy_before_bits: max_before,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> VbvParams {
        VbvParams {
            bit_rate: 1_000_000,
            buffer_size_bits: 20 * 16 * 1024, // 327 680 bits
            frame_rate: (25, 1),
        }
    }

    #[test]
    fn frame_rate_table_matches_spec() {
        assert_eq!(frame_rate_value(1).unwrap(), (24_000, 1001));
        assert_eq!(frame_rate_value(3).unwrap(), (25, 1));
        assert_eq!(frame_rate_value(4).unwrap(), (30_000, 1001));
        assert_eq!(frame_rate_value(8).unwrap(), (60, 1));
        assert!(frame_rate_value(0).is_err());
        assert!(frame_rate_value(9).is_err());
    }

    #[test]
    fn initial_delay_round_trips() {
        // Seeding with tau0 must reproduce exactly tau0 for pos(0).
        let mut m = VbvCbrModel::new(&params()).unwrap();
        m.start(9000, 800);
        assert_eq!(m.delay_for_picture(800).unwrap(), 9000);
    }

    #[test]
    fn delay_decreases_by_picture_size_and_grows_by_rate() {
        // R = 1_000_000 b/s at 25 fps: each frame interval adds
        // R/25 = 40_000 bits; a picture of d bits drains d.
        let mut m = VbvCbrModel::new(&params()).unwrap();
        m.start(9000, 800);
        // occupancy(0) = 9000/90000 s * R = 100_000 bits.
        // Picture 0 spans pos 800 .. end 60_800 (60_000 payload bits);
        // picture 1's start code ends 32 bits later.
        m.remove_picture(60_800, RemovalInterval::Frame, 1).unwrap();
        // occupancy before picture 1 (formula form, from pos(1)):
        // R*t1 - pos1 = (100_000 + 800 + 40_000) - 60_832 = 79_968.
        // delay = 90_000 * 79_968 / 1_000_000 = 7197.12 -> 7197.
        assert_eq!(m.delay_for_picture(60_832).unwrap(), 7197);
    }

    #[test]
    fn underflow_is_detected() {
        let mut m = VbvCbrModel::new(&params()).unwrap();
        m.start(9000, 800);
        // Available by t(0): occupancy 100_000 bits + pos 800; a
        // picture ending beyond that is an underflow.
        assert_eq!(m.max_end_bits(), 100_800);
        assert!(matches!(
            m.remove_picture(100_801, RemovalInterval::Frame, 1),
            Err(VbvViolation::Underflow { picture: 0 })
        ));
    }

    #[test]
    fn overflow_is_detected() {
        // Tiny buffer: occupancy before first removal already exceeds B.
        let p = VbvParams {
            bit_rate: 1_000_000,
            buffer_size_bits: 16 * 1024,
            frame_rate: (25, 1),
        };
        let mut m = VbvCbrModel::new(&p).unwrap();
        m.start(2000, 800); // occupancy ~22_222 bits + 800 > 16_384
        assert!(matches!(
            m.remove_picture(10_000, RemovalInterval::Frame, 1),
            Err(VbvViolation::Overflow { picture: 0 })
        ));
    }

    #[test]
    fn min_end_bits_tracks_overflow_bound() {
        // Small buffer (B = 65_536 bits) so the overflow bound bites:
        // tau0 = 5000 ticks at R = 1_000_000 b/s is 55_555.5… bits of
        // occupancy, so R*t(0) = 800 + 55_555.5… and R*t(1) adds one
        // 40_000-bit frame interval.
        let p = VbvParams {
            bit_rate: 1_000_000,
            buffer_size_bits: 4 * 16 * 1024,
            frame_rate: (25, 1),
        };
        let mut m = VbvCbrModel::new(&p).unwrap();
        m.start(5000, 800);
        assert_eq!(m.max_end_bits(), 56_355);
        // min = ceil(96_355.5… - 65_536) = 30_820.
        let min = m.min_end_bits(RemovalInterval::Frame, 1);
        assert_eq!(min, 30_820);
        // Removing exactly `min` keeps the next examination legal.
        m.remove_picture(min, RemovalInterval::Frame, 1).unwrap();
        assert!(m.occupancy_before_removal_bits() as u64 <= p.buffer_size_bits);
        m.remove_picture(70_000, RemovalInterval::Frame, 1).unwrap();
        // Removing one bit less than `min` would have overflowed: rerun
        // with min - 1.
        let mut m2 = VbvCbrModel::new(&p).unwrap();
        m2.start(5000, 800);
        m2.remove_picture(min - 1, RemovalInterval::Frame, 1)
            .unwrap();
        assert!(matches!(
            m2.remove_picture(70_000, RemovalInterval::Frame, 1),
            Err(VbvViolation::Overflow { picture: 1 })
        ));
    }

    #[test]
    fn field_interval_is_half_a_frame() {
        let mut m1 = VbvCbrModel::new(&params()).unwrap();
        let mut m2 = VbvCbrModel::new(&params()).unwrap();
        m1.start(4500, 800);
        m2.start(4500, 800);
        m1.remove_picture(10_000, RemovalInterval::Frame, 1)
            .unwrap();
        m2.remove_picture(10_000, RemovalInterval::Field, 1)
            .unwrap();
        m2.remove_picture(10_000, RemovalInterval::Field, 1)
            .unwrap();
        assert_eq!(
            m1.delay_for_picture(20_000).unwrap(),
            m2.delay_for_picture(20_000).unwrap()
        );
    }

    #[test]
    fn ntsc_rational_rate_is_exact() {
        // 30000/1001: one frame period at R = 1_001_000 b/s adds
        // exactly 1_001_000 * 1001 / 30_000 bits * ... — the scaled
        // arithmetic must not drift over many pictures.
        let p = VbvParams {
            bit_rate: 1_001_000,
            buffer_size_bits: 20 * 16 * 1024,
            frame_rate: (30_000, 1001),
        };
        let mut m = VbvCbrModel::new(&p).unwrap();
        m.start(9000, 800);
        let d0 = m.delay_for_picture(800).unwrap();
        assert_eq!(d0, 9000);
        // The per-frame arrival R * 1001 / 30_000 = 33_400.03… bits is
        // not integral, so integer picture sizes can never track it
        // exactly — the scaled arithmetic must absorb the fractional
        // remainder without drift or panic over a long run.
        let mut end = 20_000u64;
        for _ in 0..300 {
            m.remove_picture(end, RemovalInterval::Frame, 1001).unwrap();
            let d = m.delay_for_picture(end + 32).unwrap();
            assert!(d <= 0xFFFE);
            end += 33_366;
        }
    }

    #[test]
    fn patch_vbv_delay_rewrites_only_the_field() {
        use crate::picture_header::{Mpeg2PictureHeader, PictureCodingType};
        use crate::stream_writer::write_picture_header;
        use oxideav_core::bits::BitWriter;

        let mut bw = BitWriter::new();
        write_picture_header(&mut bw, 513, PictureCodingType::Predictive, 0b111, 0b111);
        let mut bytes = bw.finish();
        patch_vbv_delay(&mut bytes, 0xABCD).unwrap();
        let ph = Mpeg2PictureHeader::parse(&bytes).unwrap();
        assert_eq!(ph.vbv_delay, 0xABCD);
        assert_eq!(ph.temporal_reference, 513);
        assert_eq!(ph.picture_coding_type, PictureCodingType::Predictive);
        assert_eq!(ph.fwd_f_code, Some(0b111));
    }

    #[test]
    fn patch_vbv_delay_rejects_non_picture_buffers() {
        let mut buf = [0u8, 0, 1, 0xB3, 0, 0, 0, 0];
        assert!(patch_vbv_delay(&mut buf, 1).is_err());
        let mut short = [0u8, 0, 1, 0];
        assert!(patch_vbv_delay(&mut short, 1).is_err());
    }

    #[test]
    fn delay_range_validation_rejects_oversized_buffers() {
        // 90000 * B / R must fit 0xFFFE: B = 229_376 bits at
        // R = 100_000 b/s gives 206_438 ticks — far out of range.
        let p = VbvParams {
            bit_rate: 100_000,
            buffer_size_bits: 14 * 16 * 1024,
            frame_rate: (25, 1),
        };
        assert!(VbvCbrModel::new(&p).is_err());
    }
}
