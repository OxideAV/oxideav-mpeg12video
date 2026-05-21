//! Black-box validation that our `sequence_header()` parser agrees
//! with a real MPEG-2 elementary stream produced by an opaque
//! encoder. The fixture under `tests/fixtures/` was produced via:
//!
//! ```text
//! ffmpeg -y -f lavfi -i testsrc=size=352x240:rate=25:duration=0.04 \
//!        -c:v mpeg2video -b:v 800k -an -f mpeg2video out.m2v
//! ```
//!
//! Only the file's *bytes* are consumed here; the encoder's source
//! code is not. This is the standard black-box validator pattern
//! described in `docs/IMPLEMENTOR_ROUND.md`.

use oxideav_mpeg12video::{AspectRatio, Mpeg2SequenceHeader, SEQUENCE_HEADER_CODE};

const FIXTURE: &[u8] = include_bytes!("fixtures/ffmpeg-352x240-25fps.m2v");

#[test]
fn parses_ffmpeg_352x240_sequence_header() {
    // Locate the first sequence_header_code in the elementary stream.
    let pos = FIXTURE
        .windows(4)
        .position(|w| {
            (u32::from(w[0]) << 24 | u32::from(w[1]) << 16 | u32::from(w[2]) << 8 | u32::from(w[3]))
                == SEQUENCE_HEADER_CODE
        })
        .expect("fixture contains a sequence_header_code");

    let sh = Mpeg2SequenceHeader::parse(&FIXTURE[pos..]).expect("parse");

    assert_eq!(sh.width, 352, "horizontal_size_value");
    assert_eq!(sh.height, 240, "vertical_size_value");
    assert_eq!(sh.aspect_ratio, AspectRatio::Square);
    assert_eq!(sh.frame_rate_code, 0b0011, "25fps Table 6-4 entry");
    // bit_rate is the lower 18 bits of the 30-bit bit_rate field
    // (§6.3.3). Empirically this ffmpeg build writes the all-ones
    // sentinel `0x3FFFF` into `bit_rate_value` regardless of the
    // requested `-b:v`, deferring the real value to
    // `bit_rate_extension` in `sequence_extension()`. The spec only
    // forbids `bit_rate == 0`, so any non-zero pattern parses fine.
    assert_ne!(sh.bit_rate, 0, "bit_rate_value: non-zero per §6.3.3");
    assert_eq!(sh.bit_rate, 0x3_FFFF, "ffmpeg observed pattern");
    // ffmpeg's default mpeg2video encoder loads no quantiser
    // matrices in the sequence_header; updates (if any) come via
    // `quant_matrix_extension()`.
    assert!(sh.intra_quant.is_none());
    assert!(sh.non_intra_quant.is_none());
    assert!(!sh.constrained_parameters);
}
