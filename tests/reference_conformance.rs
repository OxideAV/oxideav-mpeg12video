//! Whole-sequence reference-conformance corpus: decode each staged
//! elementary stream with [`decode_video_sequence`] and compare every
//! display-order frame against the committed black-box reference
//! decode (see `tests/fixtures/conformance/README.md` for the
//! generation record and the comparison-contract rationale).
//!
//! MPEG-1 / MPEG-2 delegate the IDCT to IEEE 1180 statistical
//! accuracy rather than a bit-exact definition (ISO/IEC 11172-2
//! Annex A / ISO/IEC 13818-2 Annex A), so two conforming decoders can
//! differ by ±1 per IDCT and boundedly more through the prediction
//! chain. The contract asserted here:
//!
//! * frame count and dimensions exactly match the reference;
//! * every sample within `|Δ| <= 3`;
//! * fewer than 5 % of samples differ per frame.
//!
//! Anything beyond that is a real decoder divergence, not tolerance.

use oxideav_mpeg12video::decode_video_sequence;

/// Per-sample deviation the Annex A IDCT freedom permits, including
/// bounded propagation through a two-anchor prediction chain.
const MAX_ABS_DELTA: i32 = 3;
/// Per-frame bound on the fraction of samples allowed to differ at
/// all (per mille to stay integer).
const MAX_DIFF_PER_MILLE: u64 = 50;

struct Fixture {
    name: &'static str,
    stream: &'static [u8],
    reference: &'static [u8],
    /// Visible luma dimensions.
    width: usize,
    height: usize,
    /// Chroma plane bytes per frame (both planes together).
    chroma_bytes: usize,
    frames: usize,
}

macro_rules! fixture {
    ($name:literal, $w:expr, $h:expr, chroma420, $frames:expr) => {
        Fixture {
            name: $name,
            stream: include_bytes!(concat!("fixtures/conformance/", $name)),
            reference: include_bytes!(concat!("fixtures/conformance/", $name, ".ref.yuv")),
            width: $w,
            height: $h,
            chroma_bytes: ($w / 2 + $w % 2) * ($h / 2 + $h % 2) * 2,
            frames: $frames,
        }
    };
    ($name:literal, $w:expr, $h:expr, chroma422, $frames:expr) => {
        Fixture {
            name: $name,
            stream: include_bytes!(concat!("fixtures/conformance/", $name)),
            reference: include_bytes!(concat!("fixtures/conformance/", $name, ".ref.yuv")),
            width: $w,
            height: $h,
            chroma_bytes: ($w / 2) * $h * 2,
            frames: $frames,
        }
    };
}

// The `.ref.yuv` name is derived by the macro, so the literal is the
// stream file name.
fn run(fixture: &Fixture) {
    let frames = decode_video_sequence(fixture.stream)
        .unwrap_or_else(|e| panic!("{}: decode failed: {e:?}", fixture.name));

    let frame_bytes = fixture.width * fixture.height + fixture.chroma_bytes;
    assert_eq!(
        fixture.reference.len(),
        frame_bytes * fixture.frames,
        "{}: reference size / frame-count mismatch (fixture table wrong?)",
        fixture.name
    );
    assert_eq!(
        frames.len(),
        fixture.frames,
        "{}: decoded frame count != reference",
        fixture.name
    );

    for (index, decoded) in frames.iter().enumerate() {
        let fb = &decoded.frame;
        assert_eq!(
            (fb.width, fb.height),
            (fixture.width, fixture.height),
            "{}: frame {index} visible dimensions",
            fixture.name
        );
        let (cw, ch) = fb.visible_chroma_dims();
        let mut ours = fb.y.packed_rect(fb.width, fb.height);
        ours.extend_from_slice(&fb.cb.packed_rect(cw, ch));
        ours.extend_from_slice(&fb.cr.packed_rect(cw, ch));
        assert_eq!(
            ours.len(),
            frame_bytes,
            "{}: frame {index} packed size",
            fixture.name
        );

        let reference = &fixture.reference[index * frame_bytes..(index + 1) * frame_bytes];
        let mut diff_count = 0u64;
        for (pos, (&a, &b)) in ours.iter().zip(reference.iter()).enumerate() {
            let delta = (i32::from(a) - i32::from(b)).abs();
            if delta != 0 {
                diff_count += 1;
                assert!(
                    delta <= MAX_ABS_DELTA,
                    "{}: frame {index} byte {pos}: |{a} - {b}| = {delta} exceeds the IDCT-rounding bound",
                    fixture.name
                );
            }
        }
        let per_mille = diff_count * 1000 / frame_bytes as u64;
        assert!(
            per_mille <= MAX_DIFF_PER_MILLE,
            "{}: frame {index}: {diff_count}/{frame_bytes} samples differ ({per_mille}‰) — structural divergence",
            fixture.name
        );
    }
}

#[test]
fn mpeg1_ibbp_gop_reference_conformant() {
    run(&fixture!("mpeg1-ibbp-96x64.m1v", 96, 64, chroma420, 30));
}

#[test]
fn mpeg1_high_motion_wide_f_code_reference_conformant() {
    run(&fixture!(
        "mpeg1-bigmv-160x128.m1v",
        160,
        128,
        chroma420,
        24
    ));
}

#[test]
fn mpeg1_vcd_rate_cbr_sif_reference_conformant() {
    run(&fixture!("mpeg1-vcd-352x240.m1v", 352, 240, chroma420, 9));
}

#[test]
fn mpeg2_ibbp_adaptive_quant_reference_conformant() {
    run(&fixture!("mpeg2-ibbp-96x64.m2v", 96, 64, chroma420, 30));
}

#[test]
fn mpeg2_interlaced_field_prediction_reference_conformant() {
    run(&fixture!("mpeg2-ilaced-96x64.m2v", 96, 64, chroma420, 20));
}

#[test]
fn mpeg2_intra_vlc_nonlinear_quant_reference_conformant() {
    run(&fixture!("mpeg2-ivlc-96x64.m2v", 96, 64, chroma420, 20));
}

#[test]
fn mpeg2_422_profile_reference_conformant() {
    run(&fixture!("mpeg2-422-96x64.m2v", 96, 64, chroma422, 15));
}

#[test]
fn mpeg2_non_mb_multiple_dimensions_reference_conformant() {
    run(&fixture!("mpeg2-100x62.m2v", 100, 62, chroma420, 15));
}

#[test]
fn mpeg2_interlaced_height48_grid_reference_conformant() {
    // §6.3.3: with progressive_sequence == 0 a frame picture codes
    // 2*Ceil(48/32) = 4 macroblock rows (64 lines) even though only
    // 48 lines are visible — this high-motion interlaced stream
    // (vertical f_codes up to the 63-sample search range) exercises
    // the fourth macroblock row as reference material.
    run(&fixture!(
        "mpeg2-ilaced48hm-96x48.m2v",
        96,
        48,
        chroma420,
        18
    ));
}

#[test]
fn mpeg2_downloaded_quant_matrices_reference_conformant() {
    // Custom intra + non-intra quantiser matrices downloaded by the
    // stream (§6.3.11): the decode is only reference-conformant if
    // the §7.4.2.3 reconstruction uses the downloaded matrices, not
    // the §6.3.7 defaults.
    run(&fixture!("mpeg2-qmat-96x64.m2v", 96, 64, chroma420, 20));
}

#[test]
fn field_picture_pairs_dual_prime_16x8_reference_conformant() {
    // Hand-built field-picture stream (see the fixture README):
    // I/P/B field pairs exercising simple field prediction with both
    // field selects (incl. §7.6.2.1 same-frame second-field
    // references), §7.6.3.6 dual prime, §7.6.7.3 16x8 MC, and an
    // interpolated B-field pair.
    run(&fixture!("fieldpics-48x64.m2v", 48, 64, chroma420, 5));
}

#[test]
fn display_order_matches_temporal_references() {
    // Cross-check the §6.1.1.11 structural reorder against the
    // temporal_reference-derived display order on the two IBBP
    // corpora (one per standard). The GOPs here open on an I-frame
    // with temporal_reference > 0 (leading B-frames), so the
    // anchor-pattern (types-aware) verifier is required.
    use oxideav_mpeg12video::{verify_display_order_with_types, PictureCodingType};
    for (name, stream) in [
        (
            "mpeg1-ibbp",
            &include_bytes!("fixtures/conformance/mpeg1-ibbp-96x64.m1v")[..],
        ),
        (
            "mpeg2-ibbp",
            &include_bytes!("fixtures/conformance/mpeg2-ibbp-96x64.m2v")[..],
        ),
    ] {
        let frames = decode_video_sequence(stream).unwrap();
        let display: Vec<u16> = frames.iter().map(|f| f.temporal_reference).collect();
        // Coded order: scan the picture headers off the wire.
        let mut coded: Vec<(u16, PictureCodingType)> = Vec::new();
        let mut i = 0usize;
        while let Some(pos) = stream[i..]
            .windows(4)
            .position(|w| w[0] == 0 && w[1] == 0 && w[2] == 1 && w[3] == 0)
        {
            let off = i + pos;
            let b = &stream[off + 4..off + 6];
            let tr = (u16::from(b[0]) << 2) | (u16::from(b[1]) >> 6);
            let kind = match (b[1] >> 3) & 0x7 {
                1 => PictureCodingType::Intra,
                2 => PictureCodingType::Predictive,
                3 => PictureCodingType::Bidirectional,
                other => panic!("{name}: unexpected picture_coding_type {other}"),
            };
            coded.push((tr, kind));
            i = off + 4;
        }
        assert_eq!(coded.len(), frames.len(), "{name}: picture count");
        verify_display_order_with_types(&coded, &display)
            .unwrap_or_else(|e| panic!("{name}: display order: {e:?}"));
    }
}
