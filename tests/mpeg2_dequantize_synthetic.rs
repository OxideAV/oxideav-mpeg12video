//! End-to-end synthetic-fixture test for the MPEG-2 (ISO/IEC
//! 13818-2 / ITU-T H.262) §7.4 inverse-quantisation pipeline.
//!
//! These tests exercise the public surface of
//! `oxideav_mpeg12video::mpeg2_dequantize` against the §7.4 worked
//! examples — Table 7-4 `intra_dc_mult`, Table 7-5 weighting-matrix
//! selection, Table 7-6 `quantiser_scale_code → quantiser_scale`,
//! and §7.4.5 end-to-end reference loops.
//!
//! Spec basis: ITU-T H.262 / ISO/IEC 13818-2 §7.4 (pages 73–76).

// The §7.4.3 saturation in [`reference_inverse_quantise`] is
// transcribed as an if/else-if/else chain to match the spec's
// printed pseudocode. Rewriting it with `.clamp()` would obscure
// the spec-trace correspondence.
#![allow(clippy::manual_clamp)]
// Same rationale as the module's inner allow — the reference loop
// is a literal port of the §7.4 `for v / for u` pseudocode.
#![allow(clippy::needless_range_loop)]

use oxideav_mpeg12video::mpeg2_dequantize::{
    intra_dc_mult, inverse_quantise_block, quantiser_scale, saturate,
    select_weighting_matrix_index, sign, BlockCoding, Component, DEFAULT_INTRA_WEIGHT,
    DEFAULT_NON_INTRA_WEIGHT, F_SATURATION_MAX, F_SATURATION_MIN, QUANTISER_SCALE_LINEAR,
    QUANTISER_SCALE_NONLINEAR,
};
use oxideav_mpeg12video::sequence_extension::ChromaFormat;

/// Re-implement §7.4.5 in this file from the spec text so that the
/// crate's [`inverse_quantise_block`] is checked against an
/// independent reference.
fn reference_inverse_quantise(
    qf: &[[i32; 8]; 8],
    intra: bool,
    weight: &[[u8; 8]; 8],
    qs: i32,
    intra_dc_mult_value: i32,
) -> [[i32; 8]; 8] {
    let mut f_pp = [[0i32; 8]; 8];
    for v in 0..8usize {
        for u in 0..8usize {
            if u == 0 && v == 0 && intra {
                f_pp[v][u] = intra_dc_mult_value * qf[v][u];
            } else if intra {
                f_pp[v][u] = (qf[v][u] * 2) * i32::from(weight[v][u]) * qs / 32;
            } else {
                let k = sign(qf[v][u]);
                f_pp[v][u] = (((qf[v][u] * 2) + k) * i32::from(weight[v][u]) * qs) / 32;
            }
        }
    }
    let mut sum = 0i32;
    let mut f_p = [[0i32; 8]; 8];
    for v in 0..8 {
        for u in 0..8 {
            f_p[v][u] = if f_pp[v][u] > 2047 {
                2047
            } else if f_pp[v][u] < -2048 {
                -2048
            } else {
                f_pp[v][u]
            };
            sum += f_p[v][u];
        }
    }
    let mut f = f_p;
    if (sum & 1) == 0 {
        if (f[7][7] & 1) != 0 {
            f[7][7] -= 1;
        } else {
            f[7][7] += 1;
        }
    }
    f
}

#[test]
fn matches_independent_reference_on_intra_block() {
    // Synthetic intra block — DC plus a handful of AC entries.
    let mut qf = [[0i32; 8]; 8];
    qf[0][0] = 17;
    qf[0][1] = 3;
    qf[1][0] = -4;
    qf[2][3] = 12;
    qf[5][5] = -7;

    let intra_dc_mult_value = intra_dc_mult(1).unwrap();
    let qs = quantiser_scale(7, false).unwrap();
    let f_lib = inverse_quantise_block(
        &qf,
        BlockCoding::Intra,
        &DEFAULT_INTRA_WEIGHT,
        qs,
        intra_dc_mult_value,
    );
    let f_ref = reference_inverse_quantise(
        &qf,
        true,
        &DEFAULT_INTRA_WEIGHT,
        i32::from(qs),
        intra_dc_mult_value,
    );
    assert_eq!(f_lib, f_ref);
}

#[test]
fn matches_independent_reference_on_non_intra_block() {
    // Mix of positive, negative, and zero coefficients to exercise
    // every branch of the non-intra k = Sign(QF) handling.
    let mut qf = [[0i32; 8]; 8];
    qf[0][0] = 0; // non-intra has no §7.4.1 short-circuit at [0][0].
    qf[0][1] = 5;
    qf[0][2] = -5;
    qf[3][4] = 9;
    qf[7][7] = -3;

    let qs = quantiser_scale(14, true).unwrap(); // non-linear column.
    let f_lib = inverse_quantise_block(
        &qf,
        BlockCoding::NonIntra,
        &DEFAULT_NON_INTRA_WEIGHT,
        qs,
        // intra_dc_mult is irrelevant here.
        42,
    );
    let f_ref =
        reference_inverse_quantise(&qf, false, &DEFAULT_NON_INTRA_WEIGHT, i32::from(qs), 42);
    assert_eq!(f_lib, f_ref);
}

#[test]
fn matches_independent_reference_for_all_quantiser_scale_codes() {
    // Loop over every legal q_scale_code in both q_scale_type
    // columns. A tiny non-intra block keeps the assertion fast
    // while still exercising the lookup table.
    let mut qf = [[0i32; 8]; 8];
    qf[2][2] = 11;
    qf[3][3] = -6;

    for q_scale_type in [false, true] {
        for code in 1u8..=31 {
            let qs = quantiser_scale(code, q_scale_type).unwrap();
            let f_lib = inverse_quantise_block(
                &qf,
                BlockCoding::NonIntra,
                &DEFAULT_NON_INTRA_WEIGHT,
                qs,
                42,
            );
            let f_ref = reference_inverse_quantise(
                &qf,
                false,
                &DEFAULT_NON_INTRA_WEIGHT,
                i32::from(qs),
                42,
            );
            assert_eq!(
                f_lib, f_ref,
                "mismatch at q_scale_type = {q_scale_type}, code = {code}"
            );
        }
    }
}

#[test]
fn intra_dc_mult_full_table() {
    // Walk Table 7-4 to confirm the bits-of-precision relation
    // 2 ^ (3 - intra_dc_precision).
    let expected = [(0u8, 8i32), (1, 4), (2, 2), (3, 1)];
    for (precision, mult) in expected {
        assert_eq!(intra_dc_mult(precision).unwrap(), mult);
    }
}

#[test]
fn quantiser_scale_lookup_arrays_match_spec_columns() {
    // The first slot of each Table 7-6 column is the forbidden
    // marker (`0`); every other slot is exercised by the safe
    // accessor in the lib unit tests, but we also assert the raw
    // arrays here so a future renumbering would trip the test.
    assert_eq!(QUANTISER_SCALE_LINEAR[0], 0);
    assert_eq!(QUANTISER_SCALE_LINEAR[31], 62);
    assert_eq!(QUANTISER_SCALE_NONLINEAR[0], 0);
    assert_eq!(QUANTISER_SCALE_NONLINEAR[31], 112);
}

#[test]
fn weighting_matrix_selection_table_7_5() {
    // Drive Table 7-5 through every cell.
    let cases = [
        // (coding, component, chroma_format, expected_w)
        (
            BlockCoding::Intra,
            Component::Luminance,
            ChromaFormat::Yuv420,
            0,
        ),
        (
            BlockCoding::Intra,
            Component::Chrominance,
            ChromaFormat::Yuv420,
            0,
        ),
        (
            BlockCoding::NonIntra,
            Component::Luminance,
            ChromaFormat::Yuv420,
            1,
        ),
        (
            BlockCoding::NonIntra,
            Component::Chrominance,
            ChromaFormat::Yuv420,
            1,
        ),
        (
            BlockCoding::Intra,
            Component::Luminance,
            ChromaFormat::Yuv422,
            0,
        ),
        (
            BlockCoding::Intra,
            Component::Chrominance,
            ChromaFormat::Yuv422,
            2,
        ),
        (
            BlockCoding::NonIntra,
            Component::Luminance,
            ChromaFormat::Yuv422,
            1,
        ),
        (
            BlockCoding::NonIntra,
            Component::Chrominance,
            ChromaFormat::Yuv422,
            3,
        ),
        (
            BlockCoding::Intra,
            Component::Luminance,
            ChromaFormat::Yuv444,
            0,
        ),
        (
            BlockCoding::Intra,
            Component::Chrominance,
            ChromaFormat::Yuv444,
            2,
        ),
        (
            BlockCoding::NonIntra,
            Component::Luminance,
            ChromaFormat::Yuv444,
            1,
        ),
        (
            BlockCoding::NonIntra,
            Component::Chrominance,
            ChromaFormat::Yuv444,
            3,
        ),
    ];
    for (coding, component, format, expected) in cases {
        assert_eq!(
            select_weighting_matrix_index(coding, component, format),
            expected,
            "table 7-5 mismatch at ({coding:?}, {component:?}, {format:?})"
        );
    }
}

#[test]
fn saturation_constants_match_spec_bounds() {
    // §7.4.3 [-2048, 2047]; expose-and-check the constants the lib
    // surfaces so downstream consumers can pin against them.
    assert_eq!(F_SATURATION_MIN, -2048);
    assert_eq!(F_SATURATION_MAX, 2047);
    assert_eq!(saturate(F_SATURATION_MAX + 1), F_SATURATION_MAX);
    assert_eq!(saturate(F_SATURATION_MIN - 1), F_SATURATION_MIN);
}
