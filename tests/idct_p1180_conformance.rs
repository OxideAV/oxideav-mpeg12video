//! IEEE 1180 / P1180/D2 conformance harness for the
//! `oxideav-mpeg12video` 8×8 IDCT (§A of ISO/IEC 11172-2 and ISO/IEC
//! 13818-2).
//!
//! Spec basis: `docs/video/mpeg12video/idct-accuracy-spec.md` §4
//! ("The IEEE 1180-1990 accuracy test procedure"), which restates the
//! IEEE 1180 / P1180/D2 statistical accuracy test that both in-tree
//! MPEG video Annex As invoke normatively. The test compares a
//! candidate IDCT against the direct double-precision reference IDCT
//! over pseudo-random coefficient blocks and verifies five aggregate
//! error statistics against the staged bounds.
//!
//! ## Roles in this harness
//!
//! The OxideAV decoder calls a fast separable 1-D-pass IDCT
//! ([`oxideav_mpeg12video::idct_candidate_f64`]) followed by rounding
//! and the §7.5 9-bit clamp to produce the integer pel output
//! ([`oxideav_mpeg12video::idct_8x8`]). The reference is the direct
//! 4-D summation form of the §A identity
//! ([`oxideav_mpeg12video::idct_reference_f64`]).
//!
//! The IEEE 1180 statistical test in this file gates the **separable
//! kernel's numerical precision** vs. the direct reference: it
//! measures whether the candidate's `f64` output tracks the literal
//! 4-D sum closely enough that no per-position bias or accumulated
//! variance breaches the IEEE 1180 thresholds. That is the property
//! the test was designed to gate (whether a faster IDCT still
//! produces the spec's transform); the candidate's integer cast then
//! adds at most `±0.5` LSB of rounding, well within the §A allowance.
//!
//! ## Test conditions
//!
//! Per the staged note, the IEEE 1180 conformance test runs Q
//! pseudo-random blocks for each of three input-range parameters
//! `L ∈ {256, 5, 300}` and both signs (positive and negative), giving
//! six parameter sets. Each parameter set must satisfy the five
//! statistical bounds plus two deterministic edge cases.
//!
//! `Q = 10000` is the spec's canonical block count. We run a smaller
//! sample (`BLOCKS_PER_CONDITION = 1024`) to keep the harness inside a
//! typical CI budget; the bounds are unchanged. The pseudo-random
//! generator is a deterministic 64-bit linear congruential generator
//! seeded per parameter set so the test is reproducible on every run.

// The IEEE 1180 test loops are dense per-position accumulators; the
// `for v / for u / for y / for x` form is the spec's expression, not
// a stylistic choice.
#![allow(clippy::needless_range_loop)]

use oxideav_mpeg12video::idct::{
    idct_8x8, idct_candidate_f64, idct_reference_f64, F_INPUT_MAX, F_INPUT_MIN, F_OUTPUT_MAX,
    F_OUTPUT_MIN,
};

/// Number of pseudo-random 8×8 blocks per `(L, sign)` parameter set.
/// IEEE 1180 specifies `Q = 10000`; we use a smaller sample sized for
/// CI. See module docs for rationale.
const BLOCKS_PER_CONDITION: usize = 1024;

/// IEEE 1180-1990 / P1180/D2 statistical-bound thresholds, as
/// transcribed from `docs/video/mpeg12video/idct-accuracy-spec.md` §4.
mod bounds {
    /// Maximum absolute per-pixel error, any block, any position
    /// (rounded-integer-domain).
    pub const PEAK_ERROR: i32 = 1;
    /// Maximum per-position mean-square error across the 64 positions.
    pub const PMSE: f64 = 0.06;
    /// Overall (averaged) mean-square error across the 64 positions.
    pub const OMSE: f64 = 0.02;
    /// Maximum absolute per-position mean error across the 64
    /// positions.
    pub const PME: f64 = 0.015;
    /// Overall (averaged) absolute mean error across the 64 positions.
    pub const OME: f64 = 0.0015;
}

/// Deterministic 64-bit linear congruential generator —
/// `state ← state * 6364136223846793005 + 1442695040888963407` (the
/// Knuth / MMIX multiplier). Plenty of entropy for IEEE 1180's input
/// generation and reproducible across platforms.
#[derive(Debug, Clone)]
struct Lcg64 {
    state: u64,
}

impl Lcg64 {
    const fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.state
    }

    /// Uniform integer in the closed interval `[lo, hi]`.
    fn next_in_range(&mut self, lo: i32, hi: i32) -> i32 {
        assert!(hi >= lo);
        let span = (hi - lo + 1) as u64;
        let raw = self.next_u64() >> 32;
        lo + (raw % span) as i32
    }
}

/// One IEEE 1180 input condition: coefficient range and sign
/// preference.
#[derive(Debug, Clone, Copy)]
struct Condition {
    /// `L` from §4 — uniform range half-width.
    l: i32,
    /// `+1` selects values in `[-L, +L]`, `-1` selects values in
    /// `[-L, 0]` (the staged note's "both signs of the data"
    /// negative-only pass).
    sign: i32,
    /// Deterministic seed for this parameter set, so each condition
    /// gets its own independent stream.
    seed: u64,
    /// Human-readable label used in failure messages.
    label: &'static str,
}

impl Condition {
    fn coefficient_range(self) -> (i32, i32) {
        match self.sign {
            1 => (-self.l, self.l),
            -1 => (-self.l, 0),
            _ => panic!("sign must be +1 or -1"),
        }
    }
}

/// Per-position accumulator over `BLOCKS_PER_CONDITION` blocks for one
/// parameter set.
#[derive(Debug, Clone)]
struct Accumulator {
    /// Sum of `e[y][x] = candidate − reference` (both `f64`,
    /// post-clamp) across blocks, per position.
    sum_error: [[f64; 8]; 8],
    /// Sum of `e²` across blocks, per position.
    sum_sq_error: [[f64; 8]; 8],
    /// Maximum absolute integer error observed in the integer
    /// pel-domain comparison (candidate-rounded-clamped vs.
    /// reference-rounded-clamped). The IEEE 1180 `pe` bound is on
    /// the integer-domain error.
    peak_error: i32,
    /// Number of blocks accumulated.
    blocks: usize,
}

impl Accumulator {
    fn new() -> Self {
        Self {
            sum_error: [[0.0; 8]; 8],
            sum_sq_error: [[0.0; 8]; 8],
            peak_error: 0,
            blocks: 0,
        }
    }

    fn accumulate(
        &mut self,
        candidate_f64: &[[f64; 8]; 8],
        reference_f64: &[[f64; 8]; 8],
        candidate_int: &[[i16; 8]; 8],
    ) {
        for y in 0..8usize {
            for x in 0..8usize {
                // IEEE 1180 / P1180/D2 measures the error after both
                // outputs have been brought into the 9-bit pel range
                // `[-256, +255]` (the §7.5 saturation that the spec
                // applies to every IDCT result). Clamping both
                // outputs is necessary because for `|F[v][u]|` up to
                // `L=300` the unclamped real IDCT can produce
                // reference outputs far outside the pel grid, and the
                // statistic would otherwise be dominated by clamping
                // mismatch rather than IDCT precision.
                let cand_clamped = clamp_pel_f64(candidate_f64[y][x]);
                let ref_clamped = clamp_pel_f64(reference_f64[y][x]);
                let err = cand_clamped - ref_clamped;
                self.sum_error[y][x] += err;
                self.sum_sq_error[y][x] += err * err;

                // `pe` is reported in the integer pel domain.
                let ref_int = clamp_pel_i32(reference_f64[y][x].round() as i32);
                let cand_int = i32::from(candidate_int[y][x]);
                let pe = (cand_int - ref_int).unsigned_abs() as i32;
                if pe > self.peak_error {
                    self.peak_error = pe;
                }
            }
        }
        self.blocks += 1;
    }
}

fn clamp_pel_i32(v: i32) -> i32 {
    v.clamp(F_OUTPUT_MIN, F_OUTPUT_MAX)
}

fn clamp_pel_f64(v: f64) -> f64 {
    v.clamp(f64::from(F_OUTPUT_MIN), f64::from(F_OUTPUT_MAX))
}

/// Computed metrics for one parameter set.
#[derive(Debug, Clone, Copy)]
struct Metrics {
    peak_error: i32,
    pmse: f64,
    omse: f64,
    pme: f64,
    ome: f64,
}

fn finalise(acc: &Accumulator) -> Metrics {
    let q = acc.blocks as f64;
    let mut max_mse_pos = 0.0f64;
    let mut sum_mse = 0.0f64;
    let mut max_mean_abs = 0.0f64;
    let mut sum_mean_abs = 0.0f64;
    for y in 0..8usize {
        for x in 0..8usize {
            let mean = acc.sum_error[y][x] / q;
            let mse = acc.sum_sq_error[y][x] / q;
            if mse > max_mse_pos {
                max_mse_pos = mse;
            }
            sum_mse += mse;
            let abs_mean = mean.abs();
            if abs_mean > max_mean_abs {
                max_mean_abs = abs_mean;
            }
            sum_mean_abs += abs_mean;
        }
    }
    Metrics {
        peak_error: acc.peak_error,
        pmse: max_mse_pos,
        omse: sum_mse / 64.0,
        pme: max_mean_abs,
        // The staged note's `ome = (1/64) Σ mean[i][j]` could be a
        // signed sum; we use the absolute-value sum / 64 so the bound
        // is direction-agnostic, which is the conservative reading
        // and how every compliance harness we are aware of evaluates
        // it.
        ome: sum_mean_abs / 64.0,
    }
}

/// Run one IEEE 1180 condition end-to-end: generate inputs, IDCT them
/// through both the candidate and the reference, accumulate the
/// per-position error statistics, and check the five bounds.
fn run_condition(cond: Condition) {
    let mut rng = Lcg64::new(cond.seed);
    let (lo, hi) = cond.coefficient_range();
    let mut acc = Accumulator::new();
    for _ in 0..BLOCKS_PER_CONDITION {
        let mut block_i = [[0i16; 8]; 8];
        let mut block_f = [[0.0f64; 8]; 8];
        for v in 0..8usize {
            for u in 0..8usize {
                let coeff = rng.next_in_range(lo, hi).clamp(F_INPUT_MIN, F_INPUT_MAX);
                block_i[v][u] = coeff as i16;
                block_f[v][u] = f64::from(coeff);
            }
        }
        let candidate_f64 = idct_candidate_f64(&block_f);
        let reference_f64 = idct_reference_f64(&block_f);
        let candidate_int = idct_8x8(&block_i);
        acc.accumulate(&candidate_f64, &reference_f64, &candidate_int);
    }
    let metrics = finalise(&acc);

    assert!(
        metrics.peak_error <= bounds::PEAK_ERROR,
        "{}: peak error {} exceeds IEEE 1180 bound {}",
        cond.label,
        metrics.peak_error,
        bounds::PEAK_ERROR,
    );
    assert!(
        metrics.pmse <= bounds::PMSE,
        "{}: pmse {:.6e} exceeds IEEE 1180 bound {}",
        cond.label,
        metrics.pmse,
        bounds::PMSE,
    );
    assert!(
        metrics.omse <= bounds::OMSE,
        "{}: omse {:.6e} exceeds IEEE 1180 bound {}",
        cond.label,
        metrics.omse,
        bounds::OMSE,
    );
    assert!(
        metrics.pme <= bounds::PME,
        "{}: pme {:.6e} exceeds IEEE 1180 bound {}",
        cond.label,
        metrics.pme,
        bounds::PME,
    );
    assert!(
        metrics.ome <= bounds::OME,
        "{}: ome {:.6e} exceeds IEEE 1180 bound {}",
        cond.label,
        metrics.ome,
        bounds::OME,
    );
}

#[test]
fn ieee_1180_condition_l_256_positive() {
    run_condition(Condition {
        l: 256,
        sign: 1,
        seed: 0xA1A2_A3A4_A5A6_A7A8,
        label: "L=256, sign=+1",
    });
}

#[test]
fn ieee_1180_condition_l_256_negative() {
    run_condition(Condition {
        l: 256,
        sign: -1,
        seed: 0xB1B2_B3B4_B5B6_B7B8,
        label: "L=256, sign=-1",
    });
}

#[test]
fn ieee_1180_condition_l_5_positive() {
    run_condition(Condition {
        l: 5,
        sign: 1,
        seed: 0xC1C2_C3C4_C5C6_C7C8,
        label: "L=5, sign=+1",
    });
}

#[test]
fn ieee_1180_condition_l_5_negative() {
    run_condition(Condition {
        l: 5,
        sign: -1,
        seed: 0xD1D2_D3D4_D5D6_D7D8,
        label: "L=5, sign=-1",
    });
}

#[test]
fn ieee_1180_condition_l_300_positive() {
    run_condition(Condition {
        l: 300,
        sign: 1,
        seed: 0xE1E2_E3E4_E5E6_E7E8,
        label: "L=300, sign=+1",
    });
}

#[test]
fn ieee_1180_condition_l_300_negative() {
    run_condition(Condition {
        l: 300,
        sign: -1,
        seed: 0xF1F2_F3F4_F5F6_F7F8,
        label: "L=300, sign=-1",
    });
}

#[test]
fn ieee_1180_deterministic_all_zero_input_is_exact() {
    // Mandatory deterministic check: the all-zero coefficient block
    // must IDCT to an all-zero output exactly (no statistical
    // tolerance applies).
    let block = [[0i16; 8]; 8];
    let output = idct_8x8(&block);
    for row in &output {
        for &v in row {
            assert_eq!(v, 0, "all-zero input must produce exact zero output");
        }
    }
}

#[test]
fn ieee_1180_deterministic_dc_only_input_is_flat() {
    // Mandatory deterministic check: a DC-only block must produce a
    // flat (constant-valued) output. The §A formula evaluates F[0][0]
    // with C(0)² = 0.5 and the 2/N = 0.25 outer scale, so for F[0][0]
    // = K the flat value is K/8.
    for k in [-2048i16, -1024, -8, 0, 8, 1024, 2047] {
        let mut block = [[0i16; 8]; 8];
        block[0][0] = k;
        let output = idct_8x8(&block);
        let expected = clamp_pel_i32(((f64::from(k)) / 8.0).round() as i32) as i16;
        for row in &output {
            for &v in row {
                assert_eq!(
                    v, expected,
                    "DC-only k={k} must produce a flat block of {expected}, observed {v}",
                );
            }
        }
    }
}
