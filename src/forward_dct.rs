//! 8×8 **forward** discrete cosine transform — the encoder companion to
//! the §A IDCT in [`crate::idct`].
//!
//! ## Spec basis
//!
//! Both in-tree MPEG video specs define the forward DCT alongside the
//! inverse in their Annex A. The two-dimensional forward transform, with
//! `N = 8` (ISO/IEC 13818-2 Annex A, formulas as printed; transcribed in
//! `docs/video/mpeg12video/idct-accuracy-spec.md` §1):
//!
//! ```text
//!             2          N-1 N-1                    (2x+1)uπ      (2y+1)vπ
//!  F(u,v) = ----- C(u)C(v) Σ   Σ   f(x,y) · cos --------- · cos ---------
//!             N             x=0 y=0                  2N            2N
//!
//!  with      C(0) = 1/√2,  C(k) = 1 for k > 0.
//! ```
//!
//! The forward DCT is the exact transpose of the IDCT kernel: the same
//! `cos((2x+1)uπ/16)` cosine matrix, with the `C(u)C(v)` orthonormality
//! scale applied on the *output* (transform-domain) side rather than the
//! input side. Evaluated at `f64` precision it round-trips the
//! [`crate::idct::idct_reference_f64`] form to within floating-point
//! noise.
//!
//! ## Role in the encoder
//!
//! The encoder applies [`fdct_8x8`] to a level-shifted intra block
//! (samples in `[-256, +255]` after subtracting the §7.4.1 `128` DC
//! offset for intra, or to a motion-compensated residual for inter)
//! before forward quantisation. The output coefficients `F[v][u]` are
//! the real-valued transform; the forward quantiser
//! ([`crate::forward_quant`]) rounds and scales them into the 12-bit
//! signed `[-2048, +2047]` coefficient domain the VLC layer codes.
//!
//! The transform here is the **encoder's** DCT; it is not gated by the
//! IEEE 1180 accuracy bounds (those constrain the *decoder's* IDCT). It
//! is, however, validated by the encode→decode round-trip: a block
//! pushed through `fdct_8x8` → forward-quantise → inverse-quantise →
//! [`crate::idct::idct_8x8`] reconstructs the input to within the
//! quantiser's rounding error.

#![allow(clippy::needless_range_loop)]

/// The shared `cos((2*x + 1) * u * π / 16)` kernel for `x, u ∈ 0..8`
/// — the IDCT's correctly-rounded constant [`crate::idct::COS_TABLE`]
/// (a runtime `f64::cos()` would make the forward transform, and
/// therefore the encoder's emitted bits, platform-dependent).
fn cos_table_ref() -> &'static [[f64; 8]; 8] {
    &crate::idct::COS_TABLE
}

/// The §A `C(u)` orthonormality scale: `1/√2` for `u = 0`, `1`
/// otherwise.
#[inline]
fn alpha(k: usize) -> f64 {
    if k == 0 {
        core::f64::consts::FRAC_1_SQRT_2
    } else {
        1.0
    }
}

/// Forward 8×8 DCT at `f64` precision — the exact §A forward transform.
///
/// `input[y][x]` is the spatial-domain block (already level-shifted /
/// residual). `output[v][u]` is the transform-domain coefficient block
/// `F(u, v)` with the `(2/N)·C(u)·C(v)` scale applied. Evaluated as the
/// literal 4-D double sum; the cosine kernel is cached.
pub fn fdct_reference_f64(input: &[[f64; 8]; 8]) -> [[f64; 8]; 8] {
    let table = cos_table_ref();
    let mut out = [[0.0f64; 8]; 8];
    for v in 0..8usize {
        for u in 0..8usize {
            let mut sum = 0.0f64;
            for y in 0..8usize {
                for x in 0..8usize {
                    sum += input[y][x] * table[x][u] * table[y][v];
                }
            }
            // (2/N) per dimension → 2/8 · 2/8 = 1/16 once both Σ run,
            // times the C(u)·C(v) orthonormality scale.
            out[v][u] = sum * 0.25 * alpha(u) * alpha(v);
        }
    }
    out
}

/// Single 8-point 1-D forward DCT — inner kernel of [`fdct_candidate_f64`].
///
/// ```text
/// out[u] = (1/2) C(u) Σ_x in[x] cos((2x+1)uπ/16)   for u = 0..8.
/// ```
#[inline]
fn fdct_1d(input: &[f64; 8]) -> [f64; 8] {
    let table = cos_table_ref();
    let mut out = [0.0f64; 8];
    for u in 0..8usize {
        let mut sum = 0.0f64;
        for x in 0..8usize {
            sum += input[x] * table[x][u];
        }
        // 2/N = 1/4 per dimension; split as 1/2 here so the two passes
        // together give the overall 1/16, times C(u).
        out[u] = sum * 0.5 * alpha(u);
    }
    out
}

/// Separable 1-D-pass forward DCT — eight row transforms followed by
/// eight column transforms. Mathematically identical to
/// [`fdct_reference_f64`]; differs only in `f64` rounding order.
pub fn fdct_candidate_f64(input: &[[f64; 8]; 8]) -> [[f64; 8]; 8] {
    // Pass 1: row-wise 1-D FDCT (over x for each fixed y).
    let mut intermediate = [[0.0f64; 8]; 8];
    for y in 0..8usize {
        intermediate[y] = fdct_1d(&input[y]);
    }
    // Pass 2: column-wise 1-D FDCT (over y for each fixed u).
    let mut out = [[0.0f64; 8]; 8];
    for u in 0..8usize {
        let mut col = [0.0f64; 8];
        for y in 0..8usize {
            col[y] = intermediate[y][u];
        }
        let col_out = fdct_1d(&col);
        for v in 0..8usize {
            out[v][u] = col_out[v];
        }
    }
    out
}

/// Integer forward DCT — applies [`fdct_candidate_f64`] to an `i16`
/// spatial block and rounds each coefficient to the nearest integer
/// (ties away from zero, matching the §4.1 `Round` operator).
///
/// `input[y][x]` is the spatial block in `[-256, +255]` (intra blocks
/// are level-shifted by subtracting 128; inter residuals are already
/// signed). The output `F[v][u]` is the rounded real DCT — *not* yet
/// clamped to the 12-bit coefficient range. Forward quantisation
/// ([`crate::forward_quant`]) consumes the real-valued coefficients;
/// the integer rounding here is provided for callers that want a direct
/// transform-domain block for inspection / testing.
pub fn fdct_8x8(input: &[[i16; 8]; 8]) -> [[i32; 8]; 8] {
    let mut promoted = [[0.0f64; 8]; 8];
    for y in 0..8usize {
        for x in 0..8usize {
            promoted[y][x] = f64::from(input[y][x]);
        }
    }
    let real = fdct_candidate_f64(&promoted);
    let mut out = [[0i32; 8]; 8];
    for v in 0..8usize {
        for u in 0..8usize {
            out[v][u] = real[v][u].round() as i32;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::idct::{idct_candidate_f64, idct_reference_f64};

    #[test]
    fn all_zero_block_transforms_to_zero() {
        let out = fdct_8x8(&[[0i16; 8]; 8]);
        for row in &out {
            for &c in row {
                assert_eq!(c, 0);
            }
        }
    }

    #[test]
    fn flat_block_has_only_dc() {
        // A flat block of value K has only a DC coefficient. With the
        // (2/N)C(0)C(0) scale, F[0][0] = (1/16)·(1/2)·(Σ K) =
        // (1/16)·(1/2)·(64K) = 2K. For K=8 → F[0][0]=64.
        let input = [[8i16; 8]; 8];
        let out = fdct_8x8(&input);
        assert_eq!(out[0][0], 64);
        for v in 0..8 {
            for u in 0..8 {
                if (u, v) != (0, 0) {
                    assert_eq!(out[v][u], 0, "AC ({u},{v}) of flat block must be 0");
                }
            }
        }
    }

    #[test]
    fn fdct_then_idct_roundtrips_within_fp_noise() {
        // The forward DCT followed by the reference inverse DCT must
        // recover the input to within floating-point noise (the
        // orthonormal transform pair is exact in infinite precision).
        let mut input = [[0.0f64; 8]; 8];
        for y in 0..8usize {
            for x in 0..8usize {
                input[y][x] = ((y as f64 * 37.0 + x as f64 * 13.0) % 211.0) - 105.0;
            }
        }
        let coeffs = fdct_candidate_f64(&input);
        let recon = idct_reference_f64(&coeffs);
        for y in 0..8 {
            for x in 0..8 {
                let diff = (recon[y][x] - input[y][x]).abs();
                assert!(diff < 1e-9, "roundtrip diff {diff} at ({y},{x})");
            }
        }
    }

    #[test]
    fn reference_and_candidate_agree() {
        let mut input = [[0.0f64; 8]; 8];
        for y in 0..8usize {
            for x in 0..8usize {
                input[y][x] = ((y as f64 * 19.0 + x as f64 * 7.0) % 173.0) - 86.0;
            }
        }
        let r = fdct_reference_f64(&input);
        let c = fdct_candidate_f64(&input);
        for v in 0..8 {
            for u in 0..8 {
                assert!((r[v][u] - c[v][u]).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn integer_roundtrip_through_idct_is_near_exact() {
        // A real spatial block pushed through the integer FDCT then the
        // integer IDCT must reconstruct close to the input — the only
        // loss is the two integer-rounding steps (no quantisation).
        let mut input = [[0i16; 8]; 8];
        for y in 0..8usize {
            for x in 0..8usize {
                input[y][x] = (((y * 8 + x) as i16 * 3) % 200) - 100;
            }
        }
        let coeffs = fdct_8x8(&input);
        // Promote to f64 for the reference inverse.
        let mut cf = [[0.0f64; 8]; 8];
        for v in 0..8 {
            for u in 0..8 {
                cf[v][u] = f64::from(coeffs[v][u]);
            }
        }
        let recon = idct_candidate_f64(&cf);
        for y in 0..8 {
            for x in 0..8 {
                let diff = (recon[y][x] - f64::from(input[y][x])).abs();
                // Two integer roundings bound the error to ~1.
                assert!(diff <= 1.5, "diff {diff} at ({y},{x})");
            }
        }
    }
}
