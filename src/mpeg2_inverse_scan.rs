//! MPEG-2 §7.3 inverse-scan tables per **ISO/IEC 13818-2:1995
//! (Recommendation ITU-T H.262)** — the two `scan[alternate_scan][v][u]`
//! patterns that map the zig-zag-ordered transform coefficient list
//! `QFS[n]` (output of the §7.2.2 residual VLC walker) back to the
//! two-dimensional block `QF[v][u]` consumed by the §7.4
//! inverse-quantisation pipeline.
//!
//! ## Scope
//!
//! ISO/IEC 13818-2 §7.3 defines two scans selected by the
//! `alternate_scan` flag carried in the picture coding extension
//! (`picture_header::PictureCodingExtension::alternate_scan`):
//!
//! * **Figure 7-2 — `scan[0][v][u]`** (zig-zag scan). The reference
//!   spec page prints the matrix with `v` the vertical (row) index
//!   and `u` the horizontal (column) index. This pattern is
//!   identical, cell-for-cell, to the MPEG-1 §2.4.4.1 `scan[m][n]`
//!   matrix already encoded in [`crate::block_dc::SCAN`]; we
//!   intentionally do not re-encode the bytes here — the table is
//!   re-exported from [`block_dc`] and a unit test in
//!   `tests/mpeg2_inverse_scan_synthetic.rs` asserts the equality so
//!   any future drift on one side trips a regression immediately.
//!
//! * **Figure 7-3 — `scan[1][v][u]`** (alternate scan). Distinct
//!   from Figure 7-2; this module is the only place it appears in
//!   the crate. The matrix walks down column 0 first
//!   (`scan[1][0..=3][0] = {0, 1, 2, 3}` and
//!   `scan[1][4..=7][0] = {10, 11, 12, 13}`) — the contrasting
//!   "across-then-down" shape of Figure 7-2's first column
//!   (`scan[0][0..=7][0] = {0, 2, 3, 9, 10, 20, 21, 35}`) makes
//!   the two scans visually obvious at a glance.
//!
//! Per §7.3, the inverse scan body is:
//!
//! ```text
//! for (v = 0; v < 8; v++)
//!     for (u = 0; u < 8; u++)
//!         QF[v][u] = QFS[scan[alternate_scan][v][u]]
//! ```
//!
//! [`place_coefficient`] is the per-coefficient counterpart used by
//! the §7.2.2 walker: each `RunLevel` symbol emitted by
//! [`crate::mpeg2_dct_coeff::DctCoeffStep`] advances a `QFS[n]`
//! cursor, and [`place_coefficient`] writes the level into
//! `QF[v][u]` at the `(v, u)` named by `scan[alternate_scan][n]`.
//!
//! §7.3.1 ("Inverse scan for matrix download") fixes
//! `alternate_scan = 0` for quantisation-matrix downloads regardless
//! of the picture-coding-extension flag; callers handling matrix
//! download should pass `false` rather than the picture-extension
//! bit.

// The spec body of §7.3 is written as `for (v) for (u) QF[v][u] =
// QFS[scan[…][v][u]]`. The Rust transliteration preserves that
// shape on purpose — `for v in 0..8 { for u in 0..8 { … } }` reads
// one-for-one against the printed pseudo-code and the §7.3 tests
// pin spec-named cells via the same indices. Rewriting these as
// `iter().enumerate()` loops would obscure the spec correspondence
// without changing the generated code, so the relevant
// `needless_range_loop` suggestion is disabled at module scope.
#![allow(clippy::needless_range_loop)]

use crate::block_dc::SCAN;

// =============================================================
// §7.3 Figure 7-3 — scan[1][v][u] (alternate scan)
// =============================================================

/// `scan[1][v][u]` per **ISO/IEC 13818-2:1995 §7.3 Figure 7-3** —
/// the alternate scan selected when `alternate_scan = 1`.
///
/// Stored row-major: `ALTERNATE_SCAN[v][u]` is the spec's
/// `scan[1][v][u]` with `v` the vertical index and `u` the
/// horizontal index.
///
/// The matrix maps a position `(v, u)` of the 8×8 block to its
/// zig-zag-ordered index in the one-dimensional `QFS[n]` list.
/// The companion zig-zag scan (`alternate_scan = 0`) is the
/// MPEG-1 §2.4.4.1 matrix in [`crate::block_dc::SCAN`]; see
/// [`scan_table`] for selection by flag.
pub const ALTERNATE_SCAN: [[u8; 8]; 8] = [
    [0, 4, 6, 20, 22, 36, 38, 52],
    [1, 5, 7, 21, 23, 37, 39, 53],
    [2, 8, 19, 24, 34, 40, 50, 54],
    [3, 9, 18, 25, 35, 41, 51, 55],
    [10, 17, 26, 30, 42, 46, 56, 60],
    [11, 16, 27, 31, 43, 47, 57, 61],
    [12, 15, 28, 32, 44, 48, 58, 62],
    [13, 14, 29, 33, 45, 49, 59, 63],
];

/// Inverse of [`ALTERNATE_SCAN`]: maps a zig-zag *index* `n` in
/// `0..=63` to the `(v, u)` cell in the raster matrix it loads.
///
/// Indexed as `ALTERNATE_INVERSE_SCAN[n] = (v, u)`.
pub const ALTERNATE_INVERSE_SCAN: [(u8, u8); 64] = build_alternate_inverse_scan();

const fn build_alternate_inverse_scan() -> [(u8, u8); 64] {
    let mut out = [(0u8, 0u8); 64];
    let mut v = 0usize;
    while v < 8 {
        let mut u = 0usize;
        while u < 8 {
            let i = ALTERNATE_SCAN[v][u] as usize;
            out[i] = (v as u8, u as u8);
            u += 1;
        }
        v += 1;
    }
    out
}

// =============================================================
// §7.3 scan selector
// =============================================================

/// Returns the §7.3 `scan[alternate_scan][v][u]` matrix selected
/// by the `alternate_scan` flag.
///
/// * `false` → [`crate::block_dc::SCAN`] (Figure 7-2, zig-zag).
/// * `true`  → [`ALTERNATE_SCAN`] (Figure 7-3, alternate scan).
///
/// Per §7.3.1 the matrix-download path always uses the
/// Figure 7-2 scan; pass `false` explicitly for that codepath
/// regardless of the picture-coding-extension flag.
#[inline]
pub fn scan_table(alternate_scan: bool) -> &'static [[u8; 8]; 8] {
    if alternate_scan {
        &ALTERNATE_SCAN
    } else {
        &SCAN
    }
}

/// Returns the inverse mapping for the §7.3 scan selected by
/// `alternate_scan` — i.e. zig-zag index `n` → block cell
/// `(v, u)`.
///
/// The result of `inverse_scan_table(false)` is derived from
/// [`crate::block_dc::INVERSE_SCAN`] (identical contents, same
/// `[(u8, u8); 64]` layout); `inverse_scan_table(true)` returns
/// [`ALTERNATE_INVERSE_SCAN`].
#[inline]
pub fn inverse_scan_table(alternate_scan: bool) -> &'static [(u8, u8); 64] {
    if alternate_scan {
        &ALTERNATE_INVERSE_SCAN
    } else {
        &ZIGZAG_INVERSE_SCAN
    }
}

/// Inverse of [`crate::block_dc::SCAN`] expressed in §7.3 spelling
/// (`(v, u)` rather than `(m, n)` — semantically identical).
///
/// Co-defined here so [`inverse_scan_table`] can return a
/// `&'static [(u8, u8); 64]` for either branch without a runtime
/// branch on the array shape. The derivation walks
/// [`crate::block_dc::SCAN`] at compile time and the
/// `inverse_scan_zigzag_matches_block_dc` unit test asserts the
/// match against [`crate::block_dc::INVERSE_SCAN`].
pub const ZIGZAG_INVERSE_SCAN: [(u8, u8); 64] = build_zigzag_inverse_scan();

const fn build_zigzag_inverse_scan() -> [(u8, u8); 64] {
    let mut out = [(0u8, 0u8); 64];
    let mut v = 0usize;
    while v < 8 {
        let mut u = 0usize;
        while u < 8 {
            let i = SCAN[v][u] as usize;
            out[i] = (v as u8, u as u8);
            u += 1;
        }
        v += 1;
    }
    out
}

// =============================================================
// §7.3 per-coefficient placement
// =============================================================

/// Writes a single coefficient `value` into `qf[v][u]` at the
/// `(v, u)` cell named by `scan[alternate_scan][index]`, per the
/// §7.3 inverse-scan body.
///
/// This is the per-`RunLevel` counterpart of the full §7.3
/// `for (v) for (u) QF[v][u] = QFS[scan[…][v][u]]` loop: when a
/// caller is walking the §7.2.2 residual VLC one symbol at a
/// time, it knows the next `n` (the cursor advanced by
/// `1 + run`) and the `signed_level`; this helper does the
/// scan-table lookup and the store.
///
/// # Panics
///
/// Panics if `index >= 64`. The §7.2.2.4 walker bounds the
/// cursor at 63, so any caller that propagates the walker's
/// `position` field is safe.
#[inline]
pub fn place_coefficient(qf: &mut [[i16; 8]; 8], index: usize, value: i16, alternate_scan: bool) {
    assert!(
        index < 64,
        "MPEG-2 §7.3 inverse-scan index must be < 64, got {index}",
    );
    let (v, u) = inverse_scan_table(alternate_scan)[index];
    qf[v as usize][u as usize] = value;
}

/// Materialises the §7.3 inverse-scan as the full loop body,
/// converting the linear `qfs[0..64]` list into the
/// two-dimensional `qf[v][u]` block.
///
/// This is the direct transliteration of the §7.3 pseudo-code:
///
/// ```text
/// for (v = 0; v < 8; v++)
///     for (u = 0; u < 8; u++)
///         QF[v][u] = QFS[scan[alternate_scan][v][u]]
/// ```
///
/// Callers that have already accumulated a 64-entry coefficient
/// list (e.g. an encoder running the §7.3 forward pass, or a
/// trace tool flattening a decoder fixture) use this entry
/// point; callers walking one `RunLevel` at a time use
/// [`place_coefficient`].
#[inline]
pub fn apply_inverse_scan(qfs: &[i16; 64], alternate_scan: bool) -> [[i16; 8]; 8] {
    let mut qf = [[0i16; 8]; 8];
    let scan = scan_table(alternate_scan);
    for v in 0..8 {
        for u in 0..8 {
            qf[v][u] = qfs[scan[v][u] as usize];
        }
    }
    qf
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block_dc::{INVERSE_SCAN as BLOCK_DC_INVERSE_SCAN, SCAN as BLOCK_DC_SCAN};

    // ------------------------------------------------------------------
    // Matrix-shape invariants — every cell in 0..=63 appears exactly once
    // ------------------------------------------------------------------

    fn assert_permutation(matrix: &[[u8; 8]; 8], label: &str) {
        let mut seen = [false; 64];
        for v in 0..8 {
            for u in 0..8 {
                let n = matrix[v][u] as usize;
                assert!(
                    n < 64,
                    "{label}: scan[{v}][{u}] = {n} out of range (must be < 64)",
                );
                assert!(
                    !seen[n],
                    "{label}: zig-zag index {n} appears twice (second hit at v={v} u={u})",
                );
                seen[n] = true;
            }
        }
        for n in 0..64 {
            assert!(
                seen[n],
                "{label}: zig-zag index {n} is missing from the matrix",
            );
        }
    }

    #[test]
    fn alternate_scan_is_a_permutation_of_0_to_63() {
        assert_permutation(&ALTERNATE_SCAN, "ALTERNATE_SCAN (Figure 7-3)");
    }

    #[test]
    fn zigzag_scan_via_block_dc_is_a_permutation_of_0_to_63() {
        // Sanity — guarantees the round-trip property below isn't
        // hiding a pre-existing block_dc::SCAN bug.
        assert_permutation(&BLOCK_DC_SCAN, "block_dc::SCAN (Figure 7-2)");
    }

    // ------------------------------------------------------------------
    // Single-source-of-truth — Figure 7-2 == block_dc::SCAN
    // ------------------------------------------------------------------

    #[test]
    fn figure_7_2_equals_block_dc_scan_cell_for_cell() {
        // ISO/IEC 13818-2 §7.3 Figure 7-2 is the same matrix as
        // ISO/IEC 11172-2 §2.4.4.1 scan[m][n]. Asserting equality
        // here lets `block_dc::SCAN` stay the single source of
        // truth; any drift on either side trips this test before it
        // can corrupt a decoder.
        for v in 0..8 {
            for u in 0..8 {
                assert_eq!(
                    scan_table(false)[v][u],
                    BLOCK_DC_SCAN[v][u],
                    "Figure 7-2 mismatch at v={v} u={u}",
                );
            }
        }
    }

    #[test]
    fn zigzag_inverse_scan_matches_block_dc_inverse_scan() {
        for n in 0..64 {
            assert_eq!(
                ZIGZAG_INVERSE_SCAN[n], BLOCK_DC_INVERSE_SCAN[n],
                "zig-zag inverse-scan mismatch at n={n}",
            );
        }
    }

    // ------------------------------------------------------------------
    // Corner / endpoint spot-checks against the printed PDF
    // ------------------------------------------------------------------

    #[test]
    fn figure_7_3_spec_corners() {
        // Reproduce the printed Figure 7-3 corner values verbatim.
        // ISO/IEC 13818-2:1995 page 80, scan[1][v][u]:
        //   (0,0)=0 (0,7)=52 (7,0)=13 (7,7)=63
        assert_eq!(ALTERNATE_SCAN[0][0], 0);
        assert_eq!(ALTERNATE_SCAN[0][7], 52);
        assert_eq!(ALTERNATE_SCAN[7][0], 13);
        assert_eq!(ALTERNATE_SCAN[7][7], 63);
    }

    #[test]
    fn figure_7_3_column_0_walks_down_first() {
        // Distinguishing feature of the alternate scan: the first
        // four rows of column 0 are 0,1,2,3 and the next four are
        // 10,11,12,13 — i.e. the scan walks down column 0 before
        // crossing to column 1, unlike Figure 7-2 which crosses
        // immediately.
        assert_eq!(ALTERNATE_SCAN[0][0], 0);
        assert_eq!(ALTERNATE_SCAN[1][0], 1);
        assert_eq!(ALTERNATE_SCAN[2][0], 2);
        assert_eq!(ALTERNATE_SCAN[3][0], 3);
        assert_eq!(ALTERNATE_SCAN[4][0], 10);
        assert_eq!(ALTERNATE_SCAN[5][0], 11);
        assert_eq!(ALTERNATE_SCAN[6][0], 12);
        assert_eq!(ALTERNATE_SCAN[7][0], 13);
    }

    #[test]
    fn figure_7_3_row_0_matches_spec() {
        // Row 0 of Figure 7-3 (page 80): 0 4 6 20 22 36 38 52.
        assert_eq!(ALTERNATE_SCAN[0], [0, 4, 6, 20, 22, 36, 38, 52]);
    }

    #[test]
    fn figure_7_3_row_7_matches_spec() {
        // Row 7 of Figure 7-3 (page 80): 13 14 29 33 45 49 59 63.
        assert_eq!(ALTERNATE_SCAN[7], [13, 14, 29, 33, 45, 49, 59, 63]);
    }

    #[test]
    fn figure_7_3_diagonals_distinguish_from_figure_7_2() {
        // A second cross-check that we did not accidentally copy
        // Figure 7-2: confirm one cell that differs between the
        // two scans. scan[0][0][1] = 1 but scan[1][0][1] = 4.
        assert_eq!(BLOCK_DC_SCAN[0][1], 1);
        assert_eq!(ALTERNATE_SCAN[0][1], 4);
        // And scan[0][1][0] = 2 but scan[1][1][0] = 1.
        assert_eq!(BLOCK_DC_SCAN[1][0], 2);
        assert_eq!(ALTERNATE_SCAN[1][0], 1);
    }

    // ------------------------------------------------------------------
    // Round-trip — forward · inverse = identity (both scans)
    // ------------------------------------------------------------------

    #[test]
    fn alternate_scan_forward_and_inverse_round_trip() {
        for v in 0..8 {
            for u in 0..8 {
                let n = ALTERNATE_SCAN[v][u] as usize;
                let (v2, u2) = ALTERNATE_INVERSE_SCAN[n];
                assert_eq!(v2 as usize, v, "v mismatch for n={n}");
                assert_eq!(u2 as usize, u, "u mismatch for n={n}");
            }
        }
        // And the other direction: inverse → forward.
        for n in 0..64 {
            let (v, u) = ALTERNATE_INVERSE_SCAN[n];
            assert_eq!(ALTERNATE_SCAN[v as usize][u as usize] as usize, n);
        }
    }

    #[test]
    fn zigzag_scan_forward_and_inverse_round_trip() {
        for v in 0..8 {
            for u in 0..8 {
                let n = BLOCK_DC_SCAN[v][u] as usize;
                let (v2, u2) = ZIGZAG_INVERSE_SCAN[n];
                assert_eq!(v2 as usize, v, "v mismatch for n={n}");
                assert_eq!(u2 as usize, u, "u mismatch for n={n}");
            }
        }
        for n in 0..64 {
            let (v, u) = ZIGZAG_INVERSE_SCAN[n];
            assert_eq!(BLOCK_DC_SCAN[v as usize][u as usize] as usize, n);
        }
    }

    // ------------------------------------------------------------------
    // Selector
    // ------------------------------------------------------------------

    #[test]
    fn scan_table_selector_branches_correctly() {
        // Compare by value rather than address: rustc is free to
        // instantiate a fresh static for a const reference returned
        // through a function, so `ptr::eq` is not a reliable identity
        // test here. Value equality is what callers actually depend on.
        assert_eq!(*scan_table(false), BLOCK_DC_SCAN);
        assert_eq!(*scan_table(true), ALTERNATE_SCAN);
        // And the two branches must be distinct from each other.
        assert_ne!(*scan_table(false), *scan_table(true));
    }

    #[test]
    fn inverse_scan_table_selector_branches_correctly() {
        assert_eq!(*inverse_scan_table(false), ZIGZAG_INVERSE_SCAN);
        assert_eq!(*inverse_scan_table(true), ALTERNATE_INVERSE_SCAN);
        assert_ne!(*inverse_scan_table(false), *inverse_scan_table(true));
    }

    // ------------------------------------------------------------------
    // place_coefficient
    // ------------------------------------------------------------------

    #[test]
    fn place_coefficient_writes_at_zigzag_position() {
        let mut qf = [[0i16; 8]; 8];
        // Per the MPEG-1 §2.4.4.1 / MPEG-2 Figure 7-2 scan,
        // n = 5 lands at (v, u) = (0, 2).
        place_coefficient(&mut qf, 5, 42, false);
        assert_eq!(qf[0][2], 42);
        // Every other cell stays zero.
        for v in 0..8 {
            for u in 0..8 {
                if !(v == 0 && u == 2) {
                    assert_eq!(qf[v][u], 0, "stray write at v={v} u={u}");
                }
            }
        }
    }

    #[test]
    fn place_coefficient_writes_at_alternate_position() {
        let mut qf = [[0i16; 8]; 8];
        // Per Figure 7-3, n = 4 lands at (v, u) = (0, 1).
        place_coefficient(&mut qf, 4, -123, true);
        assert_eq!(qf[0][1], -123);
        // The same index n=4 under the zig-zag scan would land at
        // (v, u) = (1, 1) (per block_dc::SCAN row 1: 2,4,...) —
        // confirm the alternate placement does NOT touch that cell.
        assert_eq!(qf[1][1], 0);
    }

    #[test]
    fn place_coefficient_index_0_lands_at_origin_for_both_scans() {
        // DC coefficient (n = 0) always lands at (0, 0) regardless
        // of scan flag — both Figure 7-2 and Figure 7-3 print 0
        // at the (0, 0) cell.
        let mut qf0 = [[0i16; 8]; 8];
        place_coefficient(&mut qf0, 0, 17, false);
        assert_eq!(qf0[0][0], 17);

        let mut qf1 = [[0i16; 8]; 8];
        place_coefficient(&mut qf1, 0, 17, true);
        assert_eq!(qf1[0][0], 17);
    }

    #[test]
    fn place_coefficient_index_63_lands_at_corner_for_both_scans() {
        // Both scans land n = 63 at (7, 7).
        let mut qf0 = [[0i16; 8]; 8];
        place_coefficient(&mut qf0, 63, -9, false);
        assert_eq!(qf0[7][7], -9);

        let mut qf1 = [[0i16; 8]; 8];
        place_coefficient(&mut qf1, 63, -9, true);
        assert_eq!(qf1[7][7], -9);
    }

    #[test]
    #[should_panic(expected = "MPEG-2 §7.3 inverse-scan index must be < 64")]
    fn place_coefficient_panics_on_out_of_range_index() {
        let mut qf = [[0i16; 8]; 8];
        place_coefficient(&mut qf, 64, 1, false);
    }

    // ------------------------------------------------------------------
    // apply_inverse_scan — the full §7.3 loop body
    // ------------------------------------------------------------------

    #[test]
    fn apply_inverse_scan_round_trips_through_zigzag() {
        // Build a synthetic QF block with distinct values per cell,
        // forward-scan to QFS, then run the §7.3 inverse-scan loop
        // body and confirm we recover the original block.
        let mut qf_in = [[0i16; 8]; 8];
        let mut next = 1i16;
        for row in &mut qf_in {
            for cell in row.iter_mut() {
                *cell = next;
                next += 1;
            }
        }

        // Forward — walk Figure 7-2 to flatten qf_in -> qfs.
        let mut qfs = [0i16; 64];
        for v in 0..8 {
            for u in 0..8 {
                let n = BLOCK_DC_SCAN[v][u] as usize;
                qfs[n] = qf_in[v][u];
            }
        }

        let qf_out = apply_inverse_scan(&qfs, false);
        assert_eq!(qf_out, qf_in);
    }

    #[test]
    fn apply_inverse_scan_round_trips_through_alternate() {
        let mut qf_in = [[0i16; 8]; 8];
        let mut next = -100i16;
        for row in &mut qf_in {
            for cell in row.iter_mut() {
                *cell = next;
                next += 1;
            }
        }

        let mut qfs = [0i16; 64];
        for v in 0..8 {
            for u in 0..8 {
                let n = ALTERNATE_SCAN[v][u] as usize;
                qfs[n] = qf_in[v][u];
            }
        }

        let qf_out = apply_inverse_scan(&qfs, true);
        assert_eq!(qf_out, qf_in);
    }

    #[test]
    fn apply_inverse_scan_agrees_with_repeated_place_coefficient() {
        // The §7.3 loop body and a tight loop of place_coefficient
        // calls must produce identical results — they are two
        // equivalent expressions of the same spec text.
        let mut qfs = [0i16; 64];
        for (i, slot) in qfs.iter_mut().enumerate() {
            *slot = (i as i16) - 30;
        }

        for alt in [false, true] {
            let via_loop = apply_inverse_scan(&qfs, alt);

            let mut via_place = [[0i16; 8]; 8];
            for (n, &val) in qfs.iter().enumerate() {
                place_coefficient(&mut via_place, n, val, alt);
            }

            assert_eq!(via_loop, via_place, "scan flag = {alt}");
        }
    }
}
