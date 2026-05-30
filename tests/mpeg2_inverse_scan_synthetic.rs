//! Cross-module spec checks for the MPEG-2 §7.3 inverse-scan
//! tables — exercises the public surface
//! ([`oxideav_mpeg12video::mpeg2_inverse_scan`] + the
//! `MPEG2_…` re-exports at the crate root) the way a downstream
//! slice-decoder driver would, and pins the cell-for-cell match
//! against the spec's printed Figure 7-2 and Figure 7-3 matrices
//! one more time at the integration boundary.

// Same reasoning as in `src/mpeg2_inverse_scan.rs`: §7.3 is
// written `for (v) for (u) …` and the tests pin spec-named
// cells via those exact indices. Rewriting as
// `iter().enumerate()` would obscure the correspondence.
#![allow(clippy::needless_range_loop)]

use oxideav_mpeg12video::{
    mpeg2_apply_inverse_scan, mpeg2_inverse_scan_table, mpeg2_place_coefficient, mpeg2_scan_table,
    MPEG2_ALTERNATE_INVERSE_SCAN, MPEG2_ALTERNATE_SCAN, MPEG2_ZIGZAG_INVERSE_SCAN,
    SCAN as MPEG1_SCAN,
};

/// Figure 7-2 verbatim from ISO/IEC 13818-2:1995 page 80
/// (scan[0][v][u]). Re-encoded here so a single edit in the
/// library can't quietly drift the table — the integration test
/// confronts the library against the spec one more time.
const FIGURE_7_2: [[u8; 8]; 8] = [
    [0, 1, 5, 6, 14, 15, 27, 28],
    [2, 4, 7, 13, 16, 26, 29, 42],
    [3, 8, 12, 17, 25, 30, 41, 43],
    [9, 11, 18, 24, 31, 40, 44, 53],
    [10, 19, 23, 32, 39, 45, 52, 54],
    [20, 22, 33, 38, 46, 51, 55, 60],
    [21, 34, 37, 47, 50, 56, 59, 61],
    [35, 36, 48, 49, 57, 58, 62, 63],
];

/// Figure 7-3 verbatim from ISO/IEC 13818-2:1995 page 80
/// (scan[1][v][u]).
const FIGURE_7_3: [[u8; 8]; 8] = [
    [0, 4, 6, 20, 22, 36, 38, 52],
    [1, 5, 7, 21, 23, 37, 39, 53],
    [2, 8, 19, 24, 34, 40, 50, 54],
    [3, 9, 18, 25, 35, 41, 51, 55],
    [10, 17, 26, 30, 42, 46, 56, 60],
    [11, 16, 27, 31, 43, 47, 57, 61],
    [12, 15, 28, 32, 44, 48, 58, 62],
    [13, 14, 29, 33, 45, 49, 59, 63],
];

#[test]
fn library_figure_7_2_matches_spec_page_80_cell_for_cell() {
    for v in 0..8 {
        for u in 0..8 {
            assert_eq!(
                mpeg2_scan_table(false)[v][u],
                FIGURE_7_2[v][u],
                "scan[0] mismatch at v={v} u={u}",
            );
        }
    }
}

#[test]
fn library_figure_7_3_matches_spec_page_80_cell_for_cell() {
    for v in 0..8 {
        for u in 0..8 {
            assert_eq!(
                mpeg2_scan_table(true)[v][u],
                FIGURE_7_3[v][u],
                "scan[1] mismatch at v={v} u={u}",
            );
        }
    }
    // And the re-exported constant agrees with the function
    // return — both are the same datum.
    for v in 0..8 {
        for u in 0..8 {
            assert_eq!(MPEG2_ALTERNATE_SCAN[v][u], FIGURE_7_3[v][u]);
        }
    }
}

#[test]
fn mpeg1_and_mpeg2_zigzag_scan_are_the_same_matrix() {
    // ISO/IEC 11172-2 §2.4.4.1 (MPEG-1) and ISO/IEC 13818-2
    // §7.3 Figure 7-2 (MPEG-2 scan[0]) print the same numbers;
    // re-exported MPEG1_SCAN is the live source of truth.
    for v in 0..8 {
        for u in 0..8 {
            assert_eq!(
                MPEG1_SCAN[v][u],
                mpeg2_scan_table(false)[v][u],
                "MPEG-1 §2.4.4.1 and MPEG-2 §7.3 Figure 7-2 disagree at v={v} u={u}",
            );
        }
    }
}

#[test]
fn inverse_scan_table_round_trips_for_all_64_cells_in_both_scans() {
    for alt in [false, true] {
        let scan = mpeg2_scan_table(alt);
        let inv = mpeg2_inverse_scan_table(alt);
        for v in 0..8usize {
            for u in 0..8usize {
                let n = scan[v][u] as usize;
                let (vp, up) = inv[n];
                assert_eq!(
                    (vp as usize, up as usize),
                    (v, u),
                    "alt={alt} v={v} u={u} n={n}",
                );
            }
        }
        for n in 0..64usize {
            let (v, u) = inv[n];
            assert_eq!(scan[v as usize][u as usize] as usize, n);
        }
    }
}

#[test]
fn inverse_scan_constants_match_their_function_partners() {
    for n in 0..64 {
        assert_eq!(
            MPEG2_ZIGZAG_INVERSE_SCAN[n],
            mpeg2_inverse_scan_table(false)[n],
            "zig-zag inverse-scan const/fn divergence at n={n}",
        );
        assert_eq!(
            MPEG2_ALTERNATE_INVERSE_SCAN[n],
            mpeg2_inverse_scan_table(true)[n],
            "alternate inverse-scan const/fn divergence at n={n}",
        );
    }
}

#[test]
fn place_coefficient_synthetic_walker_alternate_scan() {
    // Replay a plausible §7.2.2 walker output stream against
    // the alternate scan and confirm the resulting QF block is
    // exactly what the §7.3 inverse-scan loop would build for
    // the same flattened coefficient list.
    //
    // Symbol stream (intra block in NEXT mode, post-DC):
    //   (run, signed_level) pairs followed by EoB
    //     (0, +12)  → n = 1
    //     (2, -3)   → n = 4
    //     (1, +7)   → n = 6
    //     (5, +1)   → n = 12
    //     (10, -2)  → n = 23
    let walker_emits: &[(usize, i16)] = &[(0, 12), (2, -3), (1, 7), (5, 1), (10, -2)];

    // Build the §7.3 expected output two independent ways and
    // confirm equality at every step.

    // (a) The per-coefficient placement path used by an
    //     interleaved §7.2.2 walker.
    let mut via_place = [[0i16; 8]; 8];
    let mut n_cursor: usize = 0; // After the DC prelude, the residual walker starts at n=1.
    for (run, level) in walker_emits {
        n_cursor += 1 + *run;
        mpeg2_place_coefficient(&mut via_place, n_cursor, *level, true);
    }

    // (b) The full §7.3 loop body on a pre-flattened QFS list.
    let mut qfs = [0i16; 64];
    let mut n_cursor: usize = 0;
    for (run, level) in walker_emits {
        n_cursor += 1 + *run;
        qfs[n_cursor] = *level;
    }
    let via_loop = mpeg2_apply_inverse_scan(&qfs, true);

    assert_eq!(via_place, via_loop);

    // And one explicit cell check anchored in the spec table:
    // (5, +1) at n = 12 under Figure 7-3 lands at
    // scan[1]^{-1}[12] = (v, u) = (6, 0).
    assert_eq!(via_loop[6][0], 1, "expected the (5, +1) sample at (6, 0)");
    // (0, +12) at n = 1 under Figure 7-3 lands at (1, 0).
    assert_eq!(via_loop[1][0], 12, "expected the (0, +12) sample at (1, 0)",);
}

#[test]
fn place_coefficient_synthetic_walker_zigzag_scan() {
    // Same shape as the alternate-scan test but with
    // alternate_scan = 0 — confirms the dispatch routes through
    // Figure 7-2 in the false branch.
    let walker_emits: &[(usize, i16)] = &[(0, 9), (1, -4), (3, 2)];

    let mut via_place = [[0i16; 8]; 8];
    let mut n_cursor: usize = 0;
    for (run, level) in walker_emits {
        n_cursor += 1 + *run;
        mpeg2_place_coefficient(&mut via_place, n_cursor, *level, false);
    }

    let mut qfs = [0i16; 64];
    let mut n_cursor: usize = 0;
    for (run, level) in walker_emits {
        n_cursor += 1 + *run;
        qfs[n_cursor] = *level;
    }
    let via_loop = mpeg2_apply_inverse_scan(&qfs, false);

    assert_eq!(via_place, via_loop);

    // (0, +9) at n = 1 under Figure 7-2 lands at (0, 1).
    assert_eq!(via_loop[0][1], 9);
    // (1, -4) at n = 3 under Figure 7-2 lands at (2, 0) per
    // FIGURE_7_2 row 2 col 0 == 3.
    assert_eq!(via_loop[2][0], -4);
    // (3, +2) at n = 7 under Figure 7-2 lands at (1, 2) per
    // FIGURE_7_2 row 1 col 2 == 7.
    assert_eq!(via_loop[1][2], 2);
}
