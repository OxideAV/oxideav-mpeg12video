//! §7.8.3.4 SNR-scalability addition of coefficients from two layers.
//!
//! In SNR scalability the enhancement layer carries only *refinement*
//! DCT coefficients for the coincident lower-layer block. §7.8.3.4
//! specifies how the two layers' inverse-quantised coefficient blocks
//! (`F''lower` and `F''enhance`) combine into the single block `F''`
//! that §7.8.3.5 then feeds into the remaining macroblock decode (§7.4.3
//! saturation onward):
//!
//! ```text
//!   F''[v][u] = F''lower[v][u] + F''enhance[v][u]   for all u, v
//! ```
//!
//! When `chroma_simulcast == 1` (a subset of SNR scalability where the
//! enhancement layer refines only the DC of chrominance, §3.18) the
//! chrominance blocks instead use the lower-layer **DC** as a prediction
//! and take their AC entirely from the enhancement layer:
//!
//! ```text
//!   F''[0][0]  = F''lower[0][0] + F''enhance[0][0]
//!   F''[v][u]  = F''enhance[v][u]            for all (u, v) != (0, 0)
//! ```
//!
//! The lower-layer DC used for a given enhancement chrominance block is
//! the **coincident** lower-layer chrominance block selected by
//! Table 7-27 (the lower and enhancement layers may carry different
//! chrominance formats, so the block index used as the DC predictor is
//! not always the same index). Luminance blocks always use the plain
//! `F''lower + F''enhance` sum.
//!
//! §7.8.2.2: an enhancement-layer-skipped macroblock has
//! `F''enhance == 0` (all zeros); a lower-layer-skipped (but
//! enhancement-coded) macroblock has `F''lower == 0`. Both are handled
//! by passing the appropriate all-zero block — no special path is
//! needed.
//!
//! Spec citations refer to **ISO/IEC 13818-2 (H.262) §7.8.3.4** and
//! Table 7-27.

use crate::sequence_extension::ChromaFormat;
use crate::{Error, Result};

/// One inverse-quantised 8×8 coefficient block, row-major `[v][u]`
/// (`v` = row, `u` = column), as produced by the §7.4.2 inverse
/// quantiser. Index `0` is the DC coefficient `F''[0][0]`.
pub type CoeffBlock = [i32; 64];

/// §7.8.3.4 luminance / non-simulcast block combination:
/// `F''[v][u] = F''lower[v][u] + F''enhance[v][u]` for every coefficient.
///
/// This is the combination for every luminance block, and for every
/// block (luma and chroma) when `chroma_simulcast == 0`.
pub fn add_layer_block(lower: &CoeffBlock, enhance: &CoeffBlock) -> CoeffBlock {
    let mut out = [0i32; 64];
    for (o, (&l, &e)) in out.iter_mut().zip(lower.iter().zip(enhance.iter())) {
        *o = l + e;
    }
    out
}

/// §7.8.3.4 chroma-simulcast chrominance block combination: the
/// enhancement DC is added to the lower-layer DC of the **coincident**
/// chrominance block (Table 7-27), while the AC comes entirely from the
/// enhancement layer (the lower-layer AC is discarded, §7.8.3.4):
///
/// ```text
///   F''[0][0] = lower_dc_predictor + F''enhance[0][0]
///   F''[v][u] = F''enhance[v][u]            for (u, v) != (0, 0)
/// ```
///
/// `lower_dc_predictor` is `F''lower[0][0]` of the lower-layer block the
/// Table 7-27 mapping selects (use [`simulcast_dc_predictor_block`] to
/// resolve which lower-layer block index that is, then take its DC).
pub fn add_layer_chroma_simulcast(enhance: &CoeffBlock, lower_dc_predictor: i32) -> CoeffBlock {
    let mut out = *enhance;
    out[0] = lower_dc_predictor + enhance[0];
    out
}

/// Table 7-27: the lower-layer chrominance block index whose DC
/// coefficient predicts the DC of the coincident enhancement-layer
/// chrominance block `enhance_block_index`, for a given `(base, upper)`
/// chrominance-format pair (used only when `chroma_simulcast == 1`).
///
/// `enhance_block_index` is a chrominance block index in the §6.1.1.8
/// numbering: `4..=5` for 4:2:0, `4..=7` for 4:2:2, `4..=11` for 4:4:4.
///
/// The three rows of Table 7-27 cover the three SNR-simulcast upsampling
/// pairs where the chroma resolution increases:
///
/// | base  | upper | block 4 5 6 7 8 9 10 11 → predictor |
/// |-------|-------|-------------------------------------|
/// | 4:2:0 | 4:2:2 | 4 5 4 5                             |
/// | 4:2:0 | 4:4:4 | 4 5 4 5 4 5 4 5                     |
/// | 4:2:2 | 4:4:4 | 4 5 6 7 4 5 6 7                     |
///
/// When both layers share a chroma format the predictor is the same
/// block index (`enhance_block_index`), since the blocks are coincident
/// one-to-one.
///
/// # Errors
/// * [`Error::InvalidBitstream`] if `enhance_block_index` is outside the
///   valid chrominance range for `upper`, or the `(base, upper)` pair is
///   not an allowed SNR-simulcast format pair.
pub fn simulcast_dc_predictor_block(
    base: ChromaFormat,
    upper: ChromaFormat,
    enhance_block_index: usize,
) -> Result<usize> {
    use ChromaFormat::{Yuv420, Yuv422, Yuv444};

    // Valid chrominance block range for the upper (enhancement) format.
    let max_idx = match upper {
        Yuv420 => 5,
        Yuv422 => 7,
        Yuv444 => 11,
    };
    if !(4..=max_idx).contains(&enhance_block_index) {
        return Err(Error::InvalidBitstream(
            "snr simulcast: enhancement chrominance block index outside the format's range \
             (§6.1.1.8 / Table 7-27)",
        ));
    }

    Ok(match (base, upper) {
        // Same format: blocks are coincident one-to-one.
        (Yuv420, Yuv420) | (Yuv422, Yuv422) | (Yuv444, Yuv444) => enhance_block_index,
        // base 4:2:0 / upper 4:2:2 — blocks 4 5 6 7 → 4 5 4 5.
        (Yuv420, Yuv422) => {
            // Cb blocks {4, 6} → 4; Cr blocks {5, 7} → 5.
            if enhance_block_index % 2 == 0 {
                4
            } else {
                5
            }
        }
        // base 4:2:0 / upper 4:4:4 — blocks 4..11 → 4 5 4 5 4 5 4 5.
        (Yuv420, Yuv444) => {
            if enhance_block_index % 2 == 0 {
                4
            } else {
                5
            }
        }
        // base 4:2:2 / upper 4:4:4 — blocks 4 5 6 7 8 9 10 11 → 4 5 6 7 4 5 6 7.
        (Yuv422, Yuv444) => 4 + ((enhance_block_index - 4) % 4),
        // The remaining pairs decrease chroma resolution, which SNR
        // simulcast does not define (§7.8.3.4 / Table 7-27).
        _ => {
            return Err(Error::InvalidBitstream(
                "snr simulcast: (base, upper) chroma_format pair not in Table 7-27",
            ))
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ChromaFormat::{Yuv420, Yuv422, Yuv444};

    fn ramp(offset: i32) -> CoeffBlock {
        let mut b = [0i32; 64];
        for (i, v) in b.iter_mut().enumerate() {
            *v = offset + i as i32;
        }
        b
    }

    #[test]
    fn add_layer_block_sums_every_coefficient() {
        let lower = ramp(0);
        let enhance = ramp(100);
        let out = add_layer_block(&lower, &enhance);
        for i in 0..64 {
            assert_eq!(out[i], lower[i] + enhance[i]);
        }
        assert_eq!(out[0], 100);
        assert_eq!(out[63], 63 + 163);
    }

    #[test]
    fn add_layer_block_with_zero_lower_is_identity() {
        // §7.8.2.2 lower-layer-skipped case: F''lower == 0.
        let enhance = ramp(7);
        let out = add_layer_block(&[0i32; 64], &enhance);
        assert_eq!(out, enhance);
    }

    #[test]
    fn add_layer_block_with_zero_enhance_is_lower() {
        // §7.8.2.2 enhancement-skipped case: F''enhance == 0.
        let lower = ramp(7);
        let out = add_layer_block(&lower, &[0i32; 64]);
        assert_eq!(out, lower);
    }

    #[test]
    fn chroma_simulcast_predicts_dc_and_takes_ac_from_enhance() {
        let enhance = ramp(50); // DC=50, AC ramps
        let lower_dc = 9;
        let out = add_layer_chroma_simulcast(&enhance, lower_dc);
        // DC is the sum; AC unchanged from enhance.
        assert_eq!(out[0], 9 + 50);
        for i in 1..64 {
            assert_eq!(out[i], enhance[i]);
        }
    }

    #[test]
    fn simulcast_predictor_same_format_is_identity() {
        for (fmt, max) in [(Yuv420, 5usize), (Yuv422, 7), (Yuv444, 11)] {
            for idx in 4..=max {
                assert_eq!(simulcast_dc_predictor_block(fmt, fmt, idx).unwrap(), idx);
            }
        }
    }

    #[test]
    fn simulcast_predictor_420_to_422_table_7_27() {
        // blocks 4 5 6 7 → 4 5 4 5.
        let got: Vec<usize> = (4..=7)
            .map(|i| simulcast_dc_predictor_block(Yuv420, Yuv422, i).unwrap())
            .collect();
        assert_eq!(got, vec![4, 5, 4, 5]);
    }

    #[test]
    fn simulcast_predictor_420_to_444_table_7_27() {
        // blocks 4..11 → 4 5 4 5 4 5 4 5.
        let got: Vec<usize> = (4..=11)
            .map(|i| simulcast_dc_predictor_block(Yuv420, Yuv444, i).unwrap())
            .collect();
        assert_eq!(got, vec![4, 5, 4, 5, 4, 5, 4, 5]);
    }

    #[test]
    fn simulcast_predictor_422_to_444_table_7_27() {
        // blocks 4..11 → 4 5 6 7 4 5 6 7.
        let got: Vec<usize> = (4..=11)
            .map(|i| simulcast_dc_predictor_block(Yuv422, Yuv444, i).unwrap())
            .collect();
        assert_eq!(got, vec![4, 5, 6, 7, 4, 5, 6, 7]);
    }

    #[test]
    fn simulcast_predictor_rejects_out_of_range_index() {
        // 4:2:0 has only blocks 4, 5.
        assert!(matches!(
            simulcast_dc_predictor_block(Yuv420, Yuv420, 6),
            Err(Error::InvalidBitstream(_))
        ));
        // index below the chrominance range.
        assert!(matches!(
            simulcast_dc_predictor_block(Yuv444, Yuv444, 3),
            Err(Error::InvalidBitstream(_))
        ));
    }

    #[test]
    fn simulcast_predictor_rejects_resolution_decreasing_pair() {
        // upper lower-resolution than base is not a Table 7-27 row.
        assert!(matches!(
            simulcast_dc_predictor_block(Yuv444, Yuv420, 4),
            Err(Error::InvalidBitstream(_))
        ));
        assert!(matches!(
            simulcast_dc_predictor_block(Yuv422, Yuv420, 4),
            Err(Error::InvalidBitstream(_))
        ));
    }
}
