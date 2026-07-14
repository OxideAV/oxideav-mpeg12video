//! MPEG-1 (ISO/IEC 11172-2) §2.4.3.7 / §2.4.4.1–.3 single-block
//! decoder: the bitstream→`f[y][x]` composition of the per-stage
//! MPEG-1 modules, mirroring what [`crate::mpeg2_block_decoder`] does
//! for the ISO/IEC 13818-2 syntax.
//!
//! One coded block of an MPEG-1 macroblock decodes in four stages:
//!
//! 1. **§2.4.3.7 coefficient walk** — for an **intra** block, the
//!    `dct_dc_size_*` prelude ([`crate::block_dc`], Tables B.5a /
//!    B.5b) reconstructs `dct_zz[0]`, then `dct_coeff_next` symbols
//!    ([`crate::dct_coeff`], Tables B.5c/B.5d + the B.5f escape) fill
//!    `dct_zz[i]` at `i += run + 1` until `end_of_block`. For a
//!    **non-intra** block the walk opens with `dct_coeff_first`
//!    (`i = run`) instead of the DC prelude.
//! 2. **§2.4.4.1 / §2.4.4.2 dequantisation**
//!    ([`crate::dequantize`]) — the zig-zag scan, the
//!    `quantizer_scale × quant-matrix` reconstruction, the
//!    odd-value mismatch rule, the `[-2048, 2047]` saturation, and
//!    (intra) the `dct_dc_*_past` predictor chain keyed on
//!    `macroblock_address - past_intra_address`.
//! 3. **Annex A IDCT** ([`crate::idct::idct_8x8_from_i32`]).
//!
//! The output is packaged as the same [`DecodedBlock`] shape the
//! MPEG-2 block decoder emits so the downstream macroblock placement
//! ([`crate::frame_assembly`] / [`crate::inter_reconstruction`])
//! consumes either standard's blocks identically: `qfs` carries the
//! §2.4.3.7 `dct_zz[]` array, `qf` its inverse-scan (raster) image,
//! `f_quant` the §2.4.4 `dct_recon[m][n]`, and `f_pel` the Annex A
//! IDCT output.

use crate::block_dc::{DcCoefficient, DcComponent, SCAN};
use crate::dct_coeff::{CoefficientPosition, DctCoeff, DctCoeffStep};
use crate::dequantize::{
    dequantize_intra_block, dequantize_non_intra_block, IntraBlockKind, IntraDcPredictors,
};
use crate::idct::idct_8x8_from_i32;
use crate::mpeg2_block_decoder::DecodedBlock;
use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// Map an MPEG-1 4:2:0 block index (Figure 6-10 numbering: `0..=3`
/// luma, `4` = Cb, `5` = Cr) to the §2.4.4.1 intra-block kind that
/// selects the DC predictor loop.
pub fn intra_block_kind(block_index: u8) -> Result<IntraBlockKind> {
    Ok(match block_index {
        0 => IntraBlockKind::LuminanceFirst,
        1..=3 => IntraBlockKind::LuminanceSubsequent,
        4 => IntraBlockKind::ChrominanceCb,
        5 => IntraBlockKind::ChrominanceCr,
        _ => {
            return Err(Error::InvalidBitstream(
                "mpeg1 intra block index out of the 4:2:0 range 0..=5 (§2.4.3.7)",
            ))
        }
    })
}

/// Run the §2.4.3.7 `dct_coeff_next` loop from the cursor, filling
/// `dct_zz` starting at running index `i`, until `end_of_block` is
/// consumed.
fn walk_next_coefficients(
    br: &mut BitReader<'_>,
    dct_zz: &mut [i32; 64],
    mut i: usize,
) -> Result<()> {
    loop {
        match DctCoeffStep::parse(br, CoefficientPosition::Next)?.symbol {
            DctCoeff::EndOfBlock => return Ok(()),
            DctCoeff::RunLevel {
                run, signed_level, ..
            } => {
                i = i.checked_add(run as usize + 1).filter(|&n| n < 64).ok_or(
                    Error::InvalidBitstream(
                        "dct_coeff_next run overflows the 64-coefficient block (§2.4.3.7)",
                    ),
                )?;
                dct_zz[i] = i32::from(signed_level);
            }
        }
    }
}

/// Inverse-scan a zig-zag `dct_zz[]` array to the raster `QF[v][u]`
/// image (trace field only — the dequantiser applies the scan itself).
fn inverse_scan(dct_zz: &[i32; 64]) -> [[i32; 8]; 8] {
    let mut qf = [[0i32; 8]; 8];
    for (m, row) in qf.iter_mut().enumerate() {
        for (n, cell) in row.iter_mut().enumerate() {
            *cell = dct_zz[SCAN[m][n] as usize];
        }
    }
    qf
}

/// Decode one **intra** MPEG-1 block: §2.4.3.7 DC prelude + AC walk,
/// §2.4.4.1 dequantisation against `intra_quant` with the
/// `dct_dc_*_past` predictor chain, Annex A IDCT.
///
/// `block_index` follows the 4:2:0 Figure 6-10 numbering and selects
/// both the DC VLC table (luminance vs chrominance) and the
/// [`IntraBlockKind`] predictor loop. The caller invokes
/// [`crate::dequantize::finalise_intra_macroblock`] once after all six
/// blocks of the macroblock have decoded.
///
/// # Errors
/// Propagates the §2.4.3.7 VLC walker and §2.4.4.1 dequantiser errors.
pub fn decode_intra_block(
    br: &mut BitReader<'_>,
    block_index: u8,
    quantizer_scale: u8,
    intra_quant: &[[u8; 8]; 8],
    predictors: &mut IntraDcPredictors,
    macroblock_address: i32,
) -> Result<DecodedBlock> {
    let kind = intra_block_kind(block_index)?;
    let dc_component = if block_index < 4 {
        DcComponent::Luminance
    } else {
        DcComponent::Chrominance
    };

    let mut dct_zz = [0i32; 64];
    let dc = DcCoefficient::parse(br, dc_component)?;
    dct_zz[0] = dc.dct_zz_0;
    walk_next_coefficients(br, &mut dct_zz, 0)?;

    let dct_recon = dequantize_intra_block(
        &dct_zz,
        quantizer_scale,
        intra_quant,
        kind,
        predictors,
        macroblock_address,
    )?;
    let f_pel = idct_8x8_from_i32(&dct_recon);

    Ok(DecodedBlock {
        qfs: dct_zz,
        qf: inverse_scan(&dct_zz),
        f_quant: dct_recon,
        f_pel,
        end_of_block_bit_position: br.bit_position(),
    })
}

/// Decode one block of a **D-picture** macroblock (ISO/IEC 11172-2
/// `picture_coding_type == 4`): per §2.4.2.8 the block layer of a
/// dc intra-coded picture carries **only** the `dct_dc_size_*` +
/// `dct_dc_differential` prelude — the `dct_coeff_next` loop and the
/// `end_of_block` code are gated by `if (picture_coding_type != 4)`,
/// so neither is present in the bitstream.
///
/// The reconstruction is otherwise the §2.4.4.1 intra path over a
/// `dct_zz[]` whose 63 AC entries are zero: the `dct_dc_*_past`
/// predictor chain applies unchanged, and the Annex A IDCT of the
/// DC-only block yields a flat 8×8 plane.
///
/// # Errors
/// Propagates the §2.4.3.7 DC-prelude and §2.4.4.1 dequantiser errors.
pub fn decode_d_block(
    br: &mut BitReader<'_>,
    block_index: u8,
    quantizer_scale: u8,
    intra_quant: &[[u8; 8]; 8],
    predictors: &mut IntraDcPredictors,
    macroblock_address: i32,
) -> Result<DecodedBlock> {
    let kind = intra_block_kind(block_index)?;
    let dc_component = if block_index < 4 {
        DcComponent::Luminance
    } else {
        DcComponent::Chrominance
    };

    let mut dct_zz = [0i32; 64];
    let dc = DcCoefficient::parse(br, dc_component)?;
    dct_zz[0] = dc.dct_zz_0;
    // §2.4.2.8: no dct_coeff_next walk, no end_of_block in D-pictures.

    let dct_recon = dequantize_intra_block(
        &dct_zz,
        quantizer_scale,
        intra_quant,
        kind,
        predictors,
        macroblock_address,
    )?;
    let f_pel = idct_8x8_from_i32(&dct_recon);

    Ok(DecodedBlock {
        qfs: dct_zz,
        qf: inverse_scan(&dct_zz),
        f_quant: dct_recon,
        f_pel,
        end_of_block_bit_position: br.bit_position(),
    })
}

/// Decode one **non-intra** MPEG-1 block: the §2.4.3.7
/// `dct_coeff_first` + `dct_coeff_next` walk and the §2.4.4.2
/// dequantisation against `non_intra_quant`, then the Annex A IDCT.
///
/// # Errors
/// Propagates the §2.4.3.7 VLC walker and §2.4.4.2 dequantiser errors.
pub fn decode_non_intra_block(
    br: &mut BitReader<'_>,
    quantizer_scale: u8,
    non_intra_quant: &[[u8; 8]; 8],
) -> Result<DecodedBlock> {
    let mut dct_zz = [0i32; 64];
    // §2.4.3.7: the first coefficient of a coded non-intra block is
    // `dct_coeff_first` — `i = run`, never end_of_block.
    match DctCoeffStep::parse(br, CoefficientPosition::First)?.symbol {
        DctCoeff::EndOfBlock => {
            return Err(Error::InvalidBitstream(
                "dct_coeff_first cannot be end_of_block (§2.4.3.7)",
            ))
        }
        DctCoeff::RunLevel {
            run, signed_level, ..
        } => {
            let i = run as usize;
            if i >= 64 {
                return Err(Error::InvalidBitstream(
                    "dct_coeff_first run outside the 64-coefficient block (§2.4.3.7)",
                ));
            }
            dct_zz[i] = i32::from(signed_level);
            walk_next_coefficients(br, &mut dct_zz, i)?;
        }
    }

    let dct_recon = dequantize_non_intra_block(&dct_zz, quantizer_scale, non_intra_quant)?;
    let f_pel = idct_8x8_from_i32(&dct_recon);

    Ok(DecodedBlock {
        qfs: dct_zz,
        qf: inverse_scan(&dct_zz),
        f_quant: dct_recon,
        f_pel,
        end_of_block_bit_position: br.bit_position(),
    })
}

#[cfg(test)]
mod tests {
    //! Composition checks: the block decoder must agree with the
    //! per-stage modules it chains (which carry the spec-pinned
    //! coverage themselves).

    use super::*;
    use crate::dequantize::DEFAULT_INTRA_QUANT;
    use oxideav_core::bits::BitWriter;

    /// Default non-intra matrix: all 16s per §2.4.2.3.
    const FLAT_16: [[u8; 8]; 8] = [[16u8; 8]; 8];

    #[test]
    fn intra_dc_only_block_reconstructs_flat_plane() {
        // Luminance DC prelude: dct_dc_size_luminance = 3 ('101'),
        // differential '111' -> dct_zz[0] = 7; then EOB ('10').
        let mut bw = BitWriter::new();
        bw.write_u32(0b101, 3);
        bw.write_u32(0b111, 3);
        bw.write_u32(0b10, 2);
        bw.align_to_byte();
        let buf = bw.into_bytes();
        let mut br = BitReader::new(&buf);
        let mut pred = IntraDcPredictors::at_slice_start();

        let out = decode_intra_block(&mut br, 0, 8, &DEFAULT_INTRA_QUANT, &mut pred, 0).unwrap();
        // dct_recon[0][0] = 7*8 + 128*8 (slice-start reset) = 1080.
        assert_eq!(out.f_quant[0][0], 1080);
        // DC-only IDCT is flat: f = round(1080 / 8) = 135.
        assert!(out.f_pel.iter().all(|row| row.iter().all(|&v| v == 135)));
        assert_eq!(pred.y_past, 1080);
    }

    #[test]
    fn non_intra_first_coefficient_walk() {
        // dct_coeff_first: run 0, level 1 -> code '1' + sign 0.
        // Then EOB.
        let mut bw = BitWriter::new();
        bw.write_u32(0b1, 1); // run 0 / level 1 (first form)
        bw.write_bit(false); // sign +
        bw.write_u32(0b10, 2); // end_of_block
        bw.align_to_byte();
        let buf = bw.into_bytes();
        let mut br = BitReader::new(&buf);

        let out = decode_non_intra_block(&mut br, 2, &FLAT_16).unwrap();
        // §2.4.4.2: ((2*1 + 1) * 2 * 16)/16 = 6 -> even? 6 is even ->
        // 6 - 1 = 5.
        assert_eq!(out.qfs[0], 1);
        assert_eq!(out.f_quant[0][0], 5);
    }

    #[test]
    fn intra_block_kind_mapping() {
        assert_eq!(intra_block_kind(0).unwrap(), IntraBlockKind::LuminanceFirst);
        for i in 1..=3 {
            assert_eq!(
                intra_block_kind(i).unwrap(),
                IntraBlockKind::LuminanceSubsequent
            );
        }
        assert_eq!(intra_block_kind(4).unwrap(), IntraBlockKind::ChrominanceCb);
        assert_eq!(intra_block_kind(5).unwrap(), IntraBlockKind::ChrominanceCr);
        assert!(intra_block_kind(6).is_err());
    }
}
