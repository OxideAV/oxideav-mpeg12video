//! §6.3.11 downloadable-quantiser-matrix threading through the
//! whole-stream decoder: a `quant_matrix_extension()` spliced between
//! a picture's `picture_coding_extension()` and its first slice must
//! change the §7.4.2.3 reconstruction, persist to the following
//! pictures, and be reset by the next `sequence_header_code`.
//!
//! The streams are built from this crate's own intra encoder (which
//! codes against the §6.3.7 default matrices) plus a hand-written
//! extension payload, so every expectation is exact:
//!
//! * an extension downloading the **default** matrix is a no-op —
//!   byte-identical decode;
//! * an extension downloading a **doubled-AC** intra matrix decodes
//!   differently (the encoder quantised against the defaults);
//! * the downloaded matrix **persists** to a second picture in the
//!   same sequence (§6.3.11 "replace the previous values");
//! * a repeat `sequence_header()` **resets** to the defaults
//!   (§6.3.11 "When a sequence_header_code is decoded all matrices
//!   shall be reset to their default values").

use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    decode_video_sequence, encode_intra_picture, mpeg2_inverse_scan_table, DecodedFrame,
    FrameBuffer, IntraPictureParams, DEFAULT_INTRA_QUANT,
};

fn params(width: usize, height: usize) -> IntraPictureParams {
    IntraPictureParams {
        width,
        height,
        chroma_format: ChromaFormat::Yuv420,
        frame_pred_frame_dct: true,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
    }
}

/// A busy frame (diagonal gradient + checker) so intra AC
/// coefficients are plentiful and a changed weighting matrix is
/// guaranteed to move reconstructed samples.
fn busy_frame(width: usize, height: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            let g = 16 + ((x * 3 + y * 5) % 200);
            let c = if (x / 4 + y / 4) % 2 == 0 { 20 } else { 0 };
            f.y.put_sample(x, y, (g + c).min(235) as u8);
        }
    }
    for y in 0..height / 2 {
        for x in 0..width / 2 {
            f.cb.put_sample(x, y, (100 + (x * 7 + y) % 80) as u8);
            f.cr.put_sample(x, y, (90 + (x + y * 3) % 90) as u8);
        }
    }
    f
}

/// Serialise a raster `W[v][u]` matrix into the §7.3.1 default-zigzag
/// byte order `quant_matrix_extension()` transmits.
fn to_zigzag(matrix: &[[u8; 8]; 8]) -> [u8; 64] {
    let inv = mpeg2_inverse_scan_table(false);
    let mut bytes = [0u8; 64];
    for (i, &(v, u)) in inv.iter().enumerate() {
        bytes[i] = matrix[v as usize][u as usize];
    }
    bytes
}

/// Build the byte-exact `quant_matrix_extension()` downloading only
/// the intra matrix: extension_start_code + '0011' id +
/// load_intra=1 + 64 bytes + three '0' load flags = 4 + 520 bits,
/// i.e. exactly 69 byte-aligned bytes.
fn quant_matrix_extension_bytes(intra_zigzag: &[u8; 64]) -> Vec<u8> {
    let mut out = vec![0x00, 0x00, 0x01, 0xB5];
    // '0011' id + load_intra '1' + first 3 payload bits, then the
    // remaining payload shifted by 5 bits, then the three '0' flags.
    let mut bits: Vec<bool> = Vec::with_capacity(520);
    for b in [false, false, true, true, true] {
        bits.push(b); // id '0011' + load_intra_quantiser_matrix '1'
    }
    for &byte in intra_zigzag {
        for k in (0..8).rev() {
            bits.push(byte & (1 << k) != 0);
        }
    }
    bits.extend([false, false, false]); // the three remaining load flags
    assert_eq!(bits.len() % 8, 0, "extension must exit byte-aligned");
    for chunk in bits.chunks(8) {
        let mut byte = 0u8;
        for (k, &bit) in chunk.iter().enumerate() {
            if bit {
                byte |= 1 << (7 - k);
            }
        }
        out.push(byte);
    }
    out
}

/// Find the first slice start code (`00 00 01 01..AF`) at or after
/// `from`.
fn find_first_slice(stream: &[u8], from: usize) -> usize {
    (from..stream.len() - 3)
        .find(|&i| {
            stream[i] == 0
                && stream[i + 1] == 0
                && stream[i + 2] == 1
                && (0x01..=0xAF).contains(&stream[i + 3])
        })
        .expect("stream has a slice")
}

/// Splice `insert` immediately before the first slice of the first
/// picture.
fn splice_before_first_slice(stream: &[u8], insert: &[u8]) -> Vec<u8> {
    let at = find_first_slice(stream, 0);
    let mut out = Vec::with_capacity(stream.len() + insert.len());
    out.extend_from_slice(&stream[..at]);
    out.extend_from_slice(insert);
    out.extend_from_slice(&stream[at..]);
    out
}

/// The doubled-AC intra matrix: DC weight stays 8 (§6.3.11 "The first
/// value shall always be 8"), every AC weight doubled and clamped.
fn doubled_ac_intra() -> [[u8; 8]; 8] {
    let mut m = DEFAULT_INTRA_QUANT;
    for (v, row) in m.iter_mut().enumerate() {
        for (u, w) in row.iter_mut().enumerate() {
            if (v, u) != (0, 0) {
                *w = w.saturating_mul(2);
            }
        }
    }
    m
}

fn luma_bytes(frame: &DecodedFrame) -> Vec<u8> {
    let mut out = Vec::new();
    for y in 0..frame.frame.height {
        for x in 0..frame.frame.width {
            out.push(frame.frame.y.get(x, y).unwrap());
        }
    }
    out
}

#[test]
fn default_matrix_download_is_a_noop() {
    let f = busy_frame(48, 32);
    let stream = encode_intra_picture(&f, params(48, 32), 0, 6).expect("encode");
    let plain = decode_video_sequence(&stream).expect("plain decode");

    let ext = quant_matrix_extension_bytes(&to_zigzag(&DEFAULT_INTRA_QUANT));
    let spliced = splice_before_first_slice(&stream, &ext);
    let with_ext = decode_video_sequence(&spliced).expect("spliced decode");

    assert_eq!(plain.len(), 1);
    assert_eq!(with_ext.len(), 1);
    assert_eq!(
        luma_bytes(&plain[0]),
        luma_bytes(&with_ext[0]),
        "downloading the default matrix must not change the decode"
    );
}

#[test]
fn downloaded_intra_matrix_changes_reconstruction_and_resets_on_sequence_header() {
    let f = busy_frame(48, 32);
    let stream = encode_intra_picture(&f, params(48, 32), 0, 6).expect("encode");
    let plain = decode_video_sequence(&stream).expect("plain decode");
    let plain_luma = luma_bytes(&plain[0]);

    let ext = quant_matrix_extension_bytes(&to_zigzag(&doubled_ac_intra()));
    let spliced = splice_before_first_slice(&stream, &ext);
    let with_ext = decode_video_sequence(&spliced).expect("spliced decode");
    let changed_luma = luma_bytes(&with_ext[0]);
    assert_ne!(
        plain_luma, changed_luma,
        "a doubled-AC intra matrix must change the reconstruction"
    );

    // Persistence (§6.3.11): a second picture in the same sequence,
    // with no further extension, still reconstructs under the
    // downloaded matrix. Build: [seq..pic0+ext][pic1 = pic0 with
    // temporal_reference patched to 1], no repeat sequence header.
    let end_code = spliced.len() - 4; // trailing sequence_end_code
    assert_eq!(&spliced[end_code..], &[0x00, 0x00, 0x01, 0xB7]);
    let mut two_pics = spliced[..end_code].to_vec();
    // The second picture: the (unspliced) original picture region
    // with its temporal_reference patched from 0 to 1 (10-bit field
    // right after the 32-bit start code: byte 4 carries bits 9..2,
    // byte 5's top 2 bits are bits 1..0). No quant_matrix_extension
    // this time.
    let p_start = stream
        .windows(4)
        .position(|w| w == [0x00, 0x00, 0x01, 0x00])
        .expect("picture start");
    let p_end = stream.len() - 4;
    let mut pic1 = stream[p_start..p_end].to_vec();
    pic1[5] |= 0b0100_0000; // temporal_reference 0 -> 1
    two_pics.extend_from_slice(&pic1);
    two_pics.extend_from_slice(&[0x00, 0x00, 0x01, 0xB7]);

    let frames = decode_video_sequence(&two_pics).expect("two-picture decode");
    assert_eq!(frames.len(), 2);
    assert_eq!(
        luma_bytes(&frames[1]),
        changed_luma,
        "the downloaded matrix persists to the next picture (§6.3.11)"
    );

    // Reset: the same second picture behind a repeat sequence
    // header + sequence extension decodes with the defaults again.
    let seq_end = p_start; // the leading sequence_header + extension bytes
    let mut with_reset = spliced[..end_code].to_vec();
    with_reset.extend_from_slice(&stream[..seq_end]); // repeat sequence layer
    with_reset.extend_from_slice(&pic1);
    with_reset.extend_from_slice(&[0x00, 0x00, 0x01, 0xB7]);

    let frames = decode_video_sequence(&with_reset).expect("reset decode");
    assert_eq!(frames.len(), 2);
    assert_eq!(
        luma_bytes(&frames[1]),
        plain_luma,
        "a sequence_header_code resets every matrix to its default (§6.3.11)"
    );
}
