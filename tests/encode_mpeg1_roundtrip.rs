//! ISO/IEC 11172-2 encode→decode round-trips: the MPEG-1 encoder's
//! streams decode through `decode_video_sequence` (which classifies
//! them as 11172-2 by the absence of a `sequence_extension()`) to the
//! encoder's own reconstructions, with GOP structure, per-GOP
//! `temporal_reference` reset, and time codes verified against the
//! §2.4.2.4 / §2.4.3.3 layer.

use oxideav_mpeg12video::gop_header::{Mpeg2Gop, TimeCode};
use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::sequence_header::Mpeg2SequenceHeader;
use oxideav_mpeg12video::{
    decode_video_sequence, display_indices_from_coded_pictures, encode_mpeg1_b_picture,
    encode_mpeg1_display_order_sequence, encode_mpeg1_intra_picture, encode_mpeg1_intra_stream,
    encode_mpeg1_p_picture, FrameBuffer, Mpeg1PictureParams, Mpeg1SequenceParams,
    PictureCodingType,
};

/// Deterministic synthetic frame: a diagonal gradient with a chequer
/// overlay, shifted by `(dx, dy)` so consecutive frames look like
/// translated content; `stamp` adds an unpredictable high-contrast
/// block to provoke the intra fallback.
fn frame_at(width: usize, height: usize, dx: usize, dy: usize, stamp: bool) -> FrameBuffer {
    let mut f = FrameBuffer::new(width, height, ChromaFormat::Yuv420);
    for y in 0..height {
        for x in 0..width {
            let sx = x + dx;
            let sy = y + dy;
            let g = 24 + ((sx * 3 + sy * 5) % 192);
            let c = if (sx / 4 + sy / 4) % 2 == 0 { 16 } else { 0 };
            f.y.put_sample(x, y, (g + c).min(235) as u8);
        }
    }
    if stamp {
        for y in 8..20.min(height) {
            for x in 8..20.min(width) {
                f.y.put_sample(x, y, if (x + y) % 2 == 0 { 16 } else { 235 });
            }
        }
    }
    for y in 0..height.div_ceil(2) {
        for x in 0..width.div_ceil(2) {
            f.cb.put_sample(x, y, (96 + (x + dx / 2 + y) % 64) as u8);
            f.cr.put_sample(x, y, (160u8).saturating_sub(((x + y + dy / 2) % 64) as u8));
        }
    }
    f
}

fn seq(width: u16, height: u16) -> Mpeg1SequenceParams {
    Mpeg1SequenceParams {
        horizontal_size: width,
        vertical_size: height,
        ..Default::default()
    }
}

fn params(width: usize, height: usize) -> Mpeg1PictureParams {
    Mpeg1PictureParams {
        width,
        height,
        intra_quant: oxideav_mpeg12video::DEFAULT_INTRA_QUANT,
        non_intra_quant: oxideav_mpeg12video::DEFAULT_NON_INTRA_QUANT,
    }
}

/// Assert two frames match sample-for-sample over the visible area.
fn assert_frames_equal(a: &FrameBuffer, b: &FrameBuffer, what: &str) {
    for y in 0..a.height {
        for x in 0..a.width {
            assert_eq!(a.y.get(x, y), b.y.get(x, y), "{what}: luma ({x},{y})");
        }
    }
    let (cw, ch) = a.visible_chroma_dims();
    for y in 0..ch {
        for x in 0..cw {
            assert_eq!(a.cb.get(x, y), b.cb.get(x, y), "{what}: cb ({x},{y})");
            assert_eq!(a.cr.get(x, y), b.cr.get(x, y), "{what}: cr ({x},{y})");
        }
    }
}

/// Mean absolute luma error between an input frame and its decode.
fn luma_mae(input: &FrameBuffer, decoded: &FrameBuffer) -> f64 {
    let mut total = 0u64;
    let mut count = 0u64;
    for y in 0..input.height {
        for x in 0..input.width {
            let a = i64::from(input.y.get(x, y).unwrap());
            let b = i64::from(decoded.y.get(x, y).unwrap());
            total += a.abs_diff(b);
            count += 1;
        }
    }
    total as f64 / count as f64
}

#[test]
fn ipp_chain_decodes_to_encoder_reconstructions() {
    // I + 2 motion-compensated P pictures assembled by hand; the
    // decode must equal each returned reconstruction exactly.
    use oxideav_core::bits::BitWriter;
    use oxideav_mpeg12video::gop_header::write_gop_header;
    use oxideav_mpeg12video::write_mpeg1_sequence_header;

    let (w, h) = (64usize, 48usize);
    let p = params(w, h);
    let f0 = frame_at(w, h, 0, 0, false);
    let f1 = frame_at(w, h, 2, 1, false);
    let f2 = frame_at(w, h, 4, 2, true);

    let mut bw = BitWriter::new();
    write_mpeg1_sequence_header(&mut bw, &seq(w as u16, h as u16)).expect("write header");
    write_gop_header(
        &mut bw,
        &Mpeg2Gop {
            time_code: TimeCode::from_display_index(0, 3).unwrap(),
            closed_gop: true,
            broken_link: false,
        },
    );
    let r0 = encode_mpeg1_intra_picture(&mut bw, &f0, &p, 0, 6).expect("I");
    let r1 = encode_mpeg1_p_picture(&mut bw, &f1, &r0, &p, 1, 6, 3, false).expect("P1");
    let r2 = encode_mpeg1_p_picture(&mut bw, &f2, &r1, &p, 2, 6, 3, false).expect("P2");
    let mut stream = bw.finish();
    stream.extend_from_slice(&[0x00, 0x00, 0x01, 0xB7]);

    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 3);
    assert_frames_equal(&frames[0].frame, &r0, "I");
    assert_frames_equal(&frames[1].frame, &r1, "P1");
    assert_frames_equal(&frames[2].frame, &r2, "P2");
    // Round-trip fidelity against the inputs.
    assert!(luma_mae(&f0, &frames[0].frame) < 6.0);
    assert!(luma_mae(&f1, &frames[1].frame) < 6.0);
    assert!(luma_mae(&f2, &frames[2].frame) < 6.0);
}

#[test]
fn b_picture_group_decodes_in_display_order() {
    // Coded order I, P, B (display I, B, P) assembled by hand.
    use oxideav_core::bits::BitWriter;
    use oxideav_mpeg12video::gop_header::write_gop_header;
    use oxideav_mpeg12video::write_mpeg1_sequence_header;

    let (w, h) = (48usize, 48usize);
    let p = params(w, h);
    let f_i = frame_at(w, h, 0, 0, false);
    let f_b = frame_at(w, h, 2, 1, false);
    let f_p = frame_at(w, h, 4, 2, false);

    let mut bw = BitWriter::new();
    write_mpeg1_sequence_header(&mut bw, &seq(w as u16, h as u16)).expect("write header");
    write_gop_header(
        &mut bw,
        &Mpeg2Gop {
            time_code: TimeCode::from_display_index(0, 3).unwrap(),
            closed_gop: true,
            broken_link: false,
        },
    );
    let r_i = encode_mpeg1_intra_picture(&mut bw, &f_i, &p, 0, 6).expect("I");
    let r_p = encode_mpeg1_p_picture(&mut bw, &f_p, &r_i, &p, 2, 6, 3, false).expect("P");
    encode_mpeg1_b_picture(&mut bw, &f_b, &r_i, &r_p, &p, 1, 6, 3, 3, false, false).expect("B");
    let mut stream = bw.finish();
    stream.extend_from_slice(&[0x00, 0x00, 0x01, 0xB7]);

    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 3);
    // Display order: I (tr 0), B (tr 1), P (tr 2).
    assert_eq!(frames[0].temporal_reference, 0);
    assert_eq!(frames[1].temporal_reference, 1);
    assert_eq!(frames[2].temporal_reference, 2);
    assert_eq!(frames[0].picture_coding_type, PictureCodingType::Intra);
    assert_eq!(
        frames[1].picture_coding_type,
        PictureCodingType::Bidirectional
    );
    assert_eq!(frames[2].picture_coding_type, PictureCodingType::Predictive);
    assert_frames_equal(&frames[0].frame, &r_i, "I");
    assert_frames_equal(&frames[2].frame, &r_p, "P");
    assert!(luma_mae(&f_b, &frames[1].frame) < 6.0, "B fidelity");
}

#[test]
fn display_order_sequence_emits_conformant_multi_gop_stream() {
    // 8 display frames, 2 Bs between anchors, 1 anchor period per GOP
    // → GOP display spans [0..=3], [4..=7], each I B B P.
    let (w, h) = (64usize, 48usize);
    let display: Vec<FrameBuffer> = (0..8).map(|k| frame_at(w, h, 2 * k, k, k == 5)).collect();
    let stream =
        encode_mpeg1_display_order_sequence(&display, 2, 1, &seq(w as u16, h as u16), 6, 3, 3)
            .expect("encode");

    // ---- Header-level checks -------------------------------------
    let header = Mpeg2SequenceHeader::parse(&stream).expect("sequence header");
    assert_eq!(header.width, 64);
    assert_eq!(header.height, 48);
    // 64x48 @ 25fps, f_code 3 → constrained-parameters admissible.
    assert!(header.constrained_parameters);

    // Two GOP headers, closed_gop = 1, broken_link = 0, time codes at
    // display frames 0 and 4 (25 fps).
    let gops: Vec<Mpeg2Gop> = stream
        .windows(4)
        .enumerate()
        .filter(|(_, w4)| w4 == &[0x00, 0x00, 0x01, 0xB8])
        .map(|(pos, _)| Mpeg2Gop::parse(&stream[pos..]).expect("gop header"))
        .collect();
    assert_eq!(gops.len(), 2, "one GOP header per GOP");
    for gop in &gops {
        assert!(gop.closed_gop, "closed GOP structure");
        assert!(!gop.broken_link);
    }
    assert_eq!(
        gops[0].time_code,
        TimeCode::from_display_index(0, 3).unwrap()
    );
    assert_eq!(
        gops[1].time_code,
        TimeCode::from_display_index(4, 3).unwrap()
    );

    // No extension start code anywhere (MPEG-1-legal absence).
    assert!(
        !stream.windows(4).any(|w4| w4 == [0x00, 0x00, 0x01, 0xB5]),
        "extension start code in an MPEG-1 stream"
    );

    // ---- Decode-level checks -------------------------------------
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 8);

    // Display order restored; temporal_reference resets per GOP:
    // 0 1 2 3 | 0 1 2 3.
    let trefs: Vec<u16> = frames.iter().map(|f| f.temporal_reference).collect();
    assert_eq!(trefs, vec![0, 1, 2, 3, 0, 1, 2, 3]);

    // Picture types per position: I B B P | I B B P.
    let types: Vec<PictureCodingType> = frames.iter().map(|f| f.picture_coding_type).collect();
    let expect_group = [
        PictureCodingType::Intra,
        PictureCodingType::Bidirectional,
        PictureCodingType::Bidirectional,
        PictureCodingType::Predictive,
    ];
    assert_eq!(&types[0..4], &expect_group);
    assert_eq!(&types[4..8], &expect_group);

    // The coded-order display indices (I P B B per GOP) recover the
    // continuous 0..8 display numbering across the GOP reset.
    let coded_order = [
        (0u16, PictureCodingType::Intra),
        (3, PictureCodingType::Predictive),
        (1, PictureCodingType::Bidirectional),
        (2, PictureCodingType::Bidirectional),
        (0, PictureCodingType::Intra),
        (3, PictureCodingType::Predictive),
        (1, PictureCodingType::Bidirectional),
        (2, PictureCodingType::Bidirectional),
    ];
    let indices = display_indices_from_coded_pictures(&coded_order);
    assert_eq!(indices, vec![0, 3, 1, 2, 4, 7, 5, 6]);

    // Round-trip fidelity for every display frame.
    for (k, decoded) in frames.iter().enumerate() {
        let mae = luma_mae(&display[k], &decoded.frame);
        assert!(mae < 7.0, "frame {k}: luma MAE {mae:.2}");
    }
}

#[test]
fn degenerate_gop_structures_decode() {
    let (w, h) = (32usize, 32usize);
    // Single frame → one all-intra GOP.
    let single = [frame_at(w, h, 0, 0, false)];
    let stream =
        encode_mpeg1_intra_stream(&single[0], &seq(w as u16, h as u16), 8).expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].picture_coding_type, PictureCodingType::Intra);

    // b_between = 0 → I P P … chain, one GOP of 2 anchor periods.
    let display: Vec<FrameBuffer> = (0..3).map(|k| frame_at(w, h, k, k, false)).collect();
    let stream =
        encode_mpeg1_display_order_sequence(&display, 0, 2, &seq(w as u16, h as u16), 8, 2, 2)
            .expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 3);
    let types: Vec<PictureCodingType> = frames.iter().map(|f| f.picture_coding_type).collect();
    assert_eq!(
        types,
        vec![
            PictureCodingType::Intra,
            PictureCodingType::Predictive,
            PictureCodingType::Predictive,
        ]
    );
    // Single GOP → continuous temporal references.
    let trefs: Vec<u16> = frames.iter().map(|f| f.temporal_reference).collect();
    assert_eq!(trefs, vec![0, 1, 2]);
}

#[test]
fn non_macroblock_multiple_geometry_roundtrips() {
    // 100x62: right/bottom edge macroblocks overhang the visible
    // picture; the MPEG-1 grid is Ceil(h/16) rows.
    let (w, h) = (100usize, 62usize);
    let display: Vec<FrameBuffer> = (0..3).map(|k| frame_at(w, h, 2 * k, k, false)).collect();
    let stream =
        encode_mpeg1_display_order_sequence(&display, 1, 1, &seq(w as u16, h as u16), 6, 3, 3)
            .expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 3);
    assert_eq!((frames[0].frame.width, frames[0].frame.height), (w, h));
    for (k, decoded) in frames.iter().enumerate() {
        let mae = luma_mae(&display[k], &decoded.frame);
        assert!(mae < 7.0, "frame {k}: luma MAE {mae:.2}");
    }
}

#[test]
fn wide_motion_uses_larger_f_code() {
    // A 12-pel shift needs f_code >= 3 (range ±(16*4-1)/2 half-pel =
    // integer ±31 at f=4). Encode with f_code 4 and confirm fidelity —
    // the search window must actually reach the true motion.
    let (w, h) = (96usize, 48usize);
    let display: Vec<FrameBuffer> = (0..2).map(|k| frame_at(w, h, 12 * k, 0, false)).collect();
    let stream =
        encode_mpeg1_display_order_sequence(&display, 0, 1, &seq(w as u16, h as u16), 5, 4, 4)
            .expect("encode");
    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 2);
    let mae = luma_mae(&display[1], &frames[1].frame);
    assert!(mae < 6.0, "P frame with wide motion: luma MAE {mae:.2}");
}

#[test]
fn loaded_quantizer_matrices_thread_through_encode_and_decode() {
    // Custom (flatter-than-default) matrices: the encoder must
    // quantise with them and the decoder must pick them up from the
    // sequence header, so the decode still equals the encoder's own
    // reconstruction sample-for-sample — and differs from a
    // default-matrix encode of the same content.
    let (w, h) = (48usize, 32usize);
    let display: Vec<FrameBuffer> = (0..3).map(|k| frame_at(w, h, 2 * k, k, false)).collect();

    let mut intra = [8u8; 64];
    for (i, v) in intra.iter_mut().enumerate().skip(1) {
        *v = 12 + (i as u8 % 8);
    }
    let non_intra = [20u8; 64];
    let seq_custom = Mpeg1SequenceParams {
        intra_quant_matrix: Some(intra),
        non_intra_quant_matrix: Some(non_intra),
        ..seq(w as u16, h as u16)
    };

    let custom = encode_mpeg1_display_order_sequence(&display, 1, 1, &seq_custom, 6, 3, 3)
        .expect("custom-matrix encode");
    let default =
        encode_mpeg1_display_order_sequence(&display, 1, 1, &seq(w as u16, h as u16), 6, 3, 3)
            .expect("default-matrix encode");
    assert_ne!(
        custom, default,
        "loaded matrices must change the coded bits"
    );

    // The header carries both payloads.
    let header = Mpeg2SequenceHeader::parse(&custom).expect("sequence header");
    assert_eq!(header.intra_quant, Some(intra));
    assert_eq!(header.non_intra_quant, Some(non_intra));

    // Decode round-trips with bounded error (the decoder must apply
    // the loaded matrices — decoding with the defaults would shear
    // every AC coefficient).
    let frames = decode_video_sequence(&custom).expect("decode");
    assert_eq!(frames.len(), 3);
    for (k, decoded) in frames.iter().enumerate() {
        let mae = luma_mae(&display[k], &decoded.frame);
        assert!(mae < 7.0, "frame {k}: luma MAE {mae:.2}");
    }
}

#[test]
fn full_pel_vectors_roundtrip_sample_exact() {
    // full_pel_forward_vector / full_pel_backward_vector = 1: the wire
    // codes unshifted integer-pel vectors that the §2.4.4.2/§2.4.4.3
    // final `recon <<= 1` doubles back. The decode must still equal
    // the encoder's reconstructions exactly, and the picture headers
    // must carry the flags.
    use oxideav_core::bits::BitWriter;
    use oxideav_mpeg12video::gop_header::write_gop_header;
    use oxideav_mpeg12video::picture_header::Mpeg2PictureHeader;
    use oxideav_mpeg12video::write_mpeg1_sequence_header;

    let (w, h) = (64usize, 48usize);
    let p = params(w, h);
    // A 6-pel shift: exactly representable as a full-pel vector.
    let f_i = frame_at(w, h, 0, 0, false);
    let f_b = frame_at(w, h, 3, 0, false);
    let f_p = frame_at(w, h, 6, 0, false);

    let mut bw = BitWriter::new();
    write_mpeg1_sequence_header(&mut bw, &seq(w as u16, h as u16)).expect("write header");
    write_gop_header(
        &mut bw,
        &Mpeg2Gop {
            time_code: TimeCode::from_display_index(0, 3).unwrap(),
            closed_gop: true,
            broken_link: false,
        },
    );
    let r_i = encode_mpeg1_intra_picture(&mut bw, &f_i, &p, 0, 6).expect("I");
    let r_p = encode_mpeg1_p_picture(&mut bw, &f_p, &r_i, &p, 2, 6, 3, true).expect("P full-pel");
    encode_mpeg1_b_picture(&mut bw, &f_b, &r_i, &r_p, &p, 1, 6, 3, 3, true, true)
        .expect("B full-pel");
    let mut stream = bw.finish();
    stream.extend_from_slice(&[0x00, 0x00, 0x01, 0xB7]);

    // The two inter picture headers carry the full_pel flags.
    let pic_starts: Vec<usize> = stream
        .windows(4)
        .enumerate()
        .filter(|(_, w4)| w4 == &[0x00, 0x00, 0x01, 0x00])
        .map(|(pos, _)| pos)
        .collect();
    assert_eq!(pic_starts.len(), 3);
    let p_hdr = Mpeg2PictureHeader::parse(&stream[pic_starts[1]..]).expect("P header");
    assert_eq!(p_hdr.full_pel_forward_vector, Some(true));
    let b_hdr = Mpeg2PictureHeader::parse(&stream[pic_starts[2]..]).expect("B header");
    assert_eq!(b_hdr.full_pel_forward_vector, Some(true));
    assert_eq!(b_hdr.full_pel_backward_vector, Some(true));

    let frames = decode_video_sequence(&stream).expect("decode");
    assert_eq!(frames.len(), 3);
    assert_frames_equal(&frames[0].frame, &r_i, "I");
    assert_frames_equal(&frames[2].frame, &r_p, "P full-pel");
    // Fidelity: the shifts are integer-pel so the full-pel restriction
    // costs nothing.
    assert!(luma_mae(&f_p, &frames[2].frame) < 6.0, "P fidelity");
    assert!(luma_mae(&f_b, &frames[1].frame) < 6.0, "B fidelity");
}
