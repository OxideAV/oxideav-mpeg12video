//! `SequenceDecoder` fed one picture at a time (coded order, each access
//! unit declared complete) reconstructs the same frames as the
//! whole-stream `decode_video_sequence`: frame pictures with B-frames, and
//! field-picture pairs split across units. `reset()` (a seek) drops the
//! anchors but keeps the sequence layer, so a later closed GOP pushed
//! without its sequence header decodes exactly as in the whole stream.

use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    DecodedFrame, FrameBuffer, IntraPictureParams, PictureCodingType, SequenceDecoder,
    decode_video_sequence, encode_display_order_gop_sequence,
    encode_field_display_order_gop_sequence,
};

const W: usize = 48;
const H: usize = 64;

fn params(progressive: bool) -> IntraPictureParams {
    IntraPictureParams::new(
        W,
        H,
        ChromaFormat::Yuv420,
        progressive,
        0,
        false,
        false,
        false,
        progressive,
    )
}

fn frame_at(t: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(W, H, ChromaFormat::Yuv420);
    for y in 0..H {
        for x in 0..W {
            let v = ((x * 5 + y * 3 + t * 7 + (y & 1) * 40) % 200 + 20) as u8;
            f.y.put_sample(x, y, v);
        }
    }
    let (cw, ch) = (W / 2, H / 2);
    for y in 0..ch {
        for x in 0..cw {
            f.cb.put_sample(x, y, ((x * 4 + t * 3) % 100 + 80) as u8);
            f.cr.put_sample(x, y, ((y * 4 + t * 5) % 100 + 80) as u8);
        }
    }
    f
}

/// The stream cut before every picture_start_code (a sequence header
/// and GOP header stay with the picture that follows them).
fn pictures(stream: &[u8]) -> Vec<&[u8]> {
    let mut cuts = vec![0];
    let mut i = 0;
    let mut header = false;
    while i + 4 <= stream.len() {
        if stream[i..i + 3] == [0, 0, 1] {
            match stream[i + 3] {
                0xB3 | 0xB8 => {
                    if !header && i > 0 {
                        cuts.push(i);
                    }
                    header = true;
                }
                0x00 => {
                    if !header && i > 0 {
                        cuts.push(i);
                    }
                    header = false;
                }
                _ => {}
            }
            i += 4;
        } else {
            i += 1;
        }
    }
    cuts.push(stream.len());
    cuts.dedup();
    cuts.windows(2).map(|w| &stream[w[0]..w[1]]).collect()
}

/// Offsets of every `group_of_pictures` start code.
fn gop_starts(stream: &[u8]) -> Vec<usize> {
    (0..stream.len().saturating_sub(3))
        .filter(|&i| stream[i..i + 4] == [0, 0, 1, 0xB8])
        .collect()
}

/// The §6.1.1.11 display order of coded-order frames.
fn display(coded: Vec<DecodedFrame>) -> Vec<DecodedFrame> {
    let mut out = Vec::new();
    let mut held: Option<DecodedFrame> = None;
    for f in coded {
        if f.picture_coding_type == PictureCodingType::Bidirectional {
            out.push(f);
        } else {
            out.extend(held.take());
            held = Some(f);
        }
    }
    out.extend(held);
    out
}

fn drain(decoder: &mut SequenceDecoder, out: &mut Vec<DecodedFrame>) {
    while let Some(frame) = decoder.next_frame().expect("picture decode") {
        out.push(frame);
    }
}

fn assert_same(a: &[DecodedFrame], b: &[DecodedFrame]) {
    assert_eq!(a.len(), b.len());
    for (a, b) in a.iter().zip(b) {
        assert_eq!(a.temporal_reference, b.temporal_reference);
        assert_eq!(a.picture_coding_type, b.picture_coding_type);
        assert_eq!(a.frame.y, b.frame.y);
        assert_eq!(a.frame.cb, b.frame.cb);
        assert_eq!(a.frame.cr, b.frame.cr);
    }
}

fn check(stream: &[u8]) {
    let whole = decode_video_sequence(stream).expect("whole-stream decode");
    let mut decoder = SequenceDecoder::new().coded_order();
    let mut coded = Vec::new();
    let units = pictures(stream);
    assert!(units.len() > 1);
    for unit in units {
        decoder.push(unit);
        decoder.end_access_unit();
        drain(&mut decoder, &mut coded);
    }
    decoder.finish();
    drain(&mut decoder, &mut coded);
    assert_same(&display(coded), &whole);
}

#[test]
fn frame_pictures_with_b_frames_decode_picture_by_picture() {
    let frames: Vec<FrameBuffer> = (0..7).map(frame_at).collect();
    let stream = encode_display_order_gop_sequence(&frames, 1, 2, params(true), 6, 3, 3)
        .expect("encode frame sequence");
    check(&stream);
}

#[test]
fn field_picture_pairs_decode_picture_by_picture() {
    let frames: Vec<FrameBuffer> = (0..5).map(frame_at).collect();
    let stream = encode_field_display_order_gop_sequence(&frames, 1, 2, &params(false), 6, 3, 3)
        .expect("encode field sequence");
    check(&stream);
}

#[test]
fn a_stream_without_a_sequence_header_is_an_error() {
    let frames: Vec<FrameBuffer> = (0..2).map(frame_at).collect();
    let stream = encode_display_order_gop_sequence(&frames, 0, 2, params(true), 6, 3, 3)
        .expect("encode frame sequence");
    let units = pictures(&stream);
    let mut decoder = SequenceDecoder::new();
    decoder.push(units[1]);
    decoder.end_access_unit();
    assert!(matches!(decoder.next_frame(), Ok(None)));
    assert!(!decoder.has_sequence());
    decoder.finish();
    assert!(decoder.next_frame().is_err());
}

#[test]
fn reset_drops_the_anchors_and_keeps_the_sequence_layer() {
    // Closed GOPs (every GOP opens on an I-picture, no B-picture
    // references across a boundary), so a seek to a later GOP header
    // reproduces that GOP's frames exactly.
    let frames: Vec<FrameBuffer> = (0..9).map(frame_at).collect();
    let stream = encode_display_order_gop_sequence(&frames, 1, 1, params(true), 6, 3, 3)
        .expect("encode frame sequence");
    let whole = decode_video_sequence(&stream).expect("whole-stream decode");
    let gops = gop_starts(&stream);
    assert!(gops.len() >= 3, "expected several GOPs, got {}", gops.len());
    let seek_to = gops[2];

    let mut decoder = SequenceDecoder::new();
    assert!(!decoder.has_sequence());
    // Decode the opening GOP, leaving an anchor held back and both
    // references populated.
    decoder.push(&stream[..gops[1]]);
    decoder.end_access_unit();
    let mut before = Vec::new();
    drain(&mut decoder, &mut before);
    assert!(!before.is_empty());
    assert!(decoder.has_sequence());

    decoder.reset();
    assert!(decoder.has_sequence());
    decoder.push(&stream[seek_to..]);
    decoder.finish();
    let mut after = Vec::new();
    drain(&mut decoder, &mut after);

    assert!(!after.is_empty() && after.len() < whole.len());
    assert_eq!(after[0].picture_coding_type, PictureCodingType::Intra);
    assert_same(&after, &whole[whole.len() - after.len()..]);
}
