//! `VideoSequenceDecoder` fed one picture at a time reconstructs the
//! same frames as the whole-stream `decode_video_sequence`, in coded
//! order: frame pictures with B-frames, and field-picture pairs split
//! across calls.

use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::{
    DecodedFrame, FrameBuffer, IntraPictureParams, PictureCodingType, VideoSequenceDecoder,
    decode_video_sequence, encode_display_order_gop_sequence,
    encode_field_display_order_gop_sequence,
};

const W: usize = 48;
const H: usize = 64;

fn params(progressive: bool) -> IntraPictureParams {
    IntraPictureParams {
        width: W,
        height: H,
        chroma_format: ChromaFormat::Yuv420,
        frame_pred_frame_dct: progressive,
        intra_dc_precision: 0,
        intra_vlc_format: false,
        alternate_scan: false,
        q_scale_type: false,
        progressive_sequence: progressive,
    }
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

fn check(stream: &[u8]) {
    let whole = decode_video_sequence(stream).expect("whole-stream decode");
    let mut decoder = VideoSequenceDecoder::new();
    let mut coded = Vec::new();
    let units = pictures(stream);
    assert!(units.len() > 1);
    for unit in units {
        coded.extend(decoder.decode(unit).expect("picture decode"));
    }
    let shown = display(coded);
    assert_eq!(shown.len(), whole.len());
    for (a, b) in shown.iter().zip(&whole) {
        assert_eq!(a.temporal_reference, b.temporal_reference);
        assert_eq!(a.frame.y, b.frame.y);
        assert_eq!(a.frame.cb, b.frame.cb);
        assert_eq!(a.frame.cr, b.frame.cr);
    }
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
fn a_picture_before_any_sequence_header_is_an_error() {
    let frames: Vec<FrameBuffer> = (0..2).map(frame_at).collect();
    let stream = encode_display_order_gop_sequence(&frames, 0, 2, params(true), 6, 3, 3)
        .expect("encode frame sequence");
    let units = pictures(&stream);
    let mut decoder = VideoSequenceDecoder::new();
    assert!(decoder.decode(units[1]).is_err());
}
