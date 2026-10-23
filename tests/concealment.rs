//! Error concealment in [`SequenceDecoder::with_concealment`]: damaged
//! pictures are replaced instead of aborting the decode, so a transcoder
//! keeps every frame slot and the reference chain recovers at the next
//! intra picture; B-pictures with no forward reference (the leading
//! pictures of an open GOP when decoding starts mid-stream) are dropped.
//! The strict default is unchanged.
//!
//! Damage is made from the committed black-box streams: slices are cut
//! out of one P-picture, or decoding starts at a later sequence header.

use oxideav_mpeg12video::{DecodedFrame, SequenceDecoder, decode_video_sequence};

const MPEG2: &[u8] = include_bytes!("fixtures/conformance/mpeg2-ibbp-96x64.m2v");
const MPEG1: &[u8] = include_bytes!("fixtures/conformance/mpeg1-ibbp-96x64.m1v");

const PICTURE: u8 = 0x00;
const SEQUENCE_HEADER: u8 = 0xB3;
/// ISO/IEC 13818-2 Table 6-12 / 11172-2 §2.4.3.4 `picture_coding_type`.
const P_PICTURE: u8 = 2;

fn start_codes(stream: &[u8], code: impl Fn(u8) -> bool) -> Vec<usize> {
    (0..stream.len().saturating_sub(3))
        .filter(|&i| stream[i..i + 3] == [0, 0, 1] && code(stream[i + 3]))
        .collect()
}

/// The stream with every slice after the first removed from its
/// `nth` P-picture, so that picture no longer covers its macroblocks.
fn cut_slices_of_p_picture(stream: &[u8], nth: usize) -> Vec<u8> {
    let pictures = start_codes(stream, |c| c == PICTURE);
    let p = pictures
        .iter()
        .copied()
        .filter(|&at| (stream[at + 5] >> 3) & 0x07 == P_PICTURE)
        .nth(nth)
        .expect("P-picture");
    let end = start_codes(&stream[p + 4..], |c| matches!(c, 0x00 | 0xB3 | 0xB7 | 0xB8))
        .first()
        .map_or(stream.len(), |r| p + 4 + r);
    let slices = start_codes(&stream[p..end], |c| (0x01..=0xAF).contains(&c));
    assert!(slices.len() >= 2, "picture has several slices");
    [&stream[..p + slices[1]], &stream[end..]].concat()
}

fn decode(stream: &[u8], conceal: bool) -> (Result<Vec<DecodedFrame>, String>, usize, usize) {
    let mut decoder = if conceal {
        SequenceDecoder::with_concealment()
    } else {
        SequenceDecoder::new()
    };
    decoder.push(stream);
    decoder.finish();
    let mut frames = Vec::new();
    loop {
        match decoder.next_frame() {
            Ok(Some(frame)) => frames.push(frame),
            Ok(None) => return (Ok(frames), decoder.concealed(), decoder.skipped()),
            Err(e) => {
                return (
                    Err(format!("{e:?}")),
                    decoder.concealed(),
                    decoder.skipped(),
                );
            }
        }
    }
}

fn tail_matches(got: &[DecodedFrame], want: &[DecodedFrame], n: usize) -> bool {
    got.len() >= n
        && want.len() >= n
        && got[got.len() - n..]
            .iter()
            .zip(&want[want.len() - n..])
            .all(|(a, b)| a.frame == b.frame && a.temporal_reference == b.temporal_reference)
}

#[test]
fn a_damaged_picture_keeps_its_frame_slot_and_the_chain_recovers() {
    for (name, stream) in [("mpeg2", MPEG2), ("mpeg1", MPEG1)] {
        let clean = decode_video_sequence(stream).unwrap();
        let damaged = cut_slices_of_p_picture(stream, 0);
        assert!(
            decode(&damaged, false).0.is_err(),
            "{name}: strict decode rejects the damage"
        );
        let (frames, concealed, skipped) = decode(&damaged, true);
        let frames = frames.unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_eq!((concealed, skipped), (1, 0), "{name}");
        assert_eq!(frames.len(), clean.len(), "{name}: every frame slot kept");
        // The damage sits in the first GOP; from the second I-picture on the
        // decode is untouched.
        assert!(
            tail_matches(&frames, &clean, clean.len() / 2),
            "{name}: chain recovers"
        );
    }
}

#[test]
fn decoding_from_a_later_sequence_header_drops_unreferenced_b_pictures() {
    let headers = start_codes(MPEG2, |c| c == SEQUENCE_HEADER);
    assert!(headers.len() >= 2, "repeated sequence headers");
    let suffix = &MPEG2[headers[1]..];
    let clean = decode_video_sequence(MPEG2).unwrap();
    let (frames, concealed, skipped) = decode(suffix, true);
    let frames = frames.unwrap();
    assert_eq!(concealed, 0);
    assert!(
        skipped > 0,
        "the open GOP's leading B-pictures had no forward reference"
    );
    assert!(
        tail_matches(&frames, &clean, frames.len()),
        "the rest decodes exactly"
    );
    assert!(
        decode(suffix, false).0.is_err(),
        "strict decode still rejects them"
    );
}
