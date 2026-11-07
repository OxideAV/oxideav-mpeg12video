//! Incremental `video_sequence()` decode through [`SequenceDecoder`]:
//! bytes pushed in arbitrary pieces must reconstruct exactly the frames,
//! in exactly the order, that [`decode_video_sequence`] produces from the
//! whole stream, and frames must become available before end of stream
//! so a long sequence never has to be held in memory at once.
//!
//! The streams are the committed conformance and self-encoded corpora,
//! less the scalable enhancement layers (`*-enh-*`), which only decode
//! together with their base layer.

use std::path::PathBuf;

use oxideav_mpeg12video::{DecodedFrame, SequenceDecoder, decode_video_sequence};

/// Piece sizes: single bytes and odd sizes split every start code, 188
/// is a transport-stream packet, 2048 a DVD sector.
const PIECE_SIZES: [usize; 5] = [1, 3, 188, 2048, usize::MAX];

fn corpus() -> Vec<(String, Vec<u8>)> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let mut streams = Vec::new();
    for dir in ["conformance", "selfenc"] {
        for entry in std::fs::read_dir(root.join(dir)).expect("fixture dir") {
            let path = entry.expect("dir entry").path();
            let name = path.file_name().unwrap().to_string_lossy().into_owned();
            if matches!(
                path.extension().and_then(|e| e.to_str()),
                Some("m1v" | "m2v")
            ) && !name.contains("-enh-")
            {
                streams.push((name, std::fs::read(&path).expect("fixture")));
            }
        }
    }
    streams.sort();
    assert!(streams.len() >= 10, "corpus present");
    streams
}

/// Pushes `stream` in `piece`-byte pieces, draining frames after each
/// push. Returns the frames and how many were drained before `finish`.
fn decode_in_pieces(stream: &[u8], piece: usize) -> (Vec<DecodedFrame>, usize) {
    let mut decoder = SequenceDecoder::new();
    let mut frames = Vec::new();
    for chunk in stream.chunks(piece.min(stream.len()).max(1)) {
        decoder.push(chunk);
        while let Some(frame) = decoder.next_frame().expect("decode") {
            frames.push(frame);
        }
    }
    let before_finish = frames.len();
    decoder.finish();
    while let Some(frame) = decoder.next_frame().expect("decode") {
        frames.push(frame);
    }
    (frames, before_finish)
}

fn same_frame(a: &DecodedFrame, b: &DecodedFrame) -> bool {
    a.frame == b.frame
        && a.temporal_reference == b.temporal_reference
        && a.picture_coding_type == b.picture_coding_type
        && a.top_field_first == b.top_field_first
        && a.repeat_first_field == b.repeat_first_field
        && a.progressive_frame == b.progressive_frame
}

#[test]
fn piecewise_decode_matches_whole_stream_decode() {
    for (name, stream) in corpus() {
        let whole = decode_video_sequence(&stream).unwrap_or_else(|e| panic!("{name}: {e:?}"));
        for piece in PIECE_SIZES {
            let (frames, _) = decode_in_pieces(&stream, piece);
            assert_eq!(
                frames.len(),
                whole.len(),
                "{name} piece {piece}: frame count"
            );
            for (n, (got, want)) in frames.iter().zip(&whole).enumerate() {
                assert!(
                    same_frame(got, want),
                    "{name} piece {piece}: frame {n} differs"
                );
            }
        }
    }
}

/// Only the last coded picture (whose end is unknown until the stream
/// ends) and the held-back anchor may wait for `finish`.
#[test]
fn frames_are_released_before_end_of_stream() {
    for (name, stream) in corpus() {
        let (frames, before_finish) = decode_in_pieces(&stream, 2048);
        if frames.len() > 2 {
            assert!(
                before_finish >= frames.len() - 2,
                "{name}: only {before_finish} of {} frames before finish",
                frames.len()
            );
        }
    }
}

#[test]
fn finishing_without_a_sequence_header_is_an_error() {
    let mut decoder = SequenceDecoder::new();
    decoder.push(&[0, 0, 1, 0x00, 0x12, 0x34]);
    assert!(
        matches!(decoder.next_frame(), Ok(None)),
        "waits for more input"
    );
    decoder.finish();
    assert!(decoder.next_frame().is_err());
}

/// Every frame reports the stream offset of its picture start code (the first field's for a
/// field pair), so a caller can attach the presentation time of the packet that carried it;
/// offsets are the same however the stream was split.
#[test]
fn frames_report_their_picture_start_code_offset() {
    for (name, stream) in corpus() {
        let whole = decode_video_sequence(&stream).unwrap();
        let mut seen = std::collections::BTreeSet::new();
        for frame in &whole {
            let at = frame.stream_offset.expect("offset") as usize;
            assert_eq!(
                stream.get(at..at + 4),
                Some(&[0, 0, 1, 0][..]),
                "{name}: offset {at}"
            );
            assert!(seen.insert(at), "{name}: offset {at} reported twice");
        }
        let (pieces, _) = decode_in_pieces(&stream, 188);
        let offsets = |f: &[DecodedFrame]| f.iter().map(|f| f.stream_offset).collect::<Vec<_>>();
        assert_eq!(offsets(&pieces), offsets(&whole), "{name}");
    }
}
