//! [`SequenceDecoder::coded_order`] and
//! [`SequenceDecoder::end_access_unit`]: a container that delimits access
//! units gets each picture back as soon as that unit is pushed, in coded
//! order, and applying the §6.1.1.11 reorder to the coded-order output
//! reproduces the display-order decode exactly.
//!
//! The streams are the committed conformance and self-encoded corpora,
//! less the scalable enhancement layers (`*-enh-*`), which only decode
//! together with their base layer.

use std::path::PathBuf;

use oxideav_mpeg12video::picture_header::PictureCodingType;
use oxideav_mpeg12video::{DecodedFrame, SequenceDecoder, decode_video_sequence};

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

fn same_frame(a: &DecodedFrame, b: &DecodedFrame) -> bool {
    a.frame == b.frame
        && a.temporal_reference == b.temporal_reference
        && a.picture_coding_type == b.picture_coding_type
        && a.top_field_first == b.top_field_first
        && a.repeat_first_field == b.repeat_first_field
        && a.progressive_frame == b.progressive_frame
        && a.stream_offset == b.stream_offset
}

fn drain(decoder: &mut SequenceDecoder, frames: &mut Vec<DecodedFrame>) {
    while let Some(frame) = decoder.next_frame().expect("decode") {
        frames.push(frame);
    }
}

fn coded_order_decode(stream: &[u8]) -> Vec<DecodedFrame> {
    let mut decoder = SequenceDecoder::new().coded_order();
    decoder.push(stream);
    decoder.finish();
    let mut frames = Vec::new();
    drain(&mut decoder, &mut frames);
    frames
}

/// §6.1.1.11 applied to coded-order frames: B- and D-pictures display at
/// once, an I/P picture displaces the held-back anchor.
fn reorder(coded: &[DecodedFrame]) -> Vec<DecodedFrame> {
    let mut held: Option<DecodedFrame> = None;
    let mut display = Vec::new();
    for frame in coded {
        match frame.picture_coding_type {
            PictureCodingType::Intra | PictureCodingType::Predictive => {
                display.extend(held.replace(frame.clone()));
            }
            _ => display.push(frame.clone()),
        }
    }
    display.extend(held);
    display
}

/// Start codes as `(offset, code)`.
fn start_codes(stream: &[u8]) -> Vec<(usize, u8)> {
    (0..stream.len().saturating_sub(3))
        .filter(|&i| stream[i..i + 3] == [0, 0, 1])
        .map(|i| (i, stream[i + 3]))
        .collect()
}

/// Access units the way a container delimits them: each begins at the
/// first sequence header, GOP header or picture start code after the
/// previous picture's data. Returns `(start, field_picture)` per unit.
fn access_units(stream: &[u8]) -> Vec<(usize, bool)> {
    let codes = start_codes(stream);
    let mut units = Vec::new();
    let mut open: Option<usize> = None;
    for (n, &(at, code)) in codes.iter().enumerate() {
        match code {
            0xB3 | 0xB8 => {
                open.get_or_insert(at);
            }
            0x00 => {
                let start = open.take().unwrap_or(at);
                // §6.2.3.1 picture_coding_extension: the low two bits of
                // its third byte are picture_structure (3 = frame).
                let field = codes[n + 1..]
                    .iter()
                    .take_while(|&&(_, c)| !(0x01..=0xAF).contains(&c) && c != 0x00)
                    .find(|&&(p, c)| c == 0xB5 && stream.get(p + 4).is_some_and(|b| b >> 4 == 8))
                    .is_some_and(|&(p, _)| stream[p + 6] & 0x03 != 0x03);
                units.push((start, field));
            }
            _ => {}
        }
    }
    units
}

#[test]
fn coded_order_reordered_matches_the_display_order_decode() {
    for (name, stream) in corpus() {
        let display = decode_video_sequence(&stream).unwrap_or_else(|e| panic!("{name}: {e:?}"));
        let coded = coded_order_decode(&stream);
        let reordered = reorder(&coded);
        assert_eq!(reordered.len(), display.len(), "{name}: frame count");
        for (n, (got, want)) in reordered.iter().zip(&display).enumerate() {
            assert!(same_frame(got, want), "{name}: display frame {n} differs");
        }
    }
}

#[test]
fn each_access_unit_returns_its_picture_at_once() {
    for (name, stream) in corpus() {
        let whole = coded_order_decode(&stream);
        let units = access_units(&stream);
        assert!(!units.is_empty(), "{name}: access units");
        let mut decoder = SequenceDecoder::new().coded_order();
        let mut frames = Vec::new();
        let mut expected = 0usize;
        let mut first_field_pending = false;
        for (n, &(start, field)) in units.iter().enumerate() {
            let end = units.get(n + 1).map_or(stream.len(), |&(s, _)| s);
            if n == 0 {
                decoder.push(&stream[..end]);
            } else {
                decoder.push(&stream[start..end]);
            }
            decoder.end_access_unit();
            drain(&mut decoder, &mut frames);
            if field {
                first_field_pending = !first_field_pending;
                expected += usize::from(!first_field_pending);
            } else {
                expected += 1;
            }
            assert_eq!(
                frames.len(),
                expected,
                "{name}: frames out after access unit {n}"
            );
        }
        decoder.finish();
        drain(&mut decoder, &mut frames);
        assert_eq!(frames.len(), whole.len(), "{name}: frame count");
        for (n, (got, want)) in frames.iter().zip(&whole).enumerate() {
            assert!(same_frame(got, want), "{name}: coded frame {n} differs");
        }
    }
}

/// Without the access-unit declaration a picture waits for the start code
/// that ends it; pushing more bytes after the declaration withdraws it.
#[test]
fn a_picture_waits_for_its_end_unless_the_unit_is_declared() {
    let stream = include_bytes!("fixtures/conformance/mpeg2-ibbp-96x64.m2v");
    let units = access_units(stream);
    let second = units[1].0;
    let mut frames = Vec::new();

    let mut decoder = SequenceDecoder::new().coded_order();
    decoder.push(&stream[..second]);
    drain(&mut decoder, &mut frames);
    assert!(frames.is_empty(), "the I-picture's end is not known yet");
    decoder.end_access_unit();
    drain(&mut decoder, &mut frames);
    assert_eq!(frames.len(), 1, "declared unit decodes at once");

    let mut decoder = SequenceDecoder::new().coded_order();
    decoder.push(&stream[..second]);
    decoder.end_access_unit();
    decoder.push(&stream[second..second + 8]);
    let mut late = Vec::new();
    drain(&mut decoder, &mut late);
    assert_eq!(late.len(), 1, "the next start code still ends the picture");
    assert!(same_frame(&late[0], &frames[0]));
}
