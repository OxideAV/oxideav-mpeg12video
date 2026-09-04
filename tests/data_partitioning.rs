//! §7.10 data partitioning over the whole ISO/IEC 13818-2 corpus:
//! every conformance / self-encoded `.m2v` stream is split into its
//! partition pair at every representative Table 7-30 breakpoint
//! (`1`, `2`, `3`, `64`, `65`, `72`, `127`) and merged back — the
//! merge must reproduce the original stream **byte-exactly** — and the
//! `decode_data_partitioned` path must agree with the plain decode.

use oxideav_mpeg12video::sequence_scalable_extension::{ScalableMode, SequenceScalableExtension};
use oxideav_mpeg12video::{
    decode_data_partitioned, decode_video_sequence, merge_data_partitions, split_data_partitions,
};

const BREAKPOINTS: [u8; 7] = [1, 2, 3, 64, 65, 72, 127];

/// `true` when the stream declares itself scalable (an
/// `extension_start_code` whose identifier nibble is `0101`).
fn carries_sequence_scalable_extension(stream: &[u8]) -> bool {
    stream
        .windows(5)
        .any(|w| w[..4] == [0, 0, 1, 0xB5] && w[4] >> 4 == 0b0101)
}

fn corpus() -> Vec<(String, Vec<u8>)> {
    let mut out = Vec::new();
    for dir in ["conformance", "selfenc"] {
        let path = format!("{}/tests/fixtures/{dir}/", env!("CARGO_MANIFEST_DIR"));
        let mut names: Vec<String> = std::fs::read_dir(&path)
            .expect("fixture dir")
            .filter_map(|e| e.ok())
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .filter(|n| n.ends_with(".m2v"))
            .collect();
        names.sort();
        for n in names {
            let bytes = std::fs::read(format!("{path}{n}")).expect("read fixture");
            // §7.10 partitions non-scalable streams only: a scalable
            // enhancement layer (sequence_scalable_extension present)
            // is not a split candidate.
            if carries_sequence_scalable_extension(&bytes) {
                continue;
            }
            out.push((n, bytes));
        }
    }
    let ffmpeg = format!(
        "{}/tests/fixtures/ffmpeg-352x240-25fps.m2v",
        env!("CARGO_MANIFEST_DIR")
    );
    if let Ok(bytes) = std::fs::read(&ffmpeg) {
        out.push(("ffmpeg-352x240-25fps.m2v".into(), bytes));
    }
    assert!(out.len() >= 20, "corpus present");
    out
}

fn find_all(buf: &[u8], code: u8) -> Vec<usize> {
    buf.windows(4)
        .enumerate()
        .filter(|(_, w)| w[0] == 0 && w[1] == 0 && w[2] == 1 && w[3] == code)
        .map(|(i, _)| i)
        .collect()
}

fn extension_ids(buf: &[u8]) -> Vec<u8> {
    find_all(buf, 0xB5)
        .iter()
        .map(|&i| buf[i + 4] >> 4)
        .collect()
}

#[test]
fn split_then_merge_is_byte_exact_across_the_corpus() {
    for (name, stream) in corpus() {
        for pb in BREAKPOINTS {
            let (p0, p1) = split_data_partitions(&stream, pb)
                .unwrap_or_else(|e| panic!("{name} pb {pb}: split failed: {e:?}"));
            let merged = merge_data_partitions(&p0, &p1)
                .unwrap_or_else(|e| panic!("{name} pb {pb}: merge failed: {e:?}"));
            assert!(
                merged == stream,
                "{name} pb {pb}: merge(split(s)) != s ({} vs {} bytes)",
                merged.len(),
                stream.len()
            );
            // Both partitions declare data partitioning with their
            // layer ids; partition 1 carries only the §7.10-allowed
            // extensions (sequence, scalable, picture coding).
            for (layer, p) in [(0u8, &p0), (1u8, &p1)] {
                let sse_at = find_all(p, 0xB5)
                    .into_iter()
                    .find(|&i| p[i + 4] >> 4 == 0b0101)
                    .unwrap_or_else(|| {
                        panic!("{name}: partition {layer} lacks the scalable extension")
                    });
                let sse = SequenceScalableExtension::parse(&p[sse_at..]).expect("parse sse");
                assert_eq!(sse.scalable_mode, ScalableMode::DataPartitioning);
                assert_eq!(sse.layer_id, layer);
            }
            for id in extension_ids(&p1) {
                assert!(
                    matches!(id, 0b0001 | 0b0101 | 0b1000),
                    "{name}: partition 1 carries extension id {id:04b} (§7.10 forbids it)"
                );
            }
            // Slice counts match.
            let slices = |b: &[u8]| {
                b.windows(4)
                    .filter(|w| w[0] == 0 && w[1] == 0 && w[2] == 1 && (1..=0xAF).contains(&w[3]))
                    .count()
            };
            assert_eq!(
                slices(&p0),
                slices(&stream),
                "{name} pb {pb}: partition 0 slices"
            );
            assert_eq!(
                slices(&p1),
                slices(&stream),
                "{name} pb {pb}: partition 1 slices"
            );
        }
    }
}

#[test]
fn partition_sizes_track_the_breakpoint() {
    // Higher breakpoints move data from partition 1 into partition 0.
    let (name, stream) = corpus()
        .into_iter()
        .find(|(n, _)| n == "mpeg2-ibbp-96x64.m2v")
        .expect("ibbp fixture");
    let mut last_p0 = 0usize;
    for pb in BREAKPOINTS {
        let (p0, _p1) = split_data_partitions(&stream, pb).expect("split");
        assert!(p0.len() >= last_p0, "{name}: partition 0 shrank at pb {pb}");
        last_p0 = p0.len();
    }
    // pb 1 leaves only headers in partition 0; pb 127 leaves only
    // slice headers + alignment in partition 1.
    let (p0_1, p1_1) = split_data_partitions(&stream, 1).expect("split 1");
    let (p0_127, p1_127) = split_data_partitions(&stream, 127).expect("split 127");
    assert!(p0_1.len() < p1_1.len());
    assert!(p0_127.len() > p1_127.len());
}

#[test]
fn decode_data_partitioned_matches_the_plain_decode() {
    for (name, stream) in corpus() {
        let plain = decode_video_sequence(&stream).expect("plain decode");
        for pb in [3u8, 65] {
            let (p0, p1) = split_data_partitions(&stream, pb).expect("split");
            let via = decode_data_partitioned(&p0, &p1)
                .unwrap_or_else(|e| panic!("{name} pb {pb}: partitioned decode failed: {e:?}"));
            assert_eq!(via.len(), plain.len(), "{name} pb {pb}: frame count");
            for (a, b) in plain.iter().zip(via.iter()) {
                assert_eq!(a.temporal_reference, b.temporal_reference);
                assert_eq!(
                    a.frame.y.samples(),
                    b.frame.y.samples(),
                    "{name} pb {pb}: luma"
                );
                assert_eq!(
                    a.frame.cb.samples(),
                    b.frame.cb.samples(),
                    "{name} pb {pb}: cb"
                );
                assert_eq!(
                    a.frame.cr.samples(),
                    b.frame.cr.samples(),
                    "{name} pb {pb}: cr"
                );
            }
        }
    }
}

#[test]
fn hostile_partition_pairs_are_rejected_not_panicked() {
    let (_, stream) = corpus()
        .into_iter()
        .find(|(n, _)| n == "mpeg2-ibbp-96x64.m2v")
        .expect("ibbp fixture");
    let (p0, p1) = split_data_partitions(&stream, 64).expect("split");
    // Partition 0 twice: the second copy carries priority_breakpoint 64, not 0.
    assert!(merge_data_partitions(&p0, &p0).is_err());
    // Swapped: layer ids / breakpoints disagree.
    assert!(merge_data_partitions(&p1, &p0).is_err());
    // A non-scalable stream as partition 0: no scalable extension.
    assert!(merge_data_partitions(&stream, &p1).is_err());
    // Truncated partition 1: error, never a panic.
    for cut in [p1.len() / 3, p1.len() / 2, p1.len() - 1] {
        let _ = merge_data_partitions(&p0, &p1[..cut]);
    }
    for cut in [p0.len() / 3, p0.len() / 2, p0.len() - 1] {
        let _ = merge_data_partitions(&p0[..cut], &p1);
    }
    // MPEG-1 streams cannot be partitioned (no sequence_extension).
    let m1v = std::fs::read(format!(
        "{}/tests/fixtures/conformance/mpeg1-ibbp-96x64.m1v",
        env!("CARGO_MANIFEST_DIR")
    ))
    .expect("mpeg1 fixture");
    assert!(split_data_partitions(&m1v, 64).is_err());
    // Reserved breakpoints.
    assert!(split_data_partitions(&stream, 0).is_err());
    assert!(split_data_partitions(&stream, 4).is_err());
    assert!(split_data_partitions(&stream, 63).is_err());
    assert!(split_data_partitions(&stream, 128).is_err());
}
