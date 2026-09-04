//! `vertical_size > 2800` end to end: §6.2.4 gates the 3-bit
//! `slice_vertical_position_extension` on the sequence `vertical_size`,
//! and §6.3.16 splits the macroblock row as `mb_row = (extension << 7)
//! + slice_vertical_position - 1` with `slice_vertical_position` in
//! `1..=128`. The encoders emit it, the frame / field decode drivers
//! honour it, and the §7.10 data-partitioning split / merge carries it
//! in both partitions (it precedes `priority_breakpoint`).

use oxideav_mpeg12video::sequence_extension::ChromaFormat;
use oxideav_mpeg12video::slice_header::{SliceContext, SliceHeader};
use oxideav_mpeg12video::{
    decode_data_partitioned, decode_video_sequence, encode_display_order_gop_sequence,
    encode_field_display_order_gop_sequence, merge_data_partitions, split_data_partitions,
    FrameBuffer, IntraPictureParams,
};

const W: usize = 16;
/// 176 macroblock rows: the §6.3.16 extension reaches 1 (rows 128..).
const H: usize = 2816;

fn frame_at(t: usize) -> FrameBuffer {
    let mut f = FrameBuffer::new(W, H, ChromaFormat::Yuv420);
    for y in 0..H {
        for x in 0..W {
            let v = 40 + ((x * 9 + y * 3 + t * 5) % 160);
            f.y.put_sample(x, y, v as u8);
        }
    }
    for y in 0..H / 2 {
        for x in 0..W / 2 {
            f.cb.put_sample(x, y, (100 + (x + y + t) % 50) as u8);
            f.cr.put_sample(x, y, (150u8).saturating_sub(((y + 2 * t) % 50) as u8));
        }
    }
    f
}

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

/// Every slice start code position in `stream`.
fn slice_offsets(stream: &[u8]) -> Vec<usize> {
    stream
        .windows(4)
        .enumerate()
        .filter(|(_, w)| w[0] == 0 && w[1] == 0 && w[2] == 1 && (0x01..=0xAF).contains(&w[3]))
        .map(|(i, _)| i)
        .collect()
}

fn luma_mae(a: &FrameBuffer, b: &FrameBuffer) -> f64 {
    let mut total = 0u64;
    for y in 0..H {
        for x in 0..W {
            total += u64::from(a.y.get(x, y).unwrap().abs_diff(b.y.get(x, y).unwrap()));
        }
    }
    total as f64 / (W * H) as f64
}

#[test]
fn tall_frame_pictures_carry_the_extension_and_roundtrip() {
    let frames = [frame_at(0), frame_at(1)];
    let stream = encode_display_order_gop_sequence(&frames, 0, 1, params(true), 6, 2, 2)
        .expect("tall I P encode");

    // Two pictures of 176 slices each.
    let offsets = slice_offsets(&stream);
    assert_eq!(offsets.len(), 2 * (H / 16), "one slice per macroblock row");
    let ctx = SliceContext::non_scalable(H as u32);
    // Rows 0..=127 use extension 0; row 128 wraps slice_vertical_position
    // back to 1 with extension 1 (§6.3.16).
    let row0 = SliceHeader::parse(&stream[offsets[0]..], ctx).unwrap();
    assert_eq!(row0.slice_vertical_position, 1);
    assert_eq!(row0.slice_vertical_position_extension, Some(0));
    assert_eq!(row0.mb_row(), 0);
    let row127 = SliceHeader::parse(&stream[offsets[127]..], ctx).unwrap();
    assert_eq!(row127.slice_vertical_position, 128);
    assert_eq!(row127.slice_vertical_position_extension, Some(0));
    assert_eq!(row127.mb_row(), 127);
    let row128 = SliceHeader::parse(&stream[offsets[128]..], ctx).unwrap();
    assert_eq!(row128.slice_vertical_position, 1);
    assert_eq!(row128.slice_vertical_position_extension, Some(1));
    assert_eq!(row128.mb_row(), 128);
    let last = SliceHeader::parse(&stream[offsets[175]..], ctx).unwrap();
    assert_eq!(last.mb_row(), 175);

    let decoded = decode_video_sequence(&stream).expect("tall stream decodes");
    assert_eq!(decoded.len(), 2);
    for (d, input) in decoded.iter().zip(&frames) {
        assert_eq!((d.frame.width, d.frame.height), (W, H));
        let mae = luma_mae(&d.frame, input);
        assert!(mae < 6.0, "luma MAE {mae:.2}");
    }
}

#[test]
fn tall_field_pictures_gate_the_extension_on_the_frame_height() {
    // Fields are 16x1408 (88 rows each) — below 2800 lines on their
    // own, but §6.2.4 gates the extension on the *sequence*
    // vertical_size, so every field slice still carries it.
    let frames = [frame_at(0), frame_at(1)];
    let stream = encode_field_display_order_gop_sequence(&frames, 0, 1, &params(false), 6, 2, 2)
        .expect("tall field encode");
    let offsets = slice_offsets(&stream);
    assert_eq!(
        offsets.len(),
        4 * (H / 32),
        "one slice per field macroblock row"
    );
    let ctx = SliceContext::non_scalable(H as u32);
    let first = SliceHeader::parse(&stream[offsets[0]..], ctx).unwrap();
    assert_eq!(first.slice_vertical_position_extension, Some(0));
    let last_of_top = SliceHeader::parse(&stream[offsets[H / 32 - 1]..], ctx).unwrap();
    assert_eq!(last_of_top.mb_row(), (H / 32 - 1) as u32);

    let decoded = decode_video_sequence(&stream).expect("tall field stream decodes");
    assert_eq!(decoded.len(), 2);
    for (d, input) in decoded.iter().zip(&frames) {
        assert_eq!((d.frame.width, d.frame.height), (W, H));
        let mae = luma_mae(&d.frame, input);
        assert!(mae < 6.0, "luma MAE {mae:.2}");
    }
}

#[test]
fn tall_pictures_partition_and_merge_byte_exactly() {
    let frames = [frame_at(0), frame_at(1)];
    let stream = encode_display_order_gop_sequence(&frames, 0, 1, params(true), 6, 2, 2)
        .expect("tall I P encode");
    for breakpoint in [1u8, 2, 3, 64, 70] {
        let (p0, p1) = split_data_partitions(&stream, breakpoint).expect("tall stream splits");
        // Both partitions carry the extension ahead of priority_breakpoint.
        let ctx = SliceContext {
            vertical_size: H as u32,
            priority_breakpoint_present: true,
        };
        let s0 = slice_offsets(&p0);
        let s1 = slice_offsets(&p1);
        assert_eq!(s0.len(), 2 * (H / 16));
        assert_eq!(s1.len(), 2 * (H / 16));
        let h0 = SliceHeader::parse(&p0[s0[130]..], ctx).unwrap();
        let h1 = SliceHeader::parse(&p1[s1[130]..], ctx).unwrap();
        assert_eq!(h0.slice_vertical_position_extension, Some(1));
        assert_eq!(h1.slice_vertical_position_extension, Some(1));
        assert_eq!(h0.mb_row(), 130);
        assert_eq!(h0.priority_breakpoint, Some(breakpoint));
        assert_eq!(h1.priority_breakpoint, Some(0));

        let merged = merge_data_partitions(&p0, &p1).expect("tall partitions merge");
        assert_eq!(
            merged, stream,
            "breakpoint {breakpoint}: merge is byte-exact"
        );
        let via_pair = decode_data_partitioned(&p0, &p1).expect("pair decodes");
        let direct = decode_video_sequence(&stream).expect("direct decode");
        assert_eq!(via_pair.len(), direct.len());
        for (a, b) in via_pair.iter().zip(&direct) {
            assert_eq!(a.frame.y.samples(), b.frame.y.samples());
        }
    }
}
