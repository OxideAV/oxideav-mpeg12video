//! Decode an MPEG-1 / MPEG-2 video elementary stream to raw planar
//! YCbCr, one frame after another in display order (§6.1.1.11).
//!
//! ```text
//! decode_to_yuv <input.m2v> <output.yuv>
//! ```
//!
//! The output layout is the common "rawvideo" planar convention: for
//! each frame the full Y plane (width × height bytes, row-major), then
//! the Cb plane, then the Cr plane, at the coded picture dimensions.
//! This makes the output byte-comparable against any black-box
//! reference decoder's rawvideo output for the same stream.

use std::io::Write;

use oxideav_mpeg12video::decode_video_sequence;

fn main() {
    let mut args = std::env::args().skip(1);
    let (input, output) = match (args.next(), args.next()) {
        (Some(i), Some(o)) => (i, o),
        _ => {
            eprintln!("usage: decode_to_yuv <input.m2v> <output.yuv>");
            std::process::exit(2);
        }
    };

    let stream = std::fs::read(&input).unwrap_or_else(|e| {
        eprintln!("read {input}: {e}");
        std::process::exit(1);
    });

    let frames = decode_video_sequence(&stream).unwrap_or_else(|e| {
        eprintln!("decode {input}: {e:?}");
        std::process::exit(1);
    });

    let mut out = std::io::BufWriter::new(std::fs::File::create(&output).unwrap_or_else(|e| {
        eprintln!("create {output}: {e}");
        std::process::exit(1);
    }));

    for decoded in &frames {
        let fb = &decoded.frame;
        out.write_all(fb.y.samples()).unwrap();
        out.write_all(fb.cb.samples()).unwrap();
        out.write_all(fb.cr.samples()).unwrap();
    }
    out.flush().unwrap();

    eprintln!(
        "decoded {} frame(s), {}x{} {:?}",
        frames.len(),
        frames.first().map(|f| f.frame.width).unwrap_or(0),
        frames.first().map(|f| f.frame.height).unwrap_or(0),
        frames.first().map(|f| f.frame.chroma_format),
    );
}
