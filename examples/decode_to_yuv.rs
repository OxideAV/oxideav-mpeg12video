//! Decode an MPEG-1 / MPEG-2 video elementary stream to raw planar
//! YCbCr, one frame after another in display order (§6.1.1.11).
//!
//! ```text
//! decode_to_yuv <input.m2v> <output.yuv> [threads]
//! ```
//!
//! `threads` grants a thread budget (default 1: serial).
//!
//! The output layout is the common "rawvideo" planar convention: for
//! each frame the full Y plane (width × height bytes, row-major), then
//! the Cb plane, then the Cr plane, at the coded picture dimensions.
//! This makes the output byte-comparable against any black-box
//! reference decoder's rawvideo output for the same stream.

use std::io::Write;

use oxideav_mpeg12video::SequenceDecoder;

fn main() {
    let mut args = std::env::args().skip(1);
    let (input, output) = match (args.next(), args.next()) {
        (Some(i), Some(o)) => (i, o),
        _ => {
            eprintln!("usage: decode_to_yuv <input.m2v> <output.yuv> [threads]");
            std::process::exit(2);
        }
    };

    let stream = std::fs::read(&input).unwrap_or_else(|e| {
        eprintln!("read {input}: {e}");
        std::process::exit(1);
    });

    let threads = args.next().and_then(|t| t.parse().ok()).unwrap_or(1);
    let mut decoder = SequenceDecoder::new().with_threads(threads);
    decoder.push(&stream);
    decoder.finish();
    let mut frames = Vec::new();
    loop {
        match decoder.next_frame() {
            Ok(Some(frame)) => frames.push(frame),
            Ok(None) => break,
            Err(e) => {
                eprintln!("decode {input}: {e:?}");
                std::process::exit(1);
            }
        }
    }

    let mut out = std::io::BufWriter::new(std::fs::File::create(&output).unwrap_or_else(|e| {
        eprintln!("create {output}: {e}");
        std::process::exit(1);
    }));

    for decoded in &frames {
        let fb = &decoded.frame;
        let (cw, ch) = fb.visible_chroma_dims();
        out.write_all(&fb.y.packed_rect(fb.width, fb.height))
            .unwrap();
        out.write_all(&fb.cb.packed_rect(cw, ch)).unwrap();
        out.write_all(&fb.cr.packed_rect(cw, ch)).unwrap();
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
