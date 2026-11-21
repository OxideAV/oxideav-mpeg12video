//! Encode raw planar YCbCr to an MPEG-1 / MPEG-2 video elementary
//! stream through the runtime encoder and its typed option schema.
//!
//! ```text
//! encode_yuv <mpeg1video|mpeg2video> <width> <height> <420|422|444> \
//!            <input.yuv> <output.es> [key=value ...]
//! ```
//!
//! The input holds whole frames in the "rawvideo" planar layout (Y,
//! then Cb, then Cr). `frame_rate=<num>/<den>` and `bit_rate=<bit/s>`
//! set the stream parameters, `threads=<n>` grants a thread budget;
//! every other `key=value` is an encoder option (see
//! `Mpeg12EncoderOptions`). Under `data_partitioning` the second
//! partition is written to `<output.es>.p1`.

use oxideav_core::{
    CodecId, CodecParameters, Error, Frame, PixelFormat, Rational, VideoFrame, VideoPlane,
};
use oxideav_mpeg12video::make_encoder;

fn fail(msg: &str) -> ! {
    eprintln!("{msg}");
    std::process::exit(1);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 6 {
        fail(
            "usage: encode_yuv <mpeg1video|mpeg2video> <width> <height> <420|422|444> <input.yuv> <output.es> [key=value ...]",
        );
    }
    let width: usize = args[1].parse().unwrap_or_else(|_| fail("bad width"));
    let height: usize = args[2].parse().unwrap_or_else(|_| fail("bad height"));
    let (format, cw, ch) = match args[3].as_str() {
        "420" => (PixelFormat::Yuv420P, width.div_ceil(2), height.div_ceil(2)),
        "422" => (PixelFormat::Yuv422P, width.div_ceil(2), height),
        "444" => (PixelFormat::Yuv444P, width, height),
        _ => fail("chroma format must be 420, 422 or 444"),
    };
    let input = std::fs::read(&args[4]).unwrap_or_else(|e| fail(&format!("read {}: {e}", args[4])));

    let mut threads = 1usize;
    let mut params = CodecParameters::video(CodecId::new(args[0].as_str()));
    params.width = Some(width as u32);
    params.height = Some(height as u32);
    params.pixel_format = Some(format);
    for kv in &args[6..] {
        let (k, v) = kv
            .split_once('=')
            .unwrap_or_else(|| fail("options are key=value"));
        match k {
            "frame_rate" => {
                let (n, d) = v.split_once('/').unwrap_or((v, "1"));
                params.frame_rate = Some(Rational::new(
                    n.parse().unwrap_or_else(|_| fail("bad frame_rate")),
                    d.parse().unwrap_or_else(|_| fail("bad frame_rate")),
                ));
            }
            "bit_rate" => {
                params.bit_rate = Some(v.parse().unwrap_or_else(|_| fail("bad bit_rate")));
            }
            "threads" => threads = v.parse().unwrap_or_else(|_| fail("bad threads")),
            _ => {
                params.options.insert(k, v);
            }
        }
    }

    let mut encoder = make_encoder(&params).unwrap_or_else(|e| fail(&format!("encoder: {e:?}")));
    encoder.set_execution_context(&oxideav_core::ExecutionContext::with_threads(threads));
    let frame_bytes = width * height + 2 * cw * ch;
    for (t, raw) in input.chunks_exact(frame_bytes).enumerate() {
        let (y, c) = raw.split_at(width * height);
        let (cb, cr) = c.split_at(cw * ch);
        let plane = |stride: usize, data: &[u8]| VideoPlane {
            stride,
            data: data.to_vec(),
        };
        let frame = VideoFrame {
            pts: Some(t as i64),
            planes: vec![plane(width, y), plane(cw, cb), plane(cw, cr)],
        };
        encoder
            .send_frame(&Frame::Video(frame))
            .unwrap_or_else(|e| fail(&format!("send_frame: {e:?}")));
    }
    encoder
        .flush()
        .unwrap_or_else(|e| fail(&format!("flush: {e:?}")));
    let partitioned = params
        .options
        .get("data_partitioning")
        .is_some_and(|v| v != "0");
    let mut outputs: Vec<Vec<u8>> = Vec::new();
    loop {
        match encoder.receive_packet() {
            // Two partition packets, or one packet per coded frame.
            Ok(packet) if partitioned || outputs.is_empty() => outputs.push(packet.data),
            Ok(packet) => outputs[0].extend_from_slice(&packet.data),
            Err(Error::Eof) => break,
            Err(e) => fail(&format!("receive_packet: {e:?}")),
        }
    }
    for (index, data) in outputs.iter().enumerate() {
        let path = if index == 0 {
            args[5].clone()
        } else {
            format!("{}.p{index}", args[5])
        };
        std::fs::write(&path, data).unwrap_or_else(|e| fail(&format!("write {path}: {e}")));
    }
}
