//! Live black-box interoperability matrix.
//!
//! Two sweeps against a black-box encoder / decoder binary, both generated in a
//! temporary directory at run time (nothing is committed) and both
//! **skipped when the binary is absent**:
//!
//! * **foreign → ours** — the black-box MPEG-1 / MPEG-2 encoders
//!   produce streams over every option of theirs that changes the
//!   bitstream (B-picture counts, quantiser scales, sizes from 2×2 to
//!   beyond 4096 columns, bit rates, open / closed GOPs, slice
//!   structures, 4:2:0 / 4:2:2, intra DC precision, alternate scan,
//!   intra VLC format, non-linear quantiser, custom matrices,
//!   interlaced field DCT / field prediction with both field orders,
//!   mid-stream sequence changes); this crate's decoder and the
//!   black-box decoder must agree on the frame count and on every
//!   sample within the stated bound.
//! * **ours → foreign** — this crate's encoders run over every option
//!   of [`oxideav_mpeg12video::Mpeg12EncoderOptions`]; the black-box
//!   decoder must accept each stream without a diagnostic and
//!   reproduce this crate's own decode within the same bound.
//!
//! # The bound
//!
//! ISO/IEC 13818-2 Annex A / ISO/IEC 11172-2 Annex A specify the 8×8
//! inverse transform only to IEEE Std 1180-1990 accuracy (peak error 1
//! per sample against the ideal transform), so two conforming decoders
//! are not bit-exact with each other: an intra picture may differ by
//! one or two levels, and the difference accumulates along a chain of
//! predicted pictures until the next intra picture (the reason Annex A
//! requires a refresh at least every 132 predicted pictures). The
//! matrix therefore holds intra-only streams to `|Δ| ≤ 2`, ordinary
//! GOPs (≤ 15 pictures) to `|Δ| ≤ 3`, and the long-GOP rows (120–150
//! predicted pictures without a refresh) to `|Δ| ≤ 8` — the black-box
//! decoder's own selectable transforms differ from one another by
//! 8–16 levels on those same streams.
//!
//! A debug build runs a reduced matrix (the encoders are slow without
//! optimisation); run with `--release` for the whole one.

use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

use oxideav_core::{
    CodecId, CodecParameters, Error, Frame, PixelFormat, Rational, VideoFrame, VideoPlane,
};
use oxideav_mpeg12video::{decode_video_sequence, make_encoder};

/// Executable name of the black-box encoder / decoder.
const BLACK_BOX: &str = "ffmpeg";

/// Whether the black-box binary runs here.
fn have_black_box() -> bool {
    Command::new(BLACK_BOX)
        .arg("-version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

/// A temporary directory removed on drop.
struct Scratch(PathBuf);

impl Scratch {
    fn new(tag: &str) -> Self {
        let dir = std::env::temp_dir().join(format!(
            "oxideav-mpeg12video-matrix-{tag}-{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("create scratch directory");
        Self(dir)
    }

    fn path(&self, name: &str) -> PathBuf {
        self.0.join(name)
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// Run the black-box binary with `args`; `Ok(diagnostics)` on success.
fn black_box(args: &[String]) -> Result<String, String> {
    let out = Command::new(BLACK_BOX)
        .args(["-hide_banner", "-nostdin", "-y", "-loglevel", "warning"])
        .args(args)
        .output()
        .map_err(|e| format!("spawn: {e}"))?;
    let stderr = String::from_utf8_lossy(&out.stderr).into_owned();
    if out.status.success() {
        Ok(stderr)
    } else {
        Err(stderr.lines().last().unwrap_or("failed").to_string())
    }
}

fn s(v: &str) -> String {
    v.to_string()
}

fn path_arg(p: &Path) -> String {
    p.to_string_lossy().into_owned()
}

/// Diagnostics of a black-box decode that say something about the
/// stream (the reader's duration estimate and its slice-thread count
/// notice do not).
fn stream_diagnostics(stderr: &str) -> Vec<String> {
    stderr
        .lines()
        .filter(|l| {
            !l.trim().is_empty()
                && !l.contains("Estimating duration")
                && !l.contains("too many threads/slices")
                && !l.contains("detected only with low score")
        })
        .map(str::to_string)
        .collect()
}

/// Decode `stream` with this crate into concatenated planar frames
/// (Y, Cb, Cr per frame at the visible size); returns the frame count.
fn decode_ours(stream: &[u8]) -> Result<(usize, Vec<u8>), String> {
    let frames = decode_video_sequence(stream).map_err(|e| format!("our decoder: {e:?}"))?;
    let mut out = Vec::new();
    for decoded in &frames {
        let fb = &decoded.frame;
        let (cw, ch) = fb.visible_chroma_dims();
        out.extend_from_slice(&fb.y.packed_rect(fb.width, fb.height));
        out.extend_from_slice(&fb.cb.packed_rect(cw, ch));
        out.extend_from_slice(&fb.cr.packed_rect(cw, ch));
    }
    Ok((frames.len(), out))
}

/// Decode `stream_path` with the black-box decoder to raw planar
/// frames; returns the bytes and the decoder's stream diagnostics.
fn decode_reference(stream_path: &Path, raw_path: &Path) -> Result<(Vec<u8>, Vec<String>), String> {
    let stderr = black_box(&[
        s("-i"),
        path_arg(stream_path),
        s("-fps_mode"),
        s("passthrough"),
        s("-f"),
        s("rawvideo"),
        path_arg(raw_path),
    ])
    .map_err(|e| format!("black-box decoder: {e}"))?;
    let bytes = std::fs::read(raw_path).map_err(|e| format!("read reference decode: {e}"))?;
    let _ = std::fs::remove_file(raw_path);
    Ok((bytes, stream_diagnostics(&stderr)))
}

/// Largest absolute sample difference, or an error on a size mismatch.
fn compare(ours: &[u8], reference: &[u8], bound: u8) -> Result<u8, String> {
    if ours.len() != reference.len() {
        return Err(format!(
            "decoded sizes differ: ours {} bytes, black-box {} bytes",
            ours.len(),
            reference.len()
        ));
    }
    let max = ours
        .iter()
        .zip(reference)
        .map(|(a, b)| a.abs_diff(*b))
        .max()
        .unwrap_or(0);
    if max > bound {
        return Err(format!("max |delta| {max} exceeds the bound {bound}"));
    }
    Ok(max)
}

/// Run `check` over `cases` on four threads; returns `name: reason`
/// for every failing case.
fn run_matrix<C: Sync>(
    cases: &[C],
    name: impl Fn(&C) -> &str + Sync,
    check: impl Fn(&C) -> Result<(), String> + Sync,
) -> Vec<String> {
    let next = AtomicUsize::new(0);
    let failures = Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for _ in 0..4 {
            scope.spawn(|| {
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    let Some(case) = cases.get(i) else { break };
                    if let Err(reason) = check(case) {
                        failures
                            .lock()
                            .unwrap()
                            .push(format!("{}: {reason}", name(case)));
                    }
                }
            });
        }
    });
    let mut failures = failures.into_inner().unwrap();
    failures.sort();
    failures
}

// ---------------------------------------------------------------------
// foreign → ours
// ---------------------------------------------------------------------

/// One black-box encode.
struct Foreign {
    name: String,
    mpeg1: bool,
    size: &'static str,
    pix_fmt: &'static str,
    frames: u32,
    /// Black-box encoder arguments.
    enc: String,
    /// Black-box picture source.
    source: &'static str,
    /// Black-box filter chain ahead of the encoder (may be empty).
    filter: String,
    rate: &'static str,
    /// Allowed `|Δ|` (see the module docs).
    bound: u8,
}

const INTRA_BOUND: u8 = 2;
const GOP_BOUND: u8 = 3;
const LONG_GOP_BOUND: u8 = 8;

fn foreign_cases(full: bool) -> Vec<Foreign> {
    let mut cases = Vec::new();
    let mut add = |name: String,
                   mpeg1: bool,
                   size: &'static str,
                   pix_fmt: &'static str,
                   frames: u32,
                   enc: String,
                   filter: String,
                   rate: &'static str,
                   bound: u8| {
        cases.push(Foreign {
            name,
            mpeg1,
            size,
            pix_fmt,
            frames,
            enc,
            source: "testsrc2",
            filter,
            rate,
            bound,
        });
    };
    let matrices = {
        let intra: Vec<String> = (0..64).map(|i| (8 + (i * 7) % 40).to_string()).collect();
        let inter: Vec<String> = (0..64).map(|i| (16 + (i * 5) % 50).to_string()).collect();
        format!(
            "-intra_matrix {} -inter_matrix {}",
            intra.join(","),
            inter.join(",")
        )
    };
    for mpeg1 in [true, false] {
        let t = if mpeg1 { "mpeg1" } else { "mpeg2" };
        for bf in [0, 1, 2, 3, 4, 6] {
            add(
                format!("{t}-bf{bf}"),
                mpeg1,
                "176x144",
                "yuv420p",
                30,
                format!("-bf {bf} -g 12 -q:v 4"),
                s(""),
                "25",
                GOP_BOUND,
            );
        }
        for q in [1, 2, 3, 8, 16, 31] {
            add(
                format!("{t}-q{q}"),
                mpeg1,
                "176x144",
                "yuv420p",
                14,
                format!("-bf 2 -g 12 -q:v {q}"),
                s(""),
                "25",
                GOP_BOUND,
            );
            add(
                format!("{t}-intra-q{q}"),
                mpeg1,
                "176x144",
                "yuv420p",
                4,
                format!("-g 1 -q:v {q}"),
                s(""),
                "25",
                INTRA_BOUND,
            );
        }
        let mut sizes = vec![
            "16x16", "2x2", "18x18", "8x40", "100x62", "130x98", "352x240", "352x288", "98x62",
            "99x61", "17x33",
        ];
        if full {
            sizes.extend(["720x576", "1920x1080", "4080x2160"]);
            sizes.push(if mpeg1 { "34x1202" } else { "34x2818" });
            if !mpeg1 {
                // horizontal_size_extension / vertical_size_extension.
                sizes.extend(["4098x34", "4112x2304"]);
            }
        }
        for size in sizes {
            let big = size.split('x').next().unwrap().parse::<u32>().unwrap() >= 1000;
            add(
                format!("{t}-sz{size}"),
                mpeg1,
                size,
                "yuv420p",
                if big { 4 } else { 7 },
                s("-bf 2 -g 6 -q:v 5"),
                s(""),
                "25",
                GOP_BOUND,
            );
        }
        for br in ["20k", "64k", "300k", "4M"] {
            add(
                format!("{t}-br{br}"),
                mpeg1,
                "352x288",
                "yuv420p",
                25,
                format!("-bf 2 -g 12 -b:v {br} -maxrate {br} -bufsize 1M"),
                s(""),
                "25",
                GOP_BOUND,
            );
        }
        add(
            format!("{t}-br40M"),
            mpeg1,
            "352x288",
            "yuv420p",
            25,
            s("-bf 2 -g 12 -b:v 40M"),
            s(""),
            "25",
            GOP_BOUND,
        );
        add(
            format!("{t}-opengop"),
            mpeg1,
            "176x144",
            "yuv420p",
            40,
            s("-bf 2 -g 9 -q:v 4 -flags -cgop"),
            s(""),
            "25",
            GOP_BOUND,
        );
        add(
            format!("{t}-closedgop"),
            mpeg1,
            "176x144",
            "yuv420p",
            40,
            s("-bf 2 -g 9 -q:v 4 -flags +cgop -sc_threshold 1000000000"),
            s(""),
            "25",
            GOP_BOUND,
        );
        if full {
            for (tag, enc, filter) in [
                ("longgop", "-bf 0 -g 300 -q:v 3", ""),
                ("longgop-b", "-bf 2 -g 300 -q:v 3", ""),
                (
                    "longgop-noise",
                    "-bf 0 -g 300 -q:v 2",
                    "noise=alls=20:allf=t",
                ),
            ] {
                add(
                    format!("{t}-{tag}"),
                    mpeg1,
                    "352x288",
                    "yuv420p",
                    150,
                    s(enc),
                    s(filter),
                    "25",
                    LONG_GOP_BOUND,
                );
            }
        }
        // Escape coding and saturation: noise at the finest and the
        // coarsest quantiser.
        for (tag, q) in [("noise-q1", "-q:v 1 -qmin 1"), ("noise-q31", "-q:v 31")] {
            add(
                format!("{t}-{tag}"),
                mpeg1,
                "352x288",
                "yuv420p",
                12,
                format!("-bf 2 -g 12 {q}"),
                s("noise=alls=60:allf=t"),
                "25",
                GOP_BOUND,
            );
        }
        add(
            format!("{t}-matrices"),
            mpeg1,
            "176x144",
            "yuv420p",
            14,
            format!("-bf 2 -g 12 -q:v 4 {matrices}"),
            s(""),
            "25",
            GOP_BOUND,
        );
        for (tag, enc) in [
            ("slices4", "-bf 2 -g 12 -q:v 4 -slices 4"),
            ("ps200", "-bf 2 -g 12 -q:v 4 -ps 200"),
            (
                "mbdrd",
                "-bf 2 -g 12 -q:v 4 -mbd rd -mpv_flags +qp_rd+cbp_rd+mv0 -trellis 1",
            ),
            (
                "aq",
                "-bf 2 -g 12 -b:v 500k -lumi_mask 0.1 -dark_mask 0.1 -scplx_mask 0.1 -tcplx_mask 0.1",
            ),
        ] {
            add(
                format!("{t}-{tag}"),
                mpeg1,
                "352x288",
                "yuv420p",
                14,
                s(enc),
                s(""),
                "25",
                GOP_BOUND,
            );
        }
        if full {
            add(
                format!("{t}-bigmv"),
                mpeg1,
                "720x576",
                "yuv420p",
                10,
                s("-bf 2 -g 12 -q:v 4 -me_range 1023 -dia_size 1040"),
                s("scroll=h=0.21:v=0.13"),
                "25",
                GOP_BOUND,
            );
        }
        add(
            format!("{t}-1frame"),
            mpeg1,
            "176x144",
            "yuv420p",
            1,
            s("-q:v 4"),
            s(""),
            "25",
            INTRA_BOUND,
        );
        add(
            format!("{t}-2frame"),
            mpeg1,
            "176x144",
            "yuv420p",
            2,
            s("-bf 2 -q:v 4"),
            s(""),
            "25",
            GOP_BOUND,
        );
        for rate in [
            "24000/1001",
            "24",
            "30000/1001",
            "30",
            "50",
            "60000/1001",
            "60",
        ] {
            add(
                format!("{t}-rate{}", rate.replace('/', "_")),
                mpeg1,
                "176x144",
                "yuv420p",
                8,
                s("-bf 1 -g 6 -q:v 4"),
                s(""),
                rate,
                GOP_BOUND,
            );
        }
        add(
            format!("{t}-gray"),
            mpeg1,
            "176x144",
            "yuv420p",
            8,
            s("-bf 2 -g 12 -q:v 4 -flags +gray"),
            s(""),
            "25",
            GOP_BOUND,
        );
    }

    // MPEG-2 only syntax.
    for dc in 0..4 {
        add(
            format!("mpeg2-dc{dc}"),
            false,
            "176x144",
            "yuv420p",
            14,
            format!("-bf 2 -g 12 -q:v 2 -intra_dc_precision {dc}"),
            s(""),
            "25",
            GOP_BOUND,
        );
        add(
            format!("mpeg2-dc{dc}-intra"),
            false,
            "176x144",
            "yuv420p",
            3,
            format!("-g 1 -q:v 2 -intra_dc_precision {dc}"),
            s(""),
            "25",
            INTRA_BOUND,
        );
    }
    for (tag, flags) in [
        ("alt", "-alternate_scan 1"),
        ("ivlc", "-intra_vlc 1"),
        ("nlq", "-non_linear_quant 1 -qmax 28"),
        (
            "all",
            "-alternate_scan 1 -intra_vlc 1 -non_linear_quant 1 -qmax 28 -intra_dc_precision 2",
        ),
    ] {
        for q in [2, 9, 28] {
            for pix_fmt in ["yuv420p", "yuv422p"] {
                add(
                    format!("mpeg2-{tag}-q{q}-{pix_fmt}"),
                    false,
                    "176x144",
                    pix_fmt,
                    14,
                    format!("-bf 2 -g 12 -q:v {q} {flags}"),
                    s(""),
                    "25",
                    GOP_BOUND,
                );
            }
        }
    }
    let mut sizes_422 = vec!["16x16", "18x18", "100x62", "352x288", "99x61"];
    if full {
        sizes_422.extend(["720x576", "1920x1080"]);
    }
    for size in sizes_422 {
        add(
            format!("mpeg2-422-sz{size}"),
            false,
            size,
            "yuv422p",
            if size == "1920x1080" { 4 } else { 7 },
            s("-bf 2 -g 6 -q:v 5"),
            s(""),
            "25",
            GOP_BOUND,
        );
    }
    add(
        s("mpeg2-422-intra-q1"),
        false,
        "176x144",
        "yuv422p",
        3,
        s("-g 1 -q:v 1 -qmin 1"),
        s("noise=alls=60:allf=t"),
        "25",
        INTRA_BOUND,
    );
    add(
        s("mpeg2-422-matrices"),
        false,
        "176x144",
        "yuv422p",
        14,
        format!("-bf 2 -g 12 -q:v 4 {matrices}"),
        s(""),
        "25",
        GOP_BOUND,
    );
    // Interlaced frame pictures: field DCT, field prediction, both
    // field orders.
    for pix_fmt in ["yuv420p", "yuv422p"] {
        for order in ["tff", "bff"] {
            let interlace = format!("interlace=scan={order},setfield={order}");
            for (tag, flags) in [
                ("ildct", "+ildct"),
                ("ilme", "+ilme"),
                ("ildct_ilme", "+ildct+ilme"),
            ] {
                for bf in [0, 2] {
                    let mut sizes = vec!["352x288", "100x62", "98x50"];
                    if full {
                        sizes.push("720x480");
                    }
                    for size in sizes {
                        add(
                            format!("mpeg2-i-{pix_fmt}-{order}-{tag}-bf{bf}-{size}"),
                            false,
                            size,
                            pix_fmt,
                            13,
                            format!("-bf {bf} -g 12 -q:v 3 -flags {flags}"),
                            interlace.clone(),
                            "50",
                            GOP_BOUND,
                        );
                    }
                }
            }
            if full {
                add(
                    format!("mpeg2-i-{pix_fmt}-{order}-all"),
                    false,
                    "720x576",
                    pix_fmt,
                    13,
                    s(
                        "-bf 2 -g 12 -q:v 2 -flags +ildct+ilme -alternate_scan 1 -intra_vlc 1 -non_linear_quant 1 -qmax 28 -intra_dc_precision 3 -mbd rd",
                    ),
                    format!("interlace=scan={order},noise=alls=10:allf=t,setfield={order}"),
                    "50",
                    GOP_BOUND,
                );
                add(
                    format!("mpeg2-i-{pix_fmt}-{order}-long"),
                    false,
                    "352x288",
                    pix_fmt,
                    120,
                    s("-bf 0 -g 300 -q:v 3 -flags +ildct+ilme"),
                    interlace.clone(),
                    "50",
                    LONG_GOP_BOUND,
                );
            }
        }
    }
    if full {
        add(
            s("mpeg2-i-1080"),
            false,
            "1920x1080",
            "yuv420p",
            5,
            s("-bf 2 -g 12 -q:v 4 -flags +ildct+ilme"),
            s("interlace=scan=tff,setfield=tff"),
            "50",
            GOP_BOUND,
        );
        add(
            s("mpeg2-422-high-1080"),
            false,
            "1920x1080",
            "yuv422p",
            4,
            s("-bf 2 -g 12 -b:v 80M"),
            s(""),
            "25",
            GOP_BOUND,
        );
        add(
            s("mpeg2-cbr-vbv"),
            false,
            "720x576",
            "yuv420p",
            30,
            s("-bf 2 -g 15 -b:v 6M -minrate 6M -maxrate 6M -bufsize 1835k"),
            s(""),
            "25",
            GOP_BOUND,
        );
    }
    add(
        s("mpeg1-cbr-vbv"),
        true,
        "352x288",
        "yuv420p",
        30,
        s("-bf 2 -g 15 -b:v 1150k -minrate 1150k -maxrate 1150k -bufsize 327680"),
        s(""),
        "25",
        GOP_BOUND,
    );
    add(
        s("mpeg2-seqdisp"),
        false,
        "176x144",
        "yuv420p",
        8,
        s(
            "-bf 2 -g 12 -q:v 4 -seq_disp_ext always -video_format pal -color_primaries bt470bg -color_trc gamma28 -colorspace bt470bg",
        ),
        s(""),
        "25",
        GOP_BOUND,
    );
    add(
        s("mpeg2-scan-offset-user-data"),
        false,
        "176x144",
        "yuv420p",
        8,
        s("-bf 2 -g 12 -q:v 4 -scan_offset 1"),
        s(""),
        "25",
        GOP_BOUND,
    );
    add(
        s("mpeg2-dropframe-timecode"),
        false,
        "176x144",
        "yuv420p",
        40,
        s("-bf 2 -g 12 -q:v 4 -drop_frame_timecode 1 -gop_timecode 00:59:59;28"),
        s(""),
        "30000/1001",
        GOP_BOUND,
    );
    add(
        s("mpeg2-telecined"),
        false,
        "352x240",
        "yuv420p",
        20,
        s("-bf 2 -g 12 -q:v 4 -flags +ildct+ilme"),
        s("telecine,setfield=tff"),
        "24000/1001",
        GOP_BOUND,
    );
    cases.push(Foreign {
        name: s("mpeg2-mandelbrot"),
        mpeg1: false,
        size: "352x288",
        pix_fmt: "yuv420p",
        frames: 30,
        enc: s("-bf 2 -g 15 -q:v 3"),
        source: "mandelbrot",
        filter: s(""),
        rate: "25",
        bound: GOP_BOUND,
    });
    cases.push(Foreign {
        name: s("mpeg1-mandelbrot"),
        mpeg1: true,
        size: "352x288",
        pix_fmt: "yuv420p",
        frames: 30,
        enc: s("-bf 2 -g 15 -q:v 3"),
        source: "mandelbrot",
        filter: s(""),
        rate: "25",
        bound: GOP_BOUND,
    });
    cases
}

/// The black-box picture source only makes even sizes: the size to
/// request from it and the crop that yields `width × height`.
fn even_source(width: u32, height: u32) -> (String, Option<String>) {
    let even = format!(
        "{}x{}",
        width.next_multiple_of(2),
        height.next_multiple_of(2)
    );
    let crop = (!width.is_multiple_of(2) || !height.is_multiple_of(2))
        .then(|| format!("crop={width}:{height}:0:0:exact=1"));
    (even, crop)
}

/// Encode `case` with the black-box encoder into `stream_path`.
fn encode_foreign(case: &Foreign, stream_path: &Path) -> Result<(), String> {
    let (width, height) = case.size.split_once('x').expect("WxH");
    let (width, height): (u32, u32) = (width.parse().unwrap(), height.parse().unwrap());
    let (even_size, crop) = even_source(width, height);
    let mut args = vec![
        s("-f"),
        s("lavfi"),
        s("-i"),
        format!("{}=size={even_size}:rate={}", case.source, case.rate),
    ];
    let mut filters = Vec::new();
    if !case.filter.is_empty() {
        filters.push(case.filter.clone());
    }
    filters.extend(crop);
    filters.push(format!("format={}", case.pix_fmt));
    args.extend([s("-vf"), filters.join(",")]);
    args.extend([s("-frames:v"), case.frames.to_string()]);
    args.extend([
        s("-c:v"),
        s(if case.mpeg1 {
            "mpeg1video"
        } else {
            "mpeg2video"
        }),
    ]);
    args.extend(case.enc.split_whitespace().map(str::to_string));
    args.extend([
        s("-f"),
        s(if case.mpeg1 {
            "mpeg1video"
        } else {
            "mpeg2video"
        }),
        path_arg(stream_path),
    ]);
    black_box(&args).map_err(|e| format!("black-box encoder: {e}"))?;
    Ok(())
}

fn check_foreign(scratch: &Scratch, case: &Foreign) -> Result<(), String> {
    let stream_path = scratch.path(&format!("{}.es", case.name));
    encode_foreign(case, &stream_path)?;
    let stream = std::fs::read(&stream_path).map_err(|e| format!("read stream: {e}"))?;
    let (reference, _) =
        decode_reference(&stream_path, &scratch.path(&format!("{}.yuv", case.name)))?;
    let _ = std::fs::remove_file(&stream_path);
    let (count, ours) = decode_ours(&stream)?;
    if count != case.frames as usize {
        return Err(format!("decoded {count} frames of {}", case.frames));
    }
    compare(&ours, &reference, case.bound).map(|_| ())
}

#[test]
fn foreign_streams_decode_within_the_idct_bound() {
    if !have_black_box() {
        eprintln!("skipping: black-box binary `{BLACK_BOX}` not found");
        return;
    }
    let full = !cfg!(debug_assertions);
    let mut cases = foreign_cases(full);
    if !full {
        cases.retain(|c| {
            let (w, h) = c.size.split_once('x').expect("WxH");
            w.parse::<u32>().unwrap() * h.parse::<u32>().unwrap() * c.frames <= 352 * 288 * 14
        });
    }
    let scratch = Scratch::new("foreign");
    let failures = run_matrix(&cases, |c| &c.name, |c| check_foreign(&scratch, c));
    eprintln!(
        "foreign -> ours: {} of {} streams pass",
        cases.len() - failures.len(),
        cases.len()
    );
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Sequence parameters that change mid-stream (§6.1.1.6: a new
/// `sequence_header()` may follow a `sequence_end_code`): picture
/// size, chroma format and the coding standard itself.
#[test]
fn foreign_sequence_changes_decode() {
    if !have_black_box() {
        eprintln!("skipping: black-box binary `{BLACK_BOX}` not found");
        return;
    }
    let scratch = Scratch::new("concat");
    let part = |name: &str, mpeg1: bool, size: &'static str, pix_fmt: &'static str| {
        let case = Foreign {
            name: s(name),
            mpeg1,
            size,
            pix_fmt,
            frames: 8,
            enc: s("-bf 2 -g 6 -q:v 4"),
            source: "testsrc2",
            filter: s(""),
            rate: "25",
            bound: GOP_BOUND,
        };
        let stream_path = scratch.path(&format!("{name}.es"));
        encode_foreign(&case, &stream_path).expect("black-box encode");
        let stream = std::fs::read(&stream_path).unwrap();
        let (reference, _) =
            decode_reference(&stream_path, &scratch.path(&format!("{name}.yuv"))).unwrap();
        (stream, reference)
    };
    let a = part("a", false, "176x144", "yuv420p");
    let b = part("b", false, "352x288", "yuv420p");
    let c = part("c", false, "176x144", "yuv422p");
    let d = part("d", true, "176x144", "yuv420p");
    let e = part("e", true, "320x240", "yuv420p");
    for (label, first, second) in [
        ("size change", &a, &b),
        ("chroma format change", &a, &c),
        ("MPEG-1 size change", &d, &e),
        ("MPEG-1 then MPEG-2", &d, &a),
        ("repeated sequence", &a, &a),
    ] {
        let stream = [first.0.as_slice(), second.0.as_slice()].concat();
        let reference = [first.1.as_slice(), second.1.as_slice()].concat();
        let (count, ours) = decode_ours(&stream).unwrap_or_else(|e| panic!("{label}: {e}"));
        assert_eq!(count, 16, "{label}");
        compare(&ours, &reference, GOP_BOUND).unwrap_or_else(|e| panic!("{label}: {e}"));
    }
}

// ---------------------------------------------------------------------
// ours → foreign
// ---------------------------------------------------------------------

/// One encode by this crate.
struct Ours {
    name: String,
    mpeg1: bool,
    width: usize,
    height: usize,
    /// `"420"`, `"422"` or `"444"`.
    chroma: &'static str,
    frames: u32,
    /// `key=value` encoder options (`frame_rate` / `bit_rate` set the
    /// stream parameters).
    options: String,
    /// Feed interlaced pictures (two fields a frame period apart).
    interlaced_source: bool,
    /// Temporal noise strength added to the source (0 = none).
    noise: u32,
    /// The black-box decoder has no support for the stream type (it
    /// emits no frames for ISO/IEC 11172-2 D-pictures): only this
    /// crate's own decode is checked.
    no_reference: bool,
}

fn ours_cases(full: bool) -> Vec<Ours> {
    let mut cases = Vec::new();
    let mut add = |name: String,
                   mpeg1: bool,
                   size: (usize, usize),
                   chroma: &'static str,
                   frames: u32,
                   options: String,
                   interlaced_source: bool,
                   noise: u32| {
        cases.push(Ours {
            name,
            mpeg1,
            width: size.0,
            height: size.1,
            chroma,
            frames,
            options,
            interlaced_source,
            noise,
            no_reference: false,
        });
    };
    const QCIF: (usize, usize) = (176, 144);
    const CIF: (usize, usize) = (352, 288);
    for mpeg1 in [true, false] {
        let t = if mpeg1 { "mpeg1" } else { "mpeg2" };
        for q in [1, 2, 6, 16, 31] {
            for noise in [0, 50] {
                add(
                    format!("{t}-q{q}-noise{noise}"),
                    mpeg1,
                    QCIF,
                    "420",
                    9,
                    format!("quantiser_scale_code={q}"),
                    false,
                    noise,
                );
            }
        }
        for b in 0..=5 {
            add(
                format!("{t}-b{b}"),
                mpeg1,
                QCIF,
                "420",
                14,
                format!("b_between={b} anchors_per_gop=3"),
                false,
                0,
            );
        }
        for anchors in [1, 2, 10] {
            add(
                format!("{t}-anchors{anchors}"),
                mpeg1,
                QCIF,
                "420",
                14,
                format!("anchors_per_gop={anchors}"),
                false,
                0,
            );
        }
        for f in 1..=7 {
            add(
                format!("{t}-fcode{f}"),
                mpeg1,
                QCIF,
                "420",
                9,
                format!("f_code={f}"),
                false,
                0,
            );
            add(
                format!("{t}-bfcode{f}"),
                mpeg1,
                QCIF,
                "420",
                9,
                format!("f_code=2 backward_f_code={f}"),
                false,
                0,
            );
        }
        let mut sizes = vec![
            (16, 16),
            (2, 2),
            (18, 18),
            (8, 40),
            (100, 62),
            (130, 98),
            (352, 240),
        ];
        sizes.extend([(99, 61), (17, 33)]);
        if full {
            // The sequence header's 12-bit size limit; ISO/IEC 11172-2
            // stops at 175 slice rows (2800 lines).
            sizes.extend([(720, 576), (1920, 1080), (4094, 34)]);
            sizes.push(if mpeg1 { (34, 2800) } else { (34, 4094) });
        }
        for size in sizes {
            add(
                format!("{t}-sz{}x{}", size.0, size.1),
                mpeg1,
                size,
                "420",
                if size.0 >= 1000 { 4 } else { 7 },
                s(""),
                false,
                0,
            );
        }
        for rate in [
            "24000/1001",
            "24",
            "25",
            "30000/1001",
            "30",
            "50",
            "60000/1001",
            "60",
        ] {
            add(
                format!("{t}-rate{}", rate.replace('/', "_")),
                mpeg1,
                QCIF,
                "420",
                9,
                format!("frame_rate={rate}"),
                false,
                0,
            );
        }
        for bit_rate in [300_000, 1_150_000, 4_000_000, 15_000_000] {
            // Noise at the lowest rate does not fit its buffer at any
            // quantiser (the encoder reports that instead of a stream).
            for noise in if bit_rate == 300_000 {
                &[0][..]
            } else {
                &[0, 40][..]
            } {
                let noise = *noise;
                add(
                    format!("{t}-cbr{bit_rate}-noise{noise}"),
                    mpeg1,
                    CIF,
                    "420",
                    30,
                    format!("rate_control=cbr bit_rate={bit_rate}"),
                    false,
                    noise,
                );
            }
        }
        add(
            format!("{t}-1frame"),
            mpeg1,
            QCIF,
            "420",
            1,
            s(""),
            false,
            0,
        );
        add(
            format!("{t}-2frame"),
            mpeg1,
            QCIF,
            "420",
            2,
            s(""),
            false,
            0,
        );
        if full {
            add(
                format!("{t}-long"),
                mpeg1,
                CIF,
                "420",
                120,
                s("b_between=0 anchors_per_gop=120 quantiser_scale_code=3"),
                false,
                10,
            );
        }
    }
    for chroma in ["420", "422", "444"] {
        for structure in ["frame", "field", "frame_field", "field_adaptive"] {
            // Field pictures need a frame height that is a multiple
            // of 32.
            let size = if structure.starts_with("field") {
                (176, 160)
            } else {
                (176, 144)
            };
            for b in [0, 2] {
                let base = format!("picture_structure={structure} b_between={b}");
                add(
                    format!("mpeg2-{chroma}-{structure}-b{b}"),
                    false,
                    size,
                    chroma,
                    9,
                    base.clone(),
                    true,
                    0,
                );
                add(
                    format!("mpeg2-{chroma}-{structure}-b{b}-flags"),
                    false,
                    size,
                    chroma,
                    9,
                    format!(
                        "{base} intra_vlc_format=true alternate_scan=true q_scale_type=true intra_dc_precision=3"
                    ),
                    true,
                    10,
                );
                if b == 0 && matches!(structure, "frame_field" | "field_adaptive") {
                    add(
                        format!("mpeg2-{chroma}-{structure}-dual-prime"),
                        false,
                        size,
                        chroma,
                        9,
                        format!("{base} dual_prime=true"),
                        true,
                        0,
                    );
                }
                if matches!(structure, "frame" | "field") {
                    add(
                        format!("mpeg2-{chroma}-{structure}-b{b}-cbr"),
                        false,
                        size,
                        chroma,
                        20,
                        format!("{base} rate_control=cbr bit_rate=800000"),
                        true,
                        0,
                    );
                }
            }
        }
        for option in [
            "intra_vlc_format=true",
            "alternate_scan=true",
            "q_scale_type=true",
            "intra_dc_precision=1",
            "intra_dc_precision=2",
            "intra_dc_precision=3",
            "skipped_macroblocks=true",
            "concealment_motion_vectors=true",
            "interlaced=true",
            "interlaced=true top_field_first=true",
            "interlaced=true top_field_first=true repeat_first_field=true",
            "interlaced=true pulldown=3:2",
            "repeat_first_field=true",
            "skipped_macroblocks=true concealment_motion_vectors=true intra_vlc_format=true alternate_scan=true q_scale_type=true",
        ] {
            for q in [2, 31] {
                add(
                    format!(
                        "mpeg2-{chroma}-{}-q{q}",
                        option.replace("=true", "").replace(' ', "+")
                    ),
                    false,
                    QCIF,
                    chroma,
                    10,
                    format!("{option} quantiser_scale_code={q}"),
                    false,
                    0,
                );
            }
        }
        let mut sizes = vec![(16, 16), (18, 18), (100, 62), (98, 50), (99, 61)];
        if full {
            sizes.extend([(720, 480), (32, 2880)]);
            if chroma == "420" {
                sizes.push((1920, 1080));
            }
        }
        for size in sizes {
            for (tag, options) in [
                ("frame-field", "picture_structure=frame_field"),
                ("interlaced", "interlaced=true top_field_first=true"),
            ] {
                add(
                    format!("mpeg2-{chroma}-{tag}-{}x{}", size.0, size.1),
                    false,
                    size,
                    chroma,
                    if size.0 >= 1000 { 3 } else { 7 },
                    s(options),
                    true,
                    0,
                );
            }
        }
        let mut sizes = vec![(16, 32), (48, 64)];
        if full {
            sizes.push((720, 480));
            if chroma == "420" {
                sizes.push((1920, 1088));
            }
        }
        for size in sizes {
            for structure in ["field", "field_adaptive"] {
                add(
                    format!("mpeg2-{chroma}-{structure}-{}x{}", size.0, size.1),
                    false,
                    size,
                    chroma,
                    if size.0 >= 1000 { 3 } else { 7 },
                    format!("picture_structure={structure}"),
                    true,
                    0,
                );
            }
        }
    }
    if full {
        add(
            s("mpeg2-fcode9"),
            false,
            (720, 576),
            "420",
            5,
            s("f_code=9"),
            false,
            0,
        );
        add(
            s("mpeg2-fcode8-bfcode9"),
            false,
            (720, 576),
            "420",
            5,
            s("f_code=8 backward_f_code=9"),
            false,
            0,
        );
    }
    cases.push(Ours {
        name: s("mpeg1-d-pictures"),
        mpeg1: true,
        width: 176,
        height: 144,
        chroma: "420",
        frames: 9,
        options: s("mpeg1_d_pictures=true"),
        interlaced_source: false,
        noise: 0,
        no_reference: true,
    });
    cases
}

/// `(chroma width, chroma height)` of a `width × height` picture.
fn chroma_dims(chroma: &str, width: usize, height: usize) -> (usize, usize) {
    match chroma {
        "420" => (width.div_ceil(2), height.div_ceil(2)),
        "422" => (width.div_ceil(2), height),
        _ => (width, height),
    }
}

/// Encode `source` (concatenated planar frames) with this crate.
fn encode_ours(case: &Ours, source: &[u8]) -> Result<Vec<u8>, String> {
    let (cw, ch) = chroma_dims(case.chroma, case.width, case.height);
    let mut params = CodecParameters::video(CodecId::new(if case.mpeg1 {
        "mpeg1video"
    } else {
        "mpeg2video"
    }));
    params.width = Some(case.width as u32);
    params.height = Some(case.height as u32);
    params.pixel_format = Some(match case.chroma {
        "420" => PixelFormat::Yuv420P,
        "422" => PixelFormat::Yuv422P,
        _ => PixelFormat::Yuv444P,
    });
    for option in case.options.split_whitespace() {
        let (key, value) = option.split_once('=').expect("key=value");
        match key {
            "frame_rate" => {
                let (n, d) = value.split_once('/').unwrap_or((value, "1"));
                params.frame_rate = Some(Rational::new(n.parse().unwrap(), d.parse().unwrap()));
            }
            "bit_rate" => params.bit_rate = Some(value.parse().unwrap()),
            _ => {
                params.options.insert(key, value);
            }
        }
    }
    let mut encoder = make_encoder(&params).map_err(|e| format!("make_encoder: {e:?}"))?;
    let frame_bytes = case.width * case.height + 2 * cw * ch;
    for (t, raw) in source.chunks_exact(frame_bytes).enumerate() {
        let (y, c) = raw.split_at(case.width * case.height);
        let (cb, cr) = c.split_at(cw * ch);
        let plane = |stride: usize, data: &[u8]| VideoPlane {
            stride,
            data: data.to_vec(),
        };
        let frame = VideoFrame {
            pts: Some(t as i64),
            planes: vec![plane(case.width, y), plane(cw, cb), plane(cw, cr)],
        };
        encoder
            .send_frame(&Frame::Video(frame))
            .map_err(|e| format!("send_frame: {e:?}"))?;
    }
    encoder.flush().map_err(|e| format!("flush: {e:?}"))?;
    let mut stream = Vec::new();
    loop {
        match encoder.receive_packet() {
            Ok(packet) => stream.extend_from_slice(&packet.data),
            Err(Error::Eof) => break,
            Err(e) => return Err(format!("receive_packet: {e:?}")),
        }
    }
    Ok(stream)
}

fn check_ours(scratch: &Scratch, case: &Ours) -> Result<(), String> {
    // The source pictures come from the black-box generator too.
    let source_path = scratch.path(&format!("{}.src.yuv", case.name));
    let mut filters = Vec::new();
    if case.interlaced_source {
        filters.push(s("interlace=scan=tff"));
    }
    if case.noise != 0 {
        filters.push(format!("noise=alls={}:allf=t", case.noise));
    }
    let (even_size, crop) = even_source(case.width as u32, case.height as u32);
    filters.extend(crop);
    filters.push(format!("format=yuv{}p", case.chroma));
    black_box(&[
        s("-f"),
        s("lavfi"),
        s("-i"),
        format!(
            "testsrc2=size={even_size}:rate={}",
            if case.interlaced_source { 50 } else { 25 }
        ),
        s("-vf"),
        filters.join(","),
        s("-frames:v"),
        case.frames.to_string(),
        s("-f"),
        s("rawvideo"),
        path_arg(&source_path),
    ])
    .map_err(|e| format!("source generation: {e}"))?;
    let source = std::fs::read(&source_path).map_err(|e| format!("read source: {e}"))?;
    let _ = std::fs::remove_file(&source_path);

    let stream = encode_ours(case, &source)?;
    let (count, ours) = decode_ours(&stream)?;
    if count != case.frames as usize {
        return Err(format!(
            "our decoder returned {count} frames of {}",
            case.frames
        ));
    }
    if case.no_reference {
        return Ok(());
    }
    let stream_path = scratch.path(&format!("{}.es", case.name));
    std::fs::write(&stream_path, &stream).map_err(|e| format!("write stream: {e}"))?;
    let (reference, diagnostics) =
        decode_reference(&stream_path, &scratch.path(&format!("{}.yuv", case.name)))?;
    let _ = std::fs::remove_file(&stream_path);
    if !diagnostics.is_empty() {
        return Err(format!(
            "black-box decoder diagnostics: {}",
            diagnostics.join(" | ")
        ));
    }
    let bound = if case.frames > 30 {
        LONG_GOP_BOUND
    } else {
        GOP_BOUND
    };
    compare(&ours, &reference, bound).map(|_| ())
}

#[test]
fn our_streams_decode_in_the_black_box_decoder() {
    if !have_black_box() {
        eprintln!("skipping: black-box binary `{BLACK_BOX}` not found");
        return;
    }
    let full = !cfg!(debug_assertions);
    let mut cases = ours_cases(full);
    if !full {
        // The motion searches are slow without optimisation.
        cases.retain(|c| c.width * c.height * c.frames as usize <= 100 * 62 * 10);
    }
    let scratch = Scratch::new("ours");
    let failures = run_matrix(&cases, |c| &c.name, |c| check_ours(&scratch, c));
    eprintln!(
        "ours -> foreign: {} of {} streams pass",
        cases.len() - failures.len(),
        cases.len()
    );
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// The repeat pictures the CBR controllers fall back to (every
/// macroblock a zero-vector copy, all but the row ends skipped) are
/// accepted by the black-box decoder and decode there to the reference
/// picture, in both syntaxes.
#[test]
fn repeat_pictures_decode_in_the_black_box_decoder() {
    use oxideav_core::bits::BitWriter;
    use oxideav_mpeg12video::sequence_extension::ChromaFormat;
    use oxideav_mpeg12video::{
        FrameBuffer, IntraPictureParams, Mpeg1PictureParams, Mpeg1SequenceParams,
    };

    if !have_black_box() {
        eprintln!("skipping: black-box binary `{BLACK_BOX}` not found");
        return;
    }
    let (w, h) = (80usize, 48usize);
    let mut frame = FrameBuffer::new(w, h, ChromaFormat::Yuv420);
    for y in 0..h {
        for x in 0..w {
            frame.y.put_sample(x, y, (20 + (x * 7 + y * 5) % 200) as u8);
        }
    }
    for y in 0..h / 2 {
        for x in 0..w / 2 {
            frame.cb.put_sample(x, y, (100 + x) as u8);
            frame.cr.put_sample(x, y, (160 - y) as u8);
        }
    }
    // Coded order I P B: display I B P, all three the same picture.
    let params = IntraPictureParams::new(
        w,
        h,
        ChromaFormat::Yuv420,
        true,
        0,
        false,
        false,
        false,
        true,
    );
    let mut mpeg2 = oxideav_mpeg12video::encode_intra_picture(&frame, params, 0, 4).unwrap();
    mpeg2.truncate(mpeg2.len() - 4);
    let mut bw = BitWriter::new();
    oxideav_mpeg12video::encode_repeat_p_picture(&mut bw, params, 2, 3).unwrap();
    oxideav_mpeg12video::encode_repeat_b_picture(&mut bw, params, 1, 3, 3).unwrap();
    mpeg2.extend_from_slice(&bw.finish());
    mpeg2.extend_from_slice(&[0, 0, 1, 0xB7]);

    let sequence = Mpeg1SequenceParams::default()
        .with_horizontal_size(w as u16)
        .with_vertical_size(h as u16);
    let mut mpeg1 = oxideav_mpeg12video::encode_mpeg1_intra_stream(&frame, &sequence, 4).unwrap();
    mpeg1.truncate(mpeg1.len() - 4);
    let picture = Mpeg1PictureParams::default().with_width(w).with_height(h);
    let mut bw = BitWriter::new();
    oxideav_mpeg12video::encode_mpeg1_repeat_p_picture(&mut bw, &picture, 2, 3).unwrap();
    oxideav_mpeg12video::encode_mpeg1_repeat_b_picture(&mut bw, &picture, 1, 3, 3).unwrap();
    mpeg1.extend_from_slice(&bw.finish());
    mpeg1.extend_from_slice(&[0, 0, 1, 0xB7]);

    let scratch = Scratch::new("repeat");
    for (name, stream) in [("mpeg2", mpeg2), ("mpeg1", mpeg1)] {
        let (count, ours) = decode_ours(&stream).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_eq!(count, 3, "{name}");
        let frame_bytes = ours.len() / 3;
        assert_eq!(
            ours[..frame_bytes],
            ours[frame_bytes..2 * frame_bytes],
            "{name}"
        );
        assert_eq!(ours[..frame_bytes], ours[2 * frame_bytes..], "{name}");
        let path = scratch.path(&format!("{name}.es"));
        std::fs::write(&path, &stream).unwrap();
        let (reference, diagnostics) =
            decode_reference(&path, &scratch.path(&format!("{name}.yuv"))).unwrap();
        assert!(diagnostics.is_empty(), "{name}: {diagnostics:?}");
        compare(&ours, &reference, INTRA_BOUND).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_eq!(
            reference[..frame_bytes],
            reference[2 * frame_bytes..],
            "{name}: the black box repeats the picture too"
        );
    }
}
