# Whole-sequence reference-conformance corpus

Each fixture pairs one **elementary stream** (`.m1v` MPEG-1 /
`.m2v` MPEG-2) with the **reference decode** (`.ref.yuv`) produced by
an opaque black-box encoder/decoder binary (its source is not
consulted). `tests/reference_conformance.rs` decodes every stream with
`decode_video_sequence` and compares it frame-by-frame against the
reference.

## Comparison contract

MPEG-1 / MPEG-2 do not define a bit-exact IDCT: ISO/IEC 11172-2
Annex A / ISO/IEC 13818-2 Annex A only require IEEE 1180 statistical
accuracy, so two conforming decoders may differ per sample by ±1 at
each IDCT, and the difference propagates boundedly through the
prediction chain (a second-generation B-picture can accumulate up to
about ±3). The corpus therefore asserts:

* frame count and dimensions **exactly** equal to the reference;
* every sample within **|Δ| ≤ 3** of the reference;
* fewer than **5 %** of samples differing per frame.

Any violation is a real decoder divergence to root-cause, not
tolerance slack. Empirically the corpus decodes at worst |Δ| = 3 on
a single sample (one leading B-picture of `mpeg2-ivlc`), and |Δ| ≤ 2
everywhere else.

## Generation

Generated 2026-07-11 with `ffmpeg` 8.1 (black-box invocation only).
The `.ref.yuv` files are `ffmpeg -i <stream> -f rawvideo -pix_fmt
yuv420p` (yuv422p for the 4:2:2 fixture) decodes of the streams.

| stream | command (input `-f lavfi` source → codec options) |
|---|---|
| `mpeg1-ibbp-96x64.m1v` | `testsrc2=size=96x64:rate=25:duration=1.2` → `-c:v mpeg1video -b:v 400k -g 12 -bf 2` |
| `mpeg1-bigmv-160x128.m1v` | `mandelbrot=size=160x128:rate=25` (24 frames) → `-c:v mpeg1video -qscale:v 6 -g 12 -bf 3 -me_range 127` |
| `mpeg1-vcd-352x240.m1v` | `testsrc2=size=352x240:rate=30000/1001:duration=0.28` → `-c:v mpeg1video -b:v 1150k -minrate 1150k -maxrate 1150k -bufsize 327680 -g 15 -bf 2` |
| `mpeg2-ibbp-96x64.m2v` | `testsrc2=size=96x64:rate=25:duration=1.2` → `-c:v mpeg2video -b:v 500k -g 12 -bf 2` |
| `mpeg2-ilaced-96x64.m2v` | `testsrc2=size=96x64:rate=25:duration=0.8` → `-c:v mpeg2video -qscale:v 4 -g 10 -bf 2 -flags +ildct+ilme -top 1 -alternate_scan 1` |
| `mpeg2-ivlc-96x64.m2v` | `testsrc2=size=96x64:rate=25:duration=0.8` → `-c:v mpeg2video -qscale:v 3 -qmax 28 -g 10 -bf 2 -intra_vlc 1 -non_linear_quant 1 -intra_dc_precision 2` |
| `mpeg2-422-96x64.m2v` | `testsrc2=size=96x64:rate=25:duration=0.6` → `-c:v mpeg2video -qscale:v 3 -g 6 -bf 1 -pix_fmt yuv422p` |
| `mpeg2-100x62.m2v` | `testsrc2=size=100x62:rate=25:duration=0.6` → `-c:v mpeg2video -qscale:v 3 -g 6 -bf 2` |

## What each fixture exercises

* **mpeg1-ibbp** — ISO/IEC 11172-2 IBBP GOPs: §2.4.4.1 DC chain,
  §2.4.4.2/.3 motion reconstruction, §2.4.4.4 skips, display reorder.
* **mpeg1-bigmv** — high-motion content, `f_code` up to 4, the B.5f
  escape coding, dense residuals.
* **mpeg1-vcd** — VCD-rate CBR at SIF 352×240 (classic parameters);
  hits the legal `little == -16f` wrap-seam vector.
* **mpeg2-ibbp** — MP@ML frame pictures with adaptive quantisation
  (per-MB `quantiser_scale`), skip runs between intra macroblocks.
* **mpeg2-ilaced** — interlaced frame pictures: field-based
  prediction with `motion_vertical_field_select`, field DCT,
  `alternate_scan`.
* **mpeg2-ivlc** — `intra_vlc_format` (Table B-15), non-linear
  quantiser scale (Table 7-6), 10-bit `intra_dc_precision`.
* **mpeg2-422** — 4:2:2 profile: Figure 6-11 interleaved chroma block
  numbering, 8-block macroblocks.
* **mpeg2-100x62** — non-macroblock-multiple dimensions: visible-rect
  cropping, macroblock-aligned reference storage, and a final slice
  that ends at EOF without a `sequence_end_code`.

## SHA-256

```
607221eb7822653f9c79208ae69282d57b8f2ad6a256e8e010b29ddc4564b484  mpeg1-bigmv-160x128.m1v
cf87d728734127ad78add29fb447c14c27deda56575558a30de43cffd8ebb258  mpeg1-ibbp-96x64.m1v
0ef62d88dd20ccc974e9594aa2441a4084221be3f60383a394fab402b1e7113b  mpeg1-vcd-352x240.m1v
4b491edb85e2f8484c5382f6bceedbc8810485f569b46c9a608fdf369fbe4540  mpeg2-100x62.m2v
526a3877ca5b654b9d84522ea847bacb546be372b44e6278f5356deec13de9e2  mpeg2-422-96x64.m2v
9656ce142785c600d2dc493ca86eb31fa633937295e73eed0247cd65b5265e87  mpeg2-ibbp-96x64.m2v
e3222f8f90c487dedca96bda9c0dca3570bdbec668a5ec46d25e7b72bd002014  mpeg2-ilaced-96x64.m2v
c46b840f35681568032cad8bdbc5cbcc21bdf55aff726a9d446d2da83b545513  mpeg2-ivlc-96x64.m2v
02c5f66526b0eac218a4de7e78887c5fd949e464df539a5d148acd383a313895  mpeg1-bigmv-160x128.m1v.ref.yuv
25b975a610357a12be729a1b4135d7cce04efa0bd5fbf85137aedcfad642e5d2  mpeg1-ibbp-96x64.m1v.ref.yuv
2b3679665845c91785f72952d3b20dedf2d6a77c3953425ae0e36315ecbebecd  mpeg1-vcd-352x240.m1v.ref.yuv
e92c708196cc61a75c50c1ff17df2bc90a757adbd1a8517236efaf51c3dc9440  mpeg2-100x62.m2v.ref.yuv
888a895225d13443c3a60569a3018148d3a2d24e266aeac7c2537e67d165334a  mpeg2-422-96x64.m2v.ref.yuv
46eef112d0ca0949283d147777a650b96037bc7aff9016bc7793cebed275bac6  mpeg2-ibbp-96x64.m2v.ref.yuv
a0e7b5dc40706752b90288bcf3e74bc1ab516b55874727352e1134cc8b298282  mpeg2-ilaced-96x64.m2v.ref.yuv
e25233d69c46a23855ab58065f51769c9ac056cdffa6363aaed3b7b2a8483937  mpeg2-ivlc-96x64.m2v.ref.yuv
```
