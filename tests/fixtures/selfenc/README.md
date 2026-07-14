# Self-encoded conformance corpus

Each `.m2v` here was produced by **this crate's own MPEG-2 encoder**
from deterministic synthetic frames (`examples/gen_selfenc_corpus.rs`,
regenerated bit-exactly by `tests/selfenc_conformance.rs`); the paired
`.ref.yuv` is the decode of that stream by an opaque **black-box
reference decoder** binary (its source is not consulted). Together
they pin two facts:

1. **External conformance** — a decoder we did not write accepts the
   streams (including under its strict error-detection mode, which
   reported nothing) and reconstructs them within IDCT rounding of
   our own decoder (measured max |Δ| = 2, ≤ 2.1 % samples differing;
   asserted at the corpus-wide |Δ| ≤ 3 / < 5 % contract).
2. **Encoder bit-stability** — the test regenerates every stream from
   the same synthetic inputs and requires byte identity with the
   committed fixture, so any encoder change that alters emitted bits
   must consciously refresh the fixtures *and* re-run the black-box
   validation.

## Streams

| stream | encoder entry point | what it exercises |
|---|---|---|
| `selfenc-intra-64x48.m2v` | `encode_intra_picture` | all-intra frame picture: §7.2.1 DC + §7.2.2 AC entropy coding, §7.4 forward quantisation, sequence/picture/slice layer writers |
| `selfenc-intra-100x62.m2v` | `encode_intra_picture` | non-macroblock-multiple dimensions (right/bottom edge macroblocks overhang the visible picture) |
| `selfenc-ipchain-64x48.m2v` | `encode_i_p_chain` | I + 3 motion-compensated P pictures: full-search ME (§7.6.3.8-legal vectors only), Table B-3 `MC Coded` / `MC Not Coded`, PMV differential coding, reference rotation on the decoder's exact reconstruction, intra-macroblock fallback (frames 2–3 carry an unpredictable high-contrast stamp) |
| `selfenc-ibbp-64x48.m2v` | `encode_display_order_sequence` | whole display-order sequence I B B P B B P (7 frames, 2 B-pictures between anchors): §6.1.1.11 coded-order assembly, per-display-index `temporal_reference`, anchor rotation across two P groups |
| `selfenc-ipb-64x48.m2v` | `encode_i_p_b` | I/B/P group in coded order I,P,B (display I,B,P): forward/backward/interpolated B prediction, per-direction PMV slots |

All streams: 4:2:0, `progressive_sequence = 1` (§6.3.3 `Ceil(h/16)`
macroblock grid), `frame_pred_frame_dct = 1`, linear quantiser scale,
`quantiser_scale_code` 5–6, `f_code` 3 where motion is coded.

## Generation

Generated 2026-07-14. Streams: `cargo run --example gen_selfenc_corpus
-- <dir>` (fully deterministic — no RNG, no time). References:
`ffmpeg 8.1` (black-box invocation only) via `ffmpeg -threads 1 -i
<stream> -f rawvideo -pix_fmt yuv420p <out>.ref.yuv`; a strict-mode
pass (`-err_detect explode -f null -`) accepted every stream with no
diagnostics. Earlier encoder revisions emitted §7.6.3.8-illegal
edge-macroblock motion vectors, which the reference decoder flagged
("motion vector out of boundary") and refused to predict from — the
motion search now visits only vectors whose §7.6.4 sample span stays
inside the coded picture, and the trace is clean.

## SHA-256

```
1cbab03cac938f844beb3f44c94bfbafd92b81f4488357380768c11f7646dacf  selfenc-ibbp-64x48.m2v
a73304570cf5d6bdfb51ce98b510e8cdd6aa8a5966bdc5527912fdfc409b714b  selfenc-ibbp-64x48.m2v.ref.yuv
26b59ac6f2bb945ea00b8da23681a48edea9a29941b4215d788f853afb36c052  selfenc-intra-100x62.m2v
dcc4d8bcae15b9c34a7b7eb1e53268952c2c9cb9a0672e0d73ff483290de9079  selfenc-intra-100x62.m2v.ref.yuv
98e3c4d2ac26100d433440dba07c884bdd83d4caa5a2f1cdbc195c029c1039ae  selfenc-intra-64x48.m2v
a2d0e500ff46de2018139533a3e3303787bd6f4bb583b2fc9cf9db11659470a8  selfenc-intra-64x48.m2v.ref.yuv
863953e946bec0b5dfe5dcdbbbe9ab5ab2ab58afa64c401a17b26a101fa42500  selfenc-ipb-64x48.m2v
8bdf3a4ab5e7d18c1c76894ac59b8c44e291ec2ab2483db6fcf3157b62560d51  selfenc-ipb-64x48.m2v.ref.yuv
97c04629edc150195bee32df8f26acbcdde0e4fb01b1cad6e2859effa70950e8  selfenc-ipchain-64x48.m2v
e6b5468feda0d1d5e2db5e359ec420a433cfc99cbd98fe366b7869c3acebc279  selfenc-ipchain-64x48.m2v.ref.yuv
```
