# Self-encoded conformance corpus

Each `.m2v` here was produced by **this crate's own MPEG-2 encoder**
and each `.m1v` by its **MPEG-1 (ISO/IEC 11172-2) encoder**, from
deterministic synthetic frames (`examples/gen_selfenc_corpus.rs`,
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
| `selfenc-gops-48x32.m2v` | `encode_display_order_gop_sequence` | MPEG-2 GOP structure: two §6.2.2.6 GOP headers (time codes at display 0 / 5, `closed_gop = 1`, `broken_link = 0`), one I per GOP, §6.3.9 per-GOP `temporal_reference` reset (I B P B P \| I B P) |
| `selfenc-mpeg1-intra-64x48.m1v` | `encode_mpeg1_intra_stream` | ISO/IEC 11172-2 all-intra: bare §2.4.2.3 sequence header (no extension, `constrained_parameters_flag = 1`), mandatory §2.4.2.4 GOP layer, Tables B.5a–B.5f entropy coding, §2.4.4.1 quantisation |
| `selfenc-mpeg1-ippp-64x48.m1v` | `encode_mpeg1_display_order_sequence` | MPEG-1 I P P P chain in one GOP: §2.4.4.2 dead-zone residuals, Table B.2b modes with the intra fallback (frames 2–3 carry the stamp), `recon_*_prev` differential MVs |
| `selfenc-mpeg1-ibbp2gop-64x48.m1v` | `encode_mpeg1_display_order_sequence` | MPEG-1 two-GOP I B B P \| I B B P: advancing GOP time codes, closed GOPs, §2.4.3.4 per-GOP `temporal_reference` reset, Table B.2c forward/backward/interpolated B modes |

All MPEG-2 streams: 4:2:0, `progressive_sequence = 1` (§6.3.3
`Ceil(h/16)` macroblock grid), `frame_pred_frame_dct = 1`, linear
quantiser scale, `quantiser_scale_code` 5–6, `f_code` 3 where motion
is coded. The MPEG-1 streams are 4:2:0 by definition, 25 pictures/s,
square pels, `quantizer_scale` 6, `f_code` 3, `full_pel_*_vector = 0`,
and all satisfy (and declare) the §2.4.3.2 constrained-parameters
bounds.

## Generation

MPEG-2 flat streams generated 2026-07-14; the GOP-structured MPEG-2
stream and the three MPEG-1 streams 2026-07-17. Streams: `cargo run
--example gen_selfenc_corpus -- <dir>` (fully deterministic — no RNG,
no time). References: `ffmpeg 8.1` (black-box invocation only) via
`ffmpeg -threads 1 -i <stream> -f rawvideo -pix_fmt yuv420p
<out>.ref.yuv`; a strict-mode pass (`-err_detect explode -f null -`)
accepted every stream with no decode diagnostics (the raw `.m1v`
elementary streams draw only container-level probe/duration notes).
The 2026-07-17 additions measure max |Δ| = 2 and ≤ 1.7 % samples
differing against our decoder. Earlier encoder revisions emitted §7.6.3.8-illegal
edge-macroblock motion vectors, which the reference decoder flagged
("motion vector out of boundary") and refused to predict from — the
motion search now visits only vectors whose §7.6.4 sample span stays
inside the coded picture, and the trace is clean.

## SHA-256

```
44457fbcd42c4807ccbfb9b6f0fa19f00d7638c6b7a35fb6c50f930738120a15  selfenc-gops-48x32.m2v
3d3d17f9eecf84f7f7d17cea53b54d98e2a22728d36abe32a6cb76d5a3ce65a2  selfenc-gops-48x32.m2v.ref.yuv
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
cca30f325557703935157d1c6188771e1237b3874fb091851cb64e4c0d784388  selfenc-mpeg1-ibbp2gop-64x48.m1v
2604ebe4a8a840bd9038672fcd3c00cc75ce9d161dd5351588adbcd4b624bfff  selfenc-mpeg1-ibbp2gop-64x48.m1v.ref.yuv
042ec6ddd5ab85d98d06cd37e0373f99375d505c52213a06cef02ac95d2f66dd  selfenc-mpeg1-intra-64x48.m1v
236d605e0594ed20cb41901381319a2a41313b995968b406106fc41341369954  selfenc-mpeg1-intra-64x48.m1v.ref.yuv
19e8d38b36381372214c5b368aafffc79cfd3a57c0ff6d654bf71e48449c7c0b  selfenc-mpeg1-ippp-64x48.m1v
e5d00c6f007bed3a0e5bfdf3a10875050aef68bced62c5dd04768defc6dc5d38  selfenc-mpeg1-ippp-64x48.m1v.ref.yuv
```
