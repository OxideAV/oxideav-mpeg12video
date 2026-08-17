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
| `selfenc-mpeg1-qmat-48x32.m1v` | `encode_mpeg1_display_order_sequence` | MPEG-1 I B P with **downloadable §2.4.3.2 quantiser matrices** (intra ramp + all-20 non-intra loaded by the sequence header; both the forward quantiser and the decoder derive them from the header) |
| `selfenc-cbr-64x48.m2v` | `encode_cbr_gop_sequence` | MPEG-2 **CBR** (Annex C): I B P B P \| I B P at 240 kbit/s, 65 536-bit VBV buffer — real §6.3.9 `vbv_delay` in every picture header, quantiser adaptation + zero stuffing holding the C.5/C.6 occupancy bounds (verified by `vbv::verify_cbr_stream`) |
| `selfenc-mpeg1-cbr-64x48.m1v` | `encode_mpeg1_cbr_sequence` | MPEG-1 **CBR** (11172-2 Annex C): two-GOP I B B P \| I B B P at 240 kbit/s, 65 536-bit VBV buffer — real §2.4.3.4 `vbv_delay`, constrained-parameters flag set, same occupancy-bound verification |
| `selfenc-fieldseq-48x64.m2v` | `encode_field_display_order_gop_sequence` | MPEG-2 **field-coded** I B P B P (48×64, fields 48×32): §6.1.1.4.1 field pairs top-first with shared `temporal_reference`, `field_motion_type = 01` + `motion_vertical_field_select` over both parities, the §7.6.2.1 second-P-field synthetic reference, Table B-4 B-field modes, interlaced-phased content |
| `selfenc-framefield-64x64.m2v` | `encode_ff_display_order_gop_sequence` | MPEG-2 **frame-picture field-based** I B P B P (`frame_pred_frame_dct = 0`, interlaced sequence, §6.3.3 grid): per-macroblock Table 6-17 `frame_motion_type` with `Field-based` macroblocks (two field vectors + `motion_vertical_field_select` per direction, §7.6.3.1 vertical-half-pred PMV coding), per-macroblock `dct_type` **field DCT** (§6.1.3 stride-2 luma blocks), opposite-direction field pans so field prediction genuinely wins |
| `selfenc-dualprime-64x64.m2v` | `encode_ff_display_order_gop_sequence` | MPEG-2 **dual-prime** I P P (`b_between = 0` per §7.6.3.6): Table 6-17 `Dual prime` macroblocks (one vector + Table B-11 `dmvector`, §7.6.7.4 four-field average) beside Frame-based and Field-based ones — noise on the I reference makes the two-field average the best predictor |
| `selfenc-mpeg1-dpics-48x32.m1v` | `encode_mpeg1_d_sequence` | MPEG-1 **D-only sequence** (§2.4.1 / §2.4.3.4 `picture_coding_type = 4`): four dc intra-coded pictures in two GOPs — Table B.2d macroblock type, six DC-only blocks (no AC walk / `end_of_block`, §2.4.2.8), `end_of_macroblock` bits. **No `.ref.yuv`**: the black-box binary emits zero frames for type-4 pictures (the same limitation recorded for the `mpeg1-dpics` conformance fixture), so the stream is pinned bit-exactly and decoded sample-exactly against the encoder's own §2.4.4.1 reconstruction |
| `selfenc-fieldmodes-64x64.m2v` | `encode_field_adaptive_display_order_gop_sequence` | MPEG-2 field-coded **adaptive-mode** I P P (`b_between = 0` per §7.6.3.6): per-macroblock Table 6-18 selection between simple field prediction, **§7.6.7.3 16×8 MC** (two `(vector, field-select)` region pairs) and **§7.6.3.6 dual-prime** (one vector + Table B-11 `dmvector`) — 18/9/5 macroblocks respectively, mixed inside single slices so the Table 7-11 predictor-update rows interact |
| `selfenc-422-ibbp-64x48.m2v` | `encode_display_order_gop_sequence` | **4:2:2 profile** I B P B P (one GOP): §6.3.5 `chroma_format = 10` with the High@Main `profile_and_level_indication` (Table 8-5), Figure 6-11 **eight-block macroblocks**, §6.2.5.3 `coded_block_pattern_1`, §7.6.3.7 horizontal-only chroma MV scaling; full-height chroma detail plus the intra-fallback stamp from display frame 3 |
| `selfenc-422-full-64x48.m2v` | `encode_display_order_gop_sequence_with_matrices` | 4:2:2 I B P B P with the full round-447 flag set — `intra_vlc_format = 1` (Table B-15 intra AC), `alternate_scan = 1` (§7.3), `q_scale_type = 1` (Table 7-6 non-linear), `intra_dc_precision = 2` (10-bit DC) — plus §6.3.11 **downloadable matrices**: luminance intra/non-intra loads in the sequence header and chroma intra/non-intra tables in a `quant_matrix_extension()` inside the I picture (w = 2 / w = 3 at Table 7-5) |

All MPEG-2 streams except 18–19: 4:2:0, `progressive_sequence = 1` (§6.3.3
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

The two CBR streams were generated 2026-08-11 (same deterministic
generator, streams 11–12) and validated with the same black-box
reference decoder binary (v8.1.2): the strict error-detection pass
reported nothing, and the committed `.ref.yuv` decodes agree with ours
within the corpus |Δ| ≤ 3 contract. Their `vbv_delay` fields carry
real Annex C values (not `0xFFFF`), and `tests/selfenc_conformance.rs`
additionally holds both streams to the full Annex C occupancy /
delay-consistency verification (`vbv::verify_cbr_stream`) against the
`bit_rate` / `vbv_buffer_size` they declare.

The field-coded stream (13) was generated and validated 2026-08-11
the same way; its default black-box decode agrees with ours at max
|Δ| = 1 (≤ 1.7 % samples). As with the `fieldpics-48x64` conformance
fixture, the reference binary's strict error-detection mode flags
field-picture-pair packets while still decoding every frame — the
default decode is the committed reference.

The frame-picture field-based stream (14) and the dual-prime stream
(15) were generated and validated 2026-08-15 with the same black-box
reference decoder binary (v8.1.2): the strict error-detection pass
(`-err_detect explode`) reported nothing for either stream, and the
committed `.ref.yuv` decodes agree with ours at max |Δ| = 2
(≤ 2.1 % samples, stream 14) and max |Δ| = 1 (≤ 0.8 %, stream 15).
The MPEG-1 D-picture stream (16, same date) has no black-box
reference: the reference binary produced an empty decode
(zero frames) for `picture_coding_type == 4`, so
`tests/selfenc_conformance.rs` pins it bit-exactly and holds
`decode_video_sequence` to the encoder's own reconstruction instead.

The adaptive field-mode stream (17) was generated and validated
2026-08-15 the same way: its default black-box decode agrees with
ours at max |Δ| = 1 (≤ 0.5 % samples); as with the other field-coded
streams, the strict error-detection mode flags field-picture-pair
packets while still decoding every frame — the default decode is the
committed reference.

2026-08-15 (later the same day): the DCT/IDCT cosine kernel was
changed from a runtime `cos()` build to the correctly-rounded
constant table (`idct::COS_TABLE`) — platform math libraries differ
in the final ulp of `cos()`, which had made four streams'
emitted bits host-dependent. Four fixtures were regenerated under the
constant kernel (`selfenc-intra-100x62.m2v`,
`selfenc-mpeg1-cbr-64x48.m1v`, `selfenc-dualprime-64x64.m2v`,
`selfenc-fieldmodes-64x64.m2v`) and re-validated with the same
black-box binary: strict passes as before, committed reference
decodes agree with ours at max |Δ| = 1. The other thirteen streams
regenerate byte-identical under the constant kernel.

The two 4:2:2 streams (18–19) were generated and validated
2026-08-17 with the same black-box reference decoder binary (v8.1.2):
the strict error-detection pass (`-err_detect explode`) reported
nothing for either stream, and the committed `.ref.yuv` decodes
(`-pix_fmt yuv422p`) agree with ours at max |Δ| = 2 (≤ 2.6 % samples,
stream 18; ≤ 1.4 %, stream 19). All seventeen pre-existing streams
regenerate byte-identical.

## SHA-256

```
1e1c2ed21f6f5b81725a8bbc4c11f67b6c11f5b643f36041f3a5d7345cdf4dd0  selfenc-422-full-64x48.m2v
ce73fe19e39bdc742f2f229e6aa636702f95838615584f8b707e64f6766adb10  selfenc-422-full-64x48.m2v.ref.yuv
a33ce574297e0c5d919497fa2365809049e5dc98900734272693c0bd5aa9712a  selfenc-422-ibbp-64x48.m2v
e319618a8040d28461f566cc1f76c47e6c3b58b7fe8dee8b2dca8899c3a112be  selfenc-422-ibbp-64x48.m2v.ref.yuv
256f58029bc3cd91efac30d251e9210ac93b082d6ea10e61f65159889b48ff8c  selfenc-cbr-64x48.m2v
2801a9b27965edff607d0b2b1f40e90b83cb13b50486809d4abc530875ce1dbf  selfenc-cbr-64x48.m2v.ref.yuv
df8d6ddc8b618b1e00ac8c9cd353a9f886873d171fe5d3624ed6c7b61c40ae6f  selfenc-dualprime-64x64.m2v
20943bb163043076e4528816d0a48fd13adb9c318540d49e4f9ff44e28d03f4e  selfenc-dualprime-64x64.m2v.ref.yuv
b365a287d0e07a8b05c90452fc18cf3c96a31c20a39a26a155ffa02bae3d6d4f  selfenc-fieldmodes-64x64.m2v
873e8ebb37efc6a0215efa8ec0f4aeef761c3939ee12fde17531dfe0a48cf81d  selfenc-fieldmodes-64x64.m2v.ref.yuv
88ddc2b1f6d30c33bcacfdaa0a9c118c32cacf8caa46bad0dd972ca8dbdf7dd4  selfenc-fieldseq-48x64.m2v
49f48c79988118cedd095ea3159c354651b54744f2e9dded434cfc48e9f0b198  selfenc-fieldseq-48x64.m2v.ref.yuv
83d055022c723fc196665130c6aecb1e227615a7c27aa0efa356bf71bff7b88d  selfenc-framefield-64x64.m2v
e0530b6f8a6813cca2177aeaae81c07246233ee175a8a0ecb41d4f0c8b1eaaef  selfenc-framefield-64x64.m2v.ref.yuv
44457fbcd42c4807ccbfb9b6f0fa19f00d7638c6b7a35fb6c50f930738120a15  selfenc-gops-48x32.m2v
3d3d17f9eecf84f7f7d17cea53b54d98e2a22728d36abe32a6cb76d5a3ce65a2  selfenc-gops-48x32.m2v.ref.yuv
1cbab03cac938f844beb3f44c94bfbafd92b81f4488357380768c11f7646dacf  selfenc-ibbp-64x48.m2v
a73304570cf5d6bdfb51ce98b510e8cdd6aa8a5966bdc5527912fdfc409b714b  selfenc-ibbp-64x48.m2v.ref.yuv
7cd62cb2a628b0dc1a198498dabdc278ffbba604e0ae91d3893477f7205b94e2  selfenc-intra-100x62.m2v
16e690e4f2ad453b8cee201571572f76aa4f429c05d31bbd91ba08e20e24b57e  selfenc-intra-100x62.m2v.ref.yuv
98e3c4d2ac26100d433440dba07c884bdd83d4caa5a2f1cdbc195c029c1039ae  selfenc-intra-64x48.m2v
a2d0e500ff46de2018139533a3e3303787bd6f4bb583b2fc9cf9db11659470a8  selfenc-intra-64x48.m2v.ref.yuv
863953e946bec0b5dfe5dcdbbbe9ab5ab2ab58afa64c401a17b26a101fa42500  selfenc-ipb-64x48.m2v
8bdf3a4ab5e7d18c1c76894ac59b8c44e291ec2ab2483db6fcf3157b62560d51  selfenc-ipb-64x48.m2v.ref.yuv
97c04629edc150195bee32df8f26acbcdde0e4fb01b1cad6e2859effa70950e8  selfenc-ipchain-64x48.m2v
e6b5468feda0d1d5e2db5e359ec420a433cfc99cbd98fe366b7869c3acebc279  selfenc-ipchain-64x48.m2v.ref.yuv
ca7e7bff3046608508796f19c147b4985ce0b7c9d8134d53c4972a6bec39ee96  selfenc-mpeg1-cbr-64x48.m1v
88f9728a7053358ee278a78a6274403536dce5b2eed6e5148aeea9d5af147b4c  selfenc-mpeg1-cbr-64x48.m1v.ref.yuv
5943dc602e6ea74ae44ad605bfc3375bfae130b49d0ac7644cf724a12570ba1c  selfenc-mpeg1-dpics-48x32.m1v
cca30f325557703935157d1c6188771e1237b3874fb091851cb64e4c0d784388  selfenc-mpeg1-ibbp2gop-64x48.m1v
2604ebe4a8a840bd9038672fcd3c00cc75ce9d161dd5351588adbcd4b624bfff  selfenc-mpeg1-ibbp2gop-64x48.m1v.ref.yuv
042ec6ddd5ab85d98d06cd37e0373f99375d505c52213a06cef02ac95d2f66dd  selfenc-mpeg1-intra-64x48.m1v
236d605e0594ed20cb41901381319a2a41313b995968b406106fc41341369954  selfenc-mpeg1-intra-64x48.m1v.ref.yuv
19e8d38b36381372214c5b368aafffc79cfd3a57c0ff6d654bf71e48449c7c0b  selfenc-mpeg1-ippp-64x48.m1v
e5d00c6f007bed3a0e5bfdf3a10875050aef68bced62c5dd04768defc6dc5d38  selfenc-mpeg1-ippp-64x48.m1v.ref.yuv
9882e3a029dbdcb9d643fd21696b883d8287b976b8488887729045c831fc4687  selfenc-mpeg1-qmat-48x32.m1v
4916b08fb15265aa3d795cfd37d63c0bbe870f126846cabe7a6509ec8a81ee77  selfenc-mpeg1-qmat-48x32.m1v.ref.yuv
```
