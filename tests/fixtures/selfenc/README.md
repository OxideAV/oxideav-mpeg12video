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
| `selfenc-444-ibp-64x48.m2v` | `encode_display_order_gop_sequence` | **4:4:4** I B P (one GOP): Figure 6-12 twelve-block macroblocks, full-resolution chroma, §7.6.3.7 unscaled chroma vectors, §6.2.5.3 `coded_block_pattern_2` (bits 3..0 driving blocks 8..11 per the printed §6.3.17.4 derivation; non-intra blocks 6/7 stay uncoded — bits 5..4 always zero, so the stream decodes identically under a corrected six-block reading) |
| `selfenc-skipconceal-64x48.m2v` | `encode_display_order_gop_sequence_with_options` | **Skipped macroblocks + concealment motion vectors** (round 453): I B P B P over a mostly-static scene — §7.6.6.2 P skips (zero-vector, predictor reset) and §7.6.6.4 B skips (PMV/previous-direction inheritance) folded into Table B-1 address increments, a per-frame re-rolled stamp forcing Table B-3 intra fallbacks whose macroblocks carry §7.6.3.9 concealment vectors + marker bits, `concealment_motion_vectors = 1` with a real I-picture `f_code` |
| `selfenc-fffull-64x64.m2v` | `encode_ff_display_order_gop_sequence` | **Frame-field encode under the full entropy flag set** (round 453): `frame_pred_frame_dct = 0` I B P B P with `alternate_scan = 1`, `intra_vlc_format = 1` (Table B-15 intra AC), non-linear `q_scale_type = 1` and 10-bit `intra_dc_precision = 2`, per-field opposing pans (field MC + field DCT throughout) |
| `selfenc-422-full-64x48.m2v` | `encode_display_order_gop_sequence_with_matrices` | 4:2:2 I B P B P with the full round-447 flag set — `intra_vlc_format = 1` (Table B-15 intra AC), `alternate_scan = 1` (§7.3), `q_scale_type = 1` (Table 7-6 non-linear), `intra_dc_precision = 2` (10-bit DC) — plus §6.3.11 **downloadable matrices**: luminance intra/non-intra loads in the sequence header and chroma intra/non-intra tables in a `quant_matrix_extension()` inside the I picture (w = 2 / w = 3 at Table 7-5) |
| `selfenc-422-fieldseq-48x64.m2v` | `encode_field_display_order_gop_sequence` | **4:2:2 field-coded** I B P B P (round 456): Figure 6-11 eight-block macroblocks in Top / Bottom field pictures, §6.2.5.3 `coded_block_pattern_1`, full-height chroma with per-field structure, High@Main profile signalling |
| `selfenc-422-framefield-64x64.m2v` | `encode_ff_display_order_gop_sequence` | **4:2:2 frame-picture field-based** I B P B P (round 456): per-field opposite pans in luma *and* the full-height chroma — Field-based macroblocks plus the §6.1.3 **field-DCT chroma organisation** (4:2:2 chroma blocks follow the luma under `dct_type`), `dct_type` costed over every block |
| `selfenc-422-fieldmodes-64x64.m2v` | `encode_field_adaptive_display_order_gop_sequence` | **4:2:2 adaptive field modes** I P P (round 456): the stream-17 luma over eight-block macroblocks — simple field / §7.6.7.3 16×8 / §7.6.3.6 dual-prime (18 / 9 / 5) with `coded_block_pattern_1` |
| `selfenc-444-framefield-64x64.m2v` | `encode_ff_display_order_gop_sequence` | **4:4:4 frame-picture field-based** I B P (round 456): Figure 6-12 twelve-block macroblocks under per-macroblock `dct_type` with full-resolution chroma following the luma field organisation (non-intra blocks 6/7 stay uncoded per the printed §6.3.17.4 derivation) |
| `selfenc-snr-base-64x48.m2v` | `encode_display_order_gop_sequence` | **SNR-scalable lower layer** (round 456): a coarse (`quantiser_scale_code = 14`) progressive I B P B P over busy content — an ordinary 13818-2 stream, black-box validated like every other corpus stream |
| `selfenc-snr-enh-64x48.m2v` | `encode_snr_enhancement_layer` | **§7.8 SNR enhancement layer** (round 456) for the stream above at `quantiser_scale_code = 4`: `sequence_scalable_extension()` (SNR, `layer_id = 1`), coincident GOP / picture / slice layers, Table B-8 macroblocks with non-intra refinement blocks. **No `.ref.yuv`**: no black-box decoder in reach consumes an SNR enhancement layer (the reference binary misreads it as a plain stream), so `tests/selfenc_conformance.rs` pins it bit-exactly and holds `decode_snr_scalable_sequence` sample-exact against the encoder's own combined reconstruction |
| `selfenc-temporal-base-64x48.m2v` | `encode_display_order_gop_sequence` | **Temporal-scalable lower layer** (round 456): a progressive I B P B P at the even half-frame instants — an ordinary 13818-2 stream, black-box validated like every other corpus stream |
| `selfenc-temporal-enh-64x48.m2v` | `encode_temporal_enhancement_layer` | **§7.9 temporal enhancement layer** (round 456) at the odd instants: `sequence_scalable_extension()` (temporal, `layer_id = 1`, `picture_mux_enable = 1`, `mux_to_progressive_sequence = 1`, order 0 / factor 1), one GOP, four B pictures each with a `picture_temporal_scalable_extension()` (`reference_select_code = 11`: forward = most recent lower frame, backward = next lower frame). **No `.ref.yuv`**: no black-box decoder in reach resolves the lower-layer references, so the layer is pinned bit-exactly with `decode_temporal_scalable_sequence` held sample-exact against the encoder's own reconstruction |

All MPEG-2 streams except 18–20 and 23–26: 4:2:0, `progressive_sequence = 1` (§6.3.3
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

The 4:4:4 stream (20) was generated and validated 2026-08-17 the same
way: strict pass clean, committed reference decode
(`-pix_fmt yuv444p`) agrees with ours at max |Δ| = 1 (≤ 2.0 %
samples). Its non-intra macroblocks never code blocks 6/7 (no wire
slot in the printed §6.3.17.4 derivation), so the emitted
`coded_block_pattern_2` always has bits 5..4 zero.

The ten progressive MPEG-2 streams were regenerated 2026-08-30 for a
§6.3.10 conformance fix in the picture-coding-extension writer:
`top_field_first` is now `0` where `repeat_first_field` is `0` in a
progressive sequence (previously always `1`), and `chroma_420_type`
now equals `progressive_frame` at 4:2:0 (previously always `0`). Only
those header bits moved — every regenerated black-box reference
decode came back **byte-identical** to the committed one, and the
strict error-detection pass (`-err_detect explode`, v8.1.2) stayed
clean for all ten. The interlaced / field-coded streams (13–17) and
the MPEG-1 streams already carried conforming values and regenerate
byte-identical.

The round-453 streams (21–22) were generated and validated 2026-08-30
with the same black-box reference decoder binary (v8.1.2): the strict
error-detection pass (`-err_detect explode`) reported nothing for
either stream, and the committed `.ref.yuv` decodes agree with ours
within the corpus |Δ| ≤ 3 contract. Stream 21's skipped macroblocks
and concealment vectors and stream 22's alternate-scan / B-15 intra
entropy coding are accepted by the reference decoder without a
diagnostic.

The round-456 streams (23–26) were generated and validated
2026-09-05 with the same black-box reference decoder binary (v8.1.2):
the default decodes (`-pix_fmt yuv422p` / `yuv444p`) are the committed
references and agree with ours within the corpus |Δ| ≤ 3 contract; the
strict error-detection pass (`-err_detect explode`) reported nothing
for the two frame-picture streams (24, 26) and, for the two
field-picture-pair streams (23, 25), only the documented field-pair
packet diagnostic while still decoding every frame (as for streams 13
and 17). All twenty-two pre-existing streams regenerate
byte-identical.

The SNR pair (27) and the temporal pair (28) were generated 2026-09-05: the lower layer's default
black-box decode is the committed reference (strict pass clean); the
enhancement layers are pinned by the encoder only, as recorded in the
table.

## SHA-256

```
8ccf77e19667aee7e200706b5cf13ef2b3d8af76868f1def9ced10746417e56b  selfenc-temporal-base-64x48.m2v
76ad6d93140fbf1a6359a7a330e8316687cba435b0a6bbb5dd5a9f083edaf1e1  selfenc-temporal-base-64x48.m2v.ref.yuv
68095e2ad3260929bf036cb3ee48ec4ffe1fe62e5bd60e53becaed7818e67291  selfenc-temporal-enh-64x48.m2v
50bdd9e4224aafc44d2b3fbac5a2f9a06f67c282f4cabf1df679b50aa5970d0f  selfenc-snr-base-64x48.m2v
d7c693b1be198b1467f341ad1d1f45e6a328ee65bb4804fd1289fc8d04ed3adc  selfenc-snr-base-64x48.m2v.ref.yuv
14cf831394930376a75657e415ec2c592ec67442cc8ab318b32c8f4461caf1f4  selfenc-snr-enh-64x48.m2v
445b75da0ccc619634cc922eb9d7b3fe2628ac4f9cdba04b7fca39e5e176be54  selfenc-422-fieldseq-48x64.m2v
6da22fbbe181a9be87b5e3757e0ebd89ba76b10f54c6a202e1438c374bd42695  selfenc-422-fieldseq-48x64.m2v.ref.yuv
7c7becdaf05190503a8f8e5e3681b808a991345b7fea4b05c9f9915123795c2e  selfenc-422-framefield-64x64.m2v
1f9994988f7b3adbfdc4f99806b7bd5c02300b2cb9cb094069a43f2f6d59c880  selfenc-422-framefield-64x64.m2v.ref.yuv
770c0fa3d38c0defc2322099edc5442cd674f25fb3dad9e4e96e2c58918aeb08  selfenc-422-fieldmodes-64x64.m2v
8aa00417acb99b38873f705de95ab815675c46eb617fc4c332e5d2295ee4be94  selfenc-422-fieldmodes-64x64.m2v.ref.yuv
19571cdcc9086f5000a6576e980f7dbe2b5c5dffff13b0231b8fd778b8bdc816  selfenc-444-framefield-64x64.m2v
e8022beca3f8acfef922e370b6f440e842d3cc46a20e83eecf8ab7dc136d2546  selfenc-444-framefield-64x64.m2v.ref.yuv
24bca61cff377a087ab53222fbc7359266d0634d3596d127b93127b090dcf746  selfenc-skipconceal-64x48.m2v
c04c2550505be29a7f26d4cfa67fdf3a64b9e50050a43f74f10217ce10918c09  selfenc-skipconceal-64x48.m2v.ref.yuv
9a32853f6e0852c09584b9623f86f27d53de96c6e0dcdbc0786b689200701461  selfenc-fffull-64x64.m2v
57b9e5dd5e5cd6d4086ecda013c1bc0c4913546693bb95708cf62f63bd649f01  selfenc-fffull-64x64.m2v.ref.yuv
b5ff5f2ff461c8e685e7f1e01a06bcd4de597c81bbb1ffbe4542c32657baba20  selfenc-422-full-64x48.m2v
ce73fe19e39bdc742f2f229e6aa636702f95838615584f8b707e64f6766adb10  selfenc-422-full-64x48.m2v.ref.yuv
7a29d7d3d81a31e7f685479a31608cd5bbc7f9a0301c5b2805907c9e8720ece2  selfenc-422-ibbp-64x48.m2v
e319618a8040d28461f566cc1f76c47e6c3b58b7fe8dee8b2dca8899c3a112be  selfenc-422-ibbp-64x48.m2v.ref.yuv
15d64969ad8306d6f16a67293a95944b01b9f46dd09f8b4883a4fa7f5390128a  selfenc-444-ibp-64x48.m2v
d9ce098a6cae0cbf47b5ea0d8630f758b9371e423bbd459b62fe93bbf3853f53  selfenc-444-ibp-64x48.m2v.ref.yuv
62f75dd37314da791e0e367c1279d5d7a7287509a9b8f11ce4b1a42897448d22  selfenc-cbr-64x48.m2v
2801a9b27965edff607d0b2b1f40e90b83cb13b50486809d4abc530875ce1dbf  selfenc-cbr-64x48.m2v.ref.yuv
df8d6ddc8b618b1e00ac8c9cd353a9f886873d171fe5d3624ed6c7b61c40ae6f  selfenc-dualprime-64x64.m2v
20943bb163043076e4528816d0a48fd13adb9c318540d49e4f9ff44e28d03f4e  selfenc-dualprime-64x64.m2v.ref.yuv
b365a287d0e07a8b05c90452fc18cf3c96a31c20a39a26a155ffa02bae3d6d4f  selfenc-fieldmodes-64x64.m2v
873e8ebb37efc6a0215efa8ec0f4aeef761c3939ee12fde17531dfe0a48cf81d  selfenc-fieldmodes-64x64.m2v.ref.yuv
88ddc2b1f6d30c33bcacfdaa0a9c118c32cacf8caa46bad0dd972ca8dbdf7dd4  selfenc-fieldseq-48x64.m2v
49f48c79988118cedd095ea3159c354651b54744f2e9dded434cfc48e9f0b198  selfenc-fieldseq-48x64.m2v.ref.yuv
83d055022c723fc196665130c6aecb1e227615a7c27aa0efa356bf71bff7b88d  selfenc-framefield-64x64.m2v
e0530b6f8a6813cca2177aeaae81c07246233ee175a8a0ecb41d4f0c8b1eaaef  selfenc-framefield-64x64.m2v.ref.yuv
d382c9cf368bbf2d07c607154ea716688900a4dbbf4f6b3ac5bb6ae3f1da96b9  selfenc-gops-48x32.m2v
3d3d17f9eecf84f7f7d17cea53b54d98e2a22728d36abe32a6cb76d5a3ce65a2  selfenc-gops-48x32.m2v.ref.yuv
ea80f81809769a1c5d62c028dfd5d52c67c20e7c976b05c6f6081aed3a7c8442  selfenc-ibbp-64x48.m2v
a73304570cf5d6bdfb51ce98b510e8cdd6aa8a5966bdc5527912fdfc409b714b  selfenc-ibbp-64x48.m2v.ref.yuv
fa6fe1b2967898a2778663f740da164fca2da59cab8b16007592a17bb5bde4e9  selfenc-intra-100x62.m2v
16e690e4f2ad453b8cee201571572f76aa4f429c05d31bbd91ba08e20e24b57e  selfenc-intra-100x62.m2v.ref.yuv
c2dc205c91aad285efb7a6f48ade62141e6cc9912b3ecbf14c21961720639048  selfenc-intra-64x48.m2v
a2d0e500ff46de2018139533a3e3303787bd6f4bb583b2fc9cf9db11659470a8  selfenc-intra-64x48.m2v.ref.yuv
e937c87993ee601113c8a9f77d2adf939e7b8433ae8c71bac87c40142d572572  selfenc-ipb-64x48.m2v
8bdf3a4ab5e7d18c1c76894ac59b8c44e291ec2ab2483db6fcf3157b62560d51  selfenc-ipb-64x48.m2v.ref.yuv
d8cfbd062617a4d3d8a1eb266771418a8b4b79595d3be20fac557eb2725f73c7  selfenc-ipchain-64x48.m2v
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
