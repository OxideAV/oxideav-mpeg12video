//! Picture-level assembly buffers (Y / Cb / Cr) + display-order reorder.

use oxideav_core::frame::VideoPlane;
use oxideav_core::{TimeBase, VideoFrame};

use crate::headers::PictureType;

/// MPEG-2 chroma sampling format (H.262 §6.3.3, 2-bit `chroma_format`).
///
/// MPEG-1 streams are always [`ChromaFormat::Yuv420`]; MPEG-2 streams may
/// also carry [`ChromaFormat::Yuv422`] (per ITU-T H.262 4:2:2 profile) or
/// [`ChromaFormat::Yuv444`] (4:4:4 profile) — the chroma planes have full
/// horizontal resolution in 4:2:2 and full horizontal+vertical resolution in
/// 4:4:4, with the macroblock spilling out into 8 or 12 8×8 blocks
/// respectively rather than the 4:2:0 default of 6 (4 luma + 2 chroma).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChromaFormat {
    /// 4:2:0 — chroma horizontally and vertically subsampled by 2.
    /// 4 luma + 1 Cb + 1 Cr = 6 blocks per macroblock.
    Yuv420,
    /// 4:2:2 — chroma horizontally subsampled by 2, full vertical.
    /// 4 luma + 2 Cb + 2 Cr = 8 blocks per macroblock.
    Yuv422,
    /// 4:4:4 — no chroma subsampling.
    /// 4 luma + 4 Cb + 4 Cr = 12 blocks per macroblock.
    Yuv444,
}

impl ChromaFormat {
    /// Decode the 2-bit `chroma_format` from sequence_extension.
    ///
    /// Returns `None` for the reserved code 0.
    pub fn from_code(code: u8) -> Option<Self> {
        match code {
            0b01 => Some(Self::Yuv420),
            0b10 => Some(Self::Yuv422),
            0b11 => Some(Self::Yuv444),
            _ => None,
        }
    }

    /// Encode the 2-bit `chroma_format`.
    pub fn to_code(self) -> u8 {
        match self {
            Self::Yuv420 => 0b01,
            Self::Yuv422 => 0b10,
            Self::Yuv444 => 0b11,
        }
    }

    /// Total number of 8×8 blocks per macroblock.
    pub fn blocks_per_mb(self) -> usize {
        match self {
            Self::Yuv420 => 6,
            Self::Yuv422 => 8,
            Self::Yuv444 => 12,
        }
    }

    /// Number of chroma 8×8 blocks per direction (Cb+Cr together = 2*chroma_blocks).
    pub fn chroma_blocks_per_mb(self) -> usize {
        self.blocks_per_mb() - 4
    }

    /// Chroma horizontal subsampling factor: 2 for 4:2:0/4:2:2, 1 for 4:4:4.
    pub fn chroma_h_shift(self) -> u32 {
        match self {
            Self::Yuv420 | Self::Yuv422 => 1,
            Self::Yuv444 => 0,
        }
    }

    /// Chroma vertical subsampling factor: 2 for 4:2:0, 1 for 4:2:2/4:4:4.
    pub fn chroma_v_shift(self) -> u32 {
        match self {
            Self::Yuv420 => 1,
            Self::Yuv422 | Self::Yuv444 => 0,
        }
    }
}

/// Allocate per-picture YUV buffers sized to the macroblock-aligned image.
#[derive(Clone)]
pub struct PictureBuffer {
    pub width: usize,
    pub height: usize,
    pub mb_width: usize,
    pub mb_height: usize,
    pub chroma_format: ChromaFormat,
    pub y: Vec<u8>,
    pub cb: Vec<u8>,
    pub cr: Vec<u8>,
    pub y_stride: usize,
    pub c_stride: usize,
    pub picture_type: PictureType,
    pub temporal_reference: u16,
    /// Display-order PTS computed at decode time (so the value is stable
    /// across GOP anchor roll-overs).
    pub display_pts: Option<i64>,
}

impl PictureBuffer {
    /// Allocate a 4:2:0 buffer (legacy MPEG-1 default).
    pub fn new(width: usize, height: usize, picture_type: PictureType, tr: u16) -> Self {
        Self::new_with_format(width, height, picture_type, tr, ChromaFormat::Yuv420)
    }

    /// Allocate a buffer for an explicit chroma format.
    pub fn new_with_format(
        width: usize,
        height: usize,
        picture_type: PictureType,
        tr: u16,
        chroma_format: ChromaFormat,
    ) -> Self {
        let mb_w = width.div_ceil(16);
        let mb_h = height.div_ceil(16);
        let y_stride = mb_w * 16;
        let y_h = mb_h * 16;
        let c_h_shift = chroma_format.chroma_h_shift();
        let c_v_shift = chroma_format.chroma_v_shift();
        let c_stride = y_stride >> c_h_shift;
        let c_h = y_h >> c_v_shift;
        Self {
            width,
            height,
            mb_width: mb_w,
            mb_height: mb_h,
            chroma_format,
            y: vec![0u8; y_stride * y_h],
            cb: vec![0u8; c_stride * c_h],
            cr: vec![0u8; c_stride * c_h],
            y_stride,
            c_stride,
            picture_type,
            temporal_reference: tr,
            display_pts: None,
        }
    }

    /// Output `PixelFormat` for the carried chroma format. This is what
    /// downstream consumers should expect from
    /// [`Self::to_video_frame`].
    pub fn output_pixel_format(&self) -> oxideav_core::PixelFormat {
        match self.chroma_format {
            ChromaFormat::Yuv420 => oxideav_core::PixelFormat::Yuv420P,
            ChromaFormat::Yuv422 => oxideav_core::PixelFormat::Yuv422P,
            ChromaFormat::Yuv444 => oxideav_core::PixelFormat::Yuv444P,
        }
    }

    /// Copy the MB-aligned luma / chroma buffers into a tight `VideoFrame`
    /// with no padding.
    pub fn to_video_frame(&self, pts: Option<i64>, _time_base: TimeBase) -> VideoFrame {
        let w = self.width;
        let h = self.height;
        let c_h_shift = self.chroma_format.chroma_h_shift();
        let c_v_shift = self.chroma_format.chroma_v_shift();
        // Round up so odd display sizes still get all chroma samples.
        let cw = (w + ((1 << c_h_shift) - 1)) >> c_h_shift;
        let ch = (h + ((1 << c_v_shift) - 1)) >> c_v_shift;
        let mut y = vec![0u8; w * h];
        for row in 0..h {
            y[row * w..row * w + w]
                .copy_from_slice(&self.y[row * self.y_stride..row * self.y_stride + w]);
        }
        let mut cb = vec![0u8; cw * ch];
        let mut cr = vec![0u8; cw * ch];
        for row in 0..ch {
            cb[row * cw..row * cw + cw]
                .copy_from_slice(&self.cb[row * self.c_stride..row * self.c_stride + cw]);
            cr[row * cw..row * cw + cw]
                .copy_from_slice(&self.cr[row * self.c_stride..row * self.c_stride + cw]);
        }
        VideoFrame {
            pts,
            planes: vec![
                VideoPlane { stride: w, data: y },
                VideoPlane {
                    stride: cw,
                    data: cb,
                },
                VideoPlane {
                    stride: cw,
                    data: cr,
                },
            ],
        }
    }
}

/// Manages the two reference pictures needed for P/B decode and the B-frame
/// reorder buffer.
///
/// MPEG-1 decoding semantics:
///   * I/P pictures are reference pictures. Each new I/P replaces the
///     older of the two references (sliding window of size 2).
///   * B pictures are never used as references. They are decoded after
///     the anchor they depend on (the "future" reference), so display
///     order re-orders them: an I/P "sandwich" holding between them.
///   * On decode, `prev_ref` is the forward anchor and `next_ref` is the
///     backward anchor. A B picture uses both; a P picture uses only
///     `next_ref` (which, for the first P after an I, equals `prev_ref`
///     at the point the P is decoded — conceptually the previous anchor).
#[derive(Default)]
pub struct ReferenceManager {
    /// The reference picture that appeared earliest in decode order and
    /// still has pending B pictures between it and the next anchor.
    pub prev_ref: Option<PictureBuffer>,
    /// The most recently decoded I/P picture (used as backward reference
    /// for B pictures that were decoded before but displayed before it).
    pub next_ref: Option<PictureBuffer>,
}

impl ReferenceManager {
    pub fn new() -> Self {
        Self::default()
    }

    /// Called after an I or P picture is fully decoded. Rotate the sliding
    /// window: old `next_ref` → `prev_ref`, new picture → `next_ref`. The
    /// previous `prev_ref` is dropped (its display was already emitted when
    /// it was rotated into that slot).
    ///
    /// Returns a clone of the picture that just moved from `next_ref` to
    /// `prev_ref` — the caller emits it now, since by MPEG-1 decode order
    /// no further B-pictures reference it as a backward anchor and all
    /// B-pictures that reference it as a forward anchor have just been
    /// queued (they are decoded between two anchors and emitted immediately).
    pub fn push_anchor(&mut self, pic: PictureBuffer) -> Option<PictureBuffer> {
        let ready_for_display = self.next_ref.clone();
        // Discard the now-unused forward anchor.
        self.prev_ref = self.next_ref.take();
        self.next_ref = Some(pic);
        ready_for_display
    }

    /// Consume the final reference picture on flush (`next_ref` — the
    /// most recently decoded anchor that no subsequent push has moved
    /// into display-ready state). `prev_ref` has already been emitted at
    /// rotation time.
    pub fn drain(&mut self) -> Vec<PictureBuffer> {
        let mut out = Vec::new();
        self.prev_ref.take();
        if let Some(p) = self.next_ref.take() {
            out.push(p);
        }
        out
    }

    pub fn forward(&self) -> Option<&PictureBuffer> {
        self.prev_ref.as_ref()
    }

    pub fn backward(&self) -> Option<&PictureBuffer> {
        self.next_ref.as_ref()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chroma_format_round_trip() {
        for fmt in [
            ChromaFormat::Yuv420,
            ChromaFormat::Yuv422,
            ChromaFormat::Yuv444,
        ] {
            assert_eq!(ChromaFormat::from_code(fmt.to_code()), Some(fmt));
        }
        assert_eq!(ChromaFormat::from_code(0), None);
    }

    #[test]
    fn blocks_per_mb_matches_spec() {
        assert_eq!(ChromaFormat::Yuv420.blocks_per_mb(), 6);
        assert_eq!(ChromaFormat::Yuv422.blocks_per_mb(), 8);
        assert_eq!(ChromaFormat::Yuv444.blocks_per_mb(), 12);
    }

    #[test]
    fn chroma_buffer_geometry_per_format() {
        // 32x32 frame:
        //   4:2:0 → chroma 16x16
        //   4:2:2 → chroma 16x32
        //   4:4:4 → chroma 32x32
        for (fmt, exp_cw, exp_ch) in [
            (ChromaFormat::Yuv420, 16, 16),
            (ChromaFormat::Yuv422, 16, 32),
            (ChromaFormat::Yuv444, 32, 32),
        ] {
            let pic = PictureBuffer::new_with_format(32, 32, PictureType::I, 0, fmt);
            assert_eq!(pic.c_stride, exp_cw, "{fmt:?} c_stride");
            let ch_actual = pic.cb.len() / pic.c_stride;
            assert_eq!(ch_actual, exp_ch, "{fmt:?} chroma height");
            let vf = pic.to_video_frame(None, TimeBase::new(1, 25));
            assert_eq!(vf.planes[1].stride, exp_cw, "{fmt:?} VideoFrame Cb stride");
            assert_eq!(
                vf.planes[1].data.len() / vf.planes[1].stride,
                exp_ch,
                "{fmt:?} VideoFrame Cb height"
            );
        }
    }

    #[test]
    fn yuv444_chroma_buffer_geometry() {
        // 4:4:4 chroma is the same size as luma. Sentinel-fill the chroma
        // buffer and confirm the VideoFrame copy preserves it.
        let mut pic =
            PictureBuffer::new_with_format(32, 32, PictureType::I, 0, ChromaFormat::Yuv444);
        for (i, p) in pic.cb.iter_mut().enumerate() {
            *p = (i & 0xff) as u8;
        }
        for (i, p) in pic.cr.iter_mut().enumerate() {
            *p = ((i ^ 0xa5) & 0xff) as u8;
        }
        let vf = pic.to_video_frame(None, TimeBase::new(1, 25));
        assert_eq!(vf.planes[1].stride, 32);
        assert_eq!(vf.planes[1].data.len(), 32 * 32);
        assert_eq!(vf.planes[2].data.len(), 32 * 32);
        // Spot-check a couple of pixels.
        assert_eq!(vf.planes[1].data[0], 0);
        assert_eq!(vf.planes[1].data[31], 31);
        assert_eq!(vf.planes[1].data[32], (32 & 0xff) as u8);
    }
}
