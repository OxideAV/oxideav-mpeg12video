//! Picture-level frame assembly: §6.1 block-to-sample layout and the
//! §7.6.8 intra reconstruction write-out.
//!
//! The per-block reconstruction pipeline ([`crate::mpeg2_block_decoder`])
//! produces, for every coded block, the §A IDCT output plane
//! `f[y][x]` in the §7.5 9-bit signed pel range. For an **intra**
//! macroblock the final decoded sample is simply that plane
//! saturated to `[0, 255]` (§7.6.8, with the prediction conceptually
//! all-zero — the intra DC level shift is already folded into the
//! §7.2.1 DC predictor reset value, Table 7-2). This module places
//! those 8×8 reconstructed blocks into a full-picture sample buffer
//! using the §6.1.1.8 block ordering (Figures 6-10 / 6-11 / 6-12) and
//! the §6.1.3 frame-vs-field DCT internal organisation (Figures 6-13 /
//! 6-14).
//!
//! What this module is:
//!
//! * A clean-room composition of already-landed, spec-cited stages —
//!   it adds the *spatial placement* arithmetic (which 8×8 block goes
//!   where in the frame, and how field-DCT lines are de-interleaved)
//!   that the per-block pipeline did not own.
//! * Restricted to the **intra** reconstruction path: I-pictures (and
//!   intra macroblocks of P/B pictures) decode to full pixels with no
//!   motion-compensated prediction. The non-intra path needs the
//!   §7.6.4 pel reader threaded against reference frames and is a
//!   later milestone.
//!
//! Layout summary (per §6.1.1.8 / §6.1.3):
//!
//! * The 4 luminance blocks of a macroblock tile a 16×16 luma region.
//!   In §6.1.1.8 raster order they are block 0 = top-left, block 1 =
//!   top-right, block 2 = bottom-left, block 3 = bottom-right (Figure
//!   6-10).
//! * For **frame DCT** coding each luma block holds 8 consecutive
//!   picture lines (Figure 6-13): block rows map 1:1 to frame rows.
//! * For **field DCT** coding each luma block holds lines from only
//!   one field (Figure 6-14): the top pair (blocks 0, 1) carry the 8
//!   even (field-0) lines of the 16-line region and the bottom pair
//!   (blocks 2, 3) carry the 8 odd (field-1) lines. The block's row
//!   `r` maps to frame row `2*r + field` within the macroblock.
//! * Chrominance blocks tile the chroma region. For 4:2:0 a
//!   macroblock has one Cb and one Cr 8×8 block covering the 8×8
//!   chroma region, **always** frame-organised (§6.1.3). For 4:2:2 a
//!   macroblock has two Cb (top/bottom) and two Cr blocks tiling an
//!   8×16 chroma region; for 4:4:4 four Cb and four Cr blocks tiling
//!   the full 16×16 chroma region, both following the same
//!   frame/field organisation as luma.

use crate::add_coefficients::saturate;
use crate::mpeg2_block_dc::ColourComponent;
use crate::sequence_extension::ChromaFormat;

/// One reconstructed colour component plane: a row-major `width ×
/// height` buffer of decoded 8-bit samples.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Plane {
    width: usize,
    height: usize,
    samples: Vec<u8>,
}

impl Plane {
    /// Allocate a `width × height` plane initialised to `0`.
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            samples: vec![0u8; width * height],
        }
    }

    /// Plane width in samples.
    pub fn width(&self) -> usize {
        self.width
    }

    /// Plane height in samples.
    pub fn height(&self) -> usize {
        self.height
    }

    /// Row-major sample slice (`width * height` entries).
    pub fn samples(&self) -> &[u8] {
        &self.samples
    }

    /// Sample at `(x, y)`. Out-of-bounds coordinates return `None`.
    pub fn get(&self, x: usize, y: usize) -> Option<u8> {
        if x >= self.width || y >= self.height {
            return None;
        }
        Some(self.samples[y * self.width + x])
    }

    /// Write `value` at `(x, y)`. Out-of-bounds writes are silently
    /// dropped — a picture's macroblock grid is padded up to a
    /// multiple of 16, so the bottom / right macroblocks legitimately
    /// reconstruct samples that fall outside the coded
    /// `horizontal_size × vertical_size` and must be discarded.
    fn put(&mut self, x: usize, y: usize, value: u8) {
        if x < self.width && y < self.height {
            self.samples[y * self.width + x] = value;
        }
    }
}

/// A reconstructed picture: the three colour-component planes.
///
/// The Y plane is `mb_width*16 × mb_height*16` clipped to the coded
/// `horizontal_size × vertical_size`. The chroma plane dimensions are
/// derived from the [`ChromaFormat`] subsampling (§6.1.1):
///
/// * 4:2:0 — chroma is half width, half height.
/// * 4:2:2 — chroma is half width, full height.
/// * 4:4:4 — chroma is full width, full height.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FrameBuffer {
    /// Coded picture width (`horizontal_size`).
    pub width: usize,
    /// Coded picture height (`vertical_size`).
    pub height: usize,
    /// Chroma sampling.
    pub chroma_format: ChromaFormat,
    /// Luminance plane.
    pub y: Plane,
    /// Cb chrominance plane.
    pub cb: Plane,
    /// Cr chrominance plane.
    pub cr: Plane,
}

/// Horizontal / vertical chroma subsampling shift for a
/// [`ChromaFormat`]: `(shift_x, shift_y)` such that the chroma plane
/// dimension is `(luma + (1 << shift) - 1) >> shift`.
///
/// * 4:2:0 → `(1, 1)` (half width, half height).
/// * 4:2:2 → `(1, 0)` (half width, full height).
/// * 4:4:4 → `(0, 0)` (full resolution).
pub fn chroma_shift(chroma_format: ChromaFormat) -> (u32, u32) {
    match chroma_format {
        ChromaFormat::Yuv420 => (1, 1),
        ChromaFormat::Yuv422 => (1, 0),
        ChromaFormat::Yuv444 => (0, 0),
    }
}

impl FrameBuffer {
    /// Allocate a frame buffer sized to the coded picture dimensions.
    ///
    /// The luma plane is exactly `width × height`; the chroma planes
    /// are subsampled per [`chroma_shift`]. All samples start at `0`.
    pub fn new(width: usize, height: usize, chroma_format: ChromaFormat) -> Self {
        let (sx, sy) = chroma_shift(chroma_format);
        let cw = (width + ((1usize << sx) - 1)) >> sx;
        let ch = (height + ((1usize << sy) - 1)) >> sy;
        Self {
            width,
            height,
            chroma_format,
            y: Plane::new(width, height),
            cb: Plane::new(cw, ch),
            cr: Plane::new(cw, ch),
        }
    }

    /// Mutable access to the plane for a colour component.
    fn plane_mut(&mut self, component: ColourComponent) -> &mut Plane {
        match component {
            ColourComponent::Y => &mut self.y,
            ColourComponent::Cb => &mut self.cb,
            ColourComponent::Cr => &mut self.cr,
        }
    }
}

/// The top-left sample coordinate, in that component's plane, of the
/// 8×8 block `i` (§6.1.1.8 order) within the macroblock at macroblock
/// raster column / row `(mb_col, mb_row)`, together with whether the
/// block's rows are field-interleaved (field DCT) and which field
/// (`0` = top, `1` = bottom) it carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockPlacement {
    /// Colour component the block belongs to.
    pub component: ColourComponent,
    /// Top-left X sample coordinate in the component plane.
    pub base_x: usize,
    /// Top-left Y sample coordinate in the component plane (the
    /// first frame row the block's row 0 maps to).
    pub base_y: usize,
    /// `true` when the block's 8 rows are field-organised — i.e. they
    /// occupy frame rows `base_y, base_y+2, base_y+4, …` (stride 2)
    /// rather than `base_y, base_y+1, …` (stride 1). Field DCT only
    /// applies to luma (and, for 4:2:2 / 4:4:4, chroma); 4:2:0 chroma
    /// is always frame-organised per §6.1.3.
    pub field_dct: bool,
}

impl BlockPlacement {
    /// Frame-row stride between successive block rows: `2` for a
    /// field-DCT block, `1` otherwise.
    pub fn row_stride(self) -> usize {
        if self.field_dct {
            2
        } else {
            1
        }
    }
}

/// Resolve the §6.1.1.8 spatial placement of block `i` for the
/// macroblock at `(mb_col, mb_row)`.
///
/// `field_dct` is the macroblock's `dct_type` (`true` = field DCT,
/// from §6.2.5.1 / Table 6-19); it is ignored for 4:2:0 chroma
/// blocks, which are always frame-organised (§6.1.3).
///
/// Returns `None` when `i` is not a valid block index for
/// `chroma_format` (i.e. `i >= block_count`).
pub fn block_placement(
    i: usize,
    chroma_format: ChromaFormat,
    mb_col: usize,
    mb_row: usize,
    field_dct: bool,
) -> Option<BlockPlacement> {
    let count = crate::mpeg2_macroblock_blocks::block_count(chroma_format);
    if i >= count {
        return None;
    }
    let component = crate::mpeg2_macroblock_blocks::block_component(i, chroma_format)?;

    // Luma: 16×16 macroblock, blocks 0..4 tile it.
    if matches!(component, ColourComponent::Y) {
        let luma_x0 = mb_col * 16;
        let luma_y0 = mb_row * 16;
        // §6.1.1.8 Figure 6-10/6-13/6-14: blocks 0,1 are the top
        // 8 rows of the macroblock, blocks 2,3 the bottom 8; 0,2 are
        // the left column, 1,3 the right column.
        let left = (i % 2) == 0; // 0,2 -> left; 1,3 -> right
        let top = i < 2; // 0,1 -> top pair; 2,3 -> bottom pair
        let base_x = luma_x0 + if left { 0 } else { 8 };
        return Some(luma_or_chroma_placement(
            component, base_x, luma_y0, top, field_dct,
        ));
    }

    // Chroma. Index within the component's own block run.
    let chroma_index = chroma_block_index(i, chroma_format, component);
    let (sx, sy) = chroma_shift(chroma_format);
    let chroma_mb_w = 16usize >> sx; // chroma samples per MB, horizontally
    let chroma_mb_h = 16usize >> sy; // chroma samples per MB, vertically
    let chroma_x0 = mb_col * chroma_mb_w;
    let chroma_y0 = mb_row * chroma_mb_h;

    // For 4:2:0 (chroma_mb_h == 8) there is exactly one chroma block
    // per component, frame-organised.
    if chroma_mb_h == 8 {
        return Some(BlockPlacement {
            component,
            base_x: chroma_x0,
            base_y: chroma_y0,
            field_dct: false,
        });
    }

    // 4:2:2 / 4:4:4: chroma_mb_h == 16, so the component has 2 (4:2:2)
    // or 4 (4:4:4) blocks tiling the chroma region exactly like luma.
    // chroma_mb_w is 8 (4:2:2) -> single column -> chroma_index in
    // {0 (top), 1 (bottom)}; or 16 (4:4:4) -> 2×2 tiling with the same
    // 0=TL,1=TR,2=BL,3=BR §6.1.1.8 ordering as luma.
    let blocks_wide = chroma_mb_w / 8;
    let left = (chroma_index % blocks_wide) == 0;
    let top = chroma_index < blocks_wide;
    let base_x = chroma_x0 + if left { 0 } else { 8 };
    Some(luma_or_chroma_placement(
        component, base_x, chroma_y0, top, field_dct,
    ))
}

/// Shared frame/field-DCT vertical placement for a luma block (or a
/// 4:2:2 / 4:4:4 chroma block) given whether it is the top or bottom
/// block of its 16-row column.
fn luma_or_chroma_placement(
    component: ColourComponent,
    base_x: usize,
    region_y0: usize,
    top: bool,
    field_dct: bool,
) -> BlockPlacement {
    if field_dct {
        // Field DCT (§6.1.3 / Figure 6-14): the top pair carry field 0
        // (even frame rows), the bottom pair carry field 1 (odd frame
        // rows), each with a frame-row stride of 2.
        let field = if top { 0 } else { 1 };
        BlockPlacement {
            component,
            base_x,
            base_y: region_y0 + field,
            field_dct: true,
        }
    } else {
        // Frame DCT (§6.1.3 / Figure 6-13): consecutive 8-row stacks.
        let base_y = region_y0 + if top { 0 } else { 8 };
        BlockPlacement {
            component,
            base_x,
            base_y,
            field_dct: false,
        }
    }
}

/// Index of block `i` within its own colour component's run of blocks
/// (Cb / Cr), per the §6.1.1.8 ordering. For luma this is just `i`.
fn chroma_block_index(i: usize, chroma_format: ChromaFormat, component: ColourComponent) -> usize {
    let pair = match chroma_format {
        ChromaFormat::Yuv420 => 1,
        ChromaFormat::Yuv422 => 2,
        ChromaFormat::Yuv444 => 4,
    };
    match component {
        ColourComponent::Y => i,
        ColourComponent::Cb => i - 4,
        ColourComponent::Cr => i - 4 - pair,
    }
}

/// Write one intra-reconstructed 8×8 block plane `f_pel` (the §A IDCT
/// output in the §7.5 9-bit signed range) into `frame` at the
/// resolved [`BlockPlacement`], applying the §7.6.8 intra saturation
/// (`d = saturate(f)` with the `[0, 255]` clamp).
///
/// The macroblock grid is padded up to a multiple of 16 macroblocks,
/// so block samples that fall outside the coded picture dimensions are
/// silently discarded by [`Plane::put`].
pub fn place_intra_block(
    frame: &mut FrameBuffer,
    placement: BlockPlacement,
    f_pel: &[[i16; 8]; 8],
) {
    let stride = placement.row_stride();
    let plane = frame.plane_mut(placement.component);
    for (r, row) in f_pel.iter().enumerate() {
        let y = placement.base_y + r * stride;
        for (c, &sample) in row.iter().enumerate() {
            let x = placement.base_x + c;
            plane.put(x, y, saturate(sample as i32));
        }
    }
}

/// Place every coded block of one **intra** [`MacroblockRecord`] into
/// `frame`, mapping each `decoded_blocks` entry back to its §6.1.1.8
/// block index `i` via the record's `pattern_code[12]` array and
/// resolving its spatial placement with [`block_placement`].
///
/// The macroblock's raster address determines its `(mb_col, mb_row)`:
/// `mb_col = macroblock_address % mb_width`, `mb_row = macroblock_address
/// / mb_width`. The record's `dct_type` selects the §6.1.3 frame-vs-field
/// organisation (`Some(true)` = field DCT; `None` / `Some(false)` =
/// frame DCT — the Table 6-19 default when the field is absent is frame
/// DCT).
///
/// This is the **intra-only** write path: it asserts the record is an
/// intra macroblock and writes `saturate(f_pel)` per §7.6.8. Callers
/// reconstruct an I-picture by invoking this for every record of every
/// slice (P/B intra macroblocks also route here; their inter
/// neighbours need the not-yet-composed §7.6.4 pel reader).
///
/// Returns the number of blocks written. A record whose
/// `decoded_blocks` is `None` (the walker ran in wire-only mode) writes
/// nothing and returns `0`.
pub fn place_intra_macroblock(
    frame: &mut FrameBuffer,
    record: &crate::slice_macroblock_walk::MacroblockRecord,
    mb_width: usize,
    chroma_format: ChromaFormat,
) -> usize {
    let Some(blocks) = record.decoded_blocks.as_ref() else {
        return 0;
    };
    if !record.macroblock_type.macroblock_intra {
        return 0;
    }
    let mb_col = (record.macroblock_address as usize) % mb_width;
    let mb_row = (record.macroblock_address as usize) / mb_width;
    // §6.1.3 / Table 6-19: field DCT only when dct_type == Some(true);
    // an absent dct_type defaults to frame DCT.
    let field_dct = record.dct_type == Some(true);

    let mut written = 0usize;
    // Each decoded block carries its own §6.1.1.8 block_index, so the
    // placement is resolved directly from that — no re-derivation from
    // pattern_code is needed.
    for decoded in blocks {
        let i = decoded.block_index as usize;
        if let Some(placement) = block_placement(i, chroma_format, mb_col, mb_row, field_dct) {
            place_intra_block(frame, placement, &decoded.decoded.f_pel);
            written += 1;
        }
    }
    written
}

/// The fixed per-picture parameters the intra picture driver needs:
/// the coded picture geometry, the chroma format, and the §6.2.3.1
/// `picture_coding_extension()` fields that gate the per-block
/// reconstruction pipeline.
///
/// These come straight from the already-landed
/// [`crate::sequence_extension::Mpeg2Sequence`] (width / height /
/// chroma) and [`crate::picture_header::PictureCodingExtension`]
/// (the four DCT-context flags), so the driver does not re-parse the
/// sequence layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IntraPictureParams {
    /// Coded picture width (`horizontal_size`).
    pub width: usize,
    /// Coded picture height (`vertical_size`).
    pub height: usize,
    /// Chroma sampling (§6.2.2.3).
    pub chroma_format: ChromaFormat,
    /// `frame_pred_frame_dct` (§6.2.3.1). When `true` the
    /// §6.2.5.1 `dct_type` field is absent (Table 6-19 supplies the
    /// frame-DCT default); when `false` each coded / intra macroblock
    /// in a frame picture carries an explicit `dct_type` bit.
    pub frame_pred_frame_dct: bool,
    /// `intra_dc_precision` (§6.2.3.1, Table 6-13, `0..=3`).
    pub intra_dc_precision: u8,
    /// `intra_vlc_format` (§6.2.3.1).
    pub intra_vlc_format: bool,
    /// `alternate_scan` (§6.2.3.1).
    pub alternate_scan: bool,
    /// `q_scale_type` (§6.2.3.1).
    pub q_scale_type: bool,
}

impl IntraPictureParams {
    /// Number of macroblocks across the picture (`Ceil(width / 16)`).
    pub fn mb_width(&self) -> usize {
        self.width.div_ceil(16)
    }

    /// Number of macroblock rows in the picture (`Ceil(height / 16)`).
    pub fn mb_height(&self) -> usize {
        self.height.div_ceil(16)
    }
}

/// Decode a whole **intra** (I) picture into a [`FrameBuffer`].
///
/// `picture` is the slice of the elementary stream from the first
/// `slice_start_code` of the picture (i.e. `0x00000101`..) up to but
/// not including the next picture / GOP / sequence start code. The
/// driver:
///
/// 1. Scans for each `slice_start_code` (`0x00000101`..`0x000001AF`,
///    §6.2.4 / Table 6-1).
/// 2. Parses its [`crate::SliceHeader`] for the `mb_row` (=
///    `slice_vertical_position - 1` for `vertical_size <= 2800`,
///    §6.3.16) and the per-slice `quantiser_scale_code`.
/// 3. Walks the slice with the §6.2.6 `block(i)` pipeline enabled
///    ([`crate::walk_slice_at`] from the header's `body_bit_position`).
/// 4. Places every intra macroblock's reconstructed blocks into the
///    frame via [`place_intra_macroblock`].
///
/// Returns the assembled [`FrameBuffer`] and the number of
/// macroblocks placed. This is the **intra-only** picture path: it is
/// a complete decode for an I-picture (where every macroblock is
/// intra) and a partial one for P/B pictures (only their intra
/// macroblocks are written; inter macroblocks need the not-yet-
/// composed §7.6.4 motion-compensated prediction).
///
/// # Errors
///
/// Propagates any [`crate::Error`] from slice-header parsing or the
/// macroblock walk. A picture with no slice start codes yields an
/// all-zero frame and a count of `0`.
pub fn decode_intra_picture(
    picture: &[u8],
    params: IntraPictureParams,
) -> crate::Result<(FrameBuffer, usize)> {
    use crate::picture_header::PictureStructure;
    use crate::slice_header::{SliceContext, SliceHeader};
    use crate::slice_macroblock_walk::SliceWalkContext;

    let mut frame = FrameBuffer::new(params.width, params.height, params.chroma_format);
    let mb_width = params.mb_width() as u32;
    let slice_ctx = SliceContext::non_scalable(params.height as u32);

    let mut placed = 0usize;
    let mut offset = 0usize;
    while let Some(rel) = find_slice_start_code(&picture[offset..]) {
        let start = offset + rel;
        // The slice runs until the next start code (any 0x000001??).
        let body = &picture[start..];
        let end = find_next_start_code(&body[4..])
            .map(|p| p + 4)
            .unwrap_or(body.len());
        let slice_buf = &body[..end];

        let header = SliceHeader::parse(slice_buf, slice_ctx)?;
        // §6.3.16: for vertical_size <= 2800 the macroblock row is
        // slice_vertical_position - 1 (no slice_vertical_position_extension).
        let mb_row = u32::from(header.slice_vertical_position) - 1;

        let ctx = SliceWalkContext::first_slice_with_block_decoding(
            mb_width,
            mb_row,
            crate::picture_header::PictureCodingType::Intra,
            header.quantiser_scale_code,
            PictureStructure::Frame,
            params.frame_pred_frame_dct,
            15,
            15,
            15,
            15,
            false,
            params.chroma_format,
            params.intra_vlc_format,
            params.alternate_scan,
            params.intra_dc_precision,
            params.q_scale_type,
        );

        let walk = crate::walk_slice_at(slice_buf, header.body_bit_position, ctx)?;
        for record in &walk.macroblocks {
            placed +=
                place_intra_macroblock(&mut frame, record, mb_width as usize, params.chroma_format);
        }

        offset = start + end;
    }

    Ok((frame, placed))
}

/// Find the byte offset of the next `slice_start_code`
/// (`0x000001` prefix + a `0x01..=0xAF` value byte, Table 6-1) in
/// `buf`, or `None`.
fn find_slice_start_code(buf: &[u8]) -> Option<usize> {
    buf.windows(4)
        .position(|w| w[0] == 0x00 && w[1] == 0x00 && w[2] == 0x01 && (0x01..=0xAF).contains(&w[3]))
}

/// Find the byte offset of the next start code of any kind
/// (`0x000001` prefix), or `None`.
fn find_next_start_code(buf: &[u8]) -> Option<usize> {
    buf.windows(3)
        .position(|w| w[0] == 0x00 && w[1] == 0x00 && w[2] == 0x01)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ramp_block(base: i16) -> [[i16; 8]; 8] {
        let mut b = [[0i16; 8]; 8];
        for (r, row) in b.iter_mut().enumerate() {
            for (c, s) in row.iter_mut().enumerate() {
                *s = base + (r as i16) * 8 + c as i16;
            }
        }
        b
    }

    #[test]
    fn frame_buffer_chroma_dimensions_420() {
        let f = FrameBuffer::new(352, 240, ChromaFormat::Yuv420);
        assert_eq!((f.y.width(), f.y.height()), (352, 240));
        assert_eq!((f.cb.width(), f.cb.height()), (176, 120));
        assert_eq!((f.cr.width(), f.cr.height()), (176, 120));
    }

    #[test]
    fn frame_buffer_chroma_dimensions_422_444() {
        let f = FrameBuffer::new(352, 240, ChromaFormat::Yuv422);
        assert_eq!((f.cb.width(), f.cb.height()), (176, 240));
        let f = FrameBuffer::new(352, 240, ChromaFormat::Yuv444);
        assert_eq!((f.cb.width(), f.cb.height()), (352, 240));
    }

    #[test]
    fn luma_block_placement_420_frame_dct() {
        let cf = ChromaFormat::Yuv420;
        // MB at column 2, row 3 -> luma origin (32, 48).
        let p0 = block_placement(0, cf, 2, 3, false).unwrap();
        assert_eq!(
            (p0.component, p0.base_x, p0.base_y, p0.field_dct),
            (ColourComponent::Y, 32, 48, false)
        );
        let p1 = block_placement(1, cf, 2, 3, false).unwrap();
        assert_eq!((p1.base_x, p1.base_y), (40, 48));
        let p2 = block_placement(2, cf, 2, 3, false).unwrap();
        assert_eq!((p2.base_x, p2.base_y), (32, 56));
        let p3 = block_placement(3, cf, 2, 3, false).unwrap();
        assert_eq!((p3.base_x, p3.base_y), (40, 56));
    }

    #[test]
    fn luma_block_placement_field_dct_strides_rows() {
        let cf = ChromaFormat::Yuv420;
        // Field DCT: top pair carry even rows, bottom pair odd rows.
        let p0 = block_placement(0, cf, 0, 0, true).unwrap();
        assert_eq!((p0.base_y, p0.field_dct, p0.row_stride()), (0, true, 2));
        let p2 = block_placement(2, cf, 0, 0, true).unwrap();
        assert_eq!((p2.base_y, p2.field_dct, p2.row_stride()), (1, true, 2));
        // Left/right columns unaffected by field_dct.
        let p1 = block_placement(1, cf, 0, 0, true).unwrap();
        assert_eq!(p1.base_x, 8);
    }

    #[test]
    fn chroma_block_placement_420_always_frame() {
        let cf = ChromaFormat::Yuv420;
        // Cb = block 4, Cr = block 5. MB col 2 row 3 -> chroma origin
        // (16, 24) (half-res).
        let cb = block_placement(4, cf, 2, 3, true).unwrap();
        assert_eq!(
            (cb.component, cb.base_x, cb.base_y, cb.field_dct),
            (ColourComponent::Cb, 16, 24, false)
        );
        let cr = block_placement(5, cf, 2, 3, true).unwrap();
        assert_eq!(
            (cr.component, cr.base_x, cr.base_y),
            (ColourComponent::Cr, 16, 24)
        );
    }

    #[test]
    fn chroma_block_placement_422() {
        let cf = ChromaFormat::Yuv422;
        // 4:2:2: blocks 4,5 = Cb (top,bottom), 6,7 = Cr. chroma is
        // 8 wide × 16 tall per MB. MB col 1 row 0 -> chroma origin
        // (8, 0).
        let cb_top = block_placement(4, cf, 1, 0, false).unwrap();
        assert_eq!(
            (cb_top.component, cb_top.base_x, cb_top.base_y),
            (ColourComponent::Cb, 8, 0)
        );
        let cb_bot = block_placement(5, cf, 1, 0, false).unwrap();
        assert_eq!(
            (cb_bot.component, cb_bot.base_x, cb_bot.base_y),
            (ColourComponent::Cb, 8, 8)
        );
        let cr_top = block_placement(6, cf, 1, 0, false).unwrap();
        assert_eq!((cr_top.component, cr_top.base_y), (ColourComponent::Cr, 0));
        let cr_bot = block_placement(7, cf, 1, 0, false).unwrap();
        assert_eq!(cr_bot.base_y, 8);
    }

    #[test]
    fn chroma_block_placement_444_2x2_tiling() {
        let cf = ChromaFormat::Yuv444;
        // 4:4:4: blocks 4..8 = Cb (TL,TR,BL,BR), 8..12 = Cr. chroma is
        // 16×16 per MB.
        let cb_tl = block_placement(4, cf, 0, 0, false).unwrap();
        assert_eq!((cb_tl.base_x, cb_tl.base_y), (0, 0));
        let cb_tr = block_placement(5, cf, 0, 0, false).unwrap();
        assert_eq!((cb_tr.base_x, cb_tr.base_y), (8, 0));
        let cb_bl = block_placement(6, cf, 0, 0, false).unwrap();
        assert_eq!((cb_bl.base_x, cb_bl.base_y), (0, 8));
        let cb_br = block_placement(7, cf, 0, 0, false).unwrap();
        assert_eq!((cb_br.base_x, cb_br.base_y), (8, 8));
        let cr_tl = block_placement(8, cf, 0, 0, false).unwrap();
        assert_eq!(
            (cr_tl.component, cr_tl.base_x, cr_tl.base_y),
            (ColourComponent::Cr, 0, 0)
        );
    }

    #[test]
    fn out_of_range_block_index_is_none() {
        assert!(block_placement(6, ChromaFormat::Yuv420, 0, 0, false).is_none());
        assert!(block_placement(8, ChromaFormat::Yuv422, 0, 0, false).is_none());
        assert!(block_placement(12, ChromaFormat::Yuv444, 0, 0, false).is_none());
    }

    #[test]
    fn place_intra_block_frame_dct_writes_contiguous_rows() {
        let mut frame = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
        let block = ramp_block(100);
        let p = block_placement(0, ChromaFormat::Yuv420, 0, 0, false).unwrap();
        place_intra_block(&mut frame, p, &block);
        // Block row r, col c -> frame (c, r) with value 100 + r*8 + c.
        for r in 0..8 {
            for c in 0..8 {
                assert_eq!(
                    frame.y.get(c, r),
                    Some((100 + (r as i16) * 8 + c as i16) as u8)
                );
            }
        }
        // Row 8 (outside this block) untouched.
        assert_eq!(frame.y.get(0, 8), Some(0));
    }

    #[test]
    fn place_intra_block_field_dct_strides_into_even_rows() {
        let mut frame = FrameBuffer::new(16, 16, ChromaFormat::Yuv420);
        let block = ramp_block(50);
        let p = block_placement(0, ChromaFormat::Yuv420, 0, 0, true).unwrap();
        place_intra_block(&mut frame, p, &block);
        // Field-0 block row r -> frame row 2*r. Odd rows stay 0.
        for r in 0..8 {
            assert_eq!(frame.y.get(0, 2 * r), Some((50 + (r as i16) * 8) as u8));
            assert_eq!(frame.y.get(0, 2 * r + 1), Some(0));
        }
    }

    #[test]
    fn place_intra_block_saturates_to_byte_range() {
        let mut frame = FrameBuffer::new(8, 8, ChromaFormat::Yuv420);
        let mut block = [[0i16; 8]; 8];
        block[0][0] = 300; // above 255 -> clamps to 255
        block[0][1] = -5; // below 0 -> clamps to 0
        block[0][2] = 200;
        let p = block_placement(0, ChromaFormat::Yuv420, 0, 0, false).unwrap();
        place_intra_block(&mut frame, p, &block);
        assert_eq!(frame.y.get(0, 0), Some(255));
        assert_eq!(frame.y.get(1, 0), Some(0));
        assert_eq!(frame.y.get(2, 0), Some(200));
    }

    #[test]
    fn padding_macroblock_samples_outside_picture_are_discarded() {
        // 8×8 picture but a full 16×16 macroblock — the right/bottom
        // luma blocks fall outside and must be dropped, not panic.
        let mut frame = FrameBuffer::new(8, 8, ChromaFormat::Yuv420);
        let block = ramp_block(10);
        for i in 0..4 {
            let p = block_placement(i, ChromaFormat::Yuv420, 0, 0, false).unwrap();
            place_intra_block(&mut frame, p, &block);
        }
        // Only block 0 lands inside an 8×8 picture.
        assert_eq!(frame.y.get(7, 7), Some((10 + 7 * 8 + 7) as u8));
    }
}
