//! Micro-benchmarks for the 8×8 forward and inverse DCT kernels.
//!
//! A realistic decode batch is ~100–1000 blocks per frame; we time batches
//! of 1000 blocks so the per-iteration numbers match frame-scale cost and
//! are dominated by the kernel (not loop overhead).

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use oxideav_mpeg12video::dct::{fdct8x8, idct8x8};

/// Synthesise a pseudo-realistic block of coefficients (DC + a sparse set
/// of AC lobes, similar to what the decoder produces after dequantisation).
fn mk_dequant_block(seed: u32) -> [f32; 64] {
    let mut b = [0.0f32; 64];
    // DC in the 0..2047 range.
    b[0] = 800.0 + (seed as f32 * 0.37) % 200.0;
    // A handful of low-frequency AC coefficients.
    for i in 1..10 {
        let v = (((seed.wrapping_mul(i as u32 + 1)) % 201) as i32 - 100) as f32;
        b[i * 3 % 64] = v;
    }
    b
}

fn mk_sample_block(seed: u32) -> [f32; 64] {
    // A plausible 8x8 sample block (values 0..255 with gradient + noise).
    let mut b = [0.0f32; 64];
    for j in 0..8 {
        for i in 0..8 {
            let noise = ((seed.wrapping_mul(17 + i as u32).wrapping_add(j as u32 * 13)) % 16) as f32;
            b[j * 8 + i] = 64.0 + (i + j) as f32 * 10.0 + noise;
        }
    }
    b
}

fn bench_idct(c: &mut Criterion) {
    let n = 1000usize;
    let blocks: Vec<[f32; 64]> = (0..n).map(|i| mk_dequant_block(i as u32 * 2654435761)).collect();
    let mut group = c.benchmark_group("idct");
    group.throughput(Throughput::Elements(n as u64));
    group.bench_function(BenchmarkId::new("batch", n), |b| {
        b.iter(|| {
            let mut local = blocks.clone();
            for blk in local.iter_mut() {
                idct8x8(blk);
            }
            local
        });
    });
    group.finish();
}

fn bench_fdct(c: &mut Criterion) {
    let n = 1000usize;
    let blocks: Vec<[f32; 64]> = (0..n).map(|i| mk_sample_block(i as u32 * 2654435761)).collect();
    let mut group = c.benchmark_group("fdct");
    group.throughput(Throughput::Elements(n as u64));
    group.bench_function(BenchmarkId::new("batch", n), |b| {
        b.iter(|| {
            let mut local = blocks.clone();
            for blk in local.iter_mut() {
                fdct8x8(blk);
            }
            local
        });
    });
    group.finish();
}

criterion_group!(benches, bench_idct, bench_fdct);
criterion_main!(benches);
