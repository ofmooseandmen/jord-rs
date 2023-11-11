use criterion::{criterion_group, criterion_main, Criterion};
use jord::Vec3;

pub fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("Vec3::orthogonal_to", |b| {
        let v1 = Vec3::new(1.0, 0.0, 0.0);
        let v2 = Vec3::new(0.0, 1.0, 0.0);
        b.iter(|| v1.orthogonal_to(v2));
    });

    c.bench_function("Vec3::stable_cross_prod_unit", |b| {
        let v1 = Vec3::new(1.0, 0.0, 0.0);
        let v2 = Vec3::new(0.0, 1.0, 0.0);
        b.iter(|| v1.stable_cross_prod_unit(v2));
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
