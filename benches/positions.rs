use criterion::{criterion_group, criterion_main, Criterion};
use jord::{LatLong, NVector, Vec3};

pub fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("LatLong::from_nvector", |b| {
        let v = NVector::new(Vec3::new_unit(1.0, 2.0, 3.0));
        b.iter(|| LatLong::from_nvector(v))
    });

    c.bench_function("LatLong::to_nvector", |b| {
        let ll = LatLong::from_degrees(45.0, 45.0);
        b.iter(|| ll.to_nvector())
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
