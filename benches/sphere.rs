use criterion::{criterion_group, criterion_main, Criterion};
use jord::spherical::Sphere;
use jord::NVector;

pub fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("Sphere::distance", |b| {
        let p1 = NVector::from_lat_long_degrees(54.0, 154.0);
        let p2 = NVector::from_lat_long_degrees(-54.0, -154.0);
        b.iter(|| Sphere::EARTH.distance(p1, p2))
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
