use criterion::{criterion_group, criterion_main, Criterion};
use jord::{spherical::MinorArc, NVector};

pub fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("MinorArc::intersection_some", |b| {
        let arc1: MinorArc = MinorArc::new(
            NVector::from_lat_long_degrees(-36.0, 143.0),
            NVector::from_lat_long_degrees(-34.0, 145.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(-34.0, 143.0),
            NVector::from_lat_long_degrees(-36.0, 145.0),
        );
        b.iter(|| arc1.intersection(arc2))
    });

    c.bench_function("MinorArc::intersection_none", |b| {
        let arc1: MinorArc = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(45.0, 0.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 90.0),
            NVector::from_lat_long_degrees(45.0, 90.0),
        );
        b.iter(|| arc1.intersection(arc2))
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
