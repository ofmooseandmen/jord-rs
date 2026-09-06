use criterion::{Criterion, black_box, criterion_group, criterion_main};
use jord::{Angle, NVector, spherical::Cap};

pub fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("Cap::contains_position", |b| {
        let p1 = NVector::from_lat_long_degrees(-36.0, 143.0);
        let p2 = NVector::from_lat_long_degrees(-34.0, 145.0);
        let cap = Cap::from_centre_and_radius(p1, Angle::from_degrees(10.0));
        b.iter(|| black_box(cap.contains_position(p2)));
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
