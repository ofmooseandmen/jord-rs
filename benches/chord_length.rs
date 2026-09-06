use criterion::{Criterion, black_box, criterion_group, criterion_main};
use jord::{NVector, spherical::ChordLength};

pub fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("ChordLength::new", |b| {
        let p1 = NVector::from_lat_long_degrees(-36.0, 143.0);
        let p2 = NVector::from_lat_long_degrees(-34.0, 145.0);
        b.iter(|| black_box(ChordLength::new(p1, p2)))
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
