use criterion::{black_box, criterion_group, criterion_main, Criterion};
use jord::{spherical::Rectangle, Angle};

pub fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("Rectangle::union_overlapping", |b| {
        let r1 = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(20.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(10.0),
        );
        let r2 = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
            Angle::from_degrees(15.0),
            Angle::from_degrees(15.0),
        );
        b.iter(|| black_box(r1.union(r2)))
    });

    c.bench_function("Rectangle::union_non_overlapping", |b| {
        let r1 = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(20.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(10.0),
        );
        let r2 = Rectangle::from_nesw(
            Angle::from_degrees(40.0),
            Angle::from_degrees(45.0),
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
        );
        b.iter(|| black_box(r1.union(r2)))
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
