use criterion::{Criterion, black_box, criterion_group, criterion_main};
use jord::{Angle, NVector, spherical::Cap};

fn hundred_points() -> Vec<NVector> {
    vec![
        NVector::from_lat_long_degrees(-36.497396, 20.203591),
        NVector::from_lat_long_degrees(3.800008, 42.001923),
        NVector::from_lat_long_degrees(26.678974, -38.998303),
        NVector::from_lat_long_degrees(-37.187404, 44.88179),
        NVector::from_lat_long_degrees(-27.220886, 19.388044),
        NVector::from_lat_long_degrees(37.022863, 10.902337),
        NVector::from_lat_long_degrees(6.939775, 20.927087),
        NVector::from_lat_long_degrees(54.587828, -8.778959),
        NVector::from_lat_long_degrees(32.351004, -4.700535),
        NVector::from_lat_long_degrees(-43.415016, 14.588101),
        NVector::from_lat_long_degrees(20.889671, -42.432478),
        NVector::from_lat_long_degrees(-9.402729, 39.950563),
        NVector::from_lat_long_degrees(31.704508, 71.110611),
        NVector::from_lat_long_degrees(-4.45953, 21.926141),
        NVector::from_lat_long_degrees(47.727494, -35.473681),
        NVector::from_lat_long_degrees(11.768813, -42.41535),
        NVector::from_lat_long_degrees(-32.305451, 4.210206),
        NVector::from_lat_long_degrees(43.374066, -2.99146),
        NVector::from_lat_long_degrees(43.53331, -37.28722),
        NVector::from_lat_long_degrees(54.568313, -29.490972),
        NVector::from_lat_long_degrees(-37.369183, 27.796494),
        NVector::from_lat_long_degrees(15.408428, 39.789291),
        NVector::from_lat_long_degrees(7.87431, 28.299378),
        NVector::from_lat_long_degrees(12.645073, 30.458313),
        NVector::from_lat_long_degrees(36.448065, 55.038775),
        NVector::from_lat_long_degrees(-0.123308, 46.120243),
        NVector::from_lat_long_degrees(-17.618755, 0.288244),
        NVector::from_lat_long_degrees(42.771735, -27.469575),
        NVector::from_lat_long_degrees(12.175986, -12.236321),
        NVector::from_lat_long_degrees(26.279586, 29.658446),
        NVector::from_lat_long_degrees(39.010423, -46.841536),
        NVector::from_lat_long_degrees(23.42888, -31.773637),
        NVector::from_lat_long_degrees(-1.728743, -41.623494),
        NVector::from_lat_long_degrees(-17.132821, 17.590146),
        NVector::from_lat_long_degrees(11.847732, 45.186474),
        NVector::from_lat_long_degrees(-14.886625, 2.654077),
        NVector::from_lat_long_degrees(24.80924, 68.840118),
        NVector::from_lat_long_degrees(43.993127, 50.862728),
        NVector::from_lat_long_degrees(63.367712, 40.623372),
        NVector::from_lat_long_degrees(8.063621, 42.144742),
        NVector::from_lat_long_degrees(10.347589, 56.741251),
        NVector::from_lat_long_degrees(-25.744134, -16.012994),
        NVector::from_lat_long_degrees(1.481215, 48.064201),
        NVector::from_lat_long_degrees(69.665386, 3.433523),
        NVector::from_lat_long_degrees(-6.604397, 17.021948),
        NVector::from_lat_long_degrees(22.830923, -2.735958),
        NVector::from_lat_long_degrees(52.917035, 50.389917),
        NVector::from_lat_long_degrees(20.446863, 22.409534),
        NVector::from_lat_long_degrees(67.624968, -12.413394),
        NVector::from_lat_long_degrees(-26.881191, -35.54076),
        NVector::from_lat_long_degrees(11.065212, 5.847052),
        NVector::from_lat_long_degrees(56.63243, -6.346278),
        NVector::from_lat_long_degrees(27.480422, -13.821343),
        NVector::from_lat_long_degrees(27.477577, 20.50606),
        NVector::from_lat_long_degrees(-27.660689, 0.176984),
        NVector::from_lat_long_degrees(9.557925, 68.695491),
        NVector::from_lat_long_degrees(-8.759483, 49.115036),
        NVector::from_lat_long_degrees(-28.252249, -31.822554),
        NVector::from_lat_long_degrees(28.702029, -15.330136),
        NVector::from_lat_long_degrees(-16.550567, 49.892041),
        NVector::from_lat_long_degrees(59.041518, -9.963318),
        NVector::from_lat_long_degrees(60.662271, -5.840877),
        NVector::from_lat_long_degrees(10.613229, 13.245446),
        NVector::from_lat_long_degrees(2.764703, 8.556805),
        NVector::from_lat_long_degrees(-17.635671, -37.200229),
        NVector::from_lat_long_degrees(-20.083204, 23.650437),
        NVector::from_lat_long_degrees(-42.295562, -9.479839),
        NVector::from_lat_long_degrees(26.758108, 13.633375),
        NVector::from_lat_long_degrees(8.65836, -3.261094),
        NVector::from_lat_long_degrees(-25.404889, 50.974414),
        NVector::from_lat_long_degrees(47.625455, -5.1931),
        NVector::from_lat_long_degrees(-10.962394, -9.348936),
        NVector::from_lat_long_degrees(-0.406945, 48.691295),
        NVector::from_lat_long_degrees(12.205827, -31.893541),
        NVector::from_lat_long_degrees(18.599073, 37.272322),
        NVector::from_lat_long_degrees(36.033861, -47.826846),
        NVector::from_lat_long_degrees(48.328439, 6.057325),
        NVector::from_lat_long_degrees(6.300058, 31.89453),
        NVector::from_lat_long_degrees(37.588182, -9.707663),
        NVector::from_lat_long_degrees(3.919011, 39.299542),
        NVector::from_lat_long_degrees(20.132264, 0.091157),
        NVector::from_lat_long_degrees(-13.021552, -3.484957),
        NVector::from_lat_long_degrees(-39.135879, 39.136564),
        NVector::from_lat_long_degrees(22.337788, -14.528483),
        NVector::from_lat_long_degrees(-8.267105, 31.644587),
        NVector::from_lat_long_degrees(57.614317, -31.027035),
        NVector::from_lat_long_degrees(-0.263761, -27.08012),
        NVector::from_lat_long_degrees(-10.696558, 61.498922),
        NVector::from_lat_long_degrees(26.111697, 20.266551),
        NVector::from_lat_long_degrees(46.88351, 22.674023),
        NVector::from_lat_long_degrees(27.548917, -38.520469),
        NVector::from_lat_long_degrees(-37.087102, 50.755301),
        NVector::from_lat_long_degrees(27.002368, 46.876629),
        NVector::from_lat_long_degrees(5.27857, 67.659173),
        NVector::from_lat_long_degrees(33.686049, 21.340512),
        NVector::from_lat_long_degrees(14.178554, 50.568894),
        NVector::from_lat_long_degrees(-15.682587, -1.47788),
        NVector::from_lat_long_degrees(-15.242104, -17.852686),
        NVector::from_lat_long_degrees(-31.238511, 26.585129),
        NVector::from_lat_long_degrees(-20.602631, -40.477062),
    ]
}

pub fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("Cap::contains_position", |b| {
        let p1 = NVector::from_lat_long_degrees(-36.0, 143.0);
        let p2 = NVector::from_lat_long_degrees(-34.0, 145.0);
        let cap = Cap::from_centre_and_radius(p1, Angle::from_degrees(10.0));
        b.iter(|| black_box(cap.contains_position(p2)));
    });

    c.bench_function("Cap::smallest_enclosing_cap_100_pts", |b| {
        let ps = hundred_points();
        b.iter(|| black_box(Cap::smallest_enclosing_cap(&ps)));
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
