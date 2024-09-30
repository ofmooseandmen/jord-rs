use criterion::{black_box, criterion_group, criterion_main, Bencher, Criterion};
use jord::spherical::{is_loop_clockwise, Loop};
use jord::{Angle, NVector};

pub fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("Loop::new_5_vertices", |b| {
        let vertices: Vec<NVector> = vec![
            NVector::from_lat_long_degrees(55.605, 13.0038),
            NVector::from_lat_long_degrees(55.4295, 13.82),
            NVector::from_lat_long_degrees(56.0294, 14.1567),
            NVector::from_lat_long_degrees(56.0465, 12.6945),
            NVector::from_lat_long_degrees(55.7047, 13.191),
        ];
        b.iter(|| black_box(Loop::new(&vertices)))
    });

    c.bench_function("Loop::new_94_clockwise_vertices", |b| {
        let vertices: Vec<NVector> = vertices_94();
        assert!(is_loop_clockwise(&vertices));
        b.iter(|| black_box(Loop::new(&vertices)))
    });

    c.bench_function("Loop::new_94_anticlockwise_vertices", |b| {
        let mut vertices: Vec<NVector> = vertices_94();
        vertices.reverse();
        assert!(!is_loop_clockwise(&vertices));
        b.iter(|| black_box(Loop::new(&vertices)))
    });

    c.bench_function("Loop::contains_point_true_5_vertices", |b| {
        let vertices: Vec<NVector> = vec![
            NVector::from_lat_long_degrees(55.605, 13.0038),
            NVector::from_lat_long_degrees(55.4295, 13.82),
            NVector::from_lat_long_degrees(56.0294, 14.1567),
            NVector::from_lat_long_degrees(56.0465, 12.6945),
            NVector::from_lat_long_degrees(55.7047, 13.191),
        ];
        let l = Loop::new(&vertices);
        let inside = NVector::from_lat_long_degrees(55.9295, 13.5297);
        assert!(l.contains_point(inside));
        b.iter(|| black_box(l.contains_point(inside)))
    });

    c.bench_function("Loop::contains_point_false_5_vertices", |b| {
        let vertices: Vec<NVector> = vec![
            NVector::from_lat_long_degrees(55.605, 13.0038),
            NVector::from_lat_long_degrees(55.4295, 13.82),
            NVector::from_lat_long_degrees(56.0294, 14.1567),
            NVector::from_lat_long_degrees(56.0465, 12.6945),
            NVector::from_lat_long_degrees(55.7047, 13.191),
        ];
        let l = Loop::new(&vertices);
        let outside: NVector = NVector::from_lat_long_degrees(56.1589, 13.7668);
        assert!(!l.contains_point(outside));
        b.iter(|| black_box(l.contains_point(outside)))
    });

    c.bench_function(
        "Loop::contains_point_true_94_vertices",
        |b: &mut Bencher<'_>| {
            let l = Loop::new(&vertices_94());
            let inside: NVector = NVector::from_lat_long_degrees(-14.0, 130.0);
            assert!(l.contains_point(inside));
            b.iter(|| black_box(l.contains_point(inside)))
        },
    );

    c.bench_function(
        "Loop::contains_point_false_94_vertices",
        |b: &mut Bencher<'_>| {
            let l = Loop::new(&vertices_94());
            let outside: NVector = NVector::from_lat_long_degrees(90.0, 0.0);
            assert!(!l.contains_point(outside));
            b.iter(|| black_box(l.contains_point(outside)))
        },
    );

    c.bench_function("Loop::triangle_contains_point_true", |b| {
        let inside = NVector::from_lat_long_degrees(15.0, 30.0);
        let v1 = NVector::from_lat_long_degrees(20.0, 20.0);
        let v2 = NVector::from_lat_long_degrees(10.0, 30.0);
        let v3 = NVector::from_lat_long_degrees(40.0, 40.0);
        let l = Loop::new(&vec![v1, v2, v3]);
        assert!(l.contains_point(inside));
        b.iter(|| black_box(l.contains_point(inside)));
    });

    c.bench_function("Loop::triangle_contains_point_false", |b| {
        let outside = NVector::from_lat_long_degrees(15.0, 30.0).antipode();
        let v1 = NVector::from_lat_long_degrees(20.0, 20.0);
        let v2 = NVector::from_lat_long_degrees(10.0, 30.0);
        let v3 = NVector::from_lat_long_degrees(40.0, 40.0);
        let l = Loop::new(&vec![v1, v2, v3]);
        assert!(!l.contains_point(outside));
        b.iter(|| black_box(l.contains_point(outside)));
    });

    c.bench_function("Loop::bound_5_vertices", |b: &mut Bencher<'_>| {
        let vertices: Vec<NVector> = vec![
            NVector::from_lat_long_degrees(55.605, 13.0038),
            NVector::from_lat_long_degrees(55.4295, 13.82),
            NVector::from_lat_long_degrees(56.0294, 14.1567),
            NVector::from_lat_long_degrees(56.0465, 12.6945),
            NVector::from_lat_long_degrees(55.7047, 13.191),
        ];
        let l = Loop::new(&vertices);
        b.iter(|| black_box(l.bound()))
    });

    c.bench_function("Loop::bound_94_vertices", |b: &mut Bencher<'_>| {
        let l = Loop::new(&vertices_94());
        b.iter(|| black_box(l.bound()))
    });

    c.bench_function(
        "Loop::distance_to_boundary_94_vertices",
        |b: &mut Bencher<'_>| {
            let l = Loop::new(&vertices_94());
            let p = NVector::from_lat_long_degrees(0.0, 0.0);
            b.iter(|| black_box(l.distance_to_boundary(p)))
        },
    );
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);

fn vertices_94() -> Vec<NVector> {
    vec![
        NVector::from_lat_long_degrees(-12.0, 123.33333333333333),
        NVector::from_lat_long_degrees(-11.268055555555556, 124.30527777777777),
        NVector::from_lat_long_degrees(-9.881666666666668, 126.12333333333332),
        NVector::from_lat_long_degrees(-9.333333333333334, 126.83333333333333),
        NVector::from_lat_long_degrees(-9.0, 128.0438888888889),
        NVector::from_lat_long_degrees(-8.544722222222221, 129.665),
        NVector::from_lat_long_degrees(-8.323333333333332, 130.45444444444442),
        NVector::from_lat_long_degrees(-9.810833333333335, 131.50694444444446),
        NVector::from_lat_long_degrees(-10.024166666666668, 131.6588888888889),
        NVector::from_lat_long_degrees(-10.079133601520954, 131.82131954196385),
        NVector::from_lat_long_degrees(-10.14535830719111, 131.97946829266212),
        NVector::from_lat_long_degrees(-10.221957518584153, 132.13280527801837),
        NVector::from_lat_long_degrees(-10.308586964977717, 132.2806396890081),
        NVector::from_lat_long_degrees(-10.404857112769248, 132.4223040618227),
        NVector::from_lat_long_degrees(-10.510334842786152, 132.5571571408609),
        NVector::from_lat_long_degrees(-10.624545314907795, 132.68458664483936),
        NVector::from_lat_long_degrees(-10.746974012299455, 132.80401192710187),
        NVector::from_lat_long_degrees(-10.877068957028687, 132.91488652176986),
        NVector::from_lat_long_degrees(-11.014243088373325, 133.01670056786352),
        NVector::from_lat_long_degrees(-11.157876794728857, 133.10898310392378),
        NVector::from_lat_long_degrees(-11.307320589670722, 133.19130422595288),
        NVector::from_lat_long_degrees(-11.462222222222222, 133.2625),
        NVector::from_lat_long_degrees(-11.604673858414568, 133.3185591635582),
        NVector::from_lat_long_degrees(-11.750394533636548, 133.36526410341762),
        NVector::from_lat_long_degrees(-11.898635991670819, 133.4029396937973),
        NVector::from_lat_long_degrees(-12.04885360791122, 133.4314334696092),
        NVector::from_lat_long_degrees(-12.200555555555555, 133.45),
        NVector::from_lat_long_degrees(-13.520000000000001, 134.73638888888888),
        NVector::from_lat_long_degrees(-13.680778910014899, 134.80299274470732),
        NVector::from_lat_long_degrees(-13.845914694422346, 134.85744498590665),
        NVector::from_lat_long_degrees(-14.014317486656438, 134.90017734096713),
        NVector::from_lat_long_degrees(-14.18519074058639, 134.9309629668399),
        NVector::from_lat_long_degrees(-14.357724610581228, 134.9496298700652),
        NVector::from_lat_long_degrees(-14.531099668429466, 134.9560621534133),
        NVector::from_lat_long_degrees(-14.704444444444444, 134.94944444444445),
        NVector::from_lat_long_degrees(-14.874426058848659, 134.93758544824178),
        NVector::from_lat_long_degrees(-15.04246441510527, 134.90807078327876),
        NVector::from_lat_long_degrees(-15.208166682747265, 134.8668701374394),
        NVector::from_lat_long_degrees(-15.370767198172286, 134.81414638818163),
        NVector::from_lat_long_degrees(-15.5295128143404, 134.7501167289333),
        NVector::from_lat_long_degrees(-15.683666452786785, 134.6750521971293),
        NVector::from_lat_long_degrees(-15.834722222222222, 134.59305555555557),
        NVector::from_lat_long_degrees(-15.98340322262666, 134.49891263771386),
        NVector::from_lat_long_degrees(-16.122575809077084, 134.39009922046435),
        NVector::from_lat_long_degrees(-16.254091046535823, 134.27145943654352),
        NVector::from_lat_long_degrees(-16.377306116444228, 134.14355296295713),
        NVector::from_lat_long_degrees(-16.49161744202551, 134.0069882760566),
        NVector::from_lat_long_degrees(-16.596463890741084, 133.86241986892824),
        NVector::from_lat_long_degrees(-16.691329782101587, 133.71054514359372),
        NVector::from_lat_long_degrees(-16.775747678535897, 133.5521009895791),
        NVector::from_lat_long_degrees(-16.849300937814554, 133.38786006483957),
        NVector::from_lat_long_degrees(-16.911626006662477, 133.21862679950334),
        NVector::from_lat_long_degrees(-16.96241443668242, 133.04523314729707),
        NVector::from_lat_long_degrees(-17.001414605539, 132.86853411370186),
        NVector::from_lat_long_degrees(-17.028433128503863, 132.689403093738),
        NVector::from_lat_long_degrees(-17.04333594790603, 132.50872705565843),
        NVector::from_lat_long_degrees(-17.046049090726694, 132.32740160963652),
        NVector::from_lat_long_degrees(-17.036559087475307, 132.14632600266734),
        NVector::from_lat_long_degrees(-17.014913048525397, 131.96639808228466),
        NVector::from_lat_long_degrees(-16.98121839721165, 131.788509272287),
        NVector::from_lat_long_degrees(-16.93888888888889, 131.61249999999998),
        NVector::from_lat_long_degrees(-16.873320596167037, 131.44339312658815),
        NVector::from_lat_long_degrees(-16.80441477790408, 131.27635009218574),
        NVector::from_lat_long_degrees(-16.72438796995017, 131.11480322046452),
        NVector::from_lat_long_degrees(-16.633639477781884, 130.9595500149794),
        NVector::from_lat_long_degrees(-16.532621391377564, 130.81135326453372),
        NVector::from_lat_long_degrees(-16.42183608285713, 130.6709371112483),
        NVector::from_lat_long_degrees(-16.30183344963955, 130.53898341627485),
        NVector::from_lat_long_degrees(-16.173207924885546, 130.4161284416528),
        NVector::from_lat_long_degrees(-16.036595277968726, 130.30295986217808),
        NVector::from_lat_long_degrees(-15.892669228341264, 130.20001411664074),
        NVector::from_lat_long_degrees(-15.742137896451371, 130.10777410350863),
        NVector::from_lat_long_degrees(-15.585740115357952, 130.02666722218365),
        NVector::from_lat_long_degrees(-15.424241626408252, 129.95706375741338),
        NVector::from_lat_long_degrees(-15.255555555555556, 129.90916666666666),
        NVector::from_lat_long_degrees(-15.095555555555556, 129.90583333333333),
        NVector::from_lat_long_degrees(-15.025025129462573, 129.72507283020977),
        NVector::from_lat_long_degrees(-14.942022867856721, 129.55004285938432),
        NVector::from_lat_long_degrees(-14.847774942689984, 129.38121460605188),
        NVector::from_lat_long_degrees(-14.742712345419775, 129.21934987668143),
        NVector::from_lat_long_degrees(-14.62731471155048, 129.0651753510518),
        NVector::from_lat_long_degrees(-14.50210788979704, 128.91937921525675),
        NVector::from_lat_long_degrees(-14.367661302205123, 128.78260806625974),
        NVector::from_lat_long_degrees(-14.224585114399213, 128.65546410026548),
        NVector::from_lat_long_degrees(-14.073527235662322, 128.538502593329),
        NVector::from_lat_long_degrees(-13.91517016880058, 128.4322296789631),
        NVector::from_lat_long_degrees(-13.750227729735846, 128.337100424115),
        NVector::from_lat_long_degrees(-13.579441656529466, 128.25351720183136),
        NVector::from_lat_long_degrees(-13.403578127103904, 128.18182835626672),
        NVector::from_lat_long_degrees(-13.223424204333286, 128.12232715344896),
        NVector::from_lat_long_degrees(-13.039784226456785, 128.07525100941228),
        NVector::from_lat_long_degrees(-12.853476159968212, 128.04078098594016),
        NVector::from_lat_long_degrees(-12.665277777777778, 128.01972222222224),
        NVector::from_lat_long_degrees(-12.434166666666666, 126.3236111111111),
    ]
}
