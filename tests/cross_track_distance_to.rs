use jord::{GreatCircle, HorizontalPos, Length, Micrometre};

#[test]
fn negative_cross_track_distance_if_left() {
    let p = HorizontalPos::from_s84(53.2611, -0.7972);
    let gcp1 = HorizontalPos::from_s84(53.3206, -1.7297);
    let gcp2 = HorizontalPos::from_s84(53.1887, 0.1334);
    let gc1 = GreatCircle::from_positions(gcp1, gcp2).unwrap();
    let expected = Length::from_metres(-307.549994);
    assert_eq!(expected, p.cross_track_distance_to(gc1).round(Micrometre));

    // same result with great circle from position and bearing
    let gc2 = GreatCircle::new(gcp1, gcp1.initial_bearing_to(gcp2).unwrap());
    assert_eq!(expected, p.cross_track_distance_to(gc2).round(Micrometre));
}

#[test]
fn positive_cross_track_distance_if_right() {
    let p = HorizontalPos::from_s84(53.2611, -0.7972).antipode();
    let gcp1 = HorizontalPos::from_s84(53.3206, -1.7297);
    let gcp2 = HorizontalPos::from_s84(53.1887, 0.1334);
    let gc1 = GreatCircle::from_positions(gcp1, gcp2).unwrap();
    let expected = Length::from_metres(307.549994);
    assert_eq!(expected, p.cross_track_distance_to(gc1).round(Micrometre));

    // same result with great circle from position and bearing
    let gc2 = GreatCircle::new(gcp1, gcp1.initial_bearing_to(gcp2).unwrap());
    assert_eq!(expected, p.cross_track_distance_to(gc2).round(Micrometre));
}

#[test]
fn zero_cross_track_distance() {
    let gc1 = HorizontalPos::from_s84(0.0, -10.0);
    let gc2 = HorizontalPos::from_s84(0.0, 10.0);
    let gc = GreatCircle::from_positions(gc1, gc2).unwrap();
    for f in 0..100 {
        let p = gc1.intermediate_pos_to(gc2, (f as f64) / 100.0).unwrap();
        assert_eq!(Length::zero(), p.cross_track_distance_to(gc));
    }
}
