use jord::models::S84;
use jord::surfaces::WGS84_SPHERE;
use jord::{Angle, HorizontalPos, Length, Microarcsecond, Surface};
use std::f64::consts::PI;

#[test]
fn zero_distance_destination_pos() {
    let p0 = HorizontalPos::from_s84(53.320556, -1.729722);
    let actual = p0.destination_pos(Angle::from_decimal_degrees(96.0217), Length::zero());
    assert_eq!(p0, actual);
}

#[test]
fn north_pole_destination_pos() {
    let p0 = HorizontalPos::north_pole(S84);
    let expected = HorizontalPos::from_s84(0.0, 0.0);
    let actual = p0
        .destination_pos(
            Angle::from_decimal_degrees(180.0),
            (PI / 2.0) * WGS84_SPHERE.mean_radius(),
        )
        .round(Microarcsecond);
    assert_eq!(expected, actual);
}

#[test]
fn south_pole_destination_pos() {
    let p0 = HorizontalPos::south_pole(S84);
    let expected = HorizontalPos::from_s84(0.0, 0.0);
    let actual = p0
        .destination_pos(
            Angle::from_decimal_degrees(0.0),
            (PI / 2.0) * WGS84_SPHERE.mean_radius(),
        )
        .round(Microarcsecond);
    assert_eq!(expected, actual);
}

#[test]
fn destination_pos() {
    let p0 = HorizontalPos::from_s84(53.320556, -1.729722);
    let expected = HorizontalPos::from_s84(53.18826954833333, 0.13327449083333334);
    let actual = p0
        .destination_pos(
            Angle::from_decimal_degrees(96.0217),
            Length::from_metres(124800.0),
        )
        .round(Microarcsecond);
    assert_eq!(expected, actual);
}
