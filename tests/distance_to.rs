use jord::models::S84;
use jord::surfaces::WGS84_SPHERE;
use jord::{HorizontalPos, Length, Micrometre, Surface};
use std::f64::consts::PI;

#[test]
fn distance_to_same_position() {
    let p = HorizontalPos::from_s84(50.066389, -5.714722);
    assert_eq!(Length::zero(), p.distance_to(p));
}

#[test]
fn distance_to() {
    let p1 = HorizontalPos::from_s84(50.066389, -5.714722);
    let p2 = HorizontalPos::from_s84(58.643889, -3.07);
    assert_eq!(
        Length::from_metres(968854.878007),
        p1.distance_to(p2).round(Micrometre)
    );
}

#[test]
fn north_pole_distance_to_south_pole() {
    assert_eq!(
        PI * WGS84_SPHERE.mean_radius(),
        HorizontalPos::north_pole(S84).distance_to(HorizontalPos::south_pole(S84))
    );
}

#[test]
fn distance_to_across_date_line() {
    let p1 = HorizontalPos::from_s84(50.066389, -179.999722);
    let p2 = HorizontalPos::from_s84(50.066389, 179.999722);
    assert_eq!(
        Length::from_metres(39.685096),
        p1.distance_to(p2).round(Micrometre)
    );
}
