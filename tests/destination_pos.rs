use jord::{Angle, HorizontalPos, Length, Microarcsecond};

#[test]
fn zero_distance_destination_pos() {
    let p0 = HorizontalPos::from_s84(53.320556, -1.729722);
    let actual = p0.destination_pos(Angle::from_decimal_degrees(96.0217), Length::zero());
    assert_eq!(p0, actual);
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
