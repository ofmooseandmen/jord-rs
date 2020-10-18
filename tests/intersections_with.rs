use jord::{Angle, Error, GreatCircle, HorizontalPos, Microarcsecond};

#[test]
fn intersections_with_same_great_circle() {
    let gc = GreatCircle::new(
        HorizontalPos::from_s84(51.885, 0.235),
        Angle::from_decimal_degrees(108.63),
    );
    assert_eq!(Err(Error::CoincidentalPath), gc.intersections_with(gc));
}

#[test]
fn intersections_with_opposite_great_circle() {
    let p1 = HorizontalPos::from_s84(51.885, 0.235);
    let p2 = HorizontalPos::from_s84(52.885, 1.235);
    let gc1 = GreatCircle::from_positions(p1, p2).unwrap();
    let gc2 = GreatCircle::from_positions(p2, p1).unwrap();
    assert_eq!(Err(Error::CoincidentalPath), gc1.intersections_with(gc2));
}

#[test]
fn intersections_with() {
    let gc1 = GreatCircle::new(
        HorizontalPos::from_s84(0.0, -54.0),
        Angle::from_decimal_degrees(90.0),
    );
    let gc2 = GreatCircle::new(
        HorizontalPos::from_s84(-54.0, 0.0),
        Angle::from_decimal_degrees(0.0),
    );
    let (i1, i2) = gc1.intersections_with(gc2).unwrap();
    assert_eq!(i1.round(Microarcsecond), HorizontalPos::from_s84(0.0, 0.0));
    assert_eq!(i1.antipode(), i2);
}
