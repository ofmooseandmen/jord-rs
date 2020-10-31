use jord::{Angle, Error, HorizontalPos, Microarcsecond};

#[test]
fn final_bearing_to_same_position() {
    let p = HorizontalPos::from_s84(50.066389, -5.714722);
    assert_eq!(Err(Error::CoincidentalPositions), p.final_bearing_to(p));
    assert_eq!(
        Err(Error::CoincidentalPositions),
        p.final_bearing_to(HorizontalPos::from_s84(50.066389, -5.714722))
    );
}

#[test]
fn final_bearing_to_iso_longitude_going_north() {
    let p1 = HorizontalPos::from_s84(50.066389, -5.714722);
    let p2 = HorizontalPos::from_s84(58.643889, -5.714722);
    assert_eq!(Ok(Angle::zero()), p1.final_bearing_to(p2));
}

#[test]
fn final_bearing_to_iso_longitude_going_south() {
    let p1 = HorizontalPos::from_s84(58.643889, -5.714722);
    let p2 = HorizontalPos::from_s84(50.066389, -5.714722);
    assert_eq!(
        Ok(Angle::from_decimal_degrees(180.0)),
        p1.final_bearing_to(p2)
    );
}

#[test]
fn at_equator_final_bearing_to_east() {
    let p1 = HorizontalPos::from_s84(0.0, 0.0);
    let p2 = HorizontalPos::from_s84(0.0, 1.0);
    assert_eq!(
        Ok(Angle::from_decimal_degrees(90.0)),
        p1.final_bearing_to(p2)
    );
}

#[test]
fn at_equator_final_bearing_to_west() {
    let p1 = HorizontalPos::from_s84(0.0, 1.0);
    let p2 = HorizontalPos::from_s84(0.0, 0.0);
    assert_eq!(
        Ok(Angle::from_decimal_degrees(270.0)),
        p1.final_bearing_to(p2)
    );
}

#[test]
fn final_bearing_to() {
    let p1 = HorizontalPos::from_s84(50.066389, -5.714722);
    let p2 = HorizontalPos::from_s84(58.643889, -3.07);
    assert_eq!(
        Angle::from_decimal_degrees(11.27520031611111),
        p1.final_bearing_to(p2).unwrap().round(Microarcsecond)
    );
    assert_eq!(
        Angle::from_decimal_degrees(189.1198173275),
        p2.final_bearing_to(p1).unwrap().round(Microarcsecond)
    );
    let p3 = HorizontalPos::from_s84(-53.994722, -25.9875);
    let p4 = HorizontalPos::from_s84(54.0, 154.0);
    assert_eq!(
        Angle::from_decimal_degrees(125.68508662305555),
        p3.final_bearing_to(p4).unwrap().round(Microarcsecond)
    );
}
