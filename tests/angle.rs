use jord::Angle;

#[test]
fn one_arcmillisecond() {
    let a = Angle::from_decimal_degrees(1.0 / 3_600_000.0);
    assert_eq!(0, a.arcdegrees());
    assert_eq!(0, a.arcminutes());
    assert_eq!(0, a.arcseconds());
    assert_eq!(1, a.milliarcseconds());
}

#[test]
fn one_arcsecond() {
    let a = Angle::from_decimal_degrees(1.0 / 3_600.0);
    assert_eq!(0, a.arcdegrees());
    assert_eq!(0, a.arcminutes());
    assert_eq!(1, a.arcseconds());
    assert_eq!(0, a.milliarcseconds());
}

#[test]
fn one_arcminute() {
    let a = Angle::from_decimal_degrees(1.0 / 60.0);
    assert_eq!(0, a.arcdegrees());
    assert_eq!(1, a.arcminutes());
    assert_eq!(0, a.arcseconds());
    assert_eq!(0, a.milliarcseconds());
}

#[test]
fn one_degrees() {
    let a = Angle::from_decimal_degrees(1.0);
    assert_eq!(1, a.arcdegrees());
    assert_eq!(0, a.arcminutes());
    assert_eq!(0, a.arcseconds());
    assert_eq!(0, a.milliarcseconds());
}

#[test]
fn positive_angle() {
    let a = Angle::from_decimal_degrees(154.9150300);
    assert_eq!(154, a.arcdegrees());
    assert_eq!(54, a.arcminutes());
    assert_eq!(54, a.arcseconds());
    assert_eq!(108, a.milliarcseconds());
}

#[test]
fn negative_angle() {
    let a = Angle::from_decimal_degrees(-154.915);
    assert_eq!(-154, a.arcdegrees());
    assert_eq!(54, a.arcminutes());
    assert_eq!(54, a.arcseconds());
    assert_eq!(0, a.milliarcseconds());
}
