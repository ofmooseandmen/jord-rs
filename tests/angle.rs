use jord::Angle;

#[test]
fn one_microarcsecond() {
    assert_eq!(
        Angle::from_decimal_degrees(60.0),
        Angle::from_decimal_degrees(59.9999999999)
    );
    assert_ne!(
        Angle::from_decimal_degrees(60.0),
        Angle::from_decimal_degrees(59.999999998)
    );
}

#[test]
fn one_arcmillisecond() {
    let a = Angle::from_decimal_degrees(1.0 / 3600000.0);
    assert_eq!(0, a.whole_degrees());
    assert_eq!(0, a.arcminutes());
    assert_eq!(0, a.arcseconds());
    assert_eq!(1, a.arcmilliseconds());
}

#[test]
fn one_arcsecond() {
    let a = Angle::from_decimal_degrees(1000.0 / 3600000.0);
    assert_eq!(0, a.whole_degrees());
    assert_eq!(0, a.arcminutes());
    assert_eq!(1, a.arcseconds());
    assert_eq!(0, a.arcmilliseconds());
}

#[test]
fn one_arcminute() {
    let a = Angle::from_decimal_degrees(60000.0 / 3600000.0);
    assert_eq!(0, a.whole_degrees());
    assert_eq!(1, a.arcminutes());
    assert_eq!(0, a.arcseconds());
    assert_eq!(0, a.arcmilliseconds());
}

#[test]
fn one_degrees() {
    let a = Angle::from_decimal_degrees(1.0);
    assert_eq!(1, a.whole_degrees());
    assert_eq!(0, a.arcminutes());
    assert_eq!(0, a.arcseconds());
    assert_eq!(0, a.arcmilliseconds());
}

#[test]
fn positve_value() {
    let a = Angle::from_decimal_degrees(154.9150300);
    assert_eq!(154, a.whole_degrees());
    assert_eq!(54, a.arcminutes());
    assert_eq!(54, a.arcseconds());
    assert_eq!(108, a.arcmilliseconds());
}

#[test]
fn negative_value() {
    let a = Angle::from_decimal_degrees(-154.915);
    assert_eq!(-154, a.whole_degrees());
    assert_eq!(54, a.arcminutes());
    assert_eq!(54, a.arcseconds());
    assert_eq!(0, a.arcmilliseconds());
}
