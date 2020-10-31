use jord::{Angle, Error, HorizontalPos, Microarcsecond};

#[test]
fn positive_turn() {
    assert_eq!(
        Angle::from_decimal_degrees(18.192705871944444),
        HorizontalPos::from_s84(45.0, 0.0)
            .turn(
                HorizontalPos::from_s84(0.0, 0.0),
                HorizontalPos::from_s84(60.0, -10.0)
            )
            .unwrap()
            .round(Microarcsecond)
    );
}

#[test]
fn negative_turn() {
    assert_eq!(
        Angle::from_decimal_degrees(-18.192705871944444),
        HorizontalPos::from_s84(45.0, 0.0)
            .turn(
                HorizontalPos::from_s84(0.0, 0.0),
                HorizontalPos::from_s84(60.0, 10.0)
            )
            .unwrap()
            .round(Microarcsecond)
    );
}

#[test]
fn zero_turn() {
    assert_eq!(
        Ok(Angle::zero()),
        HorizontalPos::from_s84(45.0, 0.0).turn(
            HorizontalPos::from_s84(0.0, 0.0),
            HorizontalPos::from_s84(90.0, 0.0)
        )
    );
}

#[test]
fn half_turn() {
    let a = HorizontalPos::from_s84(45.0, 63.0);
    let b = HorizontalPos::from_s84(-54.0, -89.0);
    assert_eq!(Ok(Angle::from_decimal_degrees(180.0)), a.turn(b, b));
    assert_eq!(Ok(Angle::from_decimal_degrees(180.0)), b.turn(a, a));
}

#[test]
fn no_turn() {
    let a = HorizontalPos::from_s84(45.0, 63.0);
    let b = HorizontalPos::from_s84(-54.0, -89.0);
    assert_eq!(Err(Error::CoincidentalPositions), a.turn(a, a));
    assert_eq!(Err(Error::CoincidentalPositions), a.turn(a, b));
    assert_eq!(Err(Error::CoincidentalPositions), a.turn(b, a));
    assert_eq!(Err(Error::CoincidentalPositions), b.turn(a, b));
    assert_eq!(Err(Error::CoincidentalPositions), b.turn(b, a));
    assert_eq!(Err(Error::CoincidentalPositions), b.turn(b, b));
}
