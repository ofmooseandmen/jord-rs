mod places;
use jord::models::S84;
use jord::{GreatCircle, HorizontalPos, Side};
pub use places::*;

#[test]
fn side_of_none() {
    let p = HorizontalPos::from_s84(0.0, 0.0);
    let gc = GreatCircle::from_positions(
        HorizontalPos::from_s84(45.0, 0.0),
        HorizontalPos::north_pole(S84),
    )
    .unwrap();
    assert_eq!(Side::None, p.side_of(gc));
}

#[test]
fn side_of_left() {
    assert_eq!(
        Side::LeftOf,
        ystad().side_of(GreatCircle::from_positions(kristianstad(), helsingborg()).unwrap())
    );
    assert_eq!(
        Side::LeftOf,
        malmo().side_of(GreatCircle::from_positions(lund(), helsingborg()).unwrap())
    );
}

#[test]
fn side_of_right() {
    assert_eq!(
        Side::RightOf,
        ystad().side_of(GreatCircle::from_positions(helsingborg(), kristianstad()).unwrap())
    );
    assert_eq!(
        Side::RightOf,
        malmo().side_of(GreatCircle::from_positions(helsingborg(), lund()).unwrap())
    );
}
