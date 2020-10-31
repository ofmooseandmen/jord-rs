use jord::models::S84;
use jord::{Error, HorizontalPos, MinorArc};

#[test]
fn no_minor_arc_between_same_position() {
    let p = HorizontalPos::from_s84(54.0, 154.0);
    assert_eq!(
        Err(Error::CoincidentalPositions),
        MinorArc::from_positions(p, p)
    );
}

#[test]
fn no_minor_arc_between_poles() {
    assert_eq!(
        Err(Error::AntipodalPositions),
        MinorArc::from_positions(
            HorizontalPos::north_pole(S84),
            HorizontalPos::south_pole(S84)
        )
    );
}

#[test]
fn no_minor_arc_between_antipodes() {
    let p = HorizontalPos::from_s84(54.0, 154.0);
    assert_eq!(
        Err(Error::AntipodalPositions),
        MinorArc::from_positions(p, p.antipode())
    );
}
