/*
use jord::{Error, GreatCircle, HorizontalPos, MinorArc, Microarcsecond};

#[test]
fn no_projection_outside() {
    let ma = MinorArc::from_positions(
        HorizontalPos::from_s84(54.0, 15.0),
        HorizontalPos::from_s84(54.0, 20.0),
    )
    .unwrap();
    let pos = HorizontalPos::from_s84(54.0, 10.0);
    assert_eq!(Err(Error::OutOfRange), pos.projection_onto(ma));
}

#[test]
fn projection_onto() {
    let start = HorizontalPos::from_s84(53.3206, -1.7297);
    let end = HorizontalPos::from_s84(53.1887, 0.1334);
    let ma = MinorArc::from_positions(start, end).unwrap();
    let pos = HorizontalPos::from_s84(53.2611, -0.7972);
    let actual = pos.projection_onto(ma).unwrap();
    assert_eq!(
        HorizontalPos::from_s84(
            53.25835330666666,
            -0.7977433863888889
        ),
        actual.round(Microarcsecond)
    );
    // absolute cross track distance from p to great circle should be distance between projection and p
    let stx = pos.cross_track_distance_to(GreatCircle::from_positions(start, end).unwrap());
    let dst = pos.distance_to(actual);
    assert_eq!(stx.abs(), dst);
}

#[test]
fn projection_onto_start_1() {
    let start = HorizontalPos::from_s84(54.0, 15.0);
    let ma = MinorArc::from_positions(start, HorizontalPos::from_s84(54.0, 20.0)).unwrap();
    assert_eq!(Ok(start), start.projection_onto(ma));
}

#[test]
fn projection_onto_start_2() {
    let start = HorizontalPos::from_s84(13.733333587646484, 100.5);
    let ma =
        MinorArc::from_positions(start, HorizontalPos::from_s84(12.0, 100.58499908447266)).unwrap();
    assert_eq!(Ok(start), start.projection_onto(ma));
}

#[test]
fn projection_onto_end_1() {
    let end = HorizontalPos::from_s84(54.0, 20.0);
    let ma = MinorArc::from_positions(HorizontalPos::from_s84(54.0, 15.0), end).unwrap();
    assert_eq!(Ok(end), end.projection_onto(ma));
}

#[test]
fn projection_onto_end_2() {
    let end = HorizontalPos::from_s84(12.0, 100.58499908447266);
    let ma =
        MinorArc::from_positions(HorizontalPos::from_s84(13.733333587646484, 100.5), end).unwrap();
    assert_eq!(Ok(end), end.projection_onto(ma));
}
*/
