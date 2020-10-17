use jord::{HorizontalPos, Length, Micrometre, MinorArc};

#[test]
fn negative_along_track_distance_if_behind() {
    let p = HorizontalPos::from_s84(53.3206, -1.7297);
    let ma = MinorArc::new(
        HorizontalPos::from_s84(53.2611, -0.7972),
        HorizontalPos::from_s84(53.1887, 0.1334),
    )
    .unwrap();
    assert_eq!(
        Length::from_kilometres(-62.329309973).round(Micrometre),
        p.along_track_distance_to(ma).round(Micrometre)
    )
}

#[test]
fn positive_along_track_distance_if_ahead() {
    let p = HorizontalPos::from_s84(53.2611, -0.7972);
    let ma = MinorArc::new(
        HorizontalPos::from_s84(53.3206, -1.7297),
        HorizontalPos::from_s84(53.1887, 0.1334),
    )
    .unwrap();
    assert_eq!(
        Length::from_kilometres(62.331579102).round(Micrometre),
        p.along_track_distance_to(ma).round(Micrometre)
    )
}

#[test]
fn zero_along_track_distance_if_start() {
    let p = HorizontalPos::from_s84(53.2611, -0.7972);
    let ma = MinorArc::new(p, HorizontalPos::from_s84(53.1887, 0.1334)).unwrap();
    assert_eq!(
        Length::zero(),
        p.along_track_distance_to(ma).round(Micrometre)
    )
}

#[test]
fn along_track_distance_at_end() {
    let start = HorizontalPos::from_s84(53.2611, -0.7972);
    let end = HorizontalPos::from_s84(53.1887, 0.1334);
    let ma = MinorArc::new(start, end).unwrap();
    assert_eq!(
        start.distance_to(end).round(Micrometre),
        end.along_track_distance_to(ma).round(Micrometre)
    )
}
