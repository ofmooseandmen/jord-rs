mod lat_long_pos {

    use jord::{LatLongPos, Length, MinorArc};

    #[test]
    fn negative_along_track_distance_if_behind() {
        let p = LatLongPos::from_s84(53.3206, -1.7297);
        let ma = MinorArc::from_lat_longs(
            LatLongPos::from_s84(53.2611, -0.7972),
            LatLongPos::from_s84(53.1887, 0.1334),
        )
        .unwrap();
        assert_eq!(
            Length::from_kilometres(-62.329309973),
            p.along_track_distance_to(ma)
        )
    }

    #[test]
    fn positive_along_track_distance_if_ahead() {
        let p = LatLongPos::from_s84(53.2611, -0.7972);
        let ma = MinorArc::from_lat_longs(
            LatLongPos::from_s84(53.3206, -1.7297),
            LatLongPos::from_s84(53.1887, 0.1334),
        )
        .unwrap();
        assert_eq!(
            Length::from_kilometres(62.331579102),
            p.along_track_distance_to(ma)
        )
    }

    #[test]
    fn zero_along_track_distance_if_start() {
        let p = LatLongPos::from_s84(53.2611, -0.7972);
        let ma = MinorArc::from_lat_longs(p, LatLongPos::from_s84(53.1887, 0.1334)).unwrap();
        assert_eq!(Length::zero(), p.along_track_distance_to(ma))
    }

    #[test]
    fn along_track_distance_at_end() {
        let start = LatLongPos::from_s84(53.2611, -0.7972);
        let end = LatLongPos::from_s84(53.1887, 0.1334);
        let ma = MinorArc::from_lat_longs(start, end).unwrap();
        assert_eq!(start.distance_to(end), end.along_track_distance_to(ma))
    }
}
