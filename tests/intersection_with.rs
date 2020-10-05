mod lat_long_pos {

    use jord::{Error, LatLongPos, MinorArc};

    #[test]
    fn intersection_with_same_minor_arc() {
        let ma = MinorArc::from_lat_longs(
            LatLongPos::from_s84(51.885, 0.235),
            LatLongPos::from_s84(52.885, 1.235),
        )
        .unwrap();
        assert_eq!(Err(Error::CoincidentalPath), ma.intersection_with(ma));
    }

    #[test]
    fn intersection_with_opposite_minor_arc() {
        let p1 = LatLongPos::from_s84(51.885, 0.235);
        let p2 = LatLongPos::from_s84(52.885, 1.235);
        let ma1 = MinorArc::from_lat_longs(p1, p2).unwrap();
        let ma2 = MinorArc::from_lat_longs(p2, p1).unwrap();
        assert_eq!(Err(Error::CoincidentalPath), ma1.intersection_with(ma2));
    }

    #[test]
    fn intersection_outside_either_minor_arcs() {
        let ma1 = MinorArc::from_lat_longs(
            LatLongPos::from_s84(0.0, 0.0),
            LatLongPos::from_s84(0.0, 10.0),
        )
        .unwrap();
        let ma2 = MinorArc::from_lat_longs(
            LatLongPos::from_s84(-5.0, 5.0),
            LatLongPos::from_s84(-1.0, 5.0),
        )
        .unwrap();
        assert_eq!(Err(Error::NoIntersection), ma1.intersection_with(ma2));
    }

    #[test]
    fn intersection_outside_both_minor_arcs() {
        let ma1 = MinorArc::from_lat_longs(
            LatLongPos::from_s84(0.0, -10.0),
            LatLongPos::from_s84(0.0, -1.0),
        )
        .unwrap();
        let ma2 = MinorArc::from_lat_longs(
            LatLongPos::from_s84(-5.0, 5.0),
            LatLongPos::from_s84(-1.0, 5.0),
        )
        .unwrap();
        assert_eq!(Err(Error::NoIntersection), ma1.intersection_with(ma2));
    }

    #[test]
    fn intersection_with_minor_arc_across_equator() {
        let ma1 = MinorArc::from_lat_longs(
            LatLongPos::from_s84(54.0, 154.0),
            LatLongPos::from_s84(-54.0, 154.0),
        )
        .unwrap();
        let ma2 = MinorArc::from_lat_longs(
            LatLongPos::from_s84(53.0, 153.0),
            LatLongPos::from_s84(53.0, 155.0),
        )
        .unwrap();
        assert_eq!(
            Ok(LatLongPos::from_s84(53.00419442027778, 154.0)),
            ma1.intersection_with(ma2)
        );
    }

    #[test]
    fn intersection_with_start_of_minor_arc() {
        let p = LatLongPos::from_s84(-41.52, 141.0);
        let ma1 =
            MinorArc::from_lat_longs(p, LatLongPos::from_s84(-65.444811, 111.616598)).unwrap();
        let ma2 = MinorArc::from_lat_longs(p, LatLongPos::from_s84(-39.883333, 141.0)).unwrap();
        assert_eq!(Ok(p), ma1.intersection_with(ma2));
    }

    #[test]
    fn intersection_with_end_of_minor_arc() {
        let p = LatLongPos::from_s84(-41.52, 141.0);
        let ma1 =
            MinorArc::from_lat_longs(LatLongPos::from_s84(-65.444811, 111.616598), p).unwrap();
        let ma2 = MinorArc::from_lat_longs(LatLongPos::from_s84(-39.883333, 141.0), p).unwrap();
        assert_eq!(Ok(p), ma1.intersection_with(ma2));
    }

    #[test]
    fn intersection_with_exactly_on_minor_arcs() {
        let ma1 = MinorArc::from_lat_longs(
            LatLongPos::from_s84(0.0, -10.0),
            LatLongPos::from_s84(0.0, 10.0),
        )
        .unwrap();
        let ma2 = MinorArc::from_lat_longs(
            LatLongPos::from_s84(-10.0, 0.0),
            LatLongPos::from_s84(10.0, 0.0),
        )
        .unwrap();
        assert_eq!(
            Ok(LatLongPos::from_s84(0.0, 0.0)),
            ma1.intersection_with(ma2)
        );
    }

    #[test]
    fn intersection_with() {
        let ma1 = MinorArc::from_lat_longs(
            LatLongPos::from_s84(51.885, 0.235),
            LatLongPos::from_s84(48.269, 13.093),
        )
        .unwrap();
        let ma2 = MinorArc::from_lat_longs(
            LatLongPos::from_s84(49.008, 2.549),
            LatLongPos::from_s84(56.283, 11.304),
        )
        .unwrap();
        assert_eq!(
            Ok(LatLongPos::from_s84(50.901738961111114, 4.49418117)),
            ma1.intersection_with(ma2)
        );
    }
}
