mod lat_long_pos {

    use jord::{Angle, Error, GreatCircle, LatLongPos, SurfacePos};

    #[test]
    fn intersections_with_same_great_circle() {
        let gc = GreatCircle::from_lat_long_bearing(
            LatLongPos::from_s84(51.885, 0.235),
            Angle::from_decimal_degrees(108.63),
        );
        assert_eq!(Err(Error::CoincidentalPath), gc.intersections_with(gc));
    }

    #[test]
    fn intersections_with_opposite_great_circle() {
        let p1 = LatLongPos::from_s84(51.885, 0.235);
        let p2 = LatLongPos::from_s84(52.885, 1.235);
        let gc1 = GreatCircle::from_lat_longs(p1, p2).unwrap();
        let gc2 = GreatCircle::from_lat_longs(p2, p1).unwrap();
        assert_eq!(Err(Error::CoincidentalPath), gc1.intersections_with(gc2));
    }

    #[test]
    fn intersections_with() {
        let gc1 = GreatCircle::from_lat_long_bearing(
            LatLongPos::from_s84(51.885, 0.235),
            Angle::from_decimal_degrees(108.63),
        );
        let gc2 = GreatCircle::from_lat_long_bearing(
            LatLongPos::from_s84(49.008, 2.549),
            Angle::from_decimal_degrees(32.72),
        );
        let (i1, i2) = gc1.intersections_with(gc2).unwrap();
        assert_eq!(
            i1,
            LatLongPos::from_s84(50.90172260888889, 4.494278278888889)
        );
        assert_eq!(i1.antipode(), i2);
    }
}
