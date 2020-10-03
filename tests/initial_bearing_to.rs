mod lat_long_pos {

    use jord::models::S84;
    use jord::{Angle, Error, LatLongPos};

    #[test]
    fn returns_err_equal_positions() {
        let p = LatLongPos::from_s84(50.066389, -179.999722);
        assert_eq!(Err(Error::CoincidentalPositions), p.initial_bearing_to(p));
        assert_eq!(
            Err(Error::CoincidentalPositions),
            p.initial_bearing_to(LatLongPos::from_s84(50.066389, -179.999722))
        );
    }

    #[test]
    fn returns_0_iso_longitude_going_north() {
        let p1 = LatLongPos::from_s84(50.066389, -5.714722);
        let p2 = LatLongPos::from_s84(58.643889, -5.714722);
        assert_eq!(Ok(Angle::zero()), p1.initial_bearing_to(p2));
    }

    #[test]
    fn returns_180_iso_longitude_going_south() {
        let p1 = LatLongPos::from_s84(58.643889, -5.714722);
        let p2 = LatLongPos::from_s84(50.066389, -5.714722);
        assert_eq!(
            Ok(Angle::from_decimal_degrees(180.0)),
            p1.initial_bearing_to(p2)
        );
    }

    #[test]
    fn returns_90_at_equator_going_east() {
        let p1 = LatLongPos::from_s84(0.0, 0.0);
        let p2 = LatLongPos::from_s84(0.0, 1.0);
        assert_eq!(
            Ok(Angle::from_decimal_degrees(90.0)),
            p1.initial_bearing_to(p2)
        );
    }

    #[test]
    fn returns_270_at_equator_going_west() {
        let p1 = LatLongPos::from_s84(0.0, 1.0);
        let p2 = LatLongPos::from_s84(0.0, 0.0);
        assert_eq!(
            Ok(Angle::from_decimal_degrees(270.0)),
            p1.initial_bearing_to(p2)
        );
    }

    #[test]
    fn returns_0_at_prime_meridian_going_north() {
        let p1 = LatLongPos::from_s84(50.0, 0.0);
        let p2 = LatLongPos::from_s84(58.0, 0.0);
        assert_eq!(Ok(Angle::zero()), p1.initial_bearing_to(p2));
    }

    #[test]
    fn returns_180_at_prime_meridian_going_south() {
        let p1 = LatLongPos::from_s84(58.0, 0.0);
        let p2 = LatLongPos::from_s84(50.0, 0.0);
        assert_eq!(
            Ok(Angle::from_decimal_degrees(180.0)),
            p1.initial_bearing_to(p2)
        );
    }

    #[test]
    fn returns_0_at_date_line_going_north() {
        let p1 = LatLongPos::from_s84(50.0, 180.0);
        let p2 = LatLongPos::from_s84(58.0, 180.0);
        assert_eq!(Ok(Angle::zero()), p1.initial_bearing_to(p2));
    }

    #[test]
    fn returns_180_at_date_line_going_south() {
        let p1 = LatLongPos::from_s84(58.0, 180.0);
        let p2 = LatLongPos::from_s84(50.0, 180.0);
        assert_eq!(
            Ok(Angle::from_decimal_degrees(180.0)),
            p1.initial_bearing_to(p2)
        );
    }

    #[test]
    fn returns_0_south_to_north_pole() {
        let p1 = LatLongPos::south_pole(S84);
        let p2 = LatLongPos::north_pole(S84);
        assert_eq!(Ok(Angle::zero()), p1.initial_bearing_to(p2));
    }

    #[test]
    fn returns_0_north_to_south_pole() {
        let p1 = LatLongPos::north_pole(S84);
        let p2 = LatLongPos::south_pole(S84);
        assert_eq!(Ok(Angle::zero()), p1.initial_bearing_to(p2));
    }

    #[test]
    fn returns_180_south_pole_to_date_line() {
        let p1 = LatLongPos::south_pole(S84);
        let p2 = LatLongPos::from_s84(50.0, 180.0);
        assert_eq!(
            Ok(Angle::from_decimal_degrees(180.0)),
            p1.initial_bearing_to(p2)
        );
    }

    #[test]
    fn returns_initial_bearing_compass_angle() {
        let p1 = LatLongPos::from_s84(50.066389, -5.714722);
        let p2 = LatLongPos::from_s84(58.643889, -3.07);
        assert_eq!(
            Ok(Angle::from_decimal_degrees(9.1198173275)),
            p1.initial_bearing_to(p2)
        );
        assert_eq!(
            Ok(Angle::from_decimal_degrees(191.27520031611112)),
            p2.initial_bearing_to(p1)
        );
    }
}
