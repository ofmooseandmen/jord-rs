mod pole {

    use jord::models::S84;
    use jord::HorizontalPos;

    #[test]
    fn north_pole() {
        let np = HorizontalPos::north_pole(S84);
        assert_eq!(90.0, np.to_lat_long().latitude().decimal_degrees());
        assert_eq!(0.0, np.to_lat_long().longitude().decimal_degrees());
    }

    #[test]
    fn south_pole() {
        let sp = HorizontalPos::south_pole(S84);
        assert_eq!(-90.0, sp.to_lat_long().latitude().decimal_degrees());
        assert_eq!(0.0, sp.to_lat_long().longitude().decimal_degrees());
    }

    #[test]
    fn longitude_at_north_pole_is_0() {
        let lat = 90.0;
        for x in 0..360 {
            let lon = x as f64 - 180.0;
            let p = HorizontalPos::from_decimal_lat_long(lat, lon, S84);
            assert_eq!(90.0, p.to_lat_long().latitude().decimal_degrees());
            assert_eq!(0.0, p.to_lat_long().longitude().decimal_degrees());
        }
    }

    #[test]
    fn longitude_at_south_pole_is_0() {
        let lat = -90.0;
        for x in 0..360 {
            let lon = x as f64 - 180.0;
            let p = HorizontalPos::from_decimal_lat_long(lat, lon, S84);
            assert_eq!(-90.0, p.to_lat_long().latitude().decimal_degrees());
            assert_eq!(0.0, p.to_lat_long().longitude().decimal_degrees());
        }
    }
}

mod wrap {

    use jord::{Angle, HorizontalPos, Microarcsecond};

    #[test]
    fn no_wrapping() {
        test(55.555, 22.222, 55.555, 22.222);
    }

    #[test]
    fn positive_lat_wrapping_91_degrees() {
        test(91.0, 54.0, 89.0, -126.0);
    }

    #[test]
    fn positive_lat_wrapping_181_degrees() {
        test(181.0, 0.0, -1.0, 180.0);
    }

    #[test]
    fn positive_lat_wrapping_271_degrees() {
        test(271.0, 0.0, -89.0, 0.0);
    }

    #[test]
    fn positive_lat_wrapping_361_degrees() {
        test(361.0, 0.0, 1.0, 0.0);
    }

    #[test]
    fn positive_lat_wrapping_631_degrees() {
        test(631.0, 0.0, -89.0, 0.0);
    }

    #[test]
    fn positive_lat_wrapping_721_degrees() {
        test(721.0, 0.0, 1.0, 0.0);
    }

    #[test]
    fn negative_lat_wrapping_91_degrees() {
        test(-91.0, 54.0, -89.0, -126.0);
    }

    #[test]
    fn negative_lat_wrapping_181_degrees() {
        test(-181.0, 0.0, 1.0, 180.0);
    }

    #[test]
    fn negative_lat_wrapping_271_degrees() {
        test(-271.0, 0.0, 89.0, 0.0);
    }

    #[test]
    fn negative_lat_wrapping_361_degrees() {
        test(-361.0, 0.0, -1.0, 0.0);
    }

    #[test]
    fn negative_lat_wrapping_631_degrees() {
        test(-631.0, 0.0, 89.0, 0.0);
    }

    #[test]
    fn negative_lat_wrapping_721_degrees() {
        test(-721.0, 0.0, -1.0, 0.0);
    }

    #[test]
    fn positive_lon_wrapping_181_degrees() {
        test(0.0, 181.0, 0.0, -179.0);
    }

    #[test]
    fn positive_lon_wrapping_271_degrees() {
        test(0.0, 271.0, 0.0, -89.0);
    }

    #[test]
    fn positive_lon_wrapping_361_degrees() {
        test(0.0, 361.0, 0.0, 1.0);
    }

    #[test]
    fn positive_lon_wrapping_631_degrees() {
        test(0.0, 631.0, 0.0, -89.0);
    }

    #[test]
    fn positive_lon_wrapping_721_degrees() {
        test(0.0, 721.0, 0.0, 1.0);
    }

    #[test]
    fn negative_lon_wrapping_181_degrees() {
        test(0.0, -181.0, 0.0, 179.0);
    }

    #[test]
    fn negative_lon_wrapping_271_degrees() {
        test(0.0, -271.0, 0.0, 89.0);
    }

    #[test]
    fn negative_lon_wrapping_361_degrees() {
        test(0.0, -361.0, 0.0, -1.0);
    }

    #[test]
    fn negative_lon_wrapping_631_degrees() {
        test(0.0, -631.0, 0.0, 89.0);
    }

    #[test]
    fn negative_lon_wrapping_721_degrees() {
        test(0.0, -721.0, 0.0, -1.0);
    }

    fn test(lat: f64, lon: f64, expected_lat: f64, expected_lon: f64) {
        let actual = HorizontalPos::from_s84(lat, lon)
            .to_lat_long()
            .round(Microarcsecond);
        assert_eq!(Angle::from_decimal_degrees(expected_lat), actual.latitude());
        assert_eq!(
            Angle::from_decimal_degrees(expected_lon),
            actual.longitude()
        );
    }
}

// FIXME complete
mod resolution {

    use jord::models::S84;
    use jord::{HorizontalPos, Microarcsecond, Vec3};

    #[test]
    fn micro() {
        let p1 = HorizontalPos::new(Vec3::new(0.5, 0.5, 0.5_f64.sqrt()), S84);
        let p2 = HorizontalPos::new(
            Vec3::new(0.5000000000000001, 0.5000000000000001, 0.5f64.sqrt()),
            S84,
        );
        assert_ne!(p1, p2);

        assert_eq!(p1.round(Microarcsecond), p2.round(Microarcsecond));
    }
}
