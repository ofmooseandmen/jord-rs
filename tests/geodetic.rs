mod lat_long {

    mod pole {

        use jord::models::S84;
        use jord::LatLongPos;

        #[test]
        fn north_pole() {
            let np = LatLongPos::north_pole(S84);
            assert_eq!(90.0, np.latitude().as_decimal_degrees());
            assert_eq!(0.0, np.longitude().as_decimal_degrees());
        }

        #[test]
        fn south_pole() {
            let sp = LatLongPos::south_pole(S84);
            assert_eq!(-90.0, sp.latitude().as_decimal_degrees());
            assert_eq!(0.0, sp.longitude().as_decimal_degrees());
        }

        #[test]
        fn longitude_at_north_pole_is_0() {
            let lat = 90.0;
            for x in 0..360 {
                let lon = x as f64 - 180.0;
                let p = LatLongPos::from_decimal_degrees(lat, lon, S84);
                assert_eq!(90.0, p.latitude().as_decimal_degrees());
                assert_eq!(0.0, p.longitude().as_decimal_degrees());
            }
        }

        #[test]
        fn longitude_at_south_pole_is_0() {
            let lat = -90.0;
            for x in 0..360 {
                let lon = x as f64 - 180.0;
                let p = LatLongPos::from_decimal_degrees(lat, lon, S84);
                assert_eq!(-90.0, p.latitude().as_decimal_degrees());
                assert_eq!(0.0, p.longitude().as_decimal_degrees());
            }
        }
    }

    mod wrap {

        use jord::{Angle, LatLongPos};

        #[test]
        fn no_wrapping() {
            let p = LatLongPos::from_s84(55.555, 22.222);
            assert_eq!(Angle::from_decimal_degrees(55.555), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(22.222), p.longitude());
        }

        #[test]
        fn positive_lat_wrapping_91_degrees() {
            let p = LatLongPos::from_s84(91.0, 54.0);
            assert_eq!(Angle::from_decimal_degrees(89.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(-126.0), p.longitude());
        }

        #[test]
        fn positive_lat_wrapping_181_degrees() {
            let p = LatLongPos::from_s84(181.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(-1.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(180.0), p.longitude());
        }

        #[test]
        fn positive_lat_wrapping_271_degrees() {
            let p = LatLongPos::from_s84(271.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(-89.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn positive_lat_wrapping_361_degrees() {
            let p = LatLongPos::from_s84(361.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(1.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn positive_lat_wrapping_631_degrees() {
            let p = LatLongPos::from_s84(631.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(-89.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn positive_lat_wrapping_721_degrees() {
            let p = LatLongPos::from_s84(721.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(1.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn negative_lat_wrapping_91_degrees() {
            let p = LatLongPos::from_s84(-91.0, 54.0);
            assert_eq!(Angle::from_decimal_degrees(-89.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(-126.0), p.longitude());
        }

        #[test]
        fn negative_lat_wrapping_181_degrees() {
            let p = LatLongPos::from_s84(-181.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(1.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(180.0), p.longitude());
        }

        #[test]
        fn negative_lat_wrapping_271_degrees() {
            let p = LatLongPos::from_s84(-271.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(89.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn negative_lat_wrapping_361_degrees() {
            let p = LatLongPos::from_s84(-361.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(-1.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn negative_lat_wrapping_631_degrees() {
            let p = LatLongPos::from_s84(-631.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(89.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn negative_lat_wrapping_721_degrees() {
            let p = LatLongPos::from_s84(-721.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(-1.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn positive_lon_wrapping_181_degrees() {
            let p = LatLongPos::from_s84(0.0, 181.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(-179.0), p.longitude());
        }

        #[test]
        fn positive_lon_wrapping_271_degrees() {
            let p = LatLongPos::from_s84(0.0, 271.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(-89.0), p.longitude());
        }

        #[test]
        fn positive_lon_wrapping_361_degrees() {
            let p = LatLongPos::from_s84(0.0, 361.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(1.0), p.longitude());
        }

        #[test]
        fn positive_lon_wrapping_631_degrees() {
            let p = LatLongPos::from_s84(0.0, 631.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(-89.0), p.longitude());
        }

        #[test]
        fn positive_lon_wrapping_721_degrees() {
            let p = LatLongPos::from_s84(0.0, 721.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(1.0), p.longitude());
        }

        #[test]
        fn negative_lon_wrapping_181_degrees() {
            let p = LatLongPos::from_s84(0.0, -181.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(179.0), p.longitude());
        }

        #[test]
        fn negative_lon_wrapping_271_degrees() {
            let p = LatLongPos::from_s84(0.0, -271.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(89.0), p.longitude());
        }

        #[test]
        fn negative_lon_wrapping_361_degrees() {
            let p = LatLongPos::from_s84(0.0, -361.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(-1.0), p.longitude());
        }

        #[test]
        fn negative_lon_wrapping_631_degrees() {
            let p = LatLongPos::from_s84(0.0, -631.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(89.0), p.longitude());
        }

        #[test]
        fn negative_lon_wrapping_721_degrees() {
            let p = LatLongPos::from_s84(0.0, -721.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(-1.0), p.longitude());
        }
    }
}

mod resolution {

    use jord::models::{S84Model, S84};
    use jord::{LatLongPos, NvectorPos, Vec3};

    #[test]
    fn from_nvectors() {
        let nv1 = NvectorPos::new(Vec3::new(0.5, 0.5, 0.5_f64.sqrt()), S84);
        let nv2 = NvectorPos::new(
            Vec3::new(0.5000000000000001, 0.5000000000000001, 0.5_f64.sqrt()),
            S84,
        );
        assert_ne!(nv1, nv2);

        let ll1: LatLongPos<S84Model> = nv1.into();
        let ll2: LatLongPos<S84Model> = nv2.into();
        assert_eq!(ll1, ll2);

        let nv3: NvectorPos<S84Model> = ll1.into();
        let nv4: NvectorPos<S84Model> = ll2.into();
        assert_eq!(nv3, nv4);
    }

    #[test]
    fn from_lat_long() {
        let ll1 = LatLongPos::from_s84(45.0, 45.0);
        let ll2 = LatLongPos::from_s84(45.0000000005, 45.0000000005);
        let ll3 = LatLongPos::from_s84(45.0000000001, 45.0000000001);

        assert_ne!(ll1, ll2);
        assert_eq!(ll1, ll3);
    }
}
