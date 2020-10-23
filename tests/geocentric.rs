mod ellipsoidal_test {

    use jord::models::{WGS84Model, WGS84};
    use jord::{GeocentricPos, GeodeticPos, Length, Microarcsecond, Micrometre, Millimetre};

    #[test]
    fn to_geocentric() {
        let geodetics = vec![
            GeodeticPos::from_decimal_lat_long(0.0, 0.0, Length::zero(), WGS84),
            GeodeticPos::north_pole(WGS84),
            GeodeticPos::south_pole(WGS84),
            GeodeticPos::from_decimal_lat_long(45.0, 45.0, Length::from_metres(500.0), WGS84),
            GeodeticPos::from_decimal_lat_long(-45.0, -45.0, Length::from_metres(500.0), WGS84),
        ];

        let actuals_1: Vec<GeocentricPos<WGS84Model>> = geodetics
            .iter()
            .map(|g| g.to_geocentric().round(Micrometre))
            .collect();

        let actuals_2: Vec<GeocentricPos<WGS84Model>> = geodetics
            .iter()
            .map(|g| GeocentricPos::from_geodetic(*g).round(Micrometre))
            .collect();

        let expecteds = vec![
            GeocentricPos::from_metres(6378137.0, 0.0, 0.0, WGS84).round(Micrometre),
            GeocentricPos::north_pole(WGS84).round(Micrometre),
            GeocentricPos::south_pole(WGS84).round(Micrometre),
            GeocentricPos::from_metres(3194669.145061, 3194669.145061, 4487701.962257, WGS84)
                .round(Micrometre),
            GeocentricPos::from_metres(3194669.145061, -3194669.145061, -4487701.962257, WGS84)
                .round(Micrometre),
        ];

        assert_eq!(expecteds, actuals_1);
        assert_eq!(expecteds, actuals_2);
    }

    #[test]
    fn to_geodetic() {
        let geocentrics = vec![
            GeocentricPos::from_metres(6378137.0, 0.0, 0.0, WGS84),
            GeocentricPos::north_pole(WGS84),
            GeocentricPos::south_pole(WGS84),
            GeocentricPos::from_metres(3194669.145061, 3194669.145061, 4487701.962257, WGS84),
            GeocentricPos::from_metres(3194669.145061, -3194669.145061, -4487701.962257, WGS84),
        ];

        let actuals_1: Vec<GeodeticPos<WGS84Model>> = geocentrics
            .iter()
            .map(|g| g.to_geodetic().round(Microarcsecond, Millimetre))
            .collect();

        let actuals_2: Vec<GeodeticPos<WGS84Model>> = geocentrics
            .iter()
            .map(|g| GeodeticPos::from_geocentric(*g).round(Microarcsecond, Millimetre))
            .collect();

        let expecteds = vec![
            GeodeticPos::from_decimal_lat_long(0.0, 0.0, Length::zero(), WGS84)
                .round(Microarcsecond, Millimetre),
            GeodeticPos::north_pole(WGS84).round(Microarcsecond, Millimetre),
            GeodeticPos::south_pole(WGS84).round(Microarcsecond, Millimetre),
            GeodeticPos::from_decimal_lat_long(45.0, 45.0, Length::from_metres(500.0), WGS84)
                .round(Microarcsecond, Millimetre),
            GeodeticPos::from_decimal_lat_long(-45.0, -45.0, Length::from_metres(500.0), WGS84)
                .round(Microarcsecond, Millimetre),
        ];

        assert_eq!(expecteds, actuals_1);
        assert_eq!(expecteds, actuals_2);
    }
}

mod spherical_test {

    use jord::models::{S84Model, S84};
    use jord::{GeocentricPos, GeodeticPos, Length, Microarcsecond, Micrometre, Millimetre};

    #[test]
    fn to_geocentric() {
        let geodetics = vec![
            GeodeticPos::from_decimal_lat_long(0.0, 0.0, Length::zero(), S84),
            GeodeticPos::north_pole(S84),
            GeodeticPos::south_pole(S84),
            GeodeticPos::from_decimal_lat_long(45.0, 45.0, Length::from_metres(500.0), S84),
            GeodeticPos::from_decimal_lat_long(-45.0, -45.0, Length::from_metres(500.0), S84),
        ];

        let actuals_1: Vec<GeocentricPos<S84Model>> = geodetics
            .iter()
            .map(|g| g.to_geocentric().round(Micrometre))
            .collect();

        let actuals_2: Vec<GeocentricPos<S84Model>> = geodetics
            .iter()
            .map(|g| GeocentricPos::from_geodetic(*g).round(Micrometre))
            .collect();

        let expecteds = vec![
            GeocentricPos::from_metres(6371008.771415, 0.0, 0.0, S84).round(Micrometre),
            GeocentricPos::north_pole(S84).round(Micrometre),
            GeocentricPos::south_pole(S84).round(Micrometre),
            GeocentricPos::from_metres(3185754.385708, 3185754.385708, 4505337.058657, S84)
                .round(Micrometre),
            GeocentricPos::from_metres(3185754.385708, -3185754.385708, -4505337.058657, S84)
                .round(Micrometre),
        ];

        assert_eq!(expecteds, actuals_1);
        assert_eq!(expecteds, actuals_2);
    }

    #[test]
    fn to_geodetic() {
        let geocentrics = vec![
            GeocentricPos::from_metres(6371008.771415, 0.0, 0.0, S84),
            GeocentricPos::north_pole(S84),
            GeocentricPos::south_pole(S84),
            GeocentricPos::from_metres(3185754.385708, 3185754.385708, 4505337.058657, S84),
            GeocentricPos::from_metres(3185754.385708, -3185754.385708, -4505337.058657, S84),
        ];

        let actuals_1: Vec<GeodeticPos<S84Model>> = geocentrics
            .iter()
            .map(|g| g.to_geodetic().round(Microarcsecond, Millimetre))
            .collect();

        let actuals_2: Vec<GeodeticPos<S84Model>> = geocentrics
            .iter()
            .map(|g| GeodeticPos::from_geocentric(*g).round(Microarcsecond, Millimetre))
            .collect();

        let expecteds = vec![
            GeodeticPos::from_decimal_lat_long(0.0, 0.0, Length::zero(), S84)
                .round(Microarcsecond, Millimetre),
            GeodeticPos::north_pole(S84).round(Microarcsecond, Millimetre),
            GeodeticPos::south_pole(S84).round(Microarcsecond, Millimetre),
            GeodeticPos::from_decimal_lat_long(45.0, 45.0, Length::from_metres(500.0), S84)
                .round(Microarcsecond, Millimetre),
            GeodeticPos::from_decimal_lat_long(-45.0, -45.0, Length::from_metres(500.0), S84)
                .round(Microarcsecond, Millimetre),
        ];

        assert_eq!(expecteds, actuals_1);
        assert_eq!(expecteds, actuals_2);
    }
}
