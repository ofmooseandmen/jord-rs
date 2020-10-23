mod ellipsoidal_test {

    use jord::models::WGS84;
    use jord::{GeocentricPos, GeodeticPos};

    #[test]
    fn north_pole_to_geocentric() {
        assert_eq!(
            GeocentricPos::north_pole(WGS84),
            GeodeticPos::north_pole(WGS84).to_geocentric()
        );
        assert_eq!(
            GeocentricPos::north_pole(WGS84),
            GeocentricPos::from_geodetic(GeodeticPos::north_pole(WGS84))
        );
    }

    #[test]
    fn south_pole_to_geocentric() {
        assert_eq!(
            GeocentricPos::south_pole(WGS84),
            GeodeticPos::south_pole(WGS84).to_geocentric()
        );
        assert_eq!(
            GeocentricPos::south_pole(WGS84),
            GeocentricPos::from_geodetic(GeodeticPos::south_pole(WGS84))
        );
    }

    #[test]
    fn north_pole_to_geodetic() {
        assert_eq!(
            GeodeticPos::north_pole(WGS84),
            GeocentricPos::north_pole(WGS84).to_geodetic()
        );
        assert_eq!(
            GeodeticPos::north_pole(WGS84),
            GeodeticPos::from_geocentric(GeocentricPos::north_pole(WGS84))
        );
    }

    #[test]
    fn south_pole_to_geodetic() {
        assert_eq!(
            GeodeticPos::south_pole(WGS84),
            GeocentricPos::south_pole(WGS84).to_geodetic()
        );
        assert_eq!(
            GeodeticPos::south_pole(WGS84),
            GeodeticPos::from_geocentric(GeocentricPos::south_pole(WGS84))
        );
    }
}

mod spherical_test {

    use jord::models::S84;
    use jord::{GeocentricPos, GeodeticPos};

    #[test]
    fn north_pole_to_geocentric() {
        assert_eq!(
            GeocentricPos::north_pole(S84),
            GeodeticPos::north_pole(S84).to_geocentric()
        );
        assert_eq!(
            GeocentricPos::north_pole(S84),
            GeocentricPos::from_geodetic(GeodeticPos::north_pole(S84))
        );
    }

    #[test]
    fn south_pole_to_geocentric() {
        assert_eq!(
            GeocentricPos::south_pole(S84),
            GeodeticPos::south_pole(S84).to_geocentric()
        );
        assert_eq!(
            GeocentricPos::south_pole(S84),
            GeocentricPos::from_geodetic(GeodeticPos::south_pole(S84))
        );
    }

    #[test]
    fn north_pole_to_geodetic() {
        assert_eq!(
            GeodeticPos::north_pole(S84),
            GeocentricPos::north_pole(S84).to_geodetic()
        );
        assert_eq!(
            GeodeticPos::north_pole(S84),
            GeodeticPos::from_geocentric(GeocentricPos::north_pole(S84))
        );
    }

    #[test]
    fn south_pole_to_geodetic() {
        assert_eq!(
            GeodeticPos::south_pole(S84),
            GeocentricPos::south_pole(S84).to_geodetic()
        );
        assert_eq!(
            GeodeticPos::south_pole(S84),
            GeodeticPos::from_geocentric(GeocentricPos::south_pole(S84))
        );
    }
}
