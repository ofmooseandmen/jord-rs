mod lat_long_pos {

    use jord::models::{S84Model, S84};
    use jord::{Error, LatLongPos, MinorArc, NvectorPos};

    #[test]
    fn no_projection_if_pos_is_normal_1() {
        let start = LatLongPos::from_s84(3.0, -10.0);
        let end = LatLongPos::from_s84(4.0, 10.0);
        let ma = MinorArc::from_lat_longs(start, end).unwrap();
        let pos: LatLongPos<S84Model> =
            NvectorPos::new(start.to_nvector().cross(end.to_nvector()), start.model()).into();
        assert_eq!(Err(Error::CoincidentalPositions), pos.projection_onto(ma));
    }

    #[test]
    fn no_projection_if_pos_is_normal_2() {
        let start = LatLongPos::from_s84(0.0, -10.0);
        let end = LatLongPos::from_s84(0.0, 10.0);
        let ma = MinorArc::from_lat_longs(start, end).unwrap();
        let pos = LatLongPos::north_pole(S84);
        assert_eq!(Err(Error::CoincidentalPositions), pos.projection_onto(ma));
    }

    #[test]
    fn no_projection_if_pos_antipodal() {
        let start = LatLongPos::from_s84(0.0, -10.0);
        let end = LatLongPos::from_s84(0.0, 10.0);
        let ma = MinorArc::from_lat_longs(start, end).unwrap();
        let pos = LatLongPos::south_pole(S84);
        assert_eq!(Err(Error::AntipodalPositions), pos.projection_onto(ma));
    }
}
