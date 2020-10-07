mod lat_long_pos {

    use jord::models::S84;
    use jord::{Error, LatLongPos, MinorArc, SurfacePos};

    #[test]
    fn no_minor_arc_between_same_position() {
        let p = LatLongPos::from_s84(54.0, 154.0);
        assert_eq!(
            Err(Error::CoincidentalPositions),
            MinorArc::from_lat_longs(p, p)
        );
    }

    #[test]
    fn no_minor_arc_between_poles() {
        assert_eq!(
            Err(Error::AntipodalPositions),
            MinorArc::from_lat_longs(LatLongPos::north_pole(S84), LatLongPos::south_pole(S84))
        );
    }

    #[test]
    fn no_minor_arc_between_antipodes() {
        let p = LatLongPos::from_s84(54.0, 154.0);
        assert_eq!(
            Err(Error::AntipodalPositions),
            MinorArc::from_lat_longs(p, p.antipode())
        );
    }
}

mod nvector_pos {

    use jord::models::S84;
    use jord::{Error, MinorArc, NvectorPos, SurfacePos};

    #[test]
    fn no_minor_arc_between_same_position() {
        let p = NvectorPos::from_s84(54.0, 154.0);
        assert_eq!(
            Err(Error::CoincidentalPositions),
            MinorArc::from_nvectors(p, p)
        );
    }

    #[test]
    fn no_minor_arc_between_poles() {
        assert_eq!(
            Err(Error::AntipodalPositions),
            MinorArc::from_nvectors(NvectorPos::north_pole(S84), NvectorPos::south_pole(S84))
        );
    }

    #[test]
    fn no_minor_arc_between_antipodes() {
        let p = NvectorPos::from_s84(54.0, 154.0);
        assert_eq!(
            Err(Error::AntipodalPositions),
            MinorArc::from_nvectors(p, p.antipode())
        );
    }
}
