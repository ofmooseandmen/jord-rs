mod lat_long_pos {

    use jord::{Error, LatLongPos};

    #[test]
    fn returns_err_fraction() {
        let p1 = LatLongPos::from_s84(44.0, 44.0);
        let p2 = LatLongPos::from_s84(46.0, 46.0);
        assert_eq!(Err(Error::OutOfRange), p1.intermediate_pos_to(p2, -0.9));
        assert_eq!(Err(Error::OutOfRange), p1.intermediate_pos_to(p2, 1.1));
    }

    #[test]
    fn returns_p1() {
        let p1 = LatLongPos::from_s84(44.0, 44.0);
        let p2 = LatLongPos::from_s84(46.0, 46.0);
        assert_eq!(Ok(p1), p1.intermediate_pos_to(p2, 0.0));
    }

    #[test]
    fn returns_p2() {
        let p1 = LatLongPos::from_s84(44.0, 44.0);
        let p2 = LatLongPos::from_s84(46.0, 46.0);
        assert_eq!(Ok(p2), p1.intermediate_pos_to(p2, 1.0));
    }

    #[test]
    fn returns_pos() {
        let p1 = LatLongPos::from_s84(53.479444, -2.245278);
        let p2 = LatLongPos::from_s84(55.605833, 13.035833);
        let pe = LatLongPos::from_s84(54.78355703138889, 5.194985318055555);
        assert_eq!(Ok(pe), p1.intermediate_pos_to(p2, 0.5));
    }
}
