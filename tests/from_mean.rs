mod lat_long_pos {

    use jord::models::S84Model;
    use jord::{Error, LatLongPos, Length, SurfacePos};

    #[test]
    fn mean_no_position() {
        let empty: Vec<LatLongPos<S84Model>> = Vec::new();
        assert_eq!(
            Err(Error::NotEnoughPositions),
            LatLongPos::from_mean(&empty)
        );
    }

    #[test]
    fn mean_one_position() {
        let p = LatLongPos::from_s84(0.0, 0.0);
        assert_eq!(Ok(p), LatLongPos::from_mean(&vec![p]));
    }

    #[test]
    fn mean_antipode() {
        let ps = vec![
            LatLongPos::from_s84(45.0, 1.0),
            LatLongPos::from_s84(45.0, 2.0),
            LatLongPos::from_s84(46.0, 2.0),
            LatLongPos::from_s84(46.0, 1.0),
            LatLongPos::from_s84(45.0, 2.0).antipode(),
        ];
        assert_eq!(Err(Error::AntipodalPositions), LatLongPos::from_mean(&ps));
    }

    #[test]
    fn mean_is_equidistant() {
        let ps = vec![
            LatLongPos::from_s84(40.0, -40.0),
            LatLongPos::from_s84(40.0, 40.0),
            LatLongPos::from_s84(-40.0, 40.0),
            LatLongPos::from_s84(-40.0, -40.0),
        ];
        let mean = LatLongPos::from_mean(&ps);
        let expected = LatLongPos::from_s84(0.0, 0.0);
        assert_eq!(Ok(expected), mean);

        let mut dists: Vec<Length> = ps.iter().map(|p| p.distance_to(expected)).collect();
        dists.dedup();
        assert_eq!(1, dists.len());
    }
}
