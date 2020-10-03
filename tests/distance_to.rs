mod lat_long_pos {

    use jord::models::S84;
    use jord::{LatLongPos, Length};

    #[test]
    fn returns_0_equal_positions() {
        let p = LatLongPos::from_s84(50.066389, -5.714722);
        assert_eq!(Length::zero(), p.distance_to(p));
    }

    #[test]
    fn returns_distance_between_2_positions() {
        let p1 = LatLongPos::from_s84(50.066389, -5.714722);
        let p2 = LatLongPos::from_s84(58.643889, -3.07);
        assert_eq!(Length::from_metres(968854.878007), p1.distance_to(p2));
    }

    #[test]
    fn handles_singularity_at_poles() {
        assert_eq!(
            Length::from_kilometres(20015.114352233),
            LatLongPos::north_pole(S84).distance_to(LatLongPos::south_pole(S84))
        );
    }

    #[test]
    fn handles_discontinuity_at_date_line() {
        let p1 = LatLongPos::from_s84(50.066389, -179.999722);
        let p2 = LatLongPos::from_s84(50.066389, 179.999722);
        assert_eq!(Length::from_metres(39.685092), p1.distance_to(p2));
    }
}
