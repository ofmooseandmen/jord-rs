mod lat_long_pos {

    use jord::models::S84;
    use jord::{LatLongPos, Length};

    #[test]
    fn distance_to_same_position() {
        let p = LatLongPos::from_s84(50.066389, -5.714722);
        assert_eq!(Length::zero(), p.distance_to(p));
    }

    #[test]
    fn distance_to() {
        let p1 = LatLongPos::from_s84(50.066389, -5.714722);
        let p2 = LatLongPos::from_s84(58.643889, -3.07);
        assert_eq!(Length::from_metres(968854.878007), p1.distance_to(p2));
    }

    #[test]
    fn north_pole_distance_to_south_pole() {
        assert_eq!(
            Length::from_kilometres(20015.114352233),
            LatLongPos::north_pole(S84).distance_to(LatLongPos::south_pole(S84))
        );
    }

    #[test]
    fn distance_to_across_date_line() {
        let p1 = LatLongPos::from_s84(50.066389, -179.999722);
        let p2 = LatLongPos::from_s84(50.066389, 179.999722);
        assert_eq!(Length::from_metres(39.685096), p1.distance_to(p2));
    }
}
