mod lat_long_pos {

    use jord::{Angle, LatLongPos, Length};

    #[test]
    fn returns_p0_if_distance_is_0() {
        let p0 = LatLongPos::from_s84(53.320556, -1.729722);
        assert_eq!(
            p0,
            p0.destination_pos(Angle::from_decimal_degrees(96.0217), Length::zero())
        );
    }

    #[test]
    fn returns_position_along_great_circle_at_distance_and_bearing() {
        let p0 = LatLongPos::from_s84(53.320556, -1.729722);
        let p1 = LatLongPos::from_s84(53.18826954833333, 0.13327449055555557);
        assert_eq!(
            p1,
            p0.destination_pos(
                Angle::from_decimal_degrees(96.0217),
                Length::from_metres(124800.0)
            )
        );
    }
}
