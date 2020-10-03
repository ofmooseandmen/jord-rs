mod lat_long_pos {

    use jord::{Angle, LatLongPos, Length};

    #[test]
    fn zero_distance_destination_pos() {
        let p0 = LatLongPos::from_s84(53.320556, -1.729722);
        assert_eq!(
            p0,
            p0.destination_pos(Angle::from_decimal_degrees(96.0217), Length::zero())
        );
    }

    #[test]
    fn destination_pos() {
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
