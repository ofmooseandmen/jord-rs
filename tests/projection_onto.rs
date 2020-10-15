/*mod lat_long_pos {

    use jord::models::{S84Model, S84};
    use jord::{Error, GreatCircle, LatLongPos, Length, MinorArc, NvectorPos, SurfacePos};

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

    #[test]
    fn no_projection_outside() {
        let ma = MinorArc::from_lat_longs(
            LatLongPos::from_s84(54.0, 15.0),
            LatLongPos::from_s84(54.0, 20.0),
        )
        .unwrap();
        let pos = LatLongPos::from_s84(54.0, 10.0);
        assert_eq!(Err(Error::NoIntersection), pos.projection_onto(ma));
    }

    #[test]
    fn projection_onto() {
        let start = LatLongPos::from_s84(53.3206, -1.7297);
        let end = LatLongPos::from_s84(53.1887, 0.1334);
        let ma = MinorArc::from_lat_longs(start, end).unwrap();
        let pos = LatLongPos::from_s84(53.2611, -0.7972);
        let actual = pos.projection_onto(ma);
        assert_eq!(
            Ok(LatLongPos::from_s84(53.25835330666666, -0.7977433863888889)),
            actual
        );
        // absolute cross track distance from p to great circle should be distance between projection and p
        let stx = pos.cross_track_distance_to(GreatCircle::from_lat_longs(start, end).unwrap());
        let dst = pos.distance_to(actual.unwrap());
        assert_eq!(true, (stx.abs() - dst).abs() < Length::from_micrometres(5));
    }

    #[test]
    fn projection_onto_start_1() {
        let start = LatLongPos::from_s84(54.0, 15.0);
        let ma = MinorArc::from_lat_longs(start, LatLongPos::from_s84(54.0, 20.0)).unwrap();
        assert_eq!(Ok(start), start.projection_onto(ma));
    }

    #[test]
    fn projection_onto_start_2() {
        let start = LatLongPos::from_s84(13.733333587646484, 100.5);
        let ma = MinorArc::from_lat_longs(start, LatLongPos::from_s84(12.0, 100.58499908447266))
            .unwrap();
        assert_eq!(Ok(start), start.projection_onto(ma));
    }

    #[test]
    fn projection_onto_end_1() {
        let end = LatLongPos::from_s84(54.0, 20.0);
        let ma = MinorArc::from_lat_longs(LatLongPos::from_s84(54.0, 15.0), end).unwrap();
        assert_eq!(Ok(end), end.projection_onto(ma));
    }

    #[test]
    fn projection_onto_end_2() {
        let end = LatLongPos::from_s84(12.0, 100.58499908447266);
        let ma =
            MinorArc::from_lat_longs(LatLongPos::from_s84(13.733333587646484, 100.5), end).unwrap();
        assert_eq!(Ok(end), end.projection_onto(ma));
    }
}
*/
