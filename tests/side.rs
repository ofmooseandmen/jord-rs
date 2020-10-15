/*mod places;
pub use places::*;

mod lat_long_pos {

    use crate::places::*;
    use jord::models::S84;
    use jord::{GreatCircle, LatLongPos, Side};

    #[test]
    fn side_of_none() {
        let p = LatLongPos::from_s84(0.0, 0.0);
        let gc = GreatCircle::from_lat_longs(
            LatLongPos::from_s84(45.0, 0.0),
            LatLongPos::north_pole(S84),
        )
        .unwrap();
        assert_eq!(Side::None, p.side_of(gc));
    }

    #[test]
    fn side_of_left() {
        assert_eq!(
            Side::LeftOf,
            ystad().side_of(GreatCircle::from_lat_longs(kristianstad(), helsingborg()).unwrap())
        );
        assert_eq!(
            Side::LeftOf,
            malmo().side_of(GreatCircle::from_lat_longs(lund(), helsingborg()).unwrap())
        );
    }

    #[test]
    fn side_of_right() {
        assert_eq!(
            Side::RightOf,
            ystad().side_of(GreatCircle::from_lat_longs(helsingborg(), kristianstad()).unwrap())
        );
        assert_eq!(
            Side::RightOf,
            malmo().side_of(GreatCircle::from_lat_longs(helsingborg(), lund()).unwrap())
        );
    }
}
*/
