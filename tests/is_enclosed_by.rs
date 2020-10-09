mod places;

mod lat_long_pos {

    use crate::places::*;
    use jord::models::S84Model;
    use jord::{LatLongPos, SurfacePos};

    fn p1() -> LatLongPos<S84Model> {
        LatLongPos::from_s84(45.0, 1.0)
    }

    fn p2() -> LatLongPos<S84Model> {
        LatLongPos::from_s84(45.0, 2.0)
    }

    fn p3() -> LatLongPos<S84Model> {
        LatLongPos::from_s84(46.0, 1.0)
    }

    fn p4() -> LatLongPos<S84Model> {
        LatLongPos::from_s84(46.0, 2.0)
    }

    fn p5() -> LatLongPos<S84Model> {
        LatLongPos::from_s84(45.1, 1.1)
    }

    #[test]
    fn pos_enclosed_by_closed_poly() {
        assert_eq!(
            true,
            p5().is_enclosed_by(&vec![p1(), p2(), p3(), p4(), p1()])
        );
    }

    #[test]
    fn pos_enclosed_by_concave_poly() {
        let vertices = vec![malmo(), ystad(), kristianstad(), helsingborg(), lund()];
        assert_eq!(true, hoor().is_enclosed_by(&vertices));
    }

    #[test]
    fn pos_not_enclosed_by_concave_poly() {
        let vertices = vec![malmo(), ystad(), kristianstad(), helsingborg(), lund()];
        assert_eq!(false, hassleholm().is_enclosed_by(&vertices));
    }

    #[test]
    fn pos_not_enclosed_by_empty_poly() {
        assert_eq!(false, p5().is_enclosed_by(&vec![]));
    }

    #[test]
    fn pos_not_enclosed_by_not_at_least_triangle() {
        assert_eq!(false, p1().is_enclosed_by(&vec![p2()]));
        assert_eq!(false, p1().is_enclosed_by(&vec![p2(), p3()]));
        assert_eq!(false, p1().is_enclosed_by(&vec![p2(), p3(), p2()]));
    }

    #[test]
    fn pos_enclosed_by_poly() {
        assert_eq!(true, p5().is_enclosed_by(&vec![p1(), p2(), p3(), p4()]));
    }

    #[test]
    fn pos_not_enclosed_by_poly() {
        assert_eq!(
            false,
            p5().antipode()
                .is_enclosed_by(&vec![p1(), p2(), p3(), p4()])
        );
        assert_eq!(
            false,
            lund().is_enclosed_by(&vec![malmo(), kristianstad(), ystad()])
        );
    }

    #[test]
    fn vertex_not_enclosed_by_convex_poly() {
        let convex = vec![p1(), p2(), p3(), p4()];
        for v in &convex {
            assert_eq!(false, v.is_enclosed_by(&convex));
        }
    }

    #[test]
    fn vertex_not_enclosed_by_concave_poly() {
        let concave = vec![malmo(), ystad(), kristianstad(), helsingborg(), lund()];
        for v in &concave {
            assert_eq!(false, v.is_enclosed_by(&concave));
        }
    }

    #[test]
    fn pos_on_edge_not_enclosed_by() {
        let i = helsingborg().intermediate_pos_to(lund(), 0.5).unwrap();
        let poly1 = vec![malmo(), kristianstad(), helsingborg(), lund()];
        let poly2 = vec![helsingborg(), lund(), copenhagen()];
        assert_eq!(true, i.is_enclosed_by(&poly1));
        assert_eq!(false, i.is_enclosed_by(&poly2));
    }
}
