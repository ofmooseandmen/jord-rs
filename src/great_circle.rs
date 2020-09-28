use crate::internal::Positioning;
use crate::{Angle, Length, NvectorPos, Spherical, Vec3};

pub mod fixed {

    use crate::internal::{FixedPositioning, FIXED};
    use crate::{FixedAngle, FixedLength, LatLongPos, NvectorPos, Spherical};

    pub fn destination<M: Spherical>(
        p0: LatLongPos<M>,
        bearing: FixedAngle,
        distance: FixedLength,
    ) -> LatLongPos<M> {
        let nv0: NvectorPos<M> = p0.into();
        let nv1 = super::destination(nv0, bearing, distance, FIXED);
        nv1.into()
    }

    pub fn distance<M: Spherical>(p1: LatLongPos<M>, p2: LatLongPos<M>) -> FixedLength {
        let nv1: NvectorPos<M> = p1.into();
        let nv2: NvectorPos<M> = p2.into();
        super::distance::<M, FixedAngle, FixedLength, FixedPositioning>(nv1, nv2, FIXED)
    }

    pub fn final_bearing<M: Spherical>(p1: LatLongPos<M>, p2: LatLongPos<M>) -> Option<FixedAngle> {
        let nv1: NvectorPos<M> = p1.into();
        let nv2: NvectorPos<M> = p2.into();
        super::final_bearing(nv1, nv2, FIXED)
    }

    pub fn initial_bearing<M: Spherical>(
        p1: LatLongPos<M>,
        p2: LatLongPos<M>,
    ) -> Option<FixedAngle> {
        let nv1: NvectorPos<M> = p1.into();
        let nv2: NvectorPos<M> = p2.into();
        super::initial_bearing(nv1, nv2, FIXED)
    }
}

pub mod f64 {

    use crate::internal::{F64Positioning, F64};
    use crate::{NvectorPos, Spherical};

    pub fn destination<M: Spherical>(
        p0: NvectorPos<M>,
        bearing_degrees: f64,
        distance_metres: f64,
    ) -> NvectorPos<M> {
        super::destination(p0, bearing_degrees, distance_metres, F64)
    }

    pub fn distance_metres<M: Spherical>(p1: NvectorPos<M>, p2: NvectorPos<M>) -> f64 {
        super::distance::<M, f64, f64, F64Positioning>(p1, p2, F64)
    }

    pub fn final_bearing_degrees<M: Spherical>(
        p1: NvectorPos<M>,
        p2: NvectorPos<M>,
    ) -> Option<f64> {
        super::final_bearing(p1, p2, F64)
    }

    pub fn initial_bearing_degrees<M: Spherical>(
        p1: NvectorPos<M>,
        p2: NvectorPos<M>,
    ) -> Option<f64> {
        super::initial_bearing(p1, p2, F64)
    }
}

fn destination<M: Spherical, A, L, P>(
    p0: NvectorPos<M>,
    bearing: A,
    distance: L,
    positioning: P,
) -> NvectorPos<M>
where
    A: Angle<Length = L>,
    L: Length,
    P: Positioning<Length = L>,
{
    if distance.is_zero() {
        p0
    } else {
        let v0 = p0.nvector();
        // east direction vector at p0
        let np = positioning.north_pole(p0.model());
        let ed = np.cross(v0).unit();
        // north direction vector at p0
        let nd = v0.cross(ed);
        // central angle
        let r = positioning.earth_radius(p0);
        let ca: A = A::central(distance, r);
        // unit vector in the direction of the azimuth
        let de = nd * bearing.cos() + ed * bearing.sin();
        let nv = v0 * ca.cos() + de * ca.sin();
        positioning.at_resolution(nv, p0.model())
    }
}

fn distance<M: Spherical, A, L, P>(p1: NvectorPos<M>, p2: NvectorPos<M>, positioning: P) -> L
where
    A: Angle<Length = L>,
    L: Length,
    P: Positioning<Length = L>,
{
    let a: A = signed_angle_between(p1.nvector(), p2.nvector(), None);
    a.arc_length(positioning.earth_radius(p1))
}

fn signed_angle_between<A, L>(v1: Vec3, v2: Vec3, vn: Option<Vec3>) -> A
where
    A: Angle<Length = L>,
{
    let sign = match vn {
        Some(n) => n.dot(v1.cross(v2)).signum(),
        None => 1.0,
    };
    let sin_o = sign * v1.cross(v2).norm();
    let cos_o = v1.dot(v2);
    A::atan2(sin_o, cos_o)
}

fn final_bearing<M: Spherical, A, L, P>(
    p1: NvectorPos<M>,
    p2: NvectorPos<M>,
    positioning: P,
) -> Option<A>
where
    A: Angle<Length = L>,
    L: Length,
    P: Positioning<Length = L>,
{
    let ib: Option<A> = initial_bearing(p2, p1, positioning);
    match ib {
        None => None,
        Some(b) => Some(b.normalise(A::half_circle())),
    }
}

fn initial_bearing<M: Spherical, A, L, P>(
    p1: NvectorPos<M>,
    p2: NvectorPos<M>,
    positioning: P,
) -> Option<A>
where
    A: Angle<Length = L>,
    L: Length,
    P: Positioning<Length = L>,
{
    let v1 = p1.nvector();
    let v2 = p2.nvector();
    if v1 == v2 {
        None
    } else {
        // great circle through p1 & p2
        let gc1 = v1.cross(v2);
        // great circle through p1 & north pole
        let gc2 = v1.cross(positioning.north_pole(p1.model()));
        let a: A = signed_angle_between(gc1, gc2, Some(v1));
        Some(a.normalise(A::full_circle()))
    }
}

#[cfg(test)]
mod fixed_tests {

    mod destination_test {

        use crate::great_circle;
        use crate::{FixedAngle, FixedLength, LatLongPos};

        #[test]
        fn returns_p0_if_distance_is_0() {
            let p0 = LatLongPos::s84(53.320556, -1.729722).unwrap();
            assert_eq!(
                p0,
                great_circle::fixed::destination(
                    p0,
                    FixedAngle::from_decimal_degrees(96.0217),
                    FixedLength::zero()
                )
            );
        }

        #[test]
        fn returns_position_along_great_circle_at_distance_and_bearing() {
            let p0 = LatLongPos::s84(53.320556, -1.729722).unwrap();
            let p1 = LatLongPos::s84(53.18826954833333, 0.13327449055555557).unwrap();
            assert_eq!(
                p1,
                great_circle::fixed::destination(
                    p0,
                    FixedAngle::from_decimal_degrees(96.0217),
                    FixedLength::from_metres(124800.0)
                )
            );
        }
    }

    mod distance_tests {

        use crate::great_circle;
        use crate::models::S84;
        use crate::{FixedLength, LatLongPos};

        #[test]
        fn returns_0_equal_positions() {
            let p = LatLongPos::s84(50.066389, -5.714722).unwrap();
            assert_eq!(FixedLength::zero(), great_circle::fixed::distance(p, p));
        }

        #[test]
        fn returns_distance_between_2_positions() {
            let p1 = LatLongPos::s84(50.066389, -5.714722).unwrap();
            let p2 = LatLongPos::s84(58.643889, -3.07).unwrap();
            assert_eq!(
                FixedLength::from_metres(968854.878007),
                great_circle::fixed::distance(p1, p2)
            );
        }

        #[test]
        fn handles_singularity_at_poles() {
            assert_eq!(
                FixedLength::from_kilometres(20015.114352233),
                great_circle::fixed::distance(
                    LatLongPos::north_pole(S84),
                    LatLongPos::south_pole(S84)
                )
            );
        }

        #[test]
        fn handles_discontinuity_at_date_line() {
            let p1 = LatLongPos::s84(50.066389, -179.999722).unwrap();
            let p2 = LatLongPos::s84(50.066389, 179.999722).unwrap();
            assert_eq!(
                FixedLength::from_metres(39.685092),
                great_circle::fixed::distance(p1, p2)
            );
        }
    }

    mod final_bearing_tests {

        use crate::great_circle;
        use crate::{Angle, FixedAngle, LatLongPos};

        #[test]
        fn returns_none_equal_positions() {
            let p = LatLongPos::s84(50.066389, -5.714722).unwrap();
            assert_eq!(None, great_circle::fixed::final_bearing(p, p));
            assert_eq!(
                None,
                great_circle::fixed::final_bearing(
                    p,
                    LatLongPos::s84(50.066389, -5.714722).unwrap()
                )
            );
        }

        #[test]
        fn returns_0_iso_longitude_going_north() {
            let p1 = LatLongPos::s84(50.066389, -5.714722).unwrap();
            let p2 = LatLongPos::s84(58.643889, -5.714722).unwrap();
            assert_eq!(
                Some(FixedAngle::zero()),
                great_circle::fixed::final_bearing(p1, p2)
            );
        }

        #[test]
        fn returns_180_iso_longitude_going_south() {
            let p1 = LatLongPos::s84(58.643889, -5.714722).unwrap();
            let p2 = LatLongPos::s84(50.066389, -5.714722).unwrap();
            assert_eq!(
                Some(FixedAngle::from_decimal_degrees(180.0)),
                great_circle::fixed::final_bearing(p1, p2)
            );
        }

        #[test]
        fn returns_90_at_equator_going_east() {
            let p1 = LatLongPos::s84(0.0, 0.0).unwrap();
            let p2 = LatLongPos::s84(0.0, 1.0).unwrap();
            assert_eq!(
                Some(FixedAngle::from_decimal_degrees(90.0)),
                great_circle::fixed::final_bearing(p1, p2)
            );
        }

        #[test]
        fn returns_270_at_equator_going_east() {
            let p1 = LatLongPos::s84(0.0, 1.0).unwrap();
            let p2 = LatLongPos::s84(0.0, 0.0).unwrap();
            assert_eq!(
                Some(FixedAngle::from_decimal_degrees(270.0)),
                great_circle::fixed::final_bearing(p1, p2)
            );
        }

        #[test]
        fn returns_final_bearing_compass_angle() {
            let p1 = LatLongPos::s84(50.066389, -5.714722).unwrap();
            let p2 = LatLongPos::s84(58.643889, -3.07).unwrap();
            assert_eq!(
                Some(FixedAngle::from_decimal_degrees(11.27520031611111)),
                great_circle::fixed::final_bearing(p1, p2)
            );
            assert_eq!(
                Some(FixedAngle::from_decimal_degrees(189.1198173275)),
                great_circle::fixed::final_bearing(p2, p1)
            );
            let p3 = LatLongPos::s84(-53.994722, -25.9875).unwrap();
            let p4 = LatLongPos::s84(54.0, 154.0).unwrap();
            assert_eq!(
                Some(FixedAngle::from_decimal_degrees(125.68508662305555)),
                great_circle::fixed::final_bearing(p3, p4)
            );
        }
    }

    mod initial_bearing_tests {

        use crate::great_circle;
        use crate::models::S84;
        use crate::{Angle, FixedAngle, LatLongPos};

        #[test]
        fn returns_none_equal_positions() {
            let p = LatLongPos::s84(50.066389, -179.999722).unwrap();
            assert_eq!(None, great_circle::fixed::initial_bearing(p, p));
            assert_eq!(
                None,
                great_circle::fixed::initial_bearing(
                    p,
                    LatLongPos::s84(50.066389, -179.999722).unwrap()
                )
            );
        }

        #[test]
        fn returns_0_iso_longitude_going_north() {
            let p1 = LatLongPos::s84(50.066389, -5.714722).unwrap();
            let p2 = LatLongPos::s84(58.643889, -5.714722).unwrap();
            assert_eq!(
                Some(FixedAngle::zero()),
                great_circle::fixed::initial_bearing(p1, p2)
            );
        }

        #[test]
        fn returns_180_iso_longitude_going_south() {
            let p1 = LatLongPos::s84(58.643889, -5.714722).unwrap();
            let p2 = LatLongPos::s84(50.066389, -5.714722).unwrap();
            assert_eq!(
                Some(FixedAngle::from_decimal_degrees(180.0)),
                great_circle::fixed::initial_bearing(p1, p2)
            );
        }

        #[test]
        fn returns_90_at_equator_going_east() {
            let p1 = LatLongPos::s84(0.0, 0.0).unwrap();
            let p2 = LatLongPos::s84(0.0, 1.0).unwrap();
            assert_eq!(
                Some(FixedAngle::from_decimal_degrees(90.0)),
                great_circle::fixed::initial_bearing(p1, p2)
            );
        }

        #[test]
        fn returns_270_at_equator_going_west() {
            let p1 = LatLongPos::s84(0.0, 1.0).unwrap();
            let p2 = LatLongPos::s84(0.0, 0.0).unwrap();
            assert_eq!(
                Some(FixedAngle::from_decimal_degrees(270.0)),
                great_circle::fixed::initial_bearing(p1, p2)
            );
        }

        #[test]
        fn returns_0_at_prime_meridian_going_north() {
            let p1 = LatLongPos::s84(50.0, 0.0).unwrap();
            let p2 = LatLongPos::s84(58.0, 0.0).unwrap();
            assert_eq!(
                Some(FixedAngle::zero()),
                great_circle::fixed::initial_bearing(p1, p2)
            );
        }

        #[test]
        fn returns_180_at_prime_meridian_going_south() {
            let p1 = LatLongPos::s84(58.0, 0.0).unwrap();
            let p2 = LatLongPos::s84(50.0, 0.0).unwrap();
            assert_eq!(
                Some(FixedAngle::from_decimal_degrees(180.0)),
                great_circle::fixed::initial_bearing(p1, p2)
            );
        }

        #[test]
        fn returns_0_at_date_line_going_north() {
            let p1 = LatLongPos::s84(50.0, 180.0).unwrap();
            let p2 = LatLongPos::s84(58.0, 180.0).unwrap();
            assert_eq!(
                Some(FixedAngle::zero()),
                great_circle::fixed::initial_bearing(p1, p2)
            );
        }

        #[test]
        fn returns_180_at_date_line_going_south() {
            let p1 = LatLongPos::s84(58.0, 180.0).unwrap();
            let p2 = LatLongPos::s84(50.0, 180.0).unwrap();
            assert_eq!(
                Some(FixedAngle::from_decimal_degrees(180.0)),
                great_circle::fixed::initial_bearing(p1, p2)
            );
        }

        #[test]
        fn returns_0_south_to_north_pole() {
            let p1 = LatLongPos::south_pole(S84);
            let p2 = LatLongPos::north_pole(S84);
            assert_eq!(
                Some(FixedAngle::zero()),
                great_circle::fixed::initial_bearing(p1, p2)
            );
        }

        #[test]
        fn returns_0_north_to_south_pole() {
            let p1 = LatLongPos::north_pole(S84);
            let p2 = LatLongPos::south_pole(S84);
            assert_eq!(
                Some(FixedAngle::zero()),
                great_circle::fixed::initial_bearing(p1, p2)
            );
        }

        #[test]
        fn returns_180_south_pole_to_date_line() {
            let p1 = LatLongPos::south_pole(S84);
            let p2 = LatLongPos::s84(50.0, 180.0).unwrap();
            assert_eq!(
                Some(FixedAngle::from_decimal_degrees(180.0)),
                great_circle::fixed::initial_bearing(p1, p2)
            );
        }

        #[test]
        fn returns_initial_bearing_compass_angle() {
            let p1 = LatLongPos::s84(50.066389, -5.714722).unwrap();
            let p2 = LatLongPos::s84(58.643889, -3.07).unwrap();
            assert_eq!(
                Some(FixedAngle::from_decimal_degrees(9.1198173275)),
                great_circle::fixed::initial_bearing(p1, p2)
            );
            assert_eq!(
                Some(FixedAngle::from_decimal_degrees(191.27520031611112)),
                great_circle::fixed::initial_bearing(p2, p1)
            );
        }
    }
}
