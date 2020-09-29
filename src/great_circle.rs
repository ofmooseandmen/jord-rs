use crate::internal::{LL, NV};
use crate::{Angle, LatLongPos, Length, NvectorPos, Spherical};

impl<S: Spherical> LatLongPos<S> {
    pub fn destination(&self, bearing: Angle, distance: Length) -> Self {
        let nv0: NvectorPos<S> = (*self).into();
        let nv1 = internal::destination(nv0, bearing.as_radians(), distance.as_metres(), LL);
        nv1.into()
    }

    pub fn distance_to(&self, other: Self) -> Length {
        let nv1: NvectorPos<S> = (*self).into();
        let nv2: NvectorPos<S> = other.into();
        Length::from_metres(internal::distance_metres(nv1, nv2, LL))
    }

    pub fn final_bearing_to(&self, other: Self) -> Option<Angle> {
        let nv1: NvectorPos<S> = (*self).into();
        let nv2: NvectorPos<S> = other.into();
        internal::final_bearing_radians(nv1, nv2, LL).map(|b| Angle::from_radians(b))
    }

    pub fn initial_bearing_to(&self, other: Self) -> Option<Angle> {
        let nv1: NvectorPos<S> = (*self).into();
        let nv2: NvectorPos<S> = other.into();
        internal::initial_bearing_radians(nv1, nv2, LL).map(|b| Angle::from_radians(b))
    }
}

impl<S: Spherical> NvectorPos<S> {
    pub fn destination(&self, bearing_degrees: f64, distance_metres: f64) -> NvectorPos<S> {
        internal::destination(*self, bearing_degrees.to_radians(), distance_metres, NV)
    }

    pub fn distance_metres_to(&self, other: Self) -> f64 {
        internal::distance_metres(*self, other, NV)
    }

    pub fn final_bearing_degrees_to(&self, other: Self) -> Option<f64> {
        internal::final_bearing_radians(*self, other, NV).map(|b| b.to_degrees())
    }

    pub fn initial_bearing_degrees_to(&self, other: Self) -> Option<f64> {
        internal::initial_bearing_radians(*self, other, NV).map(|b| b.to_degrees())
    }
}

mod internal {

    use crate::internal::modulo;
    use crate::internal::Rounding;
    use crate::{NvectorPos, Spherical, Surface, Vec3};
    use std::f64::consts::PI;

    pub fn destination<S: Spherical, R: Rounding>(
        p0: NvectorPos<S>,
        bearing_radians: f64,
        distance_metres: f64,
        rounding: R,
    ) -> NvectorPos<S> {
        if distance_metres == 0.0 {
            p0
        } else {
            let v0 = p0.nvector();
            // east direction vector at p0
            let np = rounding
                .round_pos(NvectorPos::north_pole(p0.model()))
                .nvector();
            let ed = np.cross(v0).unit();
            // north direction vector at p0
            let nd = v0.cross(ed);
            // central angle
            let r = p0.model().surface().mean_radius().as_metres();
            let ca = rounding.round_radians(distance_metres / r);
            // unit vector in the direction of the azimuth
            let de = nd * bearing_radians.cos() + ed * bearing_radians.sin();
            let nv = v0 * ca.cos() + de * ca.sin();
            rounding.round_pos(NvectorPos::new(nv, p0.model()))
        }
    }

    pub fn distance_metres<S: Spherical, R: Rounding>(
        p1: NvectorPos<S>,
        p2: NvectorPos<S>,
        rounding: R,
    ) -> f64 {
        let a = rounding.round_radians(signed_radians_between(p1.nvector(), p2.nvector(), None));
        a * p1.model().surface().mean_radius().as_metres()
    }

    pub fn final_bearing_radians<S: Spherical, R: Rounding>(
        p1: NvectorPos<S>,
        p2: NvectorPos<S>,
        rounding: R,
    ) -> Option<f64> {
        let ib: Option<f64> = initial_bearing_radians(p2, p1, rounding);
        match ib {
            None => None,
            Some(b) => Some(normalise_radians(b, PI)),
        }
    }

    pub fn initial_bearing_radians<S: Spherical, R: Rounding>(
        p1: NvectorPos<S>,
        p2: NvectorPos<S>,
        rounding: R,
    ) -> Option<f64> {
        let v1 = p1.nvector();
        let v2 = p2.nvector();
        if v1 == v2 {
            None
        } else {
            // great circle through p1 & p2
            let gc1 = v1.cross(v2);
            // great circle through p1 & north pole
            let np = rounding
                .round_pos(NvectorPos::north_pole(p1.model()))
                .nvector();
            let gc2 = v1.cross(np);
            let a = signed_radians_between(gc1, gc2, Some(v1));
            Some(normalise_radians(a, 2.0 * PI))
        }
    }

    fn signed_radians_between(v1: Vec3, v2: Vec3, vn: Option<Vec3>) -> f64 {
        let sign = match vn {
            Some(n) => n.dot(v1.cross(v2)).signum(),
            None => 1.0,
        };
        let sin_o = sign * v1.cross(v2).norm();
        let cos_o = v1.dot(v2);
        sin_o.atan2(cos_o)
    }

    fn normalise_radians(a: f64, b: f64) -> f64 {
        modulo(a + b, 2.0 * PI)
    }
}

#[cfg(test)]
mod fixed_tests {

    mod destination_test {

        use crate::{Angle, LatLongPos, Length};

        #[test]
        fn returns_p0_if_distance_is_0() {
            let p0 = LatLongPos::s84(53.320556, -1.729722).unwrap();
            assert_eq!(
                p0,
                p0.destination(Angle::from_decimal_degrees(96.0217), Length::zero())
            );
        }

        #[test]
        fn returns_position_along_great_circle_at_distance_and_bearing() {
            let p0 = LatLongPos::s84(53.320556, -1.729722).unwrap();
            let p1 = LatLongPos::s84(53.18826954833333, 0.13327449055555557).unwrap();
            assert_eq!(
                p1,
                p0.destination(
                    Angle::from_decimal_degrees(96.0217),
                    Length::from_metres(124800.0)
                )
            );
        }
    }

    mod distance_tests {

        use crate::models::S84;
        use crate::{LatLongPos, Length};

        #[test]
        fn returns_0_equal_positions() {
            let p = LatLongPos::s84(50.066389, -5.714722).unwrap();
            assert_eq!(Length::zero(), p.distance_to(p));
        }

        #[test]
        fn returns_distance_between_2_positions() {
            let p1 = LatLongPos::s84(50.066389, -5.714722).unwrap();
            let p2 = LatLongPos::s84(58.643889, -3.07).unwrap();
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
            let p1 = LatLongPos::s84(50.066389, -179.999722).unwrap();
            let p2 = LatLongPos::s84(50.066389, 179.999722).unwrap();
            assert_eq!(Length::from_metres(39.685092), p1.distance_to(p2));
        }
    }

    mod final_bearing_tests {

        use crate::{Angle, LatLongPos};

        #[test]
        fn returns_none_equal_positions() {
            let p = LatLongPos::s84(50.066389, -5.714722).unwrap();
            assert_eq!(None, p.final_bearing_to(p));
            assert_eq!(
                None,
                p.final_bearing_to(LatLongPos::s84(50.066389, -5.714722).unwrap())
            );
        }

        #[test]
        fn returns_0_iso_longitude_going_north() {
            let p1 = LatLongPos::s84(50.066389, -5.714722).unwrap();
            let p2 = LatLongPos::s84(58.643889, -5.714722).unwrap();
            assert_eq!(Some(Angle::zero()), p1.final_bearing_to(p2));
        }

        #[test]
        fn returns_180_iso_longitude_going_south() {
            let p1 = LatLongPos::s84(58.643889, -5.714722).unwrap();
            let p2 = LatLongPos::s84(50.066389, -5.714722).unwrap();
            assert_eq!(
                Some(Angle::from_decimal_degrees(180.0)),
                p1.final_bearing_to(p2)
            );
        }

        #[test]
        fn returns_90_at_equator_going_east() {
            let p1 = LatLongPos::s84(0.0, 0.0).unwrap();
            let p2 = LatLongPos::s84(0.0, 1.0).unwrap();
            assert_eq!(
                Some(Angle::from_decimal_degrees(90.0)),
                p1.final_bearing_to(p2)
            );
        }

        #[test]
        fn returns_270_at_equator_going_east() {
            let p1 = LatLongPos::s84(0.0, 1.0).unwrap();
            let p2 = LatLongPos::s84(0.0, 0.0).unwrap();
            assert_eq!(
                Some(Angle::from_decimal_degrees(270.0)),
                p1.final_bearing_to(p2)
            );
        }

        #[test]
        fn returns_final_bearing_compass_angle() {
            let p1 = LatLongPos::s84(50.066389, -5.714722).unwrap();
            let p2 = LatLongPos::s84(58.643889, -3.07).unwrap();
            assert_eq!(
                Some(Angle::from_decimal_degrees(11.27520031611111)),
                p1.final_bearing_to(p2)
            );
            assert_eq!(
                Some(Angle::from_decimal_degrees(189.1198173275)),
                p2.final_bearing_to(p1)
            );
            let p3 = LatLongPos::s84(-53.994722, -25.9875).unwrap();
            let p4 = LatLongPos::s84(54.0, 154.0).unwrap();
            assert_eq!(
                Some(Angle::from_decimal_degrees(125.68508662305555)),
                p3.final_bearing_to(p4)
            );
        }
    }

    mod initial_bearing_tests {

        use crate::models::S84;
        use crate::{Angle, LatLongPos};

        #[test]
        fn returns_none_equal_positions() {
            let p = LatLongPos::s84(50.066389, -179.999722).unwrap();
            assert_eq!(None, p.initial_bearing_to(p));
            assert_eq!(
                None,
                p.initial_bearing_to(LatLongPos::s84(50.066389, -179.999722).unwrap())
            );
        }

        #[test]
        fn returns_0_iso_longitude_going_north() {
            let p1 = LatLongPos::s84(50.066389, -5.714722).unwrap();
            let p2 = LatLongPos::s84(58.643889, -5.714722).unwrap();
            assert_eq!(Some(Angle::zero()), p1.initial_bearing_to(p2));
        }

        #[test]
        fn returns_180_iso_longitude_going_south() {
            let p1 = LatLongPos::s84(58.643889, -5.714722).unwrap();
            let p2 = LatLongPos::s84(50.066389, -5.714722).unwrap();
            assert_eq!(
                Some(Angle::from_decimal_degrees(180.0)),
                p1.initial_bearing_to(p2)
            );
        }

        #[test]
        fn returns_90_at_equator_going_east() {
            let p1 = LatLongPos::s84(0.0, 0.0).unwrap();
            let p2 = LatLongPos::s84(0.0, 1.0).unwrap();
            assert_eq!(
                Some(Angle::from_decimal_degrees(90.0)),
                p1.initial_bearing_to(p2)
            );
        }

        #[test]
        fn returns_270_at_equator_going_west() {
            let p1 = LatLongPos::s84(0.0, 1.0).unwrap();
            let p2 = LatLongPos::s84(0.0, 0.0).unwrap();
            assert_eq!(
                Some(Angle::from_decimal_degrees(270.0)),
                p1.initial_bearing_to(p2)
            );
        }

        #[test]
        fn returns_0_at_prime_meridian_going_north() {
            let p1 = LatLongPos::s84(50.0, 0.0).unwrap();
            let p2 = LatLongPos::s84(58.0, 0.0).unwrap();
            assert_eq!(Some(Angle::zero()), p1.initial_bearing_to(p2));
        }

        #[test]
        fn returns_180_at_prime_meridian_going_south() {
            let p1 = LatLongPos::s84(58.0, 0.0).unwrap();
            let p2 = LatLongPos::s84(50.0, 0.0).unwrap();
            assert_eq!(
                Some(Angle::from_decimal_degrees(180.0)),
                p1.initial_bearing_to(p2)
            );
        }

        #[test]
        fn returns_0_at_date_line_going_north() {
            let p1 = LatLongPos::s84(50.0, 180.0).unwrap();
            let p2 = LatLongPos::s84(58.0, 180.0).unwrap();
            assert_eq!(Some(Angle::zero()), p1.initial_bearing_to(p2));
        }

        #[test]
        fn returns_180_at_date_line_going_south() {
            let p1 = LatLongPos::s84(58.0, 180.0).unwrap();
            let p2 = LatLongPos::s84(50.0, 180.0).unwrap();
            assert_eq!(
                Some(Angle::from_decimal_degrees(180.0)),
                p1.initial_bearing_to(p2)
            );
        }

        #[test]
        fn returns_0_south_to_north_pole() {
            let p1 = LatLongPos::south_pole(S84);
            let p2 = LatLongPos::north_pole(S84);
            assert_eq!(Some(Angle::zero()), p1.initial_bearing_to(p2));
        }

        #[test]
        fn returns_0_north_to_south_pole() {
            let p1 = LatLongPos::north_pole(S84);
            let p2 = LatLongPos::south_pole(S84);
            assert_eq!(Some(Angle::zero()), p1.initial_bearing_to(p2));
        }

        #[test]
        fn returns_180_south_pole_to_date_line() {
            let p1 = LatLongPos::south_pole(S84);
            let p2 = LatLongPos::s84(50.0, 180.0).unwrap();
            assert_eq!(
                Some(Angle::from_decimal_degrees(180.0)),
                p1.initial_bearing_to(p2)
            );
        }

        #[test]
        fn returns_initial_bearing_compass_angle() {
            let p1 = LatLongPos::s84(50.066389, -5.714722).unwrap();
            let p2 = LatLongPos::s84(58.643889, -3.07).unwrap();
            assert_eq!(
                Some(Angle::from_decimal_degrees(9.1198173275)),
                p1.initial_bearing_to(p2)
            );
            assert_eq!(
                Some(Angle::from_decimal_degrees(191.27520031611112)),
                p2.initial_bearing_to(p1)
            );
        }
    }
}
