use std::marker::PhantomData;

use crate::{Angle, Error, LatLongPos, Length, NvectorPos, Rounding, Spherical, Vec3};

// FIXME Display
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GreatCircle<P> {
    position_type: PhantomData<P>,
    normal: Vec3,
}

impl<S: Spherical> GreatCircle<LatLongPos<S>> {
    pub fn from_lat_longs(
        p1: LatLongPos<S>,
        p2: LatLongPos<S>,
    ) -> Result<GreatCircle<LatLongPos<S>>, Error> {
        private::arc_normal(p1.to_nvector(), p2.to_nvector()).map(|n| GreatCircle {
            position_type: PhantomData,
            normal: n,
        })
    }

    pub fn from_lat_long_bearing(pos: LatLongPos<S>, bearing: Angle) -> GreatCircle<LatLongPos<S>> {
        let normal = private::arc_normal_bearing_radians(
            pos.to_nvector(),
            bearing.as_radians(),
            Rounding::Angle,
        );
        GreatCircle {
            position_type: PhantomData,
            normal,
        }
    }
}

impl<S: Spherical> GreatCircle<NvectorPos<S>> {
    pub fn from_nvectors(
        p1: NvectorPos<S>,
        p2: NvectorPos<S>,
    ) -> Result<GreatCircle<NvectorPos<S>>, Error> {
        private::arc_normal(p1.nvector(), p2.nvector()).map(|n| GreatCircle {
            position_type: PhantomData,
            normal: n,
        })
    }

    pub fn from_nvector_bearing_degrees(
        pos: NvectorPos<S>,
        bearing_degrees: f64,
    ) -> GreatCircle<NvectorPos<S>> {
        let normal = private::arc_normal_bearing_radians(
            pos.nvector(),
            bearing_degrees.to_radians(),
            Rounding::None,
        );
        GreatCircle {
            position_type: PhantomData,
            normal,
        }
    }
}

impl<S: Spherical> LatLongPos<S> {
    pub fn cross_track_distance(&self, gc: GreatCircle<LatLongPos<S>>) -> Length {
        let nv: NvectorPos<S> = (*self).into();
        Length::from_metres(private::cross_track_distance_metres(
            nv,
            gc.normal,
            Rounding::Angle,
        ))
    }

    pub fn destination_pos(&self, bearing: Angle, distance: Length) -> Self {
        let nv0: NvectorPos<S> = (*self).into();
        let nv1 = private::destination_pos(
            nv0,
            bearing.as_radians(),
            distance.as_metres(),
            Rounding::Angle,
        );
        nv1.into()
    }

    pub fn distance_to(&self, to: Self) -> Length {
        let nv1: NvectorPos<S> = (*self).into();
        let nv2: NvectorPos<S> = to.into();
        Length::from_metres(private::distance_metres(nv1, nv2, Rounding::Angle))
    }

    pub fn final_bearing_to(&self, to: Self) -> Result<Angle, Error> {
        private::final_bearing_radians((*self).to_nvector(), to.to_nvector(), Rounding::Angle)
            .map(Angle::from_radians)
    }

    pub fn initial_bearing_to(&self, to: Self) -> Result<Angle, Error> {
        private::initial_bearing_radians((*self).to_nvector(), to.to_nvector(), Rounding::Angle)
            .map(Angle::from_radians)
    }

    pub fn intermediate_pos_to(&self, to: Self, f: f64) -> Result<Self, Error> {
        private::intermediate_pos((*self).to_nvector(), to.to_nvector(), f)
            .map(|v| LatLongPos::from_nvector(v, (*self).model()))
    }

    pub fn turn(&self, from: Self, to: Self) -> Result<Angle, Error> {
        private::turn_radians(from.to_nvector(), (*self).to_nvector(), to.to_nvector())
            .map(Angle::from_radians)
    }
}

impl<S: Spherical> NvectorPos<S> {
    pub fn cross_track_distance_metres(&self, gc: GreatCircle<LatLongPos<S>>) -> f64 {
        private::cross_track_distance_metres(*self, gc.normal, Rounding::None)
    }

    pub fn destination_pos(&self, bearing_degrees: f64, distance_metres: f64) -> NvectorPos<S> {
        private::destination_pos(
            *self,
            bearing_degrees.to_radians(),
            distance_metres,
            Rounding::None,
        )
    }

    pub fn distance_metres_to(&self, to: Self) -> f64 {
        private::distance_metres(*self, to, Rounding::None)
    }

    pub fn final_bearing_degrees_to(&self, to: Self) -> Result<f64, Error> {
        private::final_bearing_radians((*self).nvector(), to.nvector(), Rounding::None)
            .map(|b| b.to_degrees())
    }

    pub fn initial_bearing_degrees_to(&self, to: Self) -> Result<f64, Error> {
        private::initial_bearing_radians((*self).nvector(), to.nvector(), Rounding::None)
            .map(|b| b.to_degrees())
    }

    pub fn intermediate_pos_to(&self, to: Self, f: f64) -> Result<Self, Error> {
        private::intermediate_pos((*self).nvector(), to.nvector(), f)
            .map(|v| NvectorPos::new(v, (*self).model()))
    }

    pub fn turn_degrees(&self, from: Self, to: Self) -> Result<f64, Error> {
        private::turn_radians(from.nvector(), (*self).nvector(), to.nvector())
            .map(|b| b.to_degrees())
    }
}

mod private {

    use crate::geodetic::antipode;
    use crate::{Error, NvectorPos, Rounding, Spherical, Surface, Vec3};
    use std::f64::consts::PI;

    pub(crate) fn arc_normal(v1: Vec3, v2: Vec3) -> Result<Vec3, Error> {
        if v1 == v2 {
            Err(Error::CoincidentalPositions)
        } else if antipode(v1) == v2 {
            Err(Error::AntipodalPositions)
        } else {
            Ok(v1.cross(v2))
        }
    }

    pub(crate) fn arc_normal_bearing_radians(
        v: Vec3,
        bearing_radians: f64,
        rounding: Rounding,
    ) -> Vec3 {
        // easting
        let e = rounding.north_pole().cross(v);
        // northing
        let n = v.cross(e);
        let se = e * (bearing_radians.cos() / e.norm());
        let sn = n * (bearing_radians.sin() / n.norm());
        sn - se
    }

    pub(crate) fn cross_track_distance_metres<S: Spherical>(
        p: NvectorPos<S>,
        n: Vec3,
        rounding: Rounding,
    ) -> f64 {
        let a = rounding.round_radians(signed_radians_between(n, p.nvector(), None) - (PI / 2.0));
        a * p.model().surface().mean_radius().as_metres()
    }

    pub(crate) fn destination_pos<S: Spherical>(
        p0: NvectorPos<S>,
        bearing_radians: f64,
        distance_metres: f64,
        rounding: Rounding,
    ) -> NvectorPos<S> {
        if distance_metres == 0.0 {
            p0
        } else {
            let v0 = p0.nvector();
            // east direction vector at p0
            let np = rounding.north_pole();
            let ed = np.cross(v0).unit();
            // north direction vector at p0
            let nd = v0.cross(ed);
            // central angle
            let r = p0.model().surface().mean_radius().as_metres();
            let ca = rounding.round_radians(distance_metres / r);
            // unit vector in the direction of the azimuth
            let de = nd * bearing_radians.cos() + ed * bearing_radians.sin();
            let nv = v0 * ca.cos() + de * ca.sin();
            NvectorPos::new(nv, p0.model())
        }
    }

    pub(crate) fn distance_metres<S: Spherical>(
        p1: NvectorPos<S>,
        p2: NvectorPos<S>,
        rounding: Rounding,
    ) -> f64 {
        let a = rounding.round_radians(signed_radians_between(p1.nvector(), p2.nvector(), None));
        a * p1.model().surface().mean_radius().as_metres()
    }

    pub(crate) fn final_bearing_radians(
        v1: Vec3,
        v2: Vec3,
        rounding: Rounding,
    ) -> Result<f64, Error> {
        initial_bearing_radians(v2, v1, rounding).map(|b| normalise_radians(b, PI))
    }

    pub(crate) fn initial_bearing_radians(
        v1: Vec3,
        v2: Vec3,
        rounding: Rounding,
    ) -> Result<f64, Error> {
        if v1 == v2 {
            Err(Error::CoincidentalPositions)
        } else {
            // great circle through p1 & p2
            let gc1 = v1.cross(v2);
            // great circle through p1 & north pole
            let np = rounding.north_pole();
            let gc2 = v1.cross(np);
            let a = signed_radians_between(gc1, gc2, Some(v1));
            Ok(normalise_radians(a, 2.0 * PI))
        }
    }

    pub(crate) fn intermediate_pos(v1: Vec3, v2: Vec3, f: f64) -> Result<Vec3, Error> {
        if f < 0.0 || f > 1.0 {
            Err(Error::OutOfRange)
        } else {
            Ok((v1 + f * (v2 - v1)).unit())
        }
    }

    pub(crate) fn turn_radians(from: Vec3, at: Vec3, to: Vec3) -> Result<f64, Error> {
        let nfa = arc_normal(from, at)?;
        let nat = arc_normal(at, to)?;
        Ok(signed_radians_between(nfa.unit(), nat.unit(), Some(at)))
    }

    fn signed_radians_between(v1: Vec3, v2: Vec3, vn: Option<Vec3>) -> f64 {
        let sign = vn.map_or(1.0, |n| n.dot(v1.cross(v2)).signum());
        let sin_o = sign * v1.cross(v2).norm();
        let cos_o = v1.dot(v2);
        sin_o.atan2(cos_o)
    }

    fn normalise_radians(a: f64, b: f64) -> f64 {
        (a + b) % (2.0 * PI)
    }
}

#[cfg(test)]
mod lat_long_test {

    mod cross_track_distance_test {

        use crate::{GreatCircle, LatLongPos, Length};

        #[test]
        fn returns_negative_length_if_left() {
            let p = LatLongPos::from_s84(53.2611, -0.7972);
            let gcp1 = LatLongPos::from_s84(53.3206, -1.7297);
            let gcp2 = LatLongPos::from_s84(53.1887, 0.1334);
            let gc1 = GreatCircle::from_lat_longs(gcp1, gcp2).unwrap();
            let expected = Length::from_metres(-307.549992);
            assert_eq!(expected, p.cross_track_distance(gc1));

            // same result with great circle from position and bearing
            let gc2 =
                GreatCircle::from_lat_long_bearing(gcp1, gcp1.initial_bearing_to(gcp2).unwrap());
            assert_eq!(expected, p.cross_track_distance(gc2));
        }

        #[test]
        fn returns_positive_length_if_right() {
            let p = LatLongPos::from_s84(53.2611, -0.7972).antipode();
            let gcp1 = LatLongPos::from_s84(53.3206, -1.7297);
            let gcp2 = LatLongPos::from_s84(53.1887, 0.1334);
            let gc1 = GreatCircle::from_lat_longs(gcp1, gcp2).unwrap();
            let expected = Length::from_metres(307.549992);
            assert_eq!(expected, p.cross_track_distance(gc1));

            // same result with great circle from position and bearing
            let gc2 =
                GreatCircle::from_lat_long_bearing(gcp1, gcp1.initial_bearing_to(gcp2).unwrap());
            assert_eq!(expected, p.cross_track_distance(gc2));
        }

        #[test]
        fn zero() {
            let gc1 = LatLongPos::from_s84(53.3206, -1.7297);
            let gc2 = LatLongPos::from_s84(53.1887, 0.1334);
            let gc = GreatCircle::from_lat_longs(gc1, gc2).unwrap();
            for f in 0..100 {
                let p = gc1.intermediate_pos_to(gc2, (f as f64) / 100.0).unwrap();
                assert_eq!(Length::zero(), p.cross_track_distance(gc));
            }
        }
    }

    mod destination_pos_test {

        use crate::{Angle, LatLongPos, Length};

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

    mod distance_test {

        use crate::models::S84;
        use crate::{LatLongPos, Length};

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

    mod final_bearing_test {

        use crate::{Angle, Error, LatLongPos};

        #[test]
        fn returns_err_equal_positions() {
            let p = LatLongPos::from_s84(50.066389, -5.714722);
            assert_eq!(Err(Error::CoincidentalPositions), p.final_bearing_to(p));
            assert_eq!(
                Err(Error::CoincidentalPositions),
                p.final_bearing_to(LatLongPos::from_s84(50.066389, -5.714722))
            );
        }

        #[test]
        fn returns_0_iso_longitude_going_north() {
            let p1 = LatLongPos::from_s84(50.066389, -5.714722);
            let p2 = LatLongPos::from_s84(58.643889, -5.714722);
            assert_eq!(Ok(Angle::zero()), p1.final_bearing_to(p2));
        }

        #[test]
        fn returns_180_iso_longitude_going_south() {
            let p1 = LatLongPos::from_s84(58.643889, -5.714722);
            let p2 = LatLongPos::from_s84(50.066389, -5.714722);
            assert_eq!(
                Ok(Angle::from_decimal_degrees(180.0)),
                p1.final_bearing_to(p2)
            );
        }

        #[test]
        fn returns_90_at_equator_going_east() {
            let p1 = LatLongPos::from_s84(0.0, 0.0);
            let p2 = LatLongPos::from_s84(0.0, 1.0);
            assert_eq!(
                Ok(Angle::from_decimal_degrees(90.0)),
                p1.final_bearing_to(p2)
            );
        }

        #[test]
        fn returns_270_at_equator_going_east() {
            let p1 = LatLongPos::from_s84(0.0, 1.0);
            let p2 = LatLongPos::from_s84(0.0, 0.0);
            assert_eq!(
                Ok(Angle::from_decimal_degrees(270.0)),
                p1.final_bearing_to(p2)
            );
        }

        #[test]
        fn returns_final_bearing_compass_angle() {
            let p1 = LatLongPos::from_s84(50.066389, -5.714722);
            let p2 = LatLongPos::from_s84(58.643889, -3.07);
            assert_eq!(
                Ok(Angle::from_decimal_degrees(11.27520031611111)),
                p1.final_bearing_to(p2)
            );
            assert_eq!(
                Ok(Angle::from_decimal_degrees(189.1198173275)),
                p2.final_bearing_to(p1)
            );
            let p3 = LatLongPos::from_s84(-53.994722, -25.9875);
            let p4 = LatLongPos::from_s84(54.0, 154.0);
            assert_eq!(
                Ok(Angle::from_decimal_degrees(125.68508662305555)),
                p3.final_bearing_to(p4)
            );
        }
    }

    mod initial_bearing_test {

        use crate::models::S84;
        use crate::{Angle, Error, LatLongPos};

        #[test]
        fn returns_err_equal_positions() {
            let p = LatLongPos::from_s84(50.066389, -179.999722);
            assert_eq!(Err(Error::CoincidentalPositions), p.initial_bearing_to(p));
            assert_eq!(
                Err(Error::CoincidentalPositions),
                p.initial_bearing_to(LatLongPos::from_s84(50.066389, -179.999722))
            );
        }

        #[test]
        fn returns_0_iso_longitude_going_north() {
            let p1 = LatLongPos::from_s84(50.066389, -5.714722);
            let p2 = LatLongPos::from_s84(58.643889, -5.714722);
            assert_eq!(Ok(Angle::zero()), p1.initial_bearing_to(p2));
        }

        #[test]
        fn returns_180_iso_longitude_going_south() {
            let p1 = LatLongPos::from_s84(58.643889, -5.714722);
            let p2 = LatLongPos::from_s84(50.066389, -5.714722);
            assert_eq!(
                Ok(Angle::from_decimal_degrees(180.0)),
                p1.initial_bearing_to(p2)
            );
        }

        #[test]
        fn returns_90_at_equator_going_east() {
            let p1 = LatLongPos::from_s84(0.0, 0.0);
            let p2 = LatLongPos::from_s84(0.0, 1.0);
            assert_eq!(
                Ok(Angle::from_decimal_degrees(90.0)),
                p1.initial_bearing_to(p2)
            );
        }

        #[test]
        fn returns_270_at_equator_going_west() {
            let p1 = LatLongPos::from_s84(0.0, 1.0);
            let p2 = LatLongPos::from_s84(0.0, 0.0);
            assert_eq!(
                Ok(Angle::from_decimal_degrees(270.0)),
                p1.initial_bearing_to(p2)
            );
        }

        #[test]
        fn returns_0_at_prime_meridian_going_north() {
            let p1 = LatLongPos::from_s84(50.0, 0.0);
            let p2 = LatLongPos::from_s84(58.0, 0.0);
            assert_eq!(Ok(Angle::zero()), p1.initial_bearing_to(p2));
        }

        #[test]
        fn returns_180_at_prime_meridian_going_south() {
            let p1 = LatLongPos::from_s84(58.0, 0.0);
            let p2 = LatLongPos::from_s84(50.0, 0.0);
            assert_eq!(
                Ok(Angle::from_decimal_degrees(180.0)),
                p1.initial_bearing_to(p2)
            );
        }

        #[test]
        fn returns_0_at_date_line_going_north() {
            let p1 = LatLongPos::from_s84(50.0, 180.0);
            let p2 = LatLongPos::from_s84(58.0, 180.0);
            assert_eq!(Ok(Angle::zero()), p1.initial_bearing_to(p2));
        }

        #[test]
        fn returns_180_at_date_line_going_south() {
            let p1 = LatLongPos::from_s84(58.0, 180.0);
            let p2 = LatLongPos::from_s84(50.0, 180.0);
            assert_eq!(
                Ok(Angle::from_decimal_degrees(180.0)),
                p1.initial_bearing_to(p2)
            );
        }

        #[test]
        fn returns_0_south_to_north_pole() {
            let p1 = LatLongPos::south_pole(S84);
            let p2 = LatLongPos::north_pole(S84);
            assert_eq!(Ok(Angle::zero()), p1.initial_bearing_to(p2));
        }

        #[test]
        fn returns_0_north_to_south_pole() {
            let p1 = LatLongPos::north_pole(S84);
            let p2 = LatLongPos::south_pole(S84);
            assert_eq!(Ok(Angle::zero()), p1.initial_bearing_to(p2));
        }

        #[test]
        fn returns_180_south_pole_to_date_line() {
            let p1 = LatLongPos::south_pole(S84);
            let p2 = LatLongPos::from_s84(50.0, 180.0);
            assert_eq!(
                Ok(Angle::from_decimal_degrees(180.0)),
                p1.initial_bearing_to(p2)
            );
        }

        #[test]
        fn returns_initial_bearing_compass_angle() {
            let p1 = LatLongPos::from_s84(50.066389, -5.714722);
            let p2 = LatLongPos::from_s84(58.643889, -3.07);
            assert_eq!(
                Ok(Angle::from_decimal_degrees(9.1198173275)),
                p1.initial_bearing_to(p2)
            );
            assert_eq!(
                Ok(Angle::from_decimal_degrees(191.27520031611112)),
                p2.initial_bearing_to(p1)
            );
        }
    }

    mod intermediate_pos_to_test {

        use crate::{Error, LatLongPos};

        #[test]
        fn returns_err_fraction() {
            let p1 = LatLongPos::from_s84(44.0, 44.0);
            let p2 = LatLongPos::from_s84(46.0, 46.0);
            assert_eq!(Err(Error::OutOfRange), p1.intermediate_pos_to(p2, -0.9));
            assert_eq!(Err(Error::OutOfRange), p1.intermediate_pos_to(p2, 1.1));
        }

        #[test]
        fn returns_p1() {
            let p1 = LatLongPos::from_s84(44.0, 44.0);
            let p2 = LatLongPos::from_s84(46.0, 46.0);
            assert_eq!(Ok(p1), p1.intermediate_pos_to(p2, 0.0));
        }

        #[test]
        fn returns_p2() {
            let p1 = LatLongPos::from_s84(44.0, 44.0);
            let p2 = LatLongPos::from_s84(46.0, 46.0);
            assert_eq!(Ok(p2), p1.intermediate_pos_to(p2, 1.0));
        }

        #[test]
        fn returns_pos() {
            let p1 = LatLongPos::from_s84(53.479444, -2.245278);
            let p2 = LatLongPos::from_s84(55.605833, 13.035833);
            let pe = LatLongPos::from_s84(54.78355703138889, 5.194985318055555);
            assert_eq!(Ok(pe), p1.intermediate_pos_to(p2, 0.5));
        }
    }

    mod turn_test {

        use crate::{Angle, Error, LatLongPos};

        #[test]
        fn positive_turn() {
            assert_eq!(
                Ok(Angle::from_decimal_degrees(18.192705871944444)),
                LatLongPos::from_s84(45.0, 0.0).turn(
                    LatLongPos::from_s84(0.0, 0.0),
                    LatLongPos::from_s84(60.0, -10.0)
                )
            );
        }

        #[test]
        fn negative_turn() {
            assert_eq!(
                Ok(Angle::from_decimal_degrees(-18.192705871944444)),
                LatLongPos::from_s84(45.0, 0.0).turn(
                    LatLongPos::from_s84(0.0, 0.0),
                    LatLongPos::from_s84(60.0, 10.0)
                )
            );
        }

        #[test]
        fn zero_turn() {
            assert_eq!(
                Ok(Angle::zero()),
                LatLongPos::from_s84(45.0, 0.0).turn(
                    LatLongPos::from_s84(0.0, 0.0),
                    LatLongPos::from_s84(90.0, 0.0)
                )
            );
        }

        #[test]
        fn half_turn() {
            let a = LatLongPos::from_s84(45.0, 63.0);
            let b = LatLongPos::from_s84(-54.0, -89.0);
            assert_eq!(Ok(Angle::from_decimal_degrees(180.0)), a.turn(b, b));
            assert_eq!(Ok(Angle::from_decimal_degrees(180.0)), b.turn(a, a));
        }

        #[test]
        fn no_turn() {
            let a = LatLongPos::from_s84(45.0, 63.0);
            let b = LatLongPos::from_s84(-54.0, -89.0);
            assert_eq!(Err(Error::CoincidentalPositions), a.turn(a, a));
            assert_eq!(Err(Error::CoincidentalPositions), a.turn(a, b));
            assert_eq!(Err(Error::CoincidentalPositions), a.turn(b, a));
            assert_eq!(Err(Error::CoincidentalPositions), b.turn(a, b));
            assert_eq!(Err(Error::CoincidentalPositions), b.turn(b, a));
            assert_eq!(Err(Error::CoincidentalPositions), b.turn(b, b));
        }
    }
}
