use crate::{Angle, Error, LatLongPos, Length, NvectorPos, Rounding, Spherical, Vec3};

// FIXME Display
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GreatCircle<P> {
    position: P,
    normal: Vec3,
}

impl<S: Spherical> GreatCircle<LatLongPos<S>> {
    pub fn from_lat_longs(
        p1: LatLongPos<S>,
        p2: LatLongPos<S>,
    ) -> Result<GreatCircle<LatLongPos<S>>, Error> {
        private::arc_normal(p1.to_nvector(), p2.to_nvector()).map(|n| GreatCircle {
            position: p1,
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
            position: pos,
            normal,
        }
    }

    pub fn intersections_with(&self, other: Self) -> Result<(LatLongPos<S>, LatLongPos<S>), Error> {
        let (i1, i2) = private::intersections((*self).normal, other.normal)?;
        Ok((
            LatLongPos::from_nvector(i1, (*self).position.model()),
            LatLongPos::from_nvector(i2, (*self).position.model()),
        ))
    }
}

impl<S: Spherical> GreatCircle<NvectorPos<S>> {
    pub fn from_nvectors(
        p1: NvectorPos<S>,
        p2: NvectorPos<S>,
    ) -> Result<GreatCircle<NvectorPos<S>>, Error> {
        private::arc_normal(p1.nvector(), p2.nvector()).map(|n| GreatCircle {
            position: p1,
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
            position: pos,
            normal,
        }
    }

    pub fn intersections_with(&self, other: Self) -> Result<(NvectorPos<S>, NvectorPos<S>), Error> {
        let (i1, i2) = private::intersections((*self).normal, other.normal)?;
        Ok((
            NvectorPos::new(i1, (*self).position.model()),
            NvectorPos::new(i2, (*self).position.model()),
        ))
    }
}

impl<S: Spherical> LatLongPos<S> {
    pub fn cross_track_distance_to(&self, gc: GreatCircle<LatLongPos<S>>) -> Length {
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
    pub fn cross_track_distance_metres_to(&self, gc: GreatCircle<LatLongPos<S>>) -> f64 {
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

    pub(crate) fn intersections(n1: Vec3, n2: Vec3) -> Result<(Vec3, Vec3), Error> {
        let i1 = n1.cross(n2);
        if i1 == Vec3::zero() {
            // same or opposite great circle
            Err(Error::CoincidentalGreatCircles)
        } else {
            Ok((i1, antipode(i1)))
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
