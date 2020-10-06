use crate::{Angle, Error, LatLongPos, Length, NvectorPos, Rounding, Spherical, Surface, Vec3};

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
        let i = private::gc_intersection::<LatLongPos<S>>(*self, other)?;
        let lli = LatLongPos::from_nvector(i, (*self).position.model());
        Ok((lli, lli.antipode()))
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
        let i = private::gc_intersection::<NvectorPos<S>>(*self, other)?;
        let nvi = NvectorPos::new(i, (*self).position.model());
        Ok((nvi, nvi.antipode()))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MinorArc<P> {
    start_pos: P,
    end_pos: P,
    normal: Vec3,
}

impl<P: Copy> MinorArc<P> {
    pub fn start_pos(&self) -> P {
        self.start_pos
    }

    pub fn end_pos(&self) -> P {
        self.end_pos
    }
}

impl<S: Spherical> MinorArc<LatLongPos<S>> {
    pub fn from_lat_longs(
        start_pos: LatLongPos<S>,
        end_pos: LatLongPos<S>,
    ) -> Result<MinorArc<LatLongPos<S>>, Error> {
        private::arc_normal(start_pos.to_nvector(), end_pos.to_nvector()).map(|n| MinorArc {
            start_pos,
            end_pos,
            normal: n,
        })
    }

    pub fn intersection_with(&self, other: Self) -> Result<LatLongPos<S>, Error> {
        let ma = MinorArc {
            start_pos: self.start_pos.to_nvector(),
            end_pos: self.end_pos.to_nvector(),
            normal: self.normal,
        };
        let mb = MinorArc {
            start_pos: other.start_pos.to_nvector(),
            end_pos: other.end_pos.to_nvector(),
            normal: other.normal,
        };
        let i = private::intersection(ma, mb, Rounding::Angle)?;
        Ok(LatLongPos::from_nvector(i, self.start_pos.model()))
    }
}

impl<S: Spherical> MinorArc<NvectorPos<S>> {
    pub fn from_nvectors(
        start_pos: NvectorPos<S>,
        end_pos: NvectorPos<S>,
    ) -> Result<MinorArc<NvectorPos<S>>, Error> {
        private::arc_normal(start_pos.nvector(), end_pos.nvector()).map(|n| MinorArc {
            start_pos,
            end_pos,
            normal: n,
        })
    }

    pub fn intersection_with(&self, other: Self) -> Result<NvectorPos<S>, Error> {
        let ma = MinorArc {
            start_pos: self.start_pos.nvector(),
            end_pos: self.end_pos.nvector(),
            normal: self.normal,
        };
        let mb = MinorArc {
            start_pos: other.start_pos.nvector(),
            end_pos: other.end_pos.nvector(),
            normal: other.normal,
        };
        let i = private::intersection(ma, mb, Rounding::None)?;
        Ok(NvectorPos::new(i, self.start_pos.model()))
    }
}

impl<S: Spherical> LatLongPos<S> {
    pub fn from_mean(ps: &[LatLongPos<S>]) -> Result<Self, Error> {
        let m = private::mean(
            ps.iter().map(LatLongPos::to_nvector).collect(),
            Rounding::Angle,
        )?;
        // unwrap is safe because mean returns Err if ps is empty
        Ok(LatLongPos::from_nvector(m, ps.first().unwrap().model()))
    }

    pub fn along_track_distance_to(&self, ma: MinorArc<LatLongPos<S>>) -> Length {
        let metres = private::along_track_distance_metres(
            self.to_nvector(),
            ma.start_pos.to_nvector(),
            ma.normal,
            self.earth_radius_metres(),
            Rounding::Angle,
        );
        Length::from_metres(metres)
    }

    pub fn cross_track_distance_to(&self, gc: GreatCircle<LatLongPos<S>>) -> Length {
        let metres = private::cross_track_distance_metres(
            self.to_nvector(),
            gc.normal,
            self.earth_radius_metres(),
            Rounding::Angle,
        );
        Length::from_metres(metres)
    }

    pub fn destination_pos(&self, bearing: Angle, distance: Length) -> Self {
        let dest = private::destination_pos(
            self.to_nvector(),
            bearing.as_radians(),
            distance.as_metres(),
            self.earth_radius_metres(),
            Rounding::Angle,
        );
        LatLongPos::from_nvector(dest, self.model())
    }

    pub fn distance_to(&self, to: Self) -> Length {
        let metres = private::distance_metres(
            self.to_nvector(),
            to.to_nvector(),
            self.earth_radius_metres(),
            Rounding::Angle,
        );
        Length::from_metres(metres)
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

    fn earth_radius_metres(&self) -> f64 {
        self.model().surface().mean_radius().as_metres()
    }
}

impl<S: Spherical> NvectorPos<S> {
    pub fn from_mean(ps: &[NvectorPos<S>]) -> Result<Self, Error> {
        let m = private::mean(ps.iter().map(NvectorPos::nvector).collect(), Rounding::None)?;
        // unwrap is safe because mean returns Err if ps is empty
        Ok(NvectorPos::new(m, ps.first().unwrap().model()))
    }

    pub fn along_track_distance_metres_to(&self, ma: MinorArc<NvectorPos<S>>) -> f64 {
        private::along_track_distance_metres(
            self.nvector(),
            ma.start_pos.nvector(),
            ma.normal,
            self.earth_radius_metres(),
            Rounding::None,
        )
    }

    pub fn cross_track_distance_metres_to(&self, gc: GreatCircle<NvectorPos<S>>) -> f64 {
        private::cross_track_distance_metres(
            self.nvector(),
            gc.normal,
            self.earth_radius_metres(),
            Rounding::None,
        )
    }

    pub fn destination_pos(&self, bearing_degrees: f64, distance_metres: f64) -> NvectorPos<S> {
        let dest = private::destination_pos(
            self.nvector(),
            bearing_degrees.to_radians(),
            distance_metres,
            self.earth_radius_metres(),
            Rounding::None,
        );
        NvectorPos::new(dest, self.model())
    }

    pub fn distance_metres_to(&self, to: Self) -> f64 {
        private::distance_metres(
            self.nvector(),
            to.nvector(),
            self.earth_radius_metres(),
            Rounding::None,
        )
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

    fn earth_radius_metres(&self) -> f64 {
        self.model().surface().mean_radius().as_metres()
    }
}

mod private {

    use crate::geodetic::antipode;
    use crate::{Error, GreatCircle, MinorArc, Rounding, Vec3};
    use std::f64::consts::PI;

    pub(crate) fn along_track_distance_metres(
        pos: Vec3,
        start: Vec3,
        normal: Vec3,
        radius_metres: f64,
        rounding: Rounding,
    ) -> f64 {
        let o = normal.cross(pos).cross(normal);
        let a = rounding.round_radians(signed_radians_between(start, o, Some(normal)));
        arc_length_metres(a, radius_metres)
    }

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

    pub(crate) fn cross_track_distance_metres(
        pos: Vec3,
        normal: Vec3,
        radius_metres: f64,
        rounding: Rounding,
    ) -> f64 {
        let a = rounding.round_radians(signed_radians_between(normal, pos, None) - (PI / 2.0));
        arc_length_metres(a, radius_metres)
    }

    pub(crate) fn destination_pos(
        p0: Vec3,
        bearing_radians: f64,
        distance_metres: f64,
        radius_metres: f64,
        rounding: Rounding,
    ) -> Vec3 {
        if distance_metres == 0.0 {
            p0
        } else {
            // east direction vector at p0
            let np = rounding.north_pole();
            let ed = np.cross(p0).unit();
            // north direction vector at p0
            let nd = p0.cross(ed);
            // central angle
            let ca = rounding.round_radians(distance_metres / radius_metres);
            // unit vector in the direction of the azimuth
            let de = nd * bearing_radians.cos() + ed * bearing_radians.sin();
            let nv = p0 * ca.cos() + de * ca.sin();
            nv
        }
    }

    pub(crate) fn distance_metres(
        p1: Vec3,
        p2: Vec3,
        radius_metres: f64,
        rounding: Rounding,
    ) -> f64 {
        let a = rounding.round_radians(signed_radians_between(p1, p2, None));
        arc_length_metres(a, radius_metres)
    }

    pub(crate) fn final_bearing_radians(
        v1: Vec3,
        v2: Vec3,
        rounding: Rounding,
    ) -> Result<f64, Error> {
        initial_bearing_radians(v2, v1, rounding).map(|b| normalise_radians(b, PI))
    }

    pub(crate) fn gc_intersection<P>(
        gc1: GreatCircle<P>,
        gc2: GreatCircle<P>,
    ) -> Result<Vec3, Error> {
        normal_intersection(gc1.normal, gc2.normal)
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

    pub(crate) fn intersection(
        ma: MinorArc<Vec3>,
        mb: MinorArc<Vec3>,
        rounding: Rounding,
    ) -> Result<Vec3, Error> {
        let i = rounding.round_pos(normal_intersection(ma.normal, mb.normal)?);
        let mid = unchecked_mean(vec![ma.start_pos, ma.end_pos, mb.start_pos, mb.end_pos]);
        let pot;
        if i.dot(mid) > 0.0 {
            pot = i;
        } else {
            pot = rounding.round_pos(antipode(i));
        }
        if is_on_minor_arc(pot, ma.start_pos, ma.end_pos)
            && is_on_minor_arc(pot, mb.start_pos, mb.end_pos)
        {
            Ok(pot)
        } else {
            Err(Error::NoIntersection)
        }
    }

    pub(crate) fn mean(vs: Vec<Vec3>, rounding: Rounding) -> Result<Vec3, Error> {
        if vs.is_empty() {
            Err(Error::NotEnoughPositions)
        } else if vs.len() == 1 {
            Ok(*vs.first().unwrap())
        } else if vs
            .iter()
            .map(|v| rounding.round_pos(antipode(*v)))
            .any(|v| vs.contains(&v))
        {
            Err(Error::AntipodalPositions)
        } else {
            Ok(unchecked_mean(vs))
        }
    }

    pub(crate) fn turn_radians(from: Vec3, at: Vec3, to: Vec3) -> Result<f64, Error> {
        let nfa = arc_normal(from, at)?;
        let nat = arc_normal(at, to)?;
        Ok(signed_radians_between(nfa.unit(), nat.unit(), Some(at)))
    }

    #[inline]
    fn arc_length_metres(radians: f64, radius_metres: f64) -> f64 {
        radians * radius_metres
    }

    fn is_on_minor_arc(v: Vec3, mas: Vec3, mae: Vec3) -> bool {
        let l = mas.square_distance_to(mae);
        v.square_distance_to(mas) <= l && v.square_distance_to(mae) <= l
    }

    fn normal_intersection(n1: Vec3, n2: Vec3) -> Result<Vec3, Error> {
        let i = n1.cross(n2);
        if i == Vec3::zero() {
            // same or opposite normals
            Err(Error::CoincidentalPath)
        } else {
            Ok(i)
        }
    }

    fn normalise_radians(a: f64, b: f64) -> f64 {
        (a + b) % (2.0 * PI)
    }

    fn signed_radians_between(v1: Vec3, v2: Vec3, vn: Option<Vec3>) -> f64 {
        let sign = vn.map_or(1.0, |n| n.dot(v1.cross(v2)).signum());
        let sin_o = sign * v1.cross(v2).norm();
        let cos_o = v1.dot(v2);
        sin_o.atan2(cos_o)
    }

    fn unchecked_mean(vs: Vec<Vec3>) -> Vec3 {
        vs.iter().fold(Vec3::zero(), |sum, v| sum + *v)
    }
}
