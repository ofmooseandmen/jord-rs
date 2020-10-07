use crate::{Angle, Error, LatLongPos, Length, NvectorPos, Spherical, SurfacePos, Vec3};

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
        private::arc_normal(p1, p2).map(|n| GreatCircle {
            position: p1,
            normal: n,
        })
    }

    pub fn from_lat_long_bearing(pos: LatLongPos<S>, bearing: Angle) -> GreatCircle<LatLongPos<S>> {
        let normal = private::arc_normal_bearing_radians(pos, bearing.as_radians());
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
        private::arc_normal(p1, p2).map(|n| GreatCircle {
            position: p1,
            normal: n,
        })
    }

    pub fn from_nvector_bearing_degrees(
        pos: NvectorPos<S>,
        bearing_degrees: f64,
    ) -> GreatCircle<NvectorPos<S>> {
        let normal = private::arc_normal_bearing_radians(pos, bearing_degrees.to_radians());
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
        private::arc_normal(start_pos, end_pos).map(|n| MinorArc {
            start_pos,
            end_pos,
            normal: n,
        })
    }

    pub fn intersection_with(&self, other: Self) -> Result<LatLongPos<S>, Error> {
        private::intersection(*self, other)
    }
}

impl<S: Spherical> MinorArc<NvectorPos<S>> {
    pub fn from_nvectors(
        start_pos: NvectorPos<S>,
        end_pos: NvectorPos<S>,
    ) -> Result<MinorArc<NvectorPos<S>>, Error> {
        private::arc_normal(start_pos, end_pos).map(|n| MinorArc {
            start_pos,
            end_pos,
            normal: n,
        })
    }

    pub fn intersection_with(&self, other: Self) -> Result<NvectorPos<S>, Error> {
        private::intersection(*self, other)
    }
}

impl<S: Spherical> LatLongPos<S> {
    pub fn from_mean(ps: &[LatLongPos<S>]) -> Result<Self, Error> {
        let m = private::mean(ps)?;
        // unwrap is safe because mean returns Err if ps is empty
        Ok(LatLongPos::from_nvector(m, ps.first().unwrap().model()))
    }

    pub fn along_track_distance_to(&self, ma: MinorArc<LatLongPos<S>>) -> Length {
        let metres = private::along_track_distance_metres(*self, ma);
        Length::from_metres(metres)
    }

    pub fn cross_track_distance_to(&self, gc: GreatCircle<LatLongPos<S>>) -> Length {
        let metres = private::cross_track_distance_metres(*self, gc.normal);
        Length::from_metres(metres)
    }

    pub fn destination_pos(&self, bearing: Angle, distance: Length) -> Self {
        private::destination_pos(*self, bearing.as_radians(), distance.as_metres())
    }

    pub fn distance_to(&self, to: Self) -> Length {
        let metres = private::distance_metres(*self, to);
        Length::from_metres(metres)
    }

    pub fn final_bearing_to(&self, to: Self) -> Result<Angle, Error> {
        private::final_bearing_radians(*self, to).map(Angle::from_radians)
    }

    pub fn initial_bearing_to(&self, to: Self) -> Result<Angle, Error> {
        private::initial_bearing_radians(*self, to).map(Angle::from_radians)
    }

    pub fn intermediate_pos_to(&self, to: Self, f: f64) -> Result<Self, Error> {
        private::intermediate_pos(*self, to, f)
    }

    pub fn projection_onto(&self, ma: MinorArc<LatLongPos<S>>) -> Result<LatLongPos<S>, Error> {
        private::projection(*self, ma)
    }

    pub fn turn(&self, from: Self, to: Self) -> Result<Angle, Error> {
        private::turn_radians(from, *self, to).map(Angle::from_radians)
    }
}

impl<S: Spherical> NvectorPos<S> {
    pub fn from_mean(ps: &[NvectorPos<S>]) -> Result<Self, Error> {
        let m = private::mean(ps)?;
        // unwrap is safe because mean returns Err if ps is empty
        Ok(NvectorPos::new(m, ps.first().unwrap().model()))
    }

    pub fn along_track_distance_metres_to(&self, ma: MinorArc<NvectorPos<S>>) -> f64 {
        private::along_track_distance_metres(*self, ma)
    }

    pub fn cross_track_distance_metres_to(&self, gc: GreatCircle<NvectorPos<S>>) -> f64 {
        private::cross_track_distance_metres(*self, gc.normal)
    }

    pub fn destination_pos(&self, bearing_degrees: f64, distance_metres: f64) -> NvectorPos<S> {
        private::destination_pos(*self, bearing_degrees.to_radians(), distance_metres)
    }

    pub fn distance_metres_to(&self, to: Self) -> f64 {
        private::distance_metres(*self, to)
    }

    pub fn final_bearing_degrees_to(&self, to: Self) -> Result<f64, Error> {
        private::final_bearing_radians(*self, to).map(|b| b.to_degrees())
    }

    pub fn initial_bearing_degrees_to(&self, to: Self) -> Result<f64, Error> {
        private::initial_bearing_radians(*self, to).map(|b| b.to_degrees())
    }

    pub fn intermediate_pos_to(&self, to: Self, f: f64) -> Result<Self, Error> {
        private::intermediate_pos(*self, to, f)
    }

    pub fn projection_onto(&self, ma: MinorArc<NvectorPos<S>>) -> Result<NvectorPos<S>, Error> {
        private::projection(*self, ma)
    }

    pub fn turn_degrees(&self, from: Self, to: Self) -> Result<f64, Error> {
        private::turn_radians(from, *self, to).map(|b| b.to_degrees())
    }
}

mod private {

    use crate::{Error, GreatCircle, MinorArc, Spherical, Surface, SurfacePos, Vec3};
    use std::f64::consts::PI;

    pub(crate) fn along_track_distance_metres<S: Spherical, P: SurfacePos<S>>(
        pos: P,
        ma: MinorArc<P>,
    ) -> f64 {
        let normal = ma.normal;
        let v = pos.to_nvector();
        let o = normal.cross(v).cross(normal);
        let a = signed_radians_between(ma.start_pos.to_nvector(), o, Some(normal));
        arc_length_metres(a, earth_radius_metres(pos))
    }

    pub(crate) fn arc_normal<S: Spherical, P: SurfacePos<S>>(p1: P, p2: P) -> Result<Vec3, Error> {
        if p1 == p2 {
            Err(Error::CoincidentalPositions)
        } else if p1.antipode() == p2 {
            Err(Error::AntipodalPositions)
        } else {
            Ok(p1.to_nvector().cross(p2.to_nvector()))
        }
    }

    pub(crate) fn arc_normal_bearing_radians<S: Spherical, P: SurfacePos<S>>(
        pos: P,
        bearing_radians: f64,
    ) -> Vec3 {
        let v = pos.to_nvector();
        // easting
        let e = P::north_pole().cross(v);
        // northing
        let n = v.cross(e);
        let se = e * (bearing_radians.cos() / e.norm());
        let sn = n * (bearing_radians.sin() / n.norm());
        sn - se
    }

    pub(crate) fn cross_track_distance_metres<S: Spherical, P: SurfacePos<S>>(
        pos: P,
        normal: Vec3,
    ) -> f64 {
        let a = signed_radians_between(normal, pos.to_nvector(), None) - (PI / 2.0);
        arc_length_metres(a, earth_radius_metres(pos))
    }

    pub(crate) fn destination_pos<S: Spherical, P: SurfacePos<S>>(
        p0: P,
        bearing_radians: f64,
        distance_metres: f64,
    ) -> P {
        if distance_metres == 0.0 {
            p0
        } else {
            let v0 = p0.to_nvector();
            // east direction vector at p0
            let np = P::north_pole();
            let ed = np.cross(v0).unit();
            // north direction vector at p0
            let nd = v0.cross(ed);
            // central angle
            let ca = distance_metres / earth_radius_metres(p0);
            // unit vector in the direction of the azimuth
            let de = nd * bearing_radians.cos() + ed * bearing_radians.sin();
            let nv = v0 * ca.cos() + de * ca.sin();
            P::from_nvector(nv, p0.model())
        }
    }

    pub(crate) fn distance_metres<S: Spherical, P: SurfacePos<S>>(p1: P, p2: P) -> f64 {
        let a = signed_radians_between(p1.to_nvector(), p2.to_nvector(), None);
        arc_length_metres(a, earth_radius_metres(p1))
    }

    pub(crate) fn final_bearing_radians<S: Spherical, P: SurfacePos<S>>(
        p1: P,
        p2: P,
    ) -> Result<f64, Error> {
        initial_bearing_radians(p2, p1).map(|b| normalise_radians(b, PI))
    }

    pub(crate) fn gc_intersection<P>(
        gc1: GreatCircle<P>,
        gc2: GreatCircle<P>,
    ) -> Result<Vec3, Error> {
        normal_intersection(gc1.normal, gc2.normal)
    }

    pub(crate) fn initial_bearing_radians<S: Spherical, P: SurfacePos<S>>(
        p1: P,
        p2: P,
    ) -> Result<f64, Error> {
        if p1 == p2 {
            Err(Error::CoincidentalPositions)
        } else {
            let v1 = p1.to_nvector();
            let v2 = p2.to_nvector();
            // great circle through p1 & p2
            let gc1 = v1.cross(v2);
            // great circle through p1 & north pole
            let np = P::north_pole();
            let gc2 = v1.cross(np);
            let a = signed_radians_between(gc1, gc2, Some(v1));
            Ok(normalise_radians(a, 2.0 * PI))
        }
    }

    pub(crate) fn intermediate_pos<S: Spherical, P: SurfacePos<S>>(
        p1: P,
        p2: P,
        f: f64,
    ) -> Result<P, Error> {
        if f < 0.0 || f > 1.0 {
            Err(Error::OutOfRange)
        } else {
            let v1 = p1.to_nvector();
            let v2 = p2.to_nvector();
            let v = (v1 + f * (v2 - v1)).unit();
            Ok(P::from_nvector(v, p1.model()))
        }
    }

    pub(crate) fn intersection<S: Spherical, P: SurfacePos<S>>(
        ma: MinorArc<P>,
        mb: MinorArc<P>,
    ) -> Result<P, Error> {
        let mas = ma.start_pos.to_nvector();
        let mae = ma.end_pos.to_nvector();
        let mbs = mb.start_pos.to_nvector();
        let mbe = mb.end_pos.to_nvector();
        let iv = normal_intersection(ma.normal, mb.normal)?;
        let i = P::from_nvector(iv, ma.start_pos.model());
        let mid = unchecked_mean(vec![mas, mae, mbs, mbe]);
        let pot;
        if iv.dot(mid) > 0.0 {
            pot = i;
        } else {
            pot = i.antipode()
        }
        let vpot = pot.to_nvector();
        if is_on_minor_arc(vpot, mas, mae) && is_on_minor_arc(vpot, mbs, mbe) {
            Ok(pot)
        } else {
            Err(Error::NoIntersection)
        }
    }

    pub(crate) fn mean<S: Spherical, P: SurfacePos<S>>(ps: &[P]) -> Result<Vec3, Error> {
        if ps.is_empty() {
            Err(Error::NotEnoughPositions)
        } else if ps.len() == 1 {
            Ok((*ps.first().unwrap()).to_nvector())
        } else if ps.iter().map(|p| (*p).antipode()).any(|p| ps.contains(&p)) {
            Err(Error::AntipodalPositions)
        } else {
            Ok(unchecked_mean(ps.iter().map(|p| p.to_nvector()).collect()))
        }
    }

    pub(crate) fn projection<S: Spherical, P: SurfacePos<S>>(
        pos: P,
        ma: MinorArc<P>,
    ) -> Result<P, Error> {
        let na = P::from_nvector(ma.normal.unit(), pos.model());
        // normal to great circle (na, p) - if na is p or antipode of p, then projection is not possible
        let nb = P::from_nvector(arc_normal(na, pos)?.unit(), pos.model());

        let mas = ma.start_pos.to_nvector();
        let mae = ma.end_pos.to_nvector();
        let nav = na.to_nvector();
        let nbv = nb.to_nvector();

        let mid = unchecked_mean(vec![mas, mae, nav, nbv]);
        let iv = normal_intersection(nav, nbv)?;
        let i = P::from_nvector(iv, ma.start_pos.model());
        let pot;
        if iv.dot(mid) > 0.0 {
            pot = i;
        } else {
            pot = i.antipode();
        }
        if is_on_minor_arc(pot.to_nvector(), mas, mae) {
            Ok(pot)
        } else {
            Err(Error::NoIntersection)
        }
    }

    pub(crate) fn turn_radians<S: Spherical, P: SurfacePos<S>>(
        from: P,
        at: P,
        to: P,
    ) -> Result<f64, Error> {
        let nfa = arc_normal(from, at)?;
        let nat = arc_normal(at, to)?;
        Ok(signed_radians_between(
            nfa.unit(),
            nat.unit(),
            Some(at.to_nvector()),
        ))
    }

    #[inline]
    fn arc_length_metres(radians: f64, radius_metres: f64) -> f64 {
        radians * radius_metres
    }

    #[inline]
    fn earth_radius_metres<S: Spherical, P: SurfacePos<S>>(p: P) -> f64 {
        p.model().surface().mean_radius().as_metres()
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
