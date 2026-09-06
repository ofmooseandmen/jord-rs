use crate::{
    Angle, NVector, Vec3,
    numbers::eq_zero,
    spherical::{ChordLength, Side},
};

use super::base::{angle_radians_between, side};

/// Oriented minor arc of a great circle between two positions: shortest path between positions
/// on a great circle.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))] // codecov:ignore:this
pub struct MinorArc {
    start: NVector,
    end: NVector,
    normal: Vec3,
}

/// The topological relationship between two [`MinorArc`]s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MinorArcRelation {
    /// The arcs share no points.
    Disjoint,
    /// The arcs intersect at a single point strictly interior to both arcs.
    Intersecting(NVector),
    /// The arcs meet at exactly one point that is an endpoint of at least one of them.
    Touching(NVector),
    /// The arcs lie on the same great circle and overlap along a shared sub-arc (more than a
    /// single point).
    Overlapping(MinorArc),
}

/// result of `triage_on_minor_arc`.
#[derive(Debug, Clone, Copy, PartialEq)]
enum OnMinorArcLocation {
    Endpoint,
    Interior,
    Exterior,
}

impl OnMinorArcLocation {
    fn is_within(self) -> bool {
        self != OnMinorArcLocation::Exterior
    }
}

/// result of `intersection_details`.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Intersection {
    Some(MinorArcRelation),
    None,
    Collinear,
}

impl Intersection {
    fn touching(i: Vec3) -> Intersection {
        Intersection::Some(MinorArcRelation::Touching(NVector::new(i)))
    }

    fn intersecting(i: Vec3) -> Intersection {
        Intersection::Some(MinorArcRelation::Intersecting(NVector::new(i)))
    }
}

impl MinorArc {
    /// Creates a new minor arc from the given start and end positions.
    ///
    /// Note: an arbitrary minor arc is returned if both start and end positions are equal
    /// or the antipode of one another: in this case an infinity of minor arcs exist - see [`is_great_cirle`](crate::spherical::Sphere::is_great_circle).
    pub fn new(start: NVector, end: NVector) -> Self {
        let normal = start.as_vec3().orthogonal_to(end.as_vec3());
        MinorArc { start, end, normal }
    }

    /// Returns the start position of this minor arc.
    #[inline]
    pub fn start(&self) -> NVector {
        self.start
    }

    /// Returns the end position of this minor arc.
    #[inline]
    pub fn end(&self) -> NVector {
        self.end
    }

    /// Returns the vector normal to this minor arc.
    #[inline]
    pub fn normal(&self) -> Vec3 {
        self.normal
    }

    /// Computes the [chord length](crate::spherical::ChordLength) between the given position and the closest
    /// position on this minor arc of great circle from that position.
    /// Note: a chord length is returned instead of a distance as it is much faster to compute, specially when the
    /// minimum distance to a set of edges (i.e. a loop) is to be computed.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{LatLong, Length};
    /// use jord::spherical::{ChordLength, MinorArc, Sphere};
    ///
    /// let ma = MinorArc::new(
    ///     LatLong::from_degrees(0.0, 0.0).to_nvector(),
    ///     LatLong::from_degrees(0.0, 10.0).to_nvector()
    /// );
    /// let p = LatLong::from_degrees(1.0, 5.0).to_nvector();
    ///
    /// let a = ma.distance_to(p);
    /// let e = Sphere::angle(p, ma.projection(p).unwrap());
    ///
    /// assert_eq!(e.round_d7(), a.to_angle().round_d7());
    /// ```
    pub fn distance_to(&self, p: NVector) -> ChordLength {
        // This method computes the minimum distance between p and any point on this edge; it can be either
        // on the interior of the edge or to one of the endpoints.
        //
        // This can be achieve by first projecting p on this edge and if the result is within edge, returning the
        // distance between p and the projection. Otherwise returning the minimum distance between (p, edge::start) and (p,
        // edge::end).
        //
        // The code below effectively implements this logic, using the chord length instead (as explained
        // in the documentation).

        //
        // Consider the triangle [X, A, B]
        //
        // Let XA = distance between X and A, XB = distance between X and B and AB = distance between A and B and X
        // Let ALPHA = angle [X, A] to [A, B]
        // Law of cosine: XA^2 = XB^2 + AB^2 - 2.XB.AB.cos(ALPHA), yielding XA^2 - XB^2 = AB^2 - 2.XB.AB.cos(ALPHA).
        //
        // For the closest point to be on [A, B], XAB and XBA must both be acute angles if ALPHA < 90 then
        // 2.XB.AB.cos(ALPHA) > 0 and then XA^2 - XB^2 >= AB^2 + EPS to be on the safe side.
        //
        let xa = ChordLength::new(p, self.start);
        let xb = ChordLength::new(p, self.end);
        let ab = ChordLength::new(self.start, self.end);
        if (xa.length2() - xb.length2()) >= ab.length2() + f64::EPSILON {
            return xa.min(xb);
        }

        let n2 = p.as_vec3().stable_cross_prod_unit(self.normal);
        if n2 == Vec3::ZERO {
            // p is "perpendicular" to e, so the closest distance is to any point of the edge; pick edge::start.
            return xa;
        }

        let proj = self.normal.orthogonal_to(n2);
        if self.triage_on_minor_arc(proj).is_within() {
            // p is "within" this edge, return the distance between p and the projection.
            return ChordLength::new(p, NVector::new(proj));
        }
        xa.min(xb)
    }

    /// Computes the intersection point between this minor arc and the given minor arc, if there is an
    /// intersection.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{LatLong, Length};
    /// use jord::spherical::MinorArc;
    ///
    /// let ma1 = MinorArc::new(
    ///     LatLong::from_degrees(-10.0, 0.0).to_nvector(),
    ///     LatLong::from_degrees(10.0, 0.0).to_nvector()
    /// );
    /// let ma2 = MinorArc::new(
    ///     LatLong::from_degrees(0.0, -10.0).to_nvector(),
    ///     LatLong::from_degrees(0.0, 10.0).to_nvector()
    /// );
    /// let i = ma1.intersection(ma2);
    /// assert_eq!(i, Some(LatLong::from_degrees(0.0, 0.0).to_nvector()));
    /// ```
    pub fn intersection(&self, other: MinorArc) -> Option<NVector> {
        let i = self.intersection_details(other);
        match i {
            Intersection::Some(
                MinorArcRelation::Intersecting(nv) | MinorArcRelation::Touching(nv),
            ) => Some(nv),
            _ => None,
        }
    }

    /// Determines the [topological relationship](MinorArcRelation) between
    /// this minor arc and `other`.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, NVector};
    /// use jord::spherical::{MinorArc, MinorArcRelation};
    ///
    /// let ma1 = MinorArc::new(
    ///     NVector::from_lat_long_degrees(0.0, -10.0),
    ///     NVector::from_lat_long_degrees(0.0, 5.0)
    /// );
    /// let ma2 = MinorArc::new(
    ///     NVector::from_lat_long_degrees(0.0, -5.0),
    ///     NVector::from_lat_long_degrees(0.0, 10.0)
    /// );
    ///
    /// let o = MinorArc::new(
    ///     NVector::from_lat_long_degrees(0.0, -5.0),
    ///     NVector::from_lat_long_degrees(0.0, 5.0)
    /// );
    /// assert_eq!(MinorArcRelation::Overlapping(o), ma1.relate(ma2));
    /// ```
    pub fn relate(&self, other: MinorArc) -> MinorArcRelation {
        let i = self.intersection_details(other);
        match i {
            Intersection::Collinear => self.relate_collinear(other),
            Intersection::Some(r) => r,
            Intersection::None => MinorArcRelation::Disjoint,
        }
    }

    /// Computes the projection of the given position on this minor arc. Returns [`None`] if the projection is not
    /// within the minor arc (including start and end). If the given position is strictly "perpendicular" to this
    /// minor arc, this method arbitrarily returns the start (p can be projected anywhere on the minor arc).
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{LatLong, Length};
    /// use jord::spherical::MinorArc;
    ///
    /// let ma = MinorArc::new(
    ///     LatLong::from_degrees(0.0, -10.0).to_nvector(),
    ///     LatLong::from_degrees(0.0, 10.0).to_nvector()
    /// );
    ///
    /// let o_p = ma.projection(LatLong::from_degrees(1.0, 0.0).to_nvector());
    /// assert!(o_p.is_some());
    /// assert_eq!(LatLong::from_degrees(0.0, 0.0), LatLong::from_nvector(o_p.unwrap()).round_d7());
    /// ```
    pub fn projection(&self, p: NVector) -> Option<NVector> {
        let n1 = self.normal;
        let n2 = p.as_vec3().stable_cross_prod_unit(n1);
        if n2 == Vec3::ZERO {
            Some(self.start)
        } else {
            let proj = n1.orthogonal_to(n2);
            if self.triage_on_minor_arc(proj).is_within() {
                Some(NVector::new(proj))
            } else {
                None
            }
        }
    }

    /// Determines whether this minor arc contains the given position.
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::MinorArc;
    ///
    /// let ma = MinorArc::new(
    ///     NVector::from_lat_long_degrees(0.0, -10.0),
    ///     NVector::from_lat_long_degrees(0.0, 10.0)
    /// );
    ///
    /// assert!(ma.contains_position(NVector::from_lat_long_degrees(0.0, 5.0)));
    /// assert!(!ma.contains_position(NVector::from_lat_long_degrees(1.0, 5.0)));
    /// assert!(!ma.contains_position(NVector::from_lat_long_degrees(0.0, 11.0)));
    /// assert!(!ma.contains_position(NVector::from_lat_long_degrees(0.0, -11.0)));
    /// ```
    pub fn contains_position(&self, p: NVector) -> bool {
        let v = p.as_vec3();
        eq_zero(v.dot_prod(self.normal)) && self.triage_on_minor_arc(v).is_within()
    }

    /// Determines whether p if right of, left of or on this minor arc.
    ///
    /// This is similar to [`side(p, self.start, self.end)`](crate::spherical::Sphere::side) but avoids the calculation of the orthogonal
    /// vector to (`self.start`, `self.end`).
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::MinorArc;
    ///
    /// let p = NVector::from_lat_long_degrees(55.4295, 13.82);
    /// let ma1 = MinorArc::new(
    ///     NVector::from_lat_long_degrees(56.0465, 12.6945),
    ///     NVector::from_lat_long_degrees(56.0294, 14.1567)
    /// );
    /// let ma2 = MinorArc::new(
    ///     NVector::from_lat_long_degrees(56.0294, 14.1567),
    ///     NVector::from_lat_long_degrees(56.0465, 12.6945)
    /// );
    ///
    /// assert_eq!(-1, ma1.side_of(p));
    /// assert_eq!(1, ma2.side_of(p));
    /// ```
    pub fn side_of(&self, p: NVector) -> i8 {
        let side = p.as_vec3().dot_prod(self.normal);
        if eq_zero(side) {
            0
        } else if side < 0.0 {
            -1
        } else {
            1
        }
    }

    /// Given `self` = (A, B) and `o` = (B, C): calculates the angle turned from AB to BC.
    ///
    /// Note: this function assumes that `self.end == o.start` and as such is similar to
    /// [turn(self.start, self.end, o.start)](crate::spherical::Sphere::turn) but avoids the calculation of the orthogonal
    /// vector to (`self.start`, `self.end`) and (`o.start`, `o.end`).
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, NVector};
    /// use jord::spherical::MinorArc;
    ///
    /// let ma1 = MinorArc::new(
    ///     NVector::from_lat_long_degrees(0.0, 0.0),
    ///     NVector::from_lat_long_degrees(45.0, 0.0)
    /// );
    /// let ma2 = MinorArc::new(
    ///     NVector::from_lat_long_degrees(45.0, 0.0),
    ///     NVector::from_lat_long_degrees(60.0, -10.0)
    /// );
    ///
    /// assert_eq!(Angle::from_radians(0.3175226173130951), ma1.turn(ma2));
    /// assert_eq!(-ma1.turn(ma2), ma2.turn(ma1));
    /// ```
    pub fn turn(&self, o: MinorArc) -> Angle {
        Angle::from_radians(angle_radians_between(
            self.normal,
            o.normal,
            Some(self.end.as_vec3()),
        ))
    }

    /// Returns the minor arc opposite to this minor arc: if this minor arc is (a, b), the returned minor arc is (b, a).
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::MinorArc;
    ///
    /// let p = NVector::from_lat_long_degrees(55.4295, 13.82);
    /// let ma1 = MinorArc::new(
    ///     NVector::from_lat_long_degrees(56.0465, 12.6945),
    ///     NVector::from_lat_long_degrees(56.0294, 14.1567)
    /// );
    /// let ma2 = MinorArc::new(
    ///     NVector::from_lat_long_degrees(56.0294, 14.1567),
    ///     NVector::from_lat_long_degrees(56.0465, 12.6945)
    /// );
    ///
    /// assert_eq!(ma2, ma1.opposite());
    /// assert_eq!(ma1, ma2.opposite());
    /// ```
    pub fn opposite(&self) -> MinorArc {
        Self {
            start: self.end,
            end: self.start,
            normal: -self.normal,
        }
    }

    /// Determines whether the given point which is on the great circle of the
    /// given minor arc, is an interior point of arc, an end point or outside.
    fn triage_on_minor_arc(&self, v: Vec3) -> OnMinorArcLocation {
        // v is left of (normal, start)
        // and
        // v is right of (normal, end)
        let start = self.start.as_vec3();
        let end = self.end.as_vec3();
        let n = self.normal;
        let s_start = side(v, n, start);
        if s_start == Side::Right {
            return OnMinorArcLocation::Exterior;
        }

        let s_end = side(v, n, end);
        if s_end == Side::Left {
            return OnMinorArcLocation::Exterior;
        }

        if s_start == Side::Left && s_end == Side::Right {
            OnMinorArcLocation::Interior
        } else {
            OnMinorArcLocation::Endpoint
        }
    }

    fn intersection_details(&self, other: MinorArc) -> Intersection {
        let i = self.normal.stable_cross_prod_unit(other.normal);
        if i == Vec3::ZERO {
            // collinear minor arcs: no intersection
            Intersection::Collinear
        } else {
            // select nearest intersection to start of first minor arc.
            let potential = if self.start.as_vec3().dot_prod(i) > 0.0 {
                i
            } else {
                // antipode of i.
                -i
            };

            let self_contains = self.triage_on_minor_arc(potential);
            if self_contains == OnMinorArcLocation::Exterior {
                return Intersection::None;
            }

            let other_contains = other.triage_on_minor_arc(potential);
            if other_contains == OnMinorArcLocation::Exterior {
                return Intersection::None;
            }

            if self_contains == OnMinorArcLocation::Endpoint
                || other_contains == OnMinorArcLocation::Endpoint
            {
                return Intersection::touching(potential);
            }

            // neither is outside but both are not endpoint
            Intersection::intersecting(potential)
        }
    }

    /// Handles the case where `self` and `other` lie on the same great circle, by reducing to
    /// interval overlap: a point is part of the shared overlap iff it lies on *both* arcs, and
    /// since the overlap of two closed intervals always has each boundary point equal to one
    /// of the four original endpoints, testing just those four candidates is sufficient.
    fn relate_collinear(&self, other: MinorArc) -> MinorArcRelation {
        let mut found: [Option<Vec3>; 2] = [None, None];
        let mut push = |v: Vec3| {
            if found[0] == Some(v) || found[1] == Some(v) {
                return;
            }
            if found[0].is_none() {
                found[0] = Some(v);
            } else if found[1].is_none() {
                found[1] = Some(v);
            }
            // a third distinct candidate cannot occur for two overlapping intervals; if it
            // somehow did, it would be silently dropped here rather than panicking.
        };

        if self
            .triage_on_minor_arc(other.start().as_vec3())
            .is_within()
        {
            push(other.start().as_vec3());
        }
        if self.triage_on_minor_arc(other.end().as_vec3()).is_within() {
            push(other.end().as_vec3());
        }
        if other
            .triage_on_minor_arc(self.start().as_vec3())
            .is_within()
        {
            push(self.start().as_vec3());
        }
        if other.triage_on_minor_arc(self.end().as_vec3()).is_within() {
            push(self.end().as_vec3());
        }

        match (found[0], found[1]) {
            (None, _) => MinorArcRelation::Disjoint,
            (Some(p), None) => MinorArcRelation::Touching(NVector::new(p)),
            (Some(p), Some(q)) => {
                // Orient the result to match self's own direction, so it doesn't depend on
                // which of the four checks above happened to fire first: whichever of the two
                // points is angularly closer to self.start() (larger dot product, since both
                // lie within self's own span) becomes the start.
                let self_start = self.start.as_vec3();
                let (lo, hi) = if p.dot_prod(self_start) >= q.dot_prod(self_start) {
                    (p, q)
                } else {
                    (q, p)
                };
                // lo/hi are a sub-arc of self's own great circle in self's own rotational sense,
                // so they share self's normal exactly -- no need to recompute one via `new()`.
                MinorArcRelation::Overlapping(MinorArc {
                    start: NVector::new(lo),
                    end: NVector::new(hi),
                    normal: self.normal,
                })
            }
        }
    }
}

impl PartialEq for MinorArc {
    fn eq(&self, other: &Self) -> bool {
        // no need to test normal as it is derived from start/end.
        self.start == other.start && self.end == other.end
    }
}

#[cfg(test)]
mod tests {

    #![allow(clippy::pedantic)]

    use crate::{
        Angle, LatLong, Length, NVector, Vec3,
        positions::{assert_nv_eq_d7, assert_opt_nv_eq_d7},
        spherical::{
            ChordLength, GreatCircle, MinorArc, MinorArcRelation, Side, Sphere, base::side,
            minor_arc::OnMinorArcLocation,
        },
    };

    #[test]
    fn partial_eq() {
        let start = NVector::from_lat_long_degrees(154.0, 54.0);
        let end = NVector::from_lat_long_degrees(155.0, 55.0);
        let end2 = NVector::from_lat_long_degrees(156.0, 56.0);
        assert_eq!(MinorArc::new(start, end), MinorArc::new(start, end));
        assert_ne!(MinorArc::new(start, end), MinorArc::new(end, start));
        assert_ne!(MinorArc::new(start, end), MinorArc::new(start, end2));
    }

    // distance_to
    #[test]
    fn distance_to_close_interior() {
        let e = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 10.0),
        );
        let p = NVector::from_lat_long_degrees(-1.0 / 3_600_000_000.0, 0.0);
        let actual = e.distance_to(p);
        let projection = e.projection(p).unwrap();
        let expected = ChordLength::new(p, projection);
        assert_eq!(expected, actual);
    }

    #[test]
    fn distance_to_close_start() {
        let e = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 10.0),
        );
        let p = NVector::from_lat_long_degrees(1.0, -1.0 / 3_600_000_000.0);
        let actual: ChordLength = e.distance_to(p);
        let expected = ChordLength::new(p, e.start());
        assert_eq!(expected, actual);
    }

    #[test]
    fn distance_to_end() {
        let e: MinorArc = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 10.0),
        );
        let p = NVector::from_lat_long_degrees(-1.0, 11.0);
        let actual: ChordLength = e.distance_to(p);
        let expected = ChordLength::new(p, e.end());
        assert_eq!(expected, actual);
    }

    #[test]
    fn distance_to_interior() {
        let e: MinorArc = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 10.0),
        );
        let p = NVector::from_lat_long_degrees(1.0, 5.0);
        let actual: ChordLength = e.distance_to(p);
        let projection = e.projection(p).unwrap();
        let expected = ChordLength::new(p, projection);
        assert_eq!(expected, actual);
    }

    #[test]
    fn distance_to_perpendicular() {
        let e: MinorArc = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 10.0),
        );
        let p = NVector::from_lat_long_degrees(90.0, 0.0);
        let actual = e.distance_to(p);
        assert_eq!(ChordLength::new(p, e.start()), actual);
    }

    #[test]
    fn distance_to_start() {
        let e: MinorArc = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 10.0),
        );
        let p = NVector::from_lat_long_degrees(-1.0, -1.0);
        let actual: ChordLength = e.distance_to(p);
        let expected = ChordLength::new(p, e.start());
        assert_eq!(expected, actual);
    }

    // intersection

    #[test]
    fn intersection_eq() {
        let arc = MinorArc::new(
            NVector::from_lat_long_degrees(54.0, 154.0),
            NVector::from_lat_long_degrees(-54.0, 154.0),
        );
        assert!(arc.intersection(arc).is_none());
    }

    #[test]
    fn intersection_opposite() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(54.0, 154.0),
            NVector::from_lat_long_degrees(-54.0, 154.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(-54.0, 154.0),
            NVector::from_lat_long_degrees(54.0, 154.0),
        );
        assert!(arc1.intersection(arc2).is_none());
    }

    #[test]
    fn intersection_arc_across_equator() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(54.0, 154.0),
            NVector::from_lat_long_degrees(-54.0, 154.0),
        );

        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(53.0, 153.0),
            NVector::from_lat_long_degrees(53.0, 155.0),
        );

        assert_intersection(
            NVector::new(Vec3::new(
                -0.5408552101001728,
                0.26379271166149,
                0.7986795646451562,
            )),
            arc1,
            arc2,
        );
    }

    #[test]
    fn intersection_at_end() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 20.0),
        );

        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(10.0, 20.0),
            NVector::from_lat_long_degrees(-10.0, 20.0),
        );

        assert_intersection(NVector::from_lat_long_degrees(0.0, 20.0), arc1, arc2);
    }

    #[test]
    fn intersection_at_start() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 20.0),
        );

        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(10.0, 0.0),
            NVector::from_lat_long_degrees(-10.0, 0.0),
        );

        assert_intersection(NVector::from_lat_long_degrees(0.0, 0.0), arc1, arc2);
    }

    #[test]
    fn intersection_at_shared() {
        let shared = NVector::from_lat_long_degrees(-25.0, 130.0);
        let arc1 = MinorArc::new(
            shared,
            NVector::from_lat_long_degrees(-24.950243870277777, 133.85817408527777),
        );
        let arc2 = MinorArc::new(
            shared,
            NVector::from_lat_long_degrees(-25.857954033055556, 133.75470594055557),
        );
        assert_intersection(shared, arc1, arc2);
    }

    #[test]
    fn intersection_close() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(-27.1789705075, 152.3083728075),
            NVector::from_lat_long_degrees(-27.0741667000, 152.2163889000),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(-27.1245578000, 152.1506886000),
            NVector::from_lat_long_degrees(-27.0741667000, 152.2163889000),
        );

        assert_intersection(
            NVector::from_lat_long_degrees(-27.0741667, 152.2163889),
            arc1,
            arc2,
        );
    }

    #[test]
    fn intersection_nominal() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(-36.0, 143.0),
            NVector::from_lat_long_degrees(-34.0, 145.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(-34.0, 143.0),
            NVector::from_lat_long_degrees(-36.0, 145.0),
        );
        assert_intersection(
            NVector::from_lat_long_degrees(-35.0163245, 144.0),
            arc1,
            arc2,
        );
    }

    #[test]
    fn intersection_null_island() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, -1.0),
            NVector::from_lat_long_degrees(0.0, 1.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(-1.0, 0.0),
            NVector::from_lat_long_degrees(1.0, 0.0),
        );
        assert_intersection(NVector::from_lat_long_degrees(0.0, 0.0), arc1, arc2);
    }

    #[test]
    fn intersection_pole() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(45.0, 0.0),
            NVector::from_lat_long_degrees(45.0, 180.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(45.0, 90.0),
            NVector::from_lat_long_degrees(45.0, 270.0),
        );
        let opt_i = arc1.intersection(arc2);
        assert!(opt_i.is_some());
        let i = opt_i.unwrap();
        assert_eq!(
            Angle::from_degrees(90.0),
            LatLong::from_nvector(i).latitude()
        );
    }

    #[test]
    fn intersection_small_minor_arc() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(-20.8464124400, 123.2066292450),
            NVector::from_lat_long_degrees(-20.8463888889, 123.2066666667),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(-20.3716666667, 122.2811111111),
            NVector::from_lat_long_degrees(-21.5219444444, 124.5511111111),
        );
        assert!(Sphere::EARTH.distance(arc1.start(), arc1.end()) < Length::from_metres(5.0));

        assert_intersection(
            NVector::from_lat_long_degrees(-20.8464124, 123.2066292),
            arc1,
            arc2,
        );
    }

    #[test]
    fn intersection_very_small_minor_arc() {
        let tenth_of_mm = Length::from_metres(1e-4);
        let arc1_start = NVector::from_lat_long_degrees(-32.7929069956, 135.4840669972);
        let arc1_end =
            Sphere::EARTH.destination_position(arc1_start, Angle::from_degrees(45.0), tenth_of_mm);

        let arc1 = MinorArc::new(arc1_start, arc1_end);

        let arc1_midpoint = Sphere::interpolated_position(arc1_start, arc1_end, 0.5).unwrap();

        let arc2_start = Sphere::EARTH.destination_position(
            arc1_midpoint,
            Angle::from_degrees(315.0),
            tenth_of_mm,
        );
        let arc2_end =
            Sphere::EARTH.destination_position(arc2_start, Angle::from_degrees(135.0), tenth_of_mm);
        let arc2 = MinorArc::new(arc2_start, arc2_end);

        assert_intersection(arc1_midpoint, arc1, arc2);
    }

    #[test]
    fn no_intersection() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(45.0, 0.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 90.0),
            NVector::from_lat_long_degrees(45.0, 90.0),
        );
        assert_eq!(None, arc1.intersection(arc2));
    }

    #[test]
    fn no_intersection_candidate_close_to_first() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(54.0, 178.8),
            NVector::from_lat_long_degrees(54.0, -179.8),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(-80.0, 179.0),
            NVector::from_lat_long_degrees(-85.0, 179.0),
        );
        assert_eq!(None, arc1.intersection(arc2));
    }

    #[test]
    fn no_intersection_close_first_arc() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(-27.7022222000, 152.5372222000),
            NVector::from_lat_long_degrees(-27.4319444000, 152.4188889000),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(-27.3874939000, 152.4658169000),
            NVector::from_lat_long_degrees(-27.3518653000, 152.5214517000),
        );
        assert_eq!(None, arc1.intersection(arc2));
    }

    #[test]
    fn no_intersection_close_second_arc() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(-27.7022222000, 152.5372222000),
            NVector::from_lat_long_degrees(-27.4319444000, 152.4188889000),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(-27.4754111000, 152.7457194000),
            NVector::from_lat_long_degrees(-27.4733058000, 152.6958286000),
        );
        assert_eq!(None, arc1.intersection(arc2));
    }

    #[test]
    fn no_intersection_long_arcs_far_apart() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(9.0, -83.0),
            NVector::from_lat_long_degrees(-33.8179213708, 112.4433954286),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(10.0, 55.0),
            NVector::from_lat_long_degrees(10.0, 179.0),
        );
        assert_eq!(None, arc1.intersection(arc2));
    }

    #[test]
    fn no_intersection_parallel_minor_arcs() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(45.0, 0.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(46.0, 0.0),
            NVector::from_lat_long_degrees(48.0, 0.0),
        );
        assert_eq!(None, arc1.intersection(arc1));
        assert_eq!(None, arc1.intersection(arc2));
    }

    fn assert_intersection(expected: NVector, arc1: MinorArc, arc2: MinorArc) {
        let opt_i: Option<crate::NVector> = arc1.intersection(arc2);

        assert!(opt_i.is_some());
        let i = opt_i.unwrap();
        assert_nv_eq_d7(expected, i);

        // intersection is on both minor arc
        assert_eq!(Side::Collinear, Sphere::side(i, arc1.start(), arc1.end()));
        assert_eq!(Side::Collinear, Sphere::side(i, arc2.start(), arc2.end()));
    }

    // projection
    #[test]
    fn projection_inside_minor_arc() {
        let start = NVector::from_lat_long_degrees(53.3206, -1.7297);
        let end = NVector::from_lat_long_degrees(53.1887, 0.1334);
        let pt = NVector::from_lat_long_degrees(53.2611, -0.7972);
        let o_p = MinorArc::new(start, end).projection(pt);
        assert!(o_p.is_some());
        let p = o_p.unwrap();
        assert_nv_eq_d7(NVector::from_lat_long_degrees(53.2583533, -0.7977434), p);
        assert_eq!(
            Sphere::EARTH
                .cross_track_distance(pt, GreatCircle::new(end, start))
                .round_mm(),
            Sphere::EARTH.distance(p, pt).round_mm()
        );
    }

    #[test]
    fn projection_north_pole() {
        let start = NVector::from_lat_long_degrees(0.0, -10.0);
        let end = NVector::from_lat_long_degrees(0.0, 10.0);
        let a: Option<NVector> =
            MinorArc::new(start, end).projection(NVector::from_lat_long_degrees(90.0, 0.0));
        assert!(a.is_some());
        assert_eq!(
            Angle::from_degrees(0.0),
            LatLong::from_nvector(a.unwrap()).latitude()
        );
    }

    #[test]
    fn projection_on_end() {
        let start = NVector::from_lat_long_degrees(54.0, 15.0);
        let end = NVector::from_lat_long_degrees(54.0, 20.0);
        assert_opt_nv_eq_d7(end, MinorArc::new(start, end).projection(end));
    }

    #[test]
    fn projection_on_start() {
        let start = NVector::from_lat_long_degrees(54.0, 15.0);
        let end = NVector::from_lat_long_degrees(54.0, 20.0);
        assert_opt_nv_eq_d7(start, MinorArc::new(start, end).projection(start));
    }

    #[test]
    fn projection_outside_minor_arc_after() {
        let start = NVector::from_lat_long_degrees(54.0, 15.0);
        let end = NVector::from_lat_long_degrees(54.0, 20.0);
        let p = NVector::from_lat_long_degrees(54.0, 25.0);
        assert!(MinorArc::new(start, end).projection(p).is_none());
    }

    #[test]
    fn projection_outside_minor_arc_before() {
        let start = NVector::from_lat_long_degrees(54.0, 15.0);
        let end = NVector::from_lat_long_degrees(54.0, 20.0);
        let p = NVector::from_lat_long_degrees(54.0, 10.0);
        assert!(MinorArc::new(start, end).projection(p).is_none());
    }

    #[test]
    fn projection_south_pole() {
        let start = NVector::from_lat_long_degrees(0.0, -10.0);
        let end = NVector::from_lat_long_degrees(0.0, 10.0);
        let a: Option<NVector> =
            MinorArc::new(start, end).projection(NVector::from_lat_long_degrees(-90.0, 0.0));
        assert!(a.is_some());
        assert_eq!(
            Angle::from_degrees(0.0),
            LatLong::from_nvector(a.unwrap()).latitude()
        );
    }

    #[test]
    fn projection_nearly_perpendicular_null_island() {
        let start = NVector::from_lat_long_degrees(80.0, -90.0);
        let end = NVector::from_lat_long_degrees(80.0, 90.0);
        // minor arc normal should be (-1, 0, 0) but due to floating point precision it is not exactly that
        // value, hence (0, 0) is not exactly perpendicular.
        assert_opt_nv_eq_d7(
            NVector::from_lat_long_degrees(90.0, 0.0),
            MinorArc::new(start, end).projection(NVector::from_lat_long_degrees(0.0, 0.0)),
        );
    }

    #[test]
    fn triage_on_minor_arc_start_or_end() {
        let start = NVector::from_lat_long_degrees(54.0, 154.0);
        let end = NVector::from_lat_long_degrees(54.1, 154.1);
        let a = MinorArc::new(start, end);
        assert_eq!(
            OnMinorArcLocation::Endpoint,
            a.triage_on_minor_arc(start.as_vec3())
        );
        assert_eq!(
            OnMinorArcLocation::Endpoint,
            a.triage_on_minor_arc(end.as_vec3())
        );
        assert_eq!(
            Side::Collinear,
            side(start.as_vec3(), a.normal(), start.as_vec3())
        );
        assert_eq!(
            Side::Collinear,
            side(end.as_vec3(), a.normal(), end.as_vec3())
        );
    }

    #[test]
    fn relate_intersect_reuses_intersection_nominal_case() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(-36.0, 143.0),
            NVector::from_lat_long_degrees(-34.0, 145.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(-34.0, 143.0),
            NVector::from_lat_long_degrees(-36.0, 145.0),
        );
        assert_relation(
            MinorArcRelation::Intersecting(NVector::from_lat_long_degrees(-35.0163245, 144.0)),
            arc1.relate(arc2),
        );
    }

    #[test]
    fn relate_touch_reuses_intersection_at_end_case() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 20.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(10.0, 20.0),
            NVector::from_lat_long_degrees(-10.0, 20.0),
        );
        assert_relation(
            MinorArcRelation::Touching(NVector::from_lat_long_degrees(0.0, 20.0)),
            arc1.relate(arc2),
        );
    }

    #[test]
    fn relate_touch_reuses_intersection_at_shared_case() {
        let shared = NVector::from_lat_long_degrees(-25.0, 130.0);
        let arc1 = MinorArc::new(
            shared,
            NVector::from_lat_long_degrees(-24.950243870277777, 133.85817408527777),
        );
        let arc2 = MinorArc::new(
            shared,
            NVector::from_lat_long_degrees(-25.857954033055556, 133.75470594055557),
        );
        assert!(matches!(arc1.relate(arc2), MinorArcRelation::Touching(_)));
    }

    #[test]
    fn relate_disjoint_reuses_no_intersection_case() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(45.0, 0.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 90.0),
            NVector::from_lat_long_degrees(45.0, 90.0),
        );
        assert_eq!(MinorArcRelation::Disjoint, arc1.relate(arc2));
    }

    /// Where `relate` genuinely goes beyond `intersection`: an arc related to itself is
    /// obviously a complete overlap, whereas `intersection` reports it as no intersection at
    /// all (see `intersection_eq`).
    #[test]
    fn relate_overlap_where_intersection_reports_none_for_identical_arc() {
        let arc = MinorArc::new(
            NVector::from_lat_long_degrees(54.0, 154.0),
            NVector::from_lat_long_degrees(-54.0, 154.0),
        );
        assert!(arc.intersection(arc).is_none()); // sanity-check against the existing behaviour
        assert_relation(MinorArcRelation::Overlapping(arc), arc.relate(arc));
    }

    /// Same physical arc, reversed -- `intersection_opposite` also reports None for this.
    #[test]
    fn relate_overlap_where_intersection_reports_none_for_reversed_arc() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(54.0, 154.0),
            NVector::from_lat_long_degrees(-54.0, 154.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(-54.0, 154.0),
            NVector::from_lat_long_degrees(54.0, 154.0),
        );
        assert!(arc1.intersection(arc2).is_none());
        assert!(matches!(
            arc1.relate(arc2),
            MinorArcRelation::Overlapping(_)
        ));
    }

    #[test]
    fn relate_overlap_for_partial_collinear_overlap() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 10.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 5.0),
            NVector::from_lat_long_degrees(0.0, 15.0),
        );
        let e_arc = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 5.0),
            NVector::from_lat_long_degrees(0.0, 10.0),
        );
        assert_relation(MinorArcRelation::Overlapping(e_arc), arc1.relate(arc2));
    }

    #[test]
    fn relate_touch_for_collinear_arcs_sharing_one_endpoint() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 10.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 10.0),
            NVector::from_lat_long_degrees(0.0, 20.0),
        );
        assert!(matches!(arc1.relate(arc2), MinorArcRelation::Touching(_)));
    }

    #[test]
    fn relate_disjoint_for_collinear_but_disjoint_arcs() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 10.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 15.0),
            NVector::from_lat_long_degrees(0.0, 20.0),
        );
        assert_eq!(MinorArcRelation::Disjoint, arc1.relate(arc2));
    }

    #[test]
    fn relate_overlap_is_oriented_like_self_even_when_found_points_arrive_reversed() {
        let arc1 = MinorArc::new(
            NVector::from_lat_long_degrees(54.0, 154.0),
            NVector::from_lat_long_degrees(-54.0, 154.0),
        );
        let arc2 = MinorArc::new(
            NVector::from_lat_long_degrees(-54.0, 154.0),
            NVector::from_lat_long_degrees(54.0, 154.0),
        );
        assert_relation(MinorArcRelation::Overlapping(arc1), arc1.relate(arc2));
    }

    fn assert_relation(expected: MinorArcRelation, actual: MinorArcRelation) {
        match (expected, actual) {
            (MinorArcRelation::Disjoint, MinorArcRelation::Disjoint) => {
                // passed!
            }
            (MinorArcRelation::Intersecting(e), MinorArcRelation::Intersecting(a)) => {
                assert_nv_eq_d7(e, a);
            }
            (MinorArcRelation::Touching(e), MinorArcRelation::Touching(a)) => {
                assert_nv_eq_d7(e, a);
            }
            (MinorArcRelation::Overlapping(e), MinorArcRelation::Overlapping(a)) => {
                assert_nv_eq_d7(e.start(), a.start());
                assert_nv_eq_d7(e.end(), a.end());
            }
            (e, a) => {
                panic!("expected {e:?}, got {a:?}");
            }
        };
    }
}
