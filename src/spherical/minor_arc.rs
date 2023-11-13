use crate::{
    numbers::{eq_zero, gte, lte},
    Angle, NVector, Vec3,
};

use super::base::angle_radians_between;

/// Oriented minor arc of a great circle between two positions: shortest path between positions
/// on a great circle.
#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct MinorArc {
    start: NVector,
    end: NVector,
    normal: Vec3,
}

impl MinorArc {
    /// Creates a new minor arc from the given start and end positions.
    ///
    /// Note: if both start and end positions are equal or the antipode of one another, then an
    /// arbitrary minor arc is returned - since an infinity of minor arcs exist - see [is_great_cirle](crate::spherical::Sphere::is_great_circle).
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
        let i = self.normal.stable_cross_prod_unit(other.normal);
        if i == Vec3::ZERO {
            // equal or opposite minor arcs: no intersection
            None
        } else {
            // select nearest intersection to start of first minor arc.
            let potential = if self.start.as_vec3().dot_prod(i) > 0.0 {
                i
            } else {
                // antipode of i.
                -i
            };

            if self.contains_vec3(potential) && other.contains_vec3(potential) {
                Some(NVector::new(potential))
            } else {
                None
            }
        }
    }

    /// Computes the projection of the given position on this minor arc. Returns [None] if the projection is not
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
            if self.contains_vec3(proj) {
                Some(NVector::new(proj))
            } else {
                None
            }
        }
    }

    /// Determines whether this minor arc contains the given point.
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
    /// assert!(ma.contains_point(NVector::from_lat_long_degrees(0.0, 5.0)));
    /// assert!(!ma.contains_point(NVector::from_lat_long_degrees(1.0, 5.0)));
    /// assert!(!ma.contains_point(NVector::from_lat_long_degrees(0.0, 11.0)));
    /// assert!(!ma.contains_point(NVector::from_lat_long_degrees(0.0, -11.0)));
    /// ```
    pub fn contains_point(&self, p: NVector) -> bool {
        let v = p.as_vec3();
        eq_zero(v.dot_prod(self.normal)) && self.contains_vec3(v)
    }

    /// Determines whether p if right of (negative integer), left of (positive integer) or on this
    /// minor arc (zero).
    ///
    /// This is similar to [side(p, self.start, self.end)](crate::spherical::Sphere::side) but avoids the calculation of the orthogonal
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
    /// # Exmaples
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

    /// Determines whether this minor arc contains the given point which is assumed to be on the great circle.
    fn contains_vec3(&self, v: Vec3) -> bool {
        // v is left of (normal, start)
        // and
        // v if right of (normal, end)

        // effectively this is base#side(v, normal, start) >=0 && base#side(v, normal, end) <= 0
        // however since normal is never eq or opposite to start or end, using
        // Vec3::cross_prod_unit is enough (base#side make no assumption about it's inputs and
        // therefore calls the more expensive function Vec3::orthogonal_to).
        let start = self.start.as_vec3();
        let end = self.end.as_vec3();
        let n = self.normal;
        gte(v.dot_prod(n.cross_prod_unit(start)), 0.0)
            && lte(v.dot_prod(n.cross_prod_unit(end)), 0.0)
    }
}

#[cfg(test)]
mod tests {

    use crate::{
        positions::{assert_nv_eq_d7, assert_opt_nv_eq_d7},
        spherical::{GreatCircle, MinorArc, Sphere},
        Angle, LatLong, Length, NVector, Vec3,
    };

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
            Sphere::EARTH.destination_pos(arc1_start, Angle::from_degrees(45.0), tenth_of_mm);

        let arc1 = MinorArc::new(arc1_start, arc1_end);

        let arc1_midpoint = Sphere::interpolated_pos(arc1_start, arc1_end, 0.5).unwrap();

        let arc2_start =
            Sphere::EARTH.destination_pos(arc1_midpoint, Angle::from_degrees(315.0), tenth_of_mm);
        let arc2_end =
            Sphere::EARTH.destination_pos(arc2_start, Angle::from_degrees(135.0), tenth_of_mm);
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
        assert_eq!(0, Sphere::side(i, arc1.start(), arc1.end()));
        assert_eq!(0, Sphere::side(i, arc2.start(), arc2.end()));
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
}
