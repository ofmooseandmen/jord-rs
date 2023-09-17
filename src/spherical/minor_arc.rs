use crate::{HorizontalPosition, Length, Vec3};

use super::{along_track_distance, are_ordered, orthogonal};

/// Oriented minor arc of a great circle between two positions: shortest path between positions
/// on a great circle.
#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct MinorArc<T>
where
    T: HorizontalPosition,
{
    start: T,
    end: T,
    normal: Vec3,
}

impl<T: HorizontalPosition> MinorArc<T> {
    /// Creates a new minor arc from the given start and end positions.
    ///
    /// Note: if both start and end positions are equal or the antipode of one another, then an
    /// arbitrary minor arc is returned - since an infinity of minor arcs exist (see [crate::spherical::is_great_circle]).
    pub fn new(start: T, end: T) -> Self {
        let normal = orthogonal(start.as_nvector(), end.as_nvector());
        MinorArc { start, end, normal }
    }

    /// Returns the start position of this minor arc.
    pub fn start(&self) -> T {
        self.start
    }

    /// Returns the end position of this minor arc.
    pub fn end(&self) -> T {
        self.end
    }

    /// Computes how far the given position is along a path described by this minor arc: if a
    /// perpendicular is drawn from the position to the path, the along-track distance is the
    /// signed distance from the start point to where the perpendicular crosses the path.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Length, HorizontalPosition, Point, Vec3, IUGG_EARTH_RADIUS};
    /// use jord::spherical::MinorArc;
    ///
    /// let p = Point::from_lat_long_degrees(53.2611, -0.7972);
    /// let start = Point::from_lat_long_degrees(53.3206, -1.7297);
    /// let end = Point::from_lat_long_degrees(53.1887, 0.1334);
    /// let d = MinorArc::new(start, end).along_track_distance(p, IUGG_EARTH_RADIUS);
    /// assert_eq!(d.round_mm(), Length::from_metres(62331.501));
    ///
    /// // or alternatively with Vec3:
    ///
    /// let p = Vec3::from_lat_long_degrees(53.2611, -0.7972);
    /// let start = Vec3::from_lat_long_degrees(53.3206, -1.7297);
    /// let end = Vec3::from_lat_long_degrees(53.1887, 0.1334);
    /// let d = MinorArc::new(start, end).along_track_distance(p, IUGG_EARTH_RADIUS);
    /// assert_eq!(d.round_mm(), Length::from_metres(62331.501));
    /// ```
    ///
    /// See also [crate::spherical::Navigation::along_track_distance]
    pub fn along_track_distance(&self, pos: T, radius: Length) -> Length {
        along_track_distance(
            pos.as_nvector(),
            self.start.as_nvector(),
            self.normal,
            radius,
        )
    }

    /// Computes the intersection point between this minor arc and the given minor arc, if there is an
    /// intersection.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Length, HorizontalPosition, Point, Vec3};
    /// use jord::spherical::MinorArc;
    ///
    /// let ma1 = MinorArc::new(
    ///     Point::from_lat_long_degrees(-10.0, 0.0),
    ///     Point::from_lat_long_degrees(10.0, 0.0)
    /// );
    /// let ma2 = MinorArc::new(
    ///     Point::from_lat_long_degrees(0.0, -10.0),
    ///     Point::from_lat_long_degrees(0.0, 10.0)
    /// );
    /// let i = ma1.intersection(ma2);
    /// assert_eq!(i, Some(Point::from_lat_long_degrees(0.0, 0.0)));
    ///
    /// // or alternatively with Vec3:
    ///
    /// let ma1 = MinorArc::new(
    ///     Vec3::from_lat_long_degrees(-10.0, 0.0),
    ///     Vec3::from_lat_long_degrees(10.0, 0.0)
    /// );
    /// let ma2 = MinorArc::new(
    ///     Vec3::from_lat_long_degrees(0.0, -10.0),
    ///     Vec3::from_lat_long_degrees(0.0, 10.0)
    /// );
    /// let i = ma1.intersection(ma2);
    /// assert_eq!(i, Some(Vec3::new(1.0, 0.0, 0.0)));
    /// ```
    pub fn intersection(&self, other: MinorArc<T>) -> Option<T> {
        let i = orthogonal(self.normal, other.normal);
        // select nearest intersection to start of first minor arc.
        let potential = if self.start.as_nvector().dot_prod(i) > 0.0 {
            i
        } else {
            // antipode of i.
            i * -1.0
        };

        if are_ordered(self.start.as_nvector(), potential, self.end.as_nvector())
            && are_ordered(other.start.as_nvector(), potential, other.end.as_nvector())
        {
            Some(T::from_nvector(potential))
        } else {
            None
        }
    }

    /// Computes the projection of the given position on the given minor arc. Returns [None] if the projection is not
    /// within the minor arc (including start and end). If the given position is strictly "perpendicular" to the
    /// given minor arc, this method arbitrarily returns the start (p can be projected anywhere on the minor arc).
    ///
    /// # Examples:
    ///
    /// ```
    /// use jord::{HorizontalPosition, Point, Vec3};
    /// use jord::spherical::{MinorArc, Navigation};
    ///
    /// let ma = MinorArc::new(
    ///     Point::from_lat_long_degrees(0.0, -10.0),
    ///     Point::from_lat_long_degrees(0.0, 10.0)
    /// );
    ///
    /// let o_p = ma.projection(Point::from_lat_long_degrees(1.0, 0.0));
    /// assert!(o_p.is_some());
    /// assert_eq!(Point::from_lat_long_degrees(0.0, 0.0), o_p.unwrap().normalised_d7());
    ///
    /// // or alternatively with Vec3:
    ///
    /// let ma = MinorArc::new(
    ///     Vec3::from_lat_long_degrees(0.0, -10.0),
    ///     Vec3::from_lat_long_degrees(0.0, 10.0)
    /// );
    ///
    /// let o_p = ma.projection(Vec3::from_lat_long_degrees(1.0, 0.0));
    /// assert!(o_p.is_some());
    /// assert_eq!(Vec3::from_lat_long_degrees(0.0, 0.0), o_p.unwrap().normalised_d7());
    /// ```
    pub fn projection(&self, pos: T) -> Option<T> {
        let n1 = self.normal;
        let n2 = pos.as_nvector().stable_cross_prod_unit(n1);
        if n2 == Vec3::ZERO {
            Some(self.start)
        } else {
            let proj = orthogonal(n1, n2);
            if are_ordered(self.start.as_nvector(), proj, self.end.as_nvector()) {
                Some(T::from_nvector(proj))
            } else {
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {

    use crate::{
        spherical::{side, GreatCircle, MinorArc, Navigation},
        Angle, HorizontalPosition, Length, Point, Vec3, IUGG_EARTH_RADIUS,
    };

    #[test]
    fn along_track_distance_minor_arc_length() {
        let s = Vec3::from_lat_long_degrees(53.2611, -0.7972);
        let e = Vec3::from_lat_long_degrees(53.1887, 0.1334);
        let ma = MinorArc::new(s, e);
        assert_eq!(
            s.distance(e, IUGG_EARTH_RADIUS).round_mm(),
            ma.along_track_distance(e, IUGG_EARTH_RADIUS).round_mm()
        );
    }

    #[test]
    fn along_track_distance_negative() {
        let p = Vec3::from_lat_long_degrees(53.3206, -1.7297);
        let ma = MinorArc::new(
            Vec3::from_lat_long_degrees(53.2611, -0.7972),
            Vec3::from_lat_long_degrees(53.1887, 0.1334),
        );
        assert_eq!(
            Length::from_metres(-62_329.232),
            ma.along_track_distance(p, IUGG_EARTH_RADIUS).round_mm()
        );
    }

    #[test]
    fn along_track_distance_positive() {
        let p = Vec3::from_lat_long_degrees(53.2611, -0.7972);
        let ma = MinorArc::new(
            Vec3::from_lat_long_degrees(53.3206, -1.7297),
            Vec3::from_lat_long_degrees(53.1887, 0.1334),
        );
        assert_eq!(
            Length::from_metres(62_331.501),
            ma.along_track_distance(p, IUGG_EARTH_RADIUS).round_mm()
        );
    }

    #[test]
    fn along_track_distance_zero() {
        let p = Vec3::from_lat_long_degrees(53.2611, -0.7972);
        let ma = MinorArc::new(p, Vec3::from_lat_long_degrees(53.1887, 0.1334));
        assert_eq!(
            Length::ZERO,
            ma.along_track_distance(p, IUGG_EARTH_RADIUS).round_mm()
        );
    }

    // intersection

    #[test]
    fn intersection_arc_across_equator() {
        let arc1 = MinorArc::new(
            Vec3::from_lat_long_degrees(54.0, 154.0),
            Vec3::from_lat_long_degrees(-54.0, 154.0),
        );

        let arc2 = MinorArc::new(
            Vec3::from_lat_long_degrees(53.0, 153.0),
            Vec3::from_lat_long_degrees(53.0, 155.0),
        );

        assert_intersection(
            Vec3::new(-0.5408552101001728, 0.26379271166149, 0.7986795646451562),
            arc1,
            arc2,
        );
    }

    #[test]
    fn intersection_at_end() {
        let arc1 = MinorArc::new(
            Vec3::from_lat_long_degrees(0.0, 0.0),
            Vec3::from_lat_long_degrees(0.0, 20.0),
        );

        let arc2 = MinorArc::new(
            Vec3::from_lat_long_degrees(10.0, 20.0),
            Vec3::from_lat_long_degrees(-10.0, 20.0),
        );

        assert_intersection(Vec3::from_lat_long_degrees(0.0, 20.0), arc1, arc2);
    }

    #[test]
    fn intersection_at_start() {
        let arc1 = MinorArc::new(
            Vec3::from_lat_long_degrees(0.0, 0.0),
            Vec3::from_lat_long_degrees(0.0, 20.0),
        );

        let arc2 = MinorArc::new(
            Vec3::from_lat_long_degrees(10.0, 0.0),
            Vec3::from_lat_long_degrees(-10.0, 0.0),
        );

        assert_intersection(Vec3::from_lat_long_degrees(0.0, 0.0), arc1, arc2);
    }

    #[test]
    fn intersection_close() {
        let arc1 = MinorArc::new(
            Point::from_lat_long_degrees(-27.1789705075, 152.3083728075),
            Point::from_lat_long_degrees(-27.0741667000, 152.2163889000),
        );
        let arc2 = MinorArc::new(
            Point::from_lat_long_degrees(-27.1245578000, 152.1506886000),
            Point::from_lat_long_degrees(-27.0741667000, 152.2163889000),
        );

        assert_intersection(
            Point::from_lat_long_degrees(-27.0741667, 152.2163889),
            arc1,
            arc2,
        );
    }

    #[test]
    fn intersection_nominal() {
        let arc1 = MinorArc::new(
            Point::from_lat_long_degrees(-36.0, 143.0),
            Point::from_lat_long_degrees(-34.0, 145.0),
        );
        let arc2 = MinorArc::new(
            Point::from_lat_long_degrees(-34.0, 143.0),
            Point::from_lat_long_degrees(-36.0, 145.0),
        );
        assert_intersection(Point::from_lat_long_degrees(-35.0163245, 144.0), arc1, arc2);
    }

    #[test]
    fn intersection_null_island() {
        let arc1 = MinorArc::new(
            Point::from_lat_long_degrees(0.0, -1.0),
            Point::from_lat_long_degrees(0.0, 1.0),
        );
        let arc2 = MinorArc::new(
            Point::from_lat_long_degrees(-1.0, 0.0),
            Point::from_lat_long_degrees(1.0, 0.0),
        );
        assert_intersection(Point::from_lat_long_degrees(0.0, 0.0), arc1, arc2);
    }

    #[test]
    fn intersection_pole() {
        let arc1 = MinorArc::new(
            Point::from_lat_long_degrees(45.0, 0.0),
            Point::from_lat_long_degrees(45.0, 180.0),
        );
        let arc2 = MinorArc::new(
            Point::from_lat_long_degrees(45.0, 90.0),
            Point::from_lat_long_degrees(45.0, 270.0),
        );
        assert_intersection(Point::NORTH_POLE, arc1, arc2);
    }

    #[test]
    fn intersection_small_minor_arc() {
        let arc1 = MinorArc::new(
            Point::from_lat_long_degrees(-20.8464124400, 123.2066292450),
            Point::from_lat_long_degrees(-20.8463888889, 123.2066666667),
        );
        let arc2 = MinorArc::new(
            Point::from_lat_long_degrees(-20.3716666667, 122.2811111111),
            Point::from_lat_long_degrees(-21.5219444444, 124.5511111111),
        );
        assert!(arc1.start().distance(arc1.end(), IUGG_EARTH_RADIUS) < Length::from_metres(5.0));

        assert_intersection(
            Point::from_lat_long_degrees(-20.8464124, 123.2066292),
            arc1,
            arc2,
        );
    }

    #[test]
    fn intersection_very_small_minor_arc() {
        let tenth_of_mm = Length::from_metres(1e-4);
        let arc1_start = Point::from_lat_long_degrees(-32.7929069956, 135.4840669972);
        let arc1_end =
            arc1_start.destination(Angle::from_degrees(45.0), tenth_of_mm, IUGG_EARTH_RADIUS);
        let arc1 = MinorArc::new(arc1_start, arc1_end);

        let arc1_midpoint = arc1_start.interpolated(arc1_end, 0.5).unwrap();

        let arc2_start =
            arc1_midpoint.destination(Angle::from_degrees(315.0), tenth_of_mm, IUGG_EARTH_RADIUS);
        let arc2_end =
            arc2_start.destination(Angle::from_degrees(135.0), tenth_of_mm, IUGG_EARTH_RADIUS);
        let arc2 = MinorArc::new(arc2_start, arc2_end);

        assert_intersection(arc1_midpoint, arc1, arc2);
    }

    #[test]
    fn no_intersection() {
        let arc1 = MinorArc::new(
            Point::from_lat_long_degrees(0.0, 0.0),
            Point::from_lat_long_degrees(45.0, 0.0),
        );
        let arc2 = MinorArc::new(
            Point::from_lat_long_degrees(0.0, 90.0),
            Point::from_lat_long_degrees(45.0, 90.0),
        );
        assert_eq!(None, arc1.intersection(arc2));
    }

    #[test]
    fn no_intersection_candidate_close_to_first() {
        let arc1 = MinorArc::new(
            Point::from_lat_long_degrees(54.0, 178.8),
            Point::from_lat_long_degrees(54.0, -179.8),
        );
        let arc2 = MinorArc::new(
            Point::from_lat_long_degrees(-80.0, 179.0),
            Point::from_lat_long_degrees(-85.0, 179.0),
        );
        assert_eq!(None, arc1.intersection(arc2));
    }

    #[test]
    fn no_intersection_close_first_arc() {
        let arc1 = MinorArc::new(
            Point::from_lat_long_degrees(-27.7022222000, 152.5372222000),
            Point::from_lat_long_degrees(-27.4319444000, 152.4188889000),
        );
        let arc2 = MinorArc::new(
            Point::from_lat_long_degrees(-27.3874939000, 152.4658169000),
            Point::from_lat_long_degrees(-27.3518653000, 152.5214517000),
        );
        assert_eq!(None, arc1.intersection(arc2));
    }

    #[test]
    fn no_intersection_close_second_arc() {
        let arc1 = MinorArc::new(
            Point::from_lat_long_degrees(-27.7022222000, 152.5372222000),
            Point::from_lat_long_degrees(-27.4319444000, 152.4188889000),
        );
        let arc2 = MinorArc::new(
            Point::from_lat_long_degrees(-27.4754111000, 152.7457194000),
            Point::from_lat_long_degrees(-27.4733058000, 152.6958286000),
        );
        assert_eq!(None, arc1.intersection(arc2));
    }

    #[test]
    fn no_intersection_long_arcs_far_apart() {
        let arc1 = MinorArc::new(
            Point::from_lat_long_degrees(9.0, -83.0),
            Point::from_lat_long_degrees(-33.8179213708, 112.4433954286),
        );
        let arc2 = MinorArc::new(
            Point::from_lat_long_degrees(10.0, 55.0),
            Point::from_lat_long_degrees(10.0, 179.0),
        );
        assert_eq!(None, arc1.intersection(arc2));
    }

    #[test]
    fn no_intersection_parallel_minor_arcs() {
        let arc1 = MinorArc::new(
            Point::from_lat_long_degrees(0.0, 0.0),
            Point::from_lat_long_degrees(45.0, 0.0),
        );
        let arc2 = MinorArc::new(
            Point::from_lat_long_degrees(46.0, 0.0),
            Point::from_lat_long_degrees(48.0, 0.0),
        );
        assert_eq!(None, arc1.intersection(arc1));
        assert_eq!(None, arc1.intersection(arc2));
    }

    fn assert_intersection<T: HorizontalPosition>(
        expected: T,
        arc1: MinorArc<T>,
        arc2: MinorArc<T>,
    ) {
        let i = arc1.intersection(arc2);

        assert!(i.is_some());
        assert_eq!(expected.normalised_d7(), i.unwrap().normalised_d7());

        // intersection is on both minor arc
        let v = i.unwrap().as_nvector();
        assert_eq!(
            0,
            side(v, arc1.start().as_nvector(), arc1.end().as_nvector())
        );
        assert_eq!(
            0,
            side(v, arc2.start().as_nvector(), arc2.end().as_nvector())
        );
    }

    // projection

    #[test]
    fn projection_inside_minor_arc() {
        let start = Point::from_lat_long_degrees(53.3206, -1.7297);
        let end = Point::from_lat_long_degrees(53.1887, 0.1334);
        let pt = Point::from_lat_long_degrees(53.2611, -0.7972);
        let o_p = MinorArc::new(start, end).projection(pt);
        assert!(o_p.is_some());
        let p = o_p.unwrap();
        assert_eq!(
            Point::from_lat_long_degrees(53.2583533, -0.7977434),
            p.normalised_d7()
        );
        assert_eq!(
            GreatCircle::new(end, start)
                .cross_track_distance(pt, IUGG_EARTH_RADIUS)
                .round_mm(),
            p.distance(pt, IUGG_EARTH_RADIUS).round_mm()
        );
    }

    #[test]
    fn projection_north_pole() {
        let start = Point::from_lat_long_degrees(0.0, -10.0);
        let end = Point::from_lat_long_degrees(0.0, 10.0);
        assert_eq!(
            Some(start),
            MinorArc::new(start, end).projection(Point::NORTH_POLE)
        );
    }

    #[test]
    fn projection_on_end() {
        let start = Point::from_lat_long_degrees(54.0, 15.0);
        let end = Point::from_lat_long_degrees(54.0, 20.0);
        assert_eq!(
            Some(end.normalised_d7()),
            MinorArc::new(start, end)
                .projection(end)
                .map(|p| p.normalised_d7())
        );
    }

    #[test]
    fn projection_on_start() {
        let start = Point::from_lat_long_degrees(54.0, 15.0);
        let end = Point::from_lat_long_degrees(54.0, 20.0);
        assert_eq!(
            Some(start.normalised_d7()),
            MinorArc::new(start, end)
                .projection(start)
                .map(|p| p.normalised_d7())
        );
    }

    #[test]
    fn projection_outside_minor_arc_after() {
        let start = Point::from_lat_long_degrees(54.0, 15.0);
        let end = Point::from_lat_long_degrees(54.0, 20.0);
        let p = Point::from_lat_long_degrees(54.0, 25.0);
        assert!(MinorArc::new(start, end).projection(p).is_none());
    }

    #[test]
    fn projection_outside_minor_arc_before() {
        let start = Point::from_lat_long_degrees(54.0, 15.0);
        let end = Point::from_lat_long_degrees(54.0, 20.0);
        let p = Point::from_lat_long_degrees(54.0, 10.0);
        assert!(MinorArc::new(start, end).projection(p).is_none());
    }

    #[test]
    fn projection_south_pole() {
        let start = Point::from_lat_long_degrees(0.0, -10.0);
        let end = Point::from_lat_long_degrees(0.0, 10.0);
        assert_eq!(
            Some(start),
            MinorArc::new(start, end).projection(Point::SOUTH_POLE)
        );
    }

    #[test]
    fn projection_nearly_perpendicular_null_island() {
        let start = Point::from_lat_long_degrees(80.0, -90.0);
        let end = Point::from_lat_long_degrees(80.0, 90.0);
        // minor arc normal should be (-1, 0, 0) but due to floating point precision it is not exactly that
        // value, hence (0, 0) is not exactly perpendicular.
        assert_eq!(
            Some(Point::NORTH_POLE),
            MinorArc::new(start, end).projection(Point::from_lat_long_degrees(0.0, 0.0))
        );
    }
}
