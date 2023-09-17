use super::{along_track_distance, angle_radians_between, easting, is_great_circle};
use crate::{Angle, HorizontalPosition, Length, Point, Vec3};
use std::f64::consts::PI;

use super::GreatCircle;

/// Common algorithms applicable to spherical models. If performance is paramount prefer using
/// `Vec3` over `Point`.
pub trait Navigation: HorizontalPosition {
    /// Computes how far this position is along a path starting at the given position and heading on
    /// the given bearing: if a perpendicular is drawn from the position to the path, the along-track
    /// distance is the signed distance from the start point to where the perpendicular crosses the path.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, Length, HorizontalPosition, Point, Vec3, IUGG_EARTH_RADIUS};
    /// use jord::spherical::Navigation;
    ///
    /// let bearing = Angle::from_degrees(96.0017325);
    ///
    /// let p = Point::from_lat_long_degrees(53.2611, -0.7972);
    /// let start = Point::from_lat_long_degrees(53.3206, -1.7297);
    /// let d = p.along_track_distance(start, bearing, IUGG_EARTH_RADIUS);
    /// assert_eq!(d, Length::from_kilometres(62.33150111219829));
    ///
    /// // or alternatively with Vec3:
    ///
    /// let p = Vec3::from_lat_long_degrees(53.2611, -0.7972);
    /// let start = Vec3::from_lat_long_degrees(53.3206, -1.7297);
    /// let d = p.along_track_distance(start, bearing, IUGG_EARTH_RADIUS);
    /// assert_eq!(d, Length::from_kilometres(62.33150111219829));
    /// ```
    ///
    /// See also [crate::spherical::MinorArc::along_track_distance]
    fn along_track_distance(&self, start: Self, bearing: Angle, radius: Length) -> Length {
        let gc = GreatCircle::from_heading(start, bearing);
        along_track_distance(self.as_nvector(), start.as_nvector(), gc.normal(), radius)
    }

    /// Computes the angle between this position and the given position, which is also equal to the distance
    /// between these positions on the unit sphere.
    fn angle(&self, other: Self) -> Angle {
        Angle::from_radians(angle_radians_between(
            self.as_nvector(),
            other.as_nvector(),
            None,
        ))
    }
    /// Computes the signed distance from this position to the great circle passing
    /// by the given position and heading on the given bearing.
    fn cross_track_distance(&self, other: Self, bearing: Angle, radius: Length) -> Length {
        GreatCircle::from_heading(other, bearing).cross_track_distance(*self, radius)
    }

    /// Computes the destination position from this position having travelled the given distance on the given
    /// initial bearing (compass angle) (bearing will normally vary before destination is reached).
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f64::consts::PI;
    /// use jord::{Angle, Length, HorizontalPosition, Point, Vec3, IUGG_EARTH_RADIUS};
    /// use jord::spherical::Navigation;
    ///
    /// let distance = IUGG_EARTH_RADIUS * PI / 4.0;
    /// let p = Point::from_lat_long_degrees(90.0, 0.0);
    /// let dest = p.destination(Angle::from_degrees(180.0), distance, IUGG_EARTH_RADIUS);
    ///
    /// assert_eq!(Point::from_lat_long_degrees(45.0, 0.0), dest.normalised_d7());
    /// ```
    fn destination(&self, bearing: Angle, distance: Length, radius: Length) -> Self {
        if distance == Length::ZERO {
            *self
        } else {
            let v0 = self.as_nvector();
            // east direction vector at p
            let ed = easting(v0);
            // north direction vector at p
            let nd = v0.cross_prod(ed);
            // central angle
            let ta = distance.as_metres() / radius.as_metres();
            let bearing_radians = bearing.as_radians();
            // unit vector in the direction of the azimuth
            let dir = nd * bearing_radians.cos() + ed * bearing_radians.sin();
            let dv = (v0 * ta.cos() + dir * ta.sin()).unit();
            Self::from_nvector(dv)
        }
    }

    /// Computes the surface distance on the great circle between this position and the given position.
    ///
    /// # Examples:
    ///
    /// ```
    /// use jord::{Length, HorizontalPosition, Point, Vec3, IUGG_EARTH_RADIUS};
    /// use jord::spherical::Navigation;
    ///
    /// let d = Point::from_lat_long_degrees(90.0, 0.0).distance(Point::from_lat_long_degrees(-90.0, 0.0), IUGG_EARTH_RADIUS);
    /// assert_eq!(
    ///   Length::from_metres(20_015_089.309),
    ///   d.round_mm()
    /// );
    ///
    /// // or alternatively with Vec3:
    ///
    /// let d = Vec3::from_lat_long_degrees(90.0, 0.0).distance(Vec3::from_lat_long_degrees(-90.0, 0.0), IUGG_EARTH_RADIUS);
    /// assert_eq!(
    ///   Length::from_metres(20_015_089.309),
    ///   d.round_mm()
    /// );
    /// ```
    fn distance(&self, other: Self, radius: Length) -> Length {
        self.angle(other) * radius
    }

    /// Computes the final bearing arriving at the given position from this position in compass angle.
    /// Compass angles are clockwise angles from true north: 0 = north, 90 = east, 180 = south, 270 = west.
    /// The final bearing will differ from the initial bearing by varying degrees according to distance and latitude.
    /// Returns 0 if both positions are equal or the antipode of each other (see [crate::spherical::is_great_circle]).
    /// # Examples:
    ///
    /// ```
    /// use jord::{Angle, HorizontalPosition, Point, Vec3};
    /// use jord::spherical::Navigation;
    ///
    /// assert_eq!(
    ///   Angle::from_degrees(90.0),
    ///   Point::from_lat_long_degrees(0.0, 0.0).final_bearing(Point::from_lat_long_degrees(0.0, 1.0))
    /// );
    ///
    /// // or alternatively with Vec3:
    ///
    /// assert_eq!(
    ///   Angle::from_degrees(90.0),
    ///   Vec3::from_lat_long_degrees(0.0, 0.0).final_bearing(Vec3::from_lat_long_degrees(0.0, 1.0))
    /// );
    /// ```
    fn final_bearing(&self, other: Self) -> Angle {
        if !is_great_circle(*self, other) {
            Angle::ZERO
        } else {
            Angle::from_radians(final_bearing_radians(self.as_nvector(), other.as_nvector()))
                .normalised()
        }
    }

    /// Computes the initial bearing from this position to the given position in compass angle.
    /// Compass angles are clockwise angles from true north: 0 = north, 90 = east, 180 = south, 270 = west.
    /// Returns 0 if both positions are equal or the antipode of each other (see [crate::spherical::is_great_circle]).
    ///
    /// # Examples:
    ///
    /// ```
    /// use jord::{Angle, HorizontalPosition, Point, Vec3};
    /// use jord::spherical::Navigation;
    ///
    /// assert_eq!(
    ///   Angle::from_degrees(270.0),
    ///   Point::from_lat_long_degrees(0.0, 1.0).initial_bearing(Point::from_lat_long_degrees(0.0, 0.0))
    /// );
    ///
    /// // or alternatively with Vec3:
    ///
    /// assert_eq!(
    ///   Angle::from_degrees(270.0),
    ///   Vec3::from_lat_long_degrees(0.0, 1.0).initial_bearing(Vec3::from_lat_long_degrees(0.0, 0.0))
    /// );
    /// ```
    fn initial_bearing(&self, other: Self) -> Angle {
        if !is_great_circle(*self, other) {
            Angle::ZERO
        } else {
            Angle::from_radians(initial_bearing_radians(
                self.as_nvector(),
                other.as_nvector(),
            ))
            .normalised()
        }
    }

    /// Computes the position at given fraction between this position and the given position.
    /// Returns `None` if:
    /// - the fraction is `< 0` or `> 1`,
    /// - this position and the given position are the antipodes of one another.
    fn interpolated(&self, other: Self, f: f64) -> Option<Self> {
        if !(0.0..=1.0).contains(&f) || self.is_antipode(other) {
            None
        } else if f == 0.0 {
            Some(*self)
        } else if f == 1.0 {
            Some(other)
        } else {
            let v0 = self.as_nvector();
            let v1 = other.as_nvector();
            // angular distance in radians multiplied by the fraction: how far from v0.
            let distance_radians = f * angle_radians_between(v0, v1, None);
            //  a vector representing the direction from v0 to v1.
            let dir = (v0.stable_cross_prod(v1)).cross_prod_unit(v0);
            let v = (v0 * distance_radians.cos() + dir * distance_radians.sin()).unit();
            Some(Self::from_nvector(v))
        }
    }
}

impl Navigation for Point {}
impl Navigation for Vec3 {}

fn final_bearing_radians(v1: Vec3, v2: Vec3) -> f64 {
    initial_bearing_radians(v2, v1) + PI
}

fn initial_bearing_radians(v1: Vec3, v2: Vec3) -> f64 {
    // great circle through v1 & v2.
    let gc1 = v1.cross_prod(v2);

    // this is equivalent to -easting(v1), but avoids the creation of
    // an intermediate Vec3.
    // -y if at pole or great circle through v1 & north pole (v x [0, 0, 1])
    let gc2 = if v1.z().abs() == 1.0 {
        Vec3::NEG_UNIT_Y
    } else {
        Vec3::new(v1.y(), -v1.x(), 0.0)
    };
    angle_radians_between(gc1, gc2, Some(v1))
}

#[cfg(test)]
mod tests {

    use std::f64::consts::PI;

    use crate::spherical::side;
    use crate::Angle;
    use crate::HorizontalPosition;
    use crate::Length;
    use crate::Point;
    use crate::IUGG_EARTH_RADIUS;

    use super::Navigation;

    /// destination.

    #[test]
    fn destination_across_date_line() {
        let p = Point::from_lat_long_degrees(0.0, 154.0);
        let d = p
            .destination(
                Angle::from_degrees(90.0),
                Length::from_kilometres(5000.0),
                IUGG_EARTH_RADIUS,
            )
            .normalised_d7();
        let e = Point::from_lat_long_degrees(0.0, -161.0339254);
        assert_eq!(e, d);
    }

    #[test]
    fn destination_from_north_pole() {
        let expected = Point::from_lat_long_degrees(45.0, 0.0);
        let distance = IUGG_EARTH_RADIUS * (PI / 4.0);
        let actual = Point::NORTH_POLE
            .destination(Angle::from_degrees(180.0), distance, IUGG_EARTH_RADIUS)
            .normalised_d7();
        assert_eq!(expected, actual);
    }

    #[test]
    fn destination_from_south_pole() {
        let expected = Point::from_lat_long_degrees(-45.0, 0.0);
        let distance = IUGG_EARTH_RADIUS * (PI / 4.0);
        let actual = Point::SOUTH_POLE
            .destination(Angle::ZERO, distance, IUGG_EARTH_RADIUS)
            .normalised_d7();
        assert_eq!(expected, actual);
    }

    #[test]
    fn destination_negative_distance() {
        let p = Point::from_lat_long_degrees(0.0, 0.0);
        // equivalent of -10 degree of longitude.
        let d = IUGG_EARTH_RADIUS * (-2.0 * PI / 36.0);
        let actual = p
            .destination(Angle::from_degrees(90.0), d, IUGG_EARTH_RADIUS)
            .normalised_d7();
        let expected = Point::from_lat_long_degrees(0.0, -10.0);
        assert_eq!(expected, actual);
    }

    #[test]
    fn destination_travelled_longitude_greater_than_90() {
        let p = Point::from_lat_long_degrees(60.2, 11.1);
        let d = p
            .destination(
                Angle::from_degrees(12.4),
                Length::from_nautical_miles(2000.0),
                IUGG_EARTH_RADIUS,
            )
            .normalised_d7();
        let e = Point::from_lat_long_degrees(82.6380125, 124.1259551);
        assert_eq!(e, d);
    }

    #[test]
    fn destination_zero_distance() {
        let p = Point::from_lat_long_degrees(55.6050, 13.0038);
        assert_eq!(
            p,
            p.destination(
                Angle::from_degrees(96.0217),
                Length::ZERO,
                IUGG_EARTH_RADIUS
            )
        );
    }

    /// distance.

    #[test]
    fn distance_accross_date_line() {
        let p1 = Point::from_lat_long_degrees(50.066389, -179.999722);
        let p2 = Point::from_lat_long_degrees(50.066389, 179.999722);
        assert_eq!(
            Length::from_metres(39.685),
            p1.distance(p2, IUGG_EARTH_RADIUS).round_mm()
        );
    }

    #[test]
    fn distance_between_poles() {
        assert_eq!(
            Length::from_metres(20_015_089.309),
            Point::NORTH_POLE
                .distance(Point::SOUTH_POLE, IUGG_EARTH_RADIUS)
                .round_mm()
        );
    }

    #[test]
    fn distance_test() {
        let p1 = Point::from_lat_long_degrees(50.066389, -5.714722);
        let p2 = Point::from_lat_long_degrees(58.643889, -3.07);
        assert_eq!(
            Length::from_metres(968_853.666),
            p1.distance(p2, IUGG_EARTH_RADIUS).round_mm()
        );
    }

    #[test]
    fn distance_transitivity() {
        let p1 = Point::from_lat_long_degrees(0.0, 0.0);
        let p2 = Point::from_lat_long_degrees(0.0, 10.0);
        let p3 = Point::from_lat_long_degrees(0.0, 20.0);
        let d1 = p1.distance(p2, IUGG_EARTH_RADIUS);
        let d2 = p2.distance(p3, IUGG_EARTH_RADIUS);
        let actual = (d1 + d2).round_mm();
        assert_eq!(actual, p1.distance(p3, IUGG_EARTH_RADIUS).round_mm());
    }

    #[test]
    fn distance_zero() {
        let p = Point::from_lat_long_degrees(50.066389, -5.714722);
        assert_eq!(Length::ZERO, p.distance(p, IUGG_EARTH_RADIUS));
    }

    #[test]
    fn distance_at_equator_going_east() {
        let p1 = Point::from_lat_long_degrees(0.0, 0.0);
        let p2 = Point::from_lat_long_degrees(0.0, 1.0);
        assert_eq!(Angle::from_degrees(90.0), p1.final_bearing(p2));
    }

    #[test]
    fn distance_at_equator_going_west() {
        let p1 = Point::from_lat_long_degrees(0.0, 1.0);
        let p2 = Point::from_lat_long_degrees(0.0, 0.0);
        assert_eq!(Angle::from_degrees(270.0), p1.final_bearing(p2));
    }

    #[test]
    fn distance_coincidental() {
        let p = Point::from_lat_long_degrees(50.0, -18.0);
        assert_eq!(Angle::ZERO, p.final_bearing(p));
    }

    #[test]
    fn distance_same_longitude_going_north() {
        let p1 = Point::from_lat_long_degrees(50.0, -5.0);
        let p2 = Point::from_lat_long_degrees(58.0, -5.0);
        assert_eq!(Angle::ZERO, p1.final_bearing(p2));
    }

    /// final_bearing.

    #[test]
    fn final_bearing_same_longitude_going_south() {
        let p1 = Point::from_lat_long_degrees(58.0, -5.0);
        let p2 = Point::from_lat_long_degrees(50.0, -5.0);
        assert_eq!(Angle::from_degrees(180.0), p1.final_bearing(p2));
    }

    #[test]
    fn final_bearing_test() {
        let p1 = Point::from_lat_long_degrees(50.06638889, -5.71472222);
        let p2 = Point::from_lat_long_degrees(58.64388889, -3.07);
        assert_eq!(
            Angle::from_degrees(11.2752013),
            p1.final_bearing(p2).round_d7()
        );
        assert_eq!(
            Angle::from_degrees(189.1198181),
            p2.final_bearing(p1).round_d7()
        );

        let p1 = Point::from_lat_long_degrees(-53.99472222, -25.9875);
        let p2 = Point::from_lat_long_degrees(54.0, 154.0);
        assert_eq!(
            Angle::from_degrees(125.6839551),
            p1.final_bearing(p2).round_d7()
        );
    }

    // initial_bearing

    #[test]
    fn initial_bearing_antipodal() {
        assert_eq!(
            Angle::ZERO,
            Point::NORTH_POLE.initial_bearing(Point::SOUTH_POLE)
        );
        assert_eq!(
            Angle::ZERO,
            Point::SOUTH_POLE.initial_bearing(Point::NORTH_POLE)
        );
    }

    #[test]
    fn initial_bearing_at_equator_going_east() {
        let p1 = Point::from_lat_long_degrees(0.0, 0.0);
        let p2 = Point::from_lat_long_degrees(0.0, 1.0);
        assert_eq!(Angle::from_degrees(90.0), p1.initial_bearing(p2));
    }

    #[test]
    fn initial_bearing_at_equator_going_west() {
        let p1 = Point::from_lat_long_degrees(0.0, 1.0);
        let p2 = Point::from_lat_long_degrees(0.0, 0.0);
        assert_eq!(Angle::from_degrees(270.0), p1.initial_bearing(p2));
    }

    #[test]
    fn initial_bearing_coincidental() {
        let p = Point::from_lat_long_degrees(50.0, -18.0);
        assert_eq!(Angle::ZERO, p.initial_bearing(p));
    }

    #[test]
    fn initial_bearing_from_north_pole() {
        assert_eq!(
            Angle::from_degrees(26.0),
            Point::NORTH_POLE
                .initial_bearing(Point::from_lat_long_degrees(50.0, 154.0))
                .round_d7()
        );
    }

    #[test]
    fn initial_bearing_north_pole_to_date_line() {
        assert_eq!(
            Angle::ZERO,
            Point::NORTH_POLE
                .initial_bearing(Point::from_lat_long_degrees(50.0, 180.0))
                .round_d7()
        );
    }

    #[test]
    fn initial_bearing_same_longitude_going_north() {
        let p1 = Point::from_lat_long_degrees(50.0, -5.0);
        let p2 = Point::from_lat_long_degrees(58.0, -5.0);
        assert_eq!(Angle::ZERO, p1.initial_bearing(p2).round_d7());
    }

    #[test]
    fn initial_bearing_same_longitude_going_south() {
        let p1 = Point::from_lat_long_degrees(58.0, -5.0);
        let p2 = Point::from_lat_long_degrees(50.0, -5.0);
        assert_eq!(
            Angle::from_degrees(180.0),
            p1.initial_bearing(p2).round_d7()
        );
    }

    #[test]
    fn initial_bearing_from_south_pole() {
        assert_eq!(
            Angle::from_degrees(154.0),
            Point::SOUTH_POLE
                .initial_bearing(Point::from_lat_long_degrees(50.0, 154.0))
                .round_d7()
        );
    }

    #[test]
    fn initial_bearing_south_pole_to_date_line() {
        assert_eq!(
            Angle::from_degrees(180.0),
            Point::SOUTH_POLE
                .initial_bearing(Point::from_lat_long_degrees(50.0, 180.0))
                .round_d7()
        );
    }

    #[test]
    fn initial_bearing_test() {
        let p1 = Point::from_lat_long_degrees(50.06638889, -5.71472222);
        let p2 = Point::from_lat_long_degrees(58.64388889, -3.07);
        assert_eq!(
            Angle::from_degrees(9.1198181),
            p1.initial_bearing(p2).round_d7()
        );
        assert_eq!(
            Angle::from_degrees(191.2752013),
            p2.initial_bearing(p1).round_d7()
        );
    }

    // interpolated

    #[test]
    fn interpolated_antipodal() {
        assert!(Point::NORTH_POLE
            .interpolated(Point::SOUTH_POLE, 0.0)
            .is_none());
    }

    #[test]
    fn interpolated_f0() {
        assert_eq!(
            Some(Point::NORTH_POLE),
            Point::NORTH_POLE.interpolated(Point::from_lat_long_degrees(0.0, 0.0), 0.0)
        );
    }

    #[test]
    fn interpolated_f1() {
        assert_eq!(
            Some(Point::from_lat_long_degrees(0.0, 0.0)),
            Point::NORTH_POLE.interpolated(Point::from_lat_long_degrees(0.0, 0.0), 1.0)
        );
    }

    #[test]
    fn interpolated_invalid_f() {
        assert!(Point::NORTH_POLE
            .interpolated(Point::SOUTH_POLE, -0.1)
            .is_none());
        assert!(Point::NORTH_POLE
            .interpolated(Point::SOUTH_POLE, 1.1)
            .is_none());
    }

    #[test]
    fn interpolated_test() {
        assert_eq!(
            Some(Point::from_lat_long_degrees(0.0, 0.0)),
            Point::from_lat_long_degrees(10.0, 0.0)
                .interpolated(Point::from_lat_long_degrees(-10.0, 0.0), 0.5)
        );
    }

    #[test]
    fn interpolated_half() {
        assert_eq!(
            Some(Point::from_lat_long_degrees(0.0, 0.0)),
            Point::from_lat_long_degrees(10.0, 0.0)
                .interpolated(Point::from_lat_long_degrees(-10.0, 0.0), 0.5)
        );
    }

    #[test]
    fn interpolated_side() {
        let p0 = Point::from_lat_long_degrees(154.0, 54.0);
        let p1 = Point::from_lat_long_degrees(155.0, 55.0);
        let i = p0.interpolated(p1, 0.25).unwrap();
        assert_eq!(0, side(i.as_nvector(), p0.as_nvector(), p1.as_nvector()));
    }

    #[test]
    fn interpolated_transitivity() {
        let p0 = Point::from_lat_long_degrees(10.0, 0.0);
        let p1 = Point::from_lat_long_degrees(-10.0, 0.0);
        let expected = p0.interpolated(p1, 0.5).unwrap();
        let actual = p0
            .interpolated(p1, 0.25)
            .unwrap()
            .interpolated(p1, 1.0 / 3.0)
            .unwrap();
        assert_eq!(expected, actual.normalised_d7());
    }
}
