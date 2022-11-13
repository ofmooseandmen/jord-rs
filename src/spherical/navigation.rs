use super::{along_track_distance, angle_radians_between, easting, is_great_circle, MinorArc};
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
    /// assert_eq!(Point::from_lat_long_degrees(45.0, 0.0), dest.round_d7());
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
        } else {
            Some(unchecked_interpolated(*self, other, f))
        }
    }

    /// Computes the projection of the given position on the given minor arc. Returns [None] if the projection is not
    /// within the minor arc (including start and end).
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
    /// let o_p = Point::from_lat_long_degrees(1.0, 0.0).projection(ma);
    /// assert!(o_p.is_some());
    /// assert_eq!(Point::from_lat_long_degrees(0.0, 0.0), o_p.unwrap().round_d7());
    ///
    /// // or alternatively with Vec3:
    ///
    /// let ma = MinorArc::new(
    ///     Vec3::from_lat_long_degrees(0.0, -10.0),
    ///     Vec3::from_lat_long_degrees(0.0, 10.0)
    /// );
    ///
    /// let o_p = Vec3::from_lat_long_degrees(1.0, 0.0).projection(ma);
    /// assert!(o_p.is_some());
    /// assert_eq!(Vec3::from_lat_long_degrees(0.0, 0.0), o_p.unwrap().round_d7());
    /// ```
    fn projection(&self, arc: MinorArc<Self>) -> Option<Self> {
        // we need the ratio of along track distance over distance, so we can use
        // an arbitrary radius.
        let radius = Length::from_metres(1.0);
        let dist = arc.start().distance(arc.end(), radius);
        let along = arc.along_track_distance(*self, radius);
        let ratio = along / dist;
        if !(0.0..=1.0).contains(&ratio) {
            None
        } else {
            Some(unchecked_interpolated(arc.start(), arc.end(), ratio))
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

fn unchecked_interpolated<T: HorizontalPosition>(p0: T, p1: T, f: f64) -> T {
    if f == 0.0 {
        p0
    } else if f == 1.0 {
        p1
    } else {
        let v0 = p0.as_nvector();
        let v1 = p1.as_nvector();
        T::from_nvector(v0.lerp_unit(v1, f))
    }
}
