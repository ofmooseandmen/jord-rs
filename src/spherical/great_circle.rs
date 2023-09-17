use std::f64::consts::PI;

use crate::{Angle, HorizontalPosition, Length, Vec3};

use super::{angle_radians_between, easting, orthogonal};

/// A circle on the surface of a __sphere__ which lies in a plane
// passing through the sphere centre. Every two distinct and non-antipodal points
// define a unique Great Circle.
///
/// It is internally represented as its normal vector - i.e. the normal vector
/// to the plane containing the great circle.
#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct GreatCircle {
    normal: Vec3,
}

impl GreatCircle {
    /// Creates a great circle passing by both given positions (in this direction).
    ///
    /// Note: if both start and end positions are equal or the antipode of one another, then an
    /// arbitrary minor arc is returned - since an infinity of minor arcs exist (see [crate::spherical::is_great_circle]).
    pub fn new<T: HorizontalPosition>(pos1: T, pos2: T) -> Self {
        let normal = orthogonal(pos1.as_nvector(), pos2.as_nvector());
        GreatCircle { normal }
    }

    /// Creates a great circle passing by the given position and heading on the given bearing.
    pub fn from_heading<T: HorizontalPosition>(pos: T, bearing: Angle) -> Self {
        let v = pos.as_nvector();
        // easting.
        let e = easting(v);
        // northing.
        let n = v.cross_prod(e);
        let b_rads = bearing.as_radians();
        let se = e * (b_rads.cos() / e.norm());
        let sn = n * (b_rads.sin() / n.norm());
        let normal = sn - se;
        GreatCircle { normal }
    }

    /// Returns the vector normal to this great circle.
    pub fn normal(&self) -> Vec3 {
        self.normal
    }

    /// Computes the signed distance from the given position to this great circle.
    /// Returns a negative length if the position is left of great circle, positive length if the position is right
    /// of great circle; the orientation of the great circle is therefore important.
    ///
    /// # Example:
    ///
    /// ```
    /// use jord::{Angle, Length, HorizontalPosition, Vec3, IUGG_EARTH_RADIUS};
    /// use jord::spherical::GreatCircle;
    ///
    /// let p = Vec3::from_lat_long_degrees(53.2611, -0.7972);
    /// let gc = GreatCircle::from_heading(
    ///     Vec3::from_lat_long_degrees(53.3206, -1.7297),
    ///     Angle::from_degrees(96.0)
    /// );
    /// assert_eq!(Length::from_metres(-305.665), gc.cross_track_distance(p, IUGG_EARTH_RADIUS).round_mm());
    /// ```
    pub fn cross_track_distance<T: HorizontalPosition>(&self, other: T, radius: Length) -> Length {
        let angle = angle_radians_between(self.normal, other.as_nvector(), None);
        (angle - (PI / 2.0)) * radius
    }

    /// Computes the projection of the given position on the given great circle. If the given position is strictly
    /// "perpendicular" to the given great circle, this method arbitrarily returns a position on the great circle (p
    /// can be projected anywhere on the great circle).
    ///
    /// # Examples:
    ///
    /// ```
    /// use jord::{HorizontalPosition, Point, Vec3};
    /// use jord::spherical::{GreatCircle, Navigation};
    ///
    /// let gc = GreatCircle::new(
    ///     Point::from_lat_long_degrees(0.0, -10.0),
    ///     Point::from_lat_long_degrees(0.0, 10.0)
    /// );
    ///
    /// let o_p = gc.projection(Point::from_lat_long_degrees(1.0, 0.0));
    /// assert!(o_p.is_some());
    /// assert_eq!(Point::from_lat_long_degrees(0.0, 0.0), o_p.unwrap().normalised_d7());
    ///
    /// // or alternatively with Vec3:
    ///
    /// let gc = GreatCircle::new(
    ///     Vec3::from_lat_long_degrees(0.0, -10.0),
    ///     Vec3::from_lat_long_degrees(0.0, 10.0)
    /// );
    ///
    /// let o_p = gc.projection(Vec3::from_lat_long_degrees(1.0, 0.0));
    /// assert!(o_p.is_some());
    /// assert_eq!(Vec3::from_lat_long_degrees(0.0, 0.0), o_p.unwrap().normalised_d7());
    /// ```
    pub fn projection<T: HorizontalPosition>(&self, pos: T) -> Option<T> {
        let n1 = self.normal;
        let n2 = pos.as_nvector().stable_cross_prod_unit(n1);
        if n2 == Vec3::ZERO {
            Some(T::from_nvector(pos.as_nvector().orthogonal()))
        } else {
            let proj = orthogonal(n1, n2);
            Some(T::from_nvector(proj))
        }
    }
}

#[cfg(test)]
mod tests {

    use crate::{
        spherical::{GreatCircle, Navigation},
        HorizontalPosition, Length, Point, IUGG_EARTH_RADIUS,
    };

    // cross_track_distance

    #[test]
    fn cross_track_distance_left() {
        let p = Point::from_lat_long_degrees(53.2611, -0.7972);
        let gc1 = Point::from_lat_long_degrees(53.3206, -1.7297);
        let gc2 = Point::from_lat_long_degrees(53.1887, 0.1334);
        assert_eq!(
            Length::from_metres(-307.55),
            GreatCircle::new(gc1, gc2)
                .cross_track_distance(p, IUGG_EARTH_RADIUS)
                .round_mm()
        );
    }

    #[test]
    fn cross_track_distance_right() {
        let p = Point::from_lat_long_degrees(53.2611, -0.7972).antipode();
        let gc1 = Point::from_lat_long_degrees(53.3206, -1.7297);
        let gc2 = Point::from_lat_long_degrees(53.1887, 0.1334);
        assert_eq!(
            Length::from_metres(307.55),
            GreatCircle::new(gc1, gc2)
                .cross_track_distance(p, IUGG_EARTH_RADIUS)
                .round_mm()
        );
    }

    #[test]
    fn cross_track_distance_zero() {
        let gc1 = Point::from_lat_long_degrees(53.3206, -1.7297);
        let gc2 = Point::from_lat_long_degrees(53.1887, 0.1334);
        let gct = GreatCircle::new(gc1, gc2);
        let ib = gc1.initial_bearing(gc2);
        let gch = GreatCircle::from_heading(gc1, ib);
        let mut f = 0.0;
        while f <= 1.0 {
            let p = gc1.interpolated(gc2, 0.5).unwrap();
            assert_eq!(
                Length::ZERO,
                gct.cross_track_distance(p, IUGG_EARTH_RADIUS).round_mm()
            );
            assert_eq!(
                Length::ZERO,
                gch.cross_track_distance(p, IUGG_EARTH_RADIUS).round_mm()
            );
            f = f + 0.1;
        }
    }

    // projection

    #[test]
    fn nearly_perpendicular_null_island() {
        let start = Point::from_lat_long_degrees(80.0, -90.0);
        let end = Point::from_lat_long_degrees(80.0, 90.0);
        // great circle normal should be (-1, 0, 0) but due to floating point precision it is not exactly that
        // value, hence (0, 0) is not exactly perpendicular.
        assert_eq!(
            Some(Point::NORTH_POLE),
            GreatCircle::new(start, end).projection(Point::from_lat_long_degrees(0.0, 0.0))
        );
    }

}
