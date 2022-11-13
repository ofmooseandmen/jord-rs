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
    /// assert_eq!(gc.cross_track_distance(p, IUGG_EARTH_RADIUS), Length::from_metres(-305.66489915731603));
    /// ```
    pub fn cross_track_distance<T: HorizontalPosition>(&self, other: T, radius: Length) -> Length {
        let angle = angle_radians_between(self.normal, other.as_nvector(), None);
        (angle - (PI / 2.0)) * radius
    }
}
