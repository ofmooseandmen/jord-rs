use crate::{Angle, NVector, Vec3};

use super::base::{easting, orthogonal};

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
    pub fn new(p1: NVector, p2: NVector) -> Self {
        let normal = orthogonal(p1.as_vec3(), p2.as_vec3());
        GreatCircle { normal }
    }

    /// Creates a great circle passing by the given position and heading on the given bearing.
    pub fn from_heading(p: NVector, bearing: Angle) -> Self {
        // easting.
        let e = easting(p.as_vec3());
        // northing.
        let n = p.as_vec3().cross_prod(e);
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

    /// Computes the projection of the given position on this great circle. If the given position is strictly
    /// "perpendicular" to this great circle, this method arbitrarily returns a position on the great circle (p
    /// can be projected anywhere on the great circle).
    ///
    /// # Examples:
    ///
    /// ```
    /// use jord::LatLong;
    /// use jord::spherical::GreatCircle;
    ///
    /// let gc = GreatCircle::new(
    ///     LatLong::from_degrees(0.0, -10.0).to_nvector(),
    ///     LatLong::from_degrees(0.0, 10.0).to_nvector()
    /// );
    ///
    /// let o_p = gc.projection(LatLong::from_degrees(1.0, 0.0).to_nvector());
    /// assert!(o_p.is_some());
    /// assert_eq!(LatLong::from_degrees(0.0, 0.0), LatLong::from_nvector(o_p.unwrap()).round_d7());
    /// ```
    pub fn projection(&self, p: NVector) -> Option<NVector> {
        let n1 = self.normal;
        let n2 = p.as_vec3().stable_cross_prod_unit(n1);
        if n2 == Vec3::ZERO {
            Some(NVector::new(p.as_vec3().orthogonal()))
        } else {
            let proj = orthogonal(n1, n2);
            Some(NVector::new(proj))
        }
    }
}

#[cfg(test)]
mod tests {

    use crate::{positions::assert_opt_nv_eq_d7, spherical::GreatCircle, NVector, Vec3};

    // projection

    #[test]
    fn projection_nearly_perpendicular_null_island() {
        let start = NVector::from_lat_long_degrees(80.0, -90.0);
        let end = NVector::from_lat_long_degrees(80.0, 90.0);
        // great circle normal should be (-1, 0, 0) but due to floating point precision it is not exactly that
        // value, hence (0, 0) is not exactly perpendicular.
        assert_opt_nv_eq_d7(
            NVector::new(Vec3::UNIT_Z),
            GreatCircle::new(start, end).projection(NVector::from_lat_long_degrees(0.0, 0.0)),
        );
    }
}
