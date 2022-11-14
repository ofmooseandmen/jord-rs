use crate::{numbers::eq_zero, HorizontalPosition, Length, Vec3};

/// Computes the signed angle in radians between the given vectors.
/// - if vn is `None; the angle is always in [0..PI],
/// - otherwise, the angle is positive if v1 is clockzise looking along vn,
/// - and negative if anti-clockwise looking along vn
pub fn angle_radians_between(v1: Vec3, v2: Vec3, vn: Option<Vec3>) -> f64 {
    let p1xp2 = v1.cross_prod(v2);
    let norm = p1xp2.norm();
    let sin_o = match vn {
        None => norm,
        Some(v) => {
            let d = p1xp2.dot_prod(v);
            let sign = if d >= 0.0 { 1.0 } else { -1.0 };
            sign * norm
        }
    };
    let cos_o = v1.dot_prod(v2);
    sin_o.atan2(cos_o)
}

/// Determines whether the three points a, b and c occur in this order along the minor arc (a, c).
/// This effectively determines whether b is located within the minor arc (a, c).
///
/// See [s2predicates.h](https://github.com/google/s2geometry/blob/master/src/s2/s2predicates.h)#OrderedCCW.
pub fn are_ordered(a: Vec3, b: Vec3, c: Vec3) -> bool {
    let n = orthogonal(a, c);
    side(b, n, a) >= 0 && side(c, n, b) >= 0
}

/// Easting at given /n</-vector.
pub fn easting(v: Vec3) -> Vec3 {
    // north pole = (0, 0, 1), south pole = (0, 0, -1)
    // if v.z() == 1 or -1, then v is either north or south pole, easting is therefore (0, 1, 0)
    if v.z().abs() == 1.0 {
        return Vec3::UNIT_Y;
    }
    // cross product: (0, 0, 1) x v
    Vec3::new_unit(-v.y(), v.x(), 0.0)
}

/// Determines whether the 2 given positions define a unique great circle: i.e. they are
/// not equal nor the antipode of one another.
pub fn is_great_circle<T: HorizontalPosition>(p1: T, p2: T) -> bool {
    p1 != p2 && !p1.is_antipode(p2)
}

/// Computes the mean position of the given positions: the “center of gravity” of the given positions,
/// which and can be compared to the centroid of a geometrical shape (n.b. other definitions of mean exist).
/// The mean position is undefined if either the given vector is empty or some of the given positions are
/// antipodals.
///
/// # Examples
///
/// ```
/// use jord::{Length, HorizontalPosition, Vec3};
/// use jord::spherical::mean_position;
///
/// let ps = vec![
///     Vec3::from_lat_long_degrees( 10.0,  10.0),
///     Vec3::from_lat_long_degrees( 10.0, -10.0),
///     Vec3::from_lat_long_degrees(-10.0, -10.0),
///     Vec3::from_lat_long_degrees(-10.0,  10.0)
/// ];
///
/// let o_m = mean_position(&ps);
/// assert!(o_m.is_some());
/// assert_eq!(
///     Vec3::from_lat_long_degrees(0.0, 0.0),
///     o_m.unwrap().normalised_d7()
/// );
/// ```
pub fn mean_position<T: HorizontalPosition>(ps: &[T]) -> Option<T> {
    if ps.is_empty() || contains_antipodal(ps) {
        None
    } else if ps.len() == 1 {
        ps.first().cloned()
    } else {
        let vs: Vec<Vec3> = ps.iter().map(|&v| v.as_nvector()).collect();
        let m = Vec3::mean(&vs);
        Some(T::from_nvector(m))
    }
}

/// Determines if the given vector contains antipodal positions.
fn contains_antipodal<T: HorizontalPosition>(ps: &[T]) -> bool {
    for p in ps {
        let a = p.antipode();
        let found = ps.iter().any(|&o| o == a);
        if found {
            return true;
        }
    }
    false
}

/// Returns a unit-length vector that is orthogonal to both given unit length vectors
/// This function is similar to {@code v1 x v2} except that it does a better job of ensuring
/// orthogonality when both vectors are nearly parallel and it returns a non-zero result even when
/// both vectors are equal or opposite.
///
/// See [s2edge_crossings_internal.h](https://github.com/google/s2geometry/blob/master/src/s2/s2edge_crossings_internal.h)#GetStableCrossProd.
pub fn orthogonal(v1: Vec3, v2: Vec3) -> Vec3 {
    // The direction of v1 x v2 is unstable as v2 + v1 or v2 - v1 approaches 0. To avoid this,
    // we just compute (v2 + v1) x (v2 - v1) which is twice the cross product of v2 and v1, but
    // is always perpendicular (since both v1 and v2 are unit-length vectors).
    let r = (v2 + v1).cross_prod_unit(v2 - v1);
    if r == Vec3::ZERO {
        // return an arbitrary orthogonal vector.
        v1.orthogonal()
    } else {
        r
    }
}

/// Determines whether v0 if right of (negative integer), left of (positive integer) or on the
/// great circle (zero), from v1 to v2.
///
/// # Examples
///
/// ```
/// use jord::{HorizontalPosition, Vec3};
/// use jord::spherical::side;
///
/// let v1 = Vec3::from_lat_long_degrees(55.4295, 13.82);
/// let v2 = Vec3::from_lat_long_degrees(56.0465, 12.6945);
/// let v3 = Vec3::from_lat_long_degrees(56.0294, 14.1567);
///
/// assert_eq!(-1, side(v1, v2, v3));
/// assert_eq!(1, side(v1, v3, v2));
/// ```
pub fn side(v0: Vec3, v1: Vec3, v2: Vec3) -> i8 {
    let side = side_exact(v0, v1, v2);
    if eq_zero(side) {
        0
    } else if side < 0.0 {
        -1
    } else {
        1
    }
}

/// Similar to `side` but returns the value of the dot product between v0 and the orthogonal
/// unit-length vector to v1 and v2.
/// - if the dot product is nearly-zero or zero, the 3 positions are collinear
/// - otherwise, if the dot product is negative, v0 is right of (v1, v2)
/// - otherwise, v0 is left of (v1, v2)
pub fn side_exact(v0: Vec3, v1: Vec3, v2: Vec3) -> f64 {
    let ortho = orthogonal(v1, v2);
    v0.dot_prod(ortho)
}

/// Returns the angle in radians turned from AB to BC. Angle is positive for left turn,
/// negative for right turn and 0 if all 3 positions are collinear (i.e. on the same great circle).
pub fn turn_radians(a: Vec3, b: Vec3, c: Vec3) -> f64 {
    let n1 = orthogonal(a, b);
    let n2 = orthogonal(b, c);
    angle_radians_between(n1, n2, Some(b))
}

pub(crate) fn along_track_distance(pos: Vec3, start: Vec3, normal: Vec3, radius: Length) -> Length {
    let angle = angle_radians_between(
        start,
        normal.cross_prod(pos).cross_prod(normal),
        Some(normal),
    );
    angle * radius
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use crate::{Angle, HorizontalPosition, Vec3};

    use crate::spherical::{angle_radians_between, mean_position, side, turn_radians};

    // angle_radians_between

    #[test]
    fn angle_radians_between_signed() {
        let z = Vec3::new(0.0, 0.0, -1.0);
        assert_eq!(
            angle_radians_between(Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Some(z)),
            PI / 2.0
        );
        assert_eq!(
            angle_radians_between(Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Some(z)),
            -PI / 2.0
        );
        assert_eq!(
            angle_radians_between(Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0), Some(z)),
            -PI / 4.0
        );
        assert_eq!(
            angle_radians_between(Vec3::new(1.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Some(z)),
            PI / 4.0,
        );
    }

    #[test]
    fn angle_radians_between_unsigned() {
        assert_eq!(
            angle_radians_between(Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0), None),
            PI / 2.0
        );
        assert_eq!(
            angle_radians_between(Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0), None),
            PI / 4.0
        );
    }

    // mean

    #[test]
    fn mean_antipodal() {
        assert!(mean_position(&vec!(Vec3::UNIT_Z, Vec3::NEG_UNIT_Z)).is_none());
    }

    #[test]
    fn mean_empty() {
        let vs: Vec<Vec3> = Vec::new();
        assert!(mean_position(&vs).is_none());
    }

    #[test]
    fn mean_test() {
        let vs = vec![
            Vec3::from_lat_long_degrees(10.0, 10.0),
            Vec3::from_lat_long_degrees(10.0, -10.0),
            Vec3::from_lat_long_degrees(-10.0, -10.0),
            Vec3::from_lat_long_degrees(-10.0, 10.0),
        ];

        let o_m = mean_position(&vs);
        assert!(o_m.is_some());
        assert_eq!(
            Vec3::from_lat_long_degrees(0.0, 0.0),
            o_m.unwrap().normalised_d7()
        );
    }

    #[test]
    fn mean_one() {
        assert_eq!(Some(Vec3::UNIT_X), mean_position(&vec!(Vec3::UNIT_X)));
    }

    // side

    #[test]
    fn side_collinear() {
        assert_eq!(
            0,
            side(
                Vec3::from_lat_long_degrees(0.0, 0.0),
                Vec3::from_lat_long_degrees(45.0, 0.0),
                Vec3::from_lat_long_degrees(90.0, 0.0)
            )
        );
    }

    #[test]
    fn side_equal() {
        let v1 = Vec3::new_unit(1.0, 2.0, 3.0);
        // largest component is z, orthogonal vector in x-z plan.
        assert_eq!(0, side(Vec3::UNIT_Y, v1, v1));
        assert_eq!(-1, side(Vec3::new_unit(1.0, -3.0, 0.0), v1, v1));
        assert_eq!(1, side(Vec3::new_unit(-1.0, 3.0, 0.0), v1, v1));
    }

    #[test]
    fn side_same_meridian() {
        let v0 = Vec3::from_lat_long_degrees(-78.0, 55.0);
        let v1 = Vec3::from_lat_long_degrees(-85.0, 55.0);
        let v2 = Vec3::from_lat_long_degrees(10.0, 55.0);
        assert_eq!(0, side(v0, v1, v2));
        assert_eq!(0, side(v0, v2, v1));
    }

    #[test]
    fn side_opposite() {
        let v1 = Vec3::new_unit(1.0, 2.0, 3.0);
        let v2 = Vec3::new_unit(-1.0, -2.0, -3.0);
        // largest component is z, orthogonal vector in x-z plan.
        assert_eq!(0, side(Vec3::UNIT_Y, v1, v2));
        assert_eq!(-1, side(Vec3::new_unit(1.0, -3.0, 0.0), v1, v2));
        assert_eq!(1, side(Vec3::new_unit(-1.0, 3.0, 0.0), v1, v2));
    }

    #[test]
    fn side_resolution() {
        // 1 arc microsecond.
        let one_mas = Angle::from_degrees(1.0 / 3600000000.0);

        let lng = Angle::from_degrees(55.0);
        let v1 = Vec3::from_lat_long_degrees(-85.0, lng.as_degrees());
        let v2 = Vec3::from_lat_long_degrees(10.0, lng.as_degrees());
        let right = Vec3::from_lat_long(Angle::from_degrees(-78.0), lng + one_mas);
        assert_eq!(-1, side(right, v1, v2));
        let left = Vec3::from_lat_long(Angle::from_degrees(-78.0), lng - one_mas);
        assert_eq!(1, side(left, v1, v2));
    }

    // turn_radians

    #[test]

    fn turn_radians_collinear() {
        let actual = turn_radians(
            Vec3::from_lat_long_degrees(0.0, 0.0),
            Vec3::from_lat_long_degrees(45.0, 0.0),
            Vec3::from_lat_long_degrees(90.0, 0.0),
        );
        assert_eq!(0.0, actual);
    }

    #[test]
    fn turn_radians_left() {
        let actual = turn_radians(
            Vec3::from_lat_long_degrees(0.0, 0.0),
            Vec3::from_lat_long_degrees(45.0, 0.0),
            Vec3::from_lat_long_degrees(60.0, -10.0),
        );
        assert_eq!(0.3175226173130951, actual);
    }

    #[test]
    fn turn_radians_right() {
        let actual = turn_radians(
            Vec3::from_lat_long_degrees(0.0, 0.0),
            Vec3::from_lat_long_degrees(45.0, 0.0),
            Vec3::from_lat_long_degrees(60.0, 10.0),
        );
        assert_eq!(-0.3175226173130951, actual);
    }
}
