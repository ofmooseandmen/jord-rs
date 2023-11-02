use crate::{numbers::eq_zero, Vec3};

/// Computes the signed angle in radians between the given vectors.
///
/// - if vn is `None; the angle is always in [0..PI],
/// - otherwise, the angle is positive if v1 is clockzise looking along vn,
/// - and negative if anti-clockwise looking along vn
pub(crate) fn angle_radians_between(v1: Vec3, v2: Vec3, vn: Option<Vec3>) -> f64 {
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
pub(crate) fn are_ordered(a: Vec3, b: Vec3, c: Vec3) -> bool {
    let n = orthogonal(a, c);
    side(b, n, a) >= 0 && side(c, n, b) >= 0
}

/// Easting at given *n*-vector.
pub(crate) fn easting(v: Vec3) -> Vec3 {
    // north pole = (0, 0, 1), south pole = (0, 0, -1)
    // if v.z() == 1 or -1, then v is either north or south pole, easting is therefore (0, 1, 0)
    if v.z().abs() == 1.0 {
        return Vec3::UNIT_Y;
    }
    // cross product: (0, 0, 1) x v
    Vec3::new_unit(-v.y(), v.x(), 0.0)
}

/// Returns a unit-length vector that is orthogonal to both given unit length vectors.
///
/// This function is similar to {@code v1 x v2} except that it does a better job of ensuring
/// orthogonality when both vectors are nearly parallel and it returns a non-zero result even when
/// both vectors are equal or opposite.
///
/// See [s2edge_crossings_internal.h](https://github.com/google/s2geometry/blob/master/src/s2/s2edge_crossings_internal.h)#GetStableCrossProd.
pub(crate) fn orthogonal(v1: Vec3, v2: Vec3) -> Vec3 {
    // The direction of v1 x v2 is unstable as v2 + v1 or v2 - v1 approaches 0. To avoid this,
    // we just compute (v2 + v1) x (v2 - v1) which is twice the cross product of v2 and v1, but
    // is always perpendicular (since both v1 and v2 are unit-length vectors).
    let r = v1.stable_cross_prod_unit(v2);
    if r == Vec3::ZERO {
        // return an arbitrary orthogonal vector.
        v1.orthogonal()
    } else {
        r
    }
}

/// Determines whether v0 if right of (negative integer), left of (positive integer) or on the
/// great circle (zero), from v1 to v2.
pub(crate) fn side(v0: Vec3, v1: Vec3, v2: Vec3) -> i8 {
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
///
/// - if the dot product is nearly-zero or zero, the 3 positions are collinear
/// - otherwise, if the dot product is negative, v0 is right of (v1, v2)
/// - otherwise, v0 is left of (v1, v2)
pub(crate) fn side_exact(v0: Vec3, v1: Vec3, v2: Vec3) -> f64 {
    let ortho = orthogonal(v1, v2);
    v0.dot_prod(ortho)
}

#[cfg(test)]
mod tests {

    // angle_radians_between

    use std::f64::consts::PI;

    use crate::{spherical::base::angle_radians_between, Vec3};

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
}
