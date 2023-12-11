use crate::Vec3;

/// epsilon below which expensive side is called.
const TRIAGE_SIDE_EPS: f64 = 10.0 * f64::EPSILON;

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
            if d >= 0.0 {
                norm
            } else {
                -norm
            }
        }
    };
    let cos_o = v1.dot_prod(v2);
    sin_o.atan2(cos_o)
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

/// Determines whether v0 if right (negative f64) or left (positive f64) of the
/// great circle from v1 to v2.
///
/// This function returns the value of the dot product between v0 and the orthogonal
/// unit-length vector to v1 and v2:
/// - if the dot product is nearly-zero or zero, the 3 positions are collinear
/// - otherwise, if the dot product is negative, v0 is right of (v1, v2)
/// - otherwise, v0 is left of (v1, v2)
pub(crate) fn exact_side(v0: Vec3, v1: Vec3, v2: Vec3) -> f64 {
    let triage_side = v0.dot_prod(v1.cross_prod(v2));
    // The side of v0 w.r.t. (v1, v2) is given by the triple scalar product (v0 . (v1 x v2))
    // However the cross product of v1 and v2 becomes unstable if v1 and v2 are nearly parallel (coincidental or antipodal).
    // If the result if too close to 0 (using 10 * f64::EPSILON), then call the more expensive function `orthogonal_to`.`
    if triage_side <= TRIAGE_SIDE_EPS {
        v0.dot_prod(v1.orthogonal_to(v2))
    } else {
        triage_side
    }
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
