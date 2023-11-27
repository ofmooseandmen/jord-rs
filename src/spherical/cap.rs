use std::f64::consts::PI;

use crate::{Angle, NVector, Vec3};

use super::Sphere;

/// A [spherical cap](https://en.wikipedia.org/wiki/Spherical_cap): a portion of a sphere cut off by a plane.
/// This struct and implementation is very much based on [S2Cap](https://github.com/google/s2geometry/blob/master/src/s2/s2cap.h).
#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct Cap {
    centre: NVector,
    chord_radius2: f64,
}

impl Cap {
    const MAX_CHORD_RADIUS_2: f64 = 4.0;

    /// Empty spherical cap: contains no point.
    pub const EMPTY: Cap = Self {
        centre: NVector::new(Vec3::UNIT_Z),
        chord_radius2: -1.0,
    };

    /// Full spherical cap: contains all points.
    pub const FULL: Cap = Self {
        centre: NVector::new(Vec3::UNIT_Z),
        chord_radius2: Self::MAX_CHORD_RADIUS_2,
    };

    /// Constructs a new cap from the given centre and given radius expressed as the angle between the
    /// centre and all points on the boundary of the cap.
    pub fn from_centre_and_radius(centre: NVector, radius: Angle) -> Self {
        let chord_radius2 = if radius < Angle::ZERO {
            -1.0
        } else {
            Self::radius_to_chord_radius2(radius)
        };
        Self {
            centre,
            chord_radius2,
        }
    }

    /// Constructs a new cap from the given centre and a given point on the boundary of the cap.
    pub fn from_centre_and_boundary_point(centre: NVector, boundary_point: NVector) -> Self {
        Self {
            centre,
            chord_radius2: Self::chord_radius2(centre, boundary_point),
        }
    }

    /// Constructs a new cap whose boundary passes by the 3 given points: the returned cap is the circumcircle of the
    /// triangle defined by the 3 given points.
    pub fn from_points(a: NVector, b: NVector, c: NVector) -> Self {
        // see STRIPACK: http://orion.math.iastate.edu/burkardt/f_src/stripack/stripack.f90
        // 3 points must be in anti-clockwise order
        let clockwise = Sphere::side(a, b, c) < 0;
        let v1 = a.as_vec3();
        let v2 = if clockwise { c.as_vec3() } else { b.as_vec3() };
        let v3 = if clockwise { b.as_vec3() } else { c.as_vec3() };
        let e1 = v2 - v1;
        let e2 = v3 - v1;
        let centre = NVector::new(e1.orthogonal_to(e2));
        // all chord radius should be equal, still take maximum to account for floating point errors.
        let chord_radius2 = Self::chord_radius2(a, centre)
            .max(Self::chord_radius2(b, centre).max(Self::chord_radius2(c, centre)));
        Self {
            centre,
            chord_radius2,
        }
    }

    /// Determines whether this cap is [full](crate::spherical::Cap::FULL).
    pub fn is_full(&self) -> bool {
        self.chord_radius2 >= Self::MAX_CHORD_RADIUS_2
    }

    /// Determines whether this cap is [empty](crate::spherical::Cap::EMPTY).
    pub fn is_empty(&self) -> bool {
        self.chord_radius2 < 0.0
    }

    /// Returns the complement of this cap. Both caps have the same boundary but
    /// disjoint interiors (the union of both caps is [full](crate::spherical::Cap::FULL)).
    pub fn complement(&self) -> Self {
        if self.is_empty() {
            Self::FULL
        } else if self.is_full() {
            Self::EMPTY
        } else {
            Self {
                centre: self.centre.antipode(),
                chord_radius2: (Self::MAX_CHORD_RADIUS_2 - self.chord_radius2),
            }
        }
    }

    /// Determines whether this cap contains the given point (including the boundary).
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::Cap;
    ///
    /// let cap = Cap::from_centre_and_boundary_point(
    ///     NVector::from_lat_long_degrees(90.0, 0.0),
    ///     NVector::from_lat_long_degrees(0.0, 0.0)
    /// );
    ///
    /// assert!(cap.contains_point(NVector::from_lat_long_degrees(0.0, 0.0)));
    /// assert!(cap.contains_point(NVector::from_lat_long_degrees(45.0, 45.0)));
    /// ```
    pub fn contains_point(&self, p: NVector) -> bool {
        Self::chord_radius2(self.centre, p) <= self.chord_radius2
    }

    /// Determines whether the interior of this cap contains the given point.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::Cap;
    ///
    /// let cap = Cap::from_centre_and_boundary_point(
    ///     NVector::from_lat_long_degrees(90.0, 0.0),
    ///     NVector::from_lat_long_degrees(0.0, 0.0)
    /// );
    ///
    /// assert!(!cap.interior_contains_point(NVector::from_lat_long_degrees(0.0, 0.0)));
    /// assert!(cap.interior_contains_point(NVector::from_lat_long_degrees(45.0, 45.0)));
    /// ```
    pub fn interior_contains_point(&self, p: NVector) -> bool {
        Self::chord_radius2(self.centre, p) < self.chord_radius2
    }

    /// Determines whether this cap contains the given cap. The full cap contains all caps
    /// and the empty cap is contained by all caps.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::Cap;
    ///
    /// let cap1 = Cap::from_centre_and_boundary_point(
    ///     NVector::from_lat_long_degrees(90.0, 0.0),
    ///     NVector::from_lat_long_degrees(0.0, 0.0)
    /// );
    ///
    /// let cap2 = Cap::from_centre_and_boundary_point(
    ///     NVector::from_lat_long_degrees(90.0, 0.0),
    ///     NVector::from_lat_long_degrees(45.0, 0.0)
    /// );
    ///
    /// assert!(cap1.contains_cap(cap2));
    /// ```
    pub fn contains_cap(&self, other: Self) -> bool {
        if self.is_full() || other.is_empty() {
            true
        } else {
            self.chord_radius2
                >= Self::chord_radius2(self.centre, other.centre) + other.chord_radius2
        }
    }

    ///Returns the smallest cap which encloses this cap and the other given cap.
    pub fn union(&self, other: Self) -> Self {
        if self.chord_radius2 < other.chord_radius2 {
            return other.union(*self);
        }
        if self.is_full() || other.is_empty() {
            return *self;
        }

        let self_radius = self.radius();
        let other_radius = other.radius();
        let distance = Sphere::angle(self.centre, other.centre);
        if self_radius >= distance + other_radius {
            return *self;
        }
        let union_radius = 0.5 * (distance + self_radius + other_radius);
        let ang = 0.5 * (distance - self_radius + other_radius);
        let centre = Sphere::position_on_great_circle(self.centre, other.centre, ang);
        Self {
            centre,
            chord_radius2: Self::radius_to_chord_radius2(union_radius),
        }
    }

    /// Returns the centre of this cap.
    pub fn centre(&self) -> NVector {
        self.centre
    }

    /// Returns the radius of this cap: central angle between the centre of this cap and
    /// any point on the boundary (negative for [empty](crate::spherical::Cap::EMPTY) caps).
    /// The returned value may not exactly equal the value passed
    /// to [from_centre_and_boundary_point](crate::spherical::Cap::from_centre_and_boundary_point).
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f64::consts::PI;
    ///
    /// use jord::{Angle, NVector};
    /// use jord::spherical::Cap;
    ///
    /// let cap = Cap::from_centre_and_boundary_point(
    ///      NVector::from_lat_long_degrees(90.0, 0.0),
    ///      NVector::from_lat_long_degrees(45.0, 45.0)
    /// );
    ///
    /// assert_eq!(Angle::from_radians(PI / 4.0), cap.radius().round_d7());
    /// ```
    pub fn radius(&self) -> Angle {
        if self.is_empty() {
            Angle::from_radians(-1.0)
        } else {
            Angle::from_radians(2.0 * (self.chord_radius2.sqrt() * 0.5).asin())
        }
    }

    fn chord_radius2(a: NVector, b: NVector) -> f64 {
        (a.as_vec3() - b.as_vec3())
            .squared_norm()
            .min(Self::MAX_CHORD_RADIUS_2)
    }

    fn radius_to_chord_radius2(radius: Angle) -> f64 {
        // max angle is PI
        let chord_radius = 2.0 * ((radius.as_radians().min(PI)) * 0.5).sin();
        (chord_radius * chord_radius).min(Self::MAX_CHORD_RADIUS_2)
    }
}

#[cfg(test)]
mod tests {
    use crate::{positions::assert_nv_eq_d7, spherical::Cap, Angle, LatLong, NVector};
    use std::f64::consts::PI;

    #[test]
    fn full() {
        assert!(Cap::FULL.contains_point(NVector::from_lat_long_degrees(90.0, 0.0)));
        assert!(Cap::FULL.contains_point(NVector::from_lat_long_degrees(-90.0, 0.0)));
        assert_eq!(Angle::from_radians(PI), Cap::FULL.radius());
        assert_eq!(Cap::EMPTY, Cap::FULL.complement());
    }

    #[test]
    fn empty() {
        assert!(!Cap::EMPTY.contains_point(NVector::from_lat_long_degrees(90.0, 0.0)));
        assert!(!Cap::EMPTY.contains_point(NVector::from_lat_long_degrees(-90.0, 0.0)));
        assert_eq!(Angle::from_radians(-1.0), Cap::EMPTY.radius());
        assert_eq!(Cap::FULL, Cap::EMPTY.complement());
    }

    #[test]
    fn from_points() {
        let a = NVector::from_lat_long_degrees(0.0, 0.0);
        let b = NVector::from_lat_long_degrees(20.0, 0.0);
        let c = NVector::from_lat_long_degrees(10.0, 10.0);
        let cap = Cap::from_points(a, b, c);
        assert!(cap.contains_point(a));
        assert!(cap.contains_point(b));
        assert!(cap.contains_point(c));

        let o = Cap::from_points(c, b, a);
        assert_nv_eq_d7(o.centre, cap.centre);
        assert!((o.chord_radius2 - cap.chord_radius2).abs() < 1e-16);
    }

    #[test]
    fn complement() {
        let np = NVector::from_lat_long_degrees(90.0, 0.0);
        let sp = NVector::from_lat_long_degrees(-90.0, 0.0);
        let northern = Cap::from_centre_and_radius(sp, Angle::QUARTER_CIRCLE);
        let southern = Cap::from_centre_and_radius(np, Angle::QUARTER_CIRCLE);

        let northern_complement = northern.complement();
        assert_eq!(southern.centre, northern_complement.centre);
        assert!((southern.chord_radius2 - northern_complement.chord_radius2).abs() < 1e15);

        let southern_complement = southern.complement();
        assert_eq!(northern.centre, southern_complement.centre);
        assert!((northern.chord_radius2 - southern_complement.chord_radius2).abs() < 1e15);
    }

    #[test]
    fn contains_point() {
        let cap = Cap::from_centre_and_boundary_point(
            NVector::from_lat_long_degrees(90.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 0.0),
        );
        assert!(cap.contains_point(NVector::from_lat_long_degrees(0.0, 0.0)));
        assert!(cap.contains_point(NVector::from_lat_long_degrees(45.0, 45.0)));
    }

    #[test]
    fn interior_contains_point() {
        let cap = Cap::from_centre_and_boundary_point(
            NVector::from_lat_long_degrees(90.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 0.0),
        );
        assert!(!cap.interior_contains_point(NVector::from_lat_long_degrees(0.0, 0.0)));
        assert!(cap.interior_contains_point(NVector::from_lat_long_degrees(45.0, 45.0)));
    }

    #[test]
    fn contains_cap() {
        let c = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(30.0, 30.0),
            Angle::from_degrees(10.0),
        );
        assert!(Cap::FULL.contains_cap(c));
        assert!(c.contains_cap(Cap::EMPTY));

        let o = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(30.0, 30.0),
            Angle::from_degrees(20.0),
        );
        assert!(!c.contains_cap(o));
        assert!(o.contains_cap(c));
    }

    #[test]
    fn radius() {
        assert_eq!(
            Angle::QUARTER_CIRCLE,
            Cap::from_centre_and_boundary_point(
                NVector::from_lat_long_degrees(90.0, 0.0),
                NVector::from_lat_long_degrees(0.0, 0.0)
            )
            .radius()
            .round_d7()
        );
        assert_eq!(
            Angle::from_radians(PI / 4.0),
            Cap::from_centre_and_boundary_point(
                NVector::from_lat_long_degrees(90.0, 0.0),
                NVector::from_lat_long_degrees(45.0, 45.0)
            )
            .radius()
            .round_d7()
        );
    }

    #[test]
    fn union() {
        assert!(Cap::FULL.union(Cap::EMPTY).is_full());
        assert!(Cap::EMPTY.union(Cap::FULL).is_full());

        // a and b have same centre, but radius of a  < radius of b.
        let a = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(50.0, 10.0),
            Angle::from_degrees(0.2),
        );
        let b = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(50.0, 10.0),
            Angle::from_degrees(0.3),
        );

        assert!(b.contains_cap(a));
        assert_eq!(b, a.union(b));

        assert_eq!(Cap::FULL, a.union(Cap::FULL));
        assert_eq!(a, a.union(Cap::EMPTY));

        // a and c have different centers, one entirely encompasses the other.
        let c = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(51.0, 11.0),
            Angle::from_degrees(1.5),
        );
        assert!(c.contains_cap(a));
        assert_eq!(a.union(c).centre(), c.centre());
        assert_eq!(a.union(c).radius(), c.radius());

        // e is partially overlapping a.
        let e = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(50.3, 10.3),
            Angle::from_degrees(0.2),
        );
        assert!(!e.contains_cap(a));
        let u = a.union(e);
        let c = LatLong::from_nvector(u.centre());
        assert_eq!(Angle::from_degrees(50.1501), c.latitude().round_d5());
        assert_eq!(Angle::from_degrees(10.14953), c.longitude().round_d5());
        assert_eq!(Angle::from_degrees(0.37815), u.radius().round_d5());
    }
}
