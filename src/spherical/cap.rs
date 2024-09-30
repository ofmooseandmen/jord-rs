use std::f64::consts::PI;

use crate::{Angle, LatLong, Mat33, NVector, Vec3};

use super::{ChordLength, Sphere};

/// A [spherical cap](https://en.wikipedia.org/wiki/Spherical_cap): a portion of a sphere cut off by a plane.
/// This struct and implementation is very much based on [S2Cap](https://github.com/google/s2geometry/blob/master/src/s2/s2cap.h).
#[derive(PartialEq, Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))] // codecov:ignore:this
pub struct Cap {
    centre: NVector,
    radius: ChordLength,
}

impl Cap {
    /// Empty spherical cap: contains no point.
    pub const EMPTY: Cap = Self {
        centre: NVector::new(Vec3::UNIT_Z),
        radius: ChordLength::NEGATIVE,
    };

    /// Full spherical cap: contains all points.
    pub const FULL: Cap = Self {
        centre: NVector::new(Vec3::UNIT_Z),
        radius: ChordLength::MAX,
    };

    /// Constructs a new cap from the given centre and given radius expressed as the angle between the
    /// centre and all points on the boundary of the cap.
    pub fn from_centre_and_radius(centre: NVector, radius: Angle) -> Self {
        Self {
            centre,
            radius: ChordLength::from_angle(radius),
        }
    }

    /// Constructs a new cap from the given centre and a given point on the boundary of the cap.
    pub fn from_centre_and_boundary_point(centre: NVector, boundary_point: NVector) -> Self {
        Self {
            centre,
            radius: ChordLength::new(centre, boundary_point),
        }
    }

    /// Constructs a new cap whose boundary passes by the 3 given points: the returned cap is the circumcircle of the
    /// triangle defined by the 3 given points.
    pub fn from_triangle(a: NVector, b: NVector, c: NVector) -> Self {
        // see STRIPACK: http://orion.math.iastate.edu/burkardt/f_src/stripack/stripack.f90
        // 3 points must be in anti-clockwise order
        let clockwise = Sphere::side(a, b, c) < 0;
        let v1 = a.as_vec3();
        let v2 = if clockwise { c.as_vec3() } else { b.as_vec3() };
        let v3 = if clockwise { b.as_vec3() } else { c.as_vec3() };
        let e1 = v2 - v1;
        let e2 = v3 - v1;
        let centre = NVector::new(e1.orthogonal_to(e2));
        // all chord length should be equal, still take maximum to account for floating point errors.
        let radius: ChordLength = ChordLength::new(a, centre)
            .max(ChordLength::new(b, centre).max(ChordLength::new(c, centre)));
        Self { centre, radius }
    }

    /// Determines whether this cap is [full](crate::spherical::Cap::FULL).
    pub fn is_full(&self) -> bool {
        self.radius == ChordLength::MAX
    }

    /// Determines whether this cap is [empty](crate::spherical::Cap::EMPTY).
    pub fn is_empty(&self) -> bool {
        self.radius == ChordLength::NEGATIVE
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
                radius: ChordLength::from_squared_length(
                    ChordLength::MAX.length2() - self.radius.length2(),
                ),
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
        ChordLength::new(self.centre, p) <= self.radius
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
        ChordLength::new(self.centre, p) < self.radius
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
            self.radius.length2()
                >= ChordLength::new(self.centre, other.centre).length2() + other.radius.length2()
        }
    }

    /// Returns the smallest cap which encloses this cap and the other given cap.
    pub fn union(&self, other: Self) -> Self {
        if self.radius < other.radius {
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
            radius: ChordLength::from_angle(union_radius),
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
        self.radius.to_angle()
    }

    /// Returns the list of vertices defining the boundary of this cap. If this cap is [empty](crate::spherical::Cap::EMPTY)
    /// or [full](crate::spherical::Cap::FULL) the returned vector is empty, otherwise it contains `max(3, nb_vertices)` vertices.
    ///
    /// ```
    /// use jord::{Angle, NVector};
    /// use jord::spherical::{Cap, Sphere};
    ///
    /// let centre = NVector::from_lat_long_degrees(55.6050, 13.0038);
    /// let radius = Angle::from_degrees(1.0);
    /// let cap = Cap::from_centre_and_radius(centre, radius);
    ///
    /// let vs = cap.boundary(10);
    /// for v in vs {
    ///     assert_eq!(radius, Sphere::angle(centre, v).round_d7());
    /// }
    /// ```
    pub fn boundary(&self, nb_vertices: usize) -> Vec<NVector> {
        if self.is_empty() || self.is_full() {
            return Vec::new();
        }

        let radius = self.radius().as_radians();
        let rm = radius.sin();
        let z = (1.0 - rm * rm).sqrt();

        let ll = LatLong::from_nvector(self.centre);
        let lat = ll.latitude().as_radians();
        let lon = ll.longitude().as_radians();

        let rya = PI / 2.0 - lat;
        let cy = rya.cos();
        let sy = rya.sin();
        let ry = Mat33::new(
            Vec3::new(cy, 0.0, sy),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(-sy, 0.0, cy),
        );

        let rza = lon;
        let cz = rza.cos();
        let sz = rza.sin();
        let rz = Mat33::new(
            Vec3::new(cz, -sz, 0.0),
            Vec3::new(sz, cz, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        );

        let n = nb_vertices.max(3);

        let mut angles = Vec::with_capacity(n);
        let mut r = 0.0;
        let inc = (2.0 * PI) / (n as f64);
        for _i in 0..n {
            angles.push(r);
            r += inc;
        }

        let mut res = Vec::with_capacity(n);
        for a in angles {
            // arc at north pole.
            let a_np = Vec3::new(-rm * a.cos(), rm * a.sin(), z);
            // rotate each point to arc centre.
            let a_cen = (a_np * ry) * rz;

            let p = NVector::new(a_cen.unit());
            res.push(p);
        }
        res
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
    fn from_triangle() {
        let a = NVector::from_lat_long_degrees(0.0, 0.0);
        let b = NVector::from_lat_long_degrees(20.0, 0.0);
        let c = NVector::from_lat_long_degrees(10.0, 10.0);
        let cap = Cap::from_triangle(a, b, c);
        assert!(cap.contains_point(a));
        assert!(cap.contains_point(b));
        assert!(cap.contains_point(c));

        let o = Cap::from_triangle(c, b, a);
        assert_nv_eq_d7(o.centre, cap.centre);
        assert!((o.radius.length2() - cap.radius.length2()).abs() < 1e-16);
    }

    #[test]
    fn complement() {
        let np = NVector::from_lat_long_degrees(90.0, 0.0);
        let sp = NVector::from_lat_long_degrees(-90.0, 0.0);
        let northern = Cap::from_centre_and_radius(np, Angle::QUARTER_CIRCLE);
        let southern = Cap::from_centre_and_radius(sp, Angle::QUARTER_CIRCLE);

        let northern_complement = northern.complement();
        assert_eq!(southern.centre, northern_complement.centre);
        assert!((southern.radius.length2() - northern_complement.radius.length2()).abs() < 1e15);

        let southern_complement = southern.complement();
        assert_eq!(northern.centre, southern_complement.centre);
        assert!((northern.radius.length2() - southern_complement.radius.length2()).abs() < 1e15);
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

    #[test]
    fn boundary() {
        assert!(Cap::EMPTY.boundary(1).is_empty());
        assert!(Cap::FULL.boundary(1).is_empty());
        let northern = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(90.0, 0.0),
            Angle::QUARTER_CIRCLE,
        );
        assert_eq!(
            vec![
                LatLong::from_degrees(0.0, 180.0),
                LatLong::from_degrees(0.0, 90.0),
                LatLong::from_degrees(0.0, 0.0),
                LatLong::from_degrees(0.0, -90.0)
            ],
            northern
                .boundary(4)
                .iter()
                .map(|v| LatLong::from_nvector(*v).round_d7())
                .collect::<Vec<_>>()
        );
        assert_eq!(
            vec![
                LatLong::from_degrees(0.0, 180.0),
                LatLong::from_degrees(0.0, 60.0),
                LatLong::from_degrees(0.0, -60.0)
            ],
            northern
                .boundary(2)
                .iter()
                .map(|v| LatLong::from_nvector(*v).round_d7())
                .collect::<Vec<_>>()
        );
    }
}
