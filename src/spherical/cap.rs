use std::f64::consts::PI;

use crate::{
    Angle, LatLong, Mat33, NVector, Prng, Vec3,
    numbers::{gte, lte},
    spherical::Side,
};

use super::{ChordLength, Sphere};

/// A [spherical cap](https://en.wikipedia.org/wiki/Spherical_cap): a portion of a sphere cut off by a plane.
///
/// This struct and implementation is very much based on [S2Cap](https://github.com/google/s2geometry/blob/master/src/s2/s2cap.h).
#[derive(PartialEq, Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))] // codecov:ignore:this
pub struct Cap {
    centre: NVector,
    radius: ChordLength,
}

impl Cap {
    /// Empty spherical cap: contains no position.
    pub const EMPTY: Cap = Self {
        centre: NVector::new(Vec3::UNIT_Z),
        radius: ChordLength::NEGATIVE,
    };

    /// Full spherical cap: contains all positions.
    pub const FULL: Cap = Self {
        centre: NVector::new(Vec3::UNIT_Z),
        radius: ChordLength::MAX,
    };

    /// Constructs a new cap from the given centre and a radius of `0`.
    pub fn from_centre(centre: NVector) -> Self {
        Self {
            centre,
            radius: ChordLength::ZERO,
        }
    }

    /// Constructs a new cap from the 2 given positions which are diametrically
    /// opposite on the boundary.
    ///
    /// Returns [None] If the given positions are the antipode of each other, since
    /// those positions do not **uniquely** determine a cap.
    pub fn from_boundary_positions(
        boundary_position1: NVector,
        boundary_position2: NVector,
    ) -> Option<Self> {
        Sphere::mean_position(&[boundary_position1, boundary_position2]).map(|centre| {
            let r1 = ChordLength::new(centre, boundary_position1);
            let r2 = ChordLength::new(centre, boundary_position2);
            Self {
                centre,
                radius: r1.max(r2),
            }
        })
    }

    /// Constructs a new cap from the given centre and given radius expressed as the angle between the
    /// centre and all positions on the boundary of the cap.
    pub fn from_centre_and_radius(centre: NVector, radius: Angle) -> Self {
        Self {
            centre,
            radius: ChordLength::from_angle(radius),
        }
    }

    /// Constructs a new cap from the given centre and a given position on the boundary of the cap.
    pub fn from_centre_and_boundary_position(centre: NVector, boundary_position: NVector) -> Self {
        Self {
            centre,
            radius: ChordLength::new(centre, boundary_position),
        }
    }

    /// Constructs the smallest cap whose boundary contains the three given positions.
    ///
    /// For three distinct, non-collinear positions, the boundary of the returned cap
    /// is the circumcircle of the spherical triangle defined by the positions.
    ///
    /// Degenerate inputs are handled as follows:
    ///
    /// - If all three positions coincide, a zero-radius cap centred on that position
    ///   is returned.
    /// - If two positions coincide, the result is the two-position boundary cap
    ///   defined by the distinct positions, unless the third position is the antipode
    ///   of the other position, in which case [None] is returned.
    /// - If the three positions lie on a common great circle, the returned cap is
    ///   the hemisphere determined by their orientation.
    ///
    /// The three positions do not need to be supplied in any particular order.
    /// Internally, their orientation is used to select the hemisphere containing
    /// the triangle.
    pub fn from_triangle(a: NVector, b: NVector, c: NVector) -> Option<Self> {
        Self::_from_triangle(a, b, c).0
    }

    /// see `from_triangle`: returns the enclosing cap and `true` if a, b and c are collinear.
    fn _from_triangle(a: NVector, b: NVector, c: NVector) -> (Option<Self>, bool) {
        if a == b {
            return if a == c {
                (Some(Self::from_centre(a)), false)
            } else {
                (Self::from_boundary_positions(a, c), false)
            };
        }

        if a == c {
            return (Self::from_boundary_positions(a, b), false);
        }

        if b == c {
            return (Self::from_boundary_positions(a, b), false);
        }

        // see STRIPACK: http://orion.math.iastate.edu/burkardt/f_src/stripack/stripack.f90
        // 3 positions must be in anti-clockwise order
        let side = Sphere::side(a, b, c);
        let clockwise = side == Side::Right;
        let collinear = side == Side::Collinear;
        let v1 = a.as_vec3();
        let v2 = if clockwise { c.as_vec3() } else { b.as_vec3() };
        let v3 = if clockwise { b.as_vec3() } else { c.as_vec3() };
        let e1 = v2 - v1;
        let e2 = v3 - v1;
        let centre = NVector::new(e1.orthogonal_to(e2));
        // all chord lengths should be equal, still take maximum to account for floating point errors.
        let radius: ChordLength = ChordLength::new(a, centre)
            .max(ChordLength::new(b, centre).max(ChordLength::new(c, centre)));
        (Some(Self { centre, radius }), collinear)
    }

    /// Returns the smallest cap that contains all of the given positions, using
    /// [Flemming's algorithm](https://arxiv.org/abs/2407.19840), which is an adaptation of
    /// [Welzl's algorithm](https://en.wikipedia.org/wiki/Smallest-circle_problem#Welzl's_algorithm), suitable
    /// for the sphere.
    ///
    /// The input positions are deterministically shuffled before the algorithm is
    /// applied. This preserves reproducibility while providing the randomized
    /// ordering required for expected linear-time performance.
    ///
    /// Returns [None] if the positions cannot all be contained in a hemisphere.
    ///
    /// An empty input returns [`Cap::EMPTY`]. A single position returns a
    /// zero-radius cap centred on that position.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::Cap;
    ///
    /// let p1 = NVector::from_lat_long_degrees(0.0, 0.0);
    /// let p2 = NVector::from_lat_long_degrees(10.0, 0.0);
    /// let p3 = NVector::from_lat_long_degrees(5.0, 5.0);
    ///
    /// let cap = Cap::smallest_enclosing_cap(&[p1, p2, p3]).unwrap();
    /// assert!(cap.contains_position(p1));
    /// assert!(cap.contains_position(p2));
    /// assert!(cap.contains_position(p3));
    /// ```
    pub fn smallest_enclosing_cap(ps: &[NVector]) -> Option<Self> {
        if ps.is_empty() {
            return Some(Self::EMPTY);
        }
        let seed = (ps.len() as u32).wrapping_mul(0x85EB_CA6B);
        let mut rand = Prng::new(seed);
        let mut shuffled = ps.to_vec();
        rand.shuffle(&mut shuffled);

        let mut cap = Cap::from_centre(shuffled[0]);

        for i in 1..shuffled.len() {
            let q1 = shuffled[i];
            if !cap.contains_position(q1) {
                // points[i] lies outside the current minimum cap, so any minimum
                // enclosing cap for points[0..=i] must have points[i] on its boundary.
                cap = Cap::from_centre(q1);

                for j in 0..i {
                    let q2 = shuffled[j];
                    if !cap.contains_position(q2) {
                        // if None => State (a): Initial sphere is constructed from two antipodal points.
                        cap = Cap::from_boundary_positions(q1, q2)?;

                        for k in 0..j {
                            let q3 = shuffled[k];
                            if !cap.contains_position(q3) {
                                let (opt_cap, collinear) = Cap::_from_triangle(q1, q2, q3);
                                if collinear {
                                    // State (b): The three boundary points lie on a great circle.
                                    return None;
                                }
                                cap = opt_cap?;
                                // State (c): The condition |B|=4 is satisfied. The 3-point circle
                                // does not enclose all processed points, breaking the hemisphere bound.
                                if shuffled.iter().any(|p| !cap.contains_position(*p)) {
                                    return None;
                                }
                            }
                        }
                    }
                }
            }
        }

        Some(cap)
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
                radius: self.radius.complement(),
            }
        }
    }

    /// Determines whether this cap contains the given position (including the boundary).
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::Cap;
    ///
    /// let cap = Cap::from_centre_and_boundary_position(
    ///     NVector::from_lat_long_degrees(90.0, 0.0),
    ///     NVector::from_lat_long_degrees(0.0, 0.0)
    /// );
    ///
    /// assert!(cap.contains_position(NVector::from_lat_long_degrees(0.0, 0.0)));
    /// assert!(cap.contains_position(NVector::from_lat_long_degrees(45.0, 45.0)));
    /// ```
    pub fn contains_position(&self, p: NVector) -> bool {
        lte(
            ChordLength::new(self.centre, p).length2(),
            self.radius.length2(),
        )
    }

    /// Determines whether the interior of this cap contains the given position.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::NVector;
    /// use jord::spherical::Cap;
    ///
    /// let cap = Cap::from_centre_and_boundary_position(
    ///     NVector::from_lat_long_degrees(90.0, 0.0),
    ///     NVector::from_lat_long_degrees(0.0, 0.0)
    /// );
    ///
    /// assert!(!cap.interior_contains_position(NVector::from_lat_long_degrees(0.0, 0.0)));
    /// assert!(cap.interior_contains_position(NVector::from_lat_long_degrees(45.0, 45.0)));
    /// ```
    pub fn interior_contains_position(&self, p: NVector) -> bool {
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
    /// let cap1 = Cap::from_centre_and_boundary_position(
    ///     NVector::from_lat_long_degrees(90.0, 0.0),
    ///     NVector::from_lat_long_degrees(0.0, 0.0)
    /// );
    ///
    /// let cap2 = Cap::from_centre_and_boundary_position(
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
            let o = ChordLength::new(self.centre, other.centre) + other.radius;
            gte(self.radius.length2(), o.length2())
        }
    }

    /// Return true if and only if this cap intersects the given other cap,
    /// i.e. whether they have any points in common. If either cap is empty, this
    /// method returns false.
    pub fn intersects(&self, other: Self) -> bool {
        if self.is_empty() || other.is_empty() {
            return false;
        }
        let r1 = self.radius + other.radius;
        let r2 = ChordLength::new(self.centre, other.centre);
        gte(r1.length2(), r2.length2())
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

    /// Return a cap that contains all points within a given distance of this
    /// cap. Note that any expansion of the empty cap is still empty.
    pub fn expand(&self, distance: Angle) -> Self {
        if self.is_empty() {
            return Cap::EMPTY;
        }
        Cap {
            centre: self.centre,
            radius: self.radius + ChordLength::from_angle(distance),
        }
    }

    /// Returns the centre of this cap.
    pub fn centre(&self) -> NVector {
        self.centre
    }

    /// Returns the height of the cap, i.e. the distance from the center point to
    /// the cutoff plane.
    pub fn height(&self) -> f64 {
        if self.is_empty() {
            0.0
        } else {
            0.5 * self.radius.length2()
        }
    }

    /// Returns the radius of this cap: central angle between the centre of this cap and
    /// any position on the boundary (negative for [empty](crate::spherical::Cap::EMPTY) caps).
    /// The returned value may not exactly equal the value passed to [`from_centre_and_boundary_position`](crate::spherical::Cap::from_centre_and_boundary_position).
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f64::consts::PI;
    ///
    /// use jord::{Angle, NVector};
    /// use jord::spherical::Cap;
    ///
    /// let cap = Cap::from_centre_and_boundary_position(
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
            // rotate each position to arc centre.
            let a_cen = (a_np * ry) * rz;

            let p = NVector::new(a_cen.unit());
            res.push(p);
        }
        res
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        Angle, LatLong, NVector, Vec3, numbers::eq, positions::assert_nv_eq_d7, spherical::Cap,
    };
    use std::f64::consts::PI;

    #[test]
    fn full() {
        assert!(Cap::FULL.is_full());
        assert!(!Cap::FULL.is_empty());
        assert!(Cap::FULL.contains_position(NVector::from_lat_long_degrees(90.0, 0.0)));
        assert!(Cap::FULL.contains_position(NVector::from_lat_long_degrees(-90.0, 0.0)));
        assert_eq!(Angle::from_radians(PI), Cap::FULL.radius());
        assert_eq!(Cap::EMPTY, Cap::FULL.complement());
    }

    #[test]
    fn empty() {
        assert!(Cap::EMPTY.is_empty());
        assert!(!Cap::EMPTY.is_full());
        assert!(!Cap::EMPTY.contains_position(NVector::from_lat_long_degrees(90.0, 0.0)));
        assert!(!Cap::EMPTY.contains_position(NVector::from_lat_long_degrees(-90.0, 0.0)));
        assert_eq!(Angle::from_radians(-1.0), Cap::EMPTY.radius());
        assert_eq!(Cap::FULL, Cap::EMPTY.complement());
    }

    #[test]
    fn from_centre() {
        let c = NVector::from_lat_long_degrees(20.0, 0.0);
        let cap = Cap::from_centre(c);
        assert!(!cap.is_empty());
        assert!(!cap.is_full());
        assert!(cap.contains_position(c));
        assert!(!cap.contains_position(NVector::from_lat_long_degrees(20.00001, 0.0)));
    }

    #[test]
    fn from_boundary_positions() {
        let p1 = NVector::from_lat_long_degrees(10.0, 0.0);
        let p2 = NVector::from_lat_long_degrees(20.0, 0.0);
        let cap = Cap::from_boundary_positions(p1, p2).unwrap();
        assert!(cap.contains_position(p1));
        assert!(cap.contains_position(p2));
    }

    #[test]
    fn from_boundary_positions_antipodal() {
        let p1 = NVector::from_lat_long_degrees(10.0, 0.0);
        let p2 = p1.antipode();
        assert_eq!(None, Cap::from_boundary_positions(p1, p2));
    }

    #[test]
    fn from_boundary_positions_coincident() {
        let p = NVector::from_lat_long_degrees(10.0, 0.0);
        assert!(
            Cap::from_boundary_positions(p, p)
                .unwrap()
                .contains_position(p)
        );
    }

    #[test]
    fn from_triangle() {
        let a: NVector = NVector::from_lat_long_degrees(0.0, 0.0);
        let b = NVector::from_lat_long_degrees(20.0, 0.0);
        let c = NVector::from_lat_long_degrees(10.0, 10.0);
        let cap = Cap::from_triangle(a, b, c).unwrap();
        assert!(cap.contains_position(a));
        assert!(cap.contains_position(b));
        assert!(cap.contains_position(c));

        let o = Cap::from_triangle(c, b, a).unwrap();
        assert_nv_eq_d7(o.centre, cap.centre);
        assert!((o.radius.length2() - cap.radius.length2()).abs() < 1e-16);
    }

    #[test]
    fn from_triangle_all_on_great_circle() {
        let a: NVector = NVector::from_lat_long_degrees(0.0, 0.0);
        let b = NVector::from_lat_long_degrees(0.0, 10.0);
        let c = NVector::from_lat_long_degrees(0.0, 20.0);
        let cap = Cap::from_triangle(a, b, c).unwrap();
        assert_eq!(NVector::new(Vec3::UNIT_Z), cap.centre());
        assert_eq!(
            Angle::from_degrees(90.0).round_d7(),
            cap.radius().round_d7()
        );
    }

    #[test]
    fn from_triangle_coincident() {
        let a: NVector = NVector::from_lat_long_degrees(0.0, 0.0);
        let b = NVector::from_lat_long_degrees(0.0, 10.0);
        let c = NVector::from_lat_long_degrees(0.0, 20.0);
        assert_eq!(
            Cap::from_boundary_positions(a, c),
            Cap::from_triangle(a, a, c)
        );
        assert_eq!(
            Cap::from_boundary_positions(a, b),
            Cap::from_triangle(a, b, b)
        );
        assert_eq!(
            Cap::from_boundary_positions(a, b),
            Cap::from_triangle(a, b, a)
        );
        assert_eq!(Cap::from_centre(a), Cap::from_triangle(a, a, a).unwrap());
    }

    #[test]
    fn from_triangle_antipodal() {
        let a: NVector = NVector::from_lat_long_degrees(0.0, 0.0);
        let b = NVector::from_lat_long_degrees(0.0, 20.0);
        let cap = Cap::from_triangle(a, a.antipode(), b).unwrap();
        assert_eq!(NVector::from_lat_long_degrees(-90.0, 0.0), cap.centre());
        assert_eq!(Angle::from_degrees(90.0), cap.radius().round_d7());
    }

    #[test]
    fn from_triangle_coincident_and_antipodal() {
        let a = NVector::from_lat_long_degrees(10.0, 20.0);
        let anti_a = a.antipode();
        assert_eq!(None, Cap::from_triangle(a, a, anti_a)); // a == b branch
        assert_eq!(None, Cap::from_triangle(a, anti_a, a)); // a == c branch
        assert_eq!(None, Cap::from_triangle(a, anti_a, anti_a)); // b == c branch
    }

    #[test]
    fn complement() {
        let np = NVector::from_lat_long_degrees(90.0, 0.0);
        let sp = NVector::from_lat_long_degrees(-90.0, 0.0);
        let northern = Cap::from_centre_and_radius(np, Angle::QUARTER_CIRCLE);
        let southern = Cap::from_centre_and_radius(sp, Angle::QUARTER_CIRCLE);

        let northern_complement = northern.complement();
        assert_eq!(southern.centre, northern_complement.centre);
        assert!(southern.radius.length2() - northern_complement.radius.length2() < 1e-15);

        let southern_complement = southern.complement();
        assert_eq!(northern.centre, southern_complement.centre);
        assert!(northern.radius.length2() - southern_complement.radius.length2() < 1e-15);
    }

    #[test]
    fn contains_position() {
        let cap = Cap::from_centre_and_boundary_position(
            NVector::from_lat_long_degrees(90.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 0.0),
        );
        assert!(cap.contains_position(NVector::from_lat_long_degrees(0.0, 0.0)));
        assert!(cap.contains_position(NVector::from_lat_long_degrees(45.0, 45.0)));
    }

    #[test]
    fn interior_contains_position() {
        let cap = Cap::from_centre_and_boundary_position(
            NVector::from_lat_long_degrees(90.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 0.0),
        );
        assert!(!cap.interior_contains_position(NVector::from_lat_long_degrees(0.0, 0.0)));
        assert!(cap.interior_contains_position(NVector::from_lat_long_degrees(45.0, 45.0)));
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
    fn does_not_contains_cap() {
        let c = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(0.0, 0.0),
            Angle::from_degrees(90.0),
        );
        let o = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(50.0, 0.0),
            Angle::from_degrees(45.0),
        );
        // centre distance 50° + other's radius 45° = 95° > self's radius 90°, so self should
        // NOT contain other -- other's farthest point sits outside self's boundary.
        assert!(!c.contains_cap(o));
    }

    #[test]
    fn contains_cap_tangent_internally() {
        let outer = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(0.0, 0.0),
            Angle::from_degrees(20.0),
        );
        let inner = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(0.0, 10.0),
            Angle::from_degrees(10.0),
        );
        assert!(outer.contains_cap(inner));
    }

    #[test]
    fn intersects() {
        let a = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(50.0, 10.0),
            Angle::from_degrees(0.2),
        );
        assert!(!a.intersects(Cap::EMPTY));
        assert!(!Cap::EMPTY.intersects(a));
        assert!(Cap::FULL.intersects(a));
        assert!(a.intersects(Cap::FULL));

        // a and b are disjoint.
        let b = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(51.0, 11.0),
            Angle::from_degrees(0.1),
        );
        assert!(!a.intersects(b));
        assert!(!b.intersects(a));

        // a and c are partially overlapping caps.
        let c = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(50.3, 10.3),
            Angle::from_degrees(0.2),
        );
        assert!(c.intersects(a));
        assert!(a.intersects(c));
    }

    #[test]
    fn intersects_tangent() {
        let a = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(0.0, 0.0),
            Angle::from_degrees(10.0),
        );
        let b = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(0.0, 20.0),
            Angle::from_degrees(10.0),
        );
        // radius + radius == distance exactly: caps touch at a single point.
        assert!(a.intersects(b));
        assert!(b.intersects(a));
    }

    #[test]
    fn expand() {
        assert_eq!(Cap::EMPTY, Cap::EMPTY.expand(Angle::from_degrees(1.0)));
        let c = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(0.0, 0.0),
            Angle::from_degrees(1.0),
        );
        assert!(!c.contains_position(NVector::from_lat_long_degrees(0.0, 1.5)));
        let e = c.expand(Angle::from_degrees(1.0));
        assert!(e.contains_position(NVector::from_lat_long_degrees(0.0, 1.5)));

        assert_eq!(
            Cap::FULL,
            Cap::from_centre_and_radius(
                NVector::from_lat_long_degrees(90.0, 0.0),
                Angle::from_degrees(179.0)
            )
            .expand(Angle::from_degrees(2.0))
        );
    }

    #[test]
    fn radius() {
        assert_eq!(
            Angle::QUARTER_CIRCLE,
            Cap::from_centre_and_boundary_position(
                NVector::from_lat_long_degrees(90.0, 0.0),
                NVector::from_lat_long_degrees(0.0, 0.0)
            )
            .radius()
            .round_d7()
        );
        assert_eq!(
            Angle::from_radians(PI / 4.0),
            Cap::from_centre_and_boundary_position(
                NVector::from_lat_long_degrees(90.0, 0.0),
                NVector::from_lat_long_degrees(45.0, 45.0)
            )
            .radius()
            .round_d7()
        );
    }

    #[test]
    fn height() {
        assert_eq!(0.0, Cap::EMPTY.height());
        assert_eq!(2.0, Cap::FULL.height());
        let c = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(0.0, 0.0),
            Angle::from_degrees(90.0),
        );
        assert!(eq(1.0, c.height()));
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
    fn union_contained() {
        let a = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(0.0, 0.0),
            Angle::from_degrees(1.0),
        );
        let b = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(0.0, 0.0),
            Angle::from_degrees(0.5),
        );
        let c = Cap::from_centre_and_radius(
            NVector::from_lat_long_degrees(0.1, 0.1),
            Angle::from_degrees(0.2),
        );
        assert_eq!(a, a.union(b));
        assert_eq!(a, b.union(a));
        assert_eq!(a, c.union(a));
        assert_eq!(a, a.union(c));
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

    #[test]
    fn smallest_enclosing_cap_empty() {
        assert_eq!(Cap::EMPTY, Cap::smallest_enclosing_cap(&[]).unwrap());
    }

    #[test]
    fn smallest_enclosing_cap_single() {
        let p = NVector::from_lat_long_degrees(10.0, 20.0);
        assert_smallest_enclosing_cap(&[p], p, Angle::ZERO);
    }

    #[test]
    fn smallest_enclosing_cap_two_positions() {
        let p1 = NVector::from_lat_long_degrees(0.0, 0.0);
        let p2 = NVector::from_lat_long_degrees(10.0, 0.0);
        assert_smallest_enclosing_cap(
            &[p1, p2],
            NVector::from_lat_long_degrees(5.0, 0.0),
            Angle::from_degrees(5.0),
        );
    }

    #[test]
    fn smallest_enclosing_cap_obtuse_triangle() {
        let a = NVector::from_lat_long_degrees(0.0, 0.0);
        let b = NVector::from_lat_long_degrees(10.0, 0.0);
        let c = NVector::from_lat_long_degrees(2.0, 0.0);
        assert_smallest_enclosing_cap(
            &[a, b, c],
            NVector::from_lat_long_degrees(5.0, 0.0),
            Angle::from_degrees(5.0),
        );
    }

    #[test]
    fn smallest_enclosing_cap_multiple_positions() {
        let ps = vec![
            NVector::from_lat_long_degrees(10.0, 10.0),
            NVector::from_lat_long_degrees(12.0, 15.0),
            NVector::from_lat_long_degrees(14.0, 11.0),
            NVector::from_lat_long_degrees(9.0, 13.0),
            NVector::from_lat_long_degrees(11.0, 12.0),
        ];
        assert_smallest_enclosing_cap(
            &ps,
            NVector::from_lat_long_degrees(11.573_357_8, 12.257_137_1),
            Angle::from_degrees(2.718_696_6),
        );
    }

    #[test]
    fn smallest_enclosing_cap_collinear() {
        let ps = vec![
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(0.0, 120.0),
            NVector::from_lat_long_degrees(0.0, 240.0),
        ];
        assert_eq!(None, Cap::smallest_enclosing_cap(&ps));
    }

    #[test]
    fn smallest_enclosing_cap_collinear_valid() {
        let ps = vec![
            NVector::from_lat_long_degrees(0.0, 5.0),
            NVector::from_lat_long_degrees(0.0, 45.0),
            NVector::from_lat_long_degrees(0.0, 85.0),
            NVector::from_lat_long_degrees(0.0, 125.0),
            NVector::from_lat_long_degrees(0.0, 165.0),
        ];
        // a valid cap is produced despite all points begin collinear,
        // because all gaps are 40°, expect the wrap‑around gap which
        // is 200°: the minimal covering arc is 360° - 200° = 160°, running
        // from 5° to 165° the "short way" — giving a cap radius of 80°,
        // centered at latitude 0° and longitude 85°.
        assert_smallest_enclosing_cap(
            &ps,
            NVector::from_lat_long_degrees(0.0, 85.0),
            Angle::from_degrees(80.0),
        );
    }

    #[test]
    fn smallest_enclosing_cap_regular_tetrahedron() {
        let s = 1.0 / 3.0_f64.sqrt();
        let ps = [
            NVector::new(Vec3::new(s, s, s).unit()),
            NVector::new(Vec3::new(s, -s, -s).unit()),
            NVector::new(Vec3::new(-s, s, -s).unit()),
            NVector::new(Vec3::new(-s, -s, s).unit()),
        ];
        assert_eq!(None, Cap::smallest_enclosing_cap(&ps));
    }

    #[test]
    fn smallest_enclosing_cap_duplicates() {
        let a = NVector::from_lat_long_degrees(0.0, 0.0);
        let b = NVector::from_lat_long_degrees(54.0, 154.0);
        let c = NVector::from_lat_long_degrees(55.0, 155.0);

        let e = Cap::smallest_enclosing_cap(&[a, b, c]).unwrap();
        assert_eq!(e, Cap::smallest_enclosing_cap(&[a, b, b, c]).unwrap());
        assert_eq!(
            e,
            Cap::smallest_enclosing_cap(&[a, a, a, b, b, c, c]).unwrap()
        );
    }

    #[test]
    fn smallest_enclosing_cap_antipodal() {
        let a = NVector::from_lat_long_degrees(90.0, 0.0);
        let b = NVector::from_lat_long_degrees(-90.0, 0.0);
        assert_eq!(None, Cap::smallest_enclosing_cap(&[a, b]));

        let c = NVector::from_lat_long_degrees(54.0, 154.0);
        assert_eq!(None, Cap::smallest_enclosing_cap(&[c, c.antipode()]));
    }

    #[test]
    fn smallest_enclosing_cap_two_positions_is_minimal() {
        let p1 = NVector::from_lat_long_degrees(0.0, 0.0);
        let p2 = NVector::from_lat_long_degrees(0.0, 20.0);
        assert_smallest_enclosing_cap(
            &[p1, p2],
            NVector::from_lat_long_degrees(0.0, 10.0),
            Angle::from_degrees(10.0),
        );
    }

    // Exercises the genuine 3-point / circumcircle branch: an acute triangle
    // where no 2-point cap can contain the third point, so the algorithm must
    // go all the way to `from_triangle` and the result must be the exact
    // circumcircle, not something larger.
    #[test]
    fn smallest_enclosing_cap_acute_triangle_is_minimal() {
        let a = NVector::from_lat_long_degrees(60.0, 0.0);
        let b = NVector::from_lat_long_degrees(60.0, 120.0);
        let c = NVector::from_lat_long_degrees(60.0, 240.0);
        assert_smallest_enclosing_cap(
            &[a, b, c],
            NVector::from_lat_long_degrees(90.0, 10.0),
            Angle::from_degrees(30.0),
        );
    }

    // An obtuse, non-collinear triangle: the minimal cap must still be the
    // 2-point diameter cap of the longest side, not the (larger) circumcircle.
    #[test]
    fn smallest_enclosing_cap_obtuse_triangle_is_minimal() {
        let a = NVector::from_lat_long_degrees(0.0, 0.0);
        let b = NVector::from_lat_long_degrees(0.0, 20.0);
        let c = NVector::from_lat_long_degrees(0.5, 10.0); // near the midpoint, off the equator
        assert_smallest_enclosing_cap(
            &[a, b, c],
            NVector::from_lat_long_degrees(0.0, 10.0),
            Angle::from_degrees(10.0),
        );
    }

    fn assert_smallest_enclosing_cap(ps: &[NVector], centre: NVector, radius: Angle) {
        let cap = Cap::smallest_enclosing_cap(ps).unwrap();
        // cannot be the full sphere.
        assert_ne!(Cap::FULL, cap);
        // must contain all points.
        for p in ps {
            assert!(cap.contains_position(*p));
        }
        assert_nv_eq_d7(centre, cap.centre());
        assert_eq!(radius, cap.radius().round_d7());
    }
}
