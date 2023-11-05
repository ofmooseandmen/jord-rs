use std::f64::consts::PI;

use crate::{
    surface::Surface, Angle, Cartesian3DVector, GeocentricPos, GeodeticPos, Length, NVector, Vec3,
};

use super::{
    base::{angle_radians_between, easting, side, side_exact},
    GreatCircle, MinorArc,
};

/// A sphere; for most use cases, a sphere is an acceptable approximation of the figure of a cellestial body (e.g. Earth).
///
/// [Sphere] implements several usefull navigation algorithms.
#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct Sphere {
    radius: Length,
}

impl Sphere {
    /// Spherical Earth model using the IUGG (International Union of Geodesy and Geophysics) Earth volumic radius - generally accepted
    /// as the Earth radius when assuming a spherical model.
    /// Note: this is equal to the volumetric radius of the ubiquous WGS84 ellipsoid rounded to 1 decimal.
    pub const EARTH: Sphere = Sphere {
        radius: Length::from_metres(6_371_000.8f64),
    };

    /// Spherical Moon model using the IAU/IAG radius.
    pub const MOON: Sphere = Sphere {
        radius: Length::from_metres(1_737_400.0f64),
    };

    /// Creates a new [Sphere] with the given radius.
    pub fn new(radius: Length) -> Self {
        Sphere { radius }
    }

    /// Returns the radius of this sphere.
    pub fn radius(&self) -> Length {
        self.radius
    }

    /// Computes how far the given position is along a path described by the given minor arc: if a
    /// perpendicular is drawn from the position to the path, the along-track distance is the
    /// signed distance from the start point to where the perpendicular crosses the path.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{LatLong, Length};
    /// use jord::spherical::{MinorArc, Sphere};
    ///
    /// let p = LatLong::from_degrees(53.2611, -0.7972).to_nvector();
    /// let start = LatLong::from_degrees(53.3206, -1.7297).to_nvector();
    /// let end = LatLong::from_degrees(53.1887, 0.1334).to_nvector();
    /// let d = Sphere::EARTH.along_track_distance(p, MinorArc::new(start, end));
    /// assert_eq!(Length::from_metres(62331.501), d.round_mm());
    /// ```
    pub fn along_track_distance(&self, p: NVector, ma: MinorArc) -> Length {
        let normal = ma.normal();
        let angle: f64 = angle_radians_between(
            ma.start().as_vec3(),
            normal.cross_prod(p.as_vec3()).cross_prod(normal),
            Some(normal),
        );
        angle * self.radius
    }

    /// Computes the angle between the two given positions, which is also equal to the distance
    /// between these positions on the unit sphere.
    pub fn angle(p1: NVector, p2: NVector) -> Angle {
        Angle::from_radians(angle_radians_between(p1.as_vec3(), p2.as_vec3(), None))
    }

    /// Computes the signed distance from the given position to the given great circle.
    /// Returns a negative length if the position is left of great circle, positive length if the position is right
    /// of great circle; the orientation of the great circle is therefore important.
    ///
    /// # Example:
    ///
    /// ```
    /// use jord::{Angle, LatLong, Length};
    /// use jord::spherical::{GreatCircle, Sphere};
    ///
    /// let p = LatLong::from_degrees(53.2611, -0.7972).to_nvector();
    /// let gc = GreatCircle::from_heading(
    ///     LatLong::from_degrees(53.3206, -1.7297).to_nvector(),
    ///     Angle::from_degrees(96.0)
    /// );
    /// assert_eq!(Length::from_metres(-305.665), Sphere::EARTH.cross_track_distance(p, gc).round_mm());
    /// ```
    // TODO(CL): tests are in great_circle.rs in jord-rs/next
    pub fn cross_track_distance(&self, p: NVector, gc: GreatCircle) -> Length {
        let angle = angle_radians_between(gc.normal(), p.as_vec3(), None);
        (angle - (PI / 2.0)) * self.radius
    }

    /// Computes the destination position from the given position having travelled the given distance on the given
    /// initial bearing (compass angle) (bearing will normally vary before destination is reached).
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f64::consts::PI;
    /// use jord::{Angle, Length, LatLong};
    /// use jord::spherical::Sphere;
    ///
    /// let distance = Sphere::EARTH.radius() * PI / 4.0;
    /// let p = LatLong::from_degrees(90.0, 0.0).to_nvector();
    /// let dest = Sphere::EARTH.destination_pos(p, Angle::from_degrees(180.0), distance);
    ///
    /// assert_eq!(LatLong::from_degrees(45.0, 0.0), LatLong::from_nvector(dest).round_d7());
    /// ```
    pub fn destination_pos(&self, p0: NVector, bearing: Angle, distance: Length) -> NVector {
        if distance == Length::ZERO {
            p0
        } else {
            // east direction vector at p
            let ed = easting(p0.as_vec3());
            // north direction vector at p
            let nd = p0.as_vec3().cross_prod(ed);
            // central angle
            let ta = distance.as_metres() / self.radius.as_metres();
            let bearing_radians = bearing.as_radians();
            // unit vector in the direction of the azimuth
            let dir = nd * bearing_radians.cos() + ed * bearing_radians.sin();
            NVector::new((p0.as_vec3() * ta.cos() + dir * ta.sin()).unit())
        }
    }

    /// Computes the surface distance on the great circle between the two given positions.
    ///
    /// # Examples:
    ///
    /// ```
    /// use jord::{Length, LatLong};
    /// use jord::spherical::Sphere;
    ///
    /// let d = Sphere::EARTH.distance(
    ///   LatLong::from_degrees(90.0, 0.0).to_nvector(),
    ///   LatLong::from_degrees(-90.0, 0.0).to_nvector()
    /// );
    /// assert_eq!(
    ///   Length::from_metres(20_015_089.309),
    ///   d.round_mm()
    /// );
    /// ```
    pub fn distance(&self, p1: NVector, p2: NVector) -> Length {
        Self::angle(p1, p2) * self.radius
    }

    /// Determines whether the 2 given positions define a unique great circle: i.e. they are
    /// not equal nor the antipode of one another.
    pub fn is_great_circle(p1: NVector, p2: NVector) -> bool {
        p1 != p2 && !p1.is_antipode_of(p2)
    }

    /// Computes the final bearing arriving at `p2` from `p1` in compass angle.
    /// Compass angles are clockwise angles from true north: 0 = north, 90 = east, 180 = south, 270 = west.
    /// The final bearing will differ from the initial bearing by varying degrees according to distance and latitude.
    /// Returns 0 if both positions are equal or the antipode of each other - [is_great_cirle](crate::spherical::Sphere::is_great_circle).
    /// # Examples:
    ///
    /// ```
    /// use jord::{Angle, LatLong};
    /// use jord::spherical::Sphere;
    ///
    /// assert_eq!(
    ///   Angle::from_degrees(90.0),
    ///   Sphere::final_bearing(LatLong::from_degrees(0.0, 0.0).to_nvector(), LatLong::from_degrees(0.0, 1.0).to_nvector())
    /// );
    /// ```
    pub fn final_bearing(p1: NVector, p2: NVector) -> Angle {
        if !Self::is_great_circle(p1, p2) {
            Angle::ZERO
        } else {
            Angle::from_radians(final_bearing_radians(p1, p2)).normalised()
        }
    }

    /// Computes the initial bearing from `p1` to `p2` in compass angle.
    /// Compass angles are clockwise angles from true north: 0 = north, 90 = east, 180 = south, 270 = west.
    /// Returns 0 if both positions are equal or the antipode of each other - [is_great_cirle](crate::spherical::Sphere::is_great_circle)
    ///
    /// # Examples:
    ///
    /// ```
    /// use jord::{Angle, LatLong};
    /// use jord::spherical::Sphere;
    ///
    /// assert_eq!(
    ///   Angle::from_degrees(270.0),
    ///   Sphere::initial_bearing(LatLong::from_degrees(0.0, 1.0).to_nvector(), LatLong::from_degrees(0.0, 0.0).to_nvector())
    /// );
    /// ```
    pub fn initial_bearing(p1: NVector, p2: NVector) -> Angle {
        if !Self::is_great_circle(p1, p2) {
            Angle::ZERO
        } else {
            Angle::from_radians(initial_bearing_radians(p1, p2)).normalised()
        }
    }

    /// Computes the position at given fraction between this position and the given position.
    /// Returns `None` if:
    /// - the fraction is `< 0` or `> 1`,
    /// - this position and the given position are the antipodes of one another.
    pub fn interpolated_pos(p1: NVector, p2: NVector, f: f64) -> Option<NVector> {
        if !(0.0..=1.0).contains(&f) || p1.is_antipode_of(p2) {
            None
        } else if f == 0.0 {
            Some(p1)
        } else if f == 1.0 {
            Some(p2)
        } else {
            // angular distance in radians multiplied by the fraction: how far from v0.
            let distance_radians = f * angle_radians_between(p1.as_vec3(), p2.as_vec3(), None);
            //  a vector representing the direction from v0 to v1.
            let dir = (p1.as_vec3().stable_cross_prod(p2.as_vec3())).cross_prod_unit(p1.as_vec3());
            let v = (p1.as_vec3() * distance_radians.cos() + dir * distance_radians.sin()).unit();
            Some(NVector::new(v))
        }
    }

    /// Computes the mean position of the given positions: the “center of gravity” of the given positions,
    /// which and can be compared to the centroid of a geometrical shape (n.b. other definitions of mean exist).
    ///
    /// The mean position is undefined if either the given vector is empty or some of the given positions are
    /// antipodals.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{LatLong, Length};
    /// use jord::spherical::Sphere;
    ///
    /// let ps = vec![
    ///     LatLong::from_degrees( 10.0,  10.0).to_nvector(),
    ///     LatLong::from_degrees( 10.0, -10.0).to_nvector(),
    ///     LatLong::from_degrees(-10.0, -10.0).to_nvector(),
    ///     LatLong::from_degrees(-10.0,  10.0).to_nvector()
    /// ];
    ///
    /// let o_m = Sphere::mean_position(&ps);
    /// assert!(o_m.is_some());
    /// assert_eq!(
    ///     LatLong::from_degrees(0.0, 0.0),
    ///     LatLong::from_nvector(o_m.unwrap()).round_d7()
    /// );
    /// ```
    pub fn mean_position(ps: &[NVector]) -> Option<NVector> {
        if ps.is_empty() || contains_antipodal(ps) {
            None
        } else if ps.len() == 1 {
            ps.first().cloned()
        } else {
            let vs = ps.iter().map(|nv| nv.as_vec3()).collect::<Vec<_>>();
            let m = Vec3::mean(&vs);
            Some(NVector::new(m))
        }
    }

    /// Returns the angle in radians turned from AB to BC. Angle is positive for left turn,
    /// negative for right turn and 0 if all 3 positions are collinear (i.e. on the same great circle).
    pub fn turn(a: NVector, b: NVector, c: NVector) -> Angle {
        let n1 = Vec3::from_orthogonal(a.as_vec3(), b.as_vec3());
        let n2 = Vec3::from_orthogonal(b.as_vec3(), c.as_vec3());
        Angle::from_radians(angle_radians_between(n1, n2, Some(b.as_vec3())))
    }

    /// Determines whether v0 if right of (negative integer), left of (positive integer) or on the
    /// great circle (zero), from v1 to v2.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::LatLong;
    /// use jord::spherical::Sphere;
    ///
    /// let p1 = LatLong::from_degrees(55.4295, 13.82).to_nvector();
    /// let p2 = LatLong::from_degrees(56.0465, 12.6945).to_nvector();
    /// let p3 = LatLong::from_degrees(56.0294, 14.1567).to_nvector();
    ///
    /// assert_eq!(-1, Sphere::side(p1, p2, p3));
    /// assert_eq!(1, Sphere::side(p1, p3, p2));
    /// ```
    pub fn side(p0: NVector, p1: NVector, p2: NVector) -> i8 {
        side(p0.as_vec3(), p1.as_vec3(), p2.as_vec3())
    }

    /// Similar to `side` but returns the value of the dot product between v0 and the orthogonal
    /// unit-length vector to v1 and v2.
    ///
    /// - if the dot product is nearly-zero or zero, the 3 positions are collinear
    /// - otherwise, if the dot product is negative, v0 is right of (v1, v2)
    /// - otherwise, v0 is left of (v1, v2)
    pub fn side_exact(p0: NVector, p1: NVector, p2: NVector) -> f64 {
        side_exact(p0.as_vec3(), p1.as_vec3(), p2.as_vec3())
    }
}

impl Surface for Sphere {
    fn geodetic_to_geocentric(&self, pos: GeodeticPos) -> GeocentricPos {
        let h = self.radius + pos.height();
        GeocentricPos::from_metres(h.as_metres() * pos.horizontal_position().as_vec3())
    }

    fn geocentric_to_geodetic(&self, pos: GeocentricPos) -> GeodeticPos {
        let h = Length::from_metres(pos.as_metres().norm()) - self.radius;
        GeodeticPos::new(NVector::new(Vec3::unit(pos.as_metres())), h)
    }
}

fn final_bearing_radians(v1: NVector, v2: NVector) -> f64 {
    initial_bearing_radians(v2, v1) + PI
}

fn initial_bearing_radians(v1: NVector, v2: NVector) -> f64 {
    // great circle through v1 & v2.
    let gc1 = v1.as_vec3().cross_prod(v2.as_vec3());

    // this is equivalent to -easting(v1), but avoids the creation of
    // an intermediate Vec3.
    // -y if at pole or great circle through v1 & north pole (v x [0, 0, 1])
    let gc2 = if v1.as_vec3().z().abs() == 1.0 {
        Vec3::NEG_UNIT_Y
    } else {
        Vec3::new(v1.as_vec3().y(), -v1.as_vec3().x(), 0.0)
    };
    angle_radians_between(gc1, gc2, Some(v1.as_vec3()))
}

/// Determines if the given vector contains antipodal positions.
fn contains_antipodal(ps: &[NVector]) -> bool {
    for p in ps {
        let a = p.antipode();
        let found = ps.iter().any(|&o| o == a);
        if found {
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {

    use std::f64::consts::PI;

    /// destination.

    #[test]
    fn destination_across_date_line() {
        let p = NVector::from_lat_long_degrees(0.0, 154.0);
        let actual = Sphere::EARTH.destination_pos(
            p,
            Angle::from_degrees(90.0),
            Length::from_kilometres(5000.0),
        );
        let expected = NVector::from_lat_long_degrees(0.0, -161.0339254);
        assert_nv_eq_d7(expected, actual);
    }

    #[test]
    fn destination_from_north_pole() {
        let expected = NVector::from_lat_long_degrees(45.0, 0.0);
        let distance = Sphere::EARTH.radius() * (PI / 4.0);
        let actual = Sphere::EARTH.destination_pos(
            NVector::from_lat_long_degrees(90.0, 0.0),
            Angle::from_degrees(180.0),
            distance,
        );
        assert_nv_eq_d7(expected, actual);
    }

    #[test]
    fn destination_from_south_pole() {
        let expected = NVector::from_lat_long_degrees(-45.0, 0.0);
        let distance = Sphere::EARTH.radius() * (PI / 4.0);
        let actual = Sphere::EARTH.destination_pos(
            NVector::from_lat_long_degrees(-90.0, 0.0),
            Angle::ZERO,
            distance,
        );
        assert_nv_eq_d7(expected, actual);
    }

    #[test]
    fn destination_negative_distance() {
        let p = NVector::from_lat_long_degrees(0.0, 0.0);
        // equivalent of -10 degree of longitude.
        let d = Sphere::EARTH.radius() * (-2.0 * PI / 36.0);
        let actual = Sphere::EARTH.destination_pos(p, Angle::from_degrees(90.0), d);
        let expected = NVector::from_lat_long_degrees(0.0, -10.0);
        assert_nv_eq_d7(expected, actual);
    }

    #[test]
    fn destination_travelled_longitude_greater_than_90() {
        let p = NVector::from_lat_long_degrees(60.2, 11.1);
        let d = Sphere::EARTH.destination_pos(
            p,
            Angle::from_degrees(12.4),
            Length::from_nautical_miles(2000.0),
        );
        let e = NVector::from_lat_long_degrees(82.6380125, 124.1259551);
        assert_nv_eq_d7(e, d);
    }

    #[test]
    fn destination_zero_distance() {
        let p = NVector::from_lat_long_degrees(55.6050, 13.0038);
        assert_eq!(
            p,
            Sphere::EARTH.destination_pos(p, Angle::from_degrees(96.0217), Length::ZERO,)
        );
    }

    // distance.

    #[test]
    fn distance_accross_date_line() {
        let p1 = NVector::from_lat_long_degrees(50.066389, -179.999722);
        let p2 = NVector::from_lat_long_degrees(50.066389, 179.999722);
        assert_eq!(
            Length::from_metres(39.685),
            Sphere::EARTH.distance(p1, p2).round_mm()
        );
    }

    #[test]
    fn distance_between_poles() {
        assert_eq!(
            Length::from_metres(20_015_089.309),
            Sphere::EARTH
                .distance(
                    NVector::from_lat_long_degrees(90.0, 0.0),
                    NVector::from_lat_long_degrees(-90.0, 0.0)
                )
                .round_mm()
        );
    }

    #[test]
    fn distance_test() {
        let p1 = NVector::from_lat_long_degrees(50.066389, -5.714722);
        let p2 = NVector::from_lat_long_degrees(58.643889, -3.07);
        assert_eq!(
            Length::from_metres(968_853.666),
            Sphere::EARTH.distance(p1, p2).round_mm()
        );
    }

    #[test]
    fn distance_transitivity() {
        let p1 = NVector::from_lat_long_degrees(0.0, 0.0);
        let p2 = NVector::from_lat_long_degrees(0.0, 10.0);
        let p3 = NVector::from_lat_long_degrees(0.0, 20.0);
        let d1 = Sphere::EARTH.distance(p1, p2);
        let d2 = Sphere::EARTH.distance(p2, p3);
        let actual = (d1 + d2).round_mm();
        assert_eq!(actual, Sphere::EARTH.distance(p1, p3).round_mm());
    }

    #[test]
    fn distance_zero() {
        let p = NVector::from_lat_long_degrees(50.066389, -5.714722);
        assert_eq!(Length::ZERO, Sphere::EARTH.distance(p, p));
    }

    /// final_bearing.

    #[test]
    fn final_bearing_at_equator_going_east() {
        let p1 = NVector::from_lat_long_degrees(0.0, 0.0);
        let p2 = NVector::from_lat_long_degrees(0.0, 1.0);
        assert_eq!(Angle::from_degrees(90.0), Sphere::final_bearing(p1, p2));
    }

    #[test]
    fn final_bearing_at_equator_going_west() {
        let p1 = NVector::from_lat_long_degrees(0.0, 1.0);
        let p2 = NVector::from_lat_long_degrees(0.0, 0.0);
        assert_eq!(Angle::from_degrees(270.0), Sphere::final_bearing(p1, p2));
    }

    #[test]
    fn final_bearing_coincidental() {
        let p = NVector::from_lat_long_degrees(50.0, -18.0);
        assert_eq!(Angle::ZERO, Sphere::final_bearing(p, p));
    }

    #[test]
    fn final_bearing_same_longitude_going_north() {
        let p1 = NVector::from_lat_long_degrees(50.0, -5.0);
        let p2 = NVector::from_lat_long_degrees(58.0, -5.0);
        assert_eq!(Angle::ZERO, Sphere::final_bearing(p1, p2));
    }

    #[test]
    fn final_bearing_same_longitude_going_south() {
        let p1 = NVector::from_lat_long_degrees(58.0, -5.0);
        let p2 = NVector::from_lat_long_degrees(50.0, -5.0);
        assert_eq!(Angle::from_degrees(180.0), Sphere::final_bearing(p1, p2));
    }

    #[test]
    fn final_bearing_test() {
        let p1 = NVector::from_lat_long_degrees(50.06638889, -5.71472222);
        let p2 = NVector::from_lat_long_degrees(58.64388889, -3.07);
        assert_eq!(
            Angle::from_degrees(11.2752013),
            Sphere::final_bearing(p1, p2).round_d7()
        );
        assert_eq!(
            Angle::from_degrees(189.1198181),
            Sphere::final_bearing(p2, p1).round_d7()
        );

        let p1 = NVector::from_lat_long_degrees(-53.99472222, -25.9875);
        let p2 = NVector::from_lat_long_degrees(54.0, 154.0);
        assert_eq!(
            Angle::from_degrees(125.6839551),
            Sphere::final_bearing(p1, p2).round_d7()
        );
    }

    // initial_bearing

    #[test]
    fn initial_bearing_antipodal() {
        let np = NVector::from_lat_long_degrees(90.0, 0.0);
        let sp = NVector::from_lat_long_degrees(-90.0, 0.0);
        assert_eq!(Angle::from_degrees(180.0), Sphere::initial_bearing(np, sp));
        assert_eq!(Angle::ZERO, Sphere::initial_bearing(sp, np));
    }

    #[test]
    fn initial_bearing_at_equator_going_east() {
        let p1 = NVector::from_lat_long_degrees(0.0, 0.0);
        let p2 = NVector::from_lat_long_degrees(0.0, 1.0);
        assert_eq!(Angle::from_degrees(90.0), Sphere::initial_bearing(p1, p2));
    }

    #[test]
    fn initial_bearing_at_equator_going_west() {
        let p1 = NVector::from_lat_long_degrees(0.0, 1.0);
        let p2 = NVector::from_lat_long_degrees(0.0, 0.0);
        assert_eq!(Angle::from_degrees(270.0), Sphere::initial_bearing(p1, p2));
    }

    #[test]
    fn initial_bearing_coincidental() {
        let p = NVector::from_lat_long_degrees(50.0, -18.0);
        assert_eq!(Angle::ZERO, Sphere::initial_bearing(p, p));
    }

    #[test]
    fn initial_bearing_from_north_pole() {
        assert_eq!(
            Angle::from_degrees(26.0),
            Sphere::initial_bearing(
                NVector::from_lat_long_degrees(90.0, 0.0),
                NVector::from_lat_long_degrees(50.0, 154.0)
            )
            .round_d7()
        );
    }

    #[test]
    fn initial_bearing_north_pole_to_date_line() {
        assert_eq!(
            Angle::ZERO,
            Sphere::initial_bearing(
                NVector::from_lat_long_degrees(90.0, 0.0),
                NVector::from_lat_long_degrees(50.0, 180.0)
            )
            .round_d7()
        );
    }

    #[test]
    fn initial_bearing_same_longitude_going_north() {
        let p1 = NVector::from_lat_long_degrees(50.0, -5.0);
        let p2 = NVector::from_lat_long_degrees(58.0, -5.0);
        assert_eq!(Angle::ZERO, Sphere::initial_bearing(p1, p2).round_d7());
    }

    #[test]
    fn initial_bearing_same_longitude_going_south() {
        let p1 = NVector::from_lat_long_degrees(58.0, -5.0);
        let p2 = NVector::from_lat_long_degrees(50.0, -5.0);
        assert_eq!(
            Angle::from_degrees(180.0),
            Sphere::initial_bearing(p1, p2).round_d7()
        );
    }

    #[test]
    fn initial_bearing_from_south_pole() {
        assert_eq!(
            Angle::from_degrees(154.0),
            Sphere::initial_bearing(
                NVector::from_lat_long_degrees(-90.0, 0.0),
                NVector::from_lat_long_degrees(50.0, 154.0)
            )
            .round_d7()
        );
    }

    #[test]
    fn initial_bearing_south_pole_to_date_line() {
        assert_eq!(
            Angle::from_degrees(180.0),
            Sphere::initial_bearing(
                NVector::from_lat_long_degrees(-90.0, 0.0),
                NVector::from_lat_long_degrees(50.0, 180.0)
            )
            .round_d7()
        );
    }

    #[test]
    fn initial_bearing_test() {
        let p1 = NVector::from_lat_long_degrees(50.06638889, -5.71472222);
        let p2 = NVector::from_lat_long_degrees(58.64388889, -3.07);
        assert_eq!(
            Angle::from_degrees(9.1198181),
            Sphere::initial_bearing(p1, p2).round_d7()
        );
        assert_eq!(
            Angle::from_degrees(191.2752013),
            Sphere::initial_bearing(p2, p1).round_d7()
        );
    }

    // interpolated

    #[test]
    fn interpolated_antipodal() {
        let p = NVector::from_lat_long_degrees(90.0, 0.0);
        assert!(Sphere::interpolated_pos(p, p.antipode(), 0.5).is_none());
    }

    #[test]
    fn interpolated_f0() {
        let p1 = NVector::from_lat_long_degrees(90.0, 0.0);
        let p2 = NVector::from_lat_long_degrees(0.0, 0.0);
        assert_eq!(Some(p1), Sphere::interpolated_pos(p1, p2, 0.0));
    }

    #[test]
    fn interpolated_f1() {
        let p1 = NVector::from_lat_long_degrees(90.0, 0.0);
        let p2 = NVector::from_lat_long_degrees(0.0, 0.0);
        assert_eq!(Some(p2), Sphere::interpolated_pos(p1, p2, 1.0));
    }

    #[test]
    fn interpolated_invalid_f() {
        let p1 = NVector::from_lat_long_degrees(0.0, 0.0);
        let p2 = NVector::from_lat_long_degrees(1.0, 0.0);
        assert!(Sphere::interpolated_pos(p1, p2, -0.1).is_none());
        assert!(Sphere::interpolated_pos(p1, p2, 1.1).is_none());
    }

    #[test]
    fn interpolated_half() {
        assert_eq!(
            Some(NVector::from_lat_long_degrees(0.0, 0.0)),
            Sphere::interpolated_pos(
                NVector::from_lat_long_degrees(10.0, 0.0),
                NVector::from_lat_long_degrees(-10.0, 0.0),
                0.5
            )
        );
    }

    #[test]
    fn interpolated_side() {
        let p0 = NVector::from_lat_long_degrees(154.0, 54.0);
        let p1 = NVector::from_lat_long_degrees(155.0, 55.0);
        let i = Sphere::interpolated_pos(p0, p1, 0.25).unwrap();
        assert_eq!(0, Sphere::side(i, p0, p1));
    }

    #[test]
    fn interpolated_transitivity() {
        let p0 = NVector::from_lat_long_degrees(10.0, 0.0);
        let p1 = NVector::from_lat_long_degrees(-10.0, 0.0);
        let expected = Sphere::interpolated_pos(p0, p1, 0.5).unwrap();
        let actual = Sphere::interpolated_pos(
            Sphere::interpolated_pos(p0, p1, 0.25).unwrap(),
            p1,
            1.0 / 3.0,
        );
        assert_opt_nv_eq_d7(expected, actual);
    }

    // mean

    use crate::{
        positions::{assert_nv_eq_d7, assert_opt_nv_eq_d7},
        spherical::Sphere,
        Angle, LatLong, Length, NVector, Vec3,
    };

    #[test]
    fn mean_antipodal() {
        let p = NVector::from_lat_long_degrees(0.0, 0.0);
        assert!(Sphere::mean_position(&vec!(p, p.antipode())).is_none());
    }

    #[test]
    fn mean_empty() {
        let vs: Vec<NVector> = Vec::new();
        assert!(Sphere::mean_position(&vs).is_none());
    }

    #[test]
    fn mean_test() {
        let vs = vec![
            NVector::from_lat_long_degrees(10.0, 10.0),
            NVector::from_lat_long_degrees(10.0, -10.0),
            NVector::from_lat_long_degrees(-10.0, -10.0),
            NVector::from_lat_long_degrees(-10.0, 10.0),
        ];

        assert_opt_nv_eq_d7(
            NVector::from_lat_long_degrees(0.0, 0.0),
            Sphere::mean_position(&vs),
        );
    }

    #[test]
    fn mean_one() {
        assert_eq!(
            Some(NVector::from_lat_long_degrees(0.0, 0.0)),
            Sphere::mean_position(&vec!(NVector::from_lat_long_degrees(0.0, 0.0)))
        );
    }

    // side

    #[test]
    fn side_collinear() {
        assert_eq!(
            0,
            Sphere::side(
                NVector::from_lat_long_degrees(0.0, 0.0),
                NVector::from_lat_long_degrees(45.0, 0.0),
                NVector::from_lat_long_degrees(90.0, 0.0)
            )
        );
    }

    #[test]
    fn side_equal() {
        let v1 = NVector::new(Vec3::new_unit(1.0, 2.0, 3.0));
        // largest component is z, orthogonal vector in x-z plan.
        assert_eq!(0, Sphere::side(NVector::new(Vec3::UNIT_Y), v1, v1));
        assert_eq!(
            -1,
            Sphere::side(NVector::new(Vec3::new_unit(1.0, -3.0, 0.0)), v1, v1)
        );
        assert_eq!(
            1,
            Sphere::side(NVector::new(Vec3::new_unit(-1.0, 3.0, 0.0)), v1, v1)
        );
    }

    #[test]
    fn side_same_meridian() {
        let v0 = NVector::from_lat_long_degrees(-78.0, 55.0);
        let v1 = NVector::from_lat_long_degrees(-85.0, 55.0);
        let v2 = NVector::from_lat_long_degrees(10.0, 55.0);
        assert_eq!(0, Sphere::side(v0, v1, v2));
        assert_eq!(0, Sphere::side(v0, v2, v1));
    }

    #[test]
    fn side_opposite() {
        let v1 = NVector::new(Vec3::new_unit(1.0, 2.0, 3.0));
        let v2 = NVector::new(Vec3::new_unit(-1.0, -2.0, -3.0));
        // largest component is z, orthogonal vector in x-z plan.
        assert_eq!(0, Sphere::side(NVector::new(Vec3::UNIT_Y), v1, v2));
        assert_eq!(
            -1,
            Sphere::side(NVector::new(Vec3::new_unit(1.0, -3.0, 0.0)), v1, v2)
        );
        assert_eq!(
            1,
            Sphere::side(NVector::new(Vec3::new_unit(-1.0, 3.0, 0.0)), v1, v2)
        );
    }

    #[test]
    fn side_resolution() {
        // 1 arc microsecond.
        let one_mas = Angle::from_degrees(1.0 / 3600000000.0);

        let lng = Angle::from_degrees(55.0);
        let v1 = NVector::from_lat_long_degrees(-85.0, lng.as_degrees());
        let v2 = NVector::from_lat_long_degrees(10.0, lng.as_degrees());
        let right = LatLong::new(Angle::from_degrees(-78.0), lng + one_mas).to_nvector();
        assert_eq!(-1, Sphere::side(right, v1, v2));
        let left = LatLong::new(Angle::from_degrees(-78.0), lng - one_mas).to_nvector();
        assert_eq!(1, Sphere::side(left, v1, v2));
    }

    // turn

    #[test]
    fn turn_collinear() {
        let actual = Sphere::turn(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(45.0, 0.0),
            NVector::from_lat_long_degrees(90.0, 0.0),
        );
        assert_eq!(Angle::ZERO, actual);
    }

    #[test]
    fn turn_left() {
        let actual = Sphere::turn(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(45.0, 0.0),
            NVector::from_lat_long_degrees(60.0, -10.0),
        );
        assert_eq!(Angle::from_radians(0.3175226173130951), actual);
    }

    #[test]
    fn turn_right() {
        let actual = Sphere::turn(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(45.0, 0.0),
            NVector::from_lat_long_degrees(60.0, 10.0),
        );
        assert_eq!(Angle::from_radians(-0.3175226173130951), actual);
    }
}
