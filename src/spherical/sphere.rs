use std::{f64::consts::PI, time::Duration};

use crate::{
    numbers::eq_zero, surface::Surface, Angle, Cartesian3DVector, GeocentricPos, GeodeticPos,
    LatLong, Length, Mat33, NVector, Speed, Vec3, Vehicle,
};

use super::{
    base::{angle_radians_between, easting, exact_side},
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
    // 1 millisecond in hours.
    const ONE_MILLI_HOURS: f64 = 1.0 / (3_600.0 * 1_000.0);

    // CPA Newton-Raphson maximum number of iteration. */
    const CPA_NR_MAX_ITERATIONS: u64 = 50;

    /// Spherical Earth model using the [IUGG](https://iugg.org) (International Union of Geodesy and Geophysics) Earth volumic radius - generally accepted
    /// as the Earth radius when assuming a spherical model.
    /// Note: this is equal to the volumetric radius of the ubiquous WGS84 ellipsoid rounded to 1 decimal.
    pub const EARTH: Sphere = Sphere::new(Length::from_metres(6_371_000.8f64));

    /// Spherical Moon model using the [IAU/IAG](https://lunar.gsfc.nasa.gov/library/LunCoordWhitePaper-10-08.pdf) radius.
    pub const MOON: Sphere = Sphere::new(Length::from_metres(1_737_400.0f64));

    /// Creates a new [Sphere] with the given radius.
    pub const fn new(radius: Length) -> Self {
        Sphere { radius }
    }

    /// Returns the radius of this sphere.
    #[inline]
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
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, LatLong};
    /// use jord::spherical::Sphere;
    ///
    /// let a = Sphere::angle(
    ///   LatLong::from_degrees(90.0, 0.0).to_nvector(),
    ///   LatLong::from_degrees(-90.0, 0.0).to_nvector()
    /// );
    /// assert_eq!(Angle::HALF_CIRCLE, a);
    /// ```
    pub fn angle(p1: NVector, p2: NVector) -> Angle {
        Angle::from_radians(angle_radians_between(p1.as_vec3(), p2.as_vec3(), None))
    }

    /// Computes the signed distance from the given position to the given great circle.
    /// Returns a negative length if the position is left of great circle, positive length if the position is right
    /// of great circle; the orientation of the great circle is therefore important.
    ///
    /// # Examples
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
    /// # Examples
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

    /// Converts the given great circle distance to the equivalent central angle in the range [0, 180] degrees.
    ///
    /// The given distance is normalised to the range [0, `PI * Sphere::radius`].
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f64::consts::PI;
    /// use jord::{Angle, Length, NVector};
    /// use jord::spherical::Sphere;
    ///
    /// let p1 = NVector::from_lat_long_degrees(54.0, 154.0);
    /// let p2 = NVector::from_lat_long_degrees(55.0, 155.0);
    /// let d = Sphere::EARTH.distance(p1, p2);
    ///
    /// assert_eq!(
    ///   Sphere::EARTH.distance_to_angle(d),
    ///   Sphere::angle(p1, p2)
    /// );
    /// ```
    pub fn distance_to_angle(&self, distance: Length) -> Angle {
        let max_distance = PI * self.radius;
        if distance == max_distance {
            return Angle::HALF_CIRCLE;
        }
        let d = if distance > max_distance {
            distance.as_metres() % max_distance.as_metres()
        } else {
            distance.as_metres()
        };
        Angle::from_radians(d / self.radius.as_metres())
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
    ///
    /// # Examples
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
    /// # Examples
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
    /// The mean position is undefined if:
    /// - no position are given (i.e `ps` is empty), or
    /// - any 2 given positions are the antipode of one another.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::LatLong;
    /// use jord::spherical::Sphere;
    ///
    /// let ps = vec![
    ///     LatLong::from_degrees(10.0,  10.0).to_nvector(),
    ///     LatLong::from_degrees(10.0, -10.0).to_nvector(),
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

    /// Computes the mean position of the 3 given positions: the “center of gravity” of the given positions,
    /// which and can be compared to the centroid of a geometrical shape (n.b. other definitions of mean exist).
    ///
    /// The mean position is undefined if any 2 given positions are the antipode of one another.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{LatLong, NVector};
    /// use jord::spherical::Sphere;
    ///
    /// let p1 = NVector::from_lat_long_degrees(10.0,  0.0);
    /// let p2 = NVector::from_lat_long_degrees(0.0, -10.0);
    /// let p3 = NVector::from_lat_long_degrees(0.0, 10.0);
    ///
    /// let o_m = Sphere::triangle_mean_position(p1, p2, p3);
    /// assert!(o_m.is_some());
    /// assert_eq!(
    ///     LatLong::from_degrees(3.3637274, 0.0),
    ///     LatLong::from_nvector(o_m.unwrap()).round_d7()
    /// );
    /// ```
    pub fn triangle_mean_position(p1: NVector, p2: NVector, p3: NVector) -> Option<NVector> {
        if p1.is_antipode_of(p2) || p1.is_antipode_of(p3) || p2.is_antipode_of(p3) {
            None
        } else {
            Some(NVector::new(Vec3::mean(&[
                p1.as_vec3(),
                p2.as_vec3(),
                p3.as_vec3(),
            ])))
        }
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
        let side = exact_side(p0.as_vec3(), p1.as_vec3(), p2.as_vec3());
        if eq_zero(side) {
            0
        } else if side < 0.0 {
            -1
        } else {
            1
        }
    }

    /// Returns the angle turned from AB to BC. Angle is positive for left turn,
    /// negative for right turn and 0 if all 3 positions are collinear (i.e. on the same great circle).
    pub fn turn(a: NVector, b: NVector, c: NVector) -> Angle {
        let n1 = a.as_vec3().orthogonal_to(b.as_vec3());
        let n2 = b.as_vec3().orthogonal_to(c.as_vec3());
        Angle::from_radians(angle_radians_between(n1, n2, Some(b.as_vec3())))
    }

    // kinematics

    /// Calculates the position that the given vehicle will reach after the given time.
    pub fn position_after(&self, vehicle: Vehicle, duration: Duration) -> NVector {
        Sphere::EARTH.destination_pos(
            vehicle.position(),
            vehicle.bearing(),
            vehicle.speed() * duration,
        )
    }

    ///  Computes the time to the closest point of approach (CPA) between the two given vehicles: the time at which the
    /// 2 vehicles will be the closest assuming they both maintain a constant course and heading.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, Length, NVector, Speed, Vehicle};
    /// use jord::spherical::Sphere;
    ///
    /// let ownship = Vehicle::new(
    ///     NVector::from_lat_long_degrees(20.0, -60.0),
    ///     Angle::from_degrees(10.0),
    ///     Speed::from_knots(15.0),
    /// );
    ///
    /// let intruder = Vehicle::new(
    ///     NVector::from_lat_long_degrees(34.0, -50.0),
    ///     Angle::from_degrees(220.0),
    ///     Speed::from_knots(300.0),
    /// );
    ///
    /// let opt_time_at_cpa = Sphere::EARTH.time_to_cpa(ownship, intruder);
    /// assert!(opt_time_at_cpa.is_some());
    /// let time_at_cpa = opt_time_at_cpa.unwrap();
    ///
    /// assert_eq!(113_961_40, time_at_cpa.as_millis());
    ///
    /// // Position of ownship at CPA:
    /// let p_cpa_own = Sphere::EARTH.position_after(ownship, time_at_cpa);
    ///
    /// // Position of intruder at CPA:
    /// let p_cpa_int = Sphere::EARTH.position_after(intruder, time_at_cpa);
    ///
    /// // Distance between the 2 vehicles at CPA:
    /// let d_cpa = Sphere::EARTH.distance(p_cpa_own, p_cpa_int);
    /// assert_eq!(Length::from_metres(124_232.0), d_cpa.round_m());
    ///
    /// ```
    pub fn time_to_cpa(&self, ownship: Vehicle, intruder: Vehicle) -> Option<Duration> {
        let r_nm = self.radius.as_nautical_miles();

        let own_p0 = ownship.position().as_vec3();
        let own_course = course(ownship);
        let own_speed_knots = ownship.speed().as_knots();
        let own_w = own_speed_knots / r_nm;

        let int_p0 = intruder.position().as_vec3();
        let int_course = course(intruder);
        let int_speed_knots = intruder.speed().as_knots();
        let int_w = int_speed_knots / r_nm;

        let f = cpa_fn(own_p0, own_course, own_w, int_p0, int_course, int_w, false);
        let df = cpa_fn(own_p0, own_course, own_w, int_p0, int_course, int_w, true);

        let hours_to_cpa = newton_raphson(
            f,
            df,
            0.0,
            Self::ONE_MILLI_HOURS,
            Self::CPA_NR_MAX_ITERATIONS,
        );
        hours_to_cpa.filter(|h| h >= &0.0).map(hours_to_duration)
    }

    /// Calculates the maximum time required by an interceptor at the given position to intercept the given intruder: i.e. the interceptor is
    /// travelling at the minimum speed required to achieve intercept.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, Length, NVector, Speed, Vehicle};
    /// use jord::spherical::Sphere;
    ///
    /// let interceptor_pos = NVector::from_lat_long_degrees(20.0, -60.0);
    /// let intruder = Vehicle::new(
    ///     NVector::from_lat_long_degrees(34.0, -50.0),
    ///     Angle::from_degrees(220.0),
    ///     Speed::from_knots(600.0)
    /// );
    ///
    /// let opt_max_time = Sphere::EARTH.max_time_to_intercept(interceptor_pos, intruder);
    /// assert!(opt_max_time.is_some());
    ///
    /// let max_time = opt_max_time.unwrap();
    /// assert_eq!(5_993_823, max_time.as_millis());
    ///
    /// // position of the interception = position of intruder at time of interception:
    /// let interception_pos = Sphere::EARTH.position_after(intruder, max_time);
    ///
    /// // distance to interception:
    /// let interception_distance = Sphere::EARTH.distance(interceptor_pos, interception_pos);
    ///
    /// // minimum interceptor speed to achieve intercept:
    /// let minimum_speed = interception_distance / max_time;
    /// assert_eq!(53.0, minimum_speed.as_knots().round());
    /// ```
    pub fn max_time_to_intercept(
        &self,
        interceptor_pos: NVector,
        intruder: Vehicle,
    ) -> Option<Duration> {
        let r_m: f64 = self.radius.as_metres();

        let v10 = interceptor_pos.as_vec3();

        let v20 = intruder.position().as_vec3();
        let c2 = course(intruder);
        let int_speed_mps = intruder.speed().as_metres_per_second();
        let w2 = int_speed_mps / r_m;

        let v10v20 = v10.dot_prod(v20);
        let v10c2 = v10.dot_prod(c2);
        // initial angular distance between target and interceptor.
        let s0 = angle_radians_between(v10, v20, None);
        // assume target is travelling towards interceptor.
        let t0 = r_m * s0 / int_speed_mps;

        let st: Box<dyn Fn(f64) -> f64> = sep(v10, v20, c2, int_speed_mps, r_m);

        let t_intercept_secs = int_min_nr_rec(v10v20, v10c2, w2, st, t0, 0);

        if t_intercept_secs < 0.0 {
            None
        } else {
            Some(Duration::from_secs_f64(t_intercept_secs))
        }
    }

    /// Calculates time required by an interceptor at the given position and travelling at the given speed to intercept the given intruder.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, Length, NVector, Speed, Vehicle};
    /// use jord::spherical::Sphere;
    ///
    /// let interceptor_pos = NVector::from_lat_long_degrees(20.0, -60.0);
    /// let intruder = Vehicle::new(
    ///     NVector::from_lat_long_degrees(34.0, -50.0),
    ///     Angle::from_degrees(220.0),
    ///     Speed::from_knots(600.0)
    /// );
    ///
    /// // minimum interceptor speed to achieve intercept is ~ 53 knots
    /// assert!(Sphere::EARTH.time_to_intercept(interceptor_pos, Speed::from_knots(50.0), intruder).is_none());
    ///
    /// let opt_time = Sphere::EARTH.time_to_intercept(interceptor_pos, Speed::from_knots(700.0), intruder);
    /// assert!(opt_time.is_some());
    /// assert_eq!(2_764_688, opt_time.unwrap().as_millis());
    /// ```
    pub fn time_to_intercept(
        &self,
        interceptor_pos: NVector,
        interceptor_speed: Speed,
        intruder: Vehicle,
    ) -> Option<Duration> {
        let r_m: f64 = self.radius.as_metres();

        let v10 = interceptor_pos.as_vec3();
        let intercept_speed_mps = interceptor_speed.as_metres_per_second();
        let w1 = intercept_speed_mps / r_m;

        let v20 = intruder.position().as_vec3();
        let c2 = course(intruder);
        let int_speed_mps = intruder.speed().as_metres_per_second();
        let w2 = int_speed_mps / r_m;

        let v10v20 = v10.dot_prod(v20);
        let v10c2 = v10.dot_prod(c2);

        let t0 = 0.1;

        let st: Box<dyn Fn(f64) -> f64> = sep(v10, v20, c2, int_speed_mps, r_m);

        let t_intercept_secs = int_spd_nr_rec(v10v20, v10c2, w1, w2, st, t0, 0);

        if t_intercept_secs < 0.0 {
            None
        } else {
            Some(Duration::from_secs_f64(t_intercept_secs))
        }
    }
}

impl Surface for Sphere {
    fn geodetic_to_geocentric(&self, pos: GeodeticPos) -> GeocentricPos {
        let h = self.radius + pos.height();
        GeocentricPos::from_vec3_metres(h.as_metres() * pos.horizontal_position().as_vec3())
    }

    fn geocentric_to_geodetic(&self, pos: GeocentricPos) -> GeodeticPos {
        let h = Length::from_metres(pos.as_metres().norm()) - self.radius;
        GeodeticPos::new(NVector::new(Vec3::unit(pos.as_metres())), h)
    }
}

// nanoseconds in one hour.
const NANOS_PER_HOUR: f64 = 3_600.0 * 1_000.0 * 1_000_000.0;

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

/// Implementation of the Newton Raphson root-finding algorithm.
/// See: https://en.wikipedia.org/wiki/Newton%27s_method
fn newton_raphson<F>(f: F, df: F, x0: f64, epsilon: f64, max_iters: u64) -> Option<f64>
where
    F: Fn(f64) -> f64,
{
    let mut x = x0;
    for _i in 0..max_iters {
        let y = f(x);
        let d = df(x);
        if d == 0.0 {
            return None;
        }

        let m = y / d;
        if m.abs() < epsilon {
            return Some(x);
        }
        x -= m;
    }
    None
}

fn course(vehicle: Vehicle) -> Vec3 {
    let ll = LatLong::from_nvector(vehicle.position());
    let lat_rads = ll.latitude().as_radians();
    let lng_rads = ll.longitude().as_radians();
    let bearing_rads = vehicle.bearing().as_radians();
    let r = (course_rz(-lng_rads) * course_ry(lat_rads)) * course_rx(bearing_rads);
    Vec3::UNIT_Z * r
}

fn course_rx(theta: f64) -> Mat33 {
    let c = theta.cos();
    let s = theta.sin();
    Mat33::new(Vec3::UNIT_X, Vec3::new(0.0, c, s), Vec3::new(0.0, -s, c))
}

fn course_ry(theta: f64) -> Mat33 {
    let c = theta.cos();
    let s = theta.sin();
    Mat33::new(Vec3::new(c, 0.0, -s), Vec3::UNIT_Y, Vec3::new(s, 0.0, c))
}

fn course_rz(theta: f64) -> Mat33 {
    let c = theta.cos();
    let s = theta.sin();
    Mat33::new(Vec3::new(c, s, 0.0), Vec3::new(-s, c, 0.0), Vec3::UNIT_Z)
}

/// Functions for iteratively solving for CPA.
fn cpa_fn(
    p10: Vec3,
    c10: Vec3,
    w1: f64,
    p20: Vec3,
    c20: Vec3,
    w2: f64,
    derivate: bool,
) -> Box<dyn Fn(f64) -> f64> {
    let a = -(w1 * p10.dot_prod(c20) + w2 * p20.dot_prod(c10));
    let b = w1 * c10.dot_prod(p20) + w2 * c20.dot_prod(p10);
    let c = -(w1 * p10.dot_prod(p20) - w2 * c20.dot_prod(c10));
    let d = w1 * c10.dot_prod(c20) - w2 * p20.dot_prod(p10);

    if derivate {
        let func = move |t: f64| -> f64 {
            let w1t: f64 = w1 * t;
            let w2t = w2 * t;
            let sw1t = w1t.sin();
            let sw2t = w2t.sin();
            let cw1t = w1t.cos();
            let cw2t = w2t.cos();
            -(c * w2 + d * w1) * sw1t * sw2t
                + (d * w2 + c * w1) * cw1t * cw2t
                + (a * w2 - b * w1) * sw1t * cw2t
                - (b * w2 - a * w1) * cw1t * sw2t
        };
        Box::new(func)
    } else {
        let func = move |t: f64| -> f64 {
            let w1t = w1 * t;
            let w2t = w2 * t;
            let sw1t = w1t.sin();
            let sw2t = w2t.sin();
            let cw1t = w1t.cos();
            let cw2t = w2t.cos();
            a * sw1t * sw2t + b * cw1t * cw2t + c * sw1t * cw2t + d * cw1t * sw2t
        };
        Box::new(func)
    }
}

/// Converts the given amount of decimal hours into a [Duration].
fn hours_to_duration(hours: f64) -> Duration {
    let nanos: u64 = (hours * NANOS_PER_HOUR).round() as u64;
    Duration::from_nanos(nanos)
}

/// angular separation in radians at ti (input to returned function) between v10 and track with initial position v20, course c2 and speed s2.
fn sep(v10: Vec3, v20: Vec3, c2: Vec3, s2_mps: f64, radius_m: f64) -> Box<dyn Fn(f64) -> f64> {
    let func = move |t_secs: f64| -> f64 {
        angle_radians_between(v10, pos(v20, c2, s2_mps, t_secs, radius_m), None)
    };
    Box::new(func)
}

/// position from course, speed (mps) and seconds.
fn pos(v0: Vec3, c: Vec3, mps: f64, t_secs: f64, radius_m: f64) -> Vec3 {
    let a = mps / radius_m * t_secs;
    v0 * a.cos() + c * a.sin()
}

const INTERCEPT_NR_MAX_ITERATIONS: u64 = 50;
const INTERCEPT_NR_EPSILON: f64 = 1e-11;

/// Newton-Raphson for min speed intercept.
fn int_min_nr_rec<F>(v10v20: f64, v10c2: f64, w2: f64, sep: F, ti_secs: f64, i: u64) -> f64
where
    F: Fn(f64) -> f64,
{
    if i == INTERCEPT_NR_MAX_ITERATIONS {
        -1.0 // no convergence
    } else {
        let cosw2t = (w2 * ti_secs).cos();
        let sinw2t = (w2 * ti_secs).sin();
        let v10dv2dt = -w2 * (v10v20 * sinw2t - v10c2 * cosw2t);
        let v10d2v2dt2 = (-1.0 * w2 * w2) * (v10v20 * cosw2t + v10c2 * sinw2t);
        let si = sep(ti_secs);
        // if separation = 0, intercept takes place at ti_secs.
        if si == 0.0 {
            return ti_secs;
        }
        let sin_si = si.sin();
        let a = -1.0 / sin_si;
        let b = si.cos() / (sin_si * sin_si);
        let f = ti_secs * a * v10dv2dt - si;
        let d2sdt2 = a * (b * v10dv2dt * v10dv2dt + v10d2v2dt2);
        let df = ti_secs * d2sdt2;
        let fi = f / df;
        let ti1_secs = ti_secs - fi;

        if fi.abs() < INTERCEPT_NR_EPSILON {
            ti1_secs
        } else {
            int_min_nr_rec(v10v20, v10c2, w2, sep, ti1_secs, i + 1)
        }
    }
}

///  Newton-Raphson for speed intercept.
fn int_spd_nr_rec<F>(v10v20: f64, v10c2: f64, w1: f64, w2: f64, sep: F, ti_secs: f64, i: u64) -> f64
where
    F: Fn(f64) -> f64,
{
    if i == INTERCEPT_NR_MAX_ITERATIONS {
        -1.0 // no convergence
    } else {
        let cosw2t = (w2 * ti_secs).cos();
        let sinw2t = (w2 * ti_secs).sin();
        let si = sep(ti_secs);
        let f = si / ti_secs - w1;
        let dsdt = (w2 * (v10v20 * sinw2t - v10c2 * cosw2t)) / si.sin();
        let df = (dsdt - (si / ti_secs)) / ti_secs;
        let fi = f / df;
        let ti1_secs = ti_secs - fi;
        if fi.abs() < INTERCEPT_NR_EPSILON {
            ti1_secs
        } else {
            int_spd_nr_rec(v10v20, v10c2, w1, w2, sep, ti1_secs, i + 1)
        }
    }
}

#[cfg(test)]
mod tests {

    use std::{f64::consts::PI, time::Duration};

    use crate::{
        positions::{assert_nv_eq_d7, assert_opt_nv_eq_d7},
        spherical::{GreatCircle, MinorArc, Sphere},
        Angle, LatLong, Length, NVector, Speed, Vec3, Vehicle,
    };

    use super::newton_raphson;

    // along_track_distance
    #[test]
    fn along_track_distance_positive() {
        let p = NVector::from_lat_long_degrees(53.2611, -0.7972);
        let ma = MinorArc::new(
            NVector::from_lat_long_degrees(53.3206, -1.7297),
            NVector::from_lat_long_degrees(53.1887, 0.1334),
        );
        let a = Sphere::EARTH.along_track_distance(p, ma);
        assert_eq!(Length::from_metres(62331.501), a.round_mm());
    }

    #[test]
    fn along_track_distance_negative() {
        let p = NVector::from_lat_long_degrees(53.3206, -1.7297);
        let ma = MinorArc::new(
            NVector::from_lat_long_degrees(53.2611, -0.7972),
            NVector::from_lat_long_degrees(53.1887, 0.1334),
        );
        let a = Sphere::EARTH.along_track_distance(p, ma);
        assert_eq!(Length::from_metres(-62329.232), a.round_mm());
    }

    #[test]
    fn along_track_distance_zero() {
        let p = NVector::from_lat_long_degrees(53.3206, -1.7297);
        let ma = MinorArc::new(p, NVector::from_lat_long_degrees(53.1887, 0.1334));
        let a = Sphere::EARTH.along_track_distance(p, ma);
        assert_eq!(Length::ZERO, a.round_mm());
    }

    // cross_track_distance

    #[test]
    fn cross_track_distance_negative() {
        let p = NVector::from_lat_long_degrees(53.2611, -0.7972);
        let gc = GreatCircle::from_heading(
            NVector::from_lat_long_degrees(53.3206, -1.7297),
            Angle::from_degrees(96.0),
        );
        let a = Sphere::EARTH.cross_track_distance(p, gc);
        assert_eq!(Length::from_metres(-305.665), a.round_mm());
    }

    #[test]
    fn cross_track_distance_positive() {
        let p = NVector::from_lat_long_degrees(53.2611, -1.7972);
        let gc = GreatCircle::from_heading(
            NVector::from_lat_long_degrees(53.3206, -1.7297),
            Angle::from_degrees(96.0),
        );
        let a = Sphere::EARTH.cross_track_distance(p, gc);
        assert_eq!(Length::from_metres(7047.043), a.round_mm());
    }

    // destination.

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
        assert_eq!(Angle::ZERO, Sphere::initial_bearing(np, sp));
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

    // newton_raphson
    #[test]
    fn newton_raphson_line() {
        let f: &dyn Fn(f64) -> f64 = &|x| 3.0 * x + 1.0;
        let df: &dyn Fn(f64) -> f64 = &|_| 3.0;
        let x0 = 0.0;
        let r = newton_raphson(f, df, x0, 1e-15, 100);
        assert_eq!(Some(-1.0 / 3.0), r);
    }

    #[test]
    fn newton_raphson_parabola() {
        let f: &dyn Fn(f64) -> f64 = &|x| x * x - 1.0;
        let df: &dyn Fn(f64) -> f64 = &|x| 2.0 * x;

        let mut x0 = 0.0;
        let mut r: Option<f64> = newton_raphson(f, df, x0, 1e-15, 100);
        assert!(r.is_none());

        x0 = 0.1;
        r = newton_raphson(f, df, x0, 1e-15, 100);
        assert_eq!(Some(1.0), r);

        x0 = -0.1;
        r = newton_raphson(f, df, x0, 1e-15, 100);
        assert_eq!(Some(-1.0), r);
    }

    #[test]
    fn newton_raphson_sinusoid() {
        let f: &dyn Fn(f64) -> f64 = &|x| x.sin();
        let df: &dyn Fn(f64) -> f64 = &|x| x.cos();

        let mut x0 = 0.0;

        // Initial value is root.
        let mut r: Option<f64> = newton_raphson(f, df, x0, 1e-15, 100);
        assert_eq!(Some(0.0), r);

        // No root, derivative is 0.
        x0 = PI / 2.0;
        r = newton_raphson(f, df, x0, 1e-15, 100);
        assert!(r.is_none());

        // Second root.
        x0 = PI * 3.0 / 4.0;
        r = newton_raphson(f, df, x0, 1e-15, 100);
        assert_eq!(Some(PI), r);
    }

    // time to CPA

    #[test]
    fn time_to_cpa_catch_up() {
        // faster vehicle chases other vehicle on the same great circle.
        let ownship = Vehicle::new(
            NVector::from_lat_long_degrees(-24.0, 129.0),
            Angle::from_degrees(45.0),
            Speed::from_knots(405.0),
        );

        let intruder = Vehicle::new(
            NVector::from_lat_long_degrees(-23.40981138888889, 129.64166694444444),
            Angle::from_degrees(44.742017777777775),
            Speed::from_knots(400.0),
        );

        assert_time_to_cpa(
            Duration::from_secs(10 * 60 * 60),
            Sphere::EARTH.time_to_cpa(ownship, intruder),
        );
    }

    #[test]
    fn time_to_cpa_same_fleeing_along_equator() {
        let ownship = Vehicle::new(
            NVector::from_lat_long_degrees(0.0, 0.1),
            Angle::from_degrees(90.0),
            Speed::from_knots(400.0),
        );

        let intruder = Vehicle::new(
            NVector::from_lat_long_degrees(0.0, -0.1),
            Angle::from_degrees(270.0),
            Speed::from_knots(400.0),
        );

        assert!(Sphere::EARTH.time_to_cpa(ownship, intruder).is_none());
    }

    #[test]
    fn time_to_cpa_same_head_on_along_equator() {
        let ownship = Vehicle::new(
            NVector::from_lat_long_degrees(0.0, -0.1),
            Angle::from_degrees(90.0),
            Speed::from_knots(400.0),
        );

        let intruder = Vehicle::new(
            NVector::from_lat_long_degrees(0.0, 0.1),
            Angle::from_degrees(270.0),
            Speed::from_knots(400.0),
        );

        assert_time_to_cpa(
            elapsed_at_knots(
                800.0,
                (Angle::from_degrees(0.2) * Sphere::EARTH.radius()).as_nautical_miles(),
            ),
            Sphere::EARTH.time_to_cpa(ownship, intruder),
        );
    }

    #[test]
    fn time_to_cpa_head_on_over_north_pole() {
        let ownship = Vehicle::new(
            NVector::from_lat_long_degrees(89.0, 0.0),
            Angle::ZERO,
            Speed::from_knots(400.0),
        );

        let intruder = Vehicle::new(
            NVector::from_lat_long_degrees(89.0, 180.0),
            Angle::ZERO,
            Speed::from_knots(400.0),
        );

        assert_time_to_cpa(
            elapsed_at_knots(
                800.0,
                (Angle::from_degrees(2.0) * Sphere::EARTH.radius()).as_nautical_miles(),
            ),
            Sphere::EARTH.time_to_cpa(ownship, intruder),
        );
    }

    #[test]
    fn time_to_cpa_test() {
        let ownship = Vehicle::new(
            NVector::from_lat_long_degrees(20.0, -60.0),
            Angle::from_degrees(10.0),
            Speed::from_knots(15.0),
        );

        let intruder = Vehicle::new(
            NVector::from_lat_long_degrees(34.0, -50.0),
            Angle::from_degrees(220.0),
            Speed::from_knots(300.0),
        );

        assert_time_to_cpa(
            Duration::from_millis(113_961_40),
            Sphere::EARTH.time_to_cpa(ownship, intruder),
        );
    }

    #[test]
    fn time_to_cpa_trailing_ships() {
        let ownship = Vehicle::new(
            NVector::from_lat_long_degrees(20.0, 30.0),
            Angle::from_degrees(20.000959722222223),
            Speed::from_knots(400.0),
        );

        let intruder = Vehicle::new(
            NVector::from_lat_long_degrees(20.00211277777778, 30.000818333333335),
            Angle::from_degrees(20.00169861111111),
            Speed::from_knots(400.0),
        );

        assert_time_to_cpa(
            Duration::from_millis(4_177),
            Sphere::EARTH.time_to_cpa(ownship, intruder),
        );
    }

    #[test]
    fn time_to_cpa_trailing_ships_same_speed() {
        let ownship = Vehicle::new(
            NVector::from_lat_long_degrees(20.0, 30.0),
            Angle::from_degrees(20.0000000844),
            Speed::from_knots(400.0),
        );

        let intruder = Vehicle::new(
            NVector::from_lat_long_degrees(20.0021127100, 30.0008183256),
            Angle::from_degrees(20.0002805853),
            Speed::from_knots(400.0),
        );

        assert!(Sphere::EARTH.time_to_cpa(ownship, intruder).is_none());
    }

    #[test]
    fn time_to_cpa_trailing_ships_same_speed_equator() {
        let ownship = Vehicle::new(
            NVector::from_lat_long_degrees(0.0, 1.0),
            Angle::from_degrees(90.0),
            Speed::from_knots(400.0),
        );

        let intruder = Vehicle::new(
            NVector::from_lat_long_degrees(0.0, 1.0000001),
            Angle::from_degrees(90.0),
            Speed::from_knots(400.0),
        );

        assert!(Sphere::EARTH.time_to_cpa(ownship, intruder).is_none());
    }

    fn assert_time_to_cpa(expected: Duration, actual: Option<Duration>) {
        assert!(actual.is_some());
        let a_ms = actual.unwrap().as_millis() as i128;
        let e_ms = expected.as_millis() as i128;
        let diff = (a_ms - e_ms).abs();
        assert!(
            diff < 100,
            "expected {:?}ms but was {:?}ms - diff = {:?}ms",
            expected.as_millis(),
            actual.unwrap().as_millis(),
            diff
        );
    }

    /// Time taken to cover a distance at a speed.
    fn elapsed_at_knots(knots: f64, nm: f64) -> Duration {
        let millis: u64 = (nm / knots * 60.0 * 60.0 * 1000.0).round() as u64;
        Duration::from_millis(millis)
    }

    // max_time_to_intercept

    #[test]
    fn max_time_to_intercept() {
        let interceptor_pos = NVector::from_lat_long_degrees(20.0, -60.0);
        let intruder = Vehicle::new(
            NVector::from_lat_long_degrees(34.0, -50.0),
            Angle::from_degrees(220.0),
            Speed::from_knots(600.0),
        );

        let opt_max_time: Option<Duration> =
            Sphere::EARTH.max_time_to_intercept(interceptor_pos, intruder);
        assert!(opt_max_time.is_some());

        let max_time = opt_max_time.unwrap();
        assert_eq!(5_993_823, max_time.as_millis());
    }

    #[test]
    fn max_time_to_intercept_same_pos() {
        let interceptor_pos = NVector::from_lat_long_degrees(20.0, -60.0);
        let intruder = Vehicle::new(
            interceptor_pos,
            Angle::from_degrees(220.0),
            Speed::from_knots(600.0),
        );

        let opt_max_time: Option<Duration> =
            Sphere::EARTH.max_time_to_intercept(interceptor_pos, intruder);
        assert!(opt_max_time.is_some());

        let max_time = opt_max_time.unwrap();
        assert_eq!(0, max_time.as_nanos());
    }

    #[test]
    fn max_time_to_intercept_interceptor_behind() {
        let interceptor_pos = NVector::from_lat_long_degrees(44.0, 66.0);
        let intruder = Vehicle::new(
            NVector::from_lat_long_degrees(45.0, 67.0),
            Angle::from_degrees(54.0),
            Speed::from_knots(400.0),
        );
        assert!(Sphere::EARTH
            .max_time_to_intercept(interceptor_pos, intruder)
            .is_none());
    }

    #[test]
    fn max_time_to_intercept_interceptor_in_front() {
        let intruder = Vehicle::new(
            NVector::from_lat_long_degrees(20.0, 30.0),
            Angle::from_degrees(12.0),
            Speed::from_knots(400.0),
        );
        let interceptor_pos = Sphere::EARTH.position_after(intruder, Duration::from_secs(60));

        let opt_max_time: Option<Duration> =
            Sphere::EARTH.max_time_to_intercept(interceptor_pos, intruder);
        assert!(opt_max_time.is_some());

        let max_time = opt_max_time.unwrap();
        assert_eq!(60, max_time.as_secs());
    }

    #[test]
    fn time_to_intercept() {
        let interceptor_pos = NVector::from_lat_long_degrees(20.0, -60.0);
        let intruder = Vehicle::new(
            NVector::from_lat_long_degrees(34.0, -50.0),
            Angle::from_degrees(220.0),
            Speed::from_knots(600.0),
        );

        // minimum interceptor speed to achieve intercept is ~ 53 knots
        assert!(Sphere::EARTH
            .time_to_intercept(interceptor_pos, Speed::from_knots(50.0), intruder)
            .is_none());

        let opt_time =
            Sphere::EARTH.time_to_intercept(interceptor_pos, Speed::from_knots(700.0), intruder);
        assert!(opt_time.is_some());
        assert_eq!(2_764_688, opt_time.unwrap().as_millis());
    }
}
