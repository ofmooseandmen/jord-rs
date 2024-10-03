use crate::{
    surface::Surface, Angle, Cartesian3DVector, GeocentricPosition, GeodeticPosition, LatLong,
    Length, Mat33, Vec3,
};

#[derive(PartialEq, Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
enum Orientation {
    // x = north (or forward), y = east (or right), z = down.
    #[default]
    Ned,
    // x = east, y = north, z = up.
    Enu,
}

/// A vector whose length and direction is such that it goes from the origin
/// of frame A to the origin of frame B, i.e. the position of B relative to A.
///
/// The orientation of the x, y and z axis depends on the [local Cartesian coordinate frame](crate::LocalFrame):
/// - x = north (or forward), y = east (or right), z = down: [NED](crate::LocalFrame::ned), [Body](crate::LocalFrame::body) and [Local Level](crate::LocalFrame::local_level),
/// - x = east, y = north, z = up: [ENU](crate::LocalFrame::enu).
///
/// However, the [azimuth](crate::LocalPosition::azimuth) is always relative to 'north' and the elevation is always positive if above the local
/// tangent plane and negative if below.
#[derive(PartialEq, Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LocalPosition {
    x: Length,
    y: Length,
    z: Length,
    o: Orientation,
}

impl LocalPosition {
    /// Creates a [LocalPosition] from the given coordinates.
    /// Orientation: x = north (or forward), y = east (or right), z = down
    pub const fn new(x: Length, y: Length, z: Length) -> Self {
        Self::new_with_o(x, y, z, Orientation::Ned)
    }

    /// Creates a [LocalPosition] from the given coordinates in metres.
    /// Orientation: x = north (or forward), y = east (or right), z = down
    pub fn from_metres(x: f64, y: f64, z: f64) -> Self {
        Self::new_with_o(
            Length::from_metres(x),
            Length::from_metres(y),
            Length::from_metres(z),
            Orientation::Ned,
        )
    }

    const fn new_with_o(x: Length, y: Length, z: Length, o: Orientation) -> Self {
        Self { x, y, z, o }
    }

    fn from_metres_with_o(v: Vec3, o: Orientation) -> Self {
        Self::new_with_o(
            Length::from_metres(v.x()),
            Length::from_metres(v.y()),
            Length::from_metres(v.z()),
            o,
        )
    }

    /// Converts this [LocalPosition] using the given orientation.
    fn with_orientation(&self, o: Orientation) -> Self {
        if self.o == o {
            *self
        } else {
            // ENU(x, y, z) = NED(y, x, -z)
            // NED(x, y, z) = ENU(y, x, -z)
            LocalPosition {
                x: self.y,
                y: self.x,
                z: -self.z,
                o,
            }
        }
    }

    /// Transforms the given local azimuth-elevation-range (AER) spherical coordinates to the
    /// local north-east-down (NED) Cartesian coordinates.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, Cartesian3DVector, Length, LocalPosition};
    ///
    /// let az: Angle = Angle::from_degrees(155.427);
    /// let el = Angle::from_degrees(-23.161); // elevation is negative, so resulting z (down) will be positive
    /// let sr = Length::from_metres(10.885);
    ///
    /// let local = LocalPosition::aer_to_ned(az, el, sr);
    /// assert_eq!(Length::from_metres(-9.101), local.x().round_mm());
    /// assert_eq!(Length::from_metres(4.162), local.y().round_mm());
    /// assert_eq!(Length::from_metres(4.281), local.z().round_mm());
    /// ```
    pub fn aer_to_ned(azimuth: Angle, elevation: Angle, slant_range: Length) -> Self {
        let (north, east, z) = Self::aer_to_enz(azimuth, elevation, slant_range);
        LocalPosition::new_with_o(north, east, -z, Orientation::Ned)
    }

    /// Transforms the given local azimuth-elevation-range (AER) spherical coordinates to the
    /// local east-north-up (ENU) Cartesian coordinates.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, Cartesian3DVector, Length, LocalPosition};
    ///
    /// let az: Angle = Angle::from_degrees(34.1160);
    /// let el = Angle::from_degrees(4.1931); // elevation is positive, so resulting z (down) will be positive
    /// let sr = Length::from_metres(15.1070);
    ///
    /// let local = LocalPosition::aer_to_enu(az, el, sr);
    /// assert_eq!(Length::from_metres(8.45), local.x().round_mm());
    /// assert_eq!(Length::from_metres(12.474), local.y().round_mm());
    /// assert_eq!(Length::from_metres(1.105), local.z().round_mm());
    /// ```
    pub fn aer_to_enu(azimuth: Angle, elevation: Angle, slant_range: Length) -> Self {
        let (north, east, z) = Self::aer_to_enz(azimuth, elevation, slant_range);
        LocalPosition::new_with_o(east, north, z, Orientation::Enu)
    }

    fn aer_to_enz(
        azimuth: Angle,
        elevation: Angle,
        slant_range: Length,
    ) -> (Length, Length, Length) {
        let cose = elevation.as_radians().cos();
        let east = azimuth.as_radians().sin() * cose * slant_range;
        let north = azimuth.as_radians().cos() * cose * slant_range;
        let z = elevation.as_radians().sin() * slant_range;
        (north, east, z)
    }

    /// Returns the azimuth in compass angle from the 'north'.
    pub fn azimuth(&self) -> Angle {
        let (e, n) = match self.o {
            Orientation::Ned => (self.y(), self.x()),
            Orientation::Enu => (self.x(), self.y()),
        };
        Angle::from_radians(e.as_metres().atan2(n.as_metres())).normalised()
    }

    /// Returns the elevation from horizontal (ie tangent to surface).
    pub fn elevation(&self) -> Angle {
        let ev = Angle::from_radians((self.z() / self.slant_range()).asin());
        match self.o {
            Orientation::Ned => -ev,
            Orientation::Enu => ev,
        }
    }

    /// Returns the slant range - distance from origin in the local system.
    pub fn slant_range(&self) -> Length {
        Length::from_metres(self.as_metres().norm())
    }
}

impl Cartesian3DVector for LocalPosition {
    #[inline]
    fn x(&self) -> Length {
        self.x
    }

    #[inline]
    fn y(&self) -> Length {
        self.y
    }

    #[inline]
    fn z(&self) -> Length {
        self.z
    }

    fn round<F>(&self, round: F) -> Self
    where
        F: Fn(Length) -> Length,
    {
        Self::new_with_o(round(self.x()), round(self.y()), round(self.z()), self.o)
    }
}

/// Defines a local Cartesian coordinate frame with two axes forming a horizontal
/// tangent plane to the reference surface ([ellipsoid](crate::ellipsoidal::Ellipsoid) or
/// [sphere](crate::spherical::Sphere)) at a specified tangent point. Assuming several
/// calculations are needed in a limited area, position calculations can be performed
/// relative to this system to get approximate horizontal and vertical components
#[derive(PartialEq, Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LocalFrame<S> {
    origin: Vec3,
    dir_rm: Mat33,
    inv_rm: Mat33,
    surface: S,
    o: Orientation,
}

impl<S> LocalFrame<S>
where
    S: Surface,
{
    /// East-North-Up (local level) frame. This frame is useful for many targeting and tracking applications.
    ///
    /// - Orientation: The x-axis points towards east, the y-axis points towards north (both are
    ///   horizontal), and the z-axis is pointing up.
    ///
    /// See also [NED](crate::LocalFrame::ned)
    pub fn enu(origin: GeodeticPosition, surface: S) -> Self {
        let vo = origin.horizontal_position().as_vec3();
        // up - just the n-vector.
        let ru = vo;
        // east - pointing perpendicular to the plane.
        let re = Vec3::UNIT_Z.orthogonal_to(vo);
        // north - by right hand rule.
        let rn = ru.cross_prod(re);

        let inv_rm = Mat33::new(re, rn, ru);

        Self {
            origin: surface.geodetic_to_geocentric_position(origin).as_metres(),
            dir_rm: inv_rm.transpose(),
            inv_rm,
            surface,
            o: Orientation::Enu,
        }
    }

    /// North-East-Down (local level) frame. In an airplane, most objects of interest are below the aircraft,
    /// so it is sensible to define down as a positive number.
    ///
    /// - The origin is directly beneath or above the vehicle (B), at Earth’s surface.
    /// - Orientation: The x-axis points towards north, the y-axis points towards east (both are
    ///   horizontal), and the z-axis is pointing down.
    ///
    /// Note: When moving relative to the Earth, the frame rotates about its z-axis to allow the
    /// x-axis to always point towards north. When getting close to the poles this rotation rate
    /// will increase, being infinite at the poles. The poles are thus singularities and the direction of
    /// the x- and y-axes are not defined here. Hence, this coordinate frame is not suitable for
    /// general calculations.
    ///
    /// See also: [ENU](crate::LocalFrame::enu)
    pub fn ned(origin: GeodeticPosition, surface: S) -> Self {
        let vo = origin.horizontal_position().as_vec3();
        // down (pointing opposite to n-vector).
        let rd = -1.0 * vo;
        // east (pointing perpendicular to the plane)
        let re = Vec3::UNIT_Z.orthogonal_to(vo);
        // north (by right hand rule)
        let rn = re.cross_prod(rd);

        let inv_rm = Mat33::new(rn, re, rd);

        Self {
            origin: surface.geodetic_to_geocentric_position(origin).as_metres(),
            dir_rm: inv_rm.transpose(),
            inv_rm,
            surface,
            o: Orientation::Ned,
        }
    }

    /// Body frame (typically of a vehicle). This frame is fixed to the vehicle.
    ///
    /// -The origin is in the vehicle’s reference point.
    /// - Orientation: The x-axis points forward, the y-axis to the right (starboard) and the z-axis in the vehicle’s down direction.
    pub fn body(
        yaw: Angle,
        pitch: Angle,
        roll: Angle,
        origin: GeodeticPosition,
        surface: S,
    ) -> Self {
        let r_nb = zyx2r(yaw, pitch, roll);
        let r_en = Self::ned(origin, surface).dir_rm;
        // closest frames cancel: N.
        let dir_rm = r_en * r_nb;
        Self {
            origin: surface.geodetic_to_geocentric_position(origin).as_metres(),
            dir_rm,
            inv_rm: dir_rm.transpose(),
            surface,
            o: Orientation::Ned,
        }
    }

    /// Local level, Wander azimuth frame.
    ///
    /// - The origin is directly beneath or above the vehicle (B), at Earth’s surface.
    /// - Orientation: The z-axis is pointing down. Initially, the x-axis points towards north, and the
    ///   y-axis points towards east, but as the vehicle moves they are not rotating about the z-axis
    ///   (their angular velocity relative to the Earth has zero component along the z-axis).
    ///   (Note: Any initial horizontal direction of the x- and y-axes is valid for L, but if the
    ///   initial position is outside the poles, north and east are usually chosen for convenience.)
    ///
    /// Notes: The L-frame is equal to the N-frame except for the rotation about the z-axis,
    /// which is always zero for this frame (relative to Earth). Hence, at a given time, the only
    /// difference between the frames is an angle between the x-axis of L and the north direction;
    /// this angle is called the wander azimuth angle. The L-frame is well suited for general
    /// calculations, as it is non-singular.
    pub fn local_level(wander_azimuth: Angle, origin: GeodeticPosition, surface: S) -> Self {
        let ll = LatLong::from_nvector(origin.horizontal_position());
        let r = xyz2r(ll.longitude(), -ll.latitude(), wander_azimuth);
        let r_ee = Mat33::new(Vec3::NEG_UNIT_Z, Vec3::UNIT_Y, Vec3::UNIT_X);
        let dir_rm = r_ee * r;
        Self {
            origin: surface.geodetic_to_geocentric_position(origin).as_metres(),
            dir_rm,
            inv_rm: dir_rm.transpose(),
            surface,
            o: Orientation::Ned,
        }
    }

    /// Converts the given [GeodeticPosition] into a [LocalPosition]: the exact vector between this frame
    /// origin and the given position. The resulting [LocalPosition] orientation is the one of this frame.
    pub fn geodetic_to_local_position(&self, p: GeodeticPosition) -> LocalPosition {
        let p_geocentric = self.surface.geodetic_to_geocentric_position(p).as_metres();
        // delta in 'Earth' frame.
        let de = p_geocentric - self.origin;
        let d = de * self.inv_rm;
        LocalPosition::from_metres_with_o(d, self.o)
    }

    /// Converts the given [LocalPosition] into a [GeodeticPosition]: the geodetic position of an object
    /// which is located at a bearing and distance from this frame origin. The given [LocalPosition]
    /// is re-oriented to match the orientation of this frame if required.
    pub fn local_to_geodetic_position(&self, p: LocalPosition) -> GeodeticPosition {
        let op = p.with_orientation(self.o);
        let c = op.as_metres() * self.dir_rm;
        let v = self.origin + c;
        let p_geocentric = GeocentricPosition::from_vec3_metres(v);
        self.surface.geocentric_to_geodetic_position(p_geocentric)
    }
}

/// Angles about new axes in the xyz-order from a rotation matrix.
///
/// The produced list contains 3 angles of rotation about new axes.
///
/// The x, y, z angles are called Euler angles or Tait-Bryan angles and are
/// defined by the following procedure of successive rotations:
/// Given two arbitrary coordinate frames A and B. Consider a temporary frame
/// T that initially coincides with A. In order to make T align with B, we
/// first rotate T an angle x about its x-axis (common axis for both A and T).
/// Secondly, T is rotated an angle y about the NEW y-axis of T. Finally, T
/// is rotated an angle z about its NEWEST z-axis. The final orientation of
/// T now coincides with the orientation of B.
/// The signs of the angles are given by the directions of the axes and the
/// right hand rule.
pub fn r2xyz(m: Mat33) -> (Angle, Angle, Angle) {
    let r0 = m.row0();
    let r1 = m.row1();
    let r2 = m.row2();
    let v00 = r0.x();
    let v01 = r0.y();
    let v12 = r1.z();
    let v22 = r2.z();
    let z = -v01.atan2(v00);
    let x = -v12.atan2(v22);
    let sy = r0.z();
    // cos y is based on as many elements as possible, to average out
    // numerical errors. It is selected as the positive square root since
    // y: [-pi/2 pi/2]
    let cy = ((v00 * v00 + v01 * v01 + v12 * v12 + v22 * v22) / 2.0).sqrt();
    let y = sy.atan2(cy);
    (
        Angle::from_radians(x),
        Angle::from_radians(y),
        Angle::from_radians(z),
    )
}

/// Angles about new axes in the xyz-order from a rotation matrix.
///
/// The produced list contains 3 angles of rotation about new axes.
/// The z, x, y angles are called Euler angles or Tait-Bryan angles and are
/// defined by the following procedure of successive rotations:
/// Given two arbitrary coordinate frames A and B. Consider a temporary frame
/// T that initially coincides with A. In order to make T align with B, we
/// first rotate T an angle z about its z-axis (common axis for both A and T).
/// Secondly, T is rotated an angle y about the NEW y-axis of T. Finally, T
/// is rotated an angle x about its NEWEST x-axis. The final orientation of
/// T now coincides with the orientation of B.
/// The signs of the angles are given by the directions of the axes and the
/// right hand rule.
/// Note that if A is a north-east-down frame and B is a body frame, we
/// have that z=yaw, y=pitch and x=roll.
pub fn r2zyx(m: Mat33) -> (Angle, Angle, Angle) {
    let (x, y, z) = r2xyz(m.transpose());
    (-z, -y, -x)
}

/// Rotation matrix (direction cosine matrix) from 3 angles about new axes in the zyx-order.
///
/// The produced (no unit) rotation matrix is such
/// that the relation between a vector v decomposed in A and B is given by:
/// @v_A = mdot R_AB v_B@
///
/// The rotation matrix R_AB is created based on 3 angles
/// z,y,x about new axes (intrinsic) in the order z-y-x. The angles are called
/// Euler angles or Tait-Bryan angles and are defined by the following
/// procedure of successive rotations:
/// Given two arbitrary coordinate frames A and B. Consider a temporary frame
/// T that initially coincides with A. In order to make T align with B, we
/// first rotate T an angle z about its z-axis (common axis for both A and T).
/// Secondly, T is rotated an angle y about the NEW y-axis of T. Finally, T
/// is rotated an angle x about its NEWEST x-axis. The final orientation of
/// T now coincides with the orientation of B.
/// The signs of the angles are given by the directions of the axes and the
/// right hand rule.
///
/// Note that if A is a north-east-down frame and B is a body frame, we
/// have that z=yaw, y=pitch and x=roll.
pub fn zyx2r(z: Angle, y: Angle, x: Angle) -> Mat33 {
    let cx = x.as_radians().cos();
    let sx = x.as_radians().sin();
    let cy = y.as_radians().cos();
    let sy = y.as_radians().sin();
    let cz = z.as_radians().cos();
    let sz = z.as_radians().sin();
    let r0 = Vec3::new(cz * cy, -sz * cx + cz * sy * sx, sz * sx + cz * sy * cx);
    let r1 = Vec3::new(sz * cy, cz * cx + sz * sy * sx, -cz * sx + sz * sy * cx);
    let r2 = Vec3::new(-sy, cy * sx, cy * cx);
    Mat33::new(r0, r1, r2)
}

/// Rotation matrix (direction cosine matrix) from 3 angles about new axes in the xyz-order.
///
/// The produced (no unit) rotation matrix is such
/// that the relation between a vector v decomposed in A and B is given by:
/// @v_A = mdot R_AB v_B@
///
/// The rotation matrix R_AB is created based on 3 angles x,y,z about new axes
/// (intrinsic) in the order x-y-z. The angles are called Euler angles or
/// Tait-Bryan angles and are defined by the following procedure of successive
/// rotations:
/// Given two arbitrary coordinate frames A and B. Consider a temporary frame
/// T that initially coincides with A. In order to make T align with B, we
/// first rotate T an angle x about its x-axis (common axis for both A and T).
/// Secondly, T is rotated an angle y about the NEW y-axis of T. Finally, T
/// is rotated an angle z about its NEWEST z-axis. The final orientation of
/// T now coincides with the orientation of B.
/// The signs of the angles are given by the directions of the axes and the
/// right hand rule.
pub fn xyz2r(x: Angle, y: Angle, z: Angle) -> Mat33 {
    let cx = x.as_radians().cos();
    let sx = x.as_radians().sin();
    let cy = y.as_radians().cos();
    let sy = y.as_radians().sin();
    let cz = z.as_radians().cos();
    let sz = z.as_radians().sin();
    let r0 = Vec3::new(cy * cz, -cy * sz, sy);
    let r1 = Vec3::new(sy * sx * cz + cx * sz, -sy * sx * sz + cx * cz, -cy * sx);
    let r2 = Vec3::new(-sy * cx * cz + sx * sz, sy * cx * sz + sx * cz, cy * cx);
    Mat33::new(r0, r1, r2)
}

#[cfg(test)]
mod tests {

    use crate::{
        ellipsoidal::Ellipsoid, positions::assert_geod_eq_d7_mm, r2xyz, r2zyx, Angle,
        Cartesian3DVector, GeodeticPosition, LatLong, Length, LocalFrame, LocalPosition, Mat33,
        NVector, Vec3,
    };

    // geodetic_to_local_pos

    // see https://github.com/pbrod/nvector/blob/bf1cf5e1e210b74a57ea4bb2c277b388308bdba9/src/nvector/tests/test_frames.py

    #[test]
    fn geodetic_to_local_pos_w_in_moving_frame_east() {
        let ship_position_0 =
            GeodeticPosition::new(LatLong::from_degrees(1.0, 2.0).to_nvector(), Length::ZERO);
        let ship_position_1 =
            GeodeticPosition::new(LatLong::from_degrees(1.0, 2.005).to_nvector(), Length::ZERO);
        let sensor_position = GeodeticPosition::new(
            LatLong::from_degrees(1.000090437, 2.0025).to_nvector(),
            Length::ZERO,
        );

        let f0: LocalFrame<Ellipsoid> =
            LocalFrame::local_level(Angle::from_degrees(90.0), ship_position_0, Ellipsoid::WGS84);
        let local_0 = f0.geodetic_to_local_position(sensor_position).round_mm();

        assert_eq!(Length::from_metres(278.257), local_0.x());
        assert_eq!(Length::from_metres(-10.0), local_0.y());
        assert_eq!(Length::ZERO, local_0.z().round_m());
        assert_eq!(358.0, local_0.azimuth().as_degrees().round());

        let f1: LocalFrame<Ellipsoid> =
            LocalFrame::local_level(Angle::from_degrees(90.0), ship_position_1, Ellipsoid::WGS84);

        let local_1 = f1.geodetic_to_local_position(sensor_position).round_mm();

        assert_eq!(Length::from_metres(-278.257), local_1.x());
        assert_eq!(Length::from_metres(-10.0), local_1.y());
        assert_eq!(Length::ZERO, local_1.z().round_m());
        assert_eq!(182.0, local_1.azimuth().as_degrees().round());
    }

    #[test]
    fn geodetic_to_local_pos_n_in_moving_frame_east() {
        let ship_position_0 =
            GeodeticPosition::new(LatLong::from_degrees(1.0, 2.0).to_nvector(), Length::ZERO);
        let ship_position_1 =
            GeodeticPosition::new(LatLong::from_degrees(1.0, 2.005).to_nvector(), Length::ZERO);
        let sensor_position = GeodeticPosition::new(
            LatLong::from_degrees(1.0, 2.0025).to_nvector(),
            Length::ZERO,
        );

        let f0: LocalFrame<Ellipsoid> = LocalFrame::ned(ship_position_0, Ellipsoid::WGS84);
        let local_0 = f0.geodetic_to_local_position(sensor_position).round_mm();

        assert_eq!(Length::ZERO, local_0.x());
        assert_eq!(Length::from_metres(278.257), local_0.y());
        assert_eq!(Length::ZERO, local_0.z().round_m());
        assert_eq!(90.0, local_0.azimuth().as_degrees());

        let f1: LocalFrame<Ellipsoid> = LocalFrame::ned(ship_position_1, Ellipsoid::WGS84);
        let local_1 = f1.geodetic_to_local_position(sensor_position).round_mm();

        assert_eq!(Length::ZERO, local_1.x());
        assert_eq!(Length::from_metres(-278.257), local_1.y());
        assert_eq!(Length::ZERO, local_1.z().round_m());
        assert_eq!(270.0, local_1.azimuth().as_degrees());
    }

    // see https://au.mathworks.com/help/map/ref/geodetic2ned.html
    #[test]
    fn geodetic_to_local_pos_ned() {
        let origin = GeodeticPosition::new(
            NVector::from_lat_long_degrees(44.532, -72.782),
            Length::from_metres(1699.0),
        );
        let point = GeodeticPosition::new(
            NVector::from_lat_long_degrees(44.544, -72.814),
            Length::from_metres(1340.0),
        );

        let ned = LocalFrame::ned(origin, Ellipsoid::WGS84);

        let local = ned.geodetic_to_local_position(point);

        assert_eq!(Length::from_metres(1334.252), local.x().round_mm());
        assert_eq!(Length::from_metres(-2543.564), local.y().round_mm());
        assert_eq!(Length::from_metres(359.646), local.z().round_mm());
        assert_eq!(Angle::from_degrees(297.6796990), local.azimuth().round_d7());
        assert_eq!(
            Angle::from_degrees(-7.1370359),
            local.elevation().round_d7()
        );
        assert_eq!(
            Length::from_metres(2894.701),
            local.slant_range().round_mm()
        );
    }

    // see https://au.mathworks.com/help/map/ref/geodetic2enu.html
    #[test]
    fn geodetic_to_local_pos_enu() {
        let origin = GeodeticPosition::new(
            NVector::from_lat_long_degrees(46.017, 7.750),
            Length::from_metres(1673.0),
        );
        let point = GeodeticPosition::new(
            NVector::from_lat_long_degrees(45.976, 7.658),
            Length::from_metres(4531.0),
        );

        let enu: LocalFrame<Ellipsoid> = LocalFrame::enu(origin, Ellipsoid::WGS84);

        let local = enu.geodetic_to_local_position(point);

        assert_eq!(Length::from_metres(-7134.757), local.x().round_mm());
        assert_eq!(Length::from_metres(-4556.322), local.y().round_mm());
        assert_eq!(Length::from_metres(2852.39), local.z().round_mm());
        assert_eq!(Angle::from_degrees(237.4373247), local.azimuth().round_d7());
        assert_eq!(
            Angle::from_degrees(18.6208639),
            local.elevation().round_d7()
        );
    }

    // see https://au.mathworks.com/help/map/ref/aer2ned.html
    #[test]
    fn aer_to_ned() {
        let az: Angle = Angle::from_degrees(155.427);
        let el = Angle::from_degrees(-23.161);
        let sr = Length::from_metres(10.885);

        let local = LocalPosition::aer_to_ned(az, el, sr);

        assert_eq!(Length::from_metres(-9.101), local.x().round_mm());
        assert_eq!(Length::from_metres(4.162), local.y().round_mm());
        assert_eq!(Length::from_metres(4.281), local.z().round_mm());
        assert_eq!(az, local.azimuth().round_d7());
        assert_eq!(el, local.elevation().round_d7());
        assert_eq!(sr, local.slant_range().round_mm());
    }

    // https://au.mathworks.com/help/map/ref/aer2enu.html
    #[test]
    fn aer_to_enu() {
        let az: Angle = Angle::from_degrees(34.1160);
        let el = Angle::from_degrees(4.1931);
        let sr = Length::from_metres(15.1070);

        let local = LocalPosition::aer_to_enu(az, el, sr);

        assert_eq!(Length::from_metres(8.45), local.x().round_mm());
        assert_eq!(Length::from_metres(12.474), local.y().round_mm());
        assert_eq!(Length::from_metres(1.105), local.z().round_mm());
        assert_eq!(az, local.azimuth().round_d7());
        assert_eq!(el, local.elevation().round_d7());
        assert_eq!(sr, local.slant_range().round_mm());
    }

    #[test]
    fn local_to_geodetic_pos_enu() {
        let origin = GeodeticPosition::new(
            NVector::from_lat_long_degrees(46.017, 7.750),
            Length::from_metres(1673.0),
        );
        let point = GeodeticPosition::new(
            NVector::from_lat_long_degrees(45.976, 7.658),
            Length::from_metres(4531.0),
        );

        let enu: LocalFrame<Ellipsoid> = LocalFrame::enu(origin, Ellipsoid::WGS84);

        let local_enu: LocalPosition = enu.geodetic_to_local_position(point);

        // LocalPosition::new is NED
        let local_ned = LocalPosition::new(local_enu.y(), local_enu.x(), -local_enu.z());

        assert_geod_eq_d7_mm(point, enu.local_to_geodetic_position(local_ned));
    }

    #[test]
    fn transitiviy_enu() {
        let point_a = GeodeticPosition::new(
            NVector::from_lat_long_degrees(1.0, 2.0),
            Length::from_metres(-3.0),
        );
        let point_b = GeodeticPosition::new(
            NVector::from_lat_long_degrees(4.0, 5.0),
            Length::from_metres(-6.0),
        );

        let enu = LocalFrame::enu(point_a, Ellipsoid::WGS84);
        assert_geod_eq_d7_mm(
            point_b,
            enu.local_to_geodetic_position(enu.geodetic_to_local_position(point_b)),
        )
    }

    #[test]
    fn transitiviy_ned() {
        let point_a = GeodeticPosition::new(
            NVector::from_lat_long_degrees(1.0, 2.0),
            Length::from_metres(-3.0),
        );
        let point_b = GeodeticPosition::new(
            NVector::from_lat_long_degrees(4.0, 5.0),
            Length::from_metres(-6.0),
        );

        let ned = LocalFrame::ned(point_a, Ellipsoid::WGS84);
        assert_geod_eq_d7_mm(
            point_b,
            ned.local_to_geodetic_position(ned.geodetic_to_local_position(point_b)),
        )
    }

    #[test]
    fn transitiviy_body() {
        let point_a = GeodeticPosition::new(
            NVector::from_lat_long_degrees(1.0, 2.0),
            Length::from_metres(-3.0),
        );
        let point_b = GeodeticPosition::new(
            NVector::from_lat_long_degrees(4.0, 5.0),
            Length::from_metres(-6.0),
        );

        let body = LocalFrame::body(
            Angle::from_degrees(45.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(5.0),
            point_a,
            Ellipsoid::WGS84,
        );
        assert_geod_eq_d7_mm(
            point_b,
            body.local_to_geodetic_position(body.geodetic_to_local_position(point_b)),
        )
    }

    #[test]
    fn transitiviy_local_level() {
        let point_a = GeodeticPosition::new(
            NVector::from_lat_long_degrees(1.0, 2.0),
            Length::from_metres(-3.0),
        );
        let point_b = GeodeticPosition::new(
            NVector::from_lat_long_degrees(4.0, 5.0),
            Length::from_metres(-6.0),
        );

        let local_level =
            LocalFrame::local_level(Angle::from_degrees(45.0), point_a, Ellipsoid::WGS84);
        assert_geod_eq_d7_mm(
            point_b,
            local_level.local_to_geodetic_position(local_level.geodetic_to_local_position(point_b)),
        )
    }

    #[test]
    fn test_r2xyz() {
        let m = Mat33::new(
            Vec3::new(
                0.7044160264027587,
                -6.162841671621935e-2,
                0.7071067811865475,
            ),
            Vec3::new(0.559725765762092, 0.6608381550289296, -0.5),
            Vec3::new(0.43646893232965345, 0.7479938977765876, 0.5),
        );
        let (x, y, z) = r2xyz(m);
        assert_eq!(Angle::from_degrees(45.0), x.round_d7());
        assert_eq!(Angle::from_degrees(45.0), y.round_d7());
        assert_eq!(Angle::from_degrees(5.0), z.round_d7());
    }

    #[test]
    fn test_r2zyx() {
        let m = Mat33::new(
            Vec3::new(
                0.9254165783983234,
                1.802831123629725e-2,
                0.37852230636979245,
            ),
            Vec3::new(
                0.16317591116653482,
                0.8825641192593856,
                -0.44096961052988237,
            ),
            Vec3::new(-0.3420201433256687, 0.46984631039295416, 0.8137976813493738),
        );

        let (z, y, x) = r2zyx(m);
        assert_eq!(Angle::from_degrees(10.0), z.round_d7());
        assert_eq!(Angle::from_degrees(20.0), y.round_d7());
        assert_eq!(Angle::from_degrees(30.0), x.round_d7());
    }
}
