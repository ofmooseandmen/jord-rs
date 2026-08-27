use std::fmt::Debug;
use std::marker::PhantomData;

use crate::{
    surface::Surface, Angle, Cartesian3DVector, GeocentricPosition, GeodeticPosition, LatLong,
    Length, Mat33, Vec3,
};

/// The orientation of a local frame.
pub trait FrameOrientation: Copy + Clone + Debug + PartialEq + Eq + Default {
    /// Maps North, East, and Up components to frame (x, y, z) coordinates.
    fn nez_to_xyz(north: Length, east: Length, z_up: Length) -> (Length, Length, Length);
}

/// East-North-Up (ENU) orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Enu;
impl FrameOrientation for Enu {
    #[inline]
    fn nez_to_xyz(north: Length, east: Length, z_up: Length) -> (Length, Length, Length) {
        (east, north, z_up)
    }
}

/// North-East-Down (NED) orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Ned;
impl FrameOrientation for Ned {
    #[inline]
    fn nez_to_xyz(north: Length, east: Length, z_up: Length) -> (Length, Length, Length) {
        (north, east, -z_up)
    }
}

/// Body (Vehicle) orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Body;
impl FrameOrientation for Body {
    #[inline]
    fn nez_to_xyz(north: Length, east: Length, z_up: Length) -> (Length, Length, Length) {
        (north, east, -z_up)
    }
}

/// Wander azimuth orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WanderAzimuth;
impl FrameOrientation for WanderAzimuth {
    #[inline]
    fn nez_to_xyz(north: Length, east: Length, z_up: Length) -> (Length, Length, Length) {
        (north, east, -z_up)
    }
}

/// A vector whose length and direction is such that it goes from the origin
/// of frame A to the origin of frame B, i.e. the position of B relative to A.
///
/// [LocalPosition] implements [Add](::std::ops::Add), [Sub](::std::ops::Sub),
/// [Mul](::std::ops::Mul) and [Div](::std::ops::Div), among others.
#[derive(PartialEq, Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))] // codecov:ignore:this
pub struct LocalPosition<O: FrameOrientation> {
    x: Length,
    y: Length,
    z: Length,
    _o: PhantomData<O>,
}

/// [LocalPosition] for [Enu] frame.
pub type EnuPosition = LocalPosition<Enu>;

/// [LocalPosition] for [Ned] frame.
pub type NedPosition = LocalPosition<Ned>;

/// [LocalPosition] for [Body] frame.
pub type BodyPosition = LocalPosition<Body>;

/// [LocalPosition] for [WanderAzimuth] frame.
pub type WanderAzimuthPosition = LocalPosition<WanderAzimuth>;

impl<O: FrameOrientation> LocalPosition<O> {
    /// Creates a [LocalPosition] from the given coordinates.
    pub const fn new(x: Length, y: Length, z: Length) -> Self {
        Self {
            x,
            y,
            z,
            _o: PhantomData,
        }
    }

    /// Creates a [LocalPosition] from the given coordinates in metres.
    pub fn from_metres(x: f64, y: f64, z: f64) -> Self {
        Self::new(
            Length::from_metres(x),
            Length::from_metres(y),
            Length::from_metres(z),
        )
    }

    /// Transforms the given local azimuth-elevation-range (AER) spherical coordinates
    /// to local Cartesian coordinates.
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, BodyPosition, Cartesian3DVector, Length};
    ///
    /// let az: Angle = Angle::from_degrees(155.427);
    /// let el = Angle::from_degrees(-23.161); // elevation is negative, so resulting z (down) will be positive
    /// let sr = Length::from_metres(10.885);
    ///
    /// let local = BodyPosition::from_aer(az, el, sr);
    /// assert_eq!(Length::from_metres(-9.101), local.x().round_mm());
    /// assert_eq!(Length::from_metres(4.162), local.y().round_mm());
    /// assert_eq!(Length::from_metres(4.281), local.z().round_mm());
    /// ```
    pub fn from_aer(azimuth: Angle, elevation: Angle, slant_range: Length) -> Self {
        let (north, east, z_up) = aer_to_nez(azimuth, elevation, slant_range);
        let (x, y, z) = O::nez_to_xyz(north, east, z_up);
        Self::new(x, y, z)
    }

    fn from_vec3_metres(v: Vec3) -> Self {
        Self::from_metres(v.x(), v.y(), v.z())
    }

    /// Returns the slant range - distance from origin in the local system.
    pub fn slant_range(&self) -> Length {
        Length::from_metres(self.as_metres().norm())
    }

    /// Absolute elevation with respect to the z-axis.
    fn z_elevation(&self) -> Angle {
        Angle::from_radians((self.z() / self.slant_range()).asin())
    }

    /// Bearing calculated from origin vector components 'north' and 'east'.
    fn bearing(n: Length, e: Length) -> Angle {
        Angle::from_radians(e.as_metres().atan2(n.as_metres())).normalised()
    }
}

impl NedPosition {
    /// Returns the true compass angle from North (0°..360°).
    pub fn azimuth(&self) -> Angle {
        Self::bearing(self.x(), self.y())
    }

    /// Returns the angle above the Earth's local horizontal plane.
    pub fn elevation(&self) -> Angle {
        -self.z_elevation()
    }
}

impl EnuPosition {
    /// Returns the true compass angle from North (0°..360°).
    pub fn azimuth(&self) -> Angle {
        Self::bearing(self.y(), self.x())
    }

    /// Returns the angle above the Earth's local horizontal plane.
    pub fn elevation(&self) -> Angle {
        self.z_elevation()
    }
}

impl BodyPosition {
    /// Returns the angle off the vehicle nose in the body plane (-180°..+180°).
    pub fn relative_bearing(&self) -> Angle {
        Self::bearing(self.x(), self.y())
    }

    /// Vertical angle above (+) or below (-) the vehicle's structural plane.
    pub fn relative_elevation(&self) -> Angle {
        -self.z_elevation()
    }
}

impl WanderAzimuthPosition {
    /// Returns the angle relative to the Wander x-axis (requires adding wander angle for True North).
    pub fn wander_bearing(&self) -> Angle {
        Self::bearing(self.x(), self.y())
    }

    /// Returns the angle above the Earth's local horizontal plane (z is still local Down).
    pub fn elevation(&self) -> Angle {
        -self.z_elevation()
    }
}

fn aer_to_nez(azimuth: Angle, elevation: Angle, slant_range: Length) -> (Length, Length, Length) {
    let cose = elevation.as_radians().cos();
    let east = azimuth.as_radians().sin() * cose * slant_range;
    let north = azimuth.as_radians().cos() * cose * slant_range;
    let z = elevation.as_radians().sin() * slant_range;
    (north, east, z)
}

impl<O: FrameOrientation> Cartesian3DVector for LocalPosition<O> {
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
        Self::new(round(self.x()), round(self.y()), round(self.z()))
    }
}

impl<O: FrameOrientation> ::std::ops::Add for LocalPosition<O> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        LocalPosition::from_vec3_metres(self.as_metres() + rhs.as_metres())
    }
}

impl<O: FrameOrientation> ::std::ops::Sub for LocalPosition<O> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        LocalPosition::from_vec3_metres(self.as_metres() - rhs.as_metres())
    }
}

impl<O: FrameOrientation> ::std::ops::Mul<f64> for LocalPosition<O> {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        LocalPosition::from_vec3_metres(self.as_metres() * rhs)
    }
}

impl<O: FrameOrientation> ::std::ops::Mul<LocalPosition<O>> for f64 {
    type Output = LocalPosition<O>;

    fn mul(self, rhs: LocalPosition<O>) -> Self::Output {
        LocalPosition::from_vec3_metres(rhs.as_metres() * self)
    }
}

impl<O: FrameOrientation> ::std::ops::Div<f64> for LocalPosition<O> {
    type Output = Self;

    fn div(self, rhs: f64) -> Self {
        LocalPosition::from_vec3_metres(self.as_metres() / rhs)
    }
}

impl<O: FrameOrientation> ::std::ops::Neg for LocalPosition<O> {
    type Output = Self;

    fn neg(self) -> Self {
        LocalPosition::from_vec3_metres(-self.as_metres())
    }
}

/// The origin of a [LocalFrame]. This trait exists to allow the creation
/// of local frames using either [GeodeticPosition] or [GeocentricPosition].
pub trait LocalFrameOrigin<S>: Clone + Copy + Debug + Sized {
    /// Returns the [GeodeticPosition] corresponding to the frame origin.
    fn geodetic(&self, surface: S) -> GeodeticPosition;

    /// Returns the [GeocentricPosition] corresponding to the frame origin.
    fn geocentric(&self, surface: S) -> GeocentricPosition;
}

impl<S> LocalFrameOrigin<S> for GeodeticPosition
where
    S: Surface,
{
    fn geodetic(&self, _surface: S) -> GeodeticPosition {
        *self
    }

    fn geocentric(&self, surface: S) -> GeocentricPosition {
        surface.geodetic_to_geocentric_position(*self)
    }
}

impl<S> LocalFrameOrigin<S> for GeocentricPosition
where
    S: Surface,
{
    fn geodetic(&self, surface: S) -> GeodeticPosition {
        surface.geocentric_to_geodetic_position(*self)
    }

    fn geocentric(&self, _surface: S) -> GeocentricPosition {
        *self
    }
}

/// Defines a local Cartesian coordinate frame with two axes forming a horizontal
/// tangent plane to the reference surface ([ellipsoid](crate::ellipsoidal::Ellipsoid) or
/// [sphere](crate::spherical::Sphere)) at a specified tangent point. Assuming several
/// calculations are needed in a limited area, position calculations can be performed
/// relative to this system to get approximate horizontal and vertical components.
#[derive(PartialEq, Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))] // codecov:ignore:this
pub struct LocalFrame<S: Surface, O: FrameOrientation> {
    origin: Vec3,
    dir_rm: Mat33,
    inv_rm: Mat33,
    surface: S,
    _o: PhantomData<O>,
}

/// East-North-Up Frame: placed locally on a vehicle or platform and moves around with the system.
/// This frame is typically used in targeting and tracking applications.
///
/// Orientation:
/// - the y-axis points to True North
/// - the z-axis points away from the interior of the earth
/// - the x-axis completes the right-handed system pointing east
///
/// See note in [Ned] for suitability.
pub type EnuFrame<S> = LocalFrame<S, Enu>;

impl<S: Surface> EnuFrame<S> {
    /// [East-North-Up (local level)](Enu) frame fixed to the given origin.
    pub fn new<O: LocalFrameOrigin<S>>(origin: O, surface: S) -> Self {
        let vo = origin.geodetic(surface).horizontal_position().as_vec3();
        let ru = vo;
        let re = Vec3::UNIT_Z.orthogonal_to(vo);
        let rn = ru.cross_prod(re);
        let inv_rm = Mat33::new(re, rn, ru);

        Self {
            origin: origin.geocentric(surface).as_metres(),
            dir_rm: inv_rm.transpose(),
            inv_rm,
            surface,
            _o: PhantomData,
        }
    }
}

/// North-East-Down (NED) frame: fixed to the vehicle or platform and moves with the body frame.
/// This frame is typically used in aviation, marine navigation, and standard compass-based vehicle tracking.
///
/// Orientation:
/// - the x-axis points to True North
/// - the z-axis points towards the interior of the earth
/// - the y-axis completes the right-handed system pointing east
///
/// Note: When moving relative to the Earth, the frame rotates about its z-axis to allow the
/// x-axis to always point towards north. When getting close to the poles this rotation rate
/// will increase, being infinite at the poles. The poles are thus singularities and the direction of
/// the x- and y-axes are not defined here. Hence, this coordinate frame is not suitable for
/// general calculations.
pub type NedFrame<S> = LocalFrame<S, Ned>;

impl<S: Surface> NedFrame<S> {
    /// [North-East-Down (local level)](Ned) frame fixed to the given origin.
    pub fn new<O: LocalFrameOrigin<S>>(origin: O, surface: S) -> Self {
        let vo = origin.geodetic(surface).horizontal_position().as_vec3();
        let rd = -1.0 * vo;
        let re = Vec3::UNIT_Z.orthogonal_to(vo);
        let rn = re.cross_prod(rd);
        let inv_rm = Mat33::new(rn, re, rd);

        Self {
            origin: origin.geocentric(surface).as_metres(),
            dir_rm: inv_rm.transpose(),
            inv_rm,
            surface,
            _o: PhantomData,
        }
    }
}

/// Body frame: in most applications, the position, velocity, and orientation of a system are found
/// using sensors mounted to a vehicle or platform. The platform can have its own reference frame
/// known as the body frame, or sometimes called the vehicle frame. This type of reference frame
/// consists of an origin that is typically placed at the platform's center of gravity and three orthogonal
/// axes that comprise a right-handed system.
///
/// Orientation:
/// - the x-axis is pointing forward
/// - the y-axis is pointing to the right
/// - the z-axis is pointing down
pub type BodyFrame<S> = LocalFrame<S, Body>;

impl<S: Surface> BodyFrame<S> {
    /// [Body](Body) frame fixed to a vehicle at the given reference location (origin) and yaw/pitch/roll.
    pub fn new<O: LocalFrameOrigin<S>>(
        yaw: Angle,
        pitch: Angle,
        roll: Angle,
        origin: O,
        surface: S,
    ) -> Self {
        let r_nb = zyx2r(yaw, pitch, roll);
        let r_en = NedFrame::new(origin, surface).dir_rm;
        let dir_rm = r_en * r_nb;

        Self {
            origin: origin.geocentric(surface).as_metres(),
            dir_rm,
            inv_rm: dir_rm.transpose(),
            surface,
            _o: PhantomData,
        }
    }

    /// Translates the body frame origin by a displacement vector defined in body coordinates.
    pub fn translate(&self, offset: BodyPosition) -> Self {
        let translation_ecef = offset.as_metres() * self.dir_rm;
        Self {
            origin: self.origin + translation_ecef,
            dir_rm: self.dir_rm,
            inv_rm: self.inv_rm,
            surface: self.surface,
            _o: PhantomData,
        }
    }

    /// Rotates the body frame intrinsically by relative yaw, pitch, and roll adjustments.
    pub fn rotate(&self, delta_yaw: Angle, delta_pitch: Angle, delta_roll: Angle) -> Self {
        let r_delta = zyx2r(delta_yaw, delta_pitch, delta_roll);
        let dir_rm = self.dir_rm * r_delta;
        Self {
            origin: self.origin,
            dir_rm,
            inv_rm: dir_rm.transpose(),
            surface: self.surface,
            _o: PhantomData,
        }
    }
}

/// Local level, Wander azimuth frame: advanced inertial navigation systems (INS) use this frame
/// to avoid mathematical singularities when navigating near the Earth's geographic poles, where
/// meridians converge and traditional North-tracking fails.
///
/// Orientation: The z-axis is pointing down. Initially, the x-axis points towards north, and the
///   y-axis points towards east, but as the vehicle moves they are not rotating about the z-axis
///   (their angular velocity relative to the Earth has zero component along the z-axis).
///   (Note: Any initial horizontal direction of the x- and y-axes is valid for L, but if the
///   initial position is outside the poles, north and east are usually chosen for convenience.)
///
/// Notes: This frame is equal to the [NED frame](Ned) except for the rotation about the z-axis,
/// which is always zero for this frame (relative to Earth). Hence, at a given time, the only
/// difference between the frames is an angle between the x-axis of L and the north direction;
/// this angle is called the wander azimuth angle. This frame is well suited for general
/// calculations, as it is non-singular.
pub type WanderAzimuthFrame<S> = LocalFrame<S, WanderAzimuth>;

impl<S: Surface> WanderAzimuthFrame<S> {
    /// [Local Level, Wander Azimuth](WanderAzimuth) fixed to the given origin and rotated by the given wander azimuth.
    pub fn new<O: LocalFrameOrigin<S>>(wander_azimuth: Angle, origin: O, surface: S) -> Self {
        let ll = LatLong::from_nvector(origin.geodetic(surface).horizontal_position());
        let r = xyz2r(ll.longitude(), -ll.latitude(), wander_azimuth);
        let r_ee = Mat33::new(Vec3::NEG_UNIT_Z, Vec3::UNIT_Y, Vec3::UNIT_X);
        let dir_rm = r_ee * r;

        Self {
            origin: origin.geocentric(surface).as_metres(),
            dir_rm,
            inv_rm: dir_rm.transpose(),
            surface,
            _o: PhantomData,
        }
    }
}

impl<S, O> LocalFrame<S, O>
where
    S: Surface,
    O: FrameOrientation,
{
    /// Converts the given [GeocentricPosition] into a [LocalPosition]: the exact vector between this frame
    /// origin and the given position. The resulting [LocalPosition] orientation is the one of this frame.
    pub fn geocentric_to_local_position(&self, p: GeocentricPosition) -> LocalPosition<O> {
        let p_vec3 = p.as_metres();
        // delta in 'Earth' frame.
        let de = p_vec3 - self.origin;
        let d = de * self.inv_rm;
        LocalPosition::from_vec3_metres(d)
    }

    /// Converts the given [GeodeticPosition] into a [LocalPosition]: the exact vector between this frame
    /// origin and the given position. The resulting [LocalPosition] orientation is the one of this frame.
    pub fn geodetic_to_local_position(&self, p: GeodeticPosition) -> LocalPosition<O> {
        self.geocentric_to_local_position(self.surface.geodetic_to_geocentric_position(p))
    }

    /// Converts the given [LocalPosition] into a [GeocentricPosition]: the geocentric position of an object
    /// which is located at a bearing and distance from this frame origin. The given [LocalPosition]
    /// is re-oriented to match the orientation of this frame if required.
    pub fn local_to_geocentric_position(&self, p: LocalPosition<O>) -> GeocentricPosition {
        let c = p.as_metres() * self.dir_rm;
        let v = self.origin + c;
        GeocentricPosition::from_vec3_metres(v)
    }

    /// Converts the given [LocalPosition] into a [GeodeticPosition]: the geodetic position of an object
    /// which is located at a bearing and distance from this frame origin. The given [LocalPosition]
    /// is re-oriented to match the orientation of this frame if required.
    pub fn local_to_geodetic_position(&self, p: LocalPosition<O>) -> GeodeticPosition {
        self.surface
            .geocentric_to_geodetic_position(self.local_to_geocentric_position(p))
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
        ellipsoidal::Ellipsoid, positions::assert_geod_eq_d7_mm, r2xyz, r2zyx, Angle, BodyFrame,
        BodyPosition, Cartesian3DVector, EnuFrame, EnuPosition, GeodeticPosition, LatLong, Length,
        LocalPosition, Mat33, NVector, NedFrame, NedPosition, Surface, Vec3, WanderAzimuthFrame,
        WanderAzimuthPosition,
    };

    #[test]
    fn add_local_positions() {
        let l1 = NedPosition::from_vec3_metres(Vec3::new(1.0, 2.0, 3.0));
        let l2 = NedPosition::from_vec3_metres(Vec3::new(1.0, 2.0, 3.0));

        assert_eq!(
            NedPosition::from_vec3_metres(Vec3::new(2.0, 4.0, 6.0)),
            l1 + l2
        );
    }

    #[test]
    fn sub_local_positions() {
        let l1 = EnuPosition::from_vec3_metres(Vec3::new(1.0, 2.0, 3.0));
        let l2: LocalPosition<_> = EnuPosition::from_vec3_metres(Vec3::new(1.0, 2.0, 3.0));

        assert_eq!(
            EnuPosition::from_vec3_metres(Vec3::new(0.0, 0.0, 0.0)),
            l1 - l2
        );
    }

    #[test]
    fn mul_local_position() {
        let l = BodyPosition::from_vec3_metres(Vec3::new(1.0, 2.0, 3.0));

        assert_eq!(
            BodyPosition::from_vec3_metres(Vec3::new(2.0, 4.0, 6.0)),
            l * 2.0
        );

        assert_eq!(
            BodyPosition::from_vec3_metres(Vec3::new(2.0, 4.0, 6.0)),
            2.0 * l
        )
    }

    #[test]
    fn div_local_position() {
        let l = WanderAzimuthPosition::from_vec3_metres(Vec3::new(2.0, 4.0, 6.0));

        assert_eq!(
            WanderAzimuthPosition::from_vec3_metres(Vec3::new(1.0, 2.0, 3.0)),
            l / 2.0
        )
    }

    #[test]
    fn neg_local_position() {
        let l = NedPosition::from_vec3_metres(Vec3::new(2.0, 4.0, 6.0));

        assert_eq!(
            NedPosition::from_vec3_metres(Vec3::new(-2.0, -4.0, -6.0)),
            -l
        )
    }

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

        let f0 =
            WanderAzimuthFrame::new(Angle::from_degrees(90.0), ship_position_0, Ellipsoid::WGS84);
        let local_0 = f0.geodetic_to_local_position(sensor_position).round_mm();

        assert_eq!(Length::from_metres(278.257), local_0.x());
        assert_eq!(Length::from_metres(-10.0), local_0.y());
        assert_eq!(Length::ZERO, local_0.z().round_m());
        assert_eq!(358.0, local_0.wander_bearing().as_degrees().round());

        let f1 =
            WanderAzimuthFrame::new(Angle::from_degrees(90.0), ship_position_1, Ellipsoid::WGS84);

        let local_1 = f1.geodetic_to_local_position(sensor_position).round_mm();

        assert_eq!(Length::from_metres(-278.257), local_1.x());
        assert_eq!(Length::from_metres(-10.0), local_1.y());
        assert_eq!(Length::ZERO, local_1.z().round_m());
        assert_eq!(182.0, local_1.wander_bearing().as_degrees().round());
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

        let f0 = NedFrame::new(ship_position_0, Ellipsoid::WGS84);
        let local_0 = f0.geodetic_to_local_position(sensor_position).round_mm();

        assert_eq!(Length::ZERO, local_0.x());
        assert_eq!(Length::from_metres(278.257), local_0.y());
        assert_eq!(Length::ZERO, local_0.z().round_m());
        assert_eq!(90.0, local_0.azimuth().as_degrees());

        let f1 = NedFrame::new(ship_position_1, Ellipsoid::WGS84);
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

        let ned: crate::LocalFrame<Ellipsoid, crate::Ned> = NedFrame::new(origin, Ellipsoid::WGS84);

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

        let enu = EnuFrame::new(origin, Ellipsoid::WGS84);

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

        let local = NedPosition::from_aer(az, el, sr);

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

        let local = EnuPosition::from_aer(az, el, sr);

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

        let enu = EnuFrame::new(origin, Ellipsoid::WGS84);

        let local_enu = enu.geodetic_to_local_position(point);

        assert_geod_eq_d7_mm(point, enu.local_to_geodetic_position(local_enu));
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

        let enu = EnuFrame::new(point_a, Ellipsoid::WGS84);
        assert_geod_eq_d7_mm(
            point_b,
            enu.local_to_geodetic_position(enu.geodetic_to_local_position(point_b)),
        );

        let enu2 = EnuFrame::new(
            Ellipsoid::WGS84.geodetic_to_geocentric_position(point_a),
            Ellipsoid::WGS84,
        );
        assert_geod_eq_d7_mm(
            point_b,
            enu2.local_to_geodetic_position(enu2.geodetic_to_local_position(point_b)),
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

        let ned = NedFrame::new(point_a, Ellipsoid::WGS84);
        assert_geod_eq_d7_mm(
            point_b,
            ned.local_to_geodetic_position(ned.geodetic_to_local_position(point_b)),
        );

        let ned2 = NedFrame::new(
            Ellipsoid::WGS84.geodetic_to_geocentric_position(point_a),
            Ellipsoid::WGS84,
        );
        assert_geod_eq_d7_mm(
            point_b,
            ned2.local_to_geodetic_position(ned2.geodetic_to_local_position(point_b)),
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

        let body = BodyFrame::new(
            Angle::from_degrees(45.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(5.0),
            point_a,
            Ellipsoid::WGS84,
        );
        assert_geod_eq_d7_mm(
            point_b,
            body.local_to_geodetic_position(body.geodetic_to_local_position(point_b)),
        );

        let body2 = BodyFrame::new(
            Angle::from_degrees(45.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(5.0),
            Ellipsoid::WGS84.geodetic_to_geocentric_position(point_a),
            Ellipsoid::WGS84,
        );
        assert_geod_eq_d7_mm(
            point_b,
            body2.local_to_geodetic_position(body2.geodetic_to_local_position(point_b)),
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

        let wa = WanderAzimuthFrame::new(
            Angle::from_degrees(45.0),
            Ellipsoid::WGS84.geodetic_to_geocentric_position(point_a),
            Ellipsoid::WGS84,
        );
        assert_geod_eq_d7_mm(
            point_b,
            wa.local_to_geodetic_position(wa.geodetic_to_local_position(point_b)),
        );

        let wa2 = WanderAzimuthFrame::new(Angle::from_degrees(45.0), point_a, Ellipsoid::WGS84);
        assert_geod_eq_d7_mm(
            point_b,
            wa2.local_to_geodetic_position(wa2.geodetic_to_local_position(point_b)),
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

    #[test]
    fn body_relative_angles() {
        // Target forward (+x), right (+y), above (-z)
        let target = BodyPosition::from_metres(10.0, 10.0, -10.0);

        assert_eq!(Angle::from_degrees(45.0), target.relative_bearing());
        assert_eq!(
            Angle::from_degrees(35.2643897),
            target.relative_elevation().round_d7()
        );
    }

    #[test]
    fn body_relative_elevation_below_plane() {
        // Target directly below vehicle (+z in Body frame)
        let target = BodyPosition::from_metres(0.0, 0.0, 10.0);
        assert_eq!(
            Angle::from_degrees(-90.0),
            target.relative_elevation().round_d7()
        );
    }

    #[test]
    fn sensor_observation_to_geodetic_position() {
        let ac_pos = GeodeticPosition::new(
            NVector::from_lat_long_degrees(54.0, 154.0),
            Length::from_feet(35000.0),
        );
        let yaw = Angle::from_degrees(45.0);
        let pitch = Angle::from_degrees(2.0);
        let roll = Angle::from_degrees(-1.0);
        let ac_frame = BodyFrame::new(yaw, pitch, roll, ac_pos, Ellipsoid::WGS84);

        // Sensor fixed offset relative to aircraft CG (x: +2.5m forward, y: 0.0m, z: +0.8m down)
        let sensor_offset_body = BodyPosition::new(
            Length::from_metres(2.5),
            Length::from_metres(0.0),
            Length::from_metres(0.8),
        );
        let sensor_frame = ac_frame.translate(sensor_offset_body);

        // Sensor observes a target at relative azimuth 30°, elevation -15° (downward), range 5000m
        let rel_azimuth = Angle::from_degrees(30.0);
        let rel_elevation = Angle::from_degrees(-15.0);
        let slant_range = Length::from_metres(5000.0);

        let target_rel_sensor = BodyPosition::from_aer(rel_azimuth, rel_elevation, slant_range);

        // Conversion of observation to geodetic position
        assert_geod_eq_d7_mm(
            GeodeticPosition::new(
                NVector::from_lat_long_degrees(54.01132811615137, 154.07176275677563),
                Length::from_feet(31378.3653906617),
            ),
            sensor_frame.local_to_geodetic_position(target_rel_sensor),
        )
    }
}
