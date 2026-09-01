use std::fmt::Debug;
use std::marker::PhantomData;

use crate::{
    local::{
        orientation::align_to_z_down_matrix, Body, Enu, FrameOrientation, LocalNavigationFrame,
        LocalVector, Ned, WanderAzimuth,
    },
    surface::Surface,
    Angle, GeocentricPosition, GeodeticPosition, LatLong, Mat33, PositionVector, Vec3,
};

/// A 3D local Cartesian coordinate frame anchored at a reference origin on or relative
/// to a reference surface ([Ellipsoid](crate::ellipsoidal::Ellipsoid) or [Sphere](crate::spherical::Sphere)).
///
/// Two of the frame's orthogonal axes lie within the local horizontal plane (tangent to the surface
/// at the origin position), while the third axis is aligned with the surface normal vector.
///
/// Unlike flat-surface approximations, `LocalFrame` performs exact 3D rigid-body transformations
/// between global geocentric positions ([`GeocentricPosition`]) and local Cartesian displacement
/// vectors ([`LocalVector`]).
///
/// The precise orientation and handedness of the axes are governed by the generic parameter `O`
/// implementing [`FrameOrientation`] (such as [`Body`], [`Enu`], [`Ned`], or [`WanderAzimuth`]).
#[derive(PartialEq, Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))] // codecov:ignore:this
pub struct LocalFrame<O: FrameOrientation> {
    origin: Vec3,
    dir_rm: Mat33,
    inv_rm: Mat33,
    #[cfg_attr(feature = "serde", serde(skip))]
    _o: PhantomData<O>,
}

/// A local tangent plane navigation frame aligned with East-North-Up (ENU): placed
/// locally on a vehicle or platform and moves around with the system. This frame is
/// typically used in targeting and tracking applications.
///
/// Orientation:
/// - the y-axis points to True North
/// - the z-axis points away from the interior of the earth
/// - the x-axis completes the right-handed system pointing east
///
/// See note in [NedFrame] for suitability.
pub type EnuFrame = LocalFrame<Enu>;

impl EnuFrame {
    /// [East-North-Up (local level)](Enu) frame fixed to the given [geocentric origin](GeocentricPosition).
    pub fn from_geocentric(origin: GeocentricPosition, surface: impl Surface) -> Self {
        Self::new(surface.geocentric_to_geodetic_position(origin), origin)
    }

    /// [East-North-Up (local level)](Enu) frame fixed to the given [geodetic origin](GeodeticPosition).
    pub fn from_geodetic(origin: GeodeticPosition, surface: impl Surface) -> Self {
        Self::new(origin, surface.geodetic_to_geocentric_position(origin))
    }

    fn new(o_geod: GeodeticPosition, o_geoc: GeocentricPosition) -> Self {
        let vo = o_geod.horizontal_position().as_vec3();
        let ru = vo;
        let re = Vec3::UNIT_Z.orthogonal_to(vo);
        let rn = ru.cross_prod(re);
        let inv_rm = Mat33::new(re, rn, ru);

        Self {
            origin: o_geoc.as_metres(),
            dir_rm: inv_rm.transpose(),
            inv_rm,
            _o: PhantomData,
        }
    }
}

/// A local tangent plane navigation frame aligned with North-East-Down (NED): fixed
/// to the vehicle or platform and moves with the body frame.
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
pub type NedFrame = LocalFrame<Ned>;

impl NedFrame {
    /// [North-East-Down (local level)](Ned) frame fixed to the given [geocentric origin](GeocentricPosition).
    pub fn from_geocentric(origin: GeocentricPosition, surface: impl Surface) -> Self {
        Self::new(surface.geocentric_to_geodetic_position(origin), origin)
    }

    /// [North-East-Down (local level)](Ned) frame fixed to the given [geodetic origin](GeodeticPosition).
    pub fn from_geodetic(origin: GeodeticPosition, surface: impl Surface) -> Self {
        Self::new(origin, surface.geodetic_to_geocentric_position(origin))
    }

    fn new(o_geod: GeodeticPosition, o_geoc: GeocentricPosition) -> Self {
        let vo = o_geod.horizontal_position().as_vec3();
        let rd = -1.0 * vo;
        let re = Vec3::UNIT_Z.orthogonal_to(vo);
        let rn = re.cross_prod(rd);
        let inv_rm = Mat33::new(rn, re, rd);

        Self {
            origin: o_geoc.as_metres(),
            dir_rm: inv_rm.transpose(),
            inv_rm,
            _o: PhantomData,
        }
    }
}

/// A vehicle-centric, body-fixed reference frame: in most applications, the position,
/// velocity, and orientation of a system are found using sensors mounted to a vehicle
/// or platform. The platform can have its own reference frame known as the body frame,
/// or sometimes called the vehicle frame. This type of reference frame consists of an
/// origin that is typically placed at the platform's center of gravity and three orthogonal
/// axes that comprise a right-handed system.
///
/// Orientation:
/// - the x-axis is pointing forward
/// - the y-axis is pointing to the right
/// - the z-axis is pointing down
pub type BodyFrame = LocalFrame<Body>;

impl BodyFrame {
    /// [Body] frame fixed to a vehicle at the given reference [geocentric position](GeocentricPosition)
    /// and yaw/pitch/roll.
    pub fn from_geocentric(
        yaw: Angle,
        pitch: Angle,
        roll: Angle,
        origin: GeocentricPosition,
        surface: impl Surface,
    ) -> Self {
        Self::new(
            yaw,
            pitch,
            roll,
            surface.geocentric_to_geodetic_position(origin),
            origin,
        )
    }

    /// [Body] frame fixed to a vehicle at the given reference [geodetic position](GeodeticPosition)
    /// and yaw/pitch/roll.
    pub fn from_geodetic(
        yaw: Angle,
        pitch: Angle,
        roll: Angle,
        origin: GeodeticPosition,
        surface: impl Surface,
    ) -> Self {
        Self::new(
            yaw,
            pitch,
            roll,
            origin,
            surface.geodetic_to_geocentric_position(origin),
        )
    }

    /// Builds a [Body] frame at the given reference [geodetic position](GeodeticPosition),
    /// with yaw and pitch automatically computed so that the frame's x-axis points directly at
    /// `target`, and the given `roll` about that pointing axis.
    ///
    /// This is a convenience over [`BodyFrame::from_geodetic`] for "look-at"/targeting use cases
    /// (e.g. orienting a sensor or a gimbal frame towards a known target position), where the
    /// desired orientation is naturally defined by a target position rather than known yaw/pitch
    /// angles.
    ///
    /// If `target` coincides with `origin`, the direction to look in is undefined; this returns
    /// a frame facing due north and level (yaw = 0°, pitch = 0°) in that case.
    pub fn looking_at_geodetic(
        origin: GeodeticPosition,
        target: GeodeticPosition,
        roll: Angle,
        surface: impl Surface,
    ) -> Self {
        let o_geoc = surface.geodetic_to_geocentric_position(origin);
        let t_geoc = surface.geodetic_to_geocentric_position(target);
        Self::looking_at(origin, o_geoc, t_geoc, roll)
    }

    /// Builds a [Body] frame at the given reference [geocentric position](GeocentricPosition),
    /// with yaw and pitch automatically computed so that the frame's x-axis points directly at
    /// `target`, and the given `roll` about that pointing axis.
    ///
    /// See [`BodyFrame::looking_at_geodetic`] for details.
    pub fn looking_at_geocentric(
        origin: GeocentricPosition,
        target: GeocentricPosition,
        roll: Angle,
        surface: impl Surface,
    ) -> Self {
        let o_geod = surface.geocentric_to_geodetic_position(origin);
        Self::looking_at(o_geod, origin, target, roll)
    }

    fn new(
        yaw: Angle,
        pitch: Angle,
        roll: Angle,
        o_geod: GeodeticPosition,
        o_geoc: GeocentricPosition,
    ) -> Self {
        let r_nb = zyx2r(yaw, pitch, roll);
        let r_en = NedFrame::new(o_geod, o_geoc).dir_rm;
        let dir_rm = r_en * r_nb;

        Self {
            origin: o_geoc.as_metres(),
            dir_rm,
            inv_rm: dir_rm.transpose(),
            _o: PhantomData,
        }
    }

    fn looking_at(
        o_geod: GeodeticPosition,
        o_geoc: GeocentricPosition,
        target: GeocentricPosition,
        roll: Angle,
    ) -> Self {
        let ned = NedFrame::new(o_geod, o_geoc);
        let delta = ned.local_vector_to(target);
        Self::new(delta.bearing(), delta.elevation(), roll, o_geod, o_geoc)
    }
}

/// A local-level navigation frame designed to avoid geographic polar singularities (Wander Azimuth).
///
/// Unlike North-referenced frames (like NED or ENU) whose axes rotate infinitely fast when passing directly
/// over the geographic poles, the Wander Azimuth frame offsets its horizontal axes by a dynamically tracked
/// wander angle relative to True North.
///
/// Orientation: The z-axis is pointing down. Initially, the x-axis points towards north, and the
///   y-axis points towards east, but as the vehicle moves they are not rotating about the z-axis
///   (their angular velocity relative to the Earth has zero component along the z-axis).
///   (Note: Any initial horizontal direction of the x- and y-axes is valid for L, but if the
///   initial position is outside the poles, north and east are usually chosen for convenience.)
///
/// Notes: This frame is equal to the [NED frame](NedFrame) except for the rotation about the z-axis,
/// which is always zero for this frame (relative to Earth). Hence, at a given time, the only
/// difference between the frames is an angle between the x-axis of L and the north direction;
/// this angle is called the wander azimuth angle. This frame is well suited for general
/// calculations, as it is non-singular.
pub type WanderAzimuthFrame = LocalFrame<WanderAzimuth>;

impl WanderAzimuthFrame {
    /// [Local Level, Wander Azimuth](WanderAzimuth) fixed to the given [geocentric position](GeocentricPosition)
    /// and rotated by the given wander azimuth.
    pub fn from_geocentric(
        wander_azimuth: Angle,
        origin: GeocentricPosition,
        surface: impl Surface,
    ) -> Self {
        Self::new(
            wander_azimuth,
            surface.geocentric_to_geodetic_position(origin),
            origin,
        )
    }

    /// [Local Level, Wander Azimuth](WanderAzimuth) fixed to the given [geodetic position](GeodeticPosition)
    /// and rotated by the given wander azimuth.
    pub fn from_geodetic(
        wander_azimuth: Angle,
        origin: GeodeticPosition,
        surface: impl Surface,
    ) -> Self {
        Self::new(
            wander_azimuth,
            origin,
            surface.geodetic_to_geocentric_position(origin),
        )
    }

    fn new(wander_azimuth: Angle, o_geod: GeodeticPosition, o_geoc: GeocentricPosition) -> Self {
        let ll = LatLong::from_nvector(o_geod.horizontal_position());
        let r = xyz2r(ll.longitude(), -ll.latitude(), wander_azimuth);
        let r_ee = Mat33::new(Vec3::NEG_UNIT_Z, Vec3::UNIT_Y, Vec3::UNIT_X);
        let dir_rm = r_ee * r;

        Self {
            origin: o_geoc.as_metres(),
            dir_rm,
            inv_rm: dir_rm.transpose(),
            _o: PhantomData,
        }
    }
}

impl<O: LocalNavigationFrame> LocalFrame<O> {
    /// Rotates this local navigation frame around the local vertical axis by the specified
    /// angle (clockwise from North toward East following the right-hand rule around Z-down),
    /// yielding a Z-down [WanderAzimuthFrame].
    ///
    /// If called on a Z-up frame like [EnuFrame], the frame axes are first aligned to Z-down
    /// prior to applying the vertical rotation.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{ellipsoidal::Ellipsoid, local::EnuFrame, Angle, GeodeticPosition, LatLong, Length};
    ///
    /// let origin = GeodeticPosition::new(
    ///     LatLong::from_degrees(52.2, 4.9).to_nvector(),
    ///     Length::ZERO,
    /// );
    /// let enu = EnuFrame::from_geodetic(origin, Ellipsoid::WGS84);
    /// let wa = enu.rotate_around_z(Angle::from_degrees(45.0));
    /// ```
    pub fn rotate_around_z(&self, angle: Angle) -> WanderAzimuthFrame {
        let rot_z = zyx2r(angle, Angle::ZERO, Angle::ZERO);
        let dir_rm = self.rotate(self.dir_rm, rot_z);
        LocalFrame {
            origin: self.origin,
            dir_rm,
            inv_rm: dir_rm.transpose(),
            _o: PhantomData,
        }
    }

    /// Returns the 3x3 direction matrix describing the orientation of this frame relative
    /// to the Earth-centred frame.
    ///
    /// This allows to resolve a free vector decomposed in this frame's axes into the
    /// Earth-centred frame: `v_earth = v_local * frame.local_to_earth_matrix()`.
    pub fn local_to_earth_matrix(&self) -> Mat33 {
        self.dir_rm
    }

    /// Returns the 3x3 direction matrix describing the orientation of the Earth-centred
    /// frame relative to this frame.
    ///
    /// This allows to resolve a free vector decomposed in the Earth-centred frame
    /// (e.g. a velocity, force, or angular rate — any quantity with a direction
    /// and magnitude but no associated origin, unlike a position) into this frame's
    /// axes: `v_local = v_earth * frame.earth_to_local_matrix()`.
    pub fn earth_to_local_matrix(&self) -> Mat33 {
        self.inv_rm
    }

    /// Returns the origin of this frame.
    pub fn origin(&self) -> GeocentricPosition {
        GeocentricPosition::from_vec3_metres(self.origin)
    }
}

impl<O> LocalFrame<O>
where
    O: FrameOrientation,
{
    /// Applies a physical displacement within the current frame, followed by a 3D
    /// attitude rotation (yaw, pitch, roll), transforming it into a vehicle-fixed [body](BodyFrame) frame.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{
    ///     ellipsoidal::Ellipsoid, local::NedFrame, local::NedVector, Angle,
    ///     GeodeticPosition, Length, NVector
    /// };
    ///
    /// let ac_pos: GeodeticPosition = GeodeticPosition::new(
    ///     NVector::from_lat_long_degrees(54.0, 154.0),
    ///     Length::from_feet(35000.0),
    /// );
    ///
    /// let ned = NedFrame::from_geodetic(ac_pos, Ellipsoid::WGS84);
    ///
    /// let body = ned.transform(
    ///     NedVector::ZERO,
    ///     Angle::from_degrees(45.0),
    ///     Angle::from_degrees(2.0),
    ///     Angle::from_degrees(-1.0),
    /// );
    /// ```
    pub fn transform(
        &self,
        offset: LocalVector<O>,
        yaw: Angle,
        pitch: Angle,
        roll: Angle,
    ) -> BodyFrame {
        let global_translation = offset.as_metres() * self.dir_rm;
        let new_origin = self.origin + global_translation;
        let r_delta = zyx2r(yaw, pitch, roll);
        let dir_rm = self.rotate(self.dir_rm, r_delta);
        LocalFrame {
            origin: new_origin,
            dir_rm,
            inv_rm: dir_rm.transpose(),
            _o: PhantomData,
        }
    }

    /// Computes the local 3D displacement vector from this frame's origin
    /// to the specified target geocentric position.
    pub fn local_vector_to(&self, target: GeocentricPosition) -> LocalVector<O> {
        let t_vec3 = target.as_metres();
        // delta in 'Earth' frame.
        let de = t_vec3 - self.origin;
        let d = de * self.inv_rm;
        LocalVector::from_vec3_metres(d)
    }

    /// Computes the global geocentric position resulting from displacing
    /// the frame's origin by the given local vector.
    pub fn destination_position(&self, vector: LocalVector<O>) -> GeocentricPosition {
        let c = vector.as_metres() * self.dir_rm;
        let v = self.origin + c;
        GeocentricPosition::from_vec3_metres(v)
    }

    /// m1 * m2 aligning to z-down if required.
    fn rotate(&self, m1: Mat33, m2: Mat33) -> Mat33 {
        if O::is_z_up() {
            m1 * m2 * align_to_z_down_matrix()
        } else {
            m1 * m2
        }
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
/// `v_A = R_AB * v_B`
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
/// `v_A = R_AB * v_B`
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
        ellipsoidal::Ellipsoid,
        local::{r2xyz, r2zyx, BodyFrame, BodyVector, EnuFrame, NedFrame, WanderAzimuthFrame},
        positions::assert_geod_eq_d7_mm,
        Angle, GeodeticPosition, LatLong, Length, Mat33, NVector, PositionVector, Surface, Vec3,
    };

    #[test]
    fn from_geodetic_and_from_geocentric() {
        let s = Ellipsoid::WGS84;

        let o_geod = GeodeticPosition::new(
            NVector::from_lat_long_degrees(54.0, 154.0),
            Length::from_metres(10_000.0),
        );
        let o_geoc = s.geodetic_to_geocentric_position(o_geod);

        assert_eq!(
            EnuFrame::from_geocentric(o_geoc, s),
            EnuFrame::from_geodetic(o_geod, s)
        );

        assert_eq!(
            NedFrame::from_geocentric(o_geoc, s),
            NedFrame::from_geodetic(o_geod, s)
        );

        assert_eq!(
            BodyFrame::from_geocentric(
                Angle::from_degrees(10.0),
                Angle::from_degrees(20.0),
                Angle::from_degrees(30.0),
                o_geoc,
                s
            ),
            BodyFrame::from_geodetic(
                Angle::from_degrees(10.0),
                Angle::from_degrees(20.0),
                Angle::from_degrees(30.0),
                o_geod,
                s
            )
        );

        assert_eq!(
            WanderAzimuthFrame::from_geocentric(Angle::from_degrees(10.0), o_geoc, s),
            WanderAzimuthFrame::from_geodetic(Angle::from_degrees(10.0), o_geod, s)
        )
    }

    #[test]
    fn looking_at_geodetic_and_geocentric() {
        let s: Ellipsoid = Ellipsoid::WGS84;

        let o_geod = GeodeticPosition::new(
            NVector::from_lat_long_degrees(54.0, 154.0),
            Length::from_metres(10_000.0),
        );
        let o_geoc = s.geodetic_to_geocentric_position(o_geod);

        let t_geod = GeodeticPosition::new(
            NVector::from_lat_long_degrees(55.0, 155.0),
            Length::from_metres(11_000.0),
        );
        let t_geoc = s.geodetic_to_geocentric_position(t_geod);

        let roll = Angle::from_degrees(1.0);

        assert_eq!(
            BodyFrame::looking_at_geodetic(o_geod, t_geod, roll, s),
            BodyFrame::looking_at_geocentric(o_geoc, t_geoc, roll, s)
        )
    }

    // local_vector_to

    // see https://github.com/pbrod/nvector/blob/bf1cf5e1e210b74a57ea4bb2c277b388308bdba9/src/nvector/tests/test_frames.py

    #[test]
    fn local_vector_to_w_in_moving_frame_east() {
        let s = Ellipsoid::WGS84;

        let ship_position_0 =
            GeodeticPosition::new(LatLong::from_degrees(1.0, 2.0).to_nvector(), Length::ZERO);
        let ship_position_1 =
            GeodeticPosition::new(LatLong::from_degrees(1.0, 2.005).to_nvector(), Length::ZERO);
        let sensor_position = s.geodetic_to_geocentric_position(GeodeticPosition::new(
            LatLong::from_degrees(1.000090437, 2.0025).to_nvector(),
            Length::ZERO,
        ));

        let f0 = WanderAzimuthFrame::from_geodetic(Angle::from_degrees(90.0), ship_position_0, s);
        let local_0 = f0.local_vector_to(sensor_position).round_mm();

        assert_eq!(Length::from_metres(278.257), local_0.x());
        assert_eq!(Length::from_metres(-10.0), local_0.y());
        assert_eq!(Length::ZERO, local_0.z().round_m());
        assert_eq!(358.0, local_0.bearing().as_degrees().round());

        let f1 = WanderAzimuthFrame::from_geodetic(Angle::from_degrees(90.0), ship_position_1, s);

        let local_1 = f1.local_vector_to(sensor_position).round_mm();

        assert_eq!(Length::from_metres(-278.257), local_1.x());
        assert_eq!(Length::from_metres(-10.0), local_1.y());
        assert_eq!(Length::ZERO, local_1.z().round_m());
        assert_eq!(182.0, local_1.bearing().as_degrees().round());
    }

    #[test]
    fn local_vector_to_n_in_moving_frame_east() {
        let s = Ellipsoid::WGS84;

        let ship_position_0 =
            GeodeticPosition::new(LatLong::from_degrees(1.0, 2.0).to_nvector(), Length::ZERO);
        let ship_position_1 =
            GeodeticPosition::new(LatLong::from_degrees(1.0, 2.005).to_nvector(), Length::ZERO);
        let sensor_position = s.geodetic_to_geocentric_position(GeodeticPosition::new(
            LatLong::from_degrees(1.0, 2.0025).to_nvector(),
            Length::ZERO,
        ));

        let f0 = NedFrame::from_geodetic(ship_position_0, s);
        let local_0 = f0.local_vector_to(sensor_position).round_mm();

        assert_eq!(Length::ZERO, local_0.x());
        assert_eq!(Length::from_metres(278.257), local_0.y());
        assert_eq!(Length::ZERO, local_0.z().round_m());
        assert_eq!(90.0, local_0.bearing().as_degrees());

        let f1 = NedFrame::from_geodetic(ship_position_1, s);
        let local_1 = f1.local_vector_to(sensor_position).round_mm();

        assert_eq!(Length::ZERO, local_1.x());
        assert_eq!(Length::from_metres(-278.257), local_1.y());
        assert_eq!(Length::ZERO, local_1.z().round_m());
        assert_eq!(270.0, local_1.bearing().as_degrees());
    }

    // see https://au.mathworks.com/help/map/ref/geodetic2ned.html
    #[test]
    fn local_vector_to_ned() {
        let s = Ellipsoid::WGS84;

        let origin = GeodeticPosition::new(
            NVector::from_lat_long_degrees(44.532, -72.782),
            Length::from_metres(1699.0),
        );
        let point = s.geodetic_to_geocentric_position(GeodeticPosition::new(
            NVector::from_lat_long_degrees(44.544, -72.814),
            Length::from_metres(1340.0),
        ));

        let ned = NedFrame::from_geodetic(origin, s);

        let local = ned.local_vector_to(point);

        assert_eq!(Length::from_metres(1334.252), local.x().round_mm());
        assert_eq!(Length::from_metres(-2543.564), local.y().round_mm());
        assert_eq!(Length::from_metres(359.646), local.z().round_mm());
        assert_eq!(Angle::from_degrees(297.6796990), local.bearing().round_d7());
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
    fn local_vector_to_enu() {
        let s = Ellipsoid::WGS84;

        let origin = s.geodetic_to_geocentric_position(GeodeticPosition::new(
            NVector::from_lat_long_degrees(46.017, 7.750),
            Length::from_metres(1673.0),
        ));
        let point = s.geodetic_to_geocentric_position(GeodeticPosition::new(
            NVector::from_lat_long_degrees(45.976, 7.658),
            Length::from_metres(4531.0),
        ));

        let enu = EnuFrame::from_geocentric(origin, s);

        let local = enu.local_vector_to(point);

        assert_eq!(Length::from_metres(-7134.757), local.x().round_mm());
        assert_eq!(Length::from_metres(-4556.322), local.y().round_mm());
        assert_eq!(Length::from_metres(2852.39), local.z().round_mm());
        assert_eq!(Angle::from_degrees(237.4373247), local.bearing().round_d7());
        assert_eq!(
            Angle::from_degrees(18.6208639),
            local.elevation().round_d7()
        );
    }

    #[test]
    fn destination_position_enu() {
        let s = Ellipsoid::WGS84;

        let origin = GeodeticPosition::new(
            NVector::from_lat_long_degrees(46.017, 7.750),
            Length::from_metres(1673.0),
        );
        let point = s
            .geodetic_to_geocentric_position(GeodeticPosition::new(
                NVector::from_lat_long_degrees(45.976, 7.658),
                Length::from_metres(4531.0),
            ))
            .round_mm();

        let enu = EnuFrame::from_geodetic(origin, Ellipsoid::WGS84);
        let local_enu = enu.local_vector_to(point);
        assert_eq!(point, enu.destination_position(local_enu).round_mm());
    }

    #[test]
    fn transitiviy_enu() {
        let s = Ellipsoid::WGS84;

        let point_a = GeodeticPosition::new(
            NVector::from_lat_long_degrees(1.0, 2.0),
            Length::from_metres(-3.0),
        );
        let point_b = s
            .geodetic_to_geocentric_position(GeodeticPosition::new(
                NVector::from_lat_long_degrees(4.0, 5.0),
                Length::from_metres(-6.0),
            ))
            .round_mm();

        let enu = EnuFrame::from_geodetic(point_a, s);
        assert_eq!(
            point_b,
            enu.destination_position(enu.local_vector_to(point_b))
                .round_mm(),
        )
    }

    #[test]
    fn transitiviy_ned() {
        let s = Ellipsoid::WGS84;

        let point_a = GeodeticPosition::new(
            NVector::from_lat_long_degrees(1.0, 2.0),
            Length::from_metres(-3.0),
        );
        let point_b = s
            .geodetic_to_geocentric_position(GeodeticPosition::new(
                NVector::from_lat_long_degrees(4.0, 5.0),
                Length::from_metres(-6.0),
            ))
            .round_mm();

        let ned = NedFrame::from_geodetic(point_a, s);
        assert_eq!(
            point_b,
            ned.destination_position(ned.local_vector_to(point_b))
                .round_mm(),
        )
    }

    #[test]
    fn transitiviy_body() {
        let s = Ellipsoid::WGS84;

        let point_a = GeodeticPosition::new(
            NVector::from_lat_long_degrees(1.0, 2.0),
            Length::from_metres(-3.0),
        );
        let point_b = s
            .geodetic_to_geocentric_position(GeodeticPosition::new(
                NVector::from_lat_long_degrees(4.0, 5.0),
                Length::from_metres(-6.0),
            ))
            .round_mm();

        let body = BodyFrame::from_geodetic(
            Angle::from_degrees(45.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(5.0),
            point_a,
            s,
        );
        assert_eq!(
            point_b,
            body.destination_position(body.local_vector_to(point_b))
                .round_mm(),
        )
    }

    #[test]
    fn transitiviy_wander_azimuth() {
        let s = Ellipsoid::WGS84;

        let point_a = GeodeticPosition::new(
            NVector::from_lat_long_degrees(1.0, 2.0),
            Length::from_metres(-3.0),
        );
        let point_b = s
            .geodetic_to_geocentric_position(GeodeticPosition::new(
                NVector::from_lat_long_degrees(4.0, 5.0),
                Length::from_metres(-6.0),
            ))
            .round_mm();

        let wa =
            WanderAzimuthFrame::from_geodetic(Angle::from_degrees(45.0), point_a, Ellipsoid::WGS84);
        assert_eq!(
            point_b,
            wa.destination_position(wa.local_vector_to(point_b))
                .round_mm(),
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
    fn rotate_around_z_ned_equals_wander_azimuth() {
        let s = Ellipsoid::WGS84;

        let origin = GeodeticPosition::new(
            NVector::from_lat_long_degrees(52.2, 4.9),
            Length::from_metres(100.0),
        );
        let wander_angle = Angle::from_degrees(33.5);
        let ned_frame = NedFrame::from_geodetic(origin, s);
        let rotated_frame = ned_frame.rotate_around_z(wander_angle);

        let wa_frame = WanderAzimuthFrame::from_geodetic(wander_angle, origin, s);

        let target_pos = s.geodetic_to_geocentric_position(GeodeticPosition::new(
            NVector::from_lat_long_degrees(52.25, 4.95),
            Length::from_metres(500.0),
        ));

        let local_from_rotated = rotated_frame.local_vector_to(target_pos);
        let local_from_wa = wa_frame.local_vector_to(target_pos);

        assert_eq!(
            local_from_wa.x().round_mm(),
            local_from_rotated.x().round_mm()
        );
        assert_eq!(
            local_from_wa.y().round_mm(),
            local_from_rotated.y().round_mm()
        );
        assert_eq!(
            local_from_wa.z().round_mm(),
            local_from_rotated.z().round_mm()
        );
    }

    #[test]
    fn rotate_around_z_enu_to_wander_azimuth() {
        let s = Ellipsoid::WGS84;

        let origin = GeodeticPosition::new(
            NVector::from_lat_long_degrees(-37.86, 144.97),
            Length::from_metres(10.0),
        );
        let enu_frame = EnuFrame::from_geodetic(origin, s);
        // Rotating ENU by 0° aligns ENU (Z-up) directly to NED (Z-down)
        let wa_frame_0 = enu_frame.rotate_around_z(Angle::ZERO);
        let ned_frame = NedFrame::from_geodetic(origin, s);

        let target_pos = s.geodetic_to_geocentric_position(GeodeticPosition::new(
            NVector::from_lat_long_degrees(-37.85, 144.98),
            Length::from_metres(50.0),
        ));

        let local_wa = wa_frame_0.local_vector_to(target_pos);
        let local_ned = ned_frame.local_vector_to(target_pos);

        assert_eq!(local_ned.x().round_mm(), local_wa.x().round_mm());
        assert_eq!(local_ned.y().round_mm(), local_wa.y().round_mm());
        assert_eq!(local_ned.z().round_mm(), local_wa.z().round_mm());
    }

    #[test]
    fn rotate_around_z_transitivity() {
        let s = Ellipsoid::WGS84;

        let point_a = GeodeticPosition::new(
            NVector::from_lat_long_degrees(48.8584, 2.2945),
            Length::from_metres(300.0),
        );
        let point_b = GeodeticPosition::new(
            NVector::from_lat_long_degrees(48.8600, 2.3000),
            Length::from_metres(150.0),
        );

        let enu = EnuFrame::from_geodetic(point_a, s);
        let wa = enu.rotate_around_z(Angle::from_degrees(120.0));

        let local = wa.local_vector_to(s.geodetic_to_geocentric_position(point_b));
        let actual_b = s.geocentric_to_geodetic_position(wa.destination_position(local));
        assert_geod_eq_d7_mm(point_b, actual_b);
    }

    #[test]
    fn sensor_observation_to_geocentric_position() {
        let s = Ellipsoid::WGS84;

        let ac_pos: GeodeticPosition = GeodeticPosition::new(
            NVector::from_lat_long_degrees(54.0, 154.0),
            Length::from_feet(35000.0),
        );
        let yaw = Angle::from_degrees(45.0);
        let pitch = Angle::from_degrees(2.0);
        let roll = Angle::from_degrees(-1.0);
        let ac_frame = BodyFrame::from_geodetic(yaw, pitch, roll, ac_pos, s);

        // Sensor fixed offset relative to aircraft CG (x: +2.5m forward, y: 0.0m, z: +0.8m down)
        let sensor_offset_body = BodyVector::new(
            Length::from_metres(2.5),
            Length::from_metres(0.0),
            Length::from_metres(0.8),
        );
        let sensor_frame =
            ac_frame.transform(sensor_offset_body, Angle::ZERO, Angle::ZERO, Angle::ZERO);

        // Sensor observes a target at relative azimuth 30°, elevation -15° (downward), range 5000m
        let rel_azimuth = Angle::from_degrees(30.0);
        let rel_elevation = Angle::from_degrees(-15.0);
        let slant_range = Length::from_metres(5000.0);

        let raw_observation = BodyVector::from_aer(rel_azimuth, rel_elevation, slant_range);

        // Conversion of observation to geodetic position
        let expected = s
            .geodetic_to_geocentric_position(GeodeticPosition::new(
                NVector::from_lat_long_degrees(54.01132811615137, 154.07176275677563),
                Length::from_feet(31378.3653906617),
            ))
            .round_mm();
        assert_eq!(
            expected,
            sensor_frame
                .destination_position(raw_observation)
                .round_mm()
        )
    }

    #[test]
    fn pan_tilt_gimbal() {
        let s = Ellipsoid::WGS84;

        let ac_pos: GeodeticPosition = GeodeticPosition::new(
            NVector::from_lat_long_degrees(54.0, 154.0),
            Length::from_feet(35000.0),
        );
        let yaw = Angle::from_degrees(45.0);
        let pitch = Angle::from_degrees(2.0);
        let roll = Angle::from_degrees(-1.0);
        let ac_frame = BodyFrame::from_geodetic(yaw, pitch, roll, ac_pos, s);

        // Gimbal relative angles: pan 45° right, tilt -15° down, roll 0°
        let gimbal_pan = Angle::from_degrees(45.0);
        let gimbal_tilt = Angle::from_degrees(-15.0);
        let gimbal_roll = Angle::from_degrees(0.0);

        // sensor frame by rotating the aircraft frame by gimbal angles
        let sensor_frame =
            ac_frame.transform(BodyVector::ZERO, gimbal_pan, gimbal_tilt, gimbal_roll);

        // Sensor observes a target at relative azimuth 30°, elevation -15° (downward), range 5000m
        let rel_azimuth = Angle::from_degrees(30.0);
        let rel_elevation = Angle::from_degrees(-15.0);
        let slant_range = Length::from_metres(5000.0);

        let raw_observation = BodyVector::from_aer(rel_azimuth, rel_elevation, slant_range);

        // Conversion of observation to geodetic position
        let expected = s
            .geodetic_to_geocentric_position(GeodeticPosition::new(
                NVector::from_lat_long_degrees(53.97856139187561, 154.05767069112713),
                Length::from_feet(27710.407954738614),
            ))
            .round_mm();
        assert_eq!(
            expected,
            sensor_frame
                .destination_position(raw_observation)
                .round_mm()
        )
    }

    #[test]
    fn looking_at() {
        let s: Ellipsoid = Ellipsoid::WGS84;

        let ac_pos: GeodeticPosition = GeodeticPosition::new(
            NVector::from_lat_long_degrees(54.0, 154.0),
            Length::from_feet(35000.0),
        );
        let target_pos = GeodeticPosition::new(
            NVector::from_lat_long_degrees(54.1, 154.1),
            Length::from_feet(36000.0),
        );

        let roll = Angle::from_degrees(2.0);
        let body = BodyFrame::looking_at_geodetic(ac_pos, target_pos, roll, s);

        let ned = NedFrame::from_geodetic(ac_pos, s);
        let d = ned.local_vector_to(s.geodetic_to_geocentric_position(target_pos));

        let yaw = d.bearing();
        let pitch = d.elevation();
        let expected = BodyFrame::from_geodetic(yaw, pitch, roll, ac_pos, s);
        assert_eq!(expected, body);
    }
}
