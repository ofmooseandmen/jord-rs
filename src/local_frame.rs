use crate::{
    surface::Surface, Angle, Cartesian3DVector, GeocentricPos, GeodeticPos, LatLong, Length, Mat33,
    Vec3,
};

#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct LocalPositionVector {
    x: Length,
    y: Length,
    z: Length,
}

impl LocalPositionVector {
    pub fn azimuth(&self) -> Angle {
        Angle::from_radians(self.y().as_metres().atan2(self.x().as_metres())).normalised()
    }

    // TODO(CL): new, from_bearing_and_slant_range, bearing, elevation, slant_range
}

impl Cartesian3DVector for LocalPositionVector {
    fn new(x: Length, y: Length, z: Length) -> Self {
        Self { x, y, z }
    }

    fn x(&self) -> Length {
        self.x
    }

    fn y(&self) -> Length {
        self.y
    }

    fn z(&self) -> Length {
        self.z
    }
}

#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct LocalFrame<S> {
    origin: Vec3,
    dir_rm: Mat33,
    inv_rm: Mat33,
    surface: S,
}

impl<S> LocalFrame<S>
where
    S: Surface,
{
    pub fn enu(origin: GeodeticPos, surface: S) -> Self {
        let vo = origin.horizontal_position().as_vec3();
        // up - just the n-vector.
        let ru = vo;
        // east - pointing perpendicular to the plane.
        let re = Vec3::UNIT_Z.cross_prod(vo).unit();
        // north - by right hand rule.
        let rn = ru.cross_prod(re);

        let inv_rm = Mat33::new(re, rn, ru);

        Self {
            origin: surface.geodetic_to_geocentric(origin).as_metres(),
            dir_rm: inv_rm.transpose(),
            inv_rm,
            surface,
        }
    }

    pub fn ned(origin: GeodeticPos, surface: S) -> Self {
        let vo = origin.horizontal_position().as_vec3();
        //  down (pointing opposite to n-vector).
        let rd = -1.0 * vo;
        // east (pointing perpendicular to the plane)
        let re = Vec3::UNIT_Z.cross_prod(vo).unit();
        // north (by right hand rule)
        let rn = re.cross_prod(rd);

        let inv_rm = Mat33::new(rn, re, rd);

        Self {
            origin: surface.geodetic_to_geocentric(origin).as_metres(),
            dir_rm: inv_rm.transpose(),
            inv_rm,
            surface,
        }
    }

    pub fn body(yaw: Angle, pitch: Angle, roll: Angle, origin: GeodeticPos, surface: S) -> Self {
        let r_nb = zyx2r(yaw, pitch, roll);
        let r_en = Self::ned(origin, surface).dir_rm;
        // closest frames cancel: N.
        let dir_rm = r_en * r_nb;
        Self {
            origin: surface.geodetic_to_geocentric(origin).as_metres(),
            dir_rm,
            inv_rm: dir_rm.transpose(),
            surface,
        }
    }

    pub fn local_level(wander_azimuth: Angle, origin: GeodeticPos, surface: S) -> Self {
        let ll = LatLong::from_nvector(origin.horizontal_position());
        let r = xyz2r(ll.longitude(), -ll.latitude(), wander_azimuth);
        let r_ee = Mat33::new(Vec3::NEG_UNIT_Z, Vec3::UNIT_Y, Vec3::UNIT_X);
        let dir_rm = r_ee * r;
        Self {
            origin: surface.geodetic_to_geocentric(origin).as_metres(),
            dir_rm,
            inv_rm: dir_rm.transpose(),
            surface,
        }
    }

    pub fn to_local_pos(&self, other: GeodeticPos) -> LocalPositionVector {
        let og = self.surface.geodetic_to_geocentric(other).as_metres();
        // delta in 'Earth' frame.
        let de = og - self.origin;
        let d = de * self.inv_rm;
        LocalPositionVector::from_metres(d)
    }

    pub fn to_geodetic_pos(&self, delta: LocalPositionVector) -> GeodeticPos {
        let c = delta.as_metres() * self.dir_rm;
        let v = self.origin + c;
        let g = GeocentricPos::from_metres(v);
        self.surface.geocentric_to_geodetic(g)
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

/// | Angles about new axes in the xyz-order from a rotation matrix.
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
    let (a, b, c) = r2xyz(m.transpose());
    (-a, -b, -c)
}

/// rotation matrix (direction cosine matrix) from 3 angles about new axes in the zyx-order.
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
pub fn zyx2r(x: Angle, y: Angle, z: Angle) -> Mat33 {
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

    // to_local

    use crate::{
        ellipsoidal::Ellipsoid, Angle, Cartesian3DVector, GeodeticPos, LatLong, Length, LocalFrame,
    };

    // see https://github.com/pbrod/nvector/blob/bf1cf5e1e210b74a57ea4bb2c277b388308bdba9/src/nvector/tests/test_frames.py

    #[test]
    fn to_local_pos_w_in_moving_frame_east() {
        let ship_position_0 =
            GeodeticPos::new(LatLong::from_degrees(1.0, 2.0).to_nvector(), Length::ZERO);
        let ship_position_1 =
            GeodeticPos::new(LatLong::from_degrees(1.0, 2.005).to_nvector(), Length::ZERO);
        let sensor_position = GeodeticPos::new(
            LatLong::from_degrees(1.000090437, 2.0025).to_nvector(),
            Length::ZERO,
        );

        let f0: LocalFrame<Ellipsoid> =
            LocalFrame::local_level(Angle::from_degrees(90.0), ship_position_0, Ellipsoid::WGS84);
        let local_0 = f0.to_local_pos(sensor_position).round_mm();

        assert_eq!(Length::from_metres(278.257), local_0.x());
        assert_eq!(Length::from_metres(-10.0), local_0.y());
        assert_eq!(Length::ZERO, local_0.z().round_m());
        assert_eq!(358.0, local_0.azimuth().as_degrees().round());

        let f1: LocalFrame<Ellipsoid> =
            LocalFrame::local_level(Angle::from_degrees(90.0), ship_position_1, Ellipsoid::WGS84);

        let local_1 = f1.to_local_pos(sensor_position).round_mm();

        assert_eq!(Length::from_metres(-278.257), local_1.x());
        assert_eq!(Length::from_metres(-10.0), local_1.y());
        assert_eq!(Length::ZERO, local_1.z().round_m());
        assert_eq!(182.0, local_1.azimuth().as_degrees().round());
    }

    #[test]
    fn to_local_pos_n_in_moving_frame_east() {
        let ship_position_0 =
            GeodeticPos::new(LatLong::from_degrees(1.0, 2.0).to_nvector(), Length::ZERO);
        let ship_position_1 =
            GeodeticPos::new(LatLong::from_degrees(1.0, 2.005).to_nvector(), Length::ZERO);
        let sensor_position = GeodeticPos::new(
            LatLong::from_degrees(1.0, 2.0025).to_nvector(),
            Length::ZERO,
        );

        let f0: LocalFrame<Ellipsoid> = LocalFrame::ned(ship_position_0, Ellipsoid::WGS84);
        let local_0 = f0.to_local_pos(sensor_position).round_mm();

        assert_eq!(Length::ZERO, local_0.x());
        assert_eq!(Length::from_metres(278.214), local_0.y());
        assert_eq!(Length::ZERO, local_0.z().round_m());
        assert_eq!(90.0, local_0.azimuth().as_degrees());

        let f1: LocalFrame<Ellipsoid> = LocalFrame::ned(ship_position_1, Ellipsoid::WGS84);
        let local_1 = f1.to_local_pos(sensor_position).round_mm();

        assert_eq!(Length::ZERO, local_1.x());
        assert_eq!(Length::from_metres(-278.214), local_1.y());
        assert_eq!(Length::ZERO, local_1.z().round_m());
        assert_eq!(270.0, local_1.azimuth().as_degrees());
    }

    // TODO(CL): more tests
}
