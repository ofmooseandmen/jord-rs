use crate::{
    surface::Surface, Angle, Cartesian3DVector, GeocentricPos, GeodeticPos, LatLong, Length, Mat33,
    Vec3,
};

#[derive(PartialEq, Clone, Copy, Debug, Default)]
enum Orientation {
    // x = north (or forward), y = east (or right), z = down.
    #[default]
    NED,
    // x = east, y = north, z = up.
    ENU,
}

#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct LocalPositionVector {
    x: Length,
    y: Length,
    z: Length,
    o: Orientation,
}

impl LocalPositionVector {
    fn new(x: Length, y: Length, z: Length, o: Orientation) -> Self {
        Self { x, y, z, o }
    }

    fn from_metres(v: Vec3, o: Orientation) -> Self {
        Self::new(
            Length::from_metres(v.x()),
            Length::from_metres(v.y()),
            Length::from_metres(v.z()),
            o,
        )
    }

    pub fn azimuth(&self) -> Angle {
        let (e, n) = match self.o {
            Orientation::NED => (self.y(), self.x()),
            Orientation::ENU => (self.x(), self.y()),
        };
        Angle::from_radians(e.as_metres().atan2(n.as_metres())).normalised()
    }

    pub fn elevation(&self) -> Angle {
        Angle::from_radians((self.z() / self.length()).asin())
    }

    pub fn length(&self) -> Length {
        Length::from_metres(self.as_metres().norm())
    }
}

impl Cartesian3DVector for LocalPositionVector {
    fn x(&self) -> Length {
        self.x
    }

    fn y(&self) -> Length {
        self.y
    }

    fn z(&self) -> Length {
        self.z
    }

    fn round<F>(&self, round: F) -> Self
    where
        F: Fn(Length) -> Length,
    {
        Self::new(round(self.x()), round(self.y()), round(self.z()), self.o)
    }
}

#[derive(PartialEq, Clone, Copy, Debug, Default)]
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
            o: Orientation::ENU,
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
            o: Orientation::NED,
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
            o: Orientation::NED,
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
            o: Orientation::NED,
        }
    }

    pub fn to_local_pos(&self, p: GeodeticPos) -> LocalPositionVector {
        let og = self.surface.geodetic_to_geocentric(p).as_metres();
        // delta in 'Earth' frame.
        let de = og - self.origin;
        let d = de * self.inv_rm;
        LocalPositionVector::from_metres(d, self.o)
    }

    pub fn to_geodetic_pos(&self, p: LocalPositionVector) -> GeodeticPos {
        let c = p.as_metres() * self.dir_rm;
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
    let (a, b, c) = r2xyz(m.transpose());
    (-a, -b, -c)
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

    use crate::{
        ellipsoidal::Ellipsoid, positions::assert_geod_eq_d7_mm, Angle, Cartesian3DVector,
        GeodeticPos, LatLong, Length, LocalFrame, NVector,
    };

    // see https://github.com/pbrod/nvector/blob/bf1cf5e1e210b74a57ea4bb2c277b388308bdba9/src/nvector/tests/test_frames.py

    // to_local

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
        assert_eq!(Length::from_metres(278.257), local_0.y());
        assert_eq!(Length::ZERO, local_0.z().round_m());
        assert_eq!(90.0, local_0.azimuth().as_degrees());

        let f1: LocalFrame<Ellipsoid> = LocalFrame::ned(ship_position_1, Ellipsoid::WGS84);
        let local_1 = f1.to_local_pos(sensor_position).round_mm();

        assert_eq!(Length::ZERO, local_1.x());
        assert_eq!(Length::from_metres(-278.257), local_1.y());
        assert_eq!(Length::ZERO, local_1.z().round_m());
        assert_eq!(270.0, local_1.azimuth().as_degrees());
    }

    #[test]
    fn to_local_pos_ned() {
        let point_a = GeodeticPos::new(
            NVector::from_lat_long_degrees(1.0, 2.0),
            Length::from_metres(-3.0),
        );
        let point_b = GeodeticPos::new(
            NVector::from_lat_long_degrees(4.0, 5.0),
            Length::from_metres(-6.0),
        );

        let ned = LocalFrame::ned(point_a, Ellipsoid::WGS84);

        let local = ned.to_local_pos(point_b);

        assert_eq!(Length::from_metres(331730.235), local.x().round_mm());
        assert_eq!(Length::from_metres(332997.875), local.y().round_mm());
        assert_eq!(Length::from_metres(17404.271), local.z().round_mm());
        assert_eq!(Angle::from_degrees(45.1092632), local.azimuth().round_d7());
        assert_eq!(Angle::from_degrees(2.1205586), local.elevation().round_d7());
    }

    #[test]
    fn to_local_pos_enu() {
        let point_a = GeodeticPos::new(
            NVector::from_lat_long_degrees(1.0, 2.0),
            Length::from_metres(-3.0),
        );
        let point_b = GeodeticPos::new(
            NVector::from_lat_long_degrees(4.0, 5.0),
            Length::from_metres(-6.0),
        );

        let enu = LocalFrame::enu(point_a, Ellipsoid::WGS84);

        let local = enu.to_local_pos(point_b);

        assert_eq!(Length::from_metres(332997.875), local.x().round_mm());
        assert_eq!(Length::from_metres(331730.235), local.y().round_mm());
        assert_eq!(-Length::from_metres(17404.271), local.z().round_mm());
        assert_eq!(Angle::from_degrees(45.1092632), local.azimuth().round_d7());
        assert_eq!(
            -Angle::from_degrees(2.1205586),
            local.elevation().round_d7()
        );
    }

    #[test]
    fn transitiviy_enu() {
        let point_a = GeodeticPos::new(
            NVector::from_lat_long_degrees(1.0, 2.0),
            Length::from_metres(-3.0),
        );
        let point_b = GeodeticPos::new(
            NVector::from_lat_long_degrees(4.0, 5.0),
            Length::from_metres(-6.0),
        );

        let enu = LocalFrame::enu(point_a, Ellipsoid::WGS84);
        assert_geod_eq_d7_mm(point_b, enu.to_geodetic_pos(enu.to_local_pos(point_b)))
    }

    #[test]
    fn transitiviy_ned() {
        let point_a = GeodeticPos::new(
            NVector::from_lat_long_degrees(1.0, 2.0),
            Length::from_metres(-3.0),
        );
        let point_b = GeodeticPos::new(
            NVector::from_lat_long_degrees(4.0, 5.0),
            Length::from_metres(-6.0),
        );

        let ned = LocalFrame::ned(point_a, Ellipsoid::WGS84);
        assert_geod_eq_d7_mm(point_b, ned.to_geodetic_pos(ned.to_local_pos(point_b)))
    }

    #[test]
    fn transitiviy_body() {
        let point_a = GeodeticPos::new(
            NVector::from_lat_long_degrees(1.0, 2.0),
            Length::from_metres(-3.0),
        );
        let point_b = GeodeticPos::new(
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
        assert_geod_eq_d7_mm(point_b, body.to_geodetic_pos(body.to_local_pos(point_b)))
    }

    #[test]
    fn transitiviy_local_level() {
        let point_a = GeodeticPos::new(
            NVector::from_lat_long_degrees(1.0, 2.0),
            Length::from_metres(-3.0),
        );
        let point_b = GeodeticPos::new(
            NVector::from_lat_long_degrees(4.0, 5.0),
            Length::from_metres(-6.0),
        );

        let local_level =
            LocalFrame::local_level(Angle::from_degrees(45.0), point_a, Ellipsoid::WGS84);
        assert_geod_eq_d7_mm(
            point_b,
            local_level.to_geodetic_pos(local_level.to_local_pos(point_b)),
        )
    }
}
