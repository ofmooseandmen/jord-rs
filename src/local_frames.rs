use crate::{
    nvector_to_lat_long, xyz2r, zyx2r, Angle, GeocentricPos, GeodeticPos, Length, LongitudeRange,
    Mat33, Model, Vec3,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Delta(Length, Length, Length);

impl Delta {
    pub fn new(x: Length, y: Length, z: Length) -> Delta {
        Delta(x, y, z)
    }

    pub fn from_metres(x: f64, y: f64, z: f64) -> Delta {
        Delta::new(
            Length::from_metres(x),
            Length::from_metres(y),
            Length::from_metres(z),
        )
    }

    pub fn from_vec3_metres(v: Vec3) -> Delta {
        Delta::from_metres(v.x(), v.y(), v.z())
    }

    pub fn as_metres(&self) -> Vec3 {
        Vec3::new(self.x().metres(), self.y().metres(), self.z().metres())
    }

    // FIXME: from_length_azimuth_elevation

    pub fn x(&self) -> Length {
        self.0
    }

    pub fn y(&self) -> Length {
        self.1
    }

    pub fn z(&self) -> Length {
        self.2
    }

    pub fn length(&self) -> Length {
        Length::from_metres(self.as_metres().norm())
    }

    pub fn azimuth(&self) -> Angle {
        let degs = self.y().metres().atan2(self.x().metres()).to_degrees();
        Angle::from_decimal_degrees((degs + 360.0) % 360.0)
    }

    pub fn elevation(&self) -> Angle {
        let degs = (self.z() / self.length()).asin().to_degrees();
        Angle::from_decimal_degrees(degs)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyOrientation(Angle, Angle, Angle);
impl BodyOrientation {
    pub fn new(yaw: Angle, pitch: Angle, roll: Angle) -> Self {
        BodyOrientation(yaw, pitch, roll)
    }

    pub fn from_decimal_degrees(yaw: f64, pitch: f64, roll: f64) -> Self {
        BodyOrientation::new(
            Angle::from_decimal_degrees(yaw),
            Angle::from_decimal_degrees(pitch),
            Angle::from_decimal_degrees(roll),
        )
    }

    pub fn yaw(&self) -> Angle {
        self.0
    }

    pub fn pitch(&self) -> Angle {
        self.1
    }

    pub fn roll(&self) -> Angle {
        self.2
    }
}

impl<M: Model> GeodeticPos<M> {
    pub fn delta_b_to(&self, orientation: BodyOrientation, to: GeodeticPos<M>) -> Delta {
        let rotation = n_e_and_ypr2_r_eb(self.nvector(), orientation);
        self.delta_to(to, rotation)
    }

    pub fn delta_n_to(&self, to: GeodeticPos<M>) -> Delta {
        self.delta_to(to, n_e2_r_en(self.nvector()))
    }

    pub fn delta_w_to(&self, wander_azimuth: Angle, to: GeodeticPos<M>) -> Delta {
        let rotation = n_e_and_wa2_r_el(self.nvector(), wander_azimuth);
        self.delta_to(to, rotation)
    }

    pub fn destination_pos_from_delta_b(
        &self,
        orientation: BodyOrientation,
        delta: Delta,
    ) -> GeodeticPos<M> {
        let rotation = n_e_and_ypr2_r_eb(self.nvector(), orientation);
        self.destination_pos(delta, rotation)
    }

    pub fn destination_pos_from_delta_n(&self, delta: Delta) -> GeodeticPos<M> {
        self.destination_pos(delta, n_e2_r_en(self.nvector()))
    }

    pub fn destination_pos_from_delta_w(
        &self,
        wander_azimuth: Angle,
        delta: Delta,
    ) -> GeodeticPos<M> {
        let rotation = n_e_and_wa2_r_el(self.nvector(), wander_azimuth);
        self.destination_pos(delta, rotation)
    }

    fn delta_to(&self, to: GeodeticPos<M>, r_ef: Mat33) -> Delta {
        let po = self.to_geocentric().as_metres();
        let pt = to.to_geocentric().as_metres();
        // delta in Earth Frame.
        let delta_e = pt - po;
        let rm = r_ef.transpose();
        let delta_f = rm * delta_e;
        Delta::from_vec3_metres(delta_f)
    }

    fn destination_pos(&self, delta: Delta, r_ef: Mat33) -> GeodeticPos<M> {
        let po = self.to_geocentric().as_metres();
        let c = r_ef * delta.as_metres();
        let v = po + c;
        GeocentricPos::from_vec3_metres(v, self.model()).to_geodetic()
    }
}

// FIXME: explain why not r_ee (or use it)
pub fn n_e2_r_en(n_e: Vec3) -> Mat33 {
    // down (pointing opposite to n-vector)
    let nz_e = -1.0 * n_e;
    // east (pointing perpendicular to the plane)
    let ny_e_direction = Vec3::unit_z().cross(n_e);
    let ny_e;
    if ny_e_direction == Vec3::zero() {
        // selected y-axis direction
        ny_e = Vec3::unit_y();
    } else {
        // outside poles
        ny_e = ny_e_direction.unit();
    }
    // x-axis of N (North): found by right hand rule
    let nx_e = ny_e.cross(nz_e);
    Mat33::new(nx_e, ny_e, nz_e).transpose()
}

pub fn n_e_and_wa2_r_el(n_e: Vec3, wander_azimuth: Angle) -> Mat33 {
    // longitude range does not affect rotation matrix. (FIMXE confirm)
    let ll = nvector_to_lat_long(n_e, LongitudeRange::L180);
    let lat = ll.latitude();
    let lon = ll.longitude();
    let r = xyz2r(lon, -lat, wander_azimuth);
    r_ee_t() * r
}

pub fn n_e_and_ypr2_r_eb(n_e: Vec3, orientation: BodyOrientation) -> Mat33 {
    let r_nb = zyx2r(orientation.yaw(), orientation.pitch(), orientation.roll());
    let r_en = n_e2_r_en(n_e);
    // closest frames cancel: N
    r_en * r_nb
}

const fn r_ee_t() -> Mat33 {
    Mat33::new(Vec3::neg_unit_z(), Vec3::unit_y(), Vec3::unit_x())
}
