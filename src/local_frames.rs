use crate::{xyz2r, zyx2r, Angle, GeocentricPos, GeodeticPos, Length, Mat33, Model, Vec3};

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
        Angle::from_decimal_degrees(-degs)
    }
}

pub trait LocalFrame<M: Model> {
    fn origin(&self) -> GeodeticPos<M>;

    fn r_ef(&self) -> Mat33;

    fn delta_to(&self, to: GeodeticPos<M>) -> Delta {
        let po = self.origin().to_geocentric().as_metres();
        let pt = to.to_geocentric().as_metres();
        // delta in Earth Frame.
        let delta_e = pt - po;
        // rotation matrix to go from Earth Frame to this Frame at origin.
        let rm = self.r_ef().transpose();
        let delta_f = rm * delta_e;
        Delta::from_vec3_metres(delta_f)
    }

    fn destination_pos(&self, delta: Delta) -> GeodeticPos<M> {
        let po = self.origin().to_geocentric().as_metres();
        // rotation matrix to go from Earth Frame to this Frame at origin.
        let rm = self.r_ef().transpose();
        let c = rm * delta.as_metres();
        let v = po + c;
        GeocentricPos::from_vec3_metres(v, self.origin().model()).to_geodetic()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyFrame<M>(GeodeticPos<M>, Angle, Angle, Angle);

impl<M: Model> BodyFrame<M> {
    pub fn new(origin: GeodeticPos<M>, yaw: Angle, pitch: Angle, roll: Angle) -> Self {
        BodyFrame(origin, yaw, pitch, roll)
    }

    pub fn yaw(&self) -> Angle {
        self.1
    }

    pub fn pitch(&self) -> Angle {
        self.2
    }

    pub fn roll(&self) -> Angle {
        self.3
    }
}

impl<M: Model> LocalFrame<M> for BodyFrame<M> {
    fn origin(&self) -> GeodeticPos<M> {
        self.0
    }

    fn r_ef(&self) -> Mat33 {
        let r_nb = zyx2r(self.yaw(), self.pitch(), self.roll());
        let r_en = n_e2_r_en(self.origin());
        // closest frames cancel: N
        r_en * r_nb
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LocalLevelFrame<M>(GeodeticPos<M>, Angle);

impl<M: Model> LocalLevelFrame<M> {
    pub fn new(origin: GeodeticPos<M>, wander_azimuth: Angle) -> Self {
        LocalLevelFrame(origin, wander_azimuth)
    }

    pub fn wander_azimuth(&self) -> Angle {
        self.1
    }
}

impl<M: Model> LocalFrame<M> for LocalLevelFrame<M> {
    fn origin(&self) -> GeodeticPos<M> {
        self.0
    }

    fn r_ef(&self) -> Mat33 {
        n_e_and_wa2_r_el(self.origin(), self.wander_azimuth())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NedFrame<M>(GeodeticPos<M>);

impl<M: Model> NedFrame<M> {
    pub fn new(origin: GeodeticPos<M>) -> Self {
        NedFrame(origin)
    }
}

impl<M: Model> LocalFrame<M> for NedFrame<M> {
    fn origin(&self) -> GeodeticPos<M> {
        self.0
    }

    fn r_ef(&self) -> Mat33 {
        n_e2_r_en(self.origin())
    }
}

pub fn n_e2_r_en<M: Model>(p_e: GeodeticPos<M>) -> Mat33 {
    let n_e = p_e.nvector();
    // down (pointing opposite to n-vector)
    let nz_e = -1.0 * n_e;
    // east (pointing perpendicular to the plane)
    let ny_e_direction = Vec3::unit_z().cross(n_e);
    let ny_e;
    if ny_e_direction == Vec3::zero() {
        // selected y-axis direction
        ny_e = Vec3::new(0.0, 1.0, 0.0);
    } else {
        // outside poles
        ny_e = ny_e_direction.unit();
    }
    // x-axis of N (North): found by right hand rule
    let nx_e = ny_e.cross(nz_e);
    Mat33::new(nx_e, ny_e, nz_e)
}

pub fn n_e_and_wa2_r_el<M: Model>(p_e: GeodeticPos<M>, wander_azimuth: Angle) -> Mat33 {
    let ll = p_e.to_lat_long();
    let lat = ll.latitude();
    let lon = ll.longitude();
    let r = xyz2r(lon, -lat, wander_azimuth);
    let r_ee = Mat33::new(
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    r_ee * r
}
