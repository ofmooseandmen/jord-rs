use crate::models::{S84Model, S84};
use crate::{Angle, AngleResolution, LongitudeRange, Model, Vec3};
use std::convert::From;

// FIXME Display
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LatLong(Angle, Angle);

impl LatLong {
    pub const fn latitude(&self) -> Angle {
        self.0
    }

    pub const fn longitude(&self) -> Angle {
        self.1
    }

    pub fn round(&self, resolution: AngleResolution) -> Self {
        LatLong(self.0.round(resolution), self.1.round(resolution))
    }

    pub const fn north_pole() -> Self {
        LatLong(Angle::from_decimal_degrees(90.0), Angle::zero())
    }

    pub const fn south_pole() -> Self {
        LatLong(Angle::from_decimal_degrees(-90.0), Angle::zero())
    }
}

// FIXME Display
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HorizontalPos<M>(Vec3, M);

impl<M: Model> HorizontalPos<M> {
    pub fn new(nvector: Vec3, model: M) -> Self {
        HorizontalPos(nvector, model)
    }

    pub fn from_decimal_lat_long(latitude: f64, longitude: f64, model: M) -> Self {
        if eq_lat_north_pole(latitude) {
            HorizontalPos::north_pole(model)
        } else if eq_lat_south_pole(latitude) {
            HorizontalPos::south_pole(model)
        } else {
            let nvector = nvector_from_lat_long_degrees(latitude, longitude);
            HorizontalPos(nvector, model)
        }
    }

    pub fn from_lat_long(latitude: Angle, longitude: Angle, model: M) -> Self {
        HorizontalPos::from_decimal_lat_long(
            latitude.decimal_degrees(),
            longitude.decimal_degrees(),
            model,
        )
    }

    pub fn north_pole(model: M) -> Self {
        HorizontalPos::new(Vec3::unit_z(), model)
    }

    pub fn south_pole(model: M) -> Self {
        HorizontalPos::new(Vec3::neg_unit_z(), model)
    }

    pub fn to_lat_long(&self) -> LatLong {
        if self.0 == Vec3::unit_z() {
            LatLong::north_pole()
        } else if self.0 == Vec3::neg_unit_z() {
            LatLong::south_pole()
        } else {
            let ll = nvector_to_lat_long_degrees(self.0);
            let lat = ll.0;
            let lon = ll.1;
            LatLong(
                Angle::from_decimal_degrees(lat),
                Angle::from_decimal_degrees(convert_lon(lat, lon, self.1.longitude_range())),
            )
        }
    }

    pub fn round(&self, resolution: AngleResolution) -> Self {
        let ll = self.to_lat_long().round(resolution);
        HorizontalPos::from_lat_long(ll.0, ll.1, self.1)
    }

    pub fn nvector(&self) -> Vec3 {
        self.0
    }

    pub fn model(&self) -> M {
        self.1
    }

    pub fn antipode(&self) -> Self {
        HorizontalPos(-1.0 * self.0, self.1)
    }
}

impl HorizontalPos<S84Model> {
    pub fn from_s84(latitude: f64, longitude: f64) -> Self {
        HorizontalPos::from_decimal_lat_long(latitude, longitude, S84)
    }
}

impl<M: Model> From<(Angle, Angle, M)> for HorizontalPos<M> {
    fn from(llm: (Angle, Angle, M)) -> Self {
        HorizontalPos::from_lat_long(llm.0, llm.1, llm.2)
    }
}

impl<M: Model> From<(f64, f64, M)> for HorizontalPos<M> {
    fn from(llm: (f64, f64, M)) -> Self {
        HorizontalPos::from_decimal_lat_long(llm.0, llm.1, llm.2)
    }
}

impl<M: Model> From<(Vec3, M)> for HorizontalPos<M> {
    fn from(nvm: (Vec3, M)) -> Self {
        HorizontalPos::new(nvm.0, nvm.1)
    }
}

fn nvector_from_lat_long_degrees(latitude: f64, longitude: f64) -> Vec3 {
    let lat = latitude.to_radians();
    let lon = longitude.to_radians();
    let cl = lat.cos();
    let x = cl * lon.cos();
    let y = cl * lon.sin();
    let z = lat.sin();
    Vec3::new(x, y, z)
}

fn nvector_to_lat_long_degrees(nv: Vec3) -> (f64, f64) {
    let x = nv.x();
    let y = nv.y();
    let z = nv.z();
    let lat = z.atan2((x * x + y * y).sqrt());
    let lon = y.atan2(x);
    (lat.to_degrees(), lon.to_degrees())
}

fn eq_lat_north_pole(lat: f64) -> bool {
    lat == 90.0
}

fn eq_lat_south_pole(lat: f64) -> bool {
    lat == -90.0
}

fn eq_lat_pole(lat: f64) -> bool {
    eq_lat_north_pole(lat.abs())
}

// lon is guaranteed to be within [-180, 180]
fn convert_lon(lat: f64, lon: f64, lr: LongitudeRange) -> f64 {
    if eq_lat_pole(lat) {
        0.0
    } else if lr == LongitudeRange::L180 {
        lon
    } else {
        if lon < 0.0 {
            lon + 360.0
        } else {
            lon
        }
    }
}
