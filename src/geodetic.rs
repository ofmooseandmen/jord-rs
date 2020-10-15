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

    pub fn rounded_to(&self, resolution: AngleResolution) -> Self {
        LatLong(self.0.rounded_to(resolution), self.1.rounded_to(resolution))
    }
}

// FIXME Display
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HorizontalPos<M: Model>(Vec3, M);

impl<M: Model> HorizontalPos<M> {
    pub fn new(nvector: Vec3, model: M) -> Self {
        HorizontalPos(nvector, model)
    }

    pub fn from_decimal_lat_long(latitude: f64, longitude: f64, model: M) -> Self {
        let (lat, lon) = wrap(latitude, longitude, model.longitude_range());
        let nvector = nvector_from_lat_long_radians(lat.to_radians(), lon.to_radians());
        HorizontalPos(nvector, model)
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
        let ll = nvector_to_lat_long_radians(self.0);
        let lat = ll.0.to_degrees();
        let lon = ll.1.to_degrees();
        LatLong(
            Angle::from_decimal_degrees(lat),
            Angle::from_decimal_degrees(convert_lon(lat, lon, self.1.longitude_range())),
        )
    }

    pub fn rounded_to(&self, resolution: AngleResolution) -> Self {
        let ll = self.to_lat_long().rounded_to(resolution);
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

fn nvector_from_lat_long_radians(lat: f64, lon: f64) -> Vec3 {
    let cl = lat.cos();
    let x = cl * lon.cos();
    let y = cl * lon.sin();
    let z = lat.sin();
    Vec3::new(x, y, z)
}

fn nvector_to_lat_long_radians(nv: Vec3) -> (f64, f64) {
    let x = nv.x();
    let y = nv.y();
    let z = nv.z();
    let lat = z.atan2((x * x + y * y).sqrt());
    let lon = y.atan2(x);
    (lat, lon)
}

fn eq_lat_pole(lat: f64) -> bool {
    lat.abs() == 90.0
}

fn check_pole(lat: f64, lon: f64) -> f64 {
    if eq_lat_pole(lat) {
        0.0
    } else {
        lon
    }
}

fn convert_lon(lat: f64, lon: f64, lr: LongitudeRange) -> f64 {
    if eq_lat_pole(lat) {
        0.0
    } else if lr == LongitudeRange::L180 || is_valid_lon(lon, lr) {
        lon
    } else {
        lon + 360.0
    }
}

// https://gist.github.com/missinglink/d0a085188a8eab2ca66db385bb7c023a
fn wrap(lat: f64, lon: f64, lr: LongitudeRange) -> (f64, f64) {
    if is_valid_lat(lat) && is_valid_lon(lon, lr) {
        (lat, check_pole(lat, lon))
    } else {
        let quadrant = ((lat.abs() / 90.0).floor() % 4.0) as u8;
        let pole;
        if lat > 0.0 {
            pole = 90.0;
        } else {
            pole = -90.0;
        }
        let offset = lat % 90.0;
        println!("offset {}", offset);

        let wlat;
        let mut wlon = lon;
        match quadrant {
            0 => wlat = offset,
            1 => {
                wlat = pole - offset;
                wlon += 180.0;
            }
            2 => {
                wlat = -offset;
                wlon += 180.0;
            }
            3 => wlat = -pole + offset,
            _ => panic!("invalid quadrant {}", quadrant),
        }

        if wlon > 180.0 || wlon < -180.0 {
            wlon = wlon - ((wlon + 180.0) / 360.0).floor() * 360.0;
        }

        (wlat, convert_lon(wlat, wlon, lr))
    }
}

fn is_valid_lat(lat: f64) -> bool {
    lat >= -90.0 && lat <= 90.0
}

fn is_valid_lon(lon: f64, lr: LongitudeRange) -> bool {
    if lr == LongitudeRange::L360 {
        lon >= 0.0 && lon <= 360.0
    } else {
        lon >= -180.0 && lon <= 180.0
    }
}
