use ::std::convert::{From, TryFrom};

use crate::models::{S84Model, S84};
use crate::{Angle, LongitudeRange, Model, Vec3};

// FIXME Display
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NvectorPos<M: Model> {
    nvector: Vec3,
    model: M,
}

#[derive(Debug)]
pub enum PosError {
    InvalidLatitude(f64),
    InvalidLongitude(f64),
}

impl<M: Model> NvectorPos<M> {
    pub fn new(nv: Vec3, model: M) -> Self {
        NvectorPos {
            nvector: nv,
            model: model,
        }
    }

    pub fn from_lat_long(latitude: f64, longitude: f64, model: M) -> Self {
        let nv = nvector_from_lat_long_radians(latitude.to_radians(), longitude.to_radians());
        NvectorPos {
            nvector: nv,
            model: model,
        }
    }

    pub fn north_pole(model: M) -> Self {
        NvectorPos::new(Vec3::new(0.0, 0.0, 1.0), model)
    }

    pub fn south_pole(model: M) -> Self {
        NvectorPos::new(Vec3::new(0.0, 0.0, -1.0), model)
    }

    pub fn to_lat_long(&self) -> (f64, f64) {
        let ll = nvector_to_lat_long_radians(self.nvector);
        let lat = ll.0.to_degrees();
        let lon = ll.1.to_degrees();
        (lat, convert_lon(lat, lon, self.model.longitude_range()))
    }

    pub fn nvector(&self) -> Vec3 {
        self.nvector
    }

    pub fn model(&self) -> M {
        self.model
    }

    pub fn antipode(&self) -> Self {
        let anti = self.nvector * -1.0;
        NvectorPos {
            nvector: anti,
            model: self.model,
        }
    }
}

impl<M: Model> From<(Vec3, M)> for NvectorPos<M> {
    fn from(nvm: (Vec3, M)) -> Self {
        NvectorPos::new(nvm.0, nvm.1)
    }
}

impl<M: Model> From<([f64; 3], M)> for NvectorPos<M> {
    fn from(nvm: ([f64; 3], M)) -> Self {
        NvectorPos::new(nvm.0.into(), nvm.1)
    }
}

impl<M: Model> From<LatLongPos<M>> for NvectorPos<M> {
    fn from(llp: LatLongPos<M>) -> Self {
        NvectorPos::from_lat_long(
            llp.latitude.as_decimal_degrees(),
            llp.longitude.as_decimal_degrees(),
            llp.model,
        )
    }
}

// FIXME Display & FromStr
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LatLongPos<M: Model> {
    latitude: Angle,
    longitude: Angle,
    model: M,
}

// FIXME: wrap lat/long, parse & antipode
impl<M: Model> LatLongPos<M> {
    pub fn new(latitude: Angle, longitude: Angle, model: M) -> Result<Self, PosError> {
        let latd = latitude.as_decimal_degrees();
        let longd = longitude.as_decimal_degrees();
        if !is_valid_lat(latd) {
            Err(PosError::InvalidLatitude(latd))
        } else if !is_valid_lon(longd, model.longitude_range()) {
            Err(PosError::InvalidLongitude(longd))
        } else {
            Ok(LatLongPos {
                latitude: latitude,
                longitude: Angle::from_decimal_degrees(check_pole(latd, longd)),
                model: model,
            })
        }
    }

    pub fn from_decimal_degrees(latitude: f64, longitude: f64, model: M) -> Result<Self, PosError> {
        LatLongPos::new(
            Angle::from_decimal_degrees(latitude),
            Angle::from_decimal_degrees(longitude),
            model,
        )
    }

    pub fn from_nvector(nv: Vec3, model: M) -> Self {
        let ll = nvector_to_lat_long_radians(nv);
        let lat = Angle::from_radians(ll.0);
        let lon = Angle::from_radians(ll.1);
        let clon = convert_lon(
            lat.as_decimal_degrees(),
            lon.as_decimal_degrees(),
            model.longitude_range(),
        );
        LatLongPos {
            latitude: lat,
            longitude: Angle::from_decimal_degrees(clon),
            model: model,
        }
    }

    pub fn north_pole(model: M) -> Self {
        LatLongPos::from_decimal_degrees(90.0, 0.0, model).unwrap()
    }

    pub fn south_pole(model: M) -> Self {
        LatLongPos::from_decimal_degrees(-90.0, 0.0, model).unwrap()
    }

    pub fn to_nvector(&self) -> Vec3 {
        nvector_from_lat_long_radians(self.latitude.as_radians(), self.longitude.as_radians())
    }

    pub fn latitude(&self) -> Angle {
        self.latitude
    }

    pub fn longitude(&self) -> Angle {
        self.longitude
    }

    pub fn model(&self) -> M {
        self.model
    }
}

impl LatLongPos<S84Model> {
    pub fn s84(latitude: f64, longitude: f64) -> Result<Self, PosError> {
        LatLongPos::from_decimal_degrees(latitude, longitude, S84)
    }
}

impl<M: Model> TryFrom<(Angle, Angle, M)> for LatLongPos<M> {
    type Error = PosError;

    fn try_from(llm: (Angle, Angle, M)) -> Result<Self, Self::Error> {
        LatLongPos::new(llm.0, llm.1, llm.2)
    }
}

impl<M: Model> TryFrom<(f64, f64, M)> for LatLongPos<M> {
    type Error = PosError;

    fn try_from(llm: (f64, f64, M)) -> Result<Self, Self::Error> {
        LatLongPos::from_decimal_degrees(llm.0, llm.1, llm.2)
    }
}

impl<M: Model> From<NvectorPos<M>> for LatLongPos<M> {
    fn from(nvp: NvectorPos<M>) -> Self {
        LatLongPos::from_nvector(nvp.nvector, nvp.model)
    }
}

fn nvector_to_lat_long_radians(nv: Vec3) -> (f64, f64) {
    let x = nv.x();
    let y = nv.y();
    let z = nv.z();
    let lat = z.atan2((x * x + y * y).sqrt());
    let lon = y.atan2(x);
    (lat, lon)
}

fn nvector_from_lat_long_radians(lat: f64, lon: f64) -> Vec3 {
    let cl = lat.cos();
    let x = cl * lon.cos();
    let y = cl * lon.sin();
    let z = lat.sin();
    Vec3::new(x, y, z)
}

fn check_pole(lat: f64, lon: f64) -> f64 {
    if lat.abs() == 90.0 {
        0.0
    } else {
        lon
    }
}

fn convert_lon(lat: f64, lon: f64, lr: LongitudeRange) -> f64 {
    if lat.abs() == 90.0 {
        0.0
    } else if lr == LongitudeRange::L180 {
        lon
    } else if is_valid_lon(lon, lr) {
        lon
    } else {
        lon + 360.0
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
