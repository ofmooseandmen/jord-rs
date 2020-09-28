use ::std::convert::{From, TryFrom};

use crate::models::{S84Model, S84};
use crate::{Angle, FixedAngle, LongitudeRange, Model, Vec3};

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
        let nv = nvector_from_lat_long(latitude, longitude);
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
        let ll = nvector_to_lat_long(self.nvector);
        let lat = ll.0;
        let lon = convert_lon(check_pole(lat, ll.1), self.model.longitude_range());
        (lat, lon)
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
        NvectorPos {
            nvector: nvector_from_lat_long(llp.latitude, llp.longitude),
            model: llp.model,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LatLongPos<M: Model> {
    latitude: FixedAngle,
    longitude: FixedAngle,
    model: M,
}

// FIXME: wrap lat/long
impl<M: Model> LatLongPos<M> {
    pub fn new(latitude: FixedAngle, longitude: FixedAngle, model: M) -> Result<Self, PosError> {
        if !is_valid_lat(latitude) {
            Err(PosError::InvalidLatitude(latitude.to_decimal_degrees()))
        } else if !is_valid_lon(longitude, model.longitude_range()) {
            Err(PosError::InvalidLongitude(longitude.to_decimal_degrees()))
        } else {
            Ok(LatLongPos {
                latitude: latitude,
                longitude: check_pole(latitude, longitude),
                model: model,
            })
        }
    }

    pub fn from_decimal_degrees(latitude: f64, longitude: f64, model: M) -> Result<Self, PosError> {
        LatLongPos::new(
            FixedAngle::from_decimal_degrees(latitude),
            FixedAngle::from_decimal_degrees(longitude),
            model,
        )
    }

    pub fn from_nvector(nv: Vec3, model: M) -> Self {
        let ll = nvector_to_lat_long(nv);
        let lat = ll.0;
        let lon = convert_lon(check_pole(lat, ll.1), model.longitude_range());
        LatLongPos {
            latitude: lat,
            longitude: lon,
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
        nvector_from_lat_long(self.latitude, self.longitude)
    }

    pub fn latitude(&self) -> FixedAngle {
        self.latitude
    }

    pub fn longitude(&self) -> FixedAngle {
        self.longitude
    }

    pub fn model(&self) -> M {
        self.model
    }

    // FIXME: antipode
}

impl LatLongPos<S84Model> {
    pub fn s84(latitude: f64, longitude: f64) -> Result<Self, PosError> {
        LatLongPos::from_decimal_degrees(latitude, longitude, S84)
    }
}

impl<M: Model> TryFrom<(FixedAngle, FixedAngle, M)> for LatLongPos<M> {
    type Error = PosError;

    fn try_from(llm: (FixedAngle, FixedAngle, M)) -> Result<Self, Self::Error> {
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
        let ll = nvector_to_lat_long(nvp.nvector);
        let lat = ll.0;
        let lon = convert_lon(check_pole(lat, ll.1), nvp.model.longitude_range());
        LatLongPos {
            latitude: lat,
            longitude: lon,
            model: nvp.model,
        }
    }
}

pub fn nvector_to_lat_long<A: Angle>(nv: Vec3) -> (A, A) {
    let x = nv.x();
    let y = nv.y();
    let z = nv.z();
    let lat = A::atan2(z, (x * x + y * y).sqrt());
    let lon = A::atan2(y, x);
    (lat, lon)
}

pub fn nvector_from_lat_long<A: Angle>(lat: A, lon: A) -> Vec3 {
    let cl = lat.cos();
    let x = cl * lon.cos();
    let y = cl * lon.sin();
    let z = lat.sin();
    Vec3::new(x, y, z)
}

fn check_pole<A: Angle>(lat: A, lon: A) -> A {
    if lat.abs() == A::quarter_circle() {
        A::zero()
    } else {
        lon
    }
}

fn convert_lon<A: Angle>(lon: A, lr: LongitudeRange) -> A {
    if lr == LongitudeRange::L180 {
        lon
    } else if is_valid_lon(lon, lr) {
        lon
    } else {
        lon + A::full_circle()
    }
}

fn is_valid_lat<A: Angle>(lat: A) -> bool {
    lat.is_within(-A::quarter_circle(), A::quarter_circle())
}

fn is_valid_lon<A: Angle>(lon: A, lr: LongitudeRange) -> bool {
    if lr == LongitudeRange::L360 {
        lon.is_within(A::zero(), A::full_circle())
    } else {
        lon.is_within(-A::half_circle(), A::half_circle())
    }
}
