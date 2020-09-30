use crate::geodetic::{nvector_from_lat_long_radians, nvector_to_lat_long_radians};
use crate::{Angle, Vec3};

pub enum Rounding {
    None,
    Angle,
}

impl Rounding {
    pub fn north_pole(&self) -> Vec3 {
        let np = Vec3::unit_z();
        match self {
            Rounding::None => np,
            Rounding::Angle => {
                let (lat, lon) = nvector_to_lat_long_radians(np);
                nvector_from_lat_long_radians(self.round_radians(lat), self.round_radians(lon))
            }
        }
    }

    pub fn round_radians(&self, radians: f64) -> f64 {
        match self {
            Rounding::None => radians,
            Rounding::Angle => Angle::as_radians(Angle::from_radians(radians)),
        }
    }
}

pub fn modulo(a: f64, b: f64) -> f64 {
    ((a % b) + b) % b
}
