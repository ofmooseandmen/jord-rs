use crate::geodetic::{nvector_from_lat_long_radians, nvector_to_lat_long_radians};
use crate::{Angle, Vec3};

pub(crate) enum Rounding {
    None,
    Angle,
}

impl Rounding {
    pub(crate) fn north_pole(&self) -> Vec3 {
        self.round_pos(Vec3::unit_z())
    }

    pub(crate) fn round_radians(&self, radians: f64) -> f64 {
        match self {
            Rounding::None => radians,
            Rounding::Angle => Angle::as_radians(Angle::from_radians(radians)),
        }
    }

    pub(crate) fn round_pos(&self, pos: Vec3) -> Vec3 {
        match self {
            Rounding::None => pos,
            Rounding::Angle => {
                let (lat, lon) = nvector_to_lat_long_radians(pos);
                nvector_from_lat_long_radians(self.round_radians(lat), self.round_radians(lon))
            }
        }
    }
}
