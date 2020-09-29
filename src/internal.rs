use crate::{Angle, LatLongPos, NvectorPos, Spherical};

pub trait Rounding {
    fn round_pos<S: Spherical>(&self, pos: NvectorPos<S>) -> NvectorPos<S>;
    fn round_radians(&self, radians: f64) -> f64;
}
pub struct NVRounding {}
impl Rounding for NVRounding {
    fn round_pos<S: Spherical>(&self, pos: NvectorPos<S>) -> NvectorPos<S> {
        pos
    }

    fn round_radians(&self, radians: f64) -> f64 {
        radians
    }
}
pub const NV: NVRounding = NVRounding {};

pub struct LLRounding {}
impl Rounding for LLRounding {
    fn round_pos<S: Spherical>(&self, pos: NvectorPos<S>) -> NvectorPos<S> {
        let ll: LatLongPos<S> = pos.into();
        ll.into()
    }

    fn round_radians(&self, radians: f64) -> f64 {
        Angle::as_radians(Angle::from_radians(radians))
    }
}
pub const LL: LLRounding = LLRounding {};

pub fn modulo(a: f64, b: f64) -> f64 {
    ((a % b) + b) % b
}
