// FIXME generate from txt file
use crate::surfaces::*;
use crate::{LongitudeRange, Model, ModelId, Sphere, Spherical};

pub const S84: S84Model = S84Model {};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct S84Model {}

impl Model for S84Model {
    type Surface = Sphere;
    fn model_id(&self) -> ModelId {
        ModelId::new("S84".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L180
    }
    fn surface(&self) -> Sphere {
        wgs84_sphere()
    }
}

impl Spherical for S84Model {}
