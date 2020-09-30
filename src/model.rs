use crate::Surface;

#[derive(Debug, PartialEq)]
pub enum LongitudeRange {
    L180,
    L360,
}

#[derive(Debug)]
pub struct Epoch(f64);

#[derive(Debug, Eq, PartialEq)]
pub struct ModelId {
    id: String,
}
impl ModelId {
    pub fn new(id: String) -> ModelId {
        ModelId { id }
    }
}

pub trait Model: Clone + Copy {
    type Surface: Surface;
    fn model_id(&self) -> ModelId;
    fn surface(&self) -> Self::Surface;
    fn longitude_range(&self) -> LongitudeRange;
}

pub trait Spherical: Model {}

pub trait Ellipsoidal: Model {}

pub trait EllipsoidalT0: Ellipsoidal {
    fn epoch(&self) -> Epoch;
}
