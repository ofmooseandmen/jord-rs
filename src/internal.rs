use crate::{FixedLength, LatLongPos, NvectorPos, Spherical, Surface, Vec3};

pub trait Positioning {
    type Length;

    fn at_resolution<M: Spherical>(&self, nv: Vec3, model: M) -> NvectorPos<M>;
    fn north_pole<M: Spherical>(&self, model: M) -> Vec3;
    fn earth_radius<M: Spherical>(&self, nv: NvectorPos<M>) -> Self::Length;
}

pub struct F64Positioning {}
impl Positioning for F64Positioning {
    type Length = f64;

    fn at_resolution<M: Spherical>(&self, nv: Vec3, model: M) -> NvectorPos<M> {
        (nv, model).into()
    }

    fn north_pole<M: Spherical>(&self, _model: M) -> Vec3 {
        Vec3::new(0.0, 0.0, 1.0)
    }

    fn earth_radius<M: Spherical>(&self, nv: NvectorPos<M>) -> f64 {
        nv.model().surface().mean_radius().to_metres()
    }
}
pub const F64: F64Positioning = F64Positioning {};

pub struct FixedPositioning {}
impl Positioning for FixedPositioning {
    type Length = FixedLength;

    fn at_resolution<M: Spherical>(&self, nv: Vec3, model: M) -> NvectorPos<M> {
        let nvm: NvectorPos<M> = (nv, model).into();
        let ll: LatLongPos<M> = nvm.into();
        ll.into()
    }

    fn north_pole<M: Spherical>(&self, model: M) -> Vec3 {
        let np: NvectorPos<M> = LatLongPos::north_pole(model).into();
        np.nvector()
    }

    fn earth_radius<M: Spherical>(&self, nv: NvectorPos<M>) -> FixedLength {
        nv.model().surface().mean_radius()
    }
}
pub const FIXED: FixedPositioning = FixedPositioning {};

pub fn modulo(a: f64, b: f64) -> f64 {
    ((a % b) + b) % b
}
