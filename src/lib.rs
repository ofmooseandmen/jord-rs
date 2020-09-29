#![forbid(unsafe_code)]
#![deny(missing_debug_implementations)]

#[macro_use]
mod measure;
pub use crate::measure::Measure;

pub mod angle;
pub use crate::angle::{Angle, FixedAngle};

pub mod great_circle;

pub mod geodetic;
pub use crate::geodetic::{
    nvector_from_lat_long, nvector_to_lat_long, LatLongPos, NvectorPos, PosError,
};

pub mod length;
pub use crate::length::{FixedLength, Length};

pub mod math3d;
pub use crate::math3d::Mat33;
pub use crate::math3d::Vec3;

pub mod model;
pub use crate::model::Ellipsoidal;
pub use crate::model::EllipsoidalT0;
pub use crate::model::Epoch;
pub use crate::model::LongitudeRange;
pub use crate::model::Model;
pub use crate::model::ModelId;
pub use crate::model::Spherical;

pub mod models;

pub mod surface;
pub use crate::surface::{Ellipsoid, Sphere, Surface};

pub mod surfaces;

mod positioning;
