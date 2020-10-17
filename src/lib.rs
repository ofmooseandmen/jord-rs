#![forbid(
    anonymous_parameters,
    clippy::all,
    const_err,
    illegal_floating_point_literal_pattern,
    late_bound_lifetime_arguments,
    path_statements,
    patterns_in_fns_without_body,
    rust_2018_idioms,
    trivial_casts,
    trivial_numeric_casts,
    unreachable_pub,
    unsafe_code,
    unused_extern_crates,
    missing_debug_implementations
)]

#[macro_use]
mod measurement;
pub use crate::measurement::Measurement;

pub mod angle;
pub use crate::angle::Angle;
pub use crate::angle::AngleResolution::{self, Arcsecond, Microarcsecond, Milliarcsecond};

pub mod error;
pub use crate::error::Error;

pub mod great_circle;
pub use crate::great_circle::{GreatCircle, MinorArc, Side};

pub mod geodetic;
pub use crate::geodetic::{HorizontalPos, LatLong};

pub mod length;
pub use crate::length::Length;
pub use crate::length::LengthResolution::{self, Metre, Micrometre, Millimetre};

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
