#![doc = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/README.md"))]
#![deny(
    anonymous_parameters,
    dead_code,
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
#![warn(missing_docs)]
#![deny(clippy::all)]
#![warn(clippy::pedantic)]
#![allow(clippy::must_use_candidate)]
#![allow(clippy::return_self_not_must_use)]
#![allow(clippy::many_single_char_names)]
#![allow(clippy::similar_names)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_sign_loss)]
#![allow(clippy::float_cmp)]
#![allow(clippy::cast_precision_loss)]

mod mat33;
pub use crate::mat33::Mat33;

#[macro_use]
mod measurement;
pub use crate::measurement::Measurement;

mod angle;
pub use crate::angle::Angle;

pub mod ellipsoidal;

pub mod local;

mod length;
pub use crate::length::Length;

mod numbers;

mod positions;
pub use crate::positions::{
    GeocentricPosition, GeodeticPosition, LatLong, NVector, PositionVector,
};

mod speed;
pub use crate::speed::Speed;

pub mod spherical;

mod surface;
pub use crate::surface::Surface;

mod vec3;
pub use crate::vec3::Vec3;

mod vehicle;
pub use crate::vehicle::Vehicle;
