//! The `jord` crate implements various geographical position calculations.
//!
//! # Literature
//!
//! The following reference provide the theoretical basis of most of the algorithms:
//!
//! - [Non-singular Horizontal Position Representation; Gade, K.; 2010](http://www.navlab.net/Publications/A_Nonsingular_Horizontal_Position_Representation.pdf)
//! - [Some Tactical Algorithms for Spherical Geometry](https://calhoun.nps.edu/bitstream/handle/10945/29516/sometacticalalgo00shud.pdf)
//! - [Triangulation by Ear Clipping](https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf)
//! - [STR: A Simple and Efficient Algorithm for R-Tree Packing](https://apps.dtic.mil/sti/pdfs/ADA324493.pdf)
//!
#![forbid(
    anonymous_parameters,
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
#![warn(missing_docs)]
#![deny(clippy::all)]

mod mat33;
pub use crate::mat33::Mat33;

#[macro_use]
mod measurement;
pub use crate::measurement::Measurement;

mod angle;
pub use crate::angle::Angle;

pub mod ellipsoidal;

mod local_frame;
pub use crate::local_frame::{r2xyz, r2zyx, xyz2r, zyx2r, LocalFrame, LocalPositionVector};

mod length;
pub use crate::length::Length;

mod numbers;

mod positions;
pub use crate::positions::{GeocentricPos, GeodeticPos, LatLong, NVector, PositionVector};

pub mod spherical;

mod surface;

mod vec3;
pub use crate::vec3::Vec3;
