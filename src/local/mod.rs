//! Local (topocentric) reference frames and the vectors within them.
//!
//! A [`LocalFrame`] is a 3D Cartesian coordinate frame anchored at a fixed
//! origin, used to express positions near that origin as short, human-scale
//! displacement vectors ([`LocalVector`]) instead of full [`GeocentricPosition`](crate::GeocentricPosition)s.
//! Unlike flat-surface approximations, all conversions between local vectors
//! and geocentric positions performed here are exact 3D rigid-body
//! transformations (translation + rotation), valid regardless of distance
//! from the origin.
//!
//! Four frame orientations are provided, each a [`LocalFrame<O>`] /
//! [`LocalVector<O>`] pair for a marker type `O` implementing
//! [`FrameOrientation`]:
//!
//! - [`EnuFrame`] / [`EnuVector`] — East-North-Up, tangent-plane, z-up.
//! - [`NedFrame`] / [`NedVector`] — North-East-Down, tangent-plane, z-down.
//! - [`BodyFrame`] / [`BodyVector`] — vehicle-fixed (forward/right/down),
//!   oriented by yaw/pitch/roll rather than tangency to the surface.
//! - [`WanderAzimuthFrame`] / [`WanderAzimuthVector`] — a NED-like, z-down,
//!   local-level frame that avoids the polar singularity NED and ENU suffer
//!   from, by not forcing its horizontal axes to track True North.
//!
//! [`EnuFrame`], [`NedFrame`] and [`WanderAzimuthFrame`] additionally
//! implement [`LocalNavigationFrame`], and can be rotated about their local
//! vertical axis via [`LocalFrame::rotate_around_z`] to produce a
//! [`WanderAzimuthFrame`]. [`BodyFrame`] does not: it is vehicle-fixed rather
//! than earth-tangent, so rotating about its own z-axis isn't a navigation
//! operation. Any [`LocalFrame`] can be re-oriented (offset + yaw/pitch/roll)
//! into a [`BodyFrame`] via [`LocalFrame::transform`].
//!
//! Two independent axes of navigation math live in this module:
//!
//! - Building and using the frames themselves: `from_geodetic`/`from_geocentric`
//!   constructors, [`LocalFrame::local_vector_to`] and
//!   [`LocalFrame::destination_position`] to convert to/from geocentric
//!   positions, and [`LocalVector::from_aer`] to build a vector from
//!   azimuth/elevation/range spherical coordinates.
//! - General-purpose rotation-matrix ⟷ Euler-angle conversions
//!   ([`zyx2r`], [`xyz2r`], [`r2zyx`], [`r2xyz`]) used internally to build
//!   frame rotation matrices from yaw/pitch/roll (or similar) angles, and
//!   exposed publicly for callers who need the same conversions directly.
mod frame;
pub use frame::{
    BodyFrame, EnuFrame, LocalFrame, NedFrame, WanderAzimuthFrame, r2xyz, r2zyx, xyz2r, zyx2r,
};

mod orientation;
pub use orientation::{
    Body, Enu, FrameOrientation, LocalNavigationFrame, Ned, WanderAzimuth, align_to_z_down_matrix,
};

mod vector;
pub use vector::{BodyVector, EnuVector, LocalVector, NedVector, WanderAzimuthVector};
