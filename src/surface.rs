use std::fmt::Debug;

use crate::{GeocentricPos, GeodeticPos};

/// The reference surface for a celestial body (e.g. Earth) on which calculations are done.
pub trait Surface: Clone + Copy + Debug + Sized {
    /// Converts the given [GeodeticPos] into a [GeocentricPos].
    fn geodetic_to_geocentric(&self, pos: GeodeticPos) -> GeocentricPos;

    /// Converts the given [GeocentricPos] into a [GeodeticPos].
    fn geocentric_to_geodetic(&self, pos: GeocentricPos) -> GeodeticPos;
}
