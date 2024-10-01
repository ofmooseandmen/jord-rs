use std::fmt::Debug;

use crate::{GeocentricPosition, GeodeticPosition};

/// The reference surface for a celestial body (e.g. Earth) on which calculations are done.
pub trait Surface: Clone + Copy + Debug + Sized {
    /// Converts the given [GeodeticPosition] into a [GeocentricPosition].
    fn geodetic_to_geocentric_position(&self, pos: GeodeticPosition) -> GeocentricPosition;

    /// Converts the given [GeocentricPosition] into a [GeodeticPosition].
    fn geocentric_to_geodetic_position(&self, pos: GeocentricPosition) -> GeodeticPosition;
}
