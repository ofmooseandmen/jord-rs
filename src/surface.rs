use std::fmt::Debug;

use crate::{GeocentricPos, GeodeticPos};

/// The reference surface for the celestial body (e.g. Earth) on which calculations done.

pub trait Surface: Clone + Copy + Debug + Sized {
    fn geodetic_to_geocentric(&self, pos: GeodeticPos) -> GeocentricPos;
    fn geocentric_to_geodetic(&self, pos: GeocentricPos) -> GeodeticPos;
}
