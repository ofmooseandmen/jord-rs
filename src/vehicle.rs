use crate::{Angle, NVector, Speed};

/// The state of a vehicle: its horizontal position and velocity (bearing and speed).
#[derive(PartialEq, Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Vehicle {
    position: NVector,
    bearing: Angle,
    speed: Speed,
}

impl Vehicle {
    /// Creates a [Vehicle] from given horizontal position and velocity (bearing and speed).
    pub fn new(position: NVector, bearing: Angle, speed: Speed) -> Self {
        Self {
            position,
            bearing,
            speed,
        }
    }

    /// Returns the horizontal position of this vehicle.
    #[inline]
    pub fn position(&self) -> NVector {
        self.position
    }

    /// Returns the bearing of this vehicle.
    #[inline]
    pub fn bearing(&self) -> Angle {
        self.bearing
    }

    /// Returns the speed of this vehicle.
    #[inline]
    pub fn speed(&self) -> Speed {
        self.speed
    }
}
