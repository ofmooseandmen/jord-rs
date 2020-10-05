// Copyright: (c) 2020 Cedric Liegeois
// License: BSD3
use std::f64::consts::PI;

use crate::Measure;

/// A signed angle with a resolution of a microarcsecond.
/// When used as a latitude/longitude this roughly translate to a precision
/// of 0.03 millimetres at the equator.
///
/// `Angle` implements many traits, including [`Add`], [`Sub`], [`Mul`], and
/// [`Div`], among others.
// FIXME Display & FromStr
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct Angle {
    /// Number of whole microarcseconds.
    microarcseconds: i64,
}

/// The number of microarcseconds in one degree.
const DG_TO_UAS: f64 = 3_600_000_000.0;

// FIXME parse
impl Angle {
    /// Equivalent to `Angle::from_decimal_degrees(0.0)`.
    ///
    /// ```rust
    /// # use jord::Angle;
    /// assert_eq!(Angle::from_decimal_degrees(0.0), Angle::zero());
    /// ```
    pub fn zero() -> Self {
        Angle { microarcseconds: 0 }
    }

    /// Create a new `Angle` with the given number of decimal degrees.
    pub fn from_decimal_degrees(dec: f64) -> Self {
        let uas = (dec * DG_TO_UAS).round() as i64;
        Angle {
            microarcseconds: uas,
        }
    }

    /// Create a new `Angle` with the given number of whole degrees, minutes, seconds and milliseconds.
    /// Given minutes, seconds and milliseconds are wrapped if needed.
    ///
    ///  ```rust
    /// # use jord::Angle;
    /// assert_eq!(Angle::from_decimal_degrees(10.5125), Angle::from_dms(10, 30, 45, 0));
    /// assert_eq!(Angle::from_decimal_degrees(10.5125), Angle::from_dms(9, 89, 105, 0));
    /// ```
    pub fn from_dms(degrees: i16, minutess: u8, seconds: u8, milliseconds: u16) -> Self {
        let dd = degrees.abs() as f64
            + (minutess as f64 / 60.0)
            + (seconds as f64 / 3600.0)
            + (milliseconds as f64 / (3600.0 * 1000.0));
        if degrees < 0 {
            Angle::from_decimal_degrees(-dd)
        } else {
            Angle::from_decimal_degrees(dd)
        }
    }

    /// Create a new `Angle` with the given number of radians.
    pub fn from_radians(rads: f64) -> Self {
        Angle::from_decimal_degrees(rads / PI * 180.0)
    }

    /// Returns the number of microarcseconds of this `Angle`.
    ///
    ///  ```rust
    /// # use jord::Angle;
    /// assert_eq!(3_600_000_000, Angle::from_decimal_degrees(1.0).microarcseconds());
    /// ```
    pub fn microarcseconds(self) -> i64 {
        self.microarcseconds
    }

    /// Converts this `Angle` to a number of radians.
    pub fn as_radians(self) -> f64 {
        self.as_decimal_degrees() * PI / 180.0
    }

    /// Converts this `Angle` to a number of decimal degrees.
    pub fn as_decimal_degrees(self) -> f64 {
        self.microarcseconds as f64 / DG_TO_UAS
    }

    /// Returns the degree component of this `Angle`.
    ///
    ///  ```rust
    /// # use jord::Angle;
    /// assert_eq!(-154, Angle::from_dms(-154, 3, 42, 500).whole_degrees());
    /// ```
    pub fn whole_degrees(self) -> i64 {
        let d = Angle::field(self, DG_TO_UAS, 360.0) as i64;
        if self.microarcseconds < 0 {
            -d
        } else {
            d
        }
    }

    /// Returns the arcminutes component of this `Angle`.
    ///
    ///  ```rust
    /// # use jord::Angle;
    /// assert_eq!(45, Angle::from_dms(-154, 45, 42, 500).arcminutes());
    /// ```
    pub fn arcminutes(self) -> u8 {
        Angle::field(self, 60000000.0, 60.0) as u8
    }

    /// Returns the arcseconds component of this `Angle`.
    ///
    ///  ```rust
    /// # use jord::Angle;
    /// assert_eq!(42, Angle::from_dms(-154, 45, 42, 500).arcseconds());
    /// ```
    pub fn arcseconds(self) -> u8 {
        Angle::field(self, 1000000.0, 60.0) as u8
    }

    /// Returns the arcmilliseconds component of this `Angle`.
    ///
    ///  ```rust
    /// # use jord::Angle;
    /// assert_eq!(500, Angle::from_dms(-154, 45, 42, 500).arcmilliseconds());
    /// ```
    pub fn arcmilliseconds(self) -> u16 {
        Angle::field(self, 1000.0, 1000.0) as u16
    }

    fn field(self, div: f64, modu: f64) -> u64 {
        (self.microarcseconds.abs() as f64 / div % modu) as u64
    }
}

impl Measure for Angle {
    fn from_default_unit(amount: f64) -> Self {
        Angle::from_decimal_degrees(amount)
    }

    fn from_resolution_unit(amount: i64) -> Self {
        Angle {
            microarcseconds: amount,
        }
    }

    fn as_default_unit(self) -> f64 {
        self.as_decimal_degrees()
    }

    fn as_resolution_unit(self) -> i64 {
        self.microarcseconds
    }
}

impl_measure! { Angle }
