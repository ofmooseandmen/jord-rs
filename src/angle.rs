// Copyright: (c) 2020 Cedric Liegeois
// License: BSD3
use crate::Measurement;

/// A signed angle.
///
/// `Angle` implements many traits, including [`Add`], [`Sub`], [`Mul`], and
/// [`Div`], among others.
// FIXME Display & FromStr
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Angle {
    decimal_degrees: f64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AngleResolution {
    Arcsecond,
    Milliarcsecond,
    Microarcsecond,
}

/// The number of seconds in one degree.
const DG_TO_SECS: f64 = 3_600.0;

/// The number of milliseconds in one degree.
const DG_TO_MILLIS: f64 = DG_TO_SECS * 1_000.0;

/// The number of microseconds in one degree.
const DG_TO_MICROS: f64 = DG_TO_MILLIS * 1_000.0;

// FIXME parse
impl Angle {
    /// Equivalent to `Angle::from_decimal_degrees(0.0)`.
    ///
    /// ```rust
    /// # use jord::Angle;
    /// assert_eq!(Angle::from_decimal_degrees(0.0), Angle::zero());
    /// ```
    pub const fn zero() -> Self {
        Angle {
            decimal_degrees: 0.0,
        }
    }

    /// Create a new `Angle` with the given number of decimal degrees.
    pub const fn from_decimal_degrees(decimal_degrees: f64) -> Self {
        Angle { decimal_degrees }
    }

    /// Create a new `Angle` with the given number of arc degrees, minutes, seconds and milliseconds.
    /// Given minutes, seconds and milliseconds are wrapped if needed.
    ///
    ///  ```rust
    /// # use jord::Angle;
    /// assert_eq!(Angle::from_decimal_degrees(10.5125), Angle::from_dms(10, 30, 45, 0));
    /// assert_eq!(Angle::from_decimal_degrees(10.5125), Angle::from_dms(9, 89, 105, 0));
    /// ```
    pub fn from_dms(degrees: i16, minutes: u8, seconds: u8, milliseconds: u16) -> Self {
        let add = degrees.abs() as f64
            + (minutes as f64 / 60.0)
            + (seconds as f64 / 3600.0)
            + (milliseconds as f64 / (3600.0 * 1000.0));
        let dd;
        if degrees < 0 {
            dd = -add;
        } else {
            dd = add;
        }
        Angle::from_decimal_degrees(dd).rounded_to(AngleResolution::Milliarcsecond)
    }

    pub fn rounded_to(self, resolution: AngleResolution) -> Self {
        let res = match resolution {
            AngleResolution::Arcsecond => DG_TO_SECS,
            AngleResolution::Milliarcsecond => DG_TO_MILLIS,
            AngleResolution::Microarcsecond => DG_TO_MICROS,
        };
        let decimal_degrees = ((self.decimal_degrees * res).round()) / res;
        Angle { decimal_degrees }
    }

    /// Converts this `Angle` to a number of decimal degrees.
    pub const fn decimal_degrees(self) -> f64 {
        self.decimal_degrees
    }

    /// Returns the degree component of this `Angle`.
    ///
    ///  ```rust
    /// # use jord::Angle;
    /// assert_eq!(-154, Angle::from_dms(-154, 3, 42, 500).arcdegrees());
    /// ```
    pub const fn arcdegrees(self) -> i64 {
        self.decimal_degrees as i64
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
        let uas = (self.decimal_degrees * DG_TO_MICROS).round();
        (uas.abs() / div % modu) as u64
    }
}

impl Measurement for Angle {
    fn from_default_unit(amount: f64) -> Self {
        Angle::from_decimal_degrees(amount)
    }

    fn as_default_unit(self) -> f64 {
        self.decimal_degrees()
    }
}

impl_measurement! { Angle }
