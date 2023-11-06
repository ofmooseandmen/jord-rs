use crate::{impl_measurement, Measurement};
use std::f64::consts::PI;

#[derive(PartialEq, PartialOrd, Clone, Copy, Debug, Default)]
/// A one-dimensional angle.
///
/// It primarely exists to unambigously represent an angle as opposed to a bare
/// [f64] (which could be anything and in any unit).
/// It allows conversion to or from radians and degrees.
///
/// [Angle] implements many traits, including [Add](::std::ops::Add), [Sub](::std::ops::Sub),
/// [Mul](::std::ops::Mul) and [Div](::std::ops::Div), among others.
pub struct Angle {
    radians: f64,
}

impl Angle {
    /// 2 PI radians.
    const TWO_PI: f64 = 2.0 * PI;

    /// Zero angle.
    pub const ZERO: Angle = Angle { radians: 0.0 };

    /// 180 degrees angle.
    pub const HALF_CIRCLE: Angle = Angle { radians: PI };

    /// Converts this angle to a floating point value in degrees.
    pub fn as_degrees(&self) -> f64 {
        self.radians.to_degrees()
    }

    /// Converts this angle to a floating point value in radians.
    pub fn as_radians(&self) -> f64 {
        self.radians
    }

    /// Creates an angle from a floating point value in degrees.
    pub fn from_degrees(degrees: f64) -> Self {
        Angle {
            radians: degrees.to_radians(),
        }
    }

    /// Creates an angle from a floating point value in radians.
    pub const fn from_radians(radians: f64) -> Self {
        Angle { radians }
    }

    /// Returns a new angle by normalising this angle to the range [0, 360) degrees.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Angle;
    ///
    /// assert_eq!(Angle::from_degrees(-361.0).normalised().as_degrees(), 359.0);
    /// assert_eq!(Angle::from_degrees(-2.0).normalised().as_degrees(), 358.0);
    /// assert_eq!(Angle::from_degrees(154.0).normalised().as_degrees(), 154.0);
    /// assert_eq!(Angle::from_degrees(360.0).normalised().as_degrees(), 0.0);
    /// ```
    pub fn normalised(&self) -> Self {
        if self.radians >= 0.0 && self.radians < Self::TWO_PI {
            *self
        } else {
            let res = self.radians % Self::TWO_PI;
            if res < 0.0 {
                Self::from_radians(res + Self::TWO_PI)
            } else {
                Self::from_radians(res)
            }
        }
    }

    /// Rounds this angle to the nearest decimal degrees with 5 decimal places - when representing
    /// an Earth latitude/longtiude this is approximately 1.11 metres at the equator.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Angle;
    ///
    /// assert_eq!(Angle::from_degrees(3.44444), Angle::from_degrees(3.444444).round_d5());
    /// assert_eq!(Angle::from_degrees(3.44445), Angle::from_degrees(3.444445).round_d5());
    /// ```
    pub fn round_d5(&self) -> Self {
        let d5 = (self.as_degrees() * 1e5).round() / 1e5;
        Self::from_degrees(d5)
    }

    /// Rounds this angle to the nearest decimal degrees with 6 decimal places - when representing
    /// an Earth latitude/longtiude this is approximately 0.111 metres at the equator.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Angle;
    ///
    /// assert_eq!(Angle::from_degrees(3.444444), Angle::from_degrees(3.4444444).round_d6());
    /// assert_eq!(Angle::from_degrees(3.444445), Angle::from_degrees(3.4444445).round_d6());
    /// ```
    pub fn round_d6(&self) -> Self {
        let d6 = (self.as_degrees() * 1e6).round() / 1e6;
        Self::from_degrees(d6)
    }

    /// Rounds this angle to the nearest decimal degrees with 7 decimal places - when representing
    /// an Earth latitude/longtiude this is approximately 1.11 centimetres at the equator.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Angle;
    ///
    /// assert_eq!(Angle::from_degrees(3.4444444), Angle::from_degrees(3.44444444).round_d7());
    /// assert_eq!(Angle::from_degrees(3.4444445), Angle::from_degrees(3.44444445).round_d7());
    /// ```
    pub fn round_d7(&self) -> Self {
        let d7 = (self.as_degrees() * 1e7).round() / 1e7;
        Self::from_degrees(d7)
    }
}

impl Measurement for Angle {
    fn from_default_unit(amount: f64) -> Self {
        Angle::from_radians(amount)
    }

    fn as_default_unit(&self) -> f64 {
        self.as_radians()
    }
}

impl_measurement! { Angle }
