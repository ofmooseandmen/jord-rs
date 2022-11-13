use crate::{impl_measurement, Angle, Measurement};

#[derive(PartialEq, PartialOrd, Clone, Copy, Debug, Default)]
/// A length.
///
/// It primarely exists to unambigously represent a length as opposed to a bare
/// [f64] (which could be anything and in any unit).
/// It allows conversion to or from metres, kilometres, feet and nautical miles.
///
/// # Examples
///
/// ```
/// use jord::Length;
///
/// assert_eq!(1.0, Length::from_metres(1.0).as_metres());
/// assert_eq!(1000.0, Length::from_kilometres(1.0).as_metres());
/// assert_eq!(0.3048, Length::from_feet(1.0).as_metres());
/// assert_eq!(1852.0, Length::from_nautical_miles(1.0).as_metres());
/// ```
///
/// [Length] implements many traits, including [Add](::std::ops::Add), [Sub](::std::ops::Sub),
/// [Mul](::std::ops::Mul) and [Div](::std::ops::Div), among others.
///
/// # Examples
///
/// ```
/// use std::f64::consts::PI;
/// use jord::{Angle, Length};
///
/// assert_eq!(Length::from_metres(3.0), Length::from_metres(1.0) + Length::from_metres(2.0));
/// assert_eq!(Length::ZERO, Length::from_metres(1.0) - Length::from_metres(1.0));
/// assert_eq!(Length::from_metres(6.0), 3.0 * Length::from_metres(2.0));
/// assert_eq!(Length::from_metres(6.0), Length::from_metres(2.0) * 3.0);
/// assert_eq!(Length::from_metres(2.0), Length::from_metres(4.0) / 2.0);
/// assert_eq!(Length::from_metres(PI), Length::from_metres(1.0) * Angle::from_radians(PI));
/// assert_eq!(Length::from_metres(PI), Angle::from_radians(PI) * Length::from_metres(1.0));
/// ```
pub struct Length {
    metres: f64,
}

impl Length {
    const FT_TO_M: f64 = 0.3048;

    const NM_TO_M: f64 = 1_852.0;

    const KM_TO_M: f64 = 1_000.0;

    /// Zero length.
    pub const ZERO: Length = Length { metres: 0.0 };

    /// Creates a length from a floating point value in metres.
    pub const fn from_metres(metres: f64) -> Self {
        Length { metres }
    }

    /// Creates a length from a floating point value in kilometres.
    pub fn from_kilometres(kilometres: f64) -> Self {
        Length::from_metres(kilometres * Self::KM_TO_M)
    }

    /// Creates a length from a floating point value in feet.
    pub fn from_feet(feet: f64) -> Self {
        Length::from_metres(feet * Self::FT_TO_M)
    }

    /// Creates a length from a floating point value in nautical miles.
    pub fn from_nautical_miles(nautical_miles: f64) -> Self {
        Length::from_metres(nautical_miles * Self::NM_TO_M)
    }

    /// Converts this length to a floating point value in metres.
    pub const fn as_metres(&self) -> f64 {
        self.metres
    }

    /// Converts this length to a floating point value in kilometres.
    pub fn as_kilometres(&self) -> f64 {
        self.metres / Self::KM_TO_M
    }

    /// Converts this length to a floating point value in feet.
    pub fn as_feet(&self) -> f64 {
        self.metres / Self::FT_TO_M
    }

    /// Converts this length to a floating point value in nautical miles.
    pub fn as_nautical_miles(&self) -> f64 {
        self.metres / Self::NM_TO_M
    }

    /// Rounds this length to the nearest metre.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Length;
    ///
    /// assert_eq!(Length::from_metres(3.0), Length::from_metres(3.4).round_m());
    /// assert_eq!(Length::from_metres(4.0), Length::from_metres(3.5).round_m());
    /// ```
    pub fn round_m(&self) -> Self {
        Self {
            metres: self.metres.round(),
        }
    }

    /// Rounds this length to the nearest decimetre.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Length;
    ///
    /// assert_eq!(Length::from_metres(3.4), Length::from_metres(3.44).round_dm());
    /// assert_eq!(Length::from_metres(3.5), Length::from_metres(3.45).round_dm());
    /// ```
    pub fn round_dm(&self) -> Self {
        let metres = (self.metres * 10.0).round() / 10.0;
        Self { metres }
    }

    /// Rounds this length to the nearest centimetre.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Length;
    ///
    /// assert_eq!(Length::from_metres(3.44), Length::from_metres(3.444).round_cm());
    /// assert_eq!(Length::from_metres(3.45), Length::from_metres(3.445).round_cm());
    /// ```
    pub fn round_cm(&self) -> Self {
        let metres = (self.metres * 100.0).round() / 100.0;
        Self { metres }
    }

    /// Rounds this length to the nearest millimetre.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Length;
    ///
    /// assert_eq!(Length::from_metres(3.444), Length::from_metres(3.4444).round_mm());
    /// assert_eq!(Length::from_metres(3.445), Length::from_metres(3.4445).round_mm());
    /// ```
    pub fn round_mm(&self) -> Self {
        let metres = (self.metres * 1000.0).round() / 1000.0;
        Self { metres }
    }
}

impl Measurement for Length {
    fn from_default_unit(amount: f64) -> Self {
        Length::from_metres(amount)
    }

    fn as_default_unit(&self) -> f64 {
        self.metres
    }
}

impl_measurement! { Length }

impl ::std::ops::Mul<Angle> for Length {
    type Output = Length;

    fn mul(self, rhs: Angle) -> Length {
        Length::from_metres(rhs.as_radians() * self.metres)
    }
}

impl ::std::ops::Mul<Length> for Angle {
    type Output = Length;

    fn mul(self, rhs: Length) -> Length {
        Length::from_metres(self.as_radians() * rhs.metres)
    }
}
