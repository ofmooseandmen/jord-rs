9use crate::{impl_measurement, Angle, Measurement};

#[derive(PartialEq, PartialOrd, Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))] // codecov:ignore:this
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

    /// Maximum length.
    pub const MAX: Length = Length { metres: f64::MAX };

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
    #[inline]
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

    /// Computes the absolute value of this length.
    pub fn abs(&self) -> Self {
        if self.metres >= 0.0 {
            *self
        } else {
            Self::from_metres(-self.metres)
        }
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

    #[inline]
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

#[cfg(feature = "uom")]
impl From<uom::si::f64::Length> for Length {
    fn from(value: uom::si::f64::Length) -> Self {
        Self::from_metres(value.get::<uom::si::length::meter>())
    }
}

#[cfg(feature = "uom")]
impl From<Length> for uom::si::f64::Length {
    fn from(value: Length) -> Self {
        Self::new::<uom::si::length::meter>(value.as_metres())
    }
}

#[cfg(test)]
mod tests {
    use crate::{Angle, Length};

    #[test]
    fn units() {
        assert_eq!(100.0, Length::from_metres(100.0).as_metres());
        assert_eq!(3900.0, Length::from_metres(3900.0).as_metres());

        assert_eq!(10.0, Length::from_metres(18520.0).as_nautical_miles());
        assert_eq!(18520.0, Length::from_nautical_miles(10.0).as_metres());

        assert_eq!(2.5, Length::from_metres(2500.0).as_kilometres());
        assert_eq!(2500.0, Length::from_kilometres(2.5).as_metres());

        assert_eq!(1000.0, Length::from_metres(304.8).as_feet());
        assert_eq!(304.8, Length::from_feet(1000.0).as_metres());
    }

    #[test]
    fn mul() {
        assert_eq!(
            Length::from_metres(100.0),
            Length::from_metres(50.0) * Angle::from_radians(2.0)
        );
        assert_eq!(
            Length::from_metres(100.0),
            Angle::from_radians(2.0) * Length::from_metres(50.0)
        );
    }

    #[test]
    fn round() {
        assert_eq!(
            Length::from_metres(50.0),
            Length::from_metres(50.1).round_m()
        );
        assert_eq!(
            Length::from_metres(50.2),
            Length::from_metres(50.15).round_dm()
        );
        assert_eq!(
            Length::from_metres(50.16),
            Length::from_metres(50.157).round_cm()
        );
        assert_eq!(
            Length::from_metres(50.157),
            Length::from_metres(50.1574).round_mm()
        );
    }

    #[cfg(feature = "uom")]
    #[test]
    fn uom() {
        let length = Length::from_metres(1.0);
        let uom = uom::si::f64::Length::from(length);
        let roundtrip = Length::from(uom);
        assert_eq!(length, roundtrip);
    }
}
