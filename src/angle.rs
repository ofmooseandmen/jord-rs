use crate::{impl_measurement, Measurement};
use std::f64::consts::PI;

#[derive(PartialEq, PartialOrd, Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))] // codecov:ignore:this
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
    /// Zero angle.
    pub const ZERO: Angle = Angle { radians: 0.0 };

    /// 90 degrees angle.
    pub const QUARTER_CIRCLE: Angle = Angle { radians: PI / 2.0 };

    /// -90 degrees angle.
    pub const NEG_QUARTER_CIRCLE: Angle = Angle { radians: -PI / 2.0 };

    /// 180 degrees angle.
    pub const HALF_CIRCLE: Angle = Angle { radians: PI };

    /// -180 degrees angle.
    pub const NEG_HALF_CIRCLE: Angle = Angle { radians: -PI };

    /// 360 degrees angle.
    pub const FULL_CIRCLE: Angle = Angle { radians: 2.0 * PI };

    /// `f64::EPSILON` radians.
    pub(crate) const DBL_EPSILON: Angle = Angle {
        radians: f64::EPSILON,
    };

    /// Converts this angle to a floating point value in degrees.
    pub fn as_degrees(&self) -> f64 {
        self.radians.to_degrees()
    }

    /// Converts this angle to a floating point value in radians.
    #[inline]
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

    /// Returns a new angle that is the absolute value of this angle.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Angle;
    ///
    /// assert_eq!(Angle::from_degrees(45.0), Angle::from_degrees(-45.0).abs());
    /// ```
    pub fn abs(&self) -> Self {
        Angle {
            radians: self.radians.abs(),
        }
    }

    /// Returns a new angle by normalising this angle to the range [0, 360) degrees.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Angle;
    ///
    /// assert_eq!(Angle::from_degrees(359.0), Angle::from_degrees(-361.0).normalised());
    /// assert_eq!(Angle::from_degrees(358.0), Angle::from_degrees(-2.0).normalised());
    /// assert_eq!(Angle::from_degrees(154.0), Angle::from_degrees(154.0).normalised());
    /// assert_eq!(Angle::ZERO, Angle::from_degrees(360.0).normalised());
    /// ```
    pub fn normalised(&self) -> Self {
        self.normalised_to(Self::FULL_CIRCLE)
    }

    /// Returns a new angle by normalising this angle to the range [0, `max`) degrees.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Angle;
    ///
    /// assert_eq!(
    ///     Angle::from_degrees(179.0),
    ///     Angle::from_degrees(-181.0).normalised_to(Angle::HALF_CIRCLE).round_d7()
    /// );
    /// assert_eq!(
    ///     Angle::from_degrees(1.0),
    ///     Angle::from_degrees(181.0).normalised_to(Angle::HALF_CIRCLE).round_d7()
    /// );
    /// assert_eq!(
    ///     Angle::from_degrees(154.0),
    ///     Angle::from_degrees(154.0).normalised_to(Angle::HALF_CIRCLE)
    /// );
    /// assert_eq!(
    ///     Angle::ZERO,
    ///     Angle::from_degrees(180.0).normalised_to(Angle::HALF_CIRCLE)
    /// );
    /// ```
    pub fn normalised_to(&self, max: Angle) -> Angle {
        if self.radians >= 0.0 && self.radians < max.radians {
            *self
        } else {
            let res = self.radians % max.radians;
            if res < 0.0 {
                Self::from_radians(res + max.radians)
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
    /// an Earth latitude/longtiude this is approximately 111 millimetres at the equator.
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
    /// an Earth latitude/longtiude this is approximately 11.1 millimetres at the equator.
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

    #[inline]
    fn as_default_unit(&self) -> f64 {
        self.as_radians()
    }
}

impl_measurement! { Angle }

#[cfg(feature = "uom")]
impl From<uom::si::f64::Angle> for Angle {
    fn from(value: uom::si::f64::Angle) -> Self {
        Self::from_radians(value.get::<uom::si::angle::radian>())
    }
}

#[cfg(feature = "uom")]
impl From<Angle> for uom::si::f64::Angle {
    fn from(value: Angle) -> Self {
        Self::new::<uom::si::angle::radian>(value.as_radians())
    }
}

#[cfg(test)]
mod tests {

    use std::f64::consts::PI;

    use crate::Angle;

    #[test]
    fn conversions() {
        assert_eq!(PI, Angle::from_degrees(180.0).as_radians());
        assert_eq!(180.0, Angle::from_radians(PI).as_degrees());
    }

    #[test]
    fn std_ops() {
        assert_eq!(Angle::from_degrees(2.0), 2.0 * Angle::from_degrees(1.0));
        assert_eq!(
            Angle::from_degrees(2.0),
            Angle::from_degrees(1.0) + Angle::from_degrees(1.0)
        );
        assert_eq!(
            Angle::from_degrees(0.0),
            Angle::from_degrees(1.0) - Angle::from_degrees(1.0)
        );
    }

    #[test]
    fn normalised() {
        assert_eq!(
            Angle::from_degrees(359.0),
            Angle::from_degrees(-361.0).normalised()
        );
        assert_eq!(
            Angle::from_degrees(358.0),
            Angle::from_degrees(-2.0).normalised()
        );
        assert_eq!(
            Angle::from_degrees(154.0),
            Angle::from_degrees(154.0).normalised()
        );
        assert_eq!(Angle::ZERO, Angle::from_degrees(360.0).normalised());
    }

    #[test]
    fn normalised_to() {
        assert_eq!(
            Angle::from_degrees(179.0),
            Angle::from_degrees(-181.0)
                .normalised_to(Angle::HALF_CIRCLE)
                .round_d7()
        );
        assert_eq!(
            Angle::from_degrees(1.0),
            Angle::from_degrees(181.0)
                .normalised_to(Angle::HALF_CIRCLE)
                .round_d7()
        );
        assert_eq!(
            Angle::from_degrees(154.0),
            Angle::from_degrees(154.0).normalised_to(Angle::HALF_CIRCLE)
        );
        assert_eq!(
            Angle::ZERO,
            Angle::from_degrees(180.0).normalised_to(Angle::HALF_CIRCLE)
        );
    }

    #[cfg(feature = "uom")]
    #[test]
    fn uom() {
        let speed = Angle::from_radians(1.0);
        let uom = uom::si::f64::Angle::from(speed);
        let roundtrip = Angle::from(uom);
        assert_eq!(speed, roundtrip);
    }
}
