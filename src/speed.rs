use std::time::Duration;

use crate::{impl_measurement, Length, Measurement};

#[derive(PartialEq, PartialOrd, Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
/// A speed.
///
/// It primarely exists to unambigously represent a speed as opposed to a bare
/// [f64] (which could be anything and in any unit).
/// It allows conversion to or from metres/second, kilometres/hour and knots.
///
/// # Examples
///
/// ```
/// use jord::Speed;
///
/// assert_eq!(1.0, Speed::from_metres_per_second(1.0).as_metres_per_second());
/// assert_eq!(1.852, Speed::from_knots(1.0).as_kilometres_per_hour());
/// assert_eq!(0.2777777777777778, Speed::from_kilometres_per_hour(1.0).as_metres_per_second());
/// ```
///
/// [Speed] implements many traits, including [Add](::std::ops::Add), [Sub](::std::ops::Sub),
/// [Mul](::std::ops::Mul) and [Div](::std::ops::Div), among others.
///
/// # Speed from distance and time
///
/// ```
/// use jord::{Length, Speed};
/// use std::time::Duration;
///
/// assert_eq!(
///     Speed::from_metres_per_second(1.0),
///     Length::from_metres(1.0) / Duration::from_secs(1)
/// );
///
/// assert_eq!(
///     Speed::from_knots(1.0),
///     Length::from_nautical_miles(1.0) / Duration::from_secs(3600)
/// );
/// ```
///
/// # Distance travelled at speed over time
///
/// ```
/// use jord::{Length, Speed};
/// use std::time::Duration;
///
/// assert_eq!(
///     Length::from_metres(2.0),
///     Speed::from_metres_per_second(1.0) * Duration::from_secs(2)
/// );
///
/// assert_eq!(
///     Length::from_nautical_miles(2.0),
///     (Speed::from_knots(1.0) * Duration::from_secs(7200)).round_mm()
/// );
///
/// ```
pub struct Speed {
    mps: f64,
}

impl Speed {
    const KPH_TO_MPS: f64 = 1_000.0 / 3_600.0;

    const KNOTS_TO_MPS: f64 = 1_852.0 / 3_600.0;

    /// Zero speed.
    pub const ZERO: Speed = Speed { mps: 0.0 };

    /// Creates a speed from a floating point value in metres per second.
    pub const fn from_metres_per_second(mps: f64) -> Self {
        Speed { mps }
    }

    /// Creates a speed from a floating point value in kilometres per hour.
    pub fn from_kilometres_per_hour(kph: f64) -> Self {
        Speed::from_metres_per_second(kph * Self::KPH_TO_MPS)
    }

    /// Creates a speed from a floating point value in knots.
    pub fn from_knots(knots: f64) -> Self {
        Speed::from_metres_per_second(knots * Self::KNOTS_TO_MPS)
    }

    /// Converts this speed to a floating point value in metres per second.
    #[inline]
    pub const fn as_metres_per_second(&self) -> f64 {
        self.mps
    }

    /// Converts this speed to a floating point value in kilometres per hour.
    pub fn as_kilometres_per_hour(&self) -> f64 {
        self.mps / Self::KPH_TO_MPS
    }

    /// Converts this speed to a floating point value in knots.
    pub fn as_knots(&self) -> f64 {
        self.mps / Self::KNOTS_TO_MPS
    }
}

impl Measurement for Speed {
    fn from_default_unit(amount: f64) -> Self {
        Speed::from_metres_per_second(amount)
    }

    #[inline]
    fn as_default_unit(&self) -> f64 {
        self.mps
    }
}

impl_measurement! { Speed }

impl ::std::ops::Div<Duration> for Length {
    type Output = Speed;

    fn div(self, rhs: Duration) -> Speed {
        let mps = self.as_metres() / rhs.as_secs_f64();
        Speed::from_metres_per_second(mps)
    }
}

impl ::std::ops::Mul<Duration> for Speed {
    type Output = Length;

    fn mul(self, rhs: Duration) -> Length {
        let metres = self.as_metres_per_second() * rhs.as_secs_f64();
        Length::from_metres(metres)
    }
}

#[cfg(test)]
mod tests {

    use crate::{Length, Speed};
    use std::time::Duration;

    #[test]
    fn conversions() {
        assert_eq!(1.852, Speed::from_knots(1.0).as_kilometres_per_hour());
        assert_eq_e6(0.514444, Speed::from_knots(1.0).as_metres_per_second());
        assert_eq_e6(
            0.277778,
            Speed::from_kilometres_per_hour(1.0).as_metres_per_second(),
        );
        assert_eq_e6(0.539957, Speed::from_kilometres_per_hour(1.0).as_knots());
        assert_eq_e6(
            3.6,
            Speed::from_metres_per_second(1.0).as_kilometres_per_hour(),
        );
        assert_eq_e6(1.943844, Speed::from_metres_per_second(1.0).as_knots());

        fn assert_eq_e6(expected: f64, actual: f64) {
            let d = (expected - actual).abs();
            assert!(d < 1e-6, "expected {} but was {}", expected, actual);
        }
    }

    #[test]
    fn std_ops() {
        assert_eq!(
            Speed::from_metres_per_second(2.0),
            2.0 * Speed::from_metres_per_second(1.0)
        );
        assert_eq!(
            Speed::from_metres_per_second(2.0),
            Speed::from_metres_per_second(1.0) + Speed::from_metres_per_second(1.0)
        );
        assert_eq!(
            Speed::from_metres_per_second(0.0),
            Speed::from_metres_per_second(1.0) - Speed::from_metres_per_second(1.0)
        );
        assert_eq!(
            Speed::from_metres_per_second(1.0),
            Length::from_metres(1.0) / Duration::from_secs(1)
        );
        assert_eq!(
            Length::from_metres(1.0),
            Speed::from_metres_per_second(1.0) * Duration::from_secs(1)
        );
    }
}
