use std::time::Duration;

use crate::{impl_measurement, Length, Measurement};

#[derive(PartialEq, PartialOrd, Clone, Copy, Debug, Default)]
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

    /// Creates a speed by calculating the average speed required to covered the given distance in the given duration.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::time::Duration;
    /// use jord::{Length, Speed};
    ///
    /// assert_eq!(
    ///     Speed::from_metres_per_second(1.0),
    ///     Speed::from_average(Length::from_metres(1.0), Duration::from_secs(1))
    /// );
    /// assert_eq!(
    ///     Speed::from_knots(1.0),
    ///     Speed::from_average(Length::from_nautical_miles(1.0), Duration::from_secs(3600))
    /// );
    /// ```
    pub fn from_average(distance: Length, duration: Duration) -> Self {
        let mps = distance.as_metres() / duration.as_secs_f64();
        Speed::from_metres_per_second(mps)
    }

    /// Converts this speed to a floating point value in metres per second.
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

    fn as_default_unit(&self) -> f64 {
        self.mps
    }
}

impl_measurement! { Speed }
