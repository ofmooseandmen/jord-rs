use std::f64::consts::PI;
use std::ops::Add;
use std::ops::Div;
use std::ops::Neg;
use std::ops::Sub;

use crate::internal::modulo;
use crate::measure::*;
use crate::FixedLength;

pub trait Angle
where
    Self: Sized,
    Self: Copy + Clone,
    Self: PartialEq + PartialOrd,
    Self: Neg<Output = Self>,
    Self: Add<Self, Output = Self>,
    Self: Sub<Self, Output = Self>,
    Self: Div<Self, Output = f64>,
{
    type Length;
    fn cos(self) -> f64;
    fn sin(self) -> f64;
    fn atan2(y: f64, x: f64) -> Self;
    fn abs(self) -> Self;
    fn zero() -> Self;
    fn quarter_circle() -> Self;
    fn half_circle() -> Self;
    fn full_circle() -> Self;
    fn is_within(self, low: Self, high: Self) -> bool;
    fn central(length: Self::Length, radius: Self::Length) -> Self;
    fn arc_length(self, radius: Self::Length) -> Self::Length;
    fn normalise(self, other: Self) -> Self;
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct FixedAngle {
    microarcseconds: i64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DmsError {
    InvalidArcMinutes,
    InvalidArcSeconds,
}

impl FixedAngle {
    const DG_TO_UAS: f64 = 3_600_000_000.0;

    pub fn from_decimal_degrees(dec: f64) -> Self {
        let uas = (dec * FixedAngle::DG_TO_UAS).round() as i64;
        FixedAngle {
            microarcseconds: uas,
        }
    }

    pub fn from_dms(degs: i64, mins: i64, secs: f64) -> Result<Self, DmsError> {
        if mins < 0 || mins > 59 {
            Err(DmsError::InvalidArcMinutes)
        } else if secs < 0.0 || secs >= 60.0 {
            Err(DmsError::InvalidArcSeconds)
        } else {
            let d = degs.abs() as f64 + (mins as f64 / 60.0) + (secs / 3600.0);
            if degs < 0 {
                Ok(FixedAngle::from_decimal_degrees(-d))
            } else {
                Ok(FixedAngle::from_decimal_degrees(d))
            }
        }
    }

    pub fn from_radians(rads: f64) -> Self {
        FixedAngle::from_decimal_degrees(rads / PI * 180.0)
    }

    pub fn clockwise_difference(self, other: Self) -> Self {
        let d = FixedAngle::cd(self.to_decimal_degrees(), other.to_decimal_degrees());
        FixedAngle::from_decimal_degrees(d)
    }

    fn cd(d1: f64, d2: f64) -> f64 {
        if d2 < d1 {
            FixedAngle::cd(d1, d2 + 360.0)
        } else {
            d2 - d1
        }
    }

    pub fn is_negative(self) -> bool {
        self.microarcseconds < 0
    }

    pub fn asin(a: f64) -> Self {
        FixedAngle::from_radians(a.asin())
    }

    pub fn to_radians(self) -> f64 {
        self.to_decimal_degrees() * PI / 180.0
    }

    pub fn to_decimal_degrees(self) -> f64 {
        self.microarcseconds as f64 / FixedAngle::DG_TO_UAS
    }

    pub fn get_degrees(self) -> i64 {
        let d = FixedAngle::field(self, FixedAngle::DG_TO_UAS, 360.0) as i64;
        if self.microarcseconds < 0 {
            -d
        } else {
            d
        }
    }

    pub fn get_arc_minutes(self) -> u8 {
        FixedAngle::field(self, 60000000.0, 60.0) as u8
    }

    pub fn get_arc_seconds(self) -> u8 {
        FixedAngle::field(self, 1000000.0, 60.0) as u8
    }

    pub fn get_arc_milliseconds(self) -> u8 {
        FixedAngle::field(self, 1000.0, 1000.0) as u8
    }

    fn field(self, div: f64, modu: f64) -> u64 {
        modulo(self.microarcseconds.abs() as f64 / div, modu) as u64
    }
}

impl Angle for FixedAngle {
    type Length = FixedLength;
    fn zero() -> Self {
        FixedAngle { microarcseconds: 0 }
    }

    fn quarter_circle() -> Self {
        FixedAngle {
            microarcseconds: 324_000_000_000,
        }
    }

    fn half_circle() -> Self {
        FixedAngle {
            microarcseconds: 648_000_000_000,
        }
    }

    fn full_circle() -> Self {
        FixedAngle {
            microarcseconds: 1_296_000_000_000,
        }
    }

    fn abs(self) -> Self {
        FixedAngle {
            microarcseconds: self.microarcseconds.abs(),
        }
    }

    fn cos(self) -> f64 {
        self.to_radians().cos()
    }

    fn sin(self) -> f64 {
        self.to_radians().sin()
    }

    fn atan2(y: f64, x: f64) -> Self {
        FixedAngle::from_radians(y.atan2(x))
    }

    fn is_within(self, low: Self, high: Self) -> bool {
        self.microarcseconds >= low.microarcseconds && self.microarcseconds <= high.microarcseconds
    }

    fn central(length: FixedLength, radius: FixedLength) -> Self {
        FixedAngle::from_radians(length / radius)
    }

    fn arc_length(self, radius: FixedLength) -> FixedLength {
        FixedLength::from_metres(radius.to_metres() * self.to_radians())
    }

    fn normalise(self, other: Self) -> Self {
        let a = self + other;
        let dec = modulo(a.to_decimal_degrees(), 360.0);
        FixedAngle::from_decimal_degrees(dec)
    }
}

impl Angle for f64 {
    type Length = f64;
    fn zero() -> f64 {
        0.0
    }

    fn quarter_circle() -> f64 {
        90.0
    }

    fn half_circle() -> f64 {
        180.0
    }

    fn full_circle() -> f64 {
        360.0
    }

    fn abs(self) -> f64 {
        self.abs()
    }

    fn cos(self) -> f64 {
        self.to_radians().cos()
    }

    fn sin(self) -> f64 {
        self.to_radians().sin()
    }

    fn atan2(y: f64, x: f64) -> Self {
        (y.atan2(x)).to_degrees()
    }

    fn is_within(self, low: Self, high: Self) -> bool {
        self >= low && self <= high
    }

    fn central(length: f64, radius: f64) -> Self {
        (length / radius).to_radians()
    }

    fn arc_length(self, radius: f64) -> Self {
        radius * self.to_radians()
    }
    fn normalise(self, other: Self) -> Self {
        let a = self + other;
        modulo(a, 360.0)
    }
}

impl Measure for FixedAngle {
    fn to_unit(self) -> f64 {
        self.to_decimal_degrees()
    }

    fn from_unit(amount: f64) -> Self {
        FixedAngle::from_decimal_degrees(amount)
    }

    fn to_resolution(self) -> i64 {
        self.microarcseconds
    }

    fn from_resolution(amount: i64) -> Self {
        FixedAngle {
            microarcseconds: amount,
        }
    }
}

impl_measure! { FixedAngle }

#[cfg(test)]
mod fixed_tests {
    use super::*;

    #[test]
    fn add_angles() {
        let a1 = FixedAngle::from_decimal_degrees(55.6058333);
        let a2 = FixedAngle::from_decimal_degrees(10.0);
        assert_eq!(FixedAngle::from_decimal_degrees(65.6058333), a1 + a2);
    }

    #[test]
    fn subtract_angles() {
        let a1 = FixedAngle::from_decimal_degrees(5.0);
        let a2 = FixedAngle::from_decimal_degrees(55.6058333);
        assert_eq!(FixedAngle::from_decimal_degrees(-50.6058333), a1 - a2);
    }

    #[test]
    fn normalise_370_to_0_360() {
        let a = FixedAngle::from_decimal_degrees(370.0);
        assert_eq!(
            FixedAngle::from_decimal_degrees(10.0),
            a.normalise(FixedAngle::full_circle())
        );
    }

    #[test]
    fn normalise_350_to_0_360() {
        let a = FixedAngle::from_decimal_degrees(350.0);
        assert_eq!(
            FixedAngle::from_decimal_degrees(350.0),
            a.normalise(FixedAngle::full_circle())
        );
    }
}

#[cfg(test)]
mod resolution_tests {
    use super::*;

    #[test]
    fn one_microarcsecond() {
        assert_eq!(
            FixedAngle::from_decimal_degrees(60.0),
            FixedAngle::from_decimal_degrees(59.9999999999)
        );
        assert_ne!(
            FixedAngle::from_decimal_degrees(60.0),
            FixedAngle::from_decimal_degrees(59.999999998)
        );
    }

    #[test]
    fn one_arcmillisecond() {
        let a = FixedAngle::from_decimal_degrees(1.0 / 3600000.0);
        assert_eq!(0, a.get_degrees());
        assert_eq!(0, a.get_arc_minutes());
        assert_eq!(0, a.get_arc_seconds());
        assert_eq!(1, a.get_arc_milliseconds());
    }

    #[test]
    fn one_arcsecond() {
        let a = FixedAngle::from_decimal_degrees(1000.0 / 3600000.0);
        assert_eq!(0, a.get_degrees());
        assert_eq!(0, a.get_arc_minutes());
        assert_eq!(1, a.get_arc_seconds());
        assert_eq!(0, a.get_arc_milliseconds());
    }

    #[test]
    fn one_arcminute() {
        let a = FixedAngle::from_decimal_degrees(60000.0 / 3600000.0);
        assert_eq!(0, a.get_degrees());
        assert_eq!(1, a.get_arc_minutes());
        assert_eq!(0, a.get_arc_seconds());
        assert_eq!(0, a.get_arc_milliseconds());
    }

    #[test]
    fn one_degrees() {
        let a = FixedAngle::from_decimal_degrees(1.0);
        assert_eq!(1, a.get_degrees());
        assert_eq!(0, a.get_arc_minutes());
        assert_eq!(0, a.get_arc_seconds());
        assert_eq!(0, a.get_arc_milliseconds());
    }

    #[test]
    fn positve_value() {
        let a = FixedAngle::from_decimal_degrees(154.9150300);
        assert_eq!(154, a.get_degrees());
        assert_eq!(54, a.get_arc_minutes());
        assert_eq!(54, a.get_arc_seconds());
        assert_eq!(108, a.get_arc_milliseconds());
    }

    #[test]
    fn negative_value() {
        let a = FixedAngle::from_decimal_degrees(-154.915);
        assert_eq!(-154, a.get_degrees());
        assert_eq!(54, a.get_arc_minutes());
        assert_eq!(54, a.get_arc_seconds());
        assert_eq!(0, a.get_arc_milliseconds());
    }
}

#[cfg(test)]
mod arc_length_tests {
    use super::*;

    #[test]
    fn central_angle_1_microarcsecond() {
        let a = FixedAngle::from_decimal_degrees(1.0 / 3600000000.0)
            .arc_length(FixedLength::from_kilometres(10000.0));
        assert_eq!(FixedLength::from_metres(4.8e-5), a);
    }

    #[test]
    fn central_angle_0_6_microarcsecond() {
        let a = FixedAngle::from_decimal_degrees(0.6 / 3600000000.0)
            .arc_length(FixedLength::from_kilometres(10000.0));
        assert_eq!(FixedLength::from_metres(4.8e-5), a);
    }

    #[test]
    fn central_angle_0_4_microarcsecond() {
        let a = FixedAngle::from_decimal_degrees(0.4 / 3600000000.0)
            .arc_length(FixedLength::from_kilometres(10000.0));
        assert_eq!(FixedLength::zero(), a);
    }
}

#[cfg(test)]
mod clockwise_difference_tests {
    use super::*;

    #[test]
    fn returns_0_when_both_angle_are_equal() {
        let a = FixedAngle::from_decimal_degrees(154.0);
        assert_eq!(FixedAngle::zero(), a.clockwise_difference(a));
    }

    #[test]
    fn return_the_diff_between_2_angles_clockwise() {
        assert_eq!(
            FixedAngle::from_decimal_degrees(10.0),
            FixedAngle::zero().clockwise_difference(FixedAngle::from_decimal_degrees(10.0))
        );
        assert_eq!(
            FixedAngle::from_decimal_degrees(350.0),
            FixedAngle::zero().clockwise_difference(FixedAngle::from_decimal_degrees(-10.0))
        );
        assert_eq!(
            FixedAngle::from_decimal_degrees(20.0),
            FixedAngle::from_decimal_degrees(350.0)
                .clockwise_difference(FixedAngle::from_decimal_degrees(10.0))
        );
        assert_eq!(
            FixedAngle::from_decimal_degrees(20.0),
            FixedAngle::from_decimal_degrees(350.0)
                .clockwise_difference(FixedAngle::from_decimal_degrees(370.0))
        );
    }
}
