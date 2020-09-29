use crate::measure::*;
use std::ops::Add;
use std::ops::Neg;
use std::ops::Sub;

pub trait Length
where
    Self: Sized,
    Self: Copy + Clone,
    Self: PartialEq + PartialOrd,
    Self: Neg<Output = Self>,
    Self: Add<Self, Output = Self>,
    Self: Sub<Self, Output = Self>,
{
    fn is_zero(self) -> bool;
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct FixedLength {
    micrometres: i64,
}

impl FixedLength {
    const M_TO_UM: f64 = 1_000_000.0;

    const FT_TO_M: f64 = 0.3048;

    const NM_TO_M: f64 = 1852.0;

    const KM_TO_M: f64 = 1000.0;

    pub const fn zero() -> Self {
        FixedLength { micrometres: 0 }
    }

    pub const fn min_value() -> Self {
        FixedLength {
            micrometres: i64::min_value(),
        }
    }

    pub const fn max_value() -> Self {
        FixedLength {
            micrometres: i64::max_value(),
        }
    }

    pub const fn from_micrometres(um: i64) -> Self {
        FixedLength { micrometres: um }
    }

    pub fn from_metres(m: f64) -> Self {
        let um = (m * FixedLength::M_TO_UM).round() as i64;
        FixedLength { micrometres: um }
    }

    pub fn from_kilometres(k: f64) -> Self {
        FixedLength::from_metres(k * FixedLength::KM_TO_M)
    }

    pub fn from_feet(ft: f64) -> Self {
        FixedLength::from_metres(ft * FixedLength::FT_TO_M)
    }

    pub fn from_nautical_miles(nm: f64) -> Self {
        FixedLength::from_metres(nm * FixedLength::NM_TO_M)
    }

    pub fn as_metres(self) -> f64 {
        (self.micrometres as f64) / FixedLength::M_TO_UM
    }

    pub fn as_kilometres(self) -> f64 {
        let m = self.as_metres();
        m / FixedLength::KM_TO_M
    }

    pub fn as_feet(self) -> f64 {
        let m = self.as_metres();
        m / FixedLength::FT_TO_M
    }

    pub fn as_nautical_miles(self) -> f64 {
        let m = self.as_metres();
        m / FixedLength::NM_TO_M
    }
}

impl Length for FixedLength {
    fn is_zero(self) -> bool {
        self.micrometres == 0
    }
}

impl Measure for FixedLength {
    fn to_unit(self) -> f64 {
        self.as_metres()
    }

    fn from_unit(amount: f64) -> Self {
        FixedLength::from_metres(amount)
    }

    fn to_resolution(self) -> i64 {
        self.micrometres
    }

    fn from_resolution(amount: i64) -> Self {
        FixedLength {
            micrometres: amount,
        }
    }
}

impl_measure! { FixedLength }

impl Length for f64 {
    fn is_zero(self) -> bool {
        self == 0.0
    }
}

#[cfg(test)]
mod conversion_tests {
    use super::*;

    #[test]
    fn max_value() {
        assert_eq!(9223372036.854774, FixedLength::max_value().as_kilometres());
    }

    #[test]
    fn metres_to_kilometres() {
        let l = FixedLength::from_metres(1000.0);
        assert_eq!(1.0, l.as_kilometres());
    }

    #[test]
    fn metres_to_nautical_miles() {
        let l = FixedLength::from_metres(1000.0);
        assert_eq!(0.5399568034557235, l.as_nautical_miles());
    }

    #[test]
    fn kilometres_to_nautical_miles() {
        let l = FixedLength::from_kilometres(1000.0);
        assert_eq!(539.9568034557235, l.as_nautical_miles());
    }

    #[test]
    fn nautical_miles_to_metres() {
        let l = FixedLength::from_nautical_miles(10.5);
        assert_eq!(19446.0, l.as_metres());
    }

    #[test]
    fn nautical_miles_to_kilometres() {
        let l = FixedLength::from_nautical_miles(10.5);
        assert_eq!(19.446, l.as_kilometres());
    }

    #[test]
    fn feet_to_metres() {
        let l = FixedLength::from_feet(25000.0);
        assert_eq!(7620.0, l.as_metres());
    }

    #[test]
    fn metres_to_feet() {
        let l = FixedLength::from_metres(7620.0);
        assert_eq!(25000.0, l.as_feet());
    }
}

#[cfg(test)]
mod resolution_tests {
    use super::*;

    #[test]
    fn one_metre() {
        let l = FixedLength::from_metres(1.0);
        assert_eq!(1.0, l.as_metres());
    }

    #[test]
    fn one_kilometre() {
        let l = FixedLength::from_kilometres(1.0);
        assert_eq!(1.0, l.as_kilometres());
    }

    #[test]
    fn one_nautical_mile() {
        let l = FixedLength::from_nautical_miles(1.0);
        assert_eq!(1.0, l.as_nautical_miles());
    }

    #[test]
    fn one_feet() {
        let l = FixedLength::from_feet(1.0);
        assert_eq!(1.0, l.as_feet());
    }

    #[test]
    fn one_micrometre() {
        let l1 = FixedLength::from_metres(1.000001);
        let l2 = FixedLength::from_metres(1.000002);
        let l3 = FixedLength::from_metres(1.0000011);
        assert_eq!(l1, l3);
        assert_ne!(l1, l2);
        assert_ne!(l2, l3);
    }
}
