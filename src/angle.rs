// Copyright: (c) 2020 Cedric Liegeois
// License: BSD3
use std::f64::consts::PI;

use crate::internal::modulo;
use crate::Measure;

/// A signed 'Angle' with a resolution of a microarcsecond.
/// When used as a latitude/longitude this roughly translate to a precision
/// of 0.03 millimetres at the equator.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct Angle {
    microarcseconds: i64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DmsError {
    InvalidArcMinutes,
    InvalidArcSeconds,
}

impl Angle {
    const DG_TO_UAS: f64 = 3_600_000_000.0;

    pub fn zero() -> Self {
        Angle { microarcseconds: 0 }
    }

    pub fn from_decimal_degrees(dec: f64) -> Self {
        let uas = (dec * Angle::DG_TO_UAS).round() as i64;
        Angle {
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
                Ok(Angle::from_decimal_degrees(-d))
            } else {
                Ok(Angle::from_decimal_degrees(d))
            }
        }
    }

    pub fn from_radians(rads: f64) -> Self {
        Angle::from_decimal_degrees(rads / PI * 180.0)
    }

    pub fn as_radians(self) -> f64 {
        self.as_decimal_degrees() * PI / 180.0
    }

    pub fn as_decimal_degrees(self) -> f64 {
        self.microarcseconds as f64 / Angle::DG_TO_UAS
    }

    pub fn whole_degrees(self) -> i64 {
        let d = Angle::field(self, Angle::DG_TO_UAS, 360.0) as i64;
        if self.microarcseconds < 0 {
            -d
        } else {
            d
        }
    }

    pub fn arc_minutes(self) -> u8 {
        Angle::field(self, 60000000.0, 60.0) as u8
    }

    pub fn arc_seconds(self) -> u8 {
        Angle::field(self, 1000000.0, 60.0) as u8
    }

    pub fn arc_milliseconds(self) -> u8 {
        Angle::field(self, 1000.0, 1000.0) as u8
    }

    fn field(self, div: f64, modu: f64) -> u64 {
        modulo(self.microarcseconds.abs() as f64 / div, modu) as u64
    }
}

impl Measure for Angle {
    fn to_unit(self) -> f64 {
        self.as_decimal_degrees()
    }

    fn from_unit(amount: f64) -> Self {
        Angle::from_decimal_degrees(amount)
    }

    fn to_resolution(self) -> i64 {
        self.microarcseconds
    }

    fn from_resolution(amount: i64) -> Self {
        Angle {
            microarcseconds: amount,
        }
    }
}

impl_measure! { Angle }

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_microarcsecond() {
        assert_eq!(
            Angle::from_decimal_degrees(60.0),
            Angle::from_decimal_degrees(59.9999999999)
        );
        assert_ne!(
            Angle::from_decimal_degrees(60.0),
            Angle::from_decimal_degrees(59.999999998)
        );
    }

    #[test]
    fn one_arcmillisecond() {
        let a = Angle::from_decimal_degrees(1.0 / 3600000.0);
        assert_eq!(0, a.whole_degrees());
        assert_eq!(0, a.arc_minutes());
        assert_eq!(0, a.arc_seconds());
        assert_eq!(1, a.arc_milliseconds());
    }

    #[test]
    fn one_arcsecond() {
        let a = Angle::from_decimal_degrees(1000.0 / 3600000.0);
        assert_eq!(0, a.whole_degrees());
        assert_eq!(0, a.arc_minutes());
        assert_eq!(1, a.arc_seconds());
        assert_eq!(0, a.arc_milliseconds());
    }

    #[test]
    fn one_arcminute() {
        let a = Angle::from_decimal_degrees(60000.0 / 3600000.0);
        assert_eq!(0, a.whole_degrees());
        assert_eq!(1, a.arc_minutes());
        assert_eq!(0, a.arc_seconds());
        assert_eq!(0, a.arc_milliseconds());
    }

    #[test]
    fn one_degrees() {
        let a = Angle::from_decimal_degrees(1.0);
        assert_eq!(1, a.whole_degrees());
        assert_eq!(0, a.arc_minutes());
        assert_eq!(0, a.arc_seconds());
        assert_eq!(0, a.arc_milliseconds());
    }

    #[test]
    fn positve_value() {
        let a = Angle::from_decimal_degrees(154.9150300);
        assert_eq!(154, a.whole_degrees());
        assert_eq!(54, a.arc_minutes());
        assert_eq!(54, a.arc_seconds());
        assert_eq!(108, a.arc_milliseconds());
    }

    #[test]
    fn negative_value() {
        let a = Angle::from_decimal_degrees(-154.915);
        assert_eq!(-154, a.whole_degrees());
        assert_eq!(54, a.arc_minutes());
        assert_eq!(54, a.arc_seconds());
        assert_eq!(0, a.arc_milliseconds());
    }
}
