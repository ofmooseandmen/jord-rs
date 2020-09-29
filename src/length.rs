use crate::Measure;

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct Length {
    micrometres: i64,
}

impl Length {
    const M_TO_UM: f64 = 1_000_000.0;

    const FT_TO_M: f64 = 0.3048;

    const NM_TO_M: f64 = 1852.0;

    const KM_TO_M: f64 = 1000.0;

    pub const fn zero() -> Self {
        Length { micrometres: 0 }
    }

    pub const fn from_micrometres(um: i64) -> Self {
        Length { micrometres: um }
    }

    pub fn from_metres(m: f64) -> Self {
        let um = (m * Length::M_TO_UM).round() as i64;
        Length { micrometres: um }
    }

    pub fn from_kilometres(k: f64) -> Self {
        Length::from_metres(k * Length::KM_TO_M)
    }

    pub fn from_feet(ft: f64) -> Self {
        Length::from_metres(ft * Length::FT_TO_M)
    }

    pub fn from_nautical_miles(nm: f64) -> Self {
        Length::from_metres(nm * Length::NM_TO_M)
    }

    pub fn micrometres(self) -> i64 {
        self.micrometres
    }

    pub fn as_metres(self) -> f64 {
        (self.micrometres as f64) / Length::M_TO_UM
    }

    pub fn as_kilometres(self) -> f64 {
        let m = self.as_metres();
        m / Length::KM_TO_M
    }

    pub fn as_feet(self) -> f64 {
        let m = self.as_metres();
        m / Length::FT_TO_M
    }

    pub fn as_nautical_miles(self) -> f64 {
        let m = self.as_metres();
        m / Length::NM_TO_M
    }
}

impl Measure for Length {
    fn from_default_unit(amount: f64) -> Self {
        Length::from_metres(amount)
    }

    fn from_resolution_unit(amount: i64) -> Self {
        Length {
            micrometres: amount,
        }
    }

    fn as_default_unit(self) -> f64 {
        self.as_metres()
    }

    fn as_resolution_unit(self) -> i64 {
        self.micrometres
    }
}

impl_measure! { Length }

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn metres_to_kilometres() {
        let l = Length::from_metres(1000.0);
        assert_eq!(1.0, l.as_kilometres());
    }

    #[test]
    fn metres_to_nautical_miles() {
        let l = Length::from_metres(1000.0);
        assert_eq!(0.5399568034557235, l.as_nautical_miles());
    }

    #[test]
    fn kilometres_to_nautical_miles() {
        let l = Length::from_kilometres(1000.0);
        assert_eq!(539.9568034557235, l.as_nautical_miles());
    }

    #[test]
    fn nautical_miles_to_metres() {
        let l = Length::from_nautical_miles(10.5);
        assert_eq!(19446.0, l.as_metres());
    }

    #[test]
    fn nautical_miles_to_kilometres() {
        let l = Length::from_nautical_miles(10.5);
        assert_eq!(19.446, l.as_kilometres());
    }

    #[test]
    fn feet_to_metres() {
        let l = Length::from_feet(25000.0);
        assert_eq!(7620.0, l.as_metres());
    }

    #[test]
    fn metres_to_feet() {
        let l = Length::from_metres(7620.0);
        assert_eq!(25000.0, l.as_feet());
    }

    #[test]
    fn one_metre() {
        let l = Length::from_metres(1.0);
        assert_eq!(1.0, l.as_metres());
    }

    #[test]
    fn one_kilometre() {
        let l = Length::from_kilometres(1.0);
        assert_eq!(1.0, l.as_kilometres());
    }

    #[test]
    fn one_nautical_mile() {
        let l = Length::from_nautical_miles(1.0);
        assert_eq!(1.0, l.as_nautical_miles());
    }

    #[test]
    fn one_feet() {
        let l = Length::from_feet(1.0);
        assert_eq!(1.0, l.as_feet());
    }

    #[test]
    fn one_micrometre() {
        let l1 = Length::from_metres(1.000001);
        let l2 = Length::from_metres(1.000002);
        let l3 = Length::from_metres(1.0000011);
        assert_eq!(l1, l3);
        assert_ne!(l1, l2);
        assert_ne!(l2, l3);
    }
}
