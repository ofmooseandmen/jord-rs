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
