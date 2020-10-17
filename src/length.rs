use crate::Measurement;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LengthResolution {
    Metre,
    Millimetre,
    Micrometre,
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Length {
    metres: f64,
}

const FT_TO_M: f64 = 0.3048;

const NM_TO_M: f64 = 1_852.0;

const KM_TO_M: f64 = 1_000.0;

const M_TO_MS: f64 = 1_000.0;

const M_TO_UM: f64 = M_TO_MS * 1_000.0;

impl Length {
    pub const fn zero() -> Self {
        Length { metres: 0.0 }
    }

    pub const fn from_metres(metres: f64) -> Self {
        Length { metres }
    }

    pub fn from_kilometres(kilometres: f64) -> Self {
        Length::from_metres(kilometres * KM_TO_M)
    }

    pub fn from_feet(feet: f64) -> Self {
        Length::from_metres(feet * FT_TO_M)
    }

    pub fn from_nautical_miles(nautical_miles: f64) -> Self {
        Length::from_metres(nautical_miles * NM_TO_M)
    }

    pub fn round(self, resolution: LengthResolution) -> Self {
        let scale = match resolution {
            LengthResolution::Metre => 1.0,
            LengthResolution::Millimetre => M_TO_MS,
            LengthResolution::Micrometre => M_TO_UM,
        };
        let metres = ((self.metres * scale).round()) / scale;
        Length { metres }
    }

    pub const fn metres(self) -> f64 {
        self.metres
    }

    pub fn as_kilometres(self) -> f64 {
        self.metres() / KM_TO_M
    }

    pub fn as_feet(self) -> f64 {
        self.metres() / FT_TO_M
    }

    pub fn as_nautical_miles(self) -> f64 {
        self.metres() / NM_TO_M
    }

    pub fn abs(self) -> Self {
        Length {
            metres: self.metres.abs(),
        }
    }
}

impl Measurement for Length {
    fn from_default_unit(amount: f64) -> Self {
        Length::from_metres(amount)
    }

    fn as_default_unit(self) -> f64 {
        self.metres()
    }
}

impl_measurement! { Length }
