use crate::Length;

/// IUGG (International Union of Geodesy and Geophysics) Earth volumic radius - generally accepted
/// as the Earth radius when assuming a spherical model.
/// Note: this is equal to the volumetric radius of the ubiquous WGS84 ellipsoid rounded to 1 decimal.
pub const IUGG_EARTH_RADIUS: Length = Length::from_metres(6_371_000.8f64);

/// Moon IAU/IAG radius.
pub const MOON_RADIUS: Length = Length::from_metres(1_737_400.0f64);

/// An ellipsoid
#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct Ellipsoid {
    equatorial_radius: Length,
    polar_radius: Length,
    eccentricity: f64,
    flattening: f64,
}

impl Ellipsoid {
    /// World Geodetic 84 Ellipsoid.
    pub const WGS84: Ellipsoid = Ellipsoid {
        equatorial_radius: Length::from_metres(6_378_137.0f64),
        polar_radius: Length::from_metres(6_356_752.314245f64),
        eccentricity: 0.08181919084262157f64,
        flattening: 0.0033528106647474805f64,
    };

    /// Geodetic Reference System 1980 Ellipsoid.
    pub const GRS80: Ellipsoid = Ellipsoid {
        equatorial_radius: Length::from_metres(6_378_137.0f64),
        polar_radius: Length::from_metres(6_356_752.314140356f64),
        eccentricity: 0.08181919104281514f64,
        flattening: 0.003352810681182319f64,
    };

    /// Mars Orbiter Laser Altimeter Ellipsoid.
    pub const MARS_2000: Ellipsoid = Ellipsoid {
        equatorial_radius: Length::from_metres(3398627f64),
        polar_radius: Length::from_metres(3378611.5288574793f64),
        eccentricity: 0.10836918094474898f64,
        flattening: 0.005889281507656065f64,
    };

    /// Creates a new ellipsoid from the given equatorial radius (semi-major axis A) and
    /// inverse (or reciprocal) flattening.
    pub fn new(equatorial_radius: Length, inverse_flattening: f64) -> Self {
        let a = equatorial_radius.as_metres();
        let f = 1.0 / inverse_flattening;
        let b = a * (1.0 - f);
        let e = (1.0 - (b * b) / (a * a)).sqrt();
        Ellipsoid {
            equatorial_radius,
            polar_radius: Length::from_metres(b),
            eccentricity: e,
            flattening: f,
        }
    }

    /// Returns the equatorial radius (or semi-major axis A) of this ellipsoid.
    pub fn equatorial_radius(&self) -> Length {
        self.equatorial_radius
    }

    /// Returns the polar radius (or semi-minor axis B) of this ellipsoid.
    pub fn polar_radius(&self) -> Length {
        self.polar_radius
    }

    /// Returns the eccentricity of this ellipsoid.
    pub fn eccentricity(&self) -> f64 {
        self.eccentricity
    }

    /// Returns the flattening of this ellipsoid.
    pub fn flattening(&self) -> f64 {
        self.flattening
    }

    /// Returns the mean radius (arithmetic mean) of this ellipsoid.
    pub fn mean_radius(&self) -> Length {
        let a = self.equatorial_radius();
        let b = self.polar_radius();
        (2.0 * a + b) / 3.0
    }

    /// Returns the volumetric radius of this ellipsoid: the radius of sphere of same volume.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Ellipsoid, Length};
    ///
    /// let r = (Ellipsoid::WGS84.volumetric_radius().as_metres() * 10.0).round() / 10.0;
    /// assert_eq!(jord::IUGG_EARTH_RADIUS.as_metres(), r);
    /// ```
    pub fn volumetric_radius(&self) -> Length {
        let a = self.equatorial_radius().as_metres();
        let b = self.polar_radius().as_metres();
        let r = (a * a * b).cbrt();
        Length::from_metres(r)
    }
}
