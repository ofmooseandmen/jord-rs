use crate::FixedLength;

pub trait Surface {
    fn equatorial_radius(&self) -> FixedLength;

    fn polar_radius(&self) -> FixedLength;

    fn eccentricity(&self) -> f64;

    fn flattening(&self) -> f64;

    fn mean_radius(&self) -> FixedLength;
}

#[derive(Debug)]
pub struct Ellipsoid {
    pub(crate) equatorial_radius: FixedLength,
    pub(crate) polar_radius: FixedLength,
    pub(crate) eccentricity: f64,
    pub(crate) flattening: f64,
}

impl Ellipsoid {
    pub fn new(eqr: FixedLength, invf: f64) -> Self {
        let a = eqr.to_metres();
        let f = 1.0 / invf;
        let b = a * (1.0 - f);
        let e = (1.0 - (b * b) / (a * a)).sqrt();
        Ellipsoid {
            equatorial_radius: eqr,
            polar_radius: FixedLength::from_metres(b),
            eccentricity: e,
            flattening: f,
        }
    }
}

impl Surface for Ellipsoid {
    fn equatorial_radius(&self) -> FixedLength {
        self.equatorial_radius
    }

    fn polar_radius(&self) -> FixedLength {
        self.polar_radius
    }

    fn eccentricity(&self) -> f64 {
        self.eccentricity
    }

    fn flattening(&self) -> f64 {
        self.flattening
    }

    fn mean_radius(&self) -> FixedLength {
        let a = self.equatorial_radius();
        let b = self.polar_radius();
        (2.0 * a + b) / 3.0
    }
}

#[derive(Debug)]
pub struct Sphere {
    radius: FixedLength,
}

impl Sphere {
    pub const fn new(radius: FixedLength) -> Sphere {
        Sphere { radius: radius }
    }
}

impl Surface for Sphere {
    fn equatorial_radius(&self) -> FixedLength {
        self.radius
    }

    fn polar_radius(&self) -> FixedLength {
        self.radius
    }

    fn eccentricity(&self) -> f64 {
        0.0
    }

    fn flattening(&self) -> f64 {
        0.0
    }

    fn mean_radius(&self) -> FixedLength {
        self.radius
    }
}
