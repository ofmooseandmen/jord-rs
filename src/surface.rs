use crate::Length;

pub trait Surface {
    fn equatorial_radius(&self) -> Length;

    fn polar_radius(&self) -> Length;

    fn eccentricity(&self) -> f64;

    fn flattening(&self) -> f64;

    fn mean_radius(&self) -> Length;

    fn is_sphere(&self) -> bool {
        self.flattening() == 0.0
    }
}

#[derive(Debug)]
pub struct Ellipsoid {
    equatorial_radius: Length,
    polar_radius: Length,
    eccentricity: f64,
    flattening: f64,
}

impl Ellipsoid {
    pub fn new(eqr: Length, invf: f64) -> Self {
        let a = eqr.as_metres();
        let f = 1.0 / invf;
        let b = a * (1.0 - f);
        let e = (1.0 - (b * b) / (a * a)).sqrt();
        Ellipsoid {
            equatorial_radius: eqr,
            polar_radius: Length::from_metres(b),
            eccentricity: e,
            flattening: f,
        }
    }

    pub(crate) const fn from_all(
        equatorial_radius: Length,
        polar_radius: Length,
        eccentricity: f64,
        flattening: f64,
    ) -> Self {
        Ellipsoid {
            equatorial_radius,
            polar_radius,
            eccentricity,
            flattening,
        }
    }
}

impl Surface for Ellipsoid {
    fn equatorial_radius(&self) -> Length {
        self.equatorial_radius
    }

    fn polar_radius(&self) -> Length {
        self.polar_radius
    }

    fn eccentricity(&self) -> f64 {
        self.eccentricity
    }

    fn flattening(&self) -> f64 {
        self.flattening
    }

    fn mean_radius(&self) -> Length {
        let a = self.equatorial_radius();
        let b = self.polar_radius();
        (2.0 * a + b) / 3.0
    }
}

#[derive(Debug)]
pub struct Sphere {
    radius: Length,
}

impl Sphere {
    pub const fn new(radius: Length) -> Sphere {
        Sphere { radius }
    }
}

impl Surface for Sphere {
    fn equatorial_radius(&self) -> Length {
        self.radius
    }

    fn polar_radius(&self) -> Length {
        self.radius
    }

    fn eccentricity(&self) -> f64 {
        0.0
    }

    fn flattening(&self) -> f64 {
        0.0
    }

    fn mean_radius(&self) -> Length {
        self.radius
    }

    fn is_sphere(&self) -> bool {
        true
    }
}
