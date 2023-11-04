use crate::{
    surface::Surface, Cartesian3DVector, GeocentricPos, GeodeticPos, Length, NVector, Vec3,
};

/// An ellipsoid.
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
    /// use jord::Length;
    /// use jord::ellipsoidal::Ellipsoid;
    ///
    /// let r = (Ellipsoid::WGS84.volumetric_radius().as_metres() * 10.0).round() / 10.0;
    /// assert_eq!(jord::spherical::Sphere::EARTH.radius().as_metres(), r);
    /// ```
    pub fn volumetric_radius(&self) -> Length {
        let a = self.equatorial_radius().as_metres();
        let b = self.polar_radius().as_metres();
        let r = (a * a * b).cbrt();
        Length::from_metres(r)
    }
}

impl Surface for Ellipsoid {
    fn geodetic_to_geocentric(&self, pos: GeodeticPos) -> GeocentricPos {
        let nv = pos.horizontal_position().as_vec3();
        let nx = nv.x();
        let ny = nv.y();
        let nz = nv.z();
        let a = self.equatorial_radius.as_metres();
        let b = self.polar_radius.as_metres();
        let m = (a * a) / (b * b);
        let n = b / ((nx * nx * m) + (ny * ny * m) + (nz * nz)).sqrt();
        let h = pos.height().as_metres();
        let cx = n * m * nx + h * nx;
        let cy = n * m * ny + h * ny;
        let cz = n * nz + h * nz;
        GeocentricPos::from_metres(Vec3::new(cx, cy, cz))
    }

    fn geocentric_to_geodetic(&self, pos: GeocentricPos) -> GeodeticPos {
        let pv = pos.as_metres();
        let px = pv.x();
        let py = pv.y();
        let pz = pv.z();
        let e2 = self.eccentricity * self.eccentricity;
        let e4 = e2 * e2;
        let a = self.equatorial_radius.as_metres();
        let a2 = a * a;
        let p = (px * px + py * py) / a2;
        let q = ((1.0 - e2) / a2) * (pz * pz);
        let r = (p + q - e4) / 6.0;
        let s = (e4 * p * q) / (4.0 * r * r * r);
        let t = (1.0 + s + (s * (2.0 + s)).sqrt()).powf(1.0 / 3.0);
        let u = r * (1.0 + t + 1.0 / t);
        let v = (u * u + q * e4).sqrt();
        let w = e2 * (u + v - q) / (2.0 * v);
        let k = (u + v + w * w).sqrt() - w;
        let d = k * (px * px + py * py).sqrt() / (k + e2);
        let h = ((k + e2 - 1.0) / k) * (d * d + pz * pz).sqrt();

        let fs = 1.0 / (d * d + pz * pz).sqrt();
        let fa = k / (k + e2);
        let nx = fs * fa * px;
        let ny = fs * fa * py;
        let nz = fs * pz;
        GeodeticPos::new(NVector::new(Vec3::new(nx, ny, nz)), Length::from_metres(h))
    }
}

// TODO(CL): tests
