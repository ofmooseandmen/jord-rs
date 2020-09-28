use crate::{Ellipsoid, Sphere, FixedLength};

pub const fn wgs84() -> Ellipsoid {
    Ellipsoid {
        equatorial_radius: FixedLength::from_micrometres(6378137000000),
        polar_radius: FixedLength::from_micrometres(6356752314245),
        eccentricity: 0.08181919084262157,
        flattening: 0.0033528106647474805,
    }
}

pub const fn wgs84_sphere() -> Sphere {
    Sphere::new(FixedLength::from_micrometres(6371008771415))
}

pub const fn moon() -> Sphere {
    Sphere::new(FixedLength::from_micrometres(1737400000000))
}

