use crate::{Ellipsoid, Length, Sphere};

pub const WGS84: Ellipsoid = Ellipsoid::from_all(
    Length::from_micrometres(6378137000000),
    Length::from_micrometres(6356752314245),
    0.08181919084262157,
    0.0033528106647474805,
);

pub const WGS84_SPHERE: Sphere = Sphere::new(Length::from_micrometres(6371008771415));

pub const MOON: Sphere = Sphere::new(Length::from_micrometres(1737400000000));
