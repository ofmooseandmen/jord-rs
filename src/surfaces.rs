use crate::{Ellipsoid, Length, Sphere};

pub const WGS84: Ellipsoid = Ellipsoid::from_all(
    Length::from_micrometres(6378137000000),
    Length::from_micrometres(6356752314245),
    0.08181919084262157,
    0.0033528106647474805,
);

pub const WGS84_SPHERE: Sphere = Sphere::new(Length::from_micrometres(6371008771415));

pub const MARS_2000: Ellipsoid = Ellipsoid::from_all(
    Length::from_micrometres(3398627000000),
    Length::from_micrometres(3378611528857),
    0.10836918094474898,
    0.005889281507656065,
);

pub const MARS_2000_SPHERE: Sphere = Sphere::new(Length::from_micrometres(3391955176286));

pub const MOON: Sphere = Sphere::new(Length::from_micrometres(1737400000000));
