// Copyright: (c) 2020 Cedric Liegeois
// License: BSD3

//! Common surfaces of different celestial bodies.
//!

use crate::{Ellipsoid, Length, Sphere};

/// World Geodetic 84 Ellipsoid.
pub const WGS84_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_metres(6378137f64),
    Length::from_metres(6356752.314245179f64),
    0.08181919084262157f64,
    0.0033528106647474805f64,
);

/// Sphere derived from: World Geodetic 84 Ellipsoid.
pub const WGS84_SPHERE: Sphere = Sphere::new(Length::from_metres(6371008.771415059f64));

/// Geodetic Reference System 1980 Ellipsoid.
pub const GRS80_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_metres(6378137f64),
    Length::from_metres(6356752.314140356f64),
    0.08181919104281514f64,
    0.003352810681182319f64,
);

/// Sphere derived from: Geodetic Reference System 1980 Ellipsoid.
pub const GRS80_SPHERE: Sphere = Sphere::new(Length::from_metres(6371008.771380119f64));

/// World Geodetic 72 Ellipsoid.
pub const WGS72_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_metres(6378135f64),
    Length::from_metres(6356750.520016094f64),
    0.08181881066274845f64,
    0.003352779454167505f64,
);

/// Sphere derived from: World Geodetic 72 Ellipsoid.
pub const WGS72_SPHERE: Sphere = Sphere::new(Length::from_metres(6371006.840005364f64));

/// IUGG 1924 Ellipsoid (aka Hayford).
pub const INTL_1924_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_metres(6378388f64),
    Length::from_metres(6356911.9461279465f64),
    0.08199188997902888f64,
    0.003367003367003367f64,
);

/// Sphere derived from: IUGG 1924 Ellipsoid (aka Hayford).
pub const INTL_1924_SPHERE: Sphere = Sphere::new(Length::from_metres(6371229.315375983f64));

/// Original definition Ellipsoid (1796).
pub const AIRY_1830_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_metres(6377563.396f64),
    Length::from_metres(6356256.909237285f64),
    0.08167337387414043f64,
    0.0033408506414970775f64,
);

/// Sphere derived from: Original definition Ellipsoid (1796).
pub const AIRY_1830_SPHERE: Sphere = Sphere::new(Length::from_metres(6370461.233745761f64));

/// Not specified, use only in cases where geodetic datum is unknown.
pub const AIRY_MODIFIED_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_metres(6377340.189f64),
    Length::from_metres(6356034.447938534f64),
    0.08167337387414247f64,
    0.0033408506414970775f64,
);

/// Sphere derived from: Not specified, use only in cases where geodetic datum is unknown.
pub const AIRY_MODIFIED_SPHERE: Sphere = Sphere::new(Length::from_metres(6370238.275312845f64));

/// Bessel 1841 Ellipsoid.
pub const BESSEL_1841_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_metres(6377397.155f64),
    Length::from_metres(6356078.962818189f64),
    0.08169683122252666f64,
    0.003342773182174806f64,
);

/// Sphere derived from: Bessel 1841 Ellipsoid.
pub const BESSEL_1841_SPHERE: Sphere = Sphere::new(Length::from_metres(6370291.090939396f64));

/// Clarke (1866) Ellipsoid.
pub const CLARKE_1866_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_metres(6378206.4f64),
    Length::from_metres(6356583.800000007f64),
    0.08227185422298973f64,
    0.0033900753039276207f64,
);

/// Sphere derived from: Clarke (1866) Ellipsoid.
pub const CLARKE_1866_SPHERE: Sphere = Sphere::new(Length::from_metres(6370998.86666667f64));

///  Clarke (1880) Ellipsoid.
pub const CLARKE_1880_IGN_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_metres(6378249.2f64),
    Length::from_metres(6356515.000000028f64),
    0.08248325676336525f64,
    0.003407549520011315f64,
);

/// Sphere derived from:  Clarke (1880) Ellipsoid.
pub const CLARKE_1880_IGN_SPHERE: Sphere = Sphere::new(Length::from_metres(6371004.466666676f64));

/// Mars Orbiter Laser Altimeter Ellipsoid.
pub const MARS_2000_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_metres(3398627f64),
    Length::from_metres(3378611.5288574793f64),
    0.10836918094474898f64,
    0.005889281507656065f64,
);

/// Sphere derived from: Mars Orbiter Laser Altimeter Ellipsoid.
pub const MARS_2000_SPHERE: Sphere = Sphere::new(Length::from_metres(3391955.176285826f64));

/// Moon IAU/IAG Sphere.
pub const MOON_SPHERE: Sphere = Sphere::new(Length::from_metres(1737400f64));
