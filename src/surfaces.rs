// Copyright: (c) 2020 Cedric Liegeois
// License: BSD3

//! Common surfaces of different celestial bodies.
//!

use crate::{Ellipsoid, Length, Sphere};

/// World Geodetic 84 Ellipsoid.
pub const WGS84_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_micrometres(6378137000000),
    Length::from_micrometres(6356752314245),
    0.08181919084262157,
    0.0033528106647474805,
);

/// Sphere derived from: World Geodetic 84 Ellipsoid.
pub const WGS84_SPHERE: Sphere = Sphere::new(Length::from_micrometres(6371008771415));

/// Geodetic Reference System 1980 Ellipsoid.
pub const GRS80_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_micrometres(6378137000000),
    Length::from_micrometres(6356752314140),
    0.08181919104281514,
    0.003352810681182319,
);

/// Sphere derived from: Geodetic Reference System 1980 Ellipsoid.
pub const GRS80_SPHERE: Sphere = Sphere::new(Length::from_micrometres(6371008771380));

/// World Geodetic 72 Ellipsoid.
pub const WGS72_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_micrometres(6378135000000),
    Length::from_micrometres(6356750520016),
    0.08181881066274845,
    0.003352779454167505,
);

/// Sphere derived from: World Geodetic 72 Ellipsoid.
pub const WGS72_SPHERE: Sphere = Sphere::new(Length::from_micrometres(6371006840005));

/// IUGG 1924 Ellipsoid (aka Hayford).
pub const INTL_1924_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_micrometres(6378388000000),
    Length::from_micrometres(6356911946128),
    0.08199188997902888,
    0.003367003367003367,
);

/// Sphere derived from: IUGG 1924 Ellipsoid (aka Hayford).
pub const INTL_1924_SPHERE: Sphere = Sphere::new(Length::from_micrometres(6371229315376));

/// Original definition Ellipsoid (1796).
pub const AIRY_1830_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_micrometres(6377563396000),
    Length::from_micrometres(6356256909237),
    0.08167337387414043,
    0.0033408506414970775,
);

/// Sphere derived from: Original definition Ellipsoid (1796).
pub const AIRY_1830_SPHERE: Sphere = Sphere::new(Length::from_micrometres(6370461233746));

/// Not specified, use only in cases where geodetic datum is unknown.
pub const AIRY_MODIFIED_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_micrometres(6377340189000),
    Length::from_micrometres(6356034447939),
    0.08167337387414247,
    0.0033408506414970775,
);

/// Sphere derived from: Not specified, use only in cases where geodetic datum is unknown.
pub const AIRY_MODIFIED_SPHERE: Sphere = Sphere::new(Length::from_micrometres(6370238275313));

/// Bessel 1841 Ellipsoid.
pub const BESSEL_1841_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_micrometres(6377397155000),
    Length::from_micrometres(6356078962818),
    0.08169683122252666,
    0.003342773182174806,
);

/// Sphere derived from: Bessel 1841 Ellipsoid.
pub const BESSEL_1841_SPHERE: Sphere = Sphere::new(Length::from_micrometres(6370291090939));

/// Clarke (1866) Ellipsoid.
pub const CLARKE_1866_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_micrometres(6378206400000),
    Length::from_micrometres(6356583800000),
    0.08227185422298973,
    0.0033900753039276207,
);

/// Sphere derived from: Clarke (1866) Ellipsoid.
pub const CLARKE_1866_SPHERE: Sphere = Sphere::new(Length::from_micrometres(6370998866667));

///  Clarke (1880) Ellipsoid.
pub const CLARKE_1880_IGN_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_micrometres(6378249200000),
    Length::from_micrometres(6356515000000),
    0.08248325676336525,
    0.003407549520011315,
);

/// Sphere derived from:  Clarke (1880) Ellipsoid.
pub const CLARKE_1880_IGN_SPHERE: Sphere = Sphere::new(Length::from_micrometres(6371004466667));

/// Mars Orbiter Laser Altimeter Ellipsoid.
pub const MARS_2000_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_micrometres(3398627000000),
    Length::from_micrometres(3378611528857),
    0.10836918094474898,
    0.005889281507656065,
);

/// Sphere derived from: Mars Orbiter Laser Altimeter Ellipsoid.
pub const MARS_2000_SPHERE: Sphere = Sphere::new(Length::from_micrometres(3391955176286));

/// Moon IAU/IAG Sphere.
pub const MOON_SPHERE: Sphere = Sphere::new(Length::from_micrometres(1737400000000));
