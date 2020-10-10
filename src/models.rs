// Copyright: (c) 2020 Cedric Liegeois
// License: BSD3

//! Common ellipsoidal and spherical models.
//!

use crate::{Ellipsoid, Ellipsoidal, LongitudeRange, Model, ModelId, Sphere, Spherical};

/// World Geodetic System 1984.
pub const WGS84: WGS84Model = WGS84Model {};

/// Struct for model: World Geodetic System 1984.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WGS84Model {}

impl Model for WGS84Model {
    type Surface = Ellipsoid;
    fn model_id(&self) -> ModelId {
        ModelId::new("WGS84".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L180
    }
    fn surface(&self) -> Ellipsoid {
        crate::surfaces::WGS84_ELLIPSOID
    }
}

impl Ellipsoidal for WGS84Model {}

/// Geodetic Reference System 1980.
pub const GRS80: GRS80Model = GRS80Model {};

/// Struct for model: Geodetic Reference System 1980.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GRS80Model {}

impl Model for GRS80Model {
    type Surface = Ellipsoid;
    fn model_id(&self) -> ModelId {
        ModelId::new("GRS80".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L180
    }
    fn surface(&self) -> Ellipsoid {
        crate::surfaces::GRS80_ELLIPSOID
    }
}

impl Ellipsoidal for GRS80Model {}

/// World Geodetic System 1972.
pub const WGS72: WGS72Model = WGS72Model {};

/// Struct for model: World Geodetic System 1972.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WGS72Model {}

impl Model for WGS72Model {
    type Surface = Ellipsoid;
    fn model_id(&self) -> ModelId {
        ModelId::new("WGS72".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L180
    }
    fn surface(&self) -> Ellipsoid {
        crate::surfaces::WGS72_ELLIPSOID
    }
}

impl Ellipsoidal for WGS72Model {}

/// European Terrestrial Reference System 1989.
pub const ETRS89: ETRS89Model = ETRS89Model {};

/// Struct for model: European Terrestrial Reference System 1989.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ETRS89Model {}

impl Model for ETRS89Model {
    type Surface = Ellipsoid;
    fn model_id(&self) -> ModelId {
        ModelId::new("ETRS89".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L180
    }
    fn surface(&self) -> Ellipsoid {
        crate::surfaces::GRS80_ELLIPSOID
    }
}

impl Ellipsoidal for ETRS89Model {}

/// North American Datum of 1983.
pub const NAD83: NAD83Model = NAD83Model {};

/// Struct for model: North American Datum of 1983.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NAD83Model {}

impl Model for NAD83Model {
    type Surface = Ellipsoid;
    fn model_id(&self) -> ModelId {
        ModelId::new("NAD83".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L180
    }
    fn surface(&self) -> Ellipsoid {
        crate::surfaces::GRS80_ELLIPSOID
    }
}

impl Ellipsoidal for NAD83Model {}

/// European Datum 1950.
pub const ED50: ED50Model = ED50Model {};

/// Struct for model: European Datum 1950.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ED50Model {}

impl Model for ED50Model {
    type Surface = Ellipsoid;
    fn model_id(&self) -> ModelId {
        ModelId::new("ED50".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L180
    }
    fn surface(&self) -> Ellipsoid {
        crate::surfaces::INTL_1924_ELLIPSOID
    }
}

impl Ellipsoidal for ED50Model {}

/// Irland.
pub const IRL_1975: Irl1975Model = Irl1975Model {};

/// Struct for model: Irland.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Irl1975Model {}

impl Model for Irl1975Model {
    type Surface = Ellipsoid;
    fn model_id(&self) -> ModelId {
        ModelId::new("IRL_1975".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L180
    }
    fn surface(&self) -> Ellipsoid {
        crate::surfaces::AIRY_MODIFIED_ELLIPSOID
    }
}

impl Ellipsoidal for Irl1975Model {}

/// North American Datum of 1927.
pub const NAD27: NAD27Model = NAD27Model {};

/// Struct for model: North American Datum of 1927.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NAD27Model {}

impl Model for NAD27Model {
    type Surface = Ellipsoid;
    fn model_id(&self) -> ModelId {
        ModelId::new("NAD27".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L180
    }
    fn surface(&self) -> Ellipsoid {
        crate::surfaces::CLARKE_1866_ELLIPSOID
    }
}

impl Ellipsoidal for NAD27Model {}

/// NTF (Paris) / France I.
pub const NTF: NTFModel = NTFModel {};

/// Struct for model: NTF (Paris) / France I.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NTFModel {}

impl Model for NTFModel {
    type Surface = Ellipsoid;
    fn model_id(&self) -> ModelId {
        ModelId::new("NTF".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L180
    }
    fn surface(&self) -> Ellipsoid {
        crate::surfaces::CLARKE_1880_IGN_ELLIPSOID
    }
}

impl Ellipsoidal for NTFModel {}

/// Ordnance Survey Great Britain 1936.
pub const OSGB36: OSGB36Model = OSGB36Model {};

/// Struct for model: Ordnance Survey Great Britain 1936.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OSGB36Model {}

impl Model for OSGB36Model {
    type Surface = Ellipsoid;
    fn model_id(&self) -> ModelId {
        ModelId::new("OSGB36".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L180
    }
    fn surface(&self) -> Ellipsoid {
        crate::surfaces::AIRY_1830_ELLIPSOID
    }
}

impl Ellipsoidal for OSGB36Model {}

/// Geodetic Datum for Germany.
pub const POTSDAM: PotsdamModel = PotsdamModel {};

/// Struct for model: Geodetic Datum for Germany.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PotsdamModel {}

impl Model for PotsdamModel {
    type Surface = Ellipsoid;
    fn model_id(&self) -> ModelId {
        ModelId::new("POTSDAM".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L180
    }
    fn surface(&self) -> Ellipsoid {
        crate::surfaces::BESSEL_1841_ELLIPSOID
    }
}

impl Ellipsoidal for PotsdamModel {}

/// Tokyo Japan.
pub const TOKYO_JAPAN: TokyoJapanModel = TokyoJapanModel {};

/// Struct for model: Tokyo Japan.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TokyoJapanModel {}

impl Model for TokyoJapanModel {
    type Surface = Ellipsoid;
    fn model_id(&self) -> ModelId {
        ModelId::new("TOKYO_JAPAN".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L180
    }
    fn surface(&self) -> Ellipsoid {
        crate::surfaces::BESSEL_1841_ELLIPSOID
    }
}

impl Ellipsoidal for TokyoJapanModel {}

/// Mars Orbiter Laser Altimeter.
pub const MARS_2000: Mars2000Model = Mars2000Model {};

/// Struct for model: Mars Orbiter Laser Altimeter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mars2000Model {}

impl Model for Mars2000Model {
    type Surface = Ellipsoid;
    fn model_id(&self) -> ModelId {
        ModelId::new("MARS_2000".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L360
    }
    fn surface(&self) -> Ellipsoid {
        crate::surfaces::MARS_2000_ELLIPSOID
    }
}

impl Ellipsoidal for Mars2000Model {}

/// Spherical Earth model derived from WGS84 ellipsoid.
pub const S84: S84Model = S84Model {};

/// Struct for model: Spherical Earth model derived from WGS84 ellipsoid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct S84Model {}

impl Model for S84Model {
    type Surface = Sphere;
    fn model_id(&self) -> ModelId {
        ModelId::new("S84".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L180
    }
    fn surface(&self) -> Sphere {
        crate::surfaces::WGS84_SPHERE
    }
}

impl Spherical for S84Model {}

/// Spherical Mars model derived from Mars2000 ellipsoid.
pub const SMARS_2000: SMars2000Model = SMars2000Model {};

/// Struct for model: Spherical Mars model derived from Mars2000 ellipsoid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SMars2000Model {}

impl Model for SMars2000Model {
    type Surface = Sphere;
    fn model_id(&self) -> ModelId {
        ModelId::new("SMARS_2000".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L360
    }
    fn surface(&self) -> Sphere {
        crate::surfaces::MARS_2000_SPHERE
    }
}

impl Spherical for SMars2000Model {}

/// Moon IAU/IAG.
pub const MOON: MoonModel = MoonModel {};

/// Struct for model: Moon IAU/IAG.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MoonModel {}

impl Model for MoonModel {
    type Surface = Sphere;
    fn model_id(&self) -> ModelId {
        ModelId::new("MOON".to_string())
    }
    fn longitude_range(&self) -> LongitudeRange {
        LongitudeRange::L180
    }
    fn surface(&self) -> Sphere {
        crate::surfaces::MOON_SPHERE
    }
}

impl Spherical for MoonModel {}
