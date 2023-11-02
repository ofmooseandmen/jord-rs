use crate::Length;

use {crate::Angle, crate::Vec3};

/// TODO: Cartesian 3D position vector
pub trait PositionVector: Sized {
    fn new(x: Length, y: Length, z: Length) -> Self;

    fn from_metres(v: Vec3) -> Self {
        Self::new(
            Length::from_metres(v.x()),
            Length::from_metres(v.y()),
            Length::from_metres(v.z()),
        )
    }

    fn x(&self) -> Length;

    fn y(&self) -> Length;

    fn z(&self) -> Length;

    fn as_metres(&self) -> Vec3 {
        Vec3::new(
            self.x().as_metres(),
            self.y().as_metres(),
            self.z().as_metres(),
        )
    }

    fn round_m(&self) -> Self {
        self.round(|l| l.round_m())
    }

    fn round_dm(&self) -> Self {
        self.round(|l| l.round_dm())
    }

    fn round_cm(&self) -> Self {
        self.round(|l| l.round_cm())
    }

    fn round_mm(&self) -> Self {
        self.round(|l| l.round_mm())
    }

    fn round<F>(&self, round: F) -> Self
    where
        F: Fn(Length) -> Length,
    {
        Self::new(round(self.x()), round(self.y()), round(self.z()))
    }
}

#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct GeocentricPos {
    x: Length,
    y: Length,
    z: Length,
}

impl PositionVector for GeocentricPos {
    fn new(x: Length, y: Length, z: Length) -> Self {
        Self { x, y, z }
    }

    fn x(&self) -> Length {
        self.x
    }

    fn y(&self) -> Length {
        self.y
    }

    fn z(&self) -> Length {
        self.z
    }
}

#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct GeodeticPos {
    hp: NVector,
    height: Length,
}

impl GeodeticPos {
    pub fn new(hp: NVector, height: Length) -> Self {
        Self { hp, height }
    }

    pub fn horizontal_position(&self) -> NVector {
        self.hp
    }

    pub fn height(&self) -> Length {
        self.height
    }
}

#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct LatLong {
    latitude: Angle,
    longitude: Angle,
}

impl LatLong {
    // TODO(CL): antipode + NORTH_POLE, SOUTH_POLE & NULL_ISLAND
    // TODO(CL): normalise?
    // TODO(CL): round_d(5|6|7)?

    pub fn new(latitude: Angle, longitude: Angle) -> Self {
        Self {
            latitude,
            longitude,
        }
    }

    pub fn from_degrees(latitude: f64, longitude: f64) -> Self {
        Self::new(
            Angle::from_degrees(latitude),
            Angle::from_degrees(longitude),
        )
    }

    pub fn from_nvector(nvector: NVector) -> Self {
        let x = nvector.0.x();
        let y = nvector.0.y();
        let z = nvector.0.z();
        let lat = z.atan2((x * x + y * y).sqrt());
        let lon = y.atan2(x);
        Self::new(Angle::from_radians(lat), Angle::from_radians(lon))
    }

    pub fn to_nvector(&self) -> NVector {
        let latitude_rads = self.latitude.as_radians();
        let longitude_rads = self.longitude.as_radians();
        let cl = latitude_rads.cos();
        let x = cl * longitude_rads.cos();
        let y = cl * longitude_rads.sin();
        let z = latitude_rads.sin();
        NVector::new(Vec3::new(x, y, z))
    }

    pub fn latitude(&self) -> Angle {
        self.latitude
    }

    pub fn longitude(&self) -> Angle {
        self.longitude
    }

    /// Rounds the latitude and longitude of this latlong to the nearest decimal degrees with 5 decimal places.
    ///
    /// The precision of the returned latlong corresponds to the accuracy achieved by commercial GPS
    /// units with differential correction; it allows to distinguish 2 positions about 1.1 metres apart.
    ///
    /// See also: [Angle::round_d5](crate::Angle::round_d5).
    pub fn round_d5(&self) -> Self {
        Self {
            latitude: self.latitude.round_d5(),
            longitude: self.longitude.round_d5(),
        }
    }

    /// Rounds the latitude and longitude of this latlong to the nearest decimal degrees with 6 decimal places.
    ///
    /// The precision of the returned latlong corresponds to the accuracy achieved by
    /// differentially corrected GPS; it allows to distinguish 2 positions about 0.11 metres apart.
    ///
    /// See also: [Angle::round_d6](crate::Angle::round_d6).
    pub fn round_d6(&self) -> Self {
        Self {
            latitude: self.latitude.round_d6(),
            longitude: self.longitude.round_d6(),
        }
    }

    /// Rounds the latitude and longitude of this latlong to the nearest decimal degrees with 7 decimal places.
    ///
    /// The precision of the returned latlong corresponds to the near limit of GPS-based
    /// techniques; it allows to distinguish 2 positions about 11 millimetres apart.
    ///
    /// See also: [Angle::round_d7](crate::Angle::round_d7).
    pub fn round_d7(&self) -> Self {
        Self {
            latitude: self.latitude.round_d7(),
            longitude: self.longitude.round_d7(),
        }
    }
}

#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct NVector(Vec3);

impl NVector {
    pub fn new(v: Vec3) -> Self {
        Self(v)
    }

    pub fn from_lat_long_degrees(lat: f64, lng: f64) -> Self {
        // TODO(CL): create a method and call it there & in LatLong
        LatLong::from_degrees(lat, lng).to_nvector()
    }

    pub fn antipode(&self) -> Self {
        Self::new(-1.0 * self.0)
    }

    pub fn is_antipode_of(&self, o: Self) -> bool {
        self.0 + o.0 == Vec3::ZERO
    }

    pub fn as_vec3(&self) -> Vec3 {
        self.0
    }
}

#[cfg(test)]
pub(crate) fn assert_nv_eq_d7(expected: NVector, actual: NVector) {
    let lle = LatLong::from_nvector(expected).round_d7();
    let lla: LatLong = LatLong::from_nvector(actual).round_d7();
    if lle != lla {
        panic!(
            "Expected position was {:#?} but got actual position is {:#?}",
            lle, lla
        )
    }
}

#[cfg(test)]
pub(crate) fn assert_opt_nv_eq_d7(expected: NVector, actual: Option<NVector>) {
    match actual {
        Some(a) => assert_nv_eq_d7(expected, a),
        None => panic!(
            "Expected position was {:#?} but got actual position is None",
            LatLong::from_nvector(expected).round_d7()
        ),
    }
}
