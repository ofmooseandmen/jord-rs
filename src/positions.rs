use crate::Length;

use {crate::Angle, crate::Vec3};

/// Cartesian 3D position vector: allows to represent the position of a general coordinate frame B
/// relative to a reference coordinate frame A as the position vector from A to B.
pub trait Cartesian3DVector: Sized {
    /// Returns the x component of this vector.
    fn x(&self) -> Length;

    /// Returns the y component of this vector.
    fn y(&self) -> Length;

    /// Returns the z component of this vector.
    fn z(&self) -> Length;

    /// Returns the (x, y, z) components of this vector in metres.
    fn as_metres(&self) -> Vec3 {
        Vec3::new(
            self.x().as_metres(),
            self.y().as_metres(),
            self.z().as_metres(),
        )
    }

    /// Rounds the (x, y, z) components of this vector to the nearest metre.
    fn round_m(&self) -> Self {
        self.round(|l| l.round_m())
    }

    /// Rounds the (x, y, z) components of this vector to the nearest decimetre.
    fn round_dm(&self) -> Self {
        self.round(|l| l.round_dm())
    }

    /// Rounds the (x, y, z) components of this vector to the nearest centimetre.
    fn round_cm(&self) -> Self {
        self.round(|l| l.round_cm())
    }

    /// Rounds the (x, y, z) components of this vector to the nearest millimetre.
    fn round_mm(&self) -> Self {
        self.round(|l| l.round_mm())
    }

    /// Rounds the (x, y, z) components of this vector using the given rounding function.
    fn round<F>(&self, round: F) -> Self
    where
        F: Fn(Length) -> Length;
}

/// A geocentric position or Earth Centred Earthy Fixed (ECEF) vector.
#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct GeocentricPos {
    x: Length,
    y: Length,
    z: Length,
}

impl GeocentricPos {
    /// Creates a [GeocentricPos] from the given coordinates.
    pub fn new(x: Length, y: Length, z: Length) -> Self {
        Self { x, y, z }
    }

    /// Creates a [GeocentricPos] from the given coordinates in metres.
    pub fn from_metres(x: f64, y: f64, z: f64) -> Self {
        Self::new(
            Length::from_metres(x),
            Length::from_metres(y),
            Length::from_metres(z),
        )
    }

    /// Creates a [GeocentricPos] from the given coordinates in metres.
    pub(crate) fn from_vec3_metres(v: Vec3) -> Self {
        Self::from_metres(v.x(), v.y(), v.z())
    }
}

impl Cartesian3DVector for GeocentricPos {
    fn x(&self) -> Length {
        self.x
    }

    fn y(&self) -> Length {
        self.y
    }

    fn z(&self) -> Length {
        self.z
    }

    fn round<F>(&self, round: F) -> Self
    where
        F: Fn(Length) -> Length,
    {
        Self::new(round(self.x()), round(self.y()), round(self.z()))
    }
}

/// A geodetic position: the horiztonal coordinates (as a [NVector]) and height above the surface.
#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct GeodeticPos {
    hp: NVector,
    height: Length,
}

impl GeodeticPos {
    /// Creates a new [GeodeticPos] from the given horizontal coordinates and height above the surface.
    pub fn new(hp: NVector, height: Length) -> Self {
        Self { hp, height }
    }

    /// Returns the [NVector] representing the horizontal coordinates of this [GeodeticPos].
    pub fn horizontal_position(&self) -> NVector {
        self.hp
    }

    /// Returns the height above the surface of this [GeodeticPos].
    pub fn height(&self) -> Length {
        self.height
    }
}

/// An horizontal position represented by a pair of latitude-longitude.
#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct LatLong {
    latitude: Angle,
    longitude: Angle,
}

impl LatLong {
    // TODO(CL): normalise?

    /// Creates a new [LatLong] from the given latitude and longitude.
    pub fn new(latitude: Angle, longitude: Angle) -> Self {
        Self {
            latitude,
            longitude,
        }
    }

    /// Creates a new [LatLong] from the given latitude and longitudes in degrees.
    pub fn from_degrees(latitude: f64, longitude: f64) -> Self {
        Self::new(
            Angle::from_degrees(latitude),
            Angle::from_degrees(longitude),
        )
    }

    /// Converts the given [NVector] into a [LatLong].
    pub fn from_nvector(nvector: NVector) -> Self {
        let (lat, lng) = nvector_to_latlong(nvector.0);
        Self::new(lat, lng)
    }

    /// Converts this [LatLong] into an [NVector].
    pub fn to_nvector(&self) -> NVector {
        NVector::new(latlong_to_nvector(self.latitude, self.longitude))
    }

    /// Returns the latitude of this [LatLong].
    pub fn latitude(&self) -> Angle {
        self.latitude
    }

    /// Returns the longitude of this [LatLong].
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

/// An horizontal position represented by a n-vector: the unit and normal vector to the surface.
/// Orientation:
/// - z-axis points to the North Pole along the body's rotation axis,
/// - x-axis points towards the point where latitude = longitude = 0
#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct NVector(Vec3);

impl NVector {
    /// Creates a new [NVector] from the given [unit](crate::Vec3::new_unit) 3D vector.
    pub fn new(v: Vec3) -> Self {
        Self(v)
    }

    /// Creates a new [NVector] from the given latitude and longitude in degrees.
    pub fn from_lat_long_degrees(latitude_degrees: f64, longitude_degrees: f64) -> Self {
        Self::new(latlong_to_nvector(
            Angle::from_degrees(latitude_degrees),
            Angle::from_degrees(longitude_degrees),
        ))
    }

    /// Returns the [NVector] which is the antipode of this [NVector].
    pub fn antipode(&self) -> Self {
        Self::new(-1.0 * self.0)
    }

    /// Determines whether the given [NVector] is the antipode of this [NVector].
    pub fn is_antipode_of(&self, o: Self) -> bool {
        self.0 + o.0 == Vec3::ZERO
    }

    /// Returns this [NVector] as a [Vec3].
    pub fn as_vec3(&self) -> Vec3 {
        self.0
    }
}

fn nvector_to_latlong(nvector: Vec3) -> (Angle, Angle) {
    let x: f64 = nvector.x();
    let y = nvector.y();
    let z = nvector.z();
    let lat = z.atan2((x * x + y * y).sqrt());
    let lon = y.atan2(x);
    (Angle::from_radians(lat), Angle::from_radians(lon))
}

fn latlong_to_nvector(latitude: Angle, longitude: Angle) -> Vec3 {
    let latitude_rads = latitude.as_radians();
    let longitude_rads = longitude.as_radians();
    let cl = latitude_rads.cos();
    let x = cl * longitude_rads.cos();
    let y = cl * longitude_rads.sin();
    let z = latitude_rads.sin();
    Vec3::new(x, y, z)
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

#[cfg(test)]
pub(crate) fn assert_geod_eq_d7_mm(expected: GeodeticPos, actual: GeodeticPos) {
    assert_nv_eq_d7(expected.horizontal_position(), actual.horizontal_position());
    assert_eq!(expected.height().round_mm(), actual.height().round_mm());
}
