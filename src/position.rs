use std::f64::consts::PI;

use crate::{Angle, Vec3};

/// Represents an horizontal position. This trait allows to abstract over the struct used to define a global
/// position at at the surface of a celestial body (e.g. most likely the Earth). The most common representation
/// is to use latitude and longitude.  However, this representation has a severe limitation; the two singularities
/// at latitudes +/- 90, where longitude is undefined. In addition, when getting close to the singularities, the
/// representation exhibits considerable non-linearities and extreme latitude dependency, leading to reduced
/// accuracy in many algorithms. In order to overcome these limitation any horizontal position is also represented by
/// a /n/-vector: a unit length 3-dimensional vector normal to the surface (note that the model choosen to represent the
/// surface is irrelevant here; it can be an ellipsoid or a sphere).
///
/// /n/-vector orientation:
/// - z-axis points to the North Pole along the body's rotation axis,
/// - x-axis points towards the point where latitude = longitude = 0
///
/// Every implementation of this trait shall at least store the /n/-vector as a [Vec3].
pub trait HorizontalPosition: Clone + Copy + std::fmt::Debug + PartialEq + Sized {
    /// Creates a global horizontal position (/n/-vector) from the given latitude and longitude.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, HorizontalPosition, Point, Vec3};
    ///
    /// let p = Point::from_lat_long(Angle::from_degrees(90.0), Angle::from_degrees(0.0));
    /// let (lat, lon) = p.to_lat_long();
    /// assert_eq!(lat.as_degrees(), 90.0);
    /// assert_eq!(lon.as_degrees(), 0.0);
    ///
    /// let v = Vec3::from_lat_long(Angle::from_degrees(90.0), Angle::from_degrees(0.0));
    /// let (lat, lon) = v.to_lat_long();
    /// assert_eq!(lat.as_degrees(), 90.0);
    /// assert_eq!(lon.as_degrees(), 0.0);
    /// ```
    ///
    /// Note: if the given latitude is a pole (+/- 90) then the longitudde is set to 0.
    fn from_lat_long(latitude: Angle, longitude: Angle) -> Self {
        Self::from_lat_long_radians(latitude.as_radians(), longitude.as_radians())
    }

    /// Creates a global horizontal position (/n/-vector) from the given latitude and longitude in degrees.
    fn from_lat_long_degrees(latitude: f64, longitude: f64) -> Self {
        Self::from_lat_long_radians(latitude.to_radians(), longitude.to_radians())
    }

    /// Creates a global horizontal position (/n/-vector) from the given latitude and longitude in radians.
    fn from_lat_long_radians(latitude: f64, longitude: f64) -> Self;

    /// Returns the latitude and longitude of this horizontal position (may require conversion).
    fn to_lat_long(&self) -> (Angle, Angle);

    /// Returns the latitude and longitude in degrees of this horizontal position (may require conversion).
    fn to_lat_long_degrees(&self) -> (f64, f64);

    /// Returns the latitude and longitude in radians of this horizontal position (may require conversion).
    fn to_lat_long_radians(&self) -> (f64, f64);

    /// Wraps the given /n/-vector to an global horizontal position (may perform additional conversion to latitude/longitude).
    fn from_nvector(nvector: Vec3) -> Self;

    /// Returns the /n/-vector representing this horizontal position.
    fn as_nvector(&self) -> Vec3;

    /// Returns the antipode of this position: the position which is diametrically opposite to this position.
    fn antipode(&self) -> Self;

    /// Determines if this position is the antipode of the given position.
    fn is_antipode(&self, other: Self) -> bool;

    /// Rounds both the latitude and longitude of this position to the nearest decimal
    /// degrees with 5 decimal places.
    ///
    /// See also: [Angle::round_d5](crate::Angle::round_d5).
    fn round_d5(&self) -> Self {
        let (lat, lng) = self.to_lat_long();
        Self::from_lat_long(lat.round_d5(), lng.round_d5())
    }

    /// Rounds both the latitude and longitude of this position to the nearest decimal
    /// degrees with 6 decimal places.
    ///
    /// See also: [Angle::round_d6](crate::Angle::round_d6).
    fn round_d6(&self) -> Self {
        let (lat, lng) = self.to_lat_long();
        Self::from_lat_long(lat.round_d6(), lng.round_d6())
    }

    /// Rounds both the latitude and longitude of this position to the nearest decimal
    /// degrees with 7 decimal places.
    ///
    /// See also: [Angle::round_d7](crate::Angle::round_d7).
    fn round_d7(&self) -> Self {
        let (lat, lng) = self.to_lat_long();
        Self::from_lat_long(lat.round_d7(), lng.round_d7())
    }
}

/// An horizontal position that stores the latitude, longitude and equivalent /n/-vector.
/// This struct is usefull for algorithms that rely both on the latitude/longitude and the /n/-vector
/// representation (such as point-in-polygon) or when the user always needs access to the latitude/longitude.
/// However when the latitude/longitude is not required prefer using `Vec3` directly as it saves the somewhat
/// costly conversion /n/-vector to latitude/longitude.
#[derive(Clone, Copy, Debug, Default)]
pub struct Point {
    latitude: Angle,
    longitude: Angle,
    nvector: Vec3,
}

impl Point {
    /// North Pole.
    pub const NORTH_POLE: Point = Point {
        latitude: Angle::from_radians(PI / 2.0),
        longitude: Angle::ZERO,
        nvector: Vec3::UNIT_Z,
    };

    /// South Pole.
    pub const SOUTH_POLE: Point = Point {
        latitude: Angle::from_radians(-PI / 2.0),
        longitude: Angle::ZERO,
        nvector: Vec3::NEG_UNIT_Z,
    };

    /// Returns the latitude of this point.
    pub fn latitude(&self) -> Angle {
        self.latitude
    }

    /// Returns the longitude of this point.
    pub fn longitude(&self) -> Angle {
        self.longitude
    }
}

impl HorizontalPosition for Point {
    fn from_lat_long_radians(latitude: f64, longitude: f64) -> Self {
        if eq_lat_rads_north_pole(latitude) {
            Point::NORTH_POLE
        } else if eq_lat_rads_south_pole(latitude) {
            Point::SOUTH_POLE
        } else {
            let nvector = nvector_from_lat_long_radians(latitude, longitude);
            Point {
                latitude: Angle::from_radians(latitude),
                longitude: Angle::from_radians(longitude),
                nvector,
            }
        }
    }

    fn to_lat_long(&self) -> (Angle, Angle) {
        (self.latitude, self.longitude)
    }

    fn to_lat_long_degrees(&self) -> (f64, f64) {
        (self.latitude.as_degrees(), self.longitude.as_degrees())
    }

    fn to_lat_long_radians(&self) -> (f64, f64) {
        (self.latitude.as_radians(), self.longitude.as_radians())
    }

    fn from_nvector(nvector: Vec3) -> Self {
        if nvector.z() == 1.0 {
            Point::NORTH_POLE
        } else if nvector.z() == -1.0 {
            Point::SOUTH_POLE
        } else {
            let (latitude, longitude) = nvector_to_lat_long_radians(nvector);
            Point {
                latitude: Angle::from_radians(latitude),
                longitude: Angle::from_radians(longitude),
                nvector,
            }
        }
    }

    fn as_nvector(&self) -> Vec3 {
        self.nvector
    }

    fn antipode(&self) -> Self {
        let lat = -self.latitude;
        let lon = if self.longitude > Angle::ZERO {
            self.longitude - Angle::HALF_CIRCLE
        } else {
            self.longitude + Angle::HALF_CIRCLE
        };
        Self::from_lat_long(lat, lon)
    }

    fn is_antipode(&self, other: Self) -> bool {
        if self.latitude() + other.latitude() != Angle::ZERO {
            false
        } else if self.latitude().as_degrees().abs() == 90.0 {
            // at pole
            true
        } else {
            self.longitude().as_degrees().abs() + other.longitude().as_degrees().abs() == 180.0
        }
    }
}

impl PartialEq for Point {
    fn eq(&self, other: &Self) -> bool {
        self.latitude == other.latitude && self.longitude == other.longitude
    }
}

impl HorizontalPosition for Vec3 {
    fn from_lat_long_radians(latitude: f64, longitude: f64) -> Self {
        if eq_lat_rads_north_pole(latitude) {
            Vec3::UNIT_Z
        } else if eq_lat_rads_south_pole(latitude) {
            Vec3::NEG_UNIT_Z
        } else {
            nvector_from_lat_long_radians(latitude, longitude)
        }
    }

    fn to_lat_long(&self) -> (Angle, Angle) {
        let (lat, lon) = self.to_lat_long_radians();
        (Angle::from_radians(lat), Angle::from_radians(lon))
    }

    fn to_lat_long_degrees(&self) -> (f64, f64) {
        let (lat, lon) = self.to_lat_long_radians();
        (lat.to_degrees(), lon.to_degrees())
    }

    fn to_lat_long_radians(&self) -> (f64, f64) {
        nvector_to_lat_long_radians(self.as_nvector())
    }

    fn from_nvector(nvector: Vec3) -> Self {
        nvector
    }

    fn as_nvector(&self) -> Vec3 {
        *self
    }

    fn antipode(&self) -> Self {
        -1.0 * *self
    }

    fn is_antipode(&self, other: Self) -> bool {
        *self + other == Vec3::ZERO
    }
}

/// Converts the given latitude/longitude in radians to the equivalent n-vector.
fn nvector_from_lat_long_radians(latitude_rads: f64, longitude_rads: f64) -> Vec3 {
    let cl = latitude_rads.cos();
    let x = cl * longitude_rads.cos();
    let y = cl * longitude_rads.sin();
    let z = latitude_rads.sin();
    Vec3::new(x, y, z)
}

/// Converts the given n-vector to the equivalent latitude/longitude in radians.
fn nvector_to_lat_long_radians(nvector: Vec3) -> (f64, f64) {
    let x = nvector.x();
    let y = nvector.y();
    let z = nvector.z();
    let lat = z.atan2((x * x + y * y).sqrt());
    let lon = y.atan2(x);
    (lat, lon)
}

/// Is given latitude in radians at the north pole?
fn eq_lat_rads_north_pole(latitude_rads: f64) -> bool {
    latitude_rads == PI / 2.0
}

/// Is given latitude in radians at the south pole?
fn eq_lat_rads_south_pole(latitude_rads: f64) -> bool {
    latitude_rads == -PI / 2.0
}

#[cfg(test)]
mod tests {

    use crate::{HorizontalPosition, Point, Vec3};

    #[test]
    fn point_antipode() {
        assert_eq!(
            Point::from_lat_long_degrees(-45.0, -26.0),
            Point::from_lat_long_degrees(45.0, 154.0)
                .antipode()
                .round_d7()
        );
        assert_eq!(Point::NORTH_POLE, Point::SOUTH_POLE.antipode());
        assert_eq!(Point::SOUTH_POLE, Point::NORTH_POLE.antipode());
    }

    #[test]
    fn vec3_antipode() {
        assert_eq!(
            Vec3::from_lat_long_degrees(-45.0, -26.0),
            Vec3::from_lat_long_degrees(45.0, 154.0)
                .antipode()
                .round_d7()
        );
        assert_eq!(Vec3::UNIT_Z, Vec3::NEG_UNIT_Z.antipode());
        assert_eq!(Vec3::NEG_UNIT_Z, Vec3::UNIT_Z.antipode());
    }
}
