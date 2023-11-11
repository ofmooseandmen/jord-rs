use std::cmp::Ordering;

use crate::{
    numbers::{eq_zero, gte, lte},
    Angle, LatLong, Vec3,
};

use super::MinorArc;

/// A closed rectangle defined by 2 parallels and 2 meridians (inclusive).
///
/// This struct and implementation is very much based on [S2LatLngRect](https://github.com/google/s2geometry/blob/master/src/s2/s2latlng_rect.h).
#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct Rectangle {
    lat: LatitudeInterval,
    lng: LongitudeInterval,
}

impl Rectangle {
    /// Creates an empty rectangle.
    pub fn empty() -> Self {
        Self {
            lat: LatitudeInterval::empty(),
            lng: LongitudeInterval::empty(),
        }
    }

    /// Creates a full rectangle.
    pub fn full() -> Self {
        Self {
            lat: LatitudeInterval::full(),
            lng: LongitudeInterval::full(),
        }
    }

    /// Creates the minimal bounding rectangle containing the given minor arc.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{LatLong, NVector};
    /// use jord::spherical::{MinorArc, Rectangle};
    ///
    /// let a = Rectangle::from_minor_arc(MinorArc::new(
    ///     NVector::from_lat_long_degrees(45.0, 0.0),
    ///     NVector::from_lat_long_degrees(45.0, 10.0)
    /// ));
    ///
    /// assert_eq!(LatLong::from_degrees(45.1092215, 10.0), a.north_east().round_d7());
    /// assert_eq!(LatLong::from_degrees(45.0, 0.0), a.south_west().round_d7());
    /// ```
    pub fn from_minor_arc(ma: MinorArc) -> Self {
        let lls = LatLong::from_nvector(ma.start());
        let lle = LatLong::from_nvector(ma.end());
        Self {
            lat: LatitudeInterval::from_minor_arc(ma, lls, lle),
            lng: LongitudeInterval::from_minor_arc(lls, lle),
        }
    }

    /// Creates a new rectangle spanning between the given 2 parallels and 2 given meridians. Both parallels and
    /// meridians are inclusive. The resulting ranges are:
    /// - latitude: south to north
    /// - longitude: west to east
    ///
    /// The northern parallel shall be north of the southern parallel, otherwise all points are outside.
    /// Note: this method does not check that the given angles define valid latitudes/longitudes.
    pub fn from_nesw(north: Angle, east: Angle, south: Angle, west: Angle) -> Self {
        Self {
            lat: LatitudeInterval::new(south, north),
            lng: LongitudeInterval::new(west, east),
        }
    }

    /// Determines whether this rectangle contains the given point.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, LatLong};
    /// use jord::spherical::Rectangle;
    ///
    /// let a = Rectangle::from_nesw(
    ///     Angle::from_degrees(30.0),
    ///     Angle::from_degrees(30.0),
    ///     Angle::ZERO,
    ///     Angle::ZERO
    /// );
    ///
    /// assert!(a.contains_point(LatLong::from_degrees(10.0, 10.0)));
    ///
    /// // latitude above north.
    /// assert!(!a.contains_point(LatLong::from_degrees(40.0, 10.0)));
    ///
    /// // latitude below south.
    /// assert!(!a.contains_point(LatLong::from_degrees(-1.0, 10.0)));
    ///
    /// // longitude after east.
    /// assert!(!a.contains_point(LatLong::from_degrees(10.0, 40.0)));
    ///
    /// // longitude after west.
    /// assert!(!a.contains_point(LatLong::from_degrees(10.0, -10.0)));
    /// ```
    pub fn contains_point(&self, p: LatLong) -> bool {
        self.lat.contains_lat(p.latitude()) && self.lng.contains_lng(p.longitude())
    }

    /// Determines whether this rectangle contains the given rectangle.
    pub fn contains_rectangle(&self, r: Rectangle) -> bool {
        self.lat.contains_int(r.lat) && self.lng.contains_int(r.lng)
    }

    /// Southernmost and westernmost - or 'low', point of this rectangle.
    pub fn south_west(&self) -> LatLong {
        LatLong::new(self.lat.lo, self.lng.lo)
    }

    /// Northernmost and easternmost - or 'high', point of this rectangle.
    pub fn north_east(&self) -> LatLong {
        LatLong::new(self.lat.hi, self.lng.hi)
    }

    /// Determines whether this rectangle is [full](crate::spherical::Rectangle::full).
    pub fn is_full(&self) -> bool {
        self.is_latitude_full() && self.is_longitude_full()
    }

    /// Determines whether the latitude interval of this rectangle is full.
    pub fn is_latitude_full(&self) -> bool {
        self.lat.is_full()
    }

    /// Determines whether the longitude interval of this rectangle is full.
    pub fn is_longitude_full(&self) -> bool {
        self.lng.is_full()
    }

    /// Determines whether this rectangle is [empty](crate::spherical::Rectangle::empty).
    pub fn is_empty(&self) -> bool {
        self.is_latitude_empty() && self.is_longitude_empty()
    }

    /// Determines whether the latitude interval of this rectangle is empty.
    pub fn is_latitude_empty(&self) -> bool {
        self.lat.is_empty()
    }

    /// Determines whether the longitude interval of this rectangle is empty.
    pub fn is_longitude_empty(&self) -> bool {
        self.lng.is_empty()
    }

    /// Expands this rectangle to include the north pole - the longitude interval becomes
    /// [full](crate::spherical::Rectangle::is_longitude_full) as a result.
    /// As such only the southermost latitude of this rectangle is kept.
    pub fn expand_to_north_pole(&self) -> Self {
        Self {
            lat: LatitudeInterval::new(self.lat.lo, Angle::QUARTER_CIRCLE),
            lng: LongitudeInterval::full(),
        }
    }

    /// Expands this rectangle to include the south pole - the longitude interval becomes
    /// [full](crate::spherical::Rectangle::is_longitude_full) as a result.
    /// As such only the northermost latitude of this rectangle is kept.
    pub fn expand_to_south_pole(&self) -> Self {
        Self {
            lat: LatitudeInterval::new(-Angle::QUARTER_CIRCLE, self.lat.hi),
            lng: LongitudeInterval::full(),
        }
    }

    /// If this rectangle does not include either pole, returns it unmodified.
    /// Otherwise expands the longitude range to full so that the rectangle
    /// contains all possible representations of the contained pole(s).
    pub fn polar_closure(&self) -> Self {
        if self.lat.lo == -Angle::QUARTER_CIRCLE || self.lat.hi == Angle::QUARTER_CIRCLE {
            Self {
                lat: self.lat,
                lng: LongitudeInterval::full(),
            }
        } else {
            *self
        }
    }
    /// Compares the latitude intervalsof this rectangle and the given one: the [greater](Ordering::Greater) latitude interval is defined as the
    ///  interval that is northernmost overall (including both low and high latitudes).
    pub fn cmp_by_latitude(&self, o: Self) -> Ordering {
        let s = (self.lat.lo + self.lat.hi) - (o.lat.lo + o.lat.hi);
        if s == Angle::ZERO {
            Ordering::Equal
        } else if s < Angle::ZERO {
            Ordering::Less
        } else {
            Ordering::Greater
        }
    }

    /// Compares the longitude intervals of this rectangle and the given one: the [greater](Ordering::Greater) longitude interval is defined as
    /// the interval that is easternmost overall (including both low and high longitudes).
    pub fn cmp_by_longitude(&self, o: Self) -> Ordering {
        let s = (self.lng.lo + self.lng.hi) - (o.lng.lo + o.lng.hi);
        if s == Angle::ZERO {
            Ordering::Equal
        } else if s < Angle::ZERO {
            Ordering::Less
        } else {
            Ordering::Greater
        }
    }
}

/// latitude interval: {@link #lo} is assumed to be less than {@link #hi}, otherwise the interval is empty.
#[derive(PartialEq, Clone, Copy, Debug, Default)]
struct LatitudeInterval {
    lo: Angle,
    hi: Angle,
}

impl LatitudeInterval {
    fn empty() -> Self {
        Self {
            lo: Angle::from_radians(1.0),
            hi: Angle::ZERO,
        }
    }

    fn full() -> Self {
        Self {
            lo: -Angle::QUARTER_CIRCLE,
            hi: Angle::QUARTER_CIRCLE,
        }
    }

    fn new(lo: Angle, hi: Angle) -> Self {
        Self { lo, hi }
    }

    fn from_minor_arc(ma: MinorArc, lls: LatLong, lle: LatLong) -> Self {
        let n = ma.normal();
        // m = n x north pole (0, 0, 1) = (n.y(), -n.x(), 0.0).
        let m = Vec3::new(n.y(), -n.x(), 0.0);
        let ms = m.dot_prod(ma.start().as_vec3());
        let me = m.dot_prod(ma.end().as_vec3());

        let s_lat = lls.latitude();
        let e_lat = lle.latitude();

        let (mut lo, mut hi) = if s_lat > e_lat {
            (e_lat, s_lat)
        } else {
            (s_lat, e_lat)
        };

        if ms * me < 0.0 || eq_zero(ms) || eq_zero(me) {
            let max =
                Angle::from_radians((n.x() * n.x() + n.y() * n.y()).sqrt().atan2(n.z().abs()));
            if lte(ms, 0.0) && gte(me, 0.0) {
                hi = max;
            }
            if lte(me, 0.0) && gte(ms, 0.0) {
                lo = -max;
            }
        }
        Self::new(lo, hi)
    }

    /// Returns true if and only if this latitude interval contains the given latitude.
    fn contains_lat(&self, latitude: Angle) -> bool {
        latitude >= self.lo && latitude <= self.hi
    }

    /// Returns true if and only if this latitude interval contains the given latitude interval.
    fn contains_int(&self, o: Self) -> bool {
        if o.is_empty() {
            true
        } else {
            o.lo >= self.lo && o.hi <= self.hi
        }
    }

    /// Returns true if this latitude interval is empty.
    fn is_empty(&self) -> bool {
        self.lo > self.hi
    }

    /// Returns true if this latitude interval is full.
    fn is_full(&self) -> bool {
        self.lo == -Angle::QUARTER_CIRCLE && self.hi == Angle::QUARTER_CIRCLE
    }
}

#[derive(PartialEq, Clone, Copy, Debug, Default)]
struct LongitudeInterval {
    lo: Angle,
    hi: Angle,
}

impl LongitudeInterval {
    fn empty() -> Self {
        Self {
            lo: Angle::HALF_CIRCLE,
            hi: -Angle::HALF_CIRCLE,
        }
    }

    fn full() -> Self {
        Self {
            lo: -Angle::HALF_CIRCLE,
            hi: Angle::HALF_CIRCLE,
        }
    }

    fn new(lo: Angle, hi: Angle) -> Self {
        Self { lo, hi }
    }

    fn from_minor_arc(lls: LatLong, lle: LatLong) -> Self {
        let start = Self::normalised_longitude(lls.longitude());
        let end = Self::normalised_longitude(lle.longitude());
        if Self::positive_distance(start, end) <= Angle::HALF_CIRCLE {
            Self::new(start, end)
        } else {
            Self::new(end, start)
        }
    }

    /// Normalises the given longitude: if the given longitude is -180 degrees, 180 degrees is
    /// returned. This is done to workaround the discontinuity at the date line.
    fn normalised_longitude(longitude: Angle) -> Angle {
        if longitude == -Angle::HALF_CIRCLE {
            Angle::HALF_CIRCLE
        } else {
            longitude
        }
    }

    /// Computes the distance from  a to b in the range [0, 360.0).
    fn positive_distance(a: Angle, b: Angle) -> Angle {
        let d = b - a;
        if d >= Angle::ZERO {
            d
        } else {
            (b + Angle::HALF_CIRCLE) - (a - Angle::HALF_CIRCLE)
        }
    }

    fn contains_lng(&self, longitude: Angle) -> bool {
        let lng = Self::normalised_longitude(longitude);
        if self.is_inverted() {
            (lng >= self.lo || lng <= self.hi) && !self.is_empty()
        } else {
            lng >= self.lo && lng <= self.hi
        }
    }

    /// Returns true if and only if this longitude interval contains the given longitude interval.
    fn contains_int(&self, o: Self) -> bool {
        if self.is_inverted() {
            if o.is_inverted() {
                return o.lo >= self.lo && o.hi <= self.hi;
            }
            return (o.lo >= self.lo || o.hi <= self.hi) && !self.is_empty();
        }
        if o.is_inverted() {
            return self.is_full() || o.is_empty();
        }
        o.lo >= self.lo && o.hi <= self.hi
    }

    /// Returns true if this longitude interval is full.
    fn is_full(&self) -> bool {
        self.lo == -Angle::HALF_CIRCLE && self.hi == Angle::HALF_CIRCLE
    }

    /// Returns true if this longitude interval is empty.
    fn is_empty(&self) -> bool {
        self.lo == Angle::HALF_CIRCLE && self.hi == -Angle::HALF_CIRCLE
    }

    /// Returns true if this longitude interval is inverted.
    fn is_inverted(&self) -> bool {
        self.lo > self.hi
    }
}

#[cfg(test)]
mod tests {
    use std::cmp::Ordering;

    use crate::{Angle, LatLong};

    use super::Rectangle;

    // cmp_by_latitude

    #[test]
    fn cmp_by_latitude_eq() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
            Angle::ZERO,
            Angle::ZERO,
        );
        let b = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(20.0),
            Angle::ZERO,
            Angle::from_degrees(10.0),
        );
        assert_eq!(Ordering::Equal, a.cmp_by_latitude(b));
        assert_eq!(Ordering::Equal, b.cmp_by_latitude(a));
    }

    #[test]
    fn cmp_by_latitude_northernmost_in_north_hemisphere() {
        // latitude interval of a = 0 to 30 degrees.
        let a = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
            Angle::ZERO,
            Angle::ZERO,
        );
        // latitude interval of b = 0 to 20 degrees.
        let b = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(20.0),
            Angle::ZERO,
            Angle::from_degrees(10.0),
        );

        assert_eq!(Ordering::Greater, a.cmp_by_latitude(b));
        assert_eq!(Ordering::Less, b.cmp_by_latitude(a));
    }

    #[test]
    fn cmp_by_latitude_northernmost_in_south_hemisphere() {
        // latitude interval of a = -30 to -10 degrees.
        let a = Rectangle::from_nesw(
            Angle::from_degrees(-10.0),
            Angle::from_degrees(30.0),
            Angle::from_degrees(-30.0),
            Angle::ZERO,
        );
        // latitude interval of b = -30 to -20 degrees.
        let b = Rectangle::from_nesw(
            Angle::from_degrees(-20.0),
            Angle::from_degrees(20.0),
            Angle::from_degrees(-30.0),
            Angle::from_degrees(10.0),
        );

        assert_eq!(Ordering::Greater, a.cmp_by_latitude(b));
        assert_eq!(Ordering::Less, b.cmp_by_latitude(a));
    }

    // cmp_by_longitude

    #[test]
    fn cmp_by_longitude_eq() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(30.0),
            Angle::from_degrees(10.0),
            Angle::ZERO,
        );
        let b = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
            Angle::from_degrees(20.0),
            Angle::ZERO,
        );
        assert_eq!(Ordering::Equal, a.cmp_by_longitude(b));
        assert_eq!(Ordering::Equal, b.cmp_by_longitude(a));
    }

    #[test]
    fn cmp_by_longitude_easternmost_longitude_is_negative() {
        // longitude interval of a = -40 to -20 degrees.
        let a = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(-20.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(-40.0),
        );
        //longitude interval of b = -40 to -30 degrees.
        let b = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(-30.0),
            Angle::from_degrees(20.0),
            Angle::from_degrees(-40.0),
        );
        assert_eq!(Ordering::Greater, a.cmp_by_longitude(b));
        assert_eq!(Ordering::Less, b.cmp_by_longitude(a));
    }

    #[test]
    fn cmp_by_longitude_easternmost_longitude_is_positive() {
        //longitude interval of a = 0 to 30 degrees.
        let a = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(30.0),
            Angle::from_degrees(10.0),
            Angle::ZERO,
        );
        //longitude interval of b = 0 to 20 degrees.
        let b = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(20.0),
            Angle::from_degrees(20.0),
            Angle::ZERO,
        );
        assert_eq!(Ordering::Greater, a.cmp_by_longitude(b));
        assert_eq!(Ordering::Less, b.cmp_by_longitude(a));
    }

    // contains_point

    #[test]
    fn contains_point_east() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
            Angle::ZERO,
            Angle::ZERO,
        );
        assert!(a.contains_point(LatLong::from_degrees(10.0, 30.0)));
    }

    #[test]
    fn contains_point_north() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
            Angle::ZERO,
            Angle::ZERO,
        );
        assert!(a.contains_point(LatLong::from_degrees(30.0, 20.0)));
    }

    #[test]
    fn contains_point_north_pole() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(90.0),
            Angle::from_degrees(30.0),
            Angle::ZERO,
            Angle::ZERO,
        );
        let np: LatLong = LatLong::from_degrees(90.0, 160.0);
        assert!(!a.contains_point(np));
        assert!(a.polar_closure().contains_point(np));
    }

    #[test]
    fn contains_point_south() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
            Angle::ZERO,
            Angle::ZERO,
        );
        assert!(a.contains_point(LatLong::from_degrees(0.0, 20.0)));
    }

    #[test]
    fn contains_south_pole() {
        let a = Rectangle::from_nesw(
            Angle::ZERO,
            Angle::from_degrees(30.0),
            Angle::from_degrees(-90.0),
            Angle::ZERO,
        );
        let sp = LatLong::from_degrees(-90.0, 50.0);
        assert!(!a.contains_point(sp));
        assert!(a.polar_closure().contains_point(sp));
    }

    #[test]
    fn contains_point_west() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
            Angle::ZERO,
            Angle::ZERO,
        );
        assert!(a.contains_point(LatLong::from_degrees(10.0, 0.0)));
    }

    #[test]
    fn contains_point_date_line() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(-170.0),
            Angle::ZERO,
            Angle::from_degrees(170.0),
        );
        assert!(a.contains_point(LatLong::from_degrees(10.0, 180.0)));
        assert!(a.contains_point(LatLong::from_degrees(10.0, -180.0)));
    }

    #[test]
    fn contains_point_empty() {
        assert!(!Rectangle::empty().contains_point(LatLong::from_degrees(0.0, 0.0)));
    }

    #[test]
    fn contains_point_empty_longitude_interval() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(-180.0),
            Angle::ZERO,
            Angle::from_degrees(180.0),
        );
        assert!(!a.contains_point(LatLong::from_degrees(10.0, 180.0)));
    }

    #[test]
    fn contains_point_inverted() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::ZERO,
            Angle::ZERO,
            Angle::from_degrees(30.0),
        );
        assert!(a.contains_point(LatLong::from_degrees(10.0, 40.0)));

        // latitude above north.
        assert!(!a.contains_point(LatLong::from_degrees(40.0, 40.0)));

        // latitude below south.
        assert!(!a.contains_point(LatLong::from_degrees(-1.0, 40.0)));

        // outside longitude.
        assert!(!a.contains_point(LatLong::from_degrees(10.0, 10.0)));
    }

    #[test]
    fn contains_point_nominal() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
            Angle::ZERO,
            Angle::ZERO,
        );
        assert!(a.contains_point(LatLong::from_degrees(10.0, 10.0)));

        // latitude above north.
        assert!(!a.contains_point(LatLong::from_degrees(40.0, 10.0)));

        // latitude below south.
        assert!(!a.contains_point(LatLong::from_degrees(-1.0, 10.0)));

        // longitude after east.
        assert!(!a.contains_point(LatLong::from_degrees(10.0, 40.0)));

        // longitude after west.
        assert!(!a.contains_point(LatLong::from_degrees(10.0, -10.0)));
    }
}
