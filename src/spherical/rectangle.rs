use std::{cmp::Ordering, f64::consts::PI};

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

// TODO(CL): Exmaples
impl Rectangle {
    /// Empty rectangle.
    pub const EMPTY: Rectangle = Self {
        lat: LatitudeInterval::EMPTY,
        lng: LongitudeInterval::EMPTY,
    };

    /// Creates a full rectangle.
    pub const FULL: Rectangle = Self {
        lat: LatitudeInterval::FULL,
        lng: LongitudeInterval::FULL,
    };

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

    /// Compares the latitude intervalsof this rectangle and the given one: the [greater](Ordering::Greater) latitude interval is defined as
    /// the interval that is northernmost overall (including both low and high latitudes).
    pub fn cmp_by_latitude(&self, o: Self) -> Ordering {
        let a = self.lat.lo.as_radians() + self.lat.hi.as_radians();
        let b = o.lat.lo.as_radians() + o.lat.hi.as_radians();
        if a < b {
            Ordering::Less
        } else if a > b {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    }

    /// Compares the longitude intervals of this rectangle and the given one: the [greater](Ordering::Greater) longitude interval is defined as
    /// the interval that is easternmost overall (including both low and high longitudes).
    pub fn cmp_by_longitude(&self, o: Self) -> Ordering {
        let a = self.lng.lo.as_radians() + self.lng.hi.as_radians();
        let b = o.lng.lo.as_radians() + o.lng.hi.as_radians();
        if a < b {
            Ordering::Less
        } else if a > b {
            Ordering::Greater
        } else {
            Ordering::Equal
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

    /// Determines whether this rectangle is [full](crate::spherical::Rectangle::FULL).
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

    /// Determines whether this rectangle is [empty](crate::spherical::Rectangle::EMPTY).
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

    /// Northernmost and easternmost - or 'high', point of this rectangle.
    pub fn north_east(&self) -> LatLong {
        LatLong::new(self.lat.hi, self.lng.hi)
    }

    /// Southernmost and westernmost - or 'low', point of this rectangle.
    pub fn south_west(&self) -> LatLong {
        LatLong::new(self.lat.lo, self.lng.lo)
    }

    /// Expands (`amount > 0`) or shrinks (`amount < 0`) this rectangle by the given amount
    /// on each side in latitude and longitude direction.
    /// - Latitudes are clampled to the range [-90, 90], as such the full latitude range
    ///   remains full only if the margin is positive.
    /// - Longitudes "wrap around" at +/-180 degrees, as such the full longitude range remains full.
    /// - If either the latitude or longitude interval becomes empty after
    ///   expansion by a negative margin, the result is [empty](crate::spherical::Rectangle::EMPTY).
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Angle;
    /// use jord::spherical::Rectangle;
    ///
    /// let r: Rectangle = Rectangle::from_nesw(
    ///     Angle::from_degrees(10.0),
    ///     Angle::from_degrees(45.0),
    ///     Angle::from_degrees(-10.0),
    ///     Angle::from_degrees(10.0),
    /// );
    /// let expanded = r.expand(Angle::from_degrees(1.0));
    /// let e = Rectangle::from_nesw(
    ///     Angle::from_degrees(11.0),
    ///     Angle::from_degrees(46.0),
    ///     Angle::from_degrees(-11.0),
    ///     Angle::from_degrees(9.0),
    /// );
    /// assert_eq!(e, expanded);
    /// ```
    pub fn expand(&self, amount: Angle) -> Self {
        let lat = self.lat.expand(amount);
        let lng = self.lng.expand(amount);
        if lat.is_empty() || lng.is_empty() {
            Self::EMPTY
        } else {
            Self {
                lat: lat.intersection(LatitudeInterval::FULL),
                lng,
            }
        }
    }

    /// Expands this rectangle to include the north pole - the longitude interval becomes
    /// [full](crate::spherical::Rectangle::is_longitude_full) as a result.
    /// As such only the southermost latitude of this rectangle is kept.
    pub fn expand_to_north_pole(&self) -> Self {
        Self {
            lat: LatitudeInterval::new(self.lat.lo, Angle::QUARTER_CIRCLE),
            lng: LongitudeInterval::FULL,
        }
    }

    /// Expands this rectangle to include the south pole - the longitude interval becomes
    /// [full](crate::spherical::Rectangle::is_longitude_full) as a result.
    /// As such only the northermost latitude of this rectangle is kept.
    pub fn expand_to_south_pole(&self) -> Self {
        Self {
            lat: LatitudeInterval::new(-Angle::QUARTER_CIRCLE, self.lat.hi),
            lng: LongitudeInterval::FULL,
        }
    }

    /// If this rectangle does not include either pole, returns it unmodified.
    /// Otherwise expands the longitude range to full so that the rectangle
    /// contains all possible representations of the contained pole(s).
    pub fn polar_closure(&self) -> Self {
        if self.lat.lo == -Angle::QUARTER_CIRCLE || self.lat.hi == Angle::QUARTER_CIRCLE {
            Self {
                lat: self.lat,
                lng: LongitudeInterval::FULL,
            }
        } else {
            *self
        }
    }

    /// Returns the smallest rectangle containing the union of this rectangle and the given rectangle.
    pub fn union(&self, o: Self) -> Self {
        Rectangle {
            lat: self.lat.union(o.lat),
            lng: self.lng.union(o.lng),
        }
    }

    /// Returns the smallest rectangle containing the union of all the given rectangles.
    pub fn from_union(all: &[Rectangle]) -> Self {
        let mut res = Self::EMPTY;
        for r in all {
            res.lat.mut_union(r.lat);
            res.lng.mut_union(r.lng);
        }
        res
    }
}

/// latitude interval: {@link #lo} is assumed to be less than {@link #hi}, otherwise the interval is empty.
#[derive(PartialEq, Clone, Copy, Debug, Default)]
struct LatitudeInterval {
    lo: Angle,
    hi: Angle,
}

impl LatitudeInterval {
    const EMPTY: LatitudeInterval = Self {
        lo: Angle::QUARTER_CIRCLE,
        hi: Angle::ZERO,
    };

    const FULL: LatitudeInterval = Self {
        lo: Angle::NEG_QUARTER_CIRCLE,
        hi: Angle::QUARTER_CIRCLE,
    };

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

    /// Returns an interval that has been expanded/shrinked on each side by the given amount.
    fn expand(&self, amount: Angle) -> Self {
        if self.is_empty() {
            *self
        } else {
            Self {
                lo: self.lo - amount,
                hi: self.hi + amount,
            }
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

    /// Return the intersection of this interval with the given interval.
    /// Empty intervals do not need to be special-cased.
    fn intersection(&self, o: Self) -> Self {
        let lo = if self.lo >= o.lo { self.lo } else { o.lo };
        let hi = if self.hi <= o.hi { self.hi } else { o.hi };
        Self { lo, hi }
    }

    /// Returns the smallest latitude interval that contains this latitude interval and the given latitude
    /// interval.
    fn union(&self, o: Self) -> Self {
        let mut r = *self;
        r.mut_union(o);
        r
    }

    fn mut_union(&mut self, o: Self) {
        if self.is_empty() {
            self.lo = o.lo;
            self.hi = o.hi;
        } else if o.is_empty() {
            // no-op
        } else {
            if self.lo > o.lo {
                self.lo = o.lo;
            }
            if self.hi < o.hi {
                self.hi = o.hi;
            }
        }
    }
}

#[derive(PartialEq, Clone, Copy, Debug, Default)]
struct LongitudeInterval {
    lo: Angle,
    hi: Angle,
}

impl LongitudeInterval {
    const EMPTY: LongitudeInterval = Self {
        lo: Angle::HALF_CIRCLE,
        hi: Angle::NEG_HALF_CIRCLE,
    };

    const FULL: LongitudeInterval = Self {
        lo: Angle::NEG_HALF_CIRCLE,
        hi: Angle::HALF_CIRCLE,
    };

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
        if longitude == Angle::NEG_HALF_CIRCLE {
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

    /// Returns an interval that has been expanded/shrinked on each side by the given amount.
    fn expand(&self, amount: Angle) -> Self {
        if amount > Angle::ZERO {
            if self.is_empty() {
                return *self;
            }
            // Check whether this interval will be full after expansion, allowing
            // for a 1-bit rounding error when computing each endpoint.
            if self.len() + 2.0 * amount + 2.0 * Angle::DBL_EPSILON >= Angle::FULL_CIRCLE {
                return Self::FULL;
            }
        } else {
            if self.is_full() {
                return *self;
            }
            // Check whether this interval will be empty after expansion, allowing
            // for a 1-bit rounding error when computing each endpoint.
            if self.len() + 2.0 * amount - 2.0 * Angle::DBL_EPSILON <= Angle::ZERO {
                return Self::EMPTY;
            }
        }

        let mut lo = (self.lo - amount).as_radians() % (2.0 * PI);
        let hi = (self.hi + amount).as_radians() % (2.0 * PI);
        if lo < -PI {
            lo = PI;
        }
        Self {
            lo: Angle::from_radians(lo),
            hi: Angle::from_radians(hi),
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
        self.lo == Angle::NEG_HALF_CIRCLE && self.hi == Angle::HALF_CIRCLE
    }

    /// Returns true if this longitude interval is empty.
    fn is_empty(&self) -> bool {
        self.lo == Angle::HALF_CIRCLE && self.hi == Angle::NEG_HALF_CIRCLE
    }

    /// Returns true if this longitude interval is inverted.
    fn is_inverted(&self) -> bool {
        self.lo > self.hi
    }

    /// Returns the smallest longitude interval that contains this longitude interval and the given longitude
    /// interval.
    fn union(&self, o: Self) -> Self {
        let mut r = *self;
        r.mut_union(o);
        r
    }

    fn len(&self) -> Angle {
        let mut len = self.hi - self.lo;
        if len >= Angle::ZERO {
            len
        } else {
            len = len + Angle::FULL_CIRCLE;
            // Empty intervals have a negative length.
            if len > Angle::ZERO {
                len
            } else {
                Angle::from_radians(-1.0)
            }
        }
    }

    fn mut_union(&mut self, o: Self) {
        if o.is_empty() {
            // no-op.
        } else if self.contains_lng(o.lo) {
            if self.contains_lng(o.hi) {
                // Either this interval contains o, or the union of the two intervals is the full interval.
                if self.contains_int(o) {
                    // no-op.
                } else {
                    self.lo = Angle::NEG_HALF_CIRCLE;
                    self.hi = Angle::HALF_CIRCLE;
                }
            } else {
                self.hi = o.hi;
            }
        } else if self.contains_lng(o.hi) {
            self.lo = o.lo;
        } else if self.is_empty() || o.contains_lng(self.lo) {
            // This interval contains neither endpoint of o. This means that either y contains all of this
            // interval, or the two intervals are disjoint.
            self.lo = o.lo;
            self.hi = o.hi;
        } else {
            // Check which pair of endpoints are closer together.
            let dlo = Self::positive_distance(o.hi, self.lo);
            let dhi = Self::positive_distance(self.hi, o.lo);
            if dlo < dhi {
                self.lo = o.lo;
            } else {
                self.hi = o.hi;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::cmp::Ordering;

    use crate::{spherical::MinorArc, Angle, LatLong, NVector};

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
        assert!(!Rectangle::EMPTY.contains_point(LatLong::from_degrees(0.0, 0.0)));
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

    // contains_rectangle

    #[test]
    fn contains_rectangle() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
            Angle::ZERO,
            Angle::ZERO,
        );
        let b = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(20.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(10.0),
        );
        assert_contains_rect(a, b, true);
        assert_contains_rect(b, a, false);
    }

    #[test]
    fn contains_rectangle_empty() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(20.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(10.0),
        );
        assert_contains_rect(a, Rectangle::EMPTY, true);
        assert_contains_rect(Rectangle::EMPTY, a, false);
        assert_contains_rect(Rectangle::EMPTY, Rectangle::EMPTY, true);
    }

    #[test]
    fn contains_rectangle_intersecting() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(20.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(10.0),
        );
        let b = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
            Angle::from_degrees(15.0),
            Angle::from_degrees(15.0),
        );
        assert_contains_rect(a, b, false);
        assert_contains_rect(b, a, false);
    }

    #[test]
    fn contains_rectangle_non_verlapping() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(20.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(10.0),
        );
        let b = Rectangle::from_nesw(
            Angle::from_degrees(40.0),
            Angle::from_degrees(45.0),
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
        );
        assert_contains_rect(a, b, false);
        assert_contains_rect(b, a, false);
    }

    #[test]
    fn contains_rectangle_not_by_ongitude() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
            Angle::ZERO,
            Angle::ZERO,
        );
        let b = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(40.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(10.0),
        );
        assert_contains_rect(a, b, false);
        assert_contains_rect(b, a, false);
    }

    #[test]
    fn contains_rectangle_other_longitude_empty() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
            Angle::ZERO,
            Angle::ZERO,
        );
        let b = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(-180.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(180.0),
        );
        assert_contains_rect(a, b, true);
    }

    #[test]
    fn contains_rectangle_other_longitude_inverted_does_not_contain() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
            Angle::ZERO,
            Angle::from_degrees(20.0),
        );
        let b = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(30.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(40.0),
        );
        assert_contains_rect(a, b, false);
    }

    #[test]
    fn contains_rectangle_other_longitude_inverted_this_full() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(180.0),
            Angle::ZERO,
            Angle::from_degrees(-180.0),
        );
        let b = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(30.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(40.0),
        );
        assert_contains_rect(a, b, true);
    }

    fn assert_contains_rect(a: Rectangle, b: Rectangle, expected: bool) {
        assert_eq!(expected, a.contains_rectangle(b));
        assert_eq!(expected, a.union(b) == a);
    }

    // from_minor_arc

    #[test]
    fn from_minor_arc_from_north_pole() {
        let ma = MinorArc::new(
            NVector::from_lat_long_degrees(90.0, 0.0),
            NVector::from_lat_long_degrees(45.0, 45.0),
        );
        let actual = Rectangle::from_minor_arc(ma);
        let expected = Rectangle::from_nesw(
            Angle::from_degrees(90.0),
            Angle::from_degrees(45.0),
            Angle::from_degrees(45.0),
            Angle::ZERO,
        );
        assert_rect_eq_d7(expected, actual);
    }

    #[test]
    fn from_minor_arc_from_south_pole() {
        let ma = MinorArc::new(
            NVector::from_lat_long_degrees(-90.0, 0.0),
            NVector::from_lat_long_degrees(-45.0, 45.0),
        );
        let actual = Rectangle::from_minor_arc(ma);
        let expected = Rectangle::from_nesw(
            Angle::from_degrees(-45.0),
            Angle::from_degrees(45.0),
            Angle::from_degrees(-90.0),
            Angle::ZERO,
        );
        assert_rect_eq_d7(expected, actual);
    }

    #[test]
    fn from_minor_arc_iso_latitude_north() {
        let ma = MinorArc::new(
            NVector::from_lat_long_degrees(45.0, 0.0),
            NVector::from_lat_long_degrees(45.0, 10.0),
        );
        let actual = Rectangle::from_minor_arc(ma);
        let expected = Rectangle::from_nesw(
            Angle::from_degrees(45.1092215),
            Angle::from_degrees(10.0),
            Angle::from_degrees(45.0),
            Angle::ZERO,
        );
        assert_rect_eq_d7(expected, actual);
    }

    #[test]
    fn from_minor_arc_iso_latitude_south() {
        let ma: MinorArc = MinorArc::new(
            NVector::from_lat_long_degrees(-45.0, 0.0),
            NVector::from_lat_long_degrees(-45.0, 10.0),
        );
        let actual = Rectangle::from_minor_arc(ma);
        let expected = Rectangle::from_nesw(
            Angle::from_degrees(-45.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(-45.1092215),
            Angle::ZERO,
        );
        assert_rect_eq_d7(expected, actual);
    }

    #[test]
    fn from_minor_arc_iso_longitude() {
        let ma = MinorArc::new(
            NVector::from_lat_long_degrees(0.0, 0.0),
            NVector::from_lat_long_degrees(10.0, 0.0),
        );
        let actual = Rectangle::from_minor_arc(ma);
        let expected = Rectangle::from_nesw(
            Angle::from_degrees(10.0),
            Angle::ZERO,
            Angle::ZERO,
            Angle::ZERO,
        );
        assert_rect_eq_d7(expected, actual);
        for lat in -900..900 {
            let lat_f = lat as f64;
            let p = LatLong::from_degrees(lat_f / 10.0, 0.0);
            if lat >= 0 && lat <= 100 {
                assert!(actual.contains_point(p));
            } else {
                assert!(!actual.contains_point(p));
            }
        }
    }

    #[test]
    fn from_minor_arc_smallest_longitude_interval() {
        let ma = MinorArc::new(
            NVector::from_lat_long_degrees(45.0, 0.0),
            NVector::from_lat_long_degrees(46.0, -10.0),
        );
        let actual = Rectangle::from_minor_arc(ma);
        let expected = Rectangle::from_nesw(
            Angle::from_degrees(46.0),
            Angle::ZERO,
            Angle::from_degrees(45.0),
            Angle::from_degrees(-10.0),
        );
        assert_rect_eq_d7(expected, actual);
        assert!(actual.contains_point(LatLong::from_degrees(45.5, -5.0)));
        assert!(!actual.contains_point(LatLong::from_degrees(45.5, 1.0)));
    }

    fn assert_rect_eq_d7(e: Rectangle, a: Rectangle) {
        assert_eq!(e.north_east(), a.north_east().round_d7());
        assert_eq!(e.south_west(), a.south_west().round_d7());
    }

    // from_nesw

    #[test]
    fn from_nesw_longitude_spans_180() {
        // Outside at meridian 0.
        check_nesw(false, ll(0, 0), 10, -170, -10, 170);

        // Inside at meridian 180.
        check_nesw(true, ll(0, 180), 10, -170, -10, 170);

        // Inside, either side of meridian 180.
        check_nesw(true, ll(0, -175), 10, -170, -10, 170);
        check_nesw(true, ll(0, 175), 10, -170, -10, 170);

        // Outside, either side of meridian 180.
        check_nesw(false, ll(0, -165), 10, -170, -10, 170);
        check_nesw(false, ll(0, 165), 10, -170, -10, 170);
    }

    #[test]
    fn from_nesw_nominal_inside() {
        check_nesw(true, ll(0, 0), 10, 10, -10, -10);
        // on north parallel.
        check_nesw(true, ll(10, 0), 10, 10, -10, -10);
        // on south parallel.
        check_nesw(true, ll(-10, 0), 10, 10, -10, -10);
        // on east meridian.
        check_nesw(true, ll(0, 10), 10, 10, -10, -10);
        // on west meridian.
        check_nesw(true, ll(0, -10), 10, 10, -10, -10);
    }

    #[test]
    fn from_nesw_nominal_outside() {
        check_nesw(false, ll(20, 0), 10, 10, -10, -10);
        check_nesw(false, ll(-20, 0), 10, 10, -10, -10);
        check_nesw(false, ll(0, 20), 10, 10, -10, -10);
        check_nesw(false, ll(0, -20), 10, 10, -10, -10);
        check_nesw(false, ll(90, 0), 10, 10, -10, -10);
        check_nesw(false, ll(-90, 0), 10, 10, -10, -10);
    }

    #[test]
    fn from_nesw_north_pole() {
        // At the pole.
        check_nesw(true, ll(90, 45), 90, 50, 80, 40);

        // Inside a rectangle extending to the north pole.
        check_nesw(true, ll(85, 45), 90, 50, 80, 40);

        // Outside (by longitude) a rectangle extending to the north pole.
        check_nesw(false, ll(85, 35), 90, 50, 80, 40);

        // Outside (by latitude) a rectangle extending to the north pole.
        check_nesw(false, ll(76, 0), 90, 50, 80, 40);
    }

    #[test]
    fn from_nesw_over_both_hemispheres() {
        check_nesw(true, ll(0, 0), 10, 170, -10, -170);
        check_nesw(false, ll(0, 180), 10, 170, -10, -170);
    }

    #[test]
    fn from_nesw_reversed_latitudes() {
        check_nesw(false, ll(0, 0), -10, 10, 10, -10);
    }

    #[test]
    fn from_nesw_south_pole() {
        // At the pole.
        check_nesw(true, ll(-90, 45), -80, 50, -90, 40);

        // Inside a rectangle extending to the south pole.
        check_nesw(true, ll(-85, 45), -80, 50, -90, 40);

        // Outside (by longitude) a rectangle extending to the south pole.
        check_nesw(false, ll(-85, 35), -80, 50, -90, 40);

        // Outside (by latitude) a rectangle extending to the south pole.
        check_nesw(false, ll(-76, 0), -80, 50, -90, 40);
    }

    #[test]
    fn from_nesw_zero_rectangle() {
        check_nesw(true, ll(10, 10), 10, 10, 10, 10);
        check_nesw(false, ll(10, 11), 10, 10, 10, 10);
        check_nesw(false, ll(11, 10), 10, 10, 10, 10);
    }

    fn check_nesw(expected: bool, test_point: LatLong, n: i64, e: i64, s: i64, w: i64) {
        let rect = Rectangle::from_nesw(
            Angle::from_degrees(n as f64),
            Angle::from_degrees(e as f64),
            Angle::from_degrees(s as f64),
            Angle::from_degrees(w as f64),
        );
        let actual = rect.contains_point(test_point);
        assert_eq!(expected, actual);
    }

    #[test]
    fn polar_closure_no_pole() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(20.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(10.0),
        );
        assert_eq!(a, a.polar_closure());
        assert!(!a.is_longitude_full());
    }

    #[test]
    fn polar_closure_north_pole() {
        let actual = Rectangle::from_nesw(
            Angle::from_degrees(90.0),
            Angle::from_degrees(50.0),
            Angle::from_degrees(80.0),
            Angle::from_degrees(40.0),
        )
        .polar_closure();

        // full longitude range.
        assert!(actual.is_longitude_full());

        assert!(actual.contains_point(ll(90, 0)));
        assert!(actual.contains_point(ll(85, 45)));
        // all longitudes are in
        assert!(actual.contains_point(ll(85, 35)));
        // out by latitude
        assert!(!actual.contains_point(ll(76, 0)));
    }

    #[test]
    fn polar_closure_south_pole() {
        let actual = Rectangle::from_nesw(
            Angle::from_degrees(-80.0),
            Angle::from_degrees(50.0),
            Angle::from_degrees(-90.0),
            Angle::from_degrees(40.0),
        )
        .polar_closure();

        // full longitude range.
        assert!(actual.is_longitude_full());

        assert!(actual.contains_point(ll(-90, 0)));
        assert!(actual.contains_point(ll(-85, 45)));
        // all longitudes are in
        assert!(actual.contains_point(ll(-85, 35)));
        // out by latitude
        assert!(!actual.contains_point(ll(-76, 0)));
    }

    // union
    #[test]
    fn union_both_empty() {
        assert_eq!(Rectangle::EMPTY, Rectangle::EMPTY.union(Rectangle::EMPTY));
    }

    #[test]
    fn union_non_overlapping() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(20.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(10.0),
        );
        let b = Rectangle::from_nesw(
            Angle::from_degrees(40.0),
            Angle::from_degrees(45.0),
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
        );
        let union = a.union(b);
        assert!(union.contains_rectangle(a));
        assert!(union.contains_rectangle(b));

        //p is neither in a nor b, but is in their union.
        let p = ll(25, 25);
        assert!(!a.contains_point(p));
        assert!(!b.contains_point(p));
        assert!(union.contains_point(p));
    }

    #[test]
    fn union_one_empty() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(20.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(10.0),
        );
        assert_eq!(a, a.union(Rectangle::EMPTY));
        assert_eq!(a, Rectangle::EMPTY.union(a));
    }

    #[test]
    fn overlapping() {
        let a = Rectangle::from_nesw(
            Angle::from_degrees(20.0),
            Angle::from_degrees(20.0),
            Angle::from_degrees(10.0),
            Angle::from_degrees(10.0),
        );
        let b = Rectangle::from_nesw(
            Angle::from_degrees(30.0),
            Angle::from_degrees(30.0),
            Angle::from_degrees(15.0),
            Angle::from_degrees(15.0),
        );
        let union: Rectangle = a.union(b);
        assert!(union.contains_rectangle(a));
        assert!(union.contains_rectangle(b));

        //pa is only in a, but is in the union.
        let pa = ll(14, 14);
        assert!(a.contains_point(pa));
        assert!(!b.contains_point(pa));
        assert!(union.contains_point(pa));

        //pb is only in a, but is in the union.
        let pb = ll(25, 25);
        assert!(!a.contains_point(pb));
        assert!(b.contains_point(pb));
        assert!(union.contains_point(pb));
    }

    // from_union

    #[test]
    fn from_union_all_empty() {
        let all = vec![Rectangle::EMPTY, Rectangle::EMPTY, Rectangle::EMPTY];
        let union = Rectangle::from_union(&all);
        assert_eq!(Rectangle::EMPTY, union);
    }

    #[test]
    fn from_union_all_overlapping() {
        let all = vec![
            Rectangle::from_nesw(
                Angle::from_degrees(20.0),
                Angle::from_degrees(20.0),
                Angle::from_degrees(10.0),
                Angle::from_degrees(10.0),
            ),
            Rectangle::from_nesw(
                Angle::from_degrees(30.0),
                Angle::from_degrees(30.0),
                Angle::from_degrees(15.0),
                Angle::from_degrees(15.0),
            ),
            Rectangle::from_nesw(
                Angle::from_degrees(40.0),
                Angle::from_degrees(40.0),
                Angle::from_degrees(5.0),
                Angle::from_degrees(5.0),
            ),
        ];
        let union = Rectangle::from_union(&all);
        for r in all.iter() {
            assert!(union.contains_rectangle(*r));
        }
    }

    #[test]
    fn is_longitude_full() {
        assert!(Rectangle::from_nesw(
            Angle::ZERO,
            Angle::from_degrees(180.0),
            Angle::ZERO,
            Angle::from_degrees(-180.0)
        )
        .is_longitude_full());

        assert!(!Rectangle::from_nesw(
            Angle::ZERO,
            Angle::from_degrees(179.0),
            Angle::ZERO,
            Angle::from_degrees(-180.0)
        )
        .is_longitude_full());

        assert!(!Rectangle::from_nesw(
            Angle::ZERO,
            Angle::from_degrees(180.0),
            Angle::ZERO,
            Angle::from_degrees(-179.0)
        )
        .is_longitude_full());
    }

    #[test]
    fn expand_to_north_pole() {
        let r = Rectangle::from_nesw(
            Angle::from_degrees(10.0),
            Angle::from_degrees(45.0),
            Angle::from_degrees(-10.0),
            Angle::from_degrees(10.0),
        );
        let expanded = r.expand_to_north_pole();
        assert!(expanded.is_longitude_full());
        assert!(expanded.contains_point(ll(90, 0)));
        assert!(expanded.contains_point(ll(-10, 0)));
    }

    #[test]
    fn expand_to_south_pole() {
        let r: Rectangle = Rectangle::from_nesw(
            Angle::from_degrees(10.0),
            Angle::from_degrees(45.0),
            Angle::from_degrees(-10.0),
            Angle::from_degrees(10.0),
        );
        let expanded = r.expand_to_south_pole();
        assert!(expanded.is_longitude_full());
        assert!(expanded.contains_point(ll(-90, 0)));
        assert!(expanded.contains_point(ll(10, 0)));
    }

    #[test]
    fn expand_nominal() {
        let r: Rectangle = Rectangle::from_nesw(
            Angle::from_degrees(10.0),
            Angle::from_degrees(45.0),
            Angle::from_degrees(-10.0),
            Angle::from_degrees(10.0),
        );
        let expanded = r.expand(Angle::from_degrees(1.0));
        let e = Rectangle::from_nesw(
            Angle::from_degrees(11.0),
            Angle::from_degrees(46.0),
            Angle::from_degrees(-11.0),
            Angle::from_degrees(9.0),
        );
        assert_eq!(e, expanded);
    }

    #[test]
    fn expand_full_lat_remains_full() {
        let r: Rectangle = Rectangle::from_nesw(
            Angle::from_degrees(90.0),
            Angle::from_degrees(45.0),
            Angle::from_degrees(-90.0),
            Angle::from_degrees(10.0),
        );
        let expanded = r.expand(Angle::from_degrees(1.0));
        let e = Rectangle::from_nesw(
            Angle::from_degrees(90.0),
            Angle::from_degrees(46.0),
            Angle::from_degrees(-90.0),
            Angle::from_degrees(9.0),
        );
        assert_eq!(e, expanded);
    }

    #[test]
    fn expand_full_lat_no_longer_full() {
        let r: Rectangle = Rectangle::from_nesw(
            Angle::from_degrees(90.0),
            Angle::from_degrees(45.0),
            Angle::from_degrees(-90.0),
            Angle::from_degrees(10.0),
        );
        let expanded = r.expand(Angle::from_degrees(-1.0));
        let e = Rectangle::from_nesw(
            Angle::from_degrees(89.0),
            Angle::from_degrees(44.0),
            Angle::from_degrees(-89.0),
            Angle::from_degrees(11.0),
        );
        assert_eq!(e, expanded);
    }

    #[test]
    fn expand_full_lng_positive_amount() {
        let r: Rectangle = Rectangle::from_nesw(
            Angle::from_degrees(10.0),
            Angle::from_degrees(180.0),
            Angle::from_degrees(-10.0),
            Angle::from_degrees(-180.0),
        );
        let expanded = r.expand(Angle::from_degrees(1.0));
        let e = Rectangle::from_nesw(
            Angle::from_degrees(11.0),
            Angle::from_degrees(180.0),
            Angle::from_degrees(-11.0),
            Angle::from_degrees(-180.0),
        );
        assert_eq!(e, expanded);
    }

    #[test]
    fn expand_full_lng_negative_amount() {
        let r: Rectangle = Rectangle::from_nesw(
            Angle::from_degrees(10.0),
            Angle::from_degrees(180.0),
            Angle::from_degrees(-10.0),
            Angle::from_degrees(-180.0),
        );
        let expanded = r.expand(Angle::from_degrees(-1.0));
        let e = Rectangle::from_nesw(
            Angle::from_degrees(9.0),
            Angle::from_degrees(180.0),
            Angle::from_degrees(-9.0),
            Angle::from_degrees(-180.0),
        );
        assert_eq!(e, expanded);
    }

    fn ll(lat: i64, lng: i64) -> LatLong {
        LatLong::from_degrees(lat as f64, lng as f64)
    }
}
