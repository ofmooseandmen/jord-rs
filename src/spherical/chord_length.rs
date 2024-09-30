use crate::{Angle, NVector};

/// The length of a chord: the length of the straight line segment joining two positions on the unit sphere.
///
/// A chord length is necessarily in the range [0.0, 2.0] or [negative](crate::spherical::ChordLength::NEGATIVE) (e.g represent an empty [cap](crate::spherical::Cap)).
/// Note that a chord length loses some accuracy as the length approaches 2.0:
///
/// ```
/// use jord::{Angle, NVector};
/// use jord::spherical::{ChordLength, Sphere};
///
/// let p1 = NVector::from_lat_long_degrees(90.0, 0.0);
///
/// let p2 = NVector::from_lat_long_degrees(0.0, 0.0);
/// let a1 = Sphere::angle(p1, p2);
/// let e1 = ChordLength::new(p1, p2).to_angle();
/// let d1 = (a1 - e1).abs();
/// assert!(d1.as_radians() < 2.3e-16); // about 1.4 nanometres difference
///
/// let p3 = NVector::from_lat_long_degrees(-90.0 + 1e-6, 0.0);
/// let a2 = Sphere::angle(p1, p3);
/// let e2 = ChordLength::new(p1, p3).to_angle();
/// let d2 = (a2 - e2).abs();
/// assert!(d2.as_radians() < 1.8e-8); // about 0.1 metre difference (~ worst case)
/// ```
#[derive(PartialEq, Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))] // codecov:ignore:this
pub struct ChordLength {
    length2: f64,
}

impl ChordLength {
    const MAX_CHORD_LENGTH_2: f64 = 4.0;

    /// Negative chord length (invalid).
    pub const NEGATIVE: ChordLength = Self { length2: -1.0 };

    /// Zero chord length (minimum value).
    pub const ZERO: ChordLength = Self { length2: 0.0 };

    /// Maximum chord length.
    pub const MAX: ChordLength = Self {
        length2: Self::MAX_CHORD_LENGTH_2,
    };

    #[inline]
    pub(crate) fn length2(&self) -> f64 {
        self.length2
    }

    pub(crate) fn from_squared_length(length2: f64) -> Self {
        Self { length2 }
    }

    /// Length of the chord joining the two given position.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, NVector, spherical::ChordLength};
    ///
    /// let c = ChordLength::new(
    ///   NVector::from_lat_long_degrees(90.0, 0.0),
    ///   NVector::from_lat_long_degrees(0.0, 0.0)
    /// );
    /// assert_eq!(Angle::QUARTER_CIRCLE.round_d7(), c.to_angle().round_d7());
    /// ```
    pub fn new(p1: NVector, p2: NVector) -> Self {
        let l2 = (p1.as_vec3() - p2.as_vec3()).squared_norm();
        Self {
            length2: l2.min(Self::MAX_CHORD_LENGTH_2),
        }
    }

    /// Converts the given central angle between 2 positions to the equivalent chord length on the unit sphere.
    /// The given angle is normalised to the range [0, PI].
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, NVector, spherical::ChordLength};
    ///
    /// assert_eq!(Angle::ZERO, ChordLength::from_angle(Angle::ZERO).to_angle());
    /// ```
    ///
    /// See also [central angle](crate::spherical::Sphere::angle)
    pub fn from_angle(angle: Angle) -> Self {
        let abs_angle: Angle = angle.abs();
        if abs_angle == Angle::HALF_CIRCLE {
            return Self::MAX;
        }
        let a = abs_angle.normalised_to(Angle::HALF_CIRCLE);
        let l = 2.0 * (a.as_radians() * 0.5).sin();
        Self { length2: l * l }
    }

    /// Converts this chord length to the equivalent central angle between the 2 positions joined by the chord.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::{Angle, NVector, spherical::ChordLength};
    ///
    /// let c = ChordLength::new(
    ///   NVector::from_lat_long_degrees(90.0, 0.0),
    ///   NVector::from_lat_long_degrees(-90.0, 0.0)
    /// );
    /// assert_eq!(Angle::HALF_CIRCLE, c.to_angle());
    /// ```
    pub fn to_angle(&self) -> Angle {
        if self.length2 < 0.0 {
            return Angle::from_radians(-1.0);
        }
        Angle::from_radians(2.0 * (self.length2.sqrt() * 0.5).asin())
    }
}

// length2 is always in range [0.0, 2.0] or equal to -1.0.
impl Eq for ChordLength {}

impl PartialOrd for ChordLength {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for ChordLength {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // length2 is always in range [0.0, 2.0] or equal to -1.0.
        let d = self.length2 - other.length2;
        if d == 0.0 {
            return std::cmp::Ordering::Equal;
        }
        if d < 0.0 {
            return std::cmp::Ordering::Less;
        }
        std::cmp::Ordering::Greater
    }
}

#[cfg(test)]
mod tests {
    use std::ops::Neg;

    use crate::{spherical::ChordLength, Angle, NVector};

    #[test]
    fn from_pos() {
        let c = ChordLength::new(
            NVector::from_lat_long_degrees(90.0, 0.0),
            NVector::from_lat_long_degrees(-90.0, 0.0),
        );
        assert_eq!(Angle::HALF_CIRCLE, c.to_angle());
    }

    #[test]
    fn symmetry() {
        assert_eq!(
            Angle::QUARTER_CIRCLE.round_d7(),
            ChordLength::from_angle(Angle::QUARTER_CIRCLE)
                .to_angle()
                .round_d7()
        );
    }

    #[test]
    fn negative_to_angle() {
        assert_eq!(Angle::from_radians(-1.0), ChordLength::NEGATIVE.to_angle());
    }

    #[test]
    fn from_angle_range() {
        assert_eq!(
            ChordLength::from_angle(Angle::QUARTER_CIRCLE),
            ChordLength::from_angle(Angle::HALF_CIRCLE + Angle::QUARTER_CIRCLE)
        );
        assert_eq!(
            ChordLength::from_angle(Angle::QUARTER_CIRCLE),
            ChordLength::from_angle(Angle::QUARTER_CIRCLE.neg())
        );
    }

    #[test]
    fn ord() {
        let a = Angle::from_degrees(45.0);
        assert_eq!(
            ::std::cmp::Ordering::Equal,
            ChordLength::from_angle(a).cmp(&ChordLength::from_angle(a))
        );
        assert_eq!(
            ::std::cmp::Ordering::Equal,
            ChordLength::NEGATIVE.cmp(&ChordLength::NEGATIVE)
        );
        assert_eq!(
            ::std::cmp::Ordering::Less,
            ChordLength::NEGATIVE.cmp(&ChordLength::from_angle(a))
        );
        assert_eq!(
            ::std::cmp::Ordering::Greater,
            ChordLength::from_angle(a).cmp(&ChordLength::NEGATIVE)
        );

        let b = Angle::from_degrees(90.0);
        assert_eq!(
            ::std::cmp::Ordering::Less,
            ChordLength::from_angle(a).cmp(&ChordLength::from_angle(b))
        );
        assert_eq!(
            ::std::cmp::Ordering::Greater,
            ChordLength::from_angle(b).cmp(&ChordLength::from_angle(a))
        );
    }
}
