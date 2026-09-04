use std::fmt::Debug;
use std::marker::PhantomData;

use crate::{
    local::{Body, Enu, FrameOrientation, Ned, WanderAzimuth},
    numbers::eq_zero,
    Angle, Length, PositionVector, Vec3,
};

/// A vector whose length and direction is such that it goes from the origin
/// of frame A to the origin of frame B, i.e. the position of B relative to A.
///
/// [`LocalVector`] implements [`PositionVector`], as such it can be
/// turned into a [Vec3] to perform linear algebra calculations.
///
/// [`LocalVector`] also implements [Add](::std::ops::Add), [Sub](::std::ops::Sub),
/// [Mul](::std::ops::Mul) and [Div](::std::ops::Div), among others.
#[derive(PartialEq, Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))] // codecov:ignore:this
pub struct LocalVector<O: FrameOrientation> {
    x: Length,
    y: Length,
    z: Length,
    #[cfg_attr(feature = "serde", serde(skip))]
    _o: PhantomData<O>,
}

/// [`LocalVector`] for [ENU frame](crate::local::EnuFrame).
///
/// Can be converted to an [`NedVector`] using [From].
pub type EnuVector = LocalVector<Enu>;

/// [`LocalVector`] for [NED frame](crate::local::NedFrame).
///
/// Can be converted to an [`EnuVector`] using [From].
pub type NedVector = LocalVector<Ned>;

/// [`LocalVector`] for [body frame](crate::local::BodyFrame).
pub type BodyVector = LocalVector<Body>;

/// [`LocalVector`] for [wander azimuth](crate::local::WanderAzimuthFrame).
pub type WanderAzimuthVector = LocalVector<WanderAzimuth>;

impl<O: FrameOrientation> LocalVector<O> {
    /// Origin: (0.0, 0.0, 0.0).
    pub const ZERO: LocalVector<O> = LocalVector {
        x: Length::ZERO,
        y: Length::ZERO,
        z: Length::ZERO,
        _o: PhantomData,
    };

    /// Creates a [`LocalVector`] from the given coordinates.
    pub const fn new(x: Length, y: Length, z: Length) -> Self {
        Self {
            x,
            y,
            z,
            _o: PhantomData,
        }
    }

    /// Creates a [`LocalVector`] from the given coordinates in metres.
    pub fn from_metres(x: f64, y: f64, z: f64) -> Self {
        Self::new(
            Length::from_metres(x),
            Length::from_metres(y),
            Length::from_metres(z),
        )
    }

    pub(crate) fn from_vec3_metres(v: Vec3) -> Self {
        Self::from_metres(v.x(), v.y(), v.z())
    }

    /// Transforms the given local azimuth-elevation-range (AER) spherical coordinates
    /// to local Cartesian coordinates.
    /// # Examples
    ///
    /// ```
    /// use jord::{local::BodyVector, Angle, Length, PositionVector};
    ///
    /// // sensor observation: body reference frame.
    /// let az: Angle = Angle::from_degrees(155.427);
    /// let el = Angle::from_degrees(-23.161);
    /// let sr = Length::from_metres(10.885);
    /// let local = BodyVector::from_aer(az, el, sr);
    ///
    /// assert_eq!(Length::from_metres(-9.101), local.x().round_mm());
    /// assert_eq!(Length::from_metres(4.162), local.y().round_mm());
    /// assert_eq!(Length::from_metres(4.281), local.z().round_mm());
    /// ```
    pub fn from_aer(azimuth: Angle, elevation: Angle, slant_range: Length) -> Self {
        let (north, east, z_up) = aer_to_nez(azimuth, elevation, slant_range);
        if O::is_z_up() {
            Self::new(east, north, z_up)
        } else {
            Self::new(north, east, -z_up)
        }
    }

    /// Returns the slant range - distance from origin in the local system.
    pub fn slant_range(&self) -> Length {
        Length::from_metres(self.as_metres().norm())
    }

    /// The angle between this vector and the frame's horizontal plane.
    /// `0°` lies exactly in the horizontal plane.
    ///
    /// Frame specific definitions:
    /// - [Body]: The angle of this vector above (`+`) or below (`-`) the vehicle's own
    ///   horizontal (deck) plane. The body frame's z-axis points down towards
    ///   the vehicle's belly, so a vector with negative z (physically above
    ///   the deck) has positive elevation.
    /// - [ENU](Enu): The angle of this vector above (`+`) or below (`-`) the local
    ///   horizontal plane. ENU's z-axis already points up, so this is simply
    ///   the angle between the vector and that axis.
    /// - [NED](Ned): The angle of this vector above (`+`) or below (`-`) the local
    ///   horizontal plane. NED's z-axis points down, so a vector with
    ///   negative z (physically above the plane) has positive elevation.
    /// - [Wander Azimuth](WanderAzimuth): The angle of this vector above (`+`) or below (`-`) the local
    ///   horizontal plane. This frame's z-axis points down (as in NED), so a
    ///   vector with negative z (physically above the plane) has positive elevation.
    pub fn elevation(&self) -> Angle {
        let sr = self.slant_range();
        if eq_zero(sr.as_metres()) {
            Angle::ZERO
        } else {
            let r = Angle::from_radians((self.z() / self.slant_range()).clamp(-1.0, 1.0).asin());
            if O::is_z_up() {
                r
            } else {
                -r
            }
        }
    }

    /// The angle of this vector's projection onto the frame's horizontal
    /// plane, measured from the frame's horizontal reference axis.
    ///
    /// Frame specific definitions:
    /// - [Body]: The relative bearing of this vector off the vehicle's nose: `0°` is
    ///   straight ahead, increasing clockwise (as viewed looking down from
    ///   above the vehicle) towards the right at `90°`, in the range
    ///   `-180°..+180°` — negative values are off the left/port side.
    /// - [ENU](Enu): The true compass bearing of this vector: `0°` is North, increasing
    ///   clockwise (as viewed looking down at the horizontal plane) towards
    ///   East at `90°`, in the range `0°..360°`. (Note this is the *opposite*
    ///   turning sense from ENU's own x→y axis order, since x is East and y is
    ///   North here — "bearing" always means clockwise-from-North regardless
    ///   of which axis a frame calls "primary".)
    /// - [NED](Ned): The true compass bearing of this vector: `0°` is North, increasing
    ///   clockwise (as viewed looking down at the horizontal plane) towards
    ///   East at `90°`, in the range `0°..360°`.
    /// - [Wander Azimuth](WanderAzimuth): The bearing of this vector measured from the frame's x-axis, in the
    ///   range `0°..360°`. Unlike [`Ned`] or [`Enu`] this is *not*
    ///   generally a true compass bearing: the wander-azimuth frame's x-axis
    ///   is offset from True North by the frame's wander angle, so add that
    ///   wander angle to recover a true compass bearing.
    pub fn bearing(&self) -> Angle {
        let (e, n) = if O::is_z_up() {
            (self.x(), self.y())
        } else {
            (self.y(), self.x())
        };
        Angle::from_radians(e.as_metres().atan2(n.as_metres())).normalised()
    }
}

fn aer_to_nez(azimuth: Angle, elevation: Angle, slant_range: Length) -> (Length, Length, Length) {
    let cose = elevation.as_radians().cos();
    let east = azimuth.as_radians().sin() * cose * slant_range;
    let north = azimuth.as_radians().cos() * cose * slant_range;
    let z = elevation.as_radians().sin() * slant_range;
    (north, east, z)
}

impl<O: FrameOrientation> PositionVector for LocalVector<O> {
    #[inline]
    fn x(&self) -> Length {
        self.x
    }

    #[inline]
    fn y(&self) -> Length {
        self.y
    }

    #[inline]
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

impl<O: FrameOrientation> ::std::ops::Add for LocalVector<O> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        LocalVector::from_vec3_metres(self.as_metres() + rhs.as_metres())
    }
}

impl<O: FrameOrientation> ::std::ops::Sub for LocalVector<O> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        LocalVector::from_vec3_metres(self.as_metres() - rhs.as_metres())
    }
}

impl<O: FrameOrientation> ::std::ops::Mul<f64> for LocalVector<O> {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        LocalVector::from_vec3_metres(self.as_metres() * rhs)
    }
}

impl<O: FrameOrientation> ::std::ops::Mul<LocalVector<O>> for f64 {
    type Output = LocalVector<O>;

    fn mul(self, rhs: LocalVector<O>) -> Self::Output {
        LocalVector::from_vec3_metres(rhs.as_metres() * self)
    }
}

impl<O: FrameOrientation> ::std::ops::Div<f64> for LocalVector<O> {
    type Output = Self;

    fn div(self, rhs: f64) -> Self {
        LocalVector::from_vec3_metres(self.as_metres() / rhs)
    }
}

impl<O: FrameOrientation> ::std::ops::Neg for LocalVector<O> {
    type Output = Self;

    fn neg(self) -> Self {
        LocalVector::from_vec3_metres(-self.as_metres())
    }
}

impl From<EnuVector> for NedVector {
    fn from(value: EnuVector) -> Self {
        Self::new(value.y(), value.x(), -value.z())
    }
}

impl From<NedVector> for EnuVector {
    fn from(value: NedVector) -> Self {
        Self::new(value.y(), value.x(), -value.z())
    }
}

#[cfg(test)]
mod tests {

    use crate::{
        local::BodyVector, local::EnuVector, local::LocalVector, local::NedVector,
        local::WanderAzimuthVector, Angle, Length, PositionVector, Vec3,
    };

    #[test]
    fn add_local_vectors() {
        let l1 = NedVector::from_vec3_metres(Vec3::new(1.0, 2.0, 3.0));
        let l2 = NedVector::from_vec3_metres(Vec3::new(1.0, 2.0, 3.0));

        assert_eq!(
            NedVector::from_vec3_metres(Vec3::new(2.0, 4.0, 6.0)),
            l1 + l2
        );
    }

    #[test]
    fn sub_local_vectors() {
        let l1 = EnuVector::from_vec3_metres(Vec3::new(1.0, 2.0, 3.0));
        let l2: LocalVector<_> = EnuVector::from_vec3_metres(Vec3::new(1.0, 2.0, 3.0));

        assert_eq!(
            EnuVector::from_vec3_metres(Vec3::new(0.0, 0.0, 0.0)),
            l1 - l2
        );
    }

    #[test]
    fn mul_local_vector() {
        let l = BodyVector::from_vec3_metres(Vec3::new(1.0, 2.0, 3.0));

        assert_eq!(
            BodyVector::from_vec3_metres(Vec3::new(2.0, 4.0, 6.0)),
            l * 2.0
        );

        assert_eq!(
            BodyVector::from_vec3_metres(Vec3::new(2.0, 4.0, 6.0)),
            2.0 * l
        )
    }

    #[test]
    fn div_local_vector() {
        let l = WanderAzimuthVector::from_vec3_metres(Vec3::new(2.0, 4.0, 6.0));

        assert_eq!(
            WanderAzimuthVector::from_vec3_metres(Vec3::new(1.0, 2.0, 3.0)),
            l / 2.0
        )
    }

    #[test]
    fn neg_local_vector() {
        let l = NedVector::from_vec3_metres(Vec3::new(2.0, 4.0, 6.0));

        assert_eq!(NedVector::from_vec3_metres(Vec3::new(-2.0, -4.0, -6.0)), -l)
    }

    // see https://au.mathworks.com/help/map/ref/aer2ned.html
    #[test]
    fn aer_to_ned() {
        let az: Angle = Angle::from_degrees(155.427);
        let el = Angle::from_degrees(-23.161);
        let sr = Length::from_metres(10.885);

        let local = NedVector::from_aer(az, el, sr);

        assert_eq!(Length::from_metres(-9.101), local.x().round_mm());
        assert_eq!(Length::from_metres(4.162), local.y().round_mm());
        assert_eq!(Length::from_metres(4.281), local.z().round_mm());
        assert_eq!(az, local.bearing().round_d7());
        assert_eq!(el, local.elevation().round_d7());
        assert_eq!(sr, local.slant_range().round_mm());
    }

    // https://au.mathworks.com/help/map/ref/aer2enu.html
    #[test]
    fn aer_to_enu() {
        let az: Angle = Angle::from_degrees(34.1160);
        let el = Angle::from_degrees(4.1931);
        let sr = Length::from_metres(15.1070);

        let local = EnuVector::from_aer(az, el, sr);

        assert_eq!(Length::from_metres(8.45), local.x().round_mm());
        assert_eq!(Length::from_metres(12.474), local.y().round_mm());
        assert_eq!(Length::from_metres(1.105), local.z().round_mm());
        assert_eq!(az, local.bearing().round_d7());
        assert_eq!(el, local.elevation().round_d7());
        assert_eq!(sr, local.slant_range().round_mm());
    }

    #[test]
    fn elevation_zero_vector() {
        assert_eq!(Angle::ZERO, NedVector::ZERO.elevation())
    }

    #[test]
    fn body_bearing_and_elevation() {
        // Target forward (+x), right (+y), above (-z)
        let target = BodyVector::from_metres(10.0, 10.0, -10.0);

        assert_eq!(Angle::from_degrees(45.0), target.bearing());
        assert_eq!(
            Angle::from_degrees(35.2643897),
            target.elevation().round_d7()
        );
    }

    #[test]
    fn body_elevation_below_plane() {
        // Target directly below vehicle (+z in Body frame)
        let target = BodyVector::from_metres(0.0, 0.0, 10.0);
        assert_eq!(Angle::from_degrees(-90.0), target.elevation().round_d7());
    }

    #[test]
    fn enu_to_ned() {
        let enu = EnuVector::from_metres(1.0, 2.0, 3.0);
        let ned: NedVector = enu.into();
        assert_eq!(NedVector::from_metres(2.0, 1.0, -3.0), ned)
    }

    #[test]
    fn ned_to_enu() {
        let ned = NedVector::from_metres(1.0, 2.0, 3.0);
        let enu: EnuVector = ned.into();
        assert_eq!(EnuVector::from_metres(2.0, 1.0, -3.0), enu)
    }
}
