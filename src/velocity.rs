use crate::{Speed, Vec3};

#[derive(PartialEq, PartialOrd, Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))] // codecov:ignore:this
/// A 3D velocity vector decomposed into orthogonal frame axes.
///
/// [Velocity] implements many traits, including [Add](::std::ops::Add), [Sub](::std::ops::Sub),
/// [Mul](::std::ops::Mul) and [Div](::std::ops::Div), among others.
pub struct Velocity {
    vx: Speed,
    vy: Speed,
    vz: Speed,
}

impl Velocity {
    /// Zero velocity.
    pub const ZERO: Velocity = Velocity {
        vx: Speed::ZERO,
        vy: Speed::ZERO,
        vz: Speed::ZERO,
    };

    /// Creates a new  velocity vector from individual components.
    pub const fn new(vx: Speed, vy: Speed, vz: Speed) -> Self {
        Self { vx, vy, vz }
    }

    /// Creates a [Velocity] from the given coordinates in metres/second.
    pub(crate) fn from_vec3_mps(v: Vec3) -> Self {
        Self::new(
            Speed::from_metres_per_second(v.x()),
            Speed::from_metres_per_second(v.y()),
            Speed::from_metres_per_second(v.z()),
        )
    }

    /// Returns the vx component of this vector.
    #[inline]
    pub fn vx(self) -> Speed {
        self.vx
    }

    /// Returns the vy component of this vector.
    #[inline]
    pub fn vy(self) -> Speed {
        self.vy
    }

    /// Returns the vz component of this vector.
    #[inline]
    pub fn vz(self) -> Speed {
        self.vz
    }

    /// Returns the (vx, vy, vz) components of this vector in metres/second.
    pub fn as_metres_per_second(&self) -> Vec3 {
        Vec3::new(
            self.vx.as_metres_per_second(),
            self.vy.as_metres_per_second(),
            self.vz.as_metres_per_second(),
        )
    }
}
