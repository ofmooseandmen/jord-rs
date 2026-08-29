use crate::{Mat33, Vec3};
use std::fmt::Debug;

/// Indicates the spatial orientation of a frame's axes, specifically
/// whether its Z-axis points up or down according to the right-hand rule.
pub trait FrameOrientation: Copy + Clone + Debug + PartialEq + Eq + Default {
    /// Returns true if the z-axis points UP (away from the Earth's center).
    /// Returns false if the z-axis points DOWN (towards the Earth's center).
    fn is_z_up() -> bool;

    /// Returns the matrix to align this frame's axes to a standard aerospace Z-down
    /// orientation (x-forward/north, y-right/east, z-down) prior to attitude rotations.
    #[inline]
    fn align_to_z_down_matrix() -> Mat33 {
        if Self::is_z_up() {
            // Maps Z-up (ENU: East, North, Up) to Z-down (NED: North, East, Down).
            // X_down = Y_up, Y_down = X_up, Z_down = -Z_up
            Mat33::new(
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, -1.0),
            )
        } else {
            // Already Z-down (NED, WanderAzimuth, Body), return Identity.
            Mat33::IDENTITY
        }
    }
}
/// East-North-Up (ENU) orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Enu;
impl FrameOrientation for Enu {
    #[inline]
    fn is_z_up() -> bool {
        true
    }
}

/// North-East-Down (NED) orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Ned;
impl FrameOrientation for Ned {
    #[inline]
    fn is_z_up() -> bool {
        false
    }
}

/// Body (Vehicle) orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Body;
impl FrameOrientation for Body {
    #[inline]
    fn is_z_up() -> bool {
        false
    }
}

/// Wander azimuth orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WanderAzimuth;
impl FrameOrientation for WanderAzimuth {
    #[inline]
    fn is_z_up() -> bool {
        false
    }
}

/// Marker trait representing a local-level navigation reference frame tangent to the Earth's surface.
pub trait LocalNavigationFrame: FrameOrientation {}
impl LocalNavigationFrame for Enu {}
impl LocalNavigationFrame for Ned {}
impl LocalNavigationFrame for WanderAzimuth {}
