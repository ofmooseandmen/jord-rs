//! Geographical position calculations assuming a spherical model.

mod base;
pub use base::{
    angle_radians_between, are_ordered, easting, is_great_circle, is_loop_clockwise,
    is_loop_convex, mean_position, orthogonal, side, side_exact, turn_radians,
};

pub(crate) use base::along_track_distance;

mod minor_arc;
pub use minor_arc::MinorArc;

mod great_circle;
pub use great_circle::GreatCircle;

mod navigation;
pub use navigation::Navigation;
