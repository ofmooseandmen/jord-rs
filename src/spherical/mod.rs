//! Geographical position calculations assuming a spherical model.

mod base;

mod great_circle;
pub use great_circle::GreatCircle;

mod regions;
pub use regions::Loop;
pub use regions::{is_loop_clockwise, is_loop_convex};

mod minor_arc;
pub use minor_arc::MinorArc;

mod sphere;
pub use sphere::Sphere;
