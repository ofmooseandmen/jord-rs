//! Geographical position calculations assuming a spherical model.

mod base;

mod great_circle;
pub use great_circle::GreatCircle;

mod minor_arc;
pub use minor_arc::MinorArc;

mod sphere;
pub use sphere::Sphere;
