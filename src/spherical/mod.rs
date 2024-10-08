//! Geographical position calculations assuming a spherical model.

mod base;

mod cap;
pub use cap::Cap;

mod chord_length;
pub use chord_length::ChordLength;

mod great_circle;
pub use great_circle::GreatCircle;

mod minor_arc;
pub use minor_arc::MinorArc;

mod rectangle;
pub use rectangle::Rectangle;

mod sloop;
pub use sloop::is_loop_clockwise;
pub use sloop::Loop;

mod sphere;
pub use sphere::Sphere;
