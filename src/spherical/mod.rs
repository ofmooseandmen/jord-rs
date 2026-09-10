//! Geographical position calculations assuming a spherical model.

mod base;
pub use base::Side;

mod cap;
pub use cap::Cap;

mod chord_length;
pub use chord_length::ChordLength;

mod great_circle;
pub use great_circle::GreatCircle;

mod minor_arc;
pub use minor_arc::{MinorArc, MinorArcRelation};

mod rectangle;
pub use rectangle::Rectangle;

mod sloop;
pub use sloop::{Loop, LoopRelation, is_loop_clockwise};

mod sphere;
pub use sphere::Sphere;
