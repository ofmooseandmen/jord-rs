use core::fmt::{Display, Formatter, Result};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    CoincidentalGreatCircles,
    AntipodalPositions,
    CoincidentalPositions,
    NotEnoughPositions,
    OutOfRange,
    //FIXME __NonExhaustive, or [non_exhaustive]
}

impl Display for Error {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "{:?}", self)
    }
}

impl std::error::Error for Error {}
