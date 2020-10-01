#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    AntipodalPositions,
    CoincidentalPositions,
    OutOfRange,
    //__NonExhaustive, or [non_exhaustive]
}
