/// Tests if the given [f64] is effectively 0.0.
/// Returns true iff the absolute difference between the given float and 0.0
/// is less than or equal to the difference between 1.0 and the next representable
/// value ([f64::EPSILON]).
pub(crate) fn eq_zero(f: f64) -> bool {
    f.abs() < f64::EPSILON
}
