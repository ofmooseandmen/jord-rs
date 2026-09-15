/// Tests if the given [`f64`] is effectively 0.0.
/// Returns true iff the absolute difference between the given float and 0.0
/// is less than or equal to the difference between 1.0 and the next representable
/// value ([`f64::EPSILON`]).
pub(crate) fn eq_zero(f: f64) -> bool {
    eq(f, 0.0)
}

/// Determines if the two given values are effectively equal.
///
/// Returns true iff the absolute difference between the two values is less than or equal to [`f64::EPSILON`].
pub(crate) fn eq(left: f64, right: f64) -> bool {
    eq_within_epsilon(left, right, f64::EPSILON)
}

/// Determines if the two given values are equal to within the given epsilon.
pub(crate) fn eq_within_epsilon(left: f64, right: f64, epsilon: f64) -> bool {
    (right - left).abs() <= epsilon
}

/// Evaluates less than or equal to comparison including [equal](crate::eq) values.
pub(crate) fn lte(left: f64, right: f64) -> bool {
    lte_within_epsilon(left, right, f64::EPSILON)
}

/// Evaluates less than or equal to comparison including [equal within epsilon](crate::eq_within_epsilon) values.
pub(crate) fn lte_within_epsilon(left: f64, right: f64, epsilon: f64) -> bool {
    left <= right || eq_within_epsilon(left, right, epsilon)
}

/// Evaluates greater than or equal to comparison including [equal](crate::eq) values.
pub(crate) fn gte(left: f64, right: f64) -> bool {
    gte_within_epsilon(left, right, f64::EPSILON)
}

/// Evaluates greater than or equal to comparison including [equal within epsilon](crate::eq_within_epsilon) values.
pub(crate) fn gte_within_epsilon(left: f64, right: f64, epsilon: f64) -> bool {
    left >= right || eq_within_epsilon(left, right, epsilon)
}
