pub fn eq(x: f64, y: f64, epsilon: f64) -> bool {
  let min = x.abs().min(y.abs());
  if min.abs() == 0.0 {
     (x - y).abs() < epsilon
  } else {
    // don't divide by zero.
    (x - y).abs() / f64::EPSILON.max(min) < epsilon
  }  
}