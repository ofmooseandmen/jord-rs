/// Trait implemented by all measurable quantities.
pub trait Measurement {
    /// Creates a new quantity from the given amount expressed in the default unit.
    fn from_default_unit(amount: f64) -> Self;
    /// Returns this quantity in the default unit.
    fn as_default_unit(&self) -> f64;
}

/// Macro that creates the code to implement operator overrides.
#[macro_export]
macro_rules! impl_measurement {
    ($($t:ty)*) => ($(

        impl ::std::ops::Add for $t {
            type Output = Self;

            fn add(self, rhs: Self) -> Self {
                Self::from_default_unit(self.as_default_unit() + rhs.as_default_unit())
            }
        }

        impl ::std::ops::Sub for $t {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self {
                Self::from_default_unit(self.as_default_unit() - rhs.as_default_unit())
            }
        }

        impl ::std::ops::Neg for $t {
            type Output = Self;

            fn neg(self) -> Self {
                Self::from_default_unit(self.as_default_unit().neg())
            }
        }

        impl ::std::ops::Div<$t> for $t {
            type Output = f64;

            fn div(self, rhs: Self) -> f64 {
                self.as_default_unit() / rhs.as_default_unit()
            }
        }

        impl ::std::ops::Div<f64> for $t {
            type Output = Self;

            fn div(self, rhs: f64) -> Self {
                Self::from_default_unit(self.as_default_unit() / rhs)
            }
        }

        impl ::std::ops::Mul<f64> for $t {
            type Output = Self;

            fn mul(self, rhs: f64) -> Self {
                Self::from_default_unit(self.as_default_unit() * rhs)
            }
        }

        impl ::std::ops::Mul<$t> for f64 {
            type Output = $t;

            fn mul(self, rhs: $t) -> $t {
                rhs * self
            }
        }

    )*)
}
