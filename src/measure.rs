pub trait Measure {
    fn to_unit(self) -> f64;
    fn from_unit(amount: f64) -> Self;
    fn to_resolution(self) -> i64;
    fn from_resolution(amount: i64) -> Self;
}

#[macro_export]
macro_rules! impl_measure {
    ($($t:ty)*) => ($(

        impl ::std::ops::Add for $t {
            type Output = Self;

            fn add(self, rhs: Self) -> Self {
                Self::from_resolution(self.to_resolution() + rhs.to_resolution())
            }
        }

        impl ::std::ops::Sub for $t {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self {
                Self::from_resolution(self.to_resolution() - rhs.to_resolution())
            }
        }

        impl ::std::ops::Neg for $t {
            type Output = Self;

            fn neg(self) -> Self {
                Self::from_resolution(self.to_resolution().neg())
            }
        }

        impl ::std::ops::Div<$t> for $t {
            type Output = f64;

            fn div(self, rhs: Self) -> f64 {
                self.to_unit() / rhs.to_unit()
            }
        }

        impl ::std::ops::Div<f64> for $t {
            type Output = Self;

            fn div(self, rhs: f64) -> Self {
                Self::from_unit(self.to_unit() / rhs)
            }
        }

        impl ::std::ops::Mul<f64> for $t {
            type Output = Self;

            fn mul(self, rhs: f64) -> Self {
                Self::from_unit(self.to_unit() * rhs)
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
