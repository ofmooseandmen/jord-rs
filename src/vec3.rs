/// A 3-element vector.
#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct Vec3 {
    x: f64,
    y: f64,
    z: f64,
}

impl Vec3 {
    /// `Vec3` (1, 0, 0).
    pub const UNIT_X: Vec3 = Vec3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };

    /// `Vec3` (0, 1, 0).
    pub const UNIT_Y: Vec3 = Vec3 {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };

    /// `Vec3` (0, 0, 1).
    pub const UNIT_Z: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };

    /// `Vec3` (-1, 0, 0).
    pub const NEG_UNIT_X: Vec3 = Vec3 {
        x: -1.0,
        y: 0.0,
        z: 0.0,
    };

    /// `Vec3` (0, -1, 0).
    pub const NEG_UNIT_Y: Vec3 = Vec3 {
        x: 0.0,
        y: -1.0,
        z: 0.0,
    };

    /// `Vec3` (0, 0, -1).
    pub const NEG_UNIT_Z: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: -1.0,
    };

    /// Origin: (0, 0, 0).
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// Creates a 3-dimensional vector from the given x, y, z components.
    pub fn new(vx: f64, vy: f64, vz: f64) -> Self {
        Vec3 {
            x: 0.0 + vx,
            y: 0.0 + vy,
            z: 0.0 + vz,
        }
    }

    /// Creates a 3-dimensional vector of unit length from the given x, y, z components.
    /// If the given components are all `0.0` then `Vec3::ZERO` is returned.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Vec3;
    ///
    /// assert_eq!(Vec3::new_unit(2.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
    /// ```
    pub fn new_unit(vx: f64, vy: f64, vz: f64) -> Self {
        let n = (vx * vx + vy * vy + vz * vz).sqrt();
        if n == 0.0 {
            Vec3::ZERO
        } else {
            let s = 1.0 / n;
            Vec3::new(s * vx, s * vy, s * vz)
        }
    }

    /// Creates a 3-dimensional vector of unit length which is the mean of all given vectors: unit length vector of
    /// the sum of all vectors.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Vec3;
    ///
    /// let vs = vec![Vec3::UNIT_X, Vec3::UNIT_Y, Vec3::UNIT_Z];
    /// let c = 1.0 / 3.0_f64.sqrt();
    /// assert_eq!(Vec3::new(c, c, c), Vec3::mean(&vs));
    /// ```
    pub fn mean(vs: &[Self]) -> Self {
        let mut x = 0.0;
        let mut y = 0.0;
        let mut z = 0.0;
        for v in vs {
            x += v.x();
            y += v.y();
            z += v.z();
        }
        Vec3::new_unit(x, y, z)
    }

    /// Returns the x component of this vector.
    pub fn x(self) -> f64 {
        self.x
    }

    /// Returns the y component of this vector.
    pub fn y(self) -> f64 {
        self.y
    }

    /// Returns the z component of this vector.
    pub fn z(self) -> f64 {
        self.z
    }

    /// Returns the vector perpendicular to this vector and the given vector (cross product).
    /// Note that cross product is unstable for nearly parallel (coincidental or opposite) vectors.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Vec3;
    ///
    /// let v1 = Vec3::new(1.0, 5.0, 4.0);
    /// let v2 = Vec3::new(2.0, 6.0, 5.0);
    ///
    /// assert_eq!(v1.cross_prod(v2), Vec3::new(1.0, 3.0, -4.0));
    /// ```
    pub fn cross_prod(self, o: Self) -> Self {
        let x = self.y() * o.z() - self.z() * o.y();
        let y = self.z() * o.x() - self.x() * o.z();
        let z = self.x() * o.y() - self.y() * o.x();
        Vec3::new(x, y, z)
    }

    /// Returns the unit length vector perpendicular to this vector and the given vector (normalised cross product).
    /// See also `Vec3::cross_prod` and `Vec3::unit`.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Vec3;
    ///
    /// let v1 = Vec3::new(2.0, 0.0, 0.0);
    /// let v2 = Vec3::new(0.0, 2.0, 0.0);
    ///
    /// assert_eq!(v1.cross_prod_unit(v2), Vec3::new(0.0, 0.0, 1.0));
    /// ```
    pub fn cross_prod_unit(self, o: Self) -> Self {
        let x = self.y() * o.z() - self.z() * o.y();
        let y = self.z() * o.x() - self.x() * o.z();
        let z = self.x() * o.y() - self.y() * o.x();
        Vec3::new_unit(x, y, z)
    }

    /// Returns the dot product of this vector and the given vector. Equivalently the dot product of 2 vectors
    /// is the product of their magnitudes, times the cosine of the angle between them.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Vec3;
    ///
    /// let v1 = Vec3::new(1.0, 5.0, 4.0);
    /// let v2 = Vec3::new(2.0, 6.0, 5.0);
    ///
    /// assert_eq!(v1.dot_prod(v2), 52.0);
    /// ```
    pub fn dot_prod(self, o: Self) -> f64 {
        self.x() * o.x() + self.y() * o.y() + self.z() * o.z()
    }

    /// Returns a unit length vector orthogonal to this vector.
    ///
    /// # Examples:
    /// ```
    /// use jord::Vec3;
    ///
    /// let v = Vec3::new(3.0, 2.0, 1.0);
    /// let o = v.orthogonal();
    ///
    /// assert_eq!(o.z(), 0.0);
    /// ```
    pub fn orthogonal(self) -> Self {
        let ax = self.x().abs();
        let ay = self.y().abs();
        let az = self.z().abs();
        let tmp;
        if ax > ay {
            if ax > az {
                // largest is x: select z axis.
                tmp = Vec3::UNIT_Z;
            } else {
                // largest is z: select y axis.
                tmp = Vec3::UNIT_Y;
            }
        } else if ay > az {
            // largest is y: select x axis.
            tmp = Vec3::UNIT_X;
        } else {
            // largest is z: select y axis.
            tmp = Vec3::UNIT_Y;
        }
        self.cross_prod_unit(tmp)
    }

    /// Euclidean norm of this vector (square root of the dot product with itself).
    pub fn norm(self) -> f64 {
        self.squared_norm().sqrt()
    }

    /// Similar to `Vec3::stable_cross_prod`, but returns a unit vector (without creating an intermediate
    /// [Vec3].
    ///
    /// #Examples:
    ///
    /// ```
    /// use jord::Vec3;
    ///
    /// let v1 = Vec3::new(2.0, 0.0, 0.0);
    /// let v2 = Vec3::new(0.0, 2.0, 0.0);
    /// assert_eq!(Vec3::new(0.0, 0.0, 1.0), v1.stable_cross_prod_unit(v2));
    /// ```
    pub fn stable_cross_prod_unit(self, o: Self) -> Self {
        // a = v2 + v1
        let xa = o.x() + self.x();
        let ya = o.y() + self.y();
        let za = o.z() + self.z();

        // b = v2 - v1
        let xb = o.x() - self.x();
        let yb = o.y() - self.y();
        let zb = o.z() - self.z();

        // a x b
        let x = ya * zb - za * yb;
        let y = za * xb - xa * zb;
        let z = xa * yb - ya * xb;

        Vec3::new_unit(x, y, z)
    }

    /// Calculates the vector perpendicular to given unit vectors in a numerically stable way. The direction of v1 x
    /// v2 is unstable as v2 + v1 or v2 - v1 approaches 0. In order to workaround this, this method computes (v2 +
    /// v1) x (v2 - v1) which is twice the cross product of v2 and v1, but is always perpendicular (since both v1
    /// and v2 are unit-length vectors).
    ///
    /// #Examples:
    ///
    /// ```
    /// use jord::Vec3;
    ///
    /// let v1 = Vec3::new(2.0, 0.0, 0.0);
    /// let v2 = Vec3::new(0.0, 2.0, 0.0);
    /// assert_eq!(Vec3::new(0.0, 0.0, 8.0), v1.stable_cross_prod(v2));
    /// ```
    pub fn stable_cross_prod(self, o: Self) -> Self {
        // a = v2 + v1
        let xa = o.x() + self.x();
        let ya = o.y() + self.y();
        let za = o.z() + self.z();

        // b = v2 - v1
        let xb = o.x() - self.x();
        let yb = o.y() - self.y();
        let zb = o.z() - self.z();

        // a x b
        let x = ya * zb - za * yb;
        let y = za * xb - xa * zb;
        let z = xa * yb - ya * xb;

        Vec3::new(x, y, z)
    }

    /// Squared Euclidean norm of this vector (the dot product with itself).
    pub fn squared_norm(self) -> f64 {
        self.dot_prod(self)
    }

    /// Normalised vector (or unit length vector) if the norm of this vector is nonzero.
    ///
    /// # Examples
    ///
    /// ```
    /// use jord::Vec3;
    ///
    /// let v = Vec3::new(2.0, 0.0, 0.0);
    /// assert_eq!(v.unit(), Vec3::new(1.0, 0.0, 0.0));
    /// ```
    pub fn unit(self) -> Self {
        let n = self.norm();
        if n == 0.0 {
            Vec3::ZERO
        } else {
            let s = 1.0 / n;
            s * self
        }
    }
}

impl std::fmt::Display for Vec3 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{}, {}, {}]", self.x, self.y, self.z)
    }
}

impl ::std::ops::Add for Vec3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Vec3::new(self.x() + rhs.x(), self.y() + rhs.y(), self.z() + rhs.z())
    }
}

impl ::std::ops::Sub for Vec3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Vec3::new(self.x() - rhs.x(), self.y() - rhs.y(), self.z() - rhs.z())
    }
}

impl ::std::ops::Mul<f64> for Vec3 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        Vec3::new(self.x() * rhs, self.y() * rhs, self.z() * rhs)
    }
}

impl ::std::ops::Mul<Vec3> for f64 {
    type Output = Vec3;

    fn mul(self, rhs: Vec3) -> Self::Output {
        Vec3::new(rhs.x() * self, rhs.y() * self, rhs.z() * self)
    }
}

impl ::std::ops::Div<f64> for Vec3 {
    type Output = Self;

    fn div(self, rhs: f64) -> Self {
        Vec3::new(self.x() / rhs, self.y() / rhs, self.z() / rhs)
    }
}

#[cfg(test)]
mod tests {

    use crate::Vec3;

    #[test]
    fn orthogonal_x_largest() {
        let v = Vec3::new_unit(3.0, 2.0, 1.0);
        let o = v.orthogonal();
        assert_eq!(o.z(), 0.0);
        assert_eq!(v.dot_prod(o), 0.0);
    }

    #[test]
    fn orthogonal_y_largest() {
        let v = Vec3::new_unit(2.0, 3.0, 1.0);
        let o = v.orthogonal();
        assert_eq!(o.x(), 0.0);
        assert_eq!(v.dot_prod(o), 0.0);
    }

    #[test]
    fn orthogonal_z_largest() {
        let v = Vec3::new_unit(1.0, 2.0, 3.0);
        let o = v.orthogonal();
        assert_eq!(o.y(), 0.0);
        assert_eq!(v.dot_prod(o), 0.0);
    }

    #[test]
    fn orthogonal_z_largest_2() {
        let v = Vec3::new_unit(2.0, 1.0, 3.0);
        let o = v.orthogonal();
        assert_eq!(o.y(), 0.0);
        assert_eq!(v.dot_prod(o), 0.0);
    }
}
