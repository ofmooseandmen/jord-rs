use crate::Vec3;

/// A 3*3 matrix.
#[derive(PartialEq, Clone, Copy, Debug, Default)]
pub struct Mat33 {
    r0: Vec3,
    r1: Vec3,
    r2: Vec3,
}

impl Mat33 {
    pub fn row0(&self) -> Vec3 {
        self.r0
    }

    pub fn row1(&self) -> Vec3 {
        self.r1
    }

    pub fn row2(&self) -> Vec3 {
        self.r2
    }

    pub fn new(r0: Vec3, r1: Vec3, r2: Vec3) -> Self {
        Self { r0, r1, r2 }
    }

    pub fn transpose(&self) -> Self {
        Mat33::new(
            Vec3::new(self.r0.x(), self.r1.x(), self.r2.x()),
            Vec3::new(self.r0.y(), self.r1.y(), self.r2.y()),
            Vec3::new(self.r0.z(), self.r1.z(), self.r2.z()),
        )
    }
}

impl ::std::ops::Mul<Mat33> for Vec3 {
    type Output = Self;

    fn mul(self, rhs: Mat33) -> Self {
        let x = self.dot_prod(rhs.r0);
        let y = self.dot_prod(rhs.r1);
        let z = self.dot_prod(rhs.r2);
        Vec3::new(x, y, z)
    }
}

impl ::std::ops::Mul<Mat33> for Mat33 {
    type Output = Self;

    fn mul(self, rhs: Mat33) -> Self {
        let t2 = rhs.transpose();
        let m1r0 = self.r0;
        let m1r1 = self.r1;
        let m1r2 = self.r2;

        let t2r0 = t2.r0;
        let t2r1 = t2.r1;
        let t2r2 = t2.r2;

        let mr0 = Vec3::new(
            m1r0.dot_prod(t2r0),
            m1r0.dot_prod(t2r1),
            m1r0.dot_prod(t2r2),
        );
        let mr1 = Vec3::new(
            m1r1.dot_prod(t2r0),
            m1r1.dot_prod(t2r1),
            m1r1.dot_prod(t2r2),
        );
        let mr2 = Vec3::new(
            m1r2.dot_prod(t2r0),
            m1r2.dot_prod(t2r1),
            m1r2.dot_prod(t2r2),
        );

        Mat33::new(mr0, mr1, mr2)
    }
}
