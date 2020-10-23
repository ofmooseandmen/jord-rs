use crate::{Angle, Mat33, Vec3};

pub fn r2xyz(rotation: Mat33) -> (Angle, Angle, Angle) {
    let v00 = rotation.row0().x();
    let v01 = rotation.row0().y();
    let v12 = rotation.row1().z();
    let v22 = rotation.row2().z();

    let z = Angle::from_decimal_degrees(-v01.atan2(v00).to_degrees());
    let x = Angle::from_decimal_degrees(-v12.atan2(v22).to_degrees());

    let sy = rotation.row0().z();
    /*
       cos y is based on as many elements as possible, to average out
       numerical errors. It is selected as the positive square root since
       y: [-pi/2 pi/2]
    */
    let cy = ((v00 * v00 + v01 * v01 + v12 * v12 + v22 * v22) / 2.0).sqrt();
    let y = Angle::from_decimal_degrees(sy.atan2(cy).to_degrees());
    (x, y, z)
}

pub fn r2zyx(rotation: Mat33) -> (Angle, Angle, Angle) {
    let rt = rotation.transpose();
    let (x, y, z) = r2xyz(rt);
    (-z, -y, -x)
}

pub fn xyz2r(x: Angle, y: Angle, z: Angle) -> Mat33 {
    let x_radians = x.decimal_degrees().to_radians();
    let cx = x_radians.cos();
    let sx = x_radians.sin();

    let y_radians = y.decimal_degrees().to_radians();
    let cy = y_radians.cos();
    let sy = y_radians.sin();

    let z_radians = z.decimal_degrees().to_radians();
    let cz = z_radians.cos();
    let sz = z_radians.sin();

    let row0 = Vec3::new(cy * cz, -cy * sz, sy);
    let row1 = Vec3::new(sy * sx * cz + cx * sz, -sy * sx * sz + cx * cz, -cy * sx);
    let row2 = Vec3::new(-sy * cx * cz + sx * sz, sy * cx * sz + sx * cz, cy * cx);

    Mat33::new(row0, row1, row2)
}

pub fn zyx2r(z: Angle, y: Angle, x: Angle) -> Mat33 {
    let x_radians = x.decimal_degrees().to_radians();
    let cx = x_radians.cos();
    let sx = x_radians.sin();

    let y_radians = y.decimal_degrees().to_radians();
    let cy = y_radians.cos();
    let sy = y_radians.sin();

    let z_radians = z.decimal_degrees().to_radians();
    let cz = z_radians.cos();
    let sz = z_radians.sin();

    let row0 = Vec3::new(cz * cy, -sz * cx + cz * sy * sx, sz * sx + cz * sy * cx);
    let row1 = Vec3::new(sz * cy, cz * cx + sz * sy * sx, -cz * sx + sz * sy * cx);
    let row2 = Vec3::new(-sy, cy * sx, cy * cx);

    Mat33::new(row0, row1, row2)
}
