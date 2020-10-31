use jord::{r2xyz, r2zyx, xyz2r, zyx2r, Angle, Mat33, Microarcsecond, Vec3};

#[test]
fn test_r2xyz() {
    let rm = Mat33::new(
        Vec3::new(
            0.7044160264027587,
            -6.162841671621935e-2,
            0.7071067811865475,
        ),
        Vec3::new(0.559725765762092, 0.6608381550289296, -0.5),
        Vec3::new(-0.43646893232965345, 0.7479938977765876, 0.5000000000000001),
    );

    let (x, y, z) = r2xyz(rm);

    assert_eq!(Angle::from_decimal_degrees(45.0), x.round(Microarcsecond));
    assert_eq!(Angle::from_decimal_degrees(45.0), y.round(Microarcsecond));
    assert_eq!(Angle::from_decimal_degrees(5.0), z.round(Microarcsecond));
}

#[test]
fn test_r2zyx() {
    let rm = Mat33::new(
        Vec3::new(
            0.9254165783983234,
            1.802831123629725e-2,
            0.37852230636979245,
        ),
        Vec3::new(
            0.16317591116653482,
            0.8825641192593856,
            -0.44096961052988237,
        ),
        Vec3::new(-0.3420201433256687, 0.46984631039295416, 0.8137976813493738),
    );

    let (z, y, x) = r2zyx(rm);

    assert_eq!(Angle::from_decimal_degrees(10.0), z.round(Microarcsecond));
    assert_eq!(Angle::from_decimal_degrees(20.0), y.round(Microarcsecond));
    assert_eq!(Angle::from_decimal_degrees(30.0), x.round(Microarcsecond));
}

#[test]
fn test_xyz2r() {
    let actual = xyz2r(
        Angle::from_decimal_degrees(45.0),
        Angle::from_decimal_degrees(45.0),
        Angle::from_decimal_degrees(5.0),
    );
    let expected = Mat33::new(
        Vec3::new(
            0.7044160264027587,
            -6.162841671621935e-2,
            0.7071067811865475,
        ),
        Vec3::new(0.559725765762092, 0.6608381550289296, -0.5),
        Vec3::new(-0.43646893232965345, 0.7479938977765876, 0.5000000000000001),
    );

    assert_eq!(expected, actual);
}

#[test]
fn test_zyx2r() {
    let actual = zyx2r(
        Angle::from_decimal_degrees(10.0),
        Angle::from_decimal_degrees(20.0),
        Angle::from_decimal_degrees(30.0),
    );
    let expected = Mat33::new(
        Vec3::new(
            0.9254165783983234,
            1.802831123629725e-2,
            0.37852230636979245,
        ),
        Vec3::new(
            0.16317591116653482,
            0.8825641192593856,
            -0.44096961052988237,
        ),
        Vec3::new(-0.3420201433256687, 0.46984631039295416, 0.8137976813493738),
    );

    assert_eq!(expected, actual);
}
