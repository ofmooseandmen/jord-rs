use jord::Vec3;

#[test]
fn add_vec3() {
    let v1 = Vec3::new(1.0, 2.0, 3.0);
    let v2 = Vec3::new(4.0, 5.0, 6.0);
    assert_eq!(Vec3::new(5.0, 7.0, 9.0), v1 + v2);
}

#[test]
fn multiply_vec3_by_f64() {
    let v = Vec3::new(4.0, 5.0, 6.0);
    assert_eq!(Vec3::new(8.0, 10.0, 12.0), v * 2.0);
}

#[test]
fn unit() {
    assert_eq!(Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 3.0, 0.0).unit());
}
