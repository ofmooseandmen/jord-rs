use jord::models::WGS72;
use jord::{
    n_e2_r_en, n_e_and_wa2_r_el, n_e_and_ypr2_r_eb, Angle, BodyOrientation, Delta, GeodeticPos,
    Length, Mat33, Metre, Microarcsecond, Micrometre, Millimetre, Vec3,
};

#[test]
fn test_n_e2_r_en() {
    let point_a = GeodeticPos::from_wgs84(1.0, 2.0, Length::from_metres(-3.0));
    let expected = Mat33::new(
        Vec3::new(
            -0.017441774902830158,
            -0.03489949670250097,
            -0.9992386149554826,
        ),
        Vec3::new(
            -0.0006090802009086826,
            0.9993908270190958,
            -0.03489418134011367,
        ),
        Vec3::new(0.9998476951563913, 0.0, -0.01745240643728351),
    );

    assert_eq!(expected, n_e2_r_en(point_a.nvector()));
}

#[test]
fn test_n_e_and_wa2_r_el() {
    let point_a = GeodeticPos::from_wgs84(1.0, 2.0, Length::from_metres(-3.0));
    let expected = Mat33::new(
        Vec3::new(
            -0.03701086808805652,
            -0.012344473468615448,
            -0.9992386149554826,
        ),
        Vec3::new(0.7062453461004854, 0.7071067155811835, -0.03489418134011366),
        Vec3::new(
            0.7069990853988243,
            -0.7069990853988242,
            -0.01745240643728351,
        ),
    );

    assert_eq!(
        expected,
        n_e_and_wa2_r_el(point_a.nvector(), Angle::from_decimal_degrees(45.0))
    );
}

#[test]
fn test_n_e_and_ypr2_r_eb() {
    let point_a = GeodeticPos::from_wgs84(1.0, 2.0, Length::from_metres(-3.0));
    let expected = Mat33::new(
        Vec3::new(
            0.31992406947816193,
            -0.5006040657552995,
            -0.8043905513603428,
        ),
        Vec3::new(0.1744473687980002, 0.8656206019997523, -0.46932833806732893),
        Vec3::new(0.9312447075221351, 0.009825616665137977, 0.3642619276964902),
    );

    assert_eq!(
        expected,
        n_e_and_ypr2_r_eb(
            point_a.nvector(),
            BodyOrientation::from_decimal_degrees(10.0, 20.0, 30.0)
        )
    );
}

// see https://github.com/pbrod/nvector/blob/bf1cf5e1e210b74a57ea4bb2c277b388308bdba9/src/nvector/tests/test_frames.py

#[test]
fn delta_w_in_moving_frame_east() {
    let ship_position_0 = GeodeticPos::from_wgs84(1.0, 2.0, Length::zero());
    let ship_position_1 = GeodeticPos::from_wgs84(1.0, 2.005, Length::zero());
    let sensor_position = GeodeticPos::from_wgs84(1.000090437, 2.0025, Length::zero());

    let delta_0 = ship_position_0.delta_w_to(Angle::from_decimal_degrees(90.0), sensor_position);

    assert_eq!(
        Length::from_metres(278.256617),
        delta_0.x().round(Micrometre)
    );
    assert_eq!(Length::from_metres(-10.0), delta_0.y().round(Millimetre));
    assert_eq!(Length::from_metres(0.0), delta_0.z().round(Metre));
    assert_eq!(358.0, delta_0.azimuth().decimal_degrees().round());

    let delta_1 = ship_position_1.delta_w_to(Angle::from_decimal_degrees(90.0), sensor_position);

    assert_eq!(
        Length::from_metres(-278.256617),
        delta_1.x().round(Micrometre)
    );
    assert_eq!(Length::from_metres(-10.0), delta_1.y().round(Millimetre));
    assert_eq!(Length::zero(), delta_1.z().round(Metre));
    assert_eq!(182.0, delta_1.azimuth().decimal_degrees().round());
}

#[test]
fn delta_n_in_moving_frame_east() {
    let ship_position_0 = GeodeticPos::from_wgs84(1.0, 2.0, Length::zero());
    let ship_position_1 = GeodeticPos::from_wgs84(1.0, 2.005, Length::zero());
    let sensor_position = GeodeticPos::from_wgs84(1.0, 2.0025, Length::zero());

    let delta_0 = ship_position_0.delta_n_to(sensor_position);

    assert_eq!(Length::zero(), delta_0.x().round(Millimetre));
    assert_eq!(
        Length::from_metres(278.256624),
        delta_0.y().round(Micrometre)
    );
    assert_eq!(Length::zero(), delta_0.z().round(Metre));
    assert_eq!(90.0, delta_0.azimuth().decimal_degrees().round());

    let delta_1 = ship_position_1.delta_n_to(sensor_position);

    assert_eq!(Length::zero(), delta_1.x().round(Millimetre));
    assert_eq!(
        Length::from_metres(-278.256624),
        delta_1.y().round(Micrometre)
    );
    assert_eq!(Length::zero(), delta_1.z().round(Metre));
    assert_eq!(270.0, delta_1.azimuth().decimal_degrees().round());
}

#[test]
fn delta_w_in_moving_frame_north() {
    let ship_position_0 = GeodeticPos::from_wgs84(1.0, 2.0, Length::zero());
    let ship_position_1 = GeodeticPos::from_wgs84(1.005, 2.0, Length::zero());
    let sensor_position = GeodeticPos::from_wgs84(1.0025, 2.0, Length::zero());

    let delta_0 = ship_position_0.delta_w_to(Angle::zero(), sensor_position);

    assert_eq!(
        Length::from_metres(276.436537),
        delta_0.x().round(Micrometre)
    );
    assert_eq!(Length::from_metres(0.0), delta_0.y().round(Millimetre));
    assert_eq!(Length::from_metres(0.0), delta_0.z().round(Metre));
    assert_eq!(0.0, delta_0.azimuth().decimal_degrees().round());

    let delta_1 = ship_position_1.delta_w_to(Angle::zero(), sensor_position);

    assert_eq!(
        Length::from_metres(-276.436541),
        delta_1.x().round(Micrometre)
    );
    assert_eq!(Length::from_metres(0.0), delta_1.y().round(Millimetre));
    assert_eq!(Length::zero(), delta_1.z().round(Metre));
    assert_eq!(180.0, delta_1.azimuth().decimal_degrees().round());
}

#[test]
fn ex1_a_and_b_to_delta_in_frame_n() {
    let point_a = GeodeticPos::from_wgs84(1.0, 2.0, Length::from_metres(-3.0));
    let point_b = GeodeticPos::from_wgs84(4.0, 5.0, Length::from_metres(-6.0));

    let delta = point_a.delta_n_to(point_b);

    assert_eq!(
        Length::from_metres(331730.234781),
        delta.x().round(Micrometre)
    );
    assert_eq!(
        Length::from_metres(332997.874989),
        delta.y().round(Micrometre)
    );
    assert_eq!(
        Length::from_metres(17404.271362),
        delta.z().round(Micrometre)
    );
    assert_eq!(
        Angle::from_decimal_degrees(45.109263238333334),
        delta.azimuth().round(Microarcsecond)
    );
    assert_eq!(
        Angle::from_decimal_degrees(2.1205586116666666),
        delta.elevation().round(Microarcsecond)
    );
}

#[test]
fn ex2_b_and_delta_in_frame_b_to_c_in_frame_e() {
    // Position and orientation of B is given 400m above E (WGS72)
    let point_b = GeodeticPos::new(
        Vec3::new(1.0, 2.0, 3.0).unit(),
        Length::from_metres(400.0),
        WGS72,
    );
    let orientation = BodyOrientation::from_decimal_degrees(10.0, 20.0, 30.0);
    let delta = Delta::from_metres(3000.0, 2000.0, 100.0);

    let point_c = point_b.destination_pos_from_delta_b(orientation, delta);
    let ll_c = point_c.to_lat_long().round(Microarcsecond);
    assert_eq!(53.32637826444444, ll_c.latitude().decimal_degrees());
    assert_eq!(63.468123435277775, ll_c.longitude().decimal_degrees());
    assert_eq!(406.007196, point_c.height().round(Micrometre).metres());
}
