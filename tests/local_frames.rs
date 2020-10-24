use jord::{
    Angle, GeodeticPos, Length, LocalFrame, LocalLevelFrame, Metre, Micrometre, Millimetre,
    NedFrame,
};

// see https://github.com/pbrod/nvector/blob/bf1cf5e1e210b74a57ea4bb2c277b388308bdba9/src/nvector/tests/test_frames.py

#[test]
fn delta_l_in_moving_frame_east() {
    let ship_position_0 = GeodeticPos::from_wgs84(1.0, 2.0, Length::zero());
    let ship_position_1 = GeodeticPos::from_wgs84(1.0, 2.005, Length::zero());
    let sensor_position = GeodeticPos::from_wgs84(1.000090437, 2.0025, Length::zero());

    let local_frame_0 = LocalLevelFrame::new(ship_position_0, Angle::from_decimal_degrees(90.0));

    let delta_0 = local_frame_0.delta_to(sensor_position);

    assert_eq!(
        Length::from_metres(278.256617),
        delta_0.x().round(Micrometre)
    );
    assert_eq!(Length::from_metres(-10.0), delta_0.y().round(Millimetre));
    assert_eq!(Length::from_metres(0.0), delta_0.z().round(Metre));
    assert_eq!(358.0, delta_0.azimuth().decimal_degrees().round());

    let local_frame_1 = LocalLevelFrame::new(ship_position_1, Angle::from_decimal_degrees(90.0));

    let delta_1 = local_frame_1.delta_to(sensor_position);

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

    let ned_frame_0 = NedFrame::new(ship_position_0);

    let delta_0 = ned_frame_0.delta_to(sensor_position);

    assert_eq!(Length::zero(), delta_0.x().round(Millimetre));
    assert_eq!(
        Length::from_metres(278.256624),
        delta_0.y().round(Micrometre)
    );
    assert_eq!(Length::zero(), delta_0.z().round(Metre));
    assert_eq!(90.0, delta_0.azimuth().decimal_degrees().round());

    let ned_frame_1 = NedFrame::new(ship_position_1);

    let delta_1 = ned_frame_1.delta_to(sensor_position);

    assert_eq!(Length::zero(), delta_1.x().round(Millimetre));
    assert_eq!(
        Length::from_metres(-278.256624),
        delta_1.y().round(Micrometre)
    );
    assert_eq!(Length::zero(), delta_1.z().round(Metre));
    assert_eq!(270.0, delta_1.azimuth().decimal_degrees().round());
}

#[test]
fn ned_delta_to() {
    let p1 = GeodeticPos::from_wgs84(49.66618, 3.45063, Length::zero());
    let p2 = GeodeticPos::from_wgs84(48.88667, 2.37472, Length::zero());

    let ned_frame = NedFrame::new(p1);

    let delta = ned_frame.delta_to(p2);

    assert_eq!(
        Length::from_metres(-86125.880548),
        delta.x().round(Micrometre)
    );
    assert_eq!(
        Length::from_metres(-78900.087817),
        delta.y().round(Micrometre)
    );
    assert_eq!(Length::from_metres(1069.19844), delta.z().round(Micrometre));
}
