use jord::{
    Angle, GeodeticPos, Length, LocalFrame, LocalLevelFrame, Metre, Micrometre, Millimetre,
};

// see https://github.com/pbrod/nvector/blob/bf1cf5e1e210b74a57ea4bb2c277b388308bdba9/src/nvector/tests/test_frames.py

#[test]
fn test_compute_delta_l_in_moving_frame_east() {
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
    assert_eq!(Length::from_metres(0.0), delta_1.z().round(Metre));
    assert_eq!(182.0, delta_1.azimuth().decimal_degrees().round());
}
