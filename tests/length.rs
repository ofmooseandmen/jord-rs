use jord::Length;

#[test]
fn metres_to_kilometres() {
    let l = Length::from_metres(1000.0);
    assert_eq!(1.0, l.as_kilometres());
}

#[test]
fn metres_to_nautical_miles() {
    let l = Length::from_metres(1000.0);
    assert_eq!(0.5399568034557235, l.as_nautical_miles());
}

#[test]
fn kilometres_to_nautical_miles() {
    let l = Length::from_kilometres(1000.0);
    assert_eq!(539.9568034557235, l.as_nautical_miles());
}

#[test]
fn nautical_miles_to_metres() {
    let l = Length::from_nautical_miles(10.5);
    assert_eq!(19446.0, l.as_metres());
}

#[test]
fn nautical_miles_to_kilometres() {
    let l = Length::from_nautical_miles(10.5);
    assert_eq!(19.446, l.as_kilometres());
}

#[test]
fn feet_to_metres() {
    let l = Length::from_feet(25000.0);
    assert_eq!(7620.0, l.as_metres());
}

#[test]
fn metres_to_feet() {
    let l = Length::from_metres(7620.0);
    assert_eq!(25000.0, l.as_feet());
}

#[test]
fn one_metre() {
    let l = Length::from_metres(1.0);
    assert_eq!(1.0, l.as_metres());
}

#[test]
fn one_kilometre() {
    let l = Length::from_kilometres(1.0);
    assert_eq!(1.0, l.as_kilometres());
}

#[test]
fn one_nautical_mile() {
    let l = Length::from_nautical_miles(1.0);
    assert_eq!(1.0, l.as_nautical_miles());
}

#[test]
fn one_feet() {
    let l = Length::from_feet(1.0);
    assert_eq!(1.0, l.as_feet());
}

#[test]
fn one_micrometre() {
    let l1 = Length::from_metres(1.000001);
    let l2 = Length::from_metres(1.000002);
    let l3 = Length::from_metres(1.0000011);
    assert_eq!(l1, l3);
    assert_ne!(l1, l2);
    assert_ne!(l2, l3);
}
