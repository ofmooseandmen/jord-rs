use jord::models::S84Model;
use jord::{Error, HorizontalPos, Length};

#[test]
fn mean_no_position() {
    let empty: Vec<HorizontalPos<S84Model>> = Vec::new();
    assert_eq!(
        Err(Error::NotEnoughPositions),
        HorizontalPos::from_mean(&empty)
    );
}

#[test]
fn mean_one_position() {
    let p = HorizontalPos::from_s84(0.0, 0.0);
    assert_eq!(Ok(p), HorizontalPos::from_mean(&vec![p]));
}

#[test]
fn mean_antipode() {
    let ps = vec![
        HorizontalPos::from_s84(45.0, 1.0),
        HorizontalPos::from_s84(45.0, 2.0),
        HorizontalPos::from_s84(46.0, 2.0),
        HorizontalPos::from_s84(46.0, 1.0),
        HorizontalPos::from_s84(45.0, 2.0).antipode(),
    ];
    assert_eq!(
        Err(Error::AntipodalPositions),
        HorizontalPos::from_mean(&ps)
    );
}

#[test]
fn mean_is_equidistant() {
    let ps = vec![
        HorizontalPos::from_s84(40.0, -40.0),
        HorizontalPos::from_s84(40.0, 40.0),
        HorizontalPos::from_s84(-40.0, 40.0),
        HorizontalPos::from_s84(-40.0, -40.0),
    ];
    let mean = HorizontalPos::from_mean(&ps);
    let expected = HorizontalPos::from_s84(0.0, 0.0);
    assert_eq!(Ok(expected), mean);

    let mut dists: Vec<Length> = ps.iter().map(|p| p.distance_to(expected)).collect();
    dists.dedup();
    assert_eq!(1, dists.len());
}
