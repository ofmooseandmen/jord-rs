use ::std::convert::From;

use crate::models::{S84Model, S84};
use crate::{Angle, LongitudeRange, Model, Vec3};

// FIXME Display
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NvectorPos<M: Model> {
    nvector: Vec3,
    model: M,
}

impl<M: Model> NvectorPos<M> {
    pub fn new(nvector: Vec3, model: M) -> Self {
        NvectorPos { nvector, model }
    }

    // FIXME from_decimal_lat_long and add from_lat_long accepts Angle?
    pub fn from_lat_long(latitude: f64, longitude: f64, model: M) -> Self {
        NvectorPos::from_radians(latitude.to_radians(), longitude.to_radians(), model)
    }

    fn from_radians(latitude: f64, longitude: f64, model: M) -> Self {
        let nvector = nvector_from_lat_long_radians(latitude, longitude);
        NvectorPos { nvector, model }
    }

    pub fn north_pole(model: M) -> Self {
        NvectorPos::new(Vec3::unit_z(), model)
    }

    pub fn south_pole(model: M) -> Self {
        NvectorPos::new(Vec3::neg_unit_z(), model)
    }

    pub fn to_lat_long(&self) -> (f64, f64) {
        let ll = nvector_to_lat_long_radians(self.nvector);
        let lat = ll.0.to_degrees();
        let lon = ll.1.to_degrees();
        (lat, convert_lon(lat, lon, &self.model.longitude_range()))
    }

    pub fn nvector(&self) -> Vec3 {
        self.nvector
    }

    pub fn model(&self) -> M {
        self.model
    }

    pub fn antipode(&self) -> Self {
        let anti = self.nvector * -1.0;
        NvectorPos {
            nvector: anti,
            model: self.model,
        }
    }
}

impl<M: Model> From<(Vec3, M)> for NvectorPos<M> {
    fn from(nvm: (Vec3, M)) -> Self {
        NvectorPos::new(nvm.0, nvm.1)
    }
}

impl<M: Model> From<([f64; 3], M)> for NvectorPos<M> {
    fn from(nvm: ([f64; 3], M)) -> Self {
        NvectorPos::new(nvm.0.into(), nvm.1)
    }
}

impl<M: Model> From<LatLongPos<M>> for NvectorPos<M> {
    fn from(llp: LatLongPos<M>) -> Self {
        NvectorPos::from_radians(
            llp.latitude.as_radians(),
            llp.longitude.as_radians(),
            llp.model,
        )
    }
}

// FIXME Display & FromStr
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LatLongPos<M: Model> {
    latitude: Angle,
    longitude: Angle,
    model: M,
}

// FIXME: parse & antipode
impl<M: Model> LatLongPos<M> {
    pub fn new(latitude: Angle, longitude: Angle, model: M) -> Self {
        let (lat, lon) = wrap(
            latitude.as_decimal_degrees(),
            longitude.as_decimal_degrees(),
            &model.longitude_range(),
        );
        LatLongPos {
            latitude: Angle::from_decimal_degrees(lat),
            longitude: Angle::from_decimal_degrees(lon),
            model,
        }
    }

    pub fn from_decimal_degrees(latitude: f64, longitude: f64, model: M) -> Self {
        LatLongPos::new(
            Angle::from_decimal_degrees(latitude),
            Angle::from_decimal_degrees(longitude),
            model,
        )
    }

    pub fn from_nvector(nv: Vec3, model: M) -> Self {
        let ll = nvector_to_lat_long_radians(nv);
        let lat = Angle::from_radians(ll.0);
        let lon = Angle::from_radians(ll.1);
        let clon = convert_lon(
            lat.as_decimal_degrees(),
            lon.as_decimal_degrees(),
            &model.longitude_range(),
        );
        LatLongPos {
            latitude: lat,
            longitude: Angle::from_decimal_degrees(clon),
            model,
        }
    }

    pub fn north_pole(model: M) -> Self {
        LatLongPos::from_decimal_degrees(90.0, 0.0, model)
    }

    pub fn south_pole(model: M) -> Self {
        LatLongPos::from_decimal_degrees(-90.0, 0.0, model)
    }

    pub fn to_nvector(&self) -> Vec3 {
        nvector_from_lat_long_radians(self.latitude.as_radians(), self.longitude.as_radians())
    }

    pub fn latitude(&self) -> Angle {
        self.latitude
    }

    pub fn longitude(&self) -> Angle {
        self.longitude
    }

    pub fn model(&self) -> M {
        self.model
    }
}

impl LatLongPos<S84Model> {
    pub fn from_s84(latitude: f64, longitude: f64) -> Self {
        LatLongPos::from_decimal_degrees(latitude, longitude, S84)
    }
}

impl<M: Model> From<(Angle, Angle, M)> for LatLongPos<M> {
    fn from(llm: (Angle, Angle, M)) -> Self {
        LatLongPos::new(llm.0, llm.1, llm.2)
    }
}

impl<M: Model> From<(f64, f64, M)> for LatLongPos<M> {
    fn from(llm: (f64, f64, M)) -> Self {
        LatLongPos::from_decimal_degrees(llm.0, llm.1, llm.2)
    }
}

impl<M: Model> From<NvectorPos<M>> for LatLongPos<M> {
    fn from(nvp: NvectorPos<M>) -> Self {
        LatLongPos::from_nvector(nvp.nvector, nvp.model)
    }
}

pub(crate) fn nvector_from_lat_long_radians(lat: f64, lon: f64) -> Vec3 {
    let cl = lat.cos();
    let x = cl * lon.cos();
    let y = cl * lon.sin();
    let z = lat.sin();
    Vec3::new(x, y, z)
}

pub(crate) fn nvector_to_lat_long_radians(nv: Vec3) -> (f64, f64) {
    let x = nv.x();
    let y = nv.y();
    let z = nv.z();
    let lat = z.atan2((x * x + y * y).sqrt());
    let lon = y.atan2(x);
    (lat, lon)
}

fn eq_lat_pole(lat: f64) -> bool {
    lat.abs() == 90.0
}

fn check_pole(lat: f64, lon: f64) -> f64 {
    if eq_lat_pole(lat) {
        0.0
    } else {
        lon
    }
}

fn convert_lon(lat: f64, lon: f64, lr: &LongitudeRange) -> f64 {
    if eq_lat_pole(lat) {
        0.0
    } else if *lr == LongitudeRange::L180 || is_valid_lon(lon, lr) {
        lon
    } else {
        lon + 360.0
    }
}

// https://gist.github.com/missinglink/d0a085188a8eab2ca66db385bb7c023a
fn wrap(lat: f64, lon: f64, lr: &LongitudeRange) -> (f64, f64) {
    if is_valid_lat(lat) && is_valid_lon(lon, lr) {
        (lat, check_pole(lat, lon))
    } else {
        let quadrant = ((lat.abs() / 90.0).floor() % 4.0) as u8;
        let pole;
        if lat > 0.0 {
            pole = 90.0;
        } else {
            pole = -90.0;
        }
        let offset = lat % 90.0;
        println!("offset {}", offset);

        let wlat;
        let mut wlon = lon;
        match quadrant {
            0 => wlat = offset,
            1 => {
                wlat = pole - offset;
                wlon += 180.0;
            }
            2 => {
                wlat = -offset;
                wlon += 180.0;
            }
            3 => wlat = -pole + offset,
            _ => panic!("invalid quadrant {}", quadrant),
        }

        if wlon > 180.0 || wlon < -180.0 {
            wlon = wlon - ((wlon + 180.0) / 360.0).floor() * 360.0;
        }

        (wlat, convert_lon(wlat, wlon, lr))
    }
}

fn is_valid_lat(lat: f64) -> bool {
    lat >= -90.0 && lat <= 90.0
}

fn is_valid_lon(lon: f64, lr: &LongitudeRange) -> bool {
    if *lr == LongitudeRange::L360 {
        lon >= 0.0 && lon <= 360.0
    } else {
        lon >= -180.0 && lon <= 180.0
    }
}

#[cfg(test)]
mod lat_long_test {

    mod pole_test {

        use crate::models::S84;
        use crate::LatLongPos;

        #[test]
        fn north_pole() {
            let np = LatLongPos::north_pole(S84);
            assert_eq!(90.0, np.latitude().as_decimal_degrees());
            assert_eq!(0.0, np.longitude().as_decimal_degrees());
        }

        #[test]
        fn south_pole() {
            let sp = LatLongPos::south_pole(S84);
            assert_eq!(-90.0, sp.latitude().as_decimal_degrees());
            assert_eq!(0.0, sp.longitude().as_decimal_degrees());
        }

        #[test]
        fn longitude_at_north_pole_is_0() {
            let lat = 90.0;
            for x in 0..360 {
                let lon = x as f64 - 180.0;
                let p = LatLongPos::from_decimal_degrees(lat, lon, S84);
                assert_eq!(90.0, p.latitude().as_decimal_degrees());
                assert_eq!(0.0, p.longitude().as_decimal_degrees());
            }
        }

        #[test]
        fn longitude_at_south_pole_is_0() {
            let lat = -90.0;
            for x in 0..360 {
                let lon = x as f64 - 180.0;
                let p = LatLongPos::from_decimal_degrees(lat, lon, S84);
                assert_eq!(-90.0, p.latitude().as_decimal_degrees());
                assert_eq!(0.0, p.longitude().as_decimal_degrees());
            }
        }
    }

    mod wrap_test {

        use crate::{Angle, LatLongPos};

        #[test]
        fn no_wrapping() {
            let p = LatLongPos::from_s84(55.555, 22.222);
            assert_eq!(Angle::from_decimal_degrees(55.555), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(22.222), p.longitude());
        }

        #[test]
        fn positive_lat_wrapping_91_degrees() {
            let p = LatLongPos::from_s84(91.0, 54.0);
            assert_eq!(Angle::from_decimal_degrees(89.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(-126.0), p.longitude());
        }

        #[test]
        fn positive_lat_wrapping_181_degrees() {
            let p = LatLongPos::from_s84(181.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(-1.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(180.0), p.longitude());
        }

        #[test]
        fn positive_lat_wrapping_271_degrees() {
            let p = LatLongPos::from_s84(271.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(-89.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn positive_lat_wrapping_361_degrees() {
            let p = LatLongPos::from_s84(361.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(1.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn positive_lat_wrapping_631_degrees() {
            let p = LatLongPos::from_s84(631.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(-89.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn positive_lat_wrapping_721_degrees() {
            let p = LatLongPos::from_s84(721.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(1.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn negative_lat_wrapping_91_degrees() {
            let p = LatLongPos::from_s84(-91.0, 54.0);
            assert_eq!(Angle::from_decimal_degrees(-89.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(-126.0), p.longitude());
        }

        #[test]
        fn negative_lat_wrapping_181_degrees() {
            let p = LatLongPos::from_s84(-181.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(1.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(180.0), p.longitude());
        }

        #[test]
        fn negative_lat_wrapping_271_degrees() {
            let p = LatLongPos::from_s84(-271.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(89.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn negative_lat_wrapping_361_degrees() {
            let p = LatLongPos::from_s84(-361.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(-1.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn negative_lat_wrapping_631_degrees() {
            let p = LatLongPos::from_s84(-631.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(89.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn negative_lat_wrapping_721_degrees() {
            let p = LatLongPos::from_s84(-721.0, 0.0);
            assert_eq!(Angle::from_decimal_degrees(-1.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(0.0), p.longitude());
        }

        #[test]
        fn positive_lon_wrapping_181_degrees() {
            let p = LatLongPos::from_s84(0.0, 181.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(-179.0), p.longitude());
        }

        #[test]
        fn positive_lon_wrapping_271_degrees() {
            let p = LatLongPos::from_s84(0.0, 271.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(-89.0), p.longitude());
        }

        #[test]
        fn positive_lon_wrapping_361_degrees() {
            let p = LatLongPos::from_s84(0.0, 361.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(1.0), p.longitude());
        }

        #[test]
        fn positive_lon_wrapping_631_degrees() {
            let p = LatLongPos::from_s84(0.0, 631.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(-89.0), p.longitude());
        }

        #[test]
        fn positive_lon_wrapping_721_degrees() {
            let p = LatLongPos::from_s84(0.0, 721.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(1.0), p.longitude());
        }

        #[test]
        fn negative_lon_wrapping_181_degrees() {
            let p = LatLongPos::from_s84(0.0, -181.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(179.0), p.longitude());
        }

        #[test]
        fn negative_lon_wrapping_271_degrees() {
            let p = LatLongPos::from_s84(0.0, -271.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(89.0), p.longitude());
        }

        #[test]
        fn negative_lon_wrapping_361_degrees() {
            let p = LatLongPos::from_s84(0.0, -361.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(-1.0), p.longitude());
        }

        #[test]
        fn negative_lon_wrapping_631_degrees() {
            let p = LatLongPos::from_s84(0.0, -631.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(89.0), p.longitude());
        }

        #[test]
        fn negative_lon_wrapping_721_degrees() {
            let p = LatLongPos::from_s84(0.0, -721.0);
            assert_eq!(Angle::from_decimal_degrees(0.0), p.latitude());
            assert_eq!(Angle::from_decimal_degrees(-1.0), p.longitude());
        }
    }
}

#[cfg(test)]
mod resolution_test {

    use crate::models::{S84Model, S84};
    use crate::{LatLongPos, NvectorPos, Vec3};

    #[test]
    fn from_nvectors() {
        let nv1 = NvectorPos::new(Vec3::new(0.5, 0.5, 0.5_f64.sqrt()), S84);
        let nv2 = NvectorPos::new(
            Vec3::new(0.5000000000000001, 0.5000000000000001, 0.5_f64.sqrt()),
            S84,
        );
        assert_ne!(nv1, nv2);

        let ll1: LatLongPos<S84Model> = nv1.into();
        let ll2: LatLongPos<S84Model> = nv2.into();
        assert_eq!(ll1, ll2);

        let nv3: NvectorPos<S84Model> = ll1.into();
        let nv4: NvectorPos<S84Model> = ll2.into();
        assert_eq!(nv3, nv4);
    }

    #[test]
    fn from_lat_long() {
        let ll1 = LatLongPos::from_s84(45.0, 45.0);
        let ll2 = LatLongPos::from_s84(45.0000000005, 45.0000000005);
        let ll3 = LatLongPos::from_s84(45.0000000001, 45.0000000001);

        assert_ne!(ll1, ll2);
        assert_eq!(ll1, ll3);
    }
}
