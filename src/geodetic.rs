use crate::models::{S84Model, WGS84Model, S84, WGS84};
use crate::{Angle, AngleResolution, Length, LengthResolution, LongitudeRange, Model, Vec3};
use std::convert::From;

// FIXME Display
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LatLong(Angle, Angle);

impl LatLong {
    pub const fn latitude(&self) -> Angle {
        self.0
    }

    pub const fn longitude(&self) -> Angle {
        self.1
    }

    pub fn round(&self, resolution: AngleResolution) -> Self {
        LatLong(self.0.round(resolution), self.1.round(resolution))
    }

    pub const fn north_pole() -> Self {
        LatLong(Angle::from_decimal_degrees(90.0), Angle::zero())
    }

    pub const fn south_pole() -> Self {
        LatLong(Angle::from_decimal_degrees(-90.0), Angle::zero())
    }
}

// FIXME Display
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HorizontalPos<M>(Vec3, M);

impl<M: Model> HorizontalPos<M> {
    pub fn new(nvector: Vec3, model: M) -> Self {
        HorizontalPos(nvector, model)
    }

    pub fn from_decimal_lat_long(latitude: f64, longitude: f64, model: M) -> Self {
        if eq_lat_north_pole(latitude) {
            HorizontalPos::north_pole(model)
        } else if eq_lat_south_pole(latitude) {
            HorizontalPos::south_pole(model)
        } else {
            let nvector = nvector_from_lat_long_degrees(latitude, longitude);
            HorizontalPos(nvector, model)
        }
    }

    pub fn from_lat_long(latitude: Angle, longitude: Angle, model: M) -> Self {
        HorizontalPos::from_decimal_lat_long(
            latitude.decimal_degrees(),
            longitude.decimal_degrees(),
            model,
        )
    }

    pub fn north_pole(model: M) -> Self {
        HorizontalPos::new(Vec3::unit_z(), model)
    }

    pub fn south_pole(model: M) -> Self {
        HorizontalPos::new(Vec3::neg_unit_z(), model)
    }

    pub fn to_lat_long(&self) -> LatLong {
        nvector_to_lat_long(self.0, self.1.longitude_range())
    }

    pub fn round(&self, resolution: AngleResolution) -> Self {
        let ll = self.to_lat_long().round(resolution);
        HorizontalPos::from_lat_long(ll.0, ll.1, self.1)
    }

    pub fn nvector(&self) -> Vec3 {
        self.0
    }

    pub fn model(&self) -> M {
        self.1
    }

    pub fn antipode(&self) -> Self {
        HorizontalPos(-1.0 * self.0, self.1)
    }
}

impl HorizontalPos<S84Model> {
    pub fn from_s84(latitude: f64, longitude: f64) -> Self {
        HorizontalPos::from_decimal_lat_long(latitude, longitude, S84)
    }
}

impl HorizontalPos<WGS84Model> {
    pub fn from_wgs84(latitude: f64, longitude: f64) -> Self {
        HorizontalPos::from_decimal_lat_long(latitude, longitude, WGS84)
    }
}

impl<M: Model> From<(Angle, Angle, M)> for HorizontalPos<M> {
    fn from(llm: (Angle, Angle, M)) -> Self {
        HorizontalPos::from_lat_long(llm.0, llm.1, llm.2)
    }
}

impl<M: Model> From<(f64, f64, M)> for HorizontalPos<M> {
    fn from(llm: (f64, f64, M)) -> Self {
        HorizontalPos::from_decimal_lat_long(llm.0, llm.1, llm.2)
    }
}

impl<M: Model> From<(Vec3, M)> for HorizontalPos<M> {
    fn from(nvm: (Vec3, M)) -> Self {
        HorizontalPos::new(nvm.0, nvm.1)
    }
}

// FIXME Display
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeodeticPos<M>(Vec3, Length, M);

impl<M: Model> GeodeticPos<M> {
    pub fn new(nvector: Vec3, height: Length, model: M) -> Self {
        GeodeticPos(nvector, height, model)
    }

    pub fn at_height(hp: HorizontalPos<M>, height: Length) -> Self {
        GeodeticPos(hp.nvector(), height, hp.model())
    }

    pub fn from_decimal_lat_long(latitude: f64, longitude: f64, height: Length, model: M) -> Self {
        GeodeticPos::at_height(
            HorizontalPos::from_decimal_lat_long(latitude, longitude, model),
            height,
        )
    }

    pub fn from_lat_long(latitude: Angle, longitude: Angle, height: Length, model: M) -> Self {
        GeodeticPos::at_height(
            HorizontalPos::from_lat_long(latitude, longitude, model),
            height,
        )
    }

    pub fn north_pole(model: M) -> Self {
        GeodeticPos::at_height(HorizontalPos::north_pole(model), Length::zero())
    }

    pub fn south_pole(model: M) -> Self {
        GeodeticPos::at_height(HorizontalPos::south_pole(model), Length::zero())
    }

    pub fn at_surface(&self) -> HorizontalPos<M> {
        HorizontalPos::new(self.nvector(), self.model())
    }

    pub fn round(
        &self,
        angle_resolution: AngleResolution,
        length_resolution: LengthResolution,
    ) -> Self {
        let ll = HorizontalPos::new(self.nvector(), self.model())
            .to_lat_long()
            .round(angle_resolution);
        GeodeticPos::at_height(
            HorizontalPos::from_lat_long(ll.0, ll.1, self.model()),
            self.height().round(length_resolution),
        )
    }

    pub fn to_lat_long(&self) -> LatLong {
        nvector_to_lat_long(self.0, self.2.longitude_range())
    }

    pub fn nvector(&self) -> Vec3 {
        self.0
    }

    pub fn height(&self) -> Length {
        self.1
    }

    pub fn model(&self) -> M {
        self.2
    }
}

impl GeodeticPos<S84Model> {
    pub fn from_s84(latitude: f64, longitude: f64, height: Length) -> Self {
        GeodeticPos::from_decimal_lat_long(latitude, longitude, height, S84)
    }
}

impl GeodeticPos<WGS84Model> {
    pub fn from_wgs84(latitude: f64, longitude: f64, height: Length) -> Self {
        GeodeticPos::from_decimal_lat_long(latitude, longitude, height, WGS84)
    }
}

pub fn nvector_from_lat_long_degrees(latitude: f64, longitude: f64) -> Vec3 {
    let lat = latitude.to_radians();
    let lon = longitude.to_radians();
    let cl = lat.cos();
    let x = cl * lon.cos();
    let y = cl * lon.sin();
    let z = lat.sin();
    Vec3::new(x, y, z)
}

pub fn nvector_to_lat_long(nvector: Vec3, longitude_range: LongitudeRange) -> LatLong {
    if nvector == Vec3::unit_z() {
        LatLong::north_pole()
    } else if nvector == Vec3::neg_unit_z() {
        LatLong::south_pole()
    } else {
        let x = nvector.x();
        let y = nvector.y();
        let z = nvector.z();
        let lat = z.atan2((x * x + y * y).sqrt()).to_degrees();
        let lon = y.atan2(x).to_degrees();
        LatLong(
            Angle::from_decimal_degrees(lat),
            Angle::from_decimal_degrees(convert_lon(lat, lon, longitude_range)),
        )
    }
}

fn eq_lat_north_pole(lat: f64) -> bool {
    lat == 90.0
}

fn eq_lat_south_pole(lat: f64) -> bool {
    lat == -90.0
}

fn eq_lat_pole(lat: f64) -> bool {
    eq_lat_north_pole(lat.abs())
}

// lon is guaranteed to be within [-180, 180]
fn convert_lon(lat: f64, lon: f64, lr: LongitudeRange) -> f64 {
    if eq_lat_pole(lat) {
        0.0
    } else if lr == LongitudeRange::L180 {
        lon
    } else if lon < 0.0 {
        lon + 360.0
    } else {
        lon
    }
}
