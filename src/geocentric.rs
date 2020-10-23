use crate::{GeodeticPos, Length, LengthResolution, Model, Surface, Vec3};
use std::convert::From;

// FIXME Display
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeocentricPos<M>(Length, Length, Length, M);

impl<M: Model> GeocentricPos<M> {
    pub fn new(x: Length, y: Length, z: Length, model: M) -> Self {
        GeocentricPos(x, y, z, model)
    }

    pub fn from_metres(x: f64, y: f64, z: f64, model: M) -> Self {
        GeocentricPos::new(
            Length::from_metres(x),
            Length::from_metres(y),
            Length::from_metres(z),
            model,
        )
    }

    pub fn north_pole(model: M) -> Self {
        let pr = model.surface().polar_radius();
        GeocentricPos::new(Length::zero(), Length::zero(), pr, model)
    }

    pub fn south_pole(model: M) -> Self {
        let pr = model.surface().polar_radius();
        GeocentricPos::new(Length::zero(), Length::zero(), -pr, model)
    }

    pub fn round(&self, resolution: LengthResolution) -> Self {
        GeocentricPos::new(
            self.x().round(resolution),
            self.y().round(resolution),
            self.z().round(resolution),
            self.model(),
        )
    }

    pub fn from_geodetic(geodetic: GeodeticPos<M>) -> Self {
        let geoc_metres = nvector_to_geocentric(
            geodetic.nvector(),
            geodetic.height(),
            geodetic.model().surface(),
        );
        GeocentricPos::from_metres(
            geoc_metres.x(),
            geoc_metres.y(),
            geoc_metres.z(),
            geodetic.model(),
        )
    }

    pub fn to_geodetic(&self) -> GeodeticPos<M> {
        let geoc_metres = self.as_vec3_metres();
        let (nv, h) = nvector_from_geocentric(geoc_metres, self.model().surface());
        GeodeticPos::new(nv, h, self.model())
    }

    pub fn x(&self) -> Length {
        self.0
    }

    pub fn y(&self) -> Length {
        self.1
    }

    pub fn z(&self) -> Length {
        self.2
    }

    pub fn model(&self) -> M {
        self.3
    }

    fn as_vec3_metres(&self) -> Vec3 {
        Vec3::new(self.x().metres(), self.y().metres(), self.z().metres())
    }
}

impl<M: Model> GeodeticPos<M> {
    pub fn from_geocentric(geocentric: GeocentricPos<M>) -> Self {
        let geoc_metres = geocentric.as_vec3_metres();
        let (nv, h) = nvector_from_geocentric(geoc_metres, geocentric.model().surface());
        GeodeticPos::new(nv, h, geocentric.model())
    }

    pub fn to_geocentric(&self) -> GeocentricPos<M> {
        let geoc_metres =
            nvector_to_geocentric(self.nvector(), self.height(), self.model().surface());
        GeocentricPos::from_metres(
            geoc_metres.x(),
            geoc_metres.y(),
            geoc_metres.z(),
            self.model(),
        )
    }
}

impl<M: Model> From<GeocentricPos<M>> for GeodeticPos<M> {
    fn from(geocentric: GeocentricPos<M>) -> Self {
        GeodeticPos::from_geocentric(geocentric)
    }
}

impl<M: Model> From<GeodeticPos<M>> for GeocentricPos<M> {
    fn from(geodetic: GeodeticPos<M>) -> Self {
        GeocentricPos::from_geodetic(geodetic)
    }
}

fn nvector_to_geocentric<S>(nvector: Vec3, height: Length, surface: S) -> Vec3
where
    S: Surface,
{
    if surface.is_sphere() {
        nvector_to_geocentric_s(nvector, height, surface.equatorial_radius())
    } else {
        nvector_to_geocentric_e(nvector, height, surface)
    }
}

fn nvector_to_geocentric_s(nvector: Vec3, height: Length, radius: Length) -> Vec3 {
    nvector * (height + radius).metres()
}

#[allow(clippy::many_single_char_names)]
fn nvector_to_geocentric_e<S>(nvector: Vec3, height: Length, surface: S) -> Vec3
where
    S: Surface,
{
    let nx = nvector.x();
    let ny = nvector.y();
    let nz = nvector.z();
    let a = surface.equatorial_radius().metres();
    let b = surface.polar_radius().metres();
    let m = (a * a) / (b * b);
    let n = b / ((nx * nx * m) + (ny * ny * m) + (nz * nz)).sqrt();
    let h = height.metres();
    let cx = n * m * nx + h * nx;
    let cy = n * m * ny + h * ny;
    let cz = n * nz + h * nz;
    Vec3::new(cx, cy, cz)
}

fn nvector_from_geocentric<S>(geoc_metres: Vec3, surface: S) -> (Vec3, Length)
where
    S: Surface,
{
    if surface.is_sphere() {
        nvector_from_geocentric_s(geoc_metres, surface.equatorial_radius())
    } else {
        nvector_from_geocentric_e(geoc_metres, surface)
    }
}

fn nvector_from_geocentric_s(geoc_metres: Vec3, radius: Length) -> (Vec3, Length) {
    let height = Length::from_metres(geoc_metres.norm()) - radius;
    (geoc_metres.unit(), height)
}

#[allow(clippy::many_single_char_names)]
fn nvector_from_geocentric_e<S>(geoc_metres: Vec3, surface: S) -> (Vec3, Length)
where
    S: Surface,
{
    let px = geoc_metres.x();
    let py = geoc_metres.y();
    let pz = geoc_metres.z();
    let e = surface.eccentricity();
    let e2 = e * e;
    let e4 = e2 * e2;
    let a = surface.equatorial_radius().metres();
    let a2 = a * a;
    let p = (px * px + py * py) / a2;
    let q = ((1.0 - e2) / a2) * (pz * pz);
    let r = (p + q - e4) / 6.0;
    let s = (e4 * p * q) / (4.0 * r * r * r);
    let t = (1.0 + s + (s * (2.0 + s)).sqrt()).powf(1.0 / 3.0);
    let u = r * (1.0 + t + 1.0 / t);
    let v = (u * u + q * e4).sqrt();
    let w = e2 * (u + v - q) / (2.0 * v);
    let k = (u + v + w * w).sqrt() - w;
    let d = k * (px * px + py * py).sqrt() / (k + e2);
    let h = ((k + e2 - 1.0) / k) * (d * d + pz * pz).sqrt();
    (
        nvec_ellipsoidal(d, e2, k, px, py, pz),
        Length::from_metres(h),
    )
}

fn nvec_ellipsoidal(d: f64, e2: f64, k: f64, px: f64, py: f64, pz: f64) -> Vec3 {
    let s = 1.0 / (d * d + pz * pz).sqrt();
    let a = k / (k + e2);
    let nx = s * a * px;
    let ny = s * a * py;
    let nz = s * pz;
    Vec3::new(nx, ny, nz)
}
