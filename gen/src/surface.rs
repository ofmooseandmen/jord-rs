use std::io::{self, Error, ErrorKind};

use crate::text::*;
use jord::{Ellipsoid, Length, Sphere, Surface};

pub enum SurfaceDef {
    Sphere {
        comment: String,
        name: String,
        data: Sphere,
    },
    Ellipsoid {
        comment: String,
        name: String,
        data: Ellipsoid,
    },
}

pub fn surface_imports() -> String {
    "Ellipsoid, Length, Sphere".to_string()
}

pub fn gen_surface(surface: SurfaceDef) -> String {
    match surface {
        SurfaceDef::Ellipsoid {
            comment,
            name,
            data,
        } => gen_ellispoid(comment, name, data),
        SurfaceDef::Sphere {
            comment,
            name,
            data,
        } => gen_sphere(comment, name, data),
    }
}

pub fn parse_surface(text: &Text) -> io::Result<(SurfaceDef, Text)> {
    let (comment, txt) = text.next_if_prefixed("# ")?;
    let (name, txt) = txt.next()?;
    match txt.next_if_prefixed("  a: ") {
        Err(_) => {
            let (r, txt) = txt.next_if_prefixed("  r: ")?;
            Ok((
                SurfaceDef::Sphere {
                    comment,
                    name,
                    data: Sphere::new(parse_metres(r)?),
                },
                txt,
            ))
        }
        Ok((a, txt)) => {
            let (invf, txt) = txt.next_if_prefixed("  1/f: ")?;
            Ok((
                SurfaceDef::Ellipsoid {
                    comment,
                    name,
                    data: Ellipsoid::new(parse_metres(a)?, invf.parse::<f64>().unwrap()),
                },
                txt,
            ))
        }
    }
}

fn parse_metres(s: String) -> io::Result<Length> {
    let last = s.chars().last().unwrap();
    match last {
        'm' => Ok(Length::from_metres(
            s[..s.len() - 1].parse::<f64>().unwrap(),
        )),
        _ => Err(Error::new(ErrorKind::InvalidData, "expected 'm'")),
    }
}

fn gen_sphere(c: String, n: String, e: Sphere) -> String {
    format!(
        "/// {}
pub const {}_SPHERE: Sphere = Sphere::new(Length::from_metres({}f64));

",
        c,
        n.to_uppercase(),
        e.mean_radius().metres(),
    )
}

fn gen_ellispoid(c: String, n: String, e: Ellipsoid) -> String {
    format!(
        "/// {}
pub const {}_ELLIPSOID: Ellipsoid = Ellipsoid::from_all(
    Length::from_metres({}f64),
    Length::from_metres({}f64),
    {}f64,
    {}f64,
);

/// Sphere derived from: {}
pub const {}_SPHERE: Sphere = Sphere::new(Length::from_metres({}f64));

",
        c,
        n.to_uppercase(),
        e.equatorial_radius().metres(),
        e.polar_radius().metres(),
        e.eccentricity(),
        e.flattening(),
        c,
        n.to_uppercase(),
        e.mean_radius().metres(),
    )
}
