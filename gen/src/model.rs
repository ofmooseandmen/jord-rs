use std::io::{self, Error, ErrorKind};

use crate::text::*;

enum Type {
    Spherical,
    Ellipsoidal,
}

pub struct ModelDef {
    comment: String,
    mtype: Type,
    id: String,
    surface: String,
    longitude_range: String,
}

pub fn model_imports() -> String {
    "Ellipsoid, Ellipsoidal, LongitudeRange, Model, ModelId, Sphere, Spherical".to_string()
}

pub fn gen_model(model: ModelDef) -> String {
    gen_def(&model) + &gen_impl(&model) + &gen_type_impl(&model)
}

pub fn parse_model(text: &Text) -> io::Result<(ModelDef, Text)> {
    let (comment, txt) = text.next_if_prefixed("# ")?;
    let (type_id, txt) = txt.next()?;
    let mut it = type_id.split_ascii_whitespace();
    let mtype = match it.next() {
        None => Err(Error::new(ErrorKind::InvalidData, "expected type")),
        Some(t) => match t {
            "ellipsoidal" => Ok(Type::Ellipsoidal),
            "spherical" => Ok(Type::Spherical),
            _ => Err(Error::new(
                ErrorKind::InvalidData,
                "invalid type: ".to_owned() + t,
            )),
        },
    }?;
    let id = match it.next() {
        None => Err(Error::new(ErrorKind::InvalidData, "expected id")),
        Some(i) => Ok(i),
    }?;
    let (surface, txt) = txt.next_if_prefixed("  surface: ")?;
    let (longitude_range, txt) = txt.next_if_prefixed("  longitude_range: ")?;
    Ok((
        ModelDef {
            comment,
            mtype,
            id: id.to_string(),
            surface,
            longitude_range,
        },
        txt,
    ))
}

fn gen_def(model: &ModelDef) -> String {
    let m_id_ype = model_id_type(model);
    format!(
        "/// {}
pub const {}: {}Model = {}Model {{}};

/// Struct for model: {}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct {}Model {{}}

",
        model.comment,
        model.id.to_uppercase(),
        m_id_ype,
        m_id_ype,
        model.comment,
        m_id_ype,
    )
}

fn gen_impl(model: &ModelDef) -> String {
    let (surface_type, surface) = match model.mtype {
        Type::Ellipsoidal => (
            "Ellipsoid",
            format!("{}_ELLIPSOID", model.surface.to_uppercase()),
        ),
        Type::Spherical => ("Sphere", format!("{}_SPHERE", model.surface.to_uppercase())),
    };

    format!(
        "impl Model for {}Model {{
    type Surface = {};
    fn model_id(&self) -> ModelId {{
        ModelId::new(\"{}\".to_string())
    }}
    fn longitude_range(&self) -> LongitudeRange {{
        LongitudeRange::{}
    }}
    fn surface(&self) -> {} {{
        crate::surfaces::{}
    }}
}}

",
        model_id_type(model),
        surface_type,
        model.id.to_uppercase(),
        model.longitude_range,
        surface_type,
        surface,
    )
}

fn gen_type_impl(model: &ModelDef) -> String {
    let mtrait = match model.mtype {
        Type::Ellipsoidal => "Ellipsoidal",
        Type::Spherical => "Spherical",
    };
    format!(
        "impl {} for {}Model {{}}

",
        mtrait,
        model_id_type(model)
    )
}

fn model_id_type(model: &ModelDef) -> String {
    let mut s = model.id.clone();
    s.retain(|c| c != '_');
    s
}
