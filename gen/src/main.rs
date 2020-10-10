use std::env;
use std::fs::File;
use std::io::{self, BufRead, Error, ErrorKind, Write};
use std::path::Path;

use jord::{Ellipsoid, Length, Sphere, Surface};

fn gen_surfaces(comments: Vec<String>, surfaces: Vec<Surf>, f: &str) -> io::Result<()> {
    let mut file = File::create(f)?;
    file.write_all("// Copyright: (c) 2020 Cedric Liegeois\n// License: BSD3".as_bytes())?;
    write_new_line(&mut file)?;
    write_new_line(&mut file)?;

    write_comments(&comments, &mut file)?;
    write_new_line(&mut file)?;
    file.write_all("use crate::{Ellipsoid, Length, Sphere};\n".as_bytes())?;
    write_new_line(&mut file)?;

    for surface in surfaces {
        let txt = match surface {
            Surf::Ellipsoid {
                comment,
                name,
                data,
            } => gen_ellispoid(comment, name, data),
            Surf::Sphere {
                comment,
                name,
                data,
            } => gen_sphere(comment, name, data),
        };
        file.write_all(txt.as_bytes())?;
    }
    Ok(())
}

fn write_new_line(file: &mut File) -> io::Result<()> {
    file.write_all(b"\n")?;
    Ok(())
}

fn write_comments(comments: &[String], file: &mut File) -> io::Result<()> {
    for c in comments {
        file.write_all(("//! ".to_owned() + &c).as_bytes())?;
        write_new_line(file)?;
        file.write_all(b"//! ")?;
    }
    write_new_line(file)?;
    Ok(())
}

fn gen_sphere(c: String, n: String, e: Sphere) -> String {
    format!(
        "/// {}
pub const {}: Sphere = Sphere::new(Length::from_micrometres({}));

",
        c,
        n.to_uppercase(),
        e.mean_radius().micrometres(),
    )
}

fn gen_ellispoid(c: String, n: String, e: Ellipsoid) -> String {
    format!(
        "/// {}
pub const {}: Ellipsoid =  Ellipsoid::from_all(
    Length::from_micrometres({}),
    Length::from_micrometres({}),
    {},
    {},
);

/// {}
pub const {}: Sphere = Sphere::new(Length::from_micrometres({}));

",
        c,
        n.to_uppercase(),
        e.equatorial_radius().micrometres(),
        e.polar_radius().micrometres(),
        e.eccentricity(),
        e.flattening(),
        "Sphere derived from: ".to_owned() + &c,
        n.to_uppercase() + "_SPHERE",
        e.mean_radius().micrometres(),
    )
}

enum Surf {
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

fn parse_surface(text: &Text) -> io::Result<(Surf, Text)> {
    let txt = text.skip_empty();
    let (comment, txt) = txt.next_if_prefixed("# ")?;
    let (name, txt) = txt.next()?;
    match txt.next_if_prefixed("  a: ") {
        Err(_) => {
            let (r, txt) = txt.next_if_prefixed("  r: ")?;
            Ok((
                Surf::Sphere {
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
                Surf::Ellipsoid {
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

#[derive(Debug)]
struct Text(Vec<String>);

impl Text {
    fn from_file_content<P>(filename: P) -> io::Result<Text>
    where
        P: AsRef<Path>,
    {
        let file = File::open(filename)?;
        let content = io::BufReader::new(file).lines();
        let lines = content.filter_map(Result::ok).collect();
        Ok(Text(lines))
    }

    fn is_eot(&self) -> bool {
        self.0.is_empty()
    }

    fn skip_empty(&self) -> Self {
        Text(
            self.0
                .iter()
                .skip_while(|l| l.is_empty())
                .map(|s| s.to_string())
                .collect(),
        )
    }

    fn next(&self) -> io::Result<(String, Text)> {
        match self.0.first() {
            None => Err(Error::new(ErrorKind::UnexpectedEof, "expected more")),
            Some(s) => Ok((s.to_string(), Text(self.0[1..].to_vec()))),
        }
    }

    fn next_if_prefixed(&self, prefix: &str) -> io::Result<(String, Text)> {
        match self.0.first().and_then(|s| s.strip_prefix(prefix)) {
            None => Err(Error::new(
                ErrorKind::UnexpectedEof,
                format!("expected {}, found {:?}", prefix, self.0.first()),
            )),
            Some(s) => Ok((s.to_string(), Text(self.0[1..].to_vec()))),
        }
    }
}

fn parse_surfaces(text: Text) -> io::Result<(Vec<String>, String, Vec<Surf>)> {
    let (comments, txt) = parse_comments(text);
    match parse_module(txt) {
        Err(e) => Err(e),
        Ok((module, txt)) => {
            let mut vec = Vec::new();
            let mut done = false;
            let mut rest = txt;
            while !done {
                let (s, t) = parse_surface(&rest)?;
                rest = t;
                vec.push(s);
                rest = rest.skip_empty();
                done = rest.is_eot();
            }
            Ok((comments, module, vec))
        }
    }
}

fn parse_comments(text: Text) -> (Vec<String>, Text) {
    let mut txt = text.skip_empty();
    let mut comments = Vec::new();
    let mut done = false;
    while !done {
        match txt.next_if_prefixed("# ") {
            Err(_) => {
                done = true;
            }
            Ok((cmt, rtxt)) => {
                comments.push(cmt.to_string());
                txt = rtxt
            }
        }
    }
    (comments, txt)
}

fn parse_module(text: Text) -> io::Result<(String, Text)> {
    let txt = text.skip_empty();
    let res = txt.next_if_prefixed("mod ")?;
    Ok(res)
}

pub fn main() -> std::io::Result<()> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        panic!("usage gen [input dir] [output dir]");
    }
    let in_dir = &args[1];
    let out_dir = &args[2];

    let mut in_surfaces = in_dir.to_owned();
    in_surfaces.push_str("/surfaces.txt");

    let text = Text::from_file_content(in_surfaces)?;
    let surfaces = parse_surfaces(text)?;

    let mut out_surfaces = out_dir.to_owned();
    out_surfaces.push_str(&format!("/{}.rs", surfaces.1));

    gen_surfaces(surfaces.0, surfaces.2, &out_surfaces)
}
