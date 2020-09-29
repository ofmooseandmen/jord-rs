use std::env;
use std::fs::File;
use std::io::Write;
use std::io::{self, BufRead};
use std::path::Path;

use jord::{Ellipsoid, Length, Measure, Sphere, Surface};

fn gen_surfaces(es: Vec<(String, Box<dyn Surface>)>, f: &str) -> std::io::Result<()> {
    let mut file = File::create(f)?;
    file.write("use crate::{Ellipsoid, Sphere, Length};\n".as_bytes())?;
    file.write("\n".as_bytes())?;
    for e in es {
        let s;
        if e.1.flattening() == 0.0 {
            s = gen_sphere(e.0, e.1);
        } else {
            s = gen_ellispoid(e.0, e.1);
        }
        file.write_all(s.as_bytes())?;
    }
    Ok(())
}

fn gen_sphere(n: String, e: Box<dyn Surface>) -> String {
    format!(
        "pub const {}: Sphere = Sphere::new(Length::from_micrometres({}));

",
        n.to_uppercase(),
        e.mean_radius().to_resolution(),
    )
}

fn gen_ellispoid(n: String, e: Box<dyn Surface>) -> String {
    format!(
        "pub const {}: Ellipsoid =  Ellipsoid::from_all(
    Length::from_micrometres({}),
    Length::from_micrometres({}),
    {},
    {},
);

pub const {}: Sphere = Sphere::new(Length::from_micrometres({}));

",
        n.to_uppercase(),
        e.equatorial_radius().to_resolution(),
        e.polar_radius().to_resolution(),
        e.eccentricity(),
        e.flattening(),
        n.to_uppercase() + "_SPHERE",
        e.mean_radius().to_resolution(),
    )
}

fn parse_surface(e: &str) -> (String, Box<dyn Surface>) {
    let mut iter = e.split_ascii_whitespace();
    let name = iter.next().expect("Expected name").to_string();
    let r = iter
        .next()
        .expect("Expected (equatorial) radius")
        .parse::<f64>()
        .unwrap();
    let radius = Length::from_metres(r);

    let next = iter.next();
    match next {
        Some(invf) => (
            name,
            Box::new(Ellipsoid::new(radius, invf.parse::<f64>().unwrap())),
        ),
        None => (name, Box::new(Sphere::new(radius))),
    }
}

fn parse_surfaces(lines: io::Lines<io::BufReader<File>>) -> Vec<(String, Box<dyn Surface>)> {
    let mut vec = Vec::new();
    for line in lines {
        if let Ok(e) = line {
            vec.push(parse_surface(&e));
        }
    }
    vec
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

    let lines = read_lines(in_surfaces)?;
    let surfaces = parse_surfaces(lines);

    let mut out_surfaces = out_dir.to_owned();
    out_surfaces.push_str("/surfaces.rs");

    gen_surfaces(surfaces, &out_surfaces)
}

// The output is wrapped in a Result to allow matching on errors
// Returns an Iterator to the Reader of the lines of the file.
fn read_lines<P>(filename: P) -> io::Result<io::Lines<io::BufReader<File>>>
where
    P: AsRef<Path>,
{
    let file = File::open(filename)?;
    Ok(io::BufReader::new(file).lines())
}
