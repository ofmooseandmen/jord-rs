use std::env;
use std::fs::File;
use std::io::{self, Write};

mod model;
pub use model::*;

mod surface;
pub use surface::*;

mod text;
pub use text::*;

fn gen<F, T>(
    comments: Vec<String>,
    data: Vec<T>,
    generate: F,
    imports: String,
    f: &str,
) -> io::Result<()>
where
    F: Fn(T) -> String,
{
    let mut file = File::create(f)?;
    file.write_all("// Copyright: (c) 2020 Cedric Liegeois\n// License: BSD3".as_bytes())?;
    write_new_line(&mut file)?;
    write_new_line(&mut file)?;

    write_comments(&comments, &mut file)?;
    write_new_line(&mut file)?;
    file.write_all(format!("use crate::{{{}}};\n", imports).as_bytes())?;
    write_new_line(&mut file)?;

    for d in data {
        let txt = generate(d);
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
        file.write_all(b"//!")?;
    }
    write_new_line(file)?;
    Ok(())
}

fn parse<F, T>(text: Text, parse: F) -> io::Result<(Vec<String>, String, Vec<T>)>
where
    F: Fn(&Text) -> io::Result<(T, Text)>,
{
    let (comments, txt) = parse_comments(text);
    match parse_module(txt) {
        Err(e) => Err(e),
        Ok((module, txt)) => {
            let mut vec = Vec::new();
            let mut done = false;
            let mut rest = txt.skip_empty();
            while !done {
                let (s, t) = parse(&rest)?;
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
    let surfaces = parse(text, parse_surface)?;

    let mut out_surfaces = out_dir.to_owned();
    out_surfaces.push_str(&format!("/{}.rs", surfaces.1));

    gen(
        surfaces.0,
        surfaces.2,
        gen_surface,
        surface_imports(),
        &out_surfaces,
    )?;

    let mut in_models = in_dir.to_owned();
    in_models.push_str("/models.txt");

    let text = Text::from_file_content(in_models)?;
    let models = parse(text, parse_model)?;

    let mut out_models = out_dir.to_owned();
    out_models.push_str(&format!("/{}.rs", models.1));

    gen(models.0, models.2, gen_model, model_imports(), &out_models)?;

    Ok(())
}
