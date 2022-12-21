use std::{fs, io, path::Path};

use memmap::Mmap;
use logos::Logos;
use nalgebra as na;
use anyhow::{anyhow, Result};

#[derive(Logos, Debug, PartialEq)]
pub enum PlyToken<'src> {
    #[token("ply")]
    Ply,

    #[token("format ascii 1.0")]
    FormatAscii1_0,

    #[regex("obj_info [^\n]+", |lex| lex.slice().strip_prefix("obj_info"))]
    ObjInfo(&'src str),

    #[token("element")]
    Element,

    #[token("vertex")]
    Vertex,

    #[token("property")]
    Property,

    #[token("end_header")]
    EndHeader,

    #[regex(r"(-|\+)?[0-9]+", |lex| lex.slice().parse())]
    Int(u32),

    #[regex(r"(-|\+)?[0-9]+\.[0-9]+(e(-|\+)[0-9]+)?", |lex| lex.slice().parse())]
    Num(f32),

    #[error]
    #[regex(r"[ \t\n\f]+", logos::skip)]
    Error,
}

pub fn read_ply(path: impl AsRef<Path>) -> Result<Vec<na::Point3<f64>>> {
    let file = fs::File::open(path)?;
    let mmap = unsafe { Mmap::map(&file)? };
    let contents = unsafe { std::str::from_utf8_unchecked(&mmap) };
    let data = parse_ply(contents)?;
    Ok(data)
}

pub fn parse_ply(input: &str) -> Result<Vec<na::Point3<f64>>> {
    let mut lex = PlyToken::lexer(input);

    if lex.next() != Some(PlyToken::Ply) {
        return Err(anyhow!("expected header 'ply'"));
    }

    if lex.next() != Some(PlyToken::FormatAscii1_0) {
        return Err(anyhow!("expected header 'ascii' format"));
    }

    let n_vertices = loop {
        lex.by_ref()
            .skip_while(|token| !matches!(token, PlyToken::Element))
            .next()
            .ok_or_else(|| anyhow!("expected an 'element vertex <vertex count>' line"))?;
        if Some(PlyToken::Vertex) == lex.next() {
            if let Some(PlyToken::Int(count)) = lex.next() {
                break count;
            } else {
                return Err(anyhow!("expected 'element vertex <vertex count>'"));
            }
        }
    };

    lex.by_ref()
        .skip_while(|token| !matches!(token, PlyToken::EndHeader))
        .next()
        .ok_or_else(|| anyhow!("expected the end of the header"))?;

    let mut parse_float = || -> Result<f64> {
        match lex.next() {
            Some(PlyToken::Num(val)) => Ok(val as f64),
            Some(PlyToken::Int(val)) => Ok(val as f64),
            _ => Err(anyhow!("expected a number")),
        }
    };

    let mut parse_point = || {
        Ok(na::Point3::new(
            parse_float()?,
            parse_float()?,
            parse_float()?,
        ))
    };

    (0..n_vertices).map(|_| parse_point()).try_collect()
}

pub fn output_ply(path: impl AsRef<Path>, points: &[na::Point3<f64>]) -> io::Result<()> {
    use std::io::{Write, BufWriter};

    let mut file = BufWriter::new(fs::File::create(path.as_ref())?);
    writeln!(file, "ply")?;
    writeln!(file, "format ascii 1.0")?;
    writeln!(file, "format ascii 1.0")?;
    writeln!(file, "format ascii 1.0")?;
    writeln!(file, "obj_info is_cyberware_data 1")?;
    writeln!(file, "obj_info is_mesh 0")?;
    writeln!(file, "obj_info is_warped 0")?;
    writeln!(file, "obj_info is_interlaced 1")?;
    // writeln!(file, "obj_info num_cols 512")?;
    // writeln!(file, "obj_info num_rows 400")?;
    // writeln!(file, "obj_info echo_rgb_offset_x 0.013000")?;
    // writeln!(file, "obj_info echo_rgb_offset_y 0.153600")?;
    // writeln!(file, "obj_info echo_rgb_offset_z 0.172000")?;
    // writeln!(file, "obj_info echo_rgb_frontfocus 0.930000")?;
    // writeln!(file, "obj_info echo_rgb_backfocus 0.012660")?;
    // writeln!(file, "obj_info echo_rgb_pixelsize 0.000010")?;
    // writeln!(file, "obj_info echo_rgb_centerpixel 232")?;
    // writeln!(file, "obj_info echo_frames 512")?;
    // writeln!(file, "obj_info echo_lgincr 0.000500")?;
    writeln!(file, "element vertex {}", points.len())?;
    writeln!(file, "property float x")?;
    writeln!(file, "property float y")?;
    writeln!(file, "property float z")?;
    // writeln!(file, "element range_grid 204800")?;
    // writeln!(file, "property list uchar int vertex_indices")?;
    writeln!(file, "end_header")?;
    for point in points {
        writeln!(file, "{} {} {}", point.x, point.y, point.z)?;
    }

    Ok(())
}
