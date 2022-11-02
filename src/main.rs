#![feature(iterator_try_collect)]

use std::{fs, io};

use memmap::Mmap;
use logos::Logos;
use nalgebra as na;

#[derive(Logos, Debug, PartialEq)]
enum PlyToken<'src> {
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

fn parse_ply(input: &str) -> Result<Vec<na::Point3<f32>>, &'static str> {
    let mut lex = PlyToken::lexer(input);

    if lex.next() != Some(PlyToken::Ply) {
        return Err("expected header 'ply'");
    }

    if lex.next() != Some(PlyToken::FormatAscii1_0) {
        return Err("expected header 'ascii' format");
    }

    let n_vertices = loop {
        lex.by_ref()
            .skip_while(|token| !matches!(token, PlyToken::Element))
            .next()
            .ok_or("expected an 'element vertex <vertex count>' line")?;
        if Some(PlyToken::Vertex) == lex.next() {
            if let Some(PlyToken::Int(count)) = lex.next() {
                break count;
            } else {
                return Err("expected 'element vertex <vertex count>'");
            }
        }
    };

    lex.by_ref()
        .skip_while(|token| !matches!(token, PlyToken::EndHeader))
        .next()
        .ok_or("expected the end of the header")?;

    let mut parse_float = || {
        match lex.next() {
            Some(PlyToken::Num(val)) => Ok(val),
            Some(PlyToken::Int(val)) => Ok(val as f32),
            _ => Err("expected a number"),
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

fn output_ply(path: &str, points: &[na::Point3<f32>]) -> io::Result<()> {
    use std::io::{Write, BufWriter};

    let mut file = BufWriter::new(fs::File::create(path)?);
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

fn transmute_kd_slice(points: &[na::Point3<f32>]) -> &kd_tree::KdSlice3<na::Point3<f32>> {
    unsafe { std::mem::transmute(points) }
}

fn naive_icp(
    fix: &mut [na::Point3<f32>],
    movable: &mut [na::Point3<f32>],
    n_iter: usize,
    distance_threashold: f32,
) {
    use kd_tree::KdSlice3;

    {
        let _ = KdSlice3::sort_by_ordered_float(fix);
    }
    let kdtree = transmute_kd_slice(fix);

    for _ in 0..n_iter {
        let mut matchings_fix = Vec::new();
        let mut matchings_movable = Vec::new();
        for p0 in movable.iter() {
            let near = kdtree.nearest(p0).unwrap();
            if near.squared_distance <= distance_threashold.powi(2) {
                matchings_fix.push(near.item.coords);
                matchings_movable.push(p0.coords);
            }
        }

        // center
        let centroid_fix = matchings_fix
            .iter()
            .sum::<na::Vector3<f32>>() / fix.len() as f32;
        let centroid_movable = matchings_movable
            .iter()
            .sum::<na::Vector3<f32>>() / movable.len() as f32;

        let mut a = na::Matrix3xX::from_columns(matchings_movable.as_slice());
        let mut b = na::Matrix3xX::from_columns(matchings_fix.as_slice());

        for mut row in a.column_iter_mut() {
            row -= centroid_fix;
        }

        for mut row in b.column_iter_mut() {
            row -= centroid_movable;
        }

        let h = a * b.transpose();

        let na::SVD { u: Some(u), v_t: Some(v_t), .. } = h.svd(true, true) else {
            panic!("failed to calculate svd");
        };
        let r = v_t.transpose() * u.transpose();
        let t = centroid_fix - r * centroid_movable;

        for point in movable.iter_mut() {
            *point = r * *point;
            *point += t;
        }
    }
}

fn mmap_file(path: &str) -> io::Result<Mmap> {
    let file = fs::File::open(path)?;
    Ok(unsafe { Mmap::map(&file)? })
}

fn get_data(path: &str) -> Result<Vec<na::Point3<f32>>, Box<dyn std::error::Error>> {
    let mmap = mmap_file(path)?;
    let contents = unsafe { std::str::from_utf8_unchecked(&mmap) };
    let data = parse_ply(contents)?;
    Ok(data)
}

macro_rules! time {
    ($e: expr) => {
        time_fn(concat!(stringify!($e), " at ", file!(), ":", line!()), || $e)
    }
}

fn time_fn<T>(name: &str, f: impl FnOnce() -> T) -> T {
    use std::time;
    let start = time::Instant::now();
    let res = f();
    let end = time::Instant::now();
    println!("{name} took {:?} to finish", end.duration_since(start));
    res
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut data1 = time!(get_data("data/bunny/data/bun000.ply"))?;
    let mut data2 = time!(get_data("data/bunny/data/bun045.ply"))?;

    time!(naive_icp(&mut data1, &mut data2, 10, 10.0));

    output_ply("out.ply", &data2)?;

    Ok(())
}
