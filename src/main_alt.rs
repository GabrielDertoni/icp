use std::{fs, io};

use memmap::Mmap;

#[derive(Clone, Debug)]
struct VertexBuf {
    xs: Vec<f32>,
    ys: Vec<f32>,
    zs: Vec<f32>,
}

impl VertexBuf {
    fn new() -> Self {
        Self {
            xs: Vec::new(),
            ys: Vec::new(),
            zs: Vec::new(),
        }
    }

    fn push_vertex(&mut self, (x, y, z): (f32, f32, f32)) {
        self.xs.push(x);
        self.ys.push(y);
        self.zs.push(z);
    }

    fn len(&self) -> usize {
        self.xs.len()
    }
}

/*
fn parse_ply(input: &str) -> IResult<&str, VertexBuf> {
    use nom::{
        number::complete::double,
        bytes::complete::{tag, take_till},
        character::{
            is_newline,
            complete::{line_ending, not_line_ending, space1, digit1}
        },
        combinator::{not, opt},
        multi::many0_count,
        branch::alt,
        sequence::{separated_pair, preceded},
        error::context,
        Parser,
    };

    fn element_vertex(input: &str) -> IResult<&str, usize> {
        let (input, _) = tag("element vertex ")(input)?;
        let (input, num) = digit1.map(|s: &str| s.parse().unwrap()).parse(input)?;
        let (input, _) = line_ending(input)?;
        Ok((input, num))
    }

    fn end_header(input: &str) -> IResult<&str, ()> {
        let (input, _) = tag("end_header").and(line_ending).parse(input)?;
        Ok((input, ()))
    }

    let (input, _) = preceded(tag("ply"), line_ending)
        .and(preceded(tag("format ascii 1.0"), line_ending))
        .parse(input)?;

    dbg!(&input[0..1]);
    let (input, _) = take_till(|c: char| is_newline(c as u8))(input)?;
        // .and(many0_count(not(element_vertex).and(many0_count(not_line_ending)).and(line_ending)))

    let (input, n_vertices) = context("element_vertex", element_vertex)(input)?;
    let (input, _) = many0_count(
            not(end_header)
                .and(preceded(many0_count(not_line_ending), line_ending))
        )
        .and(end_header)
        .parse(input)?;

    fn vertex(input: &str) -> IResult<&str, (f32, f32, f32)> {
        let sep = space1;
        let (input, ((x, y), z)) = separated_pair(separated_pair(double, sep, double), sep, double)(input)?;
        let (input, _) = line_ending(input)?;
        Ok((input, (x as f32, y as f32, z as f32)))
    }

    let mut buf = VertexBuf::new();
    let mut input = input;
    for _ in 0..n_vertices {
        let (inp, triplet) = vertex(input)?;
        input = inp;
        buf.push_vertex(triplet);
    }

    return Ok((input, buf));
}
*/

fn parse_ply(input: &str) -> Result<VertexBuf, String> {
    let mut lines = input.lines().fuse().peekable();

    let header = lines.next().ok_or("expected a header")?;
    if header != "ply" {
        return Err("expected header 'ply'".to_string());
    }

    while let Some(line) = lines.peek() && !line.starts_with("element vertex") {
        lines.next();
    }

    let (_, num) = lines.next().unwrap().rsplit_once(' ').ok_or("expected a number")?;
    let n_vertices: usize = num.parse().map_err(|_| "expected a number")?;

    while let Some(line) = lines.peek() && !line.starts_with("end_header") {
        lines.next();
    }
    lines.next();

    let mut buf = VertexBuf::new();
    for line in lines.take(n_vertices) {
        let mut it = line.split(' ');
        let x = it.next().ok_or("expected x")?.parse().map_err(|_| "expected x to be a number")?;
        let y = it.next().ok_or("expected y")?.parse().map_err(|_| "expected y to be a number")?;
        let z = it.next().ok_or("expected z")?.parse().map_err(|_| "expected z to be a number")?;
        buf.push_vertex((x, y, z));
    }

    Ok(buf)
}

fn naive_icp(mut cloud1: VertexBuf, mut cloud2: VertexBuf) {
    use kd_tree::KdTree3;

    // center
    let centroid1 = (0..cloud1.len())
        .fold([0.0, 0.0, 0.0], |acc, i| {
            [acc[0] + cloud1.xs[i], acc[1] + cloud1.ys[i], acc[2] + cloud1.zs[i]]
        })
        .map(|e| e / cloud1.len() as f32);

    for i in 0..cloud1.len() {
        cloud1.xs[i] -= centroid1[0];
        cloud1.ys[i] -= centroid1[1];
        cloud1.zs[i] -= centroid1[2];
    }

    let centroid2 = (0..cloud2.len())
        .fold([0.0, 0.0, 0.0], |acc, i| {
            [acc[0] + cloud2.xs[i], acc[1] + cloud2.ys[i], acc[2] + cloud2.zs[i]]
        })
        .map(|e| e / cloud2.len() as f32);

    for i in 0..cloud2.len() {
        cloud2.xs[i] -= centroid2[0];
        cloud2.ys[i] -= centroid2[1];
        cloud2.zs[i] -= centroid2[2];
    }

    let indicies = (0..cloud2.len()).collect();
    let point_dim = |i: usize, k: usize| {
        match k {
            0 => cloud2.xs[i],
            1 => cloud2.ys[i],
            2 => cloud2.zs[i],
            _ => unreachable!(),
        }
    };

    let kdtree = KdTree3::build_by(indicies, |&i, &j, k| point_dim(i, k).total_cmp(&point_dim(j, k)));

    let mut dumb = 0;
    for i in 0..cloud1.len() {
        let p0 = [cloud1.xs[i], cloud1.ys[i], cloud1.zs[i]];
        let near = kdtree.nearest_by(&p0, |&i, k| point_dim(i, k)).unwrap();
        // cloud1.xs.swap(near, i);
        dumb += near.item;
    }
    dbg!(dumb);
}

fn mmap_file(path: &str) -> io::Result<Mmap> {
    let file = fs::File::open(path)?;
    Ok(unsafe { Mmap::map(&file)? })
}

fn get_data(path: &str) -> Result<VertexBuf, Box<dyn std::error::Error>> {
    let mmap = mmap_file(path)?;
    let contents = unsafe { std::str::from_utf8_unchecked(&mmap) };
    let data = parse_ply(contents)?;
    Ok(data)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let data1 = get_data("data/bunny/data/bun000.ply")?;
    let data2 = get_data("data/bunny/data/bun045.ply")?;

    naive_icp(data1, data2);

    Ok(())
}
