use std::{io, fs, path::Path};

use memmap::Mmap;
use nalgebra as na;
use anyhow::{anyhow, Result, Context};

pub fn read_obj(path: impl AsRef<Path>) -> Result<Vec<na::Point3<f64>>> {
    let path = path.as_ref();
    let file = fs::File::open(path)
        .with_context(|| format!("failed to open file at {path:?}"))?;
    let mmap = unsafe {
        Mmap::map(&file)
            .context("mmap failed")?
    };
    // SAFETY: Hopefully it will be utf8 :)
    let contents = unsafe { std::str::from_utf8_unchecked(&mmap) };
    let data = parse_obj(contents)
        .with_context(|| format!("could not parse contents of file {path:?} as an '.obj'"))?;
    Ok(data)
}

pub fn parse_obj(input: &str) -> Result<Vec<na::Point3<f64>>> {
    let mut points = Vec::new();
    for line in input.lines() {
        let mut it = line.split_whitespace();
        match it.next() {
            Some("v") => {
                let [x, y, z] = it.next_chunk().map_err(|_| anyhow!("expected x, y and z values"))?;
                points.push(na::Point3::new(
                    x.parse().map_err(|_| anyhow!("expected a number"))?,
                    y.parse().map_err(|_| anyhow!("expected a number"))?,
                    z.parse().map_err(|_| anyhow!("expected a number"))?,
                ));
            },
            _ => return Err(anyhow!("unexpected data")),
        }
    }
    Ok(points)
}

pub fn output_obj(path: impl AsRef<Path>, points: &[na::Point3<f64>]) -> io::Result<()> {
    use std::io::{Write, BufWriter};

    let mut file = BufWriter::new(fs::File::create(path.as_ref())?);
    for point in points {
        writeln!(file, "v {} {} {}", point.x, point.y, point.z)?;
    }

    Ok(())
}
