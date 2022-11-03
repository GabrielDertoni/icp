#![feature(iterator_try_collect, slice_flatten, iter_next_chunk)]
#![allow(dead_code)]

mod ply;
mod obj;

use std::{env, ops::MulAssign, path::{Path, PathBuf}};

use nalgebra as na;
use anyhow::{anyhow, Result, Context};

fn find_transformation<SA, SB>(
    mut a: na::Matrix<f64, na::Const<3>, na::Dynamic, SA>,
    mut b: na::Matrix<f64, na::Const<3>, na::Dynamic, SB>
) -> na::IsometryMatrix3<f64>
where
    SA: na::StorageMut<f64, na::U3, na::Dynamic>,
    SB: na::StorageMut<f64, na::U3, na::Dynamic>,
{
    let n = a.ncols();
    // center
    let centroid_movable = a.column_sum() / n as f64;
    let centroid_fix     = b.column_sum() / n as f64;

    for mut row in b.column_iter_mut() {
        row -= centroid_fix;
    }

    for mut row in a.column_iter_mut() {
        row -= centroid_movable;
    }

    let h = a * b.transpose();

    let na::SVD { u: Some(u), v_t: Some(mut v_t), .. } = h.svd(true, true) else {
        panic!("failed to calculate svd");
    };
    let u_t = u.transpose();
    let r = {
        let r = v_t.tr_mul(&u_t);
        let r = if r.determinant() < 0.0 {
            v_t.row_mut(2).mul_assign(-1.0);
            v_t.tr_mul(&u_t)
        } else {
            r
        };
        na::Rotation3::from_matrix(&r)
    };

    let t = na::Translation3::from(centroid_fix - r * centroid_movable);
    na::IsometryMatrix3::from_parts(t, r)
}

fn naive_icp(
    fix: &[na::Point3<f64>],
    movable: &mut [na::Point3<f64>],
    n_iter: usize,
    distance_threashold: f64,
) -> na::IsometryMatrix3<f64> {
    use fnntw::Tree;

    // SAFETY: `Point3` is `repr(C)` with a `SVector` inside which is also `repr(C)` and has inside
    // it just the data for the vector. This data is should just be a `[[f64; 3] 1]` which we can
    // just transmute to `[f64; 3]`.
    let fix_slice_of_slice: &[[f64; 3]] = unsafe { std::mem::transmute(fix) };
    let kdtree = Tree::new(fix_slice_of_slice, 16).unwrap();

    let mut total_isom = na::IsometryMatrix3::<f64>::identity();
    for _ in 0..n_iter {
        let mut matchings_fix: Vec<[f64; 3]> = Vec::new();
        let mut matchings_movable = Vec::new();
        for p0 in movable.iter() {
            let (squared_distance, _, near) = kdtree.query_nearest(&p0.coords.data.0[0]).unwrap();
            if squared_distance <= distance_threashold.powi(2) {
                // SAFETY: `NonNan` is `repr(transparent)`
                matchings_fix.push(unsafe { std::mem::transmute(*near) });
                matchings_movable.push(p0.coords.data.0[0]);
            }
        }

        let n = matchings_fix.len();
        let a = na::MatrixSliceMut3xX::from_slice(matchings_movable.flatten_mut(), n);
        let b = na::MatrixSliceMut3xX::from_slice(matchings_fix.flatten_mut(), n);
        let isom = find_transformation(a, b);

        for point in movable.iter_mut() {
            *point = isom.transform_point(point);
        }
        total_isom = isom * total_isom;
    }
    total_isom
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

fn series<P: AsRef<Path>>(paths: impl IntoIterator<Item = P>) -> Result<Vec<na::IsometryMatrix3<f64>>> {
    let mut it = paths
        .into_iter()
        .map(|path| obj::read_obj(path));
    let mut prev = it.next().expect("paths to have at least one element")?;
    let mut trajectory = vec![na::IsometryMatrix3::identity()];
    for data in tqdm::tqdm(it) {
        let mut data = data?;
        let isom = naive_icp(&prev, &mut data, 50, 10.0);
        // let isom = naive_icp(&data, &mut prev, 50, 10.0);
        trajectory.push(isom);
        prev = data;
    }
    Ok(trajectory)
}

fn main() -> Result<()> {
    /*
    let mut data1 = time!(ply::read_ply("data/bunny/data/bun000.ply"))?;
    let mut data2 = time!(ply::read_ply("data/bunny/data/bun045.ply"))?;

    let isom = time!(naive_icp(&mut data1, &mut data2, 10, 10.0));
    dbg!(isom.rotation.axis());
    dbg!(isom.rotation.angle());
    dbg!(360.0 * isom.rotation.angle() / std::f64::consts::TAU);
    dbg!(&isom.translation);
    time!(ply::output_ply("out.ply", &data2))?;
    */

    /*
    let isom1 = {
        let mut data1 = time!(obj::read_obj("data/KITTI-Sequence/000000/000000_points.obj"))?;
        let mut data2 = time!(obj::read_obj("data/KITTI-Sequence/000001/000001_points.obj"))?;
        time!(naive_icp(&mut data1, &mut data2, 50, 10.0))
    };

    let isom = {
        let mut data1 = time!(obj::read_obj("data/KITTI-Sequence/000001/000001_points.obj"))?;
        let mut data2 = time!(obj::read_obj("data/KITTI-Sequence/000002/000002_points.obj"))?;
        time!(naive_icp(&mut data1, &mut data2, 50, 10.0))
    };
    dbg!(isom.rotation.axis());
    dbg!(isom.rotation.angle());
    dbg!(360.0 * isom.rotation.angle() / std::f64::consts::TAU);
    dbg!(&isom.translation);

    let mut points = time!(obj::read_obj("data/KITTI-Sequence/000002/000002_points.obj"))?;
    let total_isom = isom * isom1;
    for point in &mut points {
        *point = total_isom.transform_point(point);
    }
    time!(obj::output_obj("out.obj", &points))?;
    */

    let paths = env::args()
        .skip(1)
        .map(|fname| PathBuf::from(fname))
        .collect::<Vec<_>>();

    let trajectory = series(&paths)?;
    /*
    let final_isom = trajectory.iter()
        .copied()
        .reduce(|lhs, rhs| rhs * lhs)
        .unwrap();

    // let mut points = time!(obj::read_obj("data/KITTI-Sequence/000029/000029_points.obj"))?;
    let mut points = time!(obj::read_obj("data/KITTI-Sequence/000000/000000_points.obj"))?;
    for point in &mut points {
        *point = final_isom.transform_point(point);
    }
    time!(obj::output_obj("out.obj", &points))?;
    */

    let dirname: &Path = "fixed".as_ref();
    for (isom, path) in trajectory.iter().zip(paths) {
        let vec = isom.translation.vector;
        println!("[{}, {}, {}]", vec.x, vec.y, vec.z);

        let mut points = obj::read_obj(&path)?;
        for point in &mut points {
            *point = isom.transform_point(point);
        }
        let fname = dirname.join(path.file_name().unwrap());
        obj::output_obj(fname, points.as_slice())?;
    }

    Ok(())
}
