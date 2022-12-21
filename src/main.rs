#![feature(iterator_try_collect, slice_flatten, iter_next_chunk)]
#![allow(dead_code)]

mod ply;
mod obj;

use std::{
    env,
    ops::{MulAssign, RangeInclusive},
    path::{Path, PathBuf},
    collections::HashMap,
};

use nalgebra as na;
use anyhow::{anyhow, Result, Context};
use ordered_float::OrderedFloat;
use arrayvec::ArrayVec;

fn find_transformation<SA, SB>(
    mut movable: na::Matrix<f64, na::Const<3>, na::Dynamic, SA>,
    mut fix: na::Matrix<f64, na::Const<3>, na::Dynamic, SB>
) -> na::IsometryMatrix3<f64>
where
    SA: na::StorageMut<f64, na::U3, na::Dynamic>,
    SB: na::StorageMut<f64, na::U3, na::Dynamic>,
{
    let n = movable.ncols();
    // center
    let centroid_movable = movable.column_sum() / n as f64;
    let centroid_fix     = fix.column_sum() / n as f64;

    for mut row in fix.column_iter_mut() {
        row -= centroid_fix;
    }

    for mut row in movable.column_iter_mut() {
        row -= centroid_movable;
    }

    let h = movable * fix.transpose();

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
    movable: &[na::Point3<f64>],
    n_iter: usize,
    tol: f64,
) -> na::IsometryMatrix3<f64> {
    use fnntw::Tree;

    let fix_slice_of_slice: &[[f64; 3]] = bytemuck::cast_slice(fix);
    let kdtree = Tree::new(fix_slice_of_slice, 8).unwrap();

    let mut movable = movable.to_vec();

    let mut total_isom = na::IsometryMatrix3::<f64>::identity();
    let mut prev_err = f64::INFINITY;
    for _ in 0..n_iter {
        let mut matchings_fix: Vec<[f64; 3]> = Vec::new();
        let mut matchings_movable: Vec<[f64; 3]> = Vec::new();
        for p0 in &movable {
            let (_, _, near) = kdtree.query_nearest(bytemuck::cast_ref(p0)).unwrap();
            matchings_fix.push(bytemuck::cast(*near));
            matchings_movable.push(bytemuck::cast(*p0));
        }

        let n = matchings_fix.len();
        let a = na::MatrixSliceMut3xX::from_slice(matchings_movable.flatten_mut(), n);
        let b = na::MatrixSliceMut3xX::from_slice(matchings_fix.flatten_mut(), n);
        let err = (&a - &b)
            .column_iter()
            .map(|col| col.apply_norm(&na::EuclideanNorm))
            .sum::<f64>() / n as f64;
        let isom = find_transformation(a, b);

        for point in &mut movable {
            *point = isom.transform_point(point);
        }
        total_isom = isom * total_isom;

        if (prev_err - err).abs() < tol {
            break;
        }
        prev_err = err;
    }
    total_isom
}

fn parallel_naive_icp(
    fix: &[na::Point3<f64>],
    movable: &[na::Point3<f64>],
    n_iter: usize,
    tol: f64,
) -> na::IsometryMatrix3<f64> {
    use fnntw::Tree;
    use rayon::prelude::*;

    let fix_slice_of_slice: &[[f64; 3]] = bytemuck::cast_slice(fix);
    let kdtree = Tree::new_parallel(fix_slice_of_slice, 8, 12).unwrap();

    let mut movable = movable.to_vec();

    let mut total_isom = na::IsometryMatrix3::<f64>::identity();
    let mut prev_err = f64::INFINITY;
    for _ in 0..n_iter {
        const N: usize = 1;
        let mut matchings_fix: Vec<[[f64; 3]; N]> = Vec::new();
        let mut matchings_movable: Vec<[[f64; 3]; N]> = Vec::new();
        movable
            .par_chunks_exact(N)
            .map(|ps| {
                let mut arr_fix: [[f64; 3]; N] = [[0.0; 3]; N];
                let mut arr_mov: [[f64; 3]; N] = [[0.0; 3]; N];
                for i in 0..N {
                    let (_, _, near) = kdtree.query_nearest(bytemuck::cast_ref(&ps[i])).unwrap();
                    arr_fix[i] = bytemuck::cast(*near);
                    arr_mov[i] = bytemuck::cast(ps[i]);
                }
                (arr_fix, arr_mov)
            })
            .unzip_into_vecs(&mut matchings_fix, &mut matchings_movable);

        let mut matchings_fix: Vec<[f64; 3]> = bytemuck::allocation::cast_vec(matchings_fix);
        let mut matchings_movable: Vec<[f64; 3]> = bytemuck::allocation::cast_vec(matchings_movable);

        let n = matchings_fix.len();
        let a = na::MatrixSliceMut3xX::from_slice(matchings_movable.flatten_mut(), n);
        let b = na::MatrixSliceMut3xX::from_slice(matchings_fix.flatten_mut(), n);
        let err = (&a - &b)
            .column_iter()
            .map(|col| col.apply_norm(&na::EuclideanNorm))
            .sum::<f64>() / n as f64;
        let isom = find_transformation(a, b);

        for point in &mut movable {
            *point = isom.transform_point(point);
        }
        total_isom = isom * total_isom;

        if (prev_err - err).abs() < tol {
            break;
        }
        prev_err = err;
    }
    total_isom
}

pub struct AABB3<T> {
    pub x_bounds: RangeInclusive<T>,
    pub y_bounds: RangeInclusive<T>,
    pub z_bounds: RangeInclusive<T>,
}

impl AABB3<f64> {
    fn x_size(&self) -> f64 {
        return self.x_bounds.end() - self.x_bounds.start()
    }

    fn y_size(&self) -> f64 {
        return self.y_bounds.end() - self.y_bounds.start()
    }

    fn z_size(&self) -> f64 {
        return self.z_bounds.end() - self.z_bounds.start()
    }
}

fn find_min_aabb<'a>(points: impl IntoIterator<Item = &'a na::Point3<f64>>) -> AABB3<f64> {
    let mut it = points.into_iter();
    let fst = it.next().expect("points must have at least one item");
    it.fold(
        AABB3 {
            x_bounds: fst.x..=fst.x,
            y_bounds: fst.y..=fst.y,
            z_bounds: fst.z..=fst.z,
        },
        |aabb, el| AABB3 {
            x_bounds: el.x.min(*aabb.x_bounds.start())..=el.x.max(*aabb.x_bounds.end()),
            y_bounds: el.y.min(*aabb.y_bounds.start())..=el.y.max(*aabb.y_bounds.end()),
            z_bounds: el.z.min(*aabb.z_bounds.start())..=el.z.max(*aabb.z_bounds.end()),
        }
    )
}

struct Voxelized<const N: usize> {
    voxels: HashMap<na::Point3<i32>, ArrayVec<(na::Point3<f64>, usize), N>>,
}

impl<const N: usize> Voxelized<N> {
    const EMPTY: HashMap<na::Point3<i32>, ArrayVec<(na::Point3<f64>, usize), N>> = HashMap::new();

    pub fn to_vec(self) -> Vec<na::Point3<f64>> {
        // TODO: Ideally this wouldn't need to be initialized to `default()` since we know that
        // every position will be assigned to.
        let mut vec = vec![na::Point3::default(); self.voxels.len()];
        for v in self.voxels.into_values() {
            for (point, i) in v {
                vec[i] = point;
            }
        }
        vec
    }

    pub fn from_iter(
        points: impl IntoIterator<Item = na::Point3<f64>>,
        voxel_size: f64,
    ) -> Self {
        Voxelized {
            voxels: Self::grid_from_iter(points, voxel_size)
        }
    }

    pub fn sub_voxelize(&mut self, voxel_size: f64) {
        let v = self.to_vec();
        *self = Self::from_iter(v, voxel_size);
    }

    pub fn merge<const M: usize>(&mut self, other: Voxelized<M>) {
        for (key, points) in other.voxels {
            if let Some(v) = self.voxels.get_mut(&key) {
                let cap = v.remaining_capacity();
                if cap > 0 {
                    let points = &points[..points.len().min(cap)];
                    v.try_extend_from_slice(points).unwrap();
                }
            }
        }
    }

    fn grid_from_iter(
        points: impl IntoIterator<Item = na::Point3<f64>>,
        voxel_size: f64,
    ) -> HashMap<na::Point3<i32>, ArrayVec<(na::Point3<f64>, usize), N>> {
        use std::collections::hash_map::Entry;

        let mut voxels = HashMap::new();
        let mut i = 0;
        for point in points {
            let idx = na::Point3::new(
                point.x.div_euclid(voxel_size) as i32,
                point.y.div_euclid(voxel_size) as i32,
                point.z.div_euclid(voxel_size) as i32,
            );
            if let Entry::Vacant(entry) = voxels.entry(bytemuck::cast(idx)) {
                let mut v = ArrayVec::new();
                let _ = v.try_push((point, i));
                entry.insert(v);
                i += 1;
            }
        }
        voxels
    }
}


fn downsample_icp(
    fix: &[na::Point3<f64>],
    movable: &[na::Point3<f64>],
    n_iter: usize,
    tol: f64,
) -> na::IsometryMatrix3<f64> {
    let fix_aabb = find_min_aabb(fix);
    let mut ranges = [fix_aabb.x_size(), fix_aabb.y_size(), fix_aabb.z_size()];
    ranges.sort_by_key(|&el| OrderedFloat(el));
    let median_range = ranges[1];

    let voxel_size = median_range / 100.0;
    // let voxel_size = 1.0;
    // Collect into a vec, since we aren't going to really use the `HashMap`s.
    let fix_voxelized = Voxelized::<1>::from_iter(fix.iter().copied(), voxel_size * 1.5).to_vec();
    let movable_voxelized = Voxelized::<1>::from_iter(movable.iter().copied(), voxel_size * 1.5).to_vec();

    naive_icp(&fix_voxelized, &movable_voxelized, n_iter, tol)
}

struct ICP {
    poses: na::IsometryMatrix3<f64>,
    map: Vec<na::Point3<f64>>,
    voxel_size: f64,
}

impl ICP {
    fn new(voxel_size: f64) -> Self {
        ICP {
            poses: na::IsometryMatrix3::identity(),
            map: Vec::new(),
            voxel_size,
        }
    }

    fn register(&mut self, points: &[na::Point3<f64>]) {
        let voxelized = Voxelized::<1>::from_iter(points.iter().copied(), self.voxel_size).to_vec();
        if self.map.len() == 0 {
            self.map = Voxelized::<20>::from_iter(voxelized, self.voxel_size);
            return;
        }

    }
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
    let mut poses = vec![na::IsometryMatrix3::identity()];
    let mut acc = na::IsometryMatrix3::identity();
    for data in tqdm::tqdm(it) {
        let data = data?;
        // let isom = naive_icp(&prev, &data, 50, 1e-6);
        // let isom = downsample_icp(&prev, &data, 50, 1e-6);
        let isom = parallel_naive_icp(&prev, &data, 50, 1e-6);
        acc = isom * acc;
        poses.push(acc);
        prev = data;
    }
    Ok(poses)
}

fn main() -> Result<()> {
    let paths = env::args()
        .skip(1)
        .map(|fname| PathBuf::from(fname))
        .collect::<Vec<_>>();

    let trajectory = series(&paths)?;

    let dirname: &Path = "fixed".as_ref();
    for (isom, path) in trajectory.iter().zip(paths) {
        let mut points = obj::read_obj(&path)?;
        for point in &mut points {
            *point = isom.transform_point(point);
        }
        let fname = dirname.join(path.file_name().unwrap());
        obj::output_obj(fname, points.as_slice())?;
    }

    Ok(())
}
