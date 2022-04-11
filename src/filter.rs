extern crate nalgebra as na;

use super::pointcloud::{FloatData, Point, PointCloud};
use anyhow::*;
use kd_tree::KdPoint;
use nalgebra::RealField;
use num::cast::AsPrimitive;
use num_traits::{Float, FromPrimitive};
use std::collections::HashMap;
use std::iter::Sum;
use std::marker::PhantomData;
use std::ops::{Add, Div};

impl<T> PointCloud<T>
where
    T: Point + Default + Add<Output = T> + Copy + Div<<T as Point>::Item, Output = T>,
    <T as Point>::Item: FloatData + RealField + FromPrimitive + AsPrimitive<u32>,
{
    pub fn voxel_grid_filter(&self, voxel_size: <T as Point>::Item) -> Result<PointCloud<T>> {
        if self.len() <= 2 {
            return Err(anyhow::anyhow!(
                "The number of points should be at least two."
            ));
        }
        let min_bound = self.data[1..]
            .iter()
            .map(|d| *d.xyz())
            .fold(*self.data[0].xyz(), |min, x| min.inf(&x));
        let mut voxel_map = HashMap::<(u32, u32, u32), Vec<T>>::new();
        for p in self.data.iter() {
            let key = (p.xyz() - min_bound) / voxel_size;
            voxel_map
                .entry((key[0].as_(), key[1].as_(), key[2].as_()))
                .or_insert(Vec::new())
                .push(*p);
        }
        let voxelized_data = voxel_map
            .into_iter()
            .map(|(_k, v)| {
                let n = v.len();
                if n == 1 {
                    v[0]
                } else {
                    v[1..].into_iter().fold(v[0], |s, &e| s + e)
                        / <T as Point>::Item::from_usize(n).unwrap()
                }
            })
            .collect();
        Ok(PointCloud::<T> {
            data: voxelized_data,
            width: 1,
            _marker: PhantomData,
        })
    }
}

impl<T> PointCloud<T>
where
    T: Point + Default + Copy + Default + KdPoint<Scalar = <T as Point>::Item>,
    <T as Point>::Item: FloatData + RealField,
{
    pub fn remove_radius_outliers(
        &self,
        radius: <T as Point>::Item,
        neighbor_counts: usize,
    ) -> PointCloud<T> {
        let kdtree = self.build_kdtree();
        let filtered_data: Vec<T> = self
            .data
            .iter()
            .filter(|p| {
                let found = PointCloud::search_radius(&kdtree, p, radius);
                found.len() > neighbor_counts
            })
            .cloned()
            .collect();

        PointCloud::<T> {
            data: filtered_data,
            width: 1,
            _marker: PhantomData,
        }
    }
}

impl<T> PointCloud<T>
where
    T: Point + Default + Copy + Default + KdPoint<Scalar = <T as Point>::Item>,
    <T as Point>::Item: FloatData + RealField + Sum,
{
    pub fn remove_statistical_outliers(
        &self,
        k_neighbors: usize,
        std_ratio: <T as Point>::Item,
    ) -> PointCloud<T> {
        let kdtree = self.build_kdtree();
        let n_points = <T as Point>::Item::from_usize(self.len()).unwrap();
        let mean_stds = self.data.iter().map(|d| {
            let found = PointCloud::search_knn(&kdtree, d, k_neighbors);
            let n_found = <T as Point>::Item::from_usize(found.len()).unwrap();
            let mean = found
                .iter()
                .map(|f| Float::sqrt(f.squared_distance))
                .sum::<<T as Point>::Item>()
                / n_found;
            let std = found
                .iter()
                .map(|f| ((Float::sqrt(f.squared_distance)) - mean).powi(2))
                .sum::<<T as Point>::Item>()
                / n_found;
            (mean, std)
        });
        let mean = mean_stds
            .clone()
            .map(|(m, _s)| m)
            .sum::<<T as Point>::Item>()
            / n_points;
        let filtered_data = self
            .data
            .iter()
            .zip(mean_stds)
            .filter(|(_p, (m, s))| m < &(mean + std_ratio * *s))
            .map(|(p, (_m, _s))| p)
            .cloned()
            .collect();
        PointCloud::<T> {
            data: filtered_data,
            width: 1,
            _marker: PhantomData,
        }
    }
}
