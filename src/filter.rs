extern crate nalgebra as na;

use super::pointcloud::{FloatData, Point, PointCloud};
use anyhow::*;
use nalgebra::RealField;
use num::cast::AsPrimitive;
use num_traits::FromPrimitive;
use std::collections::HashMap;
use std::iter::Sum;
use std::marker::PhantomData;
use std::ops::{Add, Div};

impl<T> PointCloud<T>
where
    T: Point + Default + Add + Copy + Sum + Div<<T as Point>::Item, Output = T>,
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
                v.into_iter().sum::<T>() / <T as Point>::Item::from_usize(n).unwrap()
            })
            .collect();
        Ok(PointCloud::<T> {
            data: voxelized_data,
            width: self.width,
            _marker: PhantomData,
        })
    }
}
