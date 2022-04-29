extern crate nalgebra as na;
use kd_tree::{ItemAndDistance, KdPoint, KdTree};
use num_traits::NumAssign;
use ordered_float::OrderedFloat;
use typenum;
use super::pointcloud::{FloatData, Point, PointCloud, PointXYZ, PointXYZNormal};

impl<T> KdPoint for PointXYZ<T>
where
    T: FloatData + NumAssign,
{
    type Scalar = T;
    type Dim = typenum::U3;
    fn at(&self, k: usize) -> T {
        self.point[k]
    }
}

impl<T, U> KdPoint for PointXYZNormal<T, U>
where
    T: FloatData + NumAssign,
    U: FloatData,
{
    type Scalar = T;
    type Dim = typenum::U3;
    fn at(&self, k: usize) -> T {
        self.point[k]
    }
}

impl<T> PointCloud<T>
where
    T: Point + Copy + KdPoint<Scalar = <T as Point>::Item>,
    <T as Point>::Item: FloatData,
{
    pub fn build_kdtree(&self) -> KdTree<T> {
        KdTree::build_by(self.data.clone(), |item1, item2, k| {
            OrderedFloat(item1.xyz()[k]).cmp(&OrderedFloat(item2.xyz()[k]))
        })
    }
    pub fn search_knn<'a>(
        kdtree: &'a KdTree<T>,
        query: &T,
        k: usize,
    ) -> Vec<ItemAndDistance<'a, T, <T as Point>::Item>> {
        kdtree.nearests(query, k)
    }
    pub fn search_radius<'a>(
        kdtree: &'a KdTree<T>,
        query: &T,
        radius: <T as Point>::Item,
    ) -> Vec<&'a T> {
        kdtree.within_radius(query, radius)
    }
}
