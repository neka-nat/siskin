use super::pointcloud::{FloatData, Normal, Point, PointCloud};
use kd_tree::KdPoint;
use nalgebra::{ComplexField, Matrix3, Vector3};
use num_traits::{FromPrimitive, NumAssign};

impl<T> PointCloud<T>
where
    T: Point
        + Normal<Item = <T as Point>::Item>
        + Copy
        + Default
        + KdPoint<Scalar = <T as Point>::Item>,
    <T as Point>::Item: FloatData + FromPrimitive + ComplexField + NumAssign,
{
    pub fn compute_normals(&mut self, radius: <T as Point>::Item) {
        let kdtree = self.build_kdtree();
        for p in self.data.iter_mut() {
            let found = PointCloud::search_radius(&kdtree, p, radius);
            if found.len() < 3 {
                *p.normal_mut() = Vector3::<<T as Normal>::Item>::new(
                    <T as Normal>::Item::from_f32(0.0).unwrap(),
                    <T as Normal>::Item::from_f32(0.0).unwrap(),
                    <T as Normal>::Item::from_f32(1.0).unwrap(),
                );
            } else {
                let cov = found
                    .iter()
                    .map(|e| e.xyz() * e.xyz().transpose())
                    .sum::<Matrix3<<T as Point>::Item>>();
                let cov =
                    cov.map(|x| x / <T as Point>::Item::from_u32(found.len() as u32).unwrap());
                let eig = cov.symmetric_eigen();
                let idx = eig.eigenvalues.argmin();
                *p.normal_mut() = eig.eigenvectors.column(idx.0).into();
            }
        }
    }
}
