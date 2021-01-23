use crate::pointcloud::*;
use image::{ImageBuffer, Luma, RgbImage};
use nalgebra::{Matrix3, Matrix4, Vector3, U1, U3};
use num_traits::{FromPrimitive, NumAssign};

type FloatImage = ImageBuffer<Luma<f32>, Vec<f32>>;

pub struct RGBDImage {
    pub color: RgbImage,
    pub depth: FloatImage,
}

impl RGBDImage {
    pub fn pointcloud<T>(
        &self,
        intrinsic: Matrix3<<T as Point>::Item>,
        extrinsic: Matrix4<<T as Point>::Item>,
        depth_cutoff: f64,
    ) -> PointCloud<T>
    where
        T: PointColor + Default,
        <T as Point>::Item: FloatData + FromPrimitive + NumAssign,
    {
        let mut pointcloud = PointCloud::<T>::new();
        let n_total = self.depth.width() * self.depth.height();
        let rot = extrinsic.fixed_slice::<U3, U3>(0, 0);
        let t = extrinsic.fixed_slice::<U3, U1>(0, 3);
        pointcloud.resize(n_total as usize);
        let mut count = 0;
        for (x, y, pixel) in self.depth.enumerate_pixels() {
            let Luma(d) = *pixel;
            if d[0] > 0.0 && (depth_cutoff <= 0.0 || depth_cutoff > d[0].into()) {
                let pz = <T as Point>::Item::from_f32(d[0]).unwrap();
                let px = (<T as Point>::Item::from_u32(x).unwrap() - intrinsic[(0, 2)]) * pz
                    / intrinsic[(0, 0)];
                let py = (<T as Point>::Item::from_u32(y).unwrap() - intrinsic[(1, 2)]) * pz
                    / intrinsic[(1, 1)];
                pointcloud.data[count] =
                    T::from_point(rot * Vector3::<<T as Point>::Item>::new(px, py, pz) + t);
                count += 1;
            }
        }
        pointcloud.resize(count as usize);
        pointcloud
    }
}
