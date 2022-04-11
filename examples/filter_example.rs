use anyhow::*;
use siskin::{read_pcd, visualization, PointXYZ};

pub fn main() -> Result<()> {
    let pcd = read_pcd::<PointXYZ<f32>>("examples/data/bunny.pcd")?;
    let filted_pcd = pcd.voxel_grid_filter(0.01)?;
    let mut vis = visualization::Visualizer::new();
    vis.add_pointcloud(&filted_pcd);
    vis.spin();
    Ok(())
}
