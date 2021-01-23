use anyhow::*;
use siskin::{read_pcd, visualization, PointXYZ};

pub fn main() -> Result<()> {
    let pcd = read_pcd::<PointXYZ<f32>>("examples/data/bunny.pcd")?;
    let mut vis = visualization::Visualizer::new();
    vis.add_pointcloud(&pcd);
    vis.spin();
    Ok(())
}
