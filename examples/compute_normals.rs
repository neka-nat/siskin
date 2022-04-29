use anyhow::*;
use siskin::{read_pcd, visualization, PointXYZNormal};

pub fn main() -> Result<()> {
    let mut pcd = read_pcd::<PointXYZNormal<f32, f32>>("examples/data/bunny.pcd")?;
    pcd.compute_normals(0.01);
    let mut vis = visualization::Visualizer::new();
    vis.add_pointcloud(&pcd);
    vis.spin();
    Ok(())
}
