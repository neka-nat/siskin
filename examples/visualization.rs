use nalgebra::Vector3;

use siskin::visualization;
use siskin::PointCloudXYZRGB;

fn main() {
    let points = vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)];
    let colors = vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)];
    let pc = PointCloudXYZRGB::from_point_color_vec(points, colors);
    let mut vis = visualization::Visualizer::new();
    vis.add_colored_pointcloud(&pc);
    vis.spin()
}
