extern crate kiss3d;

use kiss3d::camera::Camera;
use kiss3d::context::Context;
use kiss3d::light::Light;
use kiss3d::nalgebra as na;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::renderer::Renderer;
use kiss3d::resource::{
    AllocationType, BufferType, Effect, GPUVec, ShaderAttribute, ShaderUniform,
};
use kiss3d::window::{State, Window};
use na::{Matrix4, Point3};
use num_traits::ToPrimitive;

use crate::pointcloud::*;

const VERTEX_SHADER_SRC: &'static str = "#version 100
    attribute vec3 position;
    attribute vec3 color;
    varying   vec3 Color;
    uniform   mat4 proj;
    uniform   mat4 view;
    void main() {
        gl_Position = proj * view * vec4(position, 1.0);
        Color = color;
    }";

const FRAGMENT_SHADER_SRC: &'static str = "#version 100
#ifdef GL_FRAGMENT_PRECISION_HIGH
   precision highp float;
#else
   precision mediump float;
#endif
    varying vec3 Color;
    void main() {
        gl_FragColor = vec4(Color, 1.0);
    }";

/// Structure which manages the display of long-living points.
struct PointCloudRenderer {
    shader: Effect,
    pos: ShaderAttribute<Point3<f32>>,
    color: ShaderAttribute<Point3<f32>>,
    proj: ShaderUniform<Matrix4<f32>>,
    view: ShaderUniform<Matrix4<f32>>,
    colored_points: GPUVec<Point3<f32>>,
    point_size: f32,
}

impl PointCloudRenderer {
    /// Creates a new points renderer.
    fn new(point_size: f32) -> PointCloudRenderer {
        let mut shader = Effect::new_from_str(VERTEX_SHADER_SRC, FRAGMENT_SHADER_SRC);

        shader.use_program();

        PointCloudRenderer {
            colored_points: GPUVec::new(Vec::new(), BufferType::Array, AllocationType::StreamDraw),
            pos: shader.get_attrib::<Point3<f32>>("position").unwrap(),
            color: shader.get_attrib::<Point3<f32>>("color").unwrap(),
            proj: shader.get_uniform::<Matrix4<f32>>("proj").unwrap(),
            view: shader.get_uniform::<Matrix4<f32>>("view").unwrap(),
            shader,
            point_size,
        }
    }

    fn add_points<T>(&mut self, pointcloud: &PointCloud<T>)
    where
        T: Point,
        <T as Point>::Item: ToPrimitive,
    {
        if let Some(colored_points) = self.colored_points.data_mut() {
            for data in pointcloud.data.iter() {
                let point = data.xyz();
                colored_points.push(Point3::<f32>::new(
                    point[0].to_f32().unwrap(),
                    point[1].to_f32().unwrap(),
                    point[2].to_f32().unwrap(),
                ));
                colored_points.push(Point3::<f32>::new(1.0, 1.0, 1.0));
            }
        }
    }

    fn add_points_colors<T>(&mut self, pointcloud: &PointCloud<T>)
    where
        T: Point + Color,
        <T as Point>::Item: ToPrimitive,
        <T as Color>::Item: ToPrimitive,
    {
        if let Some(colored_points) = self.colored_points.data_mut() {
            for data in pointcloud.data.iter() {
                let point = data.xyz();
                let color = data.rgb();
                colored_points.push(Point3::<f32>::new(
                    point[0].to_f32().unwrap(),
                    point[1].to_f32().unwrap(),
                    point[2].to_f32().unwrap(),
                ));
                colored_points.push(Point3::<f32>::new(
                    color[0].to_f32().unwrap(),
                    color[1].to_f32().unwrap(),
                    color[2].to_f32().unwrap(),
                ));
            }
        }
    }
}

impl Renderer for PointCloudRenderer {
    /// Actually draws the points.
    fn render(&mut self, pass: usize, camera: &mut dyn Camera) {
        if self.colored_points.len() == 0 {
            return;
        }

        self.shader.use_program();
        self.pos.enable();
        self.color.enable();

        camera.upload(pass, &mut self.proj, &mut self.view);

        self.color.bind_sub_buffer(&mut self.colored_points, 1, 1);
        self.pos.bind_sub_buffer(&mut self.colored_points, 1, 0);

        let ctxt = Context::get();
        ctxt.point_size(self.point_size);
        ctxt.draw_arrays(Context::POINTS, 0, (self.colored_points.len() / 2) as i32);

        self.pos.disable();
        self.color.disable();
    }
}

struct AppState {
    point_cloud_renderer: PointCloudRenderer,
}

impl State for AppState {
    // Return the custom renderer that will be called at each
    // render loop.
    fn cameras_and_effect_and_renderer(
        &mut self,
    ) -> (
        Option<&mut dyn Camera>,
        Option<&mut dyn PlanarCamera>,
        Option<&mut dyn Renderer>,
        Option<&mut dyn PostProcessingEffect>,
    ) {
        (None, None, Some(&mut self.point_cloud_renderer), None)
    }

    fn step(&mut self, _window: &mut Window) {}
}
pub struct Visualizer {
    window: Window,
    app: AppState,
}

impl Visualizer {
    pub fn new() -> Visualizer {
        let mut window = Window::new("Siskin");
        window.set_background_color(0.0, 0.0, 0.0);
        window.set_light(Light::StickToCamera);
        let app = AppState {
            point_cloud_renderer: PointCloudRenderer::new(4.0),
        };
        Visualizer {
            window: window,
            app: app,
        }
    }
    pub fn add_pointcloud<T>(&mut self, pc: &PointCloud<T>)
    where
        T: Point,
        <T as Point>::Item: ToPrimitive,
    {
        self.app.point_cloud_renderer.add_points(pc);
    }
    pub fn add_colored_pointcloud<T>(&mut self, pc: &PointCloud<T>)
    where
        T: Point + Color,
        <T as Point>::Item: ToPrimitive,
        <T as Color>::Item: ToPrimitive,
    {
        self.app.point_cloud_renderer.add_points_colors(pc);
    }
    pub fn spin(self) {
        self.window.render_loop(self.app)
    }
}
