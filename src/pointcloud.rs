extern crate nalgebra as na;
use nalgebra::base::Scalar;
use nalgebra::{ClosedAdd, Matrix3, Matrix4, RealField, Vector3, U1, U3};
use num_traits::{Float, NumAssign, Zero};
use std::any::Any;
use std::fmt::Debug;
use std::marker::PhantomData;
use std::ops::Add;

pub trait FloatData: Float + Debug + Any {}
impl<T: Float + Debug + Any> FloatData for T {}

pub trait Point {
    type Item: FloatData;
    fn from_point(point: Vector3<Self::Item>) -> Self;
    fn xyz(&self) -> &Vector3<Self::Item>;
    fn xyz_mut(&mut self) -> &mut Vector3<Self::Item>;
}

pub trait Color {
    type Item: Scalar;
    fn rgb(&self) -> &Vector3<Self::Item>;
    fn rgb_mut(&mut self) -> &mut Vector3<Self::Item>;
}

pub trait Normal {
    type Item: FloatData;
    fn normal(&self) -> &Vector3<Self::Item>;
    fn normal_mut(&mut self) -> &mut Vector3<Self::Item>;
}

pub trait PointColor: Point + Color {
    fn from_point_color(
        point: Vector3<<Self as Point>::Item>,
        color: Vector3<<Self as Color>::Item>,
    ) -> Self;
}

pub trait PointNormal: Point + Normal {
    fn from_point_normal(
        point: Vector3<<Self as Point>::Item>,
        normal: Vector3<<Self as Normal>::Item>,
    ) -> Self;
}

pub trait PointColorNormal: Point + Color + Normal {
    fn from_point_color_normal(
        point: Vector3<<Self as Point>::Item>,
        color: Vector3<<Self as Color>::Item>,
        normal: Vector3<<Self as Normal>::Item>,
    ) -> Self;
}

#[derive(Default, Debug)]
pub struct PointXYZ<T: FloatData> {
    pub point: Vector3<T>,
}

impl<T> Point for PointXYZ<T>
where
    T: FloatData,
{
    type Item = T;
    fn from_point(point: Vector3<T>) -> PointXYZ<T> {
        PointXYZ { point: point }
    }
    fn xyz(&self) -> &Vector3<T> {
        &self.point
    }
    fn xyz_mut(&mut self) -> &mut Vector3<T> {
        &mut self.point
    }
}

impl<T> Add for PointXYZ<T>
where
    T: FloatData + RealField,
{
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        Self::from_point(self.xyz() + other.xyz())
    }
}

#[derive(Default, Debug)]
pub struct PointXYZNormal<T: FloatData, U: FloatData> {
    pub point: Vector3<T>,
    pub normal: Vector3<U>,
}

impl<T, U> Point for PointXYZNormal<T, U>
where
    T: FloatData,
    U: FloatData + Zero + ClosedAdd,
{
    type Item = T;
    fn from_point(point: Vector3<T>) -> PointXYZNormal<T, U> {
        PointXYZNormal {
            point: point,
            normal: na::zero(),
        }
    }
    fn xyz(&self) -> &Vector3<T> {
        &self.point
    }
    fn xyz_mut(&mut self) -> &mut Vector3<T> {
        &mut self.point
    }
}

impl<T, U> Normal for PointXYZNormal<T, U>
where
    T: FloatData,
    U: FloatData,
{
    type Item = U;
    fn normal(&self) -> &Vector3<U> {
        &self.normal
    }
    fn normal_mut(&mut self) -> &mut Vector3<U> {
        &mut self.normal
    }
}

impl<T, U> PointNormal for PointXYZNormal<T, U>
where
    T: FloatData,
    U: FloatData + ClosedAdd,
{
    fn from_point_normal(point: Vector3<T>, normal: Vector3<U>) -> PointXYZNormal<T, U> {
        PointXYZNormal {
            point: point,
            normal: normal,
        }
    }
}

impl<T, U> Add for PointXYZNormal<T, U>
where
    T: FloatData + RealField,
    U: FloatData + RealField,
{
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        Self::from_point_normal(self.xyz() + other.xyz(), self.normal() + other.normal())
    }
}

#[derive(Default, Debug)]
pub struct PointXYZRGB<T: FloatData, U: Scalar> {
    pub point: Vector3<T>,
    pub color: Vector3<U>,
}

impl<T, U> Point for PointXYZRGB<T, U>
where
    T: FloatData,
    U: Scalar + Zero + ClosedAdd,
{
    type Item = T;
    fn from_point(point: Vector3<T>) -> PointXYZRGB<T, U> {
        PointXYZRGB {
            point: point,
            color: na::zero(),
        }
    }
    fn xyz(&self) -> &Vector3<T> {
        &self.point
    }
    fn xyz_mut(&mut self) -> &mut Vector3<T> {
        &mut self.point
    }
}

impl<T, U> Color for PointXYZRGB<T, U>
where
    T: FloatData,
    U: Scalar,
{
    type Item = U;
    fn rgb(&self) -> &Vector3<U> {
        &self.color
    }
    fn rgb_mut(&mut self) -> &mut Vector3<U> {
        &mut self.color
    }
}

impl<T, U> PointColor for PointXYZRGB<T, U>
where
    T: FloatData,
    U: Scalar + Zero + ClosedAdd,
{
    fn from_point_color(point: Vector3<T>, color: Vector3<U>) -> PointXYZRGB<T, U> {
        PointXYZRGB {
            point: point,
            color: color,
        }
    }
}

impl<T, U> Add for PointXYZRGB<T, U>
where
    T: FloatData + RealField,
    U: FloatData + RealField,
{
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        Self::from_point_color(self.xyz() + other.xyz(), self.rgb() + other.rgb())
    }
}

#[derive(Default, Debug)]
pub struct PointXYZRGBNormal<T: FloatData, U: Scalar, V: FloatData> {
    pub point: Vector3<T>,
    pub color: Vector3<U>,
    pub normal: Vector3<V>,
}

impl<T, U, V> Point for PointXYZRGBNormal<T, U, V>
where
    T: FloatData,
    U: Scalar + Zero + ClosedAdd,
    V: FloatData + Zero + ClosedAdd,
{
    type Item = T;
    fn from_point(point: Vector3<T>) -> PointXYZRGBNormal<T, U, V> {
        PointXYZRGBNormal {
            point: point,
            color: na::zero(),
            normal: na::zero(),
        }
    }
    fn xyz(&self) -> &Vector3<T> {
        &self.point
    }
    fn xyz_mut(&mut self) -> &mut Vector3<T> {
        &mut self.point
    }
}

impl<T, U, V> Color for PointXYZRGBNormal<T, U, V>
where
    T: FloatData,
    U: Scalar,
    V: FloatData,
{
    type Item = U;
    fn rgb(&self) -> &Vector3<U> {
        &self.color
    }
    fn rgb_mut(&mut self) -> &mut Vector3<U> {
        &mut self.color
    }
}

impl<T, U, V> Normal for PointXYZRGBNormal<T, U, V>
where
    T: FloatData,
    U: Scalar,
    V: FloatData,
{
    type Item = V;
    fn normal(&self) -> &Vector3<V> {
        &self.normal
    }
    fn normal_mut(&mut self) -> &mut Vector3<V> {
        &mut self.normal
    }
}

impl<T, U, V> Add for PointXYZRGBNormal<T, U, V>
where
    T: FloatData + RealField,
    U: FloatData + RealField,
    V: FloatData + RealField,
{
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        Self {
            point: self.xyz() + other.xyz(),
            color: self.rgb() + other.rgb(),
            normal: self.normal() + other.normal(),
        }
    }
}

pub struct PointCloud<T>
where
    T: Point,
{
    pub data: Vec<T>,
    pub width: u32,
    pub _marker: PhantomData<fn() -> T>,
}

impl<T> PointCloud<T>
where
    T: Point + Default,
    <T as Point>::Item: FloatData + NumAssign,
{
    pub fn new() -> PointCloud<T> {
        PointCloud {
            data: Vec::<T>::new(),
            width: 1,
            _marker: PhantomData,
        }
    }
    pub fn from_point_vec(data: Vec<Vector3<T::Item>>) -> PointCloud<T> {
        let points = data.into_iter().map(|d| T::from_point(d)).collect();
        PointCloud {
            data: points,
            width: 1,
            _marker: PhantomData,
        }
    }
    pub fn len(&self) -> usize {
        self.data.len()
    }
    pub fn item(&self, index: usize) -> &T {
        &self.data[index]
    }
    pub fn item_mut(&mut self, index: usize) -> &mut T {
        &mut self.data[index]
    }
    pub fn resize(&mut self, size: usize) {
        self.width = 1;
        self.data.resize_with(size, Default::default);
    }
    pub fn add_data(&mut self, element: T) {
        self.data.push(element);
    }
    pub fn transform(&self, trans: &Matrix4<<T as Point>::Item>) -> PointCloud<T> {
        let rot = trans.fixed_slice::<U3, U3>(0, 0);
        let t = trans.fixed_slice::<U3, U1>(0, 3);
        let mut transformed_pc = PointCloud::<T>::new();
        transformed_pc.data = self
            .data
            .iter()
            .map(|p| T::from_point(rot * p.xyz() + t))
            .collect();
        transformed_pc.width = self.width;
        transformed_pc
    }
    pub fn rotation(&self, rot: &Matrix3<<T as Point>::Item>) -> PointCloud<T> {
        let mut rotated_pc = PointCloud::<T>::new();
        rotated_pc.data = self
            .data
            .iter()
            .map(|p| T::from_point(rot * p.xyz()))
            .collect();
        rotated_pc.width = self.width;
        rotated_pc
    }
    pub fn translate(&self, t: &Vector3<<T as Point>::Item>) -> PointCloud<T> {
        let mut translated_pc = PointCloud::<T>::new();
        translated_pc.data = self
            .data
            .iter()
            .map(|p| T::from_point(p.xyz() + t))
            .collect();
        translated_pc.width = self.width;
        translated_pc
    }
}

impl<T> PointCloud<T>
where
    T: PointColor + Default,
    <T as Point>::Item: FloatData,
    <T as Color>::Item: Scalar,
{
    pub fn from_point_color_vec(
        data: Vec<Vector3<<T as Point>::Item>>,
        colors: Vec<Vector3<<T as Color>::Item>>,
    ) -> PointCloud<T> {
        let points = data
            .into_iter()
            .zip(colors.into_iter())
            .map(|(d, c)| T::from_point_color(d, c))
            .collect();
        PointCloud {
            data: points,
            width: 1,
            _marker: PhantomData,
        }
    }
}

pub type PointCloudXYZ<T> = PointCloud<PointXYZ<T>>;
pub type PointCloudXYZRGB<T> = PointCloud<PointXYZRGB<T, T>>;
pub type PointCloudXYZRGBNormal<T> = PointCloud<PointXYZRGBNormal<T, T, T>>;
