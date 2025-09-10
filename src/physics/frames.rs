use crate::math::{Vec3, Quat};
use nalgebra as na;

pub struct FrameTransforms;

impl FrameTransforms {
    pub fn body_to_inertial(q: &Quat) -> na::Matrix3<f64> {
        q.to_rotation_matrix()
    }
    
    pub fn inertial_to_body(q: &Quat) -> na::Matrix3<f64> {
        q.to_rotation_matrix().transpose()
    }
    
    pub fn wind_to_body(alpha: f64, beta: f64) -> na::Matrix3<f64> {
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cb = beta.cos();
        let sb = beta.sin();
        
        na::Matrix3::new(
            ca * cb,  -ca * sb,  -sa,
            sb,       cb,        0.0,
            sa * cb,  -sa * sb,  ca,
        )
    }
    
    pub fn body_to_wind(alpha: f64, beta: f64) -> na::Matrix3<f64> {
        Self::wind_to_body(alpha, beta).transpose()
    }
    
    pub fn transform_vector_body_to_inertial(v: &Vec3, q: &Quat) -> Vec3 {
        q.rotate_vector(v)
    }
    
    pub fn transform_vector_inertial_to_body(v: &Vec3, q: &Quat) -> Vec3 {
        q.conjugate().rotate_vector(v)
    }
    
    pub fn transform_vector_wind_to_body(v: &Vec3, alpha: f64, beta: f64) -> Vec3 {
        let r = Self::wind_to_body(alpha, beta);
        Vec3::from_na(&(r * v.to_na()))
    }
    
    pub fn transform_vector_body_to_wind(v: &Vec3, alpha: f64, beta: f64) -> Vec3 {
        let r = Self::body_to_wind(alpha, beta);
        Vec3::from_na(&(r * v.to_na()))
    }
}