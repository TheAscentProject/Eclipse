use crate::math::{Vec3, Quat};

pub struct FrameTransforms;

impl FrameTransforms {
    pub fn transform_vector_body_to_inertial(v: &Vec3, q: &Quat) -> Vec3 {
        q.rotate_vector(v)
    }
    
    pub fn transform_vector_inertial_to_body(v: &Vec3, q: &Quat) -> Vec3 {
        q.conjugate().rotate_vector(v)
    }
}