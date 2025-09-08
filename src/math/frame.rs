use super::{Quat, Vec3};

#[derive(Debug, Clone, Copy)]
pub enum Frame {
    Inertial,
    Body,
    Wind,
    Stability,
}

pub struct Transform {
    pub position: Vec3,
    pub orientation: Quat,
}

impl Transform {
    pub fn new(position: Vec3, orientation: Quat) -> Self {
        Self {
            position,
            orientation,
        }
    }

    pub fn identity() -> Self {
        Self {
            position: Vec3::zero(),
            orientation: Quat::identity(),
        }
    }

    pub fn transform_point(&self, point: &Vec3) -> Vec3 {
        self.orientation.rotate_vector(point) + self.position
    }

    pub fn transform_vector(&self, vector: &Vec3) -> Vec3 {
        self.orientation.rotate_vector(vector)
    }

    pub fn inverse(&self) -> Self {
        let inv_orientation = self.orientation.conjugate();
        let inv_position = inv_orientation.rotate_vector(&self.position) * -1.0;
        Self::new(inv_position, inv_orientation)
    }
}