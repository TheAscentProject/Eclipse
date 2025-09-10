use crate::math::{Vec3, Quat, GRAVITY};
use crate::physics::frames::FrameTransforms;
use nalgebra as na;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RigidBodyState {
    pub position: Vec3,
    pub velocity: Vec3,
    pub orientation: Quat,
    pub angular_velocity: Vec3,
}

impl RigidBodyState {
    pub fn new() -> Self {
        Self {
            position: Vec3::zero(),
            velocity: Vec3::zero(),
            orientation: Quat::identity(),
            angular_velocity: Vec3::zero(),
        }
    }

    pub fn derivative(&self, forces: &Vec3, moments: &Vec3, mass: f64, inertia: &na::Matrix3<f64>) -> StateDerivative {
        let gravity_inertial = Vec3::new(0.0, 0.0, GRAVITY * mass);
        let gravity_body = FrameTransforms::transform_vector_inertial_to_body(&gravity_inertial, &self.orientation);
        let total_force = *forces + gravity_body;
        
        let acceleration_body = total_force * (1.0 / mass);
        let acceleration_inertial = FrameTransforms::transform_vector_body_to_inertial(&acceleration_body, &self.orientation);
        
        let inertia_inv = inertia.try_inverse().unwrap_or(*inertia);
        let omega_body = self.angular_velocity;
        let omega_cross_I_omega = omega_body.cross(&Vec3::from_na(&(inertia.to_owned() * omega_body.to_na())));
        let angular_acceleration = Vec3::from_na(&(inertia_inv * (moments.to_na() - omega_cross_I_omega.to_na())));
        
        let q_dot = self.orientation.derivative(&omega_body);
        
        StateDerivative {
            velocity: acceleration_inertial,
            position_dot: self.velocity,
            orientation_dot: q_dot,
            angular_velocity_dot: angular_acceleration,
        }
    }
}

#[derive(Debug, Clone)]
pub struct StateDerivative {
    pub position_dot: Vec3,
    pub velocity: Vec3,
    pub orientation_dot: Quat,
    pub angular_velocity_dot: Vec3,
}

#[derive(Debug, Clone)]
pub struct RigidBody {
    pub mass: f64,
    pub inertia: na::Matrix3<f64>,
    pub cg_offset: Vec3,
}

impl RigidBody {
    pub fn new(mass: f64, inertia: na::Matrix3<f64>) -> Self {
        Self {
            mass,
            inertia,
            cg_offset: Vec3::zero(),
        }
    }

    pub fn point_mass(mass: f64) -> Self {
        let inertia = na::Matrix3::identity() * (mass * 0.1);
        Self::new(mass, inertia)
    }

    pub fn box_inertia(mass: f64, width: f64, height: f64, depth: f64) -> Self {
        let ixx = mass * (height * height + depth * depth) / 12.0;
        let iyy = mass * (width * width + depth * depth) / 12.0;
        let izz = mass * (width * width + height * height) / 12.0;
        
        let inertia = na::Matrix3::new(
            ixx, 0.0, 0.0,
            0.0, iyy, 0.0,
            0.0, 0.0, izz,
        );
        
        Self::new(mass, inertia)
    }

    pub fn compute_forces_and_moments(&self, state: &RigidBodyState, applied_forces: &[(Vec3, Vec3)]) -> (Vec3, Vec3) {
        let mut total_force = Vec3::zero();
        let mut total_moment = Vec3::zero();
        
        for (force, arm) in applied_forces {
            total_force = total_force + *force;
            
            let moment_arm = *arm - self.cg_offset;
            let moment = moment_arm.cross(force);
            total_moment = total_moment + moment;
        }
        
        (total_force, total_moment)
    }
}