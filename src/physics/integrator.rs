use crate::physics::rigid_body::{RigidBodyState, StateDerivative};
use crate::math::{Vec3, Quat};
use nalgebra as na;

pub trait Integrator {
    fn integrate(
        &self,
        state: &RigidBodyState,
        forces: &Vec3,
        moments: &Vec3,
        mass: f64,
        inertia: &na::Matrix3<f64>,
        dt: f64,
    ) -> RigidBodyState;
}

pub struct RungeKutta4;

impl RungeKutta4 {
    pub fn new() -> Self {
        Self
    }

    fn add_state_derivative(state: &RigidBodyState, deriv: &StateDerivative, dt: f64) -> RigidBodyState {
        let mut new_orientation = state.orientation + Quat::new(
            deriv.orientation_dot.w * dt,
            deriv.orientation_dot.x * dt,
            deriv.orientation_dot.y * dt,
            deriv.orientation_dot.z * dt,
        );
        new_orientation.renormalize();
        
        RigidBodyState {
            position: state.position + deriv.position_dot * dt,
            velocity: state.velocity + deriv.velocity * dt,
            orientation: new_orientation,
            angular_velocity: state.angular_velocity + deriv.angular_velocity_dot * dt,
        }
    }
}

impl Integrator for RungeKutta4 {
    fn integrate(
        &self,
        state: &RigidBodyState,
        forces: &Vec3,
        moments: &Vec3,
        mass: f64,
        inertia: &na::Matrix3<f64>,
        dt: f64,
    ) -> RigidBodyState {
        let k1 = state.derivative(forces, moments, mass, inertia);
        let state_k2 = Self::add_state_derivative(state, &k1, dt * 0.5);
        let k2 = state_k2.derivative(forces, moments, mass, inertia);
        
        let state_k3 = Self::add_state_derivative(state, &k2, dt * 0.5);
        let k3 = state_k3.derivative(forces, moments, mass, inertia);
        
        let state_k4 = Self::add_state_derivative(state, &k3, dt);
        let k4 = state_k4.derivative(forces, moments, mass, inertia);
        
        let final_derivative = StateDerivative {
            position_dot: (k1.position_dot + k2.position_dot * 2.0 + k3.position_dot * 2.0 + k4.position_dot) * (1.0 / 6.0),
            velocity: (k1.velocity + k2.velocity * 2.0 + k3.velocity * 2.0 + k4.velocity) * (1.0 / 6.0),
            orientation_dot: Quat::new(
                (k1.orientation_dot.w + k2.orientation_dot.w * 2.0 + k3.orientation_dot.w * 2.0 + k4.orientation_dot.w) / 6.0,
                (k1.orientation_dot.x + k2.orientation_dot.x * 2.0 + k3.orientation_dot.x * 2.0 + k4.orientation_dot.x) / 6.0,
                (k1.orientation_dot.y + k2.orientation_dot.y * 2.0 + k3.orientation_dot.y * 2.0 + k4.orientation_dot.y) / 6.0,
                (k1.orientation_dot.z + k2.orientation_dot.z * 2.0 + k3.orientation_dot.z * 2.0 + k4.orientation_dot.z) / 6.0,
            ),
            angular_velocity_dot: (k1.angular_velocity_dot + k2.angular_velocity_dot * 2.0 + k3.angular_velocity_dot * 2.0 + k4.angular_velocity_dot) * (1.0 / 6.0),
        };
        
        Self::add_state_derivative(state, &final_derivative, dt)
    }
}