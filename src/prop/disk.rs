use crate::math::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiskPropeller {
    pub diameter: f64,
    pub area: f64,
    pub figure_of_merit: f64,
    pub max_thrust: f64,
    pub thrust_rate_limit: f64,
    pub current_thrust: f64,
}

impl DiskPropeller {
    pub fn new(diameter: f64, max_thrust: f64) -> Self {
        let area = std::f64::consts::PI * (diameter / 2.0).powi(2);
        Self {
            diameter,
            area,
            figure_of_merit: 0.75,
            max_thrust,
            thrust_rate_limit: max_thrust / 0.5,
            current_thrust: 0.0,
        }
    }
    
    pub fn compute_thrust(&mut self, throttle: f64, dt: f64) -> f64 {
        let throttle_clamped = throttle.clamp(0.0, 1.0);
        let thrust_cmd = throttle_clamped * throttle_clamped * self.max_thrust;
        
        let thrust_delta = thrust_cmd - self.current_thrust;
        let max_delta = self.thrust_rate_limit * dt;
        
        if thrust_delta.abs() <= max_delta {
            self.current_thrust = thrust_cmd;
        } else {
            self.current_thrust += thrust_delta.signum() * max_delta;
        }
        
        self.current_thrust
    }
    
    pub fn compute_induced_velocity(&self, velocity: f64, density: f64) -> f64 {
        if self.current_thrust <= 0.0 {
            return 0.0;
        }
        
        if velocity.abs() < 1e-6 {
            (self.current_thrust / (2.0 * density * self.area)).sqrt()
        } else {
            let v_half = velocity / 2.0;
            let discriminant = v_half * v_half + self.current_thrust / (density * self.area);
            if discriminant > 0.0 {
                discriminant.sqrt() - v_half
            } else {
                0.0
            }
        }
    }
    
    pub fn compute_power(&self, velocity: f64, density: f64) -> f64 {
        let vi = self.compute_induced_velocity(velocity, density);
        self.current_thrust * (velocity + vi) / self.figure_of_merit
    }
    
    pub fn compute_hover_power(&self, density: f64) -> f64 {
        if self.current_thrust <= 0.0 {
            return 0.0;
        }
        let vi = (self.current_thrust / (2.0 * density * self.area)).sqrt();
        self.current_thrust * vi / self.figure_of_merit
    }
}