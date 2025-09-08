use crate::math::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Propeller {
    pub diameter: f64,
    pub n_blades: u32,
    pub pitch: f64,
    pub solidity: f64,
    pub ct_static: f64,
    pub cp_static: f64,
    pub figure_of_merit: f64,
    pub max_rpm: f64,
}

#[derive(Debug, Clone)]
pub struct PropellerState {
    pub rpm: f64,
    pub thrust: f64,
    pub torque: f64,
    pub power: f64,
    pub efficiency: f64,
    pub exit_velocity: f64,
}

impl Propeller {
    pub fn new(diameter: f64, n_blades: u32, pitch: f64) -> Self {
        let solidity = n_blades as f64 * 0.08 / std::f64::consts::PI;
        
        Self {
            diameter,
            n_blades,
            pitch,
            solidity,
            ct_static: 0.12,
            cp_static: 0.06,
            figure_of_merit: 0.75,
            max_rpm: 8000.0,
        }
    }

    pub fn compute_state(
        &self,
        rpm: f64,
        velocity: f64,
        density: f64,
    ) -> PropellerState {
        let n = rpm / 60.0;
        let advance_ratio = velocity / (n * self.diameter + 1e-10);
        
        let ct = self.compute_thrust_coefficient(advance_ratio);
        let cp = self.compute_power_coefficient(advance_ratio);
        
        let thrust = ct * density * n * n * self.diameter.powi(4);
        let power = cp * density * n.powi(3) * self.diameter.powi(5);
        let torque = power / (2.0 * std::f64::consts::PI * n);
        
        let efficiency = if power > 1e-6 {
            (thrust * velocity) / power
        } else if thrust > 1e-6 {
            self.figure_of_merit
        } else {
            0.0
        };
        
        let disk_area = std::f64::consts::PI * (self.diameter / 2.0).powi(2);
        let exit_velocity = velocity + (2.0 * thrust / (density * disk_area)).sqrt();
        
        PropellerState {
            rpm,
            thrust,
            torque,
            power,
            efficiency,
            exit_velocity,
        }
    }

    fn compute_thrust_coefficient(&self, j: f64) -> f64 {
        if j < 0.1 {
            self.ct_static * (1.0 - j * 2.0)
        } else {
            let pitch_angle = (self.pitch / (std::f64::consts::PI * self.diameter)).atan();
            let ct_ideal = 4.0 * pitch_angle * (1.0 - j / (self.pitch / self.diameter));
            ct_ideal.max(0.0) * self.figure_of_merit
        }
    }

    fn compute_power_coefficient(&self, j: f64) -> f64 {
        if j < 0.1 {
            self.cp_static
        } else {
            let ct = self.compute_thrust_coefficient(j);
            ct * j / (2.0 * std::f64::consts::PI * self.figure_of_merit) + 0.01
        }
    }

    pub fn compute_momentum_theory_hover(
        &self,
        thrust_required: f64,
        density: f64,
    ) -> PropellerState {
        let disk_area = std::f64::consts::PI * (self.diameter / 2.0).powi(2);
        
        let induced_velocity = (thrust_required / (2.0 * density * disk_area)).sqrt();
        let ideal_power = thrust_required * induced_velocity;
        let actual_power = ideal_power / self.figure_of_merit;
        
        let n = (thrust_required / (self.ct_static * density * self.diameter.powi(4))).powf(0.5);
        let rpm = n * 60.0;
        
        PropellerState {
            rpm: rpm.min(self.max_rpm),
            thrust: thrust_required,
            torque: actual_power / (2.0 * std::f64::consts::PI * n),
            power: actual_power,
            efficiency: self.figure_of_merit,
            exit_velocity: 2.0 * induced_velocity,
        }
    }
}