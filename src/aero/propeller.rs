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

    pub fn compute_thrust(
        &self,
        rpm: f64,
        velocity: f64,
        density: f64,
    ) -> f64 {
        let n = rpm / 60.0;
        let advance_ratio = velocity / (n * self.diameter + 1e-10);
        
        let ct = self.compute_thrust_coefficient(advance_ratio);
        
        ct * density * n * n * self.diameter.powi(4)
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

}