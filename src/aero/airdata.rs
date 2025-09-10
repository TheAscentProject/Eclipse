use crate::math::Vec3;

#[derive(Debug, Clone)]
pub struct AirData {
    pub velocity_body: Vec3,
    pub velocity_mag: f64,
    pub alpha: f64,
    pub beta: f64,
    pub dynamic_pressure: f64,
}

impl AirData {
    pub fn from_body_velocity(velocity_body: &Vec3, density: f64) -> Self {
        let u = velocity_body.x;
        let v = velocity_body.y;
        let w = velocity_body.z;
        
        let velocity_mag = velocity_body.magnitude();
        
        let alpha = if velocity_mag > 1e-6 {
            w.atan2(u)
        } else {
            0.0
        };
        
        let beta = if velocity_mag > 1e-6 {
            (v / velocity_mag).asin()
        } else {
            0.0
        };
        
        let dynamic_pressure = 0.5 * density * velocity_mag * velocity_mag;
        
        Self {
            velocity_body: *velocity_body,
            velocity_mag,
            alpha,
            beta,
            dynamic_pressure,
        }
    }
    
    pub fn velocity_wind(&self) -> Vec3 {
        Vec3::new(self.velocity_mag, 0.0, 0.0)
    }
}