use crate::math::Vec3;

#[derive(Debug, Clone)]
pub struct AirData {
    pub velocity_mag: f64,
    pub dynamic_pressure: f64,
}

impl AirData {
    pub fn from_body_velocity(velocity_body: &Vec3, density: f64) -> Self {
        let velocity_mag = velocity_body.magnitude();
        let dynamic_pressure = 0.5 * density * velocity_mag * velocity_mag;
        
        Self {
            velocity_mag,
            dynamic_pressure,
        }
    }
    
}