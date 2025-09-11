use crate::math::Vec3;
use crate::physics::RigidBodyState;
use rand::prelude::*;

pub struct DisturbanceModel {
    rng: StdRng,
    pub position_noise_std: f64,
    pub velocity_noise_std: f64,
    pub attitude_noise_std: f64,
    pub wind_base: Vec3,
    pub wind_gust_intensity: f64,
    pub wind_gust_frequency: f64,
    time: f64,
}

impl DisturbanceModel {
    pub fn new() -> Self {
        Self {
            rng: StdRng::seed_from_u64(42),
            position_noise_std: 0.02,
            velocity_noise_std: 0.05,
            attitude_noise_std: 0.001,
            wind_base: Vec3::new(0.0, 0.0, 0.0),
            wind_gust_intensity: 2.0,
            wind_gust_frequency: 0.5,
            time: 0.0,
        }
    }

    pub fn with_wind(mut self, base: Vec3, gust_intensity: f64) -> Self {
        self.wind_base = base;
        self.wind_gust_intensity = gust_intensity;
        self
    }

    pub fn update(&mut self, dt: f64) {
        self.time += dt;
    }

    pub fn add_sensor_noise(&mut self, state: &RigidBodyState) -> RigidBodyState {
        let mut noisy_state = state.clone();
        
        noisy_state.position.x += self.gaussian_noise() * self.position_noise_std;
        noisy_state.position.y += self.gaussian_noise() * self.position_noise_std;
        noisy_state.position.z += self.gaussian_noise() * self.position_noise_std;
        
        noisy_state.velocity.x += self.gaussian_noise() * self.velocity_noise_std;
        noisy_state.velocity.y += self.gaussian_noise() * self.velocity_noise_std;
        noisy_state.velocity.z += self.gaussian_noise() * self.velocity_noise_std;
        
        let (mut roll, mut pitch, mut yaw) = state.orientation.to_euler();
        roll += self.gaussian_noise() * self.attitude_noise_std;
        pitch += self.gaussian_noise() * self.attitude_noise_std;
        yaw += self.gaussian_noise() * self.attitude_noise_std;
        noisy_state.orientation = crate::math::Quat::from_euler(roll, pitch, yaw);
        
        noisy_state
    }

    pub fn get_wind_force(&mut self, altitude: f64) -> Vec3 {
        let gust_x = (self.time * self.wind_gust_frequency * 2.0 * std::f64::consts::PI).sin() 
                    + (self.time * self.wind_gust_frequency * 3.7).sin() * 0.5;
        let gust_y = (self.time * self.wind_gust_frequency * 1.8 * std::f64::consts::PI).sin()
                    + (self.time * self.wind_gust_frequency * 4.2).sin() * 0.3;
        let gust_z = (self.time * self.wind_gust_frequency * 2.5 * std::f64::consts::PI).sin() * 0.3;
        
        let altitude_factor = (altitude / 10.0).min(1.0).max(0.1);
        
        Vec3::new(
            (self.wind_base.x + gust_x * self.wind_gust_intensity) * altitude_factor,
            (self.wind_base.y + gust_y * self.wind_gust_intensity) * altitude_factor,
            (self.wind_base.z + gust_z * self.wind_gust_intensity * 0.5) * altitude_factor,
        )
    }

    fn gaussian_noise(&mut self) -> f64 {
        let u1: f64 = self.rng.gen();
        let u2: f64 = self.rng.gen();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

impl Default for DisturbanceModel {
    fn default() -> Self {
        Self::new()
    }
}