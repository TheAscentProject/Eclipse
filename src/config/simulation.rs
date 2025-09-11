use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimulationConfig {
    pub dt: f64,
    pub max_time: f64,
    pub initial_altitude: f64,
    pub initial_position: [f64; 3],
    pub initial_velocity: [f64; 3],
    pub wind_velocity: [f64; 3],
    pub atmosphere_altitude: f64,
    pub gravity_enabled: bool,
    pub real_time: bool,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            dt: 0.01,
            max_time: 300.0,
            initial_altitude: 0.0,
            initial_position: [0.0, 0.0, 0.0],
            initial_velocity: [0.0, 0.0, 0.0],
            wind_velocity: [0.0, 0.0, 0.0],
            atmosphere_altitude: 0.0,
            gravity_enabled: true,
            real_time: false,
        }
    }
}

impl SimulationConfig {
    pub fn hover_test() -> Self {
        Self {
            dt: 0.005,
            max_time: 60.0,
            initial_altitude: 5.0,
            initial_position: [0.0, 0.0, -5.0],
            initial_velocity: [0.0, 0.0, 0.0],
            wind_velocity: [0.0, 0.0, 0.0],
            atmosphere_altitude: 0.0,
            gravity_enabled: true,
            real_time: true,
        }
    }

    pub fn transition_test() -> Self {
        Self {
            dt: 0.01,
            max_time: 120.0,
            initial_altitude: 10.0,
            initial_position: [0.0, 0.0, -10.0],
            initial_velocity: [0.0, 5.0, 0.0],
            wind_velocity: [2.0, 0.0, 0.0],
            atmosphere_altitude: 0.0,
            gravity_enabled: true,
            real_time: false,
        }
    }
    
    pub fn takeoff_test() -> Self {
        Self {
            dt: 0.01,
            max_time: 300.0,
            initial_altitude: 0.0,
            initial_position: [0.0, 0.0, 0.0],
            initial_velocity: [0.0, 0.0, 0.0],
            wind_velocity: [0.0, 0.0, 0.0],
            atmosphere_altitude: 0.0,
            gravity_enabled: true,
            real_time: false,
        }
    }
}