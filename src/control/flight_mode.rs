use crate::math::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone)]
pub struct ControlOutputs {
    pub thrust_vtol: Vec<f64>,
    pub thrust_cruise: f64,
    pub aileron: f64,
    pub elevator: f64,
    pub rudder: f64,
}

impl ControlOutputs {
    pub fn zero(n_vtol_motors: usize) -> Self {
        Self {
            thrust_vtol: vec![0.0; n_vtol_motors],
            thrust_cruise: 0.0,
            aileron: 0.0,
            elevator: 0.0,
            rudder: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum FlightMode {
    Stabilize,
    AltHold,
    Hover,
    Loiter,
    Auto,
    Landing,
    Takeoff,
    Transition,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlTarget {
    pub mode: FlightMode,
    pub position: Option<Vec3>,
    pub velocity: Option<Vec3>,
    pub attitude: Option<(f64, f64, f64)>,
    pub thrust: Option<f64>,
    pub altitude: Option<f64>,
    pub yaw: Option<f64>,
}

impl ControlTarget {
    pub fn hover_at_altitude(altitude: f64) -> Self {
        Self {
            mode: FlightMode::Hover,
            position: None,
            velocity: None,
            attitude: Some((0.0, 0.0, 0.0)),
            thrust: None,
            altitude: Some(altitude),
            yaw: Some(0.0),
        }
    }

    pub fn takeoff_to_altitude(target_altitude: f64, climb_rate: f64) -> Self {
        Self {
            mode: FlightMode::Takeoff,
            position: None,
            velocity: Some(Vec3::new(0.0, 0.0, -climb_rate)), // Negative for up in NED
            attitude: Some((0.0, 0.0, 0.0)),
            thrust: None,
            altitude: Some(target_altitude),
            yaw: Some(0.0),
        }
    }
}