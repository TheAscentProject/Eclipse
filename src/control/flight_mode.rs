use crate::math::Vec3;
use serde::{Deserialize, Serialize};

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
    pub fn new(mode: FlightMode) -> Self {
        Self {
            mode,
            position: None,
            velocity: None,
            attitude: None,
            thrust: None,
            altitude: None,
            yaw: None,
        }
    }

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

    pub fn stabilize_attitude(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self {
            mode: FlightMode::Stabilize,
            position: None,
            velocity: None,
            attitude: Some((roll, pitch, yaw)),
            thrust: None,
            altitude: None,
            yaw: Some(yaw),
        }
    }
}