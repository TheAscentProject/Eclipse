use serde::{Deserialize, Serialize};

/// Gain scheduling system for different flight regimes
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GainSchedule {
    /// Hover gains (low airspeed, < 5 m/s)
    pub hover: PidGains,
    /// Transition gains (5-20 m/s)
    pub transition: PidGains,
    /// Cruise gains (> 20 m/s)
    pub cruise: PidGains,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PidGains {
    pub roll: GainSet,
    pub pitch: GainSet,
    pub yaw: GainSet,
    pub altitude: GainSet,
    pub climb_rate: GainSet,
    pub position_x: GainSet,
    pub position_y: GainSet,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GainSet {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub max_output: f64,
    pub max_integral: f64,
}

impl GainSet {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            max_output: 1.0,
            max_integral: 0.5,
        }
    }
    
    pub fn with_limits(mut self, max_output: f64, max_integral: f64) -> Self {
        self.max_output = max_output;
        self.max_integral = max_integral;
        self
    }
}

impl Default for GainSchedule {
    fn default() -> Self {
        Self {
            hover: PidGains {
                // Hover: Moderate attitude control to prevent oscillation
                roll: GainSet::new(1.2, 0.02, 0.4).with_limits(0.5, 0.3),
                pitch: GainSet::new(1.2, 0.02, 0.4).with_limits(0.5, 0.3),
                yaw: GainSet::new(0.8, 0.01, 0.2).with_limits(0.3, 0.2),
                altitude: GainSet::new(0.6, 0.03, 0.2).with_limits(0.5, 0.3),
                climb_rate: GainSet::new(0.4, 0.05, 0.15).with_limits(0.5, 0.3),
                position_x: GainSet::new(0.4, 0.01, 0.15).with_limits(0.3, 0.2),
                position_y: GainSet::new(0.4, 0.01, 0.15).with_limits(0.3, 0.2),
            },
            transition: PidGains {
                // Transition: Balanced control, prepare for wing-borne flight
                roll: GainSet::new(1.5, 0.05, 0.5).with_limits(0.8, 0.4),
                pitch: GainSet::new(1.5, 0.05, 0.5).with_limits(0.8, 0.4),
                yaw: GainSet::new(1.0, 0.03, 0.2).with_limits(0.4, 0.2),
                altitude: GainSet::new(0.8, 0.05, 0.3).with_limits(0.8, 0.4),
                climb_rate: GainSet::new(0.6, 0.08, 0.15).with_limits(0.8, 0.4),
                position_x: GainSet::new(0.5, 0.01, 0.15).with_limits(0.4, 0.2),
                position_y: GainSet::new(0.5, 0.01, 0.15).with_limits(0.4, 0.2),
            },
            cruise: PidGains {
                // Cruise: Gentle control for efficiency, rely on aerodynamic surfaces
                roll: GainSet::new(0.8, 0.02, 0.3).with_limits(0.6, 0.3),
                pitch: GainSet::new(0.8, 0.02, 0.3).with_limits(0.6, 0.3),
                yaw: GainSet::new(0.6, 0.01, 0.15).with_limits(0.3, 0.15),
                altitude: GainSet::new(0.5, 0.03, 0.2).with_limits(0.6, 0.3),
                climb_rate: GainSet::new(0.4, 0.05, 0.1).with_limits(0.6, 0.3),
                position_x: GainSet::new(0.3, 0.005, 0.1).with_limits(0.3, 0.15),
                position_y: GainSet::new(0.3, 0.005, 0.1).with_limits(0.3, 0.15),
            },
        }
    }
}

impl GainSchedule {
    /// Interpolate gains based on airspeed
    pub fn get_gains(&self, airspeed: f64) -> PidGains {
        const HOVER_MAX: f64 = 5.0;
        const TRANSITION_START: f64 = 5.0;
        const TRANSITION_END: f64 = 20.0;
        
        if airspeed <= HOVER_MAX {
            self.hover.clone()
        } else if airspeed >= TRANSITION_END {
            self.cruise.clone()
        } else {
            // Smooth interpolation in transition region
            let blend = (airspeed - TRANSITION_START) / (TRANSITION_END - TRANSITION_START);
            self.interpolate_gains(&self.hover, &self.cruise, blend)
        }
    }
    
    fn interpolate_gains(&self, from: &PidGains, to: &PidGains, blend: f64) -> PidGains {
        PidGains {
            roll: self.interpolate_gain_set(&from.roll, &to.roll, blend),
            pitch: self.interpolate_gain_set(&from.pitch, &to.pitch, blend),
            yaw: self.interpolate_gain_set(&from.yaw, &to.yaw, blend),
            altitude: self.interpolate_gain_set(&from.altitude, &to.altitude, blend),
            climb_rate: self.interpolate_gain_set(&from.climb_rate, &to.climb_rate, blend),
            position_x: self.interpolate_gain_set(&from.position_x, &to.position_x, blend),
            position_y: self.interpolate_gain_set(&from.position_y, &to.position_y, blend),
        }
    }
    
    fn interpolate_gain_set(&self, from: &GainSet, to: &GainSet, blend: f64) -> GainSet {
        GainSet {
            kp: from.kp + (to.kp - from.kp) * blend,
            ki: from.ki + (to.ki - from.ki) * blend,
            kd: from.kd + (to.kd - from.kd) * blend,
            max_output: from.max_output + (to.max_output - from.max_output) * blend,
            max_integral: from.max_integral + (to.max_integral - from.max_integral) * blend,
        }
    }
}