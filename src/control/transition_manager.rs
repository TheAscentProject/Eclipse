use crate::control::FlightMode;
use crate::physics::RigidBodyState;

/// Manages transitions between flight modes
pub struct TransitionManager {
    /// Current flight mode
    current_mode: FlightMode,
    /// Target flight mode
    target_mode: FlightMode,
    /// Transition progress (0.0 = start, 1.0 = complete)
    transition_progress: f64,
    /// Transition rate (progress per second)
    transition_rate: f64,
    /// Airspeed thresholds
    hover_max_airspeed: f64,
    transition_min_airspeed: f64,
    cruise_min_airspeed: f64,
    /// Altitude thresholds
    min_transition_altitude: f64,
}

impl TransitionManager {
    pub fn new() -> Self {
        Self {
            current_mode: FlightMode::Hover,
            target_mode: FlightMode::Hover,
            transition_progress: 1.0,
            transition_rate: 0.2, // 5 seconds for full transition
            hover_max_airspeed: 5.0,
            transition_min_airspeed: 8.0,
            cruise_min_airspeed: 20.0,
            min_transition_altitude: 10.0,
        }
    }
    
    /// Update transition state
    pub fn update(&mut self, state: &RigidBodyState, airspeed: f64, dt: f64) {
        // Update transition progress
        if self.current_mode != self.target_mode {
            self.transition_progress += self.transition_rate * dt;
            if self.transition_progress >= 1.0 {
                self.transition_progress = 1.0;
                self.current_mode = self.target_mode;
            }
        }
        
        // Auto-transition logic based on flight conditions
        let altitude = -state.position.z;
        
        match self.current_mode {
            FlightMode::Hover => {
                // Transition to forward flight if airspeed increases
                if airspeed > self.transition_min_airspeed && altitude > self.min_transition_altitude {
                    self.request_transition(FlightMode::Transition);
                }
            }
            FlightMode::Transition => {
                // Complete transition to cruise if airspeed is sufficient
                if airspeed > self.cruise_min_airspeed {
                    self.request_transition(FlightMode::Auto);
                }
                // Fall back to hover if airspeed drops
                else if airspeed < self.hover_max_airspeed {
                    self.request_transition(FlightMode::Hover);
                }
            }
            FlightMode::Auto | FlightMode::Stabilize => {
                // Transition back if airspeed drops
                if airspeed < self.cruise_min_airspeed {
                    self.request_transition(FlightMode::Transition);
                }
            }
            FlightMode::Landing => {
                // Automatic transition during landing
                if airspeed < self.transition_min_airspeed {
                    self.request_transition(FlightMode::Hover);
                }
            }
            _ => {}
        }
    }
    
    /// Request a mode transition
    pub fn request_transition(&mut self, new_mode: FlightMode) {
        if self.is_transition_allowed(self.current_mode, new_mode) {
            self.target_mode = new_mode;
            self.transition_progress = 0.0;
        }
    }
    
    /// Check if transition is allowed
    fn is_transition_allowed(&self, from: FlightMode, to: FlightMode) -> bool {
        match (from, to) {
            // Allow hover to/from transition
            (FlightMode::Hover, FlightMode::Transition) => true,
            (FlightMode::Transition, FlightMode::Hover) => true,
            // Allow transition to/from cruise modes
            (FlightMode::Transition, FlightMode::Auto) => true,
            (FlightMode::Transition, FlightMode::Stabilize) => true,
            (FlightMode::Auto, FlightMode::Transition) => true,
            (FlightMode::Stabilize, FlightMode::Transition) => true,
            // Allow takeoff/landing transitions
            (FlightMode::Takeoff, FlightMode::Hover) => true,
            (FlightMode::Hover, FlightMode::Landing) => true,
            // Allow altitude hold from hover
            (FlightMode::Hover, FlightMode::AltHold) => true,
            (FlightMode::AltHold, FlightMode::Hover) => true,
            // Same mode is always allowed
            _ if from == to => true,
            // Deny other transitions
            _ => false,
        }
    }
    
    /// Get current effective mode (considering transitions)
    pub fn get_effective_mode(&self) -> FlightMode {
        if self.transition_progress >= 1.0 {
            self.current_mode
        } else {
            // During transition, return transition mode
            FlightMode::Transition
        }
    }
    
    /// Get transition blend factor for control mixing
    pub fn get_transition_blend(&self, airspeed: f64) -> f64 {
        // Smooth blend based on airspeed
        let blend = ((airspeed - self.hover_max_airspeed) / 
                    (self.cruise_min_airspeed - self.hover_max_airspeed))
                    .clamp(0.0, 1.0);
        
        // Apply s-curve for smoother transition
        blend * blend * (3.0 - 2.0 * blend)
    }
    
    /// Get VTOL/cruise propulsion blend factors
    pub fn get_propulsion_blend(&self, airspeed: f64) -> (f64, f64) {
        let transition_blend = self.get_transition_blend(airspeed);
        
        match self.get_effective_mode() {
            FlightMode::Hover | FlightMode::AltHold => {
                (1.0, 0.0) // Full VTOL, no cruise
            }
            FlightMode::Transition => {
                // Gradual blend based on airspeed
                let vtol = 1.0 - transition_blend * 0.8; // Keep 20% VTOL minimum
                let cruise = transition_blend;
                (vtol, cruise)
            }
            FlightMode::Auto | FlightMode::Stabilize => {
                (0.1, 1.0) // Minimal VTOL for stability, full cruise
            }
            FlightMode::Takeoff => {
                // Use VTOL for takeoff, add cruise as speed builds
                let cruise = (airspeed / self.transition_min_airspeed).clamp(0.0, 0.3);
                (1.0, cruise)
            }
            FlightMode::Landing => {
                // Reduce cruise, increase VTOL as speed decreases
                let vtol = 1.0 - transition_blend * 0.5;
                let cruise = transition_blend * 0.5;
                (vtol, cruise)
            }
            _ => (0.5, 0.5) // Fallback balanced blend
        }
    }
}