use crate::control::{ControlOutputs, ControlTarget, FlightMode};
use crate::control::sysid::{SystemID, LinearModel, FlightRegime};
use crate::control::lqr_tuning::{LQRTuner, LQRDesign};
use crate::control::kalman::{ExtendedKalmanFilter, generate_sensor_measurements};
use crate::physics::RigidBodyState;
use crate::config::AircraftConfig;
use crate::math::Vec3;
use nalgebra::DVector;
use std::collections::HashMap;

/// Modern LQG autopilot with gain scheduling and state estimation
pub struct AutoPilotLQG {
    /// Kalman filter for state estimation
    estimator: ExtendedKalmanFilter,
    
    /// LQR designs for different flight regimes
    lqr_designs: HashMap<FlightRegime, LQRDesign>,
    
    /// System models for different regimes
    linear_models: HashMap<FlightRegime, LinearModel>,
    
    /// Current flight regime
    current_regime: FlightRegime,
    
    /// Loop counters and timing
    estimator_counter: u32,
    estimator_divider: u32,
    
    /// Configuration
    n_vtol_motors: usize,
    n_cruise_motors: usize,
    aircraft_config: Option<AircraftConfig>,
    
    /// Cached control commands
    cached_moments: Vec3,
    cached_thrust: f64,
    
    /// Manual takeoff controller state
    takeoff_altitude_integral: f64,
    takeoff_prev_altitude_error: f64,
}

impl AutoPilotLQG {
    pub fn new(n_vtol_motors: usize, n_cruise_motors: usize) -> Self {
        Self {
            estimator: ExtendedKalmanFilter::new(),
            lqr_designs: HashMap::new(),
            linear_models: HashMap::new(),
            current_regime: FlightRegime::Hover,
            estimator_counter: 0,
            estimator_divider: 5, // Run estimator at 100Hz if control is 500Hz
            n_vtol_motors,
            n_cruise_motors,
            aircraft_config: None,
            cached_moments: Vec3::zero(),
            cached_thrust: 1.0,
            takeoff_altitude_integral: 0.0,
            takeoff_prev_altitude_error: 0.0,
        }
    }
    
    /// Initialize with aircraft configuration and compute all LQR designs
    pub fn initialize(&mut self, config: AircraftConfig) {
        self.aircraft_config = Some(config.clone());
        
        // System identification for all regimes
        let sysid = SystemID::new();
        
        // Takeoff regime
        let takeoff_model = sysid.identify_takeoff(&config);
        let takeoff_design = self.design_lqr_controller(&takeoff_model);
        self.linear_models.insert(FlightRegime::Takeoff, takeoff_model);
        self.lqr_designs.insert(FlightRegime::Takeoff, takeoff_design);
        
        // Hover regime
        let hover_model = sysid.identify_hover(&config);
        let hover_design = self.design_lqr_controller(&hover_model);
        self.linear_models.insert(FlightRegime::Hover, hover_model);
        self.lqr_designs.insert(FlightRegime::Hover, hover_design);
        
        // Cruise regimes at different airspeeds
        for &airspeed in &[15.0, 25.0] {
            let cruise_model = sysid.identify_cruise(&config, airspeed);
            let cruise_design = self.design_lqr_controller(&cruise_model);
            self.linear_models.insert(FlightRegime::Cruise, cruise_model);
            self.lqr_designs.insert(FlightRegime::Cruise, cruise_design);
        }
        
        println!("LQG Autopilot initialized with {} flight regimes", self.lqr_designs.len());
        self.print_design_summary();
    }
    
    /// Design LQR controller for a linear model
    fn design_lqr_controller(&self, model: &LinearModel) -> LQRDesign {
        let tuner = LQRTuner::new();
        
        // Auto-tune for good stability margins
        let target_bandwidth = match model.regime {
            FlightRegime::Hover => 2.0,      // 2 rad/s bandwidth for hover
            FlightRegime::Takeoff => 3.0,    // Fast response for takeoff
            FlightRegime::Transition => 1.5,  // More conservative for transition
            FlightRegime::Cruise => 1.0,     // Gentle for cruise efficiency
        };
        
        let target_damping = 0.7; // 70% damping ratio
        
        let design = tuner.auto_tune(model, target_bandwidth, target_damping);
        
        println!("LQR Design for {:?}:", model.regime);
        println!("  Phase Margin: {:.1}°", design.phase_margin);
        println!("  Gain Margin: {:.1} dB", design.gain_margin);
        
        design
    }
    
    pub fn update(
        &mut self,
        true_state: &RigidBodyState, // For sensor simulation
        target: &ControlTarget,
        dt: f64,
        airspeed: f64,
    ) -> ControlOutputs {
        // Update flight regime based on conditions
        self.update_flight_regime(target.mode, airspeed);
        
        // Generate sensor measurements (in real system, these come from hardware)
        let measurements = generate_sensor_measurements(true_state, dt);
        
        // Run estimator at lower rate
        let estimated_state = if self.estimator_counter % self.estimator_divider == 0 {
            // Prediction step with current control input
            let control_input = DVector::from_vec(vec![
                self.cached_moments.x,
                self.cached_moments.y,
                self.cached_moments.z,
                self.cached_thrust,
            ]);
            
            self.estimator.predict(&control_input, dt * self.estimator_divider as f64);
            
            // Measurement update
            self.estimator.update(&measurements);
            
            self.estimator.get_state_estimate()
        } else {
            self.estimator.get_state_estimate()
        };
        
        self.estimator_counter += 1;
        
        // LQR control law
        self.compute_lqr_control(&estimated_state, target, dt, airspeed)
    }
    
    /// Update current flight regime based on conditions
    fn update_flight_regime(&mut self, mode: FlightMode, airspeed: f64) {
        let new_regime = match mode {
            FlightMode::Takeoff => FlightRegime::Takeoff,
            FlightMode::Hover | FlightMode::AltHold => FlightRegime::Hover,
            FlightMode::Transition => FlightRegime::Transition,
            FlightMode::Auto | FlightMode::Stabilize => {
                if airspeed > 20.0 {
                    FlightRegime::Cruise
                } else if airspeed > 8.0 {
                    FlightRegime::Transition
                } else {
                    FlightRegime::Hover
                }
            }
            _ => FlightRegime::Hover,
        };
        
        if new_regime as u8 != self.current_regime as u8 {
            println!("Flight regime transition: {:?} -> {:?}", self.current_regime, new_regime);
            self.current_regime = new_regime;
        }
    }
    
    /// Compute LQR control with gain scheduling
    fn compute_lqr_control(
        &mut self,
        state: &RigidBodyState,
        target: &ControlTarget,
        dt: f64,
        airspeed: f64,
    ) -> ControlOutputs {
        // Special case: manual takeoff controller since LQR solver is broken
        if target.mode == FlightMode::Takeoff {
            return self.compute_manual_takeoff_control(state, target, dt);
        }
        
        // Get LQR design for current regime
        let design = self.lqr_designs.get(&self.current_regime)
            .unwrap_or_else(|| self.lqr_designs.get(&FlightRegime::Hover).unwrap());
        
        // State error from target
        let state_error = self.compute_state_error(state, target);
        
        // LQR control law: u = -K * x_error
        let control_vec = -&design.k_gain * &state_error;
        
        // Cache commands
        self.cached_moments = Vec3::new(
            control_vec[0] * 0.01, // Scale down moments
            control_vec[1] * 0.01,
            control_vec[2] * 0.01,
        );
        
        // Thrust with baseline hover compensation - increased gain for takeoff
        self.cached_thrust = (1.0 + control_vec[3] * 1.0).clamp(0.2, 5.0); // Much higher gain and max thrust
        
        // Control allocation
        let desired_wrench = DVector::from_vec(vec![
            self.cached_moments.x,
            self.cached_moments.y,
            self.cached_moments.z,
            self.cached_thrust,
        ]);
        
        let motor_commands = self.simple_quad_mixing(&desired_wrench);
        
        // Build output
        let mut outputs = ControlOutputs::zero(self.n_vtol_motors);
        for i in 0..self.n_vtol_motors.min(motor_commands.len()) {
            outputs.thrust_vtol[i] = motor_commands[i];
        }
        
        // Cruise thrust for forward flight
        if airspeed > 8.0 {
            let cruise_factor = ((airspeed - 8.0) / 15.0).clamp(0.0, 1.0);
            outputs.thrust_cruise = cruise_factor * 0.4;
        }
        
        outputs
    }
    
    /// Compute state error vector for LQR
    fn compute_state_error(&self, state: &RigidBodyState, target: &ControlTarget) -> DVector<f64> {
        // Target state based on current target
        let target_pos = if let Some(pos) = target.position {
            pos
        } else if let Some(alt) = target.altitude {
            // For altitude-only targets (like takeoff), keep current X,Y but use target altitude
            Vec3::new(state.position.x, state.position.y, -alt) // NED: negative Z is up
        } else {
            state.position // Fallback to current position
        };
        
        let target_vel = target.velocity.unwrap_or(Vec3::zero());
        let (target_roll, target_pitch, target_yaw) = target.attitude.unwrap_or((0.0, 0.0, 0.0));
        
        // Current state
        let (roll, pitch, yaw) = state.orientation.to_euler();
        
        // State error vector [x, y, z, vx, vy, vz, φ, θ, ψ, p, q, r]
        DVector::from_vec(vec![
            state.position.x - target_pos.x,
            state.position.y - target_pos.y,
            state.position.z - target_pos.z,
            state.velocity.x - target_vel.x,
            state.velocity.y - target_vel.y,
            state.velocity.z - target_vel.z,
            self.angle_wrap(roll - target_roll),
            self.angle_wrap(pitch - target_pitch),
            self.angle_wrap(yaw - target_yaw),
            state.angular_velocity.x,  // Target rates are zero
            state.angular_velocity.y,
            state.angular_velocity.z,
        ])
    }
    
    /// Wrap angle to [-π, π]
    fn angle_wrap(&self, angle: f64) -> f64 {
        let mut wrapped = angle;
        while wrapped > std::f64::consts::PI {
            wrapped -= 2.0 * std::f64::consts::PI;
        }
        while wrapped < -std::f64::consts::PI {
            wrapped += 2.0 * std::f64::consts::PI;
        }
        wrapped
    }
    
    /// Print summary of all LQR designs
    fn print_design_summary(&self) {
        println!("\n=== LQG Autopilot Design Summary ===");
        for (regime, design) in &self.lqr_designs {
            println!("{:?} Regime:", regime);
            println!("  Phase Margin: {:.1}°", design.phase_margin);
            println!("  Gain Margin: {:.1} dB", design.gain_margin);
            println!("  Max Eigenvalue: {:.3}", 
                design.eigenvalues.iter().map(|x| x.abs()).fold(0.0, f64::max));
        }
        println!("=====================================\n");
    }
    
    /// Reset estimator
    pub fn reset(&mut self, initial_state: &RigidBodyState) {
        self.estimator.initialize(initial_state);
        self.estimator_counter = 0;
        self.takeoff_altitude_integral = 0.0;
        self.takeoff_prev_altitude_error = 0.0;
        println!("LQG Autopilot reset to initial state");
    }
    
    /// Simple quadrotor control mixing
    fn simple_quad_mixing(&self, wrench: &DVector<f64>) -> Vec<f64> {
        if wrench.len() < 4 {
            return vec![1.0; self.n_vtol_motors];
        }
        
        let mx = wrench[0] * 0.1; // Scale down moments
        let my = wrench[1] * 0.1;
        let mz = wrench[2] * 0.1;
        let thrust = wrench[3];
        
        // Simple X-configuration quadrotor mixing
        vec![
            (thrust + mx + my + mz).clamp(0.0, 2.0),  // Front-right
            (thrust - mx + my - mz).clamp(0.0, 2.0),  // Front-left
            (thrust - mx - my + mz).clamp(0.0, 2.0),  // Back-left
            (thrust + mx - my - mz).clamp(0.0, 2.0),  // Back-right
        ]
    }
    
    /// Manual takeoff controller (bypass broken LQR solver)
    fn compute_manual_takeoff_control(
        &mut self,
        state: &RigidBodyState,
        target: &ControlTarget,
        dt: f64,
    ) -> ControlOutputs {
        // Simple altitude control for takeoff
        let target_altitude = target.altitude.unwrap_or(120.0);
        let current_altitude = -state.position.z; // NED: negative Z is up
        let altitude_error = target_altitude - current_altitude;
        
        // PID controller for altitude with conservative gains
        let kp = 0.004;  // Even smaller proportional gain
        let ki = 0.0001; // Very small integral gain 
        let kd = 0.02;   // Smaller derivative gain
        
        // Update integral term
        self.takeoff_altitude_integral += altitude_error * dt;
        self.takeoff_altitude_integral = self.takeoff_altitude_integral.clamp(-50.0, 50.0); // Anti-windup
        
        // Derivative term
        let altitude_error_derivative = (altitude_error - self.takeoff_prev_altitude_error) / dt;
        self.takeoff_prev_altitude_error = altitude_error;
        
        // PID output
        let pid_output = kp * altitude_error + ki * self.takeoff_altitude_integral + kd * altitude_error_derivative;
        
        // Thrust command with smooth limits
        let thrust_command = (1.0 + pid_output).clamp(0.5, 2.0);
        
        // Attitude stabilization - very gentle
        let (roll, pitch, _yaw) = state.orientation.to_euler();
        let attitude_gain = 0.5;
        
        self.cached_moments = Vec3::new(
            -roll * attitude_gain,
            -pitch * attitude_gain,
            0.0,
        );
        
        self.cached_thrust = thrust_command;
        
        // Debug: Print manual takeoff control info
        if self.estimator_counter % 100 == 0 {
            println!("MANUAL Takeoff - Target: {:.1}m, Current: {:.1}m, Error: {:.1}m, PID: {:.3}, Thrust: {:.3}",
                target_altitude, current_altitude, altitude_error, pid_output, self.cached_thrust);
        }
        
        // Control allocation
        let desired_wrench = DVector::from_vec(vec![
            self.cached_moments.x,
            self.cached_moments.y,
            self.cached_moments.z,
            self.cached_thrust,
        ]);
        
        let motor_commands = self.simple_quad_mixing(&desired_wrench);
        
        // Build output
        let mut outputs = ControlOutputs::zero(self.n_vtol_motors);
        for i in 0..self.n_vtol_motors.min(motor_commands.len()) {
            outputs.thrust_vtol[i] = motor_commands[i];
        }
        
        outputs
    }

    /// Get current state estimate (for debugging)
    pub fn get_state_estimate(&self) -> RigidBodyState {
        self.estimator.get_state_estimate()
    }
}