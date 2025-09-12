use crate::control::{PidController, FlightMode, ControlTarget, ControlOutputs};
use crate::physics::RigidBodyState;
use crate::math::Vec3;

/// Stable autopilot that handles all flight modes correctly
pub struct AutoPilotStable {
    // PID controllers
    pub roll_pid: PidController,
    pub pitch_pid: PidController,
    pub yaw_pid: PidController,
    pub altitude_pid: PidController,
    pub climb_rate_pid: PidController,
    pub position_pid_x: PidController,
    pub position_pid_y: PidController,
    
    // Configuration
    pub n_vtol_motors: usize,
    pub n_cruise_motors: usize,
    
    // Hover thrust calibration (1.0 = weight compensation)
    hover_thrust_baseline: f64,
}

impl AutoPilotStable {
    pub fn new(n_vtol_motors: usize, n_cruise_motors: usize) -> Self {
        Self {
            // Ultra-conservative gains for absolute stability
            roll_pid: PidController::new(0.05, 0.0, 0.02).with_limits(0.1, 0.05),
            pitch_pid: PidController::new(0.05, 0.0, 0.02).with_limits(0.1, 0.05),
            yaw_pid: PidController::new(0.02, 0.0, 0.01).with_limits(0.05, 0.02),
            altitude_pid: PidController::new(0.05, 0.001, 0.02).with_limits(0.1, 0.05),
            climb_rate_pid: PidController::new(0.1, 0.002, 0.03).with_limits(0.15, 0.05),
            position_pid_x: PidController::new(0.1, 0.002, 0.03).with_limits(0.1, 0.05),
            position_pid_y: PidController::new(0.1, 0.002, 0.03).with_limits(0.1, 0.05),
            
            n_vtol_motors,
            n_cruise_motors,
            hover_thrust_baseline: 1.0, // Exactly weight compensation
        }
    }
    
    pub fn update(
        &mut self,
        state: &RigidBodyState,
        target: &ControlTarget,
        dt: f64,
        airspeed: f64,
    ) -> ControlOutputs {
        let mut outputs = ControlOutputs::zero(self.n_vtol_motors);
        
        // Get current state
        let (roll, pitch, yaw) = state.orientation.to_euler();
        let altitude = -state.position.z;
        let climb_rate = -state.velocity.z;
        
        // Determine effective mode based on target
        let effective_mode = target.mode;
        
        match effective_mode {
            FlightMode::Hover | FlightMode::AltHold => {
                self.update_hover_mode(state, target, dt, &mut outputs, 
                                      roll, pitch, yaw, altitude);
            }
            FlightMode::Takeoff => {
                self.update_takeoff_mode(state, target, dt, &mut outputs,
                                        roll, pitch, yaw, altitude, climb_rate);
            }
            FlightMode::Transition => {
                self.update_transition_mode(state, target, dt, &mut outputs,
                                           roll, pitch, yaw, altitude, airspeed);
            }
            FlightMode::Auto | FlightMode::Stabilize => {
                self.update_cruise_mode(state, target, dt, &mut outputs,
                                       roll, pitch, yaw, altitude, airspeed);
            }
            FlightMode::Landing => {
                self.update_landing_mode(state, target, dt, &mut outputs,
                                        roll, pitch, yaw, altitude, climb_rate);
            }
            _ => {
                // Default safe hover
                self.set_safe_hover(&mut outputs);
            }
        }
        
        outputs
    }
    
    fn update_hover_mode(
        &mut self,
        state: &RigidBodyState,
        target: &ControlTarget,
        dt: f64,
        outputs: &mut ControlOutputs,
        roll: f64,
        pitch: f64,
        yaw: f64,
        altitude: f64,
    ) {
        // HOVER MODE: Need thrust = 1.0 to counteract weight
        let target_altitude = target.altitude.unwrap_or(altitude);
        let altitude_error = target_altitude - altitude;
        
        // Altitude control with climb rate damping
        let climb_rate = -state.velocity.z;
        let altitude_correction = self.altitude_pid.update(altitude_error, dt);
        let climb_damping = -climb_rate * 0.05; // Damp vertical velocity
        let base_thrust = self.hover_thrust_baseline + altitude_correction + climb_damping;
        
        // Strong velocity damping for horizontal stability
        let vel_pitch_correction = -state.velocity.x * 0.05;
        let vel_roll_correction = state.velocity.y * 0.05;
        
        // Attitude stabilization
        let target_roll = target.attitude.map(|a| a.0).unwrap_or(0.0) + vel_roll_correction;
        let target_pitch = target.attitude.map(|a| a.1).unwrap_or(0.0) + vel_pitch_correction;
        let target_yaw = target.attitude.map(|a| a.2).unwrap_or(0.0);
        
        // Position control if target position is specified
        if let Some(target_pos) = target.position {
            let pos_error_x = target_pos.x - state.position.x;
            let pos_error_y = target_pos.y - state.position.y;
            
            let pitch_from_pos = self.position_pid_x.update(pos_error_x, dt) * 0.1;
            let roll_from_pos = -self.position_pid_y.update(pos_error_y, dt) * 0.1;
            
            let roll_correction = self.roll_pid.update((target_roll + roll_from_pos) - roll, dt);
            let pitch_correction = self.pitch_pid.update((target_pitch + pitch_from_pos) - pitch, dt);
            let yaw_correction = self.yaw_pid.update(target_yaw - yaw, dt);
            
            // Mix controls for quadrotor
            self.mix_hover_controls(outputs, base_thrust, roll_correction, pitch_correction, yaw_correction);
        } else {
            // Just attitude stabilization
            let roll_correction = self.roll_pid.update(target_roll - roll, dt);
            let pitch_correction = self.pitch_pid.update(target_pitch - pitch, dt);
            let yaw_correction = self.yaw_pid.update(target_yaw - yaw, dt);
            
            self.mix_hover_controls(outputs, base_thrust, roll_correction, pitch_correction, yaw_correction);
        }
        
        // No cruise thrust in hover
        outputs.thrust_cruise = 0.0;
    }
    
    fn update_takeoff_mode(
        &mut self,
        _state: &RigidBodyState,
        target: &ControlTarget,
        dt: f64,
        outputs: &mut ControlOutputs,
        roll: f64,
        pitch: f64,
        yaw: f64,
        _altitude: f64,
        climb_rate: f64,
    ) {
        // TAKEOFF MODE: Need extra thrust to climb
        let target_climb_rate = target.velocity.map(|v| -v.z).unwrap_or(3.0);
        let climb_error = target_climb_rate - climb_rate;
        
        // Higher base thrust for climbing
        let climb_correction = self.climb_rate_pid.update(climb_error, dt);
        let base_thrust = 1.15 + climb_correction; // Start with 15% extra for climb
        
        // Strong attitude stabilization during takeoff
        let roll_correction = self.roll_pid.update(0.0 - roll, dt);
        let pitch_correction = self.pitch_pid.update(0.0 - pitch, dt);
        let yaw_correction = self.yaw_pid.update(0.0 - yaw, dt);
        
        self.mix_hover_controls(outputs, base_thrust, roll_correction, pitch_correction, yaw_correction);
        
        // No cruise thrust during vertical takeoff
        outputs.thrust_cruise = 0.0;
    }
    
    fn update_transition_mode(
        &mut self,
        _state: &RigidBodyState,
        target: &ControlTarget,
        dt: f64,
        outputs: &mut ControlOutputs,
        roll: f64,
        pitch: f64,
        yaw: f64,
        altitude: f64,
        airspeed: f64,
    ) {
        // TRANSITION MODE: Blend VTOL and cruise thrust
        let target_altitude = target.altitude.unwrap_or(altitude);
        let altitude_error = target_altitude - altitude;
        
        // Reduce VTOL thrust as airspeed increases
        let transition_factor = (airspeed / 20.0).clamp(0.0, 1.0);
        let altitude_correction = self.altitude_pid.update(altitude_error, dt);
        
        // VTOL thrust reduces with speed
        let vtol_thrust = (1.0 - transition_factor * 0.5) * (1.0 + altitude_correction);
        
        // Attitude control
        let roll_correction = self.roll_pid.update(0.0 - roll, dt);
        let pitch_correction = self.pitch_pid.update(0.0 - pitch, dt);
        let yaw_correction = self.yaw_pid.update(0.0 - yaw, dt);
        
        self.mix_hover_controls(outputs, vtol_thrust, roll_correction, pitch_correction, yaw_correction);
        
        // Cruise thrust increases with speed
        outputs.thrust_cruise = transition_factor * 0.3;
        
        // Start using control surfaces
        if airspeed > 10.0 {
            outputs.aileron = roll_correction * 15.0;
            outputs.elevator = pitch_correction * 15.0;
            outputs.rudder = yaw_correction * 10.0;
        }
    }
    
    fn update_cruise_mode(
        &mut self,
        _state: &RigidBodyState,
        target: &ControlTarget,
        dt: f64,
        outputs: &mut ControlOutputs,
        roll: f64,
        pitch: f64,
        yaw: f64,
        altitude: f64,
        _airspeed: f64,
    ) {
        // CRUISE MODE: Primarily forward thrust with minimal VTOL
        let target_altitude = target.altitude.unwrap_or(altitude);
        let altitude_error = target_altitude - altitude;
        
        // Minimal VTOL for altitude control
        let altitude_correction = self.altitude_pid.update(altitude_error, dt);
        let vtol_thrust = 0.2 + altitude_correction * 0.3;
        
        // Set minimal VTOL thrust
        for i in 0..self.n_vtol_motors {
            outputs.thrust_vtol[i] = vtol_thrust.clamp(0.0, 0.5);
        }
        
        // Primary cruise thrust
        outputs.thrust_cruise = 0.4;
        
        // Control surfaces for attitude
        let roll_correction = self.roll_pid.update(0.0 - roll, dt);
        let pitch_correction = self.pitch_pid.update(0.0 - pitch, dt);
        let yaw_correction = self.yaw_pid.update(0.0 - yaw, dt);
        
        outputs.aileron = roll_correction * 20.0;
        outputs.elevator = pitch_correction * 20.0;
        outputs.rudder = yaw_correction * 15.0;
    }
    
    fn update_landing_mode(
        &mut self,
        _state: &RigidBodyState,
        _target: &ControlTarget,
        dt: f64,
        outputs: &mut ControlOutputs,
        roll: f64,
        pitch: f64,
        yaw: f64,
        _altitude: f64,
        climb_rate: f64,
    ) {
        // LANDING MODE: Controlled descent
        let target_descent_rate = -1.5; // 1.5 m/s descent
        let descent_error = target_descent_rate - climb_rate;
        
        // Lower thrust for descent
        let descent_correction = self.climb_rate_pid.update(descent_error, dt);
        let base_thrust = 0.9 + descent_correction;
        
        // Attitude stabilization
        let roll_correction = self.roll_pid.update(0.0 - roll, dt);
        let pitch_correction = self.pitch_pid.update(0.0 - pitch, dt);
        let yaw_correction = self.yaw_pid.update(0.0 - yaw, dt);
        
        self.mix_hover_controls(outputs, base_thrust, roll_correction, pitch_correction, yaw_correction);
        
        // No cruise thrust during landing
        outputs.thrust_cruise = 0.0;
    }
    
    fn mix_hover_controls(
        &self,
        outputs: &mut ControlOutputs,
        throttle: f64,
        roll: f64,
        pitch: f64,
        yaw: f64,
    ) {
        if self.n_vtol_motors >= 4 {
            // Very gentle quadcopter mixing to prevent oscillations
            // Front-left  (+roll, +pitch, +yaw)
            outputs.thrust_vtol[0] = (throttle + roll * 0.05 + pitch * 0.05 + yaw * 0.02).clamp(0.0, 2.5);
            // Back-left   (-roll, +pitch, -yaw)  
            outputs.thrust_vtol[1] = (throttle - roll * 0.05 + pitch * 0.05 - yaw * 0.02).clamp(0.0, 2.5);
            // Back-right  (-roll, -pitch, +yaw)
            outputs.thrust_vtol[2] = (throttle - roll * 0.05 - pitch * 0.05 + yaw * 0.02).clamp(0.0, 2.5);
            // Front-right (+roll, -pitch, -yaw)
            outputs.thrust_vtol[3] = (throttle + roll * 0.05 - pitch * 0.05 - yaw * 0.02).clamp(0.0, 2.5);
            
            // Additional motors get base throttle
            for i in 4..self.n_vtol_motors {
                outputs.thrust_vtol[i] = throttle.clamp(0.0, 2.5);
            }
        } else {
            // Fallback for non-quad configurations
            for i in 0..self.n_vtol_motors {
                outputs.thrust_vtol[i] = throttle.clamp(0.0, 2.5);
            }
        }
    }
    
    fn set_safe_hover(&self, outputs: &mut ControlOutputs) {
        // Emergency safe hover at slightly above weight
        for i in 0..self.n_vtol_motors {
            outputs.thrust_vtol[i] = self.hover_thrust_baseline;
        }
        outputs.thrust_cruise = 0.0;
    }
}