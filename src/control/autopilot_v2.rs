use crate::control::{
    PidController, FlightMode, ControlTarget, 
    GainSchedule, ControlAllocator, TransitionManager
};
use crate::physics::RigidBodyState;
use crate::math::Vec3;

/// Improved autopilot with gain scheduling and control allocation
pub struct AutoPilotV2 {
    // Core PID controllers
    pub roll_pid: PidController,
    pub pitch_pid: PidController,
    pub yaw_pid: PidController,
    pub altitude_pid: PidController,
    pub climb_rate_pid: PidController,
    pub position_pid_x: PidController,
    pub position_pid_y: PidController,
    
    // Advanced systems
    pub gain_schedule: GainSchedule,
    pub control_allocator: ControlAllocator,
    pub transition_manager: TransitionManager,
    
    // Configuration
    pub n_vtol_motors: usize,
    pub n_cruise_motors: usize,
}

impl AutoPilotV2 {
    pub fn new(n_vtol_motors: usize, n_cruise_motors: usize) -> Self {
        let gain_schedule = GainSchedule::default();
        let initial_gains = gain_schedule.hover.clone();
        
        Self {
            // Initialize with hover gains
            roll_pid: PidController::new(
                initial_gains.roll.kp,
                initial_gains.roll.ki,
                initial_gains.roll.kd,
            ).with_limits(initial_gains.roll.max_output, initial_gains.roll.max_integral),
            
            pitch_pid: PidController::new(
                initial_gains.pitch.kp,
                initial_gains.pitch.ki,
                initial_gains.pitch.kd,
            ).with_limits(initial_gains.pitch.max_output, initial_gains.pitch.max_integral),
            
            yaw_pid: PidController::new(
                initial_gains.yaw.kp,
                initial_gains.yaw.ki,
                initial_gains.yaw.kd,
            ).with_limits(initial_gains.yaw.max_output, initial_gains.yaw.max_integral),
            
            altitude_pid: PidController::new(
                initial_gains.altitude.kp,
                initial_gains.altitude.ki,
                initial_gains.altitude.kd,
            ).with_limits(initial_gains.altitude.max_output, initial_gains.altitude.max_integral),
            
            climb_rate_pid: PidController::new(
                initial_gains.climb_rate.kp,
                initial_gains.climb_rate.ki,
                initial_gains.climb_rate.kd,
            ).with_limits(initial_gains.climb_rate.max_output, initial_gains.climb_rate.max_integral),
            
            position_pid_x: PidController::new(
                initial_gains.position_x.kp,
                initial_gains.position_x.ki,
                initial_gains.position_x.kd,
            ).with_limits(initial_gains.position_x.max_output, initial_gains.position_x.max_integral),
            
            position_pid_y: PidController::new(
                initial_gains.position_y.kp,
                initial_gains.position_y.ki,
                initial_gains.position_y.kd,
            ).with_limits(initial_gains.position_y.max_output, initial_gains.position_y.max_integral),
            
            gain_schedule,
            control_allocator: ControlAllocator::new(n_vtol_motors, n_cruise_motors),
            transition_manager: TransitionManager::new(),
            n_vtol_motors,
            n_cruise_motors,
        }
    }
    
    /// Main update function
    pub fn update(
        &mut self,
        state: &RigidBodyState,
        target: &ControlTarget,
        dt: f64,
        airspeed: f64,
    ) -> crate::control::ControlOutputs {
        // Update transition manager
        self.transition_manager.update(state, airspeed, dt);
        
        // Update gain scheduling based on airspeed
        self.update_gains(airspeed);
        
        // Get effective flight mode
        let effective_mode = self.transition_manager.get_effective_mode();
        
        // Calculate control demands
        let (roll_demand, pitch_demand, yaw_demand) = self.calculate_attitude_demands(state, target, dt);
        let (vertical_demand, forward_demand) = self.calculate_thrust_demands(
            state, target, dt, airspeed, effective_mode
        );
        
        // Get propulsion blend factors
        let (vtol_blend, cruise_blend) = self.transition_manager.get_propulsion_blend(airspeed);
        
        // Allocate controls using the allocation matrix
        let transition_blend = self.transition_manager.get_transition_blend(airspeed);
        let (vtol_commands, cruise_commands) = self.control_allocator.allocate(
            roll_demand,
            pitch_demand,
            yaw_demand,
            vertical_demand * vtol_blend,
            forward_demand * cruise_blend,
            transition_blend,
        );
        
        // Build output structure
        let mut outputs = crate::control::ControlOutputs::zero(self.n_vtol_motors);
        
        // Set VTOL motor commands
        for (i, &cmd) in vtol_commands.iter().enumerate() {
            if i < self.n_vtol_motors {
                outputs.thrust_vtol[i] = cmd;
            }
        }
        
        // Set cruise motor command (assuming single cruise motor)
        if !cruise_commands.is_empty() {
            outputs.thrust_cruise = cruise_commands[0];
        }
        
        // Set control surface deflections based on mode
        if airspeed > 10.0 {
            outputs.aileron = (roll_demand * 30.0).clamp(-30.0, 30.0);
            outputs.elevator = (pitch_demand * 30.0).clamp(-30.0, 30.0);
            outputs.rudder = (yaw_demand * 30.0).clamp(-30.0, 30.0);
        }
        
        outputs
    }
    
    /// Update PID gains based on airspeed
    fn update_gains(&mut self, airspeed: f64) {
        let gains = self.gain_schedule.get_gains(airspeed);
        
        // Update roll controller
        self.roll_pid.kp = gains.roll.kp;
        self.roll_pid.ki = gains.roll.ki;
        self.roll_pid.kd = gains.roll.kd;
        self.roll_pid.output_limit = gains.roll.max_output;
        self.roll_pid.integral_limit = gains.roll.max_integral;
        
        // Update pitch controller
        self.pitch_pid.kp = gains.pitch.kp;
        self.pitch_pid.ki = gains.pitch.ki;
        self.pitch_pid.kd = gains.pitch.kd;
        self.pitch_pid.output_limit = gains.pitch.max_output;
        self.pitch_pid.integral_limit = gains.pitch.max_integral;
        
        // Update yaw controller
        self.yaw_pid.kp = gains.yaw.kp;
        self.yaw_pid.ki = gains.yaw.ki;
        self.yaw_pid.kd = gains.yaw.kd;
        self.yaw_pid.output_limit = gains.yaw.max_output;
        self.yaw_pid.integral_limit = gains.yaw.max_integral;
        
        // Update altitude controller
        self.altitude_pid.kp = gains.altitude.kp;
        self.altitude_pid.ki = gains.altitude.ki;
        self.altitude_pid.kd = gains.altitude.kd;
        self.altitude_pid.output_limit = gains.altitude.max_output;
        self.altitude_pid.integral_limit = gains.altitude.max_integral;
        
        // Update climb rate controller
        self.climb_rate_pid.kp = gains.climb_rate.kp;
        self.climb_rate_pid.ki = gains.climb_rate.ki;
        self.climb_rate_pid.kd = gains.climb_rate.kd;
        self.climb_rate_pid.output_limit = gains.climb_rate.max_output;
        self.climb_rate_pid.integral_limit = gains.climb_rate.max_integral;
        
        // Update position controllers
        self.position_pid_x.kp = gains.position_x.kp;
        self.position_pid_x.ki = gains.position_x.ki;
        self.position_pid_x.kd = gains.position_x.kd;
        self.position_pid_x.output_limit = gains.position_x.max_output;
        self.position_pid_x.integral_limit = gains.position_x.max_integral;
        
        self.position_pid_y.kp = gains.position_y.kp;
        self.position_pid_y.ki = gains.position_y.ki;
        self.position_pid_y.kd = gains.position_y.kd;
        self.position_pid_y.output_limit = gains.position_y.max_output;
        self.position_pid_y.integral_limit = gains.position_y.max_integral;
    }
    
    /// Calculate attitude control demands
    fn calculate_attitude_demands(
        &mut self,
        state: &RigidBodyState,
        target: &ControlTarget,
        dt: f64,
    ) -> (f64, f64, f64) {
        let (roll, pitch, yaw) = state.orientation.to_euler();
        
        // Get target attitude
        let (target_roll, target_pitch, target_yaw) = if let Some(attitude) = target.attitude {
            attitude
        } else {
            (0.0, 0.0, 0.0)
        };
        
        // Position-based attitude adjustments
        if let Some(target_pos) = target.position {
            let pos_error_x = target_pos.x - state.position.x;
            let pos_error_y = target_pos.y - state.position.y;
            
            let roll_from_pos = -self.position_pid_y.update(pos_error_y, dt);
            let pitch_from_pos = self.position_pid_x.update(pos_error_x, dt);
            
            let final_target_roll = target_roll + roll_from_pos;
            let final_target_pitch = target_pitch + pitch_from_pos;
            
            (
                self.roll_pid.update(final_target_roll - roll, dt),
                self.pitch_pid.update(final_target_pitch - pitch, dt),
                self.yaw_pid.update(target_yaw - yaw, dt),
            )
        } else {
            (
                self.roll_pid.update(target_roll - roll, dt),
                self.pitch_pid.update(target_pitch - pitch, dt),
                self.yaw_pid.update(target_yaw - yaw, dt),
            )
        }
    }
    
    /// Calculate thrust demands
    fn calculate_thrust_demands(
        &mut self,
        state: &RigidBodyState,
        target: &ControlTarget,
        dt: f64,
        airspeed: f64,
        mode: FlightMode,
    ) -> (f64, f64) {
        let altitude = -state.position.z;
        let climb_rate = -state.velocity.z;
        
        match mode {
            FlightMode::Hover | FlightMode::AltHold => {
                // Vertical thrust for altitude hold - needs more base thrust
                let target_altitude = target.altitude.unwrap_or(altitude);
                let altitude_error = target_altitude - altitude;
                let vertical_demand = 1.2 + self.altitude_pid.update(altitude_error, dt);
                (vertical_demand.clamp(0.8, 2.0), 0.0)
            }
            FlightMode::Takeoff => {
                // Controlled climb - higher base thrust for reliable takeoff
                let target_climb_rate = target.velocity.map(|v| -v.z).unwrap_or(5.0);
                let climb_error = target_climb_rate - climb_rate;
                let vertical_demand = 1.3 + self.climb_rate_pid.update(climb_error, dt);
                (vertical_demand.clamp(1.0, 2.2), 0.0)
            }
            FlightMode::Transition => {
                // Blend vertical and forward thrust
                let target_altitude = target.altitude.unwrap_or(altitude);
                let altitude_error = target_altitude - altitude;
                let vertical_demand = 0.5 + self.altitude_pid.update(altitude_error, dt) * 0.5;
                let forward_demand = (airspeed / 20.0).clamp(0.0, 0.5);
                (vertical_demand, forward_demand)
            }
            FlightMode::Auto | FlightMode::Stabilize => {
                // Cruise flight - minimal vertical, controlled forward
                let target_altitude = target.altitude.unwrap_or(altitude);
                let altitude_error = target_altitude - altitude;
                let vertical_demand = self.altitude_pid.update(altitude_error, dt) * 0.3;
                let forward_demand = 0.4; // Cruise thrust
                (vertical_demand, forward_demand)
            }
            FlightMode::Landing => {
                // Controlled descent
                let target_descent_rate = -2.0; // 2 m/s descent
                let descent_error = target_descent_rate - climb_rate;
                let vertical_demand = 0.4 + self.climb_rate_pid.update(descent_error, dt);
                (vertical_demand.clamp(0.2, 0.8), 0.0)
            }
            _ => (0.5, 0.0), // Default safe values
        }
    }
}