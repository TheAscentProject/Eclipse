use crate::control::{ControlOutputs, ControlTarget, FlightMode};
use crate::control::lqr::{AttitudeLQR, PositionLQR};
use crate::control::qp_allocation::QPAllocator;
use crate::physics::RigidBodyState;
use crate::math::Vec3;
use nalgebra::DVector;

/// Modern LQR-based autopilot with proper stability margins
pub struct AutoPilotLQR {
    /// Inner loop: Attitude controller (500 Hz)
    attitude_lqr: AttitudeLQR,
    
    /// Outer loop: Position controller (100 Hz)
    position_lqr: PositionLQR,
    
    /// Control allocator
    allocator: QPAllocator,
    
    /// Configuration
    n_vtol_motors: usize,
    n_cruise_motors: usize,
    
    /// Loop timing
    inner_loop_counter: u32,
    outer_loop_divider: u32,
    
    /// Cached commands from outer loop
    cached_roll_cmd: f64,
    cached_pitch_cmd: f64,
    cached_thrust_cmd: f64,
    
    /// Baseline hover thrust (weight compensation)
    hover_thrust: f64,
}

impl AutoPilotLQR {
    pub fn new(n_vtol_motors: usize, n_cruise_motors: usize) -> Self {
        Self {
            attitude_lqr: AttitudeLQR::new(),
            position_lqr: PositionLQR::new(),
            allocator: QPAllocator::new_quadrotor(),
            n_vtol_motors,
            n_cruise_motors,
            inner_loop_counter: 0,
            outer_loop_divider: 5, // 500Hz inner / 100Hz outer
            cached_roll_cmd: 0.0,
            cached_pitch_cmd: 0.0,
            cached_thrust_cmd: 1.0,
            hover_thrust: 1.0, // Normalized for weight
        }
    }
    
    pub fn update(
        &mut self,
        state: &RigidBodyState,
        target: &ControlTarget,
        dt: f64,
        airspeed: f64,
    ) -> ControlOutputs {
        // Run outer loop at lower rate (100 Hz)
        if self.inner_loop_counter % self.outer_loop_divider == 0 {
            self.update_outer_loop(state, target, dt * self.outer_loop_divider as f64, airspeed);
        }
        self.inner_loop_counter += 1;
        
        // Always run inner loop (500 Hz)
        self.update_inner_loop(state, dt)
    }
    
    fn update_outer_loop(
        &mut self,
        state: &RigidBodyState,
        target: &ControlTarget,
        dt: f64,
        airspeed: f64,
    ) {
        // Determine target position and velocity based on mode
        let (pos_cmd, vel_cmd) = match target.mode {
            FlightMode::Hover | FlightMode::AltHold => {
                let pos = target.position.unwrap_or(state.position);
                let vel = Vec3::zero();
                (pos, vel)
            }
            FlightMode::Takeoff => {
                // Climb at specified rate
                let pos = Vec3::new(
                    state.position.x,
                    state.position.y,
                    target.altitude.unwrap_or(-400.0),
                );
                let vel = target.velocity.unwrap_or(Vec3::new(0.0, 0.0, -5.0));
                (pos, vel)
            }
            FlightMode::Transition | FlightMode::Auto => {
                // Track position with forward velocity
                let pos = target.position.unwrap_or(state.position);
                let vel = target.velocity.unwrap_or(Vec3::new(0.0, 15.0, 0.0));
                (pos, vel)
            }
            _ => (state.position, Vec3::zero()),
        };
        
        // Position LQR generates attitude commands and thrust adjustment
        let (roll_cmd, pitch_cmd, thrust_adj) = self.position_lqr.update(
            state,
            pos_cmd,
            vel_cmd,
            airspeed,
            dt,
        );
        
        // Cache commands for inner loop
        self.cached_roll_cmd = roll_cmd.clamp(-0.5, 0.5); // Limit to ±30 degrees
        self.cached_pitch_cmd = pitch_cmd.clamp(-0.5, 0.5);
        self.cached_thrust_cmd = (self.hover_thrust + thrust_adj).clamp(0.2, 2.0);
    }
    
    fn update_inner_loop(&mut self, state: &RigidBodyState, dt: f64) -> ControlOutputs {
        // Attitude LQR generates moment commands
        let moments = self.attitude_lqr.update(
            state,
            self.cached_roll_cmd,
            self.cached_pitch_cmd,
            0.0, // Yaw command
            dt,
        );
        
        // Create desired wrench [Mx, My, Mz, Fz]  
        // Scale thrust properly - cached_thrust_cmd is normalized thrust coefficient
        let desired_wrench = DVector::from_vec(vec![
            moments.x * 0.1,  // Scale moments down
            moments.y * 0.1,  
            moments.z * 0.1,
            self.cached_thrust_cmd, // This goes directly to each motor
        ]);
        
        // Allocate to motors using QP
        let motor_commands = self.allocator.allocate(&desired_wrench, dt);
        
        // Build output structure
        let mut outputs = ControlOutputs::zero(self.n_vtol_motors);
        
        // Set VTOL motor commands
        for i in 0..self.n_vtol_motors.min(motor_commands.len()) {
            outputs.thrust_vtol[i] = motor_commands[i];
        }
        
        // Cruise thrust for forward flight (simple logic for now)
        let airspeed = state.velocity.magnitude();
        if airspeed > 10.0 {
            outputs.thrust_cruise = ((airspeed - 10.0) / 20.0).clamp(0.0, 0.5);
        }
        
        outputs
    }
    
    pub fn reset(&mut self) {
        self.attitude_lqr.integral_roll = 0.0;
        self.attitude_lqr.integral_pitch = 0.0;
        self.attitude_lqr.integral_yaw = 0.0;
        self.position_lqr.integral_x = 0.0;
        self.position_lqr.integral_y = 0.0;
        self.position_lqr.integral_z = 0.0;
        self.allocator.reset();
        self.inner_loop_counter = 0;
    }
}