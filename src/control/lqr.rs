use nalgebra::{DMatrix, DVector, Vector6, Matrix3x6};
use crate::math::Vec3;
use crate::physics::RigidBodyState;

/// LQR Controller for attitude stabilization
pub struct AttitudeLQR {
    /// State feedback gain matrix K (3x6) for [roll, pitch, yaw, p, q, r] -> [Mx, My, Mz]
    pub k_gain: Matrix3x6<f64>,
    
    /// Integral gains for attitude tracking
    pub ki_roll: f64,
    pub ki_pitch: f64,
    pub ki_yaw: f64,
    
    /// Integral states
    pub integral_roll: f64,
    pub integral_pitch: f64,
    pub integral_yaw: f64,
    
    /// Anti-windup limits
    pub integral_limit: f64,
    
    /// Rate limits (rad/s)
    pub max_rate: f64,
    
    /// Previous commanded attitudes for rate limiting
    pub prev_roll_cmd: f64,
    pub prev_pitch_cmd: f64,
}

impl AttitudeLQR {
    pub fn new() -> Self {
        // LQR gains computed for hover condition
        // Q = diag([10, 10, 1, 1, 1, 1]) for [roll, pitch, yaw, p, q, r]
        // R = diag([0.1, 0.1, 0.1]) for [Mx, My, Mz]
        let k_gain = Matrix3x6::new(
            // Roll control (Mx output)
            2.5, 0.0, 0.0, 0.8, 0.0, 0.0,
            // Pitch control (My output)
            0.0, 2.5, 0.0, 0.0, 0.8, 0.0,
            // Yaw control (Mz output)
            0.0, 0.0, 1.0, 0.0, 0.0, 0.4,
        );
        
        Self {
            k_gain,
            ki_roll: 0.5,
            ki_pitch: 0.5,
            ki_yaw: 0.2,
            integral_roll: 0.0,
            integral_pitch: 0.0,
            integral_yaw: 0.0,
            integral_limit: 0.5,
            max_rate: 1.0, // rad/s
            prev_roll_cmd: 0.0,
            prev_pitch_cmd: 0.0,
        }
    }
    
    pub fn update(
        &mut self,
        state: &RigidBodyState,
        roll_cmd: f64,
        pitch_cmd: f64,
        yaw_cmd: f64,
        dt: f64,
    ) -> Vec3 {
        // Rate limit commands
        let roll_cmd = self.rate_limit(roll_cmd, self.prev_roll_cmd, dt);
        let pitch_cmd = self.rate_limit(pitch_cmd, self.prev_pitch_cmd, dt);
        self.prev_roll_cmd = roll_cmd;
        self.prev_pitch_cmd = pitch_cmd;
        
        // Get current attitude and rates
        let (roll, pitch, yaw) = state.orientation.to_euler();
        let p = state.angular_velocity.x;
        let q = state.angular_velocity.y;
        let r = state.angular_velocity.z;
        
        // State error vector
        let state_error = Vector6::new(
            roll - roll_cmd,
            pitch - pitch_cmd,
            angle_wrap(yaw - yaw_cmd),
            p,  // Rate errors (desired rates are 0)
            q,
            r,
        );
        
        // LQR feedback
        let control = -self.k_gain * state_error;
        
        // Update integrals with anti-windup
        self.integral_roll = self.update_integral(
            self.integral_roll,
            roll - roll_cmd,
            control.x,
            dt,
        );
        self.integral_pitch = self.update_integral(
            self.integral_pitch,
            pitch - pitch_cmd,
            control.y,
            dt,
        );
        self.integral_yaw = self.update_integral(
            self.integral_yaw,
            angle_wrap(yaw - yaw_cmd),
            control.z,
            dt,
        );
        
        // Add integral action
        Vec3::new(
            control.x + self.ki_roll * self.integral_roll,
            control.y + self.ki_pitch * self.integral_pitch,
            control.z + self.ki_yaw * self.integral_yaw,
        )
    }
    
    fn rate_limit(&self, cmd: f64, prev_cmd: f64, dt: f64) -> f64 {
        let max_change = self.max_rate * dt;
        let change = cmd - prev_cmd;
        
        if change.abs() > max_change {
            prev_cmd + change.signum() * max_change
        } else {
            cmd
        }
    }
    
    fn update_integral(&self, integral: f64, error: f64, control: f64, dt: f64) -> f64 {
        // Back-calculation anti-windup
        let sat_limit = 2.0;
        let anti_windup = if control.abs() > sat_limit {
            -0.1 * (control - control.clamp(-sat_limit, sat_limit))
        } else {
            0.0
        };
        
        let new_integral = integral + (error + anti_windup) * dt;
        new_integral.clamp(-self.integral_limit, self.integral_limit)
    }
}

/// LQR Controller for position/velocity tracking
pub struct PositionLQR {
    /// State feedback gain for hover: K (3x6) for [x,y,z,vx,vy,vz] -> [roll_cmd, pitch_cmd, thrust]
    pub k_hover: DMatrix<f64>,
    
    /// State feedback gain for cruise
    pub k_cruise: DMatrix<f64>,
    
    /// Integral states for position
    pub integral_x: f64,
    pub integral_y: f64,
    pub integral_z: f64,
    
    /// Integral gains
    pub ki_pos: f64,
    
    /// Anti-windup limit
    pub integral_limit: f64,
}

impl PositionLQR {
    pub fn new() -> Self {
        // Hover LQR: Q = diag([5,5,8, 2,2,3]), R = diag([1,1,0.5])
        let k_hover = DMatrix::from_row_slice(3, 6, &[
            // Roll command from y-position and y-velocity
            0.0, -1.5, 0.0, 0.0, -0.7, 0.0,
            // Pitch command from x-position and x-velocity  
            1.5, 0.0, 0.0, 0.7, 0.0, 0.0,
            // Thrust from z-position and z-velocity
            0.0, 0.0, 2.0, 0.0, 0.0, 1.2,
        ]);
        
        // Cruise LQR: different gains for forward flight
        let k_cruise = DMatrix::from_row_slice(3, 6, &[
            0.0, -0.8, 0.0, 0.0, -0.4, 0.0,
            0.8, 0.0, 0.0, 0.4, 0.0, 0.0,
            0.0, 0.0, 1.5, 0.0, 0.0, 0.8,
        ]);
        
        Self {
            k_hover,
            k_cruise,
            integral_x: 0.0,
            integral_y: 0.0,
            integral_z: 0.0,
            ki_pos: 0.2,
            integral_limit: 0.3,
        }
    }
    
    pub fn update(
        &mut self,
        state: &RigidBodyState,
        pos_cmd: Vec3,
        vel_cmd: Vec3,
        airspeed: f64,
        dt: f64,
    ) -> (f64, f64, f64) {
        // State vector [x, y, z, vx, vy, vz]
        let state_vec = DVector::from_vec(vec![
            state.position.x - pos_cmd.x,
            state.position.y - pos_cmd.y,
            state.position.z - pos_cmd.z,
            state.velocity.x - vel_cmd.x,
            state.velocity.y - vel_cmd.y,
            state.velocity.z - vel_cmd.z,
        ]);
        
        // Blend gains based on airspeed
        let blend = ((airspeed - 5.0) / 15.0).clamp(0.0, 1.0);
        let k_gain = &self.k_hover * (1.0 - blend) + &self.k_cruise * blend;
        
        // LQR feedback
        let control = -&k_gain * &state_vec;
        
        // Update integrals with anti-windup
        self.integral_x = self.update_integral(
            self.integral_x,
            state.position.x - pos_cmd.x,
            control[1], // pitch command
            dt,
        );
        self.integral_y = self.update_integral(
            self.integral_y,
            state.position.y - pos_cmd.y,
            control[0], // roll command
            dt,
        );
        self.integral_z = self.update_integral(
            self.integral_z,
            state.position.z - pos_cmd.z,
            control[2], // thrust
            dt,
        );
        
        // Output: [roll_cmd, pitch_cmd, thrust_adjustment]
        (
            control[0] - self.ki_pos * self.integral_y,  // Roll from Y
            control[1] + self.ki_pos * self.integral_x,  // Pitch from X
            control[2] + self.ki_pos * self.integral_z,  // Thrust from Z
        )
    }
    
    fn update_integral(&self, integral: f64, error: f64, control: f64, dt: f64) -> f64 {
        let sat_limit = 0.5;
        let anti_windup = if control.abs() > sat_limit {
            -0.1 * (control - control.clamp(-sat_limit, sat_limit))
        } else {
            0.0
        };
        
        let new_integral = integral + (error + anti_windup) * dt;
        new_integral.clamp(-self.integral_limit, self.integral_limit)
    }
}

fn angle_wrap(angle: f64) -> f64 {
    let mut wrapped = angle;
    while wrapped > std::f64::consts::PI {
        wrapped -= 2.0 * std::f64::consts::PI;
    }
    while wrapped < -std::f64::consts::PI {
        wrapped += 2.0 * std::f64::consts::PI;
    }
    wrapped
}