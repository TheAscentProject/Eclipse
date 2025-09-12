use nalgebra::{DMatrix, DVector};
use crate::physics::RigidBodyState;
use crate::control::ControlOutputs;
use crate::config::AircraftConfig;
use crate::sim::Aircraft;
use crate::control::ControlTarget;
use crate::math::Vec3;

/// System identification for extracting linearized A/B matrices
pub struct SystemID {
    /// State dimension (12: pos, vel, attitude, rates)
    state_dim: usize,
    /// Input dimension (4: roll_moment, pitch_moment, yaw_moment, thrust)
    input_dim: usize,
    /// Finite difference step size
    epsilon: f64,
}

/// Linearized system matrices at a trim point
#[derive(Debug, Clone)]
pub struct LinearModel {
    /// State matrix A (dx/dt = Ax + Bu)
    pub a_matrix: DMatrix<f64>,
    /// Input matrix B
    pub b_matrix: DMatrix<f64>,
    /// Trim state
    pub x_trim: DVector<f64>,
    /// Trim input
    pub u_trim: DVector<f64>,
    /// Flight regime (hover, transition, cruise)
    pub regime: FlightRegime,
    /// Airspeed at trim
    pub airspeed: f64,
}

#[derive(Debug, Clone, Copy)]
pub enum FlightRegime {
    Hover,
    Transition,
    Cruise,
}

impl SystemID {
    pub fn new() -> Self {
        Self {
            state_dim: 12, // [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
            input_dim: 4,  // [Mx, My, Mz, Fz]
            epsilon: 1e-6,
        }
    }
    
    /// Identify system around hover trim point
    pub fn identify_hover(&self, config: &AircraftConfig) -> LinearModel {
        let airspeed = 0.0;
        let altitude = 5.0;
        
        // Hover trim state: stationary at 5m altitude
        let x_trim = DVector::from_vec(vec![
            0.0, 0.0, -altitude, // position (NED)
            0.0, 0.0, 0.0,       // velocity
            0.0, 0.0, 0.0,       // attitude
            0.0, 0.0, 0.0,       // angular rates
        ]);
        
        // Hover trim input: just enough thrust to counteract weight, no moments
        let u_trim = DVector::from_vec(vec![
            0.0, 0.0, 0.0,       // moments
            1.0,                 // normalized thrust
        ]);
        
        let (a_matrix, b_matrix) = self.compute_jacobians(config, &x_trim, &u_trim, airspeed);
        
        LinearModel {
            a_matrix,
            b_matrix,
            x_trim,
            u_trim,
            regime: FlightRegime::Hover,
            airspeed,
        }
    }
    
    /// Identify system around forward flight trim point
    pub fn identify_cruise(&self, config: &AircraftConfig, airspeed: f64) -> LinearModel {
        let altitude = 50.0;
        
        // Cruise trim state: forward flight at specified airspeed
        let x_trim = DVector::from_vec(vec![
            0.0, 0.0, -altitude,     // position (NED)
            0.0, airspeed, 0.0,      // velocity (forward in Y)
            0.0, 0.05, 0.0,          // slight pitch for cruise
            0.0, 0.0, 0.0,           // angular rates
        ]);
        
        // Cruise trim input: some thrust, pitch moment for trim
        let u_trim = DVector::from_vec(vec![
            0.0, 0.02, 0.0,          // small pitch moment
            0.6,                     // cruise thrust
        ]);
        
        let (a_matrix, b_matrix) = self.compute_jacobians(config, &x_trim, &u_trim, airspeed);
        
        LinearModel {
            a_matrix,
            b_matrix,
            x_trim,
            u_trim,
            regime: FlightRegime::Cruise,
            airspeed,
        }
    }
    
    /// Compute A and B matrices via finite differences
    fn compute_jacobians(
        &self,
        config: &AircraftConfig,
        x_trim: &DVector<f64>,
        u_trim: &DVector<f64>,
        airspeed: f64,
    ) -> (DMatrix<f64>, DMatrix<f64>) {
        let mut a_matrix = DMatrix::zeros(self.state_dim, self.state_dim);
        let mut b_matrix = DMatrix::zeros(self.state_dim, self.input_dim);
        
        // Get nominal state derivative
        let f0 = self.evaluate_dynamics(config, x_trim, u_trim);
        
        // Compute A matrix (∂f/∂x)
        for i in 0..self.state_dim {
            let mut x_pert = x_trim.clone();
            x_pert[i] += self.epsilon;
            
            let f_pert = self.evaluate_dynamics(config, &x_pert, u_trim);
            let df_dx = (&f_pert - &f0) / self.epsilon;
            
            a_matrix.set_column(i, &df_dx);
        }
        
        // Compute B matrix (∂f/∂u)
        for i in 0..self.input_dim {
            let mut u_pert = u_trim.clone();
            u_pert[i] += self.epsilon;
            
            let f_pert = self.evaluate_dynamics(config, x_trim, &u_pert);
            let df_du = (&f_pert - &f0) / self.epsilon;
            
            b_matrix.set_column(i, &df_du);
        }
        
        (a_matrix, b_matrix)
    }
    
    /// Evaluate nonlinear dynamics: dx/dt = f(x, u)
    fn evaluate_dynamics(
        &self,
        config: &AircraftConfig,
        x: &DVector<f64>,
        u: &DVector<f64>,
    ) -> DVector<f64> {
        // Convert state vector to RigidBodyState
        let state = RigidBodyState {
            position: Vec3::new(x[0], x[1], x[2]),
            velocity: Vec3::new(x[3], x[4], x[5]),
            orientation: crate::math::Quaternion::from_euler(x[6], x[7], x[8]),
            angular_velocity: Vec3::new(x[9], x[10], x[11]),
        };
        
        // Convert input vector to applied forces/moments
        let applied_forces = vec![
            // VTOL motors - distribute thrust and moments
            (Vec3::new(0.0, 0.0, -u[3] * 0.25), Vec3::new(0.5, -1.0, 0.0)),  // Motor 1
            (Vec3::new(0.0, 0.0, -u[3] * 0.25), Vec3::new(-0.5, -1.0, 0.0)), // Motor 2
            (Vec3::new(0.0, 0.0, -u[3] * 0.25), Vec3::new(-0.5, 1.0, 0.0)),  // Motor 3
            (Vec3::new(0.0, 0.0, -u[3] * 0.25), Vec3::new(0.5, 1.0, 0.0)),   // Motor 4
            // Additional moments from differential thrust (simplified)
            (Vec3::new(u[0] * 0.1, u[1] * 0.1, u[2] * 0.1), Vec3::zero()),
        ];
        
        // Compute forces and moments using rigid body dynamics
        let rigid_body = config.to_rigid_body();
        let (total_force, total_moment) = rigid_body.compute_forces_and_moments(&state, &applied_forces);
        
        // Add gravity
        let gravity_force = Vec3::new(0.0, 0.0, config.mass * 9.81);
        let total_force_with_gravity = total_force + gravity_force;
        
        // Compute accelerations
        let linear_accel = total_force_with_gravity / config.mass;
        let angular_accel = rigid_body.inertia_inv * (total_moment - state.angular_velocity.cross(&(rigid_body.inertia * state.angular_velocity)));
        
        // Compute attitude derivatives (simplified small-angle approximation)
        let attitude_rate = Vec3::new(
            state.angular_velocity.x + state.angular_velocity.y * x[8].sin() * x[7].tan() + state.angular_velocity.z * x[8].cos() * x[7].tan(),
            state.angular_velocity.y * x[8].cos() - state.angular_velocity.z * x[8].sin(),
            state.angular_velocity.y * x[8].sin() / x[7].cos() + state.angular_velocity.z * x[8].cos() / x[7].cos(),
        );
        
        // Return state derivative
        DVector::from_vec(vec![
            x[3], x[4], x[5],                                    // position derivative = velocity
            linear_accel.x, linear_accel.y, linear_accel.z,     // velocity derivative = acceleration
            attitude_rate.x, attitude_rate.y, attitude_rate.z,  // attitude derivative
            angular_accel.x, angular_accel.y, angular_accel.z,  // angular rate derivative
        ])
    }
}

/// Discretize continuous-time system for digital control
pub fn discretize_system(model: &LinearModel, dt: f64) -> (DMatrix<f64>, DMatrix<f64>) {
    // Use matrix exponential for exact discretization
    // For now, use simple forward Euler (good for small dt)
    let eye = DMatrix::identity(model.a_matrix.nrows(), model.a_matrix.ncols());
    let ad = &eye + &model.a_matrix * dt;
    let bd = &model.b_matrix * dt;
    
    (ad, bd)
}