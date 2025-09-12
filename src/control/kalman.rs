use nalgebra::{DMatrix, DVector};
use crate::physics::RigidBodyState;
use crate::control::sysid::LinearModel;
use crate::math::Vec3;

/// Extended Kalman Filter for state estimation
pub struct ExtendedKalmanFilter {
    /// State estimate [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r, bx, by, bz]
    pub x_hat: DVector<f64>,
    /// State covariance matrix
    pub p_matrix: DMatrix<f64>,
    /// Process noise covariance
    pub q_process: DMatrix<f64>,
    /// Measurement noise covariance  
    pub r_measurement: DMatrix<f64>,
    /// State dimension (15: 12 states + 3 gyro biases)
    state_dim: usize,
    /// Measurement dimension (9: 3 accel + 3 gyro + 3 pos from GPS/vision)
    measurement_dim: usize,
}

/// Sensor measurements
#[derive(Debug, Clone)]
pub struct SensorMeasurements {
    /// Accelerometer readings (m/s²)
    pub accel: Vec3,
    /// Gyroscope readings (rad/s)
    pub gyro: Vec3,
    /// Position measurements (GPS/vision) - optional
    pub position: Option<Vec3>,
    /// Velocity measurements - optional
    pub velocity: Option<Vec3>,
    /// Magnetometer/heading - optional
    pub heading: Option<f64>,
}

impl ExtendedKalmanFilter {
    pub fn new() -> Self {
        let state_dim = 15;
        let measurement_dim = 9;
        
        // Initial state estimate (at rest)
        let x_hat = DVector::zeros(state_dim);
        
        // Initial covariance (high uncertainty)
        let mut p_matrix = DMatrix::zeros(state_dim, state_dim);
        for i in 0..state_dim {
            p_matrix[(i, i)] = match i {
                0..=2 => 1.0,    // Position uncertainty
                3..=5 => 0.5,    // Velocity uncertainty
                6..=8 => 0.1,    // Attitude uncertainty
                9..=11 => 0.05,  // Rate uncertainty
                12..=14 => 0.01, // Bias uncertainty
                _ => 1.0,
            };
        }
        
        // Process noise covariance
        let mut q_process = DMatrix::zeros(state_dim, state_dim);
        for i in 0..state_dim {
            q_process[(i, i)] = match i {
                0..=2 => 0.01,   // Position process noise
                3..=5 => 0.05,   // Velocity process noise
                6..=8 => 0.001,  // Attitude process noise
                9..=11 => 0.01,  // Rate process noise
                12..=14 => 1e-6, // Bias process noise (slow)
                _ => 0.01,
            };
        }
        
        // Measurement noise covariance
        let mut r_measurement = DMatrix::zeros(measurement_dim, measurement_dim);
        for i in 0..measurement_dim {
            r_measurement[(i, i)] = match i {
                0..=2 => 0.1,    // Accelerometer noise
                3..=5 => 0.01,   // Gyroscope noise
                6..=8 => 0.5,    // Position measurement noise
                _ => 0.1,
            };
        }
        
        Self {
            x_hat,
            p_matrix,
            q_process,
            r_measurement,
            state_dim,
            measurement_dim,
        }
    }
    
    /// Initialize filter with known state
    pub fn initialize(&mut self, initial_state: &RigidBodyState) {
        let (roll, pitch, yaw) = initial_state.orientation.to_euler();
        
        self.x_hat = DVector::from_vec(vec![
            initial_state.position.x,
            initial_state.position.y,
            initial_state.position.z,
            initial_state.velocity.x,
            initial_state.velocity.y,
            initial_state.velocity.z,
            roll,
            pitch,
            yaw,
            initial_state.angular_velocity.x,
            initial_state.angular_velocity.y,
            initial_state.angular_velocity.z,
            0.0, 0.0, 0.0,  // Initial bias estimates
        ]);
        
        // Reduce initial uncertainty
        for i in 0..self.state_dim {
            self.p_matrix[(i, i)] *= 0.1;
        }
    }
    
    /// Prediction step using nonlinear dynamics
    pub fn predict(&mut self, control_input: &DVector<f64>, dt: f64) {
        // Propagate state using nonlinear dynamics
        let x_pred = self.propagate_dynamics(&self.x_hat, control_input, dt);
        
        // Compute Jacobian of dynamics for covariance propagation
        let f_jacobian = self.compute_dynamics_jacobian(&self.x_hat, control_input, dt);
        
        // Update state estimate
        self.x_hat = x_pred;
        
        // Update covariance: P = F*P*F^T + Q
        self.p_matrix = &f_jacobian * &self.p_matrix * f_jacobian.transpose() + &self.q_process;
    }
    
    /// Measurement update step
    pub fn update(&mut self, measurements: &SensorMeasurements) {
        // Form measurement vector
        let z = self.form_measurement_vector(measurements);
        
        // Predict measurements
        let z_pred = self.predict_measurements(&self.x_hat);
        
        // Compute measurement Jacobian
        let h_jacobian = self.compute_measurement_jacobian(&self.x_hat);
        
        // Innovation
        let innovation = &z - &z_pred;
        
        // Innovation covariance
        let s = &h_jacobian * &self.p_matrix * h_jacobian.transpose() + &self.r_measurement;
        
        // Kalman gain
        let s_inv = s.try_inverse().unwrap_or_else(|| {
            DMatrix::identity(s.nrows(), s.ncols())
        });
        let k_gain = &self.p_matrix * h_jacobian.transpose() * s_inv;
        
        // State update
        self.x_hat += &k_gain * innovation;
        
        // Covariance update (Joseph form for numerical stability)
        let eye = DMatrix::identity(self.state_dim, self.state_dim);
        let temp = &eye - &k_gain * &h_jacobian;
        self.p_matrix = temp * &self.p_matrix * temp.transpose() + &k_gain * &self.r_measurement * k_gain.transpose();
        
        // Normalize angles
        self.normalize_angles();
    }
    
    /// Get current state estimate as RigidBodyState
    pub fn get_state_estimate(&self) -> RigidBodyState {
        RigidBodyState {
            position: Vec3::new(self.x_hat[0], self.x_hat[1], self.x_hat[2]),
            velocity: Vec3::new(self.x_hat[3], self.x_hat[4], self.x_hat[5]),
            orientation: crate::math::Quaternion::from_euler(self.x_hat[6], self.x_hat[7], self.x_hat[8]),
            angular_velocity: Vec3::new(
                self.x_hat[9] - self.x_hat[12],   // Compensate gyro bias
                self.x_hat[10] - self.x_hat[13],
                self.x_hat[11] - self.x_hat[14],
            ),
        }
    }
    
    /// Propagate nonlinear dynamics
    fn propagate_dynamics(&self, x: &DVector<f64>, u: &DVector<f64>, dt: f64) -> DVector<f64> {
        // Simple integration using current state
        // Position derivatives = velocity
        let pos_dot = DVector::from_vec(vec![x[3], x[4], x[5]]);
        
        // Velocity derivatives = acceleration (from control inputs and gravity)
        let accel = Vec3::new(
            u[0] / 120.0,  // Convert control moments to accelerations (simplified)
            u[1] / 120.0,
            (u[3] - 1.0) * 9.81,  // Thrust acceleration
        );
        let vel_dot = DVector::from_vec(vec![accel.x, accel.y, accel.z + 9.81]);
        
        // Attitude derivatives from angular rates (small angle approximation)
        let att_dot = DVector::from_vec(vec![x[9], x[10], x[11]]);
        
        // Angular acceleration from control moments (simplified)
        let ang_accel = DVector::from_vec(vec![u[0] * 10.0, u[1] * 10.0, u[2] * 5.0]);
        
        // Bias dynamics (constant)
        let bias_dot = DVector::zeros(3);
        
        // Integrate
        let mut x_new = x.clone();
        x_new.rows_mut(0, 3).add_assign(&(pos_dot * dt));
        x_new.rows_mut(3, 3).add_assign(&(vel_dot * dt));
        x_new.rows_mut(6, 3).add_assign(&(att_dot * dt));
        x_new.rows_mut(9, 3).add_assign(&(ang_accel * dt));
        x_new.rows_mut(12, 3).add_assign(&(bias_dot * dt));
        
        x_new
    }
    
    /// Compute Jacobian of dynamics (∂f/∂x)
    fn compute_dynamics_jacobian(&self, x: &DVector<f64>, _u: &DVector<f64>, dt: f64) -> DMatrix<f64> {
        let mut f = DMatrix::identity(self.state_dim, self.state_dim);
        
        // Position derivatives depend on velocity
        f.slice_mut((0, 3), (3, 3)).copy_from(&(DMatrix::identity(3, 3) * dt));
        
        // Other derivatives are mostly identity for this simplified model
        // In practice, would compute full nonlinear Jacobian
        
        f
    }
    
    /// Form measurement vector from sensor data
    fn form_measurement_vector(&self, measurements: &SensorMeasurements) -> DVector<f64> {
        let mut z = DVector::zeros(self.measurement_dim);
        
        // Accelerometer measurements
        z[0] = measurements.accel.x;
        z[1] = measurements.accel.y;
        z[2] = measurements.accel.z;
        
        // Gyroscope measurements
        z[3] = measurements.gyro.x;
        z[4] = measurements.gyro.y;
        z[5] = measurements.gyro.z;
        
        // Position measurements (if available)
        if let Some(pos) = measurements.position {
            z[6] = pos.x;
            z[7] = pos.y;
            z[8] = pos.z;
        } else {
            // Use current estimate if no measurement
            z[6] = self.x_hat[0];
            z[7] = self.x_hat[1];
            z[8] = self.x_hat[2];
        }
        
        z
    }
    
    /// Predict measurements from current state
    fn predict_measurements(&self, x: &DVector<f64>) -> DVector<f64> {
        let mut z_pred = DVector::zeros(self.measurement_dim);
        
        // Predicted accelerometer (specific force + gravity in body frame)
        // Simplified: assume small angles
        z_pred[0] = 0.0;  // ax
        z_pred[1] = 0.0;  // ay  
        z_pred[2] = 9.81; // az (gravity in body frame)
        
        // Predicted gyroscope (angular rates + bias)
        z_pred[3] = x[9] + x[12];   // p + bias_x
        z_pred[4] = x[10] + x[13];  // q + bias_y
        z_pred[5] = x[11] + x[14];  // r + bias_z
        
        // Predicted position
        z_pred[6] = x[0];
        z_pred[7] = x[1];
        z_pred[8] = x[2];
        
        z_pred
    }
    
    /// Compute measurement Jacobian (∂h/∂x)
    fn compute_measurement_jacobian(&self, _x: &DVector<f64>) -> DMatrix<f64> {
        let mut h = DMatrix::zeros(self.measurement_dim, self.state_dim);
        
        // Accelerometer Jacobian (simplified - depends on attitude)
        // For small angles, accelerometer mostly measures gravity
        
        // Gyroscope Jacobian
        h[(3, 9)] = 1.0;   // ∂gyro_x/∂p
        h[(3, 12)] = 1.0;  // ∂gyro_x/∂bias_x
        h[(4, 10)] = 1.0;  // ∂gyro_y/∂q
        h[(4, 13)] = 1.0;  // ∂gyro_y/∂bias_y
        h[(5, 11)] = 1.0;  // ∂gyro_z/∂r
        h[(5, 14)] = 1.0;  // ∂gyro_z/∂bias_z
        
        // Position Jacobian
        h[(6, 0)] = 1.0;   // ∂pos_x/∂x
        h[(7, 1)] = 1.0;   // ∂pos_y/∂y
        h[(8, 2)] = 1.0;   // ∂pos_z/∂z
        
        h
    }
    
    /// Normalize angles to [-π, π]
    fn normalize_angles(&mut self) {
        for i in 6..9 { // Attitude angles
            while self.x_hat[i] > std::f64::consts::PI {
                self.x_hat[i] -= 2.0 * std::f64::consts::PI;
            }
            while self.x_hat[i] < -std::f64::consts::PI {
                self.x_hat[i] += 2.0 * std::f64::consts::PI;
            }
        }
    }
}

/// Generate realistic sensor measurements from true state (for simulation)
pub fn generate_sensor_measurements(true_state: &RigidBodyState, dt: f64) -> SensorMeasurements {
    use rand::Rng;
    let mut rng = rand::thread_rng();
    
    // Add noise to simulate real sensors
    let accel_noise = 0.1;
    let gyro_noise = 0.01;
    let pos_noise = 0.2;
    
    // Accelerometer: specific force (includes gravity)
    let gravity_body = Vec3::new(0.0, 0.0, 9.81); // Simplified
    let accel_true = gravity_body; // Should transform by attitude
    let accel = Vec3::new(
        accel_true.x + rng.gen::<f64>() * accel_noise - accel_noise/2.0,
        accel_true.y + rng.gen::<f64>() * accel_noise - accel_noise/2.0,
        accel_true.z + rng.gen::<f64>() * accel_noise - accel_noise/2.0,
    );
    
    // Gyroscope: angular rates with bias and noise
    let gyro_bias = Vec3::new(0.001, -0.002, 0.0005);
    let gyro = Vec3::new(
        true_state.angular_velocity.x + gyro_bias.x + rng.gen::<f64>() * gyro_noise - gyro_noise/2.0,
        true_state.angular_velocity.y + gyro_bias.y + rng.gen::<f64>() * gyro_noise - gyro_noise/2.0,
        true_state.angular_velocity.z + gyro_bias.z + rng.gen::<f64>() * gyro_noise - gyro_noise/2.0,
    );
    
    // Position (GPS/vision): lower rate, higher noise
    let position = Some(Vec3::new(
        true_state.position.x + rng.gen::<f64>() * pos_noise - pos_noise/2.0,
        true_state.position.y + rng.gen::<f64>() * pos_noise - pos_noise/2.0,
        true_state.position.z + rng.gen::<f64>() * pos_noise - pos_noise/2.0,
    ));
    
    SensorMeasurements {
        accel,
        gyro,
        position,
        velocity: None,
        heading: None,
    }
}