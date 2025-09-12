use nalgebra::{DMatrix, DVector};
use crate::math::Vec3;

/// QP-based control allocator with saturation and rate limits
pub struct QPAllocator {
    /// Control effectiveness matrix B
    pub b_matrix: DMatrix<f64>,
    
    /// Number of actuators
    pub n_actuators: usize,
    
    /// Actuator limits
    pub u_min: DVector<f64>,
    pub u_max: DVector<f64>,
    
    /// Rate limits (per second)
    pub du_max: DVector<f64>,
    
    /// Previous actuator commands
    pub u_prev: DVector<f64>,
    
    /// Preferred actuator positions (for null-space)
    pub u_pref: DVector<f64>,
    
    /// Weight matrix for preferred positions
    pub w_pref: DMatrix<f64>,
}

impl QPAllocator {
    pub fn new_quadrotor() -> Self {
        // 4 VTOL motors in X configuration
        // Control: [Mx, My, Mz, Fz]
        // Actuators: [motor1, motor2, motor3, motor4]
        
        // Motor positions relative to CG
        let l = 0.5; // arm length
        
        // B matrix: maps motor thrusts to [Mx, My, Mz, Fz]
        let b_matrix = DMatrix::from_row_slice(4, 4, &[
            // Mx (roll moment) - reduced effectiveness
            l*0.1, -l*0.1, -l*0.1, l*0.1,
            // My (pitch moment) - reduced effectiveness
            l*0.1, l*0.1, -l*0.1, -l*0.1,
            // Mz (yaw moment) - small for counter-rotation
            0.01, -0.01, 0.01, -0.01,
            // Fz (vertical force) - each motor contributes 0.25 of total
            0.25, 0.25, 0.25, 0.25,
        ]);
        
        let n_actuators = 4;
        
        // Actuator limits (normalized thrust 0-2)
        let u_min = DVector::from_element(n_actuators, 0.1); // Min thrust to avoid shutdown
        let u_max = DVector::from_element(n_actuators, 2.0);
        
        // Rate limits (per second)
        let du_max = DVector::from_element(n_actuators, 5.0);
        
        // Initial commands (hover thrust)
        let u_prev = DVector::from_element(n_actuators, 1.0);
        
        // Preferred position (balanced thrust)
        let u_pref = DVector::from_element(n_actuators, 1.0);
        
        // Weight for preferred position
        let w_pref = DMatrix::identity(n_actuators, n_actuators) * 0.01;
        
        Self {
            b_matrix,
            n_actuators,
            u_min,
            u_max,
            du_max,
            u_prev,
            u_pref,
            w_pref,
        }
    }
    
    pub fn allocate(
        &mut self,
        desired_wrench: &DVector<f64>,
        dt: f64,
    ) -> DVector<f64> {
        // Solve: min ||W(u - u_pref)||^2
        //        s.t. B*u = v (desired wrench)
        //             u_min <= u <= u_max
        //             |u - u_prev| <= du_max * dt
        
        // For simplicity, use weighted pseudo-inverse with saturation
        // (Full QP would use osqp or similar solver)
        
        // Compute pseudo-inverse
        let b_pinv = self.compute_weighted_pinv();
        
        // Basic solution
        let u_basic = &b_pinv * desired_wrench;
        
        // Add null-space contribution toward preferred
        let null_space = DMatrix::identity(self.n_actuators, self.n_actuators) - &b_pinv * &self.b_matrix;
        let u_null = &null_space * (&self.u_pref - &u_basic) * 0.1;
        
        let mut u_cmd = u_basic + u_null;
        
        // Apply rate limits
        for i in 0..self.n_actuators {
            let du = u_cmd[i] - self.u_prev[i];
            let du_limit = self.du_max[i] * dt;
            
            if du.abs() > du_limit {
                u_cmd[i] = self.u_prev[i] + du.signum() * du_limit;
            }
        }
        
        // Apply saturation
        for i in 0..self.n_actuators {
            u_cmd[i] = u_cmd[i].clamp(self.u_min[i], self.u_max[i]);
        }
        
        // Update previous
        self.u_prev = u_cmd.clone();
        
        u_cmd
    }
    
    fn compute_weighted_pinv(&self) -> DMatrix<f64> {
        // Weighted pseudo-inverse: B^T * (B * B^T)^-1
        let bt = self.b_matrix.transpose();
        let bbt = &self.b_matrix * &bt;
        
        // Add small regularization for numerical stability
        let reg = DMatrix::identity(bbt.nrows(), bbt.ncols()) * 1e-6;
        let bbt_inv = (bbt.clone() + reg).try_inverse().unwrap_or_else(|| {
            DMatrix::identity(bbt.nrows(), bbt.ncols())
        });
        
        &bt * bbt_inv
    }
    
    pub fn reset(&mut self) {
        self.u_prev = self.u_pref.clone();
    }
}