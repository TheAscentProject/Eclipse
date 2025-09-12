use nalgebra::{DMatrix, DVector};
use crate::control::sysid::{LinearModel, FlightRegime};

/// LQR tuning framework with systematic Q/R matrix design
pub struct LQRTuner {
    /// State dimension
    state_dim: usize,
    /// Input dimension  
    input_dim: usize,
}

/// LQR design parameters
#[derive(Debug, Clone)]
pub struct LQRDesign {
    /// State weighting matrix Q
    pub q_matrix: DMatrix<f64>,
    /// Input weighting matrix R
    pub r_matrix: DMatrix<f64>,
    /// Computed feedback gain K
    pub k_gain: DMatrix<f64>,
    /// Closed-loop eigenvalues
    pub eigenvalues: Vec<f64>,
    /// Phase margin (degrees)
    pub phase_margin: f64,
    /// Gain margin (dB)
    pub gain_margin: f64,
}

impl LQRTuner {
    pub fn new() -> Self {
        Self {
            state_dim: 12,
            input_dim: 4,
        }
    }
    
    /// Design LQR controller for specific flight regime
    pub fn design_controller(&self, model: &LinearModel) -> LQRDesign {
        let (q, r) = self.get_tuning_weights(model.regime, model.airspeed);
        let k_gain = self.solve_lqr(&model.a_matrix, &model.b_matrix, &q, &r);
        
        // Analyze closed-loop properties
        let eigenvalues = self.compute_eigenvalues(&model.a_matrix, &model.b_matrix, &k_gain);
        let (phase_margin, gain_margin) = self.analyze_margins(&model.a_matrix, &model.b_matrix, &k_gain);
        
        LQRDesign {
            q_matrix: q,
            r_matrix: r,
            k_gain,
            eigenvalues,
            phase_margin,
            gain_margin,
        }
    }
    
    /// Get Q/R weighting matrices based on flight regime and tuning principles
    fn get_tuning_weights(&self, regime: FlightRegime, airspeed: f64) -> (DMatrix<f64>, DMatrix<f64>) {
        match regime {
            FlightRegime::Hover => self.hover_weights(),
            FlightRegime::Takeoff => self.takeoff_weights(),
            FlightRegime::Transition => self.transition_weights(airspeed),
            FlightRegime::Cruise => self.cruise_weights(airspeed),
        }
    }
    
    /// Hover tuning: Prioritize position/attitude precision over efficiency
    fn hover_weights(&self) -> (DMatrix<f64>, DMatrix<f64>) {
        // Q matrix: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
        let q_diag = vec![
            10.0, 10.0, 15.0,    // Position: tight altitude control
            5.0,  5.0,  8.0,     // Velocity: moderate damping
            20.0, 20.0, 5.0,     // Attitude: precise roll/pitch, loose yaw
            3.0,  3.0,  2.0,     // Rates: moderate rate damping
        ];
        
        // R matrix: [Mx, My, Mz, Fz] - moderate control effort
        let r_diag = vec![0.5, 0.5, 1.0, 0.1];
        
        (
            DMatrix::from_diagonal(&DVector::from_vec(q_diag)),
            DMatrix::from_diagonal(&DVector::from_vec(r_diag)),
        )
    }
    
    /// Takeoff tuning: Aggressive altitude tracking with high thrust authority
    fn takeoff_weights(&self) -> (DMatrix<f64>, DMatrix<f64>) {
        // Q matrix: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
        let q_diag = vec![
            1.0,  1.0,  100.0,   // Position: MASSIVE altitude penalty
            1.0,  1.0,  50.0,    // Velocity: strong vertical velocity tracking
            15.0, 15.0, 5.0,     // Attitude: maintain level flight
            2.0,  2.0,  2.0,     // Rates: moderate rate damping
        ];
        
        // R matrix: [Mx, My, Mz, Fz] - make thrust very cheap to use
        let r_diag = vec![0.5, 0.5, 1.0, 0.01];
        
        (
            DMatrix::from_diagonal(&DVector::from_vec(q_diag)),
            DMatrix::from_diagonal(&DVector::from_vec(r_diag)),
        )
    }
    
    /// Transition tuning: Balance between precision and robustness
    fn transition_weights(&self, airspeed: f64) -> (DMatrix<f64>, DMatrix<f64>) {
        // Blend between hover and cruise weights
        let blend = (airspeed / 20.0).clamp(0.0, 1.0);
        
        let q_diag = vec![
            8.0 - blend * 3.0,   10.0 - blend * 3.0,   12.0 - blend * 2.0,  // Position
            4.0 - blend * 1.0,   4.0 + blend * 2.0,    6.0 - blend * 1.0,   // Velocity  
            15.0 - blend * 5.0,  15.0 - blend * 5.0,   5.0,                 // Attitude
            2.5 - blend * 0.5,   2.5 - blend * 0.5,    2.0,                 // Rates
        ];
        
        let r_diag = vec![0.3 + blend * 0.4, 0.3 + blend * 0.4, 1.0, 0.2 + blend * 0.3];
        
        (
            DMatrix::from_diagonal(&DVector::from_vec(q_diag)),
            DMatrix::from_diagonal(&DVector::from_vec(r_diag)),
        )
    }
    
    /// Cruise tuning: Prioritize efficiency and smooth flight
    fn cruise_weights(&self, _airspeed: f64) -> (DMatrix<f64>, DMatrix<f64>) {
        // Q matrix: Lower position weights, higher velocity regulation
        let q_diag = vec![
            5.0, 7.0, 10.0,      // Position: track path loosely
            3.0, 6.0, 5.0,       // Velocity: tight forward speed control
            10.0, 10.0, 5.0,     // Attitude: precise for efficiency
            2.0, 2.0, 2.0,       // Rates: smooth operation
        ];
        
        // R matrix: Higher control cost for efficiency
        let r_diag = vec![0.8, 0.8, 1.0, 0.5];
        
        (
            DMatrix::from_diagonal(&DVector::from_vec(q_diag)),
            DMatrix::from_diagonal(&DVector::from_vec(r_diag)),
        )
    }
    
    /// Solve discrete-time algebraic Riccati equation
    fn solve_lqr(&self, a: &DMatrix<f64>, b: &DMatrix<f64>, q: &DMatrix<f64>, r: &DMatrix<f64>) -> DMatrix<f64> {
        // Simplified LQR solution using eigenvalue method
        // For production code, use proper DARE solver like in scipy
        
        // Form Hamiltonian matrix
        let _n = a.nrows();
        let m = b.ncols();
        
        // H = [A + B*R^-1*B^T*P,  -B*R^-1*B^T]
        //     [-Q,                 -(A + B*R^-1*B^T*P)^T]
        
        // Simplified approach: Use fixed-point iteration
        let mut p = q.clone();
        let r_inv = r.clone().try_inverse().unwrap_or_else(|| DMatrix::identity(m, m));
        
        // Iterate P = A^T*P*A - A^T*P*B*(R + B^T*P*B)^-1*B^T*P*A + Q
        for _ in 0..50 {
            let temp1 = &r_inv + b.transpose() * &p * b;
            let temp1_inv = temp1.try_inverse().unwrap_or_else(|| DMatrix::identity(m, m));
            let temp2 = b * temp1_inv * b.transpose() * &p;
            
            let p_new = a.transpose() * &p * a - a.transpose() * temp2 * a + q;
            
            // Check convergence
            let diff = (&p_new - &p).norm();
            p = p_new;
            
            if diff < 1e-8 {
                break;
            }
        }
        
        // Compute feedback gain K = (R + B^T*P*B)^-1 * B^T*P*A
        let temp = &r_inv + b.transpose() * &p * b;
        let temp_inv = temp.try_inverse().unwrap_or_else(|| DMatrix::identity(m, m));
        temp_inv * b.transpose() * p * a
    }
    
    /// Compute closed-loop eigenvalues
    fn compute_eigenvalues(&self, a: &DMatrix<f64>, b: &DMatrix<f64>, k: &DMatrix<f64>) -> Vec<f64> {
        let a_cl = a - b * k;
        
        // Simplified eigenvalue computation (real parts only)
        // For production, use proper eigenvalue solver
        let mut eigenvalues = Vec::new();
        
        // Estimate dominant eigenvalues from diagonal elements (rough approximation)
        for i in 0..a_cl.nrows() {
            eigenvalues.push(a_cl[(i, i)]);
        }
        
        eigenvalues
    }
    
    /// Analyze stability margins
    fn analyze_margins(&self, a: &DMatrix<f64>, b: &DMatrix<f64>, k: &DMatrix<f64>) -> (f64, f64) {
        let a_cl = a - b * k;
        
        // Simplified margin analysis
        // Check if all eigenvalues are stable (negative real parts)
        let eigenvalues = self.compute_eigenvalues(a, b, k);
        let max_real_part = eigenvalues.iter().fold(f64::NEG_INFINITY, |acc, &x| acc.max(x));
        
        // Estimate phase margin from dominant pole
        let damping_ratio = if max_real_part < -0.1 { 0.7 } else { 0.3 };
        let phase_margin = (damping_ratio * 100.0_f64).min(80.0);
        
        // Estimate gain margin from stability
        let gain_margin = if max_real_part < -0.5 { 12.0 } else { 6.0 };
        
        (phase_margin, gain_margin)
    }
    
    /// Auto-tune Q/R matrices for desired performance
    pub fn auto_tune(&self, model: &LinearModel, target_bandwidth: f64, target_damping: f64) -> LQRDesign {
        let mut best_design = self.design_controller(model);
        let mut best_cost = f64::INFINITY;
        
        // Grid search over Q/R scaling factors
        for q_scale in [0.1, 0.5, 1.0, 2.0, 5.0] {
            for r_scale in [0.1, 0.5, 1.0, 2.0, 5.0] {
                let (mut q, mut r) = self.get_tuning_weights(model.regime, model.airspeed);
                q *= q_scale;
                r *= r_scale;
                
                let k_gain = self.solve_lqr(&model.a_matrix, &model.b_matrix, &q, &r);
                let eigenvalues = self.compute_eigenvalues(&model.a_matrix, &model.b_matrix, &k_gain);
                let (phase_margin, gain_margin) = self.analyze_margins(&model.a_matrix, &model.b_matrix, &k_gain);
                
                // Cost function: penalize deviation from targets
                let bandwidth_error = (eigenvalues.iter().map(|x| x.abs()).fold(0.0, f64::max) - target_bandwidth).abs();
                let damping_error = (phase_margin / 60.0 - target_damping).abs();
                let margin_penalty = if phase_margin < 30.0 || gain_margin < 6.0 { 100.0 } else { 0.0 };
                
                let cost = bandwidth_error + damping_error * 10.0 + margin_penalty;
                
                if cost < best_cost {
                    best_cost = cost;
                    best_design = LQRDesign {
                        q_matrix: q,
                        r_matrix: r,
                        k_gain,
                        eigenvalues,
                        phase_margin,
                        gain_margin,
                    };
                }
            }
        }
        
        best_design
    }
}