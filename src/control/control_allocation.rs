use crate::math::Vec3;
use nalgebra::{DMatrix, DVector};

/// Control allocation system for mixed VTOL/cruise propulsion
pub struct ControlAllocator {
    /// Number of VTOL motors
    n_vtol: usize,
    /// Number of cruise motors
    n_cruise: usize,
    /// Control effectiveness matrix
    effectiveness_matrix: DMatrix<f64>,
    /// Motor positions relative to CG
    vtol_positions: Vec<Vec3>,
    cruise_positions: Vec<Vec3>,
}

impl ControlAllocator {
    pub fn new(n_vtol: usize, n_cruise: usize) -> Self {
        // Default quad VTOL + single cruise configuration
        let vtol_positions = vec![
            Vec3::new(0.5, -1.0, 0.0),   // Front-left
            Vec3::new(-0.5, -1.0, 0.0),  // Back-left
            Vec3::new(-0.5, 1.0, 0.0),   // Back-right
            Vec3::new(0.5, 1.0, 0.0),    // Front-right
        ];
        
        let cruise_positions = vec![
            Vec3::new(1.5, 0.0, 0.0),    // Forward cruise prop
        ];
        
        let mut allocator = Self {
            n_vtol,
            n_cruise,
            effectiveness_matrix: DMatrix::zeros(0, 0),
            vtol_positions,
            cruise_positions,
        };
        
        allocator.build_effectiveness_matrix();
        allocator
    }
    
    /// Build the control effectiveness matrix
    /// Maps actuator commands to [roll_moment, pitch_moment, yaw_moment, vertical_force, forward_force]
    fn build_effectiveness_matrix(&mut self) {
        let n_controls = 5; // roll, pitch, yaw, vertical, forward
        let n_actuators = self.n_vtol + self.n_cruise;
        
        let mut matrix = DMatrix::zeros(n_controls, n_actuators);
        
        // VTOL motors contribution
        for (i, pos) in self.vtol_positions.iter().enumerate() {
            // Roll moment (y-position × vertical thrust)
            matrix[(0, i)] = pos.y;
            // Pitch moment (x-position × vertical thrust)
            matrix[(1, i)] = -pos.x;
            // Yaw moment (alternating for counter-rotation)
            matrix[(2, i)] = if i % 2 == 0 { 0.1 } else { -0.1 };
            // Vertical force
            matrix[(3, i)] = 1.0;
            // Forward force (none for pure VTOL)
            matrix[(4, i)] = 0.0;
        }
        
        // Cruise motors contribution
        for (i, _pos) in self.cruise_positions.iter().enumerate() {
            let idx = self.n_vtol + i;
            // No moment contribution (aligned with CG)
            matrix[(0, idx)] = 0.0;
            matrix[(1, idx)] = 0.0;
            matrix[(2, idx)] = 0.0;
            // No vertical force
            matrix[(3, idx)] = 0.0;
            // Forward force
            matrix[(4, idx)] = 1.0;
        }
        
        self.effectiveness_matrix = matrix;
    }
    
    /// Allocate control demands to actuators
    pub fn allocate(
        &self,
        roll_moment: f64,
        pitch_moment: f64,
        yaw_moment: f64,
        vertical_force: f64,
        forward_force: f64,
        transition_blend: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        // Control demand vector
        let demands = DVector::from_vec(vec![
            roll_moment,
            pitch_moment,
            yaw_moment,
            vertical_force,
            forward_force,
        ]);
        
        // Use pseudo-inverse for allocation (handles over/under-actuated cases)
        let pseudo_inverse = self.compute_pseudo_inverse();
        let actuator_commands = pseudo_inverse * demands;
        
        // Split into VTOL and cruise commands
        let mut vtol_commands = Vec::with_capacity(self.n_vtol);
        let mut cruise_commands = Vec::with_capacity(self.n_cruise);
        
        for i in 0..self.n_vtol {
            let cmd = actuator_commands[i];
            // Apply transition scaling to VTOL motors
            let scaled_cmd = cmd * (1.0 - transition_blend * 0.7); // Keep some VTOL in transition
            vtol_commands.push(scaled_cmd.clamp(0.0, 2.0));
        }
        
        for i in 0..self.n_cruise {
            let cmd = actuator_commands[self.n_vtol + i];
            // Scale cruise motors based on transition progress
            let scaled_cmd = cmd * (0.3 + transition_blend * 0.7);
            cruise_commands.push(scaled_cmd.clamp(0.0, 1.0));
        }
        
        (vtol_commands, cruise_commands)
    }
    
    /// Compute Moore-Penrose pseudo-inverse
    fn compute_pseudo_inverse(&self) -> DMatrix<f64> {
        let m = &self.effectiveness_matrix;
        let mt = m.transpose();
        
        // Try (M^T * M)^-1 * M^T for over-determined systems
        if let Some(inv) = (&mt * m).try_inverse() {
            inv * &mt
        } else {
            // Fall back to M^T * (M * M^T)^-1 for under-determined systems
            if let Some(inv) = (m * &mt).try_inverse() {
                &mt * inv
            } else {
                // Return transpose as last resort (won't be optimal but won't crash)
                mt
            }
        }
    }
    
    /// Simple mixing for basic multirotor control (fallback method)
    pub fn mix_multirotor(
        &self,
        throttle: f64,
        roll: f64,
        pitch: f64,
        yaw: f64,
    ) -> Vec<f64> {
        let mut commands = Vec::with_capacity(self.n_vtol);
        
        if self.n_vtol >= 4 {
            // Standard quadcopter mixing
            commands.push(throttle + roll + pitch + yaw);  // Front-left
            commands.push(throttle - roll + pitch - yaw);  // Back-left
            commands.push(throttle - roll - pitch + yaw);  // Back-right
            commands.push(throttle + roll - pitch - yaw);  // Front-right
            
            // Additional motors get base throttle
            for _ in 4..self.n_vtol {
                commands.push(throttle);
            }
        } else {
            // Fallback for non-standard configurations
            for _ in 0..self.n_vtol {
                commands.push(throttle);
            }
        }
        
        // Clamp outputs
        commands.iter_mut().for_each(|c| *c = c.clamp(0.0, 2.0));
        commands
    }
}