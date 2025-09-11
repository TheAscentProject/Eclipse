use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PidController {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub integral: f64,
    pub previous_error: f64,
    pub integral_limit: f64,
    pub output_limit: f64,
    pub derivative_filtered: f64,
    pub derivative_filter_alpha: f64,
    pub anti_windup_gain: f64,
}

impl PidController {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            previous_error: 0.0,
            integral_limit: 1000.0,
            output_limit: 1000.0,
            derivative_filtered: 0.0,
            derivative_filter_alpha: 0.8,
            anti_windup_gain: 0.1,
        }
    }

    pub fn with_limits(mut self, integral_limit: f64, output_limit: f64) -> Self {
        self.integral_limit = integral_limit;
        self.output_limit = output_limit;
        self
    }

    pub fn update(&mut self, error: f64, dt: f64) -> f64 {
        let dt_safe = dt.max(1e-6);
        
        let derivative_raw = (error - self.previous_error) / dt_safe;
        self.derivative_filtered = self.derivative_filter_alpha * self.derivative_filtered 
            + (1.0 - self.derivative_filter_alpha) * derivative_raw;
        
        let output_unsat = self.kp * error + self.ki * self.integral + self.kd * self.derivative_filtered;
        let output = output_unsat.clamp(-self.output_limit, self.output_limit);
        
        let saturation_error = output - output_unsat;
        let integral_increment = (error + self.anti_windup_gain * saturation_error) * dt;
        self.integral += integral_increment;
        self.integral = self.integral.clamp(-self.integral_limit, self.integral_limit);
        
        self.previous_error = error;
        output
    }

}
