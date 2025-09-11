use crate::control::{PidController, FlightMode, ControlTarget};
use crate::math::Vec3;
use crate::physics::RigidBodyState;

#[derive(Debug, Clone)]
pub struct ControlOutputs {
    pub thrust_vtol: Vec<f64>,
    pub thrust_cruise: f64,
    pub flap_deflection: f64,
    pub aileron: f64,
    pub elevator: f64,
    pub rudder: f64,
}

impl ControlOutputs {
    pub fn zero(n_vtol_motors: usize) -> Self {
        Self {
            thrust_vtol: vec![0.0; n_vtol_motors],
            thrust_cruise: 0.0,
            flap_deflection: 0.0,
            aileron: 0.0,
            elevator: 0.0,
            rudder: 0.0,
        }
    }
}

pub struct AutoPilot {
    pub roll_pid: PidController,
    pub pitch_pid: PidController,
    pub yaw_pid: PidController,
    pub altitude_pid: PidController,
    pub velocity_pid: PidController,
    pub position_pid_x: PidController,
    pub position_pid_y: PidController,
    pub n_vtol_motors: usize,
    pub transition_airspeed: f64,
}

impl AutoPilot {
    pub fn new(n_vtol_motors: usize) -> Self {
        Self {
            roll_pid: PidController::new(0.15, 0.002, 0.08).with_limits(0.3, 0.5),
            pitch_pid: PidController::new(0.15, 0.002, 0.08).with_limits(0.3, 0.5),
            yaw_pid: PidController::new(0.1, 0.001, 0.02).with_limits(0.2, 0.3),
            altitude_pid: PidController::new(0.3, 0.02, 0.15).with_limits(1.5, 0.5),
            velocity_pid: PidController::new(0.2, 0.01, 0.05).with_limits(1.0, 0.5),
            position_pid_x: PidController::new(0.15, 0.005, 0.05).with_limits(0.5, 0.3),
            position_pid_y: PidController::new(0.15, 0.005, 0.05).with_limits(0.5, 0.3),
            n_vtol_motors,
            transition_airspeed: 15.0,
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
        let (roll, pitch, yaw) = state.orientation.to_euler();

        match target.mode {
            FlightMode::Hover => {
                self.update_hover(state, target, dt, &mut outputs);
            }
            FlightMode::Stabilize => {
                self.update_stabilize(state, target, dt, &mut outputs, airspeed);
            }
            FlightMode::AltHold => {
                self.update_altitude_hold(state, target, dt, &mut outputs);
            }
            FlightMode::Transition => {
                self.update_transition(state, target, dt, &mut outputs, airspeed);
            }
            _ => {}
        }

        if let Some((target_roll, target_pitch, target_yaw)) = target.attitude {
            outputs.aileron = self.roll_pid.update(target_roll - roll, dt) as f64;
            outputs.elevator = self.pitch_pid.update(target_pitch - pitch, dt) as f64;
            outputs.rudder = self.yaw_pid.update(target_yaw - yaw, dt) as f64;
        }

        outputs
    }

    fn update_hover(
        &mut self,
        state: &RigidBodyState,
        target: &ControlTarget,
        dt: f64,
        outputs: &mut ControlOutputs,
    ) {
        let base_thrust = 0.667;

        if let Some(target_altitude) = target.altitude {
            let altitude_error = target_altitude - (-state.position.z);
            let thrust_adjustment = self.altitude_pid.update(altitude_error, dt);
            let total_thrust = (base_thrust + thrust_adjustment).clamp(0.0, 1.0);

            for i in 0..self.n_vtol_motors {
                outputs.thrust_vtol[i] = total_thrust;
            }
        }

        if let Some(target_pos) = target.position {
            let pos_error_x = target_pos.x - state.position.x;
            let pos_error_y = target_pos.y - state.position.y;

            let target_roll = -self.position_pid_y.update(pos_error_y, dt).to_radians();
            let target_pitch = self.position_pid_x.update(pos_error_x, dt).to_radians();

            let (roll, pitch, _) = state.orientation.to_euler();
            let roll_output = self.roll_pid.update(target_roll - roll, dt);
            let pitch_output = self.pitch_pid.update(target_pitch - pitch, dt);

            self.mix_multirotor_controls(outputs, 0.0, roll_output, pitch_output, 0.0);
        }
    }

    fn update_stabilize(
        &mut self,
        state: &RigidBodyState,
        target: &ControlTarget,
        dt: f64,
        outputs: &mut ControlOutputs,
        airspeed: f64,
    ) {
        if airspeed > self.transition_airspeed {
            outputs.thrust_cruise = 0.3;
            for thrust in &mut outputs.thrust_vtol {
                *thrust = 0.0;
            }
        } else {
            outputs.thrust_cruise = 0.0;
            let base_thrust = 0.4;
            for thrust in &mut outputs.thrust_vtol {
                *thrust = base_thrust;
            }
        }
    }

    fn update_altitude_hold(
        &mut self,
        state: &RigidBodyState,
        target: &ControlTarget,
        dt: f64,
        outputs: &mut ControlOutputs,
    ) {
        if let Some(target_altitude) = target.altitude {
            let altitude_error = target_altitude - (-state.position.z);
            let thrust_adjustment = self.altitude_pid.update(altitude_error, dt);
            
            let base_thrust = 0.25;
            let total_thrust = (base_thrust + thrust_adjustment).clamp(0.0, 1.0);

            for thrust in &mut outputs.thrust_vtol {
                *thrust = total_thrust;
            }
        }
    }

    fn update_transition(
        &mut self,
        state: &RigidBodyState,
        _target: &ControlTarget,
        _dt: f64,
        outputs: &mut ControlOutputs,
        airspeed: f64,
    ) {
        let transition_ratio = (airspeed / self.transition_airspeed).clamp(0.0, 1.0);
        
        outputs.thrust_cruise = transition_ratio * 0.5;
        
        let vtol_thrust = (1.0 - transition_ratio) * 0.3;
        for thrust in &mut outputs.thrust_vtol {
            *thrust = vtol_thrust;
        }
        
        outputs.flap_deflection = (1.0 - transition_ratio) * 30.0;
    }

    fn mix_multirotor_controls(
        &self,
        outputs: &mut ControlOutputs,
        throttle: f64,
        roll: f64,
        pitch: f64,
        yaw: f64,
    ) {
        if self.n_vtol_motors >= 4 {
            outputs.thrust_vtol[0] = (throttle + roll * 0.1 + pitch * 0.1 + yaw * 0.05).clamp(0.0, 1.0);
            outputs.thrust_vtol[1] = (throttle - roll * 0.1 + pitch * 0.1 - yaw * 0.05).clamp(0.0, 1.0);
            outputs.thrust_vtol[2] = (throttle - roll * 0.1 - pitch * 0.1 + yaw * 0.05).clamp(0.0, 1.0);
            outputs.thrust_vtol[3] = (throttle + roll * 0.1 - pitch * 0.1 - yaw * 0.05).clamp(0.0, 1.0);

            for i in 4..self.n_vtol_motors {
                outputs.thrust_vtol[i] = throttle.clamp(0.0, 1.0);
            }
        }
    }
}