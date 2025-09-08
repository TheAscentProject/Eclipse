use crate::config::AircraftConfig;
use crate::physics::{RigidBody, RigidBodyState};
use crate::aero::{AeroForces, Atmosphere};
use crate::control::{AutoPilot, ControlTarget, ControlOutputs};
use crate::math::Vec3;

pub struct Aircraft {
    pub config: AircraftConfig,
    pub body: RigidBody,
    pub state: RigidBodyState,
    pub autopilot: AutoPilot,
    pub atmosphere: Atmosphere,
}

impl Aircraft {
    pub fn new(config: AircraftConfig) -> Self {
        let body = config.to_rigid_body();
        let n_vtol_motors = config.vtol_props.len();
        let autopilot = AutoPilot::new(n_vtol_motors);
        
        Self {
            config,
            body,
            state: RigidBodyState::new(),
            autopilot,
            atmosphere: Atmosphere::sea_level(),
        }
    }

    pub fn set_initial_state(&mut self, position: Vec3, velocity: Vec3) {
        self.state.position = position;
        self.state.velocity = velocity;
    }

    pub fn update(&mut self, target: &ControlTarget, dt: f64) {
        self.atmosphere = Atmosphere::at_altitude(self.state.position.z);
        
        let airspeed = self.state.velocity.magnitude();
        let control_outputs = self.autopilot.update(&self.state, target, dt, airspeed);
        
        let (total_force, total_moment) = self.compute_forces_and_moments(&control_outputs);
        
        let forces_body = Vec3::new(total_force.x, total_force.y, total_force.z);
        let moments_body = Vec3::new(total_moment.x, total_moment.y, total_moment.z);
        
        use crate::physics::Integrator;
        let integrator = crate::physics::RungeKutta4::new();
        self.state = integrator.integrate(
            &self.state,
            &forces_body,
            &moments_body,
            self.body.mass,
            &self.body.inertia,
            dt,
        );
    }

    fn compute_forces_and_moments(&self, controls: &ControlOutputs) -> (Vec3, Vec3) {
        let mut total_forces = AeroForces::zero();
        
        let airspeed = self.state.velocity.magnitude();
        let alpha = if airspeed > 1e-6 {
            (-self.state.velocity.z / airspeed).asin()
        } else {
            0.0
        };
        
        let wing_forces = self.config.wing.compute_forces(
            &self.state.velocity,
            alpha,
            self.atmosphere.density,
            Some(controls.thrust_cruise * 20.0),
        );
        total_forces.add(&wing_forces);
        
        let mut applied_forces = Vec::new();
        
        for (i, mount) in self.config.vtol_props.iter().enumerate() {
            if i < controls.thrust_vtol.len() {
                let thrust_magnitude = controls.thrust_vtol[i] * 500.0;
                let thrust_vector = mount.direction * thrust_magnitude;
                applied_forces.push((thrust_vector, mount.position));
            }
        }
        
        for mount in &self.config.cruise_props {
            let thrust_magnitude = controls.thrust_cruise * 800.0;
            let thrust_vector = mount.direction * thrust_magnitude;
            applied_forces.push((thrust_vector, mount.position));
        }
        
        applied_forces.push((total_forces.total_force(), Vec3::zero()));
        
        let (force, moment) = self.body.compute_forces_and_moments(&self.state, &applied_forces);
        
        let additional_moment = total_forces.moment + Vec3::new(
            controls.aileron * 100.0,
            controls.elevator * 100.0,
            controls.rudder * 50.0,
        );
        
        (force, moment + additional_moment)
    }

    pub fn get_telemetry(&self) -> AircraftTelemetry {
        let (roll, pitch, yaw) = self.state.orientation.to_euler();
        let airspeed = self.state.velocity.magnitude();
        
        AircraftTelemetry {
            position: self.state.position,
            velocity: self.state.velocity,
            attitude: (roll, pitch, yaw),
            angular_velocity: self.state.angular_velocity,
            airspeed,
            altitude: self.state.position.z,
            ground_speed: (self.state.velocity.x.powi(2) + self.state.velocity.y.powi(2)).sqrt(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct AircraftTelemetry {
    pub position: Vec3,
    pub velocity: Vec3,
    pub attitude: (f64, f64, f64),
    pub angular_velocity: Vec3,
    pub airspeed: f64,
    pub altitude: f64,
    pub ground_speed: f64,
}