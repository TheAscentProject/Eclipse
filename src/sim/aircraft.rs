use crate::config::AircraftConfig;
use crate::physics::{RigidBody, RigidBodyState};
use crate::aero::{AeroForces, Atmosphere};
use crate::control::{AutoPilot, ControlTarget, ControlOutputs};
use crate::math::Vec3;
use crate::sim::DisturbanceModel;

pub struct Aircraft {
    pub config: AircraftConfig,
    pub body: RigidBody,
    pub state: RigidBodyState,
    pub autopilot: AutoPilot,
    pub atmosphere: Atmosphere,
    pub disturbances: DisturbanceModel,
    pub last_forces: ForcesTelemetry,
}

#[derive(Debug, Clone, Default)]
pub struct ForcesTelemetry {
    pub total_thrust_vtol: f64,
    pub total_thrust_cruise: f64,
    pub total_force: Vec3,
    pub total_moment: Vec3,
    pub wind_force: Vec3,
    pub gravity_force: Vec3,
    pub aero_force: Vec3,
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
            disturbances: DisturbanceModel::new().with_wind(Vec3::new(0.5, 0.3, 0.0), 1.5),
            last_forces: ForcesTelemetry::default(),
        }
    }

    pub fn set_initial_state(&mut self, position: Vec3, velocity: Vec3) {
        self.state.position = position;
        self.state.velocity = velocity;
    }

    pub fn update(&mut self, target: &ControlTarget, dt: f64) {
        self.atmosphere = Atmosphere::at_altitude(-self.state.position.z);
        self.disturbances.update(dt);
        
        let noisy_state = self.disturbances.add_sensor_noise(&self.state);
        let airspeed = self.state.velocity.magnitude();
        let control_outputs = self.autopilot.update(&noisy_state, target, dt, airspeed);
        
        let (total_force, total_moment) = self.compute_forces_and_moments(&control_outputs);
        
        let wind_force = self.disturbances.get_wind_force(-self.state.position.z);
        self.last_forces.wind_force = wind_force;
        let forces_body = Vec3::new(
            total_force.x + wind_force.x,
            total_force.y + wind_force.y,
            total_force.z + wind_force.z
        );
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

    fn compute_forces_and_moments(&mut self, controls: &ControlOutputs) -> (Vec3, Vec3) {
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
        let mut total_vtol_thrust = 0.0;
        let mut total_cruise_thrust = 0.0;
        
        for (i, mount) in self.config.vtol_props.iter().enumerate() {
            if i < controls.thrust_vtol.len() {
                let thrust_per_motor = self.body.mass * 9.81 / 4.0;
                let thrust_magnitude = controls.thrust_vtol[i] * thrust_per_motor * 1.5;
                total_vtol_thrust += thrust_magnitude;
                let thrust_vector = mount.direction * thrust_magnitude;
                applied_forces.push((thrust_vector, mount.position));
            }
        }
        
        for mount in &self.config.cruise_props {
            let thrust_magnitude = controls.thrust_cruise * self.body.mass * 2.0;
            total_cruise_thrust += thrust_magnitude;
            let thrust_vector = mount.direction * thrust_magnitude;
            applied_forces.push((thrust_vector, mount.position));
        }
        
        self.last_forces.total_thrust_vtol = total_vtol_thrust;
        self.last_forces.total_thrust_cruise = total_cruise_thrust;
        self.last_forces.aero_force = total_forces.total_force();
        
        applied_forces.push((total_forces.total_force(), Vec3::zero()));
        
        let (force, moment) = self.body.compute_forces_and_moments(&self.state, &applied_forces);
        
        self.last_forces.total_force = force;
        self.last_forces.total_moment = moment;
        self.last_forces.gravity_force = Vec3::new(0.0, 0.0, self.body.mass * 9.81);
        
        let additional_moment = total_forces.moment + Vec3::new(
            controls.aileron * 10.0,
            controls.elevator * 10.0,
            controls.rudder * 5.0,
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
            forces: self.last_forces.clone(),
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
    pub forces: ForcesTelemetry,
}