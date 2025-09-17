use crate::config::AircraftConfig;
use crate::physics::{RigidBody, RigidBodyState};
use crate::aero::{AeroForces, Atmosphere};
use crate::control::{AutoPilotLQG, ControlTarget, ControlOutputs};
use crate::math::Vec3;
use crate::sim::DisturbanceModel;
use crate::power::{Battery, PowerTelemetry};

pub struct Aircraft {
    pub config: AircraftConfig,
    pub body: RigidBody,
    pub state: RigidBodyState,
    pub autopilot: AutoPilotLQG,
    pub atmosphere: Atmosphere,
    pub disturbances: DisturbanceModel,
    pub last_forces: ForcesTelemetry,
    pub battery: Battery,
    pub last_power: PowerTelemetry,
    thrust_ramp: f64,
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
        let n_cruise_motors = config.cruise_props.len();
        let mut autopilot = AutoPilotLQG::new(n_vtol_motors, n_cruise_motors);
        
        // Initialize autopilot with aircraft configuration
        autopilot.initialize(config.clone());
        
        let battery_cfg = config.battery.clone();
        Self {
            config,
            body,
            state: RigidBodyState::new(),
            autopilot,
            atmosphere: Atmosphere::sea_level(),
            disturbances: DisturbanceModel::new().with_wind(Vec3::new(0.0, 0.0, 0.0), 0.0),
            last_forces: ForcesTelemetry::default(),
            battery: Battery::new(battery_cfg),
            last_power: PowerTelemetry::default(),
            thrust_ramp: 0.0,
        }
    }

    pub fn set_initial_state(&mut self, position: Vec3, velocity: Vec3) {
        self.state.position = position;
        self.state.velocity = velocity;
        
        // Initialize autopilot estimator with known state
        self.autopilot.reset(&self.state);
    }

    pub fn update(&mut self, target: &ControlTarget, dt: f64) {
        self.atmosphere = Atmosphere::at_altitude(-self.state.position.z);
        self.disturbances.update(dt);
        // Smooth startup ramp to avoid thrust spikes
        let ramp_time = 1.0;
        self.thrust_ramp = (self.thrust_ramp + dt / ramp_time).clamp(0.0, 1.0);
        
        let _noisy_state = self.disturbances.add_sensor_noise(&self.state);
        let airspeed = self.state.velocity.magnitude();
        let control_outputs = self.autopilot.update(&self.state, target, dt, airspeed);
        
    let (total_force, total_moment) = self.compute_forces_and_moments(&control_outputs, dt);
        
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

    fn compute_forces_and_moments(&mut self, controls: &ControlOutputs, dt: f64) -> (Vec3, Vec3) {
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
    let mut mech_power_w = 0.0;
        
    let thrust_headroom = 2.0; // allow up to 2x weight if commanded
    for (i, mount) in self.config.vtol_props.iter().enumerate() {
            if i < controls.thrust_vtol.len() {
        let thrust_per_motor = thrust_headroom * self.body.mass * 9.81 / self.config.vtol_props.len() as f64;
        let thrust_magnitude = (controls.thrust_vtol[i] * self.thrust_ramp) * thrust_per_motor;
                
                let rpm = 2000.0 + controls.thrust_vtol[i] * 1000.0;
                let _prop_thrust = mount.propeller.compute_thrust(
                    rpm,
                    self.state.velocity.dot(&mount.direction),
                    self.atmosphere.density
                );
                // Mechanical power via momentum theory
                let axial = self.state.velocity.dot(&mount.direction).max(0.0);
                let area = std::f64::consts::PI * (mount.propeller.diameter * 0.5).powi(2);
                let rho = self.atmosphere.density;
                let vi = if thrust_magnitude <= 0.0 {
                    0.0
                } else if axial.abs() < 1e-6 {
                    (thrust_magnitude / (2.0 * rho * area)).sqrt()
                } else {
                    let v_half = axial / 2.0;
                    let disc = v_half * v_half + thrust_magnitude / (rho * area);
                    if disc > 0.0 { disc.sqrt() - v_half } else { 0.0 }
                };
                let fom = mount.propeller.figure_of_merit.max(0.3);
                let p_motor = thrust_magnitude * (axial + vi) / fom;
                mech_power_w += p_motor.max(0.0);
                
                total_vtol_thrust += thrust_magnitude;
                
                let thrust_vector = mount.direction * thrust_magnitude;
                applied_forces.push((thrust_vector, mount.position));
                
            }
        }
        
        for mount in &self.config.cruise_props {
            let thrust_magnitude = (controls.thrust_cruise * self.thrust_ramp) * thrust_headroom * self.body.mass * 9.81;
            
            let rpm = 1500.0 + controls.thrust_cruise * 2000.0;
            let _prop_thrust = mount.propeller.compute_thrust(
                rpm,
                self.state.velocity.dot(&mount.direction),
                self.atmosphere.density
            );
            let axial = self.state.velocity.dot(&mount.direction).max(0.0);
            let area = std::f64::consts::PI * (mount.propeller.diameter * 0.5).powi(2);
            let rho = self.atmosphere.density;
            let vi = if thrust_magnitude <= 0.0 {
                0.0
            } else if axial.abs() < 1e-6 {
                (thrust_magnitude / (2.0 * rho * area)).sqrt()
            } else {
                let v_half = axial / 2.0;
                let disc = v_half * v_half + thrust_magnitude / (rho * area);
                if disc > 0.0 { disc.sqrt() - v_half } else { 0.0 }
            };
            let fom = mount.propeller.figure_of_merit.max(0.3);
            let p_motor = thrust_magnitude * (axial + vi) / fom;
            mech_power_w += p_motor.max(0.0);
            
            total_cruise_thrust += thrust_magnitude;
            
            let thrust_vector = mount.direction * thrust_magnitude;
            applied_forces.push((thrust_vector, mount.position));
        }
        
        // Cap total VTOL+cruise thrust to avoid runaway acceleration
        let max_total_thrust = 1.3 * self.body.mass * 9.81;
        let total_thrust_cmd = total_vtol_thrust + total_cruise_thrust;
        if total_thrust_cmd > max_total_thrust {
            let cap_scale = (max_total_thrust / total_thrust_cmd).clamp(0.0, 1.0);
            total_vtol_thrust *= cap_scale;
            total_cruise_thrust *= cap_scale;
            // Scale all thrust forces (applied_forces before aero push is thrust only)
            for f in applied_forces.iter_mut() {
                // aero force is added later, so current entries are thrust; they will all be scaled here
                f.0 = f.0 * cap_scale;
            }
            // Scale power consistently
            mech_power_w *= cap_scale;
        }

        // Compute battery power delivery
        let eff = self.battery.cfg.motor_efficiency.max(0.1).min(1.0);
        let p_req = mech_power_w / eff;
    let (p_deliv, _v_bus, _i_bus) = self.battery.limit_electrical_power(p_req);
        // Optionally scale thrusts if power-limited
        let mut scale = 1.0;
        if self.battery.cfg.limit_thrust {
            scale = if p_req > 1e-6 { (p_deliv * eff / mech_power_w).clamp(0.0, 1.0) } else { 1.0 };
            if scale < 0.9999 {
                total_vtol_thrust *= scale;
                total_cruise_thrust *= scale;
                for f in applied_forces.iter_mut() {
                    f.0 = f.0 * scale;
                }
            }
        }

        // Draw energy
        self.battery.draw_electrical_power(p_deliv, dt);
        self.last_power = self.battery.telemetry.clone();
    self.last_power.power_limited = self.battery.cfg.limit_thrust && scale < 0.9999;

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
            altitude: -self.state.position.z,  // Convert NED Z to altitude (negative Z is up)
            forces: self.last_forces.clone(),
            power: self.last_power.clone(),
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
    pub forces: ForcesTelemetry,
    pub power: PowerTelemetry,
}