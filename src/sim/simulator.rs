use crate::sim::Aircraft;
use crate::config::{AircraftConfig, SimulationConfig};
use crate::control::ControlTarget;
use crate::math::Vec3;
use std::time::{Duration, Instant};

pub struct Simulator {
    pub aircraft: Aircraft,
    pub config: SimulationConfig,
    pub time: f64,
    pub running: bool,
    pub target: ControlTarget,
    pub telemetry_log: Vec<(f64, crate::sim::aircraft::AircraftTelemetry)>,
}

impl Simulator {
    pub fn new(aircraft_config: AircraftConfig, sim_config: SimulationConfig) -> Self {
        let mut aircraft = Aircraft::new(aircraft_config);
        
        let initial_pos = Vec3::new(
            sim_config.initial_position[0],
            sim_config.initial_position[1],
            sim_config.initial_position[2],
        );
        let initial_vel = Vec3::new(
            sim_config.initial_velocity[0],
            sim_config.initial_velocity[1],
            sim_config.initial_velocity[2],
        );
        
        aircraft.set_initial_state(initial_pos, initial_vel);
        
        // Set initial target based on test type
        let target = if sim_config.initial_altitude == 0.0 && sim_config.max_time > 200.0 {
            // Takeoff test configuration
            ControlTarget::takeoff_to_altitude(400.0, 5.0)
        } else {
            ControlTarget::hover_at_altitude(sim_config.initial_altitude)
        };
        
        Self {
            aircraft,
            config: sim_config,
            time: 0.0,
            running: false,
            target,
            telemetry_log: Vec::new(),
        }
    }

    pub fn start(&mut self) {
        self.running = true;
        self.time = 0.0;
        self.telemetry_log.clear();
        
        println!("Simulation started");
        println!("Aircraft: {}", self.aircraft.config.name);
        println!("Initial altitude: {:.1} m", self.config.initial_altitude);
        println!("Target mode: {:?}", self.target.mode);
    }

    pub fn step(&mut self) -> bool {
        if !self.running || self.time >= self.config.max_time {
            return false;
        }

        let start_time = if self.config.real_time {
            Some(Instant::now())
        } else {
            None
        };

        // Check for mode transitions
        if self.target.mode == crate::control::FlightMode::Takeoff {
            let current_altitude = -self.aircraft.state.position.z;
            if let Some(target_altitude) = self.target.altitude {
                if current_altitude >= target_altitude - 5.0 {
                    println!("Reached target altitude, switching to hover mode");
                    self.target = ControlTarget::hover_at_altitude(target_altitude);
                }
            }
        }
        
        self.aircraft.update(&self.target, self.config.dt);
        self.time += self.config.dt;
        
        let telemetry = self.aircraft.get_telemetry();
        self.telemetry_log.push((self.time, telemetry.clone()));
        
        if self.time.fract() < self.config.dt {
            self.print_status(&telemetry);
        }
        
        if let Some(start) = start_time {
            let elapsed = start.elapsed();
            let target_duration = Duration::from_secs_f64(self.config.dt);
            if elapsed < target_duration {
                std::thread::sleep(target_duration - elapsed);
            }
        }

        if telemetry.altitude > 10.0 {
            println!("Simulation stopped: Aircraft crashed (altitude > 10m below ground)");
            self.running = false;
            return false;
        }

        true
    }

    pub fn run(&mut self) {
        self.start();
        
        while self.step() {
            if !self.running {
                break;
            }
        }
        
        self.finish();
    }

    pub fn finish(&mut self) {
        self.running = false;
        
        println!("\nSimulation completed");
        println!("Total time: {:.1} s", self.time);
        println!("Total telemetry points: {}", self.telemetry_log.len());
        
        if let Some((_, final_telemetry)) = self.telemetry_log.last() {
            println!("Final state:");
            println!("  Position: ({:.1}, {:.1}, {:.1}) m", 
                final_telemetry.position.x, 
                final_telemetry.position.y, 
                final_telemetry.position.z);
            println!("  Velocity: {:.1} m/s", final_telemetry.airspeed);
            println!("  Altitude: {:.1} m", final_telemetry.altitude);
        }
    }


    fn print_status(&self, telemetry: &crate::sim::aircraft::AircraftTelemetry) {
        let (roll, pitch, yaw) = telemetry.attitude;
        println!(
            "t={:6.1}s | Alt={:6.1}m | Vel={:6.1}m/s | RPY=({:5.1}°,{:5.1}°,{:5.1}°)",
            self.time,
            telemetry.altitude,
            telemetry.airspeed,
            roll.to_degrees(),
            pitch.to_degrees(),
            yaw.to_degrees()
        );
        println!(
            "         | Thrust: VTOL={:6.1}N Cruise={:6.1}N | Wind={:5.2},{:5.2},{:5.2} m/s",
            telemetry.forces.total_thrust_vtol,
            telemetry.forces.total_thrust_cruise,
            telemetry.forces.wind_force.x,
            telemetry.forces.wind_force.y,
            telemetry.forces.wind_force.z
        );
    }

    pub fn export_telemetry(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        use std::fs::File;
        use std::io::Write;
        
        let mut file = File::create(filename)?;
        writeln!(file, "time,x,y,z,vx,vy,vz,roll,pitch,yaw,wx,wy,wz,airspeed,altitude")?;
        
        for (time, telem) in &self.telemetry_log {
            let (roll, pitch, yaw) = telem.attitude;
            writeln!(
                file,
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                time,
                telem.position.x, telem.position.y, telem.position.z,
                telem.velocity.x, telem.velocity.y, telem.velocity.z,
                roll, pitch, yaw,
                telem.angular_velocity.x, telem.angular_velocity.y, telem.angular_velocity.z,
                telem.airspeed, telem.altitude
            )?;
        }
        
        println!("Telemetry exported to {}", filename);
        Ok(())
    }
}