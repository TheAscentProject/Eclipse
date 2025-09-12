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
        let target = if sim_config.initial_altitude == 0.0 && sim_config.max_time >= 200.0 {
            // Takeoff test configuration
            ControlTarget::takeoff_to_altitude(120.0, 5.0)
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

        if telemetry.altitude < -1.0 {  // Crashed if below ground (negative altitude)
            println!("\nCRASH: Aircraft hit ground");
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
        println!(); // New line after status updates
    }


    fn print_status(&self, telemetry: &crate::sim::aircraft::AircraftTelemetry) {
        // Clear previous line and print clean status
        print!("\r\x1B[K"); // Clear line
        print!("t={:6.1}s | Alt={:6.1}m | Vel={:5.1}m/s | Thrust={:5.0}N", 
            self.time,
            telemetry.altitude,
            telemetry.airspeed,
            telemetry.forces.total_thrust_vtol
        );
        
        // Add stability indicator
        let (roll, pitch, _) = telemetry.attitude;
        let attitude_deg = (roll.to_degrees().abs() + pitch.to_degrees().abs()) / 2.0;
        let stability = if attitude_deg < 2.0 { " [STABLE]" } else if attitude_deg < 10.0 { " [WOBBLE]" } else { " [UNSTABLE]" };
        print!("{}", stability);
        
        use std::io::{self, Write};
        io::stdout().flush().unwrap();
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