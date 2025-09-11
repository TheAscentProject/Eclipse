mod math;
mod physics;
mod aero;
mod control;
mod config;
mod sim;

use config::{AircraftConfig, SimulationConfig};
use sim::{Simulator, Visualization};
use std::env;

#[derive(Debug)]
enum FlightMode {
    Hover,
    Takeoff,
}

impl FlightMode {
    fn from_args() -> Self {
        let args: Vec<String> = env::args().collect();
        
        for arg in &args {
            match arg.as_str() {
                "--hover" => return FlightMode::Hover,
                "--takeoff" => return FlightMode::Takeoff,
                _ => continue,
            }
        }
        
        // Default to hover if no mode specified
        FlightMode::Hover
    }
    
    fn get_config(&self) -> SimulationConfig {
        match self {
            FlightMode::Hover => SimulationConfig::hover_test(),
            FlightMode::Takeoff => SimulationConfig::takeoff_test(),
        }
    }
    
    fn name(&self) -> &'static str {
        match self {
            FlightMode::Hover => "Hover",
            FlightMode::Takeoff => "Takeoff",
        }
    }
    
    fn description(&self) -> &'static str {
        match self {
            FlightMode::Hover => "Hold stable position at 5m altitude",
            FlightMode::Takeoff => "Climb from ground level to 400m altitude",
        }
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    
    // Check for help
    if args.contains(&"--help".to_string()) || args.contains(&"-h".to_string()) {
        print_help();
        return;
    }
    
    // Default aircraft (can be overridden with --tiltrotor)
    let aircraft_config = if args.contains(&"--tiltrotor".to_string()) {
        AircraftConfig::tiltrotor_quad()
    } else {
        AircraftConfig::lift_plus_cruise()
    };
    
    let flight_mode = FlightMode::from_args();
    let sim_config = flight_mode.get_config();
    
    println!("Eclipse Flight Dynamics Simulator");
    println!("Aircraft: {}", aircraft_config.name);
    println!("Flight Mode: {} - {}", flight_mode.name(), flight_mode.description());
    println!();
    
    let mut simulator = Simulator::new(aircraft_config, sim_config);
    simulator.run();
    
    // Generate timestamped filename with flight mode
    let timestamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_else(|_| std::time::Duration::from_secs(0))
        .as_secs();
    let filename = format!("{}_flight_{}.csv", flight_mode.name().to_lowercase(), timestamp);
    
    if let Err(e) = simulator.export_telemetry(&filename) {
        eprintln!("Failed to export telemetry: {}", e);
    }
    
    if args.contains(&"--viz".to_string()) {
        println!("Starting visualization...");
        Visualization::run_visualization(simulator.telemetry_log);
    }
    
    println!("Simulation complete");
}

fn print_help() {
    println!("Eclipse Flight Dynamics Simulator");
    println!();
    println!("USAGE:");
    println!("    cargo run [FLAGS] [MODE]");
    println!();
    println!("FLIGHT MODES:");
    println!("    --hover      Hold stable position at 5m altitude (default)");
    println!("    --takeoff    Climb from ground level to 400m altitude");
    println!();
    println!("FLAGS:");
    println!("    --tiltrotor  Use tiltrotor aircraft instead of lift+cruise");
    println!("    --viz        Show 3D visualization after simulation");
    println!("    --help, -h   Show this help message");
    println!();
    println!("EXAMPLES:");
    println!("    cargo run --hover");
    println!("    cargo run --takeoff");
    println!("    cargo run --takeoff --tiltrotor");
    println!("    cargo run --hover --viz");
}
