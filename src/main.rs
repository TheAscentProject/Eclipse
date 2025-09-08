mod math;
mod physics;
mod aero;
mod control;
mod config;
mod sim;

use config::{AircraftConfig, SimulationConfig};
use sim::{Simulator, Visualization};
use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();
    
    let aircraft_config = if args.len() > 1 && args[1] == "tiltrotor" {
        AircraftConfig::tiltrotor_quad()
    } else {
        AircraftConfig::lift_plus_cruise()
    };
    
    let sim_config = if args.len() > 2 && args[2] == "transition" {
        SimulationConfig::transition_test()
    } else {
        SimulationConfig::hover_test()
    };
    
    println!("Eclipse Flight Dynamics Simulator");
    println!("Aircraft: {}", aircraft_config.name);
    println!("Configuration: {:?} test", if args.len() > 2 && args[2] == "transition" { "Transition" } else { "Hover" });
    println!();
    
    let mut simulator = Simulator::new(aircraft_config, sim_config);
    
    simulator.run();
    
    if let Err(e) = simulator.export_telemetry("flight_data.csv") {
        eprintln!("Failed to export telemetry: {}", e);
    }
    
    if args.contains(&"--viz".to_string()) {
        println!("Starting visualization...");
        Visualization::run_visualization(simulator.telemetry_log);
    }
    
    println!("Simulation complete");
}
