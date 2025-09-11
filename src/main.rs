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
    
    let sim_config = if args.len() > 2 {
        match args[2].as_str() {
            "transition" => SimulationConfig::transition_test(),
            "takeoff" => SimulationConfig::takeoff_test(),
            _ => SimulationConfig::hover_test(),
        }
    } else {
        SimulationConfig::hover_test()
    };
    
    println!("Eclipse Flight Dynamics Simulator");
    println!("Aircraft: {}", aircraft_config.name);
    let test_name = if args.len() > 2 {
        match args[2].as_str() {
            "transition" => "Transition",
            "takeoff" => "Takeoff",
            _ => "Hover"
        }
    } else {
        "Hover"
    };
    println!("Configuration: {} test", test_name);
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
