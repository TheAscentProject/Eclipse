mod math;
mod physics;
mod aero;
mod control;
mod config;
mod sim;

use config::{AircraftConfig, SimulationConfig};
use sim::{Simulator, Visualization};
use std::env;
use std::io::{self, Write};

#[derive(Debug, Clone)]
enum FlightMode {
    Hover,
    Takeoff,
    Transition,
    Auto,
}

impl FlightMode {
    fn all_modes() -> Vec<FlightMode> {
        vec![FlightMode::Hover, FlightMode::Takeoff, FlightMode::Transition, FlightMode::Auto]
    }
    
    fn from_input(input: &str) -> Option<Self> {
        match input.trim() {
            "1" => Some(FlightMode::Hover),
            "2" => Some(FlightMode::Takeoff),
            "3" => Some(FlightMode::Transition),
            "4" => Some(FlightMode::Auto),
            _ => None,
        }
    }
    
    fn get_config(&self) -> SimulationConfig {
        match self {
            FlightMode::Hover => SimulationConfig::hover_test(),
            FlightMode::Takeoff => SimulationConfig::takeoff_test(),
            FlightMode::Transition => SimulationConfig::transition_test(),
            FlightMode::Auto => SimulationConfig::auto_test(),
        }
    }
    
    fn name(&self) -> &'static str {
        match self {
            FlightMode::Hover => "Hover",
            FlightMode::Takeoff => "Takeoff",
            FlightMode::Transition => "Transition",
            FlightMode::Auto => "Auto",
        }
    }
    
    fn description(&self) -> &'static str {
        match self {
            FlightMode::Hover => "Hold stable position at 5m altitude",
            FlightMode::Takeoff => "Climb from ground level to 400m altitude", 
            FlightMode::Transition => "Forward flight with transition between hover and cruise",
            FlightMode::Auto => "Automated waypoint navigation",
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
    
    // Show interactive menu if no direct arguments
    let flight_mode = if args.len() == 1 {
        select_flight_mode()
    } else {
        // Legacy command line support
        match args.get(1).map(|s| s.as_str()) {
            Some("hover") => FlightMode::Hover,
            Some("takeoff") => FlightMode::Takeoff,
            Some("transition") => FlightMode::Transition,
            Some("auto") => FlightMode::Auto,
            _ => select_flight_mode(),
        }
    };
    
    // Select aircraft
    let aircraft_config = if args.contains(&"--tiltrotor".to_string()) {
        AircraftConfig::tiltrotor_quad()
    } else {
        AircraftConfig::lift_plus_cruise()
    };
    
    let sim_config = flight_mode.get_config();
    
    // Clear screen and show clean header
    print!("\x1B[2J\x1B[1;1H");
    println!("Eclipse Flight Simulator");
    println!("========================");
    println!("Flight Mode: {}", flight_mode.name());
    println!("Aircraft: {}", aircraft_config.name);
    println!("Duration: {:.0}s", sim_config.max_time);
    println!();
    
    let mut simulator = Simulator::new(aircraft_config, sim_config.clone());
    let start_time = std::time::Instant::now();
    
    simulator.run();
    
    let elapsed = start_time.elapsed();
    
    // Calculate max altitude reached
    let max_altitude = simulator.telemetry_log.iter()
        .map(|(_, telem)| telem.altitude)
        .fold(f64::NEG_INFINITY, f64::max);
    
    // Determine outcome
    let outcome = determine_outcome(&simulator, &sim_config);
    
    // Show clean summary
    println!("\n┌─────────────────────────┐");
    println!("│     FLIGHT SUMMARY      │");
    println!("├─────────────────────────┤");
    println!("│ Duration: {:>6.1}s      │", simulator.time);
    println!("│ Max Alt:  {:>6.1}m      │", max_altitude);
    println!("│ Outcome:  {:>11}   │", outcome);
    println!("└─────────────────────────┘");
    
    // Export telemetry
    let timestamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_else(|_| std::time::Duration::from_secs(0))
        .as_secs();
    let filename = format!("flights/{}/flight_{}.csv", flight_mode.name().to_lowercase(), timestamp);
    
    if let Err(e) = simulator.export_telemetry(&filename) {
        eprintln!("WARNING: Failed to export telemetry: {}", e);
    }
    
    if args.contains(&"--viz".to_string()) {
        println!("Starting visualization...");
        Visualization::run_visualization(simulator.telemetry_log);
    }
}

fn select_flight_mode() -> FlightMode {
    println!("Eclipse Flight Simulator");
    println!("========================");
    println!();
    println!("Select a flight mode:");
    println!();
    
    let modes = FlightMode::all_modes();
    for (i, mode) in modes.iter().enumerate() {
        println!("  {}. {} - {}", i + 1, mode.name(), mode.description());
    }
    
    println!();
    print!("Enter choice (1-{}): ", modes.len());
    io::stdout().flush().unwrap();
    
    loop {
        let mut input = String::new();
        match io::stdin().read_line(&mut input) {
            Ok(_) => {
                if let Some(mode) = FlightMode::from_input(&input) {
                    return mode;
                } else {
                    print!("Invalid choice. Enter 1-{}: ", modes.len());
                    io::stdout().flush().unwrap();
                }
            }
            Err(_) => {
                print!("Invalid input. Enter 1-{}: ", modes.len());
                io::stdout().flush().unwrap();
            }
        }
    }
}

fn determine_outcome(simulator: &Simulator, sim_config: &SimulationConfig) -> &'static str {
    let final_altitude = simulator.telemetry_log.last()
        .map(|(_, telem)| telem.altitude)
        .unwrap_or(0.0);
    
    // Check if crashed (altitude too high means below ground)
    if final_altitude > 10.0 {
        return "CRASHED";
    }
    
    // Check if reached max time
    if simulator.time >= sim_config.max_time - 1.0 {
        return "SUCCESS";
    }
    
    // Check stability for hover tests
    if sim_config.initial_altitude > 0.0 {
        let last_10_seconds: Vec<_> = simulator.telemetry_log.iter()
            .filter(|(t, _)| *t > simulator.time - 10.0)
            .collect();
        
        if !last_10_seconds.is_empty() {
            let alt_variation = last_10_seconds.iter()
                .map(|(_, telem)| telem.altitude)
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), alt| {
                    (min.min(alt), max.max(alt))
                });
            
            if (alt_variation.1 - alt_variation.0) < 1.0 {
                return "STABLE";
            }
        }
    }
    
    "UNSTABLE"
}

fn print_help() {
    println!("Eclipse Flight Simulator");
    println!("========================");
    println!();
    println!("USAGE:");
    println!("    cargo run [mode] [flags]");
    println!();
    println!("MODES:");
    println!("    hover        Hold stable position at 5m altitude");
    println!("    takeoff      Climb from ground level to 400m altitude");
    println!("    transition   Forward flight with hover-to-cruise transition");
    println!("    auto         Automated waypoint navigation");
    println!();
    println!("FLAGS:");
    println!("    --tiltrotor  Use tiltrotor aircraft instead of lift+cruise");
    println!("    --viz        Show 3D visualization after simulation");
    println!("    --help, -h   Show this help message");
    println!();
    println!("EXAMPLES:");
    println!("    cargo run              # Interactive mode");
    println!("    cargo run hover        # Direct hover test");
    println!("    cargo run takeoff      # Takeoff test");
    println!("    cargo run transition --tiltrotor  # Transition with tiltrotor");
}
