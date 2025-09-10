use eclipse::{
    config::{AircraftConfig, SimulationConfig},
    sim::Simulator,
};

fn main() {
    let aircraft = AircraftConfig::lift_plus_cruise();
    let mut sim_config = SimulationConfig::hover_test();
    sim_config.duration = 10.0;
    sim_config.dt = 0.001;
    
    let mut simulator = Simulator::new(aircraft, sim_config);
    simulator.run();
    
    println!("t,x,y,z,u,v,w,qw,qx,qy,qz,p,q,r,altitude");
    for frame in simulator.telemetry_log.iter().step_by(100) {
        println!(
            "{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.4},{:.4},{:.4},{:.4},{:.3},{:.3},{:.3},{:.3}",
            frame.time,
            frame.position.x, frame.position.y, frame.position.z,
            frame.velocity.x, frame.velocity.y, frame.velocity.z,
            frame.orientation.0, frame.orientation.1, frame.orientation.2, frame.orientation.3,
            frame.angular_velocity.x, frame.angular_velocity.y, frame.angular_velocity.z,
            -frame.position.z
        );
    }
    
    let final_state = simulator.telemetry_log.last().unwrap();
    let altitude_error = (-final_state.position.z - 10.0).abs();
    let velocity_mag = (final_state.velocity.x.powi(2) + 
                       final_state.velocity.y.powi(2) + 
                       final_state.velocity.z.powi(2)).sqrt();
    
    eprintln!("\nHover Test Results:");
    eprintln!("Final altitude: {:.2} m (target: 10.0 m)", -final_state.position.z);
    eprintln!("Altitude error: {:.3} m", altitude_error);
    eprintln!("Final velocity: {:.3} m/s", velocity_mag);
    eprintln!("Test: {}", if altitude_error < 0.5 && velocity_mag < 0.5 { "PASS" } else { "FAIL" });
}