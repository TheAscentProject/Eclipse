# Eclipse Flight Dynamics Simulator

[![Sponsor](https://img.shields.io/badge/Sponsor-Patreon-red)](https://www.patreon.com/14815116/join)

A modular six degree-of-freedom flight dynamics simulator for hybrid vertical takeoff and landing (VTOL) aircraft, implementing hierarchical flight control with gain scheduling and smooth mode transitions.

## Project Status

**Current Phase: Control System Architecture Complete**

The simulator has a fully implemented modular architecture with advanced flight control systems. The physics engine is stable and validated. Control gains require further tuning for stable flight across all regimes.

### Working Components
- Complete 6-DOF rigid body dynamics with quaternion attitude representation
- Runge-Kutta 4 numerical integration (stable, no NaN propagation)
- Modular aerodynamics (wing theory, propeller models, atmosphere)
- Hierarchical control system with gain scheduling
- Control allocation matrix for mixed VTOL/cruise propulsion
- Smooth transition management between flight modes
- CSV telemetry export with organized flight logs

### In Development
- Control gain tuning for stable hover and transition
- Extended flight envelope testing
- Wind and turbulence models

## Architecture

The simulator implements a professional VTOL control architecture:

```
High Level: Flight Mode Manager (hover, transition, cruise)
    ↓
Mid Level:  Gain Scheduling (airspeed-based PID adaptation)
    ↓
Low Level:  Control Allocation (optimal actuator distribution)
    ↓
Hardware:   Motor Commands (VTOL fans + cruise propellers)
```

### Key Systems

**Gain Scheduling** (`control/gain_schedule.rs`)
- Separate PID gains for hover, transition, and cruise regimes
- Smooth interpolation based on airspeed
- Prevents control discontinuities during mode changes

**Control Allocation** (`control/control_allocation.rs`)
- Maps desired forces/moments to actuator commands
- Handles over/under-actuated configurations
- Optimal distribution via pseudo-inverse

**Transition Manager** (`control/transition_manager.rs`)
- State machine for flight mode transitions
- Automatic mode switching based on conditions
- Smooth blending factors for propulsion systems

## Running Instructions

### Basic Commands

```bash
# Build the simulator (optimized)
cargo build --release

# Run hover test at 5m altitude
cargo run --release -- --hover

# Run takeoff to 400m altitude
cargo run --release -- --takeoff

# Use tiltrotor configuration
cargo run --release -- --tiltrotor --hover

# Help and options
cargo run --release -- --help
```

During runs, the status line shows power metrics:

```
t=  12.0s | Alt=  25.4m | Vel=  3.1m/s | Thrust=  982N | V=22.8V I=45.2A P=1029W SoC= 78%
```

### Flight Modes

- `--hover`: Maintain stable position at 5m altitude (default)
- `--takeoff`: Climb from ground to 400m altitude
- `--tiltrotor`: Use tiltrotor aircraft instead of lift+cruise configuration

### Output

Flight data is automatically saved to timestamped CSV files:
- Hover flights: `flights/hover/flight_[timestamp].csv`
- Takeoff flights: `flights/takeoff/flight_[timestamp].csv`

Each CSV contains:
- Time, position (x,y,z), velocity (vx,vy,vz)
- Attitude (roll,pitch,yaw), angular rates (wx,wy,wz)
- Airspeed, altitude, control outputs
 - Power: `voltage`, `current`, `power_w`, `energy_wh`, `soc`, `time_remaining_s`

## Repository Structure

```
src/
├── math/               # Vector3, Quaternion, coordinate transforms
├── physics/            # 6-DOF dynamics, RK4 integration
├── aero/              # Wing/propeller models, atmosphere
├── control/           # Control system modules
│   ├── pid.rs         # PID controller with anti-windup
│   ├── flight_mode.rs # Flight mode definitions
│   ├── gain_schedule.rs    # Airspeed-based gain scheduling
│   ├── control_allocation.rs # Actuator allocation matrix
│   ├── transition_manager.rs # Mode transition logic
│   ├── autopilot_v2.rs     # Main autopilot implementation
│   └── autopilot.rs   # Legacy autopilot (deprecated)
├── config/            # Aircraft and simulation configs
└── sim/               # Simulation engine, telemetry
```

## Aircraft Configurations

### Lift+Cruise (Default)
- 4 vertical lift fans for VTOL operations
- 1 forward propeller for cruise flight
- Blown wing for enhanced low-speed lift
- Optimal for efficiency in forward flight

### Tiltrotor
- 4 tiltable propellers
- Transition from vertical to horizontal thrust
- Shared propulsion for all flight modes
- Higher mechanical complexity

## Technical Implementation

### Physics Engine
- Newton-Euler rigid body equations
- Quaternion kinematics (no gimbal lock)
- Body-fixed reference frame with NED convention
- Forces computed and summed in body frame

### Aerodynamics
- Finite wing corrections with aspect ratio effects
- Continuous stall model for smooth transitions
- Propeller momentum theory for hover
- Blade element theory for forward flight

### Power & Battery Model

- LiPo-style battery model with configurable pack:
    - `capacity_mah`, `cells`, `c_rating`
    - `internal_resistance_mohm_per_cell`, `cutoff_per_cell_v`
    - Default is a 6S 10,000 mAh pack with moderate IR
- Voltage sag computed from open-circuit voltage minus I·R drop
- Discharge curve approximates flat mid-SoC and knees at ends
- Instantaneous power draw derived from motor thrust demand; electrical draw accounts for efficiency
- Thrust is scaled when power is limited by voltage sag, current limit, or cutoff
- Simulation ends cleanly when the pack is depleted (voltage below cutoff or energy exhausted)

### Control Philosophy
- Single coherent control system with mode adaptation
- No hard switching between control laws
- Gain scheduling provides smooth parameter variation
- Control allocation handles actuator redundancy

## Known Issues

1. **Control Instability**: Current PID gains cause oscillations in hover mode. The control architecture is correct but requires parameter tuning.

2. **Thrust Scaling**: Base thrust values may not match aircraft mass correctly, leading to inadequate lift.

3. **Visualization**: Bevy-based 3D visualization has API compatibility issues with current version.

## Development Roadmap

Near term:
- Systematic gain tuning using frequency domain analysis
- Implement auto-tuning for PID parameters
- Add wind disturbance models
- Create automated test scenarios

Long term:
- Linear Quadratic Regulator (LQR) control
- Model Predictive Control (MPC) for transitions
- Hardware-in-the-loop simulation support
- Multi-vehicle coordination

## Building from Source

Requirements:
- Rust 1.70 or later
- No additional system dependencies for core simulator

```bash
# Clone repository
cd eclipse

# Run tests
cargo test

# Build with optimizations
cargo build --release

# Run with debug output
RUST_LOG=debug cargo run
```

## Battery Configuration

Battery parameters live in the aircraft config. Defaults are provided for built-in configurations. To tweak the pack, edit `src/config/aircraft.rs` and adjust the `battery` field on `AircraftConfig` (e.g., capacity, cells, C-rating). Example snippet:

```rust
Self {
    battery: crate::power::BatteryConfig {
        capacity_mah: 8000.0,
        cells: 6,
        c_rating: 25.0,
        internal_resistance_mohm_per_cell: 2.0,
        cutoff_per_cell_v: 3.2,
        nominal_cell_v: 3.7,
        full_cell_v: 4.2,
        motor_efficiency: 0.9,
    },
}
```

Telemetry now includes energy and remaining time estimates for quick flight-time checks.

## Sponsorship

Support the development of Eclipse Flight Dynamics Simulator:

**[https://www.patreon.com/14815116/join](https://www.patreon.com/14815116/join)**

Your sponsorship helps maintain and expand this open-source flight dynamics research platform.

## Contributing

The project follows a modular architecture where each system has a single responsibility. When contributing:

1. Maintain clean separation between physics, aerodynamics, and control
2. Use proper aerospace conventions (NED frame, quaternion normalization)
3. Add unit tests for mathematical functions
4. Document control parameters and their effects

## References 

- Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft Control and Simulation (3rd ed.)
- Diebel, J. (2006). Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors
- Beard, R. W., & McLain, T. W. (2012). Small Unmanned Aircraft: Theory and Practice

## License

MIT License - See LICENSE file for details
