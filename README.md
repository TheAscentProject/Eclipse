# Eclipse Flight Dynamics Simulator

A six degree-of-freedom flight dynamics simulator for hybrid vertical takeoff and landing (VTOL) aircraft, implementing quaternion-based attitude dynamics and multi-mode flight control systems.

<img width="1024" height="1024" alt="0_1" src="https://github.com/user-attachments/assets/18a49973-654c-4258-aa35-5f2276ae7203" />

## Why the Name Eclipse

Eclipse represents the transitional moment between two states - much like hybrid VTOL aircraft eclipse the boundary between rotorcraft and fixed-wing flight. As a component of the [Ascent Project](https://theascentproject.info), Eclipse provides the computational foundation for understanding these liminal flight regimes where vertical lift gradually yields to aerodynamic surfaces, creating a brief eclipse of pure flight modes.

## Abstract

This repository contains a flight dynamics simulator designed for the analysis of hybrid VTOL aircraft configurations. The implementation employs quaternion kinematics to avoid gimbal lock singularities, fourth-order Runge-Kutta integration for numerical stability, and properly transformed coordinate frames following aerospace conventions (NED). The simulator has been validated to maintain numerical stability over extended simulation periods without accumulation of integration errors.

## Technical Implementation

### Mathematical Framework

The simulator solves the Newton-Euler equations for rigid body dynamics:

```
F = ma (translational)
M = Iω̇ + ω × (Iω) (rotational)
```

Quaternion kinematics are implemented following Diebel (2006):
```
q̇ = ½q ⊗ ω_q
```

Where renormalization is applied after each integration step to maintain unit norm.

### Aerodynamic Modeling

Wing aerodynamics employ finite-wing corrections to infinite-wing theory:
```
C_L = C_L_∞ · AR/(AR + 2/e)
C_D = C_D_0 + C_L²/(πeAR)
```

A continuous stall model prevents numerical discontinuities through smooth blending near stall angles.

### Control System Architecture

The control system implements cascaded PID loops with anti-windup protection:
- Inner loop: Angular rate control (200 Hz)
- Middle loop: Attitude stabilization (100 Hz)  
- Outer loop: Position/altitude hold (50 Hz)

Back-calculation anti-windup prevents integrator saturation during actuator limiting.

### Numerical Methods

Integration employs the classical RK4 scheme with adaptive quaternion normalization. All force summations occur in the body-fixed frame to maintain consistency, with appropriate transformations applied for inertial and wind-frame quantities.

## Validation Results

Following correction of coordinate frame conventions and control gain tuning:

- Position drift: < 0.01 m over 60 second hover
- Altitude regulation: ± 0.1 m steady-state error
- Attitude stability: < 0.1° oscillation amplitude
- Numerical stability: No NaN propagation over 10⁶ timesteps

## Build and Execution

```bash
# Standard debug build with assertions
cargo build

# Optimized build for performance analysis
cargo build --release

# Execute hover stability test
cargo run --release

# Alternative aircraft configuration
cargo run --release -- tiltrotor
```

## Repository Structure

```
src/
├── math/          Vector and quaternion operations
├── physics/       Rigid body dynamics and integration
├── aero/          Aerodynamic force and moment computation
├── control/       Flight control system implementation
├── prop/          Propulsion system models
└── sim/           Simulation executive and data logging
```

## References

- Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft Control and Simulation (3rd ed.)
- Diebel, J. (2006). Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors
- Beard, R. W., & McLain, T. W. (2012). Small Unmanned Aircraft: Theory and Practice

## License

MIT License - See LICENSE file for details
