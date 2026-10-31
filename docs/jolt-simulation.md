# Jolt simulation models

Start with [simulation basics](simulation-basics.md) for a short overview.

![Simulation physics produces sensor readings and shares battery voltage](visuals/simulation-basics.png)


This repository builds the Jolt world, drivetrain model, and generic arena as a standalone C++ library. Its optional WPILib 2026 artifact connects a MECO Base drive and arena to Base's fixed-step runtime. The vendordep targets Linux and Windows desktop; it does not add a roboRIO library. A clean Linux Base plus Simulation consumer links locally in Debug and Release. Windows CI links and public Maven resolution remain release gates.

## Build and test

Run from the repository root with CMake and a C++20 compiler:

```sh
cmake -S . -B build/Release -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build/Release --config Release --parallel 2
ctest --test-dir build/Release -C Release --output-on-failure
```

The physics library has two backends. `jolt-3d` models vertical motion and full rigid-body contact. `jolt-planar` constrains the chassis to a plane. The backend selection helper reads `FRC_SIM_PHYSICS`, defaulting to `jolt-3d` when unset or empty. Construct `SimulationWorld` directly with a `Backend` when a tool or test needs an explicit choice.

## Drive and arena models

`DrivePhysics` consumes four module voltage pairs and reports body pose, velocity, and module state. `DrivePhysicsConfig` uses SI units and accepts the robot geometry, mass properties, gearing, wheel parameters, tire coefficients, and current limits. Defaults are illustrative; use measured robot values for studies.

The WPILib bridge reads Phoenix applied voltages and feeds back Jolt rotor,
CANcoder and gyro measurements. Do not call Phoenix `UpdateSimState()` in this
path: it advances a second motor plant, overwrites the measurements and resets
simulation device orientations. Phoenix firmware runs asynchronously without that
helper. Jolt is the only physical state owner. This matches the external-plant
boundary used by [MapleSim's CTRE adapter](https://github.com/Shenzhen-Robotics-Alliance/CTRE-Swerve-MapleSim).

The WPILib drive adapter adds a static ground slab at Z=0 to its otherwise empty world. This lets the 3D rigid wheel contacts support the chassis even when no season `Arena` is attached. Season field and game-piece colliders remain the consumer's responsibility.

`Arena` owns field colliders and game pieces in a `SimulationClock` world. Supply a robot-truth callback for piece interactions; its body pose and center-of-mass velocity must use the same world coordinates. Pieces, intakes, and launches are generic model primitives; season field geometry and robot-command integration belong in the consumer.

## Wheel contact and steering load

- Full3D uses fixed, hard-mounted wheel contact. Supporting terrain produces normal load; a lifted wheel cannot provide ground traction.
- Load-dependent azimuth estimates steering resistance from a finite contact patch. Wheel and tire parameters must be supplied for the robot; the result is not a measured carpet model.
- Numerical contact tolerances are solver settings, not suspension travel or measured tire compliance.
- For bumper foam and rigid backing, see [configurable contact](compliant-bumper-contact.md).

## Shared battery

Call [`UpdateBatterySupply`](../adapters/wpilib-2026/include/meco/sim/BatterySupply.h) once after a shared runtime period. It uses battery-side supply current to update bus and roboRIO voltage for the next period. The default 12 V and 20 mΩ are estimates; the half-step voltage smoothing is numerical stabilization, not a measured battery time constant. State of charge, regeneration, and branch losses are not modeled.

## Limits

The model does not validate Phoenix or REV firmware, CAN timing, hardware calibration, or physical safety. Linux and Windows desktop builds are the vendordep support targets. Other platforms may compile the standalone CMake project but are not part of the published compatibility claim.

The REBUILT depot rails retain their field dimensions. Their painted-steel contact uses a provisional sliding coefficient of 0.05; Jolt combines it with the robot contact material by geometric mean. This allows a bumper to slide up the rail under drive force, and is not a measured material calibration.
