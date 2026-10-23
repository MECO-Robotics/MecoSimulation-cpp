# Jolt simulation models

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

`Arena` owns field colliders and game pieces in a `SimulationClock` world. Supply a robot-truth callback for piece interactions; its body pose and center-of-mass velocity must use the same world coordinates. Pieces, intakes, and launches are generic model primitives; season field geometry and robot-command integration belong in the consumer.

## Limits

The model does not validate Phoenix or REV firmware, CAN timing, hardware calibration, or physical safety. Linux and Windows desktop builds are the vendordep support targets. Other platforms may compile the standalone CMake project but are not part of the published compatibility claim.
