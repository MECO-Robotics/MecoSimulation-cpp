# Native AutoDrive and simulated perception

The `2026.0.14` package supplies simulated sensing.
Native planning/control and object-detection IO/clustering belong to
MecoBase `2026.2.5`, usable on desktop and roboRIO. There is no Python
companion or `AutoDriveBridge` in the current package. The earlier dev.1
prototype validation is historical, not evidence for the current architecture.

`meco/sim/SimulatedPieceSensor.h` generates camera-coordinate 3D observations
from physical robot pose and floor-piece positions, with configurable mount,
range, FOV, occlusion, seeded dropout and angular noise. `SetMount()` supports
moving cameras without resetting random state; `ObserveFrame()` produces the
same typed frames accepted by Base's object-detection abstraction. Simulation
truth is used only to synthesize observations; projection uses estimated pose
and capture-time transforms in Base. Season applications own camera mounts,
field colliders, piece extraction, targets, controls, and match rules.

`SimulationPose.h` and `SimulatedVision.h` provide the shared floor-reference
and ideal AprilTag visibility helpers. None of these helpers steps physics.

[Practice-match control](practice-mode.md) supplies simulated DS/FMS inputs via
NT. AutoDrive's match context must still come through normal DriverStation APIs
and the application's season model, not simulation control topics.

Local Linux development packages are not public releases. A release requires
Linux/Windows native builds and clean consumers; local Linux validation does
not establish Windows or hardware acceptance.
