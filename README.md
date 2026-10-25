# MECO Simulation

Year-specific desktop physics and simulation support for MECO FRC C++ robot
projects. This repository owns the Jolt physics world, drive model, arena, and
game-piece simulation. It is optional and desktop-only; robot deployment uses
the matching MECO Base vendordep without Jolt.

The `wpilib-2026` branch is the 2026 implementation. The `wpilib-2027` branch
will track the 2027 WPILib/SystemCore API and toolchain separately. A year is
supported only after its clean consumer build and simulation checks pass. These
branches share source only when the code is genuinely compatible; artifacts
and vendordep descriptors are always year-specific.

The `adapters/wpilib-2026` folder builds the optional MECO Base `Drive` bridge.
The bridge creates a shared drive and arena simulation session on Base's fixed
step runtime. A local clean Base plus Simulation consumer links in Debug and
Release pass on Linux desktop. Windows CI consumer links and public Maven
resolution remain release gates.

## Build and test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

See [the simulation guide](docs/jolt-simulation.md) for the physics model,
configuration, and known limits.

## Install in a robot project

The intended 2026 setup is MECO Base 2026.2.0 followed by MECO Simulation
2026.0.4. This release adds a ground collider to the standalone 3D drive world
so the chassis suspension remains supported without a season arena. The Base
package alone remains sufficient for roboRIO builds.
Simulation adds Linux and Windows desktop libraries, so desktop projects need
both vendordeps. Do not add the Simulation package to a different WPILib year.

The release workflow builds both desktop platforms, builds matching split Base
archives on Linux, roboRIO, and Windows, and merges both vendordeps into a
temporary Maven repository. It links Linux and Windows Debug and Release
consumers before its optional GitHub Pages publish job. The selected Base ref
must contain the 2026.2.0 split package. The workflow resolves that ref once
and uses the same commit for each build, packaging, and consumer job. Publishing
merges the new Simulation coordinate into the existing Maven tree and verifies
deployed checksums and clean consumers.
Publishing requires a `v2026.*` tag, the explicit `publish` workflow input,
GitHub Pages configured for Actions, and a `github-pages` environment with
required reviewers configured in repository settings. The workflow exposes
that environment as the deployment gate; this repository still needs those
Pages and environment settings before its first release. 2027 uses a separate
year branch and package.

## Current compatibility

| WPILib year | Status | Platforms |
| --- | --- | --- |
| 2026 | Linux desktop consumer Debug/Release linked locally; Windows and public Maven checks pending | Linux and Windows desktop target |
| 2027/SystemCore | In development; no compatibility claim | Not yet validated |

# Simulation model configuration

Desktop practice defaults to `jolt-3d`, including the headless launcher. Set
`FRC_SIM_PHYSICS=jolt-planar` (or headless `--backend jolt-planar`) for planar
automation. Unset or empty `FRC_SIM_PHYSICS` selects 3D.

This guide covers model setup and limits. Defaults demonstrate software behavior;
use measured robot parameters for design studies.

## macOS model workflow

The standalone CMake project can be built on macOS with Xcode Command Line Tools,
CMake, and a C++20 compiler. From the repository root, run:

```sh
cmake -S . -B build/jolt-release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/jolt-release --parallel 2
ctest --test-dir build/jolt-release --output-on-failure
```

These portable model checks need no WPILib, Java, vendor libraries, or Gradle.

The optional Phoenix/WPILib adapter is built only for Linux and Windows desktop.
The portable physics build does not validate Phoenix sensors/odometry, command
scheduling, or mechanism IO adapters.

## Drivetrain

`base::DriveConfig` supplies mass (`massKg`), yaw inertia (`moi`), tire friction
(`cof`), module positions, wheel radius, drive/steer/coupling ratios, current
limits, inversion, and CANcoder offsets. `DriveConfig::simulation` supplies
bumper half dimensions, body-origin ride height, center of mass relative to that
origin, roll/pitch inertia, wheel/steer inertia, tire stiffness, and suspension.
Lengths are metres, mass kilograms, inertia kg m², and stiffness/damping use SI.
The portable `DrivePhysicsConfig` additionally exposes motor resistance, torque
constant, velocity constant, and friction voltage. Invalid/nonfinite physical
parameters throw during construction.

Planar mode fixes chassis Z, pitch, and roll. Full 3D uses four downward wheel
contact rays with spring/damper normal forces and longitudinal/lateral tire
forces capped by available friction. Motor current and wheel inertia affect
acceleration; applied forces act at contact points. The bumper is a box rigid
body. Suspension rays do not model tire volume, sidewall contact, or detailed
wheel geometry; inertia is supplied along body principal axes. This is a
simplified tire/suspension model, not a measured tire model or an articulated
robot model.

Phoenix remains responsible for native controls and estimated pose. Its desktop
transport, control, and odometry threads run asynchronously; whole-robot results
are not bitwise deterministic. Before constructing native modules, the desktop drivetrain verifies a
nondefault configuration write/readback using a temporary handle to its first
drive device. The handle is released before native ownership; normal module
configuration replaces the temporary zero output limits. A bounded 10-second
startup failure reports the device, bus, and transport status. The integration
test uses this same production startup path. Firmware status alone did not prove
configuration readiness in clean-runner tests. Initial probe transactions may retry while the vendor transport starts. A
failed native module configuration after the readiness message should be
investigated before trusting a run.

`Drive::Pose()` and `ResetPose()` access the estimator. Desktop
`SimulationTruth()` returns physical state; `ResetSimulation(pose)` places and
stops the chassis, clears wheel motion, and synchronizes sensors and estimator.
Use scene reset for a complete practice restart. A wall changes physical body
motion while wheels may continue slipping; estimator pose is never collision
clamped.

## Mechanisms

Pass `MotorSimulationConfig` to `MakeMotorIO(Mode::Sim, vendor, config, model)`.
The legacy inertia-only overload retains the free motor/flywheel load. Select
`Load::Arm` with `mass`, `armCOM`, total `outputInertia`, and
`armAngleOffset` (zero physical arm angle is horizontal). Select
`Load::Elevator` with `mass`, `drumRadius`, and base `outputInertia`; reflected
payload inertia is added automatically. Both support `initialPosition` and
optional `minimum`/`maximum` physical stops in **output-shaft radians**. Elevator
travel is shaft angle times drum radius. Controller mechanism positions remain
rotations, or metres when configured with drum circumference per output turn.

Motor reads are observational. The shared runtime advances registered simulated
plants; do not also call `StepSimulation` on an automatically registered adapter.
A standalone custom IO plant may use explicit `StepSimulation(dt)` with one
owner. Real and replay IO never register with the physics runtime. Loads include
idealized gravity, electrical drag, current limits, brake/coast, and hard travel
stops; articulated contacts, battery sag, backlash, and structural compliance
are outside this model. Mechanism gravity uses the configured joint frame and
does not automatically rotate with chassis pitch or roll.

## Arena and reusable pieces

`sim::Arena` owns its colliders and pieces and registers with the shared clock.
The clock and robot-truth provider must outlive it. Pass the robot-local center
of mass as the final constructor argument when it is offset from the body origin;
truth linear velocity is measured at the center of mass. Use `AddCollider(BodySpec)`
for static geometry, `Spawn(PieceSpec)` for spheres/boxes, and `Pieces()` for
snapshots. Piece mass, friction, restitution, dimensions, initial pose/velocity,
and rolling deceleration are configurable. Planar ground pieces are constrained;
airborne launches use 3D motion in either backend. Landing restores ground
behavior. The seed controls reproducible scene placement, not asynchronous
Phoenix scheduling.

`SetIntake` configures the default robot-relative box; `RegisterIntake` creates
additional volumes. Capture needs an active intake and free capacity. Captured
pieces leave the collision world and belong to one intake. `Launch` releases
one possessed piece from its robot-relative muzzle pose with muzzle-local
launch velocity, plus chassis center-of-mass velocity and the rotational
velocity from the center of mass to the muzzle. There is no launch when possession is empty. Connect activation and
launch commands to mechanism state in a season project; the base bindings are
illustrative controller actions.

The generic 16 m by 8 m arena uses a fixed blue-corner origin with bounds
`[0,16] × [0,8]` metres: +X downfield, +Y left, +Z up, and positive yaw
counterclockwise. Physical colliders and pieces share this frame. `Arena::Reset(seed)`
recreates pieces and clears intake state; `SimulationClock::Reset()` resets
registered plants and time. Season-specific field geometry and robot placement
belong in the consuming robot project.

## Timing and evidence

The consuming robot owns when it advances simulation time. Use one owner for
world steps and keep snapshot reads passive. Telemetry/rendering consumers must
not mutate the world from other threads.

The portable benchmark measures model/arena/Jolt work, excluding command
execution and telemetry. Run it on each intended development/practice machine;
CI runner results alone cannot establish that machine's deadline behavior.
Review p99 and individual misses, configuration, contacts, and body counts
alongside CPU/build metadata. See [the verification record](../artifacts/jolt/STATUS.md)
for retained successful and failed evidence. Simulation acceptance does not
establish hardware calibration, CAN timing, or physical safety.
