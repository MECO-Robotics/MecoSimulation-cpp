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
step runtime. The Simulation vendordep exports its own physics and adapter
headers; the matching Base vendordep supplies `base/...` headers.

`base/sim/ColliderTerrain.h` derives plan-view bounds and grades for static
box plates pitched around the field Y axis. `TerrainBoundaries` splits a path
at plate edges, and `TerrainGrade` samples its signed grade at the segment
midpoint. Applications choose colliders, obstacle margins, and terrain policy.
`meco/sim/ColliderNavigation.h` provides `ColliderRectangles`, which projects unrotated boxes intersecting a caller-supplied
height interval into field-coordinate rectangles; other shapes are omitted.

## Find a guide

| I want to… | Start here | More detail |
| --- | --- | --- |
| Understand desktop physics | [Simulation basics](docs/simulation-basics.md) | [Jolt model and limits](docs/jolt-simulation.md) |
| Configure bumper contact | [Contact quick start](docs/compliant-bumper-contact.md#start-here) | [Contact solver and telemetry](docs/compliant-bumper-contact.md) |
| Run a simulated practice match | [Practice quick start](docs/practice-mode.md#quick-start) | [NetworkTables contract](docs/practice-mode.md#nt-contract) |
| Simulate object observations | [Simulated perception](docs/autodrive.md) | [Piece sensor API](adapters/wpilib-2026/include/meco/sim/SimulatedPieceSensor.h) |

## Learn the simulator

Start with [simulation basics](docs/simulation-basics.md) for physics, battery, and practice matches.

## Build and test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

See [the simulation guide](docs/jolt-simulation.md) for the physics model,
configuration, and known limits.

## Install in a robot project

Release artifacts are built locally for Linux desktop. Public deployment and
consumer checks run for production tags; prerelease tags with a `-` suffix skip
those checks. The package does not provide a Windows desktop library. Previously
published Windows archives remain available.

With clean Simulation and compatible Base checkouts and the WPILib JDK, Linux
build tools, and CMake installed, run
`python3 scripts/build_local_release.py build --base-root <base-checkout>`,
then `python3 scripts/build_local_release.py assemble --base-root <base-checkout>`.
The packager validates the local build provenance and writes `build/vendor-maven`.

For each release, keep the descriptor version and `jsonUrl`, package and
verifier version constants, and compatible Base version in provenance checks in
sync. Use a new immutable version and tag; the existing 2026.0.21 coordinate
remains unchanged.

Create a release tag for the same commit, then package and attach the local
Maven tree to a GitHub release (creating the release itself is a separate step):

```sh
python3 scripts/local_release_bundle.py pack --package build/vendor-maven --bundle build/vendordep.zip
gh release upload "$TAG" build/vendordep.zip
gh workflow run publish-local-package.yml --ref wpilib-2026 -f release_tag="$TAG"
```

The publication job verifies the package and preserves existing Maven
coordinates. Production checks validate the deployed package and public
consumer links. A skipped or failed check remains unverified.

For prereleases published through the raw Maven branch, use
`publish-raw-maven.yml`. It adds only the new Simulation coordinate to the
`maven-repository` branch and avoids rebuilding the full Pages site. Keep the
Pages publication workflow for production releases.

The published release setup used MECO Base 2026.2.1 followed by MECO Simulation
2026.0.10. The current prerelease pair is MECO Base 2026.2.17-beta.5 and MECO
Simulation 2026.0.23-beta.3. It includes explicit robot contact profiles and
generic bumper dimensions; use these matching packages together rather than
mixing the new Base layout with an older Simulation binary. Configure each module's physical drive, steer, and encoder mounting
directions through Base simulation configuration. These settings are distinct
from electrical motor inversion and are applied once when Phoenix simulation
starts. Public consumer checks use the matching Base vendordep; Base and Simulation remain separate published vendordeps.
Release 2026.0.6 added a ground collider to the standalone 3D drive world. The
Base package alone remains sufficient for roboRIO builds.
Future Simulation releases add a Linux desktop library, so Linux desktop projects need
both vendordeps. Do not add the Simulation package to a different WPILib year.

To check Simulation 2026.0.13 with Base 2026.2.4 after both are published,
run the following from a Simulation `v2026.0.13` checkout, with `BASE_ROOT`
pointing to a checkout of the `v2026.2.4` Base tag. Repeat for Debug and
Release on Linux and Windows; Windows requires the verified Studica cache in
`FRC_VENDOR_CACHE`, as configured by the release workflow. Use `python` in
place of `python3` if that is the Python command on Windows.

```sh
python3 scripts/consumer_smoke.py --target desktop --configuration Release \
  --published --meco-base-root "$BASE_ROOT" --expected-base-version 2026.2.4
```

## Package ownership

MecoBase owns robot control, hardware IO and the Jolt-free shared motor runtime.
MecoSimulation owns physics/contact solvers, Jolt, the WPILib bridge, diagnostics,
shared battery helper, pose-reference transforms, ideal AprilTag visibility and pose observations, and drivetrain tuning estimates. Applications own hardware
parameters, commands, robot-specific game-piece behavior, starting-pose policy and
telemetry presentation. Applications must not copy `base/sim` headers or replace
Simulation archive objects.

`base/sim/SimulationTuning.h` estimates initial proportional drive gains from
a current and traction limited stopping envelope. The application supplies the
translation and heading error scales, along with its physical parameters. This
estimate is an acceleration upper bound, not a contact or steering-load model.
`base/sim/FootprintClearance.h` checks a rectangular footprint against field
bounds and pitch-only box colliders using conservative projected bounds. The
application supplies robot dimensions, height and collider margin. The margin
applies to colliders; the footprint must stay strictly inside the field bounds.
Invalid or unsupported collider geometry is rejected.
Ideal vision observations reject invalid timestamps, height bounds and standard
deviations. Particle contacts are a discrete, equal-mass approximation with a
prescribed robot body; they do not provide swept collision detection or reaction
forces on that body.

2026.0.9 adds hard-mounted contact, bumper rings, load-dependent azimuth, battery
support and diagnostics. `DriveSimulationSession::PhysicsTruth()` exposes complete
body orientation/contact loads without adding fields to the Base ABI. Field maps
helpers are public `meco/sim` headers. Season field geometry stays in the consuming robot project. Release 2026.0.9 passed its cross-platform package and public-Maven gates;
see the [release validation](docs/releases/2026.0.9.md).

## Native AutoDrive and practice control

The package supplies simulated perception and NT practice
match sequencing. Native planning/control and detection fusion live in MecoBase.
See [AutoDrive boundaries](docs/autodrive.md) and [practice control](docs/practice-mode.md).

## Current compatibility

| WPILib year | Status | Platforms |
| --- | --- | --- |
| 2026 | Production release checks link Linux Debug and Release consumers | Linux desktop target |
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

Future Phoenix/WPILib adapter releases target Linux desktop.
The portable physics build does not validate Phoenix sensors/odometry, command
scheduling, or mechanism IO adapters.

## Drivetrain

`base::DriveConfig` supplies mass (`massKg`), yaw inertia (`moi`), tire friction
(`cof`), module positions, wheel radius, drive/steer/coupling ratios, current
limits, inversion, and CANcoder offsets. `DriveConfig::simulation` supplies
bumper half dimensions, body-origin ride height, center of mass relative to that
origin, roll/pitch inertia, wheel/steer inertia, tire stiffness, bumper corner
radius and bumper friction. `driveMotor` and `steerMotor` each provide the
electrical model, current limit and friction voltage. Legacy
suspension fields remain in the ABI but are unused by rigid wheel contact.
Lengths are metres, mass kilograms, inertia kg m², and tire stiffness uses SI.
The robot supplies separate `DriveSimulationConfig::driveMotor` and
`steerMotor` models: resistance, torque constant, velocity constant, current
limit, and friction voltage. MecoSimulation does not infer a motor model from
the hardware backend. Native controller output still includes its configured
limits. The electrical plant produces speed-dependent motoring and braking
torque from voltage and back-EMF rather than a fixed deceleration rate.
`DrivePhysicsConfig` requires the physical model, motor models, bumper corner
radius, and tire-contact parameters explicitly. Invalid or nonfinite physical
parameters throw during construction.

Planar mode fixes chassis Z, pitch, and roll. Full 3D uses four finite-cylinder wheel
contact queries at fixed, hard-mounted module positions. A projected unilateral
normal-impulse solve supports the body against gravity without springs, suspension
travel or static sag. The 1 cm query margin and 0.1 mm penetration slop are
numerical contact tolerances, not physical compliance. Longitudinal/lateral tire
forces are capped by the solved normal reactions. Motor current and wheel inertia
affect acceleration; forces act at contact points. The bumper is one rigid body with four compound box segments forming a hollow
perimeter whose envelope and wall thickness come from `DrivePhysicsConfig`. Its centre
and half-height come from body ride height and bumper half extents; configure
these together with the relative CoM to preserve the intended world CoM height.
The shell contacts ground, slopes and obstacles through Jolt, independently of
wheel-support queries. The centre opening does not create an invisible belly plate.
Bumper friction comes from the robot simulation profile; Jolt combines material
friction. This is rigid contact, not measured foam compression or impact-energy
absorption.
`BodySpec::Shape::BumperRing` uses `radius` as wall thickness and preserves the
existing structure layout; invalid thickness which closes the centre is rejected.
Cylinder queries model finite tire volume, but not tire deformation, individual spikes or sidewalls;
inertia is supplied along body principal axes. This is an approximate rigid-contact
tire model, not a measured tire model or an articulated robot model.

Azimuth load uses an estimated rectangular tire contact patch. Its elastic shear
stores a small static twist; rolling renews that shear over the patch transit time.
Twisting and longitudinal/lateral forces share one friction-wrench ellipsoid, with
capacity based on the actual normal reaction (zero when unloaded). Steering and
wheel speeds are solved implicitly together, retaining current limits and drive/
steer coupling. The chassis receives the opposite actuator reaction; terrain-normal
torsion is projected onto the steering axis, including the remaining constrained
moment on a tilted chassis. This is an approximate lumped contact model, not a
measured TPU tire model or full articulated drivetrain.

`DrivePhysicsConfig` requires `wheelWidth`, `contactPatchLength`, and
`contactShearDeflection` at breakaway to be supplied.
Module truth exposes contact twist, ground torsion and combined azimuth actuator
(steering motor plus drive coupling) torque. The patch RMS radius is
`sqrt((width^2 + length^2)/12)`; stiffness is `mu*N*radius^2/shearDeflection`.
Plastic return mapping limits stored shear at sliding. When normal load increases,
carried-over twist is rescaled to preserve at most the prior shear moment, avoiding
unmodeled spring-energy creation from a stiffness jump. These are model inputs;
wheel-load, speed and carpet/tread measurements are needed to calibrate them.
These added configuration/state fields change the **DrivePhysics ABI**: rebuild
all direct owners, including the WPILib session adapter and tests, together. The
`SimulationWorld` and base robot interfaces retain their prior layouts.

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

For automatic IO selection, pass `MakeLazyMotorSimulationProvider` a builder
that returns `MakeMotorSimulationProvider` configured with the robot's motor
identities, physical properties, and load callbacks. The builder runs once on
the first simulated motor request; real and replay selection do not invoke it.
Copied providers share initialization and retained callback ownership.

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
alongside CPU/build metadata. See the verification record in the season project
for retained successful and failed evidence. Simulation acceptance does not
establish hardware calibration, CAN timing, or physical safety.

## Configurable robot contact

Version `2026.0.13` adds reusable drivetrain tuning, footprint clearance,
ideal vision observations, and particle contact helpers. Applications supply
their own robot geometry, sensor configuration, and game-specific rules.

Version `2026.0.12` removes a diagnostic-only bump scenario while retaining the rigid-contact and low-clearance traversal regressions. Runtime behavior is unchanged from 2026.0.11.

Version `2026.0.11` adds independent belly-pan clearance, rigid
plywood backing and implicit pool-noodle compression. See
[construction, material assumptions, telemetry and checks](docs/compliant-bumper-contact.md).
The legacy adapter overload retains its rigid-contact behavior.

`meco/sim/SimulationPose.h` provides `FloorReferencedRobotPose` with a body-local
height offset. `meco/sim/SimulatedVision.h` provides `VisibleTags` with camera
mount, field layout, full horizontal/vertical angles and maximum range supplied
by the caller. `meco/sim/SimulatedTagCamera.h` combines configurable tag
visibility and an ideal observation source for one camera. Connection,
tracking, alignment, camera mounts and calibration remain caller supplied.
These ideal geometric helpers do not model occlusion or noise.

The Base 2026.2.12 checksum snapshot in `scripts/base-2026.2.12-SHA256SUMS.json`
is pinned to release source `660eb31` and the assembled artifact from
[release run 41534819397](https://github.com/MECO-Robotics/MecoBase-cpp/actions/runs/41534819397).
The downloader checks this snapshot and every immutable Maven coordinate, so a
later Base release changing the site root cannot change this build.

`meco::sim::ConfigureDriveTuning` applies optional automatic PathPlanner gains from the configured robot physics and motor torque constant. Select `base::SimulationTuning::Configured` to preserve the supplied gains; the simulated physical configuration remains unchanged in either mode.
