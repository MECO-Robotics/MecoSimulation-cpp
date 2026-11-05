# WPILib 2026 adapter build

This folder builds the optional Drive bridge as a static C++ library against the pinned WPILib 2026 toolchain and the matching MECO Base public headers. The workflow checks both Debug and Release on Linux and Windows. The bridge only becomes a supported vendordep artifact after clean Base plus Simulation consumer links pass on both targets.

`meco/sim/BatterySupply.h` provides opt-in shared battery sag. Call
`UpdateBatterySupply(runtime)` once after the shared runtime's robot period. It
uses aggregate battery-side supply current, sets the runtime bus and roboRIO
input voltage/current, and does not advance physics. It applies one half-weight
numerical relaxation iteration to prevent a delayed-feedback voltage two-cycle;
this is not a calibrated battery transient model. Every registered motor plant
uses that bus on its next period (20 ms explicit feedback delay). Defaults match
WPILib BatterySim: 12 V open circuit, 0.020 ohm pack resistance. Optional
`BatterySupplyConfig` accepts measured voltage/resistance and a nonnegative
auxiliary load. No discharge curve, regeneration, branch wiring losses, breaker
thermal behavior or automatic brownout-output behavior is added by this helper.
The runtime still defaults to fixed 12 V unless the application calls the helper.

`meco/sim/LoadedMotorIO.h` adds a `base::MakeMotorIO` overload for mechanisms whose
simulated motor needs a changing output load. Supply one motor's SI resistance,
torque constant, and speed constant in `LoadedMotorModel`, plus a callback that
returns the current opposing output torque in N m. The callback is sampled once
per robot-period update; its captured season state must outlive the returned IO.
`LoadedMotorModelFromStallAndFreeSpeed` derives those constants from one motor's
nominal voltage, stall current, stall torque, and free speed, using an ideal
zero-current free speed. Configure output inertia and any other model settings
on the returned model before constructing the IO.
The model defaults use a 0.0002 s integration substep, 1 s maximum update,
0.0001 N m/(rad/s) viscous drag, and 0.01 rad/s load direction threshold.
Game-specific torque calculations remain in the consuming robot project.

`SimulationDiagnostics.h` exports opt-in latency, motor-electrical and azimuth
contact diagnostics (`FRC_SIM_LATENCY_PROBE`). Applications call its loop probes
around scheduling and may call `PublishDriverStationPacket()` once per desktop
loop to decouple HAL joystick packet publication from SimGUI redraw frequency.
`FRC_SIM_DS_PACKET_CLOCK=0` disables the latter. These helpers do not step plants.
`DriveSimulationSession::PhysicsTruth()` returns full physical orientation and
module contact loads; applications do not need to replace hardware feedback code.

`SimulationNetwork.h` provides `ConfigureSimulationNetwork()` for isolated
desktop runs. When `FRC_SIM_NT_PORT` is set to an integer from 1024 through
65535, it restarts NT on loopback with that NT4 port and NT3 port zero. Call it
once before publishing; an unset variable leaves NT server configuration alone.
Invalid values throw before changing the server.

`SimulatedPieceSensor.h` injects typed observations into MecoBase perception;
`SimulatedPieceSensorBridge` owns the sensor and a latest-frame mailbox for one
injected IO consumer. Update it on the simulation thread with the current mount,
truth pose, pieces, obstacles, timestamp, and sequence; reset clears a pending
frame. Native AutoDrive planning/control lives in MecoBase. See [AutoDrive](../../docs/autodrive.md).

`PracticeMatchController.h` provides desktop-only NT practice sequencing and
robot-visible DS/FMS inputs. See [practice control](../../docs/practice-mode.md).

`SimulationLifecycle.h` groups network setup, configurable battery state,
practice sequencing, scheduler latency probes and Driver Station packets.
Call its hooks at the existing scheduler and simulation-loop boundaries. It
does not step physics or advance the clock, and skips power, practice and
simulated Driver Station packet updates while replay is active.

`SimulationPower.h` adds optional charge integration, brownout counting and
power/physics-clock telemetry on top of the battery supply helper. Construct
one `SimulationPower` with pack capacity, discharge-curve voltages and internal
resistance, then call `Update(runtime)` once after each robot period. Keep the
instance alive through scene resets if battery charge should persist. Its simple
square-root discharge curve is an estimate, not battery characterization.

`StartPracticeMatch()` returns a command that starts the practice controller
using the simulated Driver Station alliance and can run while disabled.

`SimulatedVisionSource` turns caller-supplied field-relative robot truth and
fix status into latest-sample `InjectedVisionIO` observations. The caller owns
visibility and alignment policy, camera transforms, uncertainty and reset timing.
`Reset` clears pending observations and sensor status. The IO shares ownership of
its sample buffer, so it remains valid if the source handle is destroyed.

`DriveResetCoordinator` tracks an autonomous pose-reset callback and feedback
settling without driving the robot itself. The application supplies clearance
policy, performs accepted physics resets, cancels blocked autonomous commands,
and advances the coordinator with actual physics truth once per scene period.
It returns the odometry reconciliation pose after the configured feedback loops.

`MotorSimulationFactories.h` binds loaded or inertial motor simulation
configuration into lazy callbacks for a mode-selecting Base motor factory.
`MakeMotorSimulationProvider` accepts declarative entries keyed by vendor,
CAN bus and primary ID. It selects the loaded or standard inertial plant only
when called, and rejects duplicate or missing identities.
Binding a callback does not construct a simulated motor. Load callbacks retain
their normal lifetime and ownership requirements.
`SimulatedVisionSource` and `SimulatedPieceSensorBridge` also expose
`MakeIOFactory()` callbacks so a mode selector can defer construction of their
injected IO. Each callback retains its shared sample buffer.

`SimulatedPieceCameraBank` coordinates multiple piece-sensor bridges, dynamic
camera mounts, shared frame sequencing and reset. The application supplies each
camera's configuration and mount, the scene's field-relative truth, piece and
obstacle positions, and a clock; it retains ownership of telemetry and policy.

`FootprintNavigation.h` provides opt-in NT obstacle injection and a command
wrapper that publishes the Base footprint command's outcome. Applications choose
the NT entries, target, obstacle source, drive, navigation configuration and
outcome-to-name function.
