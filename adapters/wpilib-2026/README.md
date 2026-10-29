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

`SimulationDiagnostics.h` exports opt-in latency, motor-electrical and azimuth
contact diagnostics (`FRC_SIM_LATENCY_PROBE`). Applications call its loop probes
around scheduling and may call `PublishDriverStationPacket()` once per desktop
loop to decouple HAL joystick packet publication from SimGUI redraw frequency.
`FRC_SIM_DS_PACKET_CLOCK=0` disables the latter. These helpers do not step plants.
`DriveSimulationSession::PhysicsTruth()` returns full physical orientation and
module contact loads; applications do not need to replace hardware feedback code.

`SimulatedPieceSensor.h` injects typed observations into MecoBase perception;
native AutoDrive planning/control lives in MecoBase. See [AutoDrive](../../docs/autodrive.md).

`PracticeMatchController.h` provides desktop-only NT practice sequencing and
robot-visible DS/FMS inputs. See [practice control](../../docs/practice-mode.md).
