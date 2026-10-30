# Practice-match controller (desktop only)

For the broader physics and battery overview, see [simulation basics](simulation-basics.md).

`meco/sim/PracticeMatchController.h` exposes the reusable native
`PracticeMatchController`. Construct it only when `frc::RobotBase::IsSimulation()`
is true, keep it alive for the robot lifetime, and call `Update()` once per
simulation period. It publishes DS packets without advancing physics or time.
Its source and compiled implementation are included in the Simulation vendordep.

The constructor accepts `PracticeMatchProfile{autonomousSeconds,
assessmentSeconds, teleopSeconds}` and an optional NT instance. Applications own
the season profile; 2026 uses `{20,3,140}`. FPGA microseconds determine boundaries.
Do not use another automated DS controller concurrently.

## NT contract

All topics live under `/Sim/Practice/`:

- String `Command`: `start`, `abort`, `reset`. Publish with duplicate updates
  enabled to send the same command more than once. The retained initial value
  is ignored; each subsequent update is an event.
- String config `Config/AllianceStation`: `Red1`–`Red3` or `Blue1`–`Blue3`, default
  `Blue1`. `Config/AutoResult`: `R` or `B`, default `R`; this generic message is
  delivered at TELEOP start. The profile's assessment gap supplies the delay.
- Status `State` (string), `Running` (boolean), `RemainingSeconds` and `MatchTime`
  (double), `GameData` (string), `LastCommand` (string), `LastCommandAccepted`
  (boolean), and `Reason` (string).
- Read-only-by-contract `Profile/AutoSeconds`, `Profile/AssessmentSeconds`, and
  `Profile/TeleopSeconds` (double) describe the constructed timing profile.

Start requires disabled/no E-stop, validates and snapshots config, saves prior
attachment/alliance/match metadata, and sets DS/FMS attached for the sequence.
Game data is cleared for AUTO and the disabled assessment gap. End of match,
abort, or destruction during a run disables and restores saved metadata and
attachment settings. It never restores enabled or clears E-stop. Reset clears
session state, not the physical world. Invalid commands/configuration are
rejected with status. Manual changes to enable/mode/test/E-stop/attachment/alliance
abort via latched HAL callbacks; pending commands are discarded on interruption.
Only a fresh subsequent start can re-enable.

This emulates robot-visible DS/FMS inputs, not scoring, refereeing, lighting, or
field network protocols. Season hub activity must be derived from the normal
DriverStation APIs in season code. Real match time is approximate. GUI-loaded
acceptance is separate from deterministic/headless tests; the GUI has its own
countdown and manual controls, and manual takeover intentionally aborts practice.

WPILib new-data notification itself marks DS attached. Cleanup restores the saved
raw DS flag after its final packet; later packets from another owner may attach
again. The controller never fights a new owner after releasing the session.

The generic profile has no default season timings: callers must supply all three
positive durations (1 microsecond through 24 hours per phase). The robot's season
adapter supplies the official profile.
