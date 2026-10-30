# Simulation basics

Want to try robot code without the robot? MecoSimulation provides desktop physics. Your robot project supplies its dimensions, mass, gearing, field setup, and game rules.

![Simulation physics, sensors, and battery flow](visuals/simulation-basics.png)

[Diagram source](visuals/simulation-basics.mmd)

## Drive and contact

- The 3D model treats wheels as hard-mounted. A wheel off the ground has no modeled traction.
- Steering gets harder when the modeled wheel is loaded. The contact patch is an estimate.
- Optional bumper contact includes foam and rigid backing. You supply the dimensions and material estimates.
- The simulated physical pose and the sensor-based pose estimate can differ. Both are simulated.

Read [Jolt simulation models](jolt-simulation.md), [compliant contact](compliant-bumper-contact.md), and [DrivePhysics](../include/base/sim/DrivePhysics.h).

## Shared battery

- After each shared runtime period, `UpdateBatterySupply` estimates voltage from battery-side supply current. Plants consume that voltage in the next period.
- Defaults are 12 V open circuit and 20 mΩ resistance. The update is smoothed for numerical stability; it is not a measured battery time constant.
- This model does not include state of charge, regeneration, or branch losses.

See [BatterySupply.h](../adapters/wpilib-2026/include/meco/sim/BatterySupply.h). Robot code decides when to call it.

## Practice matches

The desktop [practice controller](practice-mode.md) publishes simulated Driver Station state from a profile supplied by the robot. The robot project chooses timings, game rules, and dashboard controls. It is not a field-management system or a scoring simulation.
