# Bumper contact with the 2026 bump

The 2026 bump retains its official 44.4-inch depth, 6.513-inch height,
15-degree ramps and 0.5-inch HDPE plate thickness. These dimensions leave an
approximately 14 mm leading edge. The previous 5 mm rounded rigid bumper could
catch there; lowering friction even to zero did not reliably cure the stop.

The corrected rigid padded envelope has a nominal 1.5-inch edge radius, clamped
to 95% of each segment half dimension. Outer dimensions, hollow center, robot
mass/inertia, floor clearance and hard-mounted wheel geometry are unchanged.
This is an estimated contact profile, not a foam spring/compression model.

Only HDPE bump collider material friction changes from 1.0 to 0.035. Jolt's
geometric-mean combine with the bumper's 0.65 produces an estimated pair sliding
coefficient of 0.151. Carpet and other obstacles keep their existing material
settings. The separate tire traction model is unchanged; this value is not a
measured material coefficient or a tire-on-HDPE calibration.

`bump_traversal` exercises the 67.6 kg Kraken/50 A stator configuration with a
4 m/s running approach at the actual 4 ms physics substep. It requires the bumper
to lift the body, physically support it during wheel unloading, preserve loss of
wheel traction while unloaded, and clear the far edge back onto the carpet.
The same regression fails linked against the published 2026.0.9 archive; see
[the regression record](validation/bump-contact-regression.txt).

A low-clearance robot can still high-center during a slow approach. Once all
wheels lift, continued motor output spins wheels rather than inventing chassis
propulsion. Bump traversal then depends on approach momentum, profile, friction
and mass distribution. No bump exclusion, teleport, artificial lift or force
from unloaded wheels is used. These parameters require physical calibration.

Primary references: [FIRST 2026 manual, section 5.5](https://firstfrc.blob.core.windows.net/frc2026/Manual/HTML/2026GameManual.htm),
[Jolt friction combination](https://github.com/jrouwe/JoltPhysics/blob/0373ec0dd762e4bc2f6acdb08371ee84fa23c6db/Jolt/Physics/Constraints/ContactConstraintManager.h).
