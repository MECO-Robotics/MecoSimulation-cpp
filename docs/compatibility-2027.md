# 2027 / SystemCore compatibility status

Status: **in development; unsupported**.

The Jolt physics and arena code is independent of WPILib, but the 2027 desktop
adapter and vendordep have not been ported or validated. No 2027 binaries or
vendordep are published from this branch. Do not infer SystemCore support from
the standalone CMake physics build.

Before declaring support, pin a WPILib 2027 release, compatible Phoenix and REV
releases, SystemCore OS image, compiler/toolchain, and desktop target set. Then
build a clean desktop consumer and exercise the adapter against that exact
matrix. Hardware deployment and physical acceptance require separate evidence.
