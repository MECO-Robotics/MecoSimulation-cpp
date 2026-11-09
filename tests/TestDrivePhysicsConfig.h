#pragma once

#include "base/sim/DrivePhysics.h"

// Synthetic four-module plant used by portable physics tests and benchmarks.
// These values are intentionally not a profile for any season robot.
inline base::sim::DrivePhysicsConfig SyntheticDrivePhysicsConfig()
{
    using namespace base::sim;
    DrivePhysicsConfig c;
    c.mass = 82.4;
    c.inertia = {4.7, 4.9, 7.25};
    c.centerOfMass = {};
    c.bumperHalfExtents = {.42, .37, .095};
    c.bumperWallThickness = .044;
    c.bumperCornerRadius = .026;
    c.bumperFriction = .58;
    c.rideHeight = .278;
    c.wheelRadius = .0475;
    c.driveRatio = 5.7;
    c.steerRatio = 17.8;
    c.couplingRatio = 0;
    c.wheelInertia = .019;
    c.steerInertia = .0034;
    c.friction = 1.08;
    c.longitudinalStiffness = 1625;
    c.lateralStiffness = 1725;
    c.suspensionRestLength = .278;
    c.suspensionTravel = .13;
    c.suspensionStiffness = 5400;
    c.suspensionDamping = 590;
    c.driveMotor = {.0355, .0206, 53.0, 72, .07};
    c.steerMotor = {.0355, .0206, 53.0, 36, .07};
    c.moduleLocations = {{{.35, .30, 0}, {.35, -.30, 0},
                          {-.35, .30, 0}, {-.35, -.30, 0}}};
    c.wheelWidth = .044;
    c.contactPatchLength = .019;
    c.contactShearDeflection = .0011;
    return c;
}
