#pragma once
#include "base/sim/SimulationWorld.h"
#include <vector>

namespace base::sim::fields::rebuilt2026 {
// WPILib 2026.2.1 welded-field, fixed blue-origin coordinates, metres.
inline constexpr double kFieldLength = 16.541;
inline constexpr double kFieldWidth = 8.069;
inline constexpr double kBlueHubX = 4.6256194;
inline constexpr double kRedHubX = 11.9154194;
inline constexpr double kHubY = 4.0346376;
inline constexpr double kStartingLineX = kBlueHubX - .5969;
// Estimated HDPE sliding material: Jolt combines with bumper .65 as
// sqrt(.010*.65) = .081. Tire traction uses its separate wheel model.
inline constexpr double kBumpContactFriction = .010;
// Provisional painted-steel sliding estimate. Jolt combines this with the
// robot contact material using the geometric mean; it is not a measurement.
inline constexpr double kDepotContactFriction = .05;
std::vector<BodySpec> FieldColliders();
// Particle geometry retains an open hub top, unlike the solid chassis envelope.
std::vector<BodySpec> ParticleFieldColliders();
}
