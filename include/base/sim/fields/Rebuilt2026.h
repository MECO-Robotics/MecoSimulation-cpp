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
std::vector<BodySpec> FieldColliders();
}
