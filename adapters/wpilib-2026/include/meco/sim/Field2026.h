#pragma once

#include <base/sim/SimulationWorld.h>

#include <vector>

namespace meco::sim::field2026 {

// Fixed blue-origin 2026 field geometry in metres.
inline constexpr double kFieldLength = 16.541;
inline constexpr double kFieldWidth = 8.069;
inline constexpr double kBlueHubX = 4.6256194;
inline constexpr double kRedHubX = 11.9154194;
inline constexpr double kHubY = 4.0346376;
inline constexpr double kStartingLineX = kBlueHubX - .5969;

// Estimated HDPE sliding friction used by the bump contact model.
inline constexpr double kBumpContactFriction = .010;

std::vector<base::sim::BodySpec> FieldColliders();

// Particle geometry retains an open hub top, unlike the solid chassis envelope.
std::vector<base::sim::BodySpec> ParticleFieldColliders();

}  // namespace meco::sim::field2026
