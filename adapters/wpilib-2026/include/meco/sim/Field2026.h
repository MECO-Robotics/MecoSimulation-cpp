#pragma once

#include "meco/sim/ColliderNavigation.h"

#include <base/FootprintClearance.h>
#include <base/FootprintSpeedProfile.h>
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
inline constexpr double kHubOpeningHeightMetres = 72 * .0254;

// Estimated HDPE sliding friction used by the bump contact model.
inline constexpr double kBumpContactFriction = .010;

std::vector<base::sim::BodySpec> FieldColliders();

// Particle geometry retains an open hub top, unlike the solid chassis envelope.
std::vector<base::sim::BodySpec> ParticleFieldColliders();

// Traversable terrain and plan-view avoidance geometry for the four 2026 hub
// ramps. Padding expands each avoidance rectangle in metres.
std::vector<double> RampBoundaries(base::FootprintPose from, base::FootprintPose to);
base::FootprintTerrainSegment RampTerrain(base::FootprintPose from, base::FootprintPose to);
std::vector<ColliderRectangle> RampObstacles(double paddingMetres);

}  // namespace meco::sim::field2026
