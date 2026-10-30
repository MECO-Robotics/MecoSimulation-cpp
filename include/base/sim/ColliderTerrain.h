#pragma once

#include "base/sim/SimulationWorld.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace base::sim {
// Plan-view bounds and X grade of an axis-aligned plate pitched around Y.
struct PitchedPlateTerrain {
  double minX, maxX, minY, maxY, gradeX;
};

// Excludes level plates and plates rotated about X or Z. Project the plate's
// centerline, without treating its thickness as traversable slope.
inline std::vector<PitchedPlateTerrain> PitchedPlateTerrains(const std::vector<BodySpec>& bodies) {
  std::vector<PitchedPlateTerrain> result;
  for (const auto& body : bodies) {
    if (body.shape != BodySpec::Shape::Box || body.mass != 0) continue;
    const auto q = body.pose.rotation;
    if (std::abs(q.x) > 1e-8 || std::abs(q.z) > 1e-8 || std::abs(q.y) < 1e-8) continue;
    const double pitch = 2 * std::atan2(q.y, q.w);
    const double hx = std::abs(std::cos(pitch)) * body.halfExtents.x;
    result.push_back({body.pose.position.x - hx, body.pose.position.x + hx,
                      body.pose.position.y - body.halfExtents.y,
                      body.pose.position.y + body.halfExtents.y, -std::tan(pitch)});
  }
  return result;
}

// Return fractions where a segment crosses the edges of a plate. Values are
// sorted and deduplicated; the segment endpoints themselves are excluded.
inline std::vector<double> TerrainBoundaries(const std::vector<PitchedPlateTerrain>& plates,
                                             double ax, double ay, double bx, double by) {
  std::vector<double> cuts;
  const auto clip = [](double origin, double delta, double lo, double hi,
                       double& enter, double& exit) {
    if (std::abs(delta) < 1e-12) return origin >= lo && origin <= hi;
    double a = (lo - origin) / delta, b = (hi - origin) / delta;
    if (a > b) std::swap(a, b);
    enter = std::max(enter, a);
    exit = std::min(exit, b);
    return enter < exit;
  };
  for (const auto& plate : plates) {
    double enter = 0, exit = 1;
    if (!clip(ax, bx - ax, plate.minX, plate.maxX, enter, exit) ||
        !clip(ay, by - ay, plate.minY, plate.maxY, enter, exit)) continue;
    if (enter > 1e-8 && enter < 1 - 1e-8) cuts.push_back(enter);
    if (exit > 1e-8 && exit < 1 - 1e-8) cuts.push_back(exit);
  }
  std::sort(cuts.begin(), cuts.end());
  cuts.erase(std::unique(cuts.begin(), cuts.end(), [](double a, double b) {
    return std::abs(a - b) < 1e-7;
  }), cuts.end());
  return cuts;
}

// Signed grade along a segment, sampled at its midpoint. Split a segment at
// TerrainBoundaries first when it crosses a plate edge.
inline double TerrainGrade(const std::vector<PitchedPlateTerrain>& plates,
                           double ax, double ay, double bx, double by) {
  const double dx = bx - ax, dy = by - ay, length = std::hypot(dx, dy);
  if (length < 1e-9) return 0;
  const double x = (ax + bx) / 2, y = (ay + by) / 2;
  for (const auto& plate : plates)
    if (x >= plate.minX && x <= plate.maxX && y >= plate.minY && y <= plate.maxY)
      return plate.gradeX * dx / length;
  return 0;
}
}
