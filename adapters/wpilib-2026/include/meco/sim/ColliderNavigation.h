#pragma once
#include <base/sim/SimulationWorld.h>
#include <cmath>
#include <vector>

namespace meco::sim {
// Plan-view rectangle in field coordinates, in metres.
struct ColliderRectangle {
  double minX, minY, maxX, maxY;
};

// Project unrotated boxes intersecting the inclusive vertical interval, in
// metres. Requires bottom <= top. Other shapes and rotated boxes are omitted;
// callers choose the interval and add terrain, padding, or other obstacles.
inline std::vector<ColliderRectangle> ColliderRectangles(
    const std::vector<base::sim::BodySpec>& bodies, double bottom, double top) {
  std::vector<ColliderRectangle> result;
  for (const auto& body : bodies) {
    const auto& q = body.pose.rotation;
    const auto& p = body.pose.position;
    const auto& h = body.halfExtents;
    if (body.shape != base::sim::BodySpec::Shape::Box ||
        std::abs(q.x) > 1e-8 || std::abs(q.y) > 1e-8 || std::abs(q.z) > 1e-8 ||
        p.z + h.z < bottom || p.z - h.z > top) continue;
    result.push_back({p.x - h.x, p.y - h.y, p.x + h.x, p.y + h.y});
  }
  return result;
}
} // namespace meco::sim
