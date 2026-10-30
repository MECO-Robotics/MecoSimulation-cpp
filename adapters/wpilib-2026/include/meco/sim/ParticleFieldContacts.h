#pragma once
#include "meco/sim/ParticleContacts.h"
#include "base/sim/SimulationWorld.h"
#include <frc/geometry/Quaternion.h>

namespace meco::sim {
// A preconverted set of static, oriented field boxes. The particle solver is
// discrete: callers must use sufficiently small substeps for their velocities.
class ParticleFieldContacts {
 public:
  explicit ParticleFieldContacts(const std::vector<base::sim::BodySpec>& specs) {
    for (const auto& spec : specs) {
      if (spec.shape != base::sim::BodySpec::Shape::Box || spec.mass != 0)
        throw std::invalid_argument("Particle field geometry requires static boxes");
      const auto& p = spec.pose.position;
      const auto& q = spec.pose.rotation;
      const auto& e = spec.halfExtents;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
          !std::isfinite(e.x) || !std::isfinite(e.y) || !std::isfinite(e.z) ||
          e.x <= 0 || e.y <= 0 || e.z <= 0)
        throw std::invalid_argument("Invalid particle field box");
      const frc::Rotation3d rotation{frc::Quaternion{q.w, q.x, q.y, q.z}};
      const auto ex = detail::Point3d(detail::Translation({e.x, 0, 0}).RotateBy(rotation));
      const auto ey = detail::Point3d(detail::Translation({0, e.y, 0}).RotateBy(rotation));
      const auto ez = detail::Point3d(detail::Translation({0, 0, e.z}).RotateBy(rotation));
      boxes_.push_back({ContactBody{frc::Pose3d{units::meter_t{p.x}, units::meter_t{p.y}, units::meter_t{p.z},
          rotation}, {}, {}, {}},
          {{{}, {e.x, e.y, e.z}}}, {p.x, p.y, p.z},
          {std::abs(ex.x)+std::abs(ey.x)+std::abs(ez.x),
           std::abs(ex.y)+std::abs(ey.y)+std::abs(ez.y),
           std::abs(ex.z)+std::abs(ey.z)+std::abs(ez.z)}});
    }
  }
  void Resolve(std::vector<Particle>& particles, double radius, double restitution) const {
    if (!std::isfinite(radius) || radius <= 0 || !std::isfinite(restitution) || restitution < 0 || restitution > 1)
      throw std::invalid_argument("Invalid particle field contact parameters");
    for (auto& particle : particles)
      for (const auto& box : boxes_) {
        const auto& p = particle.position;
        if (std::abs(p.x-box.center.x) > box.bounds.x+radius ||
            std::abs(p.y-box.center.y) > box.bounds.y+radius ||
            std::abs(p.z-box.center.z) > box.bounds.z+radius) continue;
        detail::BoxContacts(particle, box.body, radius, restitution, box.box);
      }
  }
 private:
  struct Box { ContactBody body; std::vector<ContactBox> box; ParticlePoint center, bounds; };
  std::vector<Box> boxes_;
};
} // namespace meco::sim
