#pragma once
#ifndef __FRC_ROBORIO__
#include "meco/sim/IdealVisionObservation.h"
#include <base/Vision.h>
#include <functional>
#include <memory>
#include <utility>

namespace meco::sim {
// Ideal observations use the supplied robot truth in the field frame. Acquisition
// and alignment policy remain with the caller. One update replaces any unread
// observation, matching a latest-sample sensor. IO can safely outlive this handle.
class SimulatedVisionSource {
 public:
  SimulatedVisionSource() : inputs_(std::make_shared<base::VisionInputs>()) {}

  std::function<std::unique_ptr<base::VisionIO>()> MakeIOFactory() const {
    return [inputs = inputs_]() -> std::unique_ptr<base::VisionIO> {
      return std::make_unique<base::InjectedVisionIO>([inputs](auto& out) {
        out = std::move(*inputs);
        inputs->observations.clear();
      });
    };
  }
  std::unique_ptr<base::VisionIO> MakeIO() const { return MakeIOFactory()(); }

  void Reset() { *inputs_ = {}; }

  void Update(const frc::Pose3d& truth, units::second_t timestamp,
              const frc::AprilTagFieldLayout& layout, bool hasFix,
              units::meter_t maxAbsHeight, std::array<double, 3> stdDevs,
              bool connected, bool tracking, bool aligned) {
    base::VisionInputs next;
    next.connected = connected;
    next.tracking = tracking;
    next.aligned = aligned;
    const auto observation = MakeIdealVisionObservation(truth, timestamp, layout,
                                                         hasFix, maxAbsHeight, stdDevs);
    if (observation)
      next.observations.push_back({observation->pose, observation->timestamp, observation->stdDevs});
    *inputs_ = std::move(next);
  }
 private:
  std::shared_ptr<base::VisionInputs> inputs_;
};
} // namespace meco::sim
#endif
