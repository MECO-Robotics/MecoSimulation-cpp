#pragma once
#ifndef __FRC_ROBORIO__

#include "meco/sim/SimulatedVision.h"
#include "meco/sim/SimulatedVisionSource.h"
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <array>
#include <functional>
#include <memory>
#include <utility>
#include <vector>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>

namespace meco::sim {
// Reusable per-camera AprilTag visibility and ideal observation source.
// Connection, tracking, and alignment policy remain caller supplied.
struct SimulatedTagCameraConfig {
  units::degree_t horizontalFov{63.3};
  units::degree_t verticalFov{49.7};
  units::meter_t maxRange{8.0};
  units::meter_t maxAbsTruthHeight{0.75};
  std::array<double, 3> stdDevs{0.02, 0.02, 0.04};
};

class SimulatedTagCamera {
 public:
  explicit SimulatedTagCamera(SimulatedTagCameraConfig config = {})
      : config_(std::move(config)) {}

  std::function<std::unique_ptr<base::VisionIO>()> MakeIOFactory() const {
    return source_.MakeIOFactory();
  }
  std::unique_ptr<base::VisionIO> MakeIO() const { return source_.MakeIO(); }
  const SimulatedTagCameraConfig& Config() const { return config_; }
  const std::vector<int>& TagIds() const { return tagIds_; }
  void Reset() {
    source_.Reset();
    tagIds_.clear();
  }

  void Update(const frc::Pose3d& truth, units::second_t timestamp,
              const frc::AprilTagFieldLayout& layout,
              const frc::Transform3d& robotToCamera,
              bool connected = true, bool tracking = true, bool aligned = true) {
    tagIds_ = VisibleTags(truth, robotToCamera, layout, config_.horizontalFov,
                          config_.verticalFov, config_.maxRange);
    source_.Update(truth, timestamp, layout, !tagIds_.empty(), config_.maxAbsTruthHeight,
                   config_.stdDevs, connected, tracking, aligned);
  }

 private:
  SimulatedTagCameraConfig config_;
  SimulatedVisionSource source_;
  std::vector<int> tagIds_;
};
}  // namespace meco::sim

#endif
