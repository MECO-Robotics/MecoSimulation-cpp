#pragma once
#ifndef __FRC_ROBORIO__
#include "meco/sim/SimulatedPieceSensor.h"
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace meco::sim {
// Owns a fixed set of simulated cameras, their dynamic mounts and frame sequence.
// The caller supplies scene truth, objects, obstacles and timestamp per update.
class SimulatedPieceCameraBank {
 public:
  struct Sample { std::size_t detections; units::second_t captureTime; };
  explicit SimulatedPieceCameraBank(std::size_t count) : cameras_(count) {}

  std::unique_ptr<base::ObjectDetectionIO> Configure(std::size_t index, PieceSensorConfig config,
                                                      std::function<frc::Transform3d()> mount) {
    if (!mount) throw std::invalid_argument("Missing simulated camera mount");
    auto& camera = cameras_.at(index);
    camera.mount = std::move(mount);
    config.robotToCamera = camera.mount();
    camera.sensor = std::make_unique<SimulatedPieceSensorBridge>(config);
    return camera.sensor->MakeIO();
  }

  std::vector<Sample> Update(const frc::Pose3d& truth, std::span<const frc::Translation3d> pieces,
                             std::span<const base::sim::BodySpec> obstacles,
                             const std::function<units::second_t()>& now) {
    if (!now) throw std::invalid_argument("Missing simulated camera clock");
    ++sequence_;
    std::vector<Sample> samples;
    samples.reserve(cameras_.size());
    for (auto& camera : cameras_) {
      if (!camera.sensor) throw std::logic_error("Simulated camera not configured");
      const auto& frame = camera.sensor->Update(truth, camera.mount(), pieces, obstacles, now(), sequence_);
      samples.push_back({frame.detections.size(), frame.captureTime});
    }
    return samples;
  }
  void Reset() { for (auto& camera : cameras_) if (camera.sensor) camera.sensor->Reset(); }
 private:
  struct Camera {
    std::unique_ptr<SimulatedPieceSensorBridge> sensor;
    std::function<frc::Transform3d()> mount;
  };
  std::vector<Camera> cameras_;
  uint64_t sequence_{};
};
} // namespace meco::sim
#endif
