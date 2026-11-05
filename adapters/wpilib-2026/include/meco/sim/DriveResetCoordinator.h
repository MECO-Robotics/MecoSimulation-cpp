#pragma once
#ifndef __FRC_ROBORIO__
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <optional>
#include <stdexcept>

namespace meco::sim {
// Coordinates callback-driven odometry resets with asynchronous simulated
// encoder/gyro feedback. It never moves the drivetrain or chooses a clear pose.
class DriveResetCoordinator {
 public:
  enum class PoseReset { ordinary, autonomousAccepted, autonomousRejected };
  explicit DriveResetCoordinator(int feedbackLoops) : feedbackLoops_(feedbackLoops) {
    if (feedbackLoops <= 0) throw std::invalid_argument("Reset feedback loops must be positive");
  }
  void BeginAutonomousSchedule() { capture_ = true; blocked_ = false; }
  bool CapturingAutonomous() const { return capture_; }
  bool EndAutonomousSchedule() { capture_ = false; return blocked_; }
  void BeginPhysicalReset(const frc::Pose2d& physicalOrigin) {
    physicalOrigin_ = physicalOrigin;
    remaining_ = feedbackLoops_;
  }
  // Call from Drive's pose reset callback. Evaluate clearance in the caller;
  // the return value directs it to reset physics or restore odometry.
  PoseReset OnPoseReset(const frc::Pose2d& pose, bool clear) {
    if (capture_) {
      capture_ = false;
      if (!clear) { blocked_ = true; return PoseReset::autonomousRejected; }
      BeginPhysicalReset(pose);
      return PoseReset::autonomousAccepted;
    }
    if (remaining_ > 0) seed_ = pose;
    return PoseReset::ordinary;
  }
  bool Ready() const { return remaining_ == 0; }
  const frc::Pose2d& PhysicalOrigin() const { return physicalOrigin_; }
  // Call once per scene period with actual physics truth, only when available.
  // Reconcile exactly once, preserving any physical motion since teleport.
  std::optional<frc::Pose2d> Advance(const frc::Pose2d& truth) {
    if (remaining_ > 0 && --remaining_ == 0)
      return seed_.TransformBy(frc::Transform2d{physicalOrigin_, truth});
    return std::nullopt;
  }
 private:
  int feedbackLoops_;
  int remaining_{};
  frc::Pose2d seed_{}, physicalOrigin_{};
  bool capture_{false}, blocked_{false};
};
} // namespace meco::sim
#endif
