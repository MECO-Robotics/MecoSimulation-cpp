#pragma once
#ifndef __FRC_ROBORIO__

#include "meco/sim/DriveResetCoordinator.h"
#include "meco/sim/DriveSimulationAdapter.h"
#include "meco/sim/SimulationPose.h"
#include <base/Replay.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Quaternion.h>
#include <frc/geometry/struct/Pose3dStruct.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <cstdlib>
#include <functional>
#include <string_view>
#include <optional>
#include <vector>

namespace meco::sim {
// Desktop drive scene mechanics. The robot supplies its collider geometry,
// clearance rule, reset notification and contact configuration.
class DriveScene {
 public:
  struct Configuration {
    bool fieldCollidersEnabled = true;
    int feedbackLoops = 1;
    std::vector<base::sim::BodySpec> colliders;
    std::optional<RobotContactConfig> contact;
    std::function<bool(const frc::Pose2d&)> clearStartingPose;
    std::function<void()> resetSensors;
  };
  struct Snapshot {
    frc::Pose3d visualPose;
    base::sim::BodyState body;
  };

  DriveScene(base::Drive& drive, Configuration config)
      : drive_(drive), config_(std::move(config)), reset_(config_.feedbackLoops),
        groundedHeight_(units::meter_t{drive.Config().simulation.rideHeight}),
        pose_(nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose3d>("/Simulation/RobotTruth").Publish()),
        bodyPose_(nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose3d>("/Simulation/PhysicsBodyTruth").Publish()) {
    if (const char* fieldMode = std::getenv("FRC_SIM_FIELD_COLLIDERS"))
      config_.fieldCollidersEnabled = std::string_view{fieldMode} != "0";
    const auto backend = base::sim::BackendFromEnvironment();
    const auto& physics = drive.Config().simulation;
    frc::SmartDashboard::PutNumber("FieldSimulation/GroundedBodyHeight", physics.rideHeight);
    frc::SmartDashboard::PutNumber("FieldSimulation/BumperCornerRadius", backend == base::sim::Backend::Full3D && config_.contact
        ? config_.contact->noodleDiameter / 2 : base::sim::kBumperCornerRadius);
    frc::SmartDashboard::PutNumber("FieldSimulation/BumperBottomHeight", physics.rideHeight - physics.bumperHalfHeight);
    frc::SmartDashboard::PutNumber("FieldSimulation/BumperTopHeight", physics.rideHeight + physics.bumperHalfHeight);
    auto session = backend == base::sim::Backend::Full3D && config_.contact
        ? CreateDriveSimulationAdapter(drive.Config(), backend, *config_.contact)
        : CreateDriveSimulationAdapter(drive.Config(), backend);
    if (config_.fieldCollidersEnabled) {
      auto& arena = session->CreateArena(base::sim::ArenaScene::Empty);
      for (const auto& collider : config_.colliders) arena.AddCollider(collider);
    }
    frc::SmartDashboard::PutBoolean("FieldSimulation/CollidersEnabled", config_.fieldCollidersEnabled);
    frc::SmartDashboard::PutNumber("FieldSimulation/ColliderCount", config_.fieldCollidersEnabled ? config_.colliders.size() : 0);
    session_ = session.get(); // Drive owns the adapter; callbacks require this scene to remain alive while drive is used.
    drive.SetSimulationAdapter(std::move(session));
    drive.SetPoseResetCallback([this](const frc::Pose2d& pose) {
      const bool clear = !reset_.CapturingAutonomous() || !config_.fieldCollidersEnabled ||
                         !config_.clearStartingPose || config_.clearStartingPose(pose);
      const auto action = reset_.OnPoseReset(pose, clear);
      if (action == DriveResetCoordinator::PoseReset::autonomousAccepted) {
        if (config_.resetSensors) config_.resetSensors();
        drive_.ResetSimulation(pose);
        frc::SmartDashboard::PutString("Simulation/AutoStartPose", "applied");
      } else if (action == DriveResetCoordinator::PoseReset::autonomousRejected) {
        drive_.ResetPose(reset_.PhysicalOrigin());
        frc::SmartDashboard::PutString("Simulation/AutoStartPose", "blocked-by-field");
      }
    });
  }
  void BeginAutonomousSchedule() {
    reset_.BeginAutonomousSchedule();
    frc::SmartDashboard::PutString("Simulation/AutoStartPose", "not-requested");
  }
  bool EndAutonomousSchedule() { return reset_.EndAutonomousSchedule(); }
  bool Ready() const { return reset_.Ready(); }
  bool FieldCollidersEnabled() const { return config_.fieldCollidersEnabled; }
  void Reset(const frc::Pose2d& origin) {
    reset_.BeginPhysicalReset(origin);
    drive_.ResetSimulation(origin);
  }
  std::optional<Snapshot> Update() {
    const auto truth = drive_.SimulationTruth();
    if (!truth) return std::nullopt;
    if (const auto pose = reset_.Advance(frc::Pose2d{units::meter_t{truth->x}, units::meter_t{truth->y},
                                                     frc::Rotation2d{units::radian_t{truth->yaw}}}))
      drive_.ResetPose(*pose);
    const auto body = session_->PhysicsTruth().body;
    const auto& p = body.pose.position;
    const auto& q = body.pose.rotation;
    const frc::Pose3d bodyPose{units::meter_t{p.x}, units::meter_t{p.y}, units::meter_t{p.z},
                               frc::Rotation3d{frc::Quaternion{q.w, q.x, q.y, q.z}}};
    bodyPose_.Set(bodyPose);
    const auto visualPose = FloorReferencedRobotPose(bodyPose, groundedHeight_);
    pose_.Set(visualPose);
    return Snapshot{visualPose, body};
  }
 private:
  base::Drive& drive_;
  Configuration config_;
  DriveResetCoordinator reset_;
  units::meter_t groundedHeight_;
  nt::StructPublisher<frc::Pose3d> pose_, bodyPose_;
  DriveSimulationSession* session_{};
};

// The configuration factory is evaluated only for live simulation. In replay
// and on real robots no scene or simulated drivetrain adapter is created.
template <typename ConfigurationFactory>
std::unique_ptr<DriveScene> MakeRuntimeDriveScene(base::Drive& drive, ConfigurationFactory&& configuration) {
  if (!base::replay::Active() && frc::RobotBase::IsSimulation())
    return std::make_unique<DriveScene>(drive, std::invoke(std::forward<ConfigurationFactory>(configuration)));
  return {};
}
} // namespace meco::sim
#endif
