#pragma once
#ifndef __FRC_ROBORIO__

#include "meco/sim/DriveScene.h"
#include "meco/sim/DriverInputTelemetry.h"
#include "meco/sim/SimulationLifecycle.h"
#include <base/DriverInput.h>
#include <frc2/command/Command.h>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

namespace meco::sim {
// Coordinates the reusable WPILib simulation services around the robot's
// existing scheduler and SimulationPeriodic callbacks. The application still
// supplies field geometry, clear-spawn policy, reset hooks and game simulation.
class RobotSimulationRuntime {
 public:
  using DriveRequestObserver = base::DriverInputState::DriveRequestObserver;

  RobotSimulationRuntime(SimulationLifecycleConfig lifecycleConfig,
                         bool driverInputTelemetryEnabledByDefault)
      : lifecycle_(lifecycleConfig),
        driverInputTelemetryEnabledByDefault_(driverInputTelemetryEnabledByDefault) {}
  RobotSimulationRuntime(const RobotSimulationRuntime&) = delete;
  RobotSimulationRuntime& operator=(const RobotSimulationRuntime&) = delete;

  template <typename ConfigurationFactory>
  void Initialize(base::Drive& drive, ConfigurationFactory&& configuration) {
    lifecycle_.Initialize();
    driveScene_ = MakeRuntimeDriveScene(drive, std::forward<ConfigurationFactory>(configuration));
    if (!driveScene_) return;

    driverInputTelemetry_ = std::make_unique<DriverInputTelemetry>(driverInputTelemetryEnabledByDefault_);
    driveRequestObserver_ = [this](const base::DriverInputState& input, double x, double y,
                                   double omega, frc::Rotation2d heading) {
      if (driverInputTelemetry_)
        driverInputTelemetry_->Publish(input.nt, input.driverStationAxes.leftY,
            input.driverStationAxes.leftX, input.driverStationAxes.rightX, x, y, omega,
            heading, input.nt.accepted);
    };
  }

  void BeforeScheduler() { lifecycle_.BeforeScheduler(); }
  void AfterScheduler() { lifecycle_.AfterScheduler(); }
  void BeforeScene() { lifecycle_.BeforeScene(); }
  void AfterScene() { lifecycle_.AfterScene(); }

  bool HasDriveScene() const { return static_cast<bool>(driveScene_); }
  bool Ready() const { return !driveScene_ || driveScene_->Ready(); }
  bool FieldCollidersEnabled() const { return driveScene_ && driveScene_->FieldCollidersEnabled(); }
  bool UseNtInput(bool selected) const {
    return driverInputTelemetry_ ? driverInputTelemetry_->Selected(selected) : selected;
  }
  const DriveRequestObserver* DriveRequestObserverCallback() const {
    return driveRequestObserver_ ? &driveRequestObserver_ : nullptr;
  }

  void BeginReset(const frc::Pose2d& origin) {
    if (driveScene_) driveScene_->Reset(origin);
  }
  std::optional<DriveScene::Snapshot> UpdateScene() {
    return driveScene_ ? driveScene_->Update() : std::nullopt;
  }
  void ScheduleAutonomous(frc2::Command& command) {
    if (driveScene_) driveScene_->BeginAutonomousSchedule();
    command.Schedule();
    if (driveScene_ && driveScene_->EndAutonomousSchedule()) command.Cancel();
  }

 private:
  SimulationLifecycle lifecycle_;
  bool driverInputTelemetryEnabledByDefault_{};
  std::unique_ptr<DriveScene> driveScene_;
  std::unique_ptr<DriverInputTelemetry> driverInputTelemetry_;
  DriveRequestObserver driveRequestObserver_;
};
}  // namespace meco::sim

#endif
