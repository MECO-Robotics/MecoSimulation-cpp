#pragma once

#ifndef __FRC_ROBORIO__
#include "base/Replay.h"
#include "base/SimulationRuntime.h"
#include "meco/sim/PracticeMatchController.h"
#include "meco/sim/SimulationDiagnostics.h"
#include "meco/sim/SimulationNetwork.h"
#include "meco/sim/SimulationPower.h"
#include <frc/RobotBase.h>
#include <memory>
#endif

namespace meco::sim {
// Values are robot configuration. No lifecycle method advances physics time.
struct SimulationLifecycleConfig {
  double batteryCapacityAh, batteryEmptyVoltage, batteryVoltageRange, batteryResistanceOhms;
  double practiceAutonomousSeconds, practiceAssessmentSeconds, practiceTeleopSeconds;
};

// Invoke hooks at the robot's existing scheduler and SimulationPeriodic points.
// Replay never initializes or updates power/practice or publishes DS packets.
class SimulationLifecycle {
 public:
  explicit SimulationLifecycle(SimulationLifecycleConfig config) : config_(config) {}
  SimulationLifecycle(const SimulationLifecycle&) = delete;
  SimulationLifecycle& operator=(const SimulationLifecycle&) = delete;

  void Initialize() {
#ifndef __FRC_ROBORIO__
    if (base::replay::Active() || !frc::RobotBase::IsSimulation()) return;
    ConfigureSimulationNetwork();
    power_ = std::make_unique<SimulationPower>(SimulationPowerConfig{
        config_.batteryCapacityAh, config_.batteryEmptyVoltage,
        config_.batteryVoltageRange, config_.batteryResistanceOhms});
    practice_ = std::make_unique<PracticeMatchController>(PracticeMatchProfile{
        config_.practiceAutonomousSeconds, config_.practiceAssessmentSeconds,
        config_.practiceTeleopSeconds});
#endif
  }
  void BeforeScheduler() { meco::sim::BeginLatencyProbe(); }
  void AfterScheduler() { meco::sim::EndLatencyProbe(); }
  void BeforeScene() {
#ifndef __FRC_ROBORIO__
    if (!base::replay::Active() && practice_) practice_->Update();
#endif
  }
  void AfterScene() {
#ifndef __FRC_ROBORIO__
    if (!base::replay::Active() && power_) {
      power_->Update(base::simulation::GetSimulationRuntime());
      PublishDriverStationPacket();
    }
#endif
  }
 private:
  SimulationLifecycleConfig config_;
#ifndef __FRC_ROBORIO__
  std::unique_ptr<SimulationPower> power_;
  std::unique_ptr<PracticeMatchController> practice_;
#endif
};
} // namespace meco::sim
