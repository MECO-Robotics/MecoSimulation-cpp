#pragma once
#ifndef __FRC_ROBORIO__

#include "meco/sim/BatterySupply.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <ntcore_cpp.h>

namespace meco::sim {
struct SimulationPowerConfig {
  double capacityAh;
  double emptyVoltage;
  double voltageRange;
  double resistanceOhms;
};

// Keep one instance across scene resets to retain battery charge. Update once
// after the runtime's robot period; it never advances the physics or clock.
class SimulationPower {
 public:
  explicit SimulationPower(SimulationPowerConfig config,
      nt::NetworkTableInstance nt = nt::NetworkTableInstance::GetDefault())
      : config_(config), remainingAh_(config.capacityAh),
        clock_(nt.GetDoubleArrayTopic("/Simulation/Physics/Samples").Publish()),
        power_(nt.GetDoubleArrayTopic("/Simulation/PowerSamples").Publish()),
        battery_(nt.GetDoubleArrayTopic("/Simulation/BatteryState").Publish()) {
    if (!std::isfinite(config.capacityAh) || config.capacityAh <= 0 ||
        !std::isfinite(config.emptyVoltage) || config.emptyVoltage <= 0 ||
        !std::isfinite(config.voltageRange) || config.voltageRange < 0 ||
        config.emptyVoltage + config.voltageRange > 16 ||
        !std::isfinite(config.resistanceOhms) || config.resistanceOhms < 0)
      throw std::invalid_argument("Invalid simulation power configuration");
  }

  template<class Runtime> void Update(Runtime& runtime) {
    const double now = runtime.Time();
    clock_.Set(std::array<double, 3>{static_cast<double>(nt::Now()),
        static_cast<double>(frc::RobotController::GetFPGATime()), now});
    const double current = runtime.SupplyCurrent();
    if (!std::isfinite(now) || !std::isfinite(current) || current < 0)
      throw std::invalid_argument("Invalid simulation time or supply current");
    const double dt = previousTime_ >= 0 && now >= previousTime_ ? now - previousTime_ : 0;
    previousTime_ = now;
    remainingAh_ = std::max(0.0, remainingAh_ - current * dt / 3600.0);
    const double charge = std::clamp(remainingAh_ / config_.capacityAh, 0.0, 1.0);
    const double voltage = UpdateBatterySupply(runtime, {config_.emptyVoltage +
        config_.voltageRange * std::sqrt(charge), config_.resistanceOhms, 0.0});
    const bool brownedOut = frc::RobotController::IsBrownedOut();
    if (brownedOut && !previouslyBrownedOut_) ++brownoutCount_;
    previouslyBrownedOut_ = brownedOut;
    frc::SmartDashboard::PutNumber("SimulationPower/SupplyCurrentAmps", current);
    frc::SmartDashboard::PutNumber("SimulationPower/BusVoltage", voltage);
    frc::SmartDashboard::PutNumber("SimulationPower/BatteryRemainingAh", remainingAh_);
    frc::SmartDashboard::PutNumber("SimulationPower/BatteryStateOfCharge", charge);
    frc::SmartDashboard::PutBoolean("SimulationPower/BrownedOut", brownedOut);
    frc::SmartDashboard::PutNumber("SimulationPower/BrownoutCount", brownoutCount_);
    power_.Set(std::array<double, 3>{static_cast<double>(nt::Now()), current, voltage});
    battery_.Set(std::array<double, 4>{remainingAh_, charge, brownedOut ? 1.0 : 0.0,
        static_cast<double>(brownoutCount_)});
  }
 private:
  SimulationPowerConfig config_;
  double remainingAh_;
  double previousTime_{-1};
  int brownoutCount_{0};
  bool previouslyBrownedOut_{false};
  nt::DoubleArrayPublisher clock_, power_, battery_;
};
} // namespace meco::sim
#endif
