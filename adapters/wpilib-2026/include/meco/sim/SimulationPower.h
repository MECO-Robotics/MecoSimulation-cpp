#pragma once
#ifndef __FRC_ROBORIO__

#include "meco/sim/BatterySupply.h"
#include <base/Telemetry.h>
#include <base/TelemetryPublishers.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <frc/RobotController.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <ntcore_cpp.h>
#include <memory>

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
        clock_(std::make_shared<nt::DoubleArrayPublisher>(
            nt.GetDoubleArrayTopic("/Simulation/Physics/Samples").Publish())),
        power_(std::make_shared<nt::DoubleArrayPublisher>(
            nt.GetDoubleArrayTopic("/Simulation/PowerSamples").Publish())),
        battery_(std::make_shared<nt::DoubleArrayPublisher>(
            nt.GetDoubleArrayTopic("/Simulation/BatteryState").Publish())) {
    if (!std::isfinite(config.capacityAh) || config.capacityAh <= 0 ||
        !std::isfinite(config.emptyVoltage) || config.emptyVoltage <= 0 ||
        !std::isfinite(config.voltageRange) || config.voltageRange < 0 ||
        config.emptyVoltage + config.voltageRange > 16 ||
        !std::isfinite(config.resistanceOhms) || config.resistanceOhms < 0)
      throw std::invalid_argument("Invalid simulation power configuration");
  }

  template<class Runtime> void Update(Runtime& runtime) {
    const double now = runtime.Time();
    const std::array<double, 3> clockSample{static_cast<double>(nt::Now()),
        static_cast<double>(frc::RobotController::GetFPGATime()), now};
    base::telemetry::Post("Simulation/Physics/Samples",
        [publisher = clock_, clockSample] { publisher->Set(clockSample); });
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
    base::telemetry::PostDashboardNumber("SimulationPower/SupplyCurrentAmps", current);
    base::telemetry::PostDashboardNumber("SimulationPower/BusVoltage", voltage);
    base::telemetry::PostDashboardNumber("SimulationPower/BatteryRemainingAh", remainingAh_);
    base::telemetry::PostDashboardNumber("SimulationPower/BatteryStateOfCharge", charge);
    base::telemetry::PostDashboardBoolean("SimulationPower/BrownedOut", brownedOut);
    base::telemetry::PostDashboardNumber("SimulationPower/BrownoutCount", brownoutCount_);
    const std::array<double, 3> powerSample{static_cast<double>(nt::Now()), current, voltage};
    base::telemetry::Post("Simulation/PowerSamples",
        [publisher = power_, powerSample] { publisher->Set(powerSample); });
    const std::array<double, 4> batterySample{remainingAh_, charge, brownedOut ? 1.0 : 0.0,
        static_cast<double>(brownoutCount_)};
    base::telemetry::Post("Simulation/BatteryState",
        [publisher = battery_, batterySample] { publisher->Set(batterySample); });
  }
 private:
  SimulationPowerConfig config_;
  double remainingAh_;
  double previousTime_{-1};
  int brownoutCount_{0};
  bool previouslyBrownedOut_{false};
  std::shared_ptr<nt::DoubleArrayPublisher> clock_, power_, battery_;
};
} // namespace meco::sim
#endif
