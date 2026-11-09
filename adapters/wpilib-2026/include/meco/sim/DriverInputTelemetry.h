#pragma once

#include <base/Telemetry.h>
#include <base/TelemetryPublishers.h>
#include <frc/geometry/Rotation2d.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>
#include <ntcore_cpp.h>
#include <array>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <string_view>
#include <utility>
#include <memory>

namespace meco::sim {
// Simulation-only telemetry for a robot's chosen driver inputs. The robot
// remains responsible for reading controls and selecting the heading offset.
class DriverInputTelemetry {
 public:
  explicit DriverInputTelemetry(bool enabledByDefault)
      : override_(ReadOverride()), enabled_(override_.value_or(enabledByDefault)) {
    auto nt = nt::NetworkTableInstance::GetDefault();
    consumed_ = std::make_shared<nt::DoubleArrayPublisher>(
        nt.GetDoubleArrayTopic("/Simulation/NTGamepad/Consumed").Publish());
    driverStation_ = std::make_shared<nt::DoubleArrayPublisher>(
        nt.GetDoubleArrayTopic("/Simulation/NTGamepad/DriverStationCommand").Publish());
    base::telemetry::PostDashboardBoolean("Simulation/NTGamepad/Enabled", enabled_);
  }
  bool Selected(bool selected) const { return ReadOverride().value_or(selected); }
  template <class GamepadState>
  void Publish(const GamepadState &state, double leftY, double leftX, double rightX,
               double x, double y, double omega, frc::Rotation2d heading, bool hasGamepad) {
    const int64_t timestamp = nt::Now();
    const double fieldX = x * heading.Cos() - y * heading.Sin();
    const double fieldY = x * heading.Sin() + y * heading.Cos();
    const std::array<double, 7> consumed{static_cast<double>(state.sequence),
        static_cast<double>(state.sourceTimestampUs), static_cast<double>(timestamp),
        fieldX, fieldY, omega, state.accepted ? 1.0 : 0.0};
    const std::array<double, 7> driverStation{static_cast<double>(timestamp), leftY, leftX,
        rightX, fieldX, fieldY, omega};
    base::telemetry::Post("Simulation/NTGamepad/Consumed",
        [publisher = consumed_, consumed, timestamp] {
          publisher->Set(consumed, timestamp);
        });
    base::telemetry::Post("Simulation/NTGamepad/DriverStationCommand",
        [publisher = driverStation_, driverStation, timestamp] {
          publisher->Set(driverStation, timestamp);
        });
    base::telemetry::PostDashboardBoolean("Simulation/NTGamepad/Accepted", hasGamepad && state.accepted);
    base::telemetry::PostDashboardNumber("Simulation/NTGamepad/AgeMs",
        std::isfinite(state.ageMs) ? state.ageMs : -1.0);
    base::telemetry::PostDashboardNumber("Simulation/NTGamepad/Sequence", state.sequence);
  }
 private:
  static std::optional<bool> ReadOverride() {
    if (const char *value = std::getenv("FRC_SIM_NT_GAMEPAD")) return std::string_view{value} == "1";
    return std::nullopt;
  }
  std::optional<bool> override_;
  bool enabled_;
  std::shared_ptr<nt::DoubleArrayPublisher> consumed_, driverStation_;
};
}
