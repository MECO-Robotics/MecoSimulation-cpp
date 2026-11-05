#pragma once

#include <base/NtGamepad.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <array>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <string_view>

namespace meco::sim {
// Simulation-only telemetry for a robot's chosen driver inputs. The robot
// remains responsible for reading controls and selecting the heading offset.
class DriverInputTelemetry {
 public:
  explicit DriverInputTelemetry(bool enabledByDefault)
      : override_(ReadOverride()), enabled_(override_.value_or(enabledByDefault)) {
    auto nt = nt::NetworkTableInstance::GetDefault();
    consumed_ = nt.GetDoubleArrayTopic("/Simulation/NTGamepad/Consumed").Publish();
    driverStation_ = nt.GetDoubleArrayTopic("/Simulation/NTGamepad/DriverStationCommand").Publish();
    frc::SmartDashboard::PutBoolean("Simulation/NTGamepad/Enabled", enabled_);
  }
  bool Selected(bool selected) const { return ReadOverride().value_or(selected); }
  void Publish(const base::NtGamepad::State &state, double leftY, double leftX, double rightX,
               double x, double y, double omega, frc::Rotation2d heading, bool hasGamepad) {
    const int64_t timestamp = nt::Now();
    const double fieldX = x * heading.Cos() - y * heading.Sin();
    const double fieldY = x * heading.Sin() + y * heading.Cos();
    const std::array<double, 7> consumed{static_cast<double>(state.sequence),
        static_cast<double>(state.sourceTimestampUs), static_cast<double>(timestamp),
        fieldX, fieldY, omega, state.accepted ? 1.0 : 0.0};
    const std::array<double, 7> driverStation{static_cast<double>(timestamp), leftY, leftX,
        rightX, fieldX, fieldY, omega};
    consumed_.Set(consumed, timestamp);
    driverStation_.Set(driverStation, timestamp);
    frc::SmartDashboard::PutBoolean("Simulation/NTGamepad/Accepted", hasGamepad && state.accepted);
    frc::SmartDashboard::PutNumber("Simulation/NTGamepad/AgeMs", std::isfinite(state.ageMs) ? state.ageMs : -1.0);
    frc::SmartDashboard::PutNumber("Simulation/NTGamepad/Sequence", state.sequence);
  }
 private:
  static std::optional<bool> ReadOverride() {
    if (const char *value = std::getenv("FRC_SIM_NT_GAMEPAD")) return std::string_view{value} == "1";
    return std::nullopt;
  }
  std::optional<bool> override_;
  bool enabled_;
  nt::DoubleArrayPublisher consumed_, driverStation_;
};
}
