#pragma once

// Opt-in desktop measurement only. No changes to control or simulation timing.
#ifndef __FRC_ROBORIO__
#include <algorithm>
#include <array>
#include "base/Telemetry.h"
#include <cstdlib>
#include <string_view>
#include <frc/DriverStation.h>
#include <hal/simulation/DriverStationData.h>
#include <frc/RobotController.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <ntcore_cpp.h>
namespace meco::sim {
inline bool LatencyProbeEnabled() {
  static const bool enabled = std::getenv("FRC_SIM_LATENCY_PROBE") != nullptr;
  return enabled;
}
// Application calls once per simulation robot loop, independently of GUI redraws.
// Raw HAL joystick writes are not Driver Station packets. This publishes the
// cache for the next ordinary DriverStation refresh without advancing physics.
inline void PublishDriverStationPacket() {
  static const bool enabled=[] {
    const char* value=std::getenv("FRC_SIM_DS_PACKET_CLOCK");
    return !value || std::string_view{value}!="0";
  }();
  if(enabled)HALSIM_NotifyDriverStationNewData();
}
inline std::array<double, 9> latencySample{};
inline double ObserveJoystick(double value) {
  if (LatencyProbeEnabled()) {
    latencySample[3]=value;
    latencySample[4]=nt::Now();
  }
  return value;
}
inline void BeginLatencyProbe() {
  if (!LatencyProbeEnabled()) return;
  latencySample[0]=nt::Now();
  latencySample[2]=-frc::DriverStation::GetStickAxis(0,1);
  HAL_JoystickAxes raw{};
  HALSIM_GetJoystickAxes(0,&raw);
  latencySample[6]=raw.count>1?-raw.axes[1]:0;
  latencySample[7]=frc::DriverStation::IsDSAttached();
  latencySample[8]=frc::DriverStation::IsEnabled();
}
inline void EndLatencyProbe() {
  if (!LatencyProbeEnabled()) return;
  latencySample[1]=nt::Now();
  latencySample[5]=frc::RobotController::GetFPGATime();
  static auto publisher=nt::NetworkTableInstance::GetDefault()
      .GetDoubleArrayTopic("/Diagnostics/Latency/RobotLoop").Publish();
  publisher.Set(latencySample);
  auto captured=latencySample;
  base::telemetry::Post("LatencyProbe", [captured] {
    std::array<double,10> queued{};
    std::copy(captured.begin(),captured.end(),queued.begin());
    queued[9]=nt::Now();
    static auto background=nt::NetworkTableInstance::GetDefault()
        .GetDoubleArrayTopic("/Diagnostics/Latency/Background").Publish();
    background.Set(queued);
  });
}
}
#else
namespace meco::sim {
inline double ObserveJoystick(double value) { return value; }
inline void BeginLatencyProbe() {}
inline void EndLatencyProbe() {}
}
#endif
