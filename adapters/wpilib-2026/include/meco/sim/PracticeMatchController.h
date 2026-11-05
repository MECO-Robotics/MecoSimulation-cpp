#pragma once
#ifndef __FRC_ROBORIO__
#include <memory>
#include <frc2/command/CommandPtr.h>
#include <networktables/NetworkTableInstance.h>
namespace meco::sim {
struct PracticeMatchProfile {
  double autonomousSeconds, assessmentSeconds, teleopSeconds;
};
// Start the practice controller with the current simulated alliance station.
// The returned command may run while disabled.
frc2::CommandPtr StartPracticeMatch();
// Desktop only. Call Update once per robot period. No physics or clock stepping.
class PracticeMatchController {
 public:
  explicit PracticeMatchController(PracticeMatchProfile profile,
      nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault());
  ~PracticeMatchController();
  PracticeMatchController(const PracticeMatchController&) = delete;
  PracticeMatchController& operator=(const PracticeMatchController&) = delete;
  void Update();
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
#endif
