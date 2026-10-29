#pragma once
#ifndef __FRC_ROBORIO__
#include <memory>
#include <networktables/NetworkTableInstance.h>
namespace meco::sim {
struct PracticeMatchProfile {
  double autonomousSeconds, assessmentSeconds, teleopSeconds;
};
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
