#pragma once
#ifndef __FRC_ROBORIO__
#include <memory>
#include <optional>
#include <frc2/command/CommandPtr.h>
#include <networktables/NetworkTableInstance.h>
namespace meco::sim {
struct PracticeMatchProfile {
  double autonomousSeconds, assessmentSeconds, teleopSeconds;
};
// Start the practice controller with the current simulated alliance station.
// The returned command may run while disabled.
frc2::CommandPtr StartPracticeMatch();
// Return a practice command only for live desktop simulation. Recorded replay
// and real robots never receive the command. Replay is initialized before the
// decision so the caller need not select a runtime mode.
std::optional<frc2::CommandPtr> TryStartPracticeMatch();
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
