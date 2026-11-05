#pragma once
#ifndef __FRC_ROBORIO__
#include <base/FootprintAutoDrive.h>
#include <networktables/NetworkTableEntry.h>
#include <memory>
#include <stdexcept>
#include <utility>

namespace meco::sim {
// Empty entries contribute no obstacle. A present sample must be one
// [minX,minY,maxX,maxY] field-coordinate rectangle in metres.
inline void AppendInjectedFootprintObstacle(std::vector<base::FieldRectangle>& boxes,
                                            const nt::NetworkTableEntry& entry) {
  const auto observed = entry.GetDoubleArray({});
  if (observed.empty()) return;
  if (observed.size() != 4) throw std::runtime_error("invalid simulated footprint obstacle");
  boxes.push_back({observed[0], observed[1], observed[2], observed[3]});
}

// Caller retains navigation configuration, geometry, targets and scheduling.
// The command retains its shared outcome and publishes it after termination.
inline frc2::CommandPtr FootprintCommandWithOutcome(
    base::Drive& drive, base::AutoDriveTarget target,
    std::function<std::vector<base::FieldRectangle>()> obstacles,
    base::FootprintAutoDriveConfig config, nt::NetworkTableEntry outcomeEntry) {
  auto outcome = std::make_shared<base::FootprintAutoDriveOutcome>();
  outcomeEntry.SetString("running");
  return base::AutoDriveFootprintToPose(drive, std::move(target), std::move(obstacles),
                                       std::move(config), outcome)
      .FinallyDo([outcomeEntry, outcome](bool) mutable {
        outcomeEntry.SetString(base::FootprintOutcomeName(*outcome));
      });
}
} // namespace meco::sim
#endif
