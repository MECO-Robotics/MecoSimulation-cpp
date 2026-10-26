#pragma once

#include "base/Drive.h"
#include "base/sim/Arena.h"
#include "base/sim/DrivePhysics.h"

#include <memory>

namespace meco::sim
{
class DriveSimulationSession final : public base::DriveSimulationAdapter
{
  public:
    DriveSimulationSession(const base::DriveConfig &, base::sim::Backend);
    ~DriveSimulationSession() override;
    DriveSimulationSession(const DriveSimulationSession &) = delete;
    DriveSimulationSession &operator=(const DriveSimulationSession &) = delete;

    base::sim::Arena &CreateArena(base::sim::ArenaScene scene = base::sim::ArenaScene::Generic,
                                  std::uint32_t seed = 2026, base::sim::Vec3 robotCenterOfMass = {});
    void Attach(base::Drive &) override;
    void Reset(const frc::Pose2d &) override;
    base::DriveSimulationTruth Truth() const override;
    // Complete physical pose (including tilt) and contact loads; snapshot only.
    base::sim::DriveTruth PhysicsTruth() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Creates the optional desktop adapter for a MECO Base Drive. Attach it only
// after WPILib has selected the desktop simulation backend.
std::unique_ptr<DriveSimulationSession> CreateDriveSimulationAdapter(
    const base::DriveConfig &, base::sim::Backend backend = base::sim::BackendFromEnvironment());
} // namespace meco::sim
