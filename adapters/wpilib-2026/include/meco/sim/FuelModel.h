#pragma once

#include "meco/sim/Field2026.h"
#include "meco/sim/ParticleContacts.h"

#include <cstdint>
#include <optional>
#include <random>
#include <vector>

namespace meco::sim {

// Shared 2026 FUEL motion and field-scoring approximation. Gravity, field
// contact, and particle collisions are substepped approximations; drag, spin,
// and reaction forces on the robot are omitted. Seeded launch randomness is
// repeatable. Robot dimensions, intake/hopper geometry, shooter conversion,
// and feed timing come from the consumer's FuelRobotConfig.
struct FuelGameConfig {
  double particleDiameterMetres{5.5 * .0254};
  double gravityMetresPerSecondSquared{9.80665};
  double maxStepSeconds{.25};
  double particleStepSeconds{.005};
  double fieldReturnInsetMetres{.30};
  double scoringHubHalfWidthMetres{.6096};
  double scoredReturnOffsetMetres{.65};
  double scoredReturnSpeedMetresPerSecond{3};
  double scoredReturnAngleNoiseRadians{45 * 3.14159265358979323846 / 180};
  double groundBounceScale{.15};
  double floorDampingPerSecond{2};
  double fieldRestitution{.25};
  double particleRestitution{.30};
};

// Values tied to a particular robot's hopper, intake, shooter, and bumper.
// Supply these explicitly; FuelModel does not assume a specific robot layout.
struct FuelRobotConfig {
  std::vector<ContactBox> bumperBoxes;
  int capacity{};
  int burstSize{};
  double hopperFloorMetres{};
  double hopperCeilingMetres{};
  double hopperBackMetres{};
  double hopperHalfWidthMetres{};
  double hopperExtensionScale{};
  double burstLateralCenter{};
  double burstLateralSpacingMetres{};
  double minHoodDeflectionRadians{};
  double maxHoodDeflectionRadians{};
  double shooterRestDeflectionRadians{};
  double minLaunchPitchRadians{};
  double maxLaunchPitchRadians{};
  double launchPitchNoiseRadians{};
  double launchSpeedNoiseFraction{};
  double shooterExitXMetres{};
  double releaseHeightMetres{};
  double projectileSpeedPerFlywheelRps{};
  double collectionMinExtensionMetres{};
  double collectionMouthXMetres{};
  double collectionMouthDepthMetres{};
  double collectionHalfWidthMetres{};
  double minFeedFlywheelRps{};
  double burstResetPeriods{};
  double minLaunchGapSeconds{};
  double launchGapSeconds{};
  double launchGapNoiseSeconds{};
  double burstPeriodSeconds{};
};

struct FuelModelConfig {
  FuelGameConfig game;
  FuelRobotConfig robot;
};

struct FuelModelInput {
  ContactBody robot;
  double intakeExtensionMetres{};
  double hoodRotations{};
  double flywheelRps{};
  bool collecting{};
  bool feeding{};
  bool redAlliance{};
  // The consuming robot supplies season-specific hub eligibility policy.
  bool scoringEligible{};
};

class FuelModel {
 public:
  FuelModel(unsigned seed, FuelModelConfig config);

  void Reset();
  void Step(double dt, const FuelModelInput& input);
  void AddFloorFuel(ParticlePoint point);
  void SetStoredFuel(int count);

  int StoredFuel() const { return stored_; }
  int SuccessfulScores() const { return scored_; }
  int LaunchedFuel() const { return launched_; }
  int PendingBurstShots() const { return pending_; }
  const std::vector<Particle>& FloorFuel() const { return floor_; }
  const std::vector<Particle>& Projectiles() const { return airborne_; }
  const FuelModelConfig& Config() const { return config_; }

  double LaunchPitch(double hoodRotations) const;
  std::vector<ParticlePoint> HopperCenters(int count, double extensionMetres,
                                           const frc::Pose3d& robot) const;

 private:
  void Launch(const FuelModelInput& input, int index);
  double Random(double amplitude);

  unsigned seed_;
  FuelModelConfig config_;
  std::mt19937 random_;
  std::vector<Particle> floor_, airborne_;
  int stored_{}, scored_{}, launched_{}, pending_{}, burstIndex_{};
  double time_{}, lastBurst_{-1e9}, nextShot_{-1e9};
};

}  // namespace meco::sim
