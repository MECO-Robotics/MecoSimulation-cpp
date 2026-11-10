#include "meco/sim/FuelModel.h"

#include "meco/sim/ParticleFieldContacts.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace meco::sim {
namespace {

const ParticleFieldContacts kFieldContacts{field2026::ParticleFieldColliders()};

bool Outside(const Particle& particle) {
  return particle.position.x < 0 || particle.position.x > field2026::kFieldLength ||
         particle.position.y < 0 || particle.position.y > field2026::kFieldWidth;
}

void ReturnToField(Particle& particle, double radius, double inset) {
  particle.position = {std::clamp(particle.position.x, inset, field2026::kFieldLength - inset),
                       std::clamp(particle.position.y, inset, field2026::kFieldWidth - inset),
                       radius};
  particle.velocity = {};
}

frc::Translation3d Translation(ParticlePoint point) {
  return {units::meter_t{point.x}, units::meter_t{point.y}, units::meter_t{point.z}};
}

ParticlePoint Point3d(const frc::Translation3d& point) {
  return {point.X().value(), point.Y().value(), point.Z().value()};
}

ParticlePoint World(ParticlePoint local, const frc::Pose3d& robot) {
  return Point3d(robot.Translation() + Translation(local).RotateBy(robot.Rotation()));
}

void ValidateConfig(const FuelModelConfig& config) {
  const auto positive = [](double value) { return std::isfinite(value) && value > 0; };
  const auto nonnegative = [](double value) { return std::isfinite(value) && value >= 0; };
  const auto& game = config.game;
  const auto& robot = config.robot;
  const std::array robotValues{
      robot.hopperFloorMetres, robot.hopperCeilingMetres, robot.hopperBackMetres,
      robot.hopperHalfWidthMetres, robot.hopperExtensionScale, robot.burstLateralCenter,
      robot.burstLateralSpacingMetres, robot.minHoodDeflectionRadians,
      robot.maxHoodDeflectionRadians, robot.shooterRestDeflectionRadians,
      robot.minLaunchPitchRadians, robot.maxLaunchPitchRadians, robot.launchPitchNoiseRadians,
      robot.launchSpeedNoiseFraction, robot.shooterExitXMetres, robot.releaseHeightMetres,
      robot.projectileSpeedPerFlywheelRps, robot.collectionMinExtensionMetres,
      robot.collectionMouthXMetres, robot.collectionMouthDepthMetres,
      robot.collectionHalfWidthMetres, robot.minFeedFlywheelRps, robot.burstResetPeriods,
      robot.minLaunchGapSeconds, robot.launchGapSeconds, robot.launchGapNoiseSeconds,
      robot.burstPeriodSeconds};
  if (!std::all_of(robotValues.begin(), robotValues.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw std::invalid_argument("Fuel robot configuration must be finite");
  }
  if (!positive(game.particleDiameterMetres) || !positive(game.gravityMetresPerSecondSquared) ||
      !positive(game.maxStepSeconds) || !positive(game.particleStepSeconds) ||
      !nonnegative(game.fieldReturnInsetMetres) ||
      game.fieldReturnInsetMetres * 2 >= std::min(field2026::kFieldLength, field2026::kFieldWidth) ||
      !positive(game.scoringHubHalfWidthMetres) || !nonnegative(game.scoredReturnOffsetMetres) ||
      !nonnegative(game.scoredReturnSpeedMetresPerSecond) ||
      !nonnegative(game.scoredReturnAngleNoiseRadians) || !nonnegative(game.groundBounceScale) ||
      !nonnegative(game.floorDampingPerSecond) || !std::isfinite(game.fieldRestitution) ||
      game.fieldRestitution < 0 || game.fieldRestitution > 1 || robot.capacity <= 0 ||
      !std::isfinite(game.particleRestitution) || game.particleRestitution < 0 ||
      game.particleRestitution > 1 || robot.burstSize <= 0 ||
      !positive(robot.hopperExtensionScale) || !positive(robot.hopperHalfWidthMetres) ||
      !positive(robot.hopperCeilingMetres - robot.hopperFloorMetres) ||
      !positive(robot.burstLateralSpacingMetres) ||
      !positive(robot.maxHoodDeflectionRadians - robot.minHoodDeflectionRadians) ||
      !positive(robot.maxLaunchPitchRadians - robot.minLaunchPitchRadians) ||
      !nonnegative(robot.launchPitchNoiseRadians) || !nonnegative(robot.launchSpeedNoiseFraction) ||
      !positive(robot.projectileSpeedPerFlywheelRps) || robot.launchSpeedNoiseFraction > 1 ||
      !positive(robot.collectionMouthDepthMetres) ||
      !positive(robot.collectionHalfWidthMetres) || !positive(robot.minFeedFlywheelRps) ||
      !positive(robot.burstResetPeriods) || !positive(robot.minLaunchGapSeconds) ||
      !positive(robot.launchGapSeconds) || !nonnegative(robot.launchGapNoiseSeconds) ||
      !positive(robot.burstPeriodSeconds) || robot.bumperBoxes.empty()) {
    throw std::invalid_argument("Invalid fuel model configuration");
  }
  for (const auto& box : robot.bumperBoxes) {
    const auto& center = box.center;
    const auto& extent = box.halfExtents;
    if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(center.z) ||
        !positive(extent.x) || !positive(extent.y) || !positive(extent.z)) {
      throw std::invalid_argument("Invalid fuel robot bumper geometry");
    }
  }
}

}  // namespace

FuelModel::FuelModel(unsigned seed, FuelModelConfig config)
    : seed_(seed), config_(std::move(config)), random_(seed) {
  ValidateConfig(config_);
  Reset();
}

void FuelModel::Reset() {
  random_.seed(seed_);
  floor_.clear();
  airborne_.clear();
  stored_ = scored_ = launched_ = pending_ = burstIndex_ = 0;
  time_ = 0;
  lastBurst_ = nextShot_ = -1e9;
}

void FuelModel::AddFloorFuel(ParticlePoint point) {
  point.z = config_.game.particleDiameterMetres / 2;
  floor_.push_back({point, {}});
}

void FuelModel::SetStoredFuel(int count) {
  stored_ = std::clamp(count, 0, config_.robot.capacity);
}

double FuelModel::Random(double amplitude) {
  return std::uniform_real_distribution<double>{-amplitude, amplitude}(random_);
}

double FuelModel::LaunchPitch(double hoodRotations) const {
  const auto& robot = config_.robot;
  const double deflection = std::clamp(hoodRotations * 2 * std::numbers::pi +
                                           robot.shooterRestDeflectionRadians,
                                       robot.minHoodDeflectionRadians,
                                       robot.maxHoodDeflectionRadians);
  return std::numbers::pi / 2 - deflection;
}

std::vector<ParticlePoint> FuelModel::HopperCenters(int count, double extensionMetres,
                                                    const frc::Pose3d& robotPose) const {
  const auto& game = config_.game;
  const auto& robot = config_.robot;
  const double diameter = game.particleDiameterMetres;
  const double radius = diameter / 2;
  std::vector<ParticlePoint> centers;
  count = std::clamp(count, 0, robot.capacity);
  centers.reserve(static_cast<std::size_t>(count));
  const double maxX = std::max(0., extensionMetres) * robot.hopperExtensionScale;
  int zi = 0;
  for (double z = robot.hopperFloorMetres + radius;
       z <= robot.hopperCeilingMetres - radius + 1e-9;
       z += std::numbers::sqrt3 / 2 * diameter, ++zi) {
    int xi = 0;
    for (double x = robot.hopperBackMetres + radius + (zi % 2 ? radius : 0);
         x <= maxX - radius + 1e-9; x += diameter, ++xi) {
      for (double y = -robot.hopperHalfWidthMetres + radius + ((zi + xi) % 2 ? radius : 0);
           y <= robot.hopperHalfWidthMetres - radius + 1e-9; y += diameter) {
        if (static_cast<int>(centers.size()) == count) return centers;
        centers.push_back(World({x, y, z}, robotPose));
      }
    }
  }
  return centers;
}

void FuelModel::Launch(const FuelModelInput& input, int index) {
  const auto& robot = config_.robot;
  const double lateral = (index - robot.burstLateralCenter) * robot.burstLateralSpacingMetres;
  const ParticlePoint position = World(
      {robot.shooterExitXMetres, lateral, robot.releaseHeightMetres}, input.robot.pose);
  const double pitch = std::clamp(
      LaunchPitch(input.hoodRotations) + Random(robot.launchPitchNoiseRadians),
      robot.minLaunchPitchRadians, robot.maxLaunchPitchRadians);
  const double speed = std::abs(input.flywheelRps) * robot.projectileSpeedPerFlywheelRps *
                       (1 + Random(robot.launchSpeedNoiseFraction));
  const ParticlePoint muzzleVelocity = Point3d(
      Translation({speed * std::cos(pitch), 0, speed * std::sin(pitch)})
          .RotateBy(input.robot.pose.Rotation()));
  const ParticlePoint com = World(input.robot.centerOfMass, input.robot.pose);
  const double dx = position.x - com.x, dy = position.y - com.y, dz = position.z - com.z;
  const auto& w = input.robot.angularVelocity;
  airborne_.push_back({position,
                       {muzzleVelocity.x + input.robot.velocity.x + w.y * dz - w.z * dy,
                        muzzleVelocity.y + input.robot.velocity.y + w.z * dx - w.x * dz,
                        muzzleVelocity.z + input.robot.velocity.z + w.x * dy - w.y * dx}});
  ++launched_;
}

void FuelModel::Step(double dt, const FuelModelInput& input) {
  const auto& game = config_.game;
  const auto& robot = config_.robot;
  if (!std::isfinite(dt) || dt <= 0 || dt > game.maxStepSeconds) return;

  const double radius = game.particleDiameterMetres / 2;
  time_ += dt;
  if (input.collecting && input.intakeExtensionMetres > robot.collectionMinExtensionMetres &&
      stored_ < robot.capacity) {
    for (auto it = floor_.begin(); it != floor_.end() && stored_ < robot.capacity;) {
      const auto local = (Translation(it->position) - input.robot.pose.Translation())
                             .RotateBy(-input.robot.pose.Rotation());
      const double x = local.X().value(), y = local.Y().value();
      if (x >= robot.collectionMouthXMetres &&
          x <= robot.collectionMouthXMetres + robot.collectionMouthDepthMetres + radius &&
          std::abs(y) <= robot.collectionHalfWidthMetres + radius) {
        ++stored_;
        it = floor_.erase(it);
      } else {
        ++it;
      }
    }
  }

  if (input.feeding && std::abs(input.flywheelRps) >= robot.minFeedFlywheelRps) {
    if (!pending_ && time_ - lastBurst_ >= robot.burstPeriodSeconds) {
      pending_ = robot.burstSize;
      burstIndex_ = 0;
      lastBurst_ = time_ - lastBurst_ > robot.burstResetPeriods * robot.burstPeriodSeconds
                       ? time_
                       : lastBurst_ + robot.burstPeriodSeconds;
      nextShot_ = lastBurst_;
    }
    while (pending_ && time_ >= nextShot_) {
      if (!stored_) {
        pending_ = 0;
        break;
      }
      --stored_;
      Launch(input, burstIndex_++);
      --pending_;
      if (pending_) {
        nextShot_ += std::max(robot.minLaunchGapSeconds,
                              robot.launchGapSeconds + Random(robot.launchGapNoiseSeconds));
      }
    }
  }

  const int steps = std::max(1, static_cast<int>(std::ceil(dt / game.particleStepSeconds)));
  const double h = dt / steps;
  const double attenuation = std::exp(-game.floorDampingPerSecond * h);
  const double hubX = input.redAlliance ? field2026::kRedHubX : field2026::kBlueHubX;
  const double hubY = field2026::kHubY;
  const double hubHeight = field2026::kHubOpeningHeightMetres;

  for (int step = 0; step < steps; ++step) {
    for (auto it = airborne_.begin(); it != airborne_.end();) {
      const auto previous = it->position;
      it->position.x += it->velocity.x * h;
      it->position.y += it->velocity.y * h;
      it->position.z += it->velocity.z * h - .5 * game.gravityMetresPerSecondSquared * h * h;
      it->velocity.z -= game.gravityMetresPerSecondSquared * h;
      const bool crossing = previous.z >= hubHeight && it->position.z < hubHeight && it->velocity.z < 0;
      const double fraction = crossing ? (previous.z - hubHeight) / (previous.z - it->position.z) : 0;
      const double x = previous.x + (it->position.x - previous.x) * fraction;
      const double y = previous.y + (it->position.y - previous.y) * fraction;
      if (Outside(*it)) {
        ReturnToField(*it, radius, game.fieldReturnInsetMetres);
        floor_.push_back(*it);
        it = airborne_.erase(it);
      } else if (crossing && std::abs(x - hubX) <= game.scoringHubHalfWidthMetres &&
                 std::abs(y - hubY) <= game.scoringHubHalfWidthMetres) {
        if (input.scoringEligible) ++scored_;
        const double direction = input.redAlliance ? -1 : 1;
        const double angle = (direction > 0 ? 0 : std::numbers::pi) +
                             Random(game.scoredReturnAngleNoiseRadians);
        floor_.push_back({{hubX + direction * game.scoredReturnOffsetMetres, hubY, radius},
                          {game.scoredReturnSpeedMetresPerSecond * std::cos(angle),
                           game.scoredReturnSpeedMetresPerSecond * std::sin(angle), 0}});
        it = airborne_.erase(it);
      } else if (it->position.z <= radius) {
        floor_.push_back({{it->position.x, it->position.y, radius},
                          {it->velocity.x * game.groundBounceScale,
                           it->velocity.y * game.groundBounceScale, 0}});
        it = airborne_.erase(it);
      } else {
        ++it;
      }
    }

    for (auto& particle : floor_) {
      particle.position.x += particle.velocity.x * h;
      particle.position.y += particle.velocity.y * h;
      particle.velocity.x *= attenuation;
      particle.velocity.y *= attenuation;
    }
    for (auto& particle : floor_) {
      if (Outside(particle)) ReturnToField(particle, radius, game.fieldReturnInsetMetres);
    }

    kFieldContacts.Resolve(floor_, radius, game.fieldRestitution);
    kFieldContacts.Resolve(airborne_, radius, game.fieldRestitution);
    ResolveParticleContacts(floor_, airborne_, input.robot, radius, game.fieldRestitution,
                            game.particleRestitution, robot.bumperBoxes);

    // Particle and bumper impulses can push fuel through a field boundary.
    for (auto& particle : floor_) {
      if (Outside(particle)) ReturnToField(particle, radius, game.fieldReturnInsetMetres);
    }
    for (auto it = airborne_.begin(); it != airborne_.end();) {
      if (Outside(*it)) {
        ReturnToField(*it, radius, game.fieldReturnInsetMetres);
        floor_.push_back(*it);
        it = airborne_.erase(it);
      } else {
        ++it;
      }
    }

    // A bumper or ball may lift resting fuel back into ballistic motion.
    for (auto it = floor_.begin(); it != floor_.end();) {
      if (it->position.z > radius + 1e-6 || it->velocity.z > 1e-6) {
        airborne_.push_back(*it);
        it = floor_.erase(it);
      } else {
        it->position.z = radius;
        it->velocity.z = 0;
        ++it;
      }
    }
  }
}

}  // namespace meco::sim
