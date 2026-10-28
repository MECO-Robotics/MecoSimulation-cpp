#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <units/time.h>

namespace meco::sim {

// A noiseless field-relative observation; standard deviations describe the
// observation supplied to an estimator and do not introduce simulated noise.
struct IdealVisionObservation {
    frc::Pose2d pose;
    units::second_t timestamp;
    std::array<double, 3> stdDevs;
};

// The caller determines whether its sensor has acquired or retained a fix.
// Inclusive field and height limits are evaluated in the supplied truth frame.
// Invalid timestamps, heights, or observation deviations are rejected. Truth
// poses with nonfinite orientation do not produce an observation.
inline std::optional<IdealVisionObservation> MakeIdealVisionObservation(
    const frc::Pose3d& truth, units::second_t timestamp,
    const frc::AprilTagFieldLayout& layout, bool hasFix,
    units::meter_t maxAbsHeight, std::array<double, 3> stdDevs) {
    if (!std::isfinite(timestamp.value()) || !std::isfinite(maxAbsHeight.value()) ||
        maxAbsHeight.value() < 0 ||
        !std::all_of(stdDevs.begin(), stdDevs.end(),
                     [](double deviation) { return std::isfinite(deviation) && deviation >= 0; }))
        throw std::invalid_argument("Invalid ideal vision observation configuration");
    const auto& q = truth.Rotation().GetQuaternion();
    if (!(hasFix && std::isfinite(q.W()) && std::isfinite(q.X()) &&
          std::isfinite(q.Y()) && std::isfinite(q.Z()) &&
          truth.X().value() >= 0 && truth.Y().value() >= 0 &&
          truth.X() <= layout.GetFieldLength() && truth.Y() <= layout.GetFieldWidth() &&
          std::abs(truth.Z().value()) <= maxAbsHeight.value())) return std::nullopt;
    return IdealVisionObservation{truth.ToPose2d(), timestamp, stdDevs};
}
}
