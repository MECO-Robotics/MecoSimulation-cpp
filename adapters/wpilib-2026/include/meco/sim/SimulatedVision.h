#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <units/angle.h>
#include <units/length.h>
#include <cmath>
#include <vector>

namespace meco::sim
{
// Ideal geometric visibility only: angles are full frustum widths. No occlusion
// or image noise is modeled. Range includes its boundary; FOV excludes its edge.
    inline std::vector<int> VisibleTags(const frc::Pose3d& robot,
                                       const frc::Transform3d& mount,
                                       const frc::AprilTagFieldLayout& layout,
                                       units::radian_t horizontalFov, units::radian_t verticalFov,
                                       units::meter_t maxRange) {
        const auto camera = robot.TransformBy(mount);
        std::vector<int> ids;
        for (const auto& tag : layout.GetTags()) {
            const frc::Transform3d relative{camera, tag.pose};
            const auto& p = relative.Translation();
            // Reject tag backs as well as targets outside the optical frustum.
            const frc::Transform3d tagToCamera{tag.pose, camera};
            if (p.X().value() <= 0 || p.Norm() > maxRange ||
                tagToCamera.X().value() <= 0) continue;
            if (std::abs(std::atan2(p.Y().value(), p.X().value())) < horizontalFov.value() / 2 &&
                std::abs(std::atan2(p.Z().value(), std::hypot(p.X().value(), p.Y().value()))) < verticalFov.value() / 2)
                ids.push_back(tag.ID);
        }
        return ids;
    }

}
