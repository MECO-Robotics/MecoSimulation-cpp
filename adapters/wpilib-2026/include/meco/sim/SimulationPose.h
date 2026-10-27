#pragma once
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>

namespace meco::sim
{
    // The physics body origin is above the carpet. Robot visualization assets use
    // a floor-level local origin. Keep the offset rigidly attached to the chassis
    // so tilt and real elevation changes remain visible.
    inline frc::Pose3d FloorReferencedRobotPose(const frc::Pose3d &body,
                                                units::meter_t groundedBodyHeight)
    {
        return body.TransformBy(frc::Transform3d{
            frc::Translation3d{units::meter_t{0}, units::meter_t{0}, -groundedBodyHeight},
            frc::Rotation3d{}});
    }
}
