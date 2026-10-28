#pragma once

#include "base/sim/SimulationWorld.h"
#include <array>
#include <cmath>
#include <span>

namespace base::sim {
// Conservative XY footprint check for field boxes with pitch-only rotations.
// Sloped boxes use their axis-aligned projection; obstacles above height are
// ignored. Dimensions, field bounds and collider margin are in metres. The
// margin applies to colliders only; touching the field boundary is blocked.
// Invalid or unsupported collider geometry returns false.
inline bool IsFootprintClear(double x, double y, double heading,
                             double halfLength, double halfWidth, double height,
                             double fieldLength, double fieldWidth, double margin,
                             std::span<const BodySpec> colliders) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(heading) ||
        !std::isfinite(halfLength) || !std::isfinite(halfWidth) ||
        !std::isfinite(height) || !std::isfinite(fieldLength) ||
        !std::isfinite(fieldWidth) || !std::isfinite(margin) ||
        halfLength <= 0 || halfWidth <= 0 || height < 0 ||
        fieldLength <= 0 || fieldWidth <= 0 || margin < 0) return false;
    const double c = std::cos(heading), s = std::sin(heading);
    const double rx = halfLength * std::abs(c) + halfWidth * std::abs(s);
    const double ry = halfLength * std::abs(s) + halfWidth * std::abs(c);
    if (x-rx <= 0 || x+rx >= fieldLength || y-ry <= 0 || y+ry >= fieldWidth) return false;
    for (const auto& b : colliders) {
        const auto& q = b.pose.rotation;
        const auto& p = b.pose.position;
        const auto& e = b.halfExtents;
        if (b.shape != BodySpec::Shape::Box ||
            !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(e.x) || !std::isfinite(e.y) || !std::isfinite(e.z) ||
            e.x <= 0 || e.y <= 0 || e.z <= 0 ||
            !std::isfinite(q.x) || !std::isfinite(q.y) ||
            !std::isfinite(q.z) || !std::isfinite(q.w) ||
            q.x != 0 || q.z != 0 ||
            std::abs(std::hypot(q.y, q.w) - 1.0) > 1e-9) return false;
        const double pitch = 2 * std::atan2(q.y, q.w);
        const double ex = std::abs(std::cos(pitch))*b.halfExtents.x +
                          std::abs(std::sin(pitch))*b.halfExtents.z;
        const double ez = std::abs(std::sin(pitch))*b.halfExtents.x +
                          std::abs(std::cos(pitch))*b.halfExtents.z;
        if (b.pose.position.z-ez > height) continue;
        const double dx=x-b.pose.position.x, dy=y-b.pose.position.y, ey=b.halfExtents.y;
        bool separated=false;
        for (const auto& axis : std::array<std::array<double,2>,4>{{{1,0},{0,1},{c,s},{-s,c}}}) {
            const double u=axis[0], v=axis[1];
            const double robot=halfLength*std::abs(u*c+v*s)+halfWidth*std::abs(-u*s+v*c);
            const double obstacle=ex*std::abs(u)+ey*std::abs(v);
            if (std::abs(dx*u+dy*v)>robot+obstacle+margin) separated=true;
        }
        if (!separated) return false;
    }
    return true;
}
} // namespace base::sim
