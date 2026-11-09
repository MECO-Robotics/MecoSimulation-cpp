#pragma once
#include <array>

namespace base::sim
{
// All geometry is in metres relative to the unloaded carpet. No robot profile is
// implied; callers must fill every field before passing this to a contact model.
struct RobotContactConfig
{
    double chassisClearance{}, panThickness{};
    double panHalfLength{}, panHalfWidth{};
    double bumperFloorGap{}, noodleDiameter{};
    double backingThickness{}, backingHeight{}, backingFloorGap{};
    std::array<double, 5> compressionStrain{};
    std::array<double, 5> compressionStressPa{};
    double dampingRatio{}, friction{};
};
struct FoamResponse
{
    double pressure{}, tangent{};
};
void ValidateRobotContact(const RobotContactConfig &, double outerHalfLength, double outerHalfWidth);
// Linear segments, with the final positive slope continued past the final point.
FoamResponse EvaluateFoam(const RobotContactConfig &, double strain);
} // namespace base::sim
