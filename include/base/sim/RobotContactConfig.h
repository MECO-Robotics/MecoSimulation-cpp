#pragma once
#include <array>

namespace base::sim
{
// All geometry is in metres relative to the unloaded carpet. Estimates, not calibration.
struct RobotContactConfig
{
    double chassisClearance{.0508}, panThickness{.003175};
    double panHalfLength{.3411}, panHalfWidth{.3411};
    double bumperFloorGap{.00635}, noodleDiameter{.06985};
    double backingThickness{.01905}, backingHeight{.127}, backingFloorGap{.00635};
    std::array<double, 5> compressionStrain{0, .25, .5, .7, .8};
    std::array<double, 5> compressionStressPa{0, 10000, 35000, 100000, 250000};
    double dampingRatio{.8}, friction{.65};
};
struct FoamResponse
{
    double pressure{}, tangent{};
};
void ValidateRobotContact(const RobotContactConfig &, double outerHalfLength, double outerHalfWidth);
// Linear segments, with the final positive slope continued past the final point.
FoamResponse EvaluateFoam(const RobotContactConfig &, double strain);
} // namespace base::sim
