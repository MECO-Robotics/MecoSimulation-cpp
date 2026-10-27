#include "base/sim/RobotContactConfig.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace base::sim
{
void ValidateRobotContact(const RobotContactConfig &c, double hx, double hy)
{
    const double positive[]{c.chassisClearance,
                            c.panThickness,
                            c.panHalfLength,
                            c.panHalfWidth,
                            c.noodleDiameter,
                            c.backingThickness,
                            c.backingHeight,
                            hx,
                            hy};
    for (double v : positive)
        if (!std::isfinite(v) || v <= 0)
            throw std::invalid_argument("Contact dimensions must be positive finite");
    for (double v : {c.bumperFloorGap, c.backingFloorGap, c.dampingRatio, c.friction})
        if (!std::isfinite(v) || v < 0)
            throw std::invalid_argument("Invalid contact material or floor gap");
    const double inset = c.noodleDiameter + c.backingThickness;
    if (std::max(inset, 1.5 * c.noodleDiameter) >= std::min(hx, hy) || c.panHalfLength > hx - inset + 1e-9 ||
        c.panHalfWidth > hy - inset + 1e-9 || c.backingFloorGap + 1e-9 < c.bumperFloorGap ||
        c.backingFloorGap + c.backingHeight > c.bumperFloorGap + 2 * c.noodleDiameter + 1e-9)
        throw std::invalid_argument("Pan or backing lies outside bumper envelope");
    if (c.compressionStrain[0] != 0 || c.compressionStressPa[0] != 0)
        throw std::invalid_argument("Foam curve must start at zero");
    for (size_t i = 0; i < c.compressionStrain.size(); ++i)
    {
        if (!std::isfinite(c.compressionStrain[i]) || !std::isfinite(c.compressionStressPa[i]) ||
            c.compressionStrain[i] < 0 || c.compressionStrain[i] >= 1 || c.compressionStressPa[i] < 0 ||
            (i && (c.compressionStrain[i] <= c.compressionStrain[i - 1] ||
                   c.compressionStressPa[i] <= c.compressionStressPa[i - 1])))
            throw std::invalid_argument("Foam curve must have strictly increasing finite strain and stress");
    }
}
FoamResponse EvaluateFoam(const RobotContactConfig &c, double strain)
{
    if (!std::isfinite(strain) || strain < 0)
        throw std::invalid_argument("Invalid compression strain");
    size_t i = 1;
    while (i + 1 < c.compressionStrain.size() && strain > c.compressionStrain[i])
        ++i;
    const double k = (c.compressionStressPa[i] - c.compressionStressPa[i - 1]) /
                     (c.compressionStrain[i] - c.compressionStrain[i - 1]);
    return {c.compressionStressPa[i - 1] + k * (strain - c.compressionStrain[i - 1]), k};
}
} // namespace base::sim
