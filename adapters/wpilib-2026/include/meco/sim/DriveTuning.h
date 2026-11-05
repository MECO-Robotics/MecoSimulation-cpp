#pragma once

#include <base/Drive.h>
#include <base/sim/SimulationTuning.h>
#include <array>
#include <cstddef>
#include <stdexcept>

namespace meco::sim
{
// Apply only control gains. The caller configures the simulated robot's
// physical properties independently, regardless of the selected tuning. Tuning
// is an enum exposing Configured (for example base::SimulationTuning).
template <typename Tuning>
inline base::DriveConfig ConfigureDriveTuning(base::DriveConfig config, Tuning tuning,
                                               double motorTorqueConstantNmPerAmp,
                                               double translationErrorScaleMeters,
                                               double headingErrorScaleRadians)
{
    if (tuning == Tuning::Configured)
        return config;
    if (tuning != Tuning::Automatic)
        throw std::invalid_argument("Invalid simulation tuning selection");

    std::array<base::sim::ModuleLocation, 4> locations{};
    for (std::size_t i = 0; i < locations.size(); ++i)
        locations[i] = {config.modules[i].xMeters, config.modules[i].yMeters};
    const auto result = base::sim::DeriveSimulationTuning(
        {config.massKg, config.moi, config.cof, config.wheelRadius, config.driveRatio, config.slipCurrent,
         motorTorqueConstantNmPerAmp, locations, translationErrorScaleMeters, headingErrorScaleRadians});
    config.autoTranslationKP = result.autoTranslationKP;
    config.autoRotationKP = result.autoRotationKP;
    return config;
}
} // namespace meco::sim
