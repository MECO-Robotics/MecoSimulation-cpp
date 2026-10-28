#include "base/sim/SimulationTuning.h"
#include <cmath>
#include <limits>
#include <stdexcept>

int main() {
    base::sim::TuningInputs inputs{50, 5, 1, .05, 6, 40, .02,
        {{{.3, .3}, {.3, -.3}, {-.3, .3}, {-.3, -.3}}}, .1, .1};
    const auto normal = base::sim::DeriveSimulationTuning(inputs);
    if (!std::isfinite(normal.autoTranslationKP) || !std::isfinite(normal.autoRotationKP))
        throw std::runtime_error("ordinary tuning gains must be finite");
    for (bool translation : {true, false}) {
        auto tiny = inputs;
        (translation ? tiny.translationErrorScaleMeters : tiny.headingErrorScaleRadians) =
            std::numeric_limits<double>::denorm_min();
        try {
            (void)base::sim::DeriveSimulationTuning(tiny);
        } catch (const std::invalid_argument&) {
            continue;
        }
        throw std::runtime_error("overflowing tuning gain was accepted");
    }
}
