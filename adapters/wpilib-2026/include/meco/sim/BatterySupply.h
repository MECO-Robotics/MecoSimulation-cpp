#pragma once

#include <cmath>
#include <stdexcept>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>

namespace meco::sim {
// Estimated pack resistance; no state-of-charge, regeneration or branch losses.
struct BatterySupplyConfig {
    double openCircuitVoltage{12.0};
    double resistanceOhms{0.020};
    double auxiliaryCurrentAmps{0.0};
};

// Call once after the shared runtime completes a robot period. All registered
// plants consume this voltage next period. Does not advance any plant or clock.
// Runtime is templated to support both the Base and optional Jolt runtime.
template <class Runtime>
double UpdateBatterySupply(Runtime& runtime, BatterySupplyConfig config = {}) {
    if (!std::isfinite(config.openCircuitVoltage) || config.openCircuitVoltage <= 0 ||
        config.openCircuitVoltage > 16 || !std::isfinite(config.resistanceOhms) ||
        config.resistanceOhms < 0 || !std::isfinite(config.auxiliaryCurrentAmps) ||
        config.auxiliaryCurrentAmps < 0)
        throw std::invalid_argument("Invalid simulation battery configuration");
    const double current = runtime.SupplyCurrent() + config.auxiliaryCurrentAmps;
    if (!std::isfinite(current) || current < 0)
        throw std::invalid_argument("Invalid simulation battery current");
    const auto target = frc::sim::BatterySim::Calculate(
        units::volt_t{config.openCircuitVoltage}, units::ohm_t{config.resistanceOhms},
        {units::ampere_t{current}});
    // One under-relaxed iteration avoids the two-cycle caused by feeding a full
    // resistive sag step back through delayed motor current. Numerical stabilization,
    // not an identified battery capacitance or electrochemical time constant.
    const units::volt_t voltage{0.5 * (runtime.BusVoltage() + target.value())};
    runtime.SetBusVoltage(voltage.value());
    frc::sim::RoboRioSim::SetVInVoltage(voltage);
    frc::sim::RoboRioSim::SetVInCurrent(units::ampere_t{current});
    return voltage.value();
}
} // namespace meco::sim
