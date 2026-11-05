#pragma once

#include "base/Mechanism.h"

#include <functional>
#include <memory>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace meco::sim
{
// SI motor electrical/mechanical constants. One motor's constants are scaled
// by the number of configured motors in base::MotorConfig.
struct LoadedMotorModel
{
    double resistanceOhms{};
    double torqueConstantNmPerAmp{};
    double velocityConstantRadPerSecondPerVolt{};
    double outputInertiaKgM2{0.025};
    double viscousDragNmPerRadPerSecond{0.0001};
    double maximumPhysicsStepSeconds{0.0002};
    double maximumUpdateSeconds{1.0};
    double loadDirectionSpeedThresholdRadPerSecond{0.01};
};

// Derive a single motor's constants using the ideal no-load speed (free
// current is not modeled). Pass one motor's nominal voltage, stall current,
// stall torque and free speed in V, A, N m and rpm. Configure output inertia
// and other simulation parameters on the returned model as needed.
inline LoadedMotorModel LoadedMotorModelFromStallAndFreeSpeed(double nominalVoltage,
                                                              double stallCurrentAmps,
                                                              double stallTorqueNm,
                                                              double freeSpeedRpm)
{
    if (!std::isfinite(nominalVoltage) || nominalVoltage <= 0 ||
        !std::isfinite(stallCurrentAmps) || stallCurrentAmps <= 0 ||
        !std::isfinite(stallTorqueNm) || stallTorqueNm <= 0 ||
        !std::isfinite(freeSpeedRpm) || freeSpeedRpm <= 0)
        throw std::invalid_argument("Invalid loaded motor electrical specifications");
    LoadedMotorModel model;
    model.resistanceOhms = nominalVoltage / stallCurrentAmps;
    model.torqueConstantNmPerAmp = stallTorqueNm / stallCurrentAmps;
    model.velocityConstantRadPerSecondPerVolt = freeSpeedRpm * 2 * std::numbers::pi / 60 / nominalVoltage;
    if (!std::isfinite(model.resistanceOhms) || model.resistanceOhms <= 0 ||
        !std::isfinite(model.torqueConstantNmPerAmp) || model.torqueConstantNmPerAmp <= 0 ||
        !std::isfinite(model.velocityConstantRadPerSecondPerVolt) ||
        model.velocityConstantRadPerSecondPerVolt <= 0)
        throw std::invalid_argument("Invalid derived loaded motor constants");
    return model;
}

// Adds a base::MakeMotorIO overload for a simulated motor whose load torque is sampled dynamically on
// each robot-period integration step. The callback returns an opposing load
// magnitude in N m at the output shaft; negative values are treated as zero.
// The caller owns the captured load state and must keep it alive as long as
// the returned IO. Must be constructed in desktop simulation.
} // namespace meco::sim

namespace base
{
std::unique_ptr<MotorIO> MakeMotorIO(const MotorConfig &motor,
                                     meco::sim::LoadedMotorModel model,
                                     std::function<double()> loadTorqueNm);
} // namespace base
