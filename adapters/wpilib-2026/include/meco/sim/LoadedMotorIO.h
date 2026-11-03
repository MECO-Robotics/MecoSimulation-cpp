#pragma once

#include "base/Mechanism.h"

#include <functional>
#include <memory>

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
