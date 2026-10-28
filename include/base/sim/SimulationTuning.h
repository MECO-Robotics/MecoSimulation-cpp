#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace base::sim
{
    struct ModuleLocation
    {
        double xMeters;
        double yMeters;
    };

    struct TuningInputs
    {
        double massKg;
        double yawMoiKgMetersSquared;
        double frictionCoefficient;
        double wheelRadiusMeters;
        double driveRatio;
        double driveCurrentLimitAmps;
        double motorTorqueConstantNmPerAmp;
        std::array<ModuleLocation, 4> modules;
        double translationErrorScaleMeters;
        double headingErrorScaleRadians;
    };

    struct TuningResult
    {
        double autoTranslationKP;
        double autoRotationKP;
        double maxLinearAcceleration;
        double maxAngularAcceleration;
    };

    // Initial simulation-only PathPlanner gains from a motor/traction stopping
    // envelope. This is an acceleration upper bound, not a steering-load model.
    // Steering uses the robot's configured gains with the contact-loaded plant.
    inline TuningResult DeriveSimulationTuning(const TuningInputs &inputs);
} // namespace base::sim

namespace base::sim
{
namespace tuning_detail
{
inline constexpr double kGravityMetersPerSecondSquared = 9.80665;

inline void RequirePositive(double value, const char* name)
{
    if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument(std::string{"Simulation tuning requires positive finite "} + name);
}
} // namespace

inline TuningResult DeriveSimulationTuning(const TuningInputs& in)
{
    for (const auto& [value, name] : std::array<std::pair<double, const char*>, 9>{
             {{in.massKg, "mass"}, {in.yawMoiKgMetersSquared, "yaw MOI"},
              {in.frictionCoefficient, "friction coefficient"}, {in.wheelRadiusMeters, "wheel radius"},
              {in.driveRatio, "drive ratio"}, {in.driveCurrentLimitAmps, "drive current limit"},
              {in.motorTorqueConstantNmPerAmp, "motor torque constant"},
              {in.translationErrorScaleMeters, "translation error scale"},
              {in.headingErrorScaleRadians, "heading error scale"}}})
        tuning_detail::RequirePositive(value, name);
    for (const auto& module : in.modules)
        if (!std::isfinite(module.xMeters) || !std::isfinite(module.yMeters))
            throw std::invalid_argument("Simulation tuning requires finite module locations");

    // Current-limited motor force is further capped by per-wheel traction.
    const double driveTorque = in.driveRatio * in.motorTorqueConstantNmPerAmp * in.driveCurrentLimitAmps;
    const double motorForce = driveTorque / in.wheelRadiusMeters;
    const double tractionForce = in.frictionCoefficient * in.massKg * tuning_detail::kGravityMetersPerSecondSquared / 4.0;
    const double moduleForce = std::min(motorForce, tractionForce);
    const double maxLinearAcceleration = 4.0 * moduleForce / in.massKg;
    double maxYawTorque = 0;
    for (const auto& module : in.modules)
        maxYawTorque += std::hypot(module.xMeters, module.yMeters) * moduleForce;
    const double maxAngularAcceleration = maxYawTorque / in.yawMoiKgMetersSquared;
    if (!std::isfinite(maxLinearAcceleration) || maxLinearAcceleration <= 0 ||
        !std::isfinite(maxAngularAcceleration) || maxAngularAcceleration <= 0)
        throw std::invalid_argument("Simulation tuning requires nonzero drive and yaw authority");

    // For proportional pose feedback, v=Kp*error. At the chosen error scale,
    // v^2/(2*a) equals that error, so the command remains inside the available
    // acceleration stopping envelope.
    return {std::sqrt(2.0 * maxLinearAcceleration / in.translationErrorScaleMeters),
            std::sqrt(2.0 * maxAngularAcceleration / in.headingErrorScaleRadians),
            maxLinearAcceleration,
            maxAngularAcceleration};
}
} // namespace base::sim
