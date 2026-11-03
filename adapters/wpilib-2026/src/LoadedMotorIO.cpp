#include "meco/sim/LoadedMotorIO.h"

#include "base/SimulationRuntime.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace meco::sim
{
namespace
{
class LoadedMotor final : public base::MotorIO
{
  public:
    LoadedMotor(base::MotorConfig motor, LoadedMotorModel model, std::function<double()> load)
        : motor_(std::move(motor)), model_(model), load_(std::move(load))
    {
        motor_.Validate();
        if (!load_ || !std::isfinite(model_.resistanceOhms) || model_.resistanceOhms <= 0 ||
            !std::isfinite(model_.torqueConstantNmPerAmp) || model_.torqueConstantNmPerAmp <= 0 ||
            !std::isfinite(model_.velocityConstantRadPerSecondPerVolt) ||
            model_.velocityConstantRadPerSecondPerVolt <= 0 ||
            !std::isfinite(model_.outputInertiaKgM2) || model_.outputInertiaKgM2 <= 0 ||
            !std::isfinite(model_.viscousDragNmPerRadPerSecond) || model_.viscousDragNmPerRadPerSecond < 0 ||
            !std::isfinite(model_.maximumPhysicsStepSeconds) || model_.maximumPhysicsStepSeconds <= 0 ||
            !std::isfinite(model_.maximumUpdateSeconds) || model_.maximumUpdateSeconds <= 0 ||
            !std::isfinite(model_.loadDirectionSpeedThresholdRadPerSecond) ||
            model_.loadDirectionSpeedThresholdRadPerSecond < 0)
            throw std::invalid_argument("Invalid loaded motor simulation configuration");
        registration_ = base::simulation::GetSimulationRuntime().Register({
            [this](double dt) {
                auto &runtime = base::simulation::GetSimulationRuntime();
                bus_ = runtime.BusVoltage();
                Integrate(dt);
            }, {}, {},
            [this] {
                position_ = velocity_ = voltage_ = request_ = current_ = 0;
                bus_ = 12;
            },
            [this] { return SupplyCurrent(); }});
    }

    base::MotorInputs Read() override
    {
        constexpr double tau = 2 * std::numbers::pi;
        return {position_ / tau, velocity_ / tau, voltage_, bus_, SupplyCurrent(), true};
    }
    void SetVoltage(double volts) override
    {
        if (!std::isfinite(volts))
            throw std::invalid_argument("Invalid motor voltage");
        request_ = volts;
    }
    void SetBrake(bool brake) override { brake_ = brake; }
    void StepSimulation(units::second_t duration) override { Integrate(duration.value()); }

  private:
    double SupplyCurrent() const
    {
        return bus_ > 0 ? std::max(0.0, voltage_ * current_) * motor_.ids.size() / bus_ : 0;
    }
    void Integrate(double dt)
    {
        if (!std::isfinite(dt) || dt < 0 || dt > model_.maximumUpdateSeconds)
            throw std::invalid_argument("Invalid loaded motor simulation step");
        double load = load_();
        if (!std::isfinite(load))
            throw std::invalid_argument("Loaded motor torque must be finite");
        load = std::max(0.0, load);
        const int steps = std::max(1, static_cast<int>(std::ceil(dt / model_.maximumPhysicsStepSeconds)));
        const double h = dt / steps;
        const double requested = std::clamp(request_, -bus_, bus_);
        const double torquePerAmp = model_.torqueConstantNmPerAmp * motor_.ids.size() * motor_.gearRatio;
        for (int i = 0; i < steps; ++i)
        {
            voltage_ = requested;
            current_ = (!brake_ && std::abs(voltage_) < 1e-9)
                           ? 0
                           : std::clamp((voltage_ - motor_.gearRatio * velocity_ /
                                         model_.velocityConstantRadPerSecondPerVolt) /
                                            model_.resistanceOhms,
                                        -static_cast<double>(motor_.currentLimit),
                                        static_cast<double>(motor_.currentLimit));
            const double opposing = std::abs(velocity_) > model_.loadDirectionSpeedThresholdRadPerSecond
                                        ? std::copysign(load, velocity_)
                                        : std::clamp(torquePerAmp * current_, -load, load);
            velocity_ += (torquePerAmp * current_ - opposing -
                          model_.viscousDragNmPerRadPerSecond * velocity_) /
                         model_.outputInertiaKgM2 * h;
            position_ += velocity_ * h;
            voltage_ = std::clamp(model_.resistanceOhms * current_ +
                                      motor_.gearRatio * velocity_ /
                                          model_.velocityConstantRadPerSecondPerVolt,
                                  -std::abs(voltage_), std::abs(voltage_));
        }
    }

    base::MotorConfig motor_;
    LoadedMotorModel model_;
    std::function<double()> load_;
    double position_{}, velocity_{}, voltage_{}, request_{}, current_{}, bus_{12};
    bool brake_{true};
    base::simulation::Registration registration_;
};
} // namespace

std::unique_ptr<base::MotorIO> MakeMotorIO(const base::MotorConfig &motor,
                                           LoadedMotorModel model,
                                           std::function<double()> loadTorqueNm)
{
    return std::make_unique<LoadedMotor>(motor, model, std::move(loadTorqueNm));
}
} // namespace meco::sim
