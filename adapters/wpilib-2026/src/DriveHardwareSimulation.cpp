#include "base/Drive.h"
#include "base/PortableSwerve.h"

#include <ctre/phoenix6/sim/ChassisReference.hpp>
#include <frc/RobotController.h>

#include <algorithm>
#include <stdexcept>

namespace p = ctre::phoenix6;

namespace base
{
namespace
{
constexpr double kRadiansPerTurn = 6.2831853071795864769;
}
std::array<std::array<double, 2>, 4> Drive::SimulationVoltages(double dt, double busVoltage)
{
    std::array<std::array<double, 2>, 4> result{};
    if (portable_)
    {
        for (size_t i = 0; i < result.size(); ++i)
            result[i] = portable_->SimVoltages(i, dt, busVoltage);
        return result;
    }
    if (native_)
    {
        native_->UpdateSimState(units::second_t{dt}, units::volt_t{busVoltage});
        for (size_t i = 0; i < result.size(); ++i)
        {
            auto &module = native_->GetModule(i);
            // Phoenix SimState returns the applied motor voltage after device
            // configuration; electrical inversion is already reflected here.
            result[i] = {
                std::clamp(module.GetDriveMotor().GetSimState().GetMotorVoltage().value(), -busVoltage, busVoltage),
                std::clamp(module.GetSteerMotor().GetSimState().GetMotorVoltage().value(), -busVoltage, busVoltage)};
        }
    }
    return result;
}

void Drive::ApplySimulationFeedback(const DriveSimulationFeedback &feedback)
{
    if (portable_)
    {
        for (size_t i = 0; i < feedback.modules.size(); ++i)
        {
            const auto &m = feedback.modules[i];
            portable_->SetSimInputs(i, {m.wheelAngle * config_.wheelRadius, m.wheelVelocity * config_.wheelRadius,
                                        m.steerAngle, m.steerVelocity, 0, true});
        }
        portable_->SetSimYaw(feedback.yaw);
        return;
    }
    if (!native_)
        return;
    using Ref = p::sim::ChassisReference;
    const auto toPhoenixOrientation = [](SimulationDeviceOrientation orientation) {
        switch (orientation)
        {
        case SimulationDeviceOrientation::CounterClockwisePositive:
            return Ref::CounterClockwise_Positive;
        case SimulationDeviceOrientation::ClockwisePositive:
            return Ref::Clockwise_Positive;
        }
        throw std::invalid_argument("Invalid simulated device orientation");
    };
    if (!simulationOrientationsInitialized_)
    {
        for (size_t i = 0; i < feedback.modules.size(); ++i)
        {
            auto &module = native_->GetModule(i);
            auto &drive = module.GetDriveMotor().GetSimState();
            auto &steer = module.GetSteerMotor().GetSimState();
            auto &encoder = module.GetEncoder().GetSimState();
            const auto &deviceConfig = config_.simulation.modules[i];
            drive.Orientation = toPhoenixOrientation(deviceConfig.driveOrientation);
            steer.Orientation = toPhoenixOrientation(deviceConfig.steerOrientation);
            encoder.Orientation = toPhoenixOrientation(deviceConfig.encoderOrientation);
            encoder.SensorOffset = units::turn_t{config_.modules[i].offsetTurns};
        }
        simulationOrientationsInitialized_ = true;
    }
    for (size_t i = 0; i < feedback.modules.size(); ++i)
    {
        auto &module = native_->GetModule(i);
        auto &drive = module.GetDriveMotor().GetSimState();
        auto &steer = module.GetSteerMotor().GetSimState();
        auto &encoder = module.GetEncoder().GetSimState();
        const auto supply = frc::RobotController::GetBatteryVoltage();
        drive.SetSupplyVoltage(supply);
        steer.SetSupplyVoltage(supply);
        encoder.SetSupplyVoltage(supply);
        const auto &m = feedback.modules[i];
        drive.SetRawRotorPosition(units::radian_t{m.driveRotorPosition});
        drive.SetRotorVelocity(units::radians_per_second_t{m.driveRotorVelocity});
        steer.SetRawRotorPosition(units::radian_t{m.steerRotorPosition});
        steer.SetRotorVelocity(units::radians_per_second_t{m.steerRotorVelocity});
        // Supply physical chassis coordinates; Phoenix applies device orientation,
        // sensor direction, and the configured magnet offset.
        encoder.SetRawPosition(units::turn_t{m.encoderPosition / kRadiansPerTurn});
        encoder.SetVelocity(units::turns_per_second_t{m.encoderVelocity / kRadiansPerTurn});
    }
    auto &pigeon = native_->GetPigeon2().GetSimState();
    pigeon.SetRawYaw(units::radian_t{feedback.yaw});
    pigeon.SetPitch(units::radian_t{feedback.pitch});
    pigeon.SetRoll(units::radian_t{feedback.roll});
    pigeon.SetAngularVelocityX(units::radians_per_second_t{feedback.angularVelocityX});
    pigeon.SetAngularVelocityY(units::radians_per_second_t{feedback.angularVelocityY});
    pigeon.SetAngularVelocityZ(units::radians_per_second_t{feedback.angularVelocityZ});
}
} // namespace base
