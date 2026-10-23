#include "base/Drive.h"
#include "base/PortableSwerve.h"

#include <ctre/phoenix6/sim/ChassisReference.hpp>
#include <frc/RobotController.h>

#include <algorithm>

namespace p = ctre::phoenix6;

namespace base
{
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
    for (size_t i = 0; i < feedback.modules.size(); ++i)
    {
        auto &module = native_->GetModule(i);
        const auto &config = config_.modules[i];
        auto &drive = module.GetDriveMotor().GetSimState();
        auto &steer = module.GetSteerMotor().GetSimState();
        auto &encoder = module.GetEncoder().GetSimState();
        using Ref = p::sim::ChassisReference;
        drive.Orientation = config.driveInverted ? Ref::Clockwise_Positive : Ref::CounterClockwise_Positive;
        steer.Orientation = config.steerInverted ? Ref::Clockwise_Positive : Ref::CounterClockwise_Positive;
        encoder.Orientation = config.encoderInverted ? Ref::Clockwise_Positive : Ref::CounterClockwise_Positive;
        encoder.SensorOffset = units::turn_t{config.offsetTurns};
        const auto supply = frc::RobotController::GetBatteryVoltage();
        drive.SetSupplyVoltage(supply);
        steer.SetSupplyVoltage(supply);
        encoder.SetSupplyVoltage(supply);
        const auto &m = feedback.modules[i];
        drive.SetRawRotorPosition(units::radian_t{m.driveRotorPosition});
        drive.SetRotorVelocity(units::radians_per_second_t{m.driveRotorVelocity});
        steer.SetRawRotorPosition(units::radian_t{m.steerRotorPosition});
        steer.SetRotorVelocity(units::radians_per_second_t{m.steerRotorVelocity});
        encoder.SetRawPosition(units::radian_t{m.encoderPosition});
        encoder.SetVelocity(units::radians_per_second_t{m.encoderVelocity});
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
