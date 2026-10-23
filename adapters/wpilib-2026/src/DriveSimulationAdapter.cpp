#include "meco/sim/DriveSimulationAdapter.h"

#include "base/SimulationRuntime.h"

#include <frc/system/plant/DCMotor.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

namespace meco::sim
{
namespace
{
base::sim::DrivePhysicsConfig MakePhysicsConfig(const base::DriveConfig &config)
{
    base::sim::DrivePhysicsConfig out;
    const auto &s = config.simulation;
    out.mass = config.massKg;
    out.inertia = {s.rollInertia, s.pitchInertia, config.moi};
    out.centerOfMass = {s.centerOfMassX, s.centerOfMassY, s.centerOfMassZ};
    out.bumperHalfExtents = {s.bumperHalfLength, s.bumperHalfWidth, s.bumperHalfHeight};
    out.rideHeight = s.rideHeight;
    out.wheelRadius = config.wheelRadius;
    out.driveRatio = config.driveRatio;
    out.steerRatio = config.steerRatio;
    out.couplingRatio = config.couplingRatio;
    out.wheelInertia = s.wheelInertia;
    out.steerInertia = s.steerInertia;
    out.friction = config.cof;
    out.longitudinalStiffness = s.longitudinalStiffness;
    out.lateralStiffness = s.lateralStiffness;
    out.suspensionRestLength = s.suspensionRestLength;
    out.suspensionTravel = s.suspensionTravel;
    out.suspensionStiffness = s.suspensionStiffness;
    out.suspensionDamping = s.suspensionDamping;
    out.driveCurrentLimit = config.driveCurrent;
    out.steerCurrentLimit = config.steerCurrent;
    if (config.backend == base::DriveBackend::SparkMaxNavX)
    {
        const auto motor = frc::DCMotor::NEO(1);
        out.motorResistance = motor.R.value();
        out.motorKt = motor.Kt.value();
        out.motorKv = motor.Kv.value();
    }
    for (std::size_t i = 0; i < out.moduleLocations.size(); ++i)
        out.moduleLocations[i] = {config.modules[i].xMeters, config.modules[i].yMeters, 0};
    return out;
}

} // namespace

struct DriveSimulationSession::Impl
{
    Impl(const base::DriveConfig &config, base::sim::Backend backend)
        : world_(backend), physics_(world_, MakePhysicsConfig(config)), config_(config)
    {
        if (backend == base::sim::Backend::Phoenix)
            throw std::invalid_argument("Jolt drive simulation requires jolt-planar or jolt-3d backend");
    }

    void Attach(base::Drive &drive)
    {
        if (drive_)
            throw std::logic_error("Drive simulation adapter is already attached");
        drive_ = &drive;
        WriteSensors();
        registration_ = base::simulation::GetSimulationRuntime().Register(
            {[this](double dt) {
                 ReadVoltages(dt);
                 physics_.BeforePhysics(dt);
                 if (arena_)
                     arena_->BeforePhysics(dt);
             },
             [this](double dt) { world_.Step(dt); },
             [this](double dt) {
                 physics_.AfterPhysics(dt);
                 if (arena_)
                     arena_->AfterPhysics(dt);
                 WriteSensors();
             },
             [this] {
                 Reset(frc::Pose2d{});
                 if (drive_)
                     drive_->ResetPose(frc::Pose2d{});
             },
             [this] { return physics_.SupplyCurrent(base::simulation::GetSimulationRuntime().BusVoltage()); }});
    }

    void Reset(const frc::Pose2d &pose)
    {
        const double angle = pose.Rotation().Radians().value();
        lastYaw_ = unwrappedYaw_ = angle;
        physics_.Reset({{pose.X().value(), pose.Y().value(), config_.simulation.rideHeight},
                        {0, 0, std::sin(angle / 2), std::cos(angle / 2)}});
        if (arena_)
            arena_->Reset();
        if (drive_)
            WriteSensors();
    }

    base::DriveSimulationTruth Truth() const
    {
        const auto truth = physics_.Truth();
        const auto &body = truth.body;
        const auto &q = body.pose.rotation;
        const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        return {body.pose.position.x, body.pose.position.y, body.pose.position.z, yaw,
                body.linearVelocity.x, body.linearVelocity.y, body.linearVelocity.z, body.angularVelocity.z};
    }

    base::sim::Arena &CreateArena(base::sim::ArenaScene scene, std::uint32_t seed,
                                  base::sim::Vec3 robotCenterOfMass)
    {
        if (arena_)
            throw std::logic_error("Simulation session already owns an arena");
        arena_ = std::make_unique<base::sim::Arena>(world_, [this] { return physics_.Truth().body; }, scene, seed,
                                                    robotCenterOfMass);
        return *arena_;
    }

  private:
    void ReadVoltages(double dt)
    {
        const auto volts = drive_->SimulationVoltages(dt, base::simulation::GetSimulationRuntime().BusVoltage());
        for (std::size_t i = 0; i < volts.size(); ++i)
            physics_.SetModuleVoltages(i, volts[i][0], volts[i][1]);
    }

    void WriteSensors()
    {
        const auto truth = physics_.Truth();
        base::DriveSimulationFeedback feedback;
        for (std::size_t i = 0; i < truth.modules.size(); ++i)
        {
            const auto &module = truth.modules[i];
            auto &out = feedback.modules[i];
            out.driveRotorPosition = module.driveRotorAngle;
            out.driveRotorVelocity = module.driveRotorVelocity;
            out.steerRotorPosition = module.steerRotorAngle;
            out.steerRotorVelocity = module.steerRotorVelocity;
            out.encoderPosition = module.steerAngle;
            out.encoderVelocity = module.steerVelocity;
            out.wheelAngle = module.wheelAngle;
            out.wheelVelocity = module.wheelVelocity;
            out.steerAngle = module.steerAngle;
            out.steerVelocity = module.steerVelocity;
        }
        const auto &q = truth.body.pose.rotation;
        const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        const double pitch = std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
        const double roll = std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
        unwrappedYaw_ += std::remainder(yaw - lastYaw_, 2 * std::acos(-1.0));
        lastYaw_ = yaw;
        feedback.yaw = unwrappedYaw_;
        feedback.pitch = pitch;
        feedback.roll = roll;
        feedback.angularVelocityX = truth.body.angularVelocity.x;
        feedback.angularVelocityY = truth.body.angularVelocity.y;
        feedback.angularVelocityZ = truth.body.angularVelocity.z;
        drive_->ApplySimulationFeedback(feedback);
    }

    base::sim::SimulationWorld world_;
    base::sim::DrivePhysics physics_;
    base::DriveConfig config_;
    base::Drive *drive_{};
    double lastYaw_{}, unwrappedYaw_{};
    std::unique_ptr<base::sim::Arena> arena_;
    base::simulation::Registration registration_;
};

DriveSimulationSession::DriveSimulationSession(const base::DriveConfig &config, base::sim::Backend backend)
    : impl_(std::make_unique<Impl>(config, backend))
{
}
DriveSimulationSession::~DriveSimulationSession() = default;
base::sim::Arena &DriveSimulationSession::CreateArena(base::sim::ArenaScene scene, std::uint32_t seed,
                                                      base::sim::Vec3 robotCenterOfMass)
{
    return impl_->CreateArena(scene, seed, robotCenterOfMass);
}
void DriveSimulationSession::Attach(base::Drive &drive) { impl_->Attach(drive); }
void DriveSimulationSession::Reset(const frc::Pose2d &pose) { impl_->Reset(pose); }
base::DriveSimulationTruth DriveSimulationSession::Truth() const { return impl_->Truth(); }

std::unique_ptr<DriveSimulationSession> CreateDriveSimulationAdapter(const base::DriveConfig &config,
                                                                     base::sim::Backend backend)
{
    return std::make_unique<DriveSimulationSession>(config, backend);
}
} // namespace meco::sim
