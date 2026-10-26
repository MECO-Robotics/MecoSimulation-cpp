#include "meco/sim/DriveSimulationAdapter.h"

#include "base/SimulationRuntime.h"
#include "meco/sim/SimulationDiagnostics.h"

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
    // Phoenix's driveCurrent is a battery-side supply limit. Wheel torque is
    // limited by SlipCurrent (stator current), not by that supply setting.
    out.driveCurrentLimit = config.backend == base::DriveBackend::SparkMaxNavX
                                ? config.driveCurrent : config.slipCurrent;
    out.steerCurrentLimit = config.steerCurrent;
    // Native Phoenix swerve module requests enable FOC. Match that plant;
    // SparkMax uses the NEO model and its motor-side SmartCurrentLimit.
    const auto motor = config.backend == base::DriveBackend::SparkMaxNavX
                           ? frc::DCMotor::NEO(1) : frc::DCMotor::KrakenX60FOC(1);
    out.motorResistance = motor.R.value();
    out.motorKt = motor.Kt.value();
    out.motorKv = motor.Kv.value();
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
        // The standalone drive world still needs a ground surface. Without it,
        // Full3D gravity pulls the chassis through empty space and the wheel
        // suspension rays never produce useful normal force.
        base::sim::BodySpec floor;
        floor.halfExtents = {100.0, 100.0, 0.05};
        floor.mass = 0;
        floor.friction = 1.0;
        floor.pose.position.z = -0.05;
        floor_ = world_.CreateBody(floor);
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

    base::sim::DriveTruth PhysicsTruth() const { return physics_.Truth(); }

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
        if (meco::sim::LatencyProbeEnabled())
        {
            static int64_t last=0;
            const auto now=nt::Now();
            if(now-last>=20000) {
                last=now;
                std::array<double,25> values{};
                values[0]=now;
                for(size_t i=0;i<truth.modules.size();++i) {
                    const auto&m=truth.modules[i];
                    const std::array<double,6> module{m.normalForce,m.azimuthGroundTorque,m.azimuthContactTwist,
                                                    m.steerMotorTorque,m.driveCurrent,m.steerCurrent};
                    std::copy(module.begin(),module.end(),values.begin()+1+6*i);
                }
                static auto publisher=nt::NetworkTableInstance::GetDefault()
                    .GetDoubleArrayTopic("/Diagnostics/Drive/AzimuthLoad").Publish();
                publisher.Set(values);
            }
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
    base::sim::BodyHandle floor_{};
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
base::sim::DriveTruth DriveSimulationSession::PhysicsTruth() const { return impl_->PhysicsTruth(); }

std::unique_ptr<DriveSimulationSession> CreateDriveSimulationAdapter(const base::DriveConfig &config,
                                                                     base::sim::Backend backend)
{
    return std::make_unique<DriveSimulationSession>(config, backend);
}
} // namespace meco::sim
