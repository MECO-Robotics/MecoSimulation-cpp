#include "meco/sim/DriveSimulationAdapter.h"

#include "base/SimulationRuntime.h"
#include <base/Telemetry.h>
#include <base/TelemetryPublishers.h>
#include "meco/sim/SimulationDiagnostics.h"

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
    out.bumperWallThickness = s.bumperWallThickness;
    out.bumperCornerRadius = s.bumperCornerRadius;
    out.bumperFriction = s.bumperFriction;
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
    out.wheelWidth = s.wheelWidth;
    out.contactPatchLength = s.contactPatchLength;
    out.contactShearDeflection = s.contactShearDeflection;
    out.driveMotor = {s.driveMotor.resistanceOhms, s.driveMotor.torqueConstantNmPerAmp,
                      s.driveMotor.velocityConstantRadPerSecPerVolt, s.driveMotor.currentLimitAmps,
                      s.driveMotor.frictionVoltage};
    out.steerMotor = {s.steerMotor.resistanceOhms, s.steerMotor.torqueConstantNmPerAmp,
                      s.steerMotor.velocityConstantRadPerSecPerVolt, s.steerMotor.currentLimitAmps,
                      s.steerMotor.frictionVoltage};
    for (std::size_t i = 0; i < out.moduleLocations.size(); ++i)
        out.moduleLocations[i] = {config.modules[i].xMeters, config.modules[i].yMeters, 0};
    return out;
}

} // namespace

struct DriveSimulationSession::Impl
{
    Impl(const base::DriveConfig &config, base::sim::Backend backend,
         const RobotContactConfig *contact = nullptr)
        : world_(backend),
          physics_(
              contact ? std::make_unique<base::sim::DrivePhysics>(world_, MakePhysicsConfig(config), *contact)
                      : std::make_unique<base::sim::DrivePhysics>(world_, MakePhysicsConfig(config))),
          config_(config)
    {
        if (backend == base::sim::Backend::Phoenix)
            throw std::invalid_argument("Jolt drive simulation requires jolt-planar or jolt-3d backend");
        if (contact)
        {
            const auto &c = *contact;
            base::telemetry::PostDashboardNumber("FieldSimulation/ChassisClearance", c.chassisClearance);
            base::telemetry::PostDashboardNumber("FieldSimulation/PanThickness", c.panThickness);
            base::telemetry::PostDashboardNumber("FieldSimulation/PanHalfLength", c.panHalfLength);
            base::telemetry::PostDashboardNumber("FieldSimulation/PanHalfWidth", c.panHalfWidth);
            base::telemetry::PostDashboardNumber("FieldSimulation/BackingFloorGap", c.backingFloorGap);
            base::telemetry::PostDashboardNumber("FieldSimulation/BackingThickness", c.backingThickness);
            base::telemetry::PostDashboardNumber("FieldSimulation/BackingHeight", c.backingHeight);
            base::telemetry::PostDashboardNumber("FieldSimulation/BumperFloorGap", c.bumperFloorGap);
            base::telemetry::PostDashboardNumber("FieldSimulation/NoodleDiameter", c.noodleDiameter);
            base::telemetry::PostDashboardNumber("FieldSimulation/FoamDampingRatio", c.dampingRatio);
            base::telemetry::PostDashboardBoolean("FieldSimulation/CompliantBumpers", true);
            base::telemetry::PostDashboardNumberArray("FieldSimulation/FoamCompressionStrain",
                {c.compressionStrain.begin(), c.compressionStrain.end()});
            base::telemetry::PostDashboardNumberArray("FieldSimulation/FoamCompressionStressPa",
                {c.compressionStressPa.begin(), c.compressionStressPa.end()});
            base::telemetry::PostDashboardNumber("FieldSimulation/BumperMaterialFriction", c.friction);
            base::telemetry::PostDashboardNumber("FieldSimulation/WheelRadius", config.wheelRadius);
            base::telemetry::PostDashboardNumber("FieldSimulation/WheelWidth", physics_->Config().wheelWidth);
            base::telemetry::PostDashboardNumber("FieldSimulation/TireFrictionEstimate", config.cof);
        }
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
            {[this](double dt)
             {
                 ReadVoltages(dt);
                 physics_->BeforePhysics(dt);
                 if (arena_)
                     arena_->BeforePhysics(dt);
             },
             [this](double dt) { world_.Step(dt); },
             [this](double dt)
             {
                 physics_->AfterPhysics(dt);
                 if (arena_)
                     arena_->AfterPhysics(dt);
                 WriteSensors();
             },
             [this]
             {
                 Reset(frc::Pose2d{});
                 if (drive_)
                     drive_->ResetPose(frc::Pose2d{});
             },
             [this]
             { return physics_->SupplyCurrent(base::simulation::GetSimulationRuntime().BusVoltage()); }});
    }

    void Reset(const frc::Pose2d &pose)
    {
        const double angle = pose.Rotation().Radians().value();
        lastYaw_ = unwrappedYaw_ = angle;
        physics_->Reset({{pose.X().value(), pose.Y().value(), config_.simulation.rideHeight},
                         {0, 0, std::sin(angle / 2), std::cos(angle / 2)}});
        if (arena_)
            arena_->Reset();
        if (drive_)
            WriteSensors();
    }

    base::sim::DriveTruth PhysicsTruth() const { return physics_->Truth(); }

    base::DriveSimulationTruth Truth() const
    {
        const auto truth = physics_->Truth();
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
        arena_ = std::make_unique<base::sim::Arena>(
            world_, [this] { return physics_->Truth().body; }, scene, seed, robotCenterOfMass);
        return *arena_;
    }

  private:
    void ReadVoltages(double dt)
    {
        const auto volts = drive_->SimulationVoltages(dt, base::simulation::GetSimulationRuntime().BusVoltage());
        for (std::size_t i = 0; i < volts.size(); ++i)
            physics_->SetModuleVoltages(i, volts[i][0], volts[i][1]);
    }

    void WriteSensors()
    {
        const auto truth = physics_->Truth();
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
        {
            const auto now=nt::Now();
            if (now - lastContactPublish_ >= 20000)
            {
                lastContactPublish_ = now;
                std::array<double,25> values{};
                values[0]=now;
                for(size_t i=0;i<truth.modules.size();++i) {
                    const auto&m=truth.modules[i];
                    const std::array<double,6> module{m.normalForce,m.azimuthGroundTorque,m.azimuthContactTwist,
                                                    m.steerMotorTorque,m.driveCurrent,m.steerCurrent};
                    std::copy(module.begin(),module.end(),values.begin()+1+6*i);
                }
                base::telemetry::Post("Diagnostics/Drive/AzimuthLoad",
                    [publisher = azimuthLoad_, values] { publisher->Set(values); });
                std::array<double, 4> loads{};
                for (size_t i = 0; i < 4; ++i)
                    loads[i] = truth.modules[i].normalForce;
                base::telemetry::Post("Simulation/Contact/WheelNormalLoadsN",
                    [publisher = wheelLoads_, loads] { publisher->Set(loads); });
                const auto compression = truth.foamCompression;
                base::telemetry::Post("Simulation/Contact/FoamCompressionM",
                    [publisher = foamCompression_, compression] { publisher->Set(compression); });
                const auto foamForces = truth.foamNormalForce;
                base::telemetry::Post("Simulation/Contact/FoamNormalForcesN",
                    [publisher = foamForces_, foamForces] { publisher->Set(foamForces); });
                const std::array<double, 2> rigid{double(truth.rigidContacts.pan),
                                                  double(truth.rigidContacts.backing)};
                base::telemetry::Post("Simulation/Contact/RigidContactsPanBacking",
                    [publisher = rigidContacts_, rigid] { publisher->Set(rigid); });
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
    std::unique_ptr<base::sim::DrivePhysics> physics_;
    base::DriveConfig config_;
    base::sim::BodyHandle floor_{};
    base::Drive *drive_{};
    std::shared_ptr<nt::DoubleArrayPublisher> azimuthLoad_ = std::make_shared<nt::DoubleArrayPublisher>(
        nt::NetworkTableInstance::GetDefault().GetDoubleArrayTopic("/Diagnostics/Drive/AzimuthLoad").Publish());
    std::shared_ptr<nt::DoubleArrayPublisher> wheelLoads_ = std::make_shared<nt::DoubleArrayPublisher>(
        nt::NetworkTableInstance::GetDefault().GetDoubleArrayTopic("/Simulation/Contact/WheelNormalLoadsN").Publish());
    std::shared_ptr<nt::DoubleArrayPublisher> foamCompression_ = std::make_shared<nt::DoubleArrayPublisher>(
        nt::NetworkTableInstance::GetDefault().GetDoubleArrayTopic("/Simulation/Contact/FoamCompressionM").Publish());
    std::shared_ptr<nt::DoubleArrayPublisher> foamForces_ = std::make_shared<nt::DoubleArrayPublisher>(
        nt::NetworkTableInstance::GetDefault().GetDoubleArrayTopic("/Simulation/Contact/FoamNormalForcesN").Publish());
    std::shared_ptr<nt::DoubleArrayPublisher> rigidContacts_ = std::make_shared<nt::DoubleArrayPublisher>(
        nt::NetworkTableInstance::GetDefault().GetDoubleArrayTopic("/Simulation/Contact/RigidContactsPanBacking").Publish());
    int64_t lastContactPublish_{};
    double lastYaw_{}, unwrappedYaw_{};
    std::unique_ptr<base::sim::Arena> arena_;
    base::simulation::Registration registration_;
};

DriveSimulationSession::DriveSimulationSession(const base::DriveConfig &config, base::sim::Backend backend)
    : impl_(std::make_unique<Impl>(config, backend))
{
}
DriveSimulationSession::DriveSimulationSession(const base::DriveConfig &config, base::sim::Backend backend,
                                               const RobotContactConfig &contact)
    : impl_(std::make_unique<Impl>(config, backend, &contact))
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
std::unique_ptr<DriveSimulationSession> CreateDriveSimulationAdapter(const base::DriveConfig &config,
                                                                     base::sim::Backend backend,
                                                                     const RobotContactConfig &contact)
{
    return std::make_unique<DriveSimulationSession>(config, backend, contact);
}
} // namespace meco::sim
