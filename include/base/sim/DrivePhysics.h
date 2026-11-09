#pragma once
#include "base/sim/SimulationWorld.h"
#include "base/sim/RobotContactConfig.h"
#include <array>
#include <cstddef>

namespace base::sim
{
// Electrical and current-limit properties for one motor group.
struct MotorSimulationConfig
{
    double resistanceOhms{};
    double torqueConstantNmPerAmp{};
    double velocityConstantRadPerSecPerVolt{};
    double currentLimitAmps{};
    double frictionVoltage{};
};

// SI throughout, including motor/wheel angles in radians. Every physical
// property must be supplied by the caller; a value-initialized config is invalid.
struct DrivePhysicsConfig
{
    double mass{};
    Vec3 inertia{}, centerOfMass{};
    Vec3 bumperHalfExtents{};
    double bumperWallThickness{}, bumperCornerRadius{};
    double bumperFriction{};
    double rideHeight{}, wheelRadius{};
    double driveRatio{}, steerRatio{}, couplingRatio{};
    double wheelInertia{}, steerInertia{};
    double friction{}, longitudinalStiffness{}, lateralStiffness{};
    // Legacy ABI fields; rigid wheel contact does not use spring/travel parameters.
    double suspensionRestLength{}, suspensionTravel{};
    double suspensionStiffness{}, suspensionDamping{};
    MotorSimulationConfig driveMotor{}, steerMotor{};
    std::array<Vec3, 4> moduleLocations{};
    double wheelWidth{}, contactPatchLength{}, contactShearDeflection{};
};
struct ModulePhysicsState
{
    double wheelAngle{}, wheelVelocity{}, steerAngle{}, steerVelocity{};
    double driveRotorAngle{}, driveRotorVelocity{}, steerRotorAngle{}, steerRotorVelocity{};
    double longitudinalSlip{}, longitudinalForce{}, lateralForce{}, normalForce{};
    double driveCurrent{}, steerCurrent{}; // signed winding current, amperes
    double azimuthContactTwist{}, azimuthGroundTorque{}, steerMotorTorque{};
};
struct DriveTruth
{
    BodyState body;
    std::array<ModulePhysicsState, 4> modules{};
    double timestamp{};
    std::array<double, 8> foamCompression{}, foamNormalForce{};
    RigidContacts rigidContacts{};
};
// World must outlive the model. Stepping is explicit; querying never advances time.
class DrivePhysics
{
  public:
    DrivePhysics(SimulationWorld &, DrivePhysicsConfig);
    DrivePhysics(SimulationWorld &, DrivePhysicsConfig, const RobotContactConfig &);
    ~DrivePhysics();
    DrivePhysics(const DrivePhysics &) = delete;
    DrivePhysics &operator=(const DrivePhysics &) = delete;
    void SetModuleVoltages(std::size_t module, double drive, double steer);
    void BeforePhysics(double seconds);
    void AfterPhysics(double seconds);
    void Reset(Pose pose); // Clears wheel motion and body velocities.
    DriveTruth Truth() const;
    double SupplyCurrent(double busVoltage) const;
    BodyHandle Body() const
    {
        return body_;
    }
    const DrivePhysicsConfig &Config() const
    {
        return config_;
    }

  private:
    SimulationWorld &world_;
    DrivePhysicsConfig config_;
    BodyHandle body_;
    std::array<ModulePhysicsState, 4> modules_{};
    std::array<std::array<double, 2>, 4> voltages_{};
    double time_{};
    std::optional<RobotContactConfig> contactConfig_;
    std::array<double, 8> foamCompression_{}, foamNormalForce_{};
    struct FoamShearState
    {
        Vec3 point{}, normal{}, displacement{};
        double stiffness{};
        uint32_t surface{};
        size_t capsule{};
        bool active{};
    };
    std::array<FoamShearState, 32> foamShear_{};
    void CreateRobotBody();
};
} // namespace base::sim
