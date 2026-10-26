#include "base/sim/DrivePhysics.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace base::sim
{
namespace
{
Vec3 Add(Vec3 a, Vec3 b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
Vec3 Scale(Vec3 a, double s)
{
    return {a.x * s, a.y * s, a.z * s};
}
double Dot(Vec3 a, Vec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
Vec3 Cross(Vec3 a, Vec3 b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
Vec3 Rotate(Quaternion q, Vec3 v)
{
    const Vec3 u{q.x, q.y, q.z};
    return Add(v, Add(Scale(Cross(u, v), 2 * q.w), Scale(Cross(u, Cross(u, v)), 2)));
}
Vec3 Unit(Vec3 a)
{
    double n = std::sqrt(Dot(a, a));
    return n > 1e-12 ? Scale(a, 1 / n) : Vec3{};
}
bool Finite(Vec3 v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
double FrictionVoltage(double voltage, double loss)
{
    return std::copysign(std::max(0.0, std::abs(voltage) - loss), voltage);
}
void Validate(const DrivePhysicsConfig &c)
{
    const double positive[]{c.mass,
                            c.inertia.x,
                            c.inertia.y,
                            c.inertia.z,
                            c.bumperHalfExtents.x,
                            c.bumperHalfExtents.y,
                            c.bumperHalfExtents.z,
                            c.rideHeight,
                            c.wheelRadius,
                            c.driveRatio,
                            c.steerRatio,
                            c.wheelInertia,
                            c.steerInertia,
                            c.longitudinalStiffness,
                            c.lateralStiffness,
                            c.motorResistance,
                            c.motorKt,
                            c.motorKv,
                            c.driveCurrentLimit,
                            c.steerCurrentLimit,
                            c.wheelWidth,
                            c.contactPatchLength,
                            c.contactShearDeflection};
    for (double x : positive)
        if (!std::isfinite(x) || x <= 0)
            throw std::invalid_argument("Drive physics requires positive finite physical parameters");
    if (!Finite(c.centerOfMass) || !std::isfinite(c.couplingRatio) || !std::isfinite(c.friction) || c.friction < 0 ||
        !std::isfinite(c.frictionVoltage) || c.frictionVoltage < 0)
        throw std::invalid_argument("Invalid drivetrain friction, coupling, or center of mass");
    for (auto p : c.moduleLocations)
        if (!Finite(p))
            throw std::invalid_argument("Invalid module location");
}
} // namespace
DrivePhysics::DrivePhysics(SimulationWorld &world, DrivePhysicsConfig config) : world_(world), config_(config)
{
    Validate(config_);
    BodySpec spec;
    spec.mass = config_.mass;
    spec.inertia = config_.inertia;
    spec.centerOfMass = config_.centerOfMass;
    spec.halfExtents = config_.bumperHalfExtents;
    spec.shape = BodySpec::Shape::BumperRing;
    // Nominal 3.5-inch bumper depth. Retain an opening for smaller test bodies.
    spec.radius = std::min(.0889, std::min(spec.halfExtents.x, spec.halfExtents.y)*.5);
    spec.friction = .65;
    spec.planar = world.Mode() == Backend::Planar;
    spec.pose.position.z = config_.rideHeight;
    body_ = world_.CreateBody(spec);
}
DrivePhysics::~DrivePhysics()
{
    world_.DestroyBody(body_);
}
void DrivePhysics::SetModuleVoltages(size_t i, double drive, double steer)
{
    if (i >= 4)
        throw std::out_of_range("Drive module");
    if (!std::isfinite(drive) || !std::isfinite(steer))
        throw std::invalid_argument("Nonfinite motor voltage");
    voltages_[i] = {std::clamp(drive, -16.0, 16.0), std::clamp(steer, -16.0, 16.0)};
}
void DrivePhysics::BeforePhysics(double dt)
{
    if (!std::isfinite(dt) || dt <= 0 || dt > .02)
        throw std::invalid_argument("Drive substep must be in (0,20ms]");
    auto body = world_.ReadBody(body_);
    auto q = body.pose.rotation;
    Vec3 down = Rotate(q, {0, 0, -1});
    struct Contact
    {
        Vec3 point{}, normal{0, 0, 1}, lever{};
        double gap{}, impulse{}, inverseMass{};
        bool active{};
    };
    std::array<Contact, 4> contacts{};
    const Quaternion inverseRotation{-q.x, -q.y, -q.z, q.w};
    const auto inverseInertia = [&](Vec3 torque) {
        const auto local = Rotate(inverseRotation, torque);
        return Rotate(q, {local.x / config_.inertia.x, local.y / config_.inertia.y,
                          local.z / config_.inertia.z});
    };
    if (world_.Mode() == Backend::Full3D)
    {
        for (size_t i = 0; i < contacts.size(); ++i)
        {
            auto &contact = contacts[i];
            const auto mount = Add(body.pose.position, Rotate(q, config_.moduleLocations[i]));
            // Fixed module-to-ground distance. The 1cm extension is speculative
            // contact detection, not suspension travel or an elastic element.
            const double rayLength = config_.rideHeight + .01;
            const auto hit = world_.Raycast(mount, Scale(down, rayLength), body_);
            if (!hit || Dot(hit->normal, Scale(down, -1)) <= .25)
                continue;
            contact.active = true;
            contact.point = hit->position;
            contact.normal = hit->normal;
            contact.lever = Add(Add(contact.point, Scale(body.pose.position, -1)),
                                Scale(Rotate(q, config_.centerOfMass), -1));
            const auto foot = Add(mount, Scale(down, config_.rideHeight));
            contact.gap = Dot(Add(foot, Scale(contact.point, -1)), contact.normal);
            const auto angularJacobian = Cross(contact.lever, contact.normal);
            contact.inverseMass = 1 / config_.mass +
                Dot(angularJacobian, inverseInertia(angularJacobian));
        }
        // Projected Gauss-Seidel solve of unilateral rigid normal constraints.
        // Predict gravity once, then share reaction impulses across all wheels.
        // There is no spring, damping coefficient, preload or stored elastic energy.
        auto linear = Add(body.linearVelocity, {0, 0, -9.80665 * dt});
        auto angular = body.angularVelocity;
        for (int iteration = 0; iteration < 24; ++iteration)
        {
            for (auto &contact : contacts)
            {
                if (!contact.active)
                    continue;
                const double normalVelocity = Dot(Add(linear, Cross(angular, contact.lever)), contact.normal);
                // Allow approach through a positive gap. Only penetration needs
                // numerical stabilization; the small slop is independent of load.
                const double target = contact.gap > 0 ? -contact.gap / dt :
                    .2 * std::max(0., -contact.gap - .0001) / dt;
                const double next = std::max(0., contact.impulse +
                    (target - normalVelocity) / contact.inverseMass);
                const double delta = next - contact.impulse;
                contact.impulse = next;
                linear = Add(linear, Scale(contact.normal, delta / config_.mass));
                angular = Add(angular, inverseInertia(Scale(Cross(contact.lever, contact.normal), delta)));
            }
        }
        for (const auto &contact : contacts)
            if (contact.active)
                world_.AddForce(body_, Scale(contact.normal, contact.impulse / dt), contact.point);
    }
    for (size_t i = 0; i < 4; ++i)
    {
        auto &m = modules_[i];
        const auto &c = config_;
        Vec3 lever = Rotate(q, c.moduleLocations[i]);
        Vec3 mount = Add(body.pose.position, lever);
        Vec3 velocity =
            Add(body.linearVelocity, Cross(body.angularVelocity, Add(lever, Scale(Rotate(q, c.centerOfMass), -1))));
        Vec3 contact = Add(mount, Scale(down, c.rideHeight));
        Vec3 normal{0, 0, 1};
        double normalForce = c.mass * 9.80665 / 4;
        if (world_.Mode() == Backend::Full3D)
        {
            const auto &support = contacts[i];
            normal = support.normal;
            contact = support.active ? support.point : contact;
            normalForce = support.impulse / dt;
        }
        const double driveV = FrictionVoltage(voltages_[i][0], c.frictionVoltage);
        const double steerV = FrictionVoltage(voltages_[i][1], c.frictionVoltage);
        // Freeze a midpoint azimuth basis for this implicit substep. Wheel spin
        // and azimuth speed are solved together; neither sensor is a setpoint.
        const double angle = m.steerAngle + .5 * m.steerVelocity * dt;
        Vec3 longitudinal = Rotate(q, {std::cos(angle), std::sin(angle), 0});
        longitudinal = Unit(Add(longitudinal, Scale(normal, -Dot(longitudinal, normal))));
        const Vec3 lateral = Unit(Cross(normal, longitudinal));
        const Vec3 contactVelocity = Add(body.linearVelocity,
            Cross(body.angularVelocity, Add(Add(contact, Scale(body.pose.position, -1)),
                                             Scale(Rotate(q, c.centerOfMass), -1))));
        const double forward = Dot(contactVelocity, longitudinal), side = Dot(contactVelocity, lateral);
        const Vec3 steerAxis=Scale(down,-1);
        const double axisProjection=Dot(steerAxis,normal);
        const double bodyYawRate = Dot(body.angularVelocity, normal);
        const double maximum = c.friction * normalForce;
        // RMS lever of a uniform rectangular patch (2x2 Gauss quadrature).
        const double patchRadius = std::hypot(c.wheelWidth, c.contactPatchLength) / std::sqrt(12.);
        const double torsionStiffness = maximum * patchRadius * patchRadius / c.contactShearDeflection;
        const double oldTwist = torsionStiffness > 0 ? m.azimuthContactTwist : 0.;
        auto driveCurrent = [&](double wheel, double steer) {
            return std::clamp((driveV-(c.driveRatio*wheel+c.couplingRatio*steer)/c.motorKv)/c.motorResistance,
                              -c.driveCurrentLimit,c.driveCurrentLimit);
        };
        auto steerCurrent = [&](double steer) {
            return std::clamp((steerV-c.steerRatio*steer/c.motorKv)/c.motorResistance,
                              -c.steerCurrentLimit,c.steerCurrentLimit);
        };
        auto motorTorque = [&](double wheel, double steer) {
            return c.steerRatio*c.motorKt*steerCurrent(steer) +
                   c.couplingRatio*c.motorKt*driveCurrent(wheel,steer);
        };
        auto wrench = [&](double wheel, double steer) {
            const double rolling = .5*(std::abs(wheel*c.wheelRadius)+std::abs(forward));
            // Elastic shear gives static holding torque. Rolling renews the
            // contact patch, relaxing that shear over one patch transit time.
            const double twist = (oldTwist+dt*(steer*axisProjection+bodyYawRate)) /
                                 (1+dt*rolling/c.contactPatchLength);
            double fx = c.longitudinalStiffness*(wheel*c.wheelRadius-forward);
            double fy = -c.lateralStiffness*side/(1+dt*c.lateralStiffness*4/c.mass);
            double torque = -torsionStiffness*twist;
            // One friction-wrench budget: steering cannot consume a second,
            // independent mu*N on top of saturated translational traction.
            const double magnitude = std::sqrt(fx*fx+fy*fy+torque*torque/(patchRadius*patchRadius));
            if(magnitude>maximum && magnitude>0) {
                const double scale=maximum/magnitude;
                fx*=scale; fy*=scale; torque*=scale;
            }
            return std::array<double,3>{fx,fy,torque};
        };
        const double previousWheel=m.wheelVelocity, previousSteer=m.steerVelocity;
        auto solveWheel = [&](double steer) {
            const double delta=dt*(c.driveRatio*c.motorKt*c.driveCurrentLimit+c.wheelRadius*maximum)/c.wheelInertia;
            double lo=previousWheel-delta, hi=previousWheel+delta;
            for(int iteration=0;iteration<36;++iteration) {
                const double wheel=(lo+hi)*.5;
                const double residual=c.wheelInertia*(wheel-previousWheel)/dt -
                    c.driveRatio*c.motorKt*driveCurrent(wheel,steer)+c.wheelRadius*wrench(wheel,steer)[0];
                if(residual>0) hi=wheel; else lo=wheel;
            }
            return (lo+hi)*.5;
        };
        const double steerDelta=dt*(c.steerRatio*c.motorKt*c.steerCurrentLimit+
            std::abs(c.couplingRatio)*c.motorKt*c.driveCurrentLimit+maximum*patchRadius)/c.steerInertia;
        double lo=previousSteer-steerDelta, hi=previousSteer+steerDelta;
        for(int iteration=0;iteration<36;++iteration) {
            const double steer=(lo+hi)*.5, wheel=solveWheel(steer);
            const double residual=c.steerInertia*(steer-previousSteer)/dt -
                                  motorTorque(wheel,steer)-axisProjection*wrench(wheel,steer)[2];
            if(residual>0) hi=steer; else lo=steer;
        }
        m.steerVelocity=(lo+hi)*.5;
        m.wheelVelocity=solveWheel(m.steerVelocity);
        const auto tire=wrench(m.wheelVelocity,m.steerVelocity);
        m.azimuthGroundTorque=tire[2];
        // Plastic return mapping dissipates excess shear at the joint friction
        // limit. Unloaded wheels carry no remembered carpet twist.
        m.azimuthContactTwist=torsionStiffness>0 ? -tire[2]/torsionStiffness : 0.;
        m.steerMotorTorque=motorTorque(m.wheelVelocity,m.steerVelocity);
        m.driveCurrent=driveCurrent(m.wheelVelocity,m.steerVelocity);
        m.steerCurrent=steerCurrent(m.steerVelocity);
        m.steerAngle+=m.steerVelocity*dt;
        m.wheelAngle+=m.wheelVelocity*dt;
        m.driveRotorAngle=c.driveRatio*m.wheelAngle+c.couplingRatio*m.steerAngle;
        m.driveRotorVelocity=c.driveRatio*m.wheelVelocity+c.couplingRatio*m.steerVelocity;
        m.steerRotorAngle=c.steerRatio*m.steerAngle;
        m.steerRotorVelocity=c.steerRatio*m.steerVelocity;
        m.longitudinalSlip=c.wheelRadius*m.wheelVelocity-forward;
        m.normalForce=normalForce; m.longitudinalForce=tire[0]; m.lateralForce=tire[1];
        world_.AddForce(body_,Add(Scale(longitudinal,tire[0]),Scale(lateral,tire[1])),contact);
        // Equal/opposite actuator/coupling reaction on the chassis. Ground
        // torsion loads the module; do not apply it a second time to the body.
        world_.AddTorque(body_,Add(Scale(normal,m.azimuthGroundTorque),
            Scale(steerAxis,-m.steerMotorTorque-axisProjection*m.azimuthGroundTorque)));
    }
}
void DrivePhysics::AfterPhysics(double dt)
{
    time_ += dt;
}
void DrivePhysics::Reset(Pose pose)
{
    BodyState state;
    state.pose = pose;
    world_.SetBodyState(body_, state);
    modules_ = {};
    voltages_ = {};
    time_ = 0;
}
double DrivePhysics::SupplyCurrent(double busVoltage) const
{
    if (!std::isfinite(busVoltage) || busVoltage < 0)
        throw std::invalid_argument("Invalid bus voltage");
    if (busVoltage == 0)
        return 0;
    double power = 0;
    for (size_t i = 0; i < modules_.size(); ++i)
    {
        const auto &m = modules_[i];
        // Restore the configured friction drop removed before the electrical solve.
        // Current limiting can still reduce terminal voltage below the request.
        const double driveV = m.driveCurrent * config_.motorResistance + m.driveRotorVelocity / config_.motorKv +
                              std::copysign(config_.frictionVoltage, voltages_[i][0]);
        const double steerV = m.steerCurrent * config_.motorResistance + m.steerRotorVelocity / config_.motorKv +
                              std::copysign(config_.frictionVoltage, voltages_[i][1]);
        power +=
            std::max(0., std::clamp(driveV, -std::abs(voltages_[i][0]), std::abs(voltages_[i][0])) * m.driveCurrent) +
            std::max(0., std::clamp(steerV, -std::abs(voltages_[i][1]), std::abs(voltages_[i][1])) * m.steerCurrent);
    }
    return power / busVoltage;
}
DriveTruth DrivePhysics::Truth() const
{
    return {world_.ReadBody(body_), modules_, time_};
}
} // namespace base::sim
