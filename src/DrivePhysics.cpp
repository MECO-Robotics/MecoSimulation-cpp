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
    CreateRobotBody();
}
DrivePhysics::DrivePhysics(SimulationWorld &world, DrivePhysicsConfig config,
                           const RobotContactConfig &contact)
    : world_(world), config_(config), contactConfig_(contact)
{
    Validate(config_);
    ValidateRobotContact(contact, config_.bumperHalfExtents.x, config_.bumperHalfExtents.y);
    if (world.Mode() != Backend::Full3D)
        throw std::invalid_argument("Compliant robot contact requires Full3D");
    CreateRobotBody();
}
void DrivePhysics::CreateRobotBody()
{
    BodySpec spec;
    spec.mass = config_.mass;
    spec.inertia = config_.inertia;
    spec.centerOfMass = config_.centerOfMass;
    spec.halfExtents = config_.bumperHalfExtents;
    spec.shape = BodySpec::Shape::BumperRing;
    // Nominal 3.5-inch bumper depth. Retain an opening for smaller test bodies.
    spec.radius = std::min(.0889, std::min(spec.halfExtents.x, spec.halfExtents.y)*.5);
    spec.friction = .65;
    spec.planar = world_.Mode() == Backend::Planar;
    spec.pose.position.z = config_.rideHeight;
    if (contactConfig_)
    {
        const auto &c = *contactConfig_;
        const double hx = config_.bumperHalfExtents.x, hy = config_.bumperHalfExtents.y;
        const double innerX = hx - c.noodleDiameter, innerY = hy - c.noodleDiameter, t = c.backingThickness;
        spec.shape = BodySpec::Shape::Compound;
        spec.friction = c.friction;
        spec.parts.push_back({{0, 0, c.chassisClearance + c.panThickness / 2 - config_.rideHeight},
                              {c.panHalfLength, c.panHalfWidth, c.panThickness / 2},
                              1});
        const double z = c.backingFloorGap + c.backingHeight / 2 - config_.rideHeight;
        for (double sign : {-1., 1.})
        {
            spec.parts.push_back({{sign * (innerX - t / 2), 0, z}, {t / 2, innerY, c.backingHeight / 2}, 2});
            spec.parts.push_back(
                {{0, sign * (innerY - t / 2), z}, {innerX - t, t / 2, c.backingHeight / 2}, 2});
        }
    }
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
    struct FoamContact : Contact
    {
        double target{}, softness{}, friction{}, shearStiffness{};
        Vec3 oldShear{};
        uint32_t surface{};
        Vec3 tangentImpulse{};
        size_t capsule{};
    };
    std::array<FoamContact, 32> foam{};
    size_t foamCount = 0;
    const auto previousShear = foamShear_;
    std::array<bool, 32> matchedShear{};
    foamShear_ = {};
    foamCompression_ = {};
    foamNormalForce_ = {};
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
            // Query the entire tire: the front of a 4-inch wheel contacts a lip
            // before a vertical ray through its axle reaches the obstacle.
            const double halfAngle = modules_[i].steerAngle / 2;
            const double sh = std::sin(halfAngle), ch = std::cos(halfAngle);
            const Quaternion orientation{q.x * ch + q.y * sh, q.y * ch - q.x * sh, q.z * ch + q.w * sh,
                                         q.w * ch - q.z * sh};
            const auto center = Add(mount, Rotate(q, {0, 0, config_.wheelRadius - config_.rideHeight}));
            const auto hit =
                world_.QueryWheelContact({center, orientation}, config_.wheelRadius, config_.wheelWidth);
            if (!hit)
                continue;
            contact.active = true;
            contact.point = hit->point;
            contact.normal = hit->normal;
            contact.lever = Add(Add(contact.point, Scale(body.pose.position, -1)),
                                Scale(Rotate(q, config_.centerOfMass), -1));
            contact.gap = hit->gap;
            const auto angularJacobian = Cross(contact.lever, contact.normal);
            contact.inverseMass = 1 / config_.mass +
                Dot(angularJacobian, inverseInertia(angularJacobian));
        }
        if (contactConfig_)
        {
            const auto &c = *contactConfig_;
            const double radius = c.noodleDiameter / 2;
            size_t capsule = 0;
            for (int side = 0; side < 4; ++side)
                for (int level = 0; level < 2; ++level, ++capsule)
                {
                    const bool alongY = side < 2;
                    const double sign = side % 2 ? 1. : -1.;
                    const double hx = config_.bumperHalfExtents.x, hy = config_.bumperHalfExtents.y;
                    const double halfLength = alongY ? hy - radius : hx - 3 * radius;
                    Vec3 center{alongY ? sign * (hx - radius) : 0, alongY ? 0 : sign * (hy - radius),
                                c.bumperFloorGap + radius + level * c.noodleDiameter - config_.rideHeight};
                    // Rotate local Y onto X for the side noodles.
                    Quaternion orientation = q;
                    if (!alongY)
                    {
                        constexpr double h = .7071067811865475244;
                        // q * rotationZ(pi/2)
                        orientation = {h * (q.x + q.y), h * (q.y - q.x), h * (q.z + q.w), h * (q.w - q.z)};
                    }
                    auto patches = world_.QueryFoamCapsule(
                        {Add(body.pose.position, Rotate(q, center)), orientation}, halfLength, radius);
                    for (size_t j = 0; j < patches.count; ++j)
                    {
                        const auto &p = patches.patches[j];
                        auto &f = foam[foamCount++];
                        f.point = p.point;
                        f.normal = p.normal;
                        f.capsule = capsule;
                        f.active = true;
                        f.friction = std::sqrt(c.friction * p.friction);
                        f.lever = Add(Add(p.point, Scale(body.pose.position, -1)),
                                      Scale(Rotate(q, config_.centerOfMass), -1));
                        const auto jacobian = Cross(f.lever, f.normal);
                        f.inverseMass = 1 / config_.mass + Dot(jacobian, inverseInertia(jacobian));
                        const auto response = EvaluateFoam(c, p.depth / c.noodleDiameter);
                        // Fixed projected area per noodle, shared by its current patches.
                        const double area = 2 * (halfLength + radius) * c.noodleDiameter / patches.count;
                        const double stiffness = area * response.tangent / c.noodleDiameter;
                        // Estimated low-Poisson foam approximation: G=E/2.
                        // Normal compression is damped; tangential shear is
                        // elastic with Coulomb plastic return at slip.
                        f.shearStiffness = stiffness / 2;
                        f.surface = p.surface;
                        size_t best = previousShear.size();
                        double bestDistance = .05 * .05;
                        for (size_t k = 0; k < previousShear.size(); ++k)
                        {
                            const auto &old = previousShear[k];
                            if (matchedShear[k] || !old.active || old.capsule != capsule ||
                                old.surface != p.surface || Dot(old.normal, p.normal) < .95)
                                continue;
                            const auto delta = Add(old.point, Scale(p.point, -1));
                            const double distance = Dot(delta, delta);
                            if (distance < bestDistance)
                            {
                                best = k;
                                bestDistance = distance;
                            }
                        }
                        if (best < previousShear.size())
                        {
                            matchedShear[best] = true;
                            auto stored = previousShear[best].displacement;
                            stored = Add(stored, Scale(p.normal, -Dot(stored, p.normal)));
                            // Changing contact area/stiffness must not manufacture
                            // shear energy or a larger remembered force.
                            f.oldShear =
                                Scale(stored, std::min(1., previousShear[best].stiffness / f.shearStiffness));
                        }
                        const double damping = 2 * c.dampingRatio * std::sqrt(stiffness / f.inverseMass);
                        const double denominator = damping + dt * stiffness;
                        f.softness = 1 / (dt * denominator);
                        f.target = area * response.pressure / denominator;
                        foamCompression_[capsule] = std::max(foamCompression_[capsule], p.depth);
                    }
                }
        }
        // Projected Gauss-Seidel solve of unilateral rigid normal constraints.
        // Predict gravity once, then share reaction impulses across all wheels.
        // Wheel support has no suspension or elastic energy; only foam is compliant.
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
            for (size_t i = 0; i < foamCount; ++i)
            {
                auto &f = foam[i];
                const double velocity = Dot(Add(linear, Cross(angular, f.lever)), f.normal);
                const double next = std::max(0., f.impulse + (f.target - velocity - f.softness * f.impulse) /
                                                                 (f.inverseMass + f.softness));
                const double delta = next - f.impulse;
                f.impulse = next;
                linear = Add(linear, Scale(f.normal, delta / config_.mass));
                angular = Add(angular, inverseInertia(Scale(Cross(f.lever, f.normal), delta)));
                const auto v = Add(linear, Cross(angular, f.lever));
                const auto seed = std::abs(f.normal.z) < .9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
                const auto t1 = Unit(Cross(f.normal, seed)), t2 = Cross(f.normal, t1);
                const auto j1 = Cross(f.lever, t1), j2 = Cross(f.lever, t2);
                const double shearSoftness = 1 / (dt * dt * f.shearStiffness);
                const double a = 1 / config_.mass + Dot(j1, inverseInertia(j1)) + shearSoftness;
                const double b = Dot(j1, inverseInertia(j2));
                const double d = 1 / config_.mass + Dot(j2, inverseInertia(j2)) + shearSoftness;
                const auto residual =
                    Add(Add(v, Scale(f.oldShear, 1 / dt)), Scale(f.tangentImpulse, shearSoftness));
                const double r1 = -Dot(residual, t1), r2 = -Dot(residual, t2), det = a * d - b * b;
                auto candidate = Add(f.tangentImpulse, Add(Scale(t1, (d * r1 - b * r2) / det),
                                                           Scale(t2, (a * r2 - b * r1) / det)));
                const double length = std::sqrt(Dot(candidate, candidate));
                const double limit = f.friction * f.impulse;
                if (length > limit)
                    candidate = Scale(candidate, limit / length);
                const auto change = Add(candidate, Scale(f.tangentImpulse, -1));
                f.tangentImpulse = candidate;
                linear = Add(linear, Scale(change, 1 / config_.mass));
                angular = Add(angular, inverseInertia(Cross(f.lever, change)));
            }
        }
        for (size_t i = 0; i < foamCount; ++i)
        {
            const auto &f = foam[i];
            world_.AddForce(body_, Scale(Add(Scale(f.normal, f.impulse), f.tangentImpulse), 1 / dt), f.point);
            foamNormalForce_[f.capsule] += f.impulse / dt;
            // Plastic return maps displacement to the force that was actually
            // admitted by the friction cone. A separated contact is forgotten.
            foamShear_[i] = {f.point,
                             f.normal,
                             Scale(f.tangentImpulse, -1 / (dt * f.shearStiffness)),
                             f.shearStiffness,
                             f.surface,
                             f.capsule,
                             true};
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
        // Normal-load changes are not an unmodeled energy source. Preserve at
        // most the prior shear moment when stiffness rises; unloading relaxes
        // stiffness without increasing deflection. A lifted patch forgets shear.
        const double oldTwist = torsionStiffness > 0 ? m.azimuthContactTwist *
            std::min(1., m.normalForce / normalForce) : 0.;
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
        if (world_.Mode() == Backend::Full3D)
        {
            // Applying tire force at the ground contact already includes the
            // steady rolling drive reaction. The missing term is the change
            // in wheel spin momentum, including airborne motor acceleration.
            const auto axle = Rotate(q, {-std::sin(angle), std::cos(angle), 0});
            world_.AddTorque(body_, Scale(axle, -c.wheelInertia * (m.wheelVelocity - previousWheel) / dt));
        }
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
    world_.ClearForces(body_);
    foamShear_ = {};
    foamCompression_ = {};
    foamNormalForce_ = {};
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
    return {world_.ReadBody(body_), modules_,         time_,
            foamCompression_,       foamNormalForce_, world_.ContactCounts(body_)};
}
} // namespace base::sim
