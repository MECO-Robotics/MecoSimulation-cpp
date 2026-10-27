#include "base/sim/DrivePhysics.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string_view>
using namespace base::sim;
void Check(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}
DrivePhysicsConfig Config()
{
    DrivePhysicsConfig c;
    c.mass = 67.6;
    c.rideHeight = .0762;
    c.centerOfMass.z = .2238;
    c.bumperHalfExtents.z = .06985;
    c.friction = 2.1;
    c.moduleLocations = {{{.2794, .2794, 0}, {.2794, -.2794, 0}, {-.2794, .2794, 0}, {-.2794, -.2794, 0}}};
    return c;
}
void Floor(SimulationWorld &w)
{
    BodySpec b;
    b.halfExtents = {10, 10, .05};
    b.pose.position.z = -.05;
    w.CreateBody(b);
}
void Step(SimulationWorld &w, DrivePhysics &d, int count = 1)
{
    for (int i = 0; i < count; ++i)
    {
        d.BeforePhysics(.004);
        w.Step(.004);
        d.AfterPhysics(.004);
    }
}
void Clearance()
{
    for (double height : {.0254, .0762})
    {
        SimulationWorld w(Backend::Full3D);
        Floor(w);
        auto c = Config();
        RobotContactConfig contact;
        contact.chassisClearance = height;
        DrivePhysics d(w, c, contact);
        Step(w, d, 300);
        auto t = d.Truth();
        Check(std::abs(t.body.pose.position.z - c.rideHeight) < .001,
              "Clearance changed wheel support height");
        double load = 0;
        for (auto m : t.modules)
            load += m.normalForce;
        Check(std::abs(load - c.mass * 9.80665) < 3, "Wheel loads do not support weight");
        Check(w.BodyCount() == 2, "Robot contact created extra bodies");
        // Ray through the hollow bumper centre finds the actual underside plate.
        auto hit = w.Raycast({0, 0, .4}, {0, 0, -.5});
        Check(hit && std::abs(hit->position.z - (height + contact.panThickness)) < .001,
              "Pan did not follow clearance");
    }
    auto invalid = RobotContactConfig{};
    invalid.compressionStressPa[2] = 1;
    bool rejected = false;
    try
    {
        ValidateRobotContact(invalid, .43, .43);
    }
    catch (const std::invalid_argument &)
    {
        rejected = true;
    }
    Check(rejected, "Nonmonotonic foam curve accepted");
}
double Force(double depth, double vx)
{
    SimulationWorld w(Backend::Full3D);
    Floor(w);
    BodySpec wall;
    wall.halfExtents = {.1, 2, 1};
    wall.pose.position = {.53, 0, 1};
    w.CreateBody(wall);
    DrivePhysics d(w, Config(), RobotContactConfig{});
    auto state = w.ReadBody(d.Body());
    state.pose.position.x = depth;
    state.linearVelocity.x = vx;
    w.SetBodyState(d.Body(), state);
    d.BeforePhysics(.004);
    auto t = d.Truth();
    return std::accumulate(t.foamNormalForce.begin(), t.foamNormalForce.end(), 0.);
}
void Compression()
{
    const double shallow = Force(.005, 0), deep = Force(.015, 0), approaching = Force(.015, 1),
                 separating = Force(.015, -10);
    std::cout << "shallow=" << shallow << " deep=" << deep << " approaching=" << approaching
              << " separating=" << separating << '\n';
    Check(shallow > 0 && deep > shallow, "Foam does not resist progressive compression");
    Check(approaching > deep && separating >= 0 && separating < deep,
          "Foam damping does not oppose approach/release");
    Check(Force(-.02, 0) == 0, "Separated foam attracts robot");
    RobotContactConfig c;
    auto a = EvaluateFoam(c, .8), b = EvaluateFoam(c, .9);
    Check(b.pressure > a.pressure && std::abs(b.pressure - a.pressure - .1 * a.tangent) < 1e-7,
          "Final slope not continued");
    // An impact into a wall must dissipate energy rather than launch the body.
    SimulationWorld w(Backend::Full3D);
    Floor(w);
    BodySpec wall;
    wall.halfExtents = {.1, 2, 1};
    wall.pose.position = {.53, 0, 1};
    w.CreateBody(wall);
    auto config = Config();
    DrivePhysics d(w, config, c);
    w.SetLinearVelocity(d.Body(), {1, 0, 0});
    double maxEnergy = 0;
    for (int i = 0; i < 300; ++i)
    {
        Step(w, d);
        const auto s = d.Truth().body;
        auto v = s.linearVelocity;
        auto a = s.angularVelocity;
        double energy =
            .5 * config.mass * (v.x * v.x + v.y * v.y + v.z * v.z) +
            .5 * (config.inertia.x * a.x * a.x + config.inertia.y * a.y * a.y + config.inertia.z * a.z * a.z);
        maxEnergy = std::max(maxEnergy, energy);
    }
    Check(maxEnergy < .5 * config.mass * 1.05, "Damped impact generated excessive kinetic energy");
    d.Reset({{0, 0, config.rideHeight}, {}});
    auto reset = d.Truth();
    Check(std::accumulate(reset.foamNormalForce.begin(), reset.foamNormalForce.end(), 0.) == 0,
          "Reset retained foam force");
}
void Blocking()
{
    for (bool pan : {false, true})
    {
        SimulationWorld w(Backend::Full3D);
        Floor(w);
        BodySpec obstacle;
        obstacle.halfExtents = pan ? Vec3{.05, .05, .035} : Vec3{.05, 2, .04};
        obstacle.pose.position = {pan ? 0. : .4, 0, obstacle.halfExtents.z};
        w.CreateBody(obstacle);
        auto c = Config();
        RobotContactConfig contact;
        DrivePhysics d(w, c, contact);
        auto state = w.ReadBody(d.Body());
        state.pose.position.z = pan ? .20 : c.rideHeight;
        state.linearVelocity.x = pan ? 0 : 1;
        w.SetBodyState(d.Body(), state);
        unsigned hits = 0;
        for (int i = 0; i < 400; ++i)
        {
            Step(w, d);
            auto t = d.Truth();
            hits += pan ? t.rigidContacts.pan : t.rigidContacts.backing;
        }
        Check(hits > 0, pan ? "Solid pan failed to contact centre obstacle"
                            : "Flush plywood failed to contact obstacle");
        if (pan)
            Check(d.Truth().body.pose.position.z > c.rideHeight + .01, "Pan fell through obstacle");
    }
    // Static-only overlap ignores dynamic game pieces and merges identical seams.
    SimulationWorld w(Backend::Full3D);
    BodySpec b;
    b.halfExtents = {1, 1, .1};
    b.pose.position.z = -.1;
    w.CreateBody(b);
    w.CreateBody(b);
    auto hits = w.QueryFoamCapsule({{0, 0, .02}, {}}, .2, .03);
    Check(hits.count == 1, "Duplicate static seam multiplies foam contact");
    b.mass = 1;
    b.pose.position.z = .5;
    w.CreateBody(b);
    Check(w.QueryFoamCapsule({{0, 0, .5}, {}}, .2, .03).count == 0, "Foam query includes dynamic pieces");
}
void WheelLip()
{
    SimulationWorld w(Backend::Full3D);
    Floor(w);
    BodySpec lip;
    lip.halfExtents = {1, 2, .00715};
    lip.pose.position = {1, 0, .00715};
    w.CreateBody(lip);
    auto c = Config();
    c.rideHeight = .3;
    c.bumperHalfExtents.z = .05;
    c.centerOfMass.z = 0;
    DrivePhysics d(w, c);
    d.Reset({{-.2794 - .025, 0, c.rideHeight}, {}});
    Step(w, d, 12);
    const auto body = d.Truth().body;
    const auto q = body.pose.rotation;
    const double frontHeight = body.pose.position.z - 2 * q.w * q.y * .2794;
    std::cout << "front_height=" << frontHeight << " body_x=" << body.pose.position.x << '\n';
    Check(body.pose.position.x + .2794 < 0,
          "Axle already crossed the lip; regression does not test tire-leading contact");
    Check(frontHeight > c.rideHeight + .001, "Tire failed to support chassis before axle ray reached lip");
    auto touching = RobotContactConfig{};
    touching.backingFloorGap = .01905;
    ValidateRobotContact(touching, .43, .43); // Exact inch-derived top boundary, including roundoff.
}
void DriveReaction()
{
    SimulationWorld w(Backend::Full3D);
    auto c = Config();
    c.couplingRatio = 0;
    DrivePhysics d(w, c);
    d.Reset({{0, 0, 1}, {}});
    for (size_t i = 0; i < 4; ++i)
        d.SetModuleVoltages(i, 6, 0);
    Step(w, d);
    const auto t = d.Truth();
    double spin = 0;
    for (auto &m : t.modules)
    {
        spin += c.wheelInertia * m.wheelVelocity;
        Check(m.normalForce == 0, "Airborne tire has supporting load");
    }
    const double total = c.inertia.y * t.body.angularVelocity.y + spin;
    std::cout << "wheel_momentum=" << spin << " chassis_momentum=" << c.inertia.y * t.body.angularVelocity.y
              << " total=" << total << '\n';
    Check(spin > 0 && t.body.angularVelocity.y < 0, "Wheel acceleration lacks opposite chassis reaction");
    Check(std::abs(total) < 5e-6, "Airborne drive acceleration creates angular momentum");
}
void FoamShear()
{
    SimulationWorld w(Backend::Full3D);
    BodySpec wall;
    wall.halfExtents = {.1, 2, 1};
    wall.pose.position = {.53, 0, 1};
    w.CreateBody(wall);
    auto c = Config();
    RobotContactConfig material;
    DrivePhysics d(w, c, material);
    const Pose initial{{.015, 0, c.rideHeight}, {}};
    auto firstResponse = [&]()
    {
        d.Reset(initial);
        w.SetLinearVelocity(d.Body(), {0, 1, 0});
        Step(w, d);
        return d.Truth().body.linearVelocity.y;
    };
    const double first = firstResponse();
    // Two front noodles share the projected area. An initially unstrained
    // elastic shear layer cannot immediately impose a rigid no-slip condition.
    const double normalTangent = EvaluateFoam(material, .015 / material.noodleDiameter).tangent;
    const double totalShearStiffness = 2 * .86 * normalTangent / 2;
    const double maximumFirstLoss = totalShearStiffness * .004 * .004 / c.mass;
    Check(first < 1 && first >= 1 - maximumFirstLoss - 1e-5,
          "Foam instantaneously sticks instead of deforming in shear");
    Step(w, d, 12);
    Check(std::abs(firstResponse() - first) < 1e-7, "Reset retained foam shear history");
    d.Reset({{-.1, 0, c.rideHeight}, {}});
    w.SetLinearVelocity(d.Body(), {0, 1, 0});
    Step(w, d);
    Check(std::abs(d.Truth().body.linearVelocity.y - 1) < 1e-6, "Separated foam retains tangential force");
}
int main(int argc, char **argv)
{
    if (argc != 2)
        return 2;
    std::string_view test = argv[1];
    if (test == "clearance")
        Clearance();
    else if (test == "compression")
        Compression();
    else if (test == "shear")
        FoamShear();
    else if (test == "drive_reaction")
        DriveReaction();
    else if (test == "wheel_lip")
        WheelLip();
    else if (test == "blocking")
        Blocking();
    else
        return 2;
}
