#include "base/sim/DrivePhysics.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>
using namespace base::sim;
namespace {
void Check(bool ok,std::source_location at=std::source_location::current()) {
    if(!ok) throw std::runtime_error("Azimuth load contract at line "+std::to_string(at.line()));
}
DrivePhysicsConfig Config() {
    DrivePhysicsConfig c;
    c.mass=67.6; c.friction=2.1; c.driveRatio=5.8909090909090915; c.steerRatio=12.1;
    c.wheelInertia=.01; c.steerInertia=.01; c.driveCurrentLimit=50; c.steerCurrentLimit=30;
    c.motorResistance=12./483; c.motorKt=9.37/483;
    c.motorKv=(5800.*2*std::acos(-1.)/60)/(12-2*c.motorResistance);
    return c;
}
void Step(SimulationWorld& w,DrivePhysics& d,double dt=.001) { d.BeforePhysics(dt);w.Step(dt);d.AfterPhysics(dt); }
// A restrained chassis is a steering dynamometer. Moving the belt underneath
// it imposes rolling without bypassing the motor, tire or current-limit equations.
ModulePhysicsState Dynamometer(double mass,double rolling) {
    SimulationWorld w(Backend::Planar);auto c=Config();c.mass=mass;
    DrivePhysics d(w,c);
    const auto initial=w.ReadBody(d.Body());
    for(int i=0;i<1300;++i) {
        auto held=initial;held.linearVelocity.x=rolling;w.SetBodyState(d.Body(),held);
        d.SetModuleVoltages(0,rolling==0?0:c.driveRatio*rolling/c.wheelRadius/c.motorKv+c.frictionVoltage,
                            i<300?0:.3);
        Step(w,d);
    }
    return d.Truth().modules[0];
}
double Energy(const DriveTruth&t,const DrivePhysicsConfig&c) {
    double e=.5*c.mass*(t.body.linearVelocity.x*t.body.linearVelocity.x+t.body.linearVelocity.y*t.body.linearVelocity.y);
    e+=.5*c.inertia.z*t.body.angularVelocity.z*t.body.angularVelocity.z;
    const double r=std::hypot(c.wheelWidth,c.contactPatchLength)/std::sqrt(12.);
    for(const auto&m:t.modules) {
        const double k=c.friction*m.normalForce*r*r/c.contactShearDeflection;
        e+=.5*c.wheelInertia*m.wheelVelocity*m.wheelVelocity+
           .5*c.steerInertia*m.steerVelocity*m.steerVelocity+
           c.steerInertia*t.body.angularVelocity.z*m.steerVelocity+
           .5*k*m.azimuthContactTwist*m.azimuthContactTwist;
    }
    return e;
}
}
int main() {
    const auto stationary=Dynamometer(67.6,0), heavy=Dynamometer(135.2,0), rolling=Dynamometer(67.6,2);
    // Static tire shear holds at finite deflection; more load increases stiffness.
    Check(std::abs(stationary.steerVelocity)<.01 && stationary.steerAngle>.005 && stationary.steerAngle<.1);
    Check(heavy.steerAngle<stationary.steerAngle*.7);
    Check(std::abs(stationary.azimuthGroundTorque)>.5);
    // Rolling renews the patch and relieves twisting resistance.
    Check(rolling.steerAngle>stationary.steerAngle*3 && rolling.steerVelocity>.1);
    {
        SimulationWorld w(Backend::Full3D);BodySpec floor;floor.halfExtents={5,5,.05};floor.pose.position.z=-.05;w.CreateBody(floor);
        auto c=Config();DrivePhysics d(w,c);d.SetModuleVoltages(0,0,.3);
        for(int i=0;i<300;++i) Step(w,d);
        Check(std::abs(d.Truth().modules[0].azimuthContactTwist)>.001);
        auto lifted=w.ReadBody(d.Body());lifted.pose.position.z+=1;w.SetBodyState(d.Body(),lifted);
        Step(w,d);const auto m=d.Truth().modules[0];
        Check(m.normalForce==0 && m.azimuthGroundTorque==0 && m.azimuthContactTwist==0);
        Check(m.longitudinalForce==0 && m.lateralForce==0);
    }
    {
        SimulationWorld w(Backend::Planar);auto c=Config();DrivePhysics d(w,c);
        const auto initial=w.ReadBody(d.Body());
        for(int i=0;i<500;++i) {
            auto held=initial;held.linearVelocity={2,1,0};w.SetBodyState(d.Body(),held);
            for(size_t j=0;j<4;++j) d.SetModuleVoltages(j,12,6);
            const auto before=d.Truth();Step(w,d);const auto after=d.Truth();
            const double r=std::hypot(c.wheelWidth,c.contactPatchLength)/std::sqrt(12.);
            for(size_t j=0;j<4;++j) {
                const auto&m=after.modules[j];
                Check(std::sqrt(m.longitudinalForce*m.longitudinalForce+m.lateralForce*m.lateralForce+
                      m.azimuthGroundTorque*m.azimuthGroundTorque/(r*r))<=c.friction*m.normalForce+1e-7);
                Check(std::abs(m.steerCurrent)<=30+1e-8 && std::abs(m.driveCurrent)<=50+1e-8);
                Check(std::abs(c.steerInertia*(m.steerVelocity-before.modules[j].steerVelocity)/.001-
                               m.steerMotorTorque-m.azimuthGroundTorque)<1e-6);
            }
        }
    }
    for(double roll : {0., .7}) {
        // Airborne actuator reaction follows the chassis steering axis and
        // conserves angular momentum, including when the chassis is tilted.
        SimulationWorld w(Backend::Full3D);auto c=Config();DrivePhysics d(w,c);
        d.Reset({{0,0,1},{std::sin(roll/2),0,0,std::cos(roll/2)}});
        for(size_t j=0;j<4;++j)d.SetModuleVoltages(j,0,1);
        for(int i=0;i<200;++i)Step(w,d);
        auto t=d.Truth();
        const double axial=-std::sin(roll)*t.body.angularVelocity.y+std::cos(roll)*t.body.angularVelocity.z;
        double h=c.inertia.z*axial;
        for(auto m:t.modules) { h+=c.steerInertia*m.steerVelocity;Check(m.azimuthGroundTorque==0); }
        Check(std::abs(h)<1e-5 && axial<-.001);
    }
    {
        // A larger support impulse must not manufacture stored tire energy by
        // multiplying an already strained patch's stiffness. Vertical impact
        // energy is not an unmodeled source of azimuth spring energy.
        SimulationWorld w(Backend::Full3D);BodySpec floor;floor.halfExtents={5,5,.05};floor.pose.position.z=-.05;w.CreateBody(floor);
        auto c=Config();DrivePhysics d(w,c);const auto held=w.ReadBody(d.Body());
        for(int i=0;i<500;++i) {
            w.SetBodyState(d.Body(),held);d.SetModuleVoltages(0,0,.3);Step(w,d);
        }
        const auto before=d.Truth().modules[0];
        const double r=std::hypot(c.wheelWidth,c.contactPatchLength)/std::sqrt(12.);
        auto energy=[&](const ModulePhysicsState&m) {
            return .5*c.steerInertia*m.steerVelocity*m.steerVelocity+
                .5*c.friction*m.normalForce*r*r/c.contactShearDeflection*m.azimuthContactTwist*m.azimuthContactTwist;
        };
        auto impact=held;impact.linearVelocity.z=-.5;w.SetBodyState(d.Body(),impact);
        d.SetModuleVoltages(0,0,0);Step(w,d);const auto after=d.Truth().modules[0];
        Check(after.normalForce>before.normalForce*5);
        Check(energy(after)<=energy(before)*1.03);
    }
    for(double dt : {.001,.004}) {
        // Refined step and the actual desktop robot-period substep.
        SimulationWorld w(Backend::Planar);auto c=Config();DrivePhysics d(w,c);
        for(size_t j=0;j<4;++j)d.SetModuleVoltages(j,0,2);
        for(int i=0;i<int(.3/dt);++i)Step(w,d,dt);
        for(size_t j=0;j<4;++j)d.SetModuleVoltages(j,0,0);
        const double initial=Energy(d.Truth(),c);double maximum=initial;
        for(int i=0;i<int(2./dt);++i) { Step(w,d,dt);maximum=std::max(maximum,Energy(d.Truth(),c)); }
        Check(initial>0 && maximum<initial*1.03 && Energy(d.Truth(),c)<initial*.05);
    }
    std::cout<<"Azimuth static/load, rolling, unloading, shared friction, reaction and passivity passed\n";
    std::cout<<"stationary angle="<<stationary.steerAngle<<" heavy="<<heavy.steerAngle<<" rolling="<<rolling.steerAngle
             <<" stationary ground torque="<<stationary.azimuthGroundTorque
             <<" stationary steer current="<<stationary.steerCurrent<<"\n";
}
