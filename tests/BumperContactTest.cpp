#include "base/sim/SimulationWorld.h"
#include <cmath>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>
using namespace base::sim;
namespace {
void Check(bool ok, std::source_location at=std::source_location::current()) {
    if(!ok) throw std::runtime_error("Bumper contact contract at line "+std::to_string(at.line()));
}
BodySpec Bumper() {
    BodySpec b;
    b.shape=BodySpec::Shape::BumperRing;
    b.halfExtents={.43,.43,.06985};
    b.radius=.0889;
    b.friction=.65;
    b.mass=67.6;
    b.inertia={4,4,6.883};
    b.centerOfMass.z=.2238;
    b.pose.position.z=.0762;
    return b;
}
}
int main() {
    // A perimeter shell contacts at its edges, but does not invent an underbody.
    {
        SimulationWorld w(Backend::Full3D);
        auto b=Bumper(); b.mass=0; b.pose.position.z=.5;
        const auto ring=w.CreateBody(b);
        Check(!w.Raycast({0,0,1},{0,0,-1}));
        const auto edge=w.Raycast({.4,0,1},{0,0,-1});
        Check(edge && edge->body==ring && std::abs(edge->position.z-.56985)<.001);
        b.radius=.43;
        bool rejected=false;
        try { w.CreateBody(b); } catch(const std::invalid_argument&) { rejected=true; }
        Check(rejected);
    }
    // Ground collision supports a tilted bumper after a drop, independently of wheel rays.
    {
        SimulationWorld w(Backend::Full3D);
        BodySpec floor; floor.halfExtents={3,3,.05}; floor.pose.position.z=-.05; w.CreateBody(floor);
        auto b=Bumper(); b.pose.position.z=.4;
        b.pose.rotation={0,std::sin(.15),0,std::cos(.15)};
        const auto body=w.CreateBody(b);
        bool touched=false;
        for(int i=0;i<3000;++i) {
            w.Step(.001); touched=touched || w.ContactManifoldCount()>0;
            auto s=w.ReadBody(body); const auto q=s.pose.rotation;
            const double lowest=s.pose.position.z-.43*std::abs(2*(q.x*q.z-q.w*q.y))
                -.43*std::abs(2*(q.y*q.z+q.w*q.x))-.06985*std::abs(1-2*(q.x*q.x+q.y*q.y));
            Check(lowest>-.005);
        }
        Check(touched && w.ReadBody(body).pose.position.z>.06);
    }
    // Low obstacles and ramps hit bumper material; sub-clearance objects pass below.
    for(double height : {.003,.10}) {
        SimulationWorld w(Backend::Planar);
        BodySpec obstacle; obstacle.halfExtents={.04,1,height/2}; obstacle.pose.position={1,0,height/2};
        w.CreateBody(obstacle);
        auto b=Bumper(); b.planar=true; const auto body=w.CreateBody(b);
        w.SetLinearVelocity(body,{1,0,0});
        bool touched=false;
        for(int i=0;i<1500;++i) { w.Step(.001); touched=touched || w.ContactManifoldCount()>0; }
        const auto s=w.ReadBody(body);
        if(height<.00635) Check(!touched && s.pose.position.x>1.4);
        else Check(touched && s.pose.position.x<.55 && std::abs(s.linearVelocity.x)<.01);
    }
    // A sloped plate also makes contact before the bumper can pass through it.
    {
        SimulationWorld w(Backend::Full3D);
        BodySpec ramp; ramp.halfExtents={1,1,.025}; ramp.pose.position={.8,0,.1};
        ramp.pose.rotation={0,std::sin(-.13),0,std::cos(.13)}; w.CreateBody(ramp);
        auto b=Bumper(); b.pose.position={.2,0,.5}; const auto body=w.CreateBody(b);
        w.SetLinearVelocity(body,{1,0,0});
        bool touched=false;
        for(int i=0;i<1000 && !touched;++i) { w.Step(.001); touched=w.ContactManifoldCount()>0; }
        Check(touched && w.ReadBody(body).pose.position.z>.05);
    }
    std::cout<<"Bumper shell, floor, clearance, obstacle and ramp contacts passed\n";
}
