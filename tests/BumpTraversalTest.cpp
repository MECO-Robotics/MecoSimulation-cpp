#include "base/sim/DrivePhysics.h"
#include "base/sim/fields/Rebuilt2026.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace base::sim;
int main() {
    SimulationWorld world(Backend::Full3D);
    BodySpec floor;floor.halfExtents={20,20,.05};floor.pose.position.z=-.05;floor.friction=1;world.CreateBody(floor);
    for(const auto& collider:fields::rebuilt2026::FieldColliders())world.CreateBody(collider);
    DrivePhysicsConfig c;c.mass=67.6;c.friction=2.1;c.bumperHalfExtents.z=.06985;
    c.rideHeight=.0762;c.centerOfMass.z=.2238;c.driveRatio=5.8909090909090915;
    c.steerRatio=12.1;c.couplingRatio=4.909090909090909;c.wheelInertia=c.steerInertia=.01;
    c.motorResistance=12./483;c.motorKt=9.37/483;
    constexpr double pi=3.141592653589793;
    c.motorKv=(5800*2*pi/60)/(12-c.motorResistance*2);
    c.driveCurrentLimit=50;c.steerCurrentLimit=30;
    c.moduleLocations={{{.2794,.2794,0},{.2794,-.2794,0},{-.2794,.2794,0},{-.2794,-.2794,0}}};
    DrivePhysics drive(world,c);drive.Reset({{2.8,2.5106376,c.rideHeight},{}});
    // Running approach to a low-clearance skid: with every wheel lifted, there
    // is no magic propulsion. Momentum must carry the bumper over the crest.
    world.SetLinearVelocity(drive.Body(),{4,0,0});
    double maxHeight=0;int supportedWithWheelsLifted=0;bool rodeBump=false;
    for(int n=0;n<1000;++n) {
        const auto before=drive.Truth();
        for(size_t i=0;i<4;++i) {
            const auto& m=before.modules[i];
            const double steer=std::clamp(-100/(2*pi)*m.steerAngle-.5/(2*pi)*m.steerVelocity,-12.,12.);
            drive.SetModuleVoltages(i,12,steer);
        }
        drive.BeforePhysics(.004);world.Step(.004);drive.AfterPhysics(.004);
        const auto truth=drive.Truth();
        if(truth.body.pose.position.x>3.6 && truth.body.pose.position.x<5.7) {
            maxHeight=std::max(maxHeight,truth.body.pose.position.z);
            const bool unloaded=std::all_of(truth.modules.begin(),truth.modules.end(),
                                            [](auto& m){return m.normalForce<1;});
            supportedWithWheelsLifted+=unloaded && world.ContactManifoldCount()>0 && truth.body.pose.position.z>.10;
            if(unloaded)for(const auto& m:truth.modules)
                if(std::hypot(m.longitudinalForce,m.lateralForce)>3)throw std::runtime_error("Lifted tire still propels chassis");
        }
        if(truth.body.pose.position.x>4.8 && truth.body.pose.position.z>.10)rodeBump=true;
    }
    std::cout<<"final_x="<<drive.Truth().body.pose.position.x<<" rode_bump="<<rodeBump<<" max_body_height="<<maxHeight
             <<" bumper_supported_unloaded_steps="<<supportedWithWheelsLifted<<std::endl;
    if(!rodeBump || maxHeight<.18 || maxHeight>.35 || supportedWithWheelsLifted<5)
        throw std::runtime_error("Bumper must slide over the crest and support the lifted chassis without a leading-edge launch");
}
