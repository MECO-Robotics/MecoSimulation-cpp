#include "base/sim/fields/Rebuilt2026.h"
#include <cmath>
#include <numbers>
namespace base::sim::fields::rebuilt2026 {
namespace {
constexpr double In(double value) { return value * .0254; }
base::sim::BodySpec Box(double x, double y, double z, double sx, double sy, double sz,
                        double pitch = 0) {
    base::sim::BodySpec b;
    b.mass = 0;
    b.friction = 1;
    b.halfExtents = {sx/2, sy/2, sz/2};
    b.pose = {{x,y,z}, {0,std::sin(pitch/2),0,std::cos(pitch/2)}};
    return b;
}
}
std::vector<base::sim::BodySpec> FieldColliders() {
    std::vector<base::sim::BodySpec> bodies;
    // Inner wall faces bound the carpet. Thickness lies outside the field.
    bodies.push_back(Box(-.05,kFieldWidth/2,.6,.1,kFieldWidth+.2,1.2));
    bodies.push_back(Box(kFieldLength+.05,kFieldWidth/2,.6,.1,kFieldWidth+.2,1.2));
    bodies.push_back(Box(kFieldLength/2,-.05,.6,kFieldLength,.1,1.2));
    bodies.push_back(Box(kFieldLength/2,kFieldWidth+.05,.6,kFieldLength,.1,1.2));
    for (double x : {kBlueHubX,kRedHubX}) {
        // Solid chassis-contact envelope; the goal opening/fuel exits are not modeled.
        bodies.push_back(Box(x,kHubY,In(72)/2,In(47),In(47),In(72)));
        // Four traversable bumps, each represented by two 15-degree HDPE plates.
        constexpr double angle = 15*std::numbers::pi/180;
        const double run = In(44.4)/2;
        const double height = In(6.513);
        const double thickness = In(.5);
        for (double y : {kHubY-In(47+73)/2,kHubY+In(47+73)/2}) {
            for (double side : {-1.,1.}) {
                auto ramp=Box(x+side*run/2,y,
                    height-run*std::tan(angle)/2-thickness*std::cos(angle)/2,
                    run/std::cos(angle),In(73),thickness,side*angle);
                ramp.friction=kBumpContactFriction;
                bodies.push_back(ramp);
            }
        }
        // Trench inner supports and overhead arms. Unlike a 2D blocking strip,
        // the .565m opening stays clear for the current low chassis collider.
        const double opening = In(50.34);
        for (bool upper : {false,true}) {
            const double y = upper ? kFieldWidth-opening-In(12)/2 : opening+In(12)/2;
            bodies.push_back(Box(x,y,In(40.25)/2,In(53),In(12),In(40.25)));
            const double beamY = upper ? kFieldWidth-opening/2 : opening/2;
            bodies.push_back(Box(x,beamY,(In(22.25)+In(40.25))/2,
                                 In(47),opening,In(40.25-22.25)));
        }
    }
    // Simplified tower base/uprights/rungs, placed from the MapleSim 2026 map.
    // Exact decorative frames, netting and moving trench arms are omitted.
    for (bool red : {false,true}) {
        const double y = red ? In(170) : In(159);
        const double x = red ? kFieldLength-In(42) : In(42);
        const double baseX = red ? kFieldLength-In(45.18)/2 : In(45.18)/2;
        bodies.push_back(Box(baseX,y,In(.25)/2,In(45.18),In(39),In(.25)));
        for (double side : {-1.,1.})
            bodies.push_back(Box(x,y+side*In(32.25+1.5)/2,In(72.1)/2,
                                 In(3.5),In(1.5),In(72.1)));
        for (double z : {In(27),In(45),In(63)})
            bodies.push_back(Box(x,y,z,In(1.66),In(47),In(1.66)));
    }
    // Depot's three low steel barriers (open toward the field).
    for (bool red : {false,true}) {
        const double y = red ? kFieldWidth-6.066 : 6.066;
        const double sign = red ? -1 : 1;
        const double wallX = red ? kFieldLength : 0;
        for (double side : {-1.,1.})
            bodies.push_back(Box(wallX+sign*In(27)/2,y+side*In(42-3)/2,
                                 In(1.125)/2,In(27),In(3),In(1.125)));
        bodies.push_back(Box(wallX+sign*In(3)/2,y,In(1.125)/2,In(3),In(42),In(1.125)));
    }
    return bodies;
}
}
