#pragma once

#include "base/sim/RobotContactConfig.h"

inline base::sim::RobotContactConfig SyntheticRobotContactConfig()
{
    base::sim::RobotContactConfig c;
    c.chassisClearance = .03;
    c.panThickness = .01;
    c.panHalfLength = .25;
    c.panHalfWidth = .20;
    c.bumperFloorGap = .015;
    c.noodleDiameter = .08;
    c.backingThickness = .02;
    c.backingHeight = .10;
    c.backingFloorGap = .015;
    c.compressionStrain = {0, .25, .5, .7, .8};
    c.compressionStressPa = {0, 12000, 42000, 115000, 230000};
    c.dampingRatio = .9;
    c.friction = .72;
    return c;
}
