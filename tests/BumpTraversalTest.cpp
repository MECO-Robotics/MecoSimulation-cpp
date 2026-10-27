#include "base/sim/DrivePhysics.h"
#include "base/sim/fields/Rebuilt2026.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <memory>
using namespace base::sim;
int main() {
    // Retain only scenarios with a required traversal outcome.
    for (int scenario : {0, 2})
    {
        const bool compliant = scenario != 0;
        SimulationWorld world(Backend::Full3D);
        BodySpec floor;
        floor.halfExtents = {20, 20, .05};
        floor.pose.position.z = -.05;
        floor.friction = 1;
        world.CreateBody(floor);
        for (const auto &collider : fields::rebuilt2026::FieldColliders())
            world.CreateBody(collider);
        DrivePhysicsConfig c;
        c.mass = 67.6;
        c.friction = 2.1;
        c.bumperHalfExtents.z = .06985;
        c.rideHeight = .0762;
        c.centerOfMass.z = .2238;
        c.driveRatio = 5.8909090909090915;
        c.steerRatio = 12.1;
        c.couplingRatio = 4.909090909090909;
        c.wheelInertia = c.steerInertia = .01;
        c.motorResistance = 12. / 483;
        c.motorKt = 9.37 / 483;
        constexpr double pi = 3.141592653589793;
        c.motorKv = (5800 * 2 * pi / 60) / (12 - c.motorResistance * 2);
        c.driveCurrentLimit = 50;
        c.steerCurrentLimit = 30;
        c.moduleLocations = {
            {{.2794, .2794, 0}, {.2794, -.2794, 0}, {-.2794, .2794, 0}, {-.2794, -.2794, 0}}};
        RobotContactConfig contact;
        if (scenario == 2)
            contact.bumperFloorGap = contact.backingFloorGap = .0127;
        auto plant = compliant ? std::make_unique<DrivePhysics>(world, c, contact)
                               : std::make_unique<DrivePhysics>(world, c);
        auto &drive = *plant;
        drive.Reset({{compliant ? .6 : 2.8, 2.5106376, c.rideHeight}, {}});
        // Running approach to a low-clearance skid: with every wheel lifted, there
        // is no magic propulsion. Momentum must carry the bumper over the crest.
        if (!compliant)
            world.SetLinearVelocity(drive.Body(), {4, 0, 0});
        double maxCompression = 0, maxFoamForce = 0;
        unsigned panHits = 0, backingHits = 0;
        double maxHeight = 0;
        int supportedWithWheelsLifted = 0;
        bool rodeBump = false, cleared = false;
        double maxTilt = 0;
        for (int n = 0; n < 2500; ++n)
        {
            const auto before = drive.Truth();
            for (size_t i = 0; i < 4; ++i)
            {
                const auto &m = before.modules[i];
                const double steer =
                    std::clamp(-100 / (2 * pi) * m.steerAngle - .5 / (2 * pi) * m.steerVelocity, -12., 12.);
                // Full-speed standalone plant check, without battery/Phoenix firmware.
                // Packaged joystick acceptance also verifies the actual consumer.
                // Representative speed feedback, starting with matching stationary
                // wheel/chassis state. No pre-seeded momentum in compliant cases.
                const double target = 5.24;
                const double voltage = compliant
                                           ? std::clamp(target * c.driveRatio / (c.wheelRadius * c.motorKv) +
                                                            2 * (target - m.wheelVelocity * c.wheelRadius),
                                                        -12., 12.)
                                           : 12.;
                drive.SetModuleVoltages(i, voltage, steer);
            }
            drive.BeforePhysics(.004);
            world.Step(.004);
            drive.AfterPhysics(.004);
            const auto truth = drive.Truth();
            const auto &rotation = truth.body.pose.rotation;
            maxTilt = std::max(
                maxTilt,
                std::acos(std::clamp(1 - 2 * (rotation.x * rotation.x + rotation.y * rotation.y), -1., 1.)));
            if (truth.body.pose.position.x > 6.2 && std::abs(truth.body.pose.position.z - c.rideHeight) < .02)
            {
                cleared = true;
                break;
            }
            panHits += truth.rigidContacts.pan;
            backingHits += truth.rigidContacts.backing;
            for (auto v : truth.foamCompression)
                maxCompression = std::max(maxCompression, v);
            for (auto v : truth.foamNormalForce)
                maxFoamForce = std::max(maxFoamForce, v);
            if (!std::isfinite(truth.body.pose.position.z) || truth.body.pose.position.z < -.02 ||
                truth.body.pose.position.z > 2)
                throw std::runtime_error("Bump contact became unstable");
            if (truth.body.pose.position.x > 3.6 && truth.body.pose.position.x < 5.7)
            {
                maxHeight = std::max(maxHeight, truth.body.pose.position.z);
                const bool unloaded = std::all_of(truth.modules.begin(), truth.modules.end(),
                                                  [](auto &m) { return m.normalForce < 1; });
                supportedWithWheelsLifted +=
                    unloaded && world.ContactManifoldCount() > 0 && truth.body.pose.position.z > .10;
                if (unloaded)
                    for (const auto &m : truth.modules)
                        if (std::hypot(m.longitudinalForce, m.lateralForce) > 3)
                            throw std::runtime_error("Lifted tire still propels chassis");
            }
            if (truth.body.pose.position.x > 4.8 && truth.body.pose.position.z > .10)
                rodeBump = true;
        }
        std::cout << "scenario=" << scenario << " max_tilt_deg=" << maxTilt * 180 / pi
                  << " compliant=" << compliant << " max_compression=" << maxCompression
                  << " max_foam_force=" << maxFoamForce << " pan_contacts=" << panHits
                  << " backing_contacts=" << backingHits << " cleared=" << cleared
                  << " final_x=" << drive.Truth().body.pose.position.x << " rode_bump=" << rodeBump
                  << " max_body_height=" << maxHeight
                  << " bumper_supported_unloaded_steps=" << supportedWithWheelsLifted << std::endl;
        if (scenario == 2 && (!cleared || maxTilt > pi / 4))
            throw std::runtime_error(
                "Drive from rest must clear entire bump upright with 1/2-inch flush bumper clearance");
        if (!compliant && (!rodeBump || maxHeight < .18 || maxHeight > .35 || supportedWithWheelsLifted < 5))
            throw std::runtime_error("Bumper must slide over the crest and support the lifted chassis "
                                     "without a leading-edge launch");
    }
}
