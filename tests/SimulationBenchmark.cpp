#include "base/sim/Arena.h"
#include "base/sim/DrivePhysics.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>
#include <string>
#include <memory>

// Measures the complete before -> Jolt -> after interval, including arena work.
int ContactBenchmark()
{
    using namespace base::sim;
    double baseline = 0;
    for (bool compliant : {false, true})
    {
        SimulationWorld world(Backend::Full3D);
        BodySpec floor;
        floor.halfExtents = {10, 10, .05};
        floor.pose.position.z = -.05;
        world.CreateBody(floor);
        BodySpec wall;
        wall.halfExtents = {.1, 2, 1};
        wall.pose.position = {.52, 0, 1};
        world.CreateBody(wall);
        DrivePhysicsConfig c;
        c.mass = 67.6;
        c.rideHeight = .0762;
        c.bumperHalfExtents.z = .06985;
        c.centerOfMass.z = .2238;
        auto plant = compliant ? std::make_unique<DrivePhysics>(world, c, RobotContactConfig{})
                               : std::make_unique<DrivePhysics>(world, c);
        std::vector<double> samples;
        samples.reserve(5000);
        for (int i = 0; i < 5500; ++i)
        {
            // Repeated identical pressing contact, not an empty-world timing proxy.
            BodyState state;
            state.pose.position.z = c.rideHeight;
            state.linearVelocity.x = 1;
            world.SetBodyState(plant->Body(), state);
            const auto start = std::chrono::steady_clock::now();
            plant->BeforePhysics(.004);
            world.Step(.004);
            plant->AfterPhysics(.004);
            const double seconds =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            if (i >= 500)
                samples.push_back(seconds);
        }
        std::sort(samples.begin(), samples.end());
        const double p99 = samples[samples.size() * 99 / 100];
        if (!compliant)
            baseline = p99;
        std::cout << "compliant=" << compliant << " p99_seconds=" << p99
                  << " added_p99_seconds=" << p99 - baseline
                  << " budget_pass=" << (p99 < .004 && (!compliant || p99 - baseline < .0004)) << "\n";
        if (compliant && (p99 >= .004 || p99 - baseline >= .0004))
            return 1;
    }
    return 0;
}
int main(int argc, char **argv)
{
    if (argc == 2 && std::string(argv[1]) == "--bumper-contact")
        return ContactBenchmark();
    using namespace base::sim;
    std::cout << "[\n";
    bool first = true;
    for (auto mode : {Backend::Planar, Backend::Full3D})
    {
        for (auto scene : {"empty", "typical", "dense"})
        {
            SimulationClock clock(mode);
            std::chrono::steady_clock::time_point start;
            auto startMonitor = clock.Register({[&](double) { start = std::chrono::steady_clock::now(); }, {}, {}});
            DrivePhysics drive(*clock.World());
            auto driveRegistration = clock.Register(
                {[&](double dt) { drive.BeforePhysics(dt); }, [&](double dt) { drive.AfterPhysics(dt); }, {}});
            Arena arena(clock, [&] { return drive.Truth().body; }, std::string(scene) != "empty");
            if (std::string(scene) == "empty")
            {
                BodySpec floor;
                floor.halfExtents = {100, 100, .1};
                floor.pose.position = {0, 0, -.1};
                arena.AddCollider(floor);
            }
            const auto placement = Arena::kPracticeStart;
            drive.Reset({{placement.x, placement.y, .3}, {}});
            if (std::string(scene) == "dense")
            {
                for (int i = 0; i < 300; ++i)
                {
                    PieceSpec piece;
                    piece.pose.position = {placement.x + 4 + .20 * (i % 10), placement.y - 1 + .20 * ((i / 10) % 10),
                                           .12 + .22 * (i / 100)};
                    piece.airborne = mode == Backend::Full3D;
                    arena.Spawn(piece);
                }
            }
            for (int i = 0; i < 4; ++i)
                drive.SetModuleVoltages(i, 3, 0);
            std::vector<double> samples;
            uint32_t peakManifolds = 0;
            uint64_t misses = 0;
            auto monitor =
                clock.Register({{},
                                [&](double) {
                                    double seconds =
                                        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
                                    samples.push_back(seconds);
                                    if (seconds > .004)
                                        ++misses;
                                    peakManifolds = std::max(peakManifolds, clock.World()->ContactManifoldCount());
                                },
                                {}});
            for (int i = 0; i < 100; ++i)
                clock.StepRobotPeriod();
            samples.clear();
            misses = 0;
            peakManifolds = 0;
            for (int i = 0; i < 1000; ++i)
                clock.StepRobotPeriod();
            std::sort(samples.begin(), samples.end());
            if (!first)
                std::cout << ",\n";
            first = false;
            std::cout << "{\"mode\":\"" << (mode == Backend::Planar ? "jolt-planar" : "jolt-3d") << "\",\"scene\":\""
                      << scene << "\",\"bodies\":" << clock.World()->BodyCount()
                      << ",\"peakContactManifolds\":" << peakManifolds << ",\"samples\":" << samples.size()
                      << ",\"medianSeconds\":" << samples[samples.size() / 2]
                      << ",\"p99Seconds\":" << samples[samples.size() * 99 / 100] << ",\"deadlineMisses\":" << misses
                      << "}";
        }
    }
    std::cout << "\n]\n";
}
