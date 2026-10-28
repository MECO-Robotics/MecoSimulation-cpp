#!/usr/bin/env python3
"""Resolve Base plus Simulation from a staged Maven repo and link a clean consumer."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from verify_base_compatibility import validate as validate_base


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", choices=("desktop",), required=True)
    parser.add_argument("--configuration", choices=("Debug", "Release"), required=True)
    parser.add_argument("--maven-repo", type=Path)
    parser.add_argument("--published", action="store_true", help="Resolve both vendordeps from their deployed Maven URLs")
    parser.add_argument("--meco-base-root", type=Path, required=True)
    parser.add_argument("--expected-base-version", default="2026.2.1")
    args = parser.parse_args()
    if not args.published and args.maven_repo is None:
        parser.error("--maven-repo is required unless --published is set")
    simulation_root = Path(__file__).resolve().parents[1]
    base_root = args.meco_base_root.resolve()
    base_descriptor = base_root / "vendordeps/MecoBase.json"
    if not base_descriptor.is_file():
        base_descriptor = base_root / "vendor/MecoBase.json"
    if not base_descriptor.is_file():
        raise SystemExit(f"MecoBase descriptor not found under {base_root}")
    validate_base(base_descriptor, base_root / "src/main/include/base/Drive.h", args.expected_base_version)
    repo_url = args.maven_repo.resolve().as_uri() if args.maven_repo else None
    base_coordinate = (args.maven_repo.resolve() / "com/mecorobotics/frc/meco-base-cpp" /
                       args.expected_base_version) if args.maven_repo else None
    with tempfile.TemporaryDirectory(prefix="meco-sim-consumer-") as temp:
        root = Path(temp)
        (root / "vendordeps").mkdir()
        shutil.copy2(base_descriptor, root / "vendordeps/MecoBase.json")
        shutil.copy2(simulation_root / "vendordeps/MecoSimulation.json", root / "vendordeps/MecoSimulation.json")
        base_json_path = root / "vendordeps/MecoBase.json"
        base_json = base_json_path.read_text(encoding="utf-8")
        if base_coordinate is not None and base_coordinate.is_dir():
            base_json = base_json.replace(
                "https://meco-robotics.github.io/MecoBase-cpp/maven", repo_url
            )
        base_json_path.write_text(base_json, encoding="utf-8")
        simulation_descriptor = root / "vendordeps/MecoSimulation.json"
        if not args.published:
            simulation_descriptor.write_text(
                simulation_descriptor.read_text().replace(
                    "https://meco-robotics.github.io/MecoSimulation-cpp/maven", repo_url
                ), encoding="utf-8"
            )
        (root / "build.gradle").write_text("""plugins {
    id 'cpp'
    id 'edu.wpi.first.GradleRIO' version '2026.2.1'
}
def vendorCache = providers.environmentVariable('FRC_VENDOR_CACHE').orNull
nativeUtils.platformConfigs.all {
    cppCompiler.args.removeAll { it.startsWith('-std=') || it.startsWith('/std:') }
    cppCompiler.args.add(name.startsWith('windows') ? '/std:c++20' : '-std=c++20')
}
if (vendorCache) {
    repositories {
        exclusiveContent {
            forRepository {
                maven {
                    url = uri(file(vendorCache))
                    metadataSources { mavenPom(); artifact() }
                }
            }
            filter { includeGroup('com.studica.frc') }
        }
    }
}
model {
    components {
        consumer(NativeExecutableSpec) {
            targetPlatform wpi.platforms.desktop
            sources.cpp {
                source { srcDir 'src'; include '**/*.cpp' }
                exportedHeaders { srcDir 'include' }
            }
            wpi.cpp.vendor.cpp(it)
            wpi.cpp.deps.wpilib(it)
        }
    }
}
""", encoding="utf-8")
        (root / "settings.gradle").write_text("rootProject.name = 'MecoSimulationConsumer'\n", encoding="utf-8")
        (root / "src").mkdir()
        (root / "include").mkdir()
        (root / "src/consumer.cpp").write_text("""#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <hal/HAL.h>
#include <frc/geometry/Pose2d.h>
#include <frc/simulation/SimHooks.h>
#include <base/PortableSwerve.h>
#include <base/SimulationRuntime.h>
#include <meco/sim/DriveSimulationAdapter.h>
#include <meco/sim/BatterySupply.h>
#include <meco/sim/SimulationPose.h>
#include <meco/sim/SimulatedVision.h>
#include <meco/sim/IdealVisionObservation.h>
#include <meco/sim/ParticleContacts.h>
#include <base/sim/FootprintClearance.h>
#include <base/sim/SimulationTuning.h>
#include <base/sim/fields/Rebuilt2026.h>

class Module final : public base::SwerveModuleIO {
 public:
  base::SwerveModuleInputs Read() override { return inputs; }
  void Set(const frc::SwerveModuleState&, double) override {}
  void SetVoltage(double) override {}
  void Stop() override {}
  std::array<double, 2> SimVoltages(double, double) override { return {2.0, 0.0}; }
  void SetSimInputs(base::SwerveModuleInputs value) override { inputs = value; }
 private:
  base::SwerveModuleInputs inputs{};
};

class Gyro final : public base::SwerveGyroIO {
 public:
  base::SwerveGyroInputs Read() override { return {yaw, true}; }
  void SetSimYaw(double value) override { yaw = value; }
 private:
  double yaw{};
};

int main() {
  using namespace units::literals;
  const auto floor = meco::sim::FloorReferencedRobotPose(
      frc::Pose3d{0_m, 0_m, .2_m, frc::Rotation3d{}}, .2_m);
  if (std::abs(floor.Z().value()) > 1e-12) return 30;
  const frc::AprilTagFieldLayout tags{
      std::vector<frc::AprilTag>{{1, frc::Pose3d{2_m, 0_m, 0_m,
          frc::Rotation3d{0_deg, 0_deg, 180_deg}}}}, 10_m, 8_m};
  if (meco::sim::VisibleTags(floor, frc::Transform3d{}, tags,
                           60_deg, 50_deg, 3_m) != std::vector<int>{1}) return 31;
  if (!meco::sim::MakeIdealVisionObservation(floor, 0_s, tags, true, 1_m, {1, 1, 1})) return 32;
  if (!base::sim::IsFootprintClear(2, 2, 0, .4, .4, 1, 10, 8, 0, {})) return 33;
  const base::sim::TuningInputs tuning{50, 5, 1, .05, 6, 40, .02,
      {{{.3, .3}, {.3, -.3}, {-.3, .3}, {-.3, -.3}}}, 1, 1};
  if (base::sim::DeriveSimulationTuning(tuning).maxLinearAcceleration <= 0) return 34;
  std::vector<meco::sim::Particle> ground, airborne;
  meco::sim::ResolveParticleContacts(ground, airborne, {}, .1, 0, 0, {});
  if (!HAL_Initialize(500, 0)) return 2;
  int result = 1;
  {
  base::DriveConfig config{};
  config.backend = base::DriveBackend::SparkMaxNavX;
  config.simulation.modules[0].driveOrientation =
      base::SimulationDeviceOrientation::CounterClockwisePositive;
  auto makeIO = [] {
    base::SwerveIO io;
    for (auto& module : io.modules) module = std::make_unique<Module>();
    io.gyro = std::make_unique<Gyro>();
    return io;
  };
  base::Drive drive(config, makeIO());
  base::Drive secondDrive(config, makeIO());
  auto simulation = meco::sim::CreateDriveSimulationAdapter(config, base::sim::Backend::Full3D);
  simulation->Reset(frc::Pose2d{});
  if (!std::isfinite(simulation->PhysicsTruth().body.pose.rotation.w) ||
      base::sim::fields::rebuilt2026::FieldColliders().size() != 40) return 3;
  auto& arena = simulation->CreateArena(base::sim::ArenaScene::Generic);
  drive.SetSimulationAdapter(std::move(simulation));
  drive.ResetSimulation(frc::Pose2d{});
  base::sim::PieceSpec piece;
  piece.pose.position = {4.0, 2.0, 0.6};
  piece.velocity = {3.0, 0.0, 0.0};
  piece.airborne = true;
  const auto pieceId = arena.Spawn(piece);
  drive.RunVelocity(frc::ChassisSpeeds{units::meters_per_second_t{1.0}, units::meters_per_second_t{0.0},
                                        units::radians_per_second_t{0.0}});
  bool clockAdvancedOnce = true;
  for (int step = 0; step < 50; ++step) {
    const double before = base::simulation::GetSimulationRuntime().Time();
    frc::sim::StepTiming(units::second_t{0.02});
    drive.SimulationPeriodic();
    const double afterFirst = base::simulation::GetSimulationRuntime().Time();
    secondDrive.SimulationPeriodic();
    meco::sim::UpdateBatterySupply(base::simulation::GetSimulationRuntime());
    const double afterSecond = base::simulation::GetSimulationRuntime().Time();
    clockAdvancedOnce = clockAdvancedOnce && std::abs(afterFirst - before - 0.02) < 1e-9 &&
                        afterSecond == afterFirst;
  }
  const auto pieces = arena.Pieces();
  const auto spawned = std::find_if(pieces.begin(), pieces.end(),
                                    [pieceId](const auto& item) { return item.id == pieceId; });
  const auto truth = drive.SimulationTruth();
  std::cerr << "arena pieces=" << pieces.size() << " spawned x="
            << (spawned == pieces.end() ? -1.0 : spawned->state.pose.position.x)
            << " robot x=" << (truth ? truth->x : -1.0) << '\\n';
  result = clockAdvancedOnce && pieceId != 0 && spawned != pieces.end() &&
                             std::abs(spawned->state.pose.position.x - 4.0) > 0.1 && truth &&
                             std::isfinite(truth->x) && std::abs(truth->x) > 0.02 && std::isfinite(truth->yaw)
                         ? 0
                         : 1;
  }
  // WPILib 2026.2.1's native trajectory singleton double-frees during global
  // teardown on this host; release simulation objects before framework globals.
  std::_Exit(result);
}
""", encoding="utf-8")
        wrapper = simulation_root / "adapters/wpilib-2026/gradlew.bat" if os.name == "nt" else simulation_root / "adapters/wpilib-2026/gradlew"
        command = [str(wrapper), "-p", str(root), "consumer" + args.configuration + "Executable", "--no-daemon", "--max-workers=2"]
        if os.name == "nt":
            command.insert(0, "cmd.exe")
            command.insert(1, "/c")
        subprocess.run(command, cwd=root, check=True)
        executable = root / "build/exe/consumer" / args.configuration.lower() / ("consumer.exe" if os.name == "nt" else "consumer")
        runtime_env = os.environ.copy()
        platform_dir = "windows/x86-64/shared" if os.name == "nt" else "linux/x86-64/shared"
        gradle_cache = Path.home() / ".gradle/caches"
        runtime_dirs = [str(path) for path in gradle_cache.rglob(platform_dir) if path.is_dir()]
        if os.name == "nt":
            # GradleRIO native dependencies may be unpacked into Gradle's transform cache.
            runtime_dirs.extend(str(path.parent) for path in gradle_cache.rglob("*.dll"))
            # Studica Windows Maven artifacts are staged separately for offline CI resolution.
            vendor_cache = base_root / "build/vendor-cache"
            if vendor_cache.is_dir():
                runtime_dirs.extend(str(path.parent) for path in vendor_cache.rglob("*.dll"))
        variable = "PATH" if os.name == "nt" else "LD_LIBRARY_PATH"
        runtime_env[variable] = os.pathsep.join(dict.fromkeys(runtime_dirs + [runtime_env.get(variable, "")]))
        if os.name == "nt":
            # Windows resolves dependent DLLs beside the executable before PATH entries.
            copied = {}
            for directory in runtime_dirs:
                for library in Path(directory).glob("*.dll"):
                    destination = executable.parent / library.name
                    if destination.exists():
                        if destination.read_bytes() != library.read_bytes():
                            raise RuntimeError(f"Conflicting Windows runtime DLL: {library.name}")
                        continue
                    shutil.copy2(library, destination)
                    copied[library.name] = str(library)
            print(f"Staged {len(copied)} Windows runtime DLLs beside consumer", flush=True)
        subprocess.run([str(executable)], cwd=root, env=runtime_env, check=True)


if __name__ == "__main__":
    main()
