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
    parser.add_argument("--warm-dependencies", action="store_true", help="Resolve dependencies only; does not compile or validate the consumer")
    parser.add_argument("--offline", action="store_true", help="Require dependencies to be present in the warmed Gradle cache")
    parser.add_argument("--meco-base-root", type=Path, required=True)
    parser.add_argument("--expected-base-version", default="2026.2.12")
    args = parser.parse_args()
    if args.warm_dependencies and args.offline:
        parser.error("--offline cannot be used while warming dependencies")
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
    cppCompiler.args.add(name.startsWith('windows') ? '/std:c++latest' : '-std=c++23')
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
def baseConsumerMaven = providers.gradleProperty('baseConsumerMaven').orNull
if (baseConsumerMaven) {
    repositories {
        exclusiveContent {
            forRepository {
                maven {
                    name = 'stagedMecoBaseConsumer'
                    url = uri(baseConsumerMaven)
                    metadataSources { mavenPom(); artifact() }
                }
            }
            filter {
                includeModule('com.mecorobotics.frc', 'meco-base-cpp')
            }
        }
    }
}
tasks.register('warmConsumerDependencies') {
    doLast {
        def prefix = "consumer${providers.gradleProperty('warmConfiguration').get().toLowerCase()}Executable_uber".toString()
        def expected = ['headers', 'link', 'runtime'].collect { prefix + it } as Set
        def selected = configurations.findAll { it.canBeResolved && it.name in expected }
        if ((selected.collect { it.name } as Set) != expected)
            throw new GradleException("Missing native consumer dependency configurations: ${expected - (selected.collect { it.name } as Set)}")
        selected.each { dependencyConfiguration ->
            def files = dependencyConfiguration.files
            if (files.empty) throw new GradleException("Empty consumer dependency configuration: ${dependencyConfiguration.name}")
            println "Warmed ${dependencyConfiguration.name}: ${files.size()} artifacts"
            if (dependencyConfiguration.name == prefix + 'headers') {
                def paths = files.collect { it.path.toLowerCase() }
                if (!paths.any { it.contains('meco-base-cpp') } ||
                    !paths.any { it.contains('meco-simulation-wpilib-cpp') })
                    throw new GradleException('Missing Base or Simulation headers in warmed dependencies')
            }
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
#include <memory>
#include <utility>
#include <hal/HAL.h>
#include <frc/geometry/Pose2d.h>
#include <frc/simulation/SimHooks.h>
#include <frc/simulation/DriverStationSim.h>
#include <networktables/NetworkTable.h>
#include <networktables/StringTopic.h>
#include <base/SimulationRuntime.h>
#include <meco/sim/DriverInputTelemetry.h>
#include <meco/sim/SimulatedPieceCameraBank.h>
#include <meco/sim/BatterySupply.h>
#include <meco/sim/ColliderNavigation.h>
#include <meco/sim/LoadedMotorIO.h>
#include <meco/sim/MotorSimulationFactories.h>
#include <meco/sim/SimulationNetwork.h>
#include <meco/sim/SimulationPose.h>
#include <meco/sim/SimulationLifecycle.h>
#include <meco/sim/SimulatedVision.h>
#include <meco/sim/SimulatedVisionSource.h>
#include <meco/sim/IdealVisionObservation.h>
#include <meco/sim/ParticleContacts.h>
#include <meco/sim/ParticleFieldContacts.h>
#include <base/sim/FootprintClearance.h>
#include <base/sim/SimulationTuning.h>
#include <meco/sim/DriveTuning.h>
#include <meco/sim/SimulatedPieceSensor.h>
#include <meco/sim/PracticeMatchController.h>
#include <base/sim/fields/Rebuilt2026.h>

int RunDriveSmoke();

enum class TestTuning { Automatic, Configured };

int main() {
  using namespace units::literals;
  meco::sim::ConfigureSimulationNetwork();
  const auto motor = meco::sim::LoadedMotorModelFromStallAndFreeSpeed(12, 120, 3, 6000);
  if (std::abs(motor.resistanceOhms - .1) > 1e-12 ||
      std::abs(motor.torqueConstantNmPerAmp - .025) > 1e-12) return 48;
  const auto floor = meco::sim::FloorReferencedRobotPose(
      frc::Pose3d{0_m, 0_m, .2_m, frc::Rotation3d{}}, .2_m);
  if (std::abs(floor.Z().value()) > 1e-12) return 30;
  const frc::AprilTagFieldLayout tags{
      std::vector<frc::AprilTag>{{1, frc::Pose3d{2_m, 0_m, 0_m,
          frc::Rotation3d{0_deg, 0_deg, 180_deg}}}}, 10_m, 8_m};
  if (meco::sim::VisibleTags(floor, frc::Transform3d{}, tags,
                           60_deg, 50_deg, 3_m) != std::vector<int>{1}) return 31;
  if (!meco::sim::MakeIdealVisionObservation(floor, 0_s, tags, true, 1_m, {1, 1, 1})) return 32;
  meco::sim::SimulatedVisionSource vision;
  auto visionIo = vision.MakeIO();
  vision.Update(floor, 1_s, tags, true, 1_m, {1, 1, 1}, true, true, true);
  base::VisionInputs visionInputs;
  visionIo->UpdateInputs(visionInputs);
  if (!visionInputs.connected || !visionInputs.aligned || visionInputs.observations.size() != 1 ||
      visionInputs.observations[0].timestamp != 1_s) return 45;
  visionIo->UpdateInputs(visionInputs);
  if (!visionInputs.connected || !visionInputs.observations.empty()) return 46;
  vision.Reset();
  visionIo->UpdateInputs(visionInputs);
  if (visionInputs.connected || !visionInputs.observations.empty()) return 47;
  if (!meco::sim::ColliderRectangles({}, 0, 1).empty()) return 51;
  if (!base::sim::IsFootprintClear(2, 2, 0, .4, .4, 1, 10, 8, 0, {})) return 33;
  const base::sim::TuningInputs tuning{50, 5, 1, .05, 6, 40, .02,
      {{{.3, .3}, {.3, -.3}, {-.3, .3}, {-.3, -.3}}}, 1, 1};
  if (base::sim::DeriveSimulationTuning(tuning).maxLinearAcceleration <= 0) return 34;
  base::DriveConfig tuneConfig;
  tuneConfig.massKg = 62;
  tuneConfig.autoTranslationKP = 123;
  tuneConfig.autoRotationKP = 98;
  auto retained = meco::sim::ConfigureDriveTuning(tuneConfig, TestTuning::Configured, .02, .08, .50);
  auto derived = meco::sim::ConfigureDriveTuning(tuneConfig, TestTuning::Automatic, .02, .08, .50);
  if (retained.autoTranslationKP != 123 || retained.autoRotationKP != 98 ||
      derived.autoTranslationKP == 123 || derived.autoRotationKP == 98 ||
      retained.massKg != 62 || derived.massKg != 62) return 55;
  std::vector<meco::sim::Particle> ground, airborne;
  meco::sim::ResolveParticleContacts(ground, airborne, {}, .1, 0, 0, {});
  const meco::sim::ParticleFieldContacts field{base::sim::fields::rebuilt2026::ParticleFieldColliders()};
  ground.push_back({{.03, 2, .5}, {-1, 0, 0}});
  field.Resolve(ground, .07, .25);
  if (ground.front().position.x < .069 || ground.front().velocity.x <= 0) return 44;
  meco::sim::PieceSensorConfig sensorConfig;
  sensorConfig.dropout=0; sensorConfig.angularNoise=0;
  meco::sim::SimulatedPieceSensor sensor(sensorConfig);
  const std::vector<frc::Translation3d> points{{2_m,0_m,0_m},{-1_m,0_m,0_m}};
  meco::sim::SimulatedPieceSensorBridge bridge(sensorConfig);
  auto pieceIo = bridge.MakeIO();
  bridge.Update(frc::Pose3d{}, frc::Transform3d{}, points, {}, 1_s, 7);
  auto pieceFrame = pieceIo->ReadFrame();
  if (!pieceFrame || pieceFrame->sequence != 7 || pieceFrame->detections.size() != 1 ||
      pieceIo->ReadFrame()) return 49;
  bridge.Update(frc::Pose3d{}, frc::Transform3d{}, points, {}, 1_s, 8);
  bridge.Reset();
  if (pieceIo->ReadFrame()) return 50;
  const auto ray=sensor.Observe(frc::Pose3d{}, points, {}, 1_s);
  if (!ray || std::abs(ray->yawDegrees)>1e-9) return 40;
  base::sim::BodySpec blocker;
  blocker.pose.position={1,0,0};
  const std::vector<base::sim::BodySpec> blockers{blocker};
  if (sensor.Observe(frc::Pose3d{},points,blockers,1_s)) return 41;
  if (!HAL_Initialize(500, 0)) return 2;
  bool loadRead=false;
  int providerBuilds=0;
  auto provider=meco::sim::MakeLazyMotorSimulationProvider([&] {
    ++providerBuilds;
    return meco::sim::MakeMotorSimulationProvider({
        {base::Vendor::SparkMax, "", 1, .02, motor, [&] {loadRead=true; return 0.;}}});
  });
  auto providerCopy=provider;
  base::MotorConfig motorConfig; motorConfig.ids={1}; motorConfig.reversed={false};
  if (loadRead || providerBuilds) return 61;
  auto simMotor=provider(base::Vendor::SparkMax, motorConfig);
  auto secondMotor=providerCopy(base::Vendor::SparkMax, motorConfig);
  if (!secondMotor || providerBuilds!=1) return 64;
  if (!simMotor || loadRead) return 62;
  simMotor->SetVoltage(5); simMotor->StepSimulation(.02_s);
  if (!loadRead || simMotor->Read().velocity<=0) return 63;
  struct GamepadState { long long sequence=1, sourceTimestampUs=1; bool accepted=true; double ageMs=0; };
  meco::sim::DriverInputTelemetry driverTelemetry(false);
  driverTelemetry.Publish(GamepadState{}, 0, 0, 0, 0, 0, 0, frc::Rotation2d{}, true);
  meco::sim::SimulatedPieceCameraBank cameraBank(1);
  meco::sim::PieceSensorConfig bankConfig; bankConfig.dropout=0; bankConfig.angularNoise=0;
  auto bankIo=cameraBank.Configure(0, bankConfig, [] { return frc::Transform3d{}; });
  const std::vector<frc::Translation3d> cameraPoints{{2_m,0_m,0_m}};
  auto cameraSamples=cameraBank.Update(frc::Pose3d{}, cameraPoints, {}, [] {return 1_s;});
  auto cameraFrame=bankIo->ReadFrame();
  if (cameraSamples.size()!=1 || cameraSamples[0].detections!=1 || !cameraFrame || cameraFrame->sequence!=1) return 60;
  cameraBank.Reset();
  {
    meco::sim::SimulationLifecycle lifecycle({18, 11, 1.6, .02, 20, 3, 140});
    lifecycle.Initialize();
    const auto before = base::simulation::GetSimulationRuntime().Time();
    lifecycle.BeforeScheduler(); lifecycle.AfterScheduler();
    lifecycle.BeforeScene(); lifecycle.AfterScene();
    if (base::simulation::GetSimulationRuntime().Time() != before) return 60;
  }
  // Verify the packaged implementation, including both station mappings.
  auto practiceNt=nt::NetworkTableInstance::Create();
  practiceNt.StartLocal();
  {
    meco::sim::PracticeMatchController practice({20,3,140},practiceNt);
    auto table=practiceNt.GetTable("/Sim/Practice");
    auto command=table->GetStringTopic("Command").Publish();
    for(const auto& [station,expected]: std::vector<std::pair<std::string,HAL_AllianceStationID>>{
          {"Blue1",HAL_AllianceStationID_kBlue1},{"Red1",HAL_AllianceStationID_kRed1}}) {
      table->GetEntry("Config/AllianceStation").SetString(station);
      command.Set("start");practice.Update();
      if(!frc::sim::DriverStationSim::GetEnabled() ||
          frc::sim::DriverStationSim::GetAllianceStationId()!=expected) return 42;
      command.Set("abort");practice.Update();
      if(frc::sim::DriverStationSim::GetEnabled()) return 43;
    }
  }
  practiceNt.StopLocal();nt::NetworkTableInstance::Destroy(practiceNt);
  const int result = RunDriveSmoke();
  // Preserve normal unwinding for a failed check inside the drive portion.
  if (result > 1) return result;
  // WPILib 2026.2.1's native trajectory singleton double-frees during global
  // teardown on this host; release simulation objects before framework globals.
  std::_Exit(result);
}
""", encoding="utf-8")
        (root / "src/drive_smoke.cpp").write_text("""#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <utility>
#include <frc/geometry/Pose2d.h>
#include <frc/simulation/SimHooks.h>
#include <base/PortableSwerve.h>
#include <base/SimulationRuntime.h>
#include <meco/sim/DriveSimulationAdapter.h>
#include <meco/sim/DriveScene.h>
#include <meco/sim/FootprintNavigation.h>
#include <meco/sim/SimulationPower.h>
#include <meco/sim/PracticeMatchController.h>
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

int RunDriveSmoke() {
  using namespace units::literals;
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
  base::FootprintAutoDriveConfig footprint{};
  auto footprintCommand=meco::sim::FootprintCommandWithOutcome(drive,
      []() -> std::optional<frc::Pose2d> {return std::nullopt;},
      [] {return std::vector<base::FieldRectangle>{};}, footprint,
      nt::NetworkTableInstance::GetDefault().GetEntry("/Smoke/FootprintOutcome"),
      [](base::FootprintAutoDriveOutcome) {return "complete";});
  // Exercise shared scene ownership, reset settling, and a blocked auto pose
  // through the actual Drive and simulation adapter.
  {
  base::Drive sceneDrive(config, makeIO());
  bool clear = true;
  int sensorResets = 0;
  meco::sim::DriveScene scene(sceneDrive, {.fieldCollidersEnabled=false, .feedbackLoops=2,
      .colliders={}, .contact={}, .clearStartingPose=[&](const frc::Pose2d&) { return clear; },
      .resetSensors=[&] { ++sensorResets; }});
  scene.Reset(frc::Pose2d{});
  scene.BeginAutonomousSchedule();
  sceneDrive.ResetPose(frc::Pose2d{1_m, 1_m, frc::Rotation2d{}});
  if (scene.EndAutonomousSchedule() || scene.Ready() || sensorResets != 1) return 52;
  auto initial = sceneDrive.SimulationTruth();
  if (!initial || std::abs(initial->x - 1) > .01 || std::abs(initial->y - 1) > .01) return 53;
  for (int i=0; i<2; ++i) {
    frc::sim::StepTiming(.02_s);
    sceneDrive.SimulationPeriodic();
    if (!scene.Update() || scene.Ready() != (i == 1)) return 55;
  }
  // Enable clearance rejection using an enabled but empty field.
  sceneDrive.SetPoseResetCallback({});
  }
  {
  base::Drive sceneDrive(config, makeIO());
  bool clear = false;
  meco::sim::DriveScene scene(sceneDrive, {.fieldCollidersEnabled=true, .feedbackLoops=2,
      .colliders={}, .contact={}, .clearStartingPose=[&](const frc::Pose2d&) { return clear; }, .resetSensors={}});
  scene.Reset(frc::Pose2d{1_m, 1_m, frc::Rotation2d{}});
  scene.BeginAutonomousSchedule();
  sceneDrive.ResetPose(frc::Pose2d{5_m, 5_m, frc::Rotation2d{}});
  if (!scene.EndAutonomousSchedule()) return 57;
  auto afterBlocked = sceneDrive.SimulationTruth();
  if (!afterBlocked || std::abs(afterBlocked->x - 5) < 1 || std::abs(afterBlocked->y - 5) < 1) return 58;
  sceneDrive.SetPoseResetCallback({});
  }
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
  meco::sim::SimulationPower power({18, 11, 1.6, 0.02});
  auto startPractice = meco::sim::StartPracticeMatch();
  auto optionalPractice = meco::sim::TryStartPracticeMatch();
  if (!optionalPractice) return 64;
  bool clockAdvancedOnce = true;
  for (int step = 0; step < 50; ++step) {
    const double before = base::simulation::GetSimulationRuntime().Time();
    frc::sim::StepTiming(units::second_t{0.02});
    drive.SimulationPeriodic();
    const double afterFirst = base::simulation::GetSimulationRuntime().Time();
    secondDrive.SimulationPeriodic();
    power.Update(base::simulation::GetSimulationRuntime());
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
  return result;
}
""", encoding="utf-8")
        wrapper = simulation_root / "adapters/wpilib-2026/gradlew.bat" if os.name == "nt" else simulation_root / "adapters/wpilib-2026/gradlew"
        task = "warmConsumerDependencies" if args.warm_dependencies else "consumer" + args.configuration + "Executable"
        command = [str(wrapper), "-p", str(root), task, "--no-daemon", "--max-workers=2"]
        if args.warm_dependencies:
            command.append("-PwarmConfiguration=" + args.configuration)
        if args.offline:
            command.append("--offline")
        if base_coordinate is not None and base_coordinate.is_dir():
            command.append("-PbaseConsumerMaven=" + repo_url)
        if os.name == "nt":
            command.insert(0, "cmd.exe")
            command.insert(1, "/c")
        subprocess.run(command, cwd=root, check=True)
        if args.warm_dependencies:
            return  # This only warms the cache; it is not consumer validation.
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
