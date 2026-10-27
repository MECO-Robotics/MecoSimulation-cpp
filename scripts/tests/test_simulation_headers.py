from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation_headers import collect_simulation_headers, validate_simulation_header_names


class SimulationHeadersTest(unittest.TestCase):
    def test_collector_uses_only_simulation_owned_header_roots(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            owned = (
                root / "include/base/sim/Arena.h",
                root / "include/base/sim/DrivePhysics.h",
                root / "include/base/sim/RobotContactConfig.h",
                root / "include/base/sim/SimulationWorld.h",
                root / "adapters/wpilib-2026/include/meco/sim/DriveSimulationAdapter.h",
                root / "adapters/wpilib-2026/include/meco/sim/BatterySupply.h",
                root / "adapters/wpilib-2026/include/meco/sim/SimulationPose.h",
                root / "adapters/wpilib-2026/include/meco/sim/SimulatedVision.h",
                root / "adapters/wpilib-2026/include/meco/sim/SimulationDiagnostics.h",
                root / "include/base/sim/fields/Rebuilt2026.h",
            )
            for path in owned:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("// simulation-owned\n", encoding="utf-8")
            stale_base_header = root / "MecoBase-cpp/src/main/include/base/Drive.h"
            stale_base_header.parent.mkdir(parents=True)
            stale_base_header.write_text("// Base-owned\n", encoding="utf-8")

            entries = collect_simulation_headers(root)

            self.assertEqual({name for _, name in entries}, {
                "base/sim/Arena.h",
                "base/sim/DrivePhysics.h",
                "base/sim/RobotContactConfig.h",
                "base/sim/SimulationWorld.h",
                "meco/sim/DriveSimulationAdapter.h",
                "meco/sim/BatterySupply.h",
                "meco/sim/SimulationPose.h",
                "meco/sim/SimulatedVision.h",
                "meco/sim/SimulationDiagnostics.h",
                "base/sim/fields/Rebuilt2026.h",
            })

    def test_rejects_base_headers_in_simulation_archive(self):
        valid = {
            "base/sim/Arena.h",
            "base/sim/DrivePhysics.h",
                "base/sim/RobotContactConfig.h",
            "base/sim/SimulationWorld.h",
            "meco/sim/DriveSimulationAdapter.h",
            "meco/sim/BatterySupply.h",
                "meco/sim/SimulationPose.h",
                "meco/sim/SimulatedVision.h",
            "meco/sim/SimulationDiagnostics.h",
            "base/sim/fields/Rebuilt2026.h",
        }
        with self.assertRaisesRegex(ValueError, "Base-owned headers.*base/Drive.h"):
            validate_simulation_header_names(valid | {"base/Drive.h"})


if __name__ == "__main__":
    unittest.main()
