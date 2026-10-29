import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from merge_base_maven import merge


class MergeBaseMavenTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.simulation = self.root / "simulation"
        self.base = self.root / "base"
        self.sim_maven = self.simulation / "maven"
        self.base_maven = self.base / "maven"
        self.sim_maven.mkdir(parents=True)
        self.base_maven.mkdir(parents=True)
        (self.sim_maven / "sim.pom").write_text("sim")
        (self.simulation / "SHA256SUMS.json").write_text(json.dumps({"artifacts": {}}))
        (self.base / "MecoBase.json").write_text(json.dumps({
            "version": "2026.2.5-dev.1",
            "cppDependencies": [{"groupId": "com.mecorobotics.frc", "artifactId": "meco-base-cpp", "version": "2026.2.5-dev.1"}],
        }))
        coordinate = self.base_maven / "com/mecorobotics/frc/meco-base-cpp/2026.2.5-dev.1"
        coordinate.mkdir(parents=True)
        (coordinate / "meco-base-cpp-2026.2.5-dev.1.pom").write_text("base")
        for platform in ("linuxathena", "linuxx86-64", "windowsx86-64"):
            for suffix in ("static", "staticdebug"):
                (coordinate / f"meco-base-cpp-2026.2.5-dev.1-{platform}{suffix}.zip").write_text(platform + suffix)
        for kind in ("headers", "sources"):
            (coordinate / f"meco-base-cpp-2026.2.5-dev.1-{kind}.zip").write_text(kind)

    def tearDown(self):
        self.temporary.cleanup()

    def test_merges_base_artifacts_and_checksums_both_packages(self):
        merge(self.base, self.simulation)
        relative = "maven/com/mecorobotics/frc/meco-base-cpp/2026.2.5-dev.1/meco-base-cpp-2026.2.5-dev.1.pom"
        self.assertEqual((self.sim_maven / relative.removeprefix("maven/")).read_text(), "base")
        manifest = json.loads((self.simulation / "SHA256SUMS.json").read_text())
        self.assertIn("maven/sim.pom", manifest["artifacts"])
        self.assertIn(relative, manifest["artifacts"])
        self.assertEqual(len(manifest["artifacts"]), 10)
        self.assertEqual(manifest["artifacts"][relative], hashlib.sha256(b"base").hexdigest())

    def test_refuses_to_replace_conflicting_coordinate(self):
        conflict = self.sim_maven / "com/mecorobotics/frc/meco-base-cpp/2026.2.5-dev.1/meco-base-cpp-2026.2.5-dev.1.pom"
        conflict.parent.mkdir(parents=True)
        conflict.write_text("different")
        with self.assertRaisesRegex(ValueError, "Conflicting Maven artifact"):
            merge(self.base, self.simulation)

    def test_rejects_incomplete_cross_platform_base_package(self):
        missing = self.base_maven / "com/mecorobotics/frc/meco-base-cpp/2026.2.5-dev.1/meco-base-cpp-2026.2.5-dev.1-windowsx86-64staticdebug.zip"
        missing.unlink()
        with self.assertRaisesRegex(FileNotFoundError, "Base package is incomplete"):
            merge(self.base, self.simulation)


if __name__ == "__main__":
    unittest.main()
