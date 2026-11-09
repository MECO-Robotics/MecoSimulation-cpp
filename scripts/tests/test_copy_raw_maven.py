import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from copy_raw_maven import copy_package


class CopyRawMavenTest(unittest.TestCase):
    def test_copies_one_coordinate_idempotently_and_rejects_mutation(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            package = root / "package"
            destination = root / "repository"
            version = "2026.0.23-beta.3"
            artifact = "meco-simulation-wpilib-cpp"
            descriptor = {
                "version": version,
                "cppDependencies": [{
                    "groupId": "com.mecorobotics.sim",
                    "artifactId": artifact,
                    "version": version,
                    "binaryPlatforms": ["linuxx86-64"],
                }],
            }
            (package / "MecoSimulation.json").parent.mkdir(parents=True)
            (package / "MecoSimulation.json").write_text(json.dumps(descriptor))
            coordinate = package / "maven/com/mecorobotics/sim" / artifact / version
            coordinate.mkdir(parents=True)
            files = [
                f"{artifact}-{version}.pom",
                f"{artifact}-{version}-headers.zip",
                f"{artifact}-{version}-sources.zip",
                f"{artifact}-{version}-linuxx86-64static.zip",
                f"{artifact}-{version}-linuxx86-64staticdebug.zip",
            ]
            for name in files:
                (coordinate / name).write_bytes(name.encode())

            self.assertEqual(copy_package(package, destination), len(files))
            self.assertEqual(copy_package(package, destination), 0)
            target = destination / "com/mecorobotics/sim" / artifact / version / files[0]
            target.write_text("changed")
            with self.assertRaisesRegex(ValueError, "immutable"):
                copy_package(package, destination)


if __name__ == "__main__":
    unittest.main()
