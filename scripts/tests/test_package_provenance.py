import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from package_vendordep import validate_build_provenance


class PackageProvenanceTest(unittest.TestCase):
    def test_rejects_stale_libraries_even_when_platforms_agree(self):
        platform = {"baseVendordepVersion": "2026.2.5-dev.1", "baseCommit": "base",
                    "simulationCommit": "old", "baseWorkingTreeClean": True,
                    "simulationWorkingTreeClean": True}
        builds = {"linuxx86-64": platform, "windowsx86-64": platform}
        with self.assertRaisesRegex(ValueError, "matching source commits"):
            validate_build_provenance(builds, "new", True)
        validate_build_provenance(builds, "old", True)


if __name__ == "__main__":
    unittest.main()
