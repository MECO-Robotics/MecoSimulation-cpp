import sys
import unittest
import json
import hashlib
import subprocess
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from package_vendordep import validate_build_provenance


def retained_core(platform):
    pinned = json.loads((Path(__file__).resolve().parents[1] / "fast_sim_core.json").read_text())
    members = {"Core.o": "0" * 64}
    member_hash = hashlib.sha256(json.dumps(members, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    return {
        configuration: {
            "platform": platform,
            "configuration": configuration,
            "sourceTag": pinned["sourceTag"],
            "sourceCommit": pinned["sourceCommit"],
            "baseVendordepVersion": pinned["baseVendordepVersion"],
            "baseCommit": pinned["baseCommit"],
            "manifestSha256": pinned["manifestSha256"],
            "packageArchiveSha256": pinned["archives"][platform][configuration],
            "coreSourceTrees": pinned["coreSourceTrees"],
            "retainedMemberCount": 1,
            "retainedMembers": members,
            "retainedMembersSha256": member_hash,
        }
        for configuration in ("Debug", "Release")
    }


class PackageProvenanceTest(unittest.TestCase):
    def test_rejects_stale_libraries_even_when_platforms_agree(self):
        platform = {"baseVendordepVersion": "2026.2.17-beta.1", "baseCommit": "base",
                    "simulationCommit": "old", "baseWorkingTreeClean": True,
                    "simulationWorkingTreeClean": True, "buildMode": "reused-pinned-core",
                    "coreBuild": None}
        builds = {name: {**platform, "retainedCore": retained_core(name)}
                  for name in ("linuxx86-64", "windowsx86-64")}
        with self.assertRaisesRegex(ValueError, "matching source commits"):
            validate_build_provenance(builds, "new", True)
        validate_build_provenance(builds, "old", True)

    def test_accepts_full_core_build_only_with_matching_source_trees_and_checks(self):
        root = Path(__file__).resolve().parents[2]
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
        trees = {path: subprocess.check_output(["git", "rev-parse", f"HEAD:{path}"], cwd=root, text=True).strip()
                 for path in ("CMakeLists.txt", "include", "src", "tests")}
        record = {
            "baseVendordepVersion": "2026.2.17-beta.1",
            "baseCommit": "base",
            "simulationCommit": commit,
            "baseWorkingTreeClean": True,
            "simulationWorkingTreeClean": True,
            "buildMode": "full-source-build",
            "retainedCore": None,
            "coreBuild": {"mode": "full-source-build", "sourceTrees": trees,
                          "releaseTestsPassed": True, "debugBuildPassed": True},
        }
        validate_build_provenance({"linuxx86-64": record, "windowsx86-64": record}, commit, True)
        record["coreBuild"]["releaseTestsPassed"] = False
        with self.assertRaisesRegex(ValueError, "provenance is incomplete"):
            validate_build_provenance({"linuxx86-64": record, "windowsx86-64": record}, commit, True)


if __name__ == "__main__":
    unittest.main()
