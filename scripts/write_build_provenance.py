#!/usr/bin/env python3
"""Write the pinned toolchain and Base revision for one native artifact build."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


def output(command: list[str], cwd: Path | None = None) -> str:
    result = subprocess.run(command, cwd=cwd, text=True, capture_output=True, check=True)
    return (result.stdout or result.stderr).strip()


def checkout_state(repo: Path, generated_paths: list[str]) -> tuple[bool, str]:
    # Inspect source, excluding generated output. Selecting only generated paths
    # would incorrectly attest that modified source was a clean build.
    paths = [".", *(f":(exclude){path}" for path in generated_paths)]
    status = output(["git", "status", "--porcelain", "--untracked-files=normal", "--", *paths], repo)
    patch = output(["git", "diff", "HEAD", "--binary", "--", *paths], repo)
    return not bool(status), hashlib.sha256(patch.encode()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-root", type=Path, required=True)
    parser.add_argument("--platform", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--build-mode", choices=("reused-pinned-core", "full-source-build"), required=True)
    parser.add_argument("--retained-core", type=Path,
                        help="Directory containing verified retained-core release/debug provenance")
    parser.add_argument("--release-tests-passed", action="store_true")
    parser.add_argument("--debug-build-passed", action="store_true")
    parser.add_argument("--origin", choices=("local",), help="Mark artifacts built by the local release helper")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    base = args.base_root.resolve()
    descriptor = json.loads((base / "vendor/MecoBase.json").read_text(encoding="utf-8"))
    gradle_deps = json.loads((root / "adapters/wpilib-2026/vendordeps/MecoSimulationBuildDeps.json").read_text(encoding="utf-8"))
    build_gradle = (root / "adapters/wpilib-2026/build.gradle").read_text(encoding="utf-8")
    plugin = re.search(r"GradleRIO' version '([^']+)'", build_gradle)
    wrapper = (root / "adapters/wpilib-2026/gradle/wrapper/gradle-wrapper.properties").read_text(encoding="utf-8")
    gradle = re.search(r"gradle-([0-9.]+)-(?:bin|all)\.zip", wrapper)
    if not plugin or not gradle:
        raise ValueError("Could not determine pinned GradleRIO or Gradle wrapper version")
    base_commit = output(["git", "rev-parse", "HEAD"], base)
    simulation_clean, simulation_patch_sha = checkout_state(
        root, ["build", "adapters/wpilib-2026/build", "adapters/wpilib-2026/.gradle", "MecoBase-cpp"])
    base_clean, base_patch_sha = checkout_state(base, ["build", ".gradle"])
    retained_core = None
    core_build = None
    if args.build_mode == "reused-pinned-core":
        if args.retained_core is None:
            raise ValueError("Reused-core provenance requires --retained-core")
        retained_core = {
            configuration: json.loads((args.retained_core / configuration / f"retained-core-{configuration.lower()}.json").read_text(encoding="utf-8"))
            for configuration in ("Debug", "Release")
        }
    else:
        if args.retained_core is not None:
            raise ValueError("Full-source provenance cannot include retained-core records")
        core_build = {
            "mode": "full-source-build",
            "sourceTrees": {path: output(["git", "rev-parse", f"HEAD:{path}"], root)
                            for path in ("CMakeLists.txt", "include", "src", "tests")},
            "releaseTestsPassed": args.release_tests_passed,
            "debugBuildPassed": args.debug_build_passed,
        }
    metadata = {
        "simulationCommit": os.environ.get("GITHUB_SHA", output(["git", "rev-parse", "HEAD"], root)),
        "baseCommit": base_commit,
        "baseVendordepVersion": descriptor["version"],
        "buildMode": args.build_mode,
        "baseWorkingTreeClean": base_clean,
        "baseTrackedPatchSha256": base_patch_sha,
        "simulationWorkingTreeClean": simulation_clean,
        "simulationTrackedPatchSha256": simulation_patch_sha,
        "platform": args.platform,
        "wpilibYear": "2026",
        "gradleRioVersion": plugin.group(1),
        "gradleVersion": gradle.group(1),
        "javaRuntime": output(["java", "-version"]).splitlines()[0],
        "cmakeVersion": output(["cmake", "--version"]).splitlines()[0],
        "vendorDependencies": sorted(
            (dep.get("groupId", ""), dep.get("artifactId", ""), dep.get("version", ""))
            for dep in gradle_deps.get("cppDependencies", [])
        ),
        "retainedCore": retained_core,
        "coreBuild": core_build,
    }
    if args.origin:
        metadata["buildOrigin"] = args.origin
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
