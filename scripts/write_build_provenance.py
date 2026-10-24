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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-root", type=Path, required=True)
    parser.add_argument("--platform", required=True)
    parser.add_argument("--output", type=Path, required=True)
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
    def checkout_state(repo: Path) -> tuple[bool, str]:
        status = output(["git", "status", "--porcelain"], repo)
        patch = output(["git", "diff", "HEAD", "--binary"], repo)
        return not bool(status), hashlib.sha256(patch.encode()).hexdigest()

    simulation_clean, simulation_patch_sha = checkout_state(root)
    base_clean, base_patch_sha = checkout_state(base)
    metadata = {
        "simulationCommit": os.environ.get("GITHUB_SHA", output(["git", "rev-parse", "HEAD"], root)),
        "baseCommit": base_commit,
        "baseVendordepVersion": descriptor["version"],
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
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
