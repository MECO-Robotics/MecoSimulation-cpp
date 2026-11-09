#!/usr/bin/env python3
"""Copy one immutable Simulation Maven coordinate into the raw Maven branch."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil


def copy_package(package: Path, destination: Path) -> int:
    package = package.resolve()
    destination = destination.resolve()
    descriptor = json.loads((package / "MecoSimulation.json").read_text(encoding="utf-8"))
    dependencies = descriptor.get("cppDependencies", [])
    if len(dependencies) != 1:
        raise ValueError("Expected exactly one Simulation Maven dependency")
    dependency = dependencies[0]
    group = dependency.get("groupId")
    artifact = dependency.get("artifactId")
    version = dependency.get("version")
    if (group, artifact, version) != (
        "com.mecorobotics.sim", "meco-simulation-wpilib-cpp", descriptor.get("version")
    ):
        raise ValueError("Simulation descriptor coordinates do not match the supported package")

    source_root = package / "maven" / "com/mecorobotics/sim" / artifact / version
    files = sorted(path for path in source_root.rglob("*") if path.is_file())
    required = {
        f"{artifact}-{version}.pom",
        f"{artifact}-{version}-headers.zip",
        f"{artifact}-{version}-sources.zip",
    }
    for platform in dependency.get("binaryPlatforms", []):
        required.add(f"{artifact}-{version}-{platform}static.zip")
        required.add(f"{artifact}-{version}-{platform}staticdebug.zip")
    if not files or {path.name for path in files} != required:
        raise ValueError("Simulation Maven coordinate is incomplete or contains unexpected files")

    copied = 0
    for source in files:
        relative = source.relative_to(package / "maven")
        if source.stat().st_size >= 100_000_000:
            raise ValueError(f"Raw GitHub cannot serve this Maven artifact reliably: {relative}")
        target = destination / relative
        if target.exists():
            if target.read_bytes() != source.read_bytes():
                raise ValueError(f"Published Maven artifact is immutable: {relative}")
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        copied += 1
    return copied


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--package", type=Path, required=True)
    parser.add_argument("--destination", type=Path, required=True)
    args = parser.parse_args()
    print(f"Copied or verified {copy_package(args.package, args.destination)} Simulation Maven artifacts")


if __name__ == "__main__":
    main()
