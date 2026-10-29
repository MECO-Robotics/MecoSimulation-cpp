#!/usr/bin/env python3
"""Add the matching split Base Maven artifacts to a local consumer repository."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil


def merge(base_root: Path, simulation_root: Path, expected_version: str = "2026.2.5-dev.1") -> None:
    base_descriptor = json.loads((base_root / "MecoBase.json").read_text(encoding="utf-8"))
    if base_descriptor.get("version") != expected_version:
        raise ValueError(f"Expected Base descriptor {expected_version}, got {base_descriptor.get('version')}")
    coordinates = [dependency for dependency in base_descriptor.get("cppDependencies", [])
                   if dependency.get("groupId") == "com.mecorobotics.frc" and
                   dependency.get("artifactId") == "meco-base-cpp"]
    if len(coordinates) != 1 or coordinates[0].get("version") != expected_version:
        raise ValueError("Base descriptor does not declare the matching split Maven coordinate")
    base_maven = base_root / "maven"
    if not base_maven.is_dir() or not any(base_maven.rglob("*.pom")):
        raise FileNotFoundError(f"Base Maven artifacts are missing under {base_maven}")
    base_coordinate = base_maven / "com/mecorobotics/frc/meco-base-cpp" / expected_version
    required = [base_coordinate / f"meco-base-cpp-{expected_version}.pom"]
    required.extend(base_coordinate / f"meco-base-cpp-{expected_version}-{platform}{suffix}.zip"
                    for platform in ("linuxathena", "linuxx86-64", "windowsx86-64")
                    for suffix in ("static", "staticdebug"))
    required.extend(base_coordinate / f"meco-base-cpp-{expected_version}-{kind}.zip"
                    for kind in ("headers", "sources"))
    missing = [path for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError("Base package is incomplete:\n" + "\n".join(str(path) for path in missing))

    simulation_maven = simulation_root / "maven"
    simulation_maven.mkdir(parents=True, exist_ok=True)
    for source in sorted(path for path in base_maven.rglob("*") if path.is_file()):
        relative = source.relative_to(base_maven)
        target = simulation_maven / relative
        if target.exists() and target.read_bytes() != source.read_bytes():
            raise ValueError(f"Conflicting Maven artifact: {relative}")
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists():
            shutil.copy2(source, target)

    manifest_path = simulation_root / "SHA256SUMS.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["artifacts"] = {
        f"maven/{path.relative_to(simulation_maven).as_posix()}": hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(simulation_maven.rglob("*")) if path.is_file()
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulation-root", type=Path, required=True)
    parser.add_argument("--base-root", type=Path, required=True)
    args = parser.parse_args()
    merge(args.base_root.resolve(), args.simulation_root.resolve())
    print("Merged matching Base Maven artifacts and refreshed the combined checksum manifest")


if __name__ == "__main__":
    main()
