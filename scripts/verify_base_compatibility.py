#!/usr/bin/env python3
"""Require the matching split Base release before building the adapter."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def validate(descriptor_path: Path, drive_header: Path, expected_version: str) -> None:
    descriptor = json.loads(descriptor_path.read_text(encoding="utf-8"))
    if descriptor.get("version") != expected_version:
        raise ValueError(
            f"MecoBase descriptor is {descriptor.get('version')!r}; expected the split release {expected_version}"
        )
    dependencies = descriptor.get("cppDependencies", [])
    base = [dep for dep in dependencies
            if dep.get("groupId") == "com.mecorobotics.frc" and dep.get("artifactId") == "meco-base-cpp"]
    if len(base) != 1 or base[0].get("version") != expected_version:
        raise ValueError("MecoBase descriptor does not resolve the expected meco-base-cpp coordinate")
    jolt = [dep for dep in dependencies if "jolt" in " ".join(
        str(dep.get(key, "")) for key in ("groupId", "artifactId", "libName")
    ).lower()]
    if jolt:
        coords = ", ".join(f"{dep.get('artifactId')}:{dep.get('libName')}" for dep in jolt)
        raise ValueError(f"Base still declares Jolt dependencies ({coords}); use the split Base release")
    header = drive_header.read_text(encoding="utf-8")
    if "class DriveSimulationAdapter" not in header or "SetSimulationAdapter" not in header:
        raise ValueError(f"{drive_header} lacks the optional Simulation adapter API")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--descriptor", type=Path, required=True)
    parser.add_argument("--drive-header", type=Path, required=True)
    parser.add_argument("--expected-version", default="2026.2.1")
    args = parser.parse_args()
    validate(args.descriptor, args.drive_header, args.expected_version)
    print(f"Verified split MecoBase {args.expected_version} integration boundary")


if __name__ == "__main__":
    main()
