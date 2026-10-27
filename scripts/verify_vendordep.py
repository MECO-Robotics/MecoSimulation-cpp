#!/usr/bin/env python3
"""Check that the simulation descriptor resolves to the expected Maven files."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import zipfile

from simulation_headers import validate_simulation_header_names

VERSION = "2026.0.12"
COORDINATE = Path("com/mecorobotics/sim/meco-simulation-wpilib-cpp") / VERSION
CLASSIFIERS = ("linuxx86-64", "windowsx86-64")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--maven-root", type=Path, required=True)
    parser.add_argument("--platform", action="append", choices=CLASSIFIERS,
                        help="Platforms to verify (default: every published platform)")
    args = parser.parse_args()
    platforms = tuple(args.platform or CLASSIFIERS)
    root = args.maven_root.resolve()
    descriptor_path = root.parent / "MecoSimulation.json"
    if not descriptor_path.is_file():
        descriptor_path = root / "MecoSimulation.json"
    descriptor = json.loads(descriptor_path.read_text(encoding="utf-8"))
    if descriptor.get("version") != VERSION:
        raise SystemExit("Vendordep version does not match the 2026 package")
    dependencies = descriptor.get("cppDependencies", [])
    if len(dependencies) != 1:
        raise SystemExit("Expected exactly one combined Simulation dependency")
    dep = dependencies[0]
    if (dep.get("groupId"), dep.get("artifactId"), dep.get("version"), dep.get("libName")) != (
        "com.mecorobotics.sim", "meco-simulation-wpilib-cpp", VERSION, "mecoSimAdapter"
    ):
        raise SystemExit("Vendordep coordinates or library name differ from the packaged adapter")
    artifact_dir = root / COORDINATE
    required = [artifact_dir / f"meco-simulation-wpilib-cpp-{VERSION}.pom"]
    required.extend(artifact_dir / f"meco-simulation-wpilib-cpp-{VERSION}-{classifier}{suffix}.zip"
                    for classifier in platforms for suffix in ("static", "staticdebug"))
    required.extend(artifact_dir / f"meco-simulation-wpilib-cpp-{VERSION}-{kind}.zip"
                    for kind in ("headers", "sources"))
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise SystemExit("Missing Maven package files:\n" + "\n".join(missing))
    expected_lib = {"linuxx86-64": "libmecoSimAdapter.a", "windowsx86-64": "mecoSimAdapter.lib"}
    for classifier in platforms:
        for suffix in ("static", "staticdebug"):
            path = artifact_dir / f"meco-simulation-wpilib-cpp-{VERSION}-{classifier}{suffix}.zip"
            with zipfile.ZipFile(path) as archive:
                libraries = [name for name in archive.namelist() if name.endswith(expected_lib[classifier])]
                if len(libraries) != 1:
                    raise SystemExit(f"Expected one {expected_lib[classifier]} in {path}")
    with zipfile.ZipFile(artifact_dir / f"meco-simulation-wpilib-cpp-{VERSION}-headers.zip") as archive:
        try:
            validate_simulation_header_names(archive.namelist())
        except ValueError as error:
            raise SystemExit(str(error)) from error
    manifest_path = root.parent / "SHA256SUMS.json"
    if not manifest_path.is_file():
        manifest_path = root / "SHA256SUMS.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for relative_path, expected_digest in manifest.get("artifacts", {}).items():
        artifact = root.parent / relative_path
        if not artifact.is_file():
            artifact = root / relative_path.removeprefix("maven/")
        if not artifact.is_file():
            raise SystemExit(f"Checksum manifest names a missing file: {relative_path}")
        actual = hashlib.sha256(artifact.read_bytes()).hexdigest()
        if actual != expected_digest:
            raise SystemExit(f"Checksum mismatch for {artifact}")
    provenance = manifest.get("buildProvenance", {})
    if not args.platform and (set(provenance) != set(CLASSIFIERS) or
                              any(item.get("baseVendordepVersion") != "2026.2.1" for item in provenance.values())):
        raise SystemExit("Full package lacks both platform builds or split Base provenance")
    print(f"Verified {len(required)} Maven files for {VERSION}")


if __name__ == "__main__":
    main()
