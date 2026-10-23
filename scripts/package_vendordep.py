#!/usr/bin/env python3
"""Assemble deterministic Maven artifacts for the 2026 simulation vendordep."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import zipfile

VERSION = "2026.0.1"
GROUP_PATH = Path("com/mecorobotics/sim")
PLATFORMS = {
    "linuxx86-64": ("linux/x86-64", "libfrc_jolt.a", "libJolt.a"),
    "windowsx86-64": ("windows/x86-64", "frc_jolt.lib", "Jolt.lib"),
}
EPOCH = (1980, 1, 1, 0, 0, 0)


def add_file(archive: zipfile.ZipFile, source: Path, path: str) -> None:
    info = zipfile.ZipInfo(path, EPOCH)
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = 0o100644 << 16
    archive.writestr(info, source.read_bytes())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    maven = output / "maven"
    sim_dir = maven / GROUP_PATH / "meco-simulation-cpp" / VERSION
    jolt_dir = maven / GROUP_PATH / "jolt-cpp" / VERSION
    for directory in (sim_dir, jolt_dir):
        directory.mkdir(parents=True, exist_ok=True)
        artifact = directory.parent.name
        (directory / f"{artifact}-{VERSION}.pom").write_text(
            f'<project xmlns="http://maven.apache.org/POM/4.0.0"><modelVersion>4.0.0</modelVersion>'
            f'<groupId>com.mecorobotics.sim</groupId><artifactId>{artifact}</artifactId>'
            f'<version>{VERSION}</version><packaging>pom</packaging></project>\n',
            encoding="utf-8",
        )

    for classifier, (platform, sim_lib, jolt_lib) in PLATFORMS.items():
        for config, suffix in (("Release", "static"), ("Debug", "staticdebug")):
            for directory, artifact, libname, filename in (
                (sim_dir, "meco-simulation-cpp", "frc_jolt", sim_lib),
                (jolt_dir, "jolt-cpp", "Jolt", jolt_lib),
            ):
                source = args.build_root / classifier / config / "lib" / filename
                if not source.is_file():
                    raise FileNotFoundError(f"Required archive is missing: {source}")
                target = directory / f"{artifact}-{VERSION}-{classifier}{suffix}.zip"
                with zipfile.ZipFile(target, "w") as archive:
                    add_file(archive, source, f"{platform}/static/{filename}")
                    add_file(archive, root / "LICENSE.md", "LICENSE.md")

    for directory, artifact in ((sim_dir, "meco-simulation-cpp"), (jolt_dir, "jolt-cpp")):
        with zipfile.ZipFile(directory / f"{artifact}-{VERSION}-headers.zip", "w") as archive:
            for path in sorted((root / "include").rglob("*.h")):
                add_file(archive, path, path.relative_to(root / "include").as_posix())
            add_file(archive, root / "LICENSE.md", "LICENSE.md")

    with zipfile.ZipFile(sim_dir / f"meco-simulation-cpp-{VERSION}-sources.zip", "w") as archive:
        for base in (root / "src", root / "include", root / "tests"):
            for path in sorted(item for item in base.rglob("*") if item.is_file()):
                add_file(archive, path, path.relative_to(root).as_posix())
        for path in (root / "CMakeLists.txt", root / "Jolt-LICENSE", root / "README.md"):
            if path.is_file():
                add_file(archive, path, path.relative_to(root).as_posix())

    (output / "MecoSimulation.json").write_bytes((root / "vendordeps/MecoSimulation.json").read_bytes())
    (output / "LICENSE.md").write_bytes((root / "LICENSE.md").read_bytes())
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, text=True, capture_output=True, check=True).stdout.strip()
    manifest = {
        "version": VERSION,
        "sourceCommit": os.environ.get("GITHUB_SHA", commit),
        "cmakeVersion": subprocess.run(["cmake", "--version"], text=True, capture_output=True, check=True).stdout.splitlines()[0],
        "artifacts": {},
    }
    for path in sorted(item for item in maven.rglob("*") if item.is_file()):
        manifest["artifacts"][path.relative_to(output).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    (output / "SHA256SUMS.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
