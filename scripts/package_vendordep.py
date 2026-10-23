#!/usr/bin/env python3
"""Assemble deterministic Maven artifacts for the 2026 simulation vendordep."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import zipfile

VERSION = "2026.0.1"
GROUP_PATH = Path("com/mecorobotics/sim")
PLATFORMS = {
    "linuxx86-64": ("linux/x86-64", "libmecoSimAdapter.a", ("libfrc_jolt.a", "libJolt.a")),
    "windowsx86-64": ("windows/x86-64", "mecoSimAdapter.lib", ("frc_jolt.lib", "Jolt.lib")),
}
EPOCH = (1980, 1, 1, 0, 0, 0)


def add_file(archive: zipfile.ZipFile, source: Path, path: str) -> None:
    info = zipfile.ZipInfo(path, EPOCH)
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = 0o100644 << 16
    archive.writestr(info, source.read_bytes())


def combine_archives(inputs: tuple[Path, ...], output: Path, windows: bool) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if windows:
        tool = shutil.which("llvm-lib")
        if not tool:
            versioned = sorted(Path("/usr/bin").glob("llvm-lib-[0-9]*"), reverse=True)
            tool = str(versioned[0]) if versioned else None
        if not tool:
            raise RuntimeError("llvm-lib (or a versioned llvm-lib binary) is required to combine Windows static libraries")
        subprocess.run([tool, f"/OUT:{output}", *(str(path) for path in inputs)], check=True)
        return
    script = [f"create {output}"]
    script.extend(f"addlib {path}" for path in inputs)
    script.extend(("save", "end", ""))
    subprocess.run(["ar", "-M"], input="\n".join(script), text=True, check=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--allow-missing-platform", action="append", choices=tuple(PLATFORMS), default=[])
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    if output.exists():
        shutil.rmtree(output)
    maven = output / "maven"
    adapter_dir = maven / GROUP_PATH / "meco-simulation-wpilib-cpp" / VERSION
    for directory in (adapter_dir,):
        directory.mkdir(parents=True, exist_ok=True)
        artifact = directory.parent.name
        (directory / f"{artifact}-{VERSION}.pom").write_text(
            f'<project xmlns="http://maven.apache.org/POM/4.0.0"><modelVersion>4.0.0</modelVersion>'
            f'<groupId>com.mecorobotics.sim</groupId><artifactId>{artifact}</artifactId>'
            f'<version>{VERSION}</version><packaging>pom</packaging></project>\n',
            encoding="utf-8",
        )

    for classifier, (platform, adapter_lib, physics_libs) in PLATFORMS.items():
        for config, suffix in (("Release", "static"), ("Debug", "staticdebug")):
            input_archives = tuple(args.build_root / classifier / config / "lib" / name
                                   for name in (adapter_lib, *physics_libs))
            if not all(path.is_file() for path in input_archives):
                if classifier in args.allow_missing_platform:
                    continue
                missing = next(path for path in input_archives if not path.is_file())
                raise FileNotFoundError(f"Required archive is missing: {missing}")
            combined = output / "combined" / classifier / config / adapter_lib
            combine_archives(input_archives, combined, classifier.startswith("windows"))
            target = adapter_dir / f"meco-simulation-wpilib-cpp-{VERSION}-{classifier}{suffix}.zip"
            with zipfile.ZipFile(target, "w") as archive:
                add_file(archive, combined, f"{platform}/static/{adapter_lib}")
                add_file(archive, root / "LICENSE.md", "LICENSE.md")
                add_file(archive, root / "Jolt-LICENSE", "Jolt-LICENSE")
    combined_root = output / "combined"
    if combined_root.exists():
        shutil.rmtree(combined_root)

    for directory, artifact in ((adapter_dir, "meco-simulation-wpilib-cpp"),):
        with zipfile.ZipFile(directory / f"{artifact}-{VERSION}-headers.zip", "w") as archive:
            for base in (root / "include", root / "adapters/wpilib-2026/include"):
                for path in sorted(base.rglob("*.h")):
                    add_file(archive, path, path.relative_to(base).as_posix())
            add_file(archive, root / "LICENSE.md", "LICENSE.md")
            add_file(archive, root / "Jolt-LICENSE", "Jolt-LICENSE")

    with zipfile.ZipFile(adapter_dir / f"meco-simulation-wpilib-cpp-{VERSION}-sources.zip", "w") as archive:
        for base in (root / "adapters/wpilib-2026", root / "src", root / "include"):
            for path in sorted(item for item in base.rglob("*") if item.is_file() and "build" not in item.parts and ".gradle" not in item.parts and "gradle/wrapper/gradle-wrapper.jar" not in item.as_posix()):
                add_file(archive, path, path.relative_to(root).as_posix())
        for path in (root / "CMakeLists.txt", root / "Jolt-LICENSE", root / "README.md"):
            if path.is_file():
                add_file(archive, path, path.relative_to(root).as_posix())

    (output / "MecoSimulation.json").write_bytes((root / "vendordeps/MecoSimulation.json").read_bytes())
    (output / "LICENSE.md").write_bytes((root / "LICENSE.md").read_bytes())
    descriptor = json.loads((output / "MecoSimulation.json").read_text(encoding="utf-8"))
    if descriptor.get("version") != VERSION:
        raise ValueError("Vendordep version does not match package version")
    dependencies = descriptor.get("cppDependencies", [])
    if len(dependencies) != 1 or dependencies[0].get("libName") != "mecoSimAdapter":
        raise ValueError("Vendordep must expose the combined adapter and physics archive")
    if dependencies[0].get("artifactId") != "meco-simulation-wpilib-cpp" or dependencies[0].get("version") != VERSION:
        raise ValueError("Vendordep dependency coordinates do not match emitted artifacts")
    for classifier in PLATFORMS:
        for suffix in ("static", "staticdebug"):
            archive = adapter_dir / f"meco-simulation-wpilib-cpp-{VERSION}-{classifier}{suffix}.zip"
            if not archive.is_file():
                if classifier in args.allow_missing_platform:
                    continue
                raise FileNotFoundError(f"Descriptor package artifact is missing: {archive}")
            with zipfile.ZipFile(archive) as package:
                libraries = [name for name in package.namelist()
                             if re.fullmatch(r"(?:linux/x86-64|windows/x86-64)/static/[^/]+", name)]
                if len(libraries) != 1 or Path(libraries[0]).name != PLATFORMS[classifier][1]:
                    raise ValueError(f"Expected one combined native library in {archive}")
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, text=True, capture_output=True, check=True).stdout.strip()
    build_provenance = {}
    for classifier in PLATFORMS:
        path = args.build_root / classifier / "provenance.json"
        if path.is_file():
            build_provenance[classifier] = json.loads(path.read_text(encoding="utf-8"))
        elif not args.allow_missing_platform:
            raise FileNotFoundError(f"Required build provenance is missing: {path}")
    if build_provenance:
        expected_base = {item.get("baseVendordepVersion") for item in build_provenance.values()}
        expected_commit = {item.get("baseCommit") for item in build_provenance.values()}
        expected_sim_commit = {item.get("simulationCommit") for item in build_provenance.values()}
        if (len(expected_base) != 1 or expected_base != {"2026.2.0"} or
                len(expected_commit) != 1 or len(expected_sim_commit) != 1):
            raise ValueError("Platform builds do not share the pinned split Base version and commit")
        if not args.allow_missing_platform and any(
            not item.get("baseWorkingTreeClean") or not item.get("simulationWorkingTreeClean")
            for item in build_provenance.values()
        ):
            raise ValueError("Release platform builds must come from clean Simulation and Base checkouts")
    manifest = {
        "version": VERSION,
        "sourceCommit": os.environ.get("GITHUB_SHA", commit),
        "cmakeVersion": subprocess.run(["cmake", "--version"], text=True, capture_output=True, check=True).stdout.splitlines()[0],
        "buildProvenance": build_provenance,
        "artifacts": {},
    }
    for path in sorted(item for item in maven.rglob("*") if item.is_file()):
        manifest["artifacts"][path.relative_to(output).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    (output / "SHA256SUMS.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
