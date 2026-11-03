#!/usr/bin/env python3
"""Reuse the checksummed 2026.0.16 Jolt/core objects and replace adapter objects."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import urllib.request
import zipfile

SITE = "https://meco-robotics.github.io/MecoSimulation-cpp"
ADAPTER_OBJECTS = {
    "DriveSimulationAdapter",
    "DriveHardwareSimulation",
    "PracticeMatchController",
    "LoadedMotorIO",
}
PINNED_ADAPTER_OBJECTS = {
    "DriveSimulationAdapter",
    "DriveHardwareSimulation",
    "PracticeMatchController",
}
FRC_JOLT_OBJECTS = {
    "SimulationWorld", "RobotContactConfig", "DrivePhysics", "Arena", "Rebuilt2026"
}
GROUP_PATH = "com/mecorobotics/sim/meco-simulation-wpilib-cpp"


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def fetch(url: str) -> bytes:
    with urllib.request.urlopen(url, timeout=30) as response:
        if response.status != 200:
            raise RuntimeError(f"HTTP {response.status}: {url}")
        return response.read()


def git_output(root: Path, *args: str) -> str:
    return subprocess.run(["git", *args], cwd=root, text=True, capture_output=True, check=True).stdout.strip()


def core_source_matches(root: Path, pinned: dict) -> bool:
    try:
        if git_output(root, "rev-parse", f"{pinned['sourceTag']}^{{commit}}") != pinned["tagCommit"]:
            return False
    except subprocess.CalledProcessError:
        return False
    for path, tree in pinned["coreSourceTrees"].items():
        try:
            baseline = git_output(root, "rev-parse", f"{pinned['sourceTag']}:{path}")
            current = git_output(root, "rev-parse", f"HEAD:{path}")
            status = git_output(root, "status", "--porcelain", "--", path)
        except subprocess.CalledProcessError:
            return False
        if baseline != tree or current != tree or status:
            return False
    return True


def verify_source(root: Path, pinned: dict) -> None:
    if not core_source_matches(root, pinned):
        raise ValueError(f"Core source differs from the immutable {pinned['sourceTag']} reuse baseline")


def archive_tool() -> str:
    configured = os.environ.get("LLVM_AR")
    if configured and Path(configured).is_file():
        return configured
    for name in ("llvm-ar", "llvm-ar.exe", "ar", "ar.exe"):
        found = shutil.which(name)
        if found:
            return found
    if os.name == "nt":
        for candidate in (
            Path(os.environ.get("ProgramFiles", r"C:\Program Files")) / "LLVM/bin/llvm-ar.exe",
            Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) / "LLVM/bin/llvm-ar.exe",
        ):
            if candidate.is_file():
                return str(candidate)
    raise RuntimeError("Could not find llvm-ar or ar; set LLVM_AR to the platform archive tool")


def members(tool: str, library: Path) -> list[str]:
    result = subprocess.run([tool, "t", str(library)], text=True, capture_output=True, check=True)
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def member_stem(name: str) -> str:
    basename = member_basename(name)
    return basename[:-6] if basename.endswith(".cpp.o") else basename.rsplit(".", 1)[0]


def member_basename(name: str) -> str:
    return Path(name.replace("\\", "/")).name


def verify_retained_core_provenance(platform: str, configurations: dict) -> None:
    pinned = json.loads(Path(__file__).with_name("fast_sim_core.json").read_text(encoding="utf-8"))
    if set(configurations) != {"Debug", "Release"}:
        raise ValueError(f"{platform} provenance must include retained Debug and Release cores")
    expected_trees = pinned["coreSourceTrees"]
    member_sets = []
    for config in ("Debug", "Release"):
        record = configurations[config]
        if (record.get("platform") != platform or record.get("configuration") != config or
                record.get("sourceTag") != pinned["sourceTag"] or
                record.get("sourceCommit") != pinned["sourceCommit"] or
                record.get("baseVendordepVersion") != pinned["baseVendordepVersion"] or
                record.get("baseCommit") != pinned["baseCommit"] or
                record.get("manifestSha256") != pinned["manifestSha256"] or
                record.get("packageArchiveSha256") != pinned["archives"][platform][config] or
                record.get("coreSourceTrees") != expected_trees):
            raise ValueError(f"{platform} {config} retained-core identity does not match the immutable pin")
        members = record.get("retainedMembers")
        if not isinstance(members, dict) or record.get("retainedMemberCount") != len(members) or not members:
            raise ValueError(f"{platform} {config} retained-core member inventory is incomplete")
        member_hash = sha256(json.dumps(members, sort_keys=True, separators=(",", ":")).encode())
        if member_hash != record.get("retainedMembersSha256"):
            raise ValueError(f"{platform} {config} retained-core member inventory checksum mismatch")
        member_sets.append(set(members))
    if {member_basename(name) for name in member_sets[0]} != {member_basename(name) for name in member_sets[1]}:
        raise ValueError(f"{platform} retained Debug and Release core member inventories differ")


def extract(tool: str, library: Path, destination: Path) -> dict[str, Path]:
    destination.mkdir(parents=True, exist_ok=True)
    names = members(tool, library)
    if len(names) != len(set(names)):
        raise ValueError(f"Archive has duplicate object member names: {library}")
    basenames = [member_basename(name) for name in names]
    if len(basenames) != len(set(basenames)):
        raise ValueError(f"Archive has ambiguous duplicate object basenames: {library}")
    extracted = {}
    for name in names:
        result = subprocess.run([tool, "p", str(library), name], capture_output=True, check=True)
        path = destination / member_basename(name)
        path.write_bytes(result.stdout)
        extracted[name] = path
    if sorted(path.name for path in destination.iterdir() if path.is_file()) != sorted(basenames):
        raise ValueError(f"Archive extraction did not preserve the complete member list: {library}")
    return extracted


def create_archive(tool: str, destination: Path, objects: list[Path], work: Path) -> dict[str, str]:
    if not objects:
        raise ValueError(f"Refusing to create an empty static archive: {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    script = [f"create {destination.as_posix()}"]
    script.extend(f"addmod {obj.resolve().as_posix()}" for obj in objects)
    script.extend(("save", "end", ""))
    subprocess.run([tool, "-M"], input="\n".join(script), cwd=work, text=True, check=True)
    names = members(tool, destination)
    basenames = [member_basename(name) for name in names]
    if len(names) != len(set(names)) or set(basenames) != {obj.name for obj in objects}:
        raise ValueError(f"Created archive member set differs from expected objects: {destination}")
    check_dir = work / f"verify-{destination.stem}"
    extracted = extract(tool, destination, check_dir)
    hashes = {member_basename(name): sha256(path.read_bytes()) for name, path in extracted.items()}
    expected = {obj.name: sha256(obj.read_bytes()) for obj in objects}
    if hashes != expected:
        raise ValueError(f"Created archive object bytes differ from input objects: {destination}")
    return hashes


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-source", action="store_true", help="Exit 0 only when core trees exactly match the pinned reuse baseline")
    parser.add_argument("--platform", choices=("linuxx86-64", "windowsx86-64"))
    parser.add_argument("--configuration", choices=("Release", "Debug"))
    parser.add_argument("--adapter-archive", type=Path)
    parser.add_argument("--output", type=Path, help="Directory receiving the three separate static libraries")
    parser.add_argument("--work-dir", type=Path)
    parser.add_argument("--core-manifest", type=Path, default=Path(__file__).with_name("fast_sim_core.json"))
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    pinned = json.loads(args.core_manifest.read_text(encoding="utf-8"))
    if args.check_source:
        if core_source_matches(root, pinned):
            print("core trees match the pinned reuse baseline")
            return
        print("core trees differ from the pinned reuse baseline")
        raise SystemExit(1)
    missing = [name for name in ("platform", "configuration", "adapter_archive", "output", "work_dir")
               if getattr(args, name) is None]
    if missing:
        parser.error("missing required options for reuse: " + ", ".join("--" + name.replace("_", "-") for name in missing))
    verify_source(root, pinned)
    tool = archive_tool()
    args.work_dir = args.work_dir.resolve()
    args.output = args.output.resolve()
    if args.work_dir.exists():
        shutil.rmtree(args.work_dir)
    args.work_dir.mkdir(parents=True)
    args.output.mkdir(parents=True, exist_ok=True)

    manifest_path = Path(__file__).with_name("fast_sim_core_manifest.json")
    manifest_bytes = manifest_path.read_bytes()
    if sha256(manifest_bytes) != pinned["manifestSha256"]:
        raise ValueError("Published v2026.0.16 core checksum manifest does not match the in-repository pin")
    manifest = json.loads(manifest_bytes)
    if manifest.get("version") != pinned["version"] or manifest.get("sourceCommit") != pinned["sourceCommit"]:
        raise ValueError("Published core manifest version/source does not match the in-repository pin")
    core_provenance = manifest.get("buildProvenance", {}).get(args.platform)
    if not core_provenance or any(core_provenance.get(field) is not True for field in
                                  ("baseWorkingTreeClean", "simulationWorkingTreeClean")):
        raise ValueError("Published retained-core provenance is missing or not clean")
    if (core_provenance.get("simulationCommit") != pinned["sourceCommit"] or
            core_provenance.get("baseVendordepVersion") != pinned["baseVendordepVersion"] or
            core_provenance.get("baseCommit") != pinned["baseCommit"]):
        raise ValueError("Published retained-core build provenance differs from the pinned Base/source")

    suffix = "static" if args.configuration == "Release" else "staticdebug"
    artifact_name = f"meco-simulation-wpilib-cpp-{pinned['version']}-{args.platform}{suffix}.zip"
    artifact_path = f"maven/{GROUP_PATH}/{pinned['version']}/{artifact_name}"
    archive_sha = pinned["archives"][args.platform][args.configuration]
    if manifest.get("artifacts", {}).get(artifact_path) != archive_sha:
        raise ValueError(f"Pinned archive hash is absent from the published manifest: {artifact_path}")
    archive_bytes = fetch(f"{SITE}/{artifact_path}")
    if sha256(archive_bytes) != archive_sha:
        raise ValueError(f"Retained-core package checksum mismatch: {artifact_path}")
    downloaded_zip = args.work_dir / artifact_name
    downloaded_zip.write_bytes(archive_bytes)

    library_name = "libmecoSimAdapter.a" if args.platform == "linuxx86-64" else "mecoSimAdapter.lib"
    library_entry = f"{'linux' if args.platform == 'linuxx86-64' else 'windows'}/x86-64/static/{library_name}"
    old_library = args.work_dir / library_name
    with zipfile.ZipFile(downloaded_zip) as package:
        if package.namelist().count(library_entry) != 1:
            raise ValueError(f"Expected one retained native library at {library_entry}")
        old_library.write_bytes(package.read(library_entry))

    adapter_archive = args.adapter_archive.resolve()
    if not adapter_archive.is_file():
        raise FileNotFoundError(f"Fresh adapter archive is missing: {adapter_archive}")
    adapter_dir, old_dir = args.work_dir / "new-adapter", args.work_dir / "old-combined"
    adapter_entries = extract(tool, adapter_archive, adapter_dir)
    old_entries = extract(tool, old_library, old_dir)
    adapter_by_stem = {member_stem(name): name for name in adapter_entries}
    if set(adapter_by_stem) != ADAPTER_OBJECTS:
        raise ValueError(f"Fresh adapter archive must contain exactly {sorted(ADAPTER_OBJECTS)}, got {sorted(adapter_by_stem)}")
    old_members = list(old_entries)
    old_adapter = [name for name in old_members if member_stem(name) in PINNED_ADAPTER_OBJECTS]
    if {member_stem(name) for name in old_adapter} != PINNED_ADAPTER_OBJECTS:
        raise ValueError("Retained archive does not contain the complete pinned adapter object set")
    if {member_stem(name) for name in old_members if member_stem(name) in FRC_JOLT_OBJECTS} != FRC_JOLT_OBJECTS:
        raise ValueError("Retained archive does not contain the complete pinned frc_jolt object set")
    if not PINNED_ADAPTER_OBJECTS.issubset(adapter_by_stem):
        raise ValueError("Fresh adapter archive is missing an existing replaceable adapter object")

    fresh_adapter_dir = args.work_dir / "fresh-adapter-objects"
    fresh_adapter_dir.mkdir()
    fresh_adapter_entries = extract(tool, adapter_archive, fresh_adapter_dir)
    adapter_objects = [fresh_adapter_entries[name] for name in sorted(fresh_adapter_entries)]
    frc_objects = [old_entries[name] for name in sorted(old_members) if member_stem(name) in FRC_JOLT_OBJECTS]
    jolt_objects = [old_entries[name] for name in sorted(old_members)
                    if member_stem(name) not in ADAPTER_OBJECTS | FRC_JOLT_OBJECTS]
    retained_hashes = {name: sha256(old_entries[name].read_bytes()) for name in old_members
                       if member_stem(name) not in ADAPTER_OBJECTS}
    if not jolt_objects:
        raise ValueError("Retained archive has no Jolt objects")

    extension = ".a" if args.platform == "linuxx86-64" else ".lib"
    outputs = {
        "adapter": args.output / ("libmecoSimAdapter.a" if extension == ".a" else "mecoSimAdapter.lib"),
        "frcJolt": args.output / ("libfrc_jolt.a" if extension == ".a" else "frc_jolt.lib"),
        "jolt": args.output / ("libJolt.a" if extension == ".a" else "Jolt.lib"),
    }
    new_adapter_hashes = create_archive(tool, outputs["adapter"], adapter_objects, args.work_dir)
    frc_hashes = create_archive(tool, outputs["frcJolt"], frc_objects, args.work_dir)
    jolt_hashes = create_archive(tool, outputs["jolt"], jolt_objects, args.work_dir)
    output_core_hashes = {**frc_hashes, **jolt_hashes}
    normalized_retained = {member_basename(name): digest for name, digest in retained_hashes.items()}
    if output_core_hashes != normalized_retained:
        raise ValueError("Repacked Jolt/core object hashes differ from the verified v2026.0.16 archive")

    member_manifest = json.dumps(retained_hashes, sort_keys=True, separators=(",", ":")).encode()
    provenance = {
        "sourceTag": pinned["sourceTag"],
        "sourceCommit": pinned["sourceCommit"],
        "baseVendordepVersion": pinned["baseVendordepVersion"],
        "baseCommit": pinned["baseCommit"],
        "manifestSha256": pinned["manifestSha256"],
        "packageArchiveSha256": archive_sha,
        "combinedLibrarySha256": sha256(old_library.read_bytes()),
        "retainedMemberCount": len(retained_hashes),
        "retainedMembersSha256": sha256(member_manifest),
        "retainedMembers": retained_hashes,
        "replacedAdapterMembers": new_adapter_hashes,
        "frcJoltArchiveMembersSha256": sha256(json.dumps(frc_hashes, sort_keys=True, separators=(",", ":")).encode()),
        "joltArchiveMembersSha256": sha256(json.dumps(jolt_hashes, sort_keys=True, separators=(",", ":")).encode()),
        "configuration": args.configuration,
        "platform": args.platform,
        "coreSourceTrees": pinned["coreSourceTrees"],
    }
    (args.output.parent / f"retained-core-{args.configuration.lower()}.json").write_text(
        json.dumps(provenance, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
