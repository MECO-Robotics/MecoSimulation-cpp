#!/usr/bin/env python3
"""Download and checksum the matching public Base Maven release artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import urllib.request


def fetch(url: str) -> bytes:
    with urllib.request.urlopen(url, timeout=30) as response:
        if response.status != 200:
            raise RuntimeError(f"HTTP {response.status}: {url}")
        return response.read()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-root", type=Path, required=True)
    parser.add_argument("--version", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path,
                        help="Pinned root SHA256SUMS.json snapshot for this Base release")
    parser.add_argument("--manifest-sha256", help="Required SHA-256 when --manifest is supplied")
    args = parser.parse_args()
    base_root = args.base_root.resolve()
    source_descriptor = base_root / "vendor/MecoBase.json"
    if not source_descriptor.is_file():
        source_descriptor = base_root / "vendordeps/MecoBase.json"
    descriptor_bytes = source_descriptor.read_bytes()
    descriptor = json.loads(descriptor_bytes)
    if descriptor.get("version") != args.version:
        raise ValueError(f"Expected Base {args.version}, got {descriptor.get('version')}")
    dependencies = [item for item in descriptor.get("cppDependencies", [])
                    if item.get("groupId") == "com.mecorobotics.frc" and
                    item.get("artifactId") == "meco-base-cpp"]
    if len(dependencies) != 1 or dependencies[0].get("version") != args.version:
        raise ValueError("Pinned Base source descriptor does not declare the expected split package")
    if not dependencies[0].get("binaryPlatforms"):
        raise ValueError("Pinned Base descriptor declares no binary platforms")

    site = "https://meco-robotics.github.io/MecoBase-cpp"
    default_snapshot = Path(__file__).with_name(f"base-{args.version}-SHA256SUMS.json")
    manifest_path = args.manifest or (default_snapshot if default_snapshot.is_file() else None)
    if manifest_path:
        expected_manifest_sha = args.manifest_sha256
        pin_file = manifest_path.with_suffix(manifest_path.suffix + ".sha256")
        if not expected_manifest_sha and pin_file.is_file():
            expected_manifest_sha = pin_file.read_text(encoding="utf-8").split()[0]
        if not expected_manifest_sha:
            raise ValueError("A pinned Base manifest snapshot requires --manifest-sha256")
        manifest_bytes = manifest_path.read_bytes()
        if hashlib.sha256(manifest_bytes).hexdigest() != expected_manifest_sha:
            raise ValueError("Pinned Base checksum manifest snapshot does not match its SHA-256 pin")
        published_descriptor = descriptor_bytes
    else:
        published_descriptor = fetch(f"{site}/MecoBase.json")
        if published_descriptor != descriptor_bytes:
            raise ValueError("Deployed Base descriptor does not match the pinned Base source")
        manifest_bytes = fetch(f"{site}/SHA256SUMS.json")
    checksums = json.loads(manifest_bytes)
    base_commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=base_root, text=True,
                                 capture_output=True, check=True).stdout.strip()
    if checksums.get("sourceCommit") != base_commit:
        raise ValueError("Published Base checksum manifest does not match the pinned Base source commit")
    if hashlib.sha256(descriptor_bytes).hexdigest() != checksums.get("artifacts", {}).get("MecoBase.json"):
        raise ValueError("Base descriptor checksum does not match the published Base manifest")
    artifacts = checksums.get("artifacts", {})
    if not artifacts:
        raise ValueError("Published Base checksum manifest contains no artifacts")

    coordinate = f"com/mecorobotics/frc/meco-base-cpp/{args.version}"
    artifact_name = f"meco-base-cpp-{args.version}"
    required = [f"{coordinate}/{artifact_name}.pom"]
    required.extend(f"{coordinate}/{artifact_name}-{platform}{configuration}.zip"
                    for platform in dependencies[0].get("binaryPlatforms", ())
                    for configuration in ("static", "staticdebug"))
    required.extend(f"{coordinate}/{artifact_name}-{kind}.zip" for kind in ("headers", "sources"))
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "MecoBase.json").write_bytes(published_descriptor)
    for relative in required:
        manifest_path = f"maven/{relative}"
        expected = artifacts.get(manifest_path)
        if not expected:
            raise FileNotFoundError(f"Published Base manifest lacks required artifact: {manifest_path}")
        content = fetch(f"{site}/{manifest_path}")
        actual = hashlib.sha256(content).hexdigest()
        if actual != expected:
            raise ValueError(f"Published Base artifact checksum mismatch: {manifest_path}")
        target = args.output / "maven" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(content)
    print(f"Downloaded and verified {len(required)} published Base {args.version} artifacts")


if __name__ == "__main__":
    main()
