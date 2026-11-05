#!/usr/bin/env python3
"""Resolve every published Simulation artifact and compare deployed checksums."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time
import urllib.error
import urllib.request


def fetch(url: str, attempts: int, delay: float) -> bytes:
    last_error: Exception | None = None
    for attempt in range(attempts):
        try:
            with urllib.request.urlopen(url, timeout=20) as response:
                if response.status != 200:
                    raise RuntimeError(f"HTTP {response.status}: {url}")
                return response.read()
        except (urllib.error.URLError, TimeoutError, RuntimeError) as error:
            last_error = error
            if attempt + 1 < attempts:
                time.sleep(delay)
    raise RuntimeError(f"Could not fetch deployed file {url}: {last_error}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--site-url", required=True, help="GitHub Pages root, with or without trailing slash")
    parser.add_argument("--expected-package", type=Path, help="Compare deployment against an already validated tested package")
    parser.add_argument("--attempts", type=int, default=12)
    parser.add_argument("--delay", type=float, default=10)
    args = parser.parse_args()
    site = args.site_url.rstrip("/")
    descriptor_bytes = fetch(f"{site}/MecoSimulation.json", args.attempts, args.delay)
    descriptor = json.loads(descriptor_bytes)
    if args.expected_package:
        package = args.expected_package.resolve()
        if (package / "MecoSimulation.json").read_bytes() != descriptor_bytes:
            raise SystemExit("Deployed descriptor differs from the tested package")
    source_descriptor = fetch(descriptor["jsonUrl"], args.attempts, args.delay)
    if json.loads(source_descriptor) != descriptor:
        raise SystemExit("The raw vendordep descriptor differs from the deployed Pages descriptor")
    checksum_bytes = fetch(f"{site}/SHA256SUMS.json", args.attempts, args.delay)
    checksums = json.loads(checksum_bytes)
    maven_url = descriptor["mavenUrls"][0].rstrip("/")
    expected = checksums.get("artifacts", {})
    if args.expected_package:
        tested = json.loads((package / "SHA256SUMS.json").read_text(encoding="utf-8"))
        if (checksums.get("version") != tested.get("version") or
                checksums.get("sourceCommit") != tested.get("sourceCommit")):
            raise SystemExit("Deployed version or source differs from the tested package")
        tested_artifacts = tested.get("artifacts", {})
        if not tested_artifacts:
            raise SystemExit("Tested package has no Maven artifacts")
        for path, digest in tested_artifacts.items():
            if not path.startswith("maven/") or expected.get(path) != digest:
                raise SystemExit(f"Deployed artifact does not match the tested package: {path}")
    if not expected:
        raise SystemExit("Published checksum manifest contains no Maven artifacts")
    for relative_path, digest in expected.items():
        if not relative_path.startswith("maven/"):
            continue
        artifact_url = f"{site}/{relative_path}"
        contents = fetch(artifact_url, args.attempts, args.delay)
        actual = hashlib.sha256(contents).hexdigest()
        if actual != digest:
            raise SystemExit(f"Checksum mismatch for {artifact_url}")
    dependency = descriptor["cppDependencies"][0]
    group_path = dependency["groupId"].replace(".", "/")
    coordinate_path = f"{group_path}/{dependency['artifactId']}/{dependency['version']}"
    pom_url = f"{maven_url}/{coordinate_path}/{dependency['artifactId']}-{dependency['version']}.pom"
    fetch(pom_url, args.attempts, args.delay)
    print(f"Verified deployed {dependency['artifactId']}:{dependency['version']} and {len(expected)} checksummed Maven files")


if __name__ == "__main__":
    main()
