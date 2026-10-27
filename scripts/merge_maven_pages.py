#!/usr/bin/env python3
"""Build a Pages tree that retains prior immutable Maven artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import urllib.error
import urllib.request


def merge_pages(new_root: Path, output_root: Path, fetch_existing) -> None:
    if output_root.exists():
        shutil.rmtree(output_root)
    shutil.copytree(new_root, output_root)
    current_manifest_path = output_root / "SHA256SUMS.json"
    current_manifest = json.loads(current_manifest_path.read_text(encoding="utf-8"))
    merged = dict(current_manifest.get("artifacts", {}))
    previous_manifest_bytes = fetch_existing("SHA256SUMS.json")
    if previous_manifest_bytes is not None:
        previous_manifest = json.loads(previous_manifest_bytes)
        for relative_path, digest in previous_manifest.get("artifacts", {}).items():
            if not relative_path.startswith("maven/"):
                continue
            target = output_root / relative_path
            if relative_path in merged:
                if merged[relative_path] != digest:
                    raise ValueError(f"Refusing to replace immutable Maven artifact {relative_path}")
                continue
            contents = fetch_existing(relative_path)
            if contents is None or hashlib.sha256(contents).hexdigest() != digest:
                raise ValueError(f"Existing Maven artifact is missing or corrupt: {relative_path}")
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(contents)
            merged[relative_path] = digest
    current_manifest["artifacts"] = dict(sorted(merged.items()))
    current_manifest_path.write_text(json.dumps(current_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--new-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--existing-site-url", required=True)
    args = parser.parse_args()
    site = args.existing_site_url.rstrip("/")

    def fetch_existing(relative_path: str) -> bytes | None:
        request = urllib.request.Request(f"{site}/{relative_path}")
        try:
            with urllib.request.urlopen(request, timeout=20) as response:
                if response.status != 200:
                    raise RuntimeError(f"HTTP {response.status}: {relative_path}")
                return response.read()
        except urllib.error.HTTPError as error:
            if error.code == 404:
                return None
            raise

    merge_pages(args.new_root.resolve(), args.output.resolve(), fetch_existing)
    print(f"Prepared Pages tree with {len(json.loads((args.output / 'SHA256SUMS.json').read_text())['artifacts'])} Maven artifacts")


if __name__ == "__main__":
    main()
