#!/usr/bin/env python3
"""Require versioned release tags to match the vendordep descriptor."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def validate(git_ref: str, descriptor: Path) -> None:
    if not git_ref.startswith("refs/tags/"):
        return
    version = json.loads(descriptor.read_text(encoding="utf-8"))["version"]
    expected = f"refs/tags/v{version}"
    if git_ref != expected:
        raise ValueError(f"Release tag {git_ref!r} does not match vendordep version {version!r}; expected {expected!r}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--git-ref", required=True)
    parser.add_argument("--descriptor", type=Path, default=Path("vendordeps/MecoSimulation.json"))
    args = parser.parse_args()
    validate(args.git_ref, args.descriptor)
    print(f"Verified release ref {args.git_ref}")


if __name__ == "__main__":
    main()
