"""Collect and validate headers owned by the Simulation vendordep."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable


REQUIRED_SIMULATION_HEADERS = {
    "base/sim/Arena.h",
    "base/sim/DrivePhysics.h",
    "base/sim/RobotContactConfig.h",
    "base/sim/SimulationWorld.h",
    "base/sim/SimulationTuning.h",
    "base/sim/FootprintClearance.h",
    "meco/sim/DriveSimulationAdapter.h",
    "meco/sim/BatterySupply.h",
    "meco/sim/SimulationPose.h",
    "meco/sim/SimulatedVision.h",
    "meco/sim/IdealVisionObservation.h",
    "meco/sim/ParticleContacts.h",
    "meco/sim/ParticleFieldContacts.h",
    "meco/sim/SimulatedPieceSensor.h",
    "meco/sim/PracticeMatchController.h",
    "meco/sim/SimulationDiagnostics.h",
    "base/sim/fields/Rebuilt2026.h",
}


def collect_simulation_headers(root: Path) -> list[tuple[Path, str]]:
    """Return only Simulation-owned public headers, with archive-relative names."""
    entries: dict[str, Path] = {}
    for base in (root / "include", root / "adapters/wpilib-2026/include"):
        for path in base.rglob("*.h"):
            relative = path.relative_to(base).as_posix()
            if relative in entries:
                raise ValueError(f"Simulation header is exported from two roots: {relative}")
            entries[relative] = path
    validate_simulation_header_names(entries)
    return [(entries[name], name) for name in sorted(entries)]


def validate_simulation_header_names(names: Iterable[str]) -> None:
    header_names = set(names)
    missing = sorted(REQUIRED_SIMULATION_HEADERS - header_names)
    if missing:
        raise ValueError("Simulation headers package is missing required public headers: " +
                         ", ".join(missing))

    # The Base vendordep is a separate required package. Bundling its headers
    # here lets a stale Simulation archive shadow the matching Base API.
    unexpected_base = sorted(name for name in header_names
                             if name.startswith("base/") and not name.startswith("base/sim/"))
    if unexpected_base:
        raise ValueError("Simulation headers package contains Base-owned headers: " +
                         ", ".join(unexpected_base))
