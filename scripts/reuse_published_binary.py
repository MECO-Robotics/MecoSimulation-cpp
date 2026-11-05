#!/usr/bin/env python3
"""Reuse immutable 2026.0.20 binaries only for proven header-only changes."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
TAG = 'v2026.0.20'
TAG_COMMIT = 'acb04377f1d877e4aac0527fc729a7ed7e822908'
SOURCE_COMMIT = 'b43631a1f2136802ad17a23e7753a60392362f97'
BASE_COMMIT = '660eb3112d2edecb2d34efc5f0e19a75110212f1'
MANIFEST_SHA = '411b8f617cbe3925f917b6ab5a93c42013f7edc9362e3548070326f9bc2e1e56'
ARCHIVES = {
    'linuxx86-64': {'Release': 'e6e0de037206063773b319d9de3a41fa2dbdeb14a5c3b408ab626e765e29ea27', 'Debug': 'b46c704edd518772c6be899492579db2d4f26fb4aac86c5e2e35af31a6981e84'},
    'windowsx86-64': {'Release': 'd0bcd4a438772bee9d085b5f2b586f9f6d1ffbafb45218afa66bcac7d7ee5525', 'Debug': '1054fe1ab4664ed4d31cc8d18e16bd97b8bdeb62fc56157491e6f0f20e26f03f'},
}
ALLOWED = {
    'README.md', '.gitattributes', 'vendordeps/MecoSimulation.json',
    'adapters/wpilib-2026/include/meco/sim/DriveTuning.h',
    'adapters/wpilib-2026/include/meco/sim/MotorSimulationFactories.h',
    'scripts/consumer_smoke.py', 'scripts/package_vendordep.py', 'scripts/simulation_headers.py',
    'scripts/verify_vendordep.py', 'scripts/tests/test_simulation_headers.py',
    'scripts/reuse_published_binary.py', 'scripts/published_2026_0_20_manifest.json',
    'scripts/tests/test_reuse_published_binary.py', '.github/workflows/simulation.yml',
    '.github/release/validate_tested_release.py', '.github/workflows/publish-tested-package.yml',
}
CHANGED_HEADERS = {'DriveTuning.h', 'MotorSimulationFactories.h'}

def output(*args: str) -> str:
    return subprocess.run(['git', *args], cwd=ROOT, check=True, text=True, capture_output=True).stdout.strip()

def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()

def verify_source() -> None:
    if output('rev-parse', TAG + '^{commit}') != TAG_COMMIT:
        raise ValueError('Published reuse tag does not match immutable pin')
    changed = set(output('diff', '--name-only', TAG, 'HEAD').splitlines())
    if not changed <= ALLOWED:
        raise ValueError(f'Unapproved binary build input changed: {sorted(changed - ALLOWED)}')
    # Existing definitions are not rewritten. Only independent inline additions
    # to the two unreferenced headers can bypass a fresh native build.
    for name in CHANGED_HEADERS:
        path = 'adapters/wpilib-2026/include/meco/sim/' + name
        diff = output('diff', '--unified=0', TAG, 'HEAD', '--', path)
        if any(line.startswith('-') and not line.startswith('---') for line in diff.splitlines()):
            raise ValueError(f'Existing native header lines were removed or modified: {path}')
    # Reject even indirect native inclusion of the new/modified headers, and
    # reject macro-generated includes that cannot be resolved by this check.
    pattern = re.compile(r'^\s*#\s*include\s*[<"][^">]*(DriveTuning|MotorSimulationFactories)\.h[">]')
    include_directive = re.compile(r'^\s*#\s*include\s+(\S+)')
    for name in output('ls-files', 'adapters/wpilib-2026/src', 'adapters/wpilib-2026/include', 'include', 'src').splitlines():
        path = ROOT / name
        if path.name in CHANGED_HEADERS or path.suffix not in {'.h', '.hpp', '.cpp', '.cc'}:
            continue
        for line in path.read_text(encoding='utf-8').splitlines():
            include = include_directive.match(line)
            if pattern.match(line) or (include and not include.group(1).startswith(('<', '"'))):
                raise ValueError(f'Changed header or unresolved include in native build closure: {name}')

def manifest() -> dict:
    data = (ROOT / 'scripts/published_2026_0_20_manifest.json').read_bytes()
    if digest(data) != MANIFEST_SHA:
        raise ValueError('Published binary manifest digest differs from immutable pin')
    value = json.loads(data)
    if value.get('version') != '2026.0.20' or value.get('sourceCommit') != SOURCE_COMMIT:
        raise ValueError('Published binary manifest source identity mismatch')
    return value

def archive_path(platform: str, configuration: str) -> str:
    suffix = 'static' if configuration == 'Release' else 'staticdebug'
    name = f'meco-simulation-wpilib-cpp-2026.0.20-{platform}{suffix}.zip'
    return f'maven/com/mecorobotics/sim/meco-simulation-wpilib-cpp/2026.0.20/{name}'

def verify_provenance(platform: str, record: dict) -> None:
    original = manifest()['buildProvenance'][platform]
    if (record.get('buildMode') != 'reused-published-binary' or
        record.get('binarySourceTag') != TAG or record.get('binaryTagCommit') != TAG_COMMIT or
        record.get('binarySourceCommit') != SOURCE_COMMIT or record.get('binaryManifestSha256') != MANIFEST_SHA or
        record.get('binaryArchiveSha256') != ARCHIVES[platform] or record.get('originalBuildProvenance') != original or
        original.get('simulationCommit') != SOURCE_COMMIT or original.get('baseCommit') != BASE_COMMIT or
        original.get('baseVendordepVersion') != '2026.2.12' or
        not original.get('baseWorkingTreeClean') or not original.get('simulationWorkingTreeClean')):
        raise ValueError(f'{platform} published-binary provenance mismatch')
    verify_source()

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-source', action='store_true')
    parser.add_argument('--platform', choices=tuple(ARCHIVES))
    parser.add_argument('--base-root', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    verify_source(); source = manifest()
    if args.check_source:
        print('Published binary compile inputs are unchanged'); return
    if not args.platform or not args.base_root or not args.output:
        parser.error('platform, base-root and output are required')
    base = args.base_root.resolve()
    base_commit = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=base, check=True, text=True, capture_output=True).stdout.strip()
    if base_commit != BASE_COMMIT:
        raise ValueError('Published binary Base commit differs from the selected pinned Base')
    for configuration, expected in ARCHIVES[args.platform].items():
        relative = archive_path(args.platform, configuration)
        if source['artifacts'].get(relative) != expected:
            raise ValueError('Archive digest differs from pinned manifest')
        with urllib.request.urlopen('https://meco-robotics.github.io/MecoSimulation-cpp/' + relative, timeout=15) as response:
            data = response.read()
        if digest(data) != expected: raise ValueError('Published archive checksum mismatch')
        target = args.output / configuration / 'published.zip'
        target.parent.mkdir(parents=True, exist_ok=True); target.write_bytes(data)
    from write_build_provenance import checkout_state
    sim_clean, sim_patch = checkout_state(ROOT, ['build', 'adapters/wpilib-2026/build', 'adapters/wpilib-2026/.gradle', 'MecoBase-cpp'])
    base_clean, base_patch = checkout_state(base, ['build', '.gradle'])
    record = {'simulationCommit': output('rev-parse', 'HEAD'), 'baseCommit': base_commit,
              'baseVendordepVersion': '2026.2.12', 'baseWorkingTreeClean': base_clean,
              'simulationWorkingTreeClean': sim_clean, 'baseTrackedPatchSha256': base_patch,
              'simulationTrackedPatchSha256': sim_patch, 'platform': args.platform,
              'buildMode': 'reused-published-binary', 'binarySourceTag': TAG, 'binaryTagCommit': TAG_COMMIT,
              'binarySourceCommit': SOURCE_COMMIT, 'binaryManifestSha256': MANIFEST_SHA,
              'binaryArchiveSha256': ARCHIVES[args.platform], 'originalBuildProvenance': source['buildProvenance'][args.platform]}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'provenance.json').write_text(json.dumps(record, indent=2, sort_keys=True) + '\n')

if __name__ == '__main__': main()
