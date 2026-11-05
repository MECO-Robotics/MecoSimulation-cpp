#!/usr/bin/env python3
"""Create or unpack a locally assembled release package with checked contents."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTOR = 'MecoSimulation.json'
SOURCE = ROOT / 'vendordeps/MecoSimulation.json'
ARTIFACT = 'meco-simulation-wpilib-cpp'
GROUP = 'com/mecorobotics/sim'
PLATFORMS = ('linuxx86-64',)


def validate(directory):
    manifest = json.loads((directory / 'SHA256SUMS.json').read_text(encoding='utf-8'))
    descriptor = json.loads((directory / DESCRIPTOR).read_text(encoding='utf-8'))
    if descriptor != json.loads(SOURCE.read_text(encoding='utf-8')):
        raise ValueError('Package descriptor does not match the source checkout')
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    if manifest.get('sourceCommit') != commit:
        raise ValueError('Package was assembled from a different source commit')
    version = descriptor['version']
    dependency = next((item for item in descriptor.get('cppDependencies', []) if item.get('artifactId') == ARTIFACT), {})
    if set(dependency.get('binaryPlatforms', [])) != set(PLATFORMS):
        raise ValueError('Descriptor platforms do not match the local package')
    prefix = f'maven/{GROUP}/{ARTIFACT}/{version}/{ARTIFACT}-{version}'
    required = {prefix + '.pom', prefix + '-headers.zip', prefix + '-sources.zip'}
    required.update(f'{prefix}-{platform}{suffix}.zip' for platform in PLATFORMS
                    for suffix in ('static', 'staticdebug'))
    artifacts = manifest.get('artifacts', {})
    if not isinstance(artifacts, dict) or not required.issubset(artifacts):
        raise ValueError('Package manifest does not cover every required platform artifact')
    for name, digest in artifacts.items():
        path = PurePosixPath(name)
        if path.is_absolute() or not path.parts or any(p in ('.', '..') for p in path.parts):
            raise ValueError(f'Unsafe package path: {name!r}')
        file = directory.joinpath(*path.parts)
        if not file.is_file() or hashlib.sha256(file.read_bytes()).hexdigest() != digest:
            raise ValueError(f'Missing or corrupt package file: {name}')
    builds = manifest.get('buildProvenance', {})
    if (set(builds) != set(PLATFORMS) or
            any(item.get('buildOrigin') != 'local' or item.get('buildMode') != 'full-source-build'
                or item.get('simulationCommit') != commit or not item.get('simulationWorkingTreeClean')
                or not item.get('baseWorkingTreeClean') or not (item.get('coreBuild') or {}).get('releaseTestsPassed')
                or not (item.get('coreBuild') or {}).get('debugBuildPassed') for item in builds.values())):
        raise ValueError('Package must contain a full local Linux build at this commit')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('pack', 'unpack'))
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--bundle', type=Path, required=True)
    args = parser.parse_args()
    if args.operation == 'pack':
        if args.bundle.resolve().is_relative_to(args.package.resolve()):
            raise ValueError('Bundle must be outside the package directory')
        validate(args.package)
        args.bundle.parent.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(args.bundle, 'w', compression=zipfile.ZIP_DEFLATED) as output:
            for file in sorted(args.package.rglob('*')):
                if file.is_file():
                    output.write(file, file.relative_to(args.package).as_posix())
    else:
        if args.package.exists() and any(args.package.iterdir()):
            raise ValueError('Unpack destination must be empty')
        with zipfile.ZipFile(args.bundle) as archive:
            names = archive.namelist()
            if len(names) != len(set(names)):
                raise ValueError('Duplicate paths in package bundle')
            for name in names:
                path = PurePosixPath(name)
                if path.is_absolute() or not path.parts or any(p in ('.', '..') for p in path.parts) or '\\' in name:
                    raise ValueError(f'Unsafe bundle path: {name!r}')
            archive.extractall(args.package)
        validate(args.package)


if __name__ == '__main__':
    main()
