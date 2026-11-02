#!/usr/bin/env python3
"""Validate that an already-tested package represents the tagged source tree."""
import argparse
import json
import re
import subprocess
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from reuse_simulation_core import verify_retained_core_provenance

REQUIRED_JOBS = {
    'resolve-base', 'base-vendor-cache', 'build (ubuntu-24.04, linuxx86-64)',
    'build (windows-2022, windowsx86-64)', 'package',
    *{f'consumer-smoke ({os}, desktop, {config})'
      for os in ('ubuntu-24.04', 'windows-2022') for config in ('Debug', 'Release')},
}


def validate(run, jobs, manifest, tag_commit, built_commit, descriptor, tagged_descriptor, base_commit):
    if run.get('event') != 'pull_request' or run.get('status') != 'completed' or run.get('conclusion') != 'success':
        raise ValueError('Package source run is not a successful pull request run')
    successful = {j['name'] for j in jobs if j.get('status') == 'completed' and j.get('conclusion') == 'success'}
    if not REQUIRED_JOBS <= successful:
        raise ValueError('Package source run lacks required successful platform and consumer jobs')
    if descriptor != tagged_descriptor or descriptor.get('version') != manifest.get('version'):
        raise ValueError('Package descriptor differs from the release tag')
    source = manifest.get('sourceCommit')
    parents = [p['sha'] for p in tag_commit['parents']]
    if (source != built_commit['sha'] or len(parents) != 2 or run.get('head_sha') != parents[1]
            or [p['sha'] for p in built_commit['parents']] != parents
            or tag_commit['tree']['sha'] != built_commit['tree']['sha']):
        raise ValueError('Tested merge commit is not the same source tree and parents as the release tag')
    provenance = manifest.get('buildProvenance', {})
    if set(provenance) != {'linuxx86-64', 'windowsx86-64'}:
        raise ValueError('Package does not contain both platform builds')
    modes = {item.get('buildMode') for item in provenance.values()}
    if len(modes) != 1 or not modes <= {'reused-pinned-core', 'full-source-build'}:
        raise ValueError('Package does not contain one explicit supported core build mode')
    base_commits = set()
    for platform, item in provenance.items():
        if (item.get('simulationCommit') != source or not item.get('simulationWorkingTreeClean')
                or not item.get('baseWorkingTreeClean') or item.get('baseVendordepVersion') != '2026.2.12'):
            raise ValueError('Package provenance is not a clean tested build')
        base_commits.add(item.get('baseCommit'))
        if item['buildMode'] == 'reused-pinned-core':
            verify_retained_core_provenance(platform, item.get('retainedCore', {}))
            if item.get('coreBuild') is not None:
                raise ValueError('Reused-core provenance cannot claim a fresh CMake core build')
        else:
            core_build = item.get('coreBuild') or {}
            expected_trees = {
                path: subprocess.check_output(['git', 'rev-parse', f'HEAD:{path}'], text=True).strip()
                for path in ('CMakeLists.txt', 'include', 'src')
            }
            if (core_build.get('mode') != 'full-source-build'
                    or core_build.get('releaseTestsPassed') is not True
                    or core_build.get('debugBuildPassed') is not True
                    or item.get('retainedCore') is not None
                    or core_build.get('sourceTrees') != expected_trees):
                raise ValueError(f'{platform} full-source build provenance is incomplete')
    if base_commits != {base_commit}:
        raise ValueError('Platform builds do not use the pinned Base revision')


def api(path):
    return json.loads(subprocess.check_output(['gh', 'api', path], text=True))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True, type=int)
    parser.add_argument('--release-tag', required=True)
    parser.add_argument('--package', required=True, type=Path)
    parser.add_argument('--artifact-name', required=True)
    args = parser.parse_args()
    if not re.fullmatch(r'v2026\.\d+\.\d+', args.release_tag):
        raise ValueError('Invalid release tag')
    repo = 'repos/MECO-Robotics/MecoSimulation-cpp'
    tag_sha = subprocess.check_output(['git', 'rev-parse', args.release_tag + '^{}'], text=True).strip()
    manifest = json.loads((args.package / 'SHA256SUMS.json').read_text())
    if args.artifact_name != 'meco-simulation-vendordep-' + manifest['sourceCommit']:
        raise ValueError('Artifact name does not match its source commit')
    descriptor = json.loads((args.package / 'MecoSimulation.json').read_text())
    tagged_descriptor = json.loads(subprocess.check_output(['git', 'show', tag_sha + ':vendordeps/MecoSimulation.json'], text=True))
    run = api(f'{repo}/actions/runs/{args.run_id}')
    jobs = api(f'{repo}/actions/runs/{args.run_id}/jobs?filter=latest&per_page=100')['jobs']
    tag_commit = api(f'{repo}/git/commits/{tag_sha}')
    built_commit = api(f"{repo}/git/commits/{manifest['sourceCommit']}")
    base_commit = api('repos/MECO-Robotics/MecoBase-cpp/commits/v2026.2.12')['sha']
    validate(run, jobs, manifest, tag_commit, built_commit, descriptor, tagged_descriptor, base_commit)
    print(f"Validated run {args.run_id}, package {manifest['version']} and tag {tag_sha}")


if __name__ == '__main__':
    main()
