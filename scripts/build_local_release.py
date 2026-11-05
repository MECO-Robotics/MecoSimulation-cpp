#!/usr/bin/env python3
"""Build and assemble the Linux desktop vendordep locally."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
os.chdir(ROOT)


def run(*args):
    print('+', *map(str, args), flush=True)
    subprocess.run([str(arg) for arg in args], check=True)


def require_clean(repo):
    status = subprocess.check_output(['git', '-C', str(repo), 'status', '--porcelain',
                                      '--untracked-files=normal'], text=True)
    if status.strip():
        raise SystemExit(f'Release builds require a clean committed checkout: {repo}')


def one_file(directory, name):
    matches = list(directory.rglob(name))
    if len(matches) != 1:
        raise SystemExit(f'Expected exactly one {name} under {directory}; found {len(matches)}')
    return matches[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('build', 'assemble'))
    parser.add_argument('--base-root', type=Path, required=True)
    args = parser.parse_args()
    base = args.base_root.resolve()
    require_clean(ROOT)
    require_clean(base)
    if not sys.platform.startswith('linux'):
        raise SystemExit('The release tool requires a Linux host')
    platform = 'linuxx86-64'
    stage = ROOT / 'build/local-release'
    if args.operation == 'assemble':
        run(sys.executable, 'scripts/package_vendordep.py', '--build-root', stage,
            '--output', ROOT / 'build/vendor-maven')
        run(sys.executable, 'scripts/verify_vendordep.py', '--maven-root', ROOT / 'build/vendor-maven/maven')
        print('Complete local package: build/vendor-maven')
        return
    run(sys.executable, 'scripts/verify_base_compatibility.py', '--descriptor', base / 'vendor/MecoBase.json',
        '--drive-header', base / 'src/main/include/base/Drive.h')
    gradle = ROOT / 'adapters/wpilib-2026/gradlew'
    run(gradle, '-p', ROOT / 'adapters/wpilib-2026', 'mecoSimAdapterReleaseStaticLibrary',
        'mecoSimAdapterDebugStaticLibrary', f'-PmecoBaseRoot={base}', '--max-workers=2', '--console=plain')
    names = ('libmecoSimAdapter.a', 'libfrc_jolt.a', 'libJolt.a')
    for config in ('Release', 'Debug'):
        build = ROOT / 'build' / f'local-core-{config}'
        run('cmake', '-S', ROOT, '-B', build, f'-DCMAKE_BUILD_TYPE={config}',
            f'-DBUILD_TESTING={"ON" if config == "Release" else "OFF"}')
        run('cmake', '--build', build, '--config', config, '--parallel', '2')
        if config == 'Release':
            run('ctest', '--test-dir', build, '-C', config, '--output-on-failure', '--no-tests=error')
        out = stage / platform / config / 'lib'
        out.mkdir(parents=True, exist_ok=True)
        adapter = ROOT / 'adapters/wpilib-2026/build/libs/mecoSimAdapter/static' / config.lower()
        for name, folder in zip(names, (adapter, build, build)):
            shutil.copy2(one_file(folder, name), out / name)
    run(sys.executable, 'scripts/write_build_provenance.py', '--base-root', base,
        '--platform', platform, '--build-mode', 'full-source-build', '--origin', 'local', '--release-tests-passed',
        '--debug-build-passed', '--output', stage / platform / 'provenance.json')
    print(f'{platform} staged in {stage / platform}')


if __name__ == '__main__':
    main()
