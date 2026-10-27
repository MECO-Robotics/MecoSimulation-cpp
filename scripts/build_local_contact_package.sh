#!/usr/bin/env bash
# Linux-only development package. This does not publish or qualify a release.
set -euo pipefail
root="$(cd -- "$(dirname -- "$0")/.." && pwd)"
export JAVA_HOME="${JAVA_HOME:-$HOME/wpilib/2026/jdk}"
export PATH="$JAVA_HOME/bin:$PATH"
for configuration in Release Debug; do
    native="$root/build/Debug"
    if [[ "$configuration" == Release ]]; then native="$root/build/contact-release"; fi
    cmake -S "$root" -B "$native" -DCMAKE_BUILD_TYPE="$configuration"
    cmake --build "$native" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
done
"$root/adapters/wpilib-2026/gradlew" -p "$root/adapters/wpilib-2026" \
    mecoSimAdapterReleaseStaticLibrary mecoSimAdapterDebugStaticLibrary --max-workers=2 --console=plain
for configuration in Release Debug; do
    native="$root/build/Debug"
    if [[ "$configuration" == Release ]]; then native="$root/build/contact-release"; fi
    destination="$root/build/contact-stage/linuxx86-64/$configuration/lib"
    mkdir -p -- "$destination"
    cp -- "$native/lib/libfrc_jolt.a" "$native/lib/libJolt.a" "$destination/"
    cp -- "$root/adapters/wpilib-2026/build/libs/mecoSimAdapter/static/${configuration,,}/libmecoSimAdapter.a" "$destination/"
done
python3 "$root/scripts/write_build_provenance.py" --base-root "$root/../MecoBase-cpp" \
    --platform linuxx86-64 --output "$root/build/contact-stage/linuxx86-64/provenance.json"
python3 "$root/scripts/package_vendordep.py" --build-root "$root/build/contact-stage" \
    --output "$root/build/contact-package" --allow-missing-platform windowsx86-64
python3 "$root/scripts/verify_vendordep.py" --maven-root "$root/build/contact-package/maven" --platform linuxx86-64
