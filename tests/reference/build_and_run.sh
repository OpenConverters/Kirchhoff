#!/usr/bin/env bash
# Regenerate the MKF golden reference fixtures (boost.mkf.json, flyback.mkf.json).
#
# Requires MKF built at $MKF_ROOT/build (libMKF.so present). Run from anywhere:
#   bash tests/reference/build_and_run.sh
set -euo pipefail

MKF_ROOT="${MKF_ROOT:-/home/alf/OpenMagnetics/MKF}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="$MKF_ROOT/build"

if [[ ! -f "$BUILD/libMKF.so" ]]; then
    echo "ERROR: $BUILD/libMKF.so not found. Build MKF first:" >&2
    echo "  cmake -S $MKF_ROOT -B $BUILD -G Ninja -DCMAKE_BUILD_TYPE=Release && ninja -C $BUILD -j4" >&2
    exit 1
fi

INCLUDES=(
    -I"$BUILD/_deps/json-src/include/nlohmann" -I"$BUILD/_deps/json-src/include"
    -I"$BUILD/_deps/magic-enum-src/include/magic_enum" -I"$BUILD/_deps/spline-src/src"
    -I"$BUILD/_deps/svg-src/src" -I"$BUILD/_deps/eigen-src"
    -I"$BUILD/MAS" -I"$BUILD/CAS" -I"$BUILD/generated"
    -I"$MKF_ROOT/src" -I"$MKF_ROOT/src/advisers" -I"$MKF_ROOT/src/constructive_models"
    -I"$MKF_ROOT/src/converter_models" -I"$MKF_ROOT/src/physical_models"
    -I"$MKF_ROOT/src/processors" -I"$MKF_ROOT/src/support"
    -I"$BUILD/_cmrc/include" -I"$BUILD/_deps/rapidfuzz-src/rapidfuzz/.."
    -I"$MKF_ROOT/tests"
)

echo "compiling gen_mkf_reference..."
c++ -std=gnu++23 -O2 "${INCLUDES[@]}" \
    "$HERE/gen_mkf_reference.cpp" \
    -o "$HERE/gen_mkf_reference" \
    -L"$BUILD" -lMKF -Wl,-rpath,"$BUILD"

echo "running..."
"$HERE/gen_mkf_reference" "$HERE"
echo "done."
