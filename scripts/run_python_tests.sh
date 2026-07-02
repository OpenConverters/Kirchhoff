#!/bin/bash
# Runner for the Python-side integration tests (currently tests/test_regulate.py — the closed-loop
# regulation smoke test across the duty / phase-shift / frequency control modalities). These are separate
# from the C++ Catch2 binaries: they need the PyKirchhoff pybind module built and libngspice, and each case
# runs ~20 ngspice sims, so they are slower and live in their own runner rather than the C++ loop.
#
# It (re)builds PyKirchhoff into ./build (the path tests/test_regulate.py adds to sys.path), then runs
# pytest. Pass extra args straight through to pytest, e.g.:
#   scripts/run_python_tests.sh -k frequency -v
#
# Usage:  scripts/run_python_tests.sh [pytest args...]
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"

# Configure build/ if needed (PyKirchhoff + libngspice runner both ON — the regulation loop drives ngspice).
if [ ! -f "$BUILD/build.ninja" ] && [ ! -f "$BUILD/Makefile" ]; then
    echo "=== configuring $BUILD (PyKirchhoff + ngspice) ==="
    cmake -S "$ROOT" -B "$BUILD" -DKIRCHHOFF_BUILD_PYBIND=ON -DENABLE_NGSPICE=ON
fi

echo "=== building PyKirchhoff ==="
cmake --build "$BUILD" --target PyKirchhoff -j

echo "=== pytest tests/test_regulate.py ==="
cd "$ROOT"
exec python3 -m pytest tests/test_regulate.py "$@"
