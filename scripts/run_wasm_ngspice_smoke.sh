#!/bin/bash
# End-to-end proof that the in-process ngspice runner works in WebAssembly: compile
# tests/wasm_ngspice_smoke.cpp + src/NgspiceRunner.cpp with emcc, link the WASM libngspice built by
# scripts/build_ngspice_wasm.sh, and run it under node. Expected output: "RC v(out) = 10.0 V".
# Validated 2026-06-30 (emscripten 3.1.51, node v24, ngspice 45.2): v(out)=10.000000 V, exit 0.
#
# Usage:  scripts/run_wasm_ngspice_smoke.sh [ngspice_wasm_install_dir]
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
NG="${1:-$ROOT/build-wasm/ngspice/install}"
EMSDK_ENV="${EMSDK_ENV:-/home/alf/emsdk/emsdk_env.sh}"
OUT="$ROOT/build-wasm/smoke"

LIB="$(find "$NG" -name 'libngspice.so.0' | head -1)"
[ -n "$LIB" ] || { echo "WASM libngspice not found under $NG — run scripts/build_ngspice_wasm.sh first" >&2; exit 1; }
# shellcheck disable=SC1090
source "$EMSDK_ENV" >/dev/null 2>&1
mkdir -p "$OUT"; cd "$OUT"

echo "=== emcc compile (NgspiceRunner.cpp + smoke main + WASM libngspice) ==="
emcc -std=c++17 -DENABLE_NGSPICE -O2 -fwasm-exceptions -sSUPPORT_LONGJMP=wasm \
  -I "$ROOT/src" -I "$NG/include" \
  "$ROOT/tests/wasm_ngspice_smoke.cpp" "$ROOT/src/NgspiceRunner.cpp" "$LIB" \
  -sALLOW_MEMORY_GROWTH -sEXIT_RUNTIME=1 \
  -o smoke.js

echo "=== run under node ==="
node smoke.js
