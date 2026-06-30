#!/bin/bash
# Build ngspice as a WebAssembly libngspice (the P5 in-browser-run dependency for the in-process
# Kirchhoff::run_ngspice_in_process runner). Mirrors MKF's emscripten recipe: emconfigure --with-ngshared
# + the four ngspice-WASM source patches + the wasm-exceptions/longjmp flags. Validated with ngspice
# 45.2 + emscripten 3.1.51 (produces install/lib/libngspice.so.0.0.14, a WASM static archive).
#
# Usage:  scripts/build_ngspice_wasm.sh [/path/to/ngspice-<ver>.tar.gz] [work_dir]
#   env:  EMSDK_ENV  (default /home/alf/emsdk/emsdk_env.sh)
# Output: <work_dir>/install/{lib/libngspice.so*, include/ngspice/sharedspice.h}
set -e
TARBALL="${1:-/home/alf/OpenMagnetics/ngspice-45.2.tar.gz}"
WORK="${2:-$(cd "$(dirname "$0")/.." && pwd)/build-wasm/ngspice}"
EMSDK_ENV="${EMSDK_ENV:-/home/alf/emsdk/emsdk_env.sh}"

[ -f "$TARBALL" ] || { echo "ngspice tarball not found: $TARBALL" >&2; exit 1; }
[ -f "$EMSDK_ENV" ] || { echo "emsdk env not found: $EMSDK_ENV (set EMSDK_ENV)" >&2; exit 1; }
# shellcheck disable=SC1090
source "$EMSDK_ENV" >/dev/null 2>&1

rm -rf "$WORK"; mkdir -p "$WORK"; cd "$WORK"
echo "[1/4] extract $TARBALL"
tar xzf "$TARBALL"
SRC="$(find . -maxdepth 1 -type d -name 'ngspice-*' | head -1)"
cd "$SRC"

echo "[2/4] apply WASM patches (configure .wasm output; guard main(); drop getrusage + init-file read under __EMSCRIPTEN__)"
sed -i 's/ac_files="a.out/ac_files="a.out a.out.js a.out.wasm/' configure || true
sed -i 's/^int main(int argc, char \*\*argv)/\n#ifndef SHARED_MODULE\nint main(int argc, char **argv)/' src/main.c 2>/dev/null || true
sed -i 's/#ifdef HAVE_GETRUSAGE/#if defined(HAVE_GETRUSAGE) \&\& !defined(__EMSCRIPTEN__)/' src/misc/misc_time.c 2>/dev/null || true
sed -i 's/read_initialisation_file()/#ifndef __EMSCRIPTEN__\n    read_initialisation_file()\n#endif/' src/sharedspice.c 2>/dev/null || true

echo "[3/4] emconfigure"
emconfigure ./configure --with-ngshared --disable-debug --disable-xspice --disable-cider \
  --disable-openmp --with-readline=no \
  CFLAGS="-O2 -fwasm-exceptions -sSUPPORT_LONGJMP=wasm" \
  CXXFLAGS="-O2 -fwasm-exceptions -sSUPPORT_LONGJMP=wasm" \
  LDFLAGS="-fwasm-exceptions -sSUPPORT_LONGJMP=wasm" \
  --prefix="$WORK/install" > "$WORK/configure.log" 2>&1

echo "[4/4] emmake make + install (the long step)"
emmake make -j"${JOBS:-4}" > "$WORK/make.log" 2>&1
emmake make install >> "$WORK/make.log" 2>&1

echo "DONE — WASM libngspice:"
find "$WORK/install" -name "libngspice.so*"
