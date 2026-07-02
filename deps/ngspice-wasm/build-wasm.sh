#!/bin/bash
# Build libngspice.a for WebAssembly (Emscripten), for in-process use inside
# kirchhoff.js via <ngspice/sharedspice.h> (src/NgspiceRunner.cpp).
#
# Unlike wokwi/ngspice-wasm and EEsim — which build the interactive ngspice
# EXECUTABLE and drive it over stdin with ASYNCIFY — this builds the shared-
# library API as a STATIC archive that links straight into the Kirchhoff
# module. The runner issues a synchronous `run`, so no ASYNCIFY and no
# background threads are needed (bg_run is never used).
#
# CRITICAL: Kirchhoff's WASM is compiled with -fwasm-exceptions and
# -sSUPPORT_LONGJMP=wasm (CMakeLists.txt:17-19). ngspice uses setjmp/longjmp
# for its error handling, and Emscripten refuses to link objects whose
# longjmp lowering modes differ — so the exact same flags go into CFLAGS.
#
# Usage:  source ~/emsdk/emsdk_env.sh && ./build-wasm.sh
# Output: ngspice-42/release-wasm/src/.libs/libngspice.a
#         headers in ngspice-42/src/include (ngspice/sharedspice.h)

set -euo pipefail
cd "$(dirname "$0")/ngspice-42"

if ! command -v emconfigure >/dev/null; then
  echo "emsdk not in PATH — run: source ~/emsdk/emsdk_env.sh" >&2
  exit 1
fi

# sourceforge patch #99 (same as wokwi): getrusage misdetected under emscripten
sed -i 's/AC_CHECK_FUNCS(\[time getrusage\])/AC_CHECK_FUNCS([time])/g' configure.ac
sed -i 's/-Wno-unused-but-set-variable/-Wno-unused-const-variable/g' configure.ac

# --with-ngshared normally injects `-shared` into every compile (STATIC subst)
# and onto the libngspice target. Under emscripten libtool cannot build shared
# libs (build_libtool_libs=no) and any `-shared` is a fatal configuration
# error — and we genuinely want a STATIC archive to link into kirchhoff.js.
# SHARED_MODULE (the code-level define sharedspice.c needs) is set by
# AC_DEFINE independently of these flags, so dropping them is safe.
sed -i 's/AC_SUBST(\[STATIC\], \[-shared\])/AC_SUBST([STATIC], [])/g' configure.ac
sed -i 's/^libngspice_la_CFLAGS = -shared/libngspice_la_CFLAGS =/' src/Makefile.am
sed -i 's/^libngspice_la_LDFLAGS =  -shared/libngspice_la_LDFLAGS =/' src/Makefile.am

./autogen.sh

mkdir -p release-wasm
cd release-wasm

# --with-ngshared: build the sharedspice library instead of the CLI binary
# --disable-shared --enable-static: emit a libtool static archive (.a) —
#   emscripten "shared objects" are side modules we don't want
# pthreads: detection must SUCCEED — SHARED_MODULE without HAVE_LIBPTHREAD
#   falls back to Windows CRITICAL_SECTIONs (src/misc/alloc.c) and won't
#   compile on POSIX. Emscripten's non-threaded builds provide pthread STUBS
#   (mutex ops are no-ops, pthread_create fails cleanly), which is exactly
#   right: the runner only ever issues the synchronous `run`, never bg_run.
emconfigure ../configure \
  --with-ngshared --disable-shared --enable-static \
  --disable-debug --disable-openmp --without-x \
  CFLAGS="-O2 -fwasm-exceptions -sSUPPORT_LONGJMP=wasm" \
  CXXFLAGS="-O2 -fwasm-exceptions -sSUPPORT_LONGJMP=wasm"

emmake make -j"$(nproc)"

ls -la src/.libs/libngspice.a
echo "OK: $(pwd)/src/.libs/libngspice.a"
