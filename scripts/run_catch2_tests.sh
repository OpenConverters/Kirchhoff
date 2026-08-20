#!/usr/bin/env bash
# Run the native Catch2 suite (ABT #670).
#
# npm run check covers the web layer, the CIAS/schematic/simulation equivalence and the parameter sweep.
# Nothing ran the C++ binaries — which is how tests/test_extract.cpp stayed red for ~2.5 weeks, found only
# by someone running a binary by hand while chasing something else. It is not a hypothetical: the ABT #778
# isolation fix put an undeclared floating node into three topologies, and it was these binaries, not any
# web gate, that caught it (LLC Iout collapsed to 2.8e-09 against a reference of 5).
#
# House rule: invoke the Catch2 binaries DIRECTLY, never ctest — so this loops them and reports each one.
#   ./scripts/run_catch2_tests.sh [build-dir]        (default: build-native)
# Catch2 arguments pass through after `--`, e.g. `./scripts/run_catch2_tests.sh build-native -- "[src]"`.
set -uo pipefail

# Args: an optional build directory, then anything after `--` goes to every Catch2 binary.
#   ./scripts/run_catch2_tests.sh                       all binaries in build-native
#   ./scripts/run_catch2_tests.sh build-debug           a different build tree
#   ./scripts/run_catch2_tests.sh -- "[src]"            Catch2 tag filter, default build tree
BUILD=build-native
if [[ $# -gt 0 && "$1" != "--" ]]; then BUILD="$1"; shift; fi
if [[ $# -gt 0 && "$1" == "--" ]]; then shift; fi

cd "$(dirname "$0")/.." || exit 2
if [[ ! -d "$BUILD" ]]; then
  echo "no build directory '$BUILD' — configure and build first (cmake -G Ninja -B $BUILD && cmake --build $BUILD -- -j4)" >&2
  exit 2
fi

# Per-binary timeout: a hung test must fail the run, not hold it forever. Generous, because the resonant
# and MKF-equivalence binaries run real ngspice transients.
TIMEOUT="${KH_TEST_TIMEOUT:-1200}"
LOGDIR="$(mktemp -d)"
pass=0; fail=0; failed=()

shopt -s nullglob
binaries=("$BUILD"/test_*)
# Refuse to report success over nothing — an empty build directory would otherwise print "0 failed".
if (( ${#binaries[@]} == 0 )); then
  echo "no test_* binaries in '$BUILD' — nothing was run" >&2
  exit 2
fi

for t in "${binaries[@]}"; do
  [[ -x "$t" && -f "$t" ]] || continue
  name="$(basename "$t")"
  if timeout "$TIMEOUT" "$t" "$@" > "$LOGDIR/$name.log" 2>&1; then
    printf '  ok   %s\n' "$name"
    pass=$((pass+1))
  else
    rc=$?
    printf '  FAIL %s (exit %d)\n' "$name" "$rc"
    # the assertion, not the whole log — enough to see what broke without burying the summary
    grep -E "FAILED|with expansion|with message|error:" -A2 "$LOGDIR/$name.log" | head -20 | sed 's/^/       /'
    fail=$((fail+1)); failed+=("$name")
  fi
done

echo
echo "Catch2: $pass passed, $fail failed (logs in $LOGDIR)"
if (( fail )); then
  printf 'FAILED: %s\n' "${failed[*]}"
  exit 1
fi
exit 0
