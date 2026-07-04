#!/usr/bin/env bash
# Build the per-family Kelvin index shards the web GUI serves from public/kelvin/.
# These are build artifacts (gitignored); regenerate whenever the TAS DB changes.
#
#   KELVIN_INDEX=/path/to/Kelvin/build/kelvin-index \
#   TAS_DATA=/path/to/TAS/data \
#   web/scripts/build-kelvin-shards.sh
#
# Defaults assume the sibling Kelvin checkout and the standard TAS data dir.
set -euo pipefail

HERE="$(cd "$(dirname "$0")/.." && pwd)"                       # web/
KELVIN_INDEX="${KELVIN_INDEX:-$HERE/../../Kelvin/build/kelvin-index}"
TAS_DATA="${TAS_DATA:-/home/alf/PSMA/TAS/data}"
OUT="$HERE/public/kelvin"

if [[ ! -x "$KELVIN_INDEX" ]]; then
  echo "kelvin-index not found at $KELVIN_INDEX — build Kelvin (cmake --build) or set KELVIN_INDEX" >&2
  exit 1
fi
mkdir -p "$OUT"
"$KELVIN_INDEX" --data "$TAS_DATA" --out "$OUT"
echo "Kelvin shards written to $OUT"
ls -la "$OUT"
