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

# Host each catalog's NDJSON next to its shard so the PartDrawer can Range-fetch a chosen part's
# ONE record (bytes=srcOffset-srcLength) at bind time — the full envelope never lives in the shard.
# The web fetches singular /kelvin/<category>.ndjson; the source catalogs are plural (<category>s).
# Symlinks keep local/dev disk ~0 (sirv/nginx both follow them); the prod deploy rsync -L resolves
# them into real files. The bytes MUST stay identical to what the shard was indexed from (same DB).
for cat in mosfet diode capacitor resistor controller igbt bjt varistor; do
  src="$TAS_DATA/${cat}s.ndjson"
  if [[ -f "$src" ]]; then
    ln -sfn "$src" "$OUT/${cat}.ndjson"
  else
    echo "WARN: source catalog $src missing — bind for '$cat' will 404 until it is hosted" >&2
  fi
done
echo "Kelvin shards + NDJSON written to $OUT"
ls -la "$OUT"
