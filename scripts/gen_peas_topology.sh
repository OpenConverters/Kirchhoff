#!/bin/bash
# Generate PEAS_Topology.hpp — the typed C++ `enum class PEAS::Topology` (+ to_json/from_json) — from
# the canonical PEAS schema $def `utils.json#/$defs/topology`. The converter-topology taxonomy is
# PEAS-owned vocabulary (see the $def description), so its C++ type belongs in namespace PEAS.
#
# quicktype needs a schema whose ROOT is the enum, so we first extract the $def into a standalone schema
# (via node, already on PATH for quicktype), then quicktype that. Run at build time so the header never
# drifts from the schema (mirrors how MAS.hpp is generated).
#
# Usage: scripts/gen_peas_topology.sh <PEAS/schemas/utils.json> <out/PEAS_Topology.hpp>
set -e
UTILS="$1"; OUT="$2"
[ -f "$UTILS" ] || { echo "gen_peas_topology: utils.json not found: $UTILS" >&2; exit 1; }
TMP="$(dirname "$OUT")/.peas_topology.schema.json"
mkdir -p "$(dirname "$OUT")"

node -e '
  const fs = require("fs");
  const u = JSON.parse(fs.readFileSync(process.argv[1], "utf8"));
  const t = u["$defs"] && u["$defs"]["topology"];
  if (!t) { console.error("no $defs.topology in " + process.argv[1]); process.exit(1); }
  const schema = { "$schema":"https://json-schema.org/draft/2020-12/schema",
                   "$id":"https://psma.com/peas/topology.json", "title":"Topology", ...t };
  fs.writeFileSync(process.argv[2], JSON.stringify(schema, null, 2));
' "$UTILS" "$TMP"

quicktype -l c++ -s schema --namespace PEAS --source-style single-source --type-style pascal-case \
  --member-style underscore-case --enumerator-style upper-underscore-case --no-boost \
  -t Topology -o "$OUT" "$TMP"
rm -f "$TMP"
echo "generated $OUT"
