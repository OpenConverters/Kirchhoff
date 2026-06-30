#pragma once

// Kirchhoff::Topology — the canonical converter-topology taxonomy.
//
// Kirchhoff consumes the typed `PEAS::Topology` enum as its single source of truth for "which
// converter" — instead of carrying ad-hoc topology strings. The taxonomy is PEAS-OWNED vocabulary: it
// is defined once at `PEAS/schemas/utils.json#/$defs/topology` (the $def description: "PEAS HOSTS this
// shared vocabulary because all families reference it"), and PEAS_Topology.hpp is generated from it at
// build time (scripts/gen_peas_topology.sh). MAS merely $refs the same $def, so MAS::Topology and
// PEAS::Topology have identical values + identical JSON serialization; Kirchhoff uses the PEAS one
// because PEAS owns the contract.
//
// The enum's canonical JSON string is the exact camelCase the CTAS controller-seed `topology` field
// expects (Topology::FLYBACK_CONVERTER -> "flybackConverter"), so the assembled JSON is byte-identical
// to the old hand-written strings — a pure type-safety win (compiler-checked vs typo-prone literals).

#include "PEAS_Topology.hpp"   // generated: enum class PEAS::Topology (+ to_json/from_json)
#include <nlohmann/json.hpp>
#include <string>

namespace Kirchhoff {

// The canonical taxonomy — PEAS-owned.
using Topology = PEAS::Topology;

// Serialize a topology to its canonical JSON string (PEAS's to_json map). THROWS via nlohmann if the
// enum value has no mapping — no silent fallback (per the no-fallbacks rule).
inline std::string topology_to_string(Topology t) {
    nlohmann::json j = t;            // PEAS::to_json(json&, const Topology&) via ADL
    return j.get<std::string>();
}

} // namespace Kirchhoff
