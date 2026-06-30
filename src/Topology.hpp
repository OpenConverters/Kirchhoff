#pragma once

// Kirchhoff::Topology — the canonical converter-topology taxonomy.
//
// Kirchhoff consumes the generated `MAS::Topology` enum (emitted by quicktype into MAS.hpp) as its
// source of truth for "which converter" — used directly, exactly as generated (the taxonomy is
// PEAS-owned at the SCHEMA layer via PEAS/schemas/utils.json#/$defs/topology, which MAS $refs, so the
// values + JSON serialization are PEAS-owned regardless of the C++ namespace). NEVER post-process
// MAS.hpp. Kirchhoff refers to the enum as `Kirchhoff::Topology` within its own namespace for brevity;
// that is the SAME single MAS::Topology type.
//
// The enum's canonical JSON string is the exact camelCase the CTAS controller-seed `topology` field
// expects (Topology::FLYBACK_CONVERTER -> "flybackConverter").

#include "MAS.hpp"            // the single generated MAS::Topology enum (+ its to_json/from_json)
#include <nlohmann/json.hpp>
#include <string>

namespace Kirchhoff {

// The single Topology enum (MAS::Topology), named for brevity inside the Kirchhoff namespace.
using Topology = MAS::Topology;

// Serialize a topology to its canonical JSON string. THROWS via nlohmann if the enum value has no
// mapping — no silent fallback (per the no-fallbacks rule).
inline std::string topology_to_string(Topology t) {
    nlohmann::json j = t;            // MAS::to_json(json&, const Topology&)
    return j.get<std::string>();
}

} // namespace Kirchhoff
