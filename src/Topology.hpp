#pragma once

// Kirchhoff::Topology — the canonical converter-topology taxonomy.
//
// The taxonomy enum lives in PEAS: `PEAS::Topology`, defined once in deps/PEAS/src/PeasTopology.hpp
// (generated from the PEAS schema $def utils.json#/$defs/topology, which PEAS owns). There is no
// `MAS::Topology` type — MAS.hpp's generated struct is post-processed (scripts/maslift_topology.py) to
// use `::PEAS::Topology` directly, so the whole stack shares the ONE enum. Kirchhoff refers to it as
// `Kirchhoff::Topology` within its own namespace for brevity; that is the SAME PEAS::Topology type.
//
// The enum's canonical JSON string is the exact camelCase the CTAS controller-seed `topology` field
// expects (Topology::FLYBACK_CONVERTER -> "flybackConverter").

#include "PeasTopology.hpp"   // PEAS::Topology — the single definition (+ its to_json/from_json)
#include <nlohmann/json.hpp>
#include <string>

namespace Kirchhoff {

// The single Topology enum (PEAS::Topology), named for brevity inside the Kirchhoff namespace.
using Topology = PEAS::Topology;

// Serialize a topology to its canonical JSON string. THROWS via nlohmann if the enum value has no
// mapping — no silent fallback (per the no-fallbacks rule).
inline std::string topology_to_string(Topology t) {
    nlohmann::json j = t;            // PEAS::to_json(json&, const Topology&) via ADL
    return j.get<std::string>();
}

} // namespace Kirchhoff
