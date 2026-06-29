#pragma once

// Kirchhoff::Topology — the canonical converter-topology taxonomy.
//
// Step 1 of the MAS::Topology adoption (see docs/MKF_MIGRATION.md): Kirchhoff consumes the GENERATED
// `MAS::Topology` enum as its single source of truth for "which converter" — the same typed taxonomy
// MKF and the advisers use — instead of carrying ad-hoc topology strings. The enum's canonical JSON
// serialization (via MAS's to_json) is the exact camelCase string the CTAS controller-seed `topology`
// field already expects (e.g. Topology::FLYBACK_CONVERTER -> "flybackConverter"), so adopting the enum
// is a pure type-safety win: the JSON the assembler emits is byte-identical, but the topology name is
// now compiler-checked rather than a typo-prone literal.
//
// Step 2 (a governed schema RFC) will move the taxonomy enum MAS -> PEAS; at that point `using Topology`
// below is repointed to `PEAS::Topology` and nothing else in Kirchhoff changes.

#include "MAS.hpp"            // generated typed structs incl. enum class MAS::Topology
#include <nlohmann/json.hpp>
#include <string>

namespace Kirchhoff {

// The canonical taxonomy. Repoint to PEAS::Topology after the MAS->PEAS RFC (Step 2).
using Topology = MAS::Topology;

// Serialize a topology to its canonical JSON string (MAS's to_json map). THROWS via nlohmann if the
// enum value has no mapping — no silent fallback (per the no-fallbacks rule).
inline std::string topology_to_string(Topology t) {
    nlohmann::json j = t;            // MAS::to_json(json&, const Topology&)
    return j.get<std::string>();
}

} // namespace Kirchhoff
