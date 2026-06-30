#!/usr/bin/env python3
# Post-process a quicktype-generated MAS.hpp so the converter-topology taxonomy is the canonical
# `PEAS::Topology` enum — defined ONCE in the PEAS repo (PeasTopology.hpp) — with NO `MAS::Topology`
# type at all. quicktype always emits its own `enum class MAS::Topology` (the topology $def is reachable
# from MAS::DesignRequirements and quicktype has no cross-namespace type sharing); this rewrite REMOVES
# that generated enum and points every use at `::PEAS::Topology` directly (not an alias):
#
#   - delete `enum class Topology : int { ... };`
#   - delete the Topology from_json/to_json forward declarations and inline definitions
#   - replace every remaining standalone `Topology` type token with `::PEAS::Topology`
#   - inject `#include "PeasTopology.hpp"` (the single PEAS-owned definition + its json funcs)
#
# Result: one enum, `PEAS::Topology`; `MAS::DesignRequirements::get_topology()` returns
# `std::optional<::PEAS::Topology>`; serialization uses PEAS's to_json/from_json via ADL. Fail-loud:
# aborts if any expected piece is missing, so a quicktype format change can't silently leave a stray
# `MAS::Topology`. Idempotent.
#
# Usage: maslift_topology.py <MAS.hpp>   (rewrites in place)
import re
import sys

def die(msg):
    sys.stderr.write("maslift_topology: " + msg + "\n")
    sys.exit(1)

def main():
    if len(sys.argv) != 2:
        die("usage: maslift_topology.py <MAS.hpp>")
    path = sys.argv[1]
    with open(path) as f:
        lines = f.readlines()

    if any('#include "PeasTopology.hpp"' in l for l in lines):
        return  # already lifted (idempotent)

    out = []
    removed_enum = removed_fwd = removed_inline = 0
    i, n = 0, len(lines)
    while i < n:
        line = lines[i]
        if re.match(r"\s*enum class Topology : int \{", line):
            removed_enum += 1; i += 1; continue
        if re.match(r"\s*void (from|to)_json\([^;]*\bTopology & x\);\s*$", line):
            removed_fwd += 1; i += 1; continue
        m = re.match(r"(\s*)inline void (from|to)_json\([^)]*\bTopology & x\) \{", line)
        if m:
            closing = m.group(1) + "}\n"
            i += 1
            while i < n and lines[i] != closing:
                i += 1
            if i >= n:
                die("unterminated inline Topology json block")
            i += 1; removed_inline += 1; continue
        out.append(line); i += 1

    if removed_enum != 1:
        die("expected exactly 1 `enum class Topology`, removed %d" % removed_enum)
    if removed_fwd != 2:
        die("expected 2 Topology forward decls, removed %d" % removed_fwd)
    if removed_inline != 2:
        die("expected 2 inline Topology json defs, removed %d" % removed_inline)

    # Replace every remaining standalone `Topology` type token with ::PEAS::Topology. \bTopology\b does
    # NOT match `TopologyExcitation`/`get_topology`/`topology` (word boundary / case), so only the type
    # usages (the struct field, getter/setter, and the get<>/optional<> in the struct json funcs) change.
    body = "".join(out)
    replaced = len(re.findall(r"\bTopology\b", body))
    if replaced == 0:
        die("no standalone `Topology` type usages found to repoint — unexpected")
    body = re.sub(r"\bTopology\b", "::PEAS::Topology", body)

    # Inject the PEAS header include before the first `namespace MAS {`.
    if "namespace MAS {" not in body:
        die("no `namespace MAS {` found to anchor the include")
    body = body.replace(
        "namespace MAS {",
        '#include "PeasTopology.hpp"   // the single PEAS::Topology definition (no MAS::Topology type)\n'
        "namespace MAS {", 1)

    with open(path, "w") as f:
        f.write(body)

if __name__ == "__main__":
    main()
