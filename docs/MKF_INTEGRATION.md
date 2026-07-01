# Replacing MKF `converter_models` with Kirchhoff — integration surface

This document specifies how MKF (and the OpenMagnetics Wizard) consume Kirchhoff (KH) in place of the
`converter_models` subtree that is being removed. The whole integration surface is now in KH and is
**topology-agnostic** — it operates on the assembled TAS document (`build_<topo>_tas(design_<topo>(spec))`),
which already carries every magnetic's `MAS::Inputs` and the wired circuit.

## The mapping (what replaces what)

| Removed from MKF `converter_models` | Replacement in Kirchhoff |
|---|---|
| `<Topo>::simulate_and_extract_operating_points()` → `vector<OperatingPoint>` | `Kirchhoff::extract_operating_point(tas, engine, magneticName)` → `MAS::OperatingPoint` — `engine` = `ANALYTICAL` or `NGSPICE` |
| `<Topo>::simulate_and_extract_topology_waveforms()` | `Kirchhoff::topology_waveforms(tas)` → `vector<MagneticExtract>` (every magnetic + its `MAS::Inputs`, `isMain` flagged) |
| `<Topo>::generate_ngspice_circuit()` | `Kirchhoff::tas_to_ngspice(tas, fidelity)` (generic) |
| `<Topo>::get_extra_components_inputs(mode, magnetic)` | `Kirchhoff::extra_components_inputs(tas)` (see below) |
| `<Topo>::process_design_requirements()` (the adviser's `DesignRequirements`) | `Kirchhoff::main_magnetic_inputs(tas)` → `MAS::Inputs` (the main magnetic) |
| the `<name>Diagnostics` object WebLibMKF serialized | `Kirchhoff::diagnostics(tas)` → JSON |
| `Advanced<Name>` reference-design classes | `design_<topo>(spec)` already honors the `desired*` PEAS pins (see "Advanced path") |

`ConverterExtract.hpp` is the single header for the extract/diagnostics/shim surface.

## The two MKF-side shims the migration plan calls for

The user's plan: *"two methods in MKF that get this TAS and return the design requirements for a MAS and the
get_extra_components_inputs, to maintain the legacy."* Because KH owns the TAS format and the KH↔MKF
boundary is JSON (KH's CAS submodule is a converter, not a generated-types lib), the TAS walk lives once in
KH; the MKF methods are **thin deserializing wrappers** — no duplicate TAS walk, no re-derived physics.
They require MKF to link `kirchhoff` (one `target_link_libraries(MKF ... kirchhoff)` line) OR to receive the
JSON that KH's functions already produce.

```cpp
// MKF-side, e.g. src/converter_models/KirchhoffBridge.cpp  (MKF's own MAS/CAS generated types)
#include "Kirchhoff.hpp"   // if MKF links kirchhoff; else accept the JSON these produce

namespace OpenMagnetics {

// (1) TAS -> the DesignRequirements-bearing Inputs the MagneticAdviser designs around.
Inputs design_requirements_from_tas(const nlohmann::json& tas) {
    // KH extracts the main magnetic's MAS::Inputs; re-parse into MKF's Inputs (same JSON schema).
    nlohmann::json mainInputs = Kirchhoff::main_magnetic_inputs(tas);  // MAS::Inputs -> json
    return mainInputs.get<Inputs>();
}

// (2) TAS -> the legacy get_extra_components_inputs vector<variant<Inputs, CAS::Inputs>>.
std::vector<std::variant<Inputs, CAS::Inputs>> extra_components_inputs_from_tas(const nlohmann::json& tas) {
    std::vector<std::variant<Inputs, CAS::Inputs>> out;
    for (const auto& e : Kirchhoff::extra_components_inputs(tas)) {   // tagged JSON array
        const std::string kind = e.at("componentType");
        if (kind == "magnetic")       out.emplace_back(e.at("inputs").get<Inputs>());
        else if (kind == "capacitor") out.emplace_back(e.at("inputs").get<CAS::Inputs>());
    }
    return out;
}

}  // namespace OpenMagnetics
```

`Kirchhoff::extra_components_inputs(tas)` returns:

```json
[ { "componentType": "magnetic",  "name": "Lr",  "inputs": { <MAS::Inputs json> } },
  { "componentType": "capacitor", "name": "Cr",  "inputs": { "designRequirements": { "capacitance": {"nominal": …}, "ratedVoltage": …, "role": "resonant" } } } ]
```

**Known gap (surfaced, not hidden):** capacitor entries carry the TAS `designRequirements`
(capacitance / ratedVoltage / role) but *not* per-operating-point cap voltage/current waveforms — the KH
TAS does not embed those today (MKF's `get_extra_components_inputs` computed them from stored
`extraCapVoltageWaveforms`). If the Wizard needs cap operating points, add per-OP cap excitations to the
TAS cap components in `build_<topo>_tas` (they can come from `extract_operating_point`'s simulated node
voltages), then extend `extra_components_inputs` to emit them.

## Advanced path

`design_<topo>(spec)` reproduces `Advanced<Name>::process()` when the spec/TAS carries the `desired*` PEAS
pins, honoring them verbatim instead of sizing from scratch:

- `magnetizingInductance` / `desiredInductance` / `inductance` → `req::provided_inductance` (19 topologies)
- `turnsRatios` → `req::provided_turns_ratio`
- `desiredResonantInductance` / `desiredResonantCapacitance` → `req::provided_resonant_{inductance,capacitance}`
  (LLC; CLLC/CLLLC/SRC follow the same helper — extend as needed)

Verified in `tests/test_advanced.cpp`. **Gap:** MKF's per-operating-point `desiredDutyCycle[][]` and
`desiredDeadTime` arrays are not yet plumbed as pins (KH computes them); add per-topology if the Wizard
pins them.

## WASM (Wizard) surface

`src/libKirchhoff.cpp` is the embind module (`emcmake cmake … && cmake --build … --target libKirchhoff`),
exposing the string-in/string-out API the Wizard's `taskQueue.js` expects:

- `design_tas(topology, spec)` — the 24-row topology dispatcher → TAS JSON
- `process_converter(topology, spec, engine)` — one-shot → `{inputs, operatingPoint, diagnostics, extraComponents, tas}`
- `generate_ngspice_circuit(tas, fidelity)` / `generate_ltspice_circuit(tas, fidelity)` → deck
- `extract_operating_point(tas, engine, magneticName)` / `topology_waveforms(tas)` / `diagnostics(tas)` /
  `main_magnetic_inputs(tas)` / `extra_components_inputs(tas)` → JSON

Errors surface as a returned string starting `"Exception: "` (the JS side already checks that). The browser
build has `ENABLE_NGSPICE=OFF`; `extract_operating_point(..., "ngspice")` throws there until the
ngspice-in-wasm path (P5) is wired.
