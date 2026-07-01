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
| `<Topo>::get_extra_components_inputs(mode, magnetic)` | **removed** — obsolete in Heaviside (the whole TAS already carries every extra component as its own stage; see below) |
| `<Topo>::process_design_requirements()` (the adviser's `DesignRequirements`) | `Kirchhoff::main_magnetic_inputs(tas)` → `MAS::Inputs` (the main magnetic) |
| the `<name>Diagnostics` object WebLibMKF serialized | `Kirchhoff::diagnostics(tas)` → JSON |
| `Advanced<Name>` reference-design classes | `design_<topo>(spec)` already honors the `desired*` PEAS pins (see "Advanced path") |

`ConverterExtract.hpp` is the single header for the extract/diagnostics surface.

## No more `get_extra_components_inputs`

In Heaviside the converter step returns the **whole TAS**, and every extra component the converter needs
(output inductor, resonant Lr, resonant Cr, output Co, snubbers) is already present as its own
`topology.stages[].circuit.components[]` entry with its `data.inputs`. A separate "extra components"
extraction is therefore redundant and has been removed. To reach them:

- extra **magnetics** (Lr, output inductor, CM/DM chokes): `Kirchhoff::topology_waveforms(tas)` returns
  every magnetic with its `MAS::Inputs`; the non-`isMain` ones are the extras.
- extra **capacitors** (Cr, Co): walk `topology.stages[].circuit.components[]` for `data.capacitor` and
  read `data.inputs.designRequirements` (`capacitance` / `ratedVoltage` / `role`).

## The one MKF-side shim the migration needs

`Kirchhoff::main_magnetic_inputs(tas)` gives the adviser's `MAS::Inputs`. Because the KH↔MKF boundary is
JSON (same MAS schema), MKF's wrapper is a one-liner re-parse into MKF's own `Inputs`:

```cpp
// MKF-side (MKF's own generated Inputs). MKF FetchContents kirchhoff (see below), so it links KH directly.
Inputs design_requirements_from_tas(const nlohmann::json& tas) {
    return nlohmann::json(Kirchhoff::main_magnetic_inputs(tas)).get<Inputs>();
}
```

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
- `process_converter(topology, spec, engine)` — one-shot → `{inputs, operatingPoint, diagnostics, tas}`
- `generate_ngspice_circuit(tas, fidelity)` / `generate_ltspice_circuit(tas, fidelity)` → deck
- `extract_operating_point(tas, engine, magneticName)` / `topology_waveforms(tas)` / `diagnostics(tas)` /
  `main_magnetic_inputs(tas)` → JSON

Errors surface as a returned string starting `"Exception: "` (the JS side already checks that). The browser
build has `ENABLE_NGSPICE=OFF`; `extract_operating_point(..., "ngspice")` throws there until the
ngspice-in-wasm path (P5) is wired.
