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

Errors surface as a returned string starting `"Exception: "` (the JS side already checks that).

## Native linking (MKF) — the isolation that makes it safe

MKF and KH **both** generate their types into `namespace MAS` (MKF's `OpenMagnetics::Inputs : public
MAS::Inputs`). Linking KH's objects straight into MKF would therefore be an ODR collision on the `MAS::`
symbols. The fix, verified end-to-end:

- KH builds a **shared** `libKirchhoffApi.so` (`-DKIRCHHOFF_BUILD_SHARED_API=ON`) that exports **only** the
  `Kirchhoff::api` string functions (marked `KH_API` = visibility default). Everything else — the whole
  typed core, KH's `MAS::`, CAS/SAS/… — is compiled `-fvisibility=hidden` and localized with
  `-Wl,--exclude-libs,ALL`. `nm -D` shows the 9 api funcs and **zero** strong `MAS::` symbols.
- MKF FetchContents KH via **ExternalProject** (`MKF_USE_KIRCHHOFF=ON`) so KH is *configured in its own
  CMake context* — its `CAS`/`SAS`/`MAS_kirchhoff` targets never enter MKF's target namespace. MKF links
  `libKirchhoffApi.so` and its bridge TU (`src/converter_models/KirchhoffBridge.cpp`) includes **only**
  `KirchhoffApi.hpp` (plain `std::string` signatures) → no KH MAS type ever reaches an MKF TU.
- **ngspice moves to KH.** MKF's `ENABLE_NGSPICE` stays OFF (its default); KH is built `ENABLE_NGSPICE=ON`
  inside the ExternalProject, so the simulator lives in `libKirchhoffApi.so`. Any MKF path that must solve a
  circuit calls `OpenMagnetics::simulate_tas(...)` → `Kirchhoff::api::simulate_ngspice`. (No MAS material DB
  is pulled for KH — it has none; MKF's `EMBED_MAS_*` is separate and only for HS magnetics.)

`examples/api_consumer.cpp` in KH is the reduced proof: it links `libKirchhoffApi.so` with only the string
header and runs a 151k-point ngspice transient through KH.

## WASM (browser) — why KH is its own module, not folded into WebLibMKF

Two hard constraints make "compile KH *into* WebLibMKF's single wasm module" the wrong shape:

1. **The same `namespace MAS` ODR clash** — but wasm has no hidden-visibility shared libs, so the native
   `.so` trick does not apply; both MAS type sets would statically link into one module and collide.
2. **Exception-handling model** — WebLibMKF/MKF and the in-browser ngspice are built with
   `-fwasm-exceptions` + `-sSUPPORT_LONGJMP=wasm`; you cannot mix that with a differently-compiled module.

So the clean architecture is **two wasm modules**: `webMKF` (WebLibMKF — magnetics) and **`webKirchhoff`**
(`libKirchhoff.js` — converters *and* simulation), loaded side-by-side by the Wizard. Separate modules =
separate `MAS` = no clash, and each is internally EH-consistent. **This is built and VERIFIED:**
`webKirchhoff` compiled `-fwasm-exceptions` with `ENABLE_NGSPICE=ON` links the in-browser libngspice and
**runs a real transient in Node** — `simulate_ngspice` returns `success=true` with ~97k points, and
`extract_operating_point(tas,"ngspice")` rebuilds the winding currents from that sim
(`tests/wasm/test_libkirchhoff_ngspice.mjs`). Build:

```
emcmake cmake -S . -B build-wasm-ng -G Ninja -DKIRCHHOFF_BUILD_PYBIND=OFF \
  -DENABLE_NGSPICE=ON \
  -DNGSPICE_LIB=<...>/ngspice/install/lib/libngspice.so.0.0.14 \
  -DNGSPICE_INCLUDE_DIR=<...>/ngspice/install/include
cmake --build build-wasm-ng --target libKirchhoff
```

Two link settings are essential (both in the CMake): `-fwasm-exceptions` + `-sSUPPORT_LONGJMP=wasm` to match
the ngspice EH model, and `-sSTACK_SIZE=64MB -sINITIAL_MEMORY=128MB` — the emscripten default 64 KB stack
overflows mid-simulation ("memory access out of bounds") without it (WebLibMKF's proven sizing).

If instead a single module is mandated, KH must be an emscripten **SIDE_MODULE** (`dlopen`ed by WebLibMKF as
MAIN_MODULE) — the wasm analogue of the hidden-symbol `.so` — or KH and MKF must be refactored to share one
generated MAS. Both are larger efforts; the two-module split is recommended and working.
