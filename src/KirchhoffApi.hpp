#pragma once

// Kirchhoff::api — the string-in / string-out (JSON) facade over the typed Kirchhoff core.
//
// This is the SINGLE integration boundary every out-of-process / cross-namespace consumer uses:
//   * libKirchhoff.cpp   — the Emscripten/embind WASM module (the OpenMagnetics Wizard) binds these 1:1.
//   * MKF / WebLibMKF     — link Kirchhoff and call these; they parse the returned JSON into their OWN
//                           generated types (OpenMagnetics::Inputs, …), so KH's MAS:: types never enter an
//                           MKF translation unit and the two MAS namespaces can never collide.
//
// Every function takes JSON as a std::string and returns JSON (or a raw ngspice netlist) as a std::string.
// On error the returned string starts with "Exception: " (callers check that prefix) — no exception crosses
// the boundary. Topology is a string arg (the 24-row dispatcher), so there is one entry point per verb, not
// per topology.

#include <string>

// Export macro: when KH is built as the shared libKirchhoffApi (KIRCHHOFF_BUILD_SHARED_API), ONLY these
// functions get default visibility; everything else (all of KH's MAS::/internal symbols) stays hidden
// inside the .so. That is what lets MKF link KH without its own `namespace MAS` types colliding with KH's
// identically-named ones — the string API is the only thing that crosses the library boundary.
#if defined(_WIN32)
  #if defined(KIRCHHOFF_API_EXPORTS)
    #define KH_API __declspec(dllexport)
  #else
    #define KH_API __declspec(dllimport)
  #endif
#else
  #define KH_API __attribute__((visibility("default")))
#endif

namespace Kirchhoff {
namespace api {

// design_<topo>(spec) -> TAS document. `topology` is the lowercase name (flyback, buck, llc, …).
KH_API std::string design_tas(const std::string& topology, const std::string& specJson);

// design_<topo>(spec) -> {tas, analyticalWaveforms}. Same TAS as design_tas, plus the FULL analytical
// operating points (waveforms + harmonics + processed) the builders compute and then strip when baking
// the TAS: {"analyticalWaveforms": {"<component>": <MAS::OperatingPoint json>, ...}} keyed by the TAS
// component name (e.g. "T1"). This is how time-domain waveforms for the custom-label families
// (resonant / phase-shift / half-bridge) reach a frontend without an ngspice run — out-of-band, so the
// TAS document itself stays minimal and schema-valid.
KH_API std::string design_tas_full(const std::string& topology, const std::string& specJson);

// Assemble any TAS into a runnable deck. `fidelityJson` selects component models, e.g. {"origin":"REQUIREMENTS"}.
KH_API std::string generate_ngspice_circuit(const std::string& tasJson, const std::string& fidelityJson);
KH_API std::string generate_ltspice_circuit(const std::string& tasJson, const std::string& fidelityJson);

// Assemble a TAS and RUN it in-process through libngspice, returning a compact per-vector summary:
//   {success, error, tStart, tEnd, points, vectors:{<name>:{average,min,max,last}}}
// This is why MKF no longer needs its own ngspice: the simulator lives in Kirchhoff, and any MKF/test
// functionality that needs a circuit solved calls this. Returns {"success":false,...} (not an Exception
// string) when the build has no libngspice, so callers can branch on it.
KH_API std::string simulate_ngspice(const std::string& tasJson, const std::string& fidelityJson);

// The extract surface (replaces MKF's simulate_and_extract trio) — all operate on the assembled TAS.
// engine ∈ {"analytical","ngspice"}. magneticName empty = the main magnetic.
KH_API std::string extract_operating_point(const std::string& tasJson, const std::string& engine,
                                    const std::string& magneticName);
KH_API std::string topology_waveforms(const std::string& tasJson);   // [{name,isMain,inputs}]

// Per-component time-domain waveforms for every NON-magnetic power component (switches, diodes,
// capacitors, resistors) from ONE ngspice run: {engine, referencePeriod, components:[{ref,stage,kind,
// voltage:{label,waveform,processed}, current:{waveform,processed}}]}. Magnetics keep their per-winding
// waveforms via topology_waveforms/extract_operating_point; this is the complement (switch V_DS, diode
// current, cap ripple…). Returns {"success":false,...} when built without libngspice, so callers branch.
KH_API std::string component_waveforms(const std::string& tasJson, const std::string& fidelityJson);

// Add a requirements-derived datasheet model (real Rds(on) / forward drop / ratings, NO fabricated
// parasitics) to every MOSFET/diode that isn't already bound, so a subsequent DATASHEET-fidelity
// simulate/deck/component_waveforms renders real-conduction devices instead of ideal switches. Returns
// the realized TAS. Idempotent; magnetics untouched (their real path is the MKF core export).
KH_API std::string realize_tas(const std::string& tasJson);

KH_API std::string diagnostics(const std::string& tasJson);
KH_API std::string main_magnetic_inputs(const std::string& tasJson); // the adviser's MAS::Inputs (as JSON)

// --- Kelvin component sourcing (real parts from the TAS DB) ---
// Walk a TAS and select a real candidate list per fillable component seed (delegating to Kelvin,
// the shared selector). `dataDir` = the TAS/data NDJSON dir; `cacheDir` = the .kidx shard cache
// (empty = build in memory). `optionsJson` = Kelvin options (topology, maxCandidates, …).
// Returns {"components":[{ref,family,kind?,filled,mpn?|deferred?|error?,selection?}]} — the same
// candidate lists KH shows in PartDrawer and HS feeds to its LLM chooser. Body diodes, numerical
// aids, magnetics (→ MKF), and already-bound parts are deferred.
KH_API std::string select_components(const std::string& tasJson, const std::string& dataDir,
                                     const std::string& cacheDir, const std::string& optionsJson);

// Bind a chosen candidate's envelope into the named component's data slot (verbatim; already
// schema-valid) and return the new TAS. The component then reads as DATASHEET fidelity — a
// subsequent realize/simulate renders the real part; the numerical snubber strips when it carries
// its Coss. This is the real-part replacement for realize_tas's "requirements-derived" fabrication.
KH_API std::string bind_part(const std::string& tasJson, const std::string& ref,
                             const std::string& envelopeJson);

// Browser sourcing (no filesystem): load a prebuilt Kelvin shard from raw bytes into a persistent
// in-module engine, then select over the loaded families. Candidates carry mpn/manufacturer/
// margins/evidence + the record's byte span (srcOffset/srcLength) — NOT the full envelope; the
// caller fetches the chosen record itself (HTTP Range into the hosted NDJSON) and passes it to
// bind_part. kelvin_load_shard returns {family,rowCount,buildId}; kelvin_select returns a
// SelectionResult, or {error:"NoCandidates",rejections,...} when nothing satisfies the request.
KH_API std::string kelvin_load_shard(const std::string& family, const std::string& shardBytes);
KH_API std::string kelvin_select(const std::string& category, const std::string& designReqJson,
                                 const std::string& optionsJson);

// One-shot: spec -> {topology, inputs, operatingPoint, diagnostics, tas}. Mirrors WebLibMKF process_converter.
KH_API std::string process_converter(const std::string& topology, const std::string& specJson,
                              const std::string& engine);

// spec -> the magnetic's MAS::Inputs (designRequirements + operatingPoints) for ANY topology — the
// "design requirements" entry point the OpenMagnetics Wizards / MagneticAdviser consume, WITHOUT
// having to carry the TAS. For the 24 switching topologies this is design_tas + main_magnetic_inputs
// in one call; `topology` additionally accepts "common_mode_choke" (aliases "cmc"/"commonModeChoke"),
// which has no TAS at all — a CMC is a component, not a converter — and routes to the CMC designer.
KH_API std::string design_magnetic_inputs(const std::string& topology, const std::string& specJson);

// Common-mode choke one-shot (the calculate_cmc_inputs replacement, both normal and "advanced"
// desiredInductance mode): wizard spec -> {"inputs": <MAS::Inputs>, "cmcDiagnostics":
// {computedInductance, dominantFrequency, dominantImpedance}}. The Inputs object is returned intact
// under its own key (never polluted with the diagnostics — every materialized object stays
// schema-valid); legacy shims spread `inputs` at the root and attach `cmcDiagnostics` themselves.
KH_API std::string design_cmc(const std::string& specJson);

// Differential-mode choke one-shot (the calculate_dmc_inputs replacement): wizard spec -> {"inputs":
// <MAS::Inputs>, "dmcDiagnostics": {computedInductance, computedMinFrequency, computedMaxFrequency,
// impedanceAtMinFrequency, numberWindings}}. Same envelope convention as design_cmc.
KH_API std::string design_dmc(const std::string& specJson);

// DMC "help me with the design" LC sizing (the propose_dmc_design replacement): wizard spec ->
// {inductance, capacitance, cutoffFrequency, targetAttenuation_dB, peakCurrent, ...}. The wizard reads
// `inductance` and re-calls design_dmc with it as minimumInductance. Analytical only (no ngspice verify).
KH_API std::string propose_dmc_design(const std::string& specJson);

// Current-transformer one-shot (the process_current_transformer replacement): spec (waveformLabel,
// maximumPrimaryCurrentPeak, frequency, turnsRatio, burdenResistor, ambientTemperature, +optional
// secondaryDcResistance/dutyCycle/diodeVoltageDrop) -> the MAS::Inputs (a 2-winding transformer with the
// burden-resistor sensing operating point). No diagnostics block (bare Inputs).
KH_API std::string design_current_transformer(const std::string& specJson);

// CMC ngspice simulations for the wizard's EMI / waveform views (require an ngspice-enabled build).
// simulate_cmc_ideal_waveforms(spec, L, parasiticCap_pF, dvdt_V_ns): per-winding CM sine → {success,
// inputs:{operatingPoints:[...]}, converterWaveforms:[], cmcDiagnostics}. simulate_cmc_lisn_waveforms(
// spec, L): CISPR LISN sweep over the spec impedance frequencies → {success, converterWaveforms:[...]}.
// Both return {"success":false,...} (not an Exception string) when built without libngspice.
KH_API std::string simulate_cmc_ideal_waveforms(const std::string& specJson, double inductance,
                                                double parasiticCapPf, double dvdtVPerNs);
KH_API std::string simulate_cmc_lisn_waveforms(const std::string& specJson, double inductance);

// DMC ngspice simulations (require an ngspice-enabled build). Both simulate the SAME LC filter — the
// capacitance argument, else the spec's filterCapacitance, else fc = fsw/10 auto-sizing (which needs
// switchingFrequency; capacitance 0 = auto-size). simulate_dmc_waveforms(spec, L, capacitance): LC
// low-pass sim → {success, converterWaveforms:[...], failedFrequencies?}. verify_dmc_attenuation(spec,
// L, capacitance): per-point required/measured/theoretical attenuation + simulated flag + pass/fail.
KH_API std::string simulate_dmc_waveforms(const std::string& specJson, double inductance,
                                          double capacitance = 0.0);
KH_API std::string verify_dmc_attenuation(const std::string& specJson, double inductance,
                                          double capacitance);

}  // namespace api
}  // namespace Kirchhoff
