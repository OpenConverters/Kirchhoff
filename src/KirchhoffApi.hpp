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

namespace Kirchhoff {
namespace api {

// design_<topo>(spec) -> TAS document. `topology` is the lowercase name (flyback, buck, llc, …).
std::string design_tas(const std::string& topology, const std::string& specJson);

// Assemble any TAS into a runnable deck. `fidelityJson` selects component models, e.g. {"origin":"REQUIREMENTS"}.
std::string generate_ngspice_circuit(const std::string& tasJson, const std::string& fidelityJson);
std::string generate_ltspice_circuit(const std::string& tasJson, const std::string& fidelityJson);

// Assemble a TAS and RUN it in-process through libngspice, returning a compact per-vector summary:
//   {success, error, tStart, tEnd, points, vectors:{<name>:{average,min,max,last}}}
// This is why MKF no longer needs its own ngspice: the simulator lives in Kirchhoff, and any MKF/test
// functionality that needs a circuit solved calls this. Returns {"success":false,...} (not an Exception
// string) when the build has no libngspice, so callers can branch on it.
std::string simulate_ngspice(const std::string& tasJson, const std::string& fidelityJson);

// The extract surface (replaces MKF's simulate_and_extract trio) — all operate on the assembled TAS.
// engine ∈ {"analytical","ngspice"}. magneticName empty = the main magnetic.
std::string extract_operating_point(const std::string& tasJson, const std::string& engine,
                                    const std::string& magneticName);
std::string topology_waveforms(const std::string& tasJson);   // [{name,isMain,inputs}]
std::string diagnostics(const std::string& tasJson);
std::string main_magnetic_inputs(const std::string& tasJson); // the adviser's MAS::Inputs (as JSON)

// One-shot: spec -> {topology, inputs, operatingPoint, diagnostics, tas}. Mirrors WebLibMKF process_converter.
std::string process_converter(const std::string& topology, const std::string& specJson,
                              const std::string& engine);

}  // namespace api
}  // namespace Kirchhoff
